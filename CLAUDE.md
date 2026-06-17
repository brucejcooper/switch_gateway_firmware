# Controller firmware — working notes

Raspberry Pi Pico firmware on a **WIZnet W5100S-EVB-Pico (RP2040)**. Core 0 owns
the RS-485 Modbus RTU bus (TinyModbus DALI gateways + relay boards), the button
matrix, the state cache, persistence, and a USB-CDC **text CLI**. Core 1 owns the
on-board W5100S Ethernet + MQTT and talks to Home Assistant directly — there is no
external bridge host. The two cores exchange **typed C structs** over two SPSC
queues; JSON exists only at core 1's MQTT edge. See `README.md` for the full
architecture, MQTT topics, and CLI grammar.

## Building & flashing the controller (Pico)

- **Build:** run `make` in the `build/` subdirectory (CMake-generated). Output is
  `build/light_switch.uf2`.
  ```sh
  make -C build
  ```
- **Flash:** from `build/`, load and execute the new firmware over USB:
  ```sh
  cd build && picotool load -f -x light_switch.uf2
  ```
  `-f` forces the running app into BOOTSEL, `-x` executes after loading. The Pico
  re-enumerates afterwards; **sleep ~2 s** before reconnecting to the serial port.
- After reboot the controller runs boot-time bus discovery + a full scan of each
  DALI slave. It is ready once it emits the `hello` / `serial: hello` log.
- The board is selected in `CMakeLists.txt` (`PICO_BOARD wiznet_w5100s_evb_pico`)
  and needs four submodules (`git submodule update --init --recursive`): pico-sdk,
  pico-vfs, ioLibrary_Driver, coreMQTT. A bare Pico 2 / RP2350 UF2 will NOT boot
  this board.

## On-board MQTT bridge (core 1) — W5100S Ethernet (2026-06-09)

The Python `bridge/mqtt_bridge.py` host was retired: its duties now run **on-board
on core 1** over the W5100S hardwired TCP/IP chip (SPI0, GP16–21). Core 0's domain
logic is unchanged; the seam is `src/intercore.{c,h}` (typed events up, typed
commands down). Network code lives in `src/net/` (`netif` bring-up/DHCP/PHY,
`mqtt_app` non-blocking TCP socket SM + coreMQTT connection manager, `ha_bridge`
HA discovery + event↔state translation + inbound command parsing) and
`src/core1.{c,h}` (launch + liveness). JSON is jsmn (parse) + snprintf (build);
the whole firmware stays malloc-/float-free (coreMQTT + ioLibrary are static).
Broker/network config is in `/config.json`, set via the `mqtt` / `net` CLI verbs
(applied on reboot).

**Launch order matters (flash lockout victim):** core 0 must `intercore_init()`
then `core1_launch()` **before** the first core-0 flash write (`fs_init`'s
first-boot littlefs format). Core 1 registers as the `flash_safe_execute` lockout
victim on launch, then parks on a config-ready gate until core 0 calls
`core1_config_ready()` (after the config snapshot is populated) before bringing up
the network. Get this wrong and the first-boot format returns `NOT_PERMITTED` and
the filesystem never mounts (no persistence). Core 0 feeds the watchdog only while
core 1's heartbeat is fresh (+15 s grace), so a real core-1 wedge reboots but a
network outage does not.

**Four bugs that only showed on hardware** (all fixed): (1) `button_actions.c`
used `PIO2_IRQ_0` (RP2350-only) → `pio_get_irq_num()` (RP2040 has 2 PIO blocks);
(2) W5100S at 30 MHz SPI read `VER=0x00` (SPI desync) → dropped to 10 MHz + longer
reset in `wizchip_glue.c`; (3) the `fs_init`-before-`core1_launch` ordering bug
above; (4) the ioLibrary recv() non-block bug below.

**Do NOT log in core-1 network hot paths.** A `LOG_WARN` in `transport_send`/`recv`
during the discovery burst caused a heisenbug: USB-CDC `printf` back-pressure
stalled core 1 mid-publish → W5100S timeout → connection churn → "0 retained".
Removing the instrumentation fixed it. The Docker broker was never at fault.

### ioLibrary recv() #173 — build-time patch

ioLibrary's `recv()` returns `SOCK_BUSY` *before* checking `recvsize` on the
non-IPv6 non-block path (upstream issue #173), so the MQTT CONNACK sits unread and
the connection never completes. We keep the submodule pristine and fix it with a
**build-time patch** applied idempotently at CMake configure time:
`patches/0001-ioLibrary-recv-nonblock-issue-173.patch` (gates the early return on
`recvsize == 0`). CMake reverse-checks to skip an already-patched tree and only
WARNs if it can't apply (so an upstream fix that obsoletes it doesn't break the
build). The patch dirties the submodule working tree — that's expected; the
gitlink SHA is unchanged, so **do not commit the submodule change**. To remove
once upstream lands #173: delete the patch file + the apply block in CMakeLists.

### DHCP configurable hostname — build-time patch (2026-06-16)

ioLibrary's DHCP client hardcodes the option-12 hostname to `"WIZnet"` + a
6-hex-char MAC suffix in a fixed 7-byte `HOST_NAME[]` array (`dhcp.c`). To get a
clean, configurable name (so the router registers e.g. `lights` in its local DNS
— the cheap "Option A" instead of a socket-consuming mDNS responder), a second
build-time patch `patches/0002-dhcp-configurable-hostname.patch` widens
`HOST_NAME[]` to 32 bytes and drops the MAC suffix. `netif.c` overwrites the
global from config (`extern uint8_t HOST_NAME[]`) in `netif_init`, before the
first DISCOVER/REQUEST; empty leaves the compiled default. The name is the
**`net_hostname`** config field (default `lights`), settable via CLI
`net set hostname=<label>` and the web Network tab, persisted in `/config.json`,
carried in the launch snapshot, and **applied on reboot** (like the rest of
`net`). Validated as a single RFC-1123 label (letters/digits/hyphen, ≤31).
Applied + idempotency-checked exactly like the recv patch above; same
**do-not-commit-the-submodule** rule.

### MQTT publish coalescing — transport writev

`transport_writev` in `mqtt_app.c` implements coreMQTT's optional vectored-send
callback: it gathers a packet's fragments (fixed header, topic-len, topic,
payload) into one buffer and sends once. Without it, coreMQTT calls `send()` per
fragment and each ioLibrary `send()` issues an immediate W5100S `SEND` (no Nagle),
so one PUBLISH left as 3–4 tiny TCP segments. Verified on the wire: each PUBLISH is
now a single segment.

### CLI echo defaults ON (2026-06-09)

`echo_on` defaults to **true** (`serial_protocol.c`): the CLI is now a human-only
interface (machine clients went away with the on-board bridge), so local echo +
backspace editing is what you want. `echo off` restores a clean byte stream.

## Config web UI (core 1) — 2026-06-09

An embedded HTTP server on core 1 (`src/net/http_server.c`) serves a single-page
config app (in `web/`, Vite + vanilla JS) as a **LAN-only fallback** for
commissioning a device before HA. Runs alongside MQTT on the W5100S. Full detail
in `README.md` (“Config Web UI”). The load-bearing facts:

- **W5100S has only 4 sockets** (0=DHCP, 1=MQTT) → just **2** for the web server
  (sockets 2–3). That's why the UI **polls** (`GET /api/state` every ~1.5 s)
  instead of SSE — SSE would pin one of two sockets. `netif` lends the **idle
  DHCP socket 0** as a revocable 3rd socket between renewals
  (`netif_web_sock0_enabled()`); invariant: **never call `DHCP_run()` while
  socket 0 is lent** (its self-heal at `dhcp.c:797` would reopen UDP over the web
  TCP socket). Static IP → socket 0 is web-owned permanently.
- **Read model** is `web_state.c`: a core-1 cache fed from the *same* event drain
  `ha_bridge` consumes (the drain now calls `web_state_on_event` in both the
  connected pump and the disconnected discard path, so the UI stays live exactly
  when MQTT is down). Core 1 pushes one `IC_CMD_RESYNC` at boot to hydrate it
  independent of MQTT. **Writes** become typed `ic_cmd_t` to core 0 — the new
  `IC_CMD_SET_NETCFG/MQTTCFG/AUTH` + `de_mode` in `SET_CFG`. Config stays
  **core-0-owned** (single writer to `/config.json`).
- **Auth:** Basic, creds in `/config.json` (`web_user`/`web_pass`, default
  admin/admin), in the core-0→core-1 snapshot. Plain HTTP (no TLS on W5100S).
- **Same hot-path rule as MQTT:** *do NOT log in the http recv/send pump.* All
  http I/O is non-blocking, one ≤2 KB chunk per socket per tick, one request per
  connection then close — keeps MQTT + the watchdog healthy.
- **Build requires Node:** CMake runs the Vite build and embeds the gzipped
  single-file output as a `const` flash blob via `web/tools/gen-assets.mjs` →
  `build/generated/web_assets.c` (rebuilds only when `web/` changes). `make -C
  build` does it all.

## Building & flashing the TinyModbus gateway (attiny1604)

The DALI gateways run the firmware in `../TinyModbus`. When debugging the
end-to-end system it is fine to change and reflash that firmware too.

- **Flash:** just run `make` in the TinyModbus directory (default target builds,
  erases, and flashes via pymcuprog over the UPDI programmer).
  ```sh
  make -C ../TinyModbus
  ```
- **Compile-check only** (no hardware): `make -C ../TinyModbus build`.

## Serial / test rig

- The controller's USB-CDC CLI is at **`/dev/tty.usbmodem111201`** (baud
  irrelevant — native USB). Commands are line-based text; `help` lists them.
- Useful CLI commands for bus debugging: `dali_raw <slave> <cmd16> [twice]`,
  `gw_reboot <slave>` (software-resets a wedged gateway), `get_input`/`get_holding`,
  `log dbg` (raise verbosity to capture Modbus traffic).
- End-to-end test harness: `test/run_tests.py`. Run it with the venv:
  ```sh
  .venv/bin/python test/run_tests.py            # build + flash + test
  .venv/bin/python test/run_tests.py --no-build --no-flash
  ```
  It builds, flashes, waits for discovery, raises the log level, runs the tests,
  and dumps the captured controller logs (with a heuristic diagnosis) on failure.

## Test rig wiring

Two devices share **Modbus slave id 2** on the RS-485 bus: a relay board and a
TinyModbus DALI gateway driving an LED ballast. DALI tests target slave 2.
(Two devices answering the same slave id can collide on the wire — watch the
captured logs for CRC/timeout symptoms.)

## Logic-analyzer line traces (Saleae)

A Saleae Logic 8 probes the gateway lines: **D0 DALI-TX, D1 DALI-RX, D4 RS485-RX
(controller→gateway), D5 RS485-TX (responses), D6 DE, D7 = PB3 firmware marker**.

There are TWO ways to drive it. **Prefer the MCP** (below); fall back to the
Python automation wrapper only for tightly-timed scripted runs.

### Preferred: the `saleae` MCP server  ← use this by default

Registered in `~/.claude.json` (user scope) as `saleae` →
`http://127.0.0.1:10530/` (streamable HTTP, "saleae-logic2" server). Exposes the
full automation API as native tools: `start_capture` (timed / manual /
**digital-trigger** modes), `add_analyzer` + `add_high_level_analyzer` (Async
Serial → Modbus decode), `export_raw_data_csv`, `export_data_table_csv`,
`save_capture`/`load_capture`, etc.

Why it's preferred for this project:
- Native tool calls — no venv, no `logic2-automation` package, no wrapper to
  maintain. Fewer moving parts.
- `start_capture` returns a `captureId` immediately (non-blocking), so
  synchronized stimulus still works: `start_capture` → drive the controller over
  serial (Bash/Python) → `stop_capture` → `export_*_csv` → parse the CSVs. The
  custom byte-vs-edge / RXC-marker correlation analysis is unchanged — it just
  parses the exported `export_raw_data_csv` (digital edges) and
  `export_data_table_csv` (decoded Modbus) files.
- New capability the wrapper lacked: **digital-trigger capture** — arm on a
  pulse/edge to catch intermittent events instead of hoping a timed window
  overlaps them.
- Prereq: Logic 2 running with the probe connected and its MCP/automation
  endpoint up on :10530. Set the Async Serial analyzer **baud to 19200** (the
  current bus speed). Use timed (auto-stopping) captures; a killed manual
  capture wedges the app in "recording" until restart.

### Fallback: `test/saleae_capture.py` (gRPC automation server, :10430)

Still useful when capture + serial stimulus + parse must run atomically in one
threaded Python script. Needs the venv (`logic2-automation`) and **Preferences →
Automation Server enabled** (port 10430). `SaleaeCapture(channels=[...],
baud=19200, duration=N)`; pass the current bus baud or the decode is garbage.
`test/dali_tx_probe.py` and `run_tests.py --capture` use it.
Gotcha: captures have a **multi-second arming latency** — keep bus traffic
flowing for the whole window and fire the event of interest a couple seconds in.

## Root cause found & fixed (2026-06-05): TinyModbus RX interrupt starvation

The gateway intermittently went deaf to Modbus (received bytes but never
answered), needing a manual reset. Root cause, pinned down with a D7/PB3 firmware
marker on the Saleae: `start_next_dali_work()` ran `pick_work()` — a scan of up to
~192 ballast-flag entries — inside a **global `ATOMIC_BLOCK`**. On every idle
main-loop pass it held all interrupts off for hundreds of µs, overrunning the
USART's 2-byte RX FIFO and dropping ~25% of incoming bytes → every frame failed
CRC → deaf. A running DALI scan set `dali_busy`, which gated that call out — which
is why it "worked during a scan" and on the old shorted board (whose bus never
went idle). **Not sleep, not the hardware.** Fixed in TinyModbus by moving the
scan out of the global-interrupt-off region (commit on `main`).

Diagnosis tip: PB3 on the gateway = logic-probe D7, a free firmware timing marker
(`PORTB.OUTSET/OUTCLR/OUTTGL PIN3_bm`) — invaluable for bisecting where a frame is
lost in the RX path.

## Second issue (2026-06-06): 38400 RX-sampling margin → back to 19200

After the starvation fix we ran at 38400 and saw **semi-frequent Modbus timeouts
under heavy DALI load** (retries succeeded). Pinned down with RX **error counters
added to the gateway** (FC04 `0x0300`, see below) plus a Saleae byte-for-byte
compare:

- `rx_overflow` stayed **0** — NOT FIFO overflow / not the starvation bug again.
- `crc_error`/`frame_error` rose 1:1 with the timeouts. The frames were **clean on
  the wire** (Saleae decoded 104/104 with valid CRC) but the ATtiny **latched
  mis-framed bytes** — often only 7 of 8, with bytes merged — i.e. it lost
  bit-alignment mid-frame. A receive-**sampling** failure, not the line (50 cm,
  terminated).
- Halving the baud to **19200** made `crc_error`/`frame_error` go to **0** over the
  identical load (0 timeouts). 

Root cause: the ATtiny's **internal-RC oscillator (~±2-3%) is marginal for 38400 RX
sampling**; under load it tips past the UART receive tolerance → a stop bit sampled
early → missed next start bit → frame shifts. **19200 doubles the bit-time and the
margin, and is now the canonical baud** (controller default + probe baud, gateway
USERROW `BAUD_INDEX=1`). This is exactly what the original code comment warned;
the starvation bug was a *separate*, earlier fault.

### Gateway RX error counters (FC04 `0x0300`, 16-register read)

Permanent diagnostics, read with `get_input <slave> 0x0300 16`:
`reg0 rx_overflow` (FIFO overflow / lost byte) · `reg1 frame_error` (USART
framing/parity) · `reg2 crc_error` (frame arrived, CRC≠0; foreign-slave frames
excluded) · `reg3 frames_ok` (denominator) · `reg4 last_fail_len` · `reg5..12`
the bytes of the last CRC-failed frame. Counters are zeroed on gateway reset.
Use them to confirm any RX fix keeps `crc_error`/`frame_error` flat under load.

### Gateway sysinfo / reboot diagnostics (FC04 `0x0310`, 4-register read) — 2026-06-16

Read with `get_input <slave> 0x0310 4` (all big-endian):
`reg0..1 uptime_s` (32-bit seconds since boot) · `reg2 boot_count` (reboots since
the last cold power-on) · `reg3 reset_reason` (low byte = ATtiny `RSTCTRL.RSTFR`:
bit0 PORF, 1 BORF, 2 EXTRF, 3 WDRF, 4 SWRF, 5 UPDIRF). **No
EEPROM is used** (it would wear out under a watchdog crash-loop at ~8.2 s/reboot):
reset_reason is the hardware-latched RSTFR (read + write-1-to-clear at boot),
uptime is a RAM counter in the RTC PIT ISR, and boot_count lives in `.noinit`
SRAM guarded by a magic word — it survives warm resets (WDT/`SWRR`/external RESET)
and restarts on a true power-on. The master spots a gateway reboot when
`uptime_s` drops or `boot_count` changes.

**Controller poll + HA sensors (2026-06-17):** `dirty_poll.c` reads this block for
each DALI gateway every `gw_diag_interval_ms` (cfg key, default 30000, 0=off),
piggybacked on the existing per-gateway dirty-poll loop (not a separate poller),
and emits `IC_EVT_GW_DIAG`. `ha_bridge` publishes JSON to
`<prefix>/gateway/<slave>/diag` and discovers three **diagnostic sensors**
(Uptime/Reboots/Reset Reason) grouped under the *existing* gateway device
`lighting_dali_gw_<slave>`. `reset_reason` is decoded to text
(watchdog/software/external/brownout/power-on/updi) via `reset_reason_str`,
priority-ordered so a multi-flag cold boot reports the most diagnostic cause.

### DALI scan reads settings AND level (2026-06-16)

The gateway's `SYNC_READ_FULL` flag was **renamed `SYNC_READ_SETTINGS`** — it only
re-fetches the 7 config fields (min/max/fade/power/fail/groups), NOT the live arc
level (that's `SYNC_READ_LEVEL` = QUERY STATUS + QUERY ACTUAL LEVEL). The boot
scan (`main.c`) and the master `SCAN_TRIGGER` (`0x1010`) now arm **both** flags;
previously they armed settings only. Symptom that drove this: after a gateway
reboot `regs_init()` zeroes block0, the settings scan leaves `brightness=0`, and a
physically-ON ballast reported OFF until a command or the slow periodic re-poll
cursor happened to touch it ("2:6 shows 0 while on"). Arming the level read makes
a freshly-booted/rescanned gateway converge to true state on its own. The
controller side also got a fix: `dirty_poll_full_scan` BLOCK0 (arc/status) reads
are now guarded by a per-chunk `b0_chunk_ok[]` flag (matching BLOCK1-3), so a
failed Modbus chunk read never emits uninitialised stack data as a brightness.

## Gateway recovery

`gw_reboot <slave>` (CLI) software-resets a gateway via its `MBREG_SYSTEM_RESET`
register (`set_holding <slave> 0x1015 0xA5A5`). Handy if a gateway ever wedges.
After it reboots, read `get_input <slave> 0x0310 4` — `reset_reason` bit4 (SWRF)
confirms the software reset, and `boot_count` increments.

## DALI commissioning, collisions & device identity (2026-06-06)

New ballasts often arrive pre-set to a short address (usually 0), colliding with
existing gear. Tooling to detect and fix that, anchored on the device **serial
number** (stable; the short address is not).

### CLI commands (controller)
- `dali_raw <slave> <cmd16> [twice]` — send a raw 16-bit DALI frame and **report
  the result** (`ok 0x00FF` / `nak` / `err malformed`). Polls FC04 `0x0108`.
- `read_mem <slave> <addr> <bank> <off> <n>` — read a ballast memory bank via
  `SET DTR1/DTR0` + `READ MEMORY LOCATION` (0xC5). Re-sets DTR0 per byte (some
  gear don't auto-increment and bank-0 offset `0x01` NAKs as an unreadable hole).
- `identify <slave> <addr>` — GTIN + serial as hex; a **garbled** serial read →
  `collision` (two+ gear at one address answer with different serials).
- `blink <slave> <addr>` — flash one ballast (DAPC) to locate it physically
  (DALI control gear has no IDENTIFY command).
- `provision <slave> {all|new|<addr>}` — commission: `all`=full renumber,
  `new`=incremental (un-addressed gear only), `<addr>`=**re-provision the gear at
  one collided short address** onto free addresses.

### Gateway commissioning modes (`MBREG_COMMISSION` 0x1014)
`0x0001` full · `0x0002` incremental · **`0x03YY` re-provision the gear at short
address YY** (mode 3 — reuses the binary-search commissioner; only touches the
colliding gear). Status read-back: `0x00xx` idle, `0x01xx` running, `0x02xx`
complete (count), `0x03xx` error.

**DALI gotcha that shaped this:** you can't just `INITIALISE`-by-address then
`PROGRAM SHORT ADDRESS` — PROGRAM only programs the gear the binary search has
*selected* (SEARCHADDR == its random address). So re-addressing must run the full
`INITIALISE → RANDOMISE → SEARCHADDR bisect → PROGRAM → VERIFY → WITHDRAW`, which
is why mode 3 lives in the gateway commissioner, not as raw controller commands.

### Memory bank 0 (mandatory, IEC 62386-102)
`0x00` last-addr · `0x02` last-bank · **`0x03-0x08` GTIN** (per-product, same for
identical fixtures) · `0x09-0x0A` fw ver · **`0x0B-0x12` serial** (unique per
device) · `0x13-0x14` hw ver. Verified on the rig's ballasts.

### Scripts (`test/`)
- `dali_inventory.py <slave> [--all]` — scan, print each device's GTIN/serial,
  flag collisions.
- `dali_xlsx_sync.py {populate|remap} [--dry-run]` — sync the `Lights` sheet of
  `Switch wiring.xlsx`. **populate**: record each device's GTIN+serial into its
  (Device,Address) row. **remap**: after a re-provision changed addresses, scan
  serial→address and rewrite each row's Address by matching its recorded serial.
  Writes a timestamped `.bak` first (openpyxl may drop charts/images).

### Workflow for a collision
1. `dali_inventory.py <slave>` → find collided addresses.
2. `provision <slave> <addr>` per collided address → split onto free addresses.
3. `blink`/`identify` the new addresses, update the spreadsheet — or run
   `dali_xlsx_sync.py remap` if serials were `populate`d beforehand.

## DALI per-ballast config — min/max/power-on/sys-fail/fade (2026-06-06)

For Home Assistant. The controller already fetched this config (full scan reads
gateway BLOCK1-3) but didn't expose it; now it does.

- **`state_dali` emit** carries the full config on every update:
  `min max power_on sys_fail fade_rate fade_time` (alongside `on/bri/fading/lamp_fail`).
- Get/set go through the **unified settings protocol** (see below): config
  fields (`min max power_on sys_fail fade_rate fade_time`) are just more `v`
  keys on `set_dali`/`dali <slave> <addr> key=val`, same as `on`/`bri`.
- Fields map to gateway holding-register **pairs** (high byte first):
  `0x0040+addr=(min<<8)|max`, `0x0080+addr=(power_on<<8)|sys_fail`,
  `0x00C0+addr=(fade_rate<<8)|fade_time`. A register write arms the gateway's DALI
  `STORE DTR AS …` sequence. Set is read-modify-write (preserves the pair's other
  field from cache) + a predictive cache update + re-emit.
- **Gotcha:** the ballast clamps out-of-range values — notably `min` to its
  **physical minimum** (`QUERY PHYSICAL MINIMUM` 0x9A; the rig's is 99), so
  `set min` below that is silently floored. A rescan reflects the actual stored
  value (the predictive emit shows the requested value until then).
- **Opcode fix:** `TinyModbus dali.h` had `QUERY MIN/MAX` **swapped** vs IEC 62386-102
  (`0xA1`=QUERY MAX, `0xA2`=QUERY MIN). The scan-store + controller-read swaps
  cancelled so reads looked right, but it left read and write byte-orders opposite.
  Corrected the opcodes and flipped the controller's BLOCK1 read (dirty_poll) to
  match, so read and write are now consistent at `(min<<8)|max`.

## Unified settings protocol — `{t, d, v}` (2026-06-07)

One envelope for **set, get, and report**, so a client (the Home Assistant
bridge) sends values in the *same shape it receives them*.  `d` addresses the
entity, `v` is a bag of field→value. Direction is in `t` (`set_*`/`get_*`
command; `state_*`/`config_*` report). `ack` is the command receipt; the
emitted `state_*` is the value confirmation.

| Domain | `d` | settable `v` keys | report |
|---|---|---|---|
| dali   | `{slave,addr}` | `on`(true/false/toggle) `bri` `min` `max` `power_on` `sys_fail` `fade_rate` `fade_time` | `state_dali` |
| relay  | `{ch}` | `on`(true/false/toggle) | `state_relay` |
| group  | `{slave,group}` | `on` `bri` · `members`[] (replace) / `add`[] / `remove`[] | `state_group` (incl. `members`) |
| cfg    | – | `long_press_ms` `dim_repeat_ms` `reversal_window_ms` `gw_diag_interval_ms` `baud`(reboot to apply) `relay_slaves`[] `dali_slaves`[] | `state_cfg` |
| mapping| `{fixture,button}` | `targets`(array) — empty deletes | `config_mapping` |

**DALI groups** are first-class entities: `state_group` reports `on/bri/members`
(`members` = the member short addresses), and membership is editable like any
other value — `dali 2 g5 members=0,1` (replace) / `add=2` / `remove=1` (CLI), or
`set_group` with `members`/`add`/`remove` arrays (JSON). The controller writes
the desired 64-bit mask to the gateway group-mask registers (`0x0110+`, byte-
swapped per word); the **gateway** diffs old↔new and issues the DALI
ADD/REMOVE-from-group commands (`dali_sync_arm_group_write`). Non-empty groups
are emitted in `resync`. A group level/on/off command also flags its members for
a re-read on the gateway (`apply_invalidation` walks the group mask → marks each
member `SYNC_READ_LEVEL`), so each member's `state_dali` refreshes to its actual
level shortly after — provided membership is correct (a stale/empty group mask
has nothing to flag, which is what made group commands look like a no-op before
the group-mask byte-order fix).

`cfg` is the whole **controller device config**, shown in HA like everything
else. `state_cfg` (and `hello`) also report read-only `fw_version` — built from
git: `<base>+<git describe --tags --always --dirty>` (e.g. `2.0.0+791ca0b-dirty`)
via `cmake/gen_version.cmake` → `build/generated/version.h`, regenerated each
build. CLI slave lists are comma form (`cfg dali_slaves=2,3`); JSON uses arrays.
`set_baud` was folded into `set_cfg`'s `baud` key. `state_cfg` also reports the
read-only **binding grid** dimensions `max_fixtures` `max_buttons` `max_targets`
so HA knows the fixture/button space.

**Bindings (mappings)** are fully in the scheme: `get_mapping`/`set_mapping`/
`del_mapping` (and `get_mappings` — formerly `get_config` — dumps them all) emit
`config_mapping`. CLI `map <fix> <btn> [targets=<s>:<a>,…]` — bare = get,
`targets=` (empty) deletes; address `a` is `N` (ballast) or `gN` (group), e.g.
`map 0 0 targets=2:1,2:g3`. Bulk load still uses `replace_mappings_*`.

**Persistence (2026-06-07):** bindings are stored as **line-delimited JSON** in
`/mappings.json` — a `{"format":"mappings","version":N}` header line, then one
`{"fixture":F,"button":B,"targets":[…]}` per binding, with explicit
(de)serialisation (`mapping_save`/`mapping_load`). This replaced a **versionless
raw `fwrite()` of `mapping_record_t`** (`/mappings.bin`), which was fragile: any
struct-layout change (or whole-table overwrite when the table was momentarily
empty) silently dropped every binding — that's how mappings were lost across a
firmware change while `/config.json` (tolerant JSON) survived. The new format is
struct-decoupled, version-tagged, tolerant of missing/extra fields, and
human-inspectable. `mapping_load` does a one-time migration from a valid legacy
`/mappings.bin` and re-saves as JSON.

```
set : {"t":"set_dali","d":{"slave":2,"addr":1},"v":{"min":120,"bri":200}}  → ack + state_dali
get : {"t":"get_dali","d":{"slave":2,"addr":1}}                            → state_dali
report (unsolicited, e.g. button/dirty-poll): {"t":"state_dali","d":{…},"v":{…}}
```

Fields are parsed by **name** from the whole line, so the `d`/`v` nesting is the
client's contract — the controller doesn't care where a field sits. Keys are
unique across `d`/`v` (no `addr` vs `addr_byte` collisions).

**CLI is the same model**, `key=value` pairs, **bare = get** (emits the JSON
report, same as `get_*`):
```
dali 2 1                     dali 2 1 bri=200 on=toggle      dali 2 1 min=120 max=200
dali 2 g3 bri=200            relay 0 on=true                 cfg long_press_ms=600
```
`show dali` stays as the human-readable text view. Bulk mapping load still uses
the `replace_mappings_*` transaction. The old flat commands (`cmd_dali`,
`cmd_relay`, `cmd_dali_group/broadcast`, `set_config`, `set_dali_config`) and the
old positional CLI verbs (`dali 2 1 on`) were **removed** — this envelope is the
only way.
