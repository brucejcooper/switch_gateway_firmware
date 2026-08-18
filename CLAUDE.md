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

## OTA firmware updates (pull-based, 2026-07)

The controller can pull signed firmware from `ota.8bitcloud.com` over plain HTTP,
stage it, verify an **ed25519 signature** (Monocypher, `lib/monocypher/`), and let
a custom **bootloader** commit it with crash-loop **rollback**. Modelled on the
brewery fleet (`git.8bitcloud.com/bruce/brewery`, `brewery_common`). Full detail:
**`tools/README.md`** (signing/release) and **`bootloader/README.md`** (boot flow,
flash map, rollback). Design lives in `src/{partition_map.h,boot_control.*,
fw_image.*,ota_stage.*}` and `src/net/{dns_resolve,http_client,ota,fwupdate}.*`.

**Status (2026-07-25): FULLY VALIDATED on the bench device** — boot chain
(after fixing a PRIMASK-masked-interrupts bug in the handoff, plus two
hardenings: vector-table sanity check + watchdog-armed factory jump, every
dead-end lands in BOOTSEL), a real end-to-end OTA (manifest from the dev HA's
`/local/`, install over the dev broker, staging→active commit + backup), and
**crash-loop rollback** (deliberate bad image → 3 attempts → BACKUP restored →
healthy boot). See `bootloader/README.md` "Hardware bring-up findings" and
"Re-running the bench OTA test". Production conversion remains to be done
(park-in-BOOTSEL procedure below). Flashing procedure gotcha:
**park the device in BOOTSEL first** (`picotool reboot -f -u`), load both
images, then one reboot — `picotool load -f` alone auto-reboots into a
half-converted flash.

**`cfg fw_url=<manifest-url>`** (CLI, persisted) overrides the compiled-in
manifest URL — a bench device polls a local server (e.g.
`http://<mac-ip>:8000/light_switch.json`) instead of `ota.8bitcloud.com`;
empty value reverts to the default. **Reboot to apply** (the CLI says so):
`fwupdate.c` reads the URL from the intercore snapshot, which core 0 builds once
at `core1_launch()`, so a `check` before the reboot silently polls the OLD url
and reports no update — which reads as a broken manifest.

### Flash layout (OTA build only — the default build is unaffected)

`partition_map.h`: bootloader 56K @0x0 · boot-control A/B 2×4K · **active 576K
@0x10010000** · staging 576K · backup 576K · littlefs 256K @0x1C0000 (unchanged, so
existing config survives the switch). The default `light_switch` target stays a
standalone image linked at 0x10000000; the OTA image is a **separate opt-in target**
`light_switch_ota` (`-DOTA_APP_LAYOUT=ON`) linked at 0x10010000 for the bootloader
to host. **Always OTA the `light_switch_ota` build, never plain `light_switch`.**

### One-time: convert a device to the OTA layout (physical flash, pending HW bring-up)

A device must be running the bootloader + a relocated app before OTA works. First
time only, over USB/SWD: flash `bootloader/build/bootloader.uf2` (loads at 0x0) and
the signed relocated app at `APP_OFF`. See `bootloader/README.md`. After that,
updates are over-the-air.

### Ongoing: release an update (do this whenever you ship a change)

Build the tools once (`tools/README.md`), then per release:

```sh
# 0. COMMIT first — FW_VERSION embeds `git describe`; a dirty tree yields an
#    unstable "...-dirty" version. A clean commit gives e.g. 2.0.0+abc1234.
# 1. Build the RELOCATED app + raw .bin
cmake -S . -B build_ota -DOTA_APP_LAYOUT=ON && make -C build_ota light_switch_ota
arm-none-eabi-objcopy -O binary build_ota/light_switch_ota.elf build_ota/light_switch_ota.bin
# 2. Sign it (dev key is tools/dev_privkey.bin — gitignored; rotate for prod).
#    BUILD_CTR is the anti-rollback build counter (tools/fw_build_counter) —
#    bump the file for next time only AFTER a successful sign.
BUILD_CTR=$(cat tools/fw_build_counter)
tools/fw_sign build_ota/light_switch_ota.bin tools/dev_privkey.bin "$BUILD_CTR" \
    build_ota/light_switch_ota.signed.bin
echo $((BUILD_CTR + 1)) > tools/fw_build_counter
# 3. Manifest — version MUST equal the NEW image's FW_VERSION (see gotcha below)
VER=$(sed -n 's/.*FW_VERSION "\(.*\)".*/\1/p' build_ota/generated/version.h)
tools/fw_manifest.sh "$VER" build_ota/light_switch_ota.signed.bin \
    http://ota.8bitcloud.com/light_switch.bin "Light Switch $VER" "" "notes" \
    > build_ota/light_switch.json
# 4. Publish BOTH at the URLs fwupdate.c polls:
#      http://ota.8bitcloud.com/light_switch.bin   <- light_switch_ota.signed.bin
#      http://ota.8bitcloud.com/light_switch.json  <- light_switch.json
```

**Version gotcha (the #1 footgun):** the device compares the manifest `version`
against its own `FW_VERSION` by **string inequality** (any difference = "update
available"). Set the manifest `version` to the *new image's* `FW_VERSION` (step 3
extracts it) so it (a) differs from what's installed → HA offers the update, and (b)
after install, `installed == latest` → HA reads "up to date". A hand-picked version
that never matches leaves a phantom update forever.

### On the device / in Home Assistant

`fwupdate.c` polls `light_switch.json` **hourly** (`FWCHECK_INTERVAL_S`, `fwupdate.h`)
plus on every MQTT (re)connect. On the **Switch Gateway Controller** HA device:
- **Firmware** (`update` entity) — installed vs latest + release notes; **Install**
  pulls → stages → verifies → reboots into the bootloader (which commits + can roll
  back a crash-looper).
- **Check for Updates** (button) — forces an immediate manifest re-check.

**Triggering a deploy programmatically** (agent/CI, no HA needed): after publishing
the files, poke the device directly over its HTTP API (Basic-auth, same as the rest):
`GET /api/ota` (status: installed/latest/available/state/pct), `POST /api/ota/check`
(force a re-check), `POST /api/ota/install` (start the pull+flash). Typical flow:
`POST /api/ota/check` → poll `GET /api/ota` until `latest` updates → `POST
/api/ota/install` → poll `GET /api/ota`/`GET /api/log` through the reboot. The same
triggers exist over MQTT (`<prefix>/update/check|install`, payloads `CHECK`/`INSTALL`).
The web UI's **Firmware** tab wraps these (`web/src/tabs/firmware.js`).

Trust is the ed25519 signature in the image trailer (baked-in `FW_PUBKEY`), verified
on-device and again by the bootloader — the manifest `sha256` is informational only.
The manifest `version` check is a bare string-inequality (no ordering), so the
trailer also carries a monotonic **anti-rollback build counter**
(`fw_trailer.build_ctr`, `tools/fw_build_counter`) that the device persists a
floor for and refuses to commit anything below — see `tools/README.md`'s
"Anti-rollback build counter" for the full mechanism and the release-time
contract (bump the counter file every release, never reuse/decrease a value).
**Caveats:** the *first* OTA over the factory image has no rollback fallback; rotate
the dev signing key before production (`tools/README.md`).

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

`echo_on` defaults to **true** (`cli.c`): the CLI is now a human-only
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
- **Auth:** Basic, default admin/admin unchanged from the user's perspective —
  but the PASSWORD itself is never stored, only an Argon2id hash + salt
  (`web_pass_hash`/`web_salt` in `/config.json`; `web_user` stays plaintext,
  an identifier not a secret). `src/web_auth.{h,c}` owns the KDF (a static
  32 KB work area shared by both cores under a try-lock — see that file's
  header for the concurrency argument) and the hex codec; `http_server.c`'s
  `check_auth()` caches one BLAKE2b digest after a successful KDF verify so
  the ~1.5 s `/api/state` poll doesn't re-run Argon2id on every request.
  Snapshot carries the hash+salt (hex text), not the password. Plain HTTP (no
  TLS on W5100S).
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

- The controller's USB-CDC CLI is at **`/dev/cu.usbmodem112201`** (baud
  irrelevant — native USB). Commands are line-based text; `help` lists them.
  The device number changes with the USB port — check `ls /dev/cu.usbmodem*`
  before assuming it.
- Useful CLI commands for bus debugging: `dali_raw <slave> <cmd16> [twice]`,
  `gw_reboot <slave>` (software-resets a wedged gateway), `get_input`/`get_holding`,
  `log dbg` (raise verbosity to capture Modbus traffic), `mbq [<slave>]` (queue
  transport FSM + protocol counters) and `mbq gw <slave>` (the gateway's own
  queue counters).
- End-to-end test harness: `test/run_tests.py`. Run it with the venv:
  ```sh
  .venv/bin/python test/run_tests.py            # build + flash + test
  .venv/bin/python test/run_tests.py --no-build --no-flash
  ```
  It builds, flashes, waits for discovery, raises the log level, runs the tests,
  and dumps the captured controller logs (with a heuristic diagnosis) on failure.

## Test rig wiring

Currently **only the DALI gateway at slave 3** is on the bus (ballasts 3.0, a
binary relay ballast, and 3.45, dimmable). Slaves 1 and 2 are configured but
absent, so `discover` reporting `1 ok, 2 fail` is the expected result, not a
fault. (Where two devices do share a slave id they can collide on the wire —
watch the captured logs for CRC/timeout symptoms.)

**DE mode must be `tx` (2026-08-08).** The rig now has a proper RS-485
transceiver, so holding DE asserted keeps the controller driving the bus and no
slave can ever answer — every transaction times out. `rs485 tx` (live, persisted)
is the correct setting; `rs485 always` only suited the previous non-standard
driver. Measured right after the swap, with the gateway's counters zeroed by
`gw_reboot 3`: 3048 frames, **0 rx_overflow / 0 frame_error / 0 crc_error / 0
internal faults**, across two soaks totalling 1300 DALI frames and 2305 Modbus
transactions with zero failures. For contrast, the pre-swap counters stood at
8868 frame errors and 5162 CRC errors against 29210 good frames.

**Diagnostics block sizes are exact.** `get_input <slave> 0x0300 16`,
`get_input <slave> 0x0310 4` and `get_input <slave> 0x0320 8` — asking for a
different count returns ILLEGAL DATA ADDRESS rather than a short read.

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


## RS-485 baud

**38400 is canonical** (controller default, gateway USERROW `BAUD_INDEX=2`,
probe baud). 9600 remains the factory/safe speed a device falls back to with an
unprogrammed USERROW, and boot discovery walks 38400 → 19200 → 9600 so a device
left on the old canonical speed is found and promoted rather than reported
unreachable.

**This reverses the previous verdict, and the reason matters.** 38400 used to be
recorded here as "works but is not clean, ~0.7 % bad frames, not worth trading a
zero error rate for", blamed on the ATtiny's internal RC oscillator mis-framing
bytes that were clean on the wire. The observation was right and the diagnosis
wrong. That is the signature of a documented tinyAVR silicon erratum — the USART
start-of-frame detector re-triggering mid-frame when RXDATA is read while the
next byte is arriving — not the oscillator. Scoped on the Saleae, both ends
transmit within 0.8 % of nominal at both bauds, and the OSC20ERR trim's sign is
correct. See the erratum block above `modbus_init()` in
`TinyModbus/src/modbus.c`.

**Measured 2026-08-14**, gateway counters zeroed *after* the baud settled, 40–60 s
of saturated traffic per cell:

| | SFDEN in Active (before) | SFDEN only around sleep (after) |
|---|---|---|
| 19200 | 0.13 % bad | **0.00 %** — 0 of 3046 frames |
| 38400 | 0.85 % bad | **0.00 %** — 0 of 6582 frames |

19200 was never actually clean either. With the erratum fixed, 38400 takes the
average transaction from 12.65 ms to **8.96 ms (−29 %)**, and a 600-pair soak
runs VERDICT clean with 0 transaction failures of 5791.

Note the old 8868 frame / 5162 CRC errors against 29210 frames were the **bad
transceiver**, not the baud — the driver was the dominant fault by far. Watch
`crc_error`/`frame_error` in the gateway diagnostics below to confirm any change
keeps them flat.

Switching baud: `set_slave_baud <s> <rate> dali` (persists to the gateway's
USERROW) THEN `baud <rate>` on the controller. If that ever strands a gateway,
recover over UPDI — `make -C ../TinyModbus get_config` reads USERROW, `make
configure` rewrites it. USERROW byte 0 is `slave_addr - 2` (so `01` = slave 3),
byte 1 is the baud index (0=9600, 1=19200, 2=38400). Reading USERROW over UPDI
resets the chip (shows up as `reset_reason` bit5 UPDIRF).


### Gateway RX error counters (FC04 `0x0300`, 16-register read)

Permanent diagnostics, read with `get_input <slave> 0x0300 16`:
`reg0 rx_overflow` (FIFO overflow / lost byte) · `reg1 frame_error` (USART
framing/parity) · `reg2 crc_error` (frame arrived, CRC≠0; foreign-slave frames
excluded) · `reg3 frames_ok` (denominator) · `reg4 last_fail_len` · `reg5..12`
the bytes of the last CRC-failed frame · **`reg13` internal fault count** ·
**`reg14` `[tm_err_t | tm_fault_site_t]` of the first internal fault**.
Counters are zeroed on gateway reset.

**reg13/reg14 (2026-07-29)** are the gateway's own invariant breaks — errors
raised where no caller could act on them (a bad slot index in the DALI completion
ISR, a refused command launch; see `TinyModbus/src/errors.h`). A healthy gateway
reads **0** for its whole uptime. Non-zero reg13 means a queue slot may be
stranded EXECUTING or a DALI launch was skipped; reg14 names which site.
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

### Gateway queue counters (FC04 `0x0320`, 8-register read) — task 08, queue v4

Read with `get_input <slave> 0x0320 8`, or decoded by the CLI as
`mbq gw <slave>`. All 16-bit wrapping counters, zeroed on gateway reset, read on
demand and **never polled hot** (queue-protocol §6):
`reg0 accepts` · `reg1 a_full` · `reg2 executes` · `reg3 results` ·
`reg4 acks` · `reg5 stalls` · `reg6 q_discard` · `reg7 spare` (reads 0).

Two of them look alarming and are not — both are the system working:

- **`a_full` non-zero is NORMAL and is the saturation signal.** It means the
  gateway's queue is full, i.e. the DALI engine is fully fed — the goal state.
  The master re-offers immediately. Under one saturated gateway ~72 % of
  transactions are refusals, and that share is the HEADROOM gauge: those probes
  convert into useful enqueues as more gateways are added. (It was described
  here as a canary when the master paced itself against an estimate and A_FULL
  was meant to be unreachable; A_FULL is the flow control now.)
- **`stalls` non-zero is normal.** It counts backpressure *episodes* (rising
  edge) — the gateway stalling its DALI arbiter rather than dropping a result
  whenever the master stops acking. That is the rule working, and the
  `mbq no-ack <slave> <n>` drill drives it deliberately.
- `q_discard` counts malformed FC 0x41 frames discarded without a reply.

`TinyModbus/src/registers.h` is authoritative for the map; this table, the
gateway's own `CLAUDE.md`, and the controller's `mbq` parser mirror it and must
all move together.

### Scans read status, level and groups

An interrogation reads QUERY STATUS + QUERY ACTUAL LEVEL for every present
ballast, and group membership on a full scan. It is armed at boot, by the `scan`
CLI verb, by `POST /api/scan`, and by the Rescan button in the Lights tab — all
asynchronous; the main loop drives them and state converges over a few seconds.


## Gateway recovery

`gw_reboot <slave>` (CLI) software-resets a gateway via its `MBREG_SYSTEM_RESET`
register (`set_holding <slave> 0x1F05 0xA5A5` — `TinyModbus/src/registers.h` is
authoritative; an earlier note here said `0x1015`, which was never the register).
Handy if a gateway ever wedges.
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

### Commissioning modes
`all` full renumber · `new` only gear without a short address · `address <N>`
re-provision the gear at one collided short address. The sequence runs on the
CONTROLLER (`dali_commission.c`) as ordinary queued DALI frames.

**DALI gotcha that shapes this:** you can't just `INITIALISE`-by-address then
`PROGRAM SHORT ADDRESS` — PROGRAM only programs the gear the binary search has
*selected* (SEARCHADDR == its random address). So re-addressing must run the full
`INITIALISE → RANDOMISE → SEARCHADDR bisect → PROGRAM → VERIFY → WITHDRAW`.

`new` finding nothing on a fully-addressed bus is the CORRECT result, reported as
complete, not an error — INITIALISE with the no-short data byte matches nothing
when every ballast already has an address.

### Memory bank 0 (mandatory, IEC 62386-102)
`0x00` last-addr · `0x02` last-bank · **`0x03-0x08` GTIN** (per-product, same for
identical fixtures) · `0x09-0x0A` fw ver · **`0x0B-0x12` serial** (unique per
device) · `0x13-0x14` hw ver. Verified on the rig's ballasts.

### Scripts (`test/`)
- `dali_inventory.py <slave> [--all]` — scan, print each device's GTIN/serial,
  flag collisions.

### Workflow for a collision
1. `dali_inventory.py <slave>` → find collided addresses.
2. `provision <slave> <addr>` per collided address → split onto free addresses.
3. `dali_inventory.py <slave>` again to confirm the split, then `blink`/`identify`
   each new address to physically locate it. **HA is the single source of
   truth** for device identity now — rename/reposition the light in Home
   Assistant (the `lightswitch_mapper` sidebar panel, or the device registry
   directly). There is no spreadsheet step: `Switch wiring.xlsx` is retired as
   a workflow input and kept in the repo only as frozen wiring documentation;
   a read-only spreadsheet snapshot of the live HA layout can be regenerated
   any time with `mapper/tools/export_mappings.py` (see `mapper/README.md`).

## DALI per-ballast config — min/max/power-on/sys-fail/fade (2026-06-06)

For Home Assistant. The controller already fetched this config (full scan reads
gateway BLOCK1-3) but didn't expose it; now it does.

- **`state_dali` emit** carries the full config on every update:
  `min max power_on sys_fail fade_rate fade_time` (alongside `on/bri/fading/lamp_fail`).
- Get/set go through the **unified settings protocol** (see below): config
  fields (`min max power_on sys_fail fade_rate fade_time`) are just more `v`
  keys on `set_dali`/`dali <slave> <addr> key=val`, same as `on`/`bri`.
- Set goes over raw DALI as a DTR0-load + SET pair through the command queue.
  Nothing is written to the cache or published until the query-back reports what
  the ballast actually stored.
- **Gotcha:** the ballast clamps out-of-range values — notably `min` to its
  **physical minimum** (`QUERY PHYSICAL MINIMUM` 0x9A; the rig's is 99), so
  `set min` below that is silently floored. A rescan reflects the actual stored
  value; nothing is published until the read-back confirms it.
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
`set_group` with `members`/`add`/`remove` arrays (JSON). The CONTROLLER diffs
membership against its own cache and issues the DALI ADD/REMOVE FROM GROUP
frames itself, then re-reads each touched ballast — the measured read-back is
what gets published, so a write that did not take cannot linger. A group
command likewise marks every member for verification, and each member's
measured level recomputes the group (on = any member on, brightness = max).

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
