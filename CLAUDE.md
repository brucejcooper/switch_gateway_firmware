# Controller firmware — working notes

Raspberry Pi Pico firmware that bridges a USB-CDC CLI to an RS-485 Modbus RTU bus
of TinyModbus DALI gateways and relay boards.

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
  DALI slave. It is ready once it emits the `hello` JSON / `serial: hello` log.

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

## Gateway recovery

`gw_reboot <slave>` (CLI) software-resets a gateway via its `MBREG_SYSTEM_RESET`
register (`set_holding <slave> 0x1015 0xA5A5`). Handy if a gateway ever wedges.
