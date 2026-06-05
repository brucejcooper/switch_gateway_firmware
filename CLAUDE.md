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
(controller→gateway), D5 RS485-TX (responses), D6 DE**. Capture is scripted via
the Saleae Logic 2 automation API (`logic2-automation`, in the venv).

- Prereq: the Saleae Logic app must be running with **Preferences → Automation
  Server enabled** (port 10430), and the probe connected.
- `test/saleae_capture.py` — capture wrapper + Modbus(async-serial) decode + DALI
  edge analysis. Standalone: `.venv/bin/python test/saleae_capture.py --seconds 2`.
- `test/dali_tx_probe.py` — the key diagnostic: reboots the gateway, then captures
  while firing a raw DALI query and reports **whether the gateway actually drives
  DALI TX (D0)**. `.venv/bin/python test/dali_tx_probe.py`.
- `run_tests.py --capture` wraps the DALI test with a capture.
- Gotchas (the hard-won ones): captures have a **multi-second arming latency** —
  keep bus traffic flowing for the whole window and fire the event of interest a
  couple seconds in (dali_tx_probe.py does this). Always use timed (auto-stopping)
  captures; a killed manual capture wedges the app in "recording" until restart.

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
scan out of the global-interrupt-off region (commit on `fix/modbus-rx-starvation`).
Side effect: 38400 baud is now reliable (its shorter byte time made the overrun
worse), so the controller's default + probe baud is now 38400.

Diagnosis tip: PB3 on the gateway = logic-probe D7, a free firmware timing marker
(`PORTB.OUTSET/OUTCLR/OUTTGL PIN3_bm`) — invaluable for bisecting where a frame is
lost in the RX path.

## Gateway recovery

`gw_reboot <slave>` (CLI) software-resets a gateway via its `MBREG_SYSTEM_RESET`
register (`set_holding <slave> 0x1015 0xA5A5`). Handy if a gateway ever wedges.
