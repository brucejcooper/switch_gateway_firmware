# Switch Gateway Lighting System

A complete home lighting control system built around a Raspberry Pi Pico 2 as the
central controller. The Pico owns all physical hardware, controls lights in response
to button presses, and exposes the entire system to Home Assistant via MQTT.

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│  RS-485 bus @ 19200 baud  (canonical; firmware auto-recovers    │
│                            slaves stuck at the factory 9600)    │
│                                                                 │
│  Pico 2 (Modbus master, single source of truth)                 │
│  ├── Waveshare 32CH Relay Board (slave 1)   — 32 relay coils    │
│  ├── TinyModbus DALI Gateway (slave 2)      — up to 64 ballasts │
│  └── TinyModbus DALI Gateway (slave 3)      — up to 64 ballasts │
│                                                                 │
│  Pico 2 also scans                                              │
│  └── 24 switch fixtures × up to 7 buttons each (PIO matrix)     │
└─────────────────────┬───────────────────────────────────────────┘
                      │ USB CDC serial, 115200 baud
                      │ Line-delimited JSON
                      ▼
┌─────────────────────────────────────────────────────────────────┐
│  Linux host (Docker)                                            │
│  ├── mqtt_bridge.py  — serial ↔ MQTT translation                │
│  ├── Mosquitto       — MQTT broker                              │
│  └── Home Assistant  — dashboards, automations, scenes          │
└─────────────────────────────────────────────────────────────────┘
```

### Key design principles

**Pico is the master.** It owns the RS-485 bus and all lighting state. HA is a
client — it can send commands, but if HA goes offline, buttons and lights
continue working normally.

**State is never assumed.** The Pico only updates its state cache from hardware
readback (dirty-bit polling from TinyModbus), never from the commands it sent.
This correctly handles DALI min/max level clamping and ballast-side fade curves.

**No configuration files needed for the bridge.** The bridge learns everything
from the Pico's `hello` message and subsequent state messages. All friendly
names and room assignments are done in HA's device registry UI.

---

## Hardware

| Device | Role | Modbus Slave ID |
|--------|------|-----------------|
| Waveshare Modbus RTU Relay 32CH | Relay control — lights, fans, switches | 1 |
| TinyModbus ATtiny1604 DALI Gateway #1 | DALI bus A — up to 64 ballasts | 2 |
| TinyModbus ATtiny1604 DALI Gateway #2 | DALI bus B — up to 64 ballasts | 3 |
| Raspberry Pi Pico 2 | Main controller, button scanner, Modbus master | — |

Function-code mappings and register addresses for the Waveshare board:
<https://www.waveshare.com/wiki/Modbus_RTU_Relay_32CH>.

### Pin assignments (Pico 2)

| Function | GPIO |
|----------|------|
| RS-485 UART TX | 4 |
| RS-485 UART RX | 5 |
| RS-485 DE (drive enable) | 1 |
| Button matrix SER | 6 |
| Button matrix CLK | 7 |
| Button matrix COL base | 8 |

### Baud rate setup

The canonical bus speed is **19200** — the reliable rate for the TinyModbus
slaves' internal-RC oscillator (38400 is marginal and drops occasional frames).
The firmware auto-recovers slaves stuck at the factory-default 9600 baud:
on boot it pings each slave at 19200 first, falls back to 9600 on timeout,
sends the per-device "set baud to 19200" command, and verifies the slave
came back at 19200 — all logged under the `disc` module tag.

| Slave kind | Set-baud register | Value |
|---|---|---|
| Waveshare 32CH Relay | FC06 `0x2000` | `0x0002` |
| TinyModbus DALI gateway | FC06 `0x1013` | `0x0001` |

The set-baud writes are persisted on the slaves, so the fallback path only
fires once per device. After the first successful boot, subsequent boots
take <100 ms total for all three slaves.

If a slave is unreachable at *both* 19200 and 9600, the discovery logs
`E disc: slave N (...) unreachable at both bauds` and boot continues
without that slave — the relay sync / DALI scan will see it as missing.
Most commonly this means the slave is unplugged, has a different baud
rate (e.g. left at 38400 from an earlier config), or is at a different slave
address than configured. Manual recovery via a USB-485 dongle and the per-device
commands above will get it back online. The slave-address writes for
factory-fresh devices (TinyModbus FC06 `0x1012` value `0x0002` or `0x0003`;
Waveshare FC06 `0x4000` value `0x0001`) are also one-time and must be done
manually before the first boot.

---

## Firmware

### Building

Requires the Pico SDK and pico-vfs. Set `PICO_SDK_PATH` before configuring.

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```

Flash `light_switch.uf2` by using the picotool command.  For example

```bash
build % make && picotool load -f -x light_switch.uf2 && sleep 2 && pyserial-miniterm /dev/tty.usbmodem111201
```

### Boot Sequence

Each boot, before the main loop starts, the firmware runs:

1. **Filesystem + alarm pool init** — pico-vfs mount, hardware timers.
2. **Config + mapping load** — `/config.json` and `/mappings.bin`. Defaults
   if absent.
3. **State cache init** — zeros the in-memory entity state.
4. **Modbus init** — UART at the persisted baud rate (19200 default).
5. **Bus discovery** — for each configured slave, ping at 19200, fall back
   to 9600, auto-reconfigure if needed (see "Baud rate setup" above).
6. **DALI full scan** — for each TinyModbus gateway, blocking read of every
   ballast bank + group masks, populate state cache, emit per-ballast
   `state_dali` events and a `scan_complete`.
7. **Relay full sync** — read all 32 coils on the Waveshare board into the
   state cache, emit `state_relay` events.
8. **Button + serial init** — PIO scanner, USB CDC parser.
9. **Watchdog enabled** — 5 s timeout; only after this point are stalls
   fatal. Discovery and scans run unwatchdog'd because they can take
   seconds in failure paths.
10. **Hello emitted** — first JSON event the bridge sees.

### Source file overview

| File | Purpose |
|------|---------|
| `main.c` | Startup, init sequence, main loop |
| `modbus.c/h` | Modbus RTU master — ISR-driven, non-blocking |
| `bus_discovery.c/h` | Boot-time slave ping + auto baud recovery |
| `button_actions.c/h` | PIO button matrix scanner |
| `button_fsm.c/h` | Per-button state machine (toggle/dim/reversal) |
| `button_fsm_emit.c` | JSON `button` event emission |
| `config.c/h` | Global config, persisted to `/config.json` |
| `mapping.c/h` | Button-to-light mappings, persisted to `/mappings.bin` |
| `state_cache.c/h` | In-memory cache of relay + DALI ballast + group state |
| `lighting_actions.c/h` | Modbus command dispatch, state event queue |
| `dirty_poll.c/h` | TinyModbus dirty-bit polling, background scans |
| `serial_protocol.c/h` | JSON line protocol over USB CDC |
| `dali_provision.c/h` | DALI commissioning state machine |
| `serial_protocol_provision.c` | Provision message handling |
| `log.c/h` | Logging facility (LOG_INFO/WARN/ERR/DBG macros) emitting to USB CDC |

### Main loop

```c
while (true) {
    modbus_poll();          // advance Modbus state machine (ISR-driven, non-blocking)
    watchdog_update();
    button_fsm_tick();      // classify raw button events, dispatch commands
    dirty_poll_tick();      // check dirty bits, read changed ballast state
    dali_provision_tick();  // advance commissioning state machine if active
    serial_poll();          // read inbound JSON, emit outbound state events
    watchdog_update();
}
```

Everything is single-core. All timing is cooperative — no blocking calls in
normal operation.

---

## Button Behaviour

Each of the 24 switch fixtures can have up to 7 physical buttons. Each button
can control multiple lights simultaneously (all mapped lights mirror each other).

### Gesture recognition

| Gesture | Action |
|---------|--------|
| Short press (< 500 ms) | Toggle all mapped lights on/off |
| Hold (≥ 500 ms) | Start dimming: down if any light is on, up if all are off |
| Release while dimming | Stop. Light settles at current level |
| Re-press within 400 ms of release | Reverse direction, continue dimming |
| Re-press after 400 ms | Treated as a new short press |

Dimming uses DALI Up/Down commands at the ballast's own fade rate, issued
at 200 ms intervals (5 steps/second). The actual brightness change per step
is controlled by the ballast hardware.

### Timing defaults (configurable via `set_config`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `long_press_ms` | 500 ms | Hold threshold before dimming begins |
| `dim_repeat_ms` | 200 ms | Interval between DALI Up/Down commands |
| `reversal_window_ms` | 400 ms | Window after release to trigger direction reversal |

### Button events sent to HA

Every button gesture is also published to HA as an event entity, so HA
automations can react to button presses independently of the built-in mapping.

| Event type | When fired |
|------------|------------|
| `press` | Short press released |
| `release` | Released after any hold |
| `long_press_start` | Hold threshold crossed (also fires on direction reversal) |
| `long_press_repeat` | Each dim-rate tick while held (~5 Hz) |

---

## Lighting State Tracking

### Relay channels

The Pico sends a coil write and updates its cache immediately from the Modbus
write-callback. Relay state is instantaneous — no polling needed after a command.

The Waveshare board's state is also read in full at boot to synchronise the
cache with the hardware's actual state (handles Pico restarts while power was on).

### DALI ballasts — dirty-bit polling

DALI ballasts go through TinyModbus, which maintains a dirty bitmask (FC04
registers 0x0104–0x0107, one bit per ballast). A bit is set whenever a
ballast's block-0 cache (status byte | arc level) changes, and cleared when
read.

The Pico's polling loop:

1. Every **100 ms** when a command is in flight: read the 4-register dirty
   bitmask (one RTU frame). For each dirty bit, read that ballast's single
   block-0 register via FC03 at `0x0000 + addr` and emit a `state_dali`
   event.
2. Every **60 s** (background): read the present bitmask and all block-0
   status bytes. Emit `presence` events for any changes. Emit `fault` events
   for ballasts with lamp-failure or power-cycle bits set.
3. At **boot**: trigger a TinyModbus bus scan, then blocking-read all four
   data blocks (block 0–3) and the present bitmask for each slave.

The Pico never caches a DALI arc level from the command it sent — it always
waits for the dirty bit and reads the hardware. This correctly handles:
- DALI min/max level clamping (commanded 10, got 20 → reports 20)
- Real-time fade tracking (arc level changes on each dirty-bit read during fade)
- Power-cycle events (ballast restarted → new arc level read immediately)

#### TinyModbus register contract

The `task-1-flag-driven` refactor of the TinyModbus firmware moved the
per-ballast cache from input registers to holding registers and retired
several action / DTR-setter registers. The Pico's master code follows the
new contract:

- **Per-ballast cache (FC03 read, FC06/FC10 write) at `0x0000–0x00FF`.**
  Block 0 (status | brightness) is `0x0000–0x003F`; min/max, power-on/
  sys-fail, and fade rate/time follow at `0x0040–0x00FF`. Writing the
  brightness register's low byte (DAPC level 0–254) is how the Pico drives
  ballast level changes — the firmware caches predictively and dispatches
  the DAPC asynchronously.
- **Raw DALI commands (FC06) at `0x1000` (single send) or `0x1001` (sent
  twice, for SET / INITIALISE / RANDOMISE).** The Pico uses these for
  GOTO_LAST_ACTIVE, UP, DOWN, group, broadcast, and commissioning frames.
  The asynchronous result is polled at FC04 `0x0108` (terminal-state read
  clears it).
- **Presence + dirty bitmaps (FC04) at `0x0100–0x0107`** — one 8-register
  read fetches both. The dirty range is read-clears.
- **Removed in the refactor — do not write.** The action holding registers
  `0x1002–0x100A` (DAPC, GOTO_LAST_ACTIVE, OFF/UP/DOWN/STEP/RECALL) and the
  DTR-setter registers `0x100B–0x100D` (SET_DTR0/1/2) all return
  `ILLEGAL_ADDRESS`. DAPC goes through the brightness register; everything
  else goes through the raw command channel.

The canonical specification for the gateway is
[`../TinyModbus/README.md`](../TinyModbus/README.md) — sections "FC04 —
Input Register Map", "FC03/FC06/FC10 — Holding Register Map", and
"Behavioural Changes".

### State event flow to HA

```
Hardware change
  → TinyModbus sets dirty bit
  → dirty_poll_tick() reads dirty bitmask
  → reads block-0 register for that ballast
  → lighting_emit_dali_state() pushes levt_t to event ring buffer
  → serial_poll() drains ring buffer
  → printf() emits JSON to USB CDC
  → mqtt_bridge.py reads serial line
  → publishes to lighting/dali/<slave>/<addr>/state (retained)
  → HA entity updates
```

---

## Serial Protocol

The CDC port carries three multiplexed line types: JSON events
(machine-readable, leading `{`), logs (human-readable, leading `# `), and text
command output (human-readable, anything else). The bridge only consumes lines
starting with `{` — see the [Manual Testing via CDC](#manual-testing-via-cdc)
section for everything else.

All communication between the Pico and the MQTT bridge uses line-delimited
JSON over USB CDC at 115200 baud. One message per line, `\n` terminated.

### Message envelope

```json
{"t": "<type>", "ts": <uptime_ms>, "d": {...}, "v": {...}}
```

- `t` — message type
- `ts` — Pico uptime in milliseconds (not wall clock)
- `d` — destination/identity fields (which device/slave/addr)
- `v` — value fields (state, brightness, etc.)

Commands from HA also carry `"id": N` which is echoed in the `ack` response.

### Pico → Bridge (outbound)

#### `hello`
Sent **once** at firmware boot. Bridge uses this to publish discovery payloads
and subscribe. If the bridge starts (or restarts) after the firmware boots it
will miss this message — it can re-request the full snapshot at any time via
[`resync`](#resync), which re-emits `hello` followed by every cached
state event and mapping.
```json
{"t":"hello","id":0,"ts":0,"d":{
  "fw_version":"2.0.0",
  "relay_slave":1,
  "dali_slaves":[2,3],
  "long_press_ms":500,
  "dim_repeat_ms":200,
  "reversal_window_ms":400,
  "baud":19200
}}
```

#### `state_relay`
Emitted when a relay coil changes (from a command or boot readback).
```json
{"t":"state_relay","ts":12345,"d":{"ch":3},"v":{"on":true}}
```

#### `state_dali`
Emitted when a ballast's block-0 changes (dirty-bit poll).
```json
{"t":"state_dali","ts":12345,
 "d":{"slave":2,"addr":5},
 "v":{"on":true,"bri":180,"fading":false,"lamp_fail":false}}
```
`bri` is the raw DALI arc level (0–254). `fading` is true while the ballast
reports fade-running in its status byte. Both fields update on every dirty-bit
read, giving real-time fade tracking to HA.

#### `state_group`
Emitted when a group's derived state changes. Computed from the cached group
membership bitmap (read from TinyModbus FC03 `0x0110–0x014F` at boot and on
every background presence check) and the per-member arc levels.
```json
{"t":"state_group","ts":12345,
 "d":{"slave":2,"group":5},
 "v":{"on":true,"bri":180}}
```
- `on` — true if any present member of the group has `arc_level > 0`.
- `bri` — max `arc_level` across present members (0 if none on).

Triggered after every per-member `state_dali` whose ballast belongs to the
group, by recomputing the aggregate. The event is only emitted when `on` or
`bri` actually changes (debounced against the last value sent for that group).
Per-member `state_dali` events are still emitted as before — `state_group` is
in addition, not instead.

#### `button`
Emitted at every classified button event.
```json
{"t":"button","ts":12345,
 "d":{"fixture":3,"button":1},
 "v":{"action":"press"}}
```
`action` is one of: `press`, `release`, `long_press_start`, `long_press_repeat`.

#### `scan_complete`
Emitted after `dirty_poll_full_scan()` — at boot and after commissioning.
The bridge uses `present` to set per-ballast MQTT availability and `groups`
to publish per-group entity state and member lists.
```json
{"t":"scan_complete","ts":12345,
 "d":{"slave":2},
 "v":{"present":[0,1,2,5,7,8],
      "groups":{"5":[0,5,7],"12":[1,2]}}}
```
`groups` is an object keyed by stringified group number (0–15); each value is
the array of present member short addresses. Empty groups are omitted, so the
object is `{}` on a freshly-commissioned bus with no group memberships set.

#### `presence`
Emitted during the 60 s background check when a ballast appears or disappears.
```json
{"t":"presence","ts":12345,
 "d":{"slave":2},
 "v":{"appeared":[9],"disappeared":[3]}}
```

#### `fault`
Emitted when a ballast reports lamp failure or power-cycle-event.
```json
{"t":"fault","ts":12345,
 "d":{"slave":2,"addr":5},
 "v":{"lamp_fail":true,"power_cycle":false}}
```

#### `provision_status`
Emitted on every state transition during a commissioning run.
```json
{"t":"provision_status","ts":12345,
 "d":{"slave":2},
 "v":{"state":"assigning","msg":"Found device at 0x3A21F0 → address 0","devices_found":1}}
```
`state` is one of: `idle`, `clearing`, `scanning`, `terminating`, `initialising`,
`randomising`, `searching`, `assigning`, `verifying`, `withdrawing`, `complete`,
`error`.

#### `config_mapping`
Emitted after `set_mapping` or `del_mapping` to confirm the change.
Also emitted for each mapping in response to `get_config`.
```json
{"t":"config_mapping",
 "d":{"fixture":3,"button":1},
 "v":{"targets":[{"slave":2,"addr":5},{"slave":2,"addr":6}]}}
```

#### `ack`
Response to any command carrying an `id` field.
```json
{"t":"ack","id":42,"d":{"ok":true}}
{"t":"ack","id":43,"d":{"ok":false,"err":"invalid_slave"}}
```

When `ok` is `false`, `err` is one of the following short tokens:

| Reason | Meaning |
|--------|---------|
| `invalid_slave` | Slave ID not 0 (relay), 2, or 3 (DALI) |
| `out_of_range` | Numeric argument outside its valid range |
| `invalid_group` | Group number not 0–15 |
| `invalid_baud` | Baud value not in the supported set |
| `invalid_target` | Provision target address not 0–63 |
| `invalid_mode` | Provision mode string not recognised |
| `start_failed` | Provisioning could not be started (lower-layer failure) |
| `busy` | Operation rejected because another is in flight |
| `save_failed` | Persistent write to flash failed |
| `no_txn` | `replace_mapping_add` or `replace_mappings_commit` arrived without a prior `replace_mappings_begin` |
| `txn_invalid` | `replace_mappings_commit` refused because an earlier `replace_mapping_add` failed validation |

The `set_baud` ack additionally carries `"note":"reboot required for baud change"`
in its `d` object on success.

### Bridge → Pico (inbound)

#### `cmd_relay`
Set a relay coil on or off.
```json
{"t":"cmd_relay","id":1,"d":{"ch":3,"on":true}}
```

#### `cmd_dali`
Control a DALI ballast. Include `bri` (0–254) for dimming, or `on` for toggle.
```json
{"t":"cmd_dali","id":2,"d":{"slave":2,"addr":5,"on":true}}
{"t":"cmd_dali","id":3,"d":{"slave":2,"addr":5,"bri":128}}
```

#### `cmd_dali_group`
Control a DALI group. Same shape as `cmd_dali` but addresses a group (0–15)
instead of a short address. The Pico sends a single DALI group frame via
raw command register `0x1000` — `addr_byte` is derived from `group`, the
caller does not encode it.
```json
{"t":"cmd_dali_group","id":4,"d":{"slave":2,"group":5},"v":{"on":true}}
{"t":"cmd_dali_group","id":5,"d":{"slave":2,"group":5},"v":{"bri":128}}
```

Wire format written to gateway register `0x1000` (high byte | low byte):

| Action | High byte | Low byte |
|--------|-----------|----------|
| `bri` (DAPC, level 0–254) | `0x80 \| (group << 1)` | bri value |
| `on: true` (GOTO_LAST_ACTIVE) | `0x81 \| (group << 1)` | `0x0A` |
| `on: false` (OFF) | `0x81 \| (group << 1)` | `0x00` |

Per-member `state_dali` events follow once the affected ballasts mark their
dirty bits; the aggregate `state_group` event is emitted after the last of
those (see `state_group` above).

#### `cmd_dali_broadcast`
Send a command to a raw DALI address byte (for groups or broadcast).
`addr_byte` 0xFE = broadcast, 0x81 = group 0, etc.
```json
{"t":"cmd_dali_broadcast","id":4,"d":{"slave":2,"addr_byte":254,"on":false}}
```

#### `set_mapping`
Map a button to one or more lights. `targets` is a list of `{slave, addr}` pairs.
`slave` 0 = relay board, 2 or 3 = DALI gateway.
```json
{"t":"set_mapping","id":5,
 "d":{"fixture":3,"button":1},
 "v":{"targets":[{"slave":2,"addr":5},{"slave":0,"addr":12}]}}
```

#### `del_mapping`
Remove a button mapping.
```json
{"t":"del_mapping","id":6,"d":{"fixture":3,"button":1}}
```

#### `replace_mappings_begin` / `replace_mapping_add` / `replace_mappings_commit` / `replace_mappings_abort`
Atomic wipe-and-replace transaction for the full button mapping set. Used by
`bridge/sync_bindings.py` to push a spreadsheet's worth of bindings in one
go. The transaction lives in RAM until commit; mid-transaction reboots leave
the live mappings untouched.

```json
{"t":"replace_mappings_begin","id":1}

{"t":"replace_mapping_add","id":2,
 "d":{"fixture":3,"button":1},
 "v":{"targets":[{"slave":2,"addr":5},{"slave":0,"addr":12}]}}

{"t":"replace_mappings_commit","id":99}

{"t":"replace_mappings_abort","id":100}
```

`replace_mapping_add` payload matches `set_mapping`. `commit` swaps the
scratch into `g_mappings`, writes `mappings.bin`, and emits a
`config_mapping` event for every (fixture, button) whose targets differ
(including ones cleared to empty), so a connected bridge stays in sync.

| Reason on `ack` | Meaning |
|---|---|
| `no_txn` | `add` or `commit` arrived without a prior `begin` |
| `out_of_range` | `add` rejected (bad fixture/button or too many targets); sticky — commit will fail |
| `txn_invalid` | `commit` refused because an earlier `add` errored |
| `save_failed` | swap succeeded but flash write failed |

Begin is idempotent (re-issuing it discards the current scratch and starts
over). Abort is also idempotent.

#### `get_config`
Request a dump of all current button mappings (returns a series of
`config_mapping` messages, one per mapped button).
```json
{"t":"get_config","id":7}
```

#### `set_config`
Update timing parameters. All fields optional — omit to leave unchanged.
```json
{"t":"set_config","id":8,
 "long_press_ms":600,"dim_repeat_ms":150,"reversal_window_ms":500}
```

#### `set_baud`
Change the Modbus bus baud rate. Takes effect after reboot. The `ack` for
this command carries `"note":"reboot required for baud change"` in `d`.
```json
{"t":"set_baud","id":9,"baud":19200}
```

#### `provision_start`
Start a DALI commissioning run.
```json
{"t":"provision_start","id":10,"d":{"slave":2,"mode":"all"}}
{"t":"provision_start","id":10,"d":{"slave":2,"mode":"unaddressed"}}
{"t":"provision_start","id":10,"d":{"slave":2,"mode":"address","target":5}}
```

#### `provision_abort`
Abort a running commission.
```json
{"t":"provision_abort","id":11}
```

#### `resync`
Re-emit the full snapshot — `hello`, every cached relay/DALI/group state,
`scan_complete` per slave, and every mapping. Use on bridge startup or
reconnect to recover from any boot-timing race. Idempotent. No Modbus or
DALI activity.
```json
{"t":"resync","id":42}
```

---

## Manual Testing via CDC

The same USB CDC port the bridge uses also accepts human-typed text commands
and emits a stream of human-readable log lines. This makes it possible to drive
the system, inspect state, and watch internal events from any serial terminal,
with or without the bridge running.

### Connecting

The Pico exposes a single USB CDC ACM device — typically `/dev/ttyACM0` on
Linux (or use the stable `/dev/serial/by-id/usb-Raspberry_Pi_Pico_2-if00`
symlink). The line is 115200 baud, 8N1, line-oriented (`\n` terminated). Any
serial terminal works:

```bash
screen /dev/ttyACM0 115200
picocom -b 115200 /dev/ttyACM0
cu -l /dev/ttyACM0 -s 115200
```

The firmware never echoes typed characters — enable local echo in your
terminal if you want to see what you are typing.

### Mixed line types

The firmware emits four kinds of line on the CDC port. They are all
distinguishable by their first character:

| Line type | Leading | Consumer | Description |
|-----------|---------|----------|-------------|
| JSON event | `{` | Bridge | Machine-readable state, ack, button, etc. (see Serial Protocol) |
| Log | `# ` | Human | Narration of internal state changes — see format below |
| Command output | (other) | Human | Reply to a typed text command |
| Command echo | — | Local terminal | Firmware never echoes; enable local echo if needed |

The bridge only parses lines starting with `{`, so logs and text-command
output are safe to mix with the JSON event stream — the bridge ignores them.

### Filtering

Useful one-liners for picking one line type out of a live capture:

```bash
grep '^#'         </dev/ttyACM0   # just logs
grep '^{' | jq .  </dev/ttyACM0   # just JSON events, pretty-printed
grep -v '^[#{]'   </dev/ttyACM0   # just text-command output
```

### Log line format

```
# <ts_ms> <level> <module>: <message>
```

Example:

```
# 12345 I dirty: ballast 2.5 arc=128 status=0x80
```

- `ts_ms` — Pico uptime in milliseconds (matches the `ts` field in JSON events)
- `level` — single character: `I` info, `W` warn, `E` err, `D` debug
- `module` — short lowercase tag, see [Module tags](#module-tags)
- `message` — free-form, but uses key=value style where it carries data

#### Module tags

| Tag | Description |
|-----|-------------|
| `boot` | Startup, init sequence, watchdog reboot cause |
| `disc` | Boot-time bus discovery and baud recovery |
| `modbus` | RS-485 Modbus master — frames, timeouts, retries |
| `dirty` | TinyModbus dirty-bit poller — state cache updates |
| `light` | Lighting command dispatch — relay coil and DALI writes |
| `btn` | Button matrix — gestures, classified events |
| `prov` | DALI commissioning state machine |
| `serial` | USB CDC line I/O — JSON parse, command receipt |
| `cli` | Text command interface — parsed commands and replies |

#### Log levels

| Level | Meaning |
|-------|---------|
| `I` | Normal operational events |
| `W` | Recoverable problems (timeouts, retries, ignored input) |
| `E` | Errors that left the system in a degraded state |
| `D` | Debug detail; off at the default verbosity |

Verbosity is set at runtime via the `log <level>` text command. Levels in
order of increasing detail: `off`, `err`, `warn`, `info` (default), `dbg`.
Each level emits its own output plus everything from less-verbose levels —
e.g. `log warn` emits `W` and `E` lines but suppresses `I` and `D`. Numeric
form `0`..`4` is also accepted (`log 4` is equivalent to `log dbg`).

The current level is shown by `show log`. Verbosity is **not persisted** —
boots default to `info`. Set `log dbg` after a reboot if you want to see
debug-level events.

### Text command grammar

Anything received on the CDC port that does not start with `{` is parsed as a
text command. One command per line. Tokens are whitespace-separated; keywords
are case-insensitive. Output is plain text and never starts with `{`, so the
bridge ignores it.

| Command | Description |
|---------|-------------|
| `help` | Show this list |
| `show config` | Current config (long_press_ms, baud, slave IDs, ...) |
| `show mappings` | All non-empty button → light mappings |
| `show mapping <fix> <btn>` | One mapping |
| `show relay [<ch>]` | One or all 32 relay states |
| `show dali [<slave> [<addr>]]` | All gateways / one slave / one ballast |
| `show group <slave> [<group>]` | Groups on slave (members + on/bri) |
| `show present <slave>` | List present ballast addresses on a DALI slave |
| `show provision` | Current provisioning state |
| `relay <ch> {on\|off\|toggle}` | Set relay coil |
| `dali <slave> <addr> {on\|off\|toggle\|<bri>}` | Individual DALI ballast |
| `dali <slave> g<group> {on\|off\|<bri>}` | DALI group (toggle not supported) |
| `dim <slave> <addr> {up\|down}` | One DALI step on individual |
| `dim <slave> g<group> {up\|down}` | One DALI step on group |
| `provision <slave> {all\|new\|<addr>}` | Start commissioning |
| `provision abort` | Abort a running commission |
| `scan <slave>` | Full read of one slave (blocking) |
| `map set <fix> <btn> <s>:<a>[,<s>:<a>...]` | Set mapping; `<a>` may be 0-63 or `G<n>` |
| `map del <fix> <btn>` | Delete mapping |
| `log <level>` | Set verbosity (`off`, `err`, `warn`, `info`, `dbg`; or numeric `0`..`4`) |
| `show log` | Current verbosity |
| `reboot` | Watchdog reboot |
| `resync` | Re-emit hello + all state + mappings |

### Output conventions

Successful commands that have no other output reply with `ok`. Errors reply
with `err: <reason>` where `<reason>` is a short token. The reasons in this
table apply to the **text command channel only**; the JSON `ack` event uses a
separate set of reasons (see the `ack` event section above).

| Reason | Meaning |
|--------|---------|
| `unknown_command` | First token did not match any command |
| `bad_arg` | Wrong number of arguments or unparseable token |
| `invalid_slave` | Slave ID is not 0 (relay), 2, or 3 (DALI) |
| `out_of_range` | Numeric argument outside its valid range |
| `provision_busy` | Commissioning is already running on that slave |
| `bad_arg group_toggle_unsupported` | `toggle` is not valid on a `dali ... g<n>` command (compound form: `err: bad_arg group_toggle_unsupported`) |

`show` commands emit terse `key=value` lines, one entity per line:

```
relay 3: on
dali 2.5: present=true on=true bri=128 status=0x80 fading=false min=1 max=254 poweron=128 sysfail=128 fade_rate=7 fade_time=0
group 2.G5: members=[3,7,12] on=true bri=180
present 2: 0 1 2 5 7 8
map F3.B1 -> 2:5,2:G7,0:12
provision: state=searching slave=2 devices_found=4 msg="Found device at 0x3A21F0..."
```

### Examples

```text
> relay 3 on
ok

> dali 2 5 127           # ballast 5 on slave 2 to ~50%
ok

> dali 2 g5 on           # group 5 on slave 2 — last active level
ok

> dim 2 5 down           # one DALI Down step on ballast 2.5
ok

> show dali 2 5
dali 2.5: present=true on=true bri=127 status=0x80 fading=false min=1 max=254 poweron=128 sysfail=128 fade_rate=7 fade_time=0

> show group 2
group 2.G5: members=[3,7,12] on=true bri=180
group 2.G7: members=[1,4] on=false bri=0

> map set 3 1 2:G5,2:G7   # bind fixture 3, button 1 to two groups
ok

> provision 2 all         # full commissioning of slave 2
ok

> show provision
provision: state=searching slave=2 devices_found=4 msg="Found device at 0x3A21F0..."

> log dbg                  # raise verbosity to see DBG-level events
log: dbg

> show log
log: dbg
```

Watching live while pressing a button (default `info` verbosity):

```
# 12345 I btn: F3.B1 press
# 12346 I light: dali 2.5 on
# 12348 I dirty: ballast 2.5 arc=128 status=0x80
```

After `log dbg`, the same press also surfaces debug-level activity:

```
# 12345 D btn: F3.B1 repeat
# 12348 D light: emit dali 2.5 arc=128 status=0x80
```

The same press produces JSON `button` and `state_dali` events on the same
line stream — see [Serial Protocol](#serial-protocol).

---

## MQTT Bridge

### Running

```bash
python3 mqtt_bridge.py \
    --serial /dev/serial/by-id/usb-Raspberry_Pi_Pico_2-if00 \
    --broker localhost \
    [--port 1883] [--user U] [--password P] \
    [--prefix lighting] [--debug]
```

Use the stable `/dev/serial/by-id/...` symlink rather than `/dev/ttyACM0`
— the numbered device can change on reboot.

### Docker deployment

```yaml
services:
  mqtt-bridge:
    build: ./bridge
    devices:
      - /dev/serial/by-id/usb-Raspberry_Pi_Pico_2-if00:/dev/ttyACM0
    command: ["python3", "mqtt_bridge.py", "--serial", "/dev/ttyACM0", "--broker", "mosquitto"]
    restart: unless-stopped
    depends_on: [mosquitto]
```

### Topic structure

All topics live under the prefix (default `lighting`).

| Topic | Direction | Retained | Description |
|-------|-----------|----------|-------------|
| `lighting/availability` | Bridge → HA | Yes | `online` / `offline` (system) |
| `lighting/relay/<ch>/state` | Bridge → HA | Yes | `{"state":"ON"}` |
| `lighting/relay/<ch>/set` | HA → Bridge | No | `{"state":"ON"}` |
| `lighting/dali/<slave>/<addr>/state` | Bridge → HA | Yes | `{"on":true,"bri":180,...}` |
| `lighting/dali/<slave>/<addr>/set` | HA → Bridge | No | `{"brightness":128}` or `{"state":"ON"}` |
| `lighting/dali/<slave>/<addr>/availability` | Bridge → HA | Yes | `online` / `offline` (per ballast) |
| `lighting/dali/<slave>/group/<g>/state` | Bridge → HA | Yes | `{"on":..,"bri":..}` (one per non-empty group) |
| `lighting/dali/<slave>/group/<g>/set` | HA → Bridge | No | `{"state":"ON"}` or `{"brightness":N}` |
| `lighting/dali/<slave>/group/<g>/availability` | Bridge → HA | Yes | `online` if group has ≥1 present member, else `offline` |
| `lighting/dali/<slave>/group/<g>/members` | Bridge → HA | Yes | JSON array of present member addresses |
| `lighting/button/<fixture>/<button>` | Bridge → HA | No | `{"event_type":"press"}` |
| `lighting/mapping/<fixture>/<button>` | Bridge → HA | Yes | Current mapping state |
| `lighting/mapping/<fixture>/<button>/set` | HA → Bridge | No | JSON targets array |
| `lighting/mapping/<fixture>/<button>/delete` | HA → Bridge | No | Any payload |
| `lighting/config/set` | HA → Bridge | No | `{"long_press_ms":500,...}` |
| `lighting/provision/<slave>/status` | Bridge → HA | Yes | `{"state":"...","msg":"...","devices_found":N}` |
| `lighting/provision/<slave>/start/all` | HA → Bridge | No | Any payload |
| `lighting/provision/<slave>/start/unaddressed` | HA → Bridge | No | Any payload |
| `lighting/provision/<slave>/start/target` | HA → Bridge | No | Integer 0–63 |
| `lighting/provision/<slave>/start/address` | HA → Bridge | No | Any payload |
| `lighting/provision/<slave>/abort` | HA → Bridge | No | Any payload |

### Availability model

**System availability** (`lighting/availability`): goes `offline` when the
Pico disconnects from USB or the bridge process stops. Set as MQTT Last Will.
All entities are unavailable when this is offline.

**Per-ballast availability** (`lighting/dali/<slave>/<addr>/availability`):
set to `online` or `offline` based on the DALI present bitmask. Updated after
every `scan_complete` (authoritative, full 64-bit mask) and `presence` (delta).
All 64 addresses per slave are published at `offline` immediately after `hello`;
`scan_complete` brings responding ones online.

DALI light discovery payloads use `"availability_mode": "all"` with both the
system topic and the per-ballast topic. A ballast is only shown as available
in HA when both are `online`.

**Per-group availability** (`lighting/dali/<slave>/group/<g>/availability`):
derived from the group's member list (see `scan_complete` extension above).
The bridge publishes `online` whenever the group has at least one present
member (after `scan_complete` or any `presence` delta that affects membership)
and `offline` when the last present member of the group disappears. The
companion `.../members` topic publishes the present member-address array on
the same trigger so HA can show which ballasts a group covers.

Relay channels and switch plates use only the system availability topic.

### Discovery

The bridge publishes HA MQTT auto-discovery payloads automatically on `hello`.
No manual HA configuration is needed. Payloads include:

- **Switch** entity for each relay channel (user can override to light/fan in HA UI)
- **Light** entity for each DALI ballast (dimmable, brightness 0–254)
- **Light** entity for each non-empty DALI group, published on `scan_complete`
  with names like `DALI 2.G05` (`DALI <slave>.G<NN>`) — numeric, user can
  rename in HA. See the `scan_complete` extension above for how the bridge
  learns which groups exist.
- **Event** entity for each button (7 per switch plate, 24 plates = 168 entities)
- **Sensor** entities for provision state, status message, devices found
- **Button** entities for Commission All, Commission New Devices, Commission Address, Abort
- **Number** entity for target DALI address (used with Commission Address)

All device names use numbers only (e.g. "Relay Channel 3", "DALI 2.05",
"Switch Plate 07"). Friendly names and room assignments are done in HA's
device registry UI and persist across restarts.

---

## DALI Commissioning

Commissioning assigns DALI short addresses (0–63) to ballasts. It is required
before new ballasts can be individually controlled.

### From Home Assistant

Each DALI gateway has a device card in HA with four buttons:

- **Commission All** — clears all existing addresses then assigns fresh ones
  from 0. Use when replacing or rewiring all ballasts on a bus.
- **Commission New Devices** — only addresses ballasts with no current short
  address. Safe to run on a live system; existing ballasts are untouched.
- **Commission Address** — re-initialises one specific address. Set the target
  address in the "Target Address" number entity first.
- **Abort** — sends DALI Terminate immediately.

Progress is shown in the Commission Status sensor, updating in near-real-time
as the binary search proceeds (typically every 100–400 ms per step).

### How it works

DALI commissioning uses a 24-bit binary search:

1. Devices are placed into initialisation mode (Initialise command).
2. Each device generates a random 24-bit number (Randomise command).
3. The search sets a Search Address to the midpoint of the current range and
   sends a Compare query. Any device with random address ≤ Search Address
   pulls the bus low (YES response). A bus collision also counts as YES.
4. If YES and the range has converged (low == high), the device at that random
   address is found. Programme Short Address is sent, Verify confirms it was
   accepted, then Withdraw removes it from the search pool.
5. The search resumes from random_address+1. Repeat until no more devices.

After commissioning, the Pico triggers a TinyModbus firmware rescan and waits
300 ms for it to settle, then runs `dirty_poll_full_scan()`. This immediately
updates the state cache and emits `scan_complete` so HA availability updates
without waiting for the 60 s background check.

### Commissioning timing

| Phase | Time |
|-------|------|
| Clearing all addresses (PROV_ALL) | ~1.5 s (broadcast + gap) |
| Scanning used addresses (PROV_UNADDRESSED) | ~1 s for 64 queries |
| Per device: binary search | ~2.4 s (24 steps × 4 frames × ~25 ms) |
| Per device: programme + verify + withdraw | ~100 ms |
| Post-commission rescan wait | 300 ms + ~10 s for full scan |

A full bus with 64 ballasts takes roughly 2.5–3 minutes, dominated by the
per-device binary search (~2.5 s × 64 ballasts ≈ 160 s) plus the ~10 s
post-commission rescan.

### Group membership management (future)

The current firmware **reads** group memberships at boot and on every
background presence check, but does not **change** them. Memberships are
typically configured once at commissioning via a separate DALI tool or
vendor app, and the gateway just reflects the result.

Adding HA-driven membership management would need:

- A new inbound command, e.g. `cmd_dali_group_membership` with
  `{"slave":S,"group":G,"members":[...]}`.
- The Pico computes the diff against its cached current bitmap for that
  group and writes the new 64-bit bitmap to TinyModbus FC06 register
  `0x0110 + 4*G` (4 consecutive registers covering bits 0–63).
- TinyModbus internally diffs against its own cached bitmap and dispatches
  the per-bit `ADD_TO_GROUP` / `REMOVE_FROM_GROUP` DALI commands
  automatically — see the FC03/FC06/FC10 holding-register entry for
  `0x0110–0x014F` in `../TinyModbus/README.md` for the gateway-side
  bitmap-write semantics.
- The Pico re-reads the bitmap afterward to confirm and emit a fresh
  `scan_complete` so HA group membership/availability updates.

Worst-case dispatch time for a fully-changed group is ~800 ms (16 bits ×
2 commands × 25 ms per spec), then the Pico re-reads to confirm.

Why deferred: most installations set group membership once at commissioning
and never touch it again. Exposing this surface to HA without a clear UI
need (e.g. drag-and-drop reassignment) creates more confusion than utility.
Revisit when there's a concrete use case.

---

## Persistent Storage

The Pico uses pico-vfs (littlefs) for a 256 KB filesystem in flash.

| File | Contents | Format |
|------|----------|--------|
| `/config.json` | Timing parameters and slave IDs | JSON text |
| `/mappings.bin` | Button-to-light mappings | Binary (`mapping_record_t` structs) |
| `/modbus_config.txt` | RS-485 DE mode (`tx` or `always`) | Text |
| `/modbus_baud.txt` | Modbus baud rate | Text integer |

All files can be read and hand-edited via a mass-storage tool or by mounting
the filesystem in a Pico USB bootloader session.

---

## Home Assistant Device Hierarchy

```
Relay Board (Waveshare Modbus RTU Relay 32CH)
└── via_device ← Relay Channel 0, Relay Channel 1, ... (switch entities)
                  User overrides type to light/fan in HA entity registry

DALI Gateway 2 (TinyModbus ATtiny1604)
├── Commission State   (diagnostic sensor)
├── Commission Status  (diagnostic sensor)
├── Devices Found      (diagnostic sensor)
├── Commission All     (config button)
├── Commission New Devices (config button)
├── Commission Address (config button)
├── Target Address     (config number, 0–63)
├── Abort Commission   (config button)
├── via_device ← DALI 2.00, DALI 2.01, ... (dimmable light entities, one per ballast)
└── via_device ← DALI 2.G05    (group light entity, one per non-empty group)
                  DALI 2.G07
                  DALI 2.G12
                  ...

DALI Gateway 3 (same structure)

Switch Plate 01 (fixture 0)
├── Button 1 (event entity)
├── Button 2 (event entity)
├── ...
└── Button 7 (event entity)

Switch Plate 02 ... Switch Plate 24 (same structure)
```

Group entities (`DALI <slave>.G<g>`) are separate HA lights from the per-ballast
entities, not a replacement: the same physical ballast can be a member of
several groups, so changing one ballast's level updates the per-ballast light
plus every group light it belongs to. The group light's `on` and `bri` are
the aggregate over its present members (see the `state_group` event spec).

All device names are numbered. After HA discovery, rename devices and assign
them to rooms using HA's Settings → Devices & Services UI. These names and
area assignments persist in HA's own registry across bridge restarts and
firmware updates.

---

## Button Mapping via HA

Button mappings (which buttons control which lights) are stored on the Pico
in `/mappings.bin` and are configurable from HA without firmware changes.

### Reading current mappings

The bridge retains the current mapping on `lighting/mapping/<fixture>/<button>`
— subscribe to read it. Or send `get_config` via the serial protocol directly;
the Pico emits a `config_mapping` message for each mapped button.

### Setting a mapping

Publish a JSON array of targets to `lighting/mapping/<fixture>/<button>/set`:

```json
[{"slave":2,"addr":5},{"slave":2,"group":5},{"slave":0,"addr":12}]
```

Each target is one of two forms:

- `{"slave":2,"addr":5}` — DALI ballast 5 on slave 2 (addr 0–63)
- `{"slave":2,"group":5}` — DALI group 5 on slave 2 (group 0–15)

Slave rules:

- `slave: 0` = relay board (addr = coil number 0–31; groups not supported)
- `slave: 2` or `3` = DALI gateway (addr = short address 0–63, or group 0–15)

A group target sends a single DALI group frame at button-press time rather than
N per-ballast commands, so it is both faster on the wire and atomic on the bus.

On disk in `/mappings.bin` the two forms share one byte: bit 7 of the addr
byte flags "this is a group" (`0x80 | (group_num & 0x0F)`). Existing `mappings.bin`
files written by older firmware load unchanged — all stored bytes have bit 7
clear and decode as ballast targets.

Up to 8 targets per button. All targets are controlled together — one
button press toggles all of them in unison.

### Removing a mapping

Publish any payload to `lighting/mapping/<fixture>/<button>/delete`.

### Bulk sync from a spreadsheet

`bridge/sync_bindings.py` reads `Switch wiring.xlsx` and replaces the
firmware's entire binding set in one atomic transaction over USB CDC. The
`Switch Positions` tab provides the physical button remap (per switch
plate, including plates that share a fixture ID), the `Lights` tab provides
the per-light bindings. Dry-run by default; pass `--apply` to actually send.

```bash
python3 bridge/sync_bindings.py \
    --workbook "Switch wiring.xlsx" \
    --serial /dev/serial/by-id/usb-Raspberry_Pi_Pico_2-if00 \
    --apply
```

The MQTT bridge must be stopped while running this tool — it would consume
the same acks. Requires `openpyxl` and `pyserial`.

---

## Development & Debugging

### Serial monitor

Connect to the Pico's USB CDC port with any serial terminal at 115200 baud.
The port carries both JSON event/command lines and human-readable logs and
text-command output — see [Manual Testing via CDC](#manual-testing-via-cdc)
for the line-type breakdown, log format, and full text-command grammar.

JSON commands (the same shape the bridge sends):

```bash
# Simple monitor
cat /dev/ttyACM0

# Send a command
echo '{"t":"get_config","id":1}' > /dev/ttyACM0

# Toggle relay channel 3
echo '{"t":"cmd_relay","id":2,"d":{"ch":3,"on":true}}' > /dev/ttyACM0

# Dim DALI address 5 on slave 2 to 50%
echo '{"t":"cmd_dali","id":3,"d":{"slave":2,"addr":5,"bri":127}}' > /dev/ttyACM0
```

Equivalent text commands (typeable from any terminal — easier for ad-hoc
testing, no JSON quoting required):

```bash
echo 'relay 3 on'        > /dev/ttyACM0
echo 'dali 2 5 127'      > /dev/ttyACM0
echo 'show dali 2 5'     > /dev/ttyACM0
echo 'show mappings'     > /dev/ttyACM0
```

### Bridge debug mode

```bash
python3 mqtt_bridge.py --serial /dev/ttyACM0 --broker localhost --debug
```

Debug logging shows every serial line received and every MQTT message published.

### Resetting to defaults

Delete the config file from the filesystem and reboot:
```bash
# Via picotool or mass storage
rm /config.json
```

Or send `set_config` with the default values:
```bash
echo '{"t":"set_config","id":1,"long_press_ms":500,"dim_repeat_ms":200,"reversal_window_ms":400}' \
    > /dev/ttyACM0
```

### Watchdog

The firmware enables a 5-second watchdog. If the main loop stalls (e.g.
a blocking Modbus read times out), the Pico reboots automatically. Reboot
cause is printed on the next serial connection:
```
Rebooted by Watchdog or explicit reset!
```

---

## Known Limitations

- **Baud rate change requires reboot.** The `set_baud` command persists the
  new baud rate to flash but the UART is not re-initialised at runtime.
  Reboot after changing baud rate.

- **Binary mapping format.** `/mappings.bin` is stored as raw C structs.
  If the firmware is recompiled with different struct alignment (e.g. different
  compiler flags), existing mapping files may be misread. This is safe for a
  single-compiler embedded deployment but worth noting if porting.

- **Maximum 2 DALI gateways.** `MAX_DALI_SLAVES = 2` matches the hardware —
  two TinyModbus DALI gateways (slaves 2 and 3). The relay board (slave 1)
  is handled separately and does not count against this limit. If a third
  DALI gateway were ever added, incrementing `MAX_DALI_SLAVES` and extending
  the config file format would be the only changes required.

- **DALI group membership management.** Group-level control (button mappings
  to a group, per-group HA Light entities, `cmd_dali_group`, `state_group`)
  is fully supported — see the mapping format and `state_group` /
  `cmd_dali_group` sections above. What's not implemented is **changing**
  which ballasts belong to which group from HA; the firmware only reads the
  memberships configured by an external DALI commissioning tool. See
  "Group membership management (future)" under DALI Commissioning for the
  rationale and the implementation sketch.

- **Provisioning is single-slave.** Only one DALI slave can be provisioned at
  a time. The other slave and the relay board remain fully operational during
  commissioning.
