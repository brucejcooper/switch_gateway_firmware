# Switch Gateway Lighting System

A complete home lighting control system built around a Raspberry Pi Pico 2 as the
central controller. The Pico owns all physical hardware, controls lights in response
to button presses, and exposes the entire system to Home Assistant via MQTT.

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│  RS-485 bus @ 38400 baud                                        │
│                                                                  │
│  Pico 2 (Modbus master, single source of truth)                │
│  ├── Waveshare 32CH Relay Board (slave 1)   — 32 relay coils   │
│  ├── TinyModbus DALI Gateway (slave 2)      — up to 64 ballasts│
│  └── TinyModbus DALI Gateway (slave 3)      — up to 64 ballasts│
│                                                                  │
│  Pico 2 also scans                                              │
│  └── 24 switch fixtures × up to 7 buttons each (PIO matrix)    │
└─────────────────────┬───────────────────────────────────────────┘
                       │ USB CDC serial, 115200 baud
                       │ Line-delimited JSON
                       ▼
┌─────────────────────────────────────────────────────────────────┐
│  Linux host (Docker)                                            │
│  ├── mqtt_bridge.py  — serial ↔ MQTT translation               │
│  ├── Mosquitto       — MQTT broker                              │
│  └── Home Assistant  — dashboards, automations, scenes         │
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

### Pin assignments (Pico 2)

| Function | GPIO |
|----------|------|
| RS-485 UART TX | 4 |
| RS-485 UART RX | 5 |
| RS-485 DE (drive enable) | 1 |
| Button matrix SER | 6 |
| Button matrix CLK | 7 |
| Button matrix COL base | 8 |

### Baud rate setup (one-time)

All three Modbus devices must be configured to 38400 baud before first use.
They default to 9600 from the factory.

**TinyModbus gateway** (repeat for slave 2 and slave 3):
```
FC06 write to register 0x1013, value 0x0002  → set baud to 38400
FC06 write to register 0x1012, value 0x0002  → set slave address to 2
(or value 0x0003 for slave 3)
```

**Waveshare relay board** (broadcast FC06 at current baud):
```
FC06 to address 0x00, register 0x2000, value 0x0003  → set baud to 38400
FC06 to address 0x00, register 0x4000, value 0x0001  → set slave address to 1
```

---

## Firmware

### Building

Requires the Pico SDK and pico-vfs. Set `PICO_SDK_PATH` before configuring.

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```

Flash `light_switch.uf2` by holding BOOTSEL on the Pico while connecting USB.

### Source file overview

| File | Purpose |
|------|---------|
| `main.c` | Startup, init sequence, main loop |
| `modbus.c/h` | Modbus RTU master — ISR-driven, non-blocking |
| `button_actions.c/h` | PIO button matrix scanner |
| `button_fsm.c/h` | Per-button state machine (toggle/dim/reversal) |
| `button_fsm_emit.c` | Button event serial emission |
| `config.c/h` | Global config, persisted to `/config.json` |
| `mapping.c/h` | Button-to-light mappings, persisted to `/mappings.bin` |
| `state_cache.c/h` | Canonical lighting state (relay + DALI) |
| `lighting_actions.c/h` | Modbus command dispatch, state event queue |
| `dirty_poll.c/h` | TinyModbus dirty-bit polling, background scans |
| `serial_protocol.c/h` | JSON line protocol over USB CDC |
| `dali_provision.c/h` | DALI commissioning state machine |
| `serial_protocol_provision.c` | Provision message handling |

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
registers 0x0159–0x015C, one bit per ballast). A bit is set whenever a
ballast's block-0 register (status byte | arc level) changes, and cleared
when read.

The Pico's polling loop:

1. Every **100 ms** when a command is in flight: read the 4-register dirty
   bitmask (one RTU frame). For each dirty bit, read that ballast's single
   block-0 register and emit a `state_dali` event.
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
Sent at boot. Bridge uses this to publish discovery payloads and subscribe.
```json
{"t":"hello","ts":0,"d":{
  "fw_version":"2.0.0",
  "relay_slave":1,
  "dali_slaves":[2,3],
  "long_press_ms":500,
  "dim_repeat_ms":200,
  "reversal_window_ms":400,
  "baud":38400
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
The bridge uses `present` to set per-ballast MQTT availability.
```json
{"t":"scan_complete","ts":12345,
 "d":{"slave":2},
 "v":{"present":[0,1,2,5,7,8]}}
```

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
 "v":{"state":"searching","msg":"Found device at 0x3A21F0 → address 0","devices_found":1}}
```
`state` is one of: `idle`, `clearing`, `scanning`, `terminating`, `initialising`,
`randomising`, `searching`, `assigning`, `complete`, `error`.

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
Change the Modbus bus baud rate. Takes effect after reboot.
```json
{"t":"set_baud","id":9,"baud":38400}
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

Relay channels and switch plates use only the system availability topic.

### Discovery

The bridge publishes HA MQTT auto-discovery payloads automatically on `hello`.
No manual HA configuration is needed. Payloads include:

- **Switch** entity for each relay channel (user can override to light/fan in HA UI)
- **Light** entity for each DALI ballast (dimmable, brightness 0–254)
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
| Per device: binary search | ~600 ms (24 steps × 25 ms) |
| Per device: programme + verify + withdraw | ~100 ms |
| Post-commission rescan wait | 300 ms + ~10 s for full scan |

A full bus with 64 ballasts takes roughly 45–60 seconds.

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
└── via_device ← DALI 2.00, DALI 2.01, ... (dimmable light entities)

DALI Gateway 3 (same structure)

Switch Plate 01 (fixture 0)
├── Button 1 (event entity)
├── Button 2 (event entity)
├── ...
└── Button 7 (event entity)

Switch Plate 02 ... Switch Plate 24 (same structure)
```

All device names are numbered. After HA discovery, rename devices and assign
them to rooms using HA's Settings → Devices & Services UI. These names and
area assignments persist in HA's own registry across bridge restarts and
firmware updates.

---

## Button Mapping via HA

Button mappings (which buttons control which lights) are stored on the Pico
in `/mappings.bin` and are configurable from HA without firmware changes.

### Reading current mappings

Publish any payload to `lighting/provision/<slave>/start/all` — no, wait,
that starts commissioning. To read mappings, publish to:

```
lighting/mapping/<fixture>/<button>/set
```

Or send `get_config` via the serial protocol directly. The Pico emits a
`config_mapping` message for each mapped button.

### Setting a mapping

Publish a JSON array of targets to `lighting/mapping/<fixture>/<button>/set`:

```json
[{"slave":2,"addr":5},{"slave":2,"addr":6},{"slave":0,"addr":12}]
```

- `slave: 0` = relay board (addr = coil number 0–31)
- `slave: 2` or `3` = DALI gateway (addr = short address 0–63)

Up to 8 targets per button. All targets are controlled together — one
button press toggles all of them in unison.

### Removing a mapping

Publish any payload to `lighting/mapping/<fixture>/<button>/delete`.

---

## Development & Debugging

### Serial monitor

Connect to the Pico's USB CDC port with any serial terminal at 115200 baud.
Messages flow as readable JSON lines in both directions.

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

## Applying the Patch

The firmware changes are delivered as `0001-complete.patch`, a unified diff
against the base `switch_gateway_firmware` repository.

```bash
git clone https://git.8bitcloud.com/bruce/switch_gateway_firmware
cd switch_gateway_firmware
git checkout -b c-firmware
patch -p1 < 0001-complete.patch
git add -A
git commit -m "Replace Lua/REPL with C firmware"
```

Verify the patch applies cleanly before committing — it should apply with no
rejected hunks or fuzz against the unmodified base repository.

The Lua files (`src/lualoop.c`, `src/lualoop.h`, `src/main.lua`,
`src/json.lua`, `lib/lua-5.4.8/`) are removed from the build but not deleted
from the repository, so `git revert` works cleanly if needed.

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

- **DALI group control.** The firmware does not expose DALI group commands
  through the mapping system. Group-level control (single DALI broadcast to
  all members of a group) is available via `cmd_dali_broadcast` from HA but
  is not assignable to buttons.

- **Provisioning is single-slave.** Only one DALI slave can be provisioned at
  a time. The other slave and the relay board remain fully operational during
  commissioning.
