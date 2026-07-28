# Switch Gateway Lighting System

A complete home lighting control system built around a **WIZnet W5100S-EVB-Pico
(RP2040)** as the central controller. The board owns all physical hardware,
controls lights in response to button presses, and talks to Home Assistant via
MQTT **directly over its on-board Ethernet** — no external bridge host.

> **On-board MQTT (current architecture).** The MQTT bridge that once ran as an
> external `mqtt_bridge.py` process on a USB-tethered host now runs **on-board on
> Pico core 1**, over the W5100S hardwired TCP/IP chip. The Python bridge has
> been **deleted** — it lives only in git history as a behavioural reference for
> the C reimplementation in `src/net/`. As a consequence the USB-CDC port is now
> a **human text CLI only**: it carries no JSON command/event stream.

---

## System Architecture

```
RS-485 bus @ 19200 baud (canonical; firmware auto-recovers slaves stuck at 9600)

W5100S-EVB-Pico (RP2040) — Modbus master, single source of truth
 ├── core 0: Modbus master · button matrix (PIO) · button FSM · DALI world
 │           model · command queue · state cache · persistence · USB-CDC CLI
 ├── core 1: W5100S Ethernet · MQTT connection mgmt · Home Assistant
 │           discovery / availability / state ↔ command translation (JSON)
 │
 ├── Waveshare 32CH Relay Board (slave 1)   — 32 relay coils
 ├── TinyModbus DALI Gateway (slave 2)      — up to 64 ballasts
 ├── TinyModbus DALI Gateway (slave 3)      — up to 64 ballasts
 └── 24 switch fixtures × up to 7 buttons   — PIO matrix scan

   │ Ethernet (W5100S, on-board)              │ USB CDC (human text CLI only)
   ▼                                          ▼
 Mosquitto + Home Assistant            screen / picocom / miniterm
```

The two cores exchange **typed C structs** over two single-producer/single-
consumer queues (`intercore.c`) — **not JSON**. Core 0 forwards state events to
core 1 (`ic_event_t`); core 1 forwards translated commands back to core 0
(`ic_cmd_t`). JSON exists only at core 1's MQTT edge (`src/net/ha_bridge.c`).

### Key design principles

**The board is the master.** It owns the RS-485 bus and all lighting state. HA is
a client — if HA, the broker, or the network goes away, buttons and lights keep
working normally. A network outage never reboots the device: core 1 keeps looping
through its reconnect state machine while core 0 runs the lights. Core 0 only
feeds the hardware watchdog while core 1's heartbeat is fresh, so a genuine
core-1 wedge (not a network outage) is what triggers a reboot.

**State is never assumed.** Core 0 only updates its state cache from hardware
readback — DALI queries sent to the ballasts themselves — never from the
commands it sent. Since task 05 the gateway caches nothing, so the ballasts are
the only authority.
This correctly handles DALI min/max level clamping and ballast-side fade curves.

**Discovery is republished on every connect.** On each MQTT (re)connect core 1
re-publishes the full HA auto-discovery set and then requests a `resync`, so HA
repopulates from scratch. Broker and network settings live in `/config.json`
(set with the `mqtt` / `net` CLI commands, or HA's `lighting/config/...` topics);
friendly names and rooms are done in HA's device registry UI.

---

## Hardware

| Device | Role | Modbus Slave ID |
|--------|------|-----------------|
| Waveshare Modbus RTU Relay 32CH | Relay control — lights, fans, switches | 1 |
| TinyModbus ATtiny1604 DALI Gateway #1 | DALI bus A — up to 64 ballasts | 2 |
| TinyModbus ATtiny1604 DALI Gateway #2 | DALI bus B — up to 64 ballasts | 3 |
| WIZnet W5100S-EVB-Pico (RP2040) | Main controller, button scanner, Modbus master, on-board Ethernet/MQTT | — |

Function-code mappings and register addresses for the Waveshare board:
<https://www.waveshare.com/wiki/Modbus_RTU_Relay_32CH>.

### Pin assignments (W5100S-EVB-Pico, RP2040)

| Function | GPIO |
|----------|------|
| RS-485 UART TX | 4 |
| RS-485 UART RX | 5 |
| RS-485 DE (drive enable) | 1 |
| Button matrix SER | 6 |
| Button matrix CLK | 7 |
| Button matrix COL base (8 columns → GP8–GP15) | 8 |
| W5100S SPI0 SCK | 18 |
| W5100S SPI0 MOSI | 19 |
| W5100S SPI0 MISO | 16 |
| W5100S SPI0 CSn | 17 |
| W5100S RSTn | 20 |
| W5100S INTn | 21 |

SPI0 (GP16–21) is owned exclusively by core 1; the RS-485 UART and USB CDC are
owned by core 0. The button matrix tops out at GP15, leaving the SPI block clear.

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
address than configured. Manual recovery is easy from the CLI: `ping <slave>`
to probe, `discover` to re-run auto-recovery across all configured slaves, or
`set_slave_baud <slave> {9600|19200|38400}` to push a baud at a known rate.
The slave-address writes for factory-fresh devices (TinyModbus FC06 `0x1012`
value `0x0002` or `0x0003`; Waveshare FC06 `0x4000` value `0x0001`) are
one-time and must be done manually before the first boot.

---

## Firmware

### Building

Requires the Pico SDK, pico-vfs, ioLibrary_Driver and coreMQTT (all git
submodules — `git submodule update --init --recursive`). The board is selected
in `CMakeLists.txt` (`PICO_BOARD wiznet_w5100s_evb_pico`), which targets the
RP2040 and pins the W5100S SPI defaults — a Pico 2 / RP2350 UF2 will not boot
this board.

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```

`FW_VERSION` is built from git on every build: `<base>+<git describe --tags
--always --dirty>` (e.g. `2.0.0+791ca0b-dirty`), via `cmake/gen_version.cmake`
→ `build/generated/version.h`. The base (`2.0.0`) lives in `CMakeLists.txt`.

Flash `light_switch.uf2` with picotool. For example:

```bash
build % make && picotool load -f -x light_switch.uf2 && sleep 2 && pyserial-miniterm /dev/tty.usbmodem111201
```

### Boot Sequence

Each boot, before the main loop starts, the firmware runs:

1. **stdio + alarm pool init**, then a ~2.5 s USB-enumeration settle.
2. **Core 1 launched early** (`core1_launch()`), *before* `fs_init()`. Core 1
   registers as the flash-lockout victim (`flash_safe_execute_core_init`), which
   the first-boot littlefs format requires, then **parks** until core 0 hands it
   a config snapshot — it does not bring up the network yet.
3. **Filesystem init** — pico-vfs / littlefs mount/format.
4. **Config + mapping load** — `/config.json` and `/mappings.json` (defaults if
   absent; legacy files auto-migrated, see [Persistent Storage](#persistent-storage)).
5. **State cache init** — zeros the in-memory entity state.
6. **Modbus init** — UART at the persisted baud rate (19200 default).
7. **Bus discovery** — for each configured slave, ping at 19200, fall back to
   9600, auto-reconfigure if needed (see "Baud rate setup" above).
8. **DALI full scan** — for each TinyModbus gateway, blocking read of every
   ballast bank + group masks, populate state cache, queue per-ballast state
   events and a scan-complete.
9. **Relay full sync** — read all 32 coils on the Waveshare board into the state
   cache, queue relay state events.
10. **Button + serial init** — PIO scanner, USB-CDC text CLI.
11. **Config snapshot handed to core 1** (`intercore_set_snapshot` +
    `core1_config_ready()`): slave lists + MQTT broker/creds + network settings.
    Core 1 now brings up the W5100S, DHCP/static IP, and the MQTT connection
    manager, and on connect publishes the full HA discovery set.
12. **Watchdog enabled** — 5 s timeout, plus a core-1 launch grace window. Only
    after this point are stalls fatal. Discovery and scans run unwatchdog'd
    because they can take seconds in failure paths.

### Source file overview

| File | Purpose |
|------|---------|
| `main.c` | Startup, init sequence, core-1 launch, main loop, watchdog feed |
| `modbus.c/h` | Modbus RTU master — ISR-driven, non-blocking |
| `bus_discovery.c/h` | Boot-time slave ping + auto baud recovery |
| `button_actions.c/h` | PIO button matrix scanner |
| `button_fsm.c/h` | Per-button state machine (toggle/dim/reversal) |
| `button_fsm_emit.c` | Pushes typed button events to core 1 |
| `config.c/h` | Global config, persisted to `/config.json` (timing, slaves, modbus baud, RS485 DE mode, MQTT, net) |
| `mapping.c/h` | Button-to-light mappings, persisted to `/mappings.json` |
| `state_cache.c/h` | In-memory cache of relay + DALI ballast + group state |
| `lighting_actions.c/h` | Command dispatch, any-on probe + config-query state machines, state event queue |
| `dali_queue.c/h` | DALI adapter: per-slave priority scheduler + producer API over the generic client |
| `mbq_client.c/h` | Generic DALI-agnostic command-queue client (8-slot lifecycle protocol over FC23) |
| `dali_state.c/h` | the DALI world model: presence, level/status, groups, fade tracking |
| `dali_commission.c/h` | controller-side IEC 62386-102 commissioning |
| `dirty_poll.c/h` | relay-board polling + gateway sysinfo/reachability |
| `serial_protocol.c/h` | USB-CDC text CLI + the core-0/core-1 inter-core seam |
| `serial_protocol_provision.c` | Provision status → core-1 event forwarding |
| `dali_provision.c/h` | DALI commissioning state machine |
| `intercore.c/h` | The two typed SPSC queues + the config snapshot |
| `core1.c/h` | Core-1 entry point, heartbeat, flash-lockout victim |
| `net/netif.c/h` | W5100S bring-up / DHCP / link recovery state machine |
| `net/wizchip_glue.c/h` | SPI/CS/reset glue for the WIZnet ioLibrary |
| `net/mqtt_app.c/h` | coreMQTT connection manager (connect, LWT, sub/pub, TX flow) |
| `net/ha_bridge.c/h` | HA auto-discovery, availability, event↔MQTT translation |
| `log.c/h` | Logging facility (LOG_INFO/WARN/ERR/DBG macros) emitting to USB CDC |

### Main loop (core 0)

```c
while (true) {
    modbus_poll();               // advance Modbus state machine (ISR-driven, non-blocking)
    wd_feed();                   // feed watchdog iff core 1's heartbeat is fresh
    lighting_any_on_probe_tick();// submit any-on probe queries into the gateway queue
    lighting_cfg_query_tick();   // submit config queries into the gateway queue
    dq_tick();                   // drive the gateway command queue (FC23 submit/collect)
    button_fsm_tick();           // classify raw button events, dispatch commands
    dali_state_tick();           // interrogate ballasts: verify, discover, sweep
    dirty_poll_tick();           // relay board + gateway sysinfo
    dali_provision_tick();       // advance commissioning state machine if active
    serial_poll();               // read text CLI; drain core-1 cmds; forward state events to core 1
    wd_feed();
}
```

Core 0 is cooperative — no blocking calls in normal operation. Core 1 runs its own
loop (`netif_tick → mqtt_app_tick → ha_bridge_tick`) on the second core.

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

### Timing defaults (configurable via the `cfg` CLI command or `lighting/config/<field>/set`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `long_press_ms` | 500 ms | Hold threshold before dimming begins |
| `dim_repeat_ms` | 200 ms | Interval between DALI Up/Down commands |
| `reversal_window_ms` | 400 ms | Window after release to trigger direction reversal |
| `gw_diag_interval_ms` | 30000 ms | How often each DALI gateway's sysinfo (uptime/reboots/reset reason) is polled and published to HA; 0 disables |

### Button events sent to HA

Every button gesture is also published to HA as an event entity, so HA
automations can react to button presses independently of the built-in mapping.

Physical `press`/`release` frame every interaction; the gesture events
(`click`, `long_press`, `long_press_repeat`) overlay them. A **tap** emits
`press` then `click` + `release`; a **hold** emits `press`, `long_press`,
`long_press_repeat`…, `release`.

| Event type | When fired |
|------------|------------|
| `press` | Physical button down (every interaction) |
| `release` | Physical button up (every interaction) |
| `click` | A completed short tap (down→up before the long-press threshold) |
| `long_press` | Hold threshold crossed (also fires on direction reversal) |
| `long_press_repeat` | Each dim-rate tick while held (~5 Hz) |

---

## Lighting State Tracking

### Relay channels

The Pico sends a coil write and updates its cache immediately from the Modbus
write-callback. Relay state is instantaneous — no polling needed after a command.

The Waveshare board's state is also read in full at boot to synchronise the
cache with the hardware's actual state (handles Pico restarts while power was on).

### DALI ballasts — the controller's world model (`dali_state.c`)

**The controller learns the bus by asking the ballasts.** Task 05 removed every
gateway cache, so presence, arc level, status and group membership are all
established by DALI queries sent through the command queue — nothing is read
from a gateway register any more.

Three sources of work, in priority order, at most one query in flight per slave:

1. **Post-command verification** (`DQ_PRIO_LEVEL_READ`). Any command marks what
   it touched — one ballast, a group's cached members, or everything for a
   broadcast — and the toucher is re-read immediately so user-visible state
   converges without waiting for a sweep.
2. **Discovery** (`DQ_PRIO_BACKGROUND`). A full pass over all 64 addresses:
   `QUERY STATUS` → `QUERY ACTUAL LEVEL` → `QUERY GROUPS 0-7` → `QUERY GROUPS
   8-15`. Run at boot, by CLI `scan`, and after provisioning. It is **not**
   paced by the sweep interval — measured at **4.87 s** for 64 addresses,
   because an absent address costs a single query.
3. **Background sweep** (`DQ_PRIO_BACKGROUND`). One ballast per
   `dali_sweep_interval_ms` (default 10 s, 0 = off). This is a *backstop* for
   what nothing tells us about — a ballast that lost power, or gear changed
   outside this controller — not the mechanism. Every 6th step probes an
   **absent** address instead, so gear that was off at boot or added later is
   discovered without a manual scan.

**Fade tracking.** A DAPC lands immediately but the level *ramps*, so a single
read taken mid-fade would cache a value on the way to the destination. The
verifier re-reads while `QUERY STATUS` bit 4 (FADE RUNNING) is set, at 250 ms,
until it settles — bounded at 95 s (past DALI's longest fade) so a stuck fade
bit cannot pin the bus.

**Presence — absence needs proof.** Only a CLEAN no-answer to a directed query
counts against presence, and normally only after two consecutive. A collision,
bus fault or dropped command is evidence about the **bus**, not the ballast, so
it freezes the counters and keeps the current belief. The one exception: an
**explicit scan is authoritative** and drops presence on the first clean
no-answer — a scan is a deliberate re-establishment of ground truth (boot,
`scan`, post-provisioning) and is precisely when addresses are expected to have
moved. Without that, every address vacated by a re-provision lingered as a
phantom in HA, one per sweep step.

The controller never caches an arc level from the command it sent — it reads
back what actually happened. That is what makes DALI min/max clamping report
honestly (commanded 128 on a binary relay ballast → reports 254 with status
`0x0C`, limit error), rather than the controller claiming a level the gear never
reached.

#### TinyModbus register contract

The `task-1-flag-driven` refactor of the TinyModbus firmware moved the
per-ballast cache from input registers to holding registers and retired
several action / DTR-setter registers. The Pico's master code follows the
new contract:

- **Live arc-level bank (FC03 read, FC06/FC10 write) at `0x0000–0x003F`**
  (status | brightness). Writing the low byte (DAPC level 0–254) is how the
  Pico drives per-ballast level changes and OFF — the firmware caches
  predictively and dispatches the DAPC asynchronously.
- **Per-ballast config banks at `0x0040–0x00FF` are RETIRED (task 04).** The
  gateway no longer stores min/max/power-on/sys-fail/fade — FC03 reads and
  writes there return ILLEGAL DATA ADDRESS. The controller owns config: it
  **reads** each field with raw DALI queries (the config-query state machine in
  `lighting_actions.c`, routed through the command queue) and **writes** it with
  a DTR0+SET pair submitted as one queue command. Controller and gateway are a
  **matched firmware set** (queue v2, task 06) — deploy both together.
- **DALI command queue (queue v2)** — the primary path for group/broadcast/
  specialty sends and all raw queries. 8-slot command block (FC10/FC23 write) at
  `0x1000–0x100F`, per-slot status block at `0x1010–0x1017` (served under both
  FC03 and FC04), FC23 (`0x17`). See "DALI command queue: adapter + generic
  client" below and `../TinyModbus/README.md`. The CLI `dali_raw` / `read_mem`
  go through the queue via `dq_submit_sync`.
- **Everything else is GONE (task 05).** The arc/status bank, presence and
  dirty bitmaps, group masks, scan trigger, rescan interval and the commission
  register all return ILLEGAL DATA ADDRESS. Group membership is now read with
  `QUERY GROUPS` and written with `ADD/REMOVE FROM GROUP` frames; commissioning
  runs in `dali_commission.c`. The only surviving action registers are
  slave-addr `0x1F02`, baud `0x1F03` and system-reset `0x1F05`.
- **Gateway recovery:** `0x1F05` (write `0xA5A5`) software-resets a wedged
  gateway (CLI `gw_reboot`). RX error counters live at FC04 `0x0300`; uptime /
  boot-count / reset-reason at FC04 `0x0310` (read `get_input <slave> 0x0310 4`
  — a reboot shows as uptime dropping or boot-count incrementing).

The canonical specification for the gateway is
[`../TinyModbus/README.md`](../TinyModbus/README.md).

### Scheduler model (task 05)

The queue client is fed by a **pull scheduler**, not a shared backlog. Storage is
partitioned by priority class, and each class carries **its own gateway depth**:

| class | storage | gateway depth | what uses it |
|---|---|---|---|
| provisioning | 2 | **1** | the commissioning sequence (each step depends on the previous answer) |
| user | 8 | 4 | buttons, HA commands, group commands — sized to one button's full fan-out |
| config | 3 | 3 | DTR0+SET config writes |
| background | 2 | **2** | the world-model sweep, config-query rotation |

Depth matters because the **gateway executes strictly FIFO with no priority** —
anything already submitted delays a button by a whole DALI frame. Capping
background at 2 bounds that inversion; letting user work reach 4 keeps bursts
fast. Measured on the bench: background load costs a user command **+13 ms
median**, the same load admitted at user depth costs **+75 ms**.

While a slave is provisioning the scheduler ships **only** the provisioning
class, so nothing can interleave with the binary search — a stray DTR0 write
mid-bisect would corrupt it.

**Dim steps are counters, not commands.** A held button emits repeats faster than
DALI can issue them, so queueing each step makes the light keep dimming after
release. `dq_add_delta()` accumulates a signed per-ballast/per-group step count
which the scheduler drains one frame per submit opportunity, rotating across
targets for fairness. Real UP/DOWN frames still go out — the ballast's own fade
rate and min/max clamping govern the result, deliberately not a DAPC computed
from cache. The counter decrements on **submit** and saturates at ±4: losing a
step to the clamp or a gateway reboot is invisible in a ramp, whereas replaying
one would double-step.

Duplicate idempotent submits that have not shipped are folded, so a repeated
press or a re-published HA state costs nothing. A class-full refusal is
**backpressure**, not an error — a polling producer sees it routinely — so it is
a rate-limited counter (`refused=` in `dq`), not a warning per event.

### DALI command queue: adapter + generic client (`dali_queue.c` / `mbq_client.c`)

Every group/broadcast/specialty DALI send and every raw query goes through a
**two-layer** command queue (queue v2, task 06):

1. **`mbq_client.c` — the generic, DALI-agnostic client.** It mirrors the
   gateway's **8-slot self-describing lifecycle protocol**: it carries opaque
   32-bit commands into a slave's 8 slots and delivers back a terminal outcome
   (`complete` / `failed` / `lost`) + an 8-bit response per command. It knows
   nothing about DALI. The generic protocol — the slot command/status register
   layout, the lifecycle states, the all-or-nothing reject-active write, and the
   "idle means gone" reboot signal — is specified once in
   [`docs/modbus-command-queue.md`](docs/modbus-command-queue.md).
2. **`dali_queue.c` — the DALI adapter** on top. It owns the per-slave priority
   scheduler and the producer API, packs a DALI command into the client's 4
   opaque command bytes, and maps the client's outcome back to a `dq_result_t`.

This replaced the task-04 id-ledger client (a controller-assigned id sequence
with a result ring, credit tracking, and a boot_count-driven resync/rebuild/
rollback machine). Queue v2 has **no id ledger and no resync machinery** — the 8
slot statuses are the truth, so the whole class of reboot/glitch recovery-
corruption bugs the ledger fought is gone by construction.

**Scheduler (the adapter).** Each gateway has a per-slave **priority backlog** in
front of the gateway's 8 slots. Producers (`dq_submit`) drop commands in with a
**priority class**; `dq_tick()` ships the highest-priority pending command first
(FIFO within a class) into a free slot via the client. A command carries an
**idempotency** flag: idempotent commands (GOTO_LAST_ACTIVE, group on/off/level,
config re-store, all queries) may be resubmitted after a gateway reboot; UP/DOWN
step frames may not.

**Priority ladder** (highest first, an ordered table — one-line reorder), mirroring
the gateway's own work selector: **provisioning** (bus-exclusive: while a slave
is being commissioned the scheduler admits nothing else on it) → **user actions**
(brightness/on-off/step + group commands + the any-on probe) → group-membership
writes → **config writes** (DTR0+SET) → level/status reads → **background** (config
rotation). Classes with no queue client yet (group writes, level reads — they use
FC03 / dirty_poll today) are reserved placeholders for task 05. In-flight depth is
capped at **2–3 per slave** (`DQ_INFLIGHT_MAX`, well below the gateway's 8) so a
backlog of background work never fills all 8 slots ahead of a fresh user command —
the depth cap is what keeps the priority ladder meaningful above the client.

**Payload contract (the adapter).** A DALI command packs into the 4 opaque bytes
as `[addr, data, flags, dtr0]`, where `flags` is `DQ_TWICE` / `DQ_DTR0` /
`DQ_ANSWER` (bit-for-bit the gateway's `QF_*`). On completion the client's 8-bit
response is the DALI backward-frame answer (on `complete`) or a failure code
(`NAK` / `collision` / `bus fault`, on `failed`), which the adapter maps to
`DQ_RES_OK` / `DQ_RES_NAK` / `DQ_RES_ERR`. The **collision-vs-NAK** distinction is
preserved for the any-on probe (a garbled backward frame → `DQ_RES_ERR` → "≥1
on"; silence → `DQ_RES_NAK` → "none on"). A gateway reboot that discards a
non-idempotent command surfaces as `DQ_RES_DROPPED`.

**Gateway reboot recovery — status-driven, in the client.** A reboot clears every
slot. The client detects it from the slot statuses alone: a slot the client
*confirmed* live that reads back `idle` is the unambiguous "idle means gone"
signal → fail all outstanding, delivering `lost` and resubmitting the idempotent
ones. There is no boot_count feed, no id re-baselining, no fresh-rebuild-vs-
alive-reconcile gate — the corruption modes those existed to manage cannot arise
because nothing is inferred from an id sequence. (boot_count / the FC04 sysinfo
block remain, purely as an HA diagnostic in `dirty_poll.c`.)

**Scheduler (task 05 phase 1).** There is no shared backlog. Storage is
partitioned by priority class — user 8 entries (one button's full fan-out), cfg
3, bg 2 — and each class carries **its own gateway depth** (user 4, cfg 3, bg 2).
That matters because the *gateway* executes strictly FIFO with no priority, so a
background command already submitted delays a button by a whole DALI frame;
capping background at 2 bounds that inversion at ~2 frame times while leaving
user bursts the depth they need for throughput. Measured: background load costs
a user command +22 ms median, the same load as user class costs +75 ms.
Duplicate idempotent submits that have not shipped are folded.

**Dim steps are counters, not commands.** A held button emits repeats faster
than DALI can issue them, so queueing each step makes the light keep dimming
after release. `dq_add_delta()` accumulates a signed per-ballast/per-group step
count which the scheduler drains one frame per submit opportunity, rotating
across targets for fairness. Real UP/DOWN frames still go out — the ballast's
own fade rate and min/max clamping govern the result, deliberately not a DAPC
computed from cache. The counter decrements on **submit** and saturates at ±4:
losing a step to the clamp or to a gateway reboot is invisible in a ramp,
whereas replaying one would double-step.

**Observability.** The CLI **`dq [slave]`** command dumps the per-class pending
counts and depths, the pending dim deltas and cursor, the in-flight count, and
its counters (submitted / ok / nak / err / dropped / refused / coalesced / steps
/ clamped), then the underlying client's per-slot state
(`free`/`armed`/`sent`/`live`) and counters, including the poll-transaction count.
Bare = all DALI slaves. For throughput and correctness under load, see
[`busload` and `soak`](#measuring-the-command-path-busload-and-soak).

**Submit length (task 07).** A submit's FC23 write count is in **registers**, and
the gateway derives `n_slots = wr_count / 2` from it — so a one-command submit
declares `MBQ_CMD_REGS` (2), not `MBQ_CMD_BYTES` (4). Declaring bytes there does
not fail the request: it arms the *next* slot as well, using whatever memory
followed the command buffer as a second, real DALI frame. That shipped in task 06
and is fixed here; `MBQ_CMD_REGS` and a `_Static_assert` now sit beside
`MBQ_CMD_BYTES` so the units cannot drift apart again.

**Subsystems on the queue.** Every former single-slot producer rides it: the raw
command path (button on/off/step, group commands, config DTR0+SET), the **any-on
probe**, the **config-query** state machine, and the blocking **CLI helpers**
(`dali_raw` / `read_mem`, via `dq_submit_sync`). The probe and the config queries
are ordinary expects-answer commands, each occupying its own slot — no shared-
result-register interlock, no settle-gate. `read_mem`'s per-byte SET-DTR0 + READ
is one atomic `DQ_DTR0` queue entry. The click-grace guarantee holds: a probe
query resolves in a couple of FC23 round-trips (tens of ms), inside the button
FSM's 150 ms grace; a rare overflow lands on the pre-existing assume-off fallback,
bounded by the 600 ms probe deadline.

> **Matched firmware set (no cross-version interop).** Queue v2 is a protocol
> break: the controller (`switch_gateway_firmware`) and the TinyModbus gateway
> firmware are a **matched set** and must be deployed together. A v1 (task-04
> id-ledger) master cannot drive a v2 gateway and vice versa — the register
> semantics at `0x1000`/`0x1010` are incompatible. There is no mixed-version
> operation.

### State event flow to HA

```
Hardware change
  → dali_state_tick() interrogates the ballast (QUERY STATUS / ACTUAL LEVEL)
     — from a post-command verification mark, or the background sweep
  → lighting_emit_dali_state() pushes levt_t to the core-0 event ring
  → serial_poll() drains the ring, converts to a typed ic_event_t
  → intercore_push_event() — onto the core-0 → core-1 SPSC queue
  → core 1: ha_bridge_tick() pops it, builds the MQTT JSON
  → publishes to lighting/dali/<slave>/<addr>/state (+ /config) (retained)
  → HA entity updates
```

A command from HA travels the reverse path: core 1's `ha_on_message()` parses
the MQTT JSON into a typed `ic_cmd_t`, pushes it onto the core-1 → core-0 queue,
and core 0's `serial_poll()` dispatches it through the same domain entry points
the CLI uses.

---

## Protocol — the `{t, d, v}` model

There is **one logical message model** with three roles: **set**, **get**, and
**report**. `d` addresses the entity, `v` is a bag of field→value, and direction
is in `t` (`set_*`/`get_*` command; `state_*`/`config_*` report). This model is
realised in two places:

- **At the MQTT edge** (core 1 ↔ Home Assistant) as JSON on retained/command
  topics — see [MQTT / Home Assistant](#mqtt--home-assistant).
- **At the USB-CDC CLI** (core 0) as `key=value` text — see
  [Text CLI](#text-cli-usb-cdc). Bare = get; `key=val` = set.

Internally, core 0 ↔ core 1 exchange the *typed C structs* `ic_event_t` /
`ic_cmd_t` over SPSC queues (`intercore.h`), never JSON. The JSON shapes below
describe what core 1 publishes/consumes on the wire.

| Domain | `d` | settable `v` keys | report |
|---|---|---|---|
| dali   | `{slave,addr}` | `on`(true/false/toggle) `bri` `min` `max` `power_on` `sys_fail` `fade_rate` `fade_time` | `state_dali` (+ `config`) |
| relay  | `{ch}` | `on`(true/false/toggle) | `state_relay` |
| group  | `{slave,group}` | `on` `bri` · `members`[] (replace) / `add`[] / `remove`[] | `state_group` (incl. members) |
| cfg    | – | `long_press_ms` `dim_repeat_ms` `reversal_window_ms` `gw_diag_interval_ms` `baud`(reboot to apply) `relay_slaves`[] `dali_slaves`[] | `state_cfg` (also read-only `fw_version`, grid dims) |
| mapping| `{fixture,button}` | `targets` (array; empty deletes) | `config_mapping` |

**DALI groups are first-class.** Membership is editable like any other value —
the controller writes the desired 64-bit mask to the gateway's group-mask
registers (`0x0110+`) and the gateway emits the DALI ADD/REMOVE commands. A
group level/on/off command also flags its members for re-read so each member's
`state_dali` refreshes shortly after.

**`resync`** re-emits the full snapshot: `hello`, every cached relay/DALI/group
state, scan-complete per slave, and every binding. Core 1 issues one
automatically after each discovery burst; the CLI exposes it as `resync`.

**`hello`** is a one-line boot/ready announcement (a human log on USB; core 1
derives `fw_version` and the slave lists from its own config snapshot, not from a
hello event). The provision status carries `{state, msg, devices_found}` where
`state` is one of `idle, clearing, scanning, terminating, initialising,
randomising, searching, assigning, verifying, withdrawing, complete, error`.

---

## Text CLI (USB CDC)

The USB-CDC port is a **human text CLI only**. It carries log lines (prefixed
`# `), command output, and — by default — echoes what you type. There is no JSON
command/event stream on USB; that lives entirely on core 1's MQTT edge.

### Connecting

The Pico exposes a single USB-CDC ACM device (native USB — baud is irrelevant).
Any line-oriented serial terminal works:

```bash
screen /dev/tty.usbmodem111201
picocom /dev/tty.usbmodem111201
pyserial-miniterm /dev/tty.usbmodem111201
```

The CLI **echoes typed characters and honours backspace** for line editing by
default. If you drive it from a script or a terminal that echoes locally, turn
that off with `echo off` (and `echo on` to restore; bare `echo` shows the state).

### Line types

The firmware emits these kinds of line; the leading character distinguishes them:

| Line type | Leading | Description |
|-----------|---------|-------------|
| Log | `# ` | Narration of internal state changes — see format below |
| Command output | (other) | Reply to a typed command (`ok`, `err: <reason>`, or a `key=value` listing) |
| Echoed input | — | What you typed, echoed back (unless `echo off`) |

### Command grammar

`key=value` pairs, one command per line, keywords case-insensitive. **Bare = get.**
Reproduced from `help`:

```
help                                       list commands
show config                                global tunables
show mappings                              list all mappings
show mapping <fix> <btn>                   one mapping
show relay [<ch>]                          relay state (one or all)
show dali [<slave> [<addr>]]               DALI state
show group <slave> [<group>]               DALI group state
show present <slave>                       present ballast addresses
show provision                             provisioning status
relay <ch> [on=true|false|toggle]          bare=get, else set
dali <slave> <addr|gN> [key=val ...]       bare=get; keys: on bri min max
                                           power_on sys_fail fade_rate fade_time
                                           group (gN) also: members=0,1  add=2  remove=1
cfg [key=val ...]                          bare=get; long_press_ms dim_repeat_ms
                                           reversal_window_ms baud relay_slaves=1,2 dali_slaves=2,3
btn <fixture> <button> [tap|press|release] synthesize a button event (test)
dali_raw <slave> <cmd16> [twice]           send raw 16-bit DALI frame, report result
read_mem <slave> <addr> <bank> <off> <n>   read n bytes from a ballast memory bank (hex)
identify <slave> <addr>                    read a ballast's GTIN + serial (bank 0)
blink <slave> <addr>                       flash one ballast to locate it physically
gw_reboot <slave>                          reboot a TinyModbus gateway (recover stuck bus)
dim <slave> <addr|gN> {up|down}            step one ballast / group
provision <slave> {all|new|<addr>}         commission: all / incremental / re-provision collided <addr>
provision abort                            stop commissioning
scan <slave>                               blocking full scan
discover                                   re-probe all configured slaves (blocking)
ping <slave> [relay|dali]                  one ping at current master baud
set_slave_baud <slave> {9600|19200|38400} [relay|dali]  tell slave to switch its UART baud
baud [<rate> [save]]                       show / set UART baud (live; 'save' persists)
rs485 [{tx|always}]                        show / set RS485 DE mode (saved)
mqtt [show | set host=.. port=.. user=.. pass=.. prefix=..]   broker config (reboot to apply)
net  [show | set dhcp=.. ip=.. mask=.. gw=.. dns=..]          network config (reboot to apply)
ls [<path>]                                list files (default /)
cat <path>                                 dump a file to the console
map <fix> <btn> [targets=<s>:<a>,...]      bare=get; targets= deletes (a=N or gN)
log <level>                                set verbosity (off|err|warn|info|dbg)
show log                                   current verbosity
get_input <slave> <reg> [count]            raw FC04 read input registers
get_holding <slave> <reg> [count]          raw FC03 read holding registers
set_holding <slave> <reg> <value>          raw FC06 write holding register
get_coils <slave> <reg> [count]            raw FC01 read coils
set_coil <slave> <reg> {true|false|toggle} raw FC05 write coil
echo {on|off}                              echo input + backspace editing
reboot                                     watchdog reset
resync                                     re-emit hello + all state + mappings
```

`reg`/`value` arguments accept decimal or `0x`-prefixed hex. Commands with no
other output reply `ok`; errors reply `err: <reason>` (a short token, e.g.
`invalid_slave`, `out_of_range`, `bad_field`, `provision_busy`, `save_failed`).

### Examples

```text
> relay 3 on=true            # turn relay channel 3 on
ok

> dali 2 5 bri=127           # ballast 5 on slave 2 to ~50%
{...state_dali emitted to core 1...}

> dali 2 5 on=toggle min=120 # multiple fields in one command
> dali 2 g5 on=true          # group 5 on slave 2
> dali 2 g5 members=0,5,7    # set group 5 membership
> dali 2 g5 add=9            # add ballast 9 to group 5

> dim 2 5 down               # one DALI Down step on ballast 2.5
ok

> show dali 2 5
dali 2.5: present=true on=true bri=127 status=0x80 fading=false min=99 max=254 poweron=128 sysfail=128 fade_rate=7 fade_time=0

> show group 2
group 2.G5: members=[0,5,7] on=true bri=180

> map 3 1 targets=2:g5,2:g7  # bind fixture 3, button 1 to two groups
> map 3 1                    # read it back
map F3.B1 -> 2:G5,2:G7

> cfg long_press_ms=600      # change a timing parameter (persisted)
> mqtt set host=192.168.1.10 port=1883 prefix=lighting
ok (reboot to apply)
> net set dhcp=off ip=192.168.1.50 mask=255.255.255.0 gw=192.168.1.1 dns=192.168.1.1
ok (reboot to apply)
> net set hostname=lights    # DHCP option-12 name -> router DNS registers "lights"
ok (reboot to apply)

> provision 2 all            # full commissioning of slave 2
ok
> log dbg                    # raise verbosity to see DBG-level events
log: dbg
```

### Log line format

```
# <ts_ms> <level> <module>: <message>
```

Example: `# 12345 I dirty: ballast 2.5 arc=128 status=0x80`

- `ts_ms` — Pico uptime in milliseconds
- `level` — single character: `I` info, `W` warn, `E` err, `D` debug
- `module` — short lowercase tag (see below)
- `message` — free-form, key=value style where it carries data

#### Module tags

| Tag | Description |
|-----|-------------|
| `boot` | Startup, init sequence, watchdog reboot cause |
| `disc` | Boot-time bus discovery and baud recovery |
| `config` | Config load / legacy-file migration |
| `modbus` | RS-485 Modbus master — frames, timeouts, retries |
| `dirty` | relay-board polling + gateway reachability |
| `dstate` | DALI world model — presence changes, sweep, fade tracking |
| `dcom`  | controller-side commissioning |
| `light` | Lighting command dispatch — relay coil and DALI writes |
| `btn` | Button matrix — gestures, classified events |
| `prov` | DALI commissioning state machine |
| `serial` | USB-CDC hello / ready announcements |
| `cli` | Text command interface — parsed commands and replies |
| `core1` | Core-1 liveness — heartbeat, link/MQTT status, event drops |
| `net` | W5100S bring-up, DHCP, link state, recovery |
| `mqtt` | MQTT connection manager — connect, discovery, publish flow |

#### Log levels

Verbosity is set at runtime via `log <level>`. In order of increasing detail:
`off`, `err`, `warn`, `info` (default), `dbg`. Each level emits its own output
plus all less-verbose levels. Numeric `0`..`4` is also accepted. The current
level is `show log`. Verbosity is **not persisted** — boots default to `info`.

---

## MQTT / Home Assistant

Core 1 runs the MQTT bridge automatically — there is nothing to install or start.
The broker IP/port/creds/prefix and the network config are read from
`/config.json` at boot (set with the `mqtt` / `net` CLI commands or
`lighting/config/...` topics; both note "reboot to apply"). Discovery, the
availability model, and event/command translation live in `src/net/ha_bridge.c`,
which reproduces the deleted `mqtt_bridge.py` in C.

### Topic structure

All topics live under the prefix (default `lighting`).

| Topic | Direction | Retained | Description |
|-------|-----------|----------|-------------|
| `lighting/availability` | board → HA | Yes | `online` / `offline` (system LWT) |
| `lighting/gateway/<slave>/availability` | board → HA | Yes | per-gateway `online` / `offline` (relay board + each DALI gateway) |
| `lighting/relay/<ch>/state` | board → HA | Yes | `{"state":"ON"}` |
| `lighting/relay/<ch>/set` | HA → board | No | `{"state":"ON"}` |
| `lighting/dali/<slave>/<addr>/state` | board → HA | Yes | `{"state":"ON","brightness":N,...}` (JSON light schema) |
| `lighting/dali/<slave>/<addr>/set` | HA → board | No | `{"brightness":N}` or `{"state":"ON"}` |
| `lighting/dali/<slave>/<addr>/availability` | board → HA | Yes | `online` / `offline` (per ballast) |
| `lighting/dali/<slave>/<addr>/config` | board → HA | Yes | `{"min":..,"max":..,"power_on":..,"sys_fail":..,"fade_rate":..,"fade_time":..}` |
| `lighting/dali/<slave>/<addr>/config/<field>/set` | HA → board | No | integer for one config field |
| `lighting/group/<slave>/<g>/state` | board → HA | Yes | `{"state":"ON","brightness":N}` |
| `lighting/group/<slave>/<g>/set` | HA → board | No | `{"state":"ON"}` or `{"brightness":N}` |
| `lighting/group/<slave>/<g>/members/state` | board → HA | Yes | CSV of present member addresses |
| `lighting/group/<slave>/<g>/members/set` | HA → board | No | CSV of member addresses (replace) |
| `lighting/button/<fixture>/<button>` | board → HA | No | `{"event_type":"press"}` |
| `lighting/binding/<fixture>/<button>/state` | board → HA | Yes | binding as `s:a,s:gN,...` text |
| `lighting/binding/<fixture>/<button>/set` | HA → board | No | binding text — empty clears |
| `lighting/mapping/<fixture>/<button>` | board → HA | Yes | binding as JSON targets (legacy form) |
| `lighting/mapping/<fixture>/<button>/set` | HA → board | No | JSON targets array (legacy, still subscribed) |
| `lighting/mapping/<fixture>/<button>/delete` | HA → board | No | any payload (legacy, still subscribed) |
| `lighting/gateway/<slave>/diag` | board → HA | Yes | `{uptime,reboots,reset_reason}` — per-gateway diagnostics, polled every `gw_diag_interval_ms` |
| `lighting/config/state` | board → HA | Yes | `{long_press_ms,dim_repeat_ms,reversal_window_ms,gw_diag_interval_ms,baud,fw_version,max_*}` |
| `lighting/config/set` | HA → board | No | `{"long_press_ms":..,...}` (whole-object) |
| `lighting/config/<field>/set` | HA → board | No | integer for one config field |
| `lighting/provision/<slave>/status` | board → HA | Yes | `{"state":"..","msg":"..","devices_found":N}` |
| `lighting/provision/<slave>/start/all` | HA → board | No | any payload |
| `lighting/provision/<slave>/start/unaddressed` | HA → board | No | any payload |
| `lighting/provision/<slave>/start/address` | HA → board | No | any payload (uses target_state) |
| `lighting/provision/<slave>/start/target` | HA → board | No | integer 0–63 (the target address) |
| `lighting/provision/<slave>/target_state` | board → HA | Yes | current target address |
| `lighting/provision/<slave>/abort` | HA → board | No | any payload |

The bindings have two HA-facing forms: the **`binding/...`** text entity (the one
HA's mapper drives, `s:a` / `s:gN` CSV) and the **`mapping/...`** JSON form (kept
for back-compat). Core 1 subscribes to both `binding/+/+/set` and the legacy
`mapping/+/+/{set,delete}`.

### Availability model

- **System** (`lighting/availability`) — set as the MQTT Last Will. Goes
  `offline` the moment the board drops TCP (the broker fires the LWT). All
  entities are unavailable when this is offline.
- **Per-gateway** (`lighting/gateway/<slave>/availability`) — one per Modbus
  slave (relay board *and* each DALI gateway). Driven by gateway-reachability
  events from core 0. Relay channels and DALI entities both gate on their
  gateway's topic, so a single unreachable board takes only its own entities
  offline.
- **Per-ballast** (`lighting/dali/<slave>/<addr>/availability`) — from the DALI
  present bitmask. Seeded `offline` for all 64 addresses at discovery, then a
  sweep after each scan-complete brings responding ones online; presence deltas
  keep it current.

DALI lights use `"availability_mode":"all"` over the system, gateway, and
per-ballast topics — a ballast is shown available only when all three are online.
DALI groups gate on system + gateway only (a group is deliverable whenever the
gateway is up).

### Per-ballast config source + warm start

The per-ballast config (`min`/`max`/`power_on`/`sys_fail`/`fade_rate`/`fade_time`)
is **owned by the controller over raw DALI** — the gateway's config banks were
retired in task 04, so this is the only path (the `dali_raw_cfg` dual-path flag
was removed). Config is **read** with raw DALI queries (`QUERY
MIN/MAX/POWER-ON/SYSTEM-FAILURE LEVEL`, `QUERY FADE TIME/FADE RATE`) at boot (a
sweep of every present ballast) and refreshed by a slow background rotation, and
**written** with `DTR0`-load + `SET` sequences that are verified by a query-back
before the cache and the retained `.../config` topic update. Both the reads and
the writes ride the DALI command queue (see "DALI command queue: adapter +
generic client" above).

The `.../config` topic is published **retained**, so it doubles as the **warm
start**: on boot the controller subscribes to `dali/+/+/config` and adopts the
retained values into its cache (only while the cache is still unread — the swept
measured values then win), keeping HA's config numbers populated during the few
seconds the raw sweep takes. **This requires broker persistence** (retained
messages survive a broker restart): for Mosquitto set `persistence true` and a
`persistence_location` in `mosquitto.conf` (the default HA add-on config already
does). Without it the warm start is simply a no-op — the sweep still fills the
cache within seconds.

### Retiring a removed device (tombstones)

When a ballast, group, or gateway is physically removed, its retained discovery
and state topics would otherwise linger in HA and on the broker forever. The CLI
**`tombstone dali <slave> <addr>` / `tombstone group <slave> <g>` / `tombstone gw
<slave>`** publishes empty retained payloads to that device's discovery + data
topics, so HA drops the entity and the broker keeps no stale state. It is a
deliberate operator action (a transient DALI dropout is *availability*, not
removal, and is handled by the per-ballast availability topic). A whole-gateway
tombstone clears the entire gateway HA device (`lighting_dali_gw_<slave>`) — its
diagnostic + provisioning discovery configs and gateway-scoped data topics — so no
ghost device is left; its ballasts and groups are separate HA devices, tombstoned
individually. Tombstones are best-effort idempotent retained publishes, so re-run
the command if it coincided with heavy MQTT traffic.

### Discovery

Core 1 publishes HA MQTT auto-discovery payloads automatically on every (re)connect,
incrementally (TX-flow-controlled so the W5100S socket buffer never overruns).
No manual HA configuration is needed. The set includes:

- **Switch** entity per relay channel (user can override to light/fan in HA).
- **Light** entity per DALI ballast (JSON schema, brightness 0–254).
- **Number** entities per ballast for `min`, `max`, `power_on`, `sys_fail`,
  `fade_rate`, `fade_time` (config category).
- **Light** entity per DALI group (`DALI Group <slave>.<g>`) — all 16 groups per
  gateway are published.
- **Text** entity per group for `Members` (editable membership list).
- **Event** entity per button + a **Text** "binding" entity per button (the
  button-to-light mapper).
- **Sensor** entities per gateway for provision state / status / devices-found;
  **Button** entities for Commission All / New / Address / Abort; a **Number**
  for the target address.
- **Number/Sensor** entities on the controller device for the timing parameters,
  `fw_version`, and `baud`.

All device names use numbers only. Friendly names and rooms are assigned in HA's
device registry and persist across reboots and firmware updates.

---

## Config Web UI (core 1)

An embedded HTTP server on core 1 serves a single-page configuration app directly
from the controller — a **fallback for commissioning a fresh device and pointing
it at Home Assistant** before (or instead of) MQTT. It runs alongside the MQTT
client; both share the W5100S.

- **Access:** browse to the controller's IP (port 80). Find the IP from the
  `DHCP lease` line in the serial log, your DHCP server, or set a static IP via
  the UI / `net` CLI verb.
- **Auth:** HTTP **Basic auth**, credentials in `/config.json` (`web_user` /
  `web_pass`), default **`admin` / `admin`**, changeable from the UI's *Password*
  tab. **Plain HTTP — there is no TLS on the W5100S** — so Basic auth is base64,
  not encrypted. Treat this as a **LAN-only** commissioning tool, not an
  internet-facing surface, and change the default password.
- **What it does:** view live state (gateways, ballasts, groups, button
  fixtures) and edit everything the CLI can — RS-485 baud + DE mode, slave lists,
  per-ballast settings, groups, button bindings, provisioning, network (DHCP /
  static), and the MQTT broker. Network/MQTT changes apply on reboot (core 1
  reads them from the launch snapshot); lighting/baud/DE changes apply live.
- **API:** a small REST surface — `GET /api/state` (full snapshot, polled ~1.5 s)
  and `POST /api/{relay,dali,group,cfg,net,mqtt,auth,mapping,provision}`. Reads
  come from a core-1 cache fed by the same event stream the MQTT bridge consumes
  (`web_state.c`); writes become typed inter-core commands to core 0, exactly
  like MQTT-inbound commands. The UI **polls** rather than using SSE — see the
  socket note below.
- **Wire format:** `/api/state` is organised the way the domain is — `relays` a
  boolean array indexed by channel, and `dali` keyed by gateway slave, each slave
  an object `{ online, ballasts[], groups[] }` where `ballasts` is indexed by
  short address and `groups` by group number, with `null` for an unavailable
  slot (so positions stay stable as devices drop). `mappings`/`provision`/`cfg`
  are plain keyed objects. The renderer (`web_state.c`) streams this in ≤2 KB
  chunks via a resumable cursor. The UI has a single boundary —
  `web/src/decode.js` — that flattens the per-slave structure into the one
  consistently-named view model the tabs share; keep it in lock-step with the
  renderer.

### Why polling, and the W5100S socket budget

The W5100S has only **4 hardware sockets**: 0 = DHCP, 1 = MQTT, leaving **two**
for the web server (sockets 2–3). A long-lived SSE stream would permanently
consume one of those two, so the UI **short-polls** instead — the sockets stay
interchangeable and multiple tabs degrade gracefully instead of deadlocking.

To claw back a third socket, `netif` **lends the idle DHCP socket (0)** to the
web server between lease renewals and reclaims it a guard band before each T1
renewal (`netif_web_sock0_enabled()`); under a static IP there is no DHCP socket
so socket 0 is web-owned permanently. The web server treats socket 0 as
**revocable bonus capacity** and never depends on it. Each connection serves one
request then closes (no keep-alive), and all I/O is non-blocking and bounded to
one ≤2 KB chunk per socket per tick so MQTT and the watchdog stay healthy.

### Building the web UI

The app lives in `web/` (Vite, vanilla JS, **no runtime framework**). The
firmware build **requires Node/npm**: CMake runs the Vite build, gzips the
single-file output, and `web/tools/gen-assets.mjs` emits
`build/generated/web_assets.c` — a `const` gzipped blob served from flash (XIP,
no RAM copy, `Content-Encoding: gzip`). It rebuilds only when a `web/` source
changes. The whole UI is ~8 KB gzipped; `/config.json` on littlefs remains the
only mutable state.

```sh
make -C build            # builds web/ (Vite) + firmware in one step
```

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
- **Commission Address** — re-provisions the gear at one specific (collided)
  short address onto free addresses. Set the target in the "Target Address"
  number entity first.
- **Abort** — sends DALI Terminate immediately.

Progress shows in the Commission Status sensor, updating in near-real-time
(typically every 100–400 ms per step). The same operations are available from the
CLI: `provision <slave> {all|new|<addr>}` and `provision abort`.

### How it works

DALI commissioning uses a 24-bit binary search:

1. Devices are placed into initialisation mode (Initialise command).
2. Each device generates a random 24-bit number (Randomise command).
3. The search sets a Search Address to the midpoint of the current range and
   sends a Compare query. Any device with random address ≤ Search Address
   pulls the bus low (YES response). A bus collision also counts as YES.
4. If YES and the range has converged (low == high), the device at that random
   address is found. Programme Short Address is sent, Verify confirms it, then
   Withdraw removes it from the search pool.
5. The search resumes from random_address+1. Repeat until no more devices.

Re-provisioning one collided address (mode 3, `provision <slave> <addr>`) runs
the same `INITIALISE → RANDOMISE → bisect → PROGRAM → VERIFY → WITHDRAW` loop but
only over the gear answering at that address — you can't simply `PROGRAM SHORT
ADDRESS` because PROGRAM only programs the gear the binary search has *selected*.

After commissioning, the Pico triggers a TinyModbus rescan, waits ~300 ms for it
to settle, then runs a full scan — immediately updating the state cache and
emitting a scan-complete so HA availability updates without waiting for the 60 s
background check.

### Device identity & collisions

New ballasts often arrive pre-set to short address 0, colliding with existing
gear. Each device's stable **serial number** (memory bank 0, offset `0x0B`,
8 bytes) and **GTIN** (offset `0x03`, 6 bytes) anchor identity — the short
address is not stable. CLI tools:

- `dali_raw <slave> <cmd16> [twice]` — raw 16-bit DALI frame + result.
- `read_mem <slave> <addr> <bank> <off> <n>` — read a ballast memory bank.
- `identify <slave> <addr>` — GTIN + serial; a garbled serial = a collision.
- `blink <slave> <addr>` — flash one ballast (DAPC) to locate it physically.

`test/dali_inventory.py <slave>` scans and flags collisions; the spreadsheet sync
tooling records serial→address and can remap after a re-provision.

---

## Persistent Storage

The Pico uses pico-vfs (littlefs) for a filesystem in flash. Files are readable
and editable from the CLI (`ls`, `cat`) and hand-editable as plain text.

| File | Contents | Format |
|------|----------|--------|
| `/config.json` | Timing params, slave IDs, modbus baud, RS485 DE mode, MQTT broker/creds, network config | JSON text |
| `/mappings.json` | Button-to-light mappings | Line-delimited JSON (`{format,version}` header + one object per binding) |

**Legacy migration (one-time, automatic on first boot of new firmware):**
the old `/modbus_baud.txt` and `/modbus_config.txt` are folded into
`/config.json` and deleted; a valid legacy `/mappings.bin` (raw `mapping_record_t`
structs) is read once and re-saved as `/mappings.json`. The JSON mapping format
replaced the fragile binary format precisely because a struct-layout change (or a
whole-table overwrite while empty) could silently drop every binding.

---

## Home Assistant Device Hierarchy

```
Switch Gateway Controller
├── Long-Press (ms)        (config number)
├── Dim Repeat (ms)        (config number)
├── Reversal Window (ms)   (config number)
├── Gateway Diag Poll (ms) (config number — 0 disables)
├── Firmware               (diagnostic sensor — fw_version)
└── Modbus Baud            (diagnostic sensor)

Relay Board (Waveshare Modbus RTU Relay 32CH)
└── via_device ← Relay Channel 0, Relay Channel 1, ... (switch entities)
                  User overrides type to light/fan in HA entity registry

DALI Gateway 2 (TinyModbus ATtiny1604)
├── Commission State / Status / Devices Found   (diagnostic sensors)
├── Commission All / New Devices / Address / Abort   (config buttons)
├── Target Address          (config number, 0–63)
├── via_device ← DALI 2.00, DALI 2.01, ... (dimmable light entities)
│                 each with Min/Max/Power-On/System-Failure/Fade-Rate/Fade-Time
│                 config Number entities
└── via_device ← DALI Group 2.0 ... 2.15 (group light entities)
                  each with a Members text entity

DALI Gateway 3 (same structure)

Fixture 01 (switch plate 0)
├── Button 1 (event entity) + Button 1 Binding (config text entity)
├── ...
└── Button 7 (event entity) + Button 7 Binding (config text entity)

Fixture 02 ... Fixture 24 (same structure)
```

Group entities are separate HA lights from the per-ballast entities, not a
replacement: the same physical ballast can be a member of several groups, so
changing one ballast's level updates the per-ballast light plus every group light
it belongs to. A group light's `on`/`brightness` is the aggregate over its
present members.

All device names are numbered; rename and assign rooms in HA's Settings →
Devices & Services UI. These persist in HA's own registry across reboots and
firmware updates.

---

## Button Mapping

Button mappings (which buttons control which lights) are stored on the Pico in
`/mappings.json` and are configurable from HA or the CLI without firmware changes.

### Reading / setting / removing

- **Read:** subscribe to `lighting/binding/<f>/<b>/state`, or CLI `map <f> <b>`.
- **Set:** publish the target string to `lighting/binding/<f>/<b>/set`
  (e.g. `2:5,2:g3,1:12`), or CLI `map <f> <b> targets=2:5,2:g3,1:12`.
- **Remove:** publish an empty payload, or CLI `map <f> <b> targets=` (empty).

Each target is `<slave>:<addr>`, where `<addr>` is a ballast short address
(`0–63`) or a group `gN` (`0–15`). The relay board uses its slave id with a coil
number as the address; groups are DALI-only. Up to 8 targets per button — all are
controlled together. A group target sends one DALI group frame at button-press
time rather than N per-ballast commands, so it is faster and atomic on the bus.

The JSON `mapping/...` topics remain subscribed for back-compat (targets as
`[{"slave":2,"addr":5},{"slave":2,"group":3}]`).

### Bulk load from the spreadsheet

`tools/xlsx_to_ha.py` migrates `Switch wiring.xlsx` into Home Assistant **via the
`lightswitch_mapper` integration** — it renames the DALI light and fixture
devices, sets each fixture's physical button order, places devices on the
floorplan, and writes each button's binding text entity (so the bindings flow to
the controller over the `binding/...` topics). It runs against HA's REST +
websocket API (`--token <long-lived>` or `--dev`), dry-run by default; pass
`--apply` to write, or `--offline` to just print the plan from the xlsx.

```bash
python3 tools/xlsx_to_ha.py --offline                 # parse + print, no HA
python3 tools/xlsx_to_ha.py --token <tok> --apply      # write to HA
```

---

## Development & Debugging

### Serial monitor

Connect any serial terminal to the USB-CDC port (native USB, baud irrelevant).
The port carries logs, command output, and echoed input — see
[Text CLI](#text-cli-usb-cdc) for the full grammar. Drive the system with text
commands:

```bash
# from a shell, one command per line:
printf 'relay 3 on=true\n'   > /dev/tty.usbmodem111201
printf 'dali 2 5 bri=127\n'  > /dev/tty.usbmodem111201
printf 'show dali 2 5\n'     > /dev/tty.usbmodem111201
printf 'show mappings\n'     > /dev/tty.usbmodem111201
```

Raise verbosity with `log dbg` to watch Modbus traffic and the button/DALI event
flow. Core 1's own activity (link, DHCP, MQTT connect, discovery, publish flow)
logs on the same port under the `net` / `mqtt` / `core1` tags.

### Measuring the command path: `busload` and `soak`

Two bench instruments answer "is the comms path healthy, and how fast can it
actually go?" (task 07). Both are dev-grade CLI only — no MQTT surface.

**`busload [reset]`** — who is using the RS-485 bus. Each transaction is charged
from leaving the action queue until the bus SM returns to IDLE, so the figure
includes its retries and dead time — what that transaction really costs every
other producer. The per-slave split is the point: an absent board being retried
three times a poll costs the same bus time as real work and is invisible
everywhere else (the bench's dead relay at slave 1 holds ~8 % of the bus
permanently). Also reports re-sends split into timeout vs DEVICE_BUSY, and slave
turnaround against the 50 ms response timeout — which distinguishes a reply that
was *lost* from one that was merely *late*.

**`soak <slave> <addr> [n=<pairs>] [depth=<1-8>] [path=direct|bg|user]`** (bare =
report, `soak stop` to end an open-ended run) — hammers the gateway command queue with alternating

```
SET DTR0 = v          special command 0xA3<v>, one forward frame, no answer
QUERY CONTENT DTR0    opcode 0x98, backward frame — MUST answer v
```

DTR0 is deliberately the stimulus: it is the one settable value in the control
gear that is **volatile**, so unlike fade time or power-on level a long soak
costs no ballast EEPROM write cycles. `v` walks a stride-97 cycle over all 256
values, so consecutive pairs never share an expected answer — which makes a
result that is one command stale distinguishable from random corruption, and
both distinguishable from success.

`path=direct` (default) submits straight into `mbq_client`, bypassing the
scheduler, and measures the transport. `path=bg`/`path=user` route through
`dq_submit` at that priority class instead, turning the probe into realistic
load *of a chosen class* — which is how the scheduler's priority-inversion bound
gets measured: saturate with `path=bg` and time a `dali_raw` (a USER-class
submit, which reports its own elapsed time) against it. The direct path paces on
gateway-side slot occupancy (`depth`, default 4 = half the
queue) rather than on a DEVICE_BUSY rejection. Its report gives the verdict
(correct / lag-1 / other / nak / lost), pair and DALI frame rates, latency, the
Modbus transactions that paid for it, the bus split, and a pacing breakdown that
names the actual constraint: **queue-full % = DALI-bound** (good — the gateway
queue is kept fed and the DALI bus is the limit) vs **tx-busy % = Modbus-bound**.

Reference numbers on the bench (slave 3 addr 0, 19200): depth 4 sustains ~14
pairs/s = 28 DALI frames/s at 71 ms per pair, which is ~88 % of the IEC 62386
frame budget for that pair (~63 ms); throughput saturates at depth 3 and beyond
that only latency grows. See `tasks/07-queue-soak.md`.

### Resetting to defaults

Delete the config and reboot (`rm` is not a CLI verb; remove the file via the
USB bootloader mass-storage filesystem), **or** just rewrite the tunables:

```text
> cfg long_press_ms=500 dim_repeat_ms=200 reversal_window_ms=400
```

### Watchdog

The firmware enables a 5-second watchdog once boot completes. Core 0 only feeds it
while core 1's heartbeat is fresh (or during the launch grace window), so a stalled
main loop *or* a wedged core 1 (stuck SPI, infinite loop) reboots the board — but a
mere network outage does not. The reboot cause is logged on the next boot:
`# <ts> I boot: Rebooted by Watchdog`.

---

## Known Limitations

- **Baud rate change requires reboot.** Setting `baud` via `cfg` (or the
  `lighting/config/...` topic) persists the new rate but does not re-init the
  UART live; reboot to apply. (The CLI `baud <rate>` *does* re-init live, for
  bench use, and `baud <rate> save` also persists.)

- **Maximum 2 DALI gateways.** `MAX_DALI_SLAVES = 2` matches the hardware — two
  TinyModbus DALI gateways (slaves 2 and 3). The relay board (slave 1) is handled
  separately and does not count against this limit.

- **Provisioning is single-slave.** Only one DALI slave can be provisioned at a
  time. The other slave and the relay board remain fully operational during
  commissioning.
