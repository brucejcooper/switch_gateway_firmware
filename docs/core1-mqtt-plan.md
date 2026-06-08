# Plan: on-board MQTT bridge on core 1 (W5100S-EVB-Pico, RP2040)

Status: approved 2026-06-09. Branch: `core1-mqtt`.

Move the MQTT bridge from the USB-tethered Python host (`bridge/mqtt_bridge.py`)
onto the controller itself, running on **core 1**, talking to Home Assistant
directly over Ethernet via the on-board WIZnet W5100S. Core 0 keeps all existing
domain logic unchanged.

Hardware changes from Pico 2 (RP2350) **back to RP2040** (W5100S-EVB-Pico) plus
the W5100S hard-wired TCP/IP chip on SPI0.

## Architecture

```
                ┌──────────────────── CORE 0 (domain engine) ─────────────────────┐
 buttons ──────▶│ button_fsm ─┐                                                    │
 USB text CLI ─▶│ text parser ─┼─▶ domain dispatch ─▶ lighting_actions / mapping / │
 cmd queue ◀────┤ ic_cmd_t  ───┘     (one switch)        config / dali_provision   │
   (from c1)    │                                              │                   │
                │  modbus · dirty_poll · state_cache ──▶ ic_event_t staging ring   │
                │                                              │ (pump in loop)    │
                └──────────────────────────────────────────── │ ──────────────────┘
                          event queue (ic_event_t structs) ◀───┘   ──▶ to core 1
                ┌──────────────────── CORE 1 (network bridge) ────────────────────┐
                │ ic_event_t ─▶ HA translate ─▶ snprintf payload ─▶ coreMQTT pub   │
                │ MQTT sub ─▶ jsmn parse ─▶ ic_cmd_t ─▶ cmd queue ─▶ core 0        │
                │ W5100S socket SM · DHCP · DNS · discovery · availability · resync│
                └──────────────────────────────────────────────────────────────────┘
```

Key principle: **JSON exists only at the MQTT edge on core 1** (jsmn in,
`snprintf` out). The inter-core boundary is **pure C structs**. USB CDC is a
**text-only debug CLI**. Peripheral split: USB on core 0, SPI0/W5100S on core 1.

The whole firmware is **malloc-free and float-free**: coreMQTT uses
caller-provided fixed buffers, ioLibrary is static-buffer based, jsmn writes into
a stack token array, `snprintf` uses integer formats only (never request
`_printf_float`).

## Decisions made (with rationale)

- **Inter-core transport = typed structs, not JSON.** Round-tripping JSON
  (format on core 0 → ship → re-parse on core 1) is double work. Core 0 already
  represents events as a struct ring (`levt_t`); we extend it. `scan_complete`
  drops from ~4.5 KB of JSON to a 9-byte struct, so inter-core queues are small
  fixed-record `queue_t`s.
- **MQTT client = coreMQTT.** Rigorous PINGREQ/PINGRESP keepalive built in;
  malloc-free; integer-only. The W5100S-level reliability (CLOSE_WAIT, Sn_IR,
  RTR/RCR, chip reset, PHY) lives in our own socket SM beneath it.
- **JSON lib = jsmn (parse) + `snprintf` (build).** No malloc, no float. cJSON
  was rejected for its malloc/free + double-based numbers.
- **Network default = DHCP** (static selectable via config).
- **Credentials = `/config.json` + `mqtt set`/`net set` CLI**, placeholder
  compile-time defaults only — no secrets committed.
- **CLI stays on core 0** (text-only). It is a synchronous, blocking,
  print-inline debug shell coupled to core-0 resources; moving it to core 1 would
  force every debug command into async round-trips. Clean split: core 1 =
  autonomous async network bridge; core 0 = domain + interactive CLI.
- **`bridge/mqtt_bridge.py` kept as reference** (README-marked legacy), not a
  supported runtime path. **`bridge/sync_bindings.py` deleted** (HA + mapper tool
  own binding now). `replace_mappings_*` transaction dropped (only sync_bindings
  used it).
- **Config files consolidated 4 → 2.** `config.json` absorbs modbus baud + DE
  mode + new `mqtt`/`net`; `mappings.json` stays separate (large, frequently
  rewritten, deliberately tolerant format — must not regress the robustness the
  `/mappings.bin`→`/mappings.json` split bought). Core 0 reads `config.json` with
  jsmn; one-time migration imports then deletes the legacy `.txt` files.

## Pre-work findings (verified against the tree)

- **No pin conflict.** Button matrix SER=GP6, CLK=GP7, columns GP8–GP15
  (`COL_PIN_BASE=8` + 8 consecutive). SPI0 GP16–21 is clear (adjacent to GP15).
  RS-485 on GP1/4/5 (uart1).
- **Board header** `wiznet_w5100s_evb_pico.h` forces `PICO_PLATFORM=rp2040`,
  SPI0 SCK18/MOSI19/MISO16/CS17/RST20/INT21, 2 MB flash, W25Q080 stage2 — matches
  the spec table. Use `set(PICO_BOARD wiznet_w5100s_evb_pico)`.
- **Flash recipe correction.** `lib/pico-vfs/src/blockdevice/flash.c` already
  wraps every erase/program in `flash_safe_execute` (lines 98, 109). It is NOT
  reentrant — app-level wrapping would double-nest and fail. The only app action
  is core 1 calling `flash_safe_execute_core_init()` after launch. No core-0
  write site is touched.
- SDK 2.2.0 (exposes `flash_safe_execute` + `_core_init`). No float in hot paths
  (only `buttongrid.pio` clkdiv at init). 1 PIO SM used of 8. 256 KB littlefs at
  top of 2 MB; image 206 KB — no collision. Current BSS 36 KB.

## Inter-core contract (`src/intercore.h`)

Two `pico_util queue_t` (SPSC, spinlock-backed; SIO FIFO stays free for the
`flash_safe_execute` lockout handshake). Element ≈ 32 B.

```c
/* core0 → core1 : domain events (11 variants) */
typedef enum { IC_EVT_RELAY, IC_EVT_DALI, IC_EVT_GROUP, IC_EVT_GATEWAY,
  IC_EVT_PRESENCE, IC_EVT_FAULT, IC_EVT_SCAN_COMPLETE,
  IC_EVT_BUTTON, IC_EVT_MAPPING, IC_EVT_PROVISION, IC_EVT_CFG } ic_evt_type_t;

typedef struct { ic_evt_type_t type; union {
  struct { uint8_t ch; bool on; } relay;
  struct { uint8_t slave, addr, arc, status,
                   min, max, power_on, sys_fail, fade_rate, fade_time; } dali;
  struct { uint8_t slave, group, bri; bool on; uint64_t members; } group;
  struct { uint8_t slave; bool online; } gateway;
  struct { uint8_t slave; uint64_t appeared, disappeared; } presence;
  struct { uint8_t slave, addr; bool lamp_fail, power_cycle; } fault;
  struct { uint8_t slave; uint64_t present_mask; } scan_complete;   /* 9 bytes */
  struct { uint8_t fixture, button, action; } button;
  struct { uint8_t fixture, button, count;
           mapping_target_t targets[MAX_TARGETS_PER_BTN]; } mapping;
  struct { uint8_t slave, state, count; } provision;
  struct { uint16_t long_press_ms, dim_repeat_ms, reversal_window_ms;
           uint32_t baud; } cfg;
}; } ic_event_t;

/* core1 → core0 : commands (dispatched straight to domain fns) */
typedef enum { IC_CMD_SET_RELAY, IC_CMD_SET_DALI, IC_CMD_SET_GROUP, IC_CMD_SET_CFG,
  IC_CMD_SET_MAPPING, IC_CMD_DEL_MAPPING,
  IC_CMD_PROVISION_START, IC_CMD_PROVISION_ABORT, IC_CMD_RESYNC } ic_cmd_type_t;
/* field_mask says which v-keys are present; `on` is tri-state off/on/toggle */
```

`hello`, controller slave-lists, fw_version and grid dims are **derived on
core 1** from the config snapshot it holds + `version.h` — no event needed.
`ack` is dropped (bridge only logged it).

## Core-0 changes (seam only — domain logic untouched)

Unchanged: `modbus` (except config-read path, commit 8), `dirty_poll`,
`state_cache`, `button_fsm`, `mapping`, `dali_provision`, `bus_discovery`.

1. `main.c` (~15 lines): launch core 1 with explicit 8 KB stack; conditional
   watchdog feed gated on core-1 heartbeat + startup grace.
2. `lighting_actions.c`: extend the 256-deep `s_evts` staging ring from `levt_t`
   (7) to `ic_event_t` (11); `dali` variant fills bundled config from
   `state_cache` at push time.
3. `serial_protocol.c`: remove `handle_json_line` + all JSON emitters; add event
   pump (staging ring → core0→core1 queue, backpressure on full) + inbound drain
   (core1→core0 queue → `ic_dispatch_cmd` → domain fns). Keep `handle_text_line`.
4. `button_fsm_emit.c`, `serial_protocol_provision.c`: push `IC_EVT_BUTTON` /
   `IC_EVT_PROVISION` into the staging ring instead of `printf` JSON.
5. `config.c/.h`: `mqtt`/`net` keys + `mqtt set`/`net set` CLI; jsmn read; publish
   immutable config snapshot to core 1 + reload flag on change.

## Core 1 design (`src/net/`, `src/core1_main.c`)

- **Layer 1 — W5100S socket SM** (`net/wizchip_glue.c`, `net/netif.c`; owns SPI0;
  from WIZnet RP2040-HAT-C): reliability reqs 1–5, 8–10 — non-blocking sockets
  via `getSn_SR()`; explicit `SOCK_CLOSE_WAIT` recycle; act-on-`Sn_IR` + clear;
  tuned `RTR`≈200 ms / `RCR`≈3; `Sn_KPALVTR` HW keepalive; `getVERSIONR()==0x51`;
  full chip reset + `network_initialize` on repeated failure; `CW_GET_PHYLINK`
  link-down park; wall-clock timeout on every socket op. DHCP + DNS via ioLibrary.
- **Layer 2 — coreMQTT** (`net/mqtt_app.c`): reliability 6 (keepalive), 7 (LWT
  `<prefix>/availability=offline` at CONNECT), 8 (TCP+MQTT backoff + integer-LCG
  jitter).
- **Translate/discovery** (`net/ha_bridge.c`): C port of `mqtt_bridge.py`'s
  `_pico_*`/`_cmd_*`. On (re)connect: incremental discovery (`snprintf` into a
  reused ~2 KB buffer), subscribe, push `IC_CMD_RESYNC` → core 0 re-emits cached
  state → retained topics repopulate. jsmn parses inbound JSON; bare-number / CSV
  / binding-string payloads parsed directly.
- **`core1_main.c`**: `flash_safe_execute_core_init()` → loop { heartbeat; netif
  tick; coreMQTT process (0-timeout); drain event queue → publish; service subs →
  push commands }. Fully non-blocking. Heavy `net`/`mqtt`-tagged CDC logging.

## Config surface (`/config.json`, unified)

Existing scalars + `mqtt{host,port,user,pass,prefix}` +
`net{dhcp:true,ip,mask,gw,dns}` + modbus `baud` + `de_mode`. Runtime via
`mqtt set …` / `net set …`. Placeholder defaults only — no secrets committed.
One-time migration imports `/modbus_baud.txt` + `/modbus_config.txt` then deletes
them. `mappings.json` unchanged.

## Watchdog / heartbeat

`volatile uint32_t g_core1_heartbeat_ms` (atomic word), written every core-1 loop
incl. mid-reconnect (means "loop alive", not "connected"). Core 0 feeds the 5 s
HW watchdog only if heartbeat < 2 s stale, after a ~15 s startup grace (fed
unconditionally during grace for DHCP). 2 s threshold > worst-case flash write
(both cores park during `flash_safe_execute`). A network outage never reboots —
lights keep working with HA gone. Follow-up: `multicore_reset_core1()` instead of
whole-chip reboot.

## Memory (RP2040 264 KB)

New static ≈ staging ring +2 KB, two intercore queues ≈ 2 KB, coreMQTT net buf
≈ 2 KB, discovery buf ≈ 2 KB, ioLibrary/DHCP/DNS ≈ 3 KB, core-1 stack 8 KB ≈
~19 KB atop 36 KB BSS → ~55 KB, leaving ~200 KB. No heap. W5100S's 16 KB buffers
are on-chip. Report `.map` headroom once it builds.

## Libraries

- `lib/ioLibrary_Driver` (submodule) — W5100S socket/DHCP/DNS.
- `lib/coreMQTT` (submodule) — static, malloc/float-free.
- `lib/jsmn/jsmn.h` — single MIT header, vendored.

## Commit sequence

1. Build: board switch, submodules + jsmn, CMake wiring, link
   `pico_multicore`/`pico_flash`.
2. `intercore.h` + two queues + core-0 event pump / inbound drain +
   `ic_dispatch_cmd`; extend staging ring.
3. core-1 launch + `flash_safe_execute_core_init` + heartbeat/watchdog.
   **← PAUSE for review: builds clean, core 1 a stub, nothing networked.**
4. W5100S netif + socket SM (reliability 1–5, 8–10) + DHCP/DNS.
5. coreMQTT + LWT (reliability 6–8).
6. HA discovery (incremental `snprintf`) + availability.
7. Translate port (`_pico_*`/`_cmd_*`) + jsmn inbound + resync-on-connect.
8. Config consolidation: unify `config.json` (absorb baud/DE + mqtt/net), jsmn
   read on core 0, legacy migration, `mqtt set`/`net set` CLI; `modbus.c` reads
   baud/DE from `g_config`.
9. Remove `bridge/sync_bindings.py` + `replace_mappings_*`.
10. Strip JSON emit/parse from USB (text CLI only); README rewrite; `.map`
    headroom report.

## Open items (non-blocking, verify on hardware)

- Effective 5 s watchdog timeout on RP2040 (measure on first flash).
- Final `RTR`/`RCR`/`Sn_KPALVTR` values (bench-tune).
- Core-1 stack high-water + final SRAM headroom (from `.map`).
- Cannot flash/test from the dev host — conservative code + generous logging.
