#pragma once
#include <stdint.h>
#include <stdbool.h>

/*
 * lighting_actions — bridge between button_fsm / cli / ic_seam and modbus.
 *
 * Each function enqueues a Modbus action.  The callback updates state_cache
 * and pushes a state_event for ic_seam_pump() to emit to HA.
 *
 * These link button
 * events to Modbus writes.
 */

/* All command helpers below return true if the Modbus write was queued, false if
 * it was dropped (action queue full) or skipped (slave being provisioned). On
 * false they log the reason and do NOT update the predictive cache, so the cache
 * never claims a state the hardware was never told. Callers may ignore the result
 * (the drop is already logged) or react to it. */

/* ── State event queue — written here, drained by ic_seam_pump() ───────────
 * The single definition: ic_seam.c (formerly serial_protocol.c) used to
 * hand-copy this struct for its own levt_to_ic() translation, with
 * lighting_evt_try_pop()'s prototype re-declared there as `void *out` to paper
 * over the duplication. One copy, shared. */
typedef enum {
    LEVT_RELAY, LEVT_DALI, LEVT_FAULT,
    LEVT_SCAN_COMPLETE, LEVT_GROUP, LEVT_GATEWAY, LEVT_GW_DIAG
} levt_type_t;

typedef struct {
    levt_type_t type;
    union {
        struct { uint8_t ch; bool on; }                       relay;
        struct { uint8_t slave_id; uint8_t addr;
                 uint8_t arc; uint8_t status; }               dali;
        struct { uint8_t slave_id; uint8_t addr;
                 bool lamp_fail; bool power_cycle; }          fault;
        struct { uint8_t slave_id; uint64_t present_mask; }  scan_complete;
        struct { uint8_t slave_id; uint8_t group;
                 uint8_t bri; bool on; }                      group;
        struct { uint8_t slave_id; bool online; }            gateway;
        struct { uint8_t slave_id; uint32_t uptime_s;
                 uint16_t boot_count; uint8_t reset_reason; } gw_diag;
    };
} levt_t;

/* Drain one queued state event (FIFO). Returns false when the queue is empty.
 * Called from ic_seam_pump() once per main-loop iteration to forward each
 * event to core 1 as a typed ic_event_t (see levt_to_ic in ic_seam.c). */
bool lighting_evt_try_pop(levt_t *out);

/* Relay: on/off */
bool lighting_set_relay(uint8_t channel, bool on);

/* Decode one relay channel's on/off from a raw FC01 coil-status payload (the
 * coil-data bytes that follow the Modbus byte-count field). Honors
 * g_config.relay_byteswap: some boards (e.g. Waveshare Modbus RTU Relay 32CH)
 * return the coil-data bytes in reversed order, contrary to the Modbus spec, so
 * when the flag is set the byte index is mirrored within the payload. Bits
 * within a byte are spec-standard either way (LSB = lowest channel of the byte).
 * Returns false for channels beyond byte_count*8. */
bool lighting_relay_coil_on(const uint8_t *coil_bytes, uint8_t byte_count, uint8_t ch);

/* Every DALI command this module emits — levels, on/off, steps, group sends,
 * config writes, queries — goes through the gateway command queue
 * (dali_queue.h / dq_submit), driven by dq_tick() from the main loop. The
 * producers below submit into it directly; there is no separate raw-command
 * tick to call. */

/* ── Any-on probe ────────────────────────────────────────────────────────────
 * Ground-truth "is any light in this binding on?" for button toggles. Started
 * at press time so the answer is usually in hand by release (click) or by the
 * long-press deadline (dim direction). Sends QUERY LAMP POWER ON to each DALI
 * target of the binding (group targets query the group address — a native
 * any-on, since only gear that is on answers). Fully async: rides the modbus
 * action queue, advanced by lighting_any_on_probe_tick() from the main loop.
 * Single probe slot: if a probe is already running for a different button the
 * new press simply gets UNKNOWN (callers fall back). */
typedef enum {
    ANY_ON_PROBE_PENDING = -1,   /* still in flight */
    ANY_ON_PROBE_OFF     = 0,    /* every DALI target NAKed: nothing on */
    ANY_ON_PROBE_ON      = 1,    /* ≥1 target answered: something is on */
    ANY_ON_PROBE_UNKNOWN = 2,    /* no probe for this button / failed / timed out */
} any_on_probe_res_t;

/* Toggle a ballast (is_group false) or group ASYNCHRONOUSLY: probe the gear with
 * QUERY LAMP POWER ON and apply the inverse when the answer lands, all through
 * the normal queue. For the intercore path, which runs in the main loop and must
 * never block — reached today by the HTTP API / web UI, not by Home Assistant,
 * which resolves its own toggles and only ever sends explicit ON/OFF. An unresolved probe does nothing and warns — it does not
 * fall back to cached state. Returns false if no probe slot is free or the queue
 * refused the submit. */
bool               lighting_toggle_async(uint8_t slave_id, uint8_t target,
                                         bool is_group);

/* Blocking any-on probe for ONE DALI address (ballast or group selector byte),
 * asking the gear directly with QUERY LAMP POWER ON — the same ground truth the
 * button's press-time probe uses, so a CLI toggle and a button toggle agree.
 * TEXT CLI ONLY: it spins the bus, like the other cli_mb_wait helpers.
 * Returns false if unresolved; *any_on is then untouched. */
bool               lighting_any_on_probe_sync(uint8_t slave_id, uint8_t addr_byte,
                                              bool *any_on);

void               lighting_any_on_probe_start(uint8_t fixture, uint8_t button);
any_on_probe_res_t lighting_any_on_probe_get(uint8_t fixture, uint8_t button);
void               lighting_any_on_probe_tick(void);

/* ── Config-over-raw queries ──────────────────────────────────────────────────
 * Fill the controller's per-ballast config cache (min/max/power-on/sys-fail/
 * fade) by querying the gear directly — the only config path there is, now that
 * the gateway's own config block is gone. Ballasts are marked via a per-slave
 * pending bitmap; the tick walks it, runs the 5-query sequence per ballast (as
 * rolling windows of two, see lighting_actions.c), writes
 * state_cache_set_dali_config, and re-emits state_dali so HA sees the refreshed
 * config. */
/* Arm a raw config read for every ballast in `mask` (the boot sweep). Called
 * when a scan establishes presence, so HA's config numbers are populated from
 * the gear in seconds rather than over the ~15 min rotation. */
void lighting_cfg_query_mark_mask(uint8_t slave_id, uint64_t mask);

void lighting_cfg_query_mark(uint8_t slave_id, uint8_t addr);   /* one ballast */
void lighting_cfg_query_tick(void);

/* Per-ballast config fields writable over raw DALI. */
typedef enum {
    DALI_CFG_MIN = 0,
    DALI_CFG_MAX,
    DALI_CFG_POWER_ON,
    DALI_CFG_SYS_FAIL,
    DALI_CFG_FADE_RATE,
    DALI_CFG_FADE_TIME,
    DALI_CFG_FIELD_COUNT,
} dali_cfg_field_t;

/* Config write via raw DTR0-load + SET pair. Enqueues the two
 * frames back-to-back on the raw FIFO as an indivisible unit (nothing else can
 * take the slot between them — the FIFO is strictly ordered and the probe /
 * config-query SMs both wait while it is non-empty), then schedules a query-back
 * of the ballast so state_cache + the retained HA config reflect the ACTUAL
 * stored value (the ballast clamps out-of-range settings). Does NOT touch the
 * cache itself — the verifying query-back writes it through and re-emits
 * state_dali. Returns false if the FIFO can't hold both frames (nothing is
 * enqueued — all-or-nothing) or the slave is provisioning. */
bool lighting_dali_cfg_write(uint8_t slave_id, uint8_t addr,
                             dali_cfg_field_t field, uint8_t value);

/* DALI: on (goto last active) / off */
bool lighting_set_on_off(uint8_t slave_id, uint8_t dali_addr, bool on);

/* DALI: step up or down one level (uses ballast fade rate) */
bool lighting_dali_step(uint8_t slave_id, uint8_t dali_addr, bool up);

/* DALI: explicit arc level (DAPC) */
bool lighting_dali_set_level(uint8_t slave_id, uint8_t dali_addr, uint8_t arc);

/* DALI group helpers — single bus frame per call. State updates land via the
 * normal per-member dirty-bit polling path; no cache update on send. */
bool lighting_group_set_on_off(uint8_t slave_id, uint8_t group, bool on);
bool lighting_group_step(uint8_t slave_id, uint8_t group, bool up);
bool lighting_group_set_level(uint8_t slave_id, uint8_t group, uint8_t arc);

/*
 * Called from dirty_poll when a state change is confirmed by hardware.
 * Emits a state event for ic_seam_pump() to publish.
 */
void lighting_emit_relay_state(uint8_t channel, bool on);
void lighting_emit_dali_state(uint8_t slave_id, uint8_t addr,
                               uint8_t arc, uint8_t status);
/* Emitted when a full DALI interrogation completes (dirty_poll_tick notices
 * the transition), so the bridge can set per-ballast availability. */
void lighting_emit_scan_complete(uint8_t slave_id, uint64_t present_mask);

/* Emitted by dirty_poll when a DALI gateway's reachability changes (online =
 * responding to Modbus, offline = repeated read failures). Drives per-gateway
 * availability in the bridge. */
void lighting_emit_gateway_state(uint8_t slave_id, bool online);

/* Emitted by dirty_poll's periodic gateway-sysinfo read (FC04 0x0310): uptime
 * seconds, reboot count, and the raw RSTCTRL.RSTFR reset-reason byte. Published
 * as per-gateway diagnostic sensors in HA. */
void lighting_emit_gw_diag(uint8_t slave_id, uint32_t uptime_s,
                           uint16_t boot_count, uint8_t reset_reason);

/* Recompute a group's derived (on, bri) from its present members and emit
 * a state_group event if it differs from the last emitted value. Called
 * automatically from lighting_emit_dali_state for every group containing
 * the ballast that changed. Idempotent — safe to call when state is
 * unchanged. */
void lighting_emit_group_state_if_changed(uint8_t slave_id, uint8_t group);

/* Re-emit the full state snapshot to the CDC port. Pushes hello + every
 * cached entity state + scan_complete + every mapping. Idempotent and
 * non-blocking — pushes events to the same ring buffer the dirty-poll
 * machinery uses. Safe to call from any context that already calls into
 * lighting_emit_*. */
void lighting_resync_emit(void);
