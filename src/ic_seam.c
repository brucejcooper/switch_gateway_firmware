/*
 * ic_seam.c — the core-0/core-1 intercore seam (core 0).
 *
 * This is the SINGLE writer of g_config for all core-1 traffic (web UI +
 * Home Assistant writes come in as typed ic_cmd_t through ic_dispatch_cmd
 * below), and the forwarder that turns core-0 domain events into the typed
 * ic_event_t core 1 publishes to MQTT/the web UI. JSON exists only at core
 * 1's MQTT/HTTP edge — this seam never sees or produces it.
 *
 * Split out of serial_protocol.c (2026-08), which had grown to three
 * unrelated jobs under a name that stopped being true in June 2026 (the
 * machine-readable serial protocol died when the MQTT bridge moved
 * on-board): the human USB-CDC text CLI, this config-write seam, and the
 * event-forwarding pump. The seam — arguably the most correctness-critical
 * code in core 0 — was buried at line ~2500 of that console file, under a
 * name ("serial") that had nothing to do with what it does. See cli.h/cli.c
 * for the CLI half of the split.
 *
 * Runs entirely on core 0, driven once per main-loop iteration by
 * ic_seam_pump() (called from main.c, after cli_poll()).
 */

#include "ic_seam.h"
#include "cli.h"
#include "cli_provision.h"
#include "dali_provision.h"
#include "lighting_actions.h"
#include "dali_queue.h"
#include "dali_state.h"
#include "mapping.h"
#include "config.h"
#include "web_auth.h" /* IC_CMD_SET_AUTH's hashing setter */
#include "strfit.h"   /* str_set() — the shared fits-or-fails rule */
#include "state_cache.h"
#include "dirty_poll.h"
#include "modbus.h"
#include "intercore.h"
#include "core1.h"
#include "ota_stage.h"
#include "log.h"
#include "cdc_out.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <strings.h>
#include <pico/time.h>

#include "version.h"   /* FW_VERSION — generated from git at build time */

/* ── DALI config get/set ───────────────────────────────────────────────────
 * dali_cfg_field maps a field name to an INTERNAL (base,high) encoding — the
 * `base` values 0x0040/0x0080/0x00C0 are just a compact way to identify the
 * field pair (min|max, power_on|sys_fail, fade_rate|fade_time); they are NOT
 * gateway register addresses (the gateway serves no config banks).
 * dali_cfg_set translates (base,high) into a DALI field enum and writes it via a
 * raw DTR0+SET queue entry (lighting_dali_cfg_write). The ballast may clamp a
 * value (e.g. min to its physical minimum); the query-back rescan then reflects
 * the actual stored value. */
static bool dali_cfg_field(const char *name, uint16_t *base, bool *high) {
    if (strcasecmp(name, "min")       == 0) { *base = 0x0040u; *high = true;  return true; }
    if (strcasecmp(name, "max")       == 0) { *base = 0x0040u; *high = false; return true; }
    if (strcasecmp(name, "power_on")  == 0) { *base = 0x0080u; *high = true;  return true; }
    if (strcasecmp(name, "sys_fail")  == 0) { *base = 0x0080u; *high = false; return true; }
    if (strcasecmp(name, "fade_rate") == 0) { *base = 0x00C0u; *high = true;  return true; }
    if (strcasecmp(name, "fade_time") == 0) { *base = 0x00C0u; *high = false; return true; }
    return false;
}

/* Returns 1 ok, 0 bad field, -1 not present, -2 transport error (printed). */
static int dali_cfg_set(uint8_t slave, uint8_t addr, const char *field, uint8_t value) {
    uint16_t base; bool high;
    if (!dali_cfg_field(field, &base, &high)) return 0;
    int idx = state_cache_slave_idx(slave);
    dali_ballast_state_t *b = idx >= 0 ? state_cache_get_dali((uint8_t)idx, addr) : NULL;
    if (!b) return -1;

    /* Config writes are always raw DTR0-load + SET pairs, verified by a
     * query-back that write-throughs the cache. No predictive cache update here
     * — the retained value only changes on verify. dali_cfg_field's (base,high)
     * maps 1:1 onto the field enum. */
    dali_cfg_field_t f = base == 0x0040u ? (high ? DALI_CFG_MIN       : DALI_CFG_MAX)
                       : base == 0x0080u ? (high ? DALI_CFG_POWER_ON  : DALI_CFG_SYS_FAIL)
                       :                    (high ? DALI_CFG_FADE_RATE : DALI_CFG_FADE_TIME);
    /* 2 = applied async; the query-back write-throughs the cache and emits
     * the verified (possibly clamped) value — no premature pre-verify emit. */
    return lighting_dali_cfg_write(slave, addr, f, value) ? 2 : -2;
}

/* ── Unified {t,d,v} state reports — used by get_* and as set_* confirmation ─
 * Same shape the client sends to set: {"t":"state_X","d":{addr},"v":{values}}. */

/* Push the cached state of one entity to core 1 (the network bridge). These are
 * the get/predictive paths; the unsolicited dirty-poll path goes through the
 * lighting event ring + levt_to_ic in ic_seam_pump(). USB CDC carries no JSON —
 * it is a human text CLI only (see cli.c's `show`). */
/* NOTE for both emit_state_* below: these publish to CORE 1 (MQTT/HA) via the
 * intercore queue. They print NOTHING to the console.
 *
 * That is a hangover from when the Python host bridge consumed this CLI's JSON
 * over the serial wire — emitting the report WAS the reply. Since the on-board
 * bridge moved to core 1 (2026-06-09) the report goes to a channel the human at
 * the terminal cannot see, so every CLI verb using these looked like it did
 * nothing: `dali 3 0 on=true` switched the light and printed not a byte. Text
 * command handlers must therefore finish with txt_ok() or txt_err() (cli.c);
 * ic_dispatch_cmd below must NOT, since nothing is listening there. */
void emit_state_dali(uint8_t slave, uint8_t addr) {
    int idx = state_cache_slave_idx(slave);
    dali_ballast_state_t *b = idx >= 0 ? state_cache_get_dali((uint8_t)idx, addr) : NULL;
    ic_event_t ev = { .type = IC_EVT_DALI };
    ev.dali.slave     = slave;
    ev.dali.addr      = addr;
    ev.dali.arc       = b ? b->arc_level   : 0;
    ev.dali.status    = b ? b->status_byte : 0;
    ev.dali.min       = b ? b->min_level       : 0;
    ev.dali.max       = b ? b->max_level       : 0;
    ev.dali.power_on  = b ? b->power_on_level   : 0;
    ev.dali.sys_fail  = b ? b->sys_fail_level   : 0;
    ev.dali.fade_rate = b ? b->fade_rate        : 0;
    ev.dali.fade_time = b ? b->fade_time        : 0;
    intercore_push_event(&ev);
}

void emit_state_group(uint8_t slave, uint8_t group) {
    int idx = state_cache_slave_idx(slave);
    ic_event_t ev = { .type = IC_EVT_GROUP };
    ev.group.slave   = slave;
    ev.group.group   = group;
    ev.group.on      = idx >= 0 && state_cache_dali_group_any_on((uint8_t)idx, group);
    ev.group.bri     = idx >= 0 ? state_cache_dali_group_max_bri((uint8_t)idx, group) : 0;
    ev.group.members = idx >= 0 ? state_cache_get_dali_group((uint8_t)idx, group) : 0;
    intercore_push_event(&ev);
}

/* Set a DALI group's membership.
 *
 * The controller diffs membership against its own cache and emits the
 * ADD/REMOVE FROM GROUP frames itself; the gateway holds no masks.
 *
 * ADD TO GROUP is 0x60|g and REMOVE FROM GROUP is 0x70|g, both SET-class so
 * they go send-twice, addressed to each ballast whose membership changed. They
 * are idempotent (re-adding a member is a no-op), so a retry is safe.
 *
 * The membership cache is NOT taken on trust afterwards: each touched ballast is
 * marked for a full re-interrogation, so QUERY GROUPS confirms what actually
 * stuck. Predicting it locally is what made the old bug invisible. */
void dali_group_set_members(uint8_t slave, uint8_t group, uint64_t new_mask) {
    int idx = state_cache_slave_idx(slave);
    if (idx < 0) return;
    uint64_t old_mask = state_cache_get_dali_group((uint8_t)idx, group);
    uint64_t diff = old_mask ^ new_mask;
    if (!diff) return;

    for (uint8_t a = 0; a < DALI_ADDR_COUNT; a++) {
        uint64_t bit = (uint64_t)1 << a;
        if (!(diff & bit)) continue;
        bool add = (new_mask & bit) != 0;
        uint8_t addr_byte = (uint8_t)(((a & 0x3Fu) << 1) | 0x01u);
        uint8_t opcode    = (uint8_t)((add ? 0x60u : 0x70u) | (group & 0x0Fu));

        /* The user class is bounded, so spin the bus until this one is accepted
         * rather than dropping members of a membership change on the floor. */
        absolute_time_t dl = make_timeout_time_ms(2000);
        while (!dq_submit(slave, DQ_PRIO_GROUP_WRITE, addr_byte, opcode,
                          DQ_TWICE, 0, true, NULL, NULL)) {
            if (absolute_time_diff_us(get_absolute_time(), dl) <= 0) {
                LOG_WARN("cli", "group %u.G%u %s addr %u dropped — queue busy",
                         slave, group, add ? "add" : "remove", a);
                break;
            }
            modbus_poll();
            dq_tick();
            wd_feed();
        }
        dali_state_mark_scan(slave, a);   /* confirm by reading it back */
    }
    /* NO optimistic cache write. dali_state_mark_scan above re-reads QUERY GROUPS
     * for every touched ballast, and commit_ballast writes the measured mask —
     * that is the only writer. Recording new_mask here would publish membership
     * we had merely requested, and a write that silently failed would sit in the
     * cache (and in HA) until something else happened to re-read it.
     * lighting_emit_group_state_if_changed now compares members, so the read-back
     * publishes the new list on its own. */
    (void)idx;
}

void emit_state_cfg(void) {
    /* fw_version / slave lists / grid dimensions are derived on core 1 from its
     * config snapshot + version.h. */
    ic_event_t ev = { .type = IC_EVT_CFG };
    ev.cfg.long_press_ms      = g_config.long_press_ms;
    ev.cfg.dim_repeat_ms      = g_config.dim_repeat_ms;
    ev.cfg.reversal_window_ms = g_config.reversal_window_ms;
    ev.cfg.baud               = modbus_get_baud();
    ev.cfg.relay_byteswap     = g_config.relay_byteswap ? 1u : 0u;
    ev.cfg.gw_diag_interval_ms = g_config.gw_diag_interval_ms;
    ev.cfg.dali_sweep_interval_ms = g_config.dali_sweep_interval_ms;
    intercore_push_event(&ev);
}

/* Apply one DALI key=value to a ballast (shared by set_dali JSON + `dali` CLI).
 * NOTHING HERE PREDICTS. State keys (on/bri) command the ballast and return 2 —
 * "applied, nothing measured yet" — so no value is cached or published until a
 * verification read reports what the gear actually did. Config keys go through
 * dali_cfg_set, which has always worked this way.  Returns 1 ok, 0 unknown key, -1 not present, -2 transport err. */
int dali_apply_kv(uint8_t slave, uint8_t addr, const char *key, const char *val) {
    /* No cached-state reads needed: nothing here predicts a level any more, so
     * there is no old value to carry forward. */
    if (strcasecmp(key, "bri") == 0) {
        uint8_t lvl = (uint8_t)atoi(val);
        lighting_dali_set_level(slave, addr, lvl);
        /* The requested level is NOT written to the cache and NOT published.
         * A ballast clamps to its own min/max (and its physical minimum, which
         * can be far above the request) and may fade there over seconds, so the
         * number asked for is frequently not the number the light ends up at.
         * Publishing it made HA show the request, then jump when the real level
         * arrived. Only the verification read publishes, once the fade settles —
         * see commit_ballast()/fade_watch in dali_state.c.
         * Returns 2 = applied, nothing known to report yet. */
        return 2;
    }
    if (strcasecmp(key, "on") == 0) {
        /* "toggle" reaches here only from the JSON/intercore path — the CLI
         * resolves it with a blocking probe before calling in (see cmd_dali in
         * cli.c). This path runs in the main loop and must not block, so it
         * hands off to the ASYNC probe: query the gear, apply the inverse when
         * the answer lands. */
        if (strcasecmp(val, "toggle") == 0) {
            if (!lighting_toggle_async(slave, addr, false)) return -2;
            return 2;   /* applied later — nothing to report yet */
        }
        bool on = (strcmp(val, "true") == 0 || strcmp(val, "1") == 0 ||
                   strcasecmp(val, "on") == 0);
        lighting_set_on_off(slave, addr, on);
        /* Not even OFF is written ahead of the read. The endpoint is certainly
         * level 0, but with a fade time the light is NOT there yet, and a cache
         * that says 0 while the lamp is still ramping down is an inferred value
         * on a topic that must only carry measured ones. lighting_set_on_off
         * only records the desired state; the verification read is marked when
         * its frame actually ships, and reports the real level, including each
         * step of the ramp.
         *
         * On: GOTO_LAST_ACTIVE restores a level the controller does NOT know.
         * Don't assume it (the old code guessed max_level/254) — the
         * verification read reports the real brightness shortly. */
        return 2;   /* applied — nothing measured yet to report */
    }
    return dali_cfg_set(slave, addr, key, (uint8_t)atoi(val));
}

/* Emit a config_mapping line for one (fixture, button). Empty `targets`
 * (count == 0) clears the bridge's retained MQTT entry for that button. */
void emit_config_mapping(uint8_t fixture, uint8_t button,
                         const mapping_target_t *targets, uint8_t count) {
    /* Forward the binding to core 1 (the MQTT bridge) as a typed struct. */
    ic_event_t ev = { .type = IC_EVT_MAPPING };
    ev.mapping.fixture = fixture;
    ev.mapping.button  = button;
    ev.mapping.count   = count;
    for (int i = 0; i < count && i < MAX_TARGETS_PER_BTN; i++)
        ev.mapping.targets[i] = targets[i];
    intercore_push_event(&ev);
}

/* ── State event serialisation ───────────────────────────────────────────── */
/* levt_type_t / levt_t come from lighting_actions.h (included above) — no
 * local copy; lighting_evt_try_pop()'s prototype there carries the real type. */

/* ── Inter-core seam (core0 -> core1 events, core1 -> core0 commands) ─────────
 * Convert a drained levt_t (the core-0 staging-ring event) to the typed
 * ic_event_t forwarded to core 1. Runs on core 0 in ic_seam_pump(), so it can
 * read state_cache for the bundled DALI config and group membership. Returns
 * false for kinds with no core-1 representation (none currently). */
static bool levt_to_ic(const levt_t *e, ic_event_t *ic) {
    switch (e->type) {
    case LEVT_RELAY:
        ic->type = IC_EVT_RELAY;
        ic->relay.ch = e->relay.ch;
        ic->relay.on = e->relay.on;
        return true;
    case LEVT_DALI: {
        ic->type = IC_EVT_DALI;
        ic->dali.slave  = e->dali.slave_id;
        ic->dali.addr   = e->dali.addr;
        ic->dali.arc    = e->dali.arc;
        ic->dali.status = e->dali.status;
        int didx = state_cache_slave_idx(e->dali.slave_id);
        dali_ballast_state_t *b =
            didx >= 0 ? state_cache_get_dali((uint8_t)didx, e->dali.addr) : NULL;
        ic->dali.min       = b ? b->min_level       : 0;
        ic->dali.max       = b ? b->max_level       : 0;
        ic->dali.power_on  = b ? b->power_on_level   : 0;
        ic->dali.sys_fail  = b ? b->sys_fail_level   : 0;
        ic->dali.fade_rate = b ? b->fade_rate        : 0;
        ic->dali.fade_time = b ? b->fade_time        : 0;
        /* Those six are placeholders until a raw config read has landed. Tell
         * core 1 so it can stay silent rather than publishing zeros as fact. */
        ic->dali.cfg_known = b ? b->cfg_known : false;
        return true;
    }
    case LEVT_FAULT:
        ic->type = IC_EVT_FAULT;
        ic->fault.slave       = e->fault.slave_id;
        ic->fault.addr        = e->fault.addr;
        ic->fault.lamp_fail   = e->fault.lamp_fail;
        ic->fault.power_cycle = e->fault.power_cycle;
        return true;
    case LEVT_SCAN_COMPLETE:
        ic->type = IC_EVT_SCAN_COMPLETE;
        ic->scan_complete.slave        = e->scan_complete.slave_id;
        ic->scan_complete.present_mask = e->scan_complete.present_mask;
        return true;
    case LEVT_GROUP: {
        ic->type = IC_EVT_GROUP;
        ic->group.slave = e->group.slave_id;
        ic->group.group = e->group.group;
        ic->group.bri   = e->group.bri;
        ic->group.on    = e->group.on;
        int sidx = state_cache_slave_idx(e->group.slave_id);
        ic->group.members = sidx >= 0
            ? state_cache_get_dali_group((uint8_t)sidx, e->group.group) : 0;
        return true;
    }
    case LEVT_GATEWAY:
        ic->type = IC_EVT_GATEWAY;
        ic->gateway.slave  = e->gateway.slave_id;
        ic->gateway.online = e->gateway.online;
        return true;
    case LEVT_GW_DIAG:
        ic->type = IC_EVT_GW_DIAG;
        ic->gw_diag.slave        = e->gw_diag.slave_id;
        ic->gw_diag.uptime_s     = e->gw_diag.uptime_s;
        ic->gw_diag.boot_count   = e->gw_diag.boot_count;
        ic->gw_diag.reset_reason = e->gw_diag.reset_reason;
        return true;
    }
    return false;
}

/* Dispatch a command from core 1 through the same domain entry points the JSON
 * and text CLIs use. The tiny value-string reconstruction for set_dali keeps all
 * clamping/handling inside dali_apply_kv() (one place); the cost is negligible
 * and only on infrequent commands. */
static void ic_dispatch_cmd(const ic_cmd_t *c) {
    switch (c->type) {
    case IC_CMD_SCAN: {
        uint8_t slave = c->scan.slave;
        if (state_cache_slave_idx(slave) < 0) {
            LOG_WARN("cli", "ic scan reject: slave=%u not configured", slave); break; }
        /* Same call the `scan` CLI verb makes, and asynchronous for the same
         * reason: on a populated bus an interrogation is many seconds, and this
         * arrives on a LIVE system where someone may be at a wall switch.
         * Blocking here would stall the main loop and freeze the buttons. */
        LOG_INFO("cli", "ic scan slave %u (async)", slave);
        dirty_poll_arm_scan(slave);
        break;
    }
    case IC_CMD_SET_RELAY: {
        uint8_t ch = c->set_relay.ch;
        if (ch >= RELAY_COUNT) { LOG_WARN("cli", "ic set_relay reject: ch=%u", ch); break; }
        if (c->set_relay.on == IC_TRI_TOGGLE && !state_cache_relay_known(ch)) {
            LOG_WARN("cli", "relay %u toggle ignored — state never read", ch);
            break;
        }
        bool on = (c->set_relay.on == IC_TRI_TOGGLE) ? !state_cache_get_relay_on(ch)
                                                     : (c->set_relay.on == IC_TRI_ON);
        lighting_set_relay(ch, on);
        /* No predictive write and no emit: the relay write ACK caches and emits on the
         * write ACK, and a failed write must not leave HA showing the requested
         * state. Emitting here would publish the PRE-command cached value anyway. */
        break;
    }
    case IC_CMD_SET_DALI: {
        uint8_t slave = c->set_dali.slave, addr = c->set_dali.addr;
        if (state_cache_slave_idx(slave) < 0 || addr > 63) {
            LOG_WARN("cli", "ic set_dali reject: slave=%u addr=%u", slave, addr); break; }
        char vb[12];
        bool any = false, emit_now = false;
        if (c->set_dali.fields & IC_DALI_F_ON) {
            const char *v = c->set_dali.on == IC_TRI_TOGGLE ? "toggle"
                          : c->set_dali.on == IC_TRI_ON     ? "true" : "false";
            if (dali_apply_kv(slave, addr, "on", v) == 1) emit_now = true;
            any = true;
        }
        #define IC_DALI_APPLY(flag, key, field)                                  \
            if (c->set_dali.fields & (flag)) {                                   \
                snprintf(vb, sizeof vb, "%u", c->set_dali.field);                \
                if (dali_apply_kv(slave, addr, (key), vb) == 1) emit_now = true; \
                any = true;                                                      \
            }
        IC_DALI_APPLY(IC_DALI_F_BRI,       "bri",       bri);
        IC_DALI_APPLY(IC_DALI_F_MIN,       "min",       min);
        IC_DALI_APPLY(IC_DALI_F_MAX,       "max",       max);
        IC_DALI_APPLY(IC_DALI_F_POWER_ON,  "power_on",  power_on);
        IC_DALI_APPLY(IC_DALI_F_SYS_FAIL,  "sys_fail",  sys_fail);
        IC_DALI_APPLY(IC_DALI_F_FADE_RATE, "fade_rate", fade_rate);
        IC_DALI_APPLY(IC_DALI_F_FADE_TIME, "fade_time", fade_time);
        #undef IC_DALI_APPLY
        if (any) {
            dirty_poll_mark_active(slave);
            if (emit_now) emit_state_dali(slave, addr);
        }
        break;
    }
    case IC_CMD_SET_GROUP: {
        uint8_t slave = c->set_group.slave, group = c->set_group.group;
        if (state_cache_slave_idx(slave) < 0 || group > 15) {
            LOG_WARN("cli", "ic set_group reject: slave=%u group=%u", slave, group); break; }
        bool any = false;
        if (c->set_group.fields & IC_GRP_F_MEMBERS) {
            dali_group_set_members(slave, group, c->set_group.members);
            any = true;
        }
        if (c->set_group.fields & IC_GRP_F_BRI) {
            lighting_group_set_level(slave, group, c->set_group.bri);
            any = true;
        } else if (c->set_group.fields & IC_GRP_F_ON) {
            /* A toggle probes the group and applies the inverse asynchronously.
             * Comparing against IC_TRI_ON alone would make IC_TRI_TOGGLE mean OFF.
             *
             * Reached from the HTTP API / web UI. HA cannot send a toggle: its
             * JSON light schema carries only {"state":"ON"|"OFF"} and
             * ha_bridge maps anything that is not "ON" to OFF. */
            if (c->set_group.on == IC_TRI_TOGGLE) {
                lighting_toggle_async(slave, group, true);
            } else {
                lighting_group_set_on_off(slave, group, c->set_group.on == IC_TRI_ON);
            }
            any = true;
        }
        if (any) {
            dirty_poll_mark_active(slave);
            /* Only a MEMBERSHIP change is publishable straight away — that is
             * controller-owned state. on/bri are derived from member levels that
             * have not been re-read yet, so they are left to the member
             * verification reads, which recompute the group as each lands. This
             * unconditional emit was the first of the two publishes seen per
             * group command, and it carried pre-command values. */
            if (c->set_group.fields & IC_GRP_F_MEMBERS) emit_state_group(slave, group);
        }
        break;
    }
    case IC_CMD_SET_CFG: {
        bool any = false;
        /* Schema-covered numerics/bool route through config_apply_u32(), which
         * range-checks against CFG_SCHEMA (config.c) before writing — this is
         * the backstop layer: the CLI/HTTP-API/ha_bridge callers already
         * range-check their own way in, but this seam is the single writer of
         * g_config and a future core-1 producer could skip that check, so an
         * out-of-range value is skipped (not clamped or wrapped) and logged,
         * matching the hostname/mqtt-string backstop pattern below. */
        #define APPLY_CFG_U32(FLAG, KEY, VAL) do {                                  \
            if (c->set_cfg.fields & (FLAG)) {                                       \
                cfg_apply_result_t r = config_apply_u32((KEY), (VAL));              \
                if (r == CFG_APPLY_OK) any = true;                                  \
                else if (r == CFG_APPLY_OUT_OF_RANGE)                               \
                    LOG_WARN("cli", "ic set_cfg: " KEY " out of range — field skipped"); \
            } } while (0)
        APPLY_CFG_U32(IC_CFG_F_LONG_PRESS,      "long_press_ms",          c->set_cfg.long_press_ms);
        APPLY_CFG_U32(IC_CFG_F_DIM_REPEAT,      "dim_repeat_ms",          c->set_cfg.dim_repeat_ms);
        APPLY_CFG_U32(IC_CFG_F_REVERSAL,        "reversal_window_ms",     c->set_cfg.reversal_window_ms);
        APPLY_CFG_U32(IC_CFG_F_RELAY_BYTESWAP,  "relay_byteswap",        c->set_cfg.relay_byteswap);
        APPLY_CFG_U32(IC_CFG_F_GW_DIAG,          "gw_diag_interval_ms",    c->set_cfg.gw_diag_interval_ms);
        APPLY_CFG_U32(IC_CFG_F_DALI_SWEEP,       "dali_sweep_interval_ms", c->set_cfg.dali_sweep_interval_ms);
        #undef APPLY_CFG_U32
        /* baud is not in CFG_SCHEMA — modbus_set_baud() owns its own divisor
         * validity, unchanged. */
        if ((c->set_cfg.fields & IC_CFG_F_BAUD) && c->set_cfg.baud > 0) { modbus_set_baud(c->set_cfg.baud); any = true; }
        if (any && !config_save()) LOG_ERR("cli", "config_save failed — set_cfg not persisted");
        emit_state_cfg();
        break;
    }
    case IC_CMD_SET_NETCFG: {
        uint16_t f = c->set_netcfg.fields;
        if (f & IC_NET_F_DHCP) g_config.net_dhcp = c->set_netcfg.dhcp ? 1 : 0;
        if (f & IC_NET_F_IP)   memcpy(g_config.net_ip,   c->set_netcfg.ip,   4);
        if (f & IC_NET_F_MASK) memcpy(g_config.net_mask, c->set_netcfg.mask, 4);
        if (f & IC_NET_F_GW)   memcpy(g_config.net_gw,   c->set_netcfg.gw,   4);
        if (f & IC_NET_F_DNS)  memcpy(g_config.net_dns,  c->set_netcfg.dns,  4);
        /* Backstop: /api/net rejects a bad label up front with a 400 so the
         * user sees it, but this is the single writer to g_config, and the
         * command can also arrive from a future core-1 producer that hasn't
         * checked. Keep the gate, and say so when it fires — silently dropping
         * the field while config_save() below still reported success is what
         * made a rejected hostname look like it had been applied. Routed
         * through config_apply_str()'s CFG_HOSTNAME row, which is exactly
         * config_hostname_set() underneath. */
        if (f & IC_NET_F_HOSTNAME) {
            if (config_apply_str("net_hostname", c->set_netcfg.hostname) != CFG_APPLY_OK)
                LOG_WARN("cli", "ignored invalid hostname from web/API");
        }
        if (f && !config_save()) LOG_ERR("cli", "config_save failed — netcfg not persisted");
        break;
    }
    /* Backstop only: /api/mqtt and /api/auth reject an over-long value up front
     * with a 400 so the caller actually sees it. These commands cross the
     * intercore seam into the single writer of g_config, though, and the
     * source fields are the same size as the destinations — so a value can
     * only fail here if a future core-1 producer skips that check. Refuse and
     * LOG rather than truncate: a silently shortened password or prefix saves,
     * reports success, and then simply does not work. Routed through
     * config_apply_str()/config_apply_u32() (config.c) instead of str_set()
     * directly, so this backstop enforces the SAME rule (including mqtt_port's
     * range, previously unchecked here) as every other writer. */
    case IC_CMD_SET_MQTTCFG: {
        uint16_t f = c->set_mqttcfg.fields;
        if (f & IC_MQTT_F_HOST)   { if (config_apply_str("mqtt_host",   c->set_mqttcfg.host)   != CFG_APPLY_OK) LOG_WARN("cli", "ignored over-long mqtt host from web/API"); }
        if (f & IC_MQTT_F_PORT)   { if (config_apply_u32("mqtt_port", c->set_mqttcfg.port)      != CFG_APPLY_OK) LOG_WARN("cli", "ignored out-of-range mqtt port from web/API"); }
        if (f & IC_MQTT_F_USER)   { if (config_apply_str("mqtt_user",   c->set_mqttcfg.user)   != CFG_APPLY_OK) LOG_WARN("cli", "ignored over-long mqtt user from web/API"); }
        if (f & IC_MQTT_F_PASS)   { if (config_apply_str("mqtt_pass",   c->set_mqttcfg.pass)   != CFG_APPLY_OK) LOG_WARN("cli", "ignored over-long mqtt pass from web/API"); }
        if (f & IC_MQTT_F_PREFIX) { if (config_apply_str("mqtt_prefix", c->set_mqttcfg.prefix) != CFG_APPLY_OK) LOG_WARN("cli", "ignored over-long mqtt prefix from web/API"); }
        if (f && !config_save()) LOG_ERR("cli", "config_save failed — mqttcfg not persisted");
        break;
    }
    case IC_CMD_SET_AUTH:
        /* THE core-0-side hashing setter (web_auth.h's header comment lists
         * every call site). c->set_auth.pass carries the plaintext candidate
         * across the intercore queue — transient RAM, never stored — this is
         * where it gets turned into a fresh salt + Argon2id hash and the
         * plaintext is wiped.
         *
         * Both or neither, via temporaries: a half-applied credential change
         * (new user kept, new hash refused, or vice versa) would lock the
         * operator out just as effectively as a truncated one, and writing
         * straight into g_config would risk exactly that — config_apply_str()
         * commits each field the instant it's called. Length-validate into
         * temporaries first (str_set(), same as before); only once user fits
         * AND the hash is computed does anything get written, and each write
         * goes through config_apply_str() so the actual commit shares the
         * same code path (and CFG_CRED/CFG_HEX's own validation) every other
         * config write uses. None of the three config_apply_str() calls below
         * can fail in practice — user is already length-checked, and
         * salt_hex/hash_hex are hex this function just encoded itself — but
         * each is still checked rather than assumed, same as before. */
        { char u[sizeof g_config.web_user], p[sizeof c->set_auth.pass];
          if (!str_set(u, sizeof u, c->set_auth.user) ||
              !str_set(p, sizeof p, c->set_auth.pass)) {
              LOG_WARN("cli", "ignored over-long web credential from web/API");
              break;
          }
          uint8_t salt[WEB_AUTH_SALT_LEN], hash[WEB_AUTH_HASH_LEN];
          web_auth_gen_salt(salt);
          /* Bounded retry against the shared KDF work area (web_auth.c's
           * header comment has the full concurrency argument): the only
           * realistic contender this instant is core 1's OWN /api/auth
           * handler hashing this exact same password change for its live
           * in-RAM credential — a collision self-resolves within one Argon2
           * call (tens to a few hundred ms), so 2 s is generous, not a real
           * stall, and this runs in the main loop, not a hot per-tick path. */
          if (!web_auth_hash(p, salt, hash, 2000)) {
              web_auth_wipe(p, sizeof p);
              LOG_ERR("cli", "web credential hash failed (KDF busy) — auth not changed");
              break;
          }
          char salt_hex[33], hash_hex[65];
          web_auth_hex_encode(salt, sizeof salt, salt_hex);
          web_auth_hex_encode(hash, sizeof hash, hash_hex);
          web_auth_wipe(p, sizeof p);
          web_auth_wipe(hash, sizeof hash);
          if (config_apply_str("web_user", u) != CFG_APPLY_OK ||
              config_apply_str("web_salt", salt_hex) != CFG_APPLY_OK ||
              config_apply_str("web_pass_hash", hash_hex) != CFG_APPLY_OK) {
              LOG_WARN("cli", "web credential apply failed unexpectedly");
              break;
          } }
        if (!config_save()) LOG_ERR("cli", "config_save failed — web auth not persisted");
        break;
    case IC_CMD_SET_MAPPING: {
        uint8_t count = c->set_mapping.count;
        if (count > MAX_TARGETS_PER_BTN) count = MAX_TARGETS_PER_BTN;
        if (mapping_set(c->set_mapping.fixture, c->set_mapping.button,
                        c->set_mapping.targets, count))
            emit_config_mapping(c->set_mapping.fixture, c->set_mapping.button,
                                c->set_mapping.targets, count);
        else
            LOG_ERR("cli", "mapping_set failed F%u.B%u — binding not saved",
                    c->set_mapping.fixture, c->set_mapping.button);
        break;
    }
    case IC_CMD_DEL_MAPPING:
        if (mapping_delete(c->del_mapping.fixture, c->del_mapping.button))
            emit_config_mapping(c->del_mapping.fixture, c->del_mapping.button, NULL, 0);
        else
            LOG_ERR("cli", "mapping_delete failed F%u.B%u",
                    c->del_mapping.fixture, c->del_mapping.button);
        break;
    case IC_CMD_PROVISION_START: {
        bool valid = false;
        for (int i = 0; i < g_config.dali_slave_count; i++)
            if (g_config.dali_slave_ids[i] == c->provision_start.slave) valid = true;
        if (!valid || dali_provision_busy()) {
            LOG_WARN("cli", "ic provision_start reject: slave=%u valid=%d busy=%d",
                     c->provision_start.slave, (int)valid, (int)dali_provision_busy());
            break;
        }
        prov_mode_t mode = c->provision_start.mode == IC_PROV_UNADDRESSED ? PROV_UNADDRESSED
                         : c->provision_start.mode == IC_PROV_ADDRESS     ? PROV_ADDRESS
                                                                          : PROV_ALL;
        if (mode == PROV_ADDRESS && c->provision_start.target > 63) {
            LOG_WARN("cli", "ic provision_start reject: target=%u", c->provision_start.target);
            break;
        }
        dali_provision_start(c->provision_start.slave, mode, c->provision_start.target);
        break;
    }
    case IC_CMD_PROVISION_ABORT:
        dali_provision_abort();
        break;
    case IC_CMD_RESYNC:
        serial_tx_hello();
        emit_state_cfg();          /* core 1 builds the controller config device */
        lighting_resync_emit();
        serial_tx_config_full();
        break;
    case IC_CMD_PING_REPLY:
        cli_ping_on_reply(c);      /* print one 'net ping' echo outcome */
        break;
    case IC_CMD_REBOOT:
        controller_reboot("remote");   /* never returns */
        break;
    case IC_CMD_OTA_BEGIN:
        ota_stage_begin();
        break;
    case IC_CMD_OTA_DATA: {
        ic_event_t evt = { 0 };
        /* Clamp against g_ota_buf's size before the flash write: len is a
         * uint16_t (<=65535) but the shared buffer is only OTA_BUF_SZ; a corrupt
         * or future-buggy core-1 len must never make ota_stage_write read past
         * the buffer into adjacent memory. Over-length is treated as a fault. */
        if (c->ota_data.len <= OTA_BUF_SZ && ota_stage_write(g_ota_buf, c->ota_data.len)) {
            evt.type = IC_EVT_OTA_ACK;
        } else {
            evt.type = IC_EVT_OTA_STATUS;
            evt.ota_status.code = OTA_FLASH_FAIL;
            LOG_ERR("ota", "ota_stage_write failed (len=%u)", c->ota_data.len);
        }
        intercore_push_event(&evt);
        break;
    }
    case IC_CMD_OTA_END: {
        bool ok = ota_stage_finish(c->ota_end.total_len);
        ic_event_t evt = { .type = IC_EVT_OTA_STATUS };
        evt.ota_status.code = ok ? OTA_OK : OTA_VERIFY_FAIL;
        if (!ok) LOG_ERR("ota", "ota_stage_finish failed (total_len=%lu)",
                          (unsigned long)c->ota_end.total_len);
        intercore_push_event(&evt);
        break;
    }
    case IC_CMD_OTA_ABORT:
        ota_stage_abort();
        break;
    case IC_CMD_OTA_CONFIRM:
        ota_stage_confirm();
        break;
    }
}

/* ── Public interface ────────────────────────────────────────────────────── */

/* Boot/ready announce — a human text log (no JSON on USB). core 1 derives the
 * slave lists / fw_version from its config snapshot, not from a hello event. */
void serial_tx_hello(void) {
    LOG_INFO("serial", "hello fw=" FW_VERSION " baud=%lu relays=%u dali=%u",
             (unsigned long)modbus_get_baud(),
             g_config.relay_slave_count, g_config.dali_slave_count);
}

void serial_tx_config_full(void) {
    /* Emit every populated binding as its own config_mapping line, incrementally
     * — NOT serialised into one fixed buffer. A single 4 KB serialise buffer
     * (the old mapping_serialise_all, since removed) overflowed once the table
     * grew past ~45 bindings and printed nothing, so get_mappings answered empty
     * and HA showed no bindings after every reconnect, even though the
     * controller's table was intact. */
    for (uint8_t f = 0; f < MAX_FIXTURES; f++)
        for (uint8_t b = 0; b < MAX_BUTTONS; b++) {
            const mapping_record_t *rec = mapping_lookup(f, b);
            if (rec) emit_config_mapping(f, b, rec->targets, rec->target_count);
        }
}

/* Drains inbound intercore commands and outbound lighting events, plus three
 * small pollers that used to share serial_poll()'s loop body one-for-one with
 * these two drains and must keep running in the SAME relative order (all of
 * this is a pure code move, not a behaviour change):
 *
 *   - serial_provision_poll() (cli_provision.c) and cdc_out_drain() (cdc_out.c)
 *     have no ordering dependency on the drains below; they are kept in their
 *     original tail position purely to avoid reordering unrelated queue
 *     traffic a main-loop iteration earlier or later than before.
 *   - cli_ping_poll() (cli.c) DOES depend on running after the intercore
 *     command drain within the SAME tick: an IC_CMD_PING_REPLY drained below
 *     updates the running `net ping`'s state via cli_ping_on_reply before
 *     cli_ping_poll's deadline check runs, so a reply that arrived this tick
 *     is accounted for before the timeout can fire on it. Moving the check
 *     into cli_poll() (which runs before this function each iteration) would
 *     make it see last tick's state instead — a real, if small, timing
 *     regression, which is why it lives here and not in cli.c despite being
 *     a CLI-flavoured feature. */
void ic_seam_pump(void) {
    /* Drain inbound commands from core 1 (the network bridge) and dispatch them
     * through the same domain entry points the CLI/JSON use. Empty until core 1
     * is launched. */
    ic_cmd_t cmd;
    while (intercore_pop_cmd(&cmd)) {
        ic_dispatch_cmd(&cmd);
    }

    /* Drain outbound state events and forward each to core 1 (the network
     * bridge) as a typed struct. The event queue is sized for a full resync
     * burst, so this does not drop. USB CDC carries no JSON. */
    levt_t evt;
    while (lighting_evt_try_pop(&evt)) {
        ic_event_t ic;
        if (levt_to_ic(&evt, &ic)) intercore_push_event(&ic);
    }

    /* Push any pending provision status update to core 1. */
    serial_provision_poll();

    /* Time out a network-ping run if core 1 stops replying. */
    cli_ping_poll();

    /* Push queued log output to the USB-CDC, non-blocking (writes only what the
     * TX FIFO can take this instant). Producers on either core enqueue lines into
     * the ring without ever blocking; this is the sole drain. */
    cdc_out_drain();
}
