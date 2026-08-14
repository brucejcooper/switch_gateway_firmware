/*
 * dirty_poll.c — dirty-bit polling for TinyModbus DALI slaves.
 *
 * Fixes applied:
 *   #1  Removed dead blocking_read_input_regs() with C++ lambda syntax
 *   #5  Correct Modbus response frame size checks (include CRC bytes)
 *   #7  dirty_poll_tick() skips slaves being provisioned
 *   #8  Deadtime is baud-rate-correct (was hardcoded for 9600)
 */

#include "dirty_poll.h"
#include "dali_state.h"
#include "dali_queue.h"
#include "mbq_client.h"
#include "modbus.h"
#include "state_cache.h"
#include "lighting_actions.h"
#include "config.h"
#include "dali_provision.h"
#include "log.h"
#include "pico/time.h"
#include <string.h>
#include <stdio.h>

typedef enum {
    DPOLL_IDLE,
    DPOLL_AWAITING_DIRTY,
    DPOLL_AWAITING_BALLAST,
} dpoll_state_t;

/* Consecutive failed reads before a gateway is declared unreachable. */
#define GATEWAY_FAIL_THRESHOLD 3u

typedef struct {
    dpoll_state_t state;
    bool          active;
    uint32_t      last_activity_ms;  /* last command or detected change; drives the active window */
    uint32_t      last_poll_ms;
    uint32_t      last_sweep_ms;     /* last idle dirty-mask sweep */
    uint32_t      last_bg_ms;
    uint64_t      last_present;
    uint64_t      pending_dirty;
    uint8_t       next_ballast;
    bool          reachable;     /* gateway responding to Modbus */
    uint8_t       fail_count;    /* consecutive read failures */
    uint32_t      last_cfg_ms;   /* last steady-state config-rotation mark */
    uint8_t       cfg_cursor;    /* round-robin ballast for config re-reads */
    bool          scanning;      /* a full interrogation is in progress */
} dpoll_t;

static dpoll_t s_dp[MAX_DALI_SLAVES];

typedef struct { uint8_t slave_id; uint8_t addr; } ballast_ctx_t;
static ballast_ctx_t s_bctx[MAX_DALI_SLAVES][DALI_ADDR_COUNT];
static uint8_t s_dirty_ctx_slave[MAX_DALI_SLAVES];

/* Relay-board reachability: poll each relay slave's coils periodically and emit
 * state_gateway on a transition, exactly like a DALI gateway, so HA marks relay
 * switches unavailable when the board is absent (not just on the system topic). */
#define RELAY_POLL_INTERVAL_MS 2000u
typedef struct { bool reachable; uint8_t fail_count; uint32_t last_poll_ms; bool awaiting; } rpoll_t;
static rpoll_t s_rp[MAX_RELAY_SLAVES];
static uint8_t s_rp_idx[MAX_RELAY_SLAVES];   /* callback ctx → relay-slave index */

/* Gateway sysinfo (FC04 0x0310: uptime / boot count / reset reason). Polled at
 * g_config.gw_diag_interval_ms and published to HA as per-gateway diagnostic
 * sensors. Piggybacks on the per-gateway DALI loop below (no separate poller)
 * and fires independently of the dirty-poll state machine, like the relay poll. */
typedef struct { uint32_t last_poll_ms; bool awaiting; } gwdiag_t;
static gwdiag_t s_gd[MAX_DALI_SLAVES];
static uint8_t  s_gd_idx[MAX_DALI_SLAVES];   /* callback ctx → dali-slave index */

static void relay_health_cb(int res, uint8_t *buf, void *ctx);   /* defined below */

/* Offered to the bus when it falls idle and it is this relay board's turn.
 * A cheap 32-coil read every RELAY_POLL_INTERVAL_MS; relay_health_cb flips
 * state_gateway when the board appears or disappears. */
static bool relay_offer(void *ctx) {
    int i = (int)(intptr_t)ctx;
    if (i >= g_config.relay_slave_count) return false;
    rpoll_t *rp = &s_rp[i];
    if (rp->awaiting) return false;
    uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    if (now_ms - rp->last_poll_ms < RELAY_POLL_INTERVAL_MS) return false;
    rp->last_poll_ms = now_ms;
    /* Only mark awaiting if the read actually queued — otherwise a dropped
     * action would leave awaiting stuck and we'd never poll this relay again.
     * On a drop we just retry next interval. */
    if (!modbus_get_coils(g_config.relay_slave_ids[i], 0, 32,
                          relay_health_cb, &s_rp_idx[i]))
        return false;
    rp->awaiting = true;
    return true;
}

void dirty_poll_init(void) {
    memset(s_dp, 0, sizeof(s_dp));
    for (int i = 0; i < MAX_DALI_SLAVES; i++) {
        /* UNREACHABLE until it answers. Starting true asserted that a gateway we
         * have never heard from is online, which HA then believed — and because
         * ballast entities declare availability_mode "all" over
         * [controller, gateway, ballast], one optimistic gateway made every
         * ballast under it look available too. Nothing gates polling on this
         * flag, so starting false cannot stall discovery; the first successful
         * read flips it. */
        s_dp[i].reachable = false;
        s_dirty_ctx_slave[i] = g_config.dali_slave_ids[i];
        for (int a = 0; a < DALI_ADDR_COUNT; a++) {
            s_bctx[i][a].slave_id = g_config.dali_slave_ids[i];
            s_bctx[i][a].addr     = (uint8_t)a;
        }
    }
    memset(s_rp, 0, sizeof(s_rp));
    /* Join the bus rotation, one turn per relay board. Registered here, before
     * the DALI gateways register in mbq_client_add_slave, so the rotation runs
     * relay -> gateway -> gateway. */
    for (int i = 0; i < g_config.relay_slave_count; i++)
        modbus_register_offer(relay_offer, (void *)(intptr_t)i);
    for (int i = 0; i < MAX_RELAY_SLAVES; i++) {
        s_rp[i].reachable = false;   /* same rule as DALI gateways above */
        s_rp_idx[i]       = (uint8_t)i;
    }
    memset(s_gd, 0, sizeof(s_gd));
    for (int i = 0; i < MAX_DALI_SLAVES; i++) s_gd_idx[i] = (uint8_t)i;
}

/* Relay coil-read result → reachability, debounced like gateway_report().
 * Also reconciles the relay on/off cache from the actual board state: the read
 * is authoritative, so a relay changed out-of-band (or a cache left wrong by a
 * board quirk / missed write) self-heals here, and toggling g_config.relay_byteswap
 * takes effect within one poll interval without a reboot. Only the first relay
 * slave drives the channel cache (RELAY_COUNT channels live on relay_slaves[0]). */
static void relay_health_cb(int res, uint8_t *buf, void *ctx) {
    uint8_t  i     = *(uint8_t *)ctx;
    rpoll_t *rp    = &s_rp[i];
    uint8_t  slave = g_config.relay_slave_ids[i];
    rp->awaiting   = false;
    if (res >= 7) {            /* a valid FC01 coil response */
        rp->fail_count = 0;
        if (!rp->reachable) {
            rp->reachable = true;
            LOG_INFO("dirty", "relay board %u reachable", slave);
            lighting_emit_gateway_state(slave, true);
        }
        /* buf[2] = byte count, buf[3..] = coil bytes (one bit per channel). */
        if (i == 0) {
            uint8_t bc = buf[2];
            for (uint8_t ch = 0; ch < RELAY_COUNT; ch++) {
                bool on = lighting_relay_coil_on(&buf[3], bc, ch);
                if (on != state_cache_get_relay_on(ch)) {
                    state_cache_set_relay(ch, on);
                    lighting_emit_relay_state(ch, on);
                }
            }
        }
    } else if (rp->reachable) {
        if (rp->fail_count < 0xFF) rp->fail_count++;
        if (rp->fail_count >= GATEWAY_FAIL_THRESHOLD) {
            rp->reachable = false;
            LOG_WARN("dirty", "relay board %u unreachable (%u consecutive read failures)",
                     slave, GATEWAY_FAIL_THRESHOLD);
            lighting_emit_gateway_state(slave, false);
        }
    }
}

void dirty_poll_emit_gateway_states(void) {
    /* Replay current reachability for resync. s_rp/s_dp are indexed by config
     * order, matching the relay-poll and dirty-poll loops below. */
    for (int i = 0; i < g_config.relay_slave_count; i++)
        lighting_emit_gateway_state(g_config.relay_slave_ids[i], s_rp[i].reachable);
    for (int i = 0; i < g_config.dali_slave_count; i++)
        lighting_emit_gateway_state(g_config.dali_slave_ids[i], s_dp[i].reachable);
}

void dirty_poll_mark_active(uint8_t slave_id) {
    int idx = state_cache_slave_idx(slave_id);
    if (idx < 0) return;
    s_dp[idx].active           = true;
    s_dp[idx].last_activity_ms = to_ms_since_boot(get_absolute_time());
}


/* Track a gateway's Modbus reachability and emit state_gateway on transition.
 * `ok` = this read of the gateway succeeded. Debounced by GATEWAY_FAIL_THRESHOLD
 * so a single timeout doesn't flap availability. */
static void gateway_report(uint8_t slave_id, int idx, bool ok) {
    dpoll_t *dp = &s_dp[idx];
    if (ok) {
        dp->fail_count = 0;
        if (!dp->reachable) {
            dp->reachable = true;
            LOG_INFO("dirty", "gateway %u reachable again", slave_id);
            lighting_emit_gateway_state(slave_id, true);
        }
    } else if (dp->reachable) {
        if (dp->fail_count < 0xFF) dp->fail_count++;
        if (dp->fail_count >= GATEWAY_FAIL_THRESHOLD) {
            dp->reachable = false;
            dp->active    = false;   /* stop fast-polling a dead gateway; the
                                        60s background check still probes it and
                                        recovers it (and a command re-activates) */
            LOG_WARN("dirty", "gateway %u unreachable (%u consecutive read failures)",
                     slave_id, GATEWAY_FAIL_THRESHOLD);
            lighting_emit_gateway_state(slave_id, false);
        }
    }
}

/* ── Gateway sysinfo (FC04 0x0310) callback ──────────────────────────────── */
/* Decodes uptime / boot count / reset reason and emits a gw_diag event. A
 * failed read is ignored here (the dirty sweep owns reachability); a good read
 * also reinforces reachability via gateway_report(). */
static void gwdiag_cb(int res, uint8_t *buf, void *ctx) {
    uint8_t i     = *(uint8_t *)ctx;
    s_gd[i].awaiting = false;
    uint8_t slave = g_config.dali_slave_ids[i];

    /* FC04 4-register frame = slave+FC+bytecount+8data+CRC = 13 bytes. */
    if (res < 13) return;

    uint8_t *p = buf + 3;                       /* big-endian, one word per reg */
    uint32_t uptime = ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
                      ((uint32_t)p[2] << 8)  |  (uint32_t)p[3];
    uint16_t boot   = (uint16_t)(((uint16_t)p[4] << 8) | p[5]);
    uint8_t  reason = p[7];                     /* reg3 low byte = RSTCTRL.RSTFR */

    int idx = state_cache_slave_idx(slave);
    if (idx >= 0) gateway_report(slave, idx, true);
    lighting_emit_gw_diag(slave, uptime, boot, reason);
    /* boot_count / uptime / reset_reason are a pure HA diagnostic here (queue v2
     * detects a gateway reboot from the slot statuses in mbq_client, not from
     * boot_count) — nothing to feed to the command queue. */
}



/* ── Tick ────────────────────────────────────────────────────────────────── */
/* Arm a full DALI interrogation and return immediately. Every caller uses this —
 * boot, the CLI `scan`, and dcom after a commissioning run. Completion is
 * noticed in dirty_poll_tick(), which emits the scan-complete event, so no
 * caller has to wait for or report the result. */
void dirty_poll_arm_scan(uint8_t slave_id) {
    if (state_cache_slave_idx(slave_id) < 0) return;
    LOG_INFO("dirty", "slave %u full scan armed", slave_id);
    dali_state_request_full_scan(slave_id);
}

void dirty_poll_tick(void) {
    uint32_t now_ms = to_ms_since_boot(get_absolute_time());

    /* Relay reachability polls are no longer issued from here — they are offered
     * to the bus in rotation (relay_offer, registered in dirty_poll_init), so a
     * relay board takes its turn alongside the DALI gateways instead of pushing
     * whenever this tick happens to run. */

    for (int i = 0; i < g_config.dali_slave_count; i++) {
        uint8_t  slave_id = g_config.dali_slave_ids[i];
        dpoll_t *dp       = &s_dp[i];

        /* fix #7: do not poll slaves being provisioned */
        if (dali_provision_busy_slave(slave_id)) continue;

        /* Scan completion. Every scan is asynchronous now — boot, the CLI and
         * post-provisioning all just ARM one — so completion has to be noticed
         * here rather than by whoever started it. Emitting from one place also
         * means all three paths notify HA identically. */
        if (dali_state_scan_busy(slave_id)) {
            dp->scanning = true;
        } else if (dp->scanning) {
            dp->scanning = false;
            uint64_t present = 0;
            for (uint8_t a = 0; a < DALI_ADDR_COUNT; a++) {
                dali_ballast_state_t *b = state_cache_get_dali((uint8_t)i, a);
                if (b && b->present) present |= (uint64_t)1 << a;
            }
            dp->last_present = present;
            LOG_INFO("dirty", "slave %u scan complete", slave_id);
            /* Presence is now known, so read every present ballast's config from
             * the gear straight away instead of waiting out the 14 s-per-ballast
             * rotation. Lowest priority, so it yields to anything real. */
            lighting_cfg_query_mark_mask(slave_id, present);
            lighting_emit_scan_complete(slave_id, present);
        }

        /* Low-frequency gateway sysinfo read (uptime / reboots / reset reason),
         * gated by the configurable interval (0 = disabled). Fires independently
         * of the dirty-poll SM — the modbus action queue serialises it. */
        if (g_config.gw_diag_interval_ms && !s_gd[i].awaiting
            && now_ms - s_gd[i].last_poll_ms >= g_config.gw_diag_interval_ms) {
            s_gd[i].last_poll_ms = now_ms;
            if (modbus_get_input_registers(slave_id, DALI_SYSINFO_START, 4,
                                           gwdiag_cb, &s_gd_idx[i]))
                s_gd[i].awaiting = true;
        }

        /* Steady-state config refresh: rotate one present ballast per
         * DALI_CFG_ROTATE_MS into the lowest-priority raw config sweep so an
         * external recommissioning is picked up within ~15 min. Independent of
         * the dirty-poll SM (like the gw-diag read); the sweep itself only runs
         * when the bus is otherwise idle. */
        if (dp->reachable && dp->last_present &&
            now_ms - dp->last_cfg_ms >= DALI_CFG_ROTATE_MS) {
            dp->last_cfg_ms = now_ms;
            for (int n = 0; n < DALI_ADDR_COUNT; n++) {
                uint8_t a = (uint8_t)((dp->cfg_cursor + n) % DALI_ADDR_COUNT);
                if (dp->last_present & ((uint64_t)1 << a)) {
                    lighting_cfg_query_mark(slave_id, a);
                    dp->cfg_cursor = (uint8_t)((a + 1) % DALI_ADDR_COUNT);
                    break;
                }
            }
        }

        /* The dirty-mask / block0 / background-check reads that would
         * live here are GONE. They read gateway registers (0x0104 dirty, 0x0000
         * block0, 0x0100 present, 0x0110 group masks) that the gateway strip
         * removes. Level, status, presence and groups are now learned by
         * querying the ballasts through the command queue — see dali_state.c,
         * driven from the main loop. What remains in this file is the relay
         * board (a different device entirely) and the gateway sysinfo poll,
         * neither of which the strip touches. */
        /* Gateway reachability now rides the COMMAND QUEUE rather than a
         * dedicated register poll. The queue talks to each gateway continuously
         * (the world-model sweep alone keeps it busy), so consecutive no-reply
         * transactions are both the earliest and the cheapest liveness signal
         * available — far better than the 30 s sysinfo poll, which was the only
         * other candidate once the dirty sweep went away. A Modbus exception
         * does NOT count: that is the gateway alive and disagreeing. */
        gateway_report(slave_id, i,
                       mbq_client_consec_fail(slave_id) < GATEWAY_FAIL_THRESHOLD);
    }
}

/* ── sync_read helper ────────────────────────────────────────────────────── */

typedef struct {
    volatile bool done;
    volatile bool ok;
    uint16_t     *out;
    uint16_t      count;
} sync_ctx_t;




/* ── Group-mask read (used by full_scan and background_check) ────────────── */

/* ── Blocking startup full scan ──────────────────────────────────────────── */



