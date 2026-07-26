/*
 * dirty_poll.c — dirty-bit polling for TinyModbus DALI slaves.
 *
 * Fixes applied:
 *   #1  Removed dead blocking_read_input_regs() with C++ lambda syntax
 *   #5  Correct Modbus response frame size checks (include CRC bytes)
 *   #6  dirty_poll_full_scan() now emits scan_complete via lighting_emit_scan_complete()
 *   #7  dirty_poll_tick() skips slaves being provisioned
 *   #8  Deadtime is baud-rate-correct (was hardcoded for 9600)
 */

#include "dirty_poll.h"
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
    uint32_t      last_cfg_ms;   /* last steady-state config-rotation mark (task 03) */
    uint8_t       cfg_cursor;    /* round-robin ballast for config re-reads */
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

void dirty_poll_init(void) {
    memset(s_dp, 0, sizeof(s_dp));
    for (int i = 0; i < MAX_DALI_SLAVES; i++) {
        s_dp[i].reachable = true;   /* optimistic; offline only after failures */
        s_dirty_ctx_slave[i] = g_config.dali_slave_ids[i];
        for (int a = 0; a < DALI_ADDR_COUNT; a++) {
            s_bctx[i][a].slave_id = g_config.dali_slave_ids[i];
            s_bctx[i][a].addr     = (uint8_t)a;
        }
    }
    memset(s_rp, 0, sizeof(s_rp));
    for (int i = 0; i < MAX_RELAY_SLAVES; i++) {
        s_rp[i].reachable = true;   /* optimistic, same as DALI gateways */
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
            LOG_INFO("dirty", "relay board %u reachable again", slave);
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

static void ballast_read_cb(int res, uint8_t *buf, void *ctx);   /* forward decl */

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
}

/* ── Dirty bitmask callback ──────────────────────────────────────────────── */
static void dirty_bitmask_cb(int res, uint8_t *buf, void *ctx) {
    uint8_t slave_id = *(uint8_t *)ctx;
    int     idx      = state_cache_slave_idx(slave_id);
    if (idx < 0) return;

    dpoll_t *dp     = &s_dp[idx];
    uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    dp->state       = DPOLL_IDLE;

    /* fix #5: FC04 4-register frame = slave+FC+bytecount+8data+CRC = 13 bytes */
    if (res < 13) {
        LOG_WARN("dirty", "bitmask read failed slave %u (%d)", slave_id, res);
        gateway_report(slave_id, idx, false);
        return;
    }
    gateway_report(slave_id, idx, true);

    uint8_t *p = buf + 3;
    uint64_t mask = ((uint64_t)((p[0]<<8)|p[1]))       |
                    ((uint64_t)((p[2]<<8)|p[3]) << 16)  |
                    ((uint64_t)((p[4]<<8)|p[5]) << 32)  |
                    ((uint64_t)((p[6]<<8)|p[7]) << 48);

    if (mask != 0) {
        LOG_DBG("dirty", "slave=%u dirty_mask=0x%016llx",
                 slave_id, (unsigned long long)mask);
    }

    if (mask == 0) {
        /* Deactivate only once the active window has fully elapsed since the last
         * command or detected change — not after a couple of quick zero reads.
         * On/fade/multi-target dirty bits can land hundreds of ms after the
         * command, well past the old 2-zero (~200ms) cutoff. */
        if (now_ms - dp->last_activity_ms >= DIRTY_ACTIVE_WINDOW_MS) {
            dp->active = false;
        }
    } else {
        dp->active           = true;   /* a sweep that finds a change re-activates fast polling */
        dp->last_activity_ms = now_ms;
        dp->pending_dirty    = mask;
        for (int a = 0; a < DALI_ADDR_COUNT; a++) {
            if (mask & ((uint64_t)1 << a)) {
                LOG_DBG("dirty",
                         "slave=%u: ballast %d flagged dirty — reading BLOCK0 holding reg %u for fresh arc/status",
                         slave_id, a, (unsigned)(DALI_BLOCK0_START + a));
                /* BLOCK0 is now an FC03 holding-register bank (post refactor).
                 * Only advance if queued (else stay IDLE and re-poll next tick). */
                if (modbus_get_holding_registers(slave_id,
                                                 DALI_BLOCK0_START + (uint16_t)a, 1,
                                                 ballast_read_cb, &s_bctx[idx][a]))
                    dp->state = DPOLL_AWAITING_BALLAST;
                break;
            }
        }
    }
}

/* ── Per-ballast block-0 read callback ──────────────────────────────────── */
static void ballast_read_cb(int res, uint8_t *buf, void *ctx) {
    ballast_ctx_t *bc    = (ballast_ctx_t *)ctx;
    uint8_t        slave = bc->slave_id;
    uint8_t        addr  = bc->addr;
    int            idx   = state_cache_slave_idx(slave);
    if (idx < 0) return;

    dpoll_t *dp = &s_dp[idx];
    dp->state   = DPOLL_IDLE;

    /* fix #5: FC04 1-register frame = slave+FC+bytecount+2data+CRC = 7 bytes */
    if (res >= 7) {
        uint8_t status = buf[3];
        uint8_t arc    = buf[4];
        if (arc == 0xFFu) {
            /* 0xFF is not a valid DALI level (max is 254) — a corrupt or
             * unavailable read. Drop it rather than reporting 100%; the cached
             * (last good) level stands until a valid read arrives. */
            LOG_WARN("dirty", "ballast %u.%u read arc=0xFF (invalid) — ignoring",
                     slave, addr);
        } else {
            /* Only emit when something reportable actually changed. The gateway
             * re-polls a fading ballast continuously (correct — until FADE_RUNNING
             * clears), so without this we'd spam an identical state_dali every
             * ~100 ms for the whole fade. We still update the cache and keep
             * polling (to catch the level ramp and the fade end) — we just don't
             * re-report no-ops. */
            dali_ballast_state_t *b =
                (idx >= 0) ? state_cache_get_dali((uint8_t)idx, addr) : NULL;
            bool report = (b == NULL)
                || b->arc_level != arc
                || ((b->status_byte ^ status) & 0x12u);  /* 0x10 fading | 0x02 lamp_fail */
            if (report) {
                lighting_emit_dali_state(slave, addr, arc, status);
                LOG_DBG("dirty", "ballast %u.%u arc=%u status=0x%02x",
                        slave, addr, arc, status);
                if ((status & 0x02) || (status & 0x80))
                    lighting_emit_fault(slave, addr,
                                        (status & 0x02) != 0, (status & 0x80) != 0);
            } else if (idx >= 0) {
                state_cache_set_dali_arc((uint8_t)idx, addr, arc, status);
            }
        }
    }

    dp->pending_dirty &= ~((uint64_t)1 << addr);
    for (int a = addr + 1; a < DALI_ADDR_COUNT; a++) {
        if (dp->pending_dirty & ((uint64_t)1 << a)) {
            LOG_DBG("dirty",
                     "slave=%u: ballast %d also dirty — reading next BLOCK0 holding reg %u",
                     slave, a, (unsigned)(DALI_BLOCK0_START + a));
            /* BLOCK0 is now an FC03 holding-register bank (post refactor).
             * Only advance if queued; otherwise stay IDLE so the next dirty-mask
             * poll re-detects and retries (vs. stranding state at AWAITING). */
            if (modbus_get_holding_registers(slave,
                                             DALI_BLOCK0_START + (uint16_t)a, 1,
                                             ballast_read_cb, &s_bctx[idx][a]))
                dp->state = DPOLL_AWAITING_BALLAST;
            return;
        }
    }
}

/* ── Tick ────────────────────────────────────────────────────────────────── */
void dirty_poll_tick(void) {
    uint32_t now_ms = to_ms_since_boot(get_absolute_time());

    /* Relay-board reachability: a cheap coil read every RELAY_POLL_INTERVAL_MS.
     * relay_health_cb flips state_gateway when the board appears/disappears. */
    for (int i = 0; i < g_config.relay_slave_count; i++) {
        rpoll_t *rp = &s_rp[i];
        if (rp->awaiting) continue;
        if (now_ms - rp->last_poll_ms >= RELAY_POLL_INTERVAL_MS) {
            rp->last_poll_ms = now_ms;
            /* Only mark awaiting if the read actually queued — otherwise a dropped
             * action (full queue) would leave awaiting stuck and we'd never poll
             * this relay again. On a drop we just retry next interval. */
            if (modbus_get_coils(g_config.relay_slave_ids[i], 0, 32,
                                 relay_health_cb, &s_rp_idx[i]))
                rp->awaiting = true;
        }
    }

    for (int i = 0; i < g_config.dali_slave_count; i++) {
        uint8_t  slave_id = g_config.dali_slave_ids[i];
        dpoll_t *dp       = &s_dp[i];

        /* fix #7: do not poll slaves being provisioned */
        if (dali_provision_busy_slave(slave_id)) continue;

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

        if (dp->active && dp->state == DPOLL_IDLE) {
            if (now_ms - dp->last_poll_ms >= DIRTY_POLL_INTERVAL_MS) {
                dp->last_poll_ms = now_ms;
                /* Advance state only if the read queued; a dropped action never
                 * fires its callback, which would strand state at AWAITING_* and
                 * permanently stop polling this slave. Stay IDLE and retry. */
                if (modbus_get_input_registers(slave_id, DALI_DIRTY_START, 4,
                                               dirty_bitmask_cb,
                                               &s_dirty_ctx_slave[i]))
                    dp->state = DPOLL_AWAITING_DIRTY;
            }
        }

        if (!dp->active && dp->state == DPOLL_IDLE) {
            if (now_ms - dp->last_bg_ms >= BACKGROUND_CHECK_MS) {
                dp->last_bg_ms = now_ms;
                dirty_poll_background_check(slave_id);
            } else if (dp->reachable
                       && now_ms - dp->last_sweep_ms >= DIRTY_SWEEP_MS) {
                /* Regular cheap dirty-mask sweep so external / manual / post-fade
                 * changes reflect within ~1s even when no command was issued. A
                 * non-zero mask re-activates fast polling (see dirty_bitmask_cb). */
                dp->last_sweep_ms = now_ms;
                if (modbus_get_input_registers(slave_id, DALI_DIRTY_START, 4,
                                               dirty_bitmask_cb,
                                               &s_dirty_ctx_slave[i]))
                    dp->state = DPOLL_AWAITING_DIRTY;
            }
        }
    }
}

/* ── sync_read helper ────────────────────────────────────────────────────── */

typedef struct {
    volatile bool done;
    volatile bool ok;
    uint16_t     *out;
    uint16_t      count;
} sync_ctx_t;

static void sync_read_cb(int res, uint8_t *buf, void *ctx) {
    sync_ctx_t *c = (sync_ctx_t *)ctx;
    /* fix #5: full frame = slave+FC+bytecount+data+CRC = 5 + count*2 bytes */
    if (res >= (int)(5 + c->count * 2)) {
        uint8_t *p = buf + 3;
        for (uint16_t i = 0; i < c->count; i++) {
            c->out[i] = ((uint16_t)p[0] << 8) | p[1];
            p += 2;
        }
        c->ok = true;
    }
    c->done = true;
}

static bool sync_read(uint8_t slave, uint16_t addr, uint16_t count,
                       uint16_t *out) {
    static sync_ctx_t ctx;
    ctx.done  = false;
    ctx.ok    = false;
    ctx.out   = out;
    ctx.count = count;
    modbus_get_input_registers(slave, addr, count, sync_read_cb, &ctx);
    absolute_time_t deadline = make_timeout_time_ms(1000);
    while (!ctx.done && absolute_time_diff_us(get_absolute_time(), deadline) > 0)
        modbus_poll();
    return ctx.ok;
}

/* FC03 holding-register variant — used for the per-ballast banks (BLOCK0..3)
 * which moved from FC04 input regs to FC03 holding regs in the refactor. */
static bool sync_read_holding(uint8_t slave, uint16_t addr, uint16_t count,
                              uint16_t *out) {
    static sync_ctx_t ctx;
    ctx.done  = false;
    ctx.ok    = false;
    ctx.out   = out;
    ctx.count = count;
    modbus_get_holding_registers(slave, addr, count, sync_read_cb, &ctx);
    absolute_time_t deadline = make_timeout_time_ms(1000);
    while (!ctx.done && absolute_time_diff_us(get_absolute_time(), deadline) > 0)
        modbus_poll();
    return ctx.ok;
}

/* ── Group-mask read (used by full_scan and background_check) ────────────── */
/* Group g lives at FC03 holding regs DALI_GROUP_MASK_START + 4*g .. +4*g+3.
 * TinyModbus stores each group's 8-byte member bitmap as group_mask[g][0..7]
 * where byte 0 covers ballasts 0-7 (LSB=ballast 0), byte 1 ballasts 8-15, …,
 * byte 7 ballasts 56-63 (see TinyModbus/src/registers.c:376-387).
 * The register read path packs each pair as (group_mask[g][2r] << 8) |
 * group_mask[g][2r+1] (registers.c:159-163). So in the 16-bit register value
 * received here, the HIGH byte holds the lower-numbered ballasts and the LOW
 * byte holds the higher-numbered ones. Byte-swap each register so that bit N
 * in the assembled 64-bit mask corresponds to ballast short address N. */
static void read_all_group_masks(uint8_t slave_id, uint8_t idx) {
    uint16_t regs[64];  /* 16 groups * 4 regs */
    LOG_DBG("dirty",
            "slave=%u: reading 16 DALI group membership masks (holding regs %u..%u) to refresh group state",
            slave_id, (unsigned)DALI_GROUP_MASK_START,
            (unsigned)(DALI_GROUP_MASK_START + 64 - 1));
    for (uint16_t chunk = 0; chunk < 4; chunk++) {
        uint16_t off = chunk * DALI_MAX_CHUNK;  /* 4 groups per 16-reg chunk */
        if (!sync_read_holding(slave_id,
                                DALI_GROUP_MASK_START + off,
                                DALI_MAX_CHUNK, regs + off)) {
            LOG_WARN("dirty", "group mask chunk %u failed slave %u",
                     chunk, slave_id);
            return;
        }
    }
    for (uint8_t g = 0; g < 16; g++) {
        uint16_t *r = regs + 4 * g;
        uint64_t mask = 0;
        for (int word = 0; word < 4; word++) {
            uint16_t v = r[word];
            /* Swap bytes: high byte = lower-numbered ballasts (bits N..N+7),
             * low byte = higher-numbered ballasts (bits N+8..N+15). */
            uint16_t le = (uint16_t)((v >> 8) | ((v & 0xFFu) << 8));
            mask |= (uint64_t)le << (word * 16);
        }
        state_cache_set_dali_group(idx, g, mask);
    }
}

/* ── Blocking startup full scan ──────────────────────────────────────────── */

void dirty_poll_full_scan(uint8_t slave_id) {
    int idx = state_cache_slave_idx(slave_id);
    if (idx < 0) return;

    LOG_DBG("dirty", "Full scan: slave %u", slave_id);

    /* Trigger firmware scan */
    {
        static sync_ctx_t ctx;
        static uint16_t   dummy[1];
        ctx = (sync_ctx_t){ false, false, dummy, 1 };
        modbus_set_holding_register(slave_id, DALI_SCAN_TRIGGER_REG,
                                    0x0001, sync_read_cb, &ctx);
        absolute_time_t dl = make_timeout_time_ms(500);
        while (!ctx.done && absolute_time_diff_us(get_absolute_time(), dl) > 0)
            modbus_poll();
    }
    sleep_ms(200);

    /* Present bitmask — also a gateway reachability signal. */
    uint16_t present_regs[4] = {0};
    gateway_report(slave_id, idx,
                   sync_read(slave_id, DALI_PRESENT_START, 4, present_regs));
    uint64_t present_mask =
        ((uint64_t)present_regs[0])       |
        ((uint64_t)present_regs[1] << 16) |
        ((uint64_t)present_regs[2] << 32) |
        ((uint64_t)present_regs[3] << 48);
    s_dp[idx].last_present = present_mask;

    /* Block 0 in 16-register chunks (FC03 holding regs post refactor). */
    LOG_DBG("dirty",
             "slave=%u: full scan reading BLOCK0 arc/status for all %d ballasts (holding regs %u..%u)",
             slave_id, DALI_ADDR_COUNT, (unsigned)DALI_BLOCK0_START,
             (unsigned)(DALI_BLOCK0_START + DALI_ADDR_COUNT - 1));
    uint16_t regs[DALI_ADDR_COUNT];
    bool b0_chunk_ok[4];
    for (uint16_t chunk = 0; chunk < 4; chunk++) {
        uint16_t off = chunk * DALI_MAX_CHUNK;
        b0_chunk_ok[chunk] =
            sync_read_holding(slave_id, DALI_BLOCK0_START + off, DALI_MAX_CHUNK, regs + off);
        if (!b0_chunk_ok[chunk])
            LOG_WARN("dirty", "block0 chunk %u failed", chunk);
    }
    for (int a = 0; a < DALI_ADDR_COUNT; a++) {
        bool present = (present_mask & ((uint64_t)1 << a)) != 0;
        state_cache_set_dali_present((uint8_t)idx, (uint8_t)a, present);
        if (!present) continue;
        if (!b0_chunk_ok[a / DALI_MAX_CHUNK]) continue;   /* don't cache uninit/garbage arc */
        state_cache_set_dali_arc((uint8_t)idx, (uint8_t)a,
                                  regs[a] & 0xFF,
                                  (regs[a] >> 8) & 0xFF);
    }

    /* Per-ballast config (min/max/power-on/sys-fail/fade) is owned over raw
     * DALI: don't read any gateway bank (they no longer exist — the gateway
     * returns ILLEGAL DATA ADDRESS at 0x0040-0x00FF). Queue every present
     * ballast for the background raw-DALI config sweep (lighting_cfg_query_*);
     * the cache fills over the next few seconds and each ballast re-emits
     * state_dali as its config lands. */
    LOG_INFO("dirty",
             "slave=%u: queueing %d present ballasts for background raw config sweep",
             slave_id, DALI_ADDR_COUNT);
    lighting_cfg_query_mark_mask(slave_id, present_mask);

    /* Group membership masks (FC03 0x0110-0x014F). Must be read before the
     * per-ballast emit loop below so that lighting_emit_dali_state's group
     * derivation sees populated membership masks. */
    read_all_group_masks(slave_id, (uint8_t)idx);

    /* Emit per-ballast state events for every present ballast so the bridge
     * gets initial state without having to wait for the first dirty bit.
     * This also drives Layer-2 group-state derivation since membership masks
     * are now loaded. */
    for (int a = 0; a < DALI_ADDR_COUNT; a++) {
        if (!(present_mask & ((uint64_t)1 << a))) continue;
        if (!b0_chunk_ok[a / DALI_MAX_CHUNK]) continue;   /* don't emit uninit/garbage arc */
        uint8_t arc    = regs[a] & 0xFF;
        uint8_t status = (regs[a] >> 8) & 0xFF;
        lighting_emit_dali_state(slave_id, (uint8_t)a, arc, status);
    }

    /* fix #6: emit scan_complete — bridge uses this to set per-ballast availability */
    lighting_emit_scan_complete(slave_id, present_mask);

    LOG_INFO("dirty", "Full scan slave %u complete — present=0x%016llX",
             slave_id, (unsigned long long)present_mask);
}

void dirty_poll_background_check(uint8_t slave_id) {
    int idx = state_cache_slave_idx(slave_id);
    if (idx < 0 || s_dp[idx].active) return;

    uint16_t present_regs[4] = {0};
    if (!sync_read(slave_id, DALI_PRESENT_START, 4, present_regs)) {
        gateway_report(slave_id, idx, false);
        return;
    }
    gateway_report(slave_id, idx, true);

    uint64_t present_mask =
        ((uint64_t)present_regs[0])       |
        ((uint64_t)present_regs[1] << 16) |
        ((uint64_t)present_regs[2] << 32) |
        ((uint64_t)present_regs[3] << 48);

    LOG_DBG("dirty", "background check slave=%u present=0x%016llx",
             slave_id, (unsigned long long)present_mask);

    uint64_t prev        = s_dp[idx].last_present;
    uint64_t appeared    = present_mask & ~prev;
    uint64_t disappeared = prev & ~present_mask;

    if (appeared || disappeared) {
        LOG_INFO("dirty",
                 "presence change slave=%u appeared=0x%016llx disappeared=0x%016llx",
                 slave_id,
                 (unsigned long long)appeared,
                 (unsigned long long)disappeared);
        s_dp[idx].last_present = present_mask;
        for (int a = 0; a < DALI_ADDR_COUNT; a++) {
            state_cache_set_dali_present((uint8_t)idx, (uint8_t)a,
                                          (present_mask >> a) & 1);
        }
        lighting_emit_presence(slave_id, appeared, disappeared);
    }

    /* Re-read group membership masks so memberships set via Modbus by an
     * external master are picked up (and to recover from any drift). */
    read_all_group_masks(slave_id, (uint8_t)idx);

    /* BLOCK0 is FC03 holding regs post refactor. */
    LOG_DBG("dirty",
            "slave=%u: background check reading BLOCK0 arc/status (holding regs %u..%u) to detect faults",
            slave_id, (unsigned)DALI_BLOCK0_START,
            (unsigned)(DALI_BLOCK0_START + DALI_ADDR_COUNT - 1));
    uint16_t b0[DALI_ADDR_COUNT];
    bool b0_ok[4];
    for (uint16_t c = 0; c < 4; c++) {
        b0_ok[c] = sync_read_holding(slave_id, DALI_BLOCK0_START + c * DALI_MAX_CHUNK,
                                     DALI_MAX_CHUNK, b0 + c * DALI_MAX_CHUNK);
        if (!b0_ok[c])
            LOG_WARN("dirty", "bg block0 chunk %u read failed slave %u", (unsigned)c, slave_id);
    }
    for (int a = 0; a < DALI_ADDR_COUNT; a++) {
        if (!(present_mask & ((uint64_t)1 << a))) continue;
        if (!b0_ok[a / DALI_MAX_CHUNK]) continue;   /* skip fault check on a failed read */
        uint8_t status = (b0[a] >> 8) & 0xFF;
        if ((status & 0x02) || (status & 0x80)) {
            lighting_emit_dali_state(slave_id, (uint8_t)a, b0[a] & 0xFF, status);
            lighting_emit_fault(slave_id, (uint8_t)a,
                                (status & 0x02) != 0, (status & 0x80) != 0);
        }
    }
}
