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
} dpoll_t;

static dpoll_t s_dp[MAX_DALI_SLAVES];

typedef struct { uint8_t slave_id; uint8_t addr; } ballast_ctx_t;
static ballast_ctx_t s_bctx[MAX_DALI_SLAVES][DALI_ADDR_COUNT];
static uint8_t s_dirty_ctx_slave[MAX_DALI_SLAVES];

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
            lighting_emit_gateway_state(slave_id, true);
        }
    } else if (dp->reachable) {
        if (dp->fail_count < 0xFF) dp->fail_count++;
        if (dp->fail_count >= GATEWAY_FAIL_THRESHOLD) {
            dp->reachable = false;
            dp->active    = false;   /* stop fast-polling a dead gateway; the
                                        60s background check still probes it and
                                        recovers it (and a command re-activates) */
            lighting_emit_gateway_state(slave_id, false);
        }
    }
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
                dp->state = DPOLL_AWAITING_BALLAST;
                LOG_DBG("dirty",
                         "slave=%u: ballast %d flagged dirty — reading BLOCK0 holding reg %u for fresh arc/status",
                         slave_id, a, (unsigned)(DALI_BLOCK0_START + a));
                /* BLOCK0 is now an FC03 holding-register bank (post refactor). */
                modbus_get_holding_registers(slave_id,
                                             DALI_BLOCK0_START + (uint16_t)a, 1,
                                             ballast_read_cb, &s_bctx[idx][a]);
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
            lighting_emit_dali_state(slave, addr, arc, status);
            LOG_DBG("dirty", "ballast %u.%u arc=%u status=0x%02x",
                     slave, addr, arc, status);
            if ((status & 0x02) || (status & 0x80)) {
                lighting_emit_fault(slave, addr,
                                    (status & 0x02) != 0,
                                    (status & 0x80) != 0);
            }
        }
    }

    dp->pending_dirty &= ~((uint64_t)1 << addr);
    for (int a = addr + 1; a < DALI_ADDR_COUNT; a++) {
        if (dp->pending_dirty & ((uint64_t)1 << a)) {
            dp->state = DPOLL_AWAITING_BALLAST;
            LOG_DBG("dirty",
                     "slave=%u: ballast %d also dirty — reading next BLOCK0 holding reg %u",
                     slave, a, (unsigned)(DALI_BLOCK0_START + a));
            /* BLOCK0 is now an FC03 holding-register bank (post refactor). */
            modbus_get_holding_registers(slave,
                                         DALI_BLOCK0_START + (uint16_t)a, 1,
                                         ballast_read_cb, &s_bctx[idx][a]);
            return;
        }
    }
}

/* ── Tick ────────────────────────────────────────────────────────────────── */
void dirty_poll_tick(void) {
    uint32_t now_ms = to_ms_since_boot(get_absolute_time());

    for (int i = 0; i < g_config.dali_slave_count; i++) {
        uint8_t  slave_id = g_config.dali_slave_ids[i];
        dpoll_t *dp       = &s_dp[i];

        /* fix #7: do not poll slaves being provisioned */
        if (dali_provision_busy_slave(slave_id)) continue;

        if (dp->active && dp->state == DPOLL_IDLE) {
            if (now_ms - dp->last_poll_ms >= DIRTY_POLL_INTERVAL_MS) {
                dp->last_poll_ms = now_ms;
                dp->state        = DPOLL_AWAITING_DIRTY;
                modbus_get_input_registers(slave_id, DALI_DIRTY_START, 4,
                                           dirty_bitmask_cb,
                                           &s_dirty_ctx_slave[i]);
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
                dp->state         = DPOLL_AWAITING_DIRTY;
                modbus_get_input_registers(slave_id, DALI_DIRTY_START, 4,
                                           dirty_bitmask_cb,
                                           &s_dirty_ctx_slave[i]);
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
    for (uint16_t chunk = 0; chunk < 4; chunk++) {
        uint16_t off = chunk * DALI_MAX_CHUNK;
        if (!sync_read_holding(slave_id, DALI_BLOCK0_START + off, DALI_MAX_CHUNK, regs + off))
            LOG_WARN("dirty", "block0 chunk %u failed", chunk);
    }
    for (int a = 0; a < DALI_ADDR_COUNT; a++) {
        bool present = (present_mask & ((uint64_t)1 << a)) != 0;
        state_cache_set_dali_present((uint8_t)idx, (uint8_t)a, present);
        if (!present) continue;
        state_cache_set_dali_arc((uint8_t)idx, (uint8_t)a,
                                  regs[a] & 0xFF,
                                  (regs[a] >> 8) & 0xFF);
    }

    /* BLOCK1..3 are FC03 holding regs post refactor. */
    LOG_DBG("dirty",
             "slave=%u: full scan reading BLOCK1-3 config (holding regs %u..%u) for all ballasts",
             slave_id, (unsigned)DALI_BLOCK1_START,
             (unsigned)(DALI_BLOCK3_START + DALI_ADDR_COUNT - 1));
    uint16_t b1[DALI_ADDR_COUNT], b2[DALI_ADDR_COUNT], b3[DALI_ADDR_COUNT];
    for (uint16_t chunk = 0; chunk < 4; chunk++) {
        uint16_t off = chunk * DALI_MAX_CHUNK;
        sync_read_holding(slave_id, DALI_BLOCK1_START + off, DALI_MAX_CHUNK, b1 + off);
        sync_read_holding(slave_id, DALI_BLOCK2_START + off, DALI_MAX_CHUNK, b2 + off);
        sync_read_holding(slave_id, DALI_BLOCK3_START + off, DALI_MAX_CHUNK, b3 + off);
    }
    for (int a = 0; a < DALI_ADDR_COUNT; a++) {
        if (!(present_mask & ((uint64_t)1 << a))) continue;
        /* BLOCK1 wire order is (min<<8)|max (high byte = min, low = max); the
         * cfg args are (max_l, min_l).  BLOCK2/3 high byte = power_on/fade_rate. */
        state_cache_set_dali_config((uint8_t)idx, (uint8_t)a,
                                    b1[a] & 0xFF, (b1[a] >> 8) & 0xFF,
                                    (b2[a] >> 8) & 0xFF, b2[a] & 0xFF,
                                    (b3[a] >> 8) & 0xFF, b3[a] & 0xFF);
    }

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
    for (uint16_t c = 0; c < 4; c++) {
        sync_read_holding(slave_id, DALI_BLOCK0_START + c * DALI_MAX_CHUNK,
                          DALI_MAX_CHUNK, b0 + c * DALI_MAX_CHUNK);
    }
    for (int a = 0; a < DALI_ADDR_COUNT; a++) {
        if (!(present_mask & ((uint64_t)1 << a))) continue;
        uint8_t status = (b0[a] >> 8) & 0xFF;
        if ((status & 0x02) || (status & 0x80)) {
            lighting_emit_dali_state(slave_id, (uint8_t)a, b0[a] & 0xFF, status);
            lighting_emit_fault(slave_id, (uint8_t)a,
                                (status & 0x02) != 0, (status & 0x80) != 0);
        }
    }
}
