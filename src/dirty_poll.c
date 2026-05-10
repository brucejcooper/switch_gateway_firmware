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
#include "pico/time.h"
#include <string.h>
#include <stdio.h>

typedef enum {
    DPOLL_IDLE,
    DPOLL_AWAITING_DIRTY,
    DPOLL_AWAITING_BALLAST,
} dpoll_state_t;

typedef struct {
    dpoll_state_t state;
    bool          active;
    uint8_t       zero_count;
    uint32_t      last_poll_ms;
    uint32_t      last_bg_ms;
    uint64_t      last_present;
    uint64_t      pending_dirty;
    uint8_t       next_ballast;
} dpoll_t;

static dpoll_t s_dp[MAX_DALI_SLAVES];

typedef struct { uint8_t slave_id; uint8_t addr; } ballast_ctx_t;
static ballast_ctx_t s_bctx[MAX_DALI_SLAVES][DALI_ADDR_COUNT];
static uint8_t s_dirty_ctx_slave[MAX_DALI_SLAVES];

void dirty_poll_init(void) {
    memset(s_dp, 0, sizeof(s_dp));
    for (int i = 0; i < MAX_DALI_SLAVES; i++) {
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
    s_dp[idx].active     = true;
    s_dp[idx].zero_count = 0;
}

static void ballast_read_cb(int res, uint8_t *buf, void *ctx);   /* forward decl */

/* ── Dirty bitmask callback ──────────────────────────────────────────────── */
static void dirty_bitmask_cb(int res, uint8_t *buf, void *ctx) {
    uint8_t slave_id = *(uint8_t *)ctx;
    int     idx      = state_cache_slave_idx(slave_id);
    if (idx < 0) return;

    dpoll_t *dp = &s_dp[idx];
    dp->state   = DPOLL_IDLE;

    /* fix #5: FC04 4-register frame = slave+FC+bytecount+8data+CRC = 13 bytes */
    if (res < 13) {
        printf("dirty bitmask read failed slave %u (%d)\n", slave_id, res);
        return;
    }

    uint8_t *p = buf + 3;
    uint64_t mask = ((uint64_t)((p[0]<<8)|p[1]))       |
                    ((uint64_t)((p[2]<<8)|p[3]) << 16)  |
                    ((uint64_t)((p[4]<<8)|p[5]) << 32)  |
                    ((uint64_t)((p[6]<<8)|p[7]) << 48);

    if (mask == 0) {
        dp->zero_count++;
        if (dp->zero_count >= DIRTY_ZERO_SETTLE_COUNT) {
            dp->active     = false;
            dp->zero_count = 0;
        }
    } else {
        dp->zero_count    = 0;
        dp->pending_dirty = mask;
        for (int a = 0; a < DALI_ADDR_COUNT; a++) {
            if (mask & ((uint64_t)1 << a)) {
                dp->state = DPOLL_AWAITING_BALLAST;
                modbus_get_input_registers(slave_id,
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
        lighting_emit_dali_state(slave, addr, arc, status);
        if ((status & 0x02) || (status & 0x80)) {
            lighting_emit_fault(slave, addr,
                                (status & 0x02) != 0,
                                (status & 0x80) != 0);
        }
    }

    dp->pending_dirty &= ~((uint64_t)1 << addr);
    for (int a = addr + 1; a < DALI_ADDR_COUNT; a++) {
        if (dp->pending_dirty & ((uint64_t)1 << a)) {
            dp->state = DPOLL_AWAITING_BALLAST;
            modbus_get_input_registers(slave,
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

/* ── Blocking startup full scan ──────────────────────────────────────────── */

void dirty_poll_full_scan(uint8_t slave_id) {
    int idx = state_cache_slave_idx(slave_id);
    if (idx < 0) return;

    printf("Full scan: slave %u\n", slave_id);

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

    /* Present bitmask */
    uint16_t present_regs[4] = {0};
    sync_read(slave_id, DALI_PRESENT_START, 4, present_regs);
    uint64_t present_mask =
        ((uint64_t)present_regs[0])       |
        ((uint64_t)present_regs[1] << 16) |
        ((uint64_t)present_regs[2] << 32) |
        ((uint64_t)present_regs[3] << 48);
    s_dp[idx].last_present = present_mask;

    /* Block 0 in 16-register chunks */
    uint16_t regs[DALI_ADDR_COUNT];
    for (uint16_t chunk = 0; chunk < 4; chunk++) {
        uint16_t off = chunk * DALI_MAX_CHUNK;
        if (!sync_read(slave_id, DALI_BLOCK0_START + off, DALI_MAX_CHUNK, regs + off))
            printf("block0 chunk %u failed\n", chunk);
    }
    for (int a = 0; a < DALI_ADDR_COUNT; a++) {
        bool present = (present_mask & ((uint64_t)1 << a)) != 0;
        state_cache_set_dali_present((uint8_t)idx, (uint8_t)a, present);
        if (!present) continue;
        state_cache_set_dali_arc((uint8_t)idx, (uint8_t)a,
                                  regs[a] & 0xFF,
                                  (regs[a] >> 8) & 0xFF);
    }

    uint16_t b1[DALI_ADDR_COUNT], b2[DALI_ADDR_COUNT], b3[DALI_ADDR_COUNT];
    for (uint16_t chunk = 0; chunk < 4; chunk++) {
        uint16_t off = chunk * DALI_MAX_CHUNK;
        sync_read(slave_id, DALI_BLOCK1_START + off, DALI_MAX_CHUNK, b1 + off);
        sync_read(slave_id, DALI_BLOCK2_START + off, DALI_MAX_CHUNK, b2 + off);
        sync_read(slave_id, DALI_BLOCK3_START + off, DALI_MAX_CHUNK, b3 + off);
    }
    for (int a = 0; a < DALI_ADDR_COUNT; a++) {
        if (!(present_mask & ((uint64_t)1 << a))) continue;
        state_cache_set_dali_config((uint8_t)idx, (uint8_t)a,
                                    (b1[a] >> 8) & 0xFF, b1[a] & 0xFF,
                                    (b2[a] >> 8) & 0xFF, b2[a] & 0xFF,
                                    (b3[a] >> 8) & 0xFF, b3[a] & 0xFF);
    }

    /* fix #6: emit scan_complete — bridge uses this to set per-ballast availability */
    lighting_emit_scan_complete(slave_id, present_mask);

    printf("Full scan slave %u complete — present=0x%016llX\n",
           slave_id, (unsigned long long)present_mask);
}

void dirty_poll_background_check(uint8_t slave_id) {
    int idx = state_cache_slave_idx(slave_id);
    if (idx < 0 || s_dp[idx].active) return;

    uint16_t present_regs[4] = {0};
    if (!sync_read(slave_id, DALI_PRESENT_START, 4, present_regs)) return;

    uint64_t present_mask =
        ((uint64_t)present_regs[0])       |
        ((uint64_t)present_regs[1] << 16) |
        ((uint64_t)present_regs[2] << 32) |
        ((uint64_t)present_regs[3] << 48);

    uint64_t prev        = s_dp[idx].last_present;
    uint64_t appeared    = present_mask & ~prev;
    uint64_t disappeared = prev & ~present_mask;

    if (appeared || disappeared) {
        s_dp[idx].last_present = present_mask;
        for (int a = 0; a < DALI_ADDR_COUNT; a++) {
            state_cache_set_dali_present((uint8_t)idx, (uint8_t)a,
                                          (present_mask >> a) & 1);
        }
        lighting_emit_presence(slave_id, appeared, disappeared);
    }

    uint16_t b0[DALI_ADDR_COUNT];
    for (uint16_t c = 0; c < 4; c++) {
        sync_read(slave_id, DALI_BLOCK0_START + c * DALI_MAX_CHUNK,
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
