/*
 * mbq_client.c — controller-side generic Modbus command-queue client (task 06,
 * queue v2). DALI-agnostic; see mbq_client.h and docs/modbus-command-queue.md.
 *
 * One slave_t per Modbus slave, each an OPTIMISTIC 8-slot cache + a single-in-
 * flight FC23 transaction SM. There is NO id ledger: the gateway's 8 slot statuses
 * are the truth, reconciled on every read-back.
 *
 * Per-slot cache lifecycle (controller's belief):
 *   FREE      — reusable (never used, or a terminal we already delivered)
 *   ARMED     — holds a command queued to (re)write to the gateway (fresh submit,
 *               or a reboot-resubmit); not yet on the wire
 *   SENT      — the command's FC23 write was issued but NOT yet confirmed present
 *               on the gateway (submit read-back not seen, or it timed out)
 *   LIVE      — confirmed pending/executing on the gateway; awaiting a terminal
 * A terminal read (complete/failed) delivers the callback once and returns the
 * slot to FREE.
 *
 * The `SENT` vs `LIVE` (confirmed) distinction is load-bearing and is the v2 answer
 * to the transport-glitch/reboot ambiguity that plagued v1:
 *   - "idle means gone" (gateway reboot → fail all) fires ONLY for a CONFIRMED
 *     (LIVE) slot that reads back idle. On a real reboot the gateway clears ALL
 *     slots, so every confirmed-owned slot reads idle — an unambiguous signal.
 *   - A slot still SENT (write issued, never confirmed) that reads idle is NOT
 *     treated as a reboot (so a single lost submit-write can't false-trigger
 *     fail-all). But its outcome is genuinely ambiguous — the write either never
 *     landed (never executed) OR landed+executed then the gateway rebooted before
 *     this read (already ran) — so it follows the SAME idempotency policy as the
 *     reboot path: re-arm to resubmit ONLY if idempotent, else deliver LOST. A
 *     non-idempotent step is never blind-resubmitted.
 * ABA / deliver-once: a slot is delivered exactly when it leaves owned→FREE; a FREE
 * slot is ignored on re-reads. A collected-terminal slot is only reused by a new
 * submit, whose FC23 write executes before the read (gateway ordering), so the
 * read-back shows the fresh command's slot state, never the stale terminal.
 */

#include "mbq_client.h"
#include "modbus.h"
#include "config.h"
#include "log.h"
#include "pico/time.h"
#include <string.h>
#include <stdio.h>

/* v2 register map (mirrors the gateway; see TinyModbus registers.h). */
#define MBQ_CMD_BASE   0x1000u   /* slot k command at 0x1000 + 2*k (2 regs) */
#define MBQ_STAT_BASE  0x1010u   /* slot k status  at 0x1010 + k   (1 reg)  */
#define MBQ_POLL_MS    5u        /* pace of active-slot poll reads          */

/* Gateway slot lifecycle status (hi byte of a status reg) — must match the
 * gateway MBQ_S_* enum. Generic values; not DALI. */
enum { GW_IDLE = 0u, GW_PENDING = 1u, GW_EXECUTING = 2u, GW_COMPLETE = 3u, GW_FAILED = 4u };

/* Controller per-slot cache state. */
enum { SLOT_FREE = 0u, SLOT_ARMED, SLOT_SENT, SLOT_LIVE };

typedef struct {
    uint8_t  cmd[MBQ_CMD_BYTES];
    mbq_cb_t cb;
    void    *ctx;
    bool     idempotent;
    uint8_t  st;   /* SLOT_* */
} slot_t;

typedef enum { TX_IDLE, TX_AWAIT } tx_t;

typedef struct {
    bool     in_use;
    uint8_t  slave_id;
    slot_t   slot[MBQ_SLOTS];

    tx_t     tx;
    uint8_t  tx_slot;          /* slot written this txn, or 0xFF for a read-only poll */
    absolute_time_t next_at;

    volatile bool cb_fired;
    int      cb_res;
    uint8_t  cb_buf[40];
    uint8_t  cb_len;

    /* diagnostics */
    uint32_t c_submitted, c_completed, c_failed, c_lost, c_reboots, c_timeouts;
    uint32_t c_polls;   /* read-only status-poll transactions issued */

    /* Per-transaction timing window (bench; reset by mbq_client_txn_stats_reset).
     * t_issue is stamped when the FC23 is handed to the Modbus layer, so the
     * measured span includes queueing behind other producers and transport
     * retries — which is exactly what "is this Modbus-bound?" needs to know. */
    absolute_time_t t_issue;
    uint32_t w_n, w_min, w_max, w_fail, w_busy;
    uint64_t w_sum;
} slave_t;

static slave_t s_sl[MAX_DALI_SLAVES];

/* ── helpers ─────────────────────────────────────────────────────────────── */

static slave_t *slave_for(uint8_t slave_id) {
    for (int i = 0; i < MAX_DALI_SLAVES; i++)
        if (s_sl[i].in_use && s_sl[i].slave_id == slave_id) return &s_sl[i];
    return NULL;
}

/* Status reg k (0..7) from the read body captured in cb_buf:
 * [addr, fc, bytecount, hi0, lo0, hi1, lo1, ...]. */
static uint8_t stat_status(const slave_t *s, uint8_t k) {
    uint8_t off = (uint8_t)(3u + k * 2u);
    return ((uint16_t)off < s->cb_len) ? s->cb_buf[off] : 0xFFu;
}
static uint8_t stat_resp(const slave_t *s, uint8_t k) {
    uint8_t off = (uint8_t)(4u + k * 2u);
    return ((uint16_t)off < s->cb_len) ? s->cb_buf[off] : 0u;
}

static void mbq_fc23_cb(int res, uint8_t *buf, void *ctx) {
    slave_t *s = (slave_t *)ctx;
    /* Transaction timing window: stamp the span before anything else. */
    uint64_t dt = (uint64_t)absolute_time_diff_us(s->t_issue, get_absolute_time());
    uint32_t dt32 = (dt > 0xFFFFFFFFull) ? 0xFFFFFFFFu : (uint32_t)dt;
    if (s->w_n == 0 || dt32 < s->w_min) s->w_min = dt32;
    if (dt32 > s->w_max) s->w_max = dt32;
    s->w_sum += dt32;
    s->w_n++;
    if (res <= 0) {
        s->w_fail++;
        if (res < 0 && buf && buf[2] == 0x06u) s->w_busy++;
    }
    s->cb_res = res;
    s->cb_len = 0;
    if (res > 0 && buf) {
        uint8_t n = (res > (int)sizeof s->cb_buf) ? (uint8_t)sizeof s->cb_buf : (uint8_t)res;
        memcpy(s->cb_buf, buf, n);
        s->cb_len = n;
    } else if (res < 0 && buf) {
        /* Exception frame [addr, fc|0x80, exc_code]: keep the code so a submit that
         * was rejected SLAVE_DEVICE_BUSY (0x06 = target slot active) can be read as
         * "the command actually landed". */
        s->cb_buf[0] = buf[0]; s->cb_buf[1] = buf[1]; s->cb_buf[2] = buf[2];
        s->cb_len = 3;
    }
    s->cb_fired = true;
}

static bool any_free_slot(const slave_t *s, uint8_t *slot_out) {
    for (uint8_t k = 0; k < MBQ_SLOTS; k++)
        if (s->slot[k].st == SLOT_FREE) { if (slot_out) *slot_out = k; return true; }
    return false;
}
static bool any_owned_slot(const slave_t *s) {
    for (uint8_t k = 0; k < MBQ_SLOTS; k++)
        if (s->slot[k].st != SLOT_FREE) return true;
    return false;
}
static bool any_armed_slot(const slave_t *s, uint8_t *slot_out) {
    for (uint8_t k = 0; k < MBQ_SLOTS; k++)
        if (s->slot[k].st == SLOT_ARMED) { if (slot_out) *slot_out = k; return true; }
    return false;
}

/* ── public API ──────────────────────────────────────────────────────────── */

void mbq_client_init(void) {
    memset(s_sl, 0, sizeof s_sl);
}

void mbq_client_add_slave(uint8_t slave_id) {
    if (slave_for(slave_id)) return;
    for (int i = 0; i < MAX_DALI_SLAVES; i++) {
        if (!s_sl[i].in_use) {
            s_sl[i].in_use   = true;
            s_sl[i].slave_id = slave_id;
            s_sl[i].tx       = TX_IDLE;
            s_sl[i].tx_slot  = 0xFFu;
            s_sl[i].next_at  = get_absolute_time();
            return;
        }
    }
    LOG_WARN("mbq", "no room to register slave %u", slave_id);
}

bool mbq_client_busy(uint8_t slave_id) {
    slave_t *s = slave_for(slave_id);
    if (!s) return false;
    return s->tx != TX_IDLE || any_owned_slot(s);
}

uint8_t mbq_client_inflight(uint8_t slave_id) {
    slave_t *s = slave_for(slave_id);
    if (!s) return 0;
    uint8_t n = 0;
    for (uint8_t k = 0; k < MBQ_SLOTS; k++)
        if (s->slot[k].st != SLOT_FREE) n++;
    return n;
}

void mbq_client_txn_counts(uint8_t slave_id, uint32_t *submits, uint32_t *polls) {
    slave_t *s = slave_for(slave_id);
    if (submits) *submits = s ? s->c_submitted : 0u;
    if (polls)   *polls   = s ? s->c_polls     : 0u;
}

void mbq_client_txn_stats(uint8_t slave_id, mbq_txn_stats_t *out) {
    if (!out) return;
    memset(out, 0, sizeof *out);
    slave_t *s = slave_for(slave_id);
    if (!s || s->w_n == 0) return;
    out->n      = s->w_n;
    out->us_min = s->w_min;
    out->us_max = s->w_max;
    out->us_avg = (uint32_t)(s->w_sum / s->w_n);
    out->n_fail = s->w_fail;
    out->n_busy = s->w_busy;
}

void mbq_client_txn_stats_reset(uint8_t slave_id) {
    slave_t *s = slave_for(slave_id);
    if (!s) return;
    s->w_n = 0; s->w_min = 0; s->w_max = 0; s->w_sum = 0; s->w_fail = 0; s->w_busy = 0;
}

bool mbq_client_can_submit(uint8_t slave_id) {
    slave_t *s = slave_for(slave_id);
    if (!s) return false;
    return s->tx == TX_IDLE && any_free_slot(s, NULL);
}

/* Issue the FC23 for slot k: write its 4 command bytes + read all 8 status regs.
 * tx_slot=k records that this txn's read-back should confirm slot k. */
static void issue_submit(slave_t *s, uint8_t k) {
    s->cb_fired = false;
    s->t_issue  = get_absolute_time();
    bool ok = modbus_read_write_multiple_registers(
        s->slave_id, MBQ_STAT_BASE, MBQ_SLOTS,
        (uint16_t)(MBQ_CMD_BASE + 2u * k), MBQ_CMD_BYTES, s->slot[k].cmd,
        mbq_fc23_cb, s);
    if (!ok) { s->next_at = make_timeout_time_ms(MBQ_POLL_MS); return; }
    s->slot[k].st = SLOT_SENT;   /* written, not yet confirmed present */
    s->tx      = TX_AWAIT;
    s->tx_slot = k;
    s->c_submitted++;
}

/* Issue a read-only poll of all 8 status regs. */
static void issue_poll(slave_t *s) {
    s->cb_fired = false;
    s->t_issue  = get_absolute_time();
    bool ok = modbus_read_write_multiple_registers(
        s->slave_id, MBQ_STAT_BASE, MBQ_SLOTS, MBQ_CMD_BASE, 0, NULL, mbq_fc23_cb, s);
    if (!ok) { s->next_at = make_timeout_time_ms(MBQ_POLL_MS); return; }
    s->tx      = TX_AWAIT;
    s->tx_slot = 0xFFu;
    s->c_polls++;
}

bool mbq_client_submit(uint8_t slave_id, const uint8_t cmd[MBQ_CMD_BYTES],
                       bool idempotent, mbq_cb_t cb, void *ctx) {
    slave_t *s = slave_for(slave_id);
    if (!s || s->tx != TX_IDLE) return false;
    uint8_t k;
    if (!any_free_slot(s, &k)) return false;
    memcpy(s->slot[k].cmd, cmd, MBQ_CMD_BYTES);
    s->slot[k].cb         = cb;
    s->slot[k].ctx        = ctx;
    s->slot[k].idempotent = idempotent;
    issue_submit(s, k);
    return s->tx == TX_AWAIT;   /* false if the modbus action queue was full */
}

static void deliver(slave_t *s, uint8_t k, mbq_result_t r, uint8_t resp) {
    mbq_cb_t cb = s->slot[k].cb; void *ctx = s->slot[k].ctx;
    s->slot[k].st = SLOT_FREE;   /* collected → reusable (ABA: only reuse a collected slot) */
    s->slot[k].cb = NULL;
    if (cb) cb(s->slave_id, r, resp, ctx);
}

/* Resolve an ambiguous owned slot by idempotency: re-arm to resubmit if the
 * command is safe to repeat, else deliver it LOST. Used for every case where the
 * slot's read-back status cannot be trusted as this command's true outcome. */
static void resolve_ambiguous(slave_t *s, uint8_t k) {
    if (s->slot[k].idempotent) s->slot[k].st = SLOT_ARMED;   /* resubmit; a later 0x06 confirms if it landed */
    else { deliver(s, k, MBQ_R_LOST, 0); s->c_lost++; }
}

/* Reconcile the 8 just-read statuses against the cache. `submit_slot` is the slot
 * this transaction wrote (0xFF for a read-only poll) — the ONLY slot whose status
 * is trustworthy while still SENT, because its write executed before this same
 * read (gateway FC23 ordering). Deliver terminals once, confirm actives, detect
 * the gateway-reboot "idle means gone" signal.
 *
 * TRUST RULE (the v2 mis-attribution guard): a never-confirmed SENT slot resolved
 * by anything other than its own submit read-back has an AMBIGUOUS status — it may
 * be a stale pre-reuse terminal (a collected terminal we reused whose rewrite timed
 * out), an unlanded idle, or a real landed state — so it is NEVER delivered by that
 * status; it is resolved by idempotency instead. Only LIVE slots (confirmed active
 * earlier) and the current submit_slot are trusted to carry this command's result. */
static void reconcile(slave_t *s, uint8_t submit_slot) {
    /* Pass 1: a CONFIRMED (LIVE) owned slot reading idle = whole-gateway reboot
     * (a reboot clears every slot; a still-SENT slot reading idle is handled per
     * slot below, not as a reboot). */
    bool reboot = false;
    for (uint8_t k = 0; k < MBQ_SLOTS; k++)
        if (s->slot[k].st == SLOT_LIVE && stat_status(s, k) == GW_IDLE) { reboot = true; break; }

    if (reboot) {
        s->c_reboots++;
        LOG_WARN("mbq", "slave %u reboot (idle-means-gone) — failing outstanding", s->slave_id);
        for (uint8_t k = 0; k < MBQ_SLOTS; k++)
            if (s->slot[k].st != SLOT_FREE && s->slot[k].st != SLOT_ARMED) resolve_ambiguous(s, k);
        return;
    }

    /* Pass 2: per owned slot. The rule (reviewer-04): a SENT slot is resolved only
     * by (i) reading pending/executing → LIVE, or (ii) the idempotency policy for
     * anything else; a TERMINAL is delivered only from a slot confirmed LIVE or from
     * the submit_slot's own read-back. */
    for (uint8_t k = 0; k < MBQ_SLOTS; k++) {
        uint8_t cs = s->slot[k].st;
        if (cs == SLOT_FREE || cs == SLOT_ARMED) continue;
        uint8_t gw = stat_status(s, k);

        /* PENDING/EXECUTING is unambiguous: the gateway makes a slot active ONLY in
         * response to our write, so even a still-SENT slot reading active means its
         * (response-lost) write actually landed → confirm LIVE. */
        if (gw == GW_PENDING || gw == GW_EXECUTING) { s->slot[k].st = SLOT_LIVE; continue; }

        /* Terminal or idle. Trust it as THIS command's outcome only from a slot we
         * confirmed LIVE, or the submit_slot's OWN read-back (its write executed
         * before this same read → its terminal is ours, never a stale reused one). A
         * never-confirmed SENT slot resolved by a later poll is ambiguous (a stale
         * pre-reuse terminal, or an unlanded idle) → idempotency-resolve, never
         * deliver its status. */
        bool trusted = (cs == SLOT_LIVE) || (k == submit_slot);
        if (!trusted) { resolve_ambiguous(s, k); continue; }

        switch (gw) {
            case GW_COMPLETE: deliver(s, k, MBQ_R_COMPLETE, stat_resp(s, k)); s->c_completed++; break;
            case GW_FAILED:   deliver(s, k, MBQ_R_FAILED,   stat_resp(s, k)); s->c_failed++;    break;
            case GW_IDLE:
            default:
                /* A LIVE slot idle is a reboot (Pass 1); the submit_slot reading idle
                 * in its own successful write+read can't happen (the write landed).
                 * Defensive only. */
                resolve_ambiguous(s, k);
                break;
        }
    }
}

void mbq_client_tick(void) {
    for (int i = 0; i < MAX_DALI_SLAVES; i++) {
        slave_t *s = &s_sl[i];
        if (!s->in_use) continue;

        if (s->tx == TX_AWAIT) {
            if (!s->cb_fired) continue;
            s->cb_fired = false;
            int res = s->cb_res;
            uint8_t submit_slot = s->tx_slot;
            s->tx = TX_IDLE;
            if (res > 0) {
                reconcile(s, submit_slot);
                s->next_at = get_absolute_time();   /* act again immediately if there's more */
            } else {
                /* Timeout / exception — the txn did not complete. If a SUBMIT was
                 * rejected SLAVE_DEVICE_BUSY (0x06), the target slot is ACTIVE on the
                 * gateway: our command actually landed and is executing (the usual
                 * case is a resubmit of a SENT command whose original write landed but
                 * whose response was lost). Confirm it LIVE — do NOT keep resubmitting,
                 * which would loop. Any other error: leave the slot SENT; a later
                 * poll's read-back resolves it (idempotency-gated, never trusting a
                 * possibly-stale status). */
                uint8_t exc = (res < 0 && s->cb_len >= 3) ? s->cb_buf[2] : 0u;
                if (submit_slot != 0xFFu && exc == 0x06u
                    && s->slot[submit_slot].st == SLOT_SENT) {
                    s->slot[submit_slot].st = SLOT_LIVE;
                }
                s->c_timeouts++;
                s->next_at = make_timeout_time_ms(MBQ_POLL_MS);
            }
            continue;
        }

        /* TX_IDLE: choose the next action. */
        uint8_t k;
        if (any_armed_slot(s, &k)) {           /* a fresh/resubmit command to write */
            issue_submit(s, k);
            continue;
        }
        if (any_owned_slot(s) && time_reached(s->next_at)) {   /* SENT/LIVE → poll */
            issue_poll(s);
            continue;
        }
        /* else: nothing owned → idle (the adapter submits via mbq_client_submit). */
    }
}

void mbq_client_debug_print(int slave_id) {
    bool any = false;
    for (int i = 0; i < MAX_DALI_SLAVES; i++) {
        slave_t *s = &s_sl[i];
        if (!s->in_use) continue;
        if (slave_id >= 0 && s->slave_id != (uint8_t)slave_id) continue;
        any = true;
        printf("mbq slave %u: tx=%s tx_slot=%u\n", s->slave_id,
               s->tx == TX_IDLE ? "idle" : "await", s->tx_slot);
        printf("  slots:");
        static const char *nm[] = { "free", "armed", "sent", "live" };
        for (uint8_t k = 0; k < MBQ_SLOTS; k++)
            printf(" %u:%s", k, s->slot[k].st <= SLOT_LIVE ? nm[s->slot[k].st] : "?");
        printf("\n  counters: submitted=%lu completed=%lu failed=%lu lost=%lu "
               "reboots=%lu timeouts=%lu polls=%lu\n",
               (unsigned long)s->c_submitted, (unsigned long)s->c_completed,
               (unsigned long)s->c_failed, (unsigned long)s->c_lost,
               (unsigned long)s->c_reboots, (unsigned long)s->c_timeouts,
               (unsigned long)s->c_polls);
    }
    if (!any) printf("mbq: no matching slave\n");
}
