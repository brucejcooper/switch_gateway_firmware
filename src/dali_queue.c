/*
 * dali_queue.c — DALI adapter over the generic Modbus command-queue client
 * (task 06, queue v2). See dali_queue.h for the model.
 *
 * This layer is DALI-SPECIFIC and sits ABOVE the DALI-agnostic mbq_client:
 *   1. a per-slave PRIORITY BACKLOG (the scheduler): dq_submit insertion-sorts
 *      by priority class (highest first, FIFO within a class); dq_tick ships the
 *      front of the backlog into a free gateway slot via the client, keeping at
 *      most DQ_INFLIGHT_MAX in flight so a fresh high-priority command is never
 *      stuck behind a backlog of background work already filling the 8 slots;
 *   2. a PAYLOAD ADAPTER: pack a DALI command into the client's 4 opaque command
 *      bytes ([addr, data, flags, dtr0]) and map the client's terminal outcome +
 *      response8 back to a dq_result_t for the producer's callback.
 *
 * There is NO id ledger and NO reboot resync/rebuild/rollback here — the client
 * handles gateway-reboot recovery from the slot statuses ("idle means gone"),
 * resubmitting idempotent commands and reporting the rest DQ_RES_DROPPED.
 */

#include "dali_queue.h"
#include "mbq_client.h"
#include "modbus.h"
#include "config.h"
#include "dali_provision.h"
#include "log.h"
#include "pico/time.h"
#include <string.h>
#include <stdio.h>

#define DQ_BACKLOG      16u   /* local backlog depth (per slave) */
#define DQ_INFLIGHT_MAX  3u   /* scheduler policy cap: 2-3 in flight per slave, below the
                               * gateway's 8, so a high-priority command has headroom */

/* DALI response8 failure codes on a FAILED slot — the payload contract mirrored
 * from the gateway (TinyModbus dali_sync.h). A COMPLETE slot's response8 is the
 * backward-frame answer byte instead. */
#define DALI_RESP_NAK        0x01u  /* no / short backward frame where one was expected */
#define DALI_RESP_COLLISION  0x02u  /* garbled backward frame (≥1 gear answered) */
#define DALI_RESP_BUSFAULT   0x03u  /* transport-level fault (short pulse etc.) */

typedef struct {
    uint8_t  addr, data, flags, dtr0;
    uint8_t  prio;           /* dq_prio_t */
    bool     idempotent;
    dq_result_cb_t cb;
    void    *ctx;
} dq_cmd_t;

/* One trampoline per possible in-flight command: carries the producer's
 * cb/ctx across the client, and maps the client's outcome back on completion.
 * The client stores &pend[i] as its per-slot ctx, so the record must be stable
 * for the command's lifetime (it lives in the slave struct). */
typedef struct {
    bool     in_use;
    dq_result_cb_t cb;
    void    *ctx;
} dq_pending_t;

typedef struct {
    bool     in_use;
    uint8_t  slave_id;

    dq_cmd_t bl[DQ_BACKLOG];
    uint8_t  count;          /* pending entries in the backlog */
    uint8_t  outstanding;    /* commands submitted to the client, awaiting completion */

    dq_pending_t pend[MBQ_SLOTS];

    /* Diagnostic counters (for the `dq` CLI command / bench fault drills). */
    uint32_t c_submitted;    /* commands shipped to the client */
    uint32_t c_ok;           /* delivered DQ_RES_OK */
    uint32_t c_nak;          /* delivered DQ_RES_NAK */
    uint32_t c_err;          /* delivered DQ_RES_ERR (collision / bus fault) */
    uint32_t c_dropped;      /* delivered DQ_RES_DROPPED (gateway reboot, non-idempotent) */
    uint32_t c_full;         /* submits refused — backlog full */
} dq_slave_t;

static dq_slave_t s_dq[MAX_DALI_SLAVES];

/* ── helpers ─────────────────────────────────────────────────────────────── */

static dq_slave_t *slave_for(uint8_t slave_id) {
    for (int i = 0; i < MAX_DALI_SLAVES; i++)
        if (s_dq[i].in_use && s_dq[i].slave_id == slave_id) return &s_dq[i];
    return NULL;
}

static dq_pending_t *pend_alloc(dq_slave_t *s) {
    for (uint8_t i = 0; i < MBQ_SLOTS; i++)
        if (!s->pend[i].in_use) return &s->pend[i];
    return NULL;
}

/* ── public API ──────────────────────────────────────────────────────────── */

void dq_init(void) {
    memset(s_dq, 0, sizeof s_dq);
    mbq_client_init();
    for (int i = 0; i < g_config.dali_slave_count && i < MAX_DALI_SLAVES; i++) {
        s_dq[i].in_use   = true;
        s_dq[i].slave_id = g_config.dali_slave_ids[i];
        mbq_client_add_slave(s_dq[i].slave_id);
    }
}

bool dq_submit(uint8_t slave_id, dq_prio_t prio, uint8_t addr, uint8_t data,
               uint8_t flags, uint8_t dtr0, bool idempotent,
               dq_result_cb_t cb, void *ctx) {
    dq_slave_t *s = slave_for(slave_id);
    if (!s) {
        LOG_WARN("dq", "submit to unknown DALI slave %u dropped", slave_id);
        return false;
    }
    if (s->count >= DQ_BACKLOG) {
        LOG_WARN("dq", "slave %u backlog full — cmd %02X%02X dropped", slave_id, addr, data);
        s->c_full++;
        return false;
    }
    /* Insertion-sort into the backlog by priority (ascending class number = higher
     * priority first). Insert before the first strictly-lower-priority entry, so
     * equal-priority entries keep FIFO order. */
    uint8_t pos = s->count;
    for (uint8_t i = 0; i < s->count; i++) {
        if (s->bl[i].prio > (uint8_t)prio) { pos = i; break; }
    }
    for (uint8_t i = s->count; i > pos; i--) s->bl[i] = s->bl[i - 1];
    s->bl[pos] = (dq_cmd_t){ .addr = addr, .data = data,
                             .flags = (uint8_t)(flags & 0x07u), .dtr0 = dtr0,
                             .prio = (uint8_t)prio, .idempotent = idempotent,
                             .cb = cb, .ctx = ctx };
    s->count++;
    return true;
}

bool dq_busy(uint8_t slave_id) {
    dq_slave_t *s = slave_for(slave_id);
    if (!s) return false;
    return s->count > 0 || mbq_client_busy(slave_id);
}

/* ── driver internals ────────────────────────────────────────────────────── */

/* Client completion trampoline: map the generic outcome + response8 to a
 * dq_result_t and hand it to the producer's callback, then release the
 * trampoline and the in-flight slot. Runs from mbq_client_tick (main-loop)
 * context, invoked exactly once per submitted command. */
static void dq_mbq_cb(uint8_t slave_id, mbq_result_t r, uint8_t response8, void *ctx) {
    dq_pending_t *p = (dq_pending_t *)ctx;
    dq_result_cb_t cb  = p->cb;
    void          *cctx = p->ctx;
    p->in_use = false;

    dq_slave_t *s = slave_for(slave_id);
    if (s && s->outstanding > 0) s->outstanding--;

    dq_result_t res = DQ_RES_DROPPED;
    uint8_t     answer = 0;
    switch (r) {
        case MBQ_R_COMPLETE:
            res = DQ_RES_OK; answer = response8;
            if (s) s->c_ok++;
            break;
        case MBQ_R_FAILED:
            if (response8 == DALI_RESP_NAK) { res = DQ_RES_NAK; if (s) s->c_nak++; }
            else { res = DQ_RES_ERR; answer = response8; if (s) s->c_err++; }
            break;
        case MBQ_R_LOST:
        default:
            res = DQ_RES_DROPPED;
            if (s) s->c_dropped++;
            break;
    }
    if (cb) cb(slave_id, res, answer, cctx);
}

/* Ship the front backlog command of `s` into a free gateway slot. Returns true
 * if one was submitted (backlog shrank), false to try again next tick. */
static bool ship_front(dq_slave_t *s) {
    dq_pending_t *p = pend_alloc(s);
    if (!p) return false;   /* all trampolines busy (should not happen under the cap) */

    dq_cmd_t *e = &s->bl[0];
    uint8_t cmd[MBQ_CMD_BYTES] = { e->addr, e->data, e->flags, e->dtr0 };

    p->in_use = true;
    p->cb     = e->cb;
    p->ctx    = e->ctx;
    if (!mbq_client_submit(s->slave_id, cmd, e->idempotent, dq_mbq_cb, p)) {
        p->in_use = false;   /* modbus action queue full — nothing sent; retry next tick */
        return false;
    }
    /* Landed with the client: drop the front of the backlog. */
    for (uint8_t i = 0; i + 1u < s->count; i++) s->bl[i] = s->bl[i + 1u];
    s->count--;
    s->outstanding++;
    s->c_submitted++;
    return true;
}

void dq_tick(void) {
    /* Drive the underlying client first: process completions (freeing slots and
     * trampolines) and pace polls, so a slot freed this tick is available to the
     * scheduler below. */
    mbq_client_tick();

    for (int i = 0; i < MAX_DALI_SLAVES; i++) {
        dq_slave_t *s = &s_dq[i];
        if (!s->in_use || s->count == 0) continue;

        /* Provisioning is highest priority AND bus-exclusive: while a slave is
         * being commissioned (the 0x1F04 commission SM) the scheduler admits
         * nothing else — commands accumulate in the backlog and drain once
         * provisioning finishes. */
        if (dali_provision_busy_slave(s->slave_id)) continue;

        /* Ship while we have credit and the client can take a command this
         * instant. can_submit is false once a transaction is in flight, so this
         * naturally ships one command per tick per slave. */
        if (s->outstanding < DQ_INFLIGHT_MAX && mbq_client_can_submit(s->slave_id))
            ship_front(s);
    }
}

/* ── Blocking helper for the text CLI ────────────────────────────────────── */

static volatile bool s_sync_have;
static dq_result_t   s_sync_res;
static uint8_t       s_sync_ans;

static void dq_sync_cb(uint8_t slave, dq_result_t res, uint8_t ans, void *ctx) {
    (void)slave; (void)ctx;
    s_sync_res = res; s_sync_ans = ans; s_sync_have = true;
}

bool dq_submit_sync(uint8_t slave_id, uint8_t addr, uint8_t data, uint8_t flags,
                    uint8_t dtr0, bool idempotent, uint32_t timeout_ms,
                    dq_result_t *result, uint8_t *answer) {
    s_sync_have = false;
    if (!dq_submit(slave_id, DQ_PRIO_USER, addr, data, flags, dtr0, idempotent,
                   dq_sync_cb, NULL)) {
        if (result) *result = DQ_RES_BACKLOG;   /* rejected up front */
        return false;
    }
    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);
    while (!s_sync_have && absolute_time_diff_us(get_absolute_time(), deadline) > 0) {
        modbus_poll();
        dq_tick();
    }
    if (!s_sync_have) { if (result) *result = DQ_RES_TIMEOUT; return false; }
    if (result) *result = s_sync_res;
    if (answer) *answer = s_sync_ans;
    return true;
}

/* ── `dq` CLI observability ───────────────────────────────────────────────── */

void dq_debug_print(int slave_id) {
    bool any = false;
    for (int i = 0; i < MAX_DALI_SLAVES; i++) {
        dq_slave_t *s = &s_dq[i];
        if (!s->in_use) continue;
        if (slave_id >= 0 && s->slave_id != (uint8_t)slave_id) continue;
        any = true;
        uint8_t cls[DQ_PRIO_COUNT] = {0};
        for (uint8_t j = 0; j < s->count; j++)
            if (s->bl[j].prio < DQ_PRIO_COUNT) cls[s->bl[j].prio]++;
        printf("dq slave %u: backlog=%u inflight=%u/%u\n",
               s->slave_id, s->count, s->outstanding, DQ_INFLIGHT_MAX);
        printf("  queued/class: prov=%u user=%u gwrite=%u cfgw=%u lread=%u bg=%u\n",
               cls[DQ_PRIO_PROVISION], cls[DQ_PRIO_USER], cls[DQ_PRIO_GROUP_WRITE],
               cls[DQ_PRIO_CONFIG_WRITE], cls[DQ_PRIO_LEVEL_READ], cls[DQ_PRIO_BACKGROUND]);
        printf("  counters: submitted=%lu ok=%lu nak=%lu err=%lu dropped=%lu backlog_full=%lu\n",
               (unsigned long)s->c_submitted, (unsigned long)s->c_ok,
               (unsigned long)s->c_nak, (unsigned long)s->c_err,
               (unsigned long)s->c_dropped, (unsigned long)s->c_full);
        mbq_client_debug_print((int)s->slave_id);
    }
    if (!any) printf("dq: no matching DALI slave\n");
}
