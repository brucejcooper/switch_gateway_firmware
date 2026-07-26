/*
 * dali_queue.c — controller-side driver for the gateway command queue (task 04).
 *
 * See dali_queue.h for the model. One dq_slave_t per configured DALI gateway.
 * Each holds a FIFO backlog of dq_cmd_t; the first `outstanding` entries from
 * `head` are in flight at the gateway (ids assigned), the rest are pending.
 * Every transaction is one FC23: write up to 4 new records into the submit
 * window (0x1100) and read the 10-register status block back through the
 * holding alias (0x1180) — submit-N / collect-N-1 in one round trip.
 *
 * Credit policy (19200 strict): submit min(pending, free_hint, 4), where
 * free_hint is the free_slots the gateway reported last. Because free_slots only
 * decreases when we submit (and the gateway frees slots as it executes, which we
 * learn on the next response), free_hint is a conservative lower bound — we
 * never over-submit, so the gateway never answers SLAVE_DEVICE_BUSY. The FC23
 * round-trip (~28-30 ms at 19200) is the natural pacing; a 0x06 exception is
 * treated as a defensive backstop (rollback + back off) but should not occur.
 *
 * Baud dependence lives in one place: DQ_DRAIN_POLL_MS (the collect-only pacing
 * while draining) and the fact that the submit rate is gated by the FC23
 * round-trip, which scales with baud. At 38400 the same strict-credit path stays
 * correct (just conservative); relaxing to optimistic pacing there is a future
 * knob, not needed for the 19200 bench profile.
 */

#include "dali_queue.h"
#include "modbus.h"
#include "config.h"
#include "dali_provision.h"
#include "log.h"
#include "pico/time.h"
#include <string.h>

/* Gateway register map (task 04). */
#define DQ_SUBMIT_WINDOW  0x1100u   /* QCMD base — FC23/FC16 write range */
#define DQ_STATUS_ALIAS   0x1180u   /* status block, holding alias — FC23 read range */
#define DQ_STATUS_REGS    10u       /* QSTAT + QPROG + QRES[8] */
#define DQ_GW_DEPTH       8u        /* gateway queue depth (matches GWQ_DEPTH) */
#define DQ_MAX_SUBMIT     4u        /* records per submit window */

#define DQ_BACKLOG        16u       /* local priority-queue depth (per slave) */
#define DQ_DRAIN_POLL_MS  20u       /* collect-only pacing while draining (~1 DALI frame) */
#define DQ_VERIFY_RETRIES 3u        /* status-read attempts during transport recovery */

typedef struct {
    uint8_t  addr, data, flags, dtr0;
    uint8_t  id;             /* gateway id once submitted; 0 = not yet assigned */
    bool     idempotent;
    dq_result_cb_t cb;
    void    *ctx;
} dq_cmd_t;

typedef enum { DQ_IDLE, DQ_AWAIT, DQ_VERIFY, DQ_ADOPT } dq_state_t;

typedef struct {
    bool     in_use;
    uint8_t  slave_id;
    bool     adopted;        /* have synced next_id/last_completed from the gateway */

    dq_cmd_t bl[DQ_BACKLOG];
    uint8_t  head;           /* oldest entry */
    uint8_t  count;          /* submitted + pending */
    uint8_t  outstanding;    /* first `outstanding` from head are confirmed in flight */

    uint8_t  next_id;        /* next gateway id to assign (1..31) */
    uint8_t  free_hint;      /* gateway free slots from last status (credit) */
    uint8_t  last_completed; /* completed_id seen last response (0 = none) */

    uint16_t boot_count;
    bool     boot_valid;
    bool     reboot_flag;    /* set by dq_note_boot_count on a change */

    dq_state_t st;
    uint8_t  submit_n;       /* records shipped in the in-flight txn (tentative) */
    uint8_t  submit_last_id; /* last id in the in-flight submit (verify target) */
    uint8_t  id_before;      /* next_id before the in-flight submit (rollback) */
    uint8_t  verify_left;    /* status-read attempts remaining in recovery */
    absolute_time_t next_at;

    volatile bool cb_fired;
    int      cb_res;
    uint8_t  cb_buf[32];
    uint8_t  cb_len;
} dq_slave_t;

static dq_slave_t s_dq[MAX_DALI_SLAVES];

/* ── helpers ─────────────────────────────────────────────────────────────── */

static uint8_t next_gw_id(uint8_t id) { return (id >= 31u) ? 1u : (uint8_t)(id + 1u); }

static inline uint8_t bl_idx(const dq_slave_t *s, uint8_t off) {
    return (uint8_t)((s->head + off) % DQ_BACKLOG);
}

static dq_slave_t *slave_for(uint8_t slave_id) {
    for (int i = 0; i < MAX_DALI_SLAVES; i++)
        if (s_dq[i].in_use && s_dq[i].slave_id == slave_id) return &s_dq[i];
    return NULL;
}

/* Read register i (0-based) from the FC23/FC03 read body captured in cb_buf.
 * Body layout: [addr, fc, bytecount, hi0, lo0, hi1, lo1, ...]. */
static uint16_t cb_reg(const dq_slave_t *s, uint8_t i) {
    uint8_t off = (uint8_t)(3u + i * 2u);
    if ((uint16_t)off + 1u >= s->cb_len) return 0;
    return (uint16_t)((s->cb_buf[off] << 8) | s->cb_buf[off + 1u]);
}

/* ── public API ──────────────────────────────────────────────────────────── */

void dq_init(void) {
    memset(s_dq, 0, sizeof s_dq);
    for (int i = 0; i < g_config.dali_slave_count && i < MAX_DALI_SLAVES; i++) {
        s_dq[i].in_use    = true;
        s_dq[i].slave_id  = g_config.dali_slave_ids[i];
        s_dq[i].next_id   = 1u;
        s_dq[i].free_hint = DQ_GW_DEPTH;
        s_dq[i].st        = DQ_IDLE;
        s_dq[i].next_at   = get_absolute_time();
    }
}

bool dq_submit(uint8_t slave_id, uint8_t addr, uint8_t data, uint8_t flags,
               uint8_t dtr0, bool idempotent, dq_result_cb_t cb, void *ctx) {
    dq_slave_t *s = slave_for(slave_id);
    if (!s) {
        LOG_WARN("dq", "submit to unknown DALI slave %u dropped", slave_id);
        return false;
    }
    if (s->count >= DQ_BACKLOG) {
        LOG_WARN("dq", "slave %u backlog full — cmd %02X%02X dropped", slave_id, addr, data);
        return false;
    }
    dq_cmd_t *e = &s->bl[bl_idx(s, s->count)];
    e->addr = addr; e->data = data; e->flags = (uint8_t)(flags & 0x07u);
    e->dtr0 = dtr0; e->id = 0; e->idempotent = idempotent; e->cb = cb; e->ctx = ctx;
    s->count++;
    return true;
}

void dq_note_boot_count(uint8_t slave_id, uint16_t boot_count) {
    dq_slave_t *s = slave_for(slave_id);
    if (!s) return;
    if (s->boot_valid && boot_count != s->boot_count) {
        LOG_WARN("dq", "slave %u reboot detected (boot_count %u -> %u) — resync",
                 slave_id, s->boot_count, boot_count);
        s->reboot_flag = true;
    }
    s->boot_count = boot_count;
    s->boot_valid = true;
}

bool dq_busy(uint8_t slave_id) {
    dq_slave_t *s = slave_for(slave_id);
    return s && (s->count > 0 || s->st != DQ_IDLE);
}

/* ── driver internals ────────────────────────────────────────────────────── */

static void dq_fc23_cb(int res, uint8_t *buf, void *ctx);

/* Rebuild the backlog after a detected gateway reboot: drop non-idempotent
 * in-flight commands (their result is unknowable and replaying would double a
 * step), re-pend idempotent ones and all not-yet-submitted commands, and resync
 * the id sequence. Order is preserved. */
static void dq_resync(dq_slave_t *s) {
    dq_cmd_t keep[DQ_BACKLOG];
    uint8_t  nkeep = 0;
    uint8_t  in_flight = (uint8_t)(s->outstanding + s->submit_n);
    for (uint8_t i = 0; i < s->count; i++) {
        dq_cmd_t *e = &s->bl[bl_idx(s, i)];
        if (i < in_flight && !e->idempotent) {
            if (e->cb) e->cb(s->slave_id, DQ_RES_DROPPED, 0, e->ctx);
            continue;   /* drop */
        }
        e->id = 0;                 /* re-pend (resubmit idempotent / untouched pending) */
        keep[nkeep++] = *e;
    }
    memcpy(s->bl, keep, (size_t)nkeep * sizeof(dq_cmd_t));
    s->head = 0; s->count = nkeep; s->outstanding = 0; s->submit_n = 0;
    s->next_id = 1u; s->free_hint = DQ_GW_DEPTH; s->last_completed = 0;
    s->reboot_flag = false;
    s->st = DQ_IDLE; s->next_at = get_absolute_time();
}

/* Collect every newly-completed head entry (up to and including completed_id)
 * and route its result to the command's callback. */
static void dq_collect(dq_slave_t *s, uint8_t completed_id) {
    if (completed_id == 0 || completed_id == s->last_completed) return;
    bool done = false;
    while (!done && s->outstanding > 0) {
        dq_cmd_t *h = &s->bl[s->head];
        uint8_t rid = h->id;
        uint16_t r  = cb_reg(s, (uint8_t)(2u + (rid & 0x07u)));  /* QRES[rid&7] */
        dq_result_t res = (dq_result_t)((r >> 8) & 0xFFu);
        uint8_t     ans = (uint8_t)(r & 0xFFu);
        if (h->cb) h->cb(s->slave_id, res, ans, h->ctx);
        if (rid == completed_id) done = true;
        s->head = (uint8_t)((s->head + 1u) % DQ_BACKLOG);
        s->count--; s->outstanding--;
    }
    s->last_completed = completed_id;
}

/* True if the just-read status block indicates the gateway lost our commands
 * (rebooted): it reports a fresh, empty queue while we believe commands are in
 * flight. */
static bool dq_status_is_reboot(const dq_slave_t *s, uint8_t free_slots,
                                uint8_t accepted_id) {
    return (s->outstanding > 0 || s->submit_n > 0)
        && free_slots == DQ_GW_DEPTH && accepted_id == 0;
}

/* Issue one transaction: submit `n` pending records (n may be 0 for a
 * collect-only poll) and read the status block. Assigns tentative ids to the n
 * records being shipped. */
static void dq_issue(dq_slave_t *s, uint8_t n) {
    uint8_t payload[DQ_MAX_SUBMIT * 4u];
    uint8_t plen = 0;
    s->id_before = s->next_id;
    uint8_t last_id = 0;
    for (uint8_t i = 0; i < n; i++) {
        dq_cmd_t *e = &s->bl[bl_idx(s, (uint8_t)(s->outstanding + i))];
        e->id = s->next_id;
        last_id = e->id;
        s->next_id = next_gw_id(s->next_id);
        payload[plen++] = e->addr;
        payload[plen++] = e->data;
        payload[plen++] = (uint8_t)((e->id << 3) | (e->flags & 0x07u));  /* [id:5|flags:3] */
        payload[plen++] = e->dtr0;
    }
    s->cb_fired = false;
    bool ok = modbus_read_write_multiple_registers(
        s->slave_id, DQ_STATUS_ALIAS, DQ_STATUS_REGS,
        DQ_SUBMIT_WINDOW, (uint16_t)(n * 2u), n ? payload : NULL, dq_fc23_cb, s);
    if (!ok) {
        /* modbus action queue full: roll back the tentative id assignment and
         * retry shortly. Nothing was sent. */
        for (uint8_t i = 0; i < n; i++)
            s->bl[bl_idx(s, (uint8_t)(s->outstanding + i))].id = 0;
        s->next_id  = s->id_before;
        s->next_at  = make_timeout_time_ms(DQ_DRAIN_POLL_MS);
        return;
    }
    s->submit_n       = n;
    s->submit_last_id = last_id;
    s->st             = DQ_AWAIT;
}

/* Transport-recovery read: after an FC23 timeout we don't know whether the write
 * landed. Read the status block only (no write) so we can compare accepted_id to
 * the timed-out submit's last id before deciding to resubmit — never blind-
 * resubmit, since UP/DOWN steps are not idempotent. */
static void dq_issue_verify(dq_slave_t *s) {
    s->cb_fired = false;
    bool ok = modbus_read_write_multiple_registers(
        s->slave_id, DQ_STATUS_ALIAS, DQ_STATUS_REGS,
        DQ_SUBMIT_WINDOW, 0, NULL, dq_fc23_cb, s);
    if (!ok) { s->next_at = make_timeout_time_ms(DQ_DRAIN_POLL_MS); return; }
    s->st = DQ_VERIFY;
}

/* Boot-time id adoption read: before the first submit to a gateway, read its
 * status so we can continue the id sequence past its last accepted_id and treat
 * its current completed_id as already-collected. Without this, a controller-only
 * reboot (gateway kept running) could mis-collect fresh commands against the
 * gateway's retained completed_id / stale ring results. */
static void dq_issue_adopt(dq_slave_t *s) {
    s->cb_fired = false;
    bool ok = modbus_read_write_multiple_registers(
        s->slave_id, DQ_STATUS_ALIAS, DQ_STATUS_REGS,
        DQ_SUBMIT_WINDOW, 0, NULL, dq_fc23_cb, s);
    if (!ok) { s->next_at = make_timeout_time_ms(DQ_DRAIN_POLL_MS); return; }
    s->st = DQ_ADOPT;
}

/* modbus completion: stash the response for dq_tick to process (same core-0
 * context; the copy just decouples us from buf reuse). */
static void dq_fc23_cb(int res, uint8_t *buf, void *ctx) {
    dq_slave_t *s = (dq_slave_t *)ctx;
    s->cb_res = res;
    s->cb_len = 0;
    if (res > 0 && buf) {
        uint8_t n = (res > (int)sizeof s->cb_buf) ? (uint8_t)sizeof s->cb_buf
                                                  : (uint8_t)res;
        memcpy(s->cb_buf, buf, n);
        s->cb_len = n;
    } else if (res < 0 && buf) {
        s->cb_buf[0] = buf[0]; s->cb_buf[1] = buf[1]; s->cb_buf[2] = buf[2];
        s->cb_len = 3;
    }
    s->cb_fired = true;
}

/* Promote the just-shipped tentative records to confirmed in-flight. */
static void dq_promote_submit(dq_slave_t *s) {
    s->outstanding = (uint8_t)(s->outstanding + s->submit_n);
    s->submit_n = 0;
}

/* Roll back the just-shipped tentative records to pending (their ids are
 * un-assigned and the id sequence rewound) — used when a submit provably did
 * not land. */
static void dq_rollback_submit(dq_slave_t *s) {
    for (uint8_t i = 0; i < s->submit_n; i++)
        s->bl[bl_idx(s, (uint8_t)(s->outstanding + i))].id = 0;
    s->next_id  = s->id_before;
    s->submit_n = 0;
}

/* Process a completed FC23 (normal AWAIT path). */
static void dq_handle_await(dq_slave_t *s) {
    int res = s->cb_res;
    if (res <= 0) {
        /* Timeout (res==0) or exception (res<0). A 0x06 exception means the
         * all-or-nothing submit was rejected: nothing landed, so roll back and
         * back off (defensive — strict credits should prevent it). A timeout is
         * ambiguous: enter transport recovery (verify before resubmit). */
        uint8_t exc = (res < 0 && s->cb_len >= 3) ? s->cb_buf[2] : 0;
        if (res < 0) {
            dq_rollback_submit(s);
            if (exc == 0x06u) s->free_hint = 0;   /* no credit until we re-read */
            s->st = DQ_IDLE;
            s->next_at = make_timeout_time_ms(DQ_DRAIN_POLL_MS);
            return;
        }
        if (s->submit_n > 0) {
            s->verify_left = DQ_VERIFY_RETRIES;
            dq_issue_verify(s);
        } else {
            s->st = DQ_IDLE;   /* collect-only timed out; retry later */
            s->next_at = make_timeout_time_ms(DQ_DRAIN_POLL_MS);
        }
        return;
    }
    /* Success: parse the status block. */
    uint8_t free_slots  = (uint8_t)(cb_reg(s, 0) >> 8);
    uint8_t accepted_id = (uint8_t)(cb_reg(s, 0) & 0xFF);
    uint8_t completed   = (uint8_t)(cb_reg(s, 1) & 0xFF);

    if (s->reboot_flag || dq_status_is_reboot(s, free_slots, accepted_id)) {
        dq_resync(s);
        return;
    }
    dq_promote_submit(s);
    s->free_hint = free_slots;
    dq_collect(s, completed);
    s->st = DQ_IDLE;
    /* Go straight into the next submit when there is pending work and credit;
     * otherwise (just draining in-flight results) pace the collect-only polls. */
    uint8_t pending = (uint8_t)(s->count - s->outstanding);
    s->next_at = (pending > 0 && s->free_hint > 0)
                 ? get_absolute_time() : make_timeout_time_ms(DQ_DRAIN_POLL_MS);
}

/* Process a completed status-only read during transport recovery. */
static void dq_handle_verify(dq_slave_t *s) {
    int res = s->cb_res;
    if (res <= 0) {
        if (s->verify_left > 0) { s->verify_left--; dq_issue_verify(s); return; }
        /* Give up verifying: the safe assumption is "did not land" (resubmit an
         * idempotent command, drop a non-idempotent one). Model it as a reboot-
         * style rebuild of the in-flight set, which honours idempotency. */
        LOG_WARN("dq", "slave %u verify exhausted — rebuilding in-flight set", s->slave_id);
        dq_resync(s);
        return;
    }
    uint8_t free_slots  = (uint8_t)(cb_reg(s, 0) >> 8);
    uint8_t accepted_id = (uint8_t)(cb_reg(s, 0) & 0xFF);
    uint8_t completed   = (uint8_t)(cb_reg(s, 1) & 0xFF);

    if (s->reboot_flag || dq_status_is_reboot(s, free_slots, accepted_id)) {
        dq_resync(s);
        return;
    }
    /* Did the timed-out submit land? accepted_id (gateway-latched last accepted)
     * equals our submit_last_id iff the write took effect. */
    if (accepted_id == s->submit_last_id) dq_promote_submit(s);
    else                                  dq_rollback_submit(s);
    s->free_hint = free_slots;
    dq_collect(s, completed);
    s->st = DQ_IDLE;
    uint8_t pending = (uint8_t)(s->count - s->outstanding);
    s->next_at = (pending > 0 && s->free_hint > 0)
                 ? get_absolute_time() : make_timeout_time_ms(DQ_DRAIN_POLL_MS);
}

/* Process the boot-time adoption read: sync our id sequence to the gateway's. */
static void dq_handle_adopt(dq_slave_t *s) {
    int res = s->cb_res;
    if (res <= 0) { s->next_at = make_timeout_time_ms(DQ_DRAIN_POLL_MS); return; }  /* retry */
    uint8_t accepted_id = (uint8_t)(cb_reg(s, 0) & 0xFF);
    uint8_t completed   = (uint8_t)(cb_reg(s, 1) & 0xFF);
    uint8_t free_slots  = (uint8_t)(cb_reg(s, 0) >> 8);
    s->next_id        = (accepted_id == 0) ? 1u : next_gw_id(accepted_id);
    s->last_completed = completed;       /* the gateway's current completions are "old" to us */
    s->free_hint      = free_slots;
    s->adopted        = true;
    s->st             = DQ_IDLE;
    s->next_at        = get_absolute_time();
}

void dq_tick(void) {
    for (int i = 0; i < MAX_DALI_SLAVES; i++) {
        dq_slave_t *s = &s_dq[i];
        if (!s->in_use) continue;

        /* Await a response in flight. */
        if (s->st != DQ_IDLE) {
            if (!s->cb_fired) continue;
            s->cb_fired = false;
            if      (s->st == DQ_VERIFY) dq_handle_verify(s);
            else if (s->st == DQ_ADOPT)  dq_handle_adopt(s);
            else                         dq_handle_await(s);
            continue;
        }

        /* Idle: an out-of-band reboot signal takes priority. */
        if (s->reboot_flag) { dq_resync(s); continue; }

        if (!time_reached(s->next_at)) continue;
        /* A slave mid-provisioning owns its bus — hold off. */
        if (dali_provision_busy_slave(s->slave_id)) {
            s->next_at = make_timeout_time_ms(DQ_DRAIN_POLL_MS);
            continue;
        }

        uint8_t pending = (uint8_t)(s->count - s->outstanding);
        if (pending == 0 && s->outstanding == 0) continue;   /* nothing to do */

        /* First contact with real work: adopt the gateway's id sequence before
         * submitting anything, so a controller-only reboot can't mis-collect. */
        if (!s->adopted) { dq_issue_adopt(s); continue; }

        /* Strict credits: submit min(pending, free_hint, 4). When there is
         * pending work but no credit (free_hint==0), fall through to a
         * collect-only read (n=0) to refresh credit and reap results. */
        uint8_t n = pending;
        if (n > s->free_hint)    n = s->free_hint;
        if (n > DQ_MAX_SUBMIT)   n = DQ_MAX_SUBMIT;
        dq_issue(s, n);
    }
}

