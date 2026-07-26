/*
 * dali_queue.c — controller-side driver for the gateway command queue (task 04).
 *
 * See dali_queue.h for the model. One dq_slave_t per configured DALI gateway,
 * each a per-slave scheduler state machine. The backlog is a plain array:
 * entries [0 .. outstanding-1] are in flight at the gateway (ids assigned, kept
 * in submission order for collection); entries [outstanding .. count-1] are
 * pending, kept sorted by priority class (highest first, FIFO within a class).
 * dq_submit insertion-sorts into the pending region; the scheduler always ships
 * the front of that region, so the highest-priority pending command goes first.
 *
 * Every transaction is one FC23: write up to DQ_MAX_SUBMIT new records into the
 * submit window (0x1000) and read the 10-register status block back through the
 * holding view (0x1010) — submit-N / collect-N-1 in one round trip.
 *
 * Credit policy (19200 strict): submit min(pending, free_hint, DQ_INFLIGHT_MAX -
 * outstanding, DQ_MAX_SUBMIT). free_hint (the gateway's last-reported free_slots)
 * only decreases when we submit, so it is a conservative lower bound — we never
 * over-submit, so the gateway never answers SLAVE_DEVICE_BUSY. In-flight depth is
 * capped at DQ_INFLIGHT_MAX (2-3), well below the gateway's 8, so background work
 * never fills the gateway queue and starves a fresh user command of headroom.
 * The FC23 round-trip (~28-30 ms at 19200) is the pacing; DQ_DRAIN_POLL_MS (the
 * one baud-adjacent constant) paces collect-only polls while draining.
 */

#include "dali_queue.h"
#include "modbus.h"
#include "config.h"
#include "dali_provision.h"
#include "log.h"
#include "pico/time.h"
#include <string.h>
#include <stdio.h>

/* Gateway register map (task 04, round 0x1000 base). */
#define DQ_SUBMIT_WINDOW  0x1000u   /* QCMD base — FC23/FC16 write range */
#define DQ_STATUS_ALIAS   0x1010u   /* status block, FC03 holding view — FC23 read range */
#define DQ_STATUS_REGS    10u       /* QSTAT + QPROG + QRES[8] */
#define DQ_GW_DEPTH       8u        /* gateway queue depth (matches GWQ_DEPTH) */
#define DQ_MAX_SUBMIT     4u        /* records per submit window */
#define DQ_INFLIGHT_MAX   3u        /* controller policy cap: 2-3 in flight per slave */

#define DQ_BACKLOG        16u       /* local backlog depth (per slave) */
#define DQ_DRAIN_POLL_MS  20u       /* collect-only pacing while draining (~1 DALI frame) */
#define DQ_VERIFY_RETRIES 3u        /* status-read attempts during transport recovery */

typedef struct {
    uint8_t  addr, data, flags, dtr0;
    uint8_t  prio;           /* dq_prio_t */
    uint8_t  id;             /* gateway id once in flight; 0 = pending */
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
    uint8_t  count;          /* total entries: in-flight + pending */
    uint8_t  outstanding;    /* first `outstanding` entries are in flight (submission order) */

    uint8_t  next_id;        /* next gateway id to assign (1..31) */
    uint8_t  free_hint;      /* gateway free slots from last status (credit) */
    uint8_t  last_completed; /* completed_id seen last response (0 = none) */
    uint8_t  last_accepted;  /* accepted_id seen last response (for `dq` dump) */

    /* Diagnostic counters (for the `dq` CLI command / bench fault drills). */
    uint32_t c_submitted;    /* records shipped */
    uint32_t c_collected;    /* results routed to callbacks */
    uint32_t c_timeouts;     /* FC23 transactions that timed out */
    uint32_t c_resubmits;    /* records rolled back to pending (did-not-land / busy) */
    uint32_t c_busy;         /* SLAVE_DEVICE_BUSY (0x06) exceptions */
    uint32_t c_adopts;       /* boot-time id adoptions */
    uint32_t c_resyncs;      /* gateway-reboot resyncs performed (total) */
    uint32_t c_resync_boot;  /* ...of which triggered by a boot_count change */
    uint32_t c_resync_anom;  /* ...of which triggered by a status anomaly / verify-exhaust */

    uint16_t boot_count;
    bool     boot_valid;
    bool     reboot_flag;    /* set by dq_note_boot_count on a change */
    bool     resync_pending; /* a gated resync awaits an adoption read (no status in hand yet) */
    uint8_t  resync_reason;  /* dq_resync_reason_t captured for the deferred resync */

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

static void dq_fc23_cb(int res, uint8_t *buf, void *ctx);

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
        return false;
    }
    /* Insertion-sort into the pending region [outstanding .. count-1] by priority
     * (ascending class number = higher priority first). Insert before the first
     * strictly-lower-priority pending entry, so equal-priority entries keep FIFO
     * order and the in-flight prefix [0 .. outstanding-1] is untouched. */
    uint8_t pos = s->count;
    for (uint8_t i = s->outstanding; i < s->count; i++) {
        if (s->bl[i].prio > (uint8_t)prio) { pos = i; break; }
    }
    for (uint8_t i = s->count; i > pos; i--) s->bl[i] = s->bl[i - 1];
    s->bl[pos] = (dq_cmd_t){ .addr = addr, .data = data,
                             .flags = (uint8_t)(flags & 0x07u), .dtr0 = dtr0,
                             .prio = (uint8_t)prio, .id = 0, .idempotent = idempotent,
                             .cb = cb, .ctx = ctx };
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

/* Remove the front (oldest in-flight) entry, shifting the rest down. */
static void dq_pop_front(dq_slave_t *s) {
    for (uint8_t i = 0; i + 1u < s->count; i++) s->bl[i] = s->bl[i + 1u];
    s->count--;
    if (s->outstanding > 0) s->outstanding--;
}

/* What tripped a resync, for counters + the log line (the bench drill needed
 * archaeology to tell an anomaly-triggered resync from a boot_count-triggered
 * one). */
typedef enum {
    DQ_RSN_ANOMALY,   /* status block showed a fresh/empty queue while we had work */
    DQ_RSN_BOOTCOUNT, /* sysinfo poll observed the gateway's boot_count change */
    DQ_RSN_VERIFY,    /* transport recovery exhausted its status-read retries */
} dq_resync_reason_t;

static const char *dq_reason_name(dq_resync_reason_t r) {
    switch (r) {
        case DQ_RSN_BOOTCOUNT: return "boot-count";
        case DQ_RSN_VERIFY:    return "verify-exhausted";
        default:               return "status-anomaly";
    }
}

/* Command surgery for a CONFIRMED-fresh gateway (accepted_id read as 0): the
 * gateway lost every command we believed in flight, so drop the non-idempotent
 * ones (their result is unknowable and replaying a step would double it) and
 * re-pend the idempotent ones plus all still-pending, re-sorted by priority.
 * "In flight" spans both the confirmed outstanding prefix AND any tentative
 * submit_n records — a fresh reboot lost those too. This touches ONLY the backlog
 * (outstanding/submit_n/count); it does NOT set the id baseline — dq_resync_apply
 * baselines next_id/last_completed/free_hint/adopted from the gateway's live
 * status for both branches, exactly as dq_handle_adopt does, so the fresh path can
 * never drift from the gateway's real ids the way a hardcoded 1/0 did. Callers
 * MUST gate this on accepted_id==0 (dq_resync_apply); running it against a live
 * gateway is exactly the ledger corruption this path exists to avoid. */
static void dq_rebuild_fresh(dq_slave_t *s) {
    uint8_t inflight = (uint8_t)(s->outstanding + s->submit_n);
    dq_cmd_t keep[DQ_BACKLOG];
    uint8_t  nkeep = 0;
    for (uint8_t i = 0; i < s->count; i++) {
        dq_cmd_t *e = &s->bl[i];
        if (i < inflight && !e->idempotent) {
            if (e->cb) e->cb(s->slave_id, DQ_RES_DROPPED, 0, e->ctx);
            continue;   /* drop */
        }
        e->id = 0;                 /* re-pend */
        keep[nkeep++] = *e;
    }
    /* Stable insertion sort by priority (small n). */
    for (uint8_t i = 1; i < nkeep; i++) {
        dq_cmd_t tmp = keep[i];
        int j = (int)i - 1;
        while (j >= 0 && keep[j].prio > tmp.prio) { keep[j + 1] = keep[j]; j--; }
        keep[j + 1] = tmp;
    }
    memcpy(s->bl, keep, (size_t)nkeep * sizeof(dq_cmd_t));
    s->count = nkeep; s->outstanding = 0; s->submit_n = 0;
}

/* Collect every newly-completed front entry (up to and including completed_id)
 * and route its result to the command's callback. In-flight entries are held in
 * submission order at the front, and the gateway completes in submission order,
 * so the front entry's id is the next to complete.
 *
 * KNOWN LIMITATION (tracked: tasks/06-dq-collect-hardening.md): if a gateway reboot
 * mid-burst leaves entries at the front whose ids the fresh gateway will never
 * complete (their reboot detection lagged behind normal collection), this loop can
 * momentarily attribute a stale ring slot to them while draining past — a TRANSIENT
 * wrong-answer window that self-heals once drained. The boot_count-authoritative
 * resync above prevents the PERMANENT version of this; fully closing the transient
 * needs dq_collect to drop entries the gateway can no longer complete. */
static void dq_collect(dq_slave_t *s, uint8_t completed_id) {
    if (completed_id == 0 || completed_id == s->last_completed) return;
    bool done = false;
    while (!done && s->outstanding > 0) {
        dq_cmd_t *h = &s->bl[0];
        uint8_t rid = h->id;
        uint16_t r  = cb_reg(s, (uint8_t)(2u + (rid & 0x07u)));  /* QRES[rid&7] */
        dq_result_t res = (dq_result_t)((r >> 8) & 0xFFu);
        uint8_t     ans = (uint8_t)(r & 0xFFu);
        if (h->cb) h->cb(s->slave_id, res, ans, h->ctx);
        s->c_collected++;
        if (rid == completed_id) done = true;
        dq_pop_front(s);
    }
    s->last_completed = completed_id;
}

/* True if the just-read status block indicates the gateway lost our commands
 * (rebooted): a fresh, empty queue while we believe commands are in flight. */
static bool dq_status_is_reboot(const dq_slave_t *s, uint8_t free_slots,
                                uint8_t accepted_id) {
    return (s->outstanding > 0 || s->submit_n > 0)
        && free_slots == DQ_GW_DEPTH && accepted_id == 0;
}

/* Issue one transaction: submit `n` pending records (n may be 0 for a
 * collect-only poll) and read the status block. The n records are the front of
 * the pending region (highest priority). Assigns tentative ids. */
static void dq_issue(dq_slave_t *s, uint8_t n) {
    uint8_t payload[DQ_MAX_SUBMIT * 4u];
    uint8_t plen = 0;
    s->id_before = s->next_id;
    uint8_t last_id = 0;
    for (uint8_t i = 0; i < n; i++) {
        dq_cmd_t *e = &s->bl[s->outstanding + i];
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
        for (uint8_t i = 0; i < n; i++) s->bl[s->outstanding + i].id = 0;
        s->next_id  = s->id_before;
        s->next_at  = make_timeout_time_ms(DQ_DRAIN_POLL_MS);
        return;
    }
    s->submit_n       = n;
    s->submit_last_id = last_id;
    s->c_submitted   += n;
    s->st             = DQ_AWAIT;
}

/* Transport-recovery / adoption read: status block only, no write. */
static void dq_issue_read_only(dq_slave_t *s, dq_state_t st) {
    s->cb_fired = false;
    bool ok = modbus_read_write_multiple_registers(
        s->slave_id, DQ_STATUS_ALIAS, DQ_STATUS_REGS,
        DQ_SUBMIT_WINDOW, 0, NULL, dq_fc23_cb, s);
    if (!ok) { s->next_at = make_timeout_time_ms(DQ_DRAIN_POLL_MS); return; }
    s->st = st;
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

static void dq_promote_submit(dq_slave_t *s) {
    s->outstanding = (uint8_t)(s->outstanding + s->submit_n);
    s->submit_n = 0;
}

static void dq_rollback_submit(dq_slave_t *s) {
    for (uint8_t i = 0; i < s->submit_n; i++) s->bl[s->outstanding + i].id = 0;
    s->next_id     = s->id_before;
    s->c_resubmits += s->submit_n;
    s->submit_n    = 0;
}

/* Apply a resync using the gateway's just-read status. accepted_id gates ONLY the
 * command surgery, never the id-baseline source:
 *
 *  - Gateway fresh (accepted_id==0): the in-flight set is genuinely lost, so
 *    dq_rebuild_fresh drops non-idempotent / re-pends idempotent (backlog only).
 *  - Gateway alive (accepted_id!=0): the detection was spurious or a duplicate
 *    (the two reboot detectors both firing for one reboot, classically the
 *    boot_count poll re-flagging a reboot the status-anomaly detector already
 *    consumed). The in-flight commands are still genuinely at the gateway, so we
 *    resolve any tentative submit and KEEP them — never tearing them down.
 *
 * Then BOTH branches baseline next_id/last_completed/free_hint/adopted from the
 * gateway's live status through the SAME logic dq_handle_adopt uses on a warm
 * controller boot — the path proven clean on hardware. The fresh branch used to
 * hardcode next_id=1/last_completed=0, a parallel baseline that sat one behind the
 * gateway's real completed_id and shifted the result ring by one slot (persistent
 * accepted−completed=1 / answer-lag). Deriving last_completed from the live read
 * instead of assuming 0 removes that skew. For the alive branch last_completed is
 * advanced by dq_collect (which collects the delta from its prior value up to
 * completed_id), so it must NOT be pre-set there. Either branch clears boot_valid
 * so the boot_count detector re-baselines instead of firing again. */
static void dq_resync_apply(dq_slave_t *s, dq_resync_reason_t reason,
                            uint8_t accepted_id, uint8_t completed_id,
                            uint8_t free_slots) {
    s->c_resyncs++;
    if (reason == DQ_RSN_BOOTCOUNT) s->c_resync_boot++; else s->c_resync_anom++;
    /* A boot_count delta that survived the boot_valid=false suppression is
     * AUTHORITATIVE proof the gateway rebooted and LOST its queue — even when the
     * controller's own post-reboot submits have since made accepted_id!=0 (which is
     * why the accepted==0 status-anomaly detector never fires in a busy burst, and
     * the lost pre-reboot in-flight commands stay stuck at the front of the backlog).
     * Captured from reboot_flag (or the BOOTCOUNT reason on the deferred path, where
     * dq_defer_resync already consumed the flag) BEFORE reboot_flag is cleared below,
     * so a verify-exhaust whose adopt read coincides with a boot_count change is also
     * treated as confirmed. A boot_count-confirmed trigger ALWAYS fresh-rebuilds
     * (re-pend the lost in-flight set); alive-reconcile is reserved for UNCONFIRMED
     * detections against a genuinely-live gateway (accepted!=0, boot_count unchanged),
     * so real in-flight is never torn down. The boot_valid=false suppression already
     * blocks a boot_count poll from double-firing for an anomaly-consumed reboot, so
     * treating a surviving boot_count trigger as authoritative does not reintroduce
     * the spurious-double-detection teardown. */
    bool confirmed_reboot = (reason == DQ_RSN_BOOTCOUNT) || s->reboot_flag;

    s->boot_valid     = false;
    s->reboot_flag    = false;
    s->resync_pending = false;

    if (confirmed_reboot || accepted_id == 0) {
        LOG_WARN("dq", "slave %u resync (%s): queue lost (accepted=%u) — rebuilding in-flight set",
                 s->slave_id, dq_reason_name(reason), accepted_id);
        dq_rebuild_fresh(s);
    } else {
        LOG_WARN("dq", "slave %u resync (%s): gateway alive (accepted=%u) — reconciling ids",
                 s->slave_id, dq_reason_name(reason), accepted_id);
        if (s->submit_n > 0) {   /* resolve any tentative submit before adopting the ledger */
            if (accepted_id == s->submit_last_id) dq_promote_submit(s);
            else                                  dq_rollback_submit(s);
        }
    }

    /* Unified id-baseline from the live status — identical to dq_handle_adopt. After a
     * fresh-rebuild the in-flight set is empty, so last_completed is set directly; only
     * a genuine alive-reconcile has kept in-flight entries to collect the delta for. */
    s->next_id   = (accepted_id == 0) ? 1u : next_gw_id(accepted_id);
    s->free_hint = free_slots;
    if (confirmed_reboot || accepted_id == 0) s->last_completed = completed_id;
    else                                      dq_collect(s, completed_id);
    s->adopted   = true;

    s->st = DQ_IDLE; s->next_at = get_absolute_time();
}

/* Defer a gated resync for a trigger with no status in hand (idle-tick boot_count
 * change, or transport-recovery exhaustion): request an adoption read and let
 * dq_handle_adopt route it back through dq_resync_apply once the gateway answers,
 * so the fresh-vs-alive gate always sees a real accepted_id. */
static void dq_defer_resync(dq_slave_t *s, dq_resync_reason_t reason) {
    s->resync_pending = true;
    s->resync_reason  = (uint8_t)reason;
    s->adopted        = false;
    s->reboot_flag    = false;
    s->st             = DQ_IDLE;
    s->next_at        = get_absolute_time();
}

/* Pacing after a completed transaction: straight into the next submit when there
 * is pending work with credit, otherwise pace the collect-only drain polls. */
static void dq_pace_next(dq_slave_t *s) {
    s->st = DQ_IDLE;
    uint8_t pending = (uint8_t)(s->count - s->outstanding);
    bool have_credit = s->free_hint > 0 && s->outstanding < DQ_INFLIGHT_MAX;
    s->next_at = (pending > 0 && have_credit)
                 ? get_absolute_time() : make_timeout_time_ms(DQ_DRAIN_POLL_MS);
}

static void dq_handle_await(dq_slave_t *s) {
    int res = s->cb_res;
    if (res <= 0) {
        uint8_t exc = (res < 0 && s->cb_len >= 3) ? s->cb_buf[2] : 0;
        if (res < 0) {
            /* Exception: the all-or-nothing submit was rejected, nothing landed.
             * Roll back and back off (defensive — strict credits prevent it). */
            if (exc == 0x06u) { s->c_busy++; s->free_hint = 0; }  /* no credit until we re-read */
            dq_rollback_submit(s);
            s->st = DQ_IDLE;
            s->next_at = make_timeout_time_ms(DQ_DRAIN_POLL_MS);
            return;
        }
        s->c_timeouts++;
        if (s->submit_n > 0) {                    /* timeout: verify before resubmit */
            s->verify_left = DQ_VERIFY_RETRIES;
            dq_issue_read_only(s, DQ_VERIFY);
        } else {
            s->st = DQ_IDLE;                       /* collect-only timed out; retry */
            s->next_at = make_timeout_time_ms(DQ_DRAIN_POLL_MS);
        }
        return;
    }
    uint8_t free_slots  = (uint8_t)(cb_reg(s, 0) >> 8);
    uint8_t accepted_id = (uint8_t)(cb_reg(s, 0) & 0xFF);
    uint8_t completed   = (uint8_t)(cb_reg(s, 1) & 0xFF);
    s->last_accepted = accepted_id;
    /* Confirmed-fresh reboot (queue empty + accepted 0 while we hold work): rebuild
     * against the fresh gateway. dq_status_is_reboot guarantees accepted_id==0, so
     * dq_resync_apply takes its fresh branch. */
    if (dq_status_is_reboot(s, free_slots, accepted_id)) {
        dq_resync_apply(s, s->reboot_flag ? DQ_RSN_BOOTCOUNT : DQ_RSN_ANOMALY,
                        accepted_id, completed, free_slots);
        return;
    }
    /* boot_count backstop fired but the gateway is alive (accepted!=0): a
     * duplicate/spurious detection — reconcile ids from this very read, keep the
     * in-flight set, never tear it down. */
    if (s->reboot_flag) {
        dq_resync_apply(s, DQ_RSN_BOOTCOUNT, accepted_id, completed, free_slots);
        return;
    }
    dq_promote_submit(s);
    s->free_hint = free_slots;
    dq_collect(s, completed);
    dq_pace_next(s);
}

static void dq_handle_verify(dq_slave_t *s) {
    int res = s->cb_res;
    if (res <= 0) {
        if (s->verify_left > 0) { s->verify_left--; dq_issue_read_only(s, DQ_VERIFY); return; }
        LOG_WARN("dq", "slave %u verify exhausted — deferring gated resync", s->slave_id);
        dq_defer_resync(s, DQ_RSN_VERIFY);   /* adopt read gates fresh-vs-alive once it answers */
        return;
    }
    uint8_t free_slots  = (uint8_t)(cb_reg(s, 0) >> 8);
    uint8_t accepted_id = (uint8_t)(cb_reg(s, 0) & 0xFF);
    uint8_t completed   = (uint8_t)(cb_reg(s, 1) & 0xFF);
    s->last_accepted = accepted_id;
    if (dq_status_is_reboot(s, free_slots, accepted_id)) {
        dq_resync_apply(s, s->reboot_flag ? DQ_RSN_BOOTCOUNT : DQ_RSN_ANOMALY,
                        accepted_id, completed, free_slots);
        return;
    }
    if (s->reboot_flag) {
        dq_resync_apply(s, DQ_RSN_BOOTCOUNT, accepted_id, completed, free_slots);
        return;
    }
    /* Did the timed-out submit land? accepted_id equals submit_last_id iff so. */
    if (accepted_id == s->submit_last_id) dq_promote_submit(s);
    else                                  dq_rollback_submit(s);
    s->free_hint = free_slots;
    dq_collect(s, completed);
    dq_pace_next(s);
}

/* Boot-time adoption: sync our id sequence to the gateway's before the first
 * submit, so a controller-only reboot can't mis-collect fresh commands against
 * retained gateway state. */
static void dq_handle_adopt(dq_slave_t *s) {
    int res = s->cb_res;
    if (res <= 0) { s->next_at = make_timeout_time_ms(DQ_DRAIN_POLL_MS); s->st = DQ_IDLE; return; }
    uint8_t free_slots  = (uint8_t)(cb_reg(s, 0) >> 8);
    uint8_t accepted_id = (uint8_t)(cb_reg(s, 0) & 0xFF);
    uint8_t completed   = (uint8_t)(cb_reg(s, 1) & 0xFF);
    s->last_accepted  = accepted_id;
    /* A deferred gated resync (idle boot_count change / verify-exhaust) reuses this
     * adoption read for its fresh-vs-alive decision. */
    if (s->resync_pending) {
        dq_resync_apply(s, (dq_resync_reason_t)s->resync_reason,
                        accepted_id, completed, free_slots);
        return;
    }
    s->next_id        = (accepted_id == 0) ? 1u : next_gw_id(accepted_id);
    s->last_completed = completed;
    s->free_hint      = free_slots;
    s->adopted        = true;
    s->c_adopts++;
    s->st             = DQ_IDLE;
    s->next_at        = get_absolute_time();
}

void dq_tick(void) {
    for (int i = 0; i < MAX_DALI_SLAVES; i++) {
        dq_slave_t *s = &s_dq[i];
        if (!s->in_use) continue;

        if (s->st != DQ_IDLE) {
            if (!s->cb_fired) continue;
            s->cb_fired = false;
            if      (s->st == DQ_VERIFY) dq_handle_verify(s);
            else if (s->st == DQ_ADOPT)  dq_handle_adopt(s);
            else                         dq_handle_await(s);
            continue;
        }

        if (s->reboot_flag) { dq_defer_resync(s, DQ_RSN_BOOTCOUNT); continue; }
        if (!time_reached(s->next_at)) continue;

        /* Provisioning is highest priority AND bus-exclusive: while a slave is
         * being commissioned (the 0x1F04 commission SM), the scheduler admits
         * nothing else — user/config/background commands accumulate in the
         * backlog and drain once provisioning finishes. */
        if (dali_provision_busy_slave(s->slave_id)) {
            s->next_at = make_timeout_time_ms(DQ_DRAIN_POLL_MS);
            continue;
        }

        uint8_t pending = (uint8_t)(s->count - s->outstanding);
        /* Nothing to do — unless a deferred resync (or a leftover tentative submit)
         * still needs an adoption read to reconcile against the gateway. */
        if (pending == 0 && s->outstanding == 0 && s->submit_n == 0 && !s->resync_pending)
            continue;

        if (!s->adopted) { dq_issue_read_only(s, DQ_ADOPT); continue; }

        /* Strict credits + depth cap: submit min(pending, free_hint,
         * DQ_INFLIGHT_MAX - outstanding, DQ_MAX_SUBMIT). n==0 falls through to a
         * collect-only read to reap results / refresh credit. */
        uint8_t room = (s->outstanding < DQ_INFLIGHT_MAX)
                       ? (uint8_t)(DQ_INFLIGHT_MAX - s->outstanding) : 0u;
        uint8_t n = pending;
        if (n > s->free_hint)  n = s->free_hint;
        if (n > room)          n = room;
        if (n > DQ_MAX_SUBMIT) n = DQ_MAX_SUBMIT;
        dq_issue(s, n);
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
                   dq_sync_cb, NULL))
        return false;   /* backlog full */
    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);
    while (!s_sync_have && absolute_time_diff_us(get_absolute_time(), deadline) > 0) {
        modbus_poll();
        dq_tick();
    }
    if (!s_sync_have) return false;
    if (result) *result = s_sync_res;
    if (answer) *answer = s_sync_ans;
    return true;
}

/* ── `dq` CLI observability ───────────────────────────────────────────────── */

static const char *dq_prio_name(uint8_t p) {
    switch (p) {
        case DQ_PRIO_PROVISION:    return "prov";
        case DQ_PRIO_USER:         return "user";
        case DQ_PRIO_GROUP_WRITE:  return "gwrite";
        case DQ_PRIO_CONFIG_WRITE: return "cfgw";
        case DQ_PRIO_LEVEL_READ:   return "lread";
        case DQ_PRIO_BACKGROUND:   return "bg";
        default:                   return "?";
    }
}

static const char *dq_state_name(dq_state_t st) {
    switch (st) {
        case DQ_IDLE:   return "idle";
        case DQ_AWAIT:  return "await";
        case DQ_VERIFY: return "verify";
        case DQ_ADOPT:  return "adopt";
        default:        return "?";
    }
}

void dq_debug_print(int slave_id) {
    bool any = false;
    for (int i = 0; i < MAX_DALI_SLAVES; i++) {
        dq_slave_t *s = &s_dq[i];
        if (!s->in_use) continue;
        if (slave_id >= 0 && s->slave_id != (uint8_t)slave_id) continue;
        any = true;
        uint8_t pending = (uint8_t)(s->count - s->outstanding);
        uint8_t cls[DQ_PRIO_COUNT] = {0};
        for (uint8_t j = s->outstanding; j < s->count; j++)
            if (s->bl[j].prio < DQ_PRIO_COUNT) cls[s->bl[j].prio]++;
        printf("dq slave %u: st=%s adopted=%s next_id=%u free_hint=%u\n",
               s->slave_id, dq_state_name(s->st), s->adopted ? "y" : "n",
               s->next_id, s->free_hint);
        printf("  accepted(last)=%u completed(last)=%u inflight=%u/%u pending=%u boot=%s:%u\n",
               s->last_accepted, s->last_completed, s->outstanding, DQ_INFLIGHT_MAX,
               pending, s->boot_valid ? "y" : "n", s->boot_count);
        printf("  inflight:");
        for (uint8_t j = 0; j < s->outstanding; j++)
            printf(" %u:%s", s->bl[j].id, dq_prio_name(s->bl[j].prio));
        printf("\n  queued/class: prov=%u user=%u gwrite=%u cfgw=%u lread=%u bg=%u\n",
               cls[DQ_PRIO_PROVISION], cls[DQ_PRIO_USER], cls[DQ_PRIO_GROUP_WRITE],
               cls[DQ_PRIO_CONFIG_WRITE], cls[DQ_PRIO_LEVEL_READ], cls[DQ_PRIO_BACKGROUND]);
        printf("  counters: submitted=%lu collected=%lu timeouts=%lu resubmits=%lu "
               "busy=%lu adopts=%lu resyncs=%lu(boot=%lu,anom=%lu)\n",
               (unsigned long)s->c_submitted, (unsigned long)s->c_collected,
               (unsigned long)s->c_timeouts, (unsigned long)s->c_resubmits,
               (unsigned long)s->c_busy, (unsigned long)s->c_adopts,
               (unsigned long)s->c_resyncs, (unsigned long)s->c_resync_boot,
               (unsigned long)s->c_resync_anom);
    }
    if (!any) printf("dq: no matching DALI slave\n");
}
