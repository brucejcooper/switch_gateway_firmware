/*
 * mbq_client.c — controller-side transport for the gateway's DALI command
 * queue, protocol v4. See mbq_client.h for the model and
 * TinyModbus/QUEUE-PROTOCOL.md (normative) for the wire format.
 *
 * One slave_t per Modbus slave, each a single-outstanding-transaction FSM over
 * up to CMDQ_DEPTH commands. SINGLE OUTSTANDING PER SLAVE IS LOAD-BEARING: it
 * is what makes "the next seq the gateway assigns is last_seq + 1" a safe
 * prediction, and R1 depends on that prediction.
 *
 *   VERSION ─→ DRAIN ─→ BOOT_CHECK ─→ IDLE ⇄ AWAIT_REPLY ─→ resolve
 *                                      ↑          │
 *                          DEADLINE_WAIT ┘        └→ RECOVERY_POLL ─→ resolve
 *                                                        └→ BOOT_CHECK
 *   any state ─→ INCOMPATIBLE  (the §8 compat latch, terminal)
 *
 * The first three states run once per slave at registration: identify the
 * firmware, then clear anything a previous controller epoch left in the
 * gateway's response queue (spec §4, "controller reboot"), then establish the
 * boot_count the recovery rules classify against. Only after that is the
 * "nothing in the response queue that isn't mine" invariant true, and that
 * invariant is what lets the ack-less corner adopt an orphan head as its own.
 *
 * ACK DISCIPLINE (spec §5): every request carries the ack for the most recent
 * result consumed, and keeps carrying it until a reply proves the head moved.
 * An ack that does not match the head is a no-op at the gateway, so re-acking
 * costs nothing and converges by itself — and it is what keeps R1 available,
 * because R1 reads the head's movement as proof the lost frame landed.
 */

#include "mbq_client.h"
#include "modbus.h"
#include "config.h"   /* MAX_DALI_SLAVES */
#include "log.h"
#include "cdc_out.h"
#include <string.h>

#ifdef MBQ_CLIENT_HOST_TEST
/* Host-test seam (test/mbq_client_test.c supplies this). ONLY the clock is
 * redirected. The Modbus entry points below are ordinary extern functions, so
 * the test links its own definitions of them instead of modbus.c and every
 * line of protocol logic in this file is the production one — there is no
 * second implementation of a recovery rule to drift out of sync. */
uint64_t mbq_test_now_us(void);
#define MBQ_NOW_US()  mbq_test_now_us()
#else
#include "pico/time.h"
#define MBQ_NOW_US()  to_us_since_boot(get_absolute_time())
#endif

/* ── wire constants (spec §1) ─────────────────────────────────────────────── */

#define MBQ_FC          0x41u
#define SEQ_MASK        0x1Fu   /* sequences are 1-31; 0 is reserved for "none" */

enum { A_ACCEPTED = 0u, A_FULL = 1u };   /* accept-byte status */
enum { H_OK = 0u, H_FAILED = 1u };       /* head-entry status  */

/* Cold-path gateway registers (controller CLAUDE.md, "Gateway sysinfo"). Both
 * blocks are exact-count reads — a different count answers ILLEGAL DATA
 * ADDRESS rather than a short read. */
#define GW_VERSION_ADDR  0x0200u
#define GW_VERSION_REGS  16u
#define GW_SYSINFO_ADDR  0x0310u
#define GW_SYSINFO_REGS  4u

/* ── policy constants ─────────────────────────────────────────────────────── */

/* Estimated DALI execution time per command class (task-07 measurements), used
 * only to PACE — spinning a poll loop at main-loop speed against a gateway
 * that cannot answer for another 25 ms wastes shared bus for nothing (measured
 * ~2.4 polls per frame in the FC23 soak). A wrong estimate costs a little
 * latency or a little bus, never correctness: the result deadline, not this,
 * is what decides a command is late. */
#define EST_SET_MS       25u
#define EST_QUERY_MS     40u
#define EST_DTR0_MS      50u

#define PACE_MIN_MS       2u    /* floor, so an estimate that has already passed
                                 * still yields to the rest of the main loop   */
#define PACE_FAIL_MS      5u    /* after a transaction that got no reply        */
#define PACE_DOWN_MS   1000u    /* ...once the link looks dead (below)          */
#define LINK_DOWN_FAILS   3u

/* A command may go back on the wire this many times (A_FULL retries excluded,
 * they are not resends of a landed command) before the client gives up and
 * reports it LOST. Without a cap, an idempotent command against a gateway that
 * is answering but never completing anything resubmits forever and the queue
 * never drains — R4 would fire, resubmit, fire again. */
#define MAX_RESENDS       3u

/* Bound on the registration drain. RESPQ_DEPTH is 4, so five polls empty a
 * full queue; a gateway still producing heads after that is not draining and
 * the client proceeds anyway (with the adopt rule disabled) rather than
 * refusing to ever talk to it. */
#define DRAIN_MAX_POLLS   6u

/* §8 asks the controller to refuse a non-matching queue generation at
 * discovery. FC04 0x0200 is a free-form STRING ("DALI v<maj>.<min> <serial>")
 * with no queue-generation field, and the spec never states which version
 * denotes v4 — so the version is parsed, recorded and reported here, but the
 * HARD latch is behavioural: an ILLEGAL FUNCTION reply to FC 0x41, or a reply
 * of a length the request did not call for. Those cannot false-positive,
 * whereas a guessed version constant that does not match what the gateway
 * actually reports would silently stop driving every gateway on the bus. Set
 * these to the shipped v4 gateway's version and flip GW_VER_LATCHES to 1. */
#define GW_MIN_MAJOR      1u
#define GW_MIN_MINOR      0u
#define GW_VER_LATCHES    0

/* ── per-slave state ──────────────────────────────────────────────────────── */

typedef enum {
    CMD_FREE = 0,
    CMD_PENDING,    /* held here, waiting to be (re)sent — nothing at the gateway */
    CMD_SENDING,    /* its enqueue is on the wire or in recovery; seq not yet known */
    CMD_ACCEPTED,   /* bound to a gateway seq; awaiting its result */
} cmd_state_t;

typedef struct {
    uint8_t  cmd[MBQ_CMD_BYTES];   /* addr, data, flags (wire bits), dtr0 */
    mbq_cb_t cb;
    void    *ctx;
    bool     idempotent;
    uint8_t  st;                   /* cmd_state_t */
    uint8_t  seq;                  /* gateway-assigned; 0 until accepted */
    uint8_t  resends;
    uint64_t deadline_us;          /* result deadline, armed at accept */
    uint64_t est_done_us;          /* estimated DALI completion, for pacing */
} cmd_t;

typedef enum {
    S_VERSION = 0,    /* reading the firmware version block                  */
    S_DRAIN,          /* poll + ack until the response queue is empty        */
    S_BOOT_CHECK,     /* reading sysinfo boot_count                          */
    S_IDLE,           /* nothing accepted, nothing to send                   */
    S_AWAIT_REPLY,    /* a queue request is on the wire                      */
    S_RECOVERY_POLL,  /* resolving a lost enqueue reply (R1 / R1' / ack-less)*/
    S_DEADLINE_WAIT,  /* results outstanding; the next poll is paced         */
    S_INCOMPATIBLE,   /* compat latch tripped — this slave is not driven     */
} mbq_state_t;

/* What a boot_count read is being used to decide. */
typedef enum {
    BC_BASELINE = 0,  /* registration: just establish the reference           */
    BC_R4,            /* R4: a result deadline expired — reboot or wedge?     */
    BC_ACKLESS,       /* the ack-less corner, before resending a non-idempotent */
} bc_reason_t;

typedef enum { TX_QUEUE = 0, TX_SYSINFO, TX_VERSION } tx_kind_t;

#define NO_CMD 0xFFu

typedef struct {
    bool     in_use;
    uint8_t  slave_id;
    uint8_t  st;                /* mbq_state_t */
    bool     busy;              /* a transaction is on the wire */

    cmd_t    cmd[CMDQ_DEPTH];

    /* What the in-flight request was, so its reply can be interpreted. */
    uint8_t  tx_kind;           /* tx_kind_t */
    uint8_t  tx_cmd;            /* command this request enqueues, or NO_CMD */
    uint8_t  tx_ack;            /* ack value it carried */
    uint8_t  tx_explen;         /* reply length this request called for: 6 or 7 */
    uint8_t  tx_deadline_seq;   /* R4 poll: the seq whose deadline expired, else 0 */

    /* Protocol state. */
    uint8_t  ack_pending;       /* result consumed; ack not yet proven applied */
    uint8_t  last_head;         /* hseq of the most recent reply */
    uint8_t  last_seq;          /* most recently accepted seq */
    bool     seq_known;         /* last_seq is meaningful in the current epoch */
    bool     epoch_clean;       /* the drain succeeded: nothing in the gateway's
                                 * response queue predates this controller      */

    /* Recovery bookkeeping (§4). */
    uint8_t  rec_cmd;           /* command whose reply was lost, or NO_CMD */
    uint8_t  rec_ack;           /* the ack that lost frame carried; 0 = ack-less */
    uint64_t rec_deadline_us;   /* bound on the ack-less poll loop */

    /* boot_count arbiter. */
    uint8_t  bc_reason;         /* bc_reason_t for the read in flight */
    bool     bc_valid;
    uint16_t bc_last;

    uint8_t  drain_polls;
    uint8_t  gw_major, gw_minor;

    /* Two pacing gates, deliberately separate. next_at_us holds back work the
     * client OWES the gateway (a resend, a retry after a failure, an A_FULL
     * re-offer); poll_at_us holds back the speculative POLL that collects a
     * result. Merging them would make an R1' resend wait out a DALI frame it
     * has no reason to wait for, and would let a freshly submitted command sit
     * behind a poll timer — which is the throughput this design exists to
     * gain. A fresh submit is gated by neither (spec §5 paces "on A_FULL, or
     * when awaiting results with an empty backlog", and a submit is neither). */
    uint64_t next_at_us;
    uint64_t poll_at_us;

    /* Modbus completion hand-off (resolved in tick, not in the callback). */
    volatile bool cb_fired;
    int      cb_res;
    uint8_t  cb_buf[40];
    uint8_t  cb_len;

    uint16_t drop_replies;      /* bench drill knob (C3) */

    uint64_t t_issue_us;
    uint32_t w_n, w_min, w_max, w_fail, w_busy;
    uint64_t w_sum;
    uint16_t c_consec_fail;

    mbq_stats_t stats;
} slave_t;

static slave_t s_sl[MAX_DALI_SLAVES];

/* Re-entrancy guard. Terminal callbacks run from inside reply resolution, and
 * a producer is entitled to submit from one; letting that submit issue a
 * transaction while the reply that triggered it is still half-applied would
 * interleave two epochs of state. Refusing it costs the producer one tick. */
static bool s_resolving;

/* ── helpers ─────────────────────────────────────────────────────────────── */

static slave_t *slave_for(uint8_t slave_id) {
    for (int i = 0; i < MAX_DALI_SLAVES; i++)
        if (s_sl[i].in_use && s_sl[i].slave_id == slave_id) return &s_sl[i];
    return NULL;
}

/* Sequences run 1-31 and skip 0, which means "none". An off-by-one here makes
 * "no ack" and "ack command 32" the same frame. */
static uint8_t seq_next(uint8_t s) { return (s >= SEQ_MASK) ? 1u : (uint8_t)(s + 1u); }

static uint32_t est_ms(const uint8_t cmd[MBQ_CMD_BYTES]) {
    if (cmd[2] & MBQ_QF_DTR0)   return EST_DTR0_MS;
    if (cmd[2] & MBQ_QF_ANSWER) return EST_QUERY_MS;
    return EST_SET_MS;
}

static cmd_t *cmd_by_seq(slave_t *s, uint8_t seq) {
    if (seq == 0) return NULL;
    for (uint8_t i = 0; i < CMDQ_DEPTH; i++)
        if (s->cmd[i].st == CMD_ACCEPTED && s->cmd[i].seq == seq) return &s->cmd[i];
    return NULL;
}

/* "Still tracks" for R2/R3: a seq the gateway is not free to reuse as far as
 * this master is concerned — one awaiting its result, or one whose result was
 * consumed but whose ack has not been proven applied. */
static bool seq_tracked(slave_t *s, uint8_t seq) {
    if (seq == 0) return false;
    if (s->ack_pending == seq) return true;
    return cmd_by_seq(s, seq) != NULL;
}

static uint8_t count_state(const slave_t *s, cmd_state_t st) {
    uint8_t n = 0;
    for (uint8_t i = 0; i < CMDQ_DEPTH; i++) if (s->cmd[i].st == st) n++;
    return n;
}

static uint8_t find_state(const slave_t *s, cmd_state_t st) {
    for (uint8_t i = 0; i < CMDQ_DEPTH; i++) if (s->cmd[i].st == st) return i;
    return NO_CMD;
}

static uint8_t inflight_count(const slave_t *s) {
    uint8_t n = 0;
    for (uint8_t i = 0; i < CMDQ_DEPTH; i++) if (s->cmd[i].st != CMD_FREE) n++;
    return n;
}

static void pace(slave_t *s, uint32_t ms) {
    if (ms < PACE_MIN_MS) ms = PACE_MIN_MS;
    s->next_at_us = MBQ_NOW_US() + (uint64_t)ms * 1000u;
}

/* When the OLDEST in-flight command is estimated to finish (spec §5) — the
 * moment at which a poll has something to collect. Capped at the result
 * deadline so a bad estimate can never delay the deadline machinery past the
 * point where it should fire, and floored so it always yields to the rest of
 * the main loop. */
static uint64_t oldest_done_at(const slave_t *s) {
    uint64_t now = MBQ_NOW_US(), target = 0;
    for (uint8_t i = 0; i < CMDQ_DEPTH; i++) {
        if (s->cmd[i].st != CMD_ACCEPTED) continue;
        if (target == 0 || s->cmd[i].est_done_us < target) target = s->cmd[i].est_done_us;
    }
    uint64_t floor_us = now + (uint64_t)PACE_MIN_MS * 1000u;
    uint64_t cap      = now + (uint64_t)MBQ_RESULT_DEADLINE_MS * 1000u;
    if (target == 0 || target < floor_us) return floor_us;
    return (target > cap) ? cap : target;
}

static void deliver(slave_t *s, cmd_t *c, mbq_result_t r, uint8_t resp8) {
    mbq_cb_t cb = c->cb;
    void *ctx = c->ctx;
    c->st = CMD_FREE;      /* free before the callback: it may submit again */
    c->cb = NULL;
    c->seq = 0;
    switch (r) {
        case MBQ_R_COMPLETE:    s->stats.completed++;   break;
        case MBQ_R_FAILED:      s->stats.failed++;      break;
        case MBQ_R_RESULT_LOST: s->stats.result_lost++; break;
        default:                s->stats.lost++;        break;
    }
    if (cb) cb(s->slave_id, r, resp8, ctx);
}

/* Put a command back on the wire, or give up on it. Shared by R1', A_FULL and
 * every reboot/loss disposal, so the resend cap is enforced in exactly one
 * place. */
static void resend_or_lose(slave_t *s, cmd_t *c, mbq_result_t give_up_as) {
    if (c->resends >= MAX_RESENDS) {
        LOG_WARN("mbq", "slave %u: command %02X%02X resent %u times without a "
                 "result — giving up", s->slave_id, c->cmd[0], c->cmd[1],
                 (unsigned)c->resends);
        deliver(s, c, give_up_as, 0);
        return;
    }
    c->resends++;
    c->seq = 0;
    c->st  = CMD_PENDING;
    s->stats.resends++;
}

/* Disposition after a reboot or an unrecoverable loss (spec §4). Idempotent
 * commands (queries, absolute SETs) resubmit automatically; non-idempotent
 * ones (UP/DOWN steps) are reported to their submitter, because whether they
 * executed is unknowable and pretending otherwise is how v1 corrupted itself.
 *
 * LOST vs RESULT_LOST for the non-idempotent case is decided by the result
 * deadline: a command whose deadline had already passed has, by the deadline's
 * own construction (1.5x the worst-case execution at depth 2), executed — so
 * its answer died with the gateway's response queue rather than the command
 * never having run. The spec asserts the distinction without giving a rule;
 * this is the only observable that bears on it. */
static void dispose_tracked(slave_t *s, const cmd_t *keep) {
    uint64_t now = MBQ_NOW_US();
    for (uint8_t i = 0; i < CMDQ_DEPTH; i++) {
        cmd_t *c = &s->cmd[i];
        if (c == keep) continue;
        if (c->st != CMD_ACCEPTED && c->st != CMD_SENDING) continue;
        if (c->idempotent) {
            resend_or_lose(s, c, MBQ_R_LOST);
        } else if (c->st == CMD_ACCEPTED && now >= c->deadline_us) {
            deliver(s, c, MBQ_R_RESULT_LOST, 0);
        } else {
            deliver(s, c, MBQ_R_LOST, 0);
        }
    }
}

/* A gateway reboot has been established by rule `rule` ('2', '3' or '4'). The
 * epoch is gone: no seq the master remembers means anything any more. */
static void note_reboot(slave_t *s, char rule, const cmd_t *keep) {
    switch (rule) {
        case '2': s->stats.reboots_r2++; break;
        case '3': s->stats.reboots_r3++; break;
        default:  s->stats.reboots_r4++; break;
    }
    uint8_t n = (uint8_t)(inflight_count(s) - (keep ? 1u : 0u));
    LOG_WARN("mbq", "slave %u: gateway reboot inferred by R%c — disposing %u tracked",
             s->slave_id, rule, (unsigned)n);
    dispose_tracked(s, keep);
    s->ack_pending = 0;
    s->last_head   = 0;
    s->rec_cmd     = NO_CMD;
    if (keep) {
        s->last_seq  = keep->seq;
        s->seq_known = true;
    } else {
        s->last_seq  = 0;
        s->seq_known = false;
    }
}

/* The compat latch (spec §8). Terminal per slave: stop driving its queue and
 * say so once, loudly, rather than limping and filling the log forever. */
static void latch_incompatible(slave_t *s, const char *why) {
    if (s->st == S_INCOMPATIBLE) return;
    s->st = S_INCOMPATIBLE;
    s->stats.incompatible = true;
    LOG_ERR("mbq", "slave %u INCOMPATIBLE (%s) — queue disabled. Controller and "
            "gateway are a matched set (queue v4); reflash the gateway.",
            s->slave_id, why);
    for (uint8_t i = 0; i < CMDQ_DEPTH; i++)
        if (s->cmd[i].st != CMD_FREE) deliver(s, &s->cmd[i], MBQ_R_LOST, 0);
}

/* ── Modbus hand-off ─────────────────────────────────────────────────────── */

static void mbq_modbus_cb(int res, uint8_t *buf, void *ctx) {
    slave_t *s = (slave_t *)ctx;
    uint64_t dt = MBQ_NOW_US() - s->t_issue_us;
    uint32_t dt32 = (dt > 0xFFFFFFFFull) ? 0xFFFFFFFFu : (uint32_t)dt;
    if (s->w_n == 0 || dt32 < s->w_min) s->w_min = dt32;
    if (dt32 > s->w_max) s->w_max = dt32;
    s->w_sum += dt32;
    s->w_n++;
    if (res <= 0) {
        s->w_fail++;
        /* Link liveness: a Modbus exception is still an ANSWER — the gateway is
         * alive and disagreeing — so only a genuine no-reply counts against it. */
        if (res == 0 && s->c_consec_fail < 0xFFFFu) s->c_consec_fail++;
    } else {
        s->c_consec_fail = 0;
    }
    s->cb_res = res;
    s->cb_len = 0;
    if (buf && res != 0) {
        int n = (res < 0) ? -res : res;
        if (n > (int)sizeof s->cb_buf) n = (int)sizeof s->cb_buf;
        memcpy(s->cb_buf, buf, (size_t)n);
        s->cb_len = (uint8_t)n;
    }
    s->cb_fired = true;
}

/* Issue one v4 request: `ci` is the command to enqueue (NO_CMD for a pure
 * ack/poll) and the ack is always the outstanding one (spec §5, always
 * piggyback). Returns false if the Modbus action queue was full — the caller
 * leaves its state alone and retries. */
static bool issue_queue_txn_acked(slave_t *s, uint8_t ci, bool with_ack) {
    uint8_t payload[3];
    uint8_t plen = 0;
    uint8_t ctl  = with_ack ? (uint8_t)(s->ack_pending & SEQ_MASK) : 0u;

    if (ci != NO_CMD) {
        const uint8_t *c = s->cmd[ci].cmd;
        ctl |= (uint8_t)(c[2] & MBQ_QF_MASK);
        payload[0] = c[0];
        payload[1] = c[1];
        plen = 2;
        if (c[2] & MBQ_QF_DTR0) { payload[2] = c[3]; plen = 3; }
    }

    s->cb_fired  = false;
    s->t_issue_us = MBQ_NOW_US();
    if (!modbus_mbq_v4(s->slave_id, ctl, payload, plen, mbq_modbus_cb, s)) {
        pace(s, PACE_FAIL_MS);
        return false;
    }
    s->tx_kind   = TX_QUEUE;
    s->tx_cmd    = ci;
    s->tx_ack    = (uint8_t)(ctl & SEQ_MASK);
    s->tx_explen = (plen ? 7u : 6u);
    s->busy      = true;
    if (ci != NO_CMD) {
        s->cmd[ci].st = CMD_SENDING;
        s->stats.enqueues++;
    } else {
        s->stats.polls++;
    }
    return true;
}

static bool issue_queue_txn(slave_t *s, uint8_t ci) {
    return issue_queue_txn_acked(s, ci, true);
}

static bool issue_read(slave_t *s, uint8_t kind, uint16_t addr, uint16_t regs) {
    s->cb_fired   = false;
    s->t_issue_us = MBQ_NOW_US();
    if (!modbus_get_input_registers(s->slave_id, addr, regs, mbq_modbus_cb, s)) {
        pace(s, PACE_FAIL_MS);
        return false;
    }
    s->tx_kind = kind;
    s->tx_cmd  = NO_CMD;
    s->busy    = true;
    return true;
}

/* ── reply parsing ───────────────────────────────────────────────────────── */

typedef struct {
    bool    has_accept;
    uint8_t astatus, aseq;
    uint8_t hstatus, hseq, resp8;
} reply_t;

/* Decode a v4 reply. `n` is the whole frame including its CRC bytes, which the
 * transport has already validated. The expected LENGTH is decided by the
 * request we sent, not by what came back — 6 for an ack/poll, 7 when an
 * enqueue was attempted — so a mismatch is a protocol error and not something
 * to interpret. Returns false on any structural fault; the caller latches. */
static bool parse_reply(const slave_t *s, const uint8_t *b, uint8_t n, reply_t *r) {
    if (n != s->tx_explen) return false;
    if (b[0] != s->slave_id || b[1] != MBQ_FC) return false;
    memset(r, 0, sizeof *r);
    uint8_t i = 2;
    if (s->tx_explen == 7u) {
        r->has_accept = true;
        r->astatus = (uint8_t)(b[i] >> 5);
        r->aseq    = (uint8_t)(b[i] & SEQ_MASK);
        i++;
    }
    r->hstatus = (uint8_t)(b[i] >> 5);
    r->hseq    = (uint8_t)(b[i] & SEQ_MASK);
    i++;
    r->resp8 = b[i];
    /* An empty head must carry nothing; a non-empty one must carry a defined
     * status. Both are cheap structural checks on bytes we are about to act on. */
    if (r->hseq == 0 && (r->hstatus != 0 || r->resp8 != 0)) return false;
    if (r->hstatus > H_FAILED) return false;
    return true;
}

/* ── head processing ─────────────────────────────────────────────────────── */

/* Apply the head field of a reply. Returns true if a reboot was inferred here
 * (R3), so the caller does not then apply a second rule to the same reply. */
static bool apply_head(slave_t *s, const reply_t *r) {
    /* The head cannot change except by an ack that matches it, so a head that
     * moved without an ack of ours to explain the move is structurally
     * impossible. Under the always-piggyback policy of §5 this is unreachable
     * (the ack for a head we consumed rides the very next request), which is
     * why it is counted rather than acted on — if it ever fires, the policy or
     * the gateway has changed and that is the finding.
     *
     * Not during recovery: there the head moving is the WHOLE POINT. R1's poll
     * deliberately carries no ack precisely so that a moved head can only mean
     * the lost frame's own ack was applied — an ack of ours that we simply
     * never saw acknowledged. Counting that as an anomaly would flag every
     * successful R1. */
    if (s->st != S_RECOVERY_POLL && s->last_head != 0 &&
        r->hseq != s->last_head && s->tx_ack != s->last_head)
        s->stats.head_anomalies++;

    /* Ack confirmation: the head moving off the seq we were acking proves the
     * ack was applied. While it has not moved, keep carrying the same ack. */
    if (s->ack_pending != 0 && r->hseq != s->ack_pending) s->ack_pending = 0;

    s->last_head = r->hseq;

    if (r->hseq == 0 || r->hseq == s->ack_pending) return false;  /* nothing new */

    cmd_t *c = cmd_by_seq(s, r->hseq);
    if (!c) {
        /* R3: an unacked head seq the master is not tracking has no other
         * source than a gateway that restarted its counter. */
        note_reboot(s, '3', NULL);
        /* Pop the orphan so it cannot block the queue behind it. */
        s->ack_pending = r->hseq;
        s->last_head   = r->hseq;
        return true;
    }

    if (r->hstatus == H_OK) deliver(s, c, MBQ_R_COMPLETE, r->resp8);
    else                    deliver(s, c, MBQ_R_FAILED,   r->resp8);
    s->ack_pending = r->hseq;
    return false;
}

/* Bind a command to the seq the gateway assigned it and start its clocks. */
static void bind_accepted(slave_t *s, cmd_t *c, uint8_t seq) {
    uint64_t now = MBQ_NOW_US();
    /* Pipeline model for the pacing estimate: this command starts when the one
     * ahead of it finishes, since the gateway executes strictly FIFO. */
    uint64_t start = now;
    for (uint8_t i = 0; i < CMDQ_DEPTH; i++) {
        cmd_t *o = &s->cmd[i];
        if (o == c || o->st != CMD_ACCEPTED) continue;
        if (o->est_done_us > start) start = o->est_done_us;
    }
    c->seq         = seq;
    c->st          = CMD_ACCEPTED;
    c->deadline_us = now + (uint64_t)MBQ_RESULT_DEADLINE_MS * 1000u;
    c->est_done_us = start + (uint64_t)est_ms(c->cmd) * 1000u;
    s->last_seq    = seq;
    s->seq_known   = true;
}

/* ── resolution: a queue reply arrived ───────────────────────────────────── */

static void resolve_queue_reply(slave_t *s, const reply_t *r) {
    bool rebooted = false;

    if (r->has_accept) {
        cmd_t *c = (s->tx_cmd != NO_CMD) ? &s->cmd[s->tx_cmd] : NULL;
        if (!c) { latch_incompatible(s, "accept byte for a request that enqueued nothing"); return; }

        if (r->astatus == A_FULL) {
            /* Not enqueued. The ack in the same frame WAS processed, so the head
             * below is still meaningful; only the command goes back. */
            s->stats.afull++;
            c->st = CMD_PENDING;
            c->seq = 0;
            /* Pace the retry: the queue is full because DALI is busy, and
             * re-offering at main-loop speed would burn shared bus to be told
             * so again. */
            s->next_at_us = oldest_done_at(s);
        } else if (r->astatus == A_ACCEPTED && r->aseq != 0) {
            /* R2: the gateway pins a seq until its result is acked, and acks are
             * head-only and in order, so reaching the same value again needs 31
             * further accepts and at least 27 acks — which would have drained the
             * pinned entry first. An accept byte carrying a seq we still track is
             * therefore proof the gateway restarted its counter. */
            if (seq_tracked(s, r->aseq)) {
                bind_accepted(s, c, r->aseq);
                note_reboot(s, '2', c);
                rebooted = true;
            } else {
                bind_accepted(s, c, r->aseq);
            }
        } else {
            latch_incompatible(s, "reserved accept status");
            return;
        }
    }

    if (rebooted) {
        /* The epoch is gone; the only meaningful head now is one belonging to
         * the command we just kept, which cannot have completed yet. */
        s->last_head = r->hseq;
        if (r->hseq != 0) s->ack_pending = r->hseq;   /* pop whatever is there */
        return;
    }

    apply_head(s, r);
}

/* ── resolution: a recovery poll's reply arrived (§4) ────────────────────── */

static void resolve_recovery_reply(slave_t *s, const reply_t *r) {
    cmd_t *c = (s->rec_cmd != NO_CMD) ? &s->cmd[s->rec_cmd] : NULL;
    if (!c || c->st != CMD_SENDING) { s->rec_cmd = NO_CMD; apply_head(s, r); return; }

    if (s->rec_ack != 0) {
        /* R1 / R1'. The lost frame carried both an enqueue and an ack, and they
         * travelled in ONE frame — so whether the ack was applied is an atomic
         * landed/not-landed indicator for the whole transaction. */
        if (r->hseq == s->rec_ack) {
            s->stats.r1_never++;                 /* R1': never arrived */
            s->rec_cmd = NO_CMD;
            /* Resend verbatim, ack included — no idempotency concern, because
             * the head proves the frame never reached the gateway. Through the
             * shared path so it is counted and subject to the same cap as
             * every other resend. */
            resend_or_lose(s, c, MBQ_R_LOST);
            apply_head(s, r);
            return;
        }
        /* R1: the head moved past the seq we acked, so the ack was applied, so
         * the frame landed. Its seq is the one we predicted — the single
         * outstanding transaction rule guarantees nothing else was accepted in
         * between. */
        s->stats.r1_landed++;
        s->rec_cmd = NO_CMD;
        bind_accepted(s, c, s->seq_known ? seq_next(s->last_seq) : r->hseq);
        apply_head(s, r);
        return;
    }

    /* The ack-less corner: no ack rode the lost frame, so R1 is unavailable.
     * Poll until the predicted seq appears as a result (landed), or the
     * deadline passes with the response queue observed empty (never landed —
     * retention means a result cannot have been and gone). */
    if (r->hseq != 0) {
        uint8_t predicted = s->seq_known ? seq_next(s->last_seq) : 0u;
        if (predicted != 0 && r->hseq == predicted) {
            s->stats.r1_landed++;
            s->rec_cmd = NO_CMD;
            bind_accepted(s, c, r->hseq);
            apply_head(s, r);                    /* the result is right there */
            return;
        }
        if (cmd_by_seq(s, r->hseq)) { apply_head(s, r); return; }   /* an earlier command's */
        if (!s->seq_known && s->epoch_clean && inflight_count(s) == 1u) {
            /* First enqueue of a clean epoch: the response queue was drained
             * empty and this is the only command outstanding, so nothing else
             * on this bus could have produced a head. It is ours. (The spec
             * says to match the "predicted seq", but on the first accept of an
             * epoch there is nothing to predict from — the gateway's counter is
             * its own. Exclusion is what identifies it instead.) */
            s->stats.r1_landed++;
            s->rec_cmd = NO_CMD;
            bind_accepted(s, c, r->hseq);
            apply_head(s, r);
            return;
        }
        apply_head(s, r);                        /* unknown seq → R3 fires in there */
        return;
    }

    /* Head empty. Keep polling until the deadline; only then is "empty" proof. */
    apply_head(s, r);
    if (MBQ_NOW_US() < s->rec_deadline_us) return;

    s->stats.deadlines++;
    if (c->idempotent) {
        s->rec_cmd = NO_CMD;
        resend_or_lose(s, c, MBQ_R_LOST);
        return;
    }
    /* Non-idempotent: "never landed" and "executed, then the result died with a
     * reboot" are both consistent with an empty queue, and only boot_count can
     * separate them. This is the entire residual cost of keeping the boot byte
     * off the hot path — one cold-path register read, on a double fault, for
     * one command class. */
    s->bc_reason = BC_ACKLESS;
    s->st = S_BOOT_CHECK;
}

/* ── resolution: cold-path reads ─────────────────────────────────────────── */

static void resolve_sysinfo(slave_t *s, const uint8_t *b, uint8_t n) {
    /* FC04 4-register frame: addr + fc + bytecount + 8 data + 2 CRC = 13. */
    if (n < 13u) { pace(s, PACE_FAIL_MS); s->st = S_IDLE; return; }
    uint16_t bc = (uint16_t)(((uint16_t)b[7] << 8) | b[8]);   /* reg2 = boot_count */
    bool changed = s->bc_valid && (bc != s->bc_last);
    bool first   = !s->bc_valid;
    s->bc_last  = bc;
    s->bc_valid = true;
    s->stats.boot_count = bc;

    cmd_t *c = (s->rec_cmd != NO_CMD) ? &s->cmd[s->rec_cmd] : NULL;

    switch (s->bc_reason) {
        case BC_BASELINE:
            s->st = S_IDLE;
            break;

        case BC_R4:
            if (changed || first) {
                note_reboot(s, '4', NULL);
            } else {
                /* Answering, but not completing: a gateway wedge, not a reboot.
                 * Dispose only the command that timed out. */
                LOG_WARN("mbq", "slave %u: result deadline expired with boot_count "
                         "unchanged (%u) — gateway wedge", s->slave_id, (unsigned)bc);
                for (uint8_t i = 0; i < CMDQ_DEPTH; i++) {
                    cmd_t *e = &s->cmd[i];
                    if (e->st == CMD_ACCEPTED && e->seq == s->tx_deadline_seq) {
                        if (e->idempotent) resend_or_lose(s, e, MBQ_R_LOST);
                        else               deliver(s, e, MBQ_R_RESULT_LOST, 0);
                        break;
                    }
                }
            }
            s->tx_deadline_seq = 0;
            s->st = S_IDLE;
            break;

        case BC_ACKLESS:
        default:
            s->rec_cmd = NO_CMD;
            if (!c) { s->st = S_IDLE; break; }
            if (changed || first) {
                /* "Executed, then the result died with the reboot" cannot be
                 * excluded, so the command is not repeated. */
                deliver(s, c, MBQ_R_LOST, 0);
                if (changed) note_reboot(s, '4', NULL);
            } else {
                resend_or_lose(s, c, MBQ_R_LOST);   /* genuinely never landed */
            }
            s->st = S_IDLE;
            break;
    }
}

/* The boot_count arbiter could not be read and the link is down. Every reason
 * we were reading it wanted to know "did this command run?", and the answer is
 * now unknowable — so take the disposition that assumes it might have. */
static void bootchk_give_up(slave_t *s) {
    LOG_WARN("mbq", "slave %u: boot_count unreadable — disposing conservatively",
             s->slave_id);
    s->bc_valid = false;
    if (s->bc_reason == BC_ACKLESS && s->rec_cmd != NO_CMD) {
        deliver(s, &s->cmd[s->rec_cmd], MBQ_R_LOST, 0);
        s->rec_cmd = NO_CMD;
    } else if (s->bc_reason == BC_R4) {
        dispose_tracked(s, NULL);
    }
    s->tx_deadline_seq = 0;
    s->st = S_IDLE;
}

static void resolve_version(slave_t *s, const uint8_t *b, uint8_t n) {
    /* "DALI v<maj>.<min> <20 hex chars>" in a 16-register block: addr + fc +
     * bytecount + 32 data + 2 CRC = 37 bytes. */
    if (n < 37u) { s->st = S_DRAIN; return; }   /* unreadable: let behaviour decide */
    const char *v = (const char *)(b + 3);
    unsigned maj = 0, min = 0;
    const char *p = NULL;
    for (uint8_t i = 0; i + 1u < 32u; i++)
        if (v[i] == 'v' && v[i + 1u] >= '0' && v[i + 1u] <= '9') { p = &v[i + 1u]; break; }
    if (p) {
        while (*p >= '0' && *p <= '9') maj = maj * 10u + (unsigned)(*p++ - '0');
        if (*p == '.') { p++; while (*p >= '0' && *p <= '9') min = min * 10u + (unsigned)(*p++ - '0'); }
        s->gw_major = (uint8_t)maj;
        s->gw_minor = (uint8_t)min;
        s->stats.gw_ver_major = s->gw_major;
        s->stats.gw_ver_minor = s->gw_minor;
        /* Packed compare, so the minimum can be set to a x.0 version without
         * the "unsigned < 0" that a two-term comparison would fold to. */
        bool too_old = (((maj << 8) | (min & 0xFFu))
                        < ((GW_MIN_MAJOR << 8) | GW_MIN_MINOR));
        if (too_old) {
            LOG_WARN("mbq", "slave %u reports gateway v%u.%u, queue v4 expects "
                     ">= v%u.%u — driving it anyway; a wrong-generation gateway "
                     "will be caught by its reply shape",
                     s->slave_id, maj, min, GW_MIN_MAJOR, GW_MIN_MINOR);
#if GW_VER_LATCHES
            latch_incompatible(s, "gateway firmware predates queue v4");
            return;
#endif
        } else {
            LOG_INFO("mbq", "slave %u gateway v%u.%u", s->slave_id, maj, min);
        }
    }
    s->st = S_DRAIN;
}

/* ── resolution: a transaction failed (no reply / exception) ─────────────── */

static void resolve_failure(slave_t *s) {
    s->stats.timeouts++;

    /* An ILLEGAL FUNCTION for FC 0x41 can only come from a pre-v4 gateway
     * (spec §1: this path raises no exceptions of its own). */
    if (s->cb_res < 0 && s->cb_len >= 3 && s->tx_kind == TX_QUEUE) {
        if (s->cb_buf[2] == 0x01u) { latch_incompatible(s, "ILLEGAL FUNCTION for FC 0x41"); return; }
        latch_incompatible(s, "unexpected Modbus exception on the queue path");
        return;
    }

    uint32_t ms = (s->c_consec_fail >= LINK_DOWN_FAILS) ? PACE_DOWN_MS : PACE_FAIL_MS;
    bool link_down = (s->c_consec_fail >= LINK_DOWN_FAILS);
    pace(s, ms);

    if (s->tx_kind == TX_VERSION) {
        s->st = S_DRAIN;   /* identification is advisory; do not block on it */
        return;
    }

    if (s->tx_kind == TX_SYSINFO) {
        /* The arbiter itself is unreachable. Retrying is right up to a point,
         * but a command cannot be left unresolved forever waiting for a gateway
         * that is gone — so once the link is declared down, classify the way an
         * unreadable boot_count demands: conservatively, as if it had changed. */
        if (link_down) bootchk_give_up(s);
        return;
    }

    if (s->tx_cmd == NO_CMD) {
        /* A poll or ack-only frame. Idempotent by construction — re-acking a
         * popped seq is a no-op — so there is nothing to resolve, just retry.
         * Release any deadline chase it was carrying, or the command it was
         * chasing would never be looked at again. */
        s->tx_deadline_seq = 0;
        if (s->st == S_DRAIN && ++s->drain_polls >= DRAIN_MAX_POLLS) {
            LOG_WARN("mbq", "slave %u: no answer to the epoch drain — proceeding "
                     "without a clean epoch", s->slave_id);
            s->epoch_clean = false;
            s->st = S_IDLE;
        }
        return;
    }

    /* An enqueue's reply was lost, and the transport did NOT re-send it (that
     * would be a second execution). Which of the two §4 paths applies is
     * decided by whether an ack rode along: with one, the head's movement
     * answers it in a single poll (R1/R1'); without one, only the deadline and
     * the response queue's emptiness can. */
    s->rec_cmd         = s->tx_cmd;
    s->rec_ack         = s->tx_ack;
    s->rec_deadline_us = MBQ_NOW_US() + (uint64_t)MBQ_RESULT_DEADLINE_MS * 1000u;
    s->st              = S_RECOVERY_POLL;
    pace(s, PACE_MIN_MS);
}

/* ── the tick ────────────────────────────────────────────────────────────── */

static void handle_reply(slave_t *s) {
    int res = s->cb_res;

    /* Bench drill knob: discard a parsed reply AFTER transport accounting and
     * BEFORE resolution, so the §4 paths run against real hardware exactly as
     * they would for a genuinely lost frame. */
    if (res > 0 && s->drop_replies > 0) {
        s->drop_replies--;
        LOG_WARN("mbq", "slave %u: dropping reply (drill, %u left)",
                 s->slave_id, (unsigned)s->drop_replies);
        res = 0;
        s->cb_len = 0;
    }

    if (res <= 0) { resolve_failure(s); return; }

    if (s->tx_kind == TX_SYSINFO) { resolve_sysinfo(s, s->cb_buf, s->cb_len); return; }
    if (s->tx_kind == TX_VERSION) { resolve_version(s, s->cb_buf, s->cb_len); return; }

    reply_t r;
    if (!parse_reply(s, s->cb_buf, s->cb_len, &r)) {
        s->stats.proto_errors++;
        latch_incompatible(s, "reply length or shape does not match the request");
        return;
    }

    switch (s->st) {
        case S_DRAIN:
            s->last_head = r.hseq;
            if (r.hseq == 0) {
                s->epoch_clean = true;
                s->stats.drained = true;
                s->ack_pending = 0;
                s->bc_reason = BC_BASELINE;
                s->st = S_BOOT_CHECK;
            } else if (++s->drain_polls >= DRAIN_MAX_POLLS) {
                LOG_WARN("mbq", "slave %u: response queue still not empty after %u "
                         "drain polls — proceeding without a clean epoch",
                         s->slave_id, (unsigned)s->drain_polls);
                s->ack_pending = r.hseq;
                s->bc_reason = BC_BASELINE;
                s->st = S_BOOT_CHECK;
            } else {
                s->ack_pending = r.hseq;   /* ack it on the next drain poll */
            }
            return;

        case S_RECOVERY_POLL:
            resolve_recovery_reply(s, &r);
            if (s->st == S_RECOVERY_POLL && s->rec_cmd == NO_CMD) s->st = S_IDLE;
            return;

        default:
            break;
    }

    resolve_queue_reply(s, &r);

    /* Whatever this reply settled, the next SPECULATIVE poll is worth issuing
     * only once the oldest in-flight command could plausibly have finished. A
     * submit arriving before then still goes out immediately — this gate is on
     * polls alone. */
    s->poll_at_us = oldest_done_at(s);

    /* An R4 poll that did not bring back the result it was chasing escalates to
     * the cold-path arbiter. */
    if (s->tx_deadline_seq != 0 && s->st != S_BOOT_CHECK) {
        if (cmd_by_seq(s, s->tx_deadline_seq)) {
            s->bc_reason = BC_R4;
            s->st = S_BOOT_CHECK;
        } else {
            s->tx_deadline_seq = 0;   /* it arrived after all */
        }
    }
}

/* Choose and issue this slave's next transaction. */
static void drive(slave_t *s) {
    uint64_t now = MBQ_NOW_US();

    switch (s->st) {
        case S_INCOMPATIBLE:
            return;

        case S_VERSION:
            if (now < s->next_at_us) return;
            issue_read(s, TX_VERSION, GW_VERSION_ADDR, GW_VERSION_REGS);
            return;

        case S_DRAIN:
            if (now < s->next_at_us) return;
            issue_queue_txn(s, NO_CMD);
            return;

        case S_BOOT_CHECK:
            if (now < s->next_at_us) return;
            s->stats.boot_reads++;
            issue_read(s, TX_SYSINFO, GW_SYSINFO_ADDR, GW_SYSINFO_REGS);
            return;

        case S_RECOVERY_POLL:
            if (now < s->next_at_us) return;
            s->stats.recovery_polls++;
            /* THE ONE EXCEPTION to always-piggyback (spec §5), and it is
             * load-bearing: R1 asks "did the head move past the seq the lost
             * frame acked?". A poll that carries that same ack MOVES THE HEAD
             * ITSELF, so the test would always answer "landed" and R1' could
             * never fire — a genuinely lost command would be bound to a seq the
             * gateway never issued and then quietly disposed at its deadline.
             * §3's worked recovery is explicit that this frame is `POLL (5 B,
             * ack 0)`; §5's blanket rule has to yield to it. The ack is not
             * lost, only deferred: ack_pending survives and rides the first
             * request issued after the recovery resolves. */
            if (issue_queue_txn_acked(s, NO_CMD, s->rec_ack == 0)) pace(s, PACE_MIN_MS);
            return;

        default:
            break;
    }

    /* A command waiting to be (re)sent goes first: it is either a resend of
     * something that never landed or an A_FULL retry, and either way it is
     * ahead of anything new. */
    uint8_t pi = find_state(s, CMD_PENDING);
    if (pi != NO_CMD) {
        if (now < s->next_at_us) return;
        /* Only claim AWAIT_REPLY if something actually went out: a refusal from
         * a full Modbus action queue leaves nothing on the wire, and a state
         * that says otherwise would report a phantom transaction and block
         * submits until the next tick noticed. */
        if (issue_queue_txn(s, pi)) s->st = S_AWAIT_REPLY;
        return;
    }

    /* A result deadline that has expired: poll once, and if the result still
     * is not there, escalate to boot_count (R4). */
    for (uint8_t i = 0; i < CMDQ_DEPTH; i++) {
        cmd_t *c = &s->cmd[i];
        if (c->st != CMD_ACCEPTED || now < c->deadline_us) continue;
        if (s->tx_deadline_seq == c->seq) continue;   /* already chasing this one */
        s->stats.deadlines++;
        s->tx_deadline_seq = c->seq;
        if (issue_queue_txn(s, NO_CMD)) s->st = S_AWAIT_REPLY;
        return;
    }

    /* Results outstanding, or an ack still to be delivered: poll, paced at the
     * estimated completion of the oldest in-flight command. This is the "pace,
     * don't spin" rule — the poll costs shared bus whether or not it starves
     * anybody. */
    if (count_state(s, CMD_ACCEPTED) > 0 || s->ack_pending != 0) {
        s->st = S_DEADLINE_WAIT;
        uint64_t at = (s->next_at_us > s->poll_at_us) ? s->next_at_us : s->poll_at_us;
        if (now < at) return;
        if (issue_queue_txn(s, NO_CMD)) s->st = S_AWAIT_REPLY;
        return;
    }

    s->st = S_IDLE;   /* nothing owed in either direction: no traffic at all */
}

void mbq_client_tick(void) {
    for (int i = 0; i < MAX_DALI_SLAVES; i++) {
        slave_t *s = &s_sl[i];
        if (!s->in_use) continue;

        if (s->busy) {
            if (!s->cb_fired) continue;
            s->cb_fired = false;
            s->busy     = false;
            s_resolving = true;
            handle_reply(s);
            s_resolving = false;
            if (s->st == S_AWAIT_REPLY) s->st = S_IDLE;   /* reply consumed */
            s->tx_cmd = NO_CMD;
        }
        drive(s);
    }
}

/* ── public API ──────────────────────────────────────────────────────────── */

void mbq_client_init(void) {
    memset(s_sl, 0, sizeof s_sl);
    s_resolving = false;
}

void mbq_client_add_slave(uint8_t slave_id) {
    if (slave_for(slave_id)) return;
    for (int i = 0; i < MAX_DALI_SLAVES; i++) {
        slave_t *s = &s_sl[i];
        if (s->in_use) continue;
        memset(s, 0, sizeof *s);
        s->in_use     = true;
        s->slave_id   = slave_id;
        s->st         = S_VERSION;
        s->tx_cmd     = NO_CMD;
        s->rec_cmd    = NO_CMD;
        s->next_at_us = 0;
        return;
    }
    LOG_WARN("mbq", "no room to register slave %u", slave_id);
}

bool mbq_client_can_submit(uint8_t slave_id) {
    slave_t *s = slave_for(slave_id);
    if (!s || s_resolving) return false;
    if (s->busy) return false;
    if (s->st != S_IDLE && s->st != S_DEADLINE_WAIT) return false;
    if (find_state(s, CMD_PENDING) != NO_CMD) return false;
    return find_state(s, CMD_FREE) != NO_CMD;
}

bool mbq_client_submit(uint8_t slave_id, const uint8_t cmd[MBQ_CMD_BYTES],
                       bool idempotent, mbq_cb_t cb, void *ctx) {
    if (!mbq_client_can_submit(slave_id)) return false;
    slave_t *s = slave_for(slave_id);
    uint8_t k = find_state(s, CMD_FREE);

    cmd_t *c = &s->cmd[k];
    memcpy(c->cmd, cmd, MBQ_CMD_BYTES);
    c->cb         = cb;
    c->ctx        = ctx;
    c->idempotent = idempotent;
    c->seq        = 0;
    c->resends    = 0;
    c->st         = CMD_PENDING;

    /* A fresh command is not subject to the pacing timer: pacing exists to stop
     * POLLS spinning, and holding real work back behind it would trade the
     * throughput this design exists to gain (spec §5 paces "on A_FULL, or when
     * awaiting results with an empty backlog" — this is neither). */
    if (!issue_queue_txn(s, k)) { c->st = CMD_FREE; c->cb = NULL; return false; }
    s->st = S_AWAIT_REPLY;
    return true;
}

uint8_t mbq_client_inflight(uint8_t slave_id) {
    slave_t *s = slave_for(slave_id);
    return s ? inflight_count(s) : 0u;
}

void mbq_client_txn_counts(uint8_t slave_id, uint32_t *submits, uint32_t *polls) {
    slave_t *s = slave_for(slave_id);
    if (submits) *submits = s ? s->stats.enqueues : 0u;
    if (polls)   *polls   = s ? s->stats.polls    : 0u;
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

void mbq_client_stats(uint8_t slave_id, mbq_stats_t *out) {
    if (!out) return;
    memset(out, 0, sizeof *out);
    slave_t *s = slave_for(slave_id);
    if (s) *out = s->stats;
}

uint16_t mbq_client_consec_fail(uint8_t slave_id) {
    slave_t *s = slave_for(slave_id);
    return s ? s->c_consec_fail : 0u;
}

bool mbq_client_drop_reply(uint8_t slave_id, uint16_t n) {
    slave_t *s = slave_for(slave_id);
    if (!s) return false;
    s->drop_replies = n;
    LOG_WARN("mbq", "slave %u: will drop the next %u replies (drill)", slave_id, n);
    return true;
}

/* ── observability ───────────────────────────────────────────────────────── */

static const char *state_name(uint8_t st) {
    switch (st) {
        case S_VERSION:        return "version";
        case S_DRAIN:          return "drain";
        case S_BOOT_CHECK:     return "boot-check";
        case S_IDLE:           return "idle";
        case S_AWAIT_REPLY:    return "await-reply";
        case S_RECOVERY_POLL:  return "recovery-poll";
        case S_DEADLINE_WAIT:  return "deadline-wait";
        default:               return "INCOMPATIBLE";
    }
}

static const char *cmd_state_name(uint8_t st) {
    switch (st) {
        case CMD_FREE:     return "free";
        case CMD_PENDING:  return "pending";
        case CMD_SENDING:  return "sending";
        default:           return "accepted";
    }
}

void mbq_client_debug_print(int slave_id) {
    bool any = false;
    for (int i = 0; i < MAX_DALI_SLAVES; i++) {
        slave_t *s = &s_sl[i];
        if (!s->in_use) continue;
        if (slave_id >= 0 && s->slave_id != (uint8_t)slave_id) continue;
        any = true;
        const mbq_stats_t *t = &s->stats;

        cdc_out_printf("mbq slave %u: %s%s gw=v%u.%u boot_count=%u%s\n",
                       s->slave_id, state_name(s->st), s->busy ? " (on wire)" : "",
                       s->gw_major, s->gw_minor, s->stats.boot_count,
                       s->epoch_clean ? "" : " EPOCH-UNCLEAN");
        cdc_out_printf("  ack_pending=%u head=%u last_seq=%u%s\n",
                       s->ack_pending, s->last_head, s->last_seq,
                       s->seq_known ? "" : " (unknown)");
        cdc_out_printf("  commands:");
        for (uint8_t k = 0; k < CMDQ_DEPTH; k++)
            cdc_out_printf(" %u:%s/seq%u", k, cmd_state_name(s->cmd[k].st), s->cmd[k].seq);
        cdc_out_printf("\n");
        cdc_out_printf("  txns: %lu enqueue + %lu poll (%lu recovery), %lu timeout, "
                       "%lu A_FULL, %lu resend\n",
                       (unsigned long)t->enqueues, (unsigned long)t->polls,
                       (unsigned long)t->recovery_polls, (unsigned long)t->timeouts,
                       (unsigned long)t->afull, (unsigned long)t->resends);
        cdc_out_printf("  recovery: R1 landed=%lu R1' never=%lu, deadlines=%lu, "
                       "boot reads=%lu\n",
                       (unsigned long)t->r1_landed, (unsigned long)t->r1_never,
                       (unsigned long)t->deadlines, (unsigned long)t->boot_reads);
        cdc_out_printf("  reboots: R2=%lu R3=%lu R4=%lu (head anomalies %lu, "
                       "protocol errors %lu)\n",
                       (unsigned long)t->reboots_r2, (unsigned long)t->reboots_r3,
                       (unsigned long)t->reboots_r4, (unsigned long)t->head_anomalies,
                       (unsigned long)t->proto_errors);
        cdc_out_printf("  results: complete=%lu failed=%lu lost=%lu result_lost=%lu\n",
                       (unsigned long)t->completed, (unsigned long)t->failed,
                       (unsigned long)t->lost, (unsigned long)t->result_lost);

        mbq_txn_stats_t ts;
        mbq_client_txn_stats(s->slave_id, &ts);
        if (ts.n)
            cdc_out_printf("  timing: n=%lu avg=%lu us min=%lu max=%lu fail=%lu\n",
                           (unsigned long)ts.n, (unsigned long)ts.us_avg,
                           (unsigned long)ts.us_min, (unsigned long)ts.us_max,
                           (unsigned long)ts.n_fail);
    }
    if (!any) cdc_out_printf("mbq: no matching slave\n");
}
