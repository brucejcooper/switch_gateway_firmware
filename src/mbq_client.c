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
 * boot_count the recovery rules classify against. The no-baseline adopt rule
 * below (apply_head, resolve_recovery_reply) does NOT depend on the drain
 * having succeeded — it is gated on `seq_known` alone, and that is sufficient
 * by construction: until this slave's first accept byte, the controller has
 * bound nothing to a seq, so ANY head it sees cannot be its own — it is
 * necessarily a leftover from a previous epoch, drain-clean or not. A drain
 * that gave up early (DRAIN_MAX_POLLS) just means there may be MORE leftovers
 * still to swallow; it does not change what an untracked head means while
 * seq_known is false. mbq_client_stats().drained records whether the drain
 * converged cleanly, but that is diagnostic only (see the CLI dump) — it
 * gates no recovery decision.
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
enum { H_OK = 0u, H_FAILED = 1u, H_NAK = 2u };   /* head-entry status; cq.h is authoritative */

/* Cold-path gateway registers (controller CLAUDE.md, "Gateway sysinfo"). Both
 * blocks are exact-count reads — a different count answers ILLEGAL DATA
 * ADDRESS rather than a short read. */
#define GW_VERSION_ADDR  0x0200u
#define GW_VERSION_REGS  16u
#define GW_SYSINFO_ADDR  0x0310u
#define GW_SYSINFO_REGS  4u

/* ── policy constants ─────────────────────────────────────────────────────── */

/* A command's result must arrive within this long, or rule R4 classifies it
 * as lost to a reboot or a gateway wedge. Worst queueing at depth 2 is one
 * ~50 ms DTR0-pair ahead of a ~50 ms one of your own, times a 1.5 margin
 * (spec §4). This is the ONLY DALI timing the client knows, and it is a
 * failure deadline — never flow control. */
#define PACE_MIN_MS       2u    /* floor on any pace() delay, so a zero-length
                                 * backoff still yields to the rest of the
                                 * main loop instead of spinning the bus       */
#define PACE_FAIL_MS      5u    /* first backoff after a transaction that failed */
#define PACE_DOWN_MS   1000u    /* backoff ceiling once the link looks dead      */
/* LINK_DOWN_FAILS is in mbq_client.h — dali_state gates on it too. */

/* How long a DECLARED-DOWN link must stay down before the client stops paying
 * for the §4 recovery ladder and fails new work immediately.
 *
 * The threshold alone cannot separate the two cases that both read
 * "c_consec_fail >= LINK_DOWN_FAILS", and they want opposite treatment:
 *
 *   a gateway REBOOTING — down for ~1 s and coming back. Its in-flight
 *     idempotent work must ride the outage out, because dropping it is the
 *     gate-V2 defect: the soak's SET goes missing, DTR0 keeps the previous
 *     pair's value, and the paired query reads stale (see recovery_give_up).
 *
 *   a gateway that is NOT THERE — configured but absent, down since boot. Every
 *     submission is doomed before it is made, and the ladder spends 8 s of held
 *     slots and ~5 WARN lines per attempt proving it. dali_state probes such a
 *     slave every LINK_RETRY_MS (5 s) forever, so that is a permanent saga.
 *
 * TIME is what tells them apart, so time is what this gates on. Below it,
 * NOTHING changes — R1/R1' and the whole ladder run exactly as before, which is
 * what a single lost frame on a healthy link needs.
 *
 * The value is set by how long REDISCOVERY takes, not by how long a restart
 * takes, and rediscovery is the transport's to do: modbus.c stops putting a
 * dead slave on the wire and lets exactly one probe through per backoff window,
 * the first of which is 1 s (MODBUS_BACKOFF_MIN_MS). So a gateway that restarts
 * in ~1 s is not seen answering again until ~2-2.5 s after the link went down,
 * and anything shorter than that would drop the very work the ride-out exists
 * to carry. Four seconds leaves a comfortable margin and is still half of
 * CMD_MAX_LIFE_MS, the absolute bound this short-circuits: a permanently absent
 * slave pays for one 4 s ladder at boot and nothing after it. */
#define LINK_QUIET_MS  4000u

/* Grace given to the SCHEDULER before the client spends a transaction on a
 * poll. When the oldest command comes due, the right next transaction is
 * "enqueue the next command + ack the old result" — one frame doing both — but
 * only the scheduler above knows whether there is a command to send. dq_tick
 * drives this client BEFORE its own ship loop, so without a grace the client
 * always wins that race and polls, and the enqueue then costs a second
 * transaction. That is the 2.04-transactions-per-command defect. A couple of
 * milliseconds is thousands of main-loop iterations; if no command appears in
 * that window the backlog really is empty and the poll is the right call. */
#define PACE_YIELD_MS     2u

/* How many times one command's expired result deadline may be chased with a
 * poll before escalating to the cold-path boot_count arbiter. Without a bound,
 * a gateway that has stopped answering is re-polled on every main-loop pass:
 * measured 29 852 "deadline expiries" for a single command in 30 s, which is
 * not just wasted bus but a §6 diagnostic rendered unreadable exactly when it
 * is needed. */
#define MAX_DEADLINE_CHASES 3u

/* A Modbus EXCEPTION is still an ANSWER (mbq_modbus_cb below counts only
 * res == 0 — a genuine no-reply — toward c_consec_fail/link_down), so a
 * gateway that persistently EXCEPTIONS the sysinfo read (FC04 0x0310) rather
 * than going silent never trips link_down, and S_BOOT_CHECK's `if (down)
 * bootchk_give_up(s)` never fires. Without a bound of its own, that parks the
 * slave in S_BOOT_CHECK forever, re-issuing the read every PACE_FAIL_MS with
 * the whole command path stalled behind it (drive() returns from the
 * S_BOOT_CHECK case before any command work runs). Give up after this many
 * CONSECUTIVE exceptions, matching how TX_VERSION already treats its own
 * unreadability as advisory rather than fatal. */
#define MAX_SYSINFO_EXCEPTIONS 3u

/* A command may go back on the wire this many times (A_FULL retries excluded,
 * they are not resends of a landed command) before the client gives up and
 * reports it LOST. Without a cap, an idempotent command against a gateway that
 * is answering but never completing anything resubmits forever and the queue
 * never drains — R4 would fire, resubmit, fire again. */
#define MAX_RESENDS       3u
/* Absolute lifetime of a command that keeps failing to REACH the gateway.
 * MAX_RESENDS bounds re-executions; this bounds the wholly different case of a
 * link that is simply down, where no frame has happened and counting attempts
 * measures nothing but how fast we retried. Comfortably longer than a gateway
 * restart (~1 s) and short enough that a gateway which never returns still
 * gives every caller a terminal answer. */
#define CMD_MAX_LIFE_MS 8000u

/* Bound on the registration drain. RESPQ_DEPTH is 4, so five polls empty a
 * full queue; a gateway still producing heads after that is not draining and
 * the client proceeds anyway rather than refusing to ever talk to it. The
 * no-baseline adopt rule (seq_known == false) is unaffected either way — see
 * the file header — so giving up here only costs the mbq_client_stats().
 * drained diagnostic, not correctness. */
#define DRAIN_MAX_POLLS   6u

/* The §8 compat latch's version half. FC04 0x0200 is a free-form STRING
 * ("DALI v<maj>.<min> <serial>") rather than a numeric queue-generation field,
 * so the mapping has to be agreed rather than derived: the v4 gateway sets
 * FW_VER 1.0 and defines 0.x = pre-v4, MAJOR >= 1 = queue v4 (TinyModbus
 * task-08 G4b). That is what makes this a safe hard refusal rather than a
 * guess. The BEHAVIOURAL half latches independently and needs no agreement at
 * all — an ILLEGAL FUNCTION reply to FC 0x41, or a reply of a length the
 * request did not call for — and it is the backstop if a gateway ever reports
 * a version that does not match what it actually speaks. */
#define GW_MIN_MAJOR      1u
#define GW_MIN_MINOR      0u
#define GW_VER_LATCHES    1

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
    uint32_t ord;                  /* producer order, preserved across resends */
    uint8_t  resends;
    uint8_t  chases;               /* deadline polls spent on this command */
    bool     pred;                 /* seq was PREDICTED by R1, not read from an
                                    * accept byte — must be verified when its
                                    * result surfaces (spec §4) */
    uint64_t deadline_us;          /* result deadline, armed at accept */
    uint64_t born_us;              /* first submit; bounds recovery in TIME */
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
    BC_R1_EMPTY,      /* the lost-reply poll found an empty head: R1 or R3?    */
} bc_reason_t;

typedef enum { TX_QUEUE = 0, TX_SYSINFO, TX_VERSION } tx_kind_t;


#define NO_CMD 0xFFu

typedef struct {
    bool     in_use;
    uint8_t  slave_id;
    uint8_t  st;                /* mbq_state_t */
    bool     busy;              /* a transaction is on the wire */

    cmd_t    cmd[MBQ_CMD_SLOTS];

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

    /* Recovery bookkeeping (§4). */
    uint8_t  rec_cmd;           /* command whose reply was lost, or NO_CMD */
    uint8_t  rec_ack;           /* the ack that lost frame carried; 0 = ack-less */
    uint64_t rec_deadline_us;   /* bound on the ack-less poll loop */

    /* boot_count arbiter. */
    uint8_t  bc_reason;         /* bc_reason_t for the read in flight */
    bool     bc_valid;
    uint16_t bc_last;
    uint8_t  sysinfo_exc;       /* consecutive Modbus EXCEPTIONS to the sysinfo
                                 * read — see MAX_SYSINFO_EXCEPTIONS; reset on
                                 * any non-exception outcome */

    uint32_t next_ord;          /* monotonic submit counter, source of cmd_t.ord */
    uint8_t  drain_polls;
    uint8_t  drain_pass;        /* 1 = first sweep, 2 = the post-wait sweep */
    uint64_t drain_wait_us;     /* when the second sweep may begin */
    /* A gateway reboot leaves results in the response queue for commands whose
     * bindings we have just thrown away. Each one surfaces as an untracked head
     * — R3's exact signature — so without a window R3 re-fires on every one,
     * and each firing disposes the healthy work that replaced it, whose results
     * are then themselves untracked. It is self-sustaining: MEASURED 3801 R3
     * firings and 959 commands lost in one 35 s reboot drill, from ONE real
     * reboot. During this window an untracked head is consumed as an orphan
     * instead. Same reasoning as §4's controller-reboot drain, from the other
     * side of the wire. */
    uint64_t epoch_drain_us;    /* suppress repeat R3 until this passes */
    uint8_t  gw_major, gw_minor;

    /* The ONE pacing gate, and it exists only for TRANSPORT trouble: a retry
     * after a transaction that got no answer, backing off a slave that has
     * stopped responding. Nothing here paces against DALI — the gateway
     * answers A_FULL when it is busy and that is the whole of the flow
     * control. A fresh submit is not gated by it at all. */
    uint64_t next_at_us;
    uint64_t poll_yield_us;     /* one-tick grace for the producer before a poll */

    /* Modbus completion hand-off (resolved in tick, not in the callback). */
    volatile bool cb_fired;
    int      cb_res;
    uint8_t  cb_buf[40];
    uint8_t  cb_len;

    uint16_t drop_replies;      /* bench drill knob (C3) */
    uint16_t withhold_acks;     /* backpressure drill: suppress the piggyback ack */

    uint64_t t_issue_us;
    uint32_t w_n, w_min, w_max, w_fail, w_busy;
    uint64_t w_sum;
    uint16_t c_consec_fail;
    uint64_t link_down_us;      /* when c_consec_fail crossed LINK_DOWN_FAILS;
                                 * 0 = the link is up. Any valid reply clears
                                 * it, exactly as it clears c_consec_fail.    */

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


static cmd_t *cmd_by_seq(slave_t *s, uint8_t seq) {
    if (seq == 0) return NULL;
    for (uint8_t i = 0; i < MBQ_CMD_SLOTS; i++)
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

/* The sequence the gateway will assign next, mirroring its 1..31 counter with
 * 0 reserved. Used only to PREDICT a binding after a lost reply (rule R1); the
 * prediction is verified when the result surfaces. */
static uint8_t seq_next(uint8_t s) { return (s >= SEQ_MASK) ? 1u : (uint8_t)(s + 1u); }

static uint8_t count_state(const slave_t *s, cmd_state_t st) {
    uint8_t n = 0;
    for (uint8_t i = 0; i < MBQ_CMD_SLOTS; i++) if (s->cmd[i].st == st) n++;
    return n;
}

static uint8_t find_state(const slave_t *s, cmd_state_t st) {
    for (uint8_t i = 0; i < MBQ_CMD_SLOTS; i++) if (s->cmd[i].st == st) return i;
    return NO_CMD;
}

/* The PENDING command that must go out next — the one submitted EARLIEST, not
 * the one in the lowest slot.
 *
 * This is load-bearing and it is not obvious. The gateway executes strictly
 * FIFO in accept order, so the order the client re-offers commands in IS the
 * order the DALI frames happen in. A reboot disposes several commands at once
 * and re-queues the idempotent ones together; re-offering those in slot-array
 * order silently reverses DALI frames that the producer issued in a meaningful
 * sequence. Measured on the bench: a soak SET/QUERY pair straddling a gateway
 * reboot came back with the query answering 0x62 where 0x01 was written — the
 * value the NEXT pair's SET had put there, because the resent SET overtook the
 * resent query. Deterministic, one mis-attribution per reboot, and exactly the
 * result-misattribution class v4 exists to eliminate. */
static uint8_t find_next_pending(const slave_t *s) {
    uint8_t best = NO_CMD;
    for (uint8_t i = 0; i < MBQ_CMD_SLOTS; i++) {
        if (s->cmd[i].st != CMD_PENDING) continue;
        if (best == NO_CMD || s->cmd[i].ord < s->cmd[best].ord) best = i;
    }
    return best;
}

static cmd_t *find_pred_bound(slave_t *s) {
    for (uint8_t i = 0; i < MBQ_CMD_SLOTS; i++)
        if (s->cmd[i].st == CMD_ACCEPTED && s->cmd[i].pred) return &s->cmd[i];
    return NULL;
}

static uint8_t inflight_count(const slave_t *s) {
    uint8_t n = 0;
    for (uint8_t i = 0; i < MBQ_CMD_SLOTS; i++) if (s->cmd[i].st != CMD_FREE) n++;
    return n;
}

/* Room to accept one more command: a free slot, and fewer than
 * MBQ_QUEUE_DEPTH already staged waiting to go out. Note this asks nothing
 * about the GATEWAY's occupancy — that is the gateway's to answer.
 *
 * A_FULL IS FLOW CONTROL (user decision, 2026-08-12). The client does NOT
 * predict how full the gateway is — it offers a command and lets the gateway
 * answer. A refusal costs one transaction and that transaction is not wasted:
 * every reply carries the head of the response queue, so a bounced enqueue
 * still collects a result.
 *
 * What this replaces, and why: the client used to estimate when each command
 * would finish (by class, ~26 ms SET / ~38 ms query) and admit a new one only
 * when the estimate said a gateway slot had freed. That bought one transaction
 * per DALI frame instead of two — and cost an estimator that had to be
 * calibrated against the rig, drifted, and put a command's admission at the
 * mercy of a guess. It was the direct cause of the scheduler-path defect
 * (1.79 txns/command): a command whose ESTIMATE had expired but whose RESULT
 * had not arrived still held a local slot, so the client ran out of slots,
 * could not enqueue, and polled instead.
 *
 * The trade, accepted deliberately: more Modbus transactions, in exchange for
 * no guesswork, lower command latency (a retry lands within a transaction of a
 * slot opening, where a deliberately-late estimate does not), and a much
 * smaller client. With several gateways the round-robin supplies the spacing
 * for free — by the time the master comes back to a slave, it is more likely
 * to have room. */
static bool have_free_slot(const slave_t *s) {
    return count_state(s, CMD_PENDING) < MBQ_QUEUE_DEPTH &&
           find_state(s, CMD_FREE) != NO_CMD;
}


static void pace(slave_t *s, uint32_t ms) {
    if (ms < PACE_MIN_MS) ms = PACE_MIN_MS;
    s->next_at_us = MBQ_NOW_US() + (uint64_t)ms * 1000u;
}

/* Back off after a transaction that produced no answer, doubling with each
 * consecutive failure up to PACE_DOWN_MS.
 *
 * The flat retry this replaces was the worse half of a measured defect: when
 * the transport REFUSES an action outright (slave in backoff, action queue
 * full) the callback fires immediately, so "wait 5 ms then retry" became a
 * main-loop-speed spin — 8785 transactions in one bench reboot drill, at
 * 1.6 ms average against a 50 ms wire timeout. A refusal is not a timeout and
 * a spin is not a retry; both need the same gate, and it has to grow. */
static void pace_backoff(slave_t *s) {
    uint32_t ms = PACE_FAIL_MS;
    for (uint16_t i = 1; i < s->c_consec_fail && ms < PACE_DOWN_MS; i++) ms <<= 1;
    if (ms > PACE_DOWN_MS) ms = PACE_DOWN_MS;
    pace(s, ms);
}


/* The link has been DECLARED down (LINK_DOWN_FAILS consecutive no-replies).
 * Used only to pick a log LEVEL: while a slave is not answering, the ladder's
 * per-command narration is noise, and the one line worth printing has already
 * been printed one layer down (modbus.c's "slave %u unresponsive — backing off"
 * / "answering again — backoff cleared"). Mirror it, do not duplicate it. */
static bool link_down(const slave_t *s) { return s->link_down_us != 0; }

/* ...and it has stayed down past the point where a gateway restart could
 * explain it (LINK_QUIET_MS). This, not link_down(), gates BEHAVIOUR. */
static bool link_written_off(const slave_t *s) {
    return s->link_down_us != 0 &&
           (MBQ_NOW_US() - s->link_down_us) >= (uint64_t)LINK_QUIET_MS * 1000u;
}

/* WARN on a healthy link, DBG on a link already known to be down. Same text
 * either way — the difference is whether it is news. */
#define LOG_LINK(s, ...) \
    log_emit(link_down(s) ? LOG_LEVEL_DBG : LOG_LEVEL_WARN, \
             link_down(s) ? "D" : "W", "mbq", __VA_ARGS__)

/* Deliver a terminal outcome and charge it to `stat`. Split out so the two
 * outcomes that are NOT protocol losses — a NAK, and a command failed fast
 * against a link that is down — can be delivered as MBQ_R_NAK/MBQ_R_LOST to the
 * caller (which is what they are) while being counted somewhere honest. */
static void deliver_as(slave_t *s, cmd_t *c, mbq_result_t r, uint8_t resp8,
                       uint32_t *stat) {
    mbq_cb_t cb = c->cb;
    void *ctx = c->ctx;
    c->st = CMD_FREE;      /* free before the callback: it may submit again */
    c->cb = NULL;
    c->seq = 0;
    (*stat)++;
    if (cb) cb(s->slave_id, r, resp8, ctx);
}

static void deliver(slave_t *s, cmd_t *c, mbq_result_t r, uint8_t resp8) {
    uint32_t *stat;
    switch (r) {
        case MBQ_R_COMPLETE:    stat = &s->stats.completed;   break;
        case MBQ_R_FAILED:      stat = &s->stats.failed;      break;
        case MBQ_R_RESULT_LOST: stat = &s->stats.result_lost; break;
        /* A clean "the gear did not answer" is a normal outcome on a sparse
         * bus, not a loss. It used to fall into the default: arm below and was
         * most of the bench's lost=1106 — 62 empty addresses per scan. */
        case MBQ_R_NAK:         stat = &s->stats.naks;        break;
        default:                stat = &s->stats.lost;        break;
    }
    deliver_as(s, c, r, resp8, stat);
}

/* Put a command back on the wire, or give up on it. Shared by R1', A_FULL and
 * every reboot/loss disposal, so the resend cap is enforced in exactly one
 * place. */
static void resend_or_lose(slave_t *s, cmd_t *c, mbq_result_t give_up_as) {
    if (c->resends >= MAX_RESENDS) {
        LOG_LINK(s, "slave %u: command %02X%02X resent %u times without a "
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
    for (uint8_t i = 0; i < MBQ_CMD_SLOTS; i++) {
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
    /* Open the drain window: everything still sitting in the gateway's response
     * queue belongs to the epoch we just abandoned, and must be swallowed
     * rather than re-diagnosed. One result deadline is the same bound §4 uses
     * for the controller-reboot drain, and for the same reason — the gateway
     * holds at most CMDQ_DEPTH commands and each finishes within a frame. */
    s->epoch_drain_us = MBQ_NOW_US() + (uint64_t)MBQ_RESULT_DEADLINE_MS * 1000u;
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
    for (uint8_t i = 0; i < MBQ_CMD_SLOTS; i++)
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
        /* Stamp the DOWN transition once. What the fast-fail needs to know is
         * not how many failures there have been but how long they have been
         * going on — a gateway restart is short, an absent gateway is not. */
        if (s->c_consec_fail >= LINK_DOWN_FAILS && s->link_down_us == 0)
            s->link_down_us = MBQ_NOW_US();
    } else {
        s->c_consec_fail = 0;
        s->link_down_us  = 0;
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
    /* The backpressure drill withholds the ack so the gateway's response queue
     * fills and its arbiter stalls (spec §2's backpressure rule). Nothing else
     * changes: ack_pending survives and is delivered once the drill ends, so
     * the queue drains and the run recovers to full rate.
     *
     * Spend the budget only when there is an ack actually DUE
     * (s->ack_pending != 0): with nothing outstanding, `ctl`'s ack field comes
     * out 0 whether with_ack is left true or forced false (ack_pending &
     * SEQ_MASK is 0 either way), so decrementing here withholds nothing at the
     * gateway. Every such request — issued whenever a result has not yet been
     * consumed, e.g. while staging the very first command, or immediately
     * after a reply already cleared ack_pending — still counted against `n`,
     * so `mbq no-ack <slave> <n>` could exhaust its whole budget without ever
     * making the gateway's response queue fill, and the drill would read
     * clean (gate V4, QUEUE-PROTOCOL.md §10) having withheld nothing. */
    if (with_ack && s->ack_pending != 0 && s->withhold_acks > 0) {
        s->withhold_acks--;
        with_ack = false;
    }
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
    if (r->hstatus > H_NAK) return false;
    return true;
}


/* ── head processing ─────────────────────────────────────────────────────── */

/* Apply the head field of a reply. Returns true if a reboot was inferred here
 * (R3), so the caller does not then apply a second rule to the same reply. */
static bool apply_head(slave_t *s, const reply_t *r) {
    /* R3, second clause: an unacked result cannot disappear. The head going
     * empty while the master still holds a consumed-but-unacked result is only
     * possible if something other than our ack emptied it — and the only thing
     * that can is a restart. "No ack unresolved" is the qualifier that makes
     * this decidable: if THIS request carried the ack for that result, an empty
     * head is just the ack working.
     * NOT during recovery. There the master is deliberately polling WITHOUT an
     * ack, so "no ack unresolved" is true by construction on every poll, and an
     * empty head is the genuinely ambiguous branch 3 of the lost-reply
     * procedure — resolved against boot_count, not by declaring a reboot here. */
    if (s->st != S_RECOVERY_POLL && r->hseq == 0 &&
        s->ack_pending != 0 && s->tx_ack != s->ack_pending) {
        note_reboot(s, '3', NULL);
        s->last_head = 0;
        return true;
    }

    /* Results are BLOCKED, not missing. While the head still holds a result the
     * master has consumed but not managed to ack, every result behind it is
     * invisible — the gateway is retaining them exactly as designed. Letting
     * the result deadline expire in that state makes R4 read a stalled queue as
     * a wedged gateway and resend a command the gateway is still holding, which
     * is a DOUBLE EXECUTION of a real DALI frame, reported to nobody. It is
     * reachable deliberately (the `mbq no-ack` backpressure drill, gate V4) and
     * accidentally (an ack whose frame keeps getting lost). The client can tell
     * the difference, so it must: push the deadlines out while the head is
     * known-blocked, and let them run again once it moves. */
    if (r->hseq != 0 && r->hseq == s->ack_pending) {
        uint64_t push = MBQ_NOW_US() + (uint64_t)MBQ_RESULT_DEADLINE_MS * 1000u;
        for (uint8_t i = 0; i < MBQ_CMD_SLOTS; i++) {
            cmd_t *c = &s->cmd[i];
            if (c->st == CMD_ACCEPTED && c->deadline_us < push) {
                c->deadline_us = push;
                c->chases = 0;
            }
        }
        s->stats.head_blocked++;
    }

    /* Ack confirmation: the head moving off the seq we were acking proves the
     * ack was applied. While it has not moved, keep carrying the same ack. */
    if (s->ack_pending != 0 && r->hseq != s->ack_pending) s->ack_pending = 0;

    s->last_head = r->hseq;

    if (r->hseq == 0 || r->hseq == s->ack_pending) return false;  /* nothing new */

    cmd_t *c = cmd_by_seq(s, r->hseq);
    if (c) {
        /* The prediction, verified. A command bound by R1 was bound to a seq
         * the master calculated rather than read, and this is where that
         * calculation is confirmed against the gateway. */
        c->pred = false;
        s->tx_deadline_seq = 0;
        if      (r->hstatus == H_OK)  deliver(s, c, MBQ_R_COMPLETE, r->resp8);
        else if (r->hstatus == H_NAK) deliver(s, c, MBQ_R_NAK,      0);
        else                          deliver(s, c, MBQ_R_FAILED,   r->resp8);
        s->ack_pending = r->hseq;
        return false;
    }

    /* An untracked head. Three quite different things can produce one. */
    if (!s->seq_known) {
        /* NO BASELINE: the gateway's counter is wherever its uptime left it and
         * this controller has never seen an accept byte from it, so a result
         * bearing an unknown seq is a leftover from the PREVIOUS controller
         * epoch — the norm in this state, not evidence of anything. Consume and
         * discard it; reading it as R3 would declare a reboot that never
         * happened, on almost every controller restart. */
        s->stats.orphans++;
        s->ack_pending = r->hseq;
        return false;
    }

    cmd_t *p = find_pred_bound(s);
    if (p) {
        /* A command R1 bound to a PREDICTED seq, and the seq that actually
         * surfaced is not it. The prediction was wrong, which is a detected
         * fault and not a reboot: silently accepting a mis-binding is exactly
         * the result-misattribution class that took task 04 three fix cycles.
         * Count it, say so, dispose of the command by policy, and drop the
         * orphan result rather than handing it to the wrong callback. */
        s->stats.pred_faults++;
        LOG_ERR("mbq", "slave %u: predicted seq %u for command %02X%02X but the "
                "gateway produced seq %u — mis-binding detected, disposing",
                s->slave_id, p->seq, p->cmd[0], p->cmd[1], r->hseq);
        p->pred = false;
        if (p->idempotent) resend_or_lose(s, p, MBQ_R_LOST);
        else               deliver(s, p, MBQ_R_LOST, 0);
        s->ack_pending = r->hseq;
        return false;
    }

    /* Already draining a dead epoch: this is a leftover, not news. Swallow it.
     * Re-declaring the reboot here is what made one real reboot cascade into
     * thousands (see epoch_drain_us) — each disposal orphaned the commands that
     * had just replaced the disposed ones, so the signature regenerated itself
     * faster than the queue could drain. */
    if (MBQ_NOW_US() < s->epoch_drain_us) {
        s->stats.orphans++;
        s->ack_pending = r->hseq;
        s->last_head   = r->hseq;
        return false;
    }

    /* R3, first clause: a head seq the master is not tracking, with a baseline
     * established and no prediction outstanding, has no other source than a
     * gateway that restarted its counter. */
    note_reboot(s, '3', NULL);
    s->ack_pending = r->hseq;   /* pop the orphan so it cannot block the queue */
    s->last_head   = r->hseq;
    return true;
}

/* Bind a command to the seq the gateway assigned it and start its deadline.
 *
 * There is no completion ESTIMATE any more. The deadline below is a failure
 * timer, not a schedule: it decides when a missing result becomes rule R4's
 * problem, and nothing reads it to decide when to transmit. */
static void bind_accepted(slave_t *s, cmd_t *c, uint8_t seq, bool predicted) {
    c->seq         = seq;
    c->pred        = predicted;
    c->st          = CMD_ACCEPTED;
    c->deadline_us = MBQ_NOW_US() + (uint64_t)MBQ_RESULT_DEADLINE_MS * 1000u;
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
            /* A_FULL IS FLOW CONTROL, not an anomaly (user decision,
             * 2026-08-12). The gateway is the authority on its own queue: we
             * offer, it answers. Hold the command and re-offer it on the next
             * transaction to this slave.
             *
             * The refused transaction is NOT wasted — its reply carried the
             * head of the response queue like any other, so it still collected
             * a result. That is what makes offer-and-be-told affordable, and
             * it is why there is no backoff here: retrying at bus rate lands
             * the command within one transaction of a slot opening, which is
             * better latency than any estimate would give. With several
             * gateways the scheduler's round-robin supplies the spacing.
             *
             * (Kept deliberately quiet at LOG_DBG. It used to be LOG_ERR on
             * the theory that A_FULL was unreachable — that claim was false
             * twice over and is retracted in QUEUE-PROTOCOL.md §4.) */
            s->stats.afull++;
            LOG_DBG("mbq", "slave %u: A_FULL, re-offering (total %lu)",
                    s->slave_id, (unsigned long)s->stats.afull);
            c->st  = CMD_PENDING;
            c->seq = 0;
        } else if (r->astatus == A_ACCEPTED && r->aseq != 0) {
            /* R2: the gateway pins a seq until its result is acked, and acks are
             * head-only and in order, so reaching the same value again needs 31
             * further accepts and at least 27 acks — which would have drained the
             * pinned entry first. An accept byte carrying a seq we still track is
             * therefore proof the gateway restarted its counter. */
            if (seq_tracked(s, r->aseq)) {
                bind_accepted(s, c, r->aseq, false);
                /* KEEPING `c` IS ONLY SAFE IF IT IS THE OLDEST. The gateway has
                 * already accepted it, so it is first in the new epoch's FIFO —
                 * while every command we are about to dispose is re-offered
                 * BEHIND it. If any of those is earlier in producer order, the
                 * reboot has just reversed two frames whose meaning is their
                 * order, and no amount of ordered re-offering downstream can
                 * put `c` back where it belongs.
                 *
                 * That is the last gate-V2 defect: the soak submits SET(v) then
                 * QUERY, both in flight at depth 2. When the accept that reveals
                 * the reboot is the QUERY's, the SET is resent behind it, the
                 * QUERY runs first and reads the DTR0 the PREVIOUS pair left in
                 * the ballast (a gateway reboot does not clear the gear), and
                 * the probe sees expect=0xA0 got=0x62. Exactly one per reboot,
                 * deterministic, and invisible to the producer-order stamp
                 * because the mis-ordered command is the one NOT being
                 * re-offered.
                 *
                 * So when `c` is out of order, dispose it too and let it take
                 * its proper place in the re-offer. It still executes once in
                 * the gateway — we cannot recall it — but that result arrives
                 * under a seq we no longer track and is swallowed as an orphan
                 * by the drain window note_reboot opens. Costs one redundant
                 * DALI frame; buys back the ordering.
                 *
                 * Note the test is "will actually be RE-OFFERED", not merely
                 * "is older": a non-idempotent command is delivered LOST and
                 * never goes back on the wire, so nothing lands behind `c` and
                 * keeping it is correct. Only a command that will be resent
                 * can overtake. */
                const cmd_t *keep = c;
                for (uint8_t i = 0; i < MBQ_CMD_SLOTS; i++) {
                    const cmd_t *t = &s->cmd[i];
                    if (t == c) continue;
                    if (t->st != CMD_ACCEPTED && t->st != CMD_SENDING) continue;
                    if (t->ord < c->ord && t->idempotent &&
                        t->resends < MAX_RESENDS) { keep = NULL; break; }
                }
                note_reboot(s, '2', keep);
                if (!keep) {
                    /* We DID observe the gateway assign this seq, so the new
                     * epoch's baseline is known even though we kept nothing —
                     * dropping it would put the slave into the no-baseline
                     * state for no reason and cost the next lost reply its
                     * prediction. */
                    s->last_seq  = r->aseq;
                    s->seq_known = true;
                }
                rebooted = true;
            } else {
                bind_accepted(s, c, r->aseq, false);
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
        /* THE LOST-REPLY PROCEDURE, IN EVALUATION ORDER (spec §4). R1/R1' and
         * R3 can look at the same observation and disagree, so the order is
         * part of the contract, not an implementation detail. First match wins.
         *
         * The poll that produced this reply deliberately carried NO ack, so the
         * head's state reflects only what the lost frame itself did. */

        /* 1. The head still holds the seq we tried to ack: the ack did not
         *    apply, so the frame never arrived. R1'. */
        if (r->hseq == s->rec_ack) {
            s->stats.r1_never++;
            s->rec_cmd = NO_CMD;
            /* Resend verbatim, ack included — no idempotency concern, because
             * the head proves the frame never reached the gateway. Through the
             * shared path so it is counted and capped like any other resend. */
            resend_or_lose(s, c, MBQ_R_LOST);
            apply_head(s, r);
            return;
        }

        /* 2. The head holds a seq we are not tracking: R3, the gateway
         *    restarted. (With no baseline yet an unknown seq is instead a
         *    leftover from the previous controller epoch — apply_head knows the
         *    difference, so the command is disposed conservatively and the
         *    orphan consumed.) */
        if (r->hseq != 0 && !cmd_by_seq(s, r->hseq)) {
            s->rec_cmd = NO_CMD;
            if (!s->seq_known) {
                if (c->idempotent) resend_or_lose(s, c, MBQ_R_LOST);
                else               deliver(s, c, MBQ_R_LOST, 0);
            }
            apply_head(s, r);   /* fires R3, or discards the orphan */
            return;
        }

        /* 3. The head is empty: genuinely ambiguous. "The ack applied and the
         *    queue happened to drain" and "a reboot wiped the queue" cannot be
         *    told apart from the head alone, so this is the one R1 branch that
         *    pays for a cold-path boot_count read — on a path that is already
         *    exceptional. */
        if (r->hseq == 0) {
            /* Deliberately does NOT apply the head: whether the ack took effect
             * is the very thing in question, so ack_pending is left alone until
             * boot_count says which way this went. */
            s->last_head = 0;
            s->bc_reason = BC_R1_EMPTY;
            s->st = S_BOOT_CHECK;
            return;
        }

        /* 4. The head holds a DIFFERENT seq we are tracking: R1, landed. A
         *    reboot would have emptied the response queue, so a tracked seq
         *    proves the epoch survived, which means the ack applied, which
         *    means the whole frame landed — ack and enqueue travel together. */
        s->stats.r1_landed++;
        s->rec_cmd = NO_CMD;
        if (s->seq_known) {
            bind_accepted(s, c, seq_next(s->last_seq), true);
        } else {
            /* No baseline: nothing to predict from, so resolve conservatively
             * rather than invent a seq (spec §4). */
            if (c->idempotent) resend_or_lose(s, c, MBQ_R_LOST);
            else               deliver(s, c, MBQ_R_LOST, 0);
        }
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
            bind_accepted(s, c, r->hseq, false);  /* observed, not predicted */
            apply_head(s, r);                     /* the result is right there */
            return;
        }
        if (cmd_by_seq(s, r->hseq)) { apply_head(s, r); return; }   /* an earlier command's */
        if (!s->seq_known) {
            /* No baseline, so there is no prediction to match against and no
             * way to tell our result from a previous epoch's. Resolve
             * conservatively: this costs at most one command, once per
             * controller start, and only if that first reply was lost. */
            s->rec_cmd = NO_CMD;
            if (c->idempotent) resend_or_lose(s, c, MBQ_R_LOST);
            else               deliver(s, c, MBQ_R_LOST, 0);
            apply_head(s, r);                     /* discards the orphan */
            return;
        }
        apply_head(s, r);                         /* unknown seq → R3 fires in there */
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
    s->sysinfo_exc = 0;   /* a real reply — the exception streak, if any, is over */
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
                for (uint8_t i = 0; i < MBQ_CMD_SLOTS; i++) {
                    cmd_t *e = &s->cmd[i];
                    if (e->st != CMD_ACCEPTED || e->seq != s->tx_deadline_seq) continue;
                    if (e->pred) {
                        /* The other way a prediction can be wrong: not a
                         * DIFFERENT seq surfacing, but the predicted one never
                         * surfacing at all. apply_head can only catch the
                         * former, because nothing re-examines a prediction that
                         * simply never materialises. It is still a prediction
                         * fault and must be counted as one, or the §6 stat only
                         * reports half of R1 guessing wrong.
                         *
                         * And it disposes as LOST, not RESULT_LOST: the
                         * prediction was the only evidence this command ever
                         * had a seq at all, so "it never ran" is the honest
                         * reading. RESULT_LOST would tell the submitter the
                         * DALI frame executed — and specifically not to retry —
                         * when it did not. */
                        s->stats.pred_faults++;
                        LOG_ERR("mbq", "slave %u: predicted seq %u for command "
                                "%02X%02X never appeared — the gateway never "
                                "issued it", s->slave_id, e->seq, e->cmd[0], e->cmd[1]);
                        deliver(s, e, MBQ_R_LOST, 0);
                    } else if (e->idempotent) {
                        resend_or_lose(s, e, MBQ_R_LOST);
                    } else {
                        deliver(s, e, MBQ_R_RESULT_LOST, 0);
                    }
                    break;
                }
            }
            s->tx_deadline_seq = 0;
            s->st = S_IDLE;
            break;

        case BC_R1_EMPTY:
            /* Branch 3 of the lost-reply procedure. An empty head means either
             * the lost frame's ack applied and the queue then drained, or a
             * reboot wiped it; boot_count is the only thing that separates
             * them. */
            if (!c) { s->st = S_IDLE; break; }
            s->rec_cmd = NO_CMD;
            if (changed || first) {
                note_reboot(s, '3', NULL);       /* wiped: R3, and c goes with it */
            } else if (s->seq_known) {
                s->stats.r1_landed++;            /* survived: R1, the frame landed */
                s->ack_pending = 0;              /* ...so its ack applied too    */
                bind_accepted(s, c, seq_next(s->last_seq), true);
            } else {
                if (c->idempotent) resend_or_lose(s, c, MBQ_R_LOST);
                else               deliver(s, c, MBQ_R_LOST, 0);
            }
            s->st = S_IDLE;
            break;

        case BC_ACKLESS:
        default:
            s->rec_cmd = NO_CMD;
            if (!c) { s->st = S_IDLE; break; }
            if (changed || first) {
                /* "Executed, then the result died with the reboot" cannot be
                 * excluded — which is a reason not to repeat a NON-idempotent
                 * command, and no reason at all to drop an idempotent one.
                 * Re-running something that may already have run is precisely
                 * what idempotent means, and §4 says so: only the
                 * non-idempotent case disposes here.
                 *
                 * Dropping both was the last gate-V2 defect. The soak's SET is
                 * idempotent; losing it left DTR0 holding the PREVIOUS pair's
                 * value, so its paired query returned that instead and the
                 * probe scored a stale read — measured as LAG-1 in 4 of 6
                 * reboot drills, with the SET reported lost in the same run.
                 * The query was never wrong; the write it depended on had been
                 * thrown away.
                 *
                 * Order matters below: resend_or_lose leaves the command
                 * CMD_PENDING, and dispose_tracked only touches ACCEPTED and
                 * SENDING, so the resend survives the note_reboot that
                 * follows. */
                if (c->idempotent) resend_or_lose(s, c, MBQ_R_LOST);
                else               deliver(s, c, MBQ_R_LOST, 0);
                if (changed) note_reboot(s, '4', NULL);
            } else {
                resend_or_lose(s, c, MBQ_R_LOST);   /* genuinely never landed */
            }
            s->st = S_IDLE;
            break;
    }
}

/* The boot_count arbiter could not be read — either the link is down
 * (LINK_DOWN_FAILS consecutive no-replies) or the gateway keeps answering the
 * sysinfo read with a Modbus exception (MAX_SYSINFO_EXCEPTIONS in a row; an
 * exception is an ANSWER, so it never trips the link-down path on its own).
 * Every reason we were reading it wanted to know "did this command run?", and
 * the answer is now unknowable either way — so take the disposition that
 * assumes it might have. */
static void bootchk_give_up(slave_t *s) {
    LOG_LINK(s, "slave %u: boot_count unreadable — disposing conservatively",
             s->slave_id);
    s->bc_valid = false;
    s->sysinfo_exc = 0;   /* fresh budget the next time S_BOOT_CHECK is entered */
    if ((s->bc_reason == BC_ACKLESS || s->bc_reason == BC_R1_EMPTY)
        && s->rec_cmd != NO_CMD) {
        /* Both of these are "did this command run?" questions, and the arbiter
         * that would have answered is unreachable. Assume it might have. */
        deliver(s, &s->cmd[s->rec_cmd], MBQ_R_LOST, 0);
        s->rec_cmd = NO_CMD;
    } else if (s->bc_reason == BC_R4) {
        dispose_tracked(s, NULL);
    }
    s->tx_deadline_seq = 0;
    s->st = S_IDLE;
}

/* Every §4 path must terminate. The R1/R1' branch resolves in a single poll
 * when the gateway answers at all — but if it has stopped answering, nothing
 * in the branch itself ever gives up, and the command's callback never fires.
 * That does not merely lose the command: dali_queue releases its pending
 * record only on that callback, so the slave's DALI queue stays wedged until
 * the controller reboots. Bound it with the same result deadline the ack-less
 * branch already uses, and take the ordinary conservative disposition. */
static void recovery_give_up(slave_t *s) {
    cmd_t *c = (s->rec_cmd != NO_CMD) ? &s->cmd[s->rec_cmd] : NULL;
    s->rec_cmd = NO_CMD;
    s->st = S_IDLE;
    if (!c || c->st != CMD_SENDING) return;
    s->stats.deadlines++;

    /* A recovery that timed out because the gateway said NOTHING AT ALL is not
     * evidence the command needs re-executing — nothing was re-offered to
     * anything, so no DALI frame happened. Charging it to MAX_RESENDS, which
     * exists to bound RE-EXECUTIONS, spends the whole budget on a gateway that
     * is merely absent for a moment.
     *
     * That is what a reboot looks like: the recovery bound is one result
     * deadline (150 ms), a gateway restart takes on the order of a second, so
     * ~10 give-ups arrive back to back and 3 resends are gone inside half a
     * second. The command is then dropped as LOST — and for the soak's SET
     * that leaves DTR0 holding the previous pair's value, so its paired query
     * reads stale and the probe scores LAG-1. Measured in 3-4 of every 6
     * reboot drills, with the log showing four "timed out unresolved" lines
     * followed by "resent 3 times without a result — giving up".
     *
     * So while the link is down, re-queue WITHOUT spending the budget, and
     * bound the command in TIME instead. The transport's exponential backoff
     * already stops this costing bus, and CMD_MAX_LIFE_MS guarantees a
     * terminal disposition for a gateway that never comes back — which is the
     * property the recovery path must not lose. */
    bool failing  = (s->c_consec_fail > 0);
    bool too_old  = (MBQ_NOW_US() - c->born_us) >= (uint64_t)CMD_MAX_LIFE_MS * 1000u;

    /* A link that has been down long enough to be written off (LINK_QUIET_MS)
     * is not a gateway mid-restart, and re-queueing into it just repeats this
     * whole procedure until CMD_MAX_LIFE_MS. Fail it here instead — same
     * terminal verdict, 8 s sooner, and charged to link_down_drops because the
     * protocol lost nothing. Above the ride-out window this is unchanged. */
    if (link_written_off(s)) {
        deliver_as(s, c, MBQ_R_LOST, 0, &s->stats.link_down_drops);
        return;
    }

    if (c->idempotent && failing && !too_old) {
        c->seq = 0;
        c->st  = CMD_PENDING;
        return;                       /* deliberately NOT counted as a resend */
    }

    LOG_LINK(s, "slave %u: recovery for %02X%02X timed out unresolved — "
             "disposing", s->slave_id, c->cmd[0], c->cmd[1]);
    if (c->idempotent) resend_or_lose(s, c, MBQ_R_LOST);
    else               deliver(s, c, MBQ_R_LOST, 0);
}

static void resolve_version(slave_t *s, const uint8_t *b, uint8_t n) {
    /* "DALI v<maj>.<min> <20 hex chars>" in a 16-register block: addr + fc +
     * bytecount + 32 data + 2 CRC = 37 bytes. */
    if (n < 37u) { s->st = S_DRAIN; return; }   /* unreadable: let behaviour decide */
    const char *v = (const char *)(b + 3);
    const uint8_t vlen = (uint8_t)(GW_VERSION_REGS * 2u);   /* the 32 data bytes */
    unsigned maj = 0, min = 0;
    /* Every scan below is bounded by vlen, and nothing leans on the block
     * containing a NUL or a '.'/' ' terminator — it is a FREE-FORM string from
     * off-device, so it gets no more trust than any other gateway reply. The
     * previous `while (*p >= '0' ...)` walked a raw pointer with no end: the
     * 'v' can match as late as v[30], so a digit run to the end of the field
     * would read past cb_buf[40] and on into the rest of slave_t (same class
     * as the jfind overread fixed in a8d30f4). Digits also saturate at 255
     * instead of wrapping, so the packed compare below and the uint8_t stored
     * in gw_major/gw_minor can never disagree about what was reported — a
     * "v256.0" used to compare as compatible while reporting itself as v0.0. */
    uint8_t i = 0;
    bool have_ver = false;
    for (; i + 1u < vlen; i++)
        if (v[i] == 'v' && v[i + 1u] >= '0' && v[i + 1u] <= '9') { i++; have_ver = true; break; }
    if (have_ver) {
        for (; i < vlen && v[i] >= '0' && v[i] <= '9'; i++) {
            maj = maj * 10u + (unsigned)(v[i] - '0');
            if (maj > 255u) maj = 255u;
        }
        if (i < vlen && v[i] == '.') {
            for (i++; i < vlen && v[i] >= '0' && v[i] <= '9'; i++) {
                min = min * 10u + (unsigned)(v[i] - '0');
                if (min > 255u) min = 255u;
            }
        }
        s->gw_major = (uint8_t)maj;
        s->gw_minor = (uint8_t)min;
        s->stats.gw_ver_major = s->gw_major;
        s->stats.gw_ver_minor = s->gw_minor;
        /* Packed compare, so the minimum can be set to a x.0 version without
         * the "unsigned < 0" that a two-term comparison would fold to. */
        bool too_old = (((maj << 8) | (min & 0xFFu))
                        < ((GW_MIN_MAJOR << 8) | GW_MIN_MINOR));
        if (too_old) {
#if GW_VER_LATCHES
            LOG_ERR("mbq", "slave %u reports gateway v%u.%u; queue v4 needs "
                    ">= v%u.%u", s->slave_id, maj, min, GW_MIN_MAJOR, GW_MIN_MINOR);
            latch_incompatible(s, "gateway firmware predates queue v4");
            return;
#else
            LOG_WARN("mbq", "slave %u reports gateway v%u.%u, queue v4 expects "
                     ">= v%u.%u — driving it anyway; a wrong-generation gateway "
                     "will be caught by its reply shape",
                     s->slave_id, maj, min, GW_MIN_MAJOR, GW_MIN_MINOR);
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

    bool down = link_down(s);
    pace_backoff(s);

    if (s->tx_kind == TX_VERSION) {
        s->st = S_DRAIN;   /* identification is advisory; do not block on it */
        return;
    }

    if (s->tx_kind == TX_SYSINFO) {
        /* The arbiter itself is unreachable. Retrying is right up to a point,
         * but a command cannot be left unresolved forever waiting for a gateway
         * that is gone — so once the link is declared down, classify the way an
         * unreadable boot_count demands: conservatively, as if it had changed. */
        if (s->cb_res < 0) {
            /* An EXCEPTION reply — e.g. a register-map skew that still reports
             * a queue-v4-compatible version at 0x0200 — does not move
             * c_consec_fail (see mbq_modbus_cb), so `down` alone cannot bound
             * this case: a gateway that always exceptions this read would
             * otherwise never satisfy `down` and would sit in S_BOOT_CHECK
             * forever. Count consecutive exceptions here instead, and once
             * they exhaust the same small budget TX_VERSION already tolerates,
             * give up through the SAME path an unreadable boot_count uses —
             * so any command mid-arbitration (including one stranded there by
             * a BC_ACKLESS/BC_R1_EMPTY poll) resolves exactly per the §4
             * contract, conservatively, as-if-changed, regardless of which way
             * the arbiter became unreachable. */
            if (s->sysinfo_exc < 0xFFu) s->sysinfo_exc++;
            if (s->sysinfo_exc >= MAX_SYSINFO_EXCEPTIONS) { bootchk_give_up(s); return; }
        } else {
            s->sysinfo_exc = 0;
        }
        if (down) bootchk_give_up(s);
        return;
    }

    if (s->tx_cmd == NO_CMD) {
        /* A poll or ack-only frame. Idempotent by construction — re-acking a
         * popped seq is a no-op — so there is nothing to resolve, just retry
         * behind the backoff above. The deadline chase it may have been
         * carrying stays claimed: clearing it here is what let the chase
         * re-arm on every main-loop pass and count each pass as a fresh
         * deadline expiry. The per-command chase budget releases it instead. */
        if (s->st == S_DRAIN && ++s->drain_polls >= DRAIN_MAX_POLLS) {
            LOG_WARN("mbq", "slave %u: no answer to the epoch drain — proceeding "
                     "without a clean epoch", s->slave_id);
            s->st = S_IDLE;
        }
        return;
    }

    /* THE FAST-FAIL. A link written off (declared down, and down for longer
     * than a gateway restart takes) has already answered the only question the
     * recovery ladder would ask: the transport says NOTHING, so no frame
     * reached the gateway, so no DALI frame happened and there is no result to
     * reconcile. Running the ladder anyway costs ~8 s of held slots and five
     * WARN lines per attempt to reach the verdict it starts with — and against
     * a configured-but-absent slave, which dali_state probes every 5 s forever,
     * that is a permanent saga. Deliver the verdict now.
     *
     * NOT a shortcut around §4. Everything below LINK_QUIET_MS still runs the
     * full ladder, which is where R1/R1' live and where a single lost frame on
     * an otherwise healthy link is resolved — that case is scar tissue and this
     * does not touch it. Counted as link_down_drops, not `lost`: the protocol
     * lost nothing, there was no wire for it to lose anything on. */
    if (link_written_off(s)) {
        cmd_t *c = &s->cmd[s->tx_cmd];
        LOG_DBG("mbq", "slave %u: link down, failing %02X%02X without recovery",
                s->slave_id, c->cmd[0], c->cmd[1]);
        deliver_as(s, c, MBQ_R_LOST, 0, &s->stats.link_down_drops);
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
                s->ack_pending = 0;
                if (s->drain_pass < 2u) {
                    /* First sweep done. Let a result deadline pass so anything
                     * still executing from the previous controller epoch has
                     * surfaced, then sweep again. */
                    s->drain_pass    = 2u;
                    s->drain_polls   = 0;
                    s->drain_wait_us = MBQ_NOW_US() +
                                       (uint64_t)MBQ_RESULT_DEADLINE_MS * 1000u;
                    return;
                }
                s->stats.drained = true;
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
            /* Between the two sweeps, wait one result deadline out (spec §4).
             * A command the gateway accepted before the controller died is
             * still executing, and its result lands AFTER the first sweep —
             * where a fresh controller, tracking nothing, would read it as an
             * untracked seq and declare a gateway reboot that never happened.
             * The gateway holds at most CMDQ_DEPTH commands and each finishes
             * within a frame time, so nothing from the dead epoch survives a
             * deadline of draining. The wait comes AFTER the first sweep
             * because a response queue that was full when the controller died
             * has the arbiter stalled until that sweep unstalls it. */
            if (now < s->drain_wait_us || now < s->next_at_us) return;
            issue_queue_txn(s, NO_CMD);
            return;

        case S_BOOT_CHECK:
            if (now < s->next_at_us) return;
            s->stats.boot_reads++;
            issue_read(s, TX_SYSINFO, GW_SYSINFO_ADDR, GW_SYSINFO_REGS);
            return;

        case S_RECOVERY_POLL:
            /* Bounded. Every other §4 path terminates; without this one the
             * client polls a gateway that has stopped answering forever, and
             * the command's callback never fires — which permanently wedges
             * the slave, because dali_queue's pending record is only released
             * by that callback. */
            if (now >= s->rec_deadline_us) { recovery_give_up(s); return; }
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
    uint8_t pi = find_next_pending(s);
    if (pi != NO_CMD) {
        if (now < s->next_at_us) return;
        /* Only claim AWAIT_REPLY if something actually went out: a refusal from
         * a full Modbus action queue leaves nothing on the wire, and a state
         * that says otherwise would report a phantom transaction and block
         * submits until the next tick noticed. */
        if (issue_queue_txn(s, pi)) s->st = S_AWAIT_REPLY;
        return;
    }

    /* A result deadline that has expired: chase it with a poll, and if the
     * result still is not there, escalate to the boot_count arbiter (R4). The
     * chase respects the backoff gate and has a per-command budget — the
     * absence of both is what turned one unanswerable command into ~30 000
     * transactions and an equal number of bogus deadline expiries. */
    for (uint8_t i = 0; i < MBQ_CMD_SLOTS; i++) {
        cmd_t *c = &s->cmd[i];
        if (c->st != CMD_ACCEPTED || now < c->deadline_us) continue;
        if (c->chases >= MAX_DEADLINE_CHASES) {
            /* Out of budget: stop asking and let boot_count decide. */
            s->tx_deadline_seq = c->seq;
            s->bc_reason = BC_R4;
            s->st = S_BOOT_CHECK;
            return;
        }
        if (now < s->next_at_us) return;
        if (c->chases == 0) s->stats.deadlines++;   /* count the command, not the poll */
        c->chases++;
        s->tx_deadline_seq = c->seq;
        if (issue_queue_txn(s, NO_CMD)) s->st = S_AWAIT_REPLY;
        return;
    }

    /* Results outstanding, or an ack still owed: poll. There is nothing to
     * predict and nothing to wait for — Modbus is master-slave, so a result
     * only arrives when we ask, and asking sooner gets it sooner.
     *
     * This used to defer the poll to an ESTIMATE of when the oldest command
     * would finish, and yield 2 ms first in case the scheduler had a command
     * to send instead. That bought one transaction per DALI frame rather than
     * two, and cost the estimator, the yield latch, and the slot-starvation
     * defect they produced between them. The command path above already
     * prefers an enqueue whenever one is waiting, which is the part that
     * actually mattered: a poll only happens when there is genuinely nothing
     * to send. */
    if (count_state(s, CMD_ACCEPTED) > 0 || s->ack_pending != 0) {
        s->st = S_DEADLINE_WAIT;
        if (now < s->next_at_us) return;      /* transport backoff only */

        /* Before spending a transaction on a POLL, give the layer above first
         * refusal — only it knows whether a command is waiting, and an enqueue
         * does the poll's job for free by carrying the ack and returning the
         * head. This is NOT pacing and involves no estimate: it is a
         * yield-to-producer, and it lasts one scheduler tick, not a DALI frame.
         *
         * Removing it entirely was measured and is much worse than it looks:
         * the client polls back-to-back, so `mbq_client_can_submit()` is false
         * almost always (a transaction is on the wire), the producer cannot
         * hand over a command, and the bus fills with polls that a single
         * enqueue would have replaced — 5.5 transactions per command in the
         * host model, with commands waiting behind polls. */
        if (have_free_slot(s)) {
            if (s->poll_yield_us == 0) {
                s->poll_yield_us = now + (uint64_t)PACE_YIELD_MS * 1000u;
                return;
            }
            if (now < s->poll_yield_us) return;
        }
        s->poll_yield_us = 0;
        if (issue_queue_txn(s, NO_CMD)) s->st = S_AWAIT_REPLY;
        return;
    }

    s->poll_yield_us = 0;
    s->st = S_IDLE;   /* nothing owed in either direction: no traffic at all */
}

/* TWO PHASES, with producers running between them (see dq_tick):
 *     collect  — take the reply in, complete commands, free the wire
 *     ...producers queue whatever they have...
 *     drive    — send one packet, with a queued command attached if there is
 *                one and the owed ack if there is one
 *
 * Driven as a single pass the loop decides BEFORE producers have had their
 * turn, so after every reply it finds an empty queue and sends a packet with
 * nothing attached — measured as exactly one bare poll per command. */
void mbq_client_collect(void) {
    for (int i = 0; i < MAX_DALI_SLAVES; i++) {
        slave_t *s = &s_sl[i];
        if (!s->in_use || !s->busy || !s->cb_fired) continue;
        s->cb_fired = false;
        s->busy     = false;
        s_resolving = true;
        handle_reply(s);
        s_resolving = false;
        if (s->st == S_AWAIT_REPLY) s->st = S_IDLE;
        s->tx_cmd = NO_CMD;
    }
}

/* Drive every eligible slave. The FIRMWARE does not call this — the transport
 * asks each slave for work in rotation (mbq_offer / modbus_register_offer), so
 * this exists for the host test, which has no transport to be asked by.
 *
 * A round-robin cursor lived here briefly, on the theory that a fixed scan
 * would let slave 0 starve slave 1. It could not: drive() ENQUEUES rather than
 * seizing the wire, so every eligible slave got a turn in the same pass and the
 * shared FIFO served them in order. The rotation that does matter is the
 * transport's, one level down. */
void mbq_client_drive(void) {
    for (int i = 0; i < MAX_DALI_SLAVES; i++) {
        slave_t *s = &s_sl[i];
        /* Never drive a slave with a transaction already on the wire: a
         * producer may have queued between the phases and mbq_client_submit()
         * transmits when it can, so driving anyway puts a SECOND transaction
         * on top. Measured with a weaker guard: 14.8 -> 0.13 pairs/s. */
        if (!s->in_use || s->busy) continue;
        drive(s);
    }
}

void mbq_client_tick(void) {
    mbq_client_collect();
    mbq_client_drive();
}

/* ── public API ──────────────────────────────────────────────────────────── */

void mbq_client_init(void) {
    memset(s_sl, 0, sizeof s_sl);
    s_resolving = false;
}

#ifndef MBQ_CLIENT_HOST_TEST
/* Offered to the bus when it falls idle and it is this slave's turn. drive()
 * is unchanged — it already was "if this slave has something to do, do it";
 * the only difference is that it is now ASKED rather than free to push. */
static bool mbq_offer(void *ctx) {
    slave_t *s = (slave_t *)ctx;
    if (!s->in_use || s->busy) return false;
    drive(s);
    return s->busy;          /* true only if it actually took the wire */
}
#endif

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
        s->drain_pass = 1u;
        s->next_at_us = 0;
#ifndef MBQ_CLIENT_HOST_TEST
        /* Join the bus rotation. The host test drives the client directly via
         * mbq_client_tick(), so it does not register. */
        modbus_register_offer(mbq_offer, s);
#endif
        return;
    }
    LOG_WARN("mbq", "no room to register slave %u", slave_id);
}

bool mbq_client_can_submit(uint8_t slave_id) {
    slave_t *s = slave_for(slave_id);
    if (!s || s_resolving) return false;
    if (s->st == S_INCOMPATIBLE) return false;
    /* ROOM IN THE QUEUE. That is the whole test.
     *
     * NOT whether a transaction is on the wire, NOT what the recovery state
     * machine is doing, NOT how full the gateway is — the gateway answers that
     * itself with A_FULL. Accepting a command is bookkeeping; transmitting it
     * is mbq_client_drive()'s job, and coupling the two starves every producer
     * that is not interleaved into dq_tick's ship phase: a transaction lasts
     * ~12 ms and the controller runs ~340 loop passes inside one, so a
     * producer had to CATCH A WINDOW rather than simply hand work over.
     *
     * NOTE this is no longer a READINESS signal. It is true from the first
     * instant, including during the startup drain — a producer may queue work
     * then and the client transmits it once the drain finishes. Callers that
     * want "is the client up" should look at mbq_client_stats().drained. */
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
    c->chases     = 0;
    c->ord        = s->next_ord++;   /* producer order, kept across any resend */
    c->born_us    = MBQ_NOW_US();    /* bounds recovery in time, see CMD_MAX_LIFE_MS */
    c->st         = CMD_PENDING;

    /* QUEUED, NOT TRANSMITTED. mbq_client_drive() attaches it to the next
     * packet. Accepting work and putting it on the wire are different jobs,
     * and this one always succeeds once there is a slot. */
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
    if (!s) return;
    *out = s->stats;
    /* Live figures rather than counters. Without a completion estimate the
     * client can no longer guess how many results are SITTING in the gateway's
     * response queue — it only knows about the one it has consumed and not yet
     * acked. That is enough for the backpressure drill to be legible from this
     * side (`unacked` climbs under `mbq no-ack`), and it has the merit of
     * being observed rather than inferred. `inflight` is what the client is
     * genuinely tracking: commands accepted whose results have not arrived. */
    out->inflight = count_state(s, CMD_ACCEPTED);
    out->unacked  = (s->ack_pending != 0) ? 1u : 0u;
    out->respq_est = out->unacked;
}

uint16_t mbq_client_consec_fail(uint8_t slave_id) {
    slave_t *s = slave_for(slave_id);
    return s ? s->c_consec_fail : 0u;
}

bool mbq_client_no_ack(uint8_t slave_id, uint16_t n) {
    slave_t *s = slave_for(slave_id);
    if (!s) return false;
    s->withhold_acks = n;
    LOG_WARN("mbq", "slave %u: withholding the ack on the next %u requests (drill)",
             slave_id, n);
    return true;
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
                       s->stats.drained ? "" : " EPOCH-UNCLEAN");
        if (link_down(s))
            cdc_out_printf("  LINK DOWN for %lu ms (%u consecutive no-reply)%s\n",
                           (unsigned long)((MBQ_NOW_US() - s->link_down_us) / 1000u),
                           (unsigned)s->c_consec_fail,
                           link_written_off(s) ? " — failing submissions fast" : "");
        mbq_stats_t live;
        mbq_client_stats(s->slave_id, &live);
        cdc_out_printf("  ack_pending=%u head=%u last_seq=%u%s  respq~%u occupancy=%u/%u\n",
                       s->ack_pending, s->last_head, s->last_seq,
                       s->seq_known ? "" : " (no baseline)",
                       live.respq_est, count_state(s, CMD_ACCEPTED), CMDQ_DEPTH);
        cdc_out_printf("  commands:");
        for (uint8_t k = 0; k < MBQ_CMD_SLOTS; k++)
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
        cdc_out_printf("  reboots: R2=%lu R3=%lu R4=%lu\n",
                       (unsigned long)t->reboots_r2, (unsigned long)t->reboots_r3,
                       (unsigned long)t->reboots_r4);
        /* Both should read 0 forever. A prediction fault is a detected
         * mis-binding — the class of defect that took task 04 three cycles —
         * and orphans past the first moments after a restart mean results are
         * arriving that this controller cannot account for. */
        cdc_out_printf("  faults: predicted-seq=%lu orphans=%lu protocol=%lu\n",
                       (unsigned long)t->pred_faults, (unsigned long)t->orphans,
                       (unsigned long)t->proto_errors);
        cdc_out_printf("  results: complete=%lu failed=%lu lost=%lu result_lost=%lu "
                       "nak=%lu link_down=%lu\n",
                       (unsigned long)t->completed, (unsigned long)t->failed,
                       (unsigned long)t->lost, (unsigned long)t->result_lost,
                       (unsigned long)t->naks, (unsigned long)t->link_down_drops);

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
