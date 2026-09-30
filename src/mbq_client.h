#pragma once
/*
 * mbq_client.h — controller-side transport for the gateway's DALI command
 * queue, protocol v4. NORMATIVE REFERENCE: TinyModbus/QUEUE-PROTOCOL.md.
 *
 * One user-defined function code (0x41) carries all queue traffic. A request
 * optionally ENQUEUES one command and optionally ACKS the head of the gateway's
 * response queue; the reply reports the fate of the enqueue (an accept byte
 * carrying a gateway-assigned sequence) and the current head of the response
 * queue. Results are RETAINED by the gateway until acked, so no lost reply can
 * destroy one. In steady state each transaction enqueues one command, acks the
 * previous result and collects the next: one ~14-byte transaction per DALI
 * frame.
 *
 * Sequence numbers (1-31; 0 means "none") are assigned EXCLUSIVELY by the
 * gateway. The master never proposes one — it learns each command's identity
 * from the accept byte. That is what makes a gateway reboot self-correcting:
 * the counter restarts and the master simply learns the new numbers. There is
 * no ledger to reconcile, which was v1's fatal flaw, and no optimistic slot
 * mirror to mis-attribute a status to, which was v2's.
 *
 * The client is DALI-aware only to the extent the wire forces it to be: the
 * 4-byte command it carries is [addr, data, flags, dtr0], and `flags` must be
 * packed into the request's top three bits while `dtr0` is present on the wire
 * only when MBQ_QF_DTR0 is set. The MEANING of addr/data/dtr0 and of the
 * answer byte still belongs entirely to the caller (dali_queue.c). The
 * per-slave priority scheduler and the producer API live ABOVE this client.
 *
 * RECOVERY (spec §4) is the whole of this module's reconcile logic — there is
 * no other path, and deliberately no fallback:
 *   R1   an enqueue+ack whose reply was lost, then a poll showing the head has
 *        moved past the acked seq: the ack was applied, and since ack and
 *        enqueue travel in ONE frame that proves the whole frame landed. Bind
 *        the command to the predicted seq (last accepted + 1).
 *   R1'  same, but the head still shows the acked seq: the frame never
 *        arrived. Resend it verbatim, ack included — no idempotency concern.
 *   R2   an accept byte returning a seq the master still tracks. A live seq
 *        cannot recur without a reboot (spec §2), so the gateway rebooted.
 *   R3   a head seq the master is not tracking. It has no other source.
 *   R4   a result deadline expiring with no result: poll, then classify
 *        against the gateway's sysinfo boot_count (FC04 0x0310, cold path).
 * The one ambiguous corner — an ACK-LESS enqueue whose reply was lost, which
 * only happens on a cold start or after idle — cannot use R1, so it polls
 * until the predicted seq appears as a result (landed) or the deadline passes
 * with the response queue observed empty (never landed), and consults
 * boot_count before resending a non-idempotent command down that path.
 */

#include <stdbool.h>
#include <stdint.h>

/* The GATEWAY's command-queue depth. MUST match the gateway's CMDQ_DEPTH
 * (TinyModbus/src/cq.h) — controller and gateway are a matched firmware set
 * with no negotiation and no fallback (spec §8), so deploy them together. Two
 * is "executing + next", which is all a one-frame-at-a-time DALI bus can use;
 * the wire format is depth-agnostic, so raising it is a one-line change on
 * both sides and nothing on the wire moves.
 *
 * This is NOT a client-side admission cap — the client has none. A_FULL IS
 * the flow control (spec §5, decided 2026-08-12): the client submits until
 * the gateway refuses and re-offers immediately. What this constant actually
 * bounds is MBQ_INFLIGHT_MAX below — how many commands the client must be
 * able to REMEMBER, not a limit it imposes on itself. */
#define CMDQ_DEPTH     2u
#define MBQ_CMD_BYTES  4u   /* one command: addr, data, flags, dtr0 */

/* Request flag bits, at their WIRE positions (spec §1: control in the top 3
 * bits of the byte, sequence in the low 5). These moved from v2's payload-byte
 * values 0x01/0x02/0x04 — dali_queue's DQ_* aliases point here so the command
 * byte the producer builds IS the wire flags byte and nothing translates in
 * between. Getting this wrong does not fail loudly; it silently changes the
 * meaning of every command.
 *
 * PEER DEFINITION: TinyModbus/src/fc41.h. These constants and the status codes
 * below are mirrored there for the gateway, in a separate repo — so nothing
 * makes them agree except this note. Change one, change the other; the spec
 * (QUEUE-PROTOCOL.md §1) stays the authority for both. */
#define MBQ_QF_TWICE   0x80u   /* send the forward frame twice; no answer expected */
#define MBQ_QF_DTR0    0x40u   /* emit 0xA3 <dtr0> back-to-back BEFORE the frame */
#define MBQ_QF_ANSWER  0x20u   /* a backward frame is expected; capture the answer */
#define MBQ_QF_MASK    0xE0u

/* An accepted command must produce a result within this long, or rule R4
 * classifies it as lost to a reboot or a gateway wedge. Worst queueing at
 * depth 2 is one ~50 ms DTR0-pair ahead of a ~50 ms one of your own, times a
 * 1.5 margin (spec §4). Tunable — raise it before suspecting the protocol if a
 * slower gateway ever reports spurious R4 expiries. */
#define MBQ_RESULT_DEADLINE_MS  150u

/* ONE knob, and it is deliberately as small as it can be.
 *
 *   MBQ_QUEUE_DEPTH   STAGING. Commands accepted from the scheduler and not
 *                     yet offered to the gateway. ONE is enough and one is
 *                     what we want: the transmission loop only needs a command
 *                     available at the instant it builds a packet, and
 *                     dq_tick refills this slot between every transaction
 *                     (collect -> ship -> drive), so one slot keeps every
 *                     packet carrying work.
 *
 *                     DEEPER IS WORSE, NOT BETTER. dali_queue already holds
 *                     the real backlog, sorted by PRIORITY. A command staged
 *                     here has left that ordering behind — it is committed,
 *                     FIFO, and a later high-priority command cannot overtake
 *                     it. Every extra staging slot is therefore a DALI frame
 *                     of latency added to the next button press. Four slots
 *                     measured no faster than one and let background work hold
 *                     three of them.
 *
 *   MBQ_INFLIGHT_MAX  NOT A KNOB — derived. At most CMDQ_DEPTH commands sit at
 *                     the gateway (it says A_FULL beyond that) plus one on the
 *                     wire awaiting its accept byte. This is how many we must
 *                     be able to REMEMBER, not a policy we choose.
 */
#define MBQ_QUEUE_DEPTH   1u
#define MBQ_INFLIGHT_MAX  (CMDQ_DEPTH + 1u)
#define MBQ_CMD_SLOTS     (MBQ_QUEUE_DEPTH + MBQ_INFLIGHT_MAX)

/* Terminal outcome delivered to a submitted command's callback, exactly once. */
typedef enum {
    MBQ_R_COMPLETE    = 0,  /* the command succeeded; response8 = its answer byte  */
    MBQ_R_FAILED      = 1,  /* the command failed;    response8 = the failure code */
    MBQ_R_LOST        = 2,  /* never executed, or unknowably so, and not safe to
                             * repeat: a non-idempotent command caught by a reboot
                             * or a never-landed frame we may not blind-resend    */
    MBQ_R_RESULT_LOST = 3,  /* the command RAN and its answer died with the
                             * gateway's response queue. Distinct from LOST
                             * because the bus state changed and a caller that
                             * would retry on LOST must not retry on this.       */
    MBQ_R_NAK         = 4,  /* the command RAN and EXPECTED a reply; none came.
                             * response8 = 0. This is a FACT, not a verdict:
                             * silence means "no" to a yes/no query and "absent"
                             * to a value query, and only the issuer knows which
                             * it asked. Deliberately NOT MBQ_R_FAILED — nothing
                             * broke, so failure counters must not count it, and
                             * for the same reason it has its own `naks` stat
                             * rather than falling into `lost`.                  */
} mbq_result_t;

/* Delivered once when a submitted command resolves. Fires from one of TWO
 * core-0 main-loop contexts: mbq_client_collect() (the normal path, called
 * every iteration via dq_tick, with s_resolving held for its duration) or
 * from drive()'s recovery-give-up when a stalled §4 recovery poll's own
 * deadline expires (reached via the bus offer callback inside modbus_poll(),
 * NOT under s_resolving). See mbq_client_collect()/mbq_client_drive() below
 * for what production actually calls and why the second context is safe. cb
 * may be NULL for fire-and-forget commands. */
typedef void (*mbq_cb_t)(uint8_t slave_id, mbq_result_t result, uint8_t response8,
                         void *ctx);

/* Reset all per-slave client state. Call once at boot before adding slaves. */
void mbq_client_init(void);

/* Register a Modbus slave that hosts a v4 command queue. The adapter calls this
 * once per configured gateway after mbq_client_init(). The slave starts in the
 * DRAIN state (spec §4, "controller reboot"): poll, ack whatever head appears,
 * repeat until empty, then read boot_count — which both clears results left by
 * a previous controller epoch and establishes the reference the recovery rules
 * classify against. Stale acks from the dead epoch are harmless, because an ack
 * that does not match the head is a no-op. */
void mbq_client_add_slave(uint8_t slave_id);

/* True if `slave_id` can accept a new command this instant: it is registered,
 * compatible, and has a FREE SLOT in its command table. That is the whole test
 * — NOT whether a transaction is on the wire, NOT how full the gateway is
 * (A_FULL answers that), and NOT whether the startup drain has finished. It is
 * true from the first instant; work queued during the drain goes out when the
 * drain completes. Callers wanting "is the client up" want
 * mbq_client_stats().drained. The long-form rationale is on the definition in
 * mbq_client.c. The adapter's scheduler gates a submit on this. */
bool mbq_client_can_submit(uint8_t slave_id);

/* A user command was just submitted for this slave: go now, not after the
 * failure pacing (see modbus_backoff_kick). */
void mbq_client_kick(uint8_t slave_id);

/* Stage one command for `slave_id`'s gateway queue. QUEUED, NOT TRANSMITTED:
 * mbq_client_drive() attaches it to the next packet.
 * `cmd` is [addr, data, flags (MBQ_QF_*), dtr0]. `idempotent` = safe to run
 * twice, so a reboot or a never-landed frame resubmits it rather than
 * reporting MBQ_R_LOST; queries and absolute SETs are, UP/DOWN steps are not.
 * cb/ctx receive the result exactly once. Returns false and enqueues nothing if
 * !mbq_client_can_submit (caller retries next tick). */
bool mbq_client_submit(uint8_t slave_id, const uint8_t cmd[MBQ_CMD_BYTES],
                       bool idempotent, mbq_cb_t cb, void *ctx);

/* mbq_client_collect() takes in whatever reply arrived, resolves it through
 * the §4 rules, and delivers terminals once. Call every main-loop iteration —
 * dq_tick() does this before it ships anything, so a slot freed this tick is
 * available to the scheduler in the same tick (dali_queue.c).
 *
 * mbq_client_drive() issues the next request (enqueue, ack/poll, recovery
 * poll or boot_count read) for every eligible slave, when its pacing timer
 * allows. Production does NOT call this directly: the transport asks each
 * slave for work when the bus falls idle (modbus_register_offer / mbq_offer,
 * inside modbus_poll()). That offer path is also where a stalled §4 recovery
 * poll gives up and fires its callback — outside mbq_client_collect()'s
 * s_resolving guard (see mbq_cb_t above). That is safe: a submit triggered
 * from such a callback is pure bookkeeping (it stages a command; it does not
 * transmit), and the multi-command disposal s_resolving actually protects
 * against (dispose_tracked(), reachable only from inside handle_reply()) is
 * never called from drive()'s path.
 *
 * mbq_client_tick() is collect()+drive() in one call. Nothing in the firmware
 * calls it — it exists for the host test, which has no transport to offer it
 * work and so drives the client directly every iteration. */
void mbq_client_collect(void);
void mbq_client_drive(void);
void mbq_client_tick(void);

/* Commands of `slave_id`'s CMDQ_DEPTH the client is holding — waiting to be
 * sent, on the wire, or accepted and awaiting a result. This is the
 * controller's view of GATEWAY-SIDE queue occupancy across ALL producers, and
 * is what a producer pacing itself against the gateway queue (rather than
 * against its own submissions) should gate on. 0 if the slave is unknown. */
uint8_t mbq_client_inflight(uint8_t slave_id);

/* Modbus transactions this client has issued to `slave_id` since boot, split
 * into enqueue-carrying requests (`submits`) and 5-byte ack/poll requests
 * (`polls`). Bench instrumentation: transactions-per-DALI-frame is the number
 * v4 exists to drive to 1.0, and it is (submits + polls) / commands. Either
 * pointer may be NULL. */
void mbq_client_txn_counts(uint8_t slave_id, uint32_t *submits, uint32_t *polls);

/* Per-transaction wall time for `slave_id`, measured from the moment the
 * request is handed to the Modbus layer to the moment its callback fires — so
 * it INCLUDES time spent queued behind other producers' Modbus actions. `n` is
 * the number of completed transactions, times in µs; also reports how many
 * ended in a transport failure (no reply / exception). `n_busy` counted
 * DEVICE_BUSY under v2 and stays at 0 under v4, which has no exceptions on
 * this path at all — queue-full is the in-band A_FULL instead. */
typedef struct {
    uint32_t n;
    uint32_t us_min, us_max, us_avg;
    uint32_t n_fail, n_busy;
} mbq_txn_stats_t;
void mbq_client_txn_stats(uint8_t slave_id, mbq_txn_stats_t *out);

/* Zero the per-transaction timing accumulators (not the lifetime counters), so
 * a bench run measures only its own window. */
void mbq_client_txn_stats_reset(uint8_t slave_id);

/* The gateway's own queue counter block (spec §6), FC04, exactly 8 registers:
 * accepts, a_full, executes, results, acks, stalls, q_discard, spare. Read on
 * demand from the CLI, never polled hot — hot-path bytes are the scarce
 * resource and the soak's probe verifies itself without them. */
#define MBQ_GW_DIAG_ADDR  0x0320u
#define MBQ_GW_DIAG_REGS  8u

/* Per-slave protocol diagnostics (spec §6). The reboot count is split by which
 * rule noticed the reboot, because the split says HOW reboots are being seen —
 * a fleet that only ever notices them via R4 deadlines is one whose structural
 * rules are not firing, which is a finding in itself. */
typedef struct {
    uint32_t enqueues;        /* requests carrying an enqueue payload        */
    uint32_t polls;           /* 5-byte ack/poll requests                    */
    uint32_t recovery_polls;  /* of those polls, ones issued by §4 recovery  */
    uint32_t timeouts;        /* transactions with no valid reply            */
    uint32_t r1_landed;       /* R1:  lost frame had landed                  */
    uint32_t r1_never;        /* R1': lost frame never arrived; resent       */
    uint32_t reboots_r2;      /* reboot inferred from a colliding accept seq */
    uint32_t reboots_r3;      /* reboot inferred from an untracked head seq  */
    uint32_t reboots_r4;      /* reboot inferred at a deadline, via boot_count */
    uint32_t pred_faults;     /* R1 bound a command to a seq the gateway did not
                               * use — a DETECTED mis-binding, never silent    */
    uint32_t orphans;         /* results discarded as a previous controller
                               * epoch's leftovers (no-baseline state)         */
    uint8_t  inflight;        /* commands accepted whose result has not come
                               * back — what the client is actually tracking   */
    uint8_t  respq_est;       /* lower bound on gateway response-queue
                               * occupancy. Since the estimator went, this is
                               * only what we have consumed and not yet acked —
                               * observed rather than inferred. Climbs during
                               * the `no-ack` backpressure drill               */
    uint8_t  unacked;         /* results consumed but not yet proven popped    */
    uint32_t head_blocked;    /* replies seen with the head still holding a
                               * result we have consumed but not acked — the
                               * response queue backing up. Climbs during the
                               * `no-ack` drill and should be 0 otherwise      */
    uint32_t deadlines;       /* result deadlines that expired               */
    uint32_t afull;           /* A_FULL accept bytes (gateway queue full)    */
    uint32_t completed, failed;
    uint32_t lost, result_lost;
    uint32_t naks;            /* MBQ_R_NAK terminals — the gear did not answer,
                               * which is an ANSWER. These used to land in
                               * `lost` (the default: arm), and on a sparse bus
                               * that is ~62 phantom losses per scan: the
                               * counter that should say "a reboot ate work"
                               * instead reported the empty addresses.        */
    uint32_t link_down_drops; /* commands failed fast because the slave's link
                               * was already declared down and had stayed down
                               * past a gateway restart. The protocol lost
                               * nothing — there was no wire to lose it on —
                               * so these are kept out of `lost` too.         */
    uint32_t resends;         /* commands put back on the wire (R1', A_FULL, reboot) */
    uint32_t boot_reads;      /* cold-path sysinfo boot_count arbitrations   */
    uint32_t proto_errors;    /* wrong reply length / shape / bad accept status */
    bool     incompatible;    /* compat latch tripped — this slave is not driven */
    bool     drained;         /* the boot-time epoch drain completed          */
    uint16_t boot_count;      /* last sysinfo boot_count read (0 if never)   */
    uint8_t  gw_ver_major, gw_ver_minor;   /* parsed from FC04 0x0200, 0 if unread */
} mbq_stats_t;
void mbq_client_stats(uint8_t slave_id, mbq_stats_t *out);

/* Consecutive transactions to `slave_id` that got NO REPLY (0 = healthy). This
 * is the controller's best liveness signal for a gateway: the queue talks to it
 * continuously, so it notices a dead link far sooner than any periodic probe. A
 * Modbus EXCEPTION does not count — that is the gateway alive and disagreeing.
 *
 * LINK_DOWN_FAILS is the count at which the client DECLARES the link down and
 * changes its own behaviour (mbq_client.c). It lives here because dali_state
 * gates its liveness probe on the same threshold against the same counter, and
 * the two were separate `#define 3u`s that nothing kept in agreement. */
#define LINK_DOWN_FAILS 3u
uint16_t mbq_client_consec_fail(uint8_t slave_id);

/* Bench drill knob (spec §10 stage C3): discard the next `n` parsed replies
 * from `slave_id` inside the client — AFTER transport accounting, BEFORE
 * resolution — so the §4 recovery paths run for real against real hardware
 * instead of being simulated. Dropping the reply to an enqueue+ack drives
 * R1/R1'; dropping the reply to an ack-less first enqueue drives the deadline
 * and boot_count corner. Returns false for an unknown slave. */
bool mbq_client_drop_reply(uint8_t slave_id, uint16_t n);

/* Bench drill knob for the backpressure gate: withhold the piggybacked ack on
 * the next `n` requests to `slave_id`. The gateway's response queue fills, its
 * backpressure rule stalls the DALI arbiter rather than dropping a result, and
 * nothing is lost — resuming acks drains it and the rate recovers. Returns
 * false for an unknown slave. */
bool mbq_client_no_ack(uint8_t slave_id, uint16_t n);

/* Dump per-slave client state to the console (dev CLI). slave_id < 0 = all. */
void mbq_client_debug_print(int slave_id);
