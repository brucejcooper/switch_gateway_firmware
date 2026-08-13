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

/* Gateway command-queue depth, and therefore the master's in-flight cap.
 * MUST match the gateway's CMDQ_DEPTH (TinyModbus/src/cq.h) — controller and
 * gateway are a matched firmware set with no negotiation and no fallback
 * (spec §8), so deploy them together. Two is "executing + next", which is all
 * a one-frame-at-a-time DALI bus can use; the wire format is depth-agnostic,
 * so raising it is a one-line change on both sides and nothing on the wire
 * moves. The cap is the flow control — A_FULL is only the backstop (task-07
 * lesson). */
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

/* TWO CAPACITIES, SEPARATE, BECAUSE THEY MEAN DIFFERENT THINGS.
 *
 *   MBQ_QUEUE_DEPTH   the QUEUE: commands accepted from a producer and not yet
 *                     offered to the gateway (CMD_PENDING). This is OUR policy
 *                     — how much backlog we are willing to hold so the
 *                     transmission loop always has something to attach.
 *
 *   MBQ_INFLIGHT_MAX  IN FLIGHT: commands at the gateway or on the wire
 *                     (CMD_ACCEPTED / CMD_SENDING). This is NOT policy and we
 *                     do not enforce it — the GATEWAY bounds its own queue and
 *                     says A_FULL. It is tracking capacity only: at most
 *                     CMDQ_DEPTH sit at the gateway and one more can be on the
 *                     wire, so CMDQ_DEPTH + 1 is what we must be able to
 *                     remember.
 *
 * They were one number (MBQ_CMD_SLOTS = CMDQ_DEPTH + n) holding both, which is
 * why "how much backlog?" and "how deep is the gateway?" could not be answered
 * independently — and why in-flight work could fill the array and leave the
 * loop with nothing to attach, so it sent a packet with no command in it.
 *
 * The array is their sum. Nothing else should derive from CMDQ_DEPTH here. */
/* Four measured best on the bench: the loop only needs ONE queued command to
 * keep every packet carrying work, and deeper only adds queueing latency
 * (queue 8 measured 14.85 pairs/s against 15.12 at 4, with the gateway
 * unchanged — which is also the check that these two really are independent). */
#define MBQ_QUEUE_DEPTH   4u
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
} mbq_result_t;

/* Delivered once when a submitted command resolves. Runs from mbq_client_tick()
 * context (core-0 main loop). cb may be NULL for fire-and-forget commands. */
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

/* True if `slave_id` can accept a new command this instant: it is registered
 * and compatible, has finished draining, holds fewer than CMDQ_DEPTH commands,
 * has no transaction in flight and no command already waiting to be (re)sent.
 * The adapter's scheduler gates a submit on this. */
bool mbq_client_can_submit(uint8_t slave_id);

/* Submit one command to `slave_id`'s gateway queue and issue the transaction.
 * `cmd` is [addr, data, flags (MBQ_QF_*), dtr0]. `idempotent` = safe to run
 * twice, so a reboot or a never-landed frame resubmits it rather than
 * reporting MBQ_R_LOST; queries and absolute SETs are, UP/DOWN steps are not.
 * cb/ctx receive the result exactly once. Returns false and enqueues nothing if
 * !mbq_client_can_submit (caller retries next tick). */
bool mbq_client_submit(uint8_t slave_id, const uint8_t cmd[MBQ_CMD_BYTES],
                       bool idempotent, mbq_cb_t cb, void *ctx);

/* Per-loop driver: advance every slave's FSM — resolve a completed
 * transaction through the §4 rules, deliver terminals once, arm and check
 * result deadlines, and issue the next request (enqueue, ack/poll, recovery
 * poll or boot_count read) when its pacing timer allows. Call every main-loop
 * iteration. */
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
 * Modbus EXCEPTION does not count — that is the gateway alive and disagreeing. */
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
