#pragma once
/*
 * mbq_client.h — controller-side generic Modbus command-queue client (
 * queue v2). The controller mirror of the gateway's mbq_slots protocol.
 *
 * DALI-AGNOSTIC. It carries OPAQUE 4-byte commands to a slave's 8-slot queue and
 * delivers back an 8-bit response + a terminal outcome per command. It has no
 * knowledge of the payload — the meaning of the command bytes and of the response8
 * byte belongs to the caller (the DALI adapter in dali_queue.c). The per-slave
 * PRIORITY SCHEDULER and the producer API live ABOVE this client, in the adapter.
 * Protocol reference: docs/modbus-command-queue.md.
 *
 * Model (per slave): an OPTIMISTIC 8-slot cache mirroring the gateway's slots.
 * Submit = one FC23 (write the target slot's 2 command regs + read all 8 status
 * regs) — issued only when the adapter has a new command; otherwise a plain read
 * polls active slots. Every read-back RECONCILES the cache against the 8 live
 * statuses: a slot the cache owns that reads complete/failed is delivered to its
 * callback exactly once; an owned+confirmed slot reading idle is the gateway-reboot
 * "idle means gone" signal → fail ALL outstanding (deliver LOST) and resubmit the
 * idempotent ones. No id ledger, no resync/rebuild/rollback — the slot statuses
 * are the truth.
 */

#include <stdbool.h>
#include <stdint.h>

#define MBQ_SLOTS      8u   /* gateway queue depth (matches the gateway MBQ_SLOTS) */
#define MBQ_CMD_BYTES  4u   /* opaque command size (32-bit, 2 Modbus registers)   */

/* Terminal outcome delivered to a submitted command's callback (exactly once). */
typedef enum {
    MBQ_R_COMPLETE = 0,  /* slot reached complete; response8 = value byte  */
    MBQ_R_FAILED   = 1,  /* slot reached failed;   response8 = failure code */
    MBQ_R_LOST     = 2,  /* command lost (gateway reboot / never landed & non-idempotent) */
} mbq_result_t;

/* Delivered once when a submitted command resolves. Runs from mbq_client_tick()
 * context (core-0 main loop). cb may be NULL for fire-and-forget commands. */
typedef void (*mbq_cb_t)(uint8_t slave_id, mbq_result_t result, uint8_t response8,
                         void *ctx);

/* Reset all per-slave client state. Call once at boot before adding slaves. */
void mbq_client_init(void);

/* Register a Modbus slave that hosts an 8-slot command queue. The adapter calls
 * this once per configured gateway after mbq_client_init(). */
void mbq_client_add_slave(uint8_t slave_id);

/* True if `slave_id` can accept a new command right now: a cache slot is free
 * (idle or a collected terminal) AND no transaction is mid-flight (so a submit can
 * issue this instant). The adapter's scheduler gates a submit on this. */
bool mbq_client_can_submit(uint8_t slave_id);

/* Submit one opaque command into a free slot of `slave_id` and issue the FC23
 * write+read. `idempotent` = safe to resubmit on a gateway reboot (else a reboot
 * delivers MBQ_R_LOST). cb/ctx receive the result exactly once. Returns false and
 * enqueues nothing if !mbq_client_can_submit (caller retries next tick). */
bool mbq_client_submit(uint8_t slave_id, const uint8_t cmd[MBQ_CMD_BYTES],
                       bool idempotent, mbq_cb_t cb, void *ctx);

/* Per-loop driver: advance every slave's transaction SM — process a completed
 * FC23 (reconcile the 8 statuses, deliver terminals once, free collected slots,
 * handle idle-means-gone), pace a poll read while any slot is active, and re-issue
 * a resubmit queued by a reboot. Call every main-loop iteration. */
void mbq_client_tick(void);


/* Number of `slave_id`'s 8 gateway slots the client believes are occupied —
 * armed, written-but-unconfirmed, or confirmed active. This is the controller's
 * view of GATEWAY-SIDE queue occupancy across ALL producers, and is what a
 * producer pacing itself against the gateway queue (rather than against its own
 * submissions) should gate on: back off at MBQ_SLOTS/2 to keep the queue fed
 * without ever provoking a DEVICE_BUSY rejection. 0 if the slave is unknown. */
uint8_t mbq_client_inflight(uint8_t slave_id);

/* Modbus transactions this client has issued to `slave_id` since boot, split
 * into submits (FC23 write+read) and read-only status polls. Bench
 * instrumentation: transactions-per-command is what says whether a workload is
 * Modbus-bound or DALI-bound. Either pointer may be NULL. */
void mbq_client_txn_counts(uint8_t slave_id, uint32_t *submits, uint32_t *polls);

/* Per-transaction wall time for `slave_id`, measured from the moment the FC23 is
 * handed to the Modbus layer to the moment its callback fires — so it INCLUDES
 * time spent queued behind other producers' Modbus actions, and any retry the
 * transport did. Bench instrumentation for "is the command path Modbus-bound?".
 * `n` is the number of completed transactions, times in µs; also reports how many
 * ended in a transport failure (timeout / exception) and how many of those were
 * DEVICE_BUSY (0x06). Any pointer may be NULL. */
typedef struct {
    uint32_t n;
    uint32_t us_min, us_max, us_avg;
    uint32_t n_fail, n_busy;
} mbq_txn_stats_t;
void mbq_client_txn_stats(uint8_t slave_id, mbq_txn_stats_t *out);

/* Zero the per-transaction timing accumulators (not the lifetime counters), so a
 * bench run measures only its own window. */
void mbq_client_txn_stats_reset(uint8_t slave_id);

/* Consecutive transactions to `slave_id` that got NO REPLY (0 = healthy). This
 * is the controller's best liveness signal for a gateway: the queue talks to it
 * continuously, so it notices a dead link far sooner than any periodic probe. A
 * Modbus EXCEPTION does not count — that is the gateway alive and disagreeing. */
uint16_t mbq_client_consec_fail(uint8_t slave_id);

/* Dump per-slave client state to the console (dev CLI). slave_id < 0 = all. */
void mbq_client_debug_print(int slave_id);
