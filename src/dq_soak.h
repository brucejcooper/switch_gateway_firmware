#pragma once
/*
 * dq_soak.h — command-queue soak/throughput probe (task 07).
 *
 * A bench instrument, not a product feature. It hammers one gateway's command
 * queue with an ALTERNATING WRITE/READ pair whose read answer is fully
 * DISCRIMINATING, so any dropped, reordered or mis-attributed command shows up
 * as a value mismatch rather than as a silent success:
 *
 *     SET DTR0 = v      (special command 0xA3<v>, one forward frame, no answer)
 *     QUERY CONTENT DTR0 (addr|1, 0x98, backward frame)  → MUST answer v
 *
 * DTR0 is a VOLATILE register in the control gear: unlike fade time / power-on
 * level (the other "something that changes" candidates) it costs no ballast
 * EEPROM write cycles, so a multi-hour soak cannot wear the gear out. `v` walks
 * a full 256-value cycle (stride 97, coprime with 256), so consecutive pairs
 * never share an expected answer and a one-command lag is distinguishable from
 * a random corruption — the probe reports "lag-1" separately from "other".
 *
 * It BYPASSES the dali_queue priority backlog and submits straight into
 * mbq_client, and it paces itself on the queue-client's view of GATEWAY-SIDE
 * occupancy: it stops submitting once the gateway's 8-slot queue is half full
 * (SOAK_DEPTH_DEFAULT = 4 active slots), so the queue is kept fed without ever
 * relying on a BUSY rejection as flow control.
 *
 * Everything it measures is reported by `soak` (see dq_soak_report): pair rate,
 * DALI frame rate, per-command latency, the Modbus transaction count that paid
 * for it, and the in-flight histogram that says whether the gateway queue was
 * actually kept fed.
 */

#include <stdbool.h>
#include <stdint.h>

#define SOAK_DEPTH_DEFAULT  4u   /* half of the gateway's 8 slots */

/* Which path the probe's commands take to the gateway. The default bypasses the
 * scheduler entirely (measuring the transport); routing through the scheduler
 * instead turns the probe into realistic load OF A GIVEN CLASS, which is how the
 * scheduler's priority-inversion bound gets measured: run the probe as
 * background and time a user command against it. */
typedef enum {
    SOAK_PATH_DIRECT = 0,   /* straight into mbq_client (transport measurement) */
    SOAK_PATH_BG,           /* via dq_submit at DQ_PRIO_BACKGROUND */
    SOAK_PATH_USER,         /* via dq_submit at DQ_PRIO_USER */
} soak_path_t;

/* Start a soak against `addr` on `slave`. `pairs` = number of set/query pairs to
 * run (0 = until `soak stop`). `depth` = max gateway-side active slots (1..8),
 * used only on the direct path — on a scheduler path the class's own depth
 * policy governs. Returns false if a soak is already running or the slave is not
 * a DALI slave. */
bool dq_soak_start(uint8_t slave, uint8_t addr, uint8_t depth, uint32_t pairs,
                   soak_path_t path);

/* Stop the running soak (results are kept for `soak` to report). */
void dq_soak_stop(void);

/* True while a soak is submitting or draining. */
bool dq_soak_active(void);

/* Per-loop driver: top the gateway queue back up to `depth`. Call every main-loop
 * iteration, after dq_tick(). No-op when idle. */
void dq_soak_tick(void);

/* Print the last (or running) soak's results to the console. */
void dq_soak_report(void);
