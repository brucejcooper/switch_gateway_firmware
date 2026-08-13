#pragma once
/*
 * dali_queue.h — DALI adapter over the generic Modbus command-queue client
 *
 *
 * This is the DALI-SPECIFIC layer. It owns the per-slave PRIORITY SCHEDULER and
 * the producer API; the transaction layer beneath it is mbq_client.c (opaque
 * 32-bit command in, terminal outcome + 8-bit response out — DALI-agnostic; see
 * mbq_client.h and, normatively, ../TinyModbus/QUEUE-PROTOCOL.md). Producers
 * (the raw command path, the any-on probe, the config-query SM) submit into a
 * per-slave local backlog sorted by priority class. dq_tick() drains the
 * highest-priority pending command into the client's QUEUE (via mbq_client),
 * filling at most DQ_QUEUE_MAX of it — and background at most DQ_QUEUE_MAX-1 —
 * so a fresh high-priority command never waits behind a backlog of background
 * work. Note this is queue occupancy, not gateway depth: the gateway bounds
 * itself with A_FULL. Each command's terminal
 * outcome is mapped back to a dq_result_t and delivered to its callback exactly
 * once.
 *
 * The adapter packs a DALI command into the 4 opaque command bytes the client
 * carries: byte0 addr, byte1 data, byte2 flags (DQ_* below), byte3 dtr0. On
 * completion the client's response8 is the DALI backward-frame answer (OK) or a
 * DALI failure code (NAK / collision / bus fault), which the adapter maps to
 * DQ_RES_*.
 *
 * There is NO id ledger and NO reboot resync/rebuild/rollback machinery here.
 * Reboot recovery belongs entirely to the client's §4 rules, which either
 * resubmit an idempotent command or report it DQ_RES_DROPPED. (boot_count / the
 * FC04 sysinfo block remain a pure HA diagnostic in dirty_poll.c; the client
 * reads them for itself when a recovery rule needs an arbiter.)
 */

#include <stdbool.h>
#include <stdint.h>
#include "mbq_client.h"   /* MBQ_QF_* — the wire flag bits, aliased below */

/* Command flags. These ARE the wire bits (queue-protocol §1 packs the control
 * flags into the top three bits of the request's first byte), so a command
 * built here needs no translation on the way out — which is the point: the v2
 * values were 0x01/0x02/0x04 in a payload byte, and a stack that translated
 * between two encodings had a place for them to silently disagree, changing
 * the meaning of every command without failing. Single definition in
 * mbq_client.h; these are aliases so DALI callers keep reading as DALI. */
#define DQ_TWICE   MBQ_QF_TWICE    /* send the forward frame twice, no answer */
#define DQ_DTR0    MBQ_QF_DTR0     /* preload DTR0 (0xA3<dtr0>) before the frame */
#define DQ_ANSWER  MBQ_QF_ANSWER   /* expects a backward frame; deliver the answer */

/* Priority classes, highest first — an ordered table so re-ordering the ladder
 * is a one-line change. Mirrors the gateway's own work selector (commissioning →
 * raw → group bit → writes → reads → scans). The per-slave scheduler always
 * submits the highest-priority pending command first; within a class it is FIFO.
 * Classes marked "reserved" have no queue client yet (that transport still lives
 * elsewhere — group masks via FC03, level reads via dirty_poll, provisioning via
 */
typedef enum {
    DQ_PRIO_PROVISION = 0,  /* reserved: provisioning is bus-exclusive via the busy-gate */
    DQ_PRIO_USER,           /* brightness/on-off/step + group commands + the any-on probe */
    DQ_PRIO_GROUP_WRITE,    /* reserved (group-membership writes go via FC03 today) */
    DQ_PRIO_CONFIG_WRITE,   /* DTR0+SET config-write pairs */
    DQ_PRIO_LEVEL_READ,     /* reserved (level/status reads go via dirty_poll today) */
    DQ_PRIO_BACKGROUND,     /* config-rotation queries (cfgq) and other background sweeps */
    DQ_PRIO_COUNT
} dq_prio_t;

/* Result codes delivered to a command's completion callback. */
typedef enum {
    DQ_RES_OK  = 0,   /* backward frame received; `answer` is the response byte */
    DQ_RES_NAK = 1,   /* no backward frame (normal for fire-and-forget sends) */
    DQ_RES_ERR = 2,   /* bus/timing error or answer collision; `answer` = failure code */
    DQ_RES_DROPPED = 3, /* command lost (gateway reboot; non-idempotent → not resubmitted) */
    /* Synchronous-submit failure reasons (dq_submit_sync only; never delivered to
     * a callback). Set in *result on the false return so the CLI can distinguish
     * them instead of printing a bare "timeout" for both. */
    DQ_RES_BACKLOG = 4, /* submit rejected up front — controller backlog full */
    DQ_RES_TIMEOUT = 5, /* submitted, but no result within the sync deadline (bus congestion) */
} dq_result_t;

/* Completion callback for one submitted command. Invoked exactly once (from
 * dq_tick context, i.e. the core-0 main loop) when the command's result is
 * collected — or with DQ_RES_DROPPED if a gateway reboot discards it. May be
 * NULL for fire-and-forget commands (e.g. button on/off/step). */
typedef void (*dq_result_cb_t)(uint8_t slave_id, dq_result_t result,
                               uint8_t answer, void *ctx);

/* Reset all per-slave adapter state and (re)register each configured DALI slave
 * with the underlying command-queue client. Call once at boot. */
void dq_init(void);

/* Submit one DALI command to slave_id's gateway queue. The command joins the
 * local backlog (priority-sorted, FIFO within a class) and is shipped by a later
 * dq_tick(). `flags` is a bitwise OR of DQ_*; `dtr0` is used only with DQ_DTR0.
 * `idempotent` = safe to resubmit after a gateway reboot (queries,
 * GOTO_LAST_ACTIVE, OFF, DAPC level, group on/off/level are idempotent; UP/DOWN
 * step frames are NOT). `cb`/`ctx` receive the result (cb may be NULL). Returns
 * false (and does not enqueue) if the local backlog is full. */
bool dq_submit(uint8_t slave_id, dq_prio_t prio, uint8_t addr, uint8_t data,
               uint8_t flags, uint8_t dtr0, bool idempotent,
               dq_result_cb_t cb, void *ctx);

/* Accumulate `steps` dim steps (signed; + = UP, − = DOWN) for a ballast short
 * address (`is_group` false) or a group number (`is_group` true).
 *
 * Dim steps do NOT go through dq_submit. A step is an event — it cannot be
 * regenerated from cached state — but queueing one command per step is what
 * makes a held button keep dimming after release, because the repeats arrive
 * faster than DALI can issue them. So they accumulate into a per-target signed
 * counter which the scheduler drains one frame per submit opportunity, in
 * rotating order across targets. Real UP/DOWN frames go on the wire (the
 * ballast's own fade rate and min/max clamping still apply — this is
 * deliberately not a DAPC computed from cache), the counter is decremented on
 * SUBMIT, and it saturates at ±DQ_DELTA_MAX. Losing a step at the clamp or to a
 * gateway reboot is by design: one missing step in a ramp is invisible, whereas
 * a replayed one would double-step.
 *
 * Returns false only for an unknown slave. */
bool dq_add_delta(uint8_t slave_id, bool is_group, uint8_t addr, int8_t steps);

/* Per-loop driver: drive the underlying client (mbq_client_tick), then ship any
 * pending backlog commands into free gateway slots. Call every main-loop
 * iteration. */
void dq_tick(void);

/* Blocking submit-and-wait, for the text CLI (dali_raw / read_mem). Submits one
 * command and spins the bus (modbus_poll + dq_tick) until its result is
 * collected or `timeout_ms` elapses. Returns true on a result (fills *result /
 * *answer), false on backlog-full or timeout. Do NOT call from the non-CLI
 * main-loop path (it blocks the loop like the other cli_mb_wait helpers).
 *
 * `idempotent` matters on a gateway REBOOT mid-command: an idempotent command is
 * resubmitted, a NON-idempotent one is DROPPED (DQ_RES_DROPPED, surfaced to the
 * caller). So an arbitrary `dali_raw` frame (a human can make it an UP/DOWN step)
 * passes false — a reboot mid-command drops-and-reports rather than resubmitting
 * and risking a double-apply; queries / SET DTR / DAPC pass true. */
bool dq_submit_sync(uint8_t slave_id, uint8_t addr, uint8_t data, uint8_t flags,
                    uint8_t dtr0, bool idempotent, uint32_t timeout_ms,
                    dq_result_t *result, uint8_t *answer);

/* Dump per-slave adapter state to the console (CLI `dq [slave]`): per-class
 * backlog depths, in-flight count, and the diagnostic counters, then the
 * underlying client's slot state. slave_id < 0 dumps all configured DALI slaves.
 * Dev-grade; no MQTT surface. */
void dq_debug_print(int slave_id);
