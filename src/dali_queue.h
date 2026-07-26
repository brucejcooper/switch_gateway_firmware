#pragma once
/*
 * dali_queue.h — controller-side client for the gateway's 8-deep id-tagged
 * DALI command queue (task 04).
 *
 * One driver per DALI gateway (slave). Producers (the raw command path, the
 * any-on probe, the config-query SM) submit commands into a per-slave local
 * backlog — the priority queue that sits in front of the gateway's queue. The
 * driver assigns each command a gateway id (1..31, cycling, 0 reserved), ships
 * a window of them via FC23 (write submit-window + read status block in one
 * transaction), and routes each command's result back to its callback as the
 * status block reports it collected. Because every command carries a unique id
 * and its result lands in a ring slot the controller reads back, the old
 * shared-result-register interlocks (probe_query_in_flight / cfgq settle-gate)
 * are gone — id-tagging makes result attribution unambiguous.
 *
 * Policy (see dali_queue.c): submit min(backlog, free_slots, 4) per transaction,
 * tracking credits (free_slots) from every response — strict credits at 19200 so
 * transactions never bounce on SLAVE_DEVICE_BUSY. Transport recovery: on an FC23
 * timeout the driver reads status before resubmitting, so a landed-but-unacked
 * submit is never blind-resubmitted (UP/DOWN steps are not idempotent). Gateway
 * reboot (boot_count change / status anomaly): recovery reads the gateway's live
 * status. A boot_count-confirmed reboot (or accepted_id==0) is AUTHORITATIVE queue
 * loss → FRESH-REBUILD: drop non-idempotent in-flight, re-pend idempotent. This is
 * load-bearing in a busy burst, where the controller's own post-reboot submits make
 * accepted_id!=0 (so the accepted==0 anomaly detector never fires) and the lost
 * pre-reboot commands would otherwise stay stuck at the front of the backlog —
 * gating fresh-vs-alive on accepted alone (a boot-confirmed reboot taking the alive
 * branch) is what desynced the result ring by one slot. Alive-reconcile (KEEP the
 * in-flight set) is reserved for UNCONFIRMED detections against a genuinely-live
 * gateway (accepted_id!=0, boot_count unchanged, e.g. a verify-exhaust transport
 * glitch). Then BOTH branches baseline next_id/last_completed from the gateway's
 * live status through the same logic a warm controller boot uses (dq_handle_adopt).
 * A resync also clears the stored boot_count so the sysinfo poll re-baselines rather
 * than reading the same reboot as a second one.
 *
 * KNOWN LIMITATIONS on a gateway reboot mid-burst (tracked: one unified kept-vs-lost
 * fix in tasks/06-dq-collect-hardening.md). Two residual result-ATTRIBUTION offsets
 * remain — both are controller-side only: the gateway executes every command
 * correctly and neither corrupts gateway or ballast state; they mis-map ANSWERS to
 * commands (a one-command answer lag) and both self-heal on a controller reboot.
 *
 *   (i) Fast-poll transient: within the ~2 s transport-recovery window normal
 *   collection can momentarily attribute a stale ring slot to a lost in-flight
 *   command. A boot-count poll landing INSIDE that window USED to make it permanent;
 *   the fresh-rebuild above removes that permanent case, so this is now transient and
 *   UNREACHABLE at the 30 s production `gw_diag_interval_ms` default (recovery drains
 *   first) — it only shows at aggressive poll intervals (≤2 s).
 *
 *   (ii) Transport-glitch submit_n rollback (PROD-REACHABLE, independent of poll
 *   rate): a verify-exhaust against a genuinely-live gateway takes the alive-reconcile
 *   branch; if the status read driving it predates the gateway's accept of the just-
 *   sent submit, accepted_id != submit_last_id, so dq_rollback_submit re-pends a
 *   command the gateway actually accepted → a persistent inflight-vs-gateway id offset.
 *   This is on the transport-recovery path, NOT gated by the gw-diag poll, so it is
 *   reachable at the production poll interval. It is rare (transport-glitch drills at
 *   the 30 s poll came back clean 6/6) and self-heals on controller reboot.
 *
 * The tracked follow-up is ONE unified fix (re-pend/drop only genuinely-lost commands,
 * verify-before-drop, keep-on-ambiguity) spanning the dq_resync_apply submit_n
 * resolution AND dq_collect (drop entries whose ids the gateway can no longer
 * complete), not two separate patches.
 */

#include <stdbool.h>
#include <stdint.h>

/* Command flags — mirror the gateway QF_* record flags. */
#define DQ_TWICE   0x01u   /* send the forward frame twice, no answer */
#define DQ_DTR0    0x02u   /* preload DTR0 (0xA3<dtr0>) back-to-back before the frame */
#define DQ_ANSWER  0x04u   /* expects a backward frame; deliver the 8-bit answer */

/* Priority classes, highest first — an ordered table so re-ordering the ladder
 * is a one-line change. Mirrors the gateway's own work selector (commissioning →
 * raw → group bit → writes → reads → scans). The per-slave scheduler always
 * submits the highest-priority pending command first; within a class it is FIFO.
 * Classes marked "reserved" have no queue client yet (that transport still lives
 * elsewhere — group masks via FC03, level reads via dirty_poll, provisioning via
 * the commission register) and are placeholders for task 05. */
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
    DQ_RES_ERR = 2,   /* bus/timing error; `answer` = error subtype */
    DQ_RES_DROPPED = 3, /* command dropped without executing (gateway reboot; non-idempotent) */
} dq_result_t;

/* Completion callback for one submitted command. Invoked exactly once (from
 * dq_tick context, i.e. the core-0 main loop) when the command's result is
 * collected — or with DQ_RES_DROPPED if a gateway reboot discards it. May be
 * NULL for fire-and-forget commands (e.g. button on/off/step). */
typedef void (*dq_result_cb_t)(uint8_t slave_id, dq_result_t result,
                               uint8_t answer, void *ctx);

/* Reset all per-slave driver state. Call once at boot. */
void dq_init(void);

/* Submit one DALI command to slave_id's gateway queue. The command joins the
 * local backlog (FIFO) and is shipped by a later dq_tick(). `flags` is a bitwise
 * OR of DQ_*; `dtr0` is used only with DQ_DTR0. `idempotent` = safe to resubmit
 * after a transport fault or gateway reboot (queries, GOTO_LAST_ACTIVE, OFF,
 * DAPC level, group on/off/level are idempotent; UP/DOWN step frames are NOT).
 * `cb`/`ctx` receive the result (cb may be NULL). Returns false (and does not
 * enqueue) if the local backlog is full. */
bool dq_submit(uint8_t slave_id, dq_prio_t prio, uint8_t addr, uint8_t data,
               uint8_t flags, uint8_t dtr0, bool idempotent,
               dq_result_cb_t cb, void *ctx);

/* Per-loop driver: ship FC23 transactions, collect results, run recovery. Call
 * every main-loop iteration. */
void dq_tick(void);

/* Feed the gateway boot_count observed by the sysinfo (0x0310) poll, for reboot
 * detection/resync. Safe to call every poll; a change from the last-seen value
 * triggers a resync of that slave's in-flight commands. */
void dq_note_boot_count(uint8_t slave_id, uint16_t boot_count);

/* True if slave_id has any command pending in the backlog or in flight at the
 * gateway. Producers that must serialise against the queue (e.g. a probe that
 * needs the bus quiet) can gate on this. */
bool dq_busy(uint8_t slave_id);

/* Blocking submit-and-wait, for the text CLI (dali_raw / read_mem). Submits one
 * command and spins the bus (modbus_poll + dq_tick) until its result is
 * collected or `timeout_ms` elapses. Returns true on a result (fills *result /
 * *answer), false on backlog-full or timeout. Do NOT call from the non-CLI
 * main-loop path (it blocks the loop like the other cli_mb_wait helpers).
 *
 * `idempotent` matters on a gateway REBOOT mid-command (not a plain transport
 * timeout — that path reads status and only resends a command that did NOT land,
 * safe regardless of the flag). On a reboot resync an idempotent command is
 * re-pended/resubmitted, a NON-idempotent one is DROPPED (DQ_RES_DROPPED, surfaced
 * to the caller). So an arbitrary `dali_raw` frame (a human can make it an UP/DOWN
 * step) passes false — a reboot mid-command drops-and-reports rather than
 * resubmitting and risking a double-apply ("stale-but-never-wrong"); queries /
 * SET DTR / DAPC pass true. */
bool dq_submit_sync(uint8_t slave_id, uint8_t addr, uint8_t data, uint8_t flags,
                    uint8_t dtr0, bool idempotent, uint32_t timeout_ms,
                    dq_result_t *result, uint8_t *answer);

/* Dump per-slave queue-client state to the console (CLI `dq [slave]`): scheduler
 * state, ids (next/accepted/completed), in-flight (id:class) list, per-class
 * queued depths, and the diagnostic counters (submitted/collected/timeouts/
 * resubmits/busy/adopts/resyncs) the bench fault drills read — resyncs breaks
 * out as boot=<boot_count-triggered>,anom=<status-anomaly/verify-triggered> so a
 * drill can tell the two detectors apart. slave_id < 0 dumps all configured DALI
 * slaves. Dev-grade; no MQTT surface. */
void dq_debug_print(int slave_id);
