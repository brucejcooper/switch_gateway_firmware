/*
 * dali_queue.c — DALI adapter over the generic Modbus command-queue client
 * See dali_queue.h for the model.
 *
 * This layer is DALI-SPECIFIC and sits ABOVE the DALI-agnostic mbq_client:
 *   1. a per-slave PULL SCHEDULER: storage partitioned by
 *      priority class rather than one shared FIFO, plus per-ballast dim-step
 *      COUNTERS that hold no commands at all. dq_tick walks the priority ladder
 *      and ships at most one command per tick per slave, applying that class's
 *      own gateway depth — so background work can neither displace a user
 *      command's storage nor sit more than 2 slots deep in front of one;
 *   2. a PAYLOAD ADAPTER: pack a DALI command into the client's 4 opaque command
 *      bytes ([addr, data, flags, dtr0]) and map the client's terminal outcome +
 *      response8 back to a dq_result_t for the producer's callback.
 *
 * There is NO id ledger and NO reboot resync/rebuild/rollback here — the client
 * handles gateway-reboot recovery through the queue-protocol §4 rules,
 * resubmitting idempotent commands and reporting the rest DQ_RES_DROPPED.
 *
 * FAIRNESS is this layer's job, not the protocol's (spec §5). Slaves with work
 * pending are served ROUND-ROBIN from a rotating cursor, so with per-slave
 * single-outstanding transactions any slave's wait is bounded by one
 * transaction time per competing slave. This is what replaced spinning as the
 * anti-starvation mechanism when the client started pacing its polls.
 */

#include "dali_queue.h"
#include "mbq_client.h"
#include "modbus.h"
#include "config.h"
#include "dali_provision.h"
#include "log.h"
#include "pico/time.h"
#include <string.h>
#include "cdc_out.h"
#include "core1.h"   /* wd_feed */

/* ── Scheduler shape ────────────────────────────────────────────────────────
 * There is no single FIFO backlog any more. Two reasons, both measured:
 *
 * 1. A shared pool lets background work occupy the storage a button press needs
 *    — the old 16-deep backlog reported backlog_full=2 in normal operation, and
 *    every one of those is a silently dropped command. Storage is now
 *    PARTITIONED BY CLASS, so a burst of background work can neither displace a
 *    user command nor sit in front of one. The user class is sized to one
 *    button's full fan-out (MAX_TARGETS_PER_BTN), so a press always fits.
 *
 * 2. Latency comes from commands already at the GATEWAY, which executes strictly
 *    FIFO with no priority — so a queued background command delays a button by a
 *    whole DALI frame. Hence a per-class DEPTH: background may occupy only ONE
 *    of the gateway's command slots, bounding that inversion to one frame time,
 *    while user work may fill the queue. The task-07 soak measured the trade at
 *    the older 8-slot depth (depth 2 = 82 % of peak throughput, depth 4 = 96 %,
 *    each extra slot ~25 ms of inversion); at the v4 depth of 2 the ladder's
 *    only remaining lever is keeping background out of the second slot.
 *    Background work is not latency-sensitive; buttons are.
 *
 * Dim steps are not stored as commands at all — see the delta counters below. */
#define DQ_CLASS_MAX     8u   /* per-class ring capacity (max over the classes) */

/* Each class's GATEWAY DEPTH is the SCHEDULER's own ordering policy, not the
 * protocol's flow control — that is A_FULL (spec §5, decided 2026-08-12): the
 * client has no in-flight cap of its own, and submits until the gateway
 * refuses. What depth actually bounds is s->outstanding: commands this
 * scheduler has SHIPPED to the client and not yet had resolved, i.e. how much
 * of a class's commitment is allowed to sit ahead of the priority ladder
 * (see DQ_PIPELINE_MAX below). Note v2 needed every class strictly BELOW the
 * depth, because a submit carried its own status read-back and so needed a
 * slot it already knew was free. A v4 enqueue is its own frame and needs no
 * free slot in advance, so a class may fill the queue completely — that
 * retired constraint is what buys the one-transaction-per-DALI-frame steady
 * state. */
/* How much of the PIPELINE the scheduler keeps filled — client staging plus
 * whatever the gateway holds, which is what s->outstanding counts.
 *
 * CMDQ_DEPTH at the gateway plus ONE staged is the whole of it. Going deeper
 * does not buy throughput (the gateway is the bottleneck, and it says A_FULL)
 * and it costs latency: work staged in the client has left the priority
 * ladder behind, so anything queued past the gateway's depth is a DALI frame
 * a button press may have to wait out. */
#define DQ_PIPELINE_MAX  (CMDQ_DEPTH + 1u)

/* Pipeline share per class. The ladder's job is priority, not flow control.
 * What matters is that BACKGROUND may never take the staging slot: capped at
 * CMDQ_DEPTH it can fill the gateway but never the slot in front of it, so a
 * button press arriving now is staged next and waits only for the DALI frame
 * already executing. That bound is the whole point of the ladder. */
#define DQ_DEPTH_USER    DQ_PIPELINE_MAX          /* user work may fill the queue */
#define DQ_DEPTH_CFG     DQ_PIPELINE_MAX
#define DQ_DEPTH_READ    DQ_PIPELINE_MAX          /* verification reads: an outcome to HA */
/* ONE, and the file's own design note above says why: the gateway executes
 * strictly FIFO with no priority, so every background command already sitting
 * there delays a button press by a whole DALI frame. At depth 2 background can
 * hold BOTH gateway slots and a press waits ~76 ms behind them; at 1 the
 * inversion is bounded to the single frame already executing.
 *
 * This was CMDQ_DEPTH, which contradicted that note — the comment said "only
 * ONE of the gateway's command slots" while the constant allowed both.
 * Background throughput drops to ~0.6x (one idle DALI frame per collect round
 * trip), which is irrelevant to a 10 s-cadence backstop sweep and costs a few
 * seconds on the boot scan. Buttons are latency-sensitive; sweeps are not. */
#define DQ_DEPTH_BG      1u

/* ...EXCEPT while a scan is armed for that slave (dq_set_bg_burst, task 10).
 *
 * Depth 1 also serialises background work to one command per round trip: the
 * gateway's DALI engine finishes a frame and then sits idle while the result is
 * collected and the next command enqueued. That idle is most of what made a
 * full scan hold 66 % of the RS-485 bus for 3.4 s — the producer windows above
 * cannot ship their second query while a background command is outstanding.
 *
 * A scan is armed at boot, by `scan`, and after provisioning: seconds at a
 * time, not the steady state. For that window background may hold BOTH gateway
 * slots, so a button press can again wait ~76 ms behind two frames (dd83279
 * measured the depth-1 bound as a 241->189 ms median improvement under
 * SATURATING load). Bruce's call, 2026-08-15: a few seconds of scan is worth
 * it; the steady-state sweep stays at 1, where dd83279's bound is what
 * matters. */
#define DQ_DEPTH_BG_BURST 2u
_Static_assert(DQ_DEPTH_USER <= DQ_PIPELINE_MAX && DQ_DEPTH_CFG <= DQ_PIPELINE_MAX &&
               DQ_DEPTH_READ <= DQ_PIPELINE_MAX && DQ_DEPTH_BG < DQ_PIPELINE_MAX &&
               DQ_DEPTH_BG_BURST < DQ_PIPELINE_MAX,
               "no class may exceed the pipeline, and background must not take the staging slot");

/* Pending dim steps, per ballast and per group, instead of queued UP/DOWN
 * frames (user decision, 2026-07-28). A held button emits dim repeats faster
 * than DALI can issue them, so queueing each one builds a backlog that keeps
 * dimming after release — the exact latency problem this task removes. A signed
 * counter collapses that: the scheduler issues ONE step per submit opportunity
 * and decrements ON SUBMIT.
 *
 * Decrementing on submit (not on completion) is deliberate: a step lost to a
 * gateway reboot simply doesn't happen, which is invisible in a dim ramp, and
 * it can never be replayed into a double-step. That is also why these frames go
 * out non-idempotent. The clamp drops excess by design — the ballast's own fade
 * rate and min/max clamping still govern the result, which is why this issues
 * real UP/DOWN rather than computing a level from cache. */
#define DQ_DELTA_MAX     4

/* DALI response8 failure codes on a FAILED slot — the payload contract mirrored
 * from the gateway (TinyModbus dali_sync.h). A COMPLETE slot's response8 is the
 * backward-frame answer byte instead. */
/* 0x01 was DALI_RESP_NAK, retired 2026-08-13 — "a query got silence" is the
 * status MBQ_R_NAK now, not a failure payload. Reserved, not reused. */
#define DALI_RESP_COLLISION  0x02u  /* garbled backward frame (≥1 gear answered) */
#define DALI_RESP_BUSFAULT   0x03u  /* transport-level fault (short pulse etc.) */

typedef struct {
    uint8_t  addr, data, flags, dtr0;
    uint8_t  prio;           /* dq_prio_t — kept for the debug dump */
    bool     idempotent;
    dq_result_cb_t cb;
    void    *ctx;
} dq_cmd_t;

/* One priority class's storage. FIFO within a class; classes are consulted in
 * order, so ordering between classes is priority, not arrival. */
typedef struct {
    dq_cmd_t e[DQ_CLASS_MAX];
    uint8_t  n;
    uint8_t  cap;      /* entries this class may hold */
    uint8_t  depth;    /* gateway slots this class may fill to */
} dq_class_t;

/* Class index. dq_prio_t maps onto these — the ladder is unchanged, only its
 * storage is partitioned. */
typedef enum { DQ_CL_PROV = 0, DQ_CL_USER, DQ_CL_CFG, DQ_CL_READ, DQ_CL_BG,
               DQ_CL_COUNT } dq_class_id_t;

/* DQ_CL_READ is its own class so verification reads do not queue behind the
 * rotating refresh: they carry a command's real outcome to HA.
 *
 * Ranked BELOW config writes: those are user-initiated changes, and a follow-up
 * read should not displace one. GROUP_WRITE still shares DQ_CL_USER, and that is
 * deliberate rather than an oversight — a group command IS a user action and
 * wants exactly the user class's storage and depth. */
static dq_class_id_t class_of(dq_prio_t p) {
    switch (p) {
        case DQ_PRIO_PROVISION:     return DQ_CL_PROV;
        case DQ_PRIO_USER:
        case DQ_PRIO_GROUP_WRITE:   return DQ_CL_USER;
        case DQ_PRIO_CONFIG_WRITE:  return DQ_CL_CFG;
        case DQ_PRIO_LEVEL_READ:    return DQ_CL_READ;
        default:                    return DQ_CL_BG;
    }
}

/* One trampoline per possible in-flight command: carries the producer's
 * cb/ctx across the client, and maps the client's outcome back on completion.
 * The client stores &pend[i] as its per-slot ctx, so the record must be stable
 * for the command's lifetime (it lives in the slave struct). */
typedef struct {
    bool     in_use;
    dq_result_cb_t cb;
    void    *ctx;
} dq_pending_t;

typedef struct {
    bool     in_use;
    uint8_t  slave_id;

    dq_class_t cl[DQ_CL_COUNT];
    uint8_t  outstanding;    /* commands submitted to the client, awaiting completion */

    /* Pending dim steps — state, not queued commands (see DQ_DELTA_MAX). */
    int8_t   delta[64];      /* per short address */
    int8_t   gdelta[16];     /* per group */
    uint8_t  delta_cursor;   /* rotating start point, so one ballast being dimmed
                              * cannot starve another's steps */

    /* One record per command handed to the client and not yet resolved — so
     * it must cover the client's whole capacity, queue and in-flight alike. */
    dq_pending_t pend[MBQ_CMD_SLOTS];

    /* Diagnostic counters (for the `dq` CLI command / bench fault drills). */
    uint32_t c_submitted;    /* commands shipped to the client */
    uint32_t c_ok;           /* delivered DQ_RES_OK */
    uint32_t c_nak;          /* delivered DQ_RES_NAK */
    uint32_t c_err;          /* delivered DQ_RES_ERR (collision / bus fault) */
    uint32_t c_dropped;      /* delivered DQ_RES_DROPPED (gateway reboot, non-idempotent) */
    uint32_t c_full;         /* submits refused — that class's storage was full */
    uint32_t c_coalesced;    /* duplicate idempotent submits folded into a pending one */
    uint32_t c_steps;        /* dim steps issued from the delta counters */
    uint32_t c_steps_clamp;  /* dim steps dropped by the delta clamp */
} dq_slave_t;

static dq_slave_t s_dq[MAX_DALI_SLAVES];

/* ── helpers ─────────────────────────────────────────────────────────────── */

static dq_slave_t *slave_for(uint8_t slave_id) {
    for (int i = 0; i < MAX_DALI_SLAVES; i++)
        if (s_dq[i].in_use && s_dq[i].slave_id == slave_id) return &s_dq[i];
    return NULL;
}

static dq_pending_t *pend_alloc(dq_slave_t *s) {
    for (uint8_t i = 0; i < MBQ_CMD_SLOTS; i++)
        if (!s->pend[i].in_use) return &s->pend[i];
    return NULL;
}

/* ── public API ──────────────────────────────────────────────────────────── */

/* See dali_queue.h for why a collision is YES and a bus fault is not. */
dq_yes_no_t dq_yes_no(dq_result_t res, uint8_t answer) {
    switch (res) {
        case DQ_RES_OK:  return DQ_YN_YES;              /* a clean backward frame */
        case DQ_RES_NAK: return DQ_YN_NO;               /* silence IS the answer  */
        case DQ_RES_ERR:
            /* Collision means two or more gear answered simultaneously, so at
             * least one said yes. A bus fault means nothing was learned. */
            return (answer == DALI_RESP_COLLISION) ? DQ_YN_YES : DQ_YN_UNRESOLVED;
        default:         return DQ_YN_UNRESOLVED;       /* dropped / lost         */
    }
}

void dq_init(void) {
    memset(s_dq, 0, sizeof s_dq);
    mbq_client_init();
    for (int i = 0; i < g_config.dali_slave_count && i < MAX_DALI_SLAVES; i++) {
        s_dq[i].in_use   = true;
        s_dq[i].slave_id = g_config.dali_slave_ids[i];
        /* Per-class capacity and gateway depth. USER is sized to one button's
         * full fan-out so a press is never refused by a background burst. */
        /* Provisioning is strictly sequential — each bisect step depends on the
         * previous answer — so depth 1. It gets its own class because while a
         * slave is being commissioned the scheduler must ship THESE frames and
         * nothing else; they are the commissioning sequence. */
        s_dq[i].cl[DQ_CL_PROV] = (dq_class_t){ .cap = 2u,           .depth = 1u            };
        s_dq[i].cl[DQ_CL_USER] = (dq_class_t){ .cap = DQ_CLASS_MAX, .depth = DQ_DEPTH_USER };
        s_dq[i].cl[DQ_CL_CFG]  = (dq_class_t){ .cap = 3u,           .depth = DQ_DEPTH_CFG  };
        /* Verification reads arrive per touched ballast, so a group command's
         * fan-out can mark several at once — hence a wider ring than the sweep,
         * which only ever has one rotation entry outstanding. */
        s_dq[i].cl[DQ_CL_READ] = (dq_class_t){ .cap = 4u,           .depth = DQ_DEPTH_READ };
        /* Two background producers each submit a rolling window of two (the
         * dali_state sweep's STATUS+LEVEL, the config rotation's next two
         * fields), so a ring of 2 let whichever arrived first lock the other
         * out for a whole round trip. This is STORAGE, not gateway depth: the
         * inversion a button press can suffer is bounded by .depth below, which
         * is unchanged. */
        s_dq[i].cl[DQ_CL_BG]   = (dq_class_t){ .cap = 4u,           .depth = DQ_DEPTH_BG   };
        mbq_client_add_slave(s_dq[i].slave_id);
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
    /* Someone pressed something. If this gateway was written off as absent,
     * try it now rather than when the backoff next allows (up to 60 s): a
     * gateway that has just come back must answer the first press, not the
     * one a minute later. */
    if (prio == DQ_PRIO_USER) {
        modbus_backoff_kick(slave_id);
        mbq_client_kick(slave_id);
    }
    dq_class_t *c = &s->cl[class_of(prio)];
    dq_cmd_t entry = { .addr = addr, .data = data,
                       .flags = (uint8_t)(flags & MBQ_QF_MASK), .dtr0 = dtr0,
                       .prio = (uint8_t)prio, .idempotent = idempotent,
                       .cb = cb, .ctx = ctx };

    /* Coalesce an exact duplicate that has not shipped yet. Only for idempotent
     * commands with no callback: repeating one has no additional effect, so a
     * repeated press or a re-published HA state costs nothing. A command with a
     * callback is never folded — its caller is waiting for its own result. */
    if (idempotent && !cb) {
        for (uint8_t i = 0; i < c->n; i++) {
            if (c->e[i].addr == entry.addr && c->e[i].data == entry.data
                && c->e[i].flags == entry.flags && c->e[i].dtr0 == entry.dtr0
                && !c->e[i].cb) {
                s->c_coalesced++;
                return true;
            }
        }
    }

    if (c->n >= c->cap) {
        /* A refusal means different things to different producers: to a button it
         * is a dropped command (and that caller logs it), to a polling producer
         * (config-query rotation, bench probes) it is ordinary backpressure —
         * "try again next tick". So this is a rate-limited counter, not a warning
         * per event: a polling producer against a 2-deep class would otherwise
         * emit hundreds of thousands of them. The counter is the signal. */
        s->c_full++;
        if ((s->c_full & 0xFFu) == 1u)
            LOG_WARN("dq", "slave %u class %u full — cmd %02X%02X refused (total %lu)",
                     slave_id, (unsigned)class_of(prio), addr, data,
                     (unsigned long)s->c_full);
        return false;
    }
    c->e[c->n++] = entry;   /* FIFO within the class */
    return true;
}

void dq_set_bg_burst(uint8_t slave_id, bool on) {
    dq_slave_t *s = slave_for(slave_id);
    if (!s) return;
    s->cl[DQ_CL_BG].depth = on ? DQ_DEPTH_BG_BURST : DQ_DEPTH_BG;
}

bool dq_add_delta(uint8_t slave_id, bool is_group, uint8_t addr, int8_t steps) {
    dq_slave_t *s = slave_for(slave_id);
    if (!s) return false;
    int8_t *d = is_group ? &s->gdelta[addr & 0x0Fu] : &s->delta[addr & 0x3Fu];
    int v = *d + steps;
    if (v >  DQ_DELTA_MAX) { v =  DQ_DELTA_MAX; s->c_steps_clamp++; }
    if (v < -DQ_DELTA_MAX) { v = -DQ_DELTA_MAX; s->c_steps_clamp++; }
    *d = (int8_t)v;
    return true;
}

static bool any_delta(const dq_slave_t *s) {
    for (uint8_t i = 0; i < 64; i++) if (s->delta[i]) return true;
    for (uint8_t i = 0; i < 16; i++) if (s->gdelta[i]) return true;
    return false;
}

static uint8_t pending_total(const dq_slave_t *s) {
    uint8_t n = 0;
    for (uint8_t i = 0; i < DQ_CL_COUNT; i++) n = (uint8_t)(n + s->cl[i].n);
    return n;
}

/* ── driver internals ────────────────────────────────────────────────────── */

/* Client completion trampoline: map the generic outcome + response8 to a
 * dq_result_t and hand it to the producer's callback, then release the
 * trampoline and the in-flight slot. Runs from core-0 main-loop context,
 * invoked exactly once per submitted command — from mbq_client_collect() (via
 * dq_tick, below) on the normal path, or occasionally from inside drive()'s
 * recovery-give-up when a stalled §4 poll times out (mbq_client.h has the
 * full contract). */
static void dq_mbq_cb(uint8_t slave_id, mbq_result_t r, uint8_t response8, void *ctx) {
    dq_pending_t *p = (dq_pending_t *)ctx;
    dq_result_cb_t cb  = p->cb;
    void          *cctx = p->ctx;
    p->in_use = false;

    dq_slave_t *s = slave_for(slave_id);
    if (s && s->outstanding > 0) s->outstanding--;

    dq_result_t res = DQ_RES_DROPPED;
    uint8_t     answer = 0;
    switch (r) {
        case MBQ_R_COMPLETE:
            res = DQ_RES_OK; answer = response8;
            if (s) s->c_ok++;
            break;
        case MBQ_R_NAK:
            /* Expected a reply, heard silence. Passed straight through: what it
             * MEANS is the producer's call (no / absent), so this layer must not
             * decide. Previously this arrived as MBQ_R_FAILED carrying a
             * DALI_RESP_NAK payload and was decoded back here by convention. */
            res = DQ_RES_NAK; if (s) s->c_nak++;
            break;
        case MBQ_R_FAILED:
            res = DQ_RES_ERR; answer = response8; if (s) s->c_err++;
            break;
        case MBQ_R_LOST:
        case MBQ_R_RESULT_LOST:
        default:
            /* Both reach the producer as DROPPED — a DALI command's caller can
             * do nothing different with "never ran" versus "ran, answer gone",
             * since either way it has no measured value to publish and the
             * verification read that follows every command is what recovers the
             * truth. The distinction is kept in the client's counters, where it
             * says whether a reboot ate work or ate answers. */
            res = DQ_RES_DROPPED;
            if (s) s->c_dropped++;
            break;
    }
    if (cb) cb(slave_id, res, answer, cctx);
}

/* Hand one command to the client. Returns false if it did not go out (no
 * trampoline, or the Modbus action queue was full) — the caller leaves its work
 * where it was and retries next tick. */
static bool ship(dq_slave_t *s, const dq_cmd_t *e) {
    dq_pending_t *p = pend_alloc(s);
    if (!p) return false;   /* all trampolines busy (should not happen under the cap) */

    uint8_t cmd[MBQ_CMD_BYTES] = { e->addr, e->data, e->flags, e->dtr0 };
    p->in_use = true;
    p->cb     = e->cb;
    p->ctx    = e->ctx;
    if (!mbq_client_submit(s->slave_id, cmd, e->idempotent, dq_mbq_cb, p)) {
        p->in_use = false;
        return false;
    }
    s->outstanding++;
    s->c_submitted++;
    return true;
}

/* Ship the front of class `ci` if its depth policy allows it. */
static bool ship_class(dq_slave_t *s, dq_class_id_t ci) {
    dq_class_t *c = &s->cl[ci];
    if (c->n == 0 || s->outstanding >= c->depth) return false;
    if (!ship(s, &c->e[0])) return false;
    for (uint8_t i = 0; i + 1u < c->n; i++) c->e[i] = c->e[i + 1u];
    c->n--;
    return true;
}

/* Issue one pending dim step, scanning from a rotating cursor so a ballast being
 * held down cannot starve another's steps. The counter is decremented HERE, on
 * submit — a step that is subsequently lost just doesn't happen, and can never
 * be replayed as a double-step (hence idempotent=false). */
static bool ship_delta(dq_slave_t *s) {
    if (s->outstanding >= DQ_DEPTH_USER) return false;

    for (uint8_t i = 0; i < 64u + 16u; i++) {
        uint8_t k = (uint8_t)((s->delta_cursor + i) % (64u + 16u));
        bool is_group = (k >= 64u);
        int8_t *d = is_group ? &s->gdelta[k - 64u] : &s->delta[k];
        if (*d == 0) continue;

        bool up = (*d > 0);
        uint8_t addr_byte = is_group
            ? (uint8_t)(0x81u | (((k - 64u) & 0x0Fu) << 1))          /* group, command form */
            : (uint8_t)(((k & 0x3Fu) << 1) | 0x01u);                 /* short address, command form */
        dq_cmd_t e = { .addr = addr_byte, .data = up ? 0x01u : 0x02u,  /* UP / DOWN */
                       .flags = 0, .dtr0 = 0, .prio = (uint8_t)DQ_PRIO_USER,
                       .idempotent = false, .cb = NULL, .ctx = NULL };
        if (!ship(s, &e)) return false;

        *d = (int8_t)(up ? *d - 1 : *d + 1);
        s->delta_cursor = (uint8_t)((k + 1u) % (64u + 16u));
        s->c_steps++;
        return true;
    }
    return false;
}

/* Walk the priority ladder for one slave and ship at most one command.
 * Returns true if something went out. */
static bool ship_ladder(dq_slave_t *s) {
    /* Provisioning is bus-exclusive: while a slave is being commissioned the
     * scheduler ships the commissioning frames AND NOTHING ELSE, so nothing
     * can interleave with the binary search (a stray DTR0 write mid-bisect
     * would corrupt it). Note this must still ship the PROV class — the
     * sequence now runs on the controller and travels through this very
     * queue, so gating it out entirely deadlocks the run. */
    if (dali_provision_busy_slave(s->slave_id)) return ship_class(s, DQ_CL_PROV);

    /* Each step applies its own class depth, so background work stops filling
     * the gateway one slot before user work does and a button press waits at
     * most the DALI frame already in flight. */
    if (ship_class(s, DQ_CL_PROV)) return true;   /* commissioning steps */
    if (ship_class(s, DQ_CL_USER)) return true;   /* discrete presses / HA commands */
    if (ship_delta(s))             return true;   /* dim ramps, one step at a time */
    if (ship_class(s, DQ_CL_CFG))  return true;   /* config writes */
    if (ship_class(s, DQ_CL_READ)) return true;   /* verification reads */
    return ship_class(s, DQ_CL_BG);               /* sweeps, rotation queries */
}

/* Rotating start point for the per-slave sweep below. Fairness across slaves is
 * the scheduler's job (spec §5): with per-slave single-outstanding transactions
 * and one shared RS-485 bus, always starting the sweep at slave index 0 lets
 * the lowest-numbered gateway take every submit opportunity it can use before
 * the next one is even looked at. Rotating the start bounds any slave's wait to
 * one transaction time per competing slave. */
static uint8_t s_rr_cursor;

void dq_tick(void) {
    /* Drive the underlying client first: process completions (freeing slots and
     * trampolines) and pace polls, so a slot freed this tick is available to the
     * scheduler below. */
    mbq_client_collect();

    for (int n = 0; n < MAX_DALI_SLAVES; n++) {
        int i = (s_rr_cursor + n) % MAX_DALI_SLAVES;
        dq_slave_t *s = &s_dq[i];
        if (!s->in_use) continue;

        /* can_submit means ROOM IN THE CLIENT'S SLOT TABLE, nothing more — it
         * is true with a transaction already on the wire (see the note on
         * mbq_client_can_submit). What bounds this to one command per tick per
         * slave is ship_ladder(), which returns after a single ship; what
         * bounds how much may be in flight at once is the per-class depth each
         * ship_class() applies against s->outstanding. */
        if (!mbq_client_can_submit(s->slave_id)) continue;

        /* One command shipped means this slave used its turn: hand the next
         * sweep's first look to the slave after it. */
        if (ship_ladder(s)) s_rr_cursor = (uint8_t)((i + 1) % MAX_DALI_SLAVES);
    }
    /* No drive() here any more. The transport asks each slave for work when the
     * bus falls idle (modbus_register_offer), so pushing from here would both
     * bypass the rotation and race the offer. Staging above is unchanged — this
     * tick still decides WHICH command is next for each slave; the bus decides
     * WHEN it goes out. */
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
                   dq_sync_cb, NULL)) {
        if (result) *result = DQ_RES_BACKLOG;   /* rejected up front */
        return false;
    }
    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);
    while (!s_sync_have && absolute_time_diff_us(get_absolute_time(), deadline) > 0) {
        modbus_poll();
        dq_tick();
        /* Long CLI-driven chains (dali_read_mem, cmd_blink) call this up to
         * ~193 times; feed here or the 5 s watchdog fires mid-command
         * (core-1-gated, so a real core-1 wedge still reboots). */
        wd_feed();
    }
    if (!s_sync_have) { if (result) *result = DQ_RES_TIMEOUT; return false; }
    if (result) *result = s_sync_res;
    if (answer) *answer = s_sync_ans;
    return true;
}

/* ── `dq` CLI observability ───────────────────────────────────────────────── */

void dq_debug_print(int slave_id) {
    bool any = false;
    for (int i = 0; i < MAX_DALI_SLAVES; i++) {
        dq_slave_t *s = &s_dq[i];
        if (!s->in_use) continue;
        if (slave_id >= 0 && s->slave_id != (uint8_t)slave_id) continue;
        any = true;
        int dsum = 0, dn = 0;
        for (uint8_t j = 0; j < 64; j++) if (s->delta[j])  { dsum += s->delta[j]; dn++; }
        for (uint8_t j = 0; j < 16; j++) if (s->gdelta[j]) { dsum += s->gdelta[j]; dn++; }
        cdc_out_printf("dq slave %u: inflight=%u\n", s->slave_id, s->outstanding);
        /* The background depth is the live one: it is raised while a scan is
         * armed, and "why is a press slow right now" is exactly what this dump
         * is read for. */
        cdc_out_printf("  pending/class: prov=%u/%u user=%u/%u cfg=%u/%u read=%u/%u bg=%u/%u"
                       "  (depth %u/%u/%u/%u%s)\n",
                       s->cl[DQ_CL_PROV].n, s->cl[DQ_CL_PROV].cap,
                       s->cl[DQ_CL_USER].n, s->cl[DQ_CL_USER].cap,
                       s->cl[DQ_CL_CFG].n,  s->cl[DQ_CL_CFG].cap,
                       s->cl[DQ_CL_READ].n, s->cl[DQ_CL_READ].cap,
                       s->cl[DQ_CL_BG].n,   s->cl[DQ_CL_BG].cap,
                       DQ_DEPTH_USER, DQ_DEPTH_CFG, DQ_DEPTH_READ,
                       s->cl[DQ_CL_BG].depth,
                       s->cl[DQ_CL_BG].depth == DQ_DEPTH_BG_BURST ? " burst" : "");
        cdc_out_printf("  dim delta: %d target(s), net %d steps, cursor=%u\n",
                       dn, dsum, s->delta_cursor);
        cdc_out_printf("  counters: submitted=%lu ok=%lu nak=%lu err=%lu dropped=%lu "
                       "refused=%lu coalesced=%lu steps=%lu clamped=%lu\n",
                       (unsigned long)s->c_submitted, (unsigned long)s->c_ok,
                       (unsigned long)s->c_nak, (unsigned long)s->c_err,
                       (unsigned long)s->c_dropped, (unsigned long)s->c_full,
                       (unsigned long)s->c_coalesced, (unsigned long)s->c_steps,
                       (unsigned long)s->c_steps_clamp);
        mbq_client_debug_print((int)s->slave_id);
    }
    if (!any) cdc_out_printf("dq: no matching DALI slave\n");
}
