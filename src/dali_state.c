/*
 * dali_state.c — the controller's DALI world model (task 05). See dali_state.h.
 *
 * One sweep state machine per DALI slave. Each holds at most ONE query in flight
 * (like the config-query rotation it sits beside), so it paces itself against
 * the bus without ever needing to queue a burst: the work to do is held as
 * BITMAPS of what needs asking, not as a queue of commands. A ballast marked
 * twice before its query runs is asked once — which is the whole point of
 * keeping wants as state.
 *
 * Selection order per slave:
 *   1. verification marks (a command just touched these) at DQ_PRIO_LEVEL_READ
 *   2. scan marks (boot / post-provision discovery)       at DQ_PRIO_BACKGROUND
 *   3. the rotating idle sweep                            at DQ_PRIO_BACKGROUND
 */

#include "dali_state.h"
#include "dali_queue.h"
#include "mbq_client.h"
#include "state_cache.h"
#include "lighting_actions.h"
#include "dali_provision.h"
#include "config.h"
#include "cdc_out.h"
#include "log.h"
#include "pico/time.h"
#include <string.h>

/* DALI query opcodes (IEC 62386-102), short-address command form. */
#define Q_STATUS        0x90u   /* status byte: fade running, lamp fail, limit error, ... */
#define Q_ACTUAL_LEVEL  0xA0u   /* current arc level */
#define Q_GROUPS_0_7    0xC0u
#define Q_GROUPS_8_15   0xC1u

/* Background sweep pace: ONE ballast interrogated per this interval, per slave.
 *
 * This is deliberately slow. The sweep exists only to catch what nothing tells
 * us about — a ballast that lost power and came back, or gear changed by
 * something outside this controller. Everything the system itself does is
 * verified immediately by the post-command path below, so the sweep is a
 * backstop, not the mechanism.
 *
 * For scale: the gateway's own rescan cursor (which this replaces) ran at ONE
 * ballast per 64 s. An earlier version of this file used 250 ms, which is 256x
 * faster than the thing it replaced and kept the RS-485 bus ~59% busy doing
 * nothing useful. */

/* While a ballast reports FADE RUNNING, re-read it at this pace until it
 * settles. A DAPC lands immediately but the level ramps, so a single read taken
 * mid-fade caches an intermediate value and — without this — would stand until
 * the next sweep, minutes later. Bounded by FADE_TRACK_MAX_MS because a stuck
 * fade bit must not pin the bus forever; DALI's longest fade time is ~90 s. */
#define FADE_POLL_MS        250u
#define FADE_TRACK_MAX_MS   95000u

/* Every Nth background step probes an ABSENT address instead of re-reading a
 * present one. Without this the sweep only ever walks ballasts it already
 * believes in, so a ballast that was powered off at boot — or added afterwards —
 * would never be rediscovered short of a manual `scan` or a reboot. A probe of
 * an absent address costs exactly one query (STATUS gets a clean no-answer and
 * the pass is abandoned), and at one probe per 6 steps a full pass over 64
 * addresses takes ~64 min at the default interval: cheap, and far faster than
 * "never". */
#define ABSENT_PROBE_EVERY  6u

/* Consecutive no-reply transactions after which a gateway is treated as down,
 * and how often to re-attempt while it is. Matches the transport's own backoff
 * trip point so the two agree about what "dead" means. */
#define LINK_DOWN_FAILS   3u
#define LINK_RETRY_MS     5000u

/* QUERY STATUS bit 4 = fade running (IEC 62386-102). */
#define STATUS_FADE_RUNNING 0x10u

/* Per-ballast query sequence. */
typedef enum {
    STEP_STATUS = 0,
    STEP_LEVEL,
    STEP_GROUPS_LO,   /* scan only */
    STEP_GROUPS_HI,   /* scan only */
    STEP_DONE,
} step_t;

typedef struct {
    bool     in_use;
    uint8_t  slave_id;
    uint8_t  slave_idx;

    uint64_t verify;      /* ballasts needing a post-command read (high priority) */
    uint64_t scan;        /* ballasts needing a full discovery pass */
    uint64_t present;     /* our current belief */
    uint8_t  strikes[64]; /* consecutive clean no-answers, per address */

    /* the query currently in flight */
    bool     busy;
    uint8_t  addr;
    step_t   step;
    bool     is_scan;     /* this pass includes the group reads */
    bool     have;        /* a result landed */
    dq_result_t res;
    uint8_t  answer;
    uint16_t gen;         /* stale-callback guard */
    uint16_t exp_gen;
    absolute_time_t deadline;

    /* partial results for the ballast being read */
    uint8_t  got_status;
    bool     got_status_ok;
    uint8_t  got_level;      /* captured at STEP_LEVEL; the scan path commits later */
    bool     got_level_ok;
    uint16_t got_groups;

    uint8_t  cursor;        /* rotating idle sweep position (present ballasts) */
    uint8_t  absent_cursor; /* rotating probe position across ALL addresses */
    uint8_t  step_count;    /* sweep steps taken, for the absent-probe cadence */
    absolute_time_t next_step_at;
    absolute_time_t link_retry_at;  /* gateway unreachable: don't re-attempt until this */

    /* Fade tracking: re-verify a ballast while it reports FADE RUNNING. */
    uint64_t fade_watch;              /* ballasts being tracked through a fade */
    absolute_time_t fade_next_at;
    absolute_time_t fade_give_up_at;
} ds_slave_t;

static ds_slave_t s_ds[MAX_DALI_SLAVES];

#define QUERY_DEADLINE_MS 400u

/* ── helpers ─────────────────────────────────────────────────────────────── */

/* Background sweep pace, from config. 0 DISABLES the sweep (same convention as
 * gw_diag_interval_ms). Disabling it is safe: every command the system issues is
 * still verified immediately by the post-command path, so switching the sweep
 * off only means "never re-check a ballast that nothing told us about". */
static uint32_t sweep_interval_ms(void) {
    return g_config.dali_sweep_interval_ms;
}

static ds_slave_t *slave_for(uint8_t slave_id) {
    for (int i = 0; i < MAX_DALI_SLAVES; i++)
        if (s_ds[i].in_use && s_ds[i].slave_id == slave_id) return &s_ds[i];
    return NULL;
}

static uint8_t cached_arc(const ds_slave_t *d, uint8_t addr) {
    dali_ballast_state_t *b = state_cache_get_dali(d->slave_idx, addr);
    return b ? b->arc_level : 0u;
}

static uint16_t next_gen(ds_slave_t *d) {
    if (++d->gen == 0) d->gen = 1;   /* 0 = "no live query" */
    return d->gen;
}

void dali_state_init(void) {
    memset(s_ds, 0, sizeof s_ds);
    for (int i = 0; i < g_config.dali_slave_count && i < MAX_DALI_SLAVES; i++) {
        s_ds[i].in_use    = true;
        s_ds[i].slave_id  = g_config.dali_slave_ids[i];
        s_ds[i].slave_idx = (uint8_t)i;
        s_ds[i].next_step_at = get_absolute_time();
    }
}

void dali_state_mark_dirty(uint8_t slave_id, uint8_t addr) {
    ds_slave_t *d = slave_for(slave_id);
    if (!d || addr >= 64) return;
    d->verify |= (uint64_t)1 << addr;
}

void dali_state_mark_dirty_group(uint8_t slave_id, uint8_t group) {
    ds_slave_t *d = slave_for(slave_id);
    if (!d) return;
    /* Fan out through the cached membership. If the cache is empty or stale we
     * simply verify nothing extra — the idle sweep still catches it — rather
     * than guessing at members. */
    uint64_t members = state_cache_get_dali_group(d->slave_idx, group);
    d->verify |= members & d->present;
}

void dali_state_request_full_scan(uint8_t slave_id) {
    ds_slave_t *d = slave_for(slave_id);
    if (!d) return;
    d->scan = ~(uint64_t)0;   /* every address — presence is what we're discovering */
    LOG_INFO("dstate", "slave %u full scan armed", slave_id);
}

void dali_state_mark_scan(uint8_t slave_id, uint8_t addr) {
    ds_slave_t *d = slave_for(slave_id);
    if (!d || addr >= 64) return;
    d->scan |= (uint64_t)1 << addr;
}

bool dali_state_scan_busy(uint8_t slave_id) {
    ds_slave_t *d = slave_for(slave_id);
    return d && (d->scan != 0 || (d->busy && d->is_scan));
}

/* ── result handling ─────────────────────────────────────────────────────── */

static void ds_cb(uint8_t slave_id, dq_result_t res, uint8_t answer, void *ctx) {
    ds_slave_t *d = slave_for(slave_id);
    if (!d) return;
    uint16_t g = (uint16_t)(uintptr_t)ctx;
    if (g == 0 || g != d->exp_gen) return;   /* superseded / abandoned */
    d->res    = res;
    d->answer = answer;
    d->have   = true;
}

/* Apply the presence evidence rules to one query outcome. Returns true if the
 * ballast is (still) believed present. */
static bool presence_from(ds_slave_t *d, uint8_t addr, dq_result_t res,
                          bool authoritative) {
    uint64_t bit = (uint64_t)1 << addr;
    switch (res) {
        case DQ_RES_OK:
            d->strikes[addr] = 0;
            if (!(d->present & bit)) {
                d->present |= bit;
                state_cache_set_dali_present(d->slave_idx, addr, true);
                LOG_INFO("dstate", "%u.%u present", d->slave_id, addr);
            }
            return true;
        case DQ_RES_NAK:
            /* The only outcome that is evidence of absence: the bus was healthy
             * and nobody answered a directed query.
             *
             * An EXPLICIT SCAN is authoritative and skips the debounce. The
             * debounce exists so a single missed reply during routine background
             * polling cannot drop a healthy ballast — but a scan is a deliberate
             * re-establishment of ground truth (boot, `scan`, post-provisioning),
             * and there the caution actively hurts: after a re-provision every
             * vacated address would linger as a phantom in HA, one per background
             * sweep step, so a full renumber could take minutes to converge. */
            if (d->strikes[addr] < 0xFFu) d->strikes[addr]++;
            if (authoritative) d->strikes[addr] = DALI_ABSENT_STRIKES;
            if (d->strikes[addr] >= DALI_ABSENT_STRIKES && (d->present & bit)) {
                d->present &= ~bit;
                state_cache_set_dali_present(d->slave_idx, addr, false);
                LOG_INFO("dstate", "%u.%u absent (%u clean no-answers)",
                         d->slave_id, addr, d->strikes[addr]);
            }
            return (d->present & bit) != 0;
        default:
            /* Collision / bus fault / dropped: evidence about the BUS, not about
             * this ballast. Freeze the counters and keep the current belief —
             * stale-but-never-wrong beats fresh-but-maybe-wrong. */
            return (d->present & bit) != 0;
    }
}

/* Commit whatever we learned about `addr` and emit if it changed. */
static void commit_ballast(ds_slave_t *d, uint8_t addr, uint8_t arc, uint8_t status,
                           bool have_groups, uint16_t groups) {
    dali_ballast_state_t *b = state_cache_get_dali(d->slave_idx, addr);
    bool changed = !b || b->arc_level != arc || b->status_byte != status;

    state_cache_set_dali_arc(d->slave_idx, addr, arc, status);

    if (have_groups) {
        for (uint8_t g = 0; g < 16; g++) {
            uint64_t mask = state_cache_get_dali_group(d->slave_idx, g);
            uint64_t bit  = (uint64_t)1 << addr;
            bool member   = (groups >> g) & 1u;
            uint64_t nm   = member ? (mask | bit) : (mask & ~bit);
            if (nm != mask) state_cache_set_dali_group(d->slave_idx, g, nm);
        }
    }
    if (changed) lighting_emit_dali_state(d->slave_id, addr, arc, status);
}

/* ── the sweep ───────────────────────────────────────────────────────────── */

/* Pick the next address to interrogate. Verification marks win; then scan
 * marks; then the rotating idle sweep over believed-present ballasts. */
static bool pick_next(ds_slave_t *d, uint8_t *addr_out, bool *is_scan) {
    /* A ballast mid-fade is re-queued for verification as soon as its poll
     * interval elapses — ahead of the background sweep, behind fresh commands. */
    if (d->fade_watch) {
        if (time_reached(d->fade_give_up_at)) {
            LOG_WARN("dstate", "slave %u fade never settled — dropping watch", d->slave_id);
            d->fade_watch = 0;
        } else if (time_reached(d->fade_next_at)) {
            d->verify |= d->fade_watch;
        }
    }
    if (d->verify) {
        *addr_out = (uint8_t)__builtin_ctzll(d->verify);
        *is_scan  = false;
        return true;
    }
    if (d->scan) {
        *addr_out = (uint8_t)__builtin_ctzll(d->scan);
        *is_scan  = true;
        return true;
    }
    if (sweep_interval_ms() == 0u) return false;   /* sweep disabled by config */
    if (!time_reached(d->next_step_at)) return false;

    /* Periodically probe an address we believe is EMPTY, so gear that was off at
     * boot or added later is discovered without a manual scan. */
    if (++d->step_count >= ABSENT_PROBE_EVERY) {
        d->step_count = 0;
        for (uint8_t i = 0; i < 64; i++) {
            uint8_t a = (uint8_t)((d->absent_cursor + i) & 0x3Fu);
            if (!(d->present & ((uint64_t)1 << a))) {
                d->absent_cursor = (uint8_t)((a + 1u) & 0x3Fu);
                *addr_out = a;
                /* Scan-style pass: a newly-found ballast needs its groups read
                 * too, not just its level. */
                *is_scan  = true;
                return true;
            }
        }
        /* Every address occupied — nothing to probe for. */
    }

    for (uint8_t i = 0; i < 64; i++) {
        uint8_t a = (uint8_t)((d->cursor + i) & 0x3Fu);
        if (d->present & ((uint64_t)1 << a)) {
            d->cursor = (uint8_t)((a + 1u) & 0x3Fu);
            *addr_out = a;
            *is_scan  = false;
            return true;
        }
    }
    /* Nothing believed present: re-arm a discovery pass rather than idling
     * forever, so a bus that comes up after us is still found. */
    d->next_step_at = make_timeout_time_ms(5000u);
    d->scan = ~(uint64_t)0;
    return false;
}

static bool issue(ds_slave_t *d) {
    static const uint8_t OPCODE[] = {
        [STEP_STATUS]     = Q_STATUS,
        [STEP_LEVEL]      = Q_ACTUAL_LEVEL,
        [STEP_GROUPS_LO]  = Q_GROUPS_0_7,
        [STEP_GROUPS_HI]  = Q_GROUPS_8_15,
    };
    uint8_t addr_byte = (uint8_t)(((d->addr & 0x3Fu) << 1) | 0x01u);
    uint16_t g = next_gen(d);
    /* Verification reads outrank the sweep: a command's real outcome must reach
     * HA promptly, while a background refresh can wait behind anything. This now
     * holds — DQ_PRIO_LEVEL_READ has its own scheduler class. Until task 07 it
     * fell through into the background class alongside DQ_PRIO_BACKGROUND, so
     * this ternary picked between two names for the same queue and the comment
     * promised an ordering the code did not deliver. */
    dq_prio_t prio = d->is_scan ? DQ_PRIO_BACKGROUND : DQ_PRIO_LEVEL_READ;
    if (!dq_submit(d->slave_id, prio, addr_byte, OPCODE[d->step],
                   DQ_ANSWER, 0, true, ds_cb, (void *)(uintptr_t)g))
        return false;   /* class full — retry next tick */
    d->exp_gen  = g;
    d->have     = false;
    d->busy     = true;
    d->deadline = make_timeout_time_ms(QUERY_DEADLINE_MS);
    return true;
}

static void advance(ds_slave_t *d) {
    uint64_t bit = (uint64_t)1 << d->addr;

    /* Record this step's answer. */
    bool ok = (d->res == DQ_RES_OK);
    switch (d->step) {
        case STEP_STATUS:
            d->got_status_ok = ok;
            d->got_status    = ok ? d->answer : 0u;
            break;
        case STEP_LEVEL:
            /* A query that did not answer cleanly must not publish a fabricated
             * 0 — keep whatever the cache already holds. */
            d->got_level_ok = ok;
            d->got_level    = ok ? d->answer : cached_arc(d, d->addr);
            break;
        case STEP_GROUPS_LO:
            if (ok) d->got_groups = (uint16_t)((d->got_groups & 0xFF00u) | d->answer);
            break;
        case STEP_GROUPS_HI:
            if (ok) d->got_groups = (uint16_t)((d->got_groups & 0x00FFu) | ((uint16_t)d->answer << 8));
            break;
        default: break;
    }

    /* Presence is judged on the FIRST directed query of the pass (QUERY STATUS):
     * one clean no-answer there is the absence evidence; the remaining steps are
     * pointless if nobody is home. */
    if (d->step == STEP_STATUS) {
        bool present = presence_from(d, d->addr, d->res, d->is_scan);
        if (!present) {
            /* Abandon the rest of this ballast's pass. */
            d->verify &= ~bit;
            d->scan   &= ~bit;
            d->busy    = false;
            d->step    = STEP_STATUS;
            return;
        }
    }

    if (d->step == STEP_LEVEL) {
        if (!d->is_scan) {
            /* Verification pass ends here: status + level are what a command
             * changes. Group membership is only re-read on a full scan. */
            commit_ballast(d, d->addr, d->got_level, d->got_status, false, 0);
            d->verify &= ~bit;
            /* Still ramping? Keep re-reading at FADE_POLL_MS until it settles,
             * and PUBLISH EACH SAMPLE. Every one is a real measured level, so HA
             * gets a live ramp rather than a jump at the end — that is wanted.
             *
             * What must never be published is a level we only ASSUMED: the
             * requested value before the ballast has clamped it to its own
             * min/max. See the "bri" branch of dali_apply_kv, which deliberately
             * reports nothing and lets these reads speak. */
            if (d->got_status & STATUS_FADE_RUNNING) {
                if (!(d->fade_watch & bit)) {
                    d->fade_watch |= bit;
                    d->fade_give_up_at = make_timeout_time_ms(FADE_TRACK_MAX_MS);
                }
                d->fade_next_at = make_timeout_time_ms(FADE_POLL_MS);
            } else {
                d->fade_watch &= ~bit;
            }
            d->busy    = false;
            d->step    = STEP_STATUS;
            return;
        }
        d->got_groups = 0;   /* about to read them */
    }

    if (d->step == STEP_GROUPS_HI) {
        commit_ballast(d, d->addr, d->got_level, d->got_status, true, d->got_groups);
        d->scan  &= ~bit;
        d->busy   = false;
        d->step   = STEP_STATUS;
        return;
    }

    d->step = (step_t)(d->step + 1);
    d->busy = false;   /* issue the next step on the following tick */
}

void dali_state_tick(void) {
    for (int i = 0; i < MAX_DALI_SLAVES; i++) {
        ds_slave_t *d = &s_ds[i];
        if (!d->in_use) continue;

        /* A slave mid-provisioning owns its bus. */
        if (dali_provision_busy_slave(d->slave_id)) continue;

        /* Gateway not answering: stop generating work for it. The transport
         * keeps a dead slave off the wire (modbus.c absent-slave backoff), but
         * that alone is not enough — a skipped request completes instantly, and
         * every producer's pacing assumes a transaction costs ~25 ms of wall
         * time, so the sweep would churn through all 64 addresses at main-loop
         * speed and re-arm forever. Measured: 10589 skipped requests in 68 s
         * against two dead slaves. Off the bus, but pointless work.
         *
         * One attempt per LINK_RETRY_MS is enough to notice it come back — the
         * queue client's own reconnect and the 5 s scan re-arm do the rest. */
        if (mbq_client_consec_fail(d->slave_id) >= LINK_DOWN_FAILS) {
            if (!time_reached(d->link_retry_at)) continue;
            d->link_retry_at = make_timeout_time_ms(LINK_RETRY_MS);
        }

        if (d->busy) {
            if (d->have) {
                d->have = false;
                advance(d);
            } else if (time_reached(d->deadline)) {
                /* Abandon: bump the token so a late answer is ignored, and treat
                 * it as bus evidence (not absence evidence) — the counters stay
                 * frozen, the belief stands. */
                d->exp_gen = 0;
                d->res     = DQ_RES_ERR;
                d->answer  = 0;
                advance(d);
            }
            continue;
        }

        uint8_t addr; bool is_scan;
        if (d->step == STEP_STATUS) {
            if (!pick_next(d, &addr, &is_scan)) continue;
            d->addr    = addr;
            d->is_scan = is_scan;
            d->next_step_at = make_timeout_time_ms(sweep_interval_ms());
        }
        issue(d);   /* false = class full; retried next tick */
    }
}

/* ── observability ───────────────────────────────────────────────────────── */

void dali_state_debug_print(int slave_id) {
    bool any = false;
    for (int i = 0; i < MAX_DALI_SLAVES; i++) {
        ds_slave_t *d = &s_ds[i];
        if (!d->in_use) continue;
        if (slave_id >= 0 && d->slave_id != (uint8_t)slave_id) continue;
        any = true;
        int np = 0;
        for (uint8_t a = 0; a < 64; a++) if (d->present & ((uint64_t)1 << a)) np++;
        cdc_out_printf("dstate slave %u: present=%d busy=%u addr=%u step=%u scan=%s\n",
                       d->slave_id, np, d->busy, d->addr, (unsigned)d->step,
                       d->scan ? "armed" : "idle");
        cdc_out_printf("  present mask: %08lx%08lx\n",
                       (unsigned long)(d->present >> 32), (unsigned long)(d->present & 0xFFFFFFFFu));
        cdc_out_printf("  verify mask:  %08lx%08lx  cursor=%u\n",
                       (unsigned long)(d->verify >> 32), (unsigned long)(d->verify & 0xFFFFFFFFu),
                       d->cursor);
    }
    if (!any) cdc_out_printf("dstate: no matching DALI slave\n");
}
