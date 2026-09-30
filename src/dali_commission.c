/*
 * dali_commission.c — controller-side DALI commissioning. See dali_commission.h.
 *
 * One frame in flight at a time, submitted at DQ_PRIO_PROVISION. The queue's
 * scheduler holds everything else off for the slave while a run is active
 * (dq_tick gates on dali_provision_busy_slave), so the bisect cannot be
 * corrupted by an interleaved command — the same exclusivity the gateway got
 * from owning the sequence itself.
 */

#include "dali_commission.h"
#include "dali_queue.h"
#include "dali_state.h"
#include "state_cache.h"
#include "mapping.h"
#include "log.h"
#include "pico/time.h"
#include <string.h>

/* Special-command opcodes: these occupy the ADDRESS byte of the forward frame
 * (they replace per-ballast addressing entirely). IEC 62386-102. */
#define A_TERMINATE       0xA1u
#define A_INITIALISE      0xA5u   /* twice */
#define A_RANDOMISE       0xA7u   /* twice */
#define A_COMPARE         0xA9u   /* backward frame if any random <= search */
#define A_WITHDRAW        0xABu
#define A_SEARCHADDRH     0xB1u
#define A_SEARCHADDRM     0xB3u
#define A_SEARCHADDRL     0xB5u
#define A_PROGRAM_SHORT   0xB7u
#define A_VERIFY_SHORT    0xB9u

#define INIT_DATA_ALL      0x00u
#define INIT_DATA_NO_SHORT 0xFFu
#define PROG_SHORT(s)      ((uint8_t)(((s) << 1) | 0x01u))
#define Q_GEAR_PRESENT     0x91u   /* QUERY CONTROL GEAR PRESENT (short-address cmd) */
#define C_SET_SHORT_ADDR   0x80u   /* SET SHORT ADDRESS (DTR0), send twice */

#define STEP_DEADLINE_MS   1500u   /* one queued frame; generous vs ~40 ms actual */
#define MAX_VERIFY_FAILS   3u

typedef enum {
    P_IDLE = 0,
    P_INITIALISE,
    P_RANDOMISE,
    P_SEARCH_H,
    P_SEARCH_M,
    P_SEARCH_L,
    P_COMPARE,
    P_PROGRAM,
    P_VERIFY,
    P_WITHDRAW,
    P_CHECK_FREE,     /* ask a candidate short address if anyone is on it */
    P_RESTORE,        /* MODE_ONE: move the last gear back onto the target */
    P_RESTORE_VERIFY,
    P_TERMINATE,
} phase_t;

static const char *PHASE_NAME[] = {
    "idle", "initialise", "randomise", "search", "search", "search",
    "compare", "assign", "verify", "withdraw", "check-free", "restore",
    "restore-verify", "terminate",
};

static struct {
    bool         active;
    uint8_t      slave_id;
    dcom_mode_t  mode;
    uint8_t      target;          /* MODE_ONE: the collided short address */
    phase_t      phase;
    dcom_state_t state;

    uint32_t     search;          /* 24-bit search address */
    uint8_t      bisect_bit;      /* 24 = the initial "anyone there?" probe */
    bool         resend_pending;  /* re-send SEARCHADDR before PROGRAM */

    uint8_t      next_addr;       /* short address being assigned */
    /* Addresses this run must not hand out (non-ALL modes): the MODE_ONE target
     * while its gear is being moved off it, every address assigned so far this
     * run, and every candidate that answered on the wire. */
    uint64_t     claimed;
    phase_t      after_check;     /* where P_CHECK_FREE continues when free */
    bool         restoring;       /* P_CHECK_FREE is checking the target */
    uint8_t      last_addr;       /* address the most recent gear was given */
    uint8_t      count;           /* successfully addressed this run */
    uint8_t      verify_fails;
    bool         abort_req;

    /* in-flight frame */
    bool         busy;
    volatile bool have;
    dq_result_t  res;
    uint8_t      answer;
    uint16_t     gen, exp_gen;
    absolute_time_t deadline;
} s;

/* ── helpers ─────────────────────────────────────────────────────────────── */

/* True if any button mapping targets this ballast address on our slave. A new
 * gear must not land on such an address: the mapping would start driving it
 * (house, 2026-09-30: mappings to 3.17/24/32/40 point at nothing today). */
static bool addr_is_mapped(uint8_t a) {
    for (int f = 0; f < MAX_FIXTURES; f++)
        for (int b = 0; b < MAX_BUTTONS; b++) {
            const mapping_record_t *r = &g_mappings[f][b];
            for (int i = 0; i < r->target_count; i++) {
                const mapping_target_t *t = &r->targets[i];
                if (t->slave == s.slave_id && !TARGET_IS_GROUP(*t) &&
                    TARGET_BALLAST_ADDR(*t) == a)
                    return true;
            }
        }
    return false;
}

/* Lowest address that is a CANDIDATE for incremental / one-address modes. Not
 * proof it is free: the presence cache only knows gear that answered a scan
 * cleanly, so two gear sharing an address (a collision) or gear the scan could
 * not read look "absent". P_CHECK_FREE asks the address on the wire before it
 * is used. MODE_ALL never calls this — it renumbers every gear from 0, so every
 * address is available. */
static uint8_t lowest_unused_addr(void) {
    int idx = state_cache_slave_idx(s.slave_id);
    if (idx < 0) return 0;
    for (uint8_t a = 0; a < 64; a++) {
        if (s.claimed & ((uint64_t)1 << a)) continue;
        dali_ballast_state_t *b = state_cache_get_dali((uint8_t)idx, a);
        if (b && b->present) continue;
        if (addr_is_mapped(a)) continue;
        return a;
    }
    return 64;   /* pool exhausted */
}

static void finish(dcom_state_t st, const char *why) {
    s.state  = st;
    s.active = false;
    s.busy   = false;
    s.phase  = P_IDLE;
    LOG_INFO("dcom", "slave %u run ended: %s (%u addressed)",
             s.slave_id, why, s.count);
    /* Whatever happened, re-interrogate the bus: addresses may have moved. */
    dali_state_request_full_scan(s.slave_id);
}

static void frame_cb(uint8_t slave_id, dq_result_t res, uint8_t answer, void *ctx) {
    (void)slave_id;
    uint16_t g = (uint16_t)(uintptr_t)ctx;
    if (g == 0 || g != s.exp_gen) return;   /* superseded */
    s.res    = res;
    s.answer = answer;
    s.have   = true;
}

/* Submit the frame for the current phase. Returns false if the queue refused it
 * (retried next tick). */
static bool issue(void) {
    uint8_t addr, data = 0x00u, flags = 0u, dtr0 = 0u;

    switch (s.phase) {
        case P_INITIALISE:
            addr  = A_INITIALISE;
            data  = (s.mode == DCOM_MODE_ALL) ? INIT_DATA_ALL
                  : (s.mode == DCOM_MODE_ONE) ? PROG_SHORT(s.target)
                                              : INIT_DATA_NO_SHORT;
            flags = DQ_TWICE;
            break;
        case P_RANDOMISE:  addr = A_RANDOMISE; flags = DQ_TWICE; break;
        case P_SEARCH_H:   addr = A_SEARCHADDRH; data = (uint8_t)(s.search >> 16); break;
        case P_SEARCH_M:   addr = A_SEARCHADDRM; data = (uint8_t)(s.search >> 8);  break;
        case P_SEARCH_L:   addr = A_SEARCHADDRL; data = (uint8_t)(s.search);       break;
        case P_COMPARE:    addr = A_COMPARE;  flags = DQ_ANSWER; break;
        case P_PROGRAM:    addr = A_PROGRAM_SHORT; data = PROG_SHORT(s.next_addr); break;
        case P_VERIFY:     addr = A_VERIFY_SHORT;  data = PROG_SHORT(s.next_addr);
                           flags = DQ_ANSWER; break;
        case P_WITHDRAW:   addr = A_WITHDRAW;  break;
        case P_CHECK_FREE: addr  = PROG_SHORT(s.restoring ? s.target : s.next_addr);
                           data  = Q_GEAR_PRESENT; flags = DQ_ANSWER; break;
        case P_RESTORE:    addr  = PROG_SHORT(s.last_addr); data = C_SET_SHORT_ADDR;
                           flags = DQ_DTR0 | DQ_TWICE; dtr0 = PROG_SHORT(s.target); break;
        case P_RESTORE_VERIFY:
                           addr  = PROG_SHORT(s.target);
                           data  = Q_GEAR_PRESENT; flags = DQ_ANSWER; break;
        case P_TERMINATE:  addr = A_TERMINATE; break;
        default: return false;
    }

    if (++s.gen == 0) s.gen = 1;
    /* NOT idempotent: replaying a PROGRAM or WITHDRAW mid-sequence would corrupt
     * the run, so a gateway reboot must drop it and fail the run rather than
     * silently re-apply a step. */
    if (!dq_submit(s.slave_id, DQ_PRIO_PROVISION, addr, data, flags, dtr0,
                   false, frame_cb, (void *)(uintptr_t)s.gen))
        return false;
    s.exp_gen  = s.gen;
    s.have     = false;
    s.busy     = true;
    s.deadline = make_timeout_time_ms(STEP_DEADLINE_MS);
    return true;
}

/* ── phase advance ───────────────────────────────────────────────────────── */

static void advance(void) {
    /* An abort takes effect at any boundary, but always through TERMINATE so the
     * bus is never left in commissioning mode. */
    if (s.abort_req && s.phase != P_TERMINATE) {
        s.abort_req = false;
        s.state     = DCOM_ERR_ABORTED;
        s.phase     = P_TERMINATE;
        return;
    }

    switch (s.phase) {
        case P_INITIALISE: s.phase = P_RANDOMISE; break;
        case P_RANDOMISE:  s.phase = P_SEARCH_H;  break;
        case P_SEARCH_H:   s.phase = P_SEARCH_M;  break;
        case P_SEARCH_M:   s.phase = P_SEARCH_L;  break;

        case P_SEARCH_L:
            /* Normally SEARCH_L is followed by COMPARE. The exception is the
             * bit-0 "no" case below, which re-sent the search purely to correct
             * the gear-side registers — go straight to PROGRAM rather than
             * waste a COMPARE round. */
            if (s.resend_pending) { s.resend_pending = false; s.phase = P_PROGRAM; }
            else                  { s.phase = P_COMPARE; }
            break;

        case P_COMPARE: {
            /* COMPARE is anyone-pulls-low: ANY decoded response means at least
             * one gear's random address is <= the search value. Only a clean
             * no-answer means none — a collided/garbled frame is a match, which
             * is why this goes through dq_yes_no() rather than testing the
             * result code directly.
             *
             * A DROPPED/lost command must NOT be guessed — nothing was
             * learned. Abort the run; DCOM_ERR_TIMEOUT already means exactly
             * "a step produced no result". This used to read `s.res !=
             * DQ_RES_NAK`, folding a dropped command into "yes".
             *
             * A garbled answer (DQ_RES_ERR) IS an answer, whatever failure
             * code the gateway put on it. Several gear answering COMPARE at
             * once is the normal case — it is the whole mechanism — and the
             * overlapping backward frames reach us as COLLISION or BUSFAULT
             * depending on which pulse the gateway's decoder tripped on first
             * (house, 2026-09-30: a four-member group query came back
             * BUSFAULT). Treating BUSFAULT as unresolved made every COMPARE
             * over two or more gear abort the run, so a collided address could
             * never be split. Reading it as YES is safe here: PROGRAM SHORT
             * ADDRESS only touches gear whose random address equals the search
             * value, so a wrong YES can only steer the bisect to a value no
             * gear holds — PROGRAM then addresses nothing and the strict
             * VERIFY below fails and re-bisects. */
            dq_yes_no_t yn = (s.res == DQ_RES_ERR) ? DQ_YN_YES
                                                  : dq_yes_no(s.res, s.answer);
            if (yn == DQ_YN_UNRESOLVED) {
                s.state = DCOM_ERR_TIMEOUT;
                s.phase = P_TERMINATE;
                break;
            }
            bool yes = (yn == DQ_YN_YES);
            uint8_t bb = s.bisect_bit;

            if (!yes) {
                if (bb == 24u) {
                    /* Nobody left in the comparison set: either the bus is empty
                     * or we have addressed everything. */
                    if (s.count == 0u) {
                        s.state = DCOM_ERR_NO_BALLASTS;
                        s.phase = P_TERMINATE;
                    } else if (s.mode == DCOM_MODE_ONE) {
                        /* Every gear that shared the target has been moved off
                         * it, so the target is free again. Give it back to the
                         * last one moved, so existing mappings to the target
                         * keep driving one real device. Confirm it is empty on
                         * the wire first. */
                        s.restoring = true;
                        s.phase     = P_CHECK_FREE;
                    } else {
                        s.phase = P_TERMINATE;
                    }
                    break;
                }
                s.search |= ((uint32_t)1 << bb);   /* restore the cleared bit */
            } else if (bb == 24u && s.next_addr >= 64u) {
                s.state = DCOM_ERR_ADDR_EXHAUST;
                s.phase = P_TERMINATE;
                break;
            }

            if (bb == 0u) {
                if (!yes) {
                    /* The gear still holds the bit-0-cleared search value while
                     * our copy has bit 0 restored. PROGRAM would address nothing
                     * and VERIFY would fail — re-send the corrected SEARCHADDR
                     * first. */
                    s.resend_pending = true;
                    s.phase = P_SEARCH_H;
                } else {
                    s.phase = P_PROGRAM;
                }
            } else {
                s.bisect_bit = (uint8_t)(bb - 1u);
                s.search &= ~((uint32_t)1 << (bb - 1u));
                s.phase = P_SEARCH_H;
            }
            break;
        }

        case P_PROGRAM: s.phase = P_VERIFY; break;

        case P_VERIFY: {
            /* Strict: only the just-programmed gear may answer, with 0xFF.
             * Anything else — no answer, wrong value, or a collision (which
             * would mean two gears at one address) — is a verify failure. */
            bool ok = (s.res == DQ_RES_OK && s.answer == 0xFFu);
            if (ok) {
                s.count++;
                s.verify_fails = 0;
                LOG_INFO("dcom", "slave %u addressed %u (%u total)",
                         s.slave_id, s.next_addr, s.count);
                /* Advance BEFORE withdrawing, or incremental mode re-picks the
                 * slot we just filled. The presence cache does not learn about
                 * this address until the post-run scan, so claim it here. */
                s.claimed  |= (uint64_t)1 << s.next_addr;
                s.last_addr = s.next_addr;
                if (s.mode == DCOM_MODE_ALL) {
                    s.next_addr = (uint8_t)(s.next_addr + 1u);
                    s.phase     = P_WITHDRAW;
                } else {
                    s.next_addr = lowest_unused_addr();
                    /* >= 64 is caught at the next COMPARE (ADDR_EXHAUST). */
                    s.after_check = P_WITHDRAW;
                    s.phase = (s.next_addr < 64u) ? P_CHECK_FREE : P_WITHDRAW;
                }
            } else {
                if (++s.verify_fails > MAX_VERIFY_FAILS) {
                    s.state = DCOM_ERR_VERIFY;
                    s.phase = P_TERMINATE;
                } else {
                    /* Re-bisect from scratch for this gear. */
                    s.search     = 0xFFFFFFu;
                    s.bisect_bit = 24u;
                    s.phase      = P_SEARCH_H;
                }
            }
            break;
        }

        case P_CHECK_FREE: {
            /* QUERY CONTROL GEAR PRESENT to one short address. Silence = free.
             * A clean answer OR a garbled one (two gear answering) = occupied —
             * the collided addresses this exists for answer garbled. */
            bool nak = (s.res == DQ_RES_NAK);
            bool occ = (s.res == DQ_RES_OK || s.res == DQ_RES_ERR);
            if (!nak && !occ) {                    /* dropped / lost */
                s.state = DCOM_ERR_TIMEOUT;
                s.phase = P_TERMINATE;
                break;
            }
            if (s.restoring) {
                if (nak) {
                    s.phase = P_RESTORE;
                } else {
                    LOG_WARN("dcom", "slave %u: target %u still answers after the move "
                             "— leaving gear at %u", s.slave_id, s.target, s.last_addr);
                    s.phase = P_TERMINATE;
                }
                break;
            }
            if (nak) { s.phase = s.after_check; break; }
            LOG_INFO("dcom", "slave %u: address %u answers — not free, skipping",
                     s.slave_id, s.next_addr);
            s.claimed  |= (uint64_t)1 << s.next_addr;
            s.next_addr = lowest_unused_addr();
            if (s.next_addr >= 64u) {
                s.state = DCOM_ERR_ADDR_EXHAUST;
                s.phase = P_TERMINATE;
            }
            /* else: stay in P_CHECK_FREE and ask the next candidate */
            break;
        }

        case P_RESTORE: s.phase = P_RESTORE_VERIFY; break;

        case P_RESTORE_VERIFY:
            if (s.res == DQ_RES_OK)
                LOG_INFO("dcom", "slave %u: gear at %u moved back to %u",
                         s.slave_id, s.last_addr, s.target);
            else
                LOG_WARN("dcom", "slave %u: move %u -> %u not confirmed (res %u)",
                         s.slave_id, s.last_addr, s.target, (unsigned)s.res);
            s.phase = P_TERMINATE;
            break;

        case P_WITHDRAW:
            /* Gear withdrawn from the comparison set; start the next bisect. */
            s.search     = 0xFFFFFFu;
            s.bisect_bit = 24u;
            s.phase      = P_SEARCH_H;
            break;

        case P_TERMINATE:
            if (s.state == DCOM_RUNNING) s.state = DCOM_COMPLETE;
            finish(s.state,
                   s.state == DCOM_COMPLETE          ? "complete"
                 : s.state == DCOM_ERR_ABORTED       ? "aborted"
                 : s.state == DCOM_ERR_NO_BALLASTS   ? "no ballasts found"
                 : s.state == DCOM_ERR_VERIFY        ? "verify failures"
                 : s.state == DCOM_ERR_ADDR_EXHAUST  ? "address pool exhausted"
                 : s.state == DCOM_ERR_TIMEOUT       ? "step timed out"
                                                     : "error");
            return;

        default:
            s.phase = P_TERMINATE;
            break;
    }
}

/* ── public API ──────────────────────────────────────────────────────────── */

bool dcom_start(uint8_t slave_id, dcom_mode_t mode, uint8_t target) {
    if (s.active) return false;
    memset(&s, 0, sizeof s);
    s.active   = true;
    s.slave_id = slave_id;
    s.mode     = mode;
    s.target   = target;
    s.state    = DCOM_RUNNING;
    s.phase    = P_INITIALISE;
    s.search   = 0xFFFFFFu;
    s.bisect_bit = 24u;
    /* MODE_ALL renumbers every gear from 0, so every address is available. The
     * others fill the lowest free slot so existing gear keeps its address — and
     * check each candidate on the wire first (P_CHECK_FREE), because the
     * presence cache cannot see collided or unread gear. MODE_ONE's target is
     * claimed while its gear moves off; it is handed back to the last gear
     * moved once the run confirms it empty (P_RESTORE). */
    if (mode == DCOM_MODE_ONE) s.claimed = (uint64_t)1 << (target & 63u);
    s.next_addr = (mode == DCOM_MODE_ALL) ? 0u : lowest_unused_addr();
    if (s.next_addr >= 64u) {
        s.state = DCOM_ERR_ADDR_EXHAUST;
        s.active = false;
        LOG_WARN("dcom", "slave %u: no free short address", slave_id);
        return true;   /* armed and immediately terminal — caller reads state */
    }
    LOG_INFO("dcom", "slave %u commissioning mode=%u from addr %u",
             slave_id, (unsigned)mode, s.next_addr);
    if (mode != DCOM_MODE_ALL) {
        s.after_check = P_INITIALISE;
        s.phase       = P_CHECK_FREE;
    }
    return true;
}

void dcom_abort(void) {
    if (s.active) s.abort_req = true;
}

void dcom_tick(void) {
    if (!s.active) return;

    if (s.busy) {
        if (s.have) {
            s.have = false;
            s.busy = false;
            advance();
        } else if (time_reached(s.deadline)) {
            /* No result in time: treat as a failed step. TERMINATE regardless so
             * the bus does not stay in commissioning mode. */
            s.exp_gen = 0;
            s.busy    = false;
            LOG_WARN("dcom", "slave %u step %s timed out", s.slave_id, dcom_phase_name());
            if (s.phase == P_TERMINATE) { finish(DCOM_ERR_TIMEOUT, "timeout"); return; }
            s.state = DCOM_ERR_TIMEOUT;
            s.phase = P_TERMINATE;
        }
        return;
    }
    issue();   /* false = queue momentarily full; retried next tick */
}

dcom_state_t dcom_state(void)   { return s.state; }
uint8_t      dcom_count(void)   { return s.count; }
bool         dcom_busy(void)    { return s.active; }
const char  *dcom_phase_name(void) {
    return (s.phase <= P_TERMINATE) ? PHASE_NAME[s.phase] : "?";
}
