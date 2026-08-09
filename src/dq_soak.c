/*
 * dq_soak.c — command-queue soak/throughput probe. See dq_soak.h.
 *
 * Stimulus: alternating SET DTR0 / QUERY CONTENT DTR0 against one ballast.
 * The query's answer must equal the byte the immediately preceding SET wrote,
 * so ordering, attribution and delivery are all checked by one comparison —
 * and because consecutive pairs use different values, a result that is one
 * command stale ("lag-1", the v1 failure signature) is distinguishable from
 * random corruption.
 *
 * Flow control: the probe submits straight into mbq_client (bypassing the
 * dali_queue priority backlog) and gates on mbq_client_inflight() — the
 * client's view of how many of the gateway's CMDQ_DEPTH command slots are
 * occupied — so it never leans on a queue-full rejection as flow control. The
 * gate counts commands from ALL producers, so background work sharing the
 * queue correctly narrows the probe's window instead of overfilling the
 * gateway.
 *
 * Under queue v4 the useful depth range is 1..CMDQ_DEPTH (2), and the number
 * the run exists to produce is "txns per command" ≈ 1.0 — one Modbus
 * transaction per DALI frame is the whole point of the protocol, and a
 * correctness-only check would miss its loss (task-07 lesson: validate rate,
 * not just correctness).
 */

#include "dq_soak.h"
#include "dali_queue.h"     /* DQ_ANSWER */
#include "mbq_client.h"
#include "modbus.h"
#include "log.h"
#include "pico/time.h"
#include <string.h>
#include "cdc_out.h"
#include <stdio.h>

/* DALI wire constants used by the probe (the gateway's dali.h equivalents). */
#define DALI_ADDR_SET_DTR0        0xA3u   /* special command: addr byte = 0xA3, data = value */
#define DALI_CMD_QUERY_DTR0       0x98u   /* QUERY CONTENT DTR0 (IEC 62386-102) */
#define SOAK_DALI_NAK             0x01u   /* gateway DALI_RESP_NAK: no backward frame */

/* Stride for the value walk: odd ⇒ coprime with 256 ⇒ all 256 values before a
 * repeat, and adjacent pairs never collide. */
#define SOAK_VAL_STRIDE           97u

#define SOAK_MISS_LOG             6u   /* mismatches kept in detail for the report */

/* Scheduler-path backoff after a class-full refusal — roughly a fifth of a DALI
 * frame, so the class is re-offered promptly without spinning. */
#define SOAK_CLASS_RETRY_MS       5u

typedef struct {
    bool     in_use;
    bool     is_query;
    uint32_t seq;
    uint8_t  expect;    /* query: the value its paired SET wrote */
    uint8_t  prev;      /* query: the value of the PREVIOUS pair (lag-1 signature) */
    absolute_time_t t_submit;
} soak_rec_t;

static struct {
    bool     running;      /* still submitting new pairs */
    soak_path_t path;      /* how commands reach the gateway */
    absolute_time_t retry_at;  /* scheduler path: don't re-offer until this time */
    bool     draining;     /* target reached; waiting for in-flight to resolve */
    uint8_t  slave, addr, depth;
    uint32_t pairs_target; /* 0 = unlimited */

    uint32_t seq_next;     /* next submit sequence number */
    uint32_t seq_last_done; /* last delivered sequence (order check) */
    bool     have_done;
    uint8_t  val_next;     /* value the next SET will write */
    uint8_t  val_cur;      /* value the last SET wrote (the pending query's expectation) */
    uint8_t  val_prev;     /* the pair before that */
    bool     next_is_query;
    uint32_t pairs_done;

    soak_rec_t rec[CMDQ_DEPTH + 1u];

    absolute_time_t t_start, t_end;

    /* outcome counters */
    uint32_t n_submit, n_set_ok, n_set_fail;
    uint32_t n_query_ok, n_lag1, n_other, n_query_fail, n_lost;
    uint32_t n_result_lost;  /* of n_lost: the command ran, its answer died */
    uint32_t n_reject;     /* mbq_client_submit refused (modbus action queue full) */
    uint32_t n_ooo;        /* results delivered out of submit order (informational) */

    /* latency, µs */
    uint32_t lat_min, lat_max;
    uint64_t lat_sum;
    uint32_t lat_n;
    uint64_t lat_q_sum;    /* query commands only */
    uint32_t lat_q_n;

    /* pacing diagnostics, sampled once per tick while submitting */
    uint32_t t_queue_full; /* gate closed: gateway queue at `depth` — DALI-bound */
    uint32_t t_tx_busy;    /* room in the queue but the Modbus SM was mid-transaction */
    uint32_t t_ready;      /* room AND the bus was free — we submitted */
    uint32_t inflight_hist[CMDQ_DEPTH + 2u];

    /* Modbus transactions attributable to this slave over the run */
    uint32_t tx0_submits, tx0_polls;
    uint32_t tx_submits, tx_polls;

    struct {
        uint32_t seq;
        uint8_t  expect, got, kind;   /* kind: 0=value, 1=nak, 2=err, 3=lost */
    } miss[SOAK_MISS_LOG];
    uint8_t  miss_n;

    bool     have_result;  /* a run has been completed/stopped — report is meaningful */
} S;

/* ── helpers ─────────────────────────────────────────────────────────────── */

static soak_rec_t *rec_alloc(void) {
    for (uint8_t i = 0; i < CMDQ_DEPTH + 1u; i++)
        if (!S.rec[i].in_use) return &S.rec[i];
    return NULL;
}

static uint8_t inflight_recs(void) {
    uint8_t n = 0;
    for (uint8_t i = 0; i < CMDQ_DEPTH + 1u; i++) if (S.rec[i].in_use) n++;
    return n;
}

static void miss_log(uint32_t seq, uint8_t expect, uint8_t got, uint8_t kind) {
    if (S.miss_n < SOAK_MISS_LOG) {
        S.miss[S.miss_n].seq    = seq;
        S.miss[S.miss_n].expect = expect;
        S.miss[S.miss_n].got    = got;
        S.miss[S.miss_n].kind   = kind;
        S.miss_n++;
    }
}

static void run_finish(void) {
    S.running = false;
    S.draining = false;
    S.t_end = get_absolute_time();
    mbq_client_txn_counts(S.slave, &S.tx_submits, &S.tx_polls);
    S.tx_submits -= S.tx0_submits;
    S.tx_polls   -= S.tx0_polls;
    S.have_result = true;
    LOG_INFO("soak", "finished: %lu pairs, %lu mismatches",
             (unsigned long)S.pairs_done,
             (unsigned long)(S.n_lag1 + S.n_other));
}

/* ── completion ──────────────────────────────────────────────────────────── */

static void soak_cb(uint8_t slave, mbq_result_t r, uint8_t response8, void *ctx) {
    (void)slave;
    soak_rec_t *rec = (soak_rec_t *)ctx;

    uint64_t lat = (uint64_t)absolute_time_diff_us(rec->t_submit, get_absolute_time());
    if (lat > 0xFFFFFFFFull) lat = 0xFFFFFFFFull;
    uint32_t lat32 = (uint32_t)lat;
    if (S.lat_n == 0 || lat32 < S.lat_min) S.lat_min = lat32;
    if (lat32 > S.lat_max) S.lat_max = lat32;
    S.lat_sum += lat32;
    S.lat_n++;

    /* Delivery order: the gateway executes in accept order and its response
     * queue is FIFO, and v4 delivers at most one result per reply — so under a
     * correct implementation results arrive in submit order and this counter
     * should read 0. (Under v2 it legitimately did not: two terminals collected
     * by one status read were delivered in slot-index order, which inverted
     * ~6 % of pairs.) Still reported rather than failed; a value mismatch is
     * the real defect signal. */
    if (S.have_done && rec->seq != S.seq_last_done + 1u) S.n_ooo++;
    S.have_done = true;
    S.seq_last_done = rec->seq;

    if (!rec->is_query) {
        /* A no-answer forward frame (KIND_CMD) is completed by the gateway as
         * FAILED/DALI_RESP_NAK — "no backward frame" is the normal, successful
         * outcome for a fire-and-forget send (dali_queue.h). Only a genuine bus
         * error (collision/bus fault) or LOST is a failure here. */
        if (r == MBQ_R_COMPLETE
            || (r == MBQ_R_FAILED && response8 == SOAK_DALI_NAK)) S.n_set_ok++;
        else if (r == MBQ_R_LOST || r == MBQ_R_RESULT_LOST) {
            S.n_lost++;
            if (r == MBQ_R_RESULT_LOST) S.n_result_lost++;
            miss_log(rec->seq, 0, 0, 3);
        }
        else                        { S.n_set_fail++; miss_log(rec->seq, 0, response8, 1); }
    } else {
        S.lat_q_sum += lat32;
        S.lat_q_n++;
        if (r == MBQ_R_COMPLETE) {
            if (response8 == rec->expect)    S.n_query_ok++;
            else if (response8 == rec->prev) { S.n_lag1++;  miss_log(rec->seq, rec->expect, response8, 0); }
            else                             { S.n_other++; miss_log(rec->seq, rec->expect, response8, 0); }
        } else if (r == MBQ_R_LOST || r == MBQ_R_RESULT_LOST) {
            S.n_lost++;
            if (r == MBQ_R_RESULT_LOST) S.n_result_lost++;
            miss_log(rec->seq, rec->expect, 0, 3);
        } else {
            S.n_query_fail++; miss_log(rec->seq, rec->expect, response8, 2);
        }
    }

    rec->in_use = false;

    if (S.draining && inflight_recs() == 0) run_finish();
}

/* Adapter for the scheduler path: dq_result_t carries the same information in a
 * different shape, so translate it back and reuse the one verification path. */
static void soak_dq_cb(uint8_t slave, dq_result_t r, uint8_t answer, void *ctx) {
    switch (r) {
        case DQ_RES_OK:  soak_cb(slave, MBQ_R_COMPLETE, answer, ctx); break;
        case DQ_RES_NAK: soak_cb(slave, MBQ_R_FAILED, SOAK_DALI_NAK, ctx); break;
        case DQ_RES_ERR: soak_cb(slave, MBQ_R_FAILED, answer ? answer : 0x03u, ctx); break;
        default:         soak_cb(slave, MBQ_R_LOST, 0, ctx); break;
    }
}

/* ── driver ──────────────────────────────────────────────────────────────── */

/* Build and submit the next command of the alternating sequence. Returns false
 * if the client refused it (modbus action queue full) — nothing was consumed. */
static bool submit_next(void) {
    soak_rec_t *rec = rec_alloc();
    if (!rec) return false;

    uint8_t cmd[MBQ_CMD_BYTES];
    if (!S.next_is_query) {
        /* SET DTR0 = val_next. No answer expected, no send-twice: the gateway
         * issues it as KIND_CMD and completes the slot as soon as the forward
         * frame is out. */
        cmd[0] = DALI_ADDR_SET_DTR0;
        cmd[1] = S.val_next;
        cmd[2] = 0u;
        cmd[3] = 0u;
        rec->is_query = false;
    } else {
        /* QUERY CONTENT DTR0 from the target ballast — must answer val_cur. */
        cmd[0] = (uint8_t)((S.addr << 1) | 0x01u);
        cmd[1] = DALI_CMD_QUERY_DTR0;
        cmd[2] = DQ_ANSWER;
        cmd[3] = 0u;
        rec->is_query = true;
        rec->expect   = S.val_cur;
        rec->prev     = S.val_prev;
    }

    rec->seq      = S.seq_next;
    rec->t_submit = get_absolute_time();
    rec->in_use   = true;

    /* Idempotent: both frames are safe to repeat after a gateway reboot (a SET
     * DTR0 re-write and a query have no cumulative effect), so a reboot mid-run
     * resubmits rather than reporting LOST — the value check still holds. */
    bool sent;
    if (S.path == SOAK_PATH_DIRECT) {
        sent = mbq_client_submit(S.slave, cmd, true, soak_cb, rec);
    } else {
        sent = dq_submit(S.slave, S.path == SOAK_PATH_BG ? DQ_PRIO_BACKGROUND
                                                         : DQ_PRIO_USER,
                         cmd[0], cmd[1], cmd[2], cmd[3], true, soak_dq_cb, rec);
    }
    if (!sent) {
        rec->in_use = false;
        S.n_reject++;
        return false;
    }

    S.seq_next++;
    S.n_submit++;
    if (!S.next_is_query) {
        /* The SET landed: it now defines the next query's expectation. */
        S.val_prev = S.val_cur;
        S.val_cur  = S.val_next;
        S.val_next = (uint8_t)(S.val_next + SOAK_VAL_STRIDE);
        S.next_is_query = true;
    } else {
        S.next_is_query = false;
        S.pairs_done++;
    }
    return true;
}

void dq_soak_tick(void) {
    if (!S.running) return;

    if (S.pairs_target && S.pairs_done >= S.pairs_target && !S.next_is_query) {
        S.draining = true;
        S.running  = false;
        if (inflight_recs() == 0) run_finish();
        return;
    }

    /* Pacing sample: why we are (or are not) submitting this instant. */
    uint8_t occupied = mbq_client_inflight(S.slave);
    S.inflight_hist[occupied <= CMDQ_DEPTH + 1u ? occupied : CMDQ_DEPTH + 1u]++;

    if (S.path != SOAK_PATH_DIRECT) {
        /* Scheduler path: keep that class's storage fed and let its own depth
         * policy decide when to ship. A refused submit means the class is full,
         * which is the scheduler doing its job, not an error — but the main loop
         * spins far faster than DALI drains, so back off before re-offering
         * instead of retrying thousands of times per second. Any polling producer
         * against a deliberately shallow class needs this. */
        if (inflight_recs() >= S.depth) { S.t_queue_full++; return; }
        if (!time_reached(S.retry_at)) { S.t_tx_busy++; return; }
        if (!submit_next()) {
            S.t_queue_full++;
            S.retry_at = make_timeout_time_ms(SOAK_CLASS_RETRY_MS);
        } else {
            S.t_ready++;
        }
        return;
    }

    /* Gate on the client's OWN admission policy, not on a raw count. The
     * client admits one command beyond the gateway's depth once the oldest is
     * estimated complete — that is what makes a single transaction collect a
     * result and submit the next command — and a probe that second-guesses it
     * with its own counter would throttle exactly the behaviour gate V1
     * measures. `depth` still caps the run for the sweep. */
    if (occupied > S.depth) { S.t_queue_full++; return; }
    if (!mbq_client_can_submit(S.slave)) { S.t_tx_busy++; return; }
    S.t_ready++;
    submit_next();
}

/* ── control ─────────────────────────────────────────────────────────────── */

bool dq_soak_start(uint8_t slave, uint8_t addr, uint8_t depth, uint32_t pairs,
                   soak_path_t path) {
    if (S.running || S.draining) return false;
    if (depth < 1u) depth = 1u;
    if (depth > CMDQ_DEPTH) depth = CMDQ_DEPTH;

    memset(&S, 0, sizeof S);
    S.slave = slave;
    S.addr  = addr;
    S.depth = depth;
    S.path  = path;
    S.pairs_target = pairs;
    S.val_next = 0x5Au;      /* arbitrary non-trivial start */
    S.val_cur  = 0u;
    S.val_prev = 0u;
    S.next_is_query = false;
    S.t_start = get_absolute_time();
    mbq_client_txn_counts(slave, &S.tx0_submits, &S.tx0_polls);
    mbq_client_txn_stats_reset(slave);
    modbus_busload_reset();   /* the run's report includes who else used the bus */
    S.running = true;
    LOG_INFO("soak", "start slave=%u addr=%u depth=%u pairs=%lu path=%u",
             slave, addr, depth, (unsigned long)pairs, (unsigned)path);
    return true;
}

void dq_soak_stop(void) {
    if (!S.running && !S.draining) return;
    S.running  = false;
    S.draining = true;
    if (inflight_recs() == 0) run_finish();
}

bool dq_soak_active(void) { return S.running || S.draining; }

/* ── report ──────────────────────────────────────────────────────────────── */

/* Print `num/den` as a fixed-point value with 2 decimals, integer math only
 * (this firmware is float-free). */
static void print_rate(const char *label, uint64_t num, uint64_t den, const char *unit) {
    if (den == 0) { cdc_out_printf("  %-22s n/a\n", label); return; }
    uint64_t x100 = (num * 100u) / den;
    cdc_out_printf("  %-22s %lu.%02lu %s\n", label,
           (unsigned long)(x100 / 100u), (unsigned long)(x100 % 100u), unit);
}

static void print_ms(const char *label, uint64_t us) {
    cdc_out_printf("  %-22s %lu.%02lu ms\n", label,
           (unsigned long)(us / 1000u), (unsigned long)((us % 1000u) / 10u));
}

void dq_soak_report(void) {
    if (!S.have_result && !S.running && !S.draining) {
        cdc_out_printf("soak: never run\n");
        return;
    }

    absolute_time_t end = (S.running || S.draining) ? get_absolute_time() : S.t_end;
    uint64_t el_us = (uint64_t)absolute_time_diff_us(S.t_start, end);
    if (el_us == 0) el_us = 1;

    uint32_t txs = S.tx_submits, txp = S.tx_polls;
    if (S.running || S.draining) {
        mbq_client_txn_counts(S.slave, &txs, &txp);
        txs -= S.tx0_submits;
        txp -= S.tx0_polls;
    }

    uint32_t bad = S.n_lag1 + S.n_other + S.n_query_fail + S.n_lost + S.n_set_fail;

    static const char *pn[] = { "direct", "sched-bg", "sched-user" };
    cdc_out_printf("soak %s: slave=%u addr=%u depth=%u path=%s\n",
           S.running ? "RUNNING" : (S.draining ? "DRAINING" : "done"),
           S.slave, S.addr, S.depth, pn[S.path <= SOAK_PATH_USER ? S.path : 0]);
    cdc_out_printf("  elapsed %lu.%02lu s, %lu pairs, %lu commands (%lu DALI frames)\n",
           (unsigned long)(el_us / 1000000u), (unsigned long)((el_us % 1000000u) / 10000u),
           (unsigned long)S.pairs_done, (unsigned long)S.n_submit,
           (unsigned long)S.n_submit);

    cdc_out_printf(" verify:\n");
    cdc_out_printf("  %-22s %lu\n", "query correct", (unsigned long)S.n_query_ok);
    cdc_out_printf("  %-22s %lu\n", "query LAG-1 (stale)", (unsigned long)S.n_lag1);
    cdc_out_printf("  %-22s %lu\n", "query OTHER (wrong)", (unsigned long)S.n_other);
    cdc_out_printf("  %-22s %lu\n", "query nak/err", (unsigned long)S.n_query_fail);
    cdc_out_printf("  %-22s %lu / %lu\n", "set ok / failed",
           (unsigned long)S.n_set_ok, (unsigned long)S.n_set_fail);
    cdc_out_printf("  %-22s %lu (%lu ran, answer lost)\n", "lost (gw reboot)",
           (unsigned long)S.n_lost, (unsigned long)S.n_result_lost);
    cdc_out_printf("  %-22s %lu\n", "submit rejected", (unsigned long)S.n_reject);
    cdc_out_printf("  %-22s %lu (informational)\n", "out-of-order delivery", (unsigned long)S.n_ooo);
    cdc_out_printf("  %-22s %s (%lu bad)\n", "VERDICT", bad ? "FAIL" : "clean",
           (unsigned long)bad);

    for (uint8_t i = 0; i < S.miss_n; i++) {
        static const char *kn[] = { "value", "nak", "err", "lost" };
        cdc_out_printf("   #%lu seq=%lu %s expect=0x%02X got=0x%02X\n",
               (unsigned long)i, (unsigned long)S.miss[i].seq,
               kn[S.miss[i].kind & 3u], S.miss[i].expect, S.miss[i].got);
    }

    cdc_out_printf(" timing:\n");
    print_rate("pair rate", (uint64_t)S.pairs_done * 1000000u, el_us, "pairs/s");
    print_rate("DALI frame rate", (uint64_t)S.n_submit * 1000000u, el_us, "frames/s");
    if (S.pairs_done)
        print_ms("per pair", el_us / S.pairs_done);
    if (S.lat_n)   print_ms("latency avg (all)", S.lat_sum / S.lat_n);
    if (S.lat_q_n) print_ms("latency avg (query)", S.lat_q_sum / S.lat_q_n);
    if (S.lat_n) {
        print_ms("latency min", S.lat_min);
        print_ms("latency max", S.lat_max);
    }

    cdc_out_printf(" transport:\n");
    /* Queue transactions only: the client's cold-path sysinfo/version reads are
     * not counted here, so on a recovery-heavy run this under-reports what was
     * actually on the wire. `busload` measures the truth. */
    cdc_out_printf("  %-22s %lu submit + %lu poll = %lu (queue txns only)\n", "modbus txns",
           (unsigned long)txs, (unsigned long)txp, (unsigned long)(txs + txp));
    if (S.n_submit)
        print_rate("txns per command", (uint64_t)(txs + txp), S.n_submit, "");
    print_rate("txn rate", (uint64_t)(txs + txp) * 1000000u, el_us, "txn/s");
    mbq_txn_stats_t ts;
    mbq_client_txn_stats(S.slave, &ts);
    if (ts.n) {
        print_ms("txn time avg", ts.us_avg);
        print_ms("txn time min", ts.us_min);
        print_ms("txn time max", ts.us_max);
        cdc_out_printf("  %-22s %lu fail (%lu device-busy) of %lu\n", "txn failures",
               (unsigned long)ts.n_fail, (unsigned long)ts.n_busy, (unsigned long)ts.n);
    }

    /* Bus share: the soak can only run as fast as the share of the RS-485 bus it
     * actually gets, so the report names the competition rather than leaving a
     * slow result unexplained. */
    {
        uint64_t win = 0, busy = 0;
        uint32_t bt = 0;
        modbus_busload_slave_t per[MODBUS_BUSLOAD_SLAVES];
        modbus_busload_get(&win, &busy, &bt, per);
        if (win == 0) win = 1;
        uint32_t rt = 0, rb = 0, ta = 0, tm = 0;
        modbus_busload_retries(&rt, &rb);
        modbus_busload_turnaround(&ta, &tm, NULL);
        cdc_out_printf("  %-22s %lu%% busy over %lu txns\n", "RS-485 bus",
               (unsigned long)(busy * 100u / win), (unsigned long)bt);
        cdc_out_printf("  %-22s %lu timeout, %lu device-busy\n", "re-sends",
               (unsigned long)rt, (unsigned long)rb);
        cdc_out_printf("  %-22s avg %lu.%02lu ms, max %lu.%02lu ms\n", "slave turnaround",
               (unsigned long)(ta / 1000u), (unsigned long)((ta % 1000u) / 10u),
               (unsigned long)(tm / 1000u), (unsigned long)((tm % 1000u) / 10u));
        for (uint8_t i = 0; i < MODBUS_BUSLOAD_SLAVES; i++) {
            if (!per[i].txns) continue;
            cdc_out_printf("    slave %-3u %5lu txns %3lu%% of wall  %lu.%02lu ms avg\n",
                   per[i].slave, (unsigned long)per[i].txns,
                   (unsigned long)(per[i].us * 100u / win),
                   (unsigned long)((per[i].us / per[i].txns) / 1000u),
                   (unsigned long)(((per[i].us / per[i].txns) % 1000u) / 10u));
        }
    }

    cdc_out_printf(" pacing (why we weren't submitting):\n");
    uint64_t tt = (uint64_t)S.t_queue_full + S.t_tx_busy + S.t_ready;
    if (tt == 0) tt = 1;
    cdc_out_printf("  %-22s %lu%%  (gateway queue at depth — DALI-bound)\n", "queue full",
           (unsigned long)((uint64_t)S.t_queue_full * 100u / tt));
    cdc_out_printf("  %-22s %lu%%  (room in queue, bus mid-txn — Modbus-bound)\n", "tx busy",
           (unsigned long)((uint64_t)S.t_tx_busy * 100u / tt));
    cdc_out_printf("  %-22s %lu%%\n", "submitted", (unsigned long)((uint64_t)S.t_ready * 100u / tt));
    cdc_out_printf("  inflight:");
    for (uint8_t i = 0; i <= CMDQ_DEPTH + 1u; i++) {
        uint64_t pct = (uint64_t)S.inflight_hist[i] * 100u / tt;
        if (S.inflight_hist[i]) cdc_out_printf(" %u:%lu%%", i, (unsigned long)pct);
    }
    cdc_out_printf("\n");
}
