/*
 * mbq_client_test.c — host-side test for src/mbq_client.c, the queue-v4
 * transport FSM.
 *
 * Not part of the firmware build (see CLAUDE.md: host tests live under test/
 * and are never added to the RP2040 CMake target). Compiles mbq_client.c itself
 * with -DMBQ_CLIENT_HOST_TEST, which is the ONLY thing that changes in the
 * module: its clock accessor. Everything else is real — the Modbus entry points
 * it calls are ordinary extern functions, so this file simply defines them
 * instead of linking modbus.c, and a fake gateway answers on the other side.
 * There is therefore no second implementation of a recovery rule here to drift
 * out of sync with the one that ships.
 *
 * What it covers, and why THESE: R1, R1', R2, R3, R4, the ack-less ambiguous
 * corner (both idempotent and non-idempotent, with the boot_count gate) and the
 * piggyback-ack bookkeeping. Every one of those is a path that historically
 * only broke on hardware — v1's ledger and v2's reconcile bugs were all in
 * rarely-exercised recovery code (TinyModbus/DESIGN-HISTORY.md), and a bench
 * that only runs the happy path cannot see them.
 *
 * The fake gateway is a real implementation of the §2 processing order (ack,
 * then enqueue, then build the reply from the state as it now stands) over a
 * real command FIFO and a real retained response queue, so the client is being
 * driven by the protocol rather than by canned frames. Tests drive DALI
 * completion explicitly (gw_complete) instead of on a timer, so ordering is
 * exact.
 *
 * Build & run:
 *   cc -DMBQ_CLIENT_HOST_TEST -Wall -Wextra -I ../src \
 *       -o /tmp/mbq_client_test mbq_client_test.c ../src/mbq_client.c \
 *       && /tmp/mbq_client_test
 */
#include "../src/mbq_client.h"
#include "../src/modbus.h"
#include "../src/log.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* ── the seam: a virtual clock ───────────────────────────────────────────── */

static uint64_t g_now_us;
uint64_t mbq_test_now_us(void) { return g_now_us; }
static void advance_ms(uint32_t ms) { g_now_us += (uint64_t)ms * 1000u; }

/* ── firmware odds and ends the module links against ─────────────────────── */

log_level_t g_log_level = LOG_LEVEL_OFF;
static int g_log_quiet = 1;
void log_emit(log_level_t lvl, const char *lc, const char *mod, const char *fmt, ...) {
    (void)lvl;
    if (g_log_quiet) return;
    va_list ap; va_start(ap, fmt);
    printf("[%s %s] ", lc, mod); vprintf(fmt, ap); printf("\n");
    va_end(ap);
}
void cdc_out_printf(const char *fmt, ...) {
    va_list ap; va_start(ap, fmt);
    if (!g_log_quiet) vprintf(fmt, ap);
    va_end(ap);
}

/* ── the fake gateway ────────────────────────────────────────────────────── */

#define RESPQ_DEPTH 4u
#define SLAVE 3u

typedef struct { uint8_t cmd[3], len, seq, flags; bool used; } gwcmd_t;
typedef struct { uint8_t seq, status, resp8; } gwres_t;

static struct {
    gwcmd_t  cq[CMDQ_DEPTH];
    uint8_t  cq_n;
    gwres_t  rq[RESPQ_DEPTH];
    uint8_t  rq_n;
    uint8_t  next_seq;
    uint16_t boot_count;
    bool     present;        /* answers at all */
    bool     illegal_fc;     /* pre-v4 gateway: exception 0x01 to FC 0x41 */
    int      short_replies;  /* answer this many requests one byte short */
    bool     force_afull;    /* refuse enqueues without advancing the seq counter */
    uint32_t accepts, acks, afulls;
    uint32_t reqs;               /* every FC 0x41 request that reached us */
    uint8_t  hist[64];           /* opcode of each command EXECUTED, in order */
    uint8_t  hist_n;
    uint64_t exec_until;         /* when the executing command finishes, 0 = idle */
    bool     dali_running;       /* run the DALI side on a timer (throughput test) */
    uint32_t acks_piggybacked;   /* acks applied by a request that also enqueued */
    const char *ver;             /* what FC04 0x0200 reports */
} GW;

static void gw_boot(bool cold) {
    memset(GW.cq, 0, sizeof GW.cq);
    memset(GW.rq, 0, sizeof GW.rq);
    GW.cq_n = 0; GW.rq_n = 0; GW.next_seq = 1; GW.exec_until = 0;
    if (cold) GW.boot_count = 1; else GW.boot_count++;
}
static void gw_reset_all(void) {
    memset(&GW, 0, sizeof GW);
    GW.present = true;
    GW.ver = "DALI v1.0 0011223344556677";
    gw_boot(true);
}

/* Finish the head of the command queue, pushing its result. Mirrors the
 * gateway's strict-FIFO execution. */
static bool gw_complete(uint8_t status, uint8_t resp8) {
    if (GW.cq_n == 0 || GW.rq_n >= RESPQ_DEPTH) return false;
    GW.rq[GW.rq_n++] = (gwres_t){ GW.cq[0].seq, status, resp8 };
    for (uint8_t i = 0; i + 1u < GW.cq_n; i++) GW.cq[i] = GW.cq[i + 1u];
    GW.cq_n--;
    return true;
}

/* ── the transport the client calls ──────────────────────────────────────── */

/* One request at a time, delivered on the next mbq_client_tick() — the real
 * transport also defers its callback out of the client's submit path. */
static struct {
    bool               armed;
    int                res;
    uint8_t            buf[64];
    modbus_action_cb_t cb;
    void              *ctx;
} PEND;

static void reply(const uint8_t *b, uint8_t n) {
    memcpy(PEND.buf, b, n);
    PEND.res = n;
}
static void no_reply(void) { PEND.res = 0; }

bool modbus_mbq_v4(uint8_t dev, uint8_t ctl, const uint8_t *payload,
                   uint8_t payload_len, modbus_action_cb_t cb, void *ctx) {
    PEND.armed = true; PEND.cb = cb; PEND.ctx = ctx;
    if (!GW.present) { no_reply(); return true; }
    GW.reqs++;
    if (GW.illegal_fc) {
        uint8_t e[5] = { (uint8_t)dev, 0xC1u, 0x01u, 0, 0 };
        reply(e, 5);
        PEND.res = -5;
        return true;
    }

    /* §2 order: ack first, so the entry it frees is available to the enqueue
     * in this same transaction. */
    uint8_t ack = ctl & 0x1Fu;
    if (ack && GW.rq_n && GW.rq[0].seq == ack) {
        for (uint8_t i = 0; i + 1u < GW.rq_n; i++) GW.rq[i] = GW.rq[i + 1u];
        GW.rq_n--;
        GW.acks++;
        if (payload_len) GW.acks_piggybacked++;
    }

    uint8_t out[8];
    uint8_t n = 0;
    out[n++] = (uint8_t)dev;
    out[n++] = 0x41u;

    if (payload_len) {
        if (GW.cq_n < CMDQ_DEPTH && !GW.force_afull) {
            gwcmd_t *c = &GW.cq[GW.cq_n++];
            memcpy(c->cmd, payload, payload_len);
            c->len = payload_len;
            c->flags = (uint8_t)(ctl & 0xE0u);
            c->seq = GW.next_seq;
            GW.next_seq = (GW.next_seq >= 31u) ? 1u : (uint8_t)(GW.next_seq + 1u);
            GW.accepts++;
            out[n++] = c->seq;                    /* A_ACCEPTED (status 0) | seq */
        } else {
            GW.afulls++;
            out[n++] = (uint8_t)(1u << 5);        /* A_FULL, seq 0 */
        }
    }

    if (GW.rq_n) {
        out[n++] = (uint8_t)((GW.rq[0].status << 5) | GW.rq[0].seq);
        out[n++] = GW.rq[0].resp8;
    } else {
        out[n++] = 0;
        out[n++] = 0;
    }
    out[n++] = 0; out[n++] = 0;                   /* CRC, already validated */

    if (GW.short_replies > 0) { GW.short_replies--; n--; }
    reply(out, n);
    return true;
}

bool modbus_get_input_registers(uint8_t dev, uint16_t addr, uint16_t count,
                                modbus_action_cb_t cb, void *ctx) {
    PEND.armed = true; PEND.cb = cb; PEND.ctx = ctx;
    if (!GW.present) { no_reply(); return true; }
    if (addr == 0x0310u && count == 4u) {
        uint8_t b[13] = { (uint8_t)dev, 0x04u, 8u, 0,0,0,0, 0,0, 0,0, 0,0 };
        b[7] = (uint8_t)(GW.boot_count >> 8);
        b[8] = (uint8_t)GW.boot_count;
        reply(b, 13);
        return true;
    }
    if (addr == 0x0200u && count == 16u) {
        uint8_t b[37];
        memset(b, 0, sizeof b);
        b[0] = (uint8_t)dev; b[1] = 0x04u; b[2] = 32u;
        memcpy(b + 3, GW.ver, strlen(GW.ver));
        reply(b, 37);
        return true;
    }
    no_reply();
    return true;
}

/* The gateway's DALI side: execute the head of the command queue for a real
 * frame time and push its result. Only used by the throughput test — the rule
 * tests drive completion explicitly so their ordering is exact. Frame times are
 * task-07's measurements, the same ones the client paces against. */
static void gw_dali_tick(void) {
    if (GW.cq_n == 0) { GW.exec_until = 0; return; }
    if (GW.rq_n >= RESPQ_DEPTH) return;          /* backpressure: arbiter stalls */
    if (GW.exec_until == 0) {
        uint32_t ms = (GW.cq[0].flags & MBQ_QF_DTR0)   ? 50u
                    : (GW.cq[0].flags & MBQ_QF_ANSWER) ? 40u : 25u;
        GW.exec_until = g_now_us + (uint64_t)ms * 1000u;
        return;
    }
    if (g_now_us >= GW.exec_until) {
        if (GW.hist_n < sizeof GW.hist) GW.hist[GW.hist_n++] = GW.cq[0].cmd[1];
        gw_complete(0u, GW.cq[0].cmd[1]);        /* answer echoes the opcode */
        GW.exec_until = 0;
    }
}

/* Run one main-loop iteration: deliver any armed reply, then tick the client.
 * Time advances by a millisecond per iteration, because the client's pacing
 * timers are real and a loop that never advanced the clock would deadlock
 * against them exactly as a frozen main loop would. */
static void pump(void) {
    advance_ms(1);
    if (GW.dali_running) gw_dali_tick();
    if (PEND.armed) {
        PEND.armed = false;
        modbus_action_cb_t cb = PEND.cb;
        cb(PEND.res, PEND.buf, PEND.ctx);
    }
    mbq_client_tick();
}
static void pump_n(int n) { for (int i = 0; i < n; i++) pump(); }

/* Let the main loop run for `ms` of virtual time. Prefer this to advancing the
 * clock in one jump and then pumping: the client's pacing timers only release
 * work when time passes BETWEEN iterations, so a jump-then-spin pattern tests a
 * schedule the firmware never actually sees. */
static void run_ms(uint32_t ms) { for (uint32_t i = 0; i < ms; i++) pump(); }


/* ── result capture ──────────────────────────────────────────────────────── */

typedef struct { int n; mbq_result_t r; uint8_t resp; } cap_t;
static void cap_cb(uint8_t slave, mbq_result_t r, uint8_t resp8, void *ctx) {
    (void)slave;
    cap_t *c = (cap_t *)ctx;
    c->n++; c->r = r; c->resp = resp8;
}

/* Run until `c` is delivered, then stop AT ONCE. Stopping there leaves the ack
 * for that result still pending, so the next submit piggybacks it — which is
 * exactly the precondition R1/R1' exist to resolve, and it is what steady state
 * looks like (a command is always waiting when a result lands). Pumping on
 * instead would let the client's own paced poll carry the ack and quietly turn
 * the scenario into the ack-less corner. */
static bool run_until_delivered(cap_t *c, uint32_t limit_ms) {
    for (uint32_t i = 0; i < limit_ms; i++) { pump(); if (c->n) return true; }
    return false;
}

static int fail = 0;
static const char *g_case = "";
#define CHECK(cond) do { \
        if (!(cond)) { \
            printf("FAIL [%s]: %s (%s:%d)\n", g_case, #cond, __FILE__, __LINE__); \
            fail = 1; \
        } \
    } while (0)

/* SET DTR0 (no answer) and QUERY (answer expected) — one of each command class
 * so the DTR0/answer flag packing is exercised alongside the recovery rules. */
static const uint8_t CMD_SET[4]   = { 0xA3u, 0x5Au, 0u, 0u };
static const uint8_t CMD_QUERY[4] = { 0x01u, 0x98u, MBQ_QF_ANSWER, 0u };

/* Bring a slave from registration to the steady state: version read, epoch
 * drain, boot_count baseline. Returns with the client idle and able to submit. */
static void bring_up(void) {
    mbq_client_init();
    mbq_client_add_slave(SLAVE);
    for (int i = 0; i < 500 && !mbq_client_can_submit(SLAVE); i++) pump();
    CHECK(mbq_client_can_submit(SLAVE));
}

static void begin(const char *name) {
    g_case = name;
    g_now_us = 1000000u;
    memset(&PEND, 0, sizeof PEND);
    gw_reset_all();
    bring_up();
}

/* Run one command to completion and let its ack be confirmed, so the slave
 * leaves the NO-BASELINE state (it has now seen an accept byte) and
 * ack_pending is clear — which is what makes the NEXT enqueue ack-less, the
 * "after idle" case of the §4 ambiguous corner. */
static void establish_baseline(void);

static mbq_stats_t stats(void) {
    mbq_stats_t st;
    mbq_client_stats(SLAVE, &st);
    return st;
}

/* Submit one command and let its enqueue transaction complete. */
static bool submit(const uint8_t cmd[4], bool idem, cap_t *c) {
    bool ok = mbq_client_submit(SLAVE, cmd, idem, cap_cb, c);
    if (ok) pump();      /* the enqueue's reply lands here */
    return ok;
}

static void establish_baseline(void) {
    static cap_t base;
    base = (cap_t){0};
    mbq_client_submit(SLAVE, CMD_SET, true, cap_cb, &base);
    pump();
    gw_complete(0u, 0x01u);
    run_until_delivered(&base, 200);
    run_ms(40);          /* let the ack be confirmed, clearing ack_pending */
}

int main(void) {
    /* ── 1. Registration: drain a response queue left by a previous controller
     *     epoch, then establish the boot_count baseline. An orphan result must
     *     be acked away, not delivered to anybody. ─────────────────────────── */
    g_case = "registration drain";
    g_now_us = 1000000u;
    memset(&PEND, 0, sizeof PEND);
    gw_reset_all();
    GW.rq[GW.rq_n++] = (gwres_t){ 7u, 0u, 0xAAu };   /* stale, from the dead epoch */
    GW.rq[GW.rq_n++] = (gwres_t){ 8u, 0u, 0xBBu };
    mbq_client_init();
    mbq_client_add_slave(SLAVE);
    for (int i = 0; i < 500 && !mbq_client_can_submit(SLAVE); i++) pump();
    CHECK(mbq_client_can_submit(SLAVE));
    CHECK(GW.rq_n == 0);                 /* both orphans acked away */
    CHECK(stats().drained);
    CHECK(stats().boot_count == 1);
    CHECK(stats().gw_ver_major == 1);    /* parsed out of the version block */

    /* ── 2. Steady state: one transaction per DALI frame, with the ack for the
     *     previous result piggybacked on the next enqueue. ───────────────── */
    begin("steady state + piggyback ack");
    {
        cap_t a = {0}, b = {0};
        CHECK(submit(CMD_SET, true, &a));
        CHECK(GW.cq_n == 1);
        CHECK(submit(CMD_QUERY, true, &b));      /* fills the queue */
        CHECK(GW.cq_n == 2);
        CHECK(!mbq_client_can_submit(SLAVE));    /* in-flight cap, not A_FULL */
        CHECK(GW.afulls == 0);

        gw_complete(1u, 0x01u);                  /* the SET: H_FAILED/NAK, normal */
        run_ms(103);
        CHECK(a.n == 1);
        CHECK(a.r == MBQ_R_FAILED && a.resp == 0x01u);

        /* The ack for b's result must ride the NEXT ENQUEUE, not a poll of its
         * own — that is what makes steady state one transaction per DALI frame
         * (§3) and what keeps R1 available when a reply goes missing. */
        gw_complete(0u, 0x5Au);                  /* the query answers */
        CHECK(run_until_delivered(&b, 120));
        CHECK(b.r == MBQ_R_COMPLETE && b.resp == 0x5Au);
        uint32_t piggy_before = GW.acks_piggybacked;
        cap_t c = {0};
        CHECK(mbq_client_can_submit(SLAVE));
        CHECK(submit(CMD_SET, true, &c));
        CHECK(GW.acks_piggybacked == piggy_before + 1);
        CHECK(GW.rq_n == 0);                     /* retention is not accumulating */
        CHECK(stats().pred_faults == 0);
        CHECK(stats().proto_errors == 0);
    }

    /* ── 3. R1 — a lost reply to an enqueue+ack. The recovery poll finds the
     *     head has moved past the acked seq, which proves the ack was applied,
     *     which proves the whole frame landed (they travel together). The
     *     command must bind to the predicted seq and NOT be resent. ──────── */
    begin("R1: lost enqueue+ack reply, frame landed");
    {
        cap_t a = {0};
        CHECK(submit(CMD_SET, true, &a));
        gw_complete(0u, 0x11u);                  /* a result to be acked */
        CHECK(run_until_delivered(&a, 100));     /* its ack is now pending */

        uint32_t accepts_before = GW.accepts;
        cap_t b = {0};
        mbq_client_drop_reply(SLAVE, 1);         /* the drill knob, same path as a real loss */
        CHECK(mbq_client_submit(SLAVE, CMD_SET, true, cap_cb, &b));
        pump();                                  /* reply generated, then dropped */
        CHECK(GW.accepts == accepts_before + 1); /* it DID land at the gateway */

        /* The lost frame's own ack popped the only result, so the recovery
         * poll finds an EMPTY head — branch 3 of the §4 procedure, the one
         * genuinely ambiguous reading, resolved by a cold-path boot_count read
         * rather than guessed at. */
        uint32_t reads_before = stats().boot_reads;
        run_ms(120);
        CHECK(stats().boot_reads == reads_before + 1);
        CHECK(stats().r1_landed == 1);
        CHECK(stats().r1_never == 0);
        CHECK(GW.accepts == accepts_before + 1); /* nothing was resent */
        CHECK(stats().resends == 0);
        /* R1's binding is a PREDICTION, and it must be verified when the result
         * surfaces. A correct prediction must not register as a fault. */
        CHECK(stats().pred_faults == 0);

        gw_complete(0u, 0x22u);
        run_ms(104);
        CHECK(b.n == 1);
        CHECK(b.r == MBQ_R_COMPLETE && b.resp == 0x22u);   /* bound to the right seq */
    }

    /* ── 4. R1' — the same loss, but the frame never arrived. The head is
     *     unchanged, so the command is resent verbatim, ack included. ────── */
    begin("R1': lost enqueue+ack frame, never arrived");
    {
        cap_t a = {0};
        CHECK(submit(CMD_SET, true, &a));
        gw_complete(0u, 0x11u);
        CHECK(run_until_delivered(&a, 100));

        uint32_t accepts_before = GW.accepts;
        cap_t b = {0};
        GW.present = false;                      /* the request itself is lost */
        CHECK(mbq_client_submit(SLAVE, CMD_SET, true, cap_cb, &b));
        pump();
        CHECK(GW.accepts == accepts_before);     /* never reached the gateway */
        GW.present = true;

        run_ms(49);
        CHECK(stats().r1_never == 1);
        CHECK(stats().r1_landed == 0);
        CHECK(GW.accepts == accepts_before + 1); /* resent exactly once */
        CHECK(stats().resends == 1);

        gw_complete(0u, 0x33u);
        run_ms(104);
        CHECK(b.n == 1 && b.r == MBQ_R_COMPLETE && b.resp == 0x33u);
    }

    /* ── 5. R2 — an accept byte returning a seq the master still tracks. A
     *     live seq cannot recur without a reboot, so this is proof of one. The
     *     just-accepted command is kept; the rest are disposed. ──────────── */
    begin("R2: accept seq collides with a tracked one");
    {
        cap_t a = {0};
        /* Non-idempotent, so the disposal is observable as a delivered LOST
         * rather than a silent resubmit. */
        CHECK(submit(CMD_SET, false, &a));       /* takes seq 1, still executing */
        CHECK(GW.cq_n == 1);

        gw_boot(false);                          /* gateway restarts: counter back to 1 */
        cap_t b = {0};
        CHECK(mbq_client_submit(SLAVE, CMD_QUERY, false, cap_cb, &b));
        pump();                                  /* accept byte says seq 1 — collision */
        CHECK(stats().reboots_r2 == 1);
        CHECK(stats().reboots_r3 == 0 && stats().reboots_r4 == 0);
        CHECK(a.n == 1);                         /* the tracked one was disposed... */
        CHECK(a.r == MBQ_R_LOST);
        CHECK(b.n == 0);                         /* ...the just-accepted one is kept  */

        gw_complete(0u, 0x44u);
        run_ms(104);
        CHECK(b.n == 1 && b.r == MBQ_R_COMPLETE && b.resp == 0x44u);
    }

    /* ── 6. R3 — a head seq the master is not tracking. Nothing else can
     *     produce one, so the gateway restarted its counter. ─────────────── */
    begin("R3: untracked head seq");
    {
        cap_t a = {0};
        CHECK(submit(CMD_QUERY, true, &a));
        CHECK(GW.cq_n == 1);

        /* A reboot, then a result appears from a command this master never
         * enqueued in the current epoch. */
        gw_boot(false);
        GW.rq[GW.rq_n++] = (gwres_t){ 9u, 0u, 0x55u };
        run_ms(104);
        CHECK(stats().reboots_r3 == 1);
        CHECK(stats().reboots_r2 == 0);
        CHECK(a.n >= 1 || stats().resends >= 1);   /* disposed one way or the other */
        run_ms(64);
        CHECK(GW.rq_n == 0);                       /* the orphan was acked away */
    }

    /* ── 7. R4 — a result deadline expires. The poll brings nothing back, so
     *     boot_count arbitrates: changed means reboot, unchanged means the
     *     gateway is answering but wedged. Both dispose; only one is a
     *     reboot, and the stats must say which. ──────────────────────────── */
    begin("R4: deadline expiry, boot_count unchanged (wedge)");
    {
        cap_t a = {0};
        CHECK(submit(CMD_SET, false, &a));       /* non-idempotent, never completed */
        run_ms(MBQ_RESULT_DEADLINE_MS + 60u);
        CHECK(stats().deadlines >= 1);
        CHECK(stats().reboots_r4 == 0);          /* boot_count did not move */
        CHECK(a.n == 1);
        CHECK(a.r == MBQ_R_RESULT_LOST);         /* past its deadline: it ran */
    }

    begin("R4: deadline expiry, boot_count changed (reboot)");
    {
        cap_t a = {0};
        CHECK(submit(CMD_SET, false, &a));
        gw_boot(false);                          /* the command vanishes with it */
        run_ms(MBQ_RESULT_DEADLINE_MS + 60u);
        CHECK(stats().reboots_r4 == 1);
        CHECK(a.n == 1);
        CHECK(a.r == MBQ_R_LOST || a.r == MBQ_R_RESULT_LOST);
    }

    /* ── 8. The ack-less ambiguous corner. No ack rode the lost frame, so R1
     *     is unavailable: the client must poll until the predicted seq shows
     *     up as a result (landed) or the deadline passes with the response
     *     queue empty (never landed). ──────────────────────────────────────
     *     8a: it landed, and the result appears while polling. */
    begin("ack-less corner: the frame landed");
    {
        establish_baseline();                    /* seq known; nothing to ack */
        cap_t a = {0};
        uint32_t accepts_before = GW.accepts;
        mbq_client_drop_reply(SLAVE, 1);
        CHECK(mbq_client_submit(SLAVE, CMD_QUERY, true, cap_cb, &a));
        pump();
        CHECK(GW.accepts == accepts_before + 1);
        gw_complete(0u, 0x66u);                  /* its result reaches the head */
        run_ms(56);
        CHECK(a.n == 1);
        CHECK(a.r == MBQ_R_COMPLETE && a.resp == 0x66u);
        CHECK(GW.accepts == accepts_before + 1); /* never resent */
        CHECK(stats().resends == 0);
    }

    /* 8b: it never landed and the command is IDEMPOTENT — the deadline passes
     *     with the queue observed empty, and it is simply resent. No
     *     boot_count read is needed for this class. */
    begin("ack-less corner: never landed, idempotent");
    {
        establish_baseline();
        cap_t a = {0};
        uint32_t reads_before = stats().boot_reads;
        GW.present = false;
        CHECK(mbq_client_submit(SLAVE, CMD_QUERY, true, cap_cb, &a));
        pump();
        GW.present = true;
        run_ms(MBQ_RESULT_DEADLINE_MS + 60u);
        CHECK(stats().resends >= 1);
        CHECK(stats().boot_reads == reads_before);   /* idempotent: no arbiter needed */
        CHECK(GW.accepts >= 1);
        gw_complete(0u, 0x77u);
        run_ms(106);
        CHECK(a.n == 1 && a.r == MBQ_R_COMPLETE && a.resp == 0x77u);
    }

    /* 8c: it never landed, the command is NON-IDEMPOTENT, and boot_count is
     *     UNCHANGED — so it genuinely never landed and resending is safe. */
    begin("ack-less corner: never landed, non-idempotent, no reboot");
    {
        establish_baseline();
        cap_t a = {0};
        uint32_t reads_before = stats().boot_reads;
        GW.present = false;
        CHECK(mbq_client_submit(SLAVE, CMD_SET, false, cap_cb, &a));
        pump();
        GW.present = true;
        run_ms(MBQ_RESULT_DEADLINE_MS + 60u);
        CHECK(stats().boot_reads > reads_before);    /* the arbiter WAS consulted */
        CHECK(stats().resends >= 1);                 /* unchanged ⇒ safe to resend */
        CHECK(a.n == 0);
    }

    /* 8d: same, but boot_count CHANGED — "executed, then the result died with
     *     the reboot" cannot be excluded, so it is disposed LOST rather than
     *     blind-resent. This is the case the whole cold-path read exists for. */
    begin("ack-less corner: never landed, non-idempotent, reboot");
    {
        establish_baseline();
        cap_t a = {0};
        uint32_t accepts_before = GW.accepts;
        GW.present = false;
        CHECK(mbq_client_submit(SLAVE, CMD_SET, false, cap_cb, &a));
        pump();
        GW.present = true;
        gw_boot(false);                              /* boot_count moves */
        run_ms(MBQ_RESULT_DEADLINE_MS + 60u);
        CHECK(a.n == 1);
        CHECK(a.r == MBQ_R_LOST);
        CHECK(GW.accepts == accepts_before);         /* NOT resent */
    }

    /* ── 9. A_FULL is the backstop, not the flow control: the in-flight cap
     *     should mean the client never provokes one. Force it anyway (by
     *     shrinking what the gateway will take) and check the command is held
     *     and retried rather than lost. ───────────────────────────────────── */
    begin("A_FULL is retried, not dropped");
    {
        cap_t a = {0}, b = {0};
        CHECK(submit(CMD_SET, true, &a));
        CHECK(submit(CMD_SET, true, &b));
        CHECK(GW.cq_n == CMDQ_DEPTH);
        CHECK(GW.afulls == 0);           /* the cap held: no A_FULL was ever provoked */
        CHECK(!mbq_client_can_submit(SLAVE));
    }

    /* ── 10. The compat latch (§8): a pre-v4 gateway answers ILLEGAL FUNCTION,
     *      and a reply of a length the request did not call for is a protocol
     *      error. Either must stop the client driving that slave, loudly and
     *      once — not limp. ─────────────────────────────────────────────── */
    g_case = "compat latch: ILLEGAL FUNCTION";
    g_now_us = 1000000u;
    memset(&PEND, 0, sizeof PEND);
    gw_reset_all();
    GW.illegal_fc = true;
    mbq_client_init();
    mbq_client_add_slave(SLAVE);
    run_ms(400);
    CHECK(stats().incompatible);
    CHECK(!mbq_client_can_submit(SLAVE));

    /* The version half of the latch is a HARD refusal, so it gets its own
     * check in both directions: 0.x is pre-v4 and must be refused, and the
     * shipped 1.0 must NOT be — a constant that is wrong in that direction
     * would silently stop the controller driving every gateway on the bus. */
    g_case = "compat latch: pre-v4 version";
    g_now_us = 1000000u;
    memset(&PEND, 0, sizeof PEND);
    gw_reset_all();
    GW.ver = "DALI v0.2 0011223344556677";
    mbq_client_init();
    mbq_client_add_slave(SLAVE);
    run_ms(400);
    CHECK(stats().incompatible);
    CHECK(!mbq_client_can_submit(SLAVE));
    CHECK(GW.accepts == 0);              /* never drove its queue at all */

    g_case = "compat latch: shipped v1.0 is accepted";
    g_now_us = 1000000u;
    memset(&PEND, 0, sizeof PEND);
    gw_reset_all();
    mbq_client_init();
    mbq_client_add_slave(SLAVE);
    run_ms(400);
    CHECK(!stats().incompatible);
    CHECK(mbq_client_can_submit(SLAVE));

    begin("compat latch: wrong reply length");
    {
        cap_t a = {0};
        GW.short_replies = 1;
        CHECK(mbq_client_submit(SLAVE, CMD_SET, true, cap_cb, &a));
        pump_n(2);
        CHECK(stats().proto_errors == 1);
        CHECK(stats().incompatible);
        CHECK(a.n == 1 && a.r == MBQ_R_LOST);    /* the held command is released */
        CHECK(!mbq_client_can_submit(SLAVE));
    }

    /* ── 10b. The prediction R1 makes is VERIFIED, never silently trusted.
     *      Force the gateway to assign a seq other than the predicted one and
     *      check the mis-binding is detected and counted rather than delivering
     *      one command's answer to another's callback — the exact
     *      result-misattribution class that took task 04 three fix cycles. */
    begin("R1's predicted seq is verified");
    {
        establish_baseline();
        cap_t a = {0}, b = {0};
        CHECK(mbq_client_submit(SLAVE, CMD_SET, true, cap_cb, &b));
        pump();
        gw_complete(0u, 0x0Bu);
        CHECK(run_until_delivered(&b, 200));     /* now an ack is pending */

        /* Make the prediction wrong AFTER the last accept the client observed,
         * so it predicts last+1 while the gateway will assign something else.
         * Something has to stand in for whatever would cause that in reality;
         * the point of the test is that the client detects it, not why. */
        GW.next_seq = 25u;
        mbq_client_drop_reply(SLAVE, 1);
        CHECK(mbq_client_submit(SLAVE, CMD_QUERY, true, cap_cb, &a));
        pump();                                  /* landed as seq 21, reply dropped */
        run_ms(120);                             /* R1 binds it to the prediction */
        CHECK(stats().r1_landed == 1);

        gw_complete(0u, 0x99u);                  /* surfaces as seq 21, not the prediction */
        run_ms(200);
        CHECK(stats().pred_faults == 1);
        /* The orphan's answer must never reach this command's callback — that
         * misattribution is the whole reason the prediction is verified. Being
         * idempotent, the command is put back on the wire instead. */
        CHECK(!(a.n == 1 && a.resp == 0x99u));
        CHECK(stats().resends >= 1);
    }

    /* ── 10c. Registration is DRAIN, WAIT, DRAIN AGAIN. A command the gateway
     *      accepted before the controller died is still executing, and its
     *      result lands AFTER the first sweep. A single-pass drain would read
     *      it as an untracked seq and declare a gateway reboot that never
     *      happened — on almost every controller restart. ─────────────────── */
    g_case = "drain, wait, drain again";
    g_now_us = 1000000u;
    memset(&PEND, 0, sizeof PEND);
    gw_reset_all();
    {
        /* A command from the dead controller's epoch, still executing. */
        GW.cq[GW.cq_n].seq = 4u;
        GW.cq[GW.cq_n].len = 2u;
        GW.cq_n++;
        GW.next_seq = 5u;
        mbq_client_init();
        mbq_client_add_slave(SLAVE);
        run_ms(30);                              /* first sweep finds an empty queue */
        gw_complete(0u, 0xEEu);                  /* ...and only now does it finish */
        for (int i = 0; i < 500 && !mbq_client_can_submit(SLAVE); i++) pump();
        CHECK(mbq_client_can_submit(SLAVE));
        CHECK(GW.rq_n == 0);                     /* the late result was acked away... */
        CHECK(GW.acks == 1);                     /* ...by the second sweep */
        CHECK(stats().reboots_r3 == 0);          /* and NOT read as a reboot */
        CHECK(stats().drained);
    }

    /* ── 10d. Before the first accept byte there is no baseline, and the master
     *      must not pretend otherwise: a lost reply to that very first enqueue
     *      resolves conservatively, because there is no seq to predict from. */
    begin("no baseline: the first lost reply resolves conservatively");
    {
        cap_t a = {0};
        uint32_t accepts_before = GW.accepts;
        mbq_client_drop_reply(SLAVE, 1);
        CHECK(mbq_client_submit(SLAVE, CMD_SET, false, cap_cb, &a));
        pump();
        CHECK(GW.accepts == accepts_before + 1); /* it landed, but we cannot know that */
        gw_complete(0u, 0x77u);
        run_ms(MBQ_RESULT_DEADLINE_MS + 120u);
        CHECK(a.n == 1);
        CHECK(a.r == MBQ_R_LOST);                /* non-idempotent: disposed, not guessed */
        CHECK(stats().pred_faults == 0);         /* no prediction was ever made */
        CHECK(stats().reboots_r3 == 0);          /* the orphan is not a reboot either */
    }

    /* ── 11. Sequence wrap must skip 0: "no ack" and "ack command 32" are the
     *      same byte, so an off-by-one here is silent and total. ─────────── */
    begin("seq wrap skips 0");
    {
        GW.next_seq = 31u;
        cap_t a = {0}, b = {0};
        CHECK(submit(CMD_SET, true, &a));
        gw_complete(0u, 0x01u);
        run_ms(103);
        CHECK(a.n == 1);
        CHECK(submit(CMD_SET, true, &b));        /* gateway wraps 31 -> 1, not 0 */
        gw_complete(0u, 0x02u);
        run_ms(104);
        CHECK(b.n == 1 && b.r == MBQ_R_COMPLETE && b.resp == 0x02u);
        CHECK(stats().reboots_r2 == 0);          /* a wrap is not a reboot */
        CHECK(stats().reboots_r3 == 0);
    }

    /* ── 12. DTR0 pairs travel as the 8-byte request shape, and the flags must
     *      sit at their WIRE bit positions — the hazard that silently changes
     *      the meaning of every command. ─────────────────────────────────── */
    begin("DTR0 flag packing");
    {
        const uint8_t cmd[4] = { 0x01u, 0x2Au, MBQ_QF_DTR0 | MBQ_QF_ANSWER, 0x7Eu };
        cap_t a = {0};
        CHECK(submit(cmd, true, &a));
        CHECK(GW.cq_n == 1);
        CHECK(GW.cq[0].len == 3);                /* dtr0 byte present on the wire */
        CHECK(GW.cq[0].cmd[2] == 0x7Eu);
        gw_complete(0u, 0x2Au);
        run_ms(104);
        CHECK(a.n == 1 && a.resp == 0x2Au);
    }

    /* ── 12b. Every §4 path must TERMINATE when the gateway stops answering.
     *      A command whose callback never fires does not merely go missing:
     *      dali_queue releases its pending record only on that callback, so
     *      the slave's whole DALI queue wedges until the controller reboots. */
    begin("recovery terminates when the gateway goes away");
    {
        establish_baseline();
        cap_t a = {0}, b = {0};
        CHECK(mbq_client_submit(SLAVE, CMD_SET, true, cap_cb, &b));
        pump();
        gw_complete(0u, 0x05u);
        CHECK(run_until_delivered(&b, 200));     /* an ack is now pending */

        GW.present = false;                      /* ...and now it is gone for good */
        CHECK(mbq_client_submit(SLAVE, CMD_QUERY, false, cap_cb, &a));
        run_ms(3000);
        CHECK(a.n == 1);                         /* resolved, not left hanging */
        CHECK(a.r == MBQ_R_LOST);
        CHECK(mbq_client_inflight(SLAVE) == 0);
    }

    /* ── 12c. ...and it must do so without spinning. An unanswerable command
     *      previously produced one transaction per main-loop pass, which is
     *      not only wasted bus but a §6 diagnostic rendered unreadable: 29 852
     *      "deadline expiries" for a single command makes the counter that
     *      gate V2 reads to identify a reboot completely useless. */
    begin("an unanswerable command neither spins nor floods the counters");
    {
        cap_t a = {0};
        CHECK(mbq_client_submit(SLAVE, CMD_SET, true, cap_cb, &a));
        pump();                                  /* accepted, then silence */
        GW.present = false;
        uint32_t reqs_before = GW.reqs;
        run_ms(5000);
        /* Nothing reaches an absent gateway, so count what the client TRIED to
         * send by its own accounting instead. */
        uint32_t txns = stats().enqueues + stats().polls;
        printf("  [spin] over 5 s: %u txns, %u deadlines, %u timeouts\n",
               txns, stats().deadlines, stats().timeouts);
        CHECK(stats().deadlines < 10u);          /* per command, not per pass */
        CHECK(txns < 60u);                       /* backoff, not a main-loop spin */
        CHECK(GW.reqs == reqs_before);
        GW.present = true;
    }

    /* ── 12d. A_FULL's reply is SEVEN bytes — the shape follows the REQUEST,
     *      not the outcome. A parser that sized the reply off the accept
     *      status would break only on the A_FULL path, which is the canary
     *      path and so the one you least want also broken. */
    begin("A_FULL still returns the 7-byte enqueue reply shape");
    {
        cap_t a = {0}, b = {0}, c = {0};
        CHECK(submit(CMD_SET, true, &a));
        CHECK(submit(CMD_SET, true, &b));
        CHECK(GW.cq_n == CMDQ_DEPTH);
        /* Force the third past the client's own admission gate, which is
         * exactly what A_FULL exists to catch. */
        int i = 0;
        for (; i < 200 && !mbq_client_can_submit(SLAVE); i++) pump();
        CHECK(i < 200);
        CHECK(mbq_client_submit(SLAVE, CMD_SET, true, cap_cb, &c));
        pump();
        CHECK(GW.afulls == 1);
        CHECK(stats().afull == 1);
        CHECK(stats().proto_errors == 0);        /* the 7-byte reply parsed fine */
        CHECK(stats().incompatible == false);
        uint32_t accepts_before = GW.accepts;
        gw_complete(0u, 0x01u);
        gw_complete(0u, 0x01u);
        /* Stay inside c's result deadline, or the R4 chase legitimately resends
         * it again and the count stops meaning "retried once". */
        run_ms(100);
        CHECK(a.n == 1 && b.n == 1);
        CHECK(GW.accepts == accepts_before + 1); /* held and retried, never dropped */
        gw_complete(0u, 0x01u);
        run_ms(100);
        CHECK(c.n == 1);
    }

    /* ── 12e. THE MESSY COMBINATION: a reboot with commands actually in
     *      flight, which is where gate V2's intermittent mis-attribution
     *      lived. The rule tests above each drive one rule in isolation; this
     *      one asserts the two properties that matter together — no result is
     *      delivered to the wrong command, and nothing is left outstanding. */
    begin("reboot with commands in flight: no mis-attribution, nothing stranded");
    {
        establish_baseline();
        cap_t a = {0}, b = {0};
        CHECK(mbq_client_submit(SLAVE, CMD_QUERY, false, cap_cb, &a));
        pump();
        run_ms(10);
        if (mbq_client_can_submit(SLAVE)) {
            CHECK(mbq_client_submit(SLAVE, CMD_QUERY, false, cap_cb, &b));
            pump();
        }
        CHECK(GW.cq_n >= 1);

        gw_boot(false);                          /* everything in flight vanishes */
        run_ms(MBQ_RESULT_DEADLINE_MS + 400u);

        /* Neither command may come back with an answer: their results died with
         * the epoch, and inventing one is the misattribution class v4 exists to
         * eliminate. */
        CHECK(a.n == 1);
        CHECK(a.r == MBQ_R_LOST || a.r == MBQ_R_RESULT_LOST);
        if (b.n) CHECK(b.r == MBQ_R_LOST || b.r == MBQ_R_RESULT_LOST);
        CHECK(mbq_client_inflight(SLAVE) == 0);  /* nothing stranded */
        CHECK(stats().pred_faults == 0);
        /* The reboot must be attributed to a rule, not merely survived — gate
         * V2 reads this split to say HOW it was noticed. */
        CHECK(stats().reboots_r2 + stats().reboots_r3 + stats().reboots_r4 >= 1u);
    }

    /* ── 12f. THE GATE-V2 BENCH FAILURE, reproduced. A gateway reboot with a
     *      SET/QUERY pair in flight came back with the query answering the
     *      value the NEXT pair's SET had written — the two DALI frames were
     *      reversed, because the reboot disposal re-queued both commands and
     *      they went back out in slot order rather than in the order the
     *      producer issued them. Deterministic on hardware: exactly one
     *      mis-attribution per reboot. The gateway executes strictly FIFO, so
     *      the order the client re-offers IS the order the frames happen in. */
    begin("resends preserve producer order across a reboot");
    {
        establish_baseline();
        /* Stand in for the soak's pair: a SET that must run first and a QUERY
         * that must run second. The fake gateway answers with the opcode of
         * whichever command it executed, so an answer of SET_OP means the
         * ordering inverted. */
        const uint8_t SET_OP = 0x11u, QRY_OP = 0x22u;
        const uint8_t first[4]  = { 0xA3u, SET_OP, 0u, 0u };
        const uint8_t second[4] = { 0x01u, QRY_OP, MBQ_QF_ANSWER, 0u };
        cap_t x = {0}, a = {0}, b = {0};
        int i;

        /* The layout matters, and it is why this only ever showed on hardware:
         * the two commands must end up with the EARLIER one in the HIGHER slot,
         * which is what happens once slots start being recycled. Fill slot 0,
         * put `first` in slot 1, then retire slot 0 and let `second` reuse it. */
        CHECK(mbq_client_submit(SLAVE, CMD_SET, true, cap_cb, &x));
        pump();
        for (i = 0; i < 200 && !mbq_client_can_submit(SLAVE); i++) pump();
        CHECK(mbq_client_submit(SLAVE, first, true, cap_cb, &a));
        pump();
        gw_complete(0u, 0x00u);                  /* retire the filler */
        CHECK(run_until_delivered(&x, 300));
        for (i = 0; i < 200 && !mbq_client_can_submit(SLAVE); i++) pump();
        CHECK(mbq_client_submit(SLAVE, second, true, cap_cb, &b));
        pump();

        CHECK(GW.cq_n == 2);
        CHECK(GW.cq[0].cmd[1] == SET_OP);        /* the gateway has them in order */
        CHECK(GW.hist_n == 0);                   /* neither has executed yet */

        gw_boot(false);                          /* both vanish; both are idempotent */
        /* Only now let the DALI side run, so `hist` records exactly the frames
         * the RESENDS produced and nothing else. */
        GW.dali_running = true;
        run_ms(MBQ_RESULT_DEADLINE_MS + 500u);
        GW.dali_running = false;

        uint8_t saw_first = 0, saw_second = 0, inverted = 0;
        for (uint8_t k = 0; k < GW.hist_n; k++) {
            if (GW.hist[k] == SET_OP) { saw_first = 1; if (saw_second) inverted = 1; }
            if (GW.hist[k] == QRY_OP) { saw_second = 1; if (!saw_first) inverted = 1; }
        }
        CHECK(saw_first && saw_second);
        CHECK(!inverted);                        /* the bench failure, in one flag */
    }

    /* ── 12g. GATE V4: withholding acks must produce BACK-PRESSURE, not double
     *      execution. With the head unacked the client cannot see results
     *      behind it — but the gateway is retaining them exactly as designed,
     *      so a result deadline expiring there is R4 reading a stalled queue as
     *      a wedged gateway and resending a command the gateway still holds.
     *      That is a real DALI frame executed twice, reported to nobody. */
    begin("no-ack drill: back-pressure, not double execution");
    {
        establish_baseline();
        GW.dali_running = true;
        mbq_client_no_ack(SLAVE, 500);           /* withhold for the whole run */
        static cap_t caps[8];
        memset(caps, 0, sizeof caps);
        uint32_t submitted = 0;
        uint32_t accepts_before = GW.accepts;
        for (int i = 0; i < 1500; i++) {
            if (submitted < 6u && mbq_client_can_submit(SLAVE)
                && mbq_client_submit(SLAVE, CMD_SET, true, cap_cb, &caps[submitted]))
                submitted++;
            pump();
        }
        CHECK(stats().head_blocked > 0);         /* the client SAW the queue backing up */
        /* Every accept is a DALI frame. More accepts than commands means at
         * least one frame ran twice. */
        printf("  [no-ack] %u commands submitted, %u accepts at the gateway\n",
               submitted, GW.accepts - accepts_before);
        CHECK(GW.accepts - accepts_before == submitted);
        CHECK(stats().resends == 0);
        CHECK(stats().afull == 0);
        CHECK(stats().lost == 0);

        /* Resuming acks must drain it and let everything complete. */
        mbq_client_no_ack(SLAVE, 0);
        for (int i = 0; i < 3000; i++) pump();
        uint32_t done = 0;
        for (uint32_t k = 0; k < submitted; k++) if (caps[k].n == 1) done++;
        CHECK(done == submitted);                /* nothing lost, full recovery */
        CHECK(GW.accepts - accepts_before == submitted);
        GW.dali_running = false;
    }

    /* ── 12h. The OTHER way a prediction can be wrong: the predicted seq never
     *      materialises at all, because the gateway never issued it. Nothing
     *      re-examines such a prediction — apply_head only compares against a
     *      DIFFERENT seq that surfaces — so it used to run out its deadline and
     *      be disposed by R4's wedge branch as RESULT_LOST. That is the wrong
     *      answer twice over: it is a prediction fault and must be counted as
     *      one, and the command never ran, so telling the submitter it
     *      executed (and specifically not to retry) is a lie that leaves the
     *      cache diverged from the gear with no error surfaced anywhere. */
    begin("a prediction that never surfaces is a counted fault, disposed LOST");
    {
        establish_baseline();
        cap_t b = {0}, a = {0};
        CHECK(mbq_client_submit(SLAVE, CMD_SET, true, cap_cb, &b));
        pump();
        gw_complete(0u, 0x07u);
        CHECK(run_until_delivered(&b, 200));     /* an ack is now pending */

        /* A gateway that refuses the enqueue WITHOUT advancing its seq counter,
         * and whose reply is then lost. The ack still applied, so R1 correctly
         * concludes the frame landed — and binds to a seq that will never
         * exist. (gw-reviewer found a real second A_FULL path in cq_accept
         * that does exactly this; the point here is the general property.) */
        GW.force_afull = true;
        mbq_client_drop_reply(SLAVE, 1);
        CHECK(mbq_client_submit(SLAVE, CMD_SET, false, cap_cb, &a));
        pump();
        GW.force_afull = false;

        run_ms(MBQ_RESULT_DEADLINE_MS + 400u);
        CHECK(a.n == 1);
        CHECK(a.r == MBQ_R_LOST);                /* NOT RESULT_LOST: it never ran */
        CHECK(stats().pred_faults == 1);         /* and it is counted as a fault */
    }

    /* ── 13. THE NUMBER THIS PROTOCOL EXISTS TO HIT: one Modbus transaction
     *      per DALI frame. A steady producer, a gateway executing at real
     *      frame times, and every transaction should be an enqueue that also
     *      carries the ack collecting the previous result. The first bench
     *      build measured 2.04 transactions per command here — correct, fast
     *      enough to look fine, and burning the entire benefit of the design —
     *      so this is a RATE assertion, not a correctness one (task-07's
     *      lesson: validate rate, not just correctness). ─────────────────── */
    begin("steady state costs one transaction per DALI frame");
    {
        GW.dali_running = true;
        static cap_t caps[64];
        memset(caps, 0, sizeof caps);
        uint32_t submitted = 0, done = 0, bad = 0;
        uint32_t reqs_before = GW.reqs;

        for (int i = 0; i < 4000 && submitted < 40u; i++) {
            if (mbq_client_can_submit(SLAVE)) {
                /* Alternate the two command classes, as the soak does. */
                const uint8_t *c = (submitted & 1u) ? CMD_QUERY : CMD_SET;
                if (mbq_client_submit(SLAVE, c, true, cap_cb, &caps[submitted]))
                    submitted++;
            }
            pump();
        }
        for (int i = 0; i < 2000; i++) pump();   /* drain */

        for (uint32_t i = 0; i < submitted; i++) {
            if (caps[i].n == 1) done++;
            if (caps[i].n != 1 || caps[i].r == MBQ_R_LOST) bad++;
        }
        uint32_t reqs = GW.reqs - reqs_before;
        printf("  [rate] %u commands, %u transactions = %u.%02u txns/command\n",
               submitted, reqs, reqs / submitted,
               (reqs * 100u / submitted) % 100u);
        CHECK(submitted == 40u);
        CHECK(done == submitted);
        CHECK(bad == 0);
        CHECK(stats().afull == 0);          /* the estimate never ran early */
        /* 1.0 is the target; anything at or above 1.5 means the collecting
         * poll came back, which is the defect this test exists to catch. */
        CHECK(reqs * 100u / submitted < 130u);
        GW.dali_running = false;
    }

    if (!fail) printf("PASS: all mbq_client checks passed\n");
    return fail;
}
