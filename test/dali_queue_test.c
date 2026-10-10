/*
 * dali_queue_test.c — host test for the scheduler side of task 14: how the
 * pending desired-state table drains through src/dali_queue.c's priority ladder.
 *
 * Not part of the firmware build. Compiles the real dali_queue.c and
 * dali_pending.c; the client below it (mbq_client), the transport kicks, config,
 * logging and the watchdog are faked here as plain extern definitions — the
 * same approach as mbq_client_test.c. The fake client records every command in
 * wire order and completes them only when a test says so, so ordering is exact.
 * test/stubs supplies pico/time.h (dq_submit_sync's deadline loop, never called).
 *
 * Covers what only the ladder can show: a button press arriving mid-drain ships
 * NEXT, ahead of everything still pending; the table never takes the staging
 * slot; every slot ships exactly once in write order; the verification hook
 * fires per ship; a dim step drops its target's pending slot.
 *
 * Build & run:
 *   cc -Wall -Wextra -I stubs -I ../src -o /tmp/dali_queue_test \
 *       dali_queue_test.c ../src/dali_queue.c ../src/dali_pending.c \
 *       && /tmp/dali_queue_test
 */
#include "dali_queue.h"
#include "mbq_client.h"
#include "config.h"
#include "log.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static int s_fail;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); s_fail++; } } while (0)

/* ── fakes ───────────────────────────────────────────────────────────────── */
global_config_t g_config;
log_level_t g_log_level = LOG_LEVEL_OFF;
void log_emit(log_level_t lvl, const char *lc, const char *m, const char *fmt, ...) {
    (void)lvl; (void)lc; (void)m; (void)fmt;
}
void cdc_out_printf(const char *fmt, ...) { (void)fmt; }
void wd_feed(void) {}
void modbus_poll(void) {}
void modbus_backoff_kick(uint8_t slave) { (void)slave; }
bool dali_provision_busy_slave(uint8_t slave_id) { (void)slave_id; return false; }

typedef struct { uint8_t addr, data, flags; mbq_cb_t cb; void *ctx; bool done; } wire_t;
static wire_t s_wire[512];
static int    s_nwire, s_ndone;

void mbq_client_init(void) { s_nwire = s_ndone = 0; }
void mbq_client_add_slave(uint8_t slave_id) { (void)slave_id; }
bool mbq_client_can_submit(uint8_t slave_id) { (void)slave_id; return true; }
void mbq_client_kick(uint8_t slave_id) { (void)slave_id; }
void mbq_client_collect(void) {}
void mbq_client_debug_print(int slave_id) { (void)slave_id; }
bool mbq_client_submit(uint8_t slave_id, const uint8_t cmd[MBQ_CMD_BYTES],
                       bool idempotent, mbq_cb_t cb, void *ctx) {
    (void)slave_id; (void)idempotent;
    s_wire[s_nwire++] = (wire_t){ .addr = cmd[0], .data = cmd[1], .flags = cmd[2],
                                  .cb = cb, .ctx = ctx };
    return true;
}

/* The gateway finishes its oldest command. */
static void complete_one(void) {
    wire_t *w = &s_wire[s_ndone++];
    w->done = true;
    w->cb(3, MBQ_R_COMPLETE, 0, w->ctx);
}

static int s_hooks; static uint8_t s_hook_tgt[256]; static dp_kind_t s_hook_kind[256];
static void hook(uint8_t slave, dp_kind_t kind, uint8_t tgt) {
    (void)slave; s_hook_kind[s_hooks] = kind; s_hook_tgt[s_hooks++] = tgt;
}

static void setup(void) {
    memset(&g_config, 0, sizeof g_config);
    g_config.dali_slave_count = 1;
    g_config.dali_slave_ids[0] = 3;
    dq_init();
    s_hooks = 0;
    dq_set_ship_hook(hook);
}

static int in_flight(void) { return s_nwire - s_ndone; }

/* Ballast a, DAPC form = a<<1. */
#define DAPC(a) ((uint8_t)((a) << 1))

static void test_drain_all_in_order(void) {
    setup();
    for (uint8_t a = 0; a < 40; a++) CHECK(dq_set_desired(3, false, a, DP_OFF, 0));
    CHECK(dq_set_desired(3, false, 10, DP_LEVEL, 77));   /* overwrite: 10 moves last */
    CHECK(dq_pending_count(3) == 40);
    for (int guard = 0; guard < 1000 && (dq_pending_count(3) || in_flight()); guard++) {
        dq_tick();
        CHECK(in_flight() <= (int)CMDQ_DEPTH);   /* never the staging slot */
        if (in_flight()) complete_one();
    }
    CHECK(s_nwire == 40);                         /* each target exactly once */
    int k = 0;
    for (uint8_t a = 0; a < 40; a++) if (a != 10) { CHECK(s_wire[k].addr == DAPC(a) && s_wire[k].data == 0); k++; }
    CHECK(s_wire[39].addr == DAPC(10) && s_wire[39].data == 77);
    CHECK(s_hooks == 40 && s_hook_kind[0] == DP_K_BALLAST && s_hook_tgt[39] == 10);
}

static void test_button_jumps_the_drain(void) {
    setup();
    for (uint8_t a = 0; a < 40; a++) dq_set_desired(3, false, a, DP_OFF, 0);
    for (int i = 0; i < 5; i++) dq_tick();
    CHECK(in_flight() == (int)CMDQ_DEPTH);        /* table filled the gateway, no more */
    int before = s_nwire;

    /* Wall press on 3.50 (ON = GOTO LAST ACTIVE, command form). */
    CHECK(dq_submit(3, DQ_PRIO_USER, (uint8_t)(DAPC(50) | 1u), 0x0A, 0, 0, true, NULL, NULL));
    dq_tick();
    CHECK(s_nwire == before + 1);                 /* shipped THIS tick... */
    CHECK(s_wire[before].addr == (DAPC(50) | 1u)); /* ...into the staging slot */
    /* So it waits only for the CMDQ_DEPTH frames already at the gateway. */
    CHECK(s_nwire - 1 - s_ndone == (int)CMDQ_DEPTH);

    /* The drain carries on behind it. */
    for (int guard = 0; guard < 1000 && (dq_pending_count(3) || in_flight()); guard++) {
        dq_tick(); if (in_flight()) complete_one();
    }
    CHECK(s_nwire == 41);
}

static void test_button_settle_reasserts(void) {
    setup();
    dq_set_desired(3, true, 15, DP_OFF, 0);       /* Siri all-off via group 15 */
    dq_set_desired(3, false, 4, DP_OFF, 0);
    dq_set_desired(3, false, 5, DP_OFF, 0);
    for (int i = 0; i < 3; i++) dq_tick();        /* group + 3.4 shipped; 3.5 waits */
    CHECK(dq_pending_count(3) == 1);
    /* Press for 3.4 lands now: ships directly, and since 3.5 is still pending
     * the press is re-asserted behind it. */
    dq_submit(3, DQ_PRIO_USER, (uint8_t)(DAPC(4) | 1u), 0x0A, 0, 0, true, NULL, NULL);
    CHECK(dq_button_settle(3, false, 4, DP_ON, 0) == true);
    for (int guard = 0; guard < 100 && (dq_pending_count(3) || in_flight()); guard++) {
        dq_tick(); if (in_flight()) complete_one();
    }
    CHECK(s_nwire == 5);
    CHECK(s_wire[4].addr == (DAPC(4) | 1u) && s_wire[4].data == 0x0A);   /* last word: ON */

    /* Table idle: a press only clears, it adds no frame. */
    setup();
    dq_set_desired(3, false, 4, DP_OFF, 0);
    CHECK(dq_button_settle(3, false, 4, DP_ON, 0) == false);
    CHECK(dq_pending_count(3) == 0);
}

static void test_dim_clears_pending(void) {
    setup();
    dq_set_desired(3, false, 8, DP_OFF, 0);
    dq_set_desired(3, true, 2, DP_OFF, 0);
    CHECK(dq_add_delta(3, false, 8, +1));
    CHECK(dq_pending_count(3) == 1);              /* 3.8's OFF dropped; group stays */
    CHECK(dq_add_delta(3, true, 2, -1));
    CHECK(dq_pending_count(3) == 0);
}

/* A big membership replace no longer blocks: it returns at once, ships send-
 * twice frames in the table's order, below a button, and the hook asks for a
 * re-interrogation of each touched ballast. */
static void test_members_nonblocking(void) {
    setup();
    uint64_t add = 0;
    for (int a = 0; a < 36; a++) add |= 1ull << a;
    CHECK(dq_set_members(3, 15, add, 0));            /* returns immediately */
    CHECK(dq_set_desired(3, true, 15, DP_ON, 0));    /* then "group 15 on" */
    for (int i = 0; i < 5; i++) dq_tick();
    CHECK(in_flight() == (int)CMDQ_DEPTH);
    int before = s_nwire;
    dq_submit(3, DQ_PRIO_USER, 0x7F, 0x00, 0, 0, true, NULL, NULL);   /* a press */
    dq_tick();
    CHECK(s_wire[before].addr == 0x7F);              /* the press went next */
    for (int guard = 0; guard < 1000 && (dq_pending_count(3) || in_flight()); guard++) {
        dq_tick(); if (in_flight()) complete_one();
    }
    CHECK(s_nwire == 36 + 1 + 1);
    int adds = 0;
    for (int i = 0; i < s_nwire; i++)
        if (s_wire[i].data == 0x6F) { adds++; CHECK(s_wire[i].flags == DQ_TWICE); }
    CHECK(adds == 36);
    CHECK(s_wire[s_nwire - 1].addr == 0x9F && s_wire[s_nwire - 1].data == 0x0A);  /* ON last */
    CHECK(s_hook_kind[0] == DP_K_MEMBER && s_hook_tgt[0] == 0);
    CHECK(s_hook_kind[s_hooks - 1] == DP_K_GROUP && s_hook_tgt[s_hooks - 1] == 15);

    uint64_t m;
    CHECK(!dq_member_intent(3, 15, &m));
    dq_note_member_intent(3, 15, add);
    CHECK(dq_member_intent(3, 15, &m) && m == add);
}

/* dq_submit_sync's blocking wait runs the wait hook on every spin (the seam
 * hangs the HA-command absorber on it). */
static int s_wait_calls;
static void wait_hook(void) {
    s_wait_calls++;
    if (s_wait_calls == 3) complete_one();   /* the gateway answers mid-wait */
}

static void test_wait_hook(void) {
    setup();
    s_wait_calls = 0;
    dq_set_wait_hook(wait_hook);
    dq_result_t r; uint8_t ans;
    CHECK(dq_submit_sync(3, 0x01, 0x90, DQ_ANSWER, 0, true, 1000, &r, &ans));
    CHECK(s_wait_calls >= 3 && r == DQ_RES_OK);
    dq_set_wait_hook(NULL);
}

static void test_unknown_slave(void) {
    setup();
    CHECK(!dq_set_desired(9, false, 1, DP_ON, 0));
    CHECK(dq_pending_count(9) == 0);
}

int main(void) {
    test_drain_all_in_order();
    test_button_jumps_the_drain();
    test_button_settle_reasserts();
    test_dim_clears_pending();
    test_members_nonblocking();
    test_wait_hook();
    test_unknown_slave();
    if (s_fail) { printf("dali_queue_test: %d FAILED\n", s_fail); return 1; }
    printf("dali_queue_test: all passed\n");
    return 0;
}
