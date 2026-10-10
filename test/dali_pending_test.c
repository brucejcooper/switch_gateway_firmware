/*
 * dali_pending_test.c — host test for src/dali_pending.c, the latest-wins
 * desired-state table (task 14).
 *
 * Not part of the firmware build. The module is pure, so this links it as-is.
 *
 * Covers: overwrite (only the newest command per target survives), drain order
 * (oldest-written first), group-vs-member handling, the button rule (clear vs
 * re-assert), and the frame encodings. The group-vs-member test does not check
 * the drain order against a hand-written expectation; it replays random
 * command streams through a model DALI bus with real group membership and
 * compares the final levels against "the last command that addressed each
 * ballast wins" — which is exactly what a FIFO that never overflowed would do.
 *
 * Build & run:
 *   cc -Wall -Wextra -I ../src -o /tmp/dali_pending_test \
 *       dali_pending_test.c ../src/dali_pending.c && /tmp/dali_pending_test
 */
#include "dali_pending.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int s_fail;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); s_fail++; } } while (0)

/* Drain the whole table, recording ship order as slot indices. */
static int drain(dp_table_t *t, uint8_t *order, dp_slot_t *state) {
    int n = 0; uint8_t idx;
    while (dp_next(t, &idx)) {
        if (state) state[n] = t->slot[idx];
        order[n++] = idx;
        dp_take(t, idx);
    }
    return n;
}

static void test_overwrite(void) {
    dp_table_t t; dp_init(&t);
    dp_write(&t, false, 5, DP_ON, 0);
    dp_write(&t, false, 5, DP_LEVEL, 100);
    dp_write(&t, false, 5, DP_OFF, 0);
    CHECK(t.n_dirty == 1);
    CHECK(t.c_written == 3 && t.c_overwritten == 2);
    uint8_t order[DP_SLOTS]; dp_slot_t st[DP_SLOTS];
    CHECK(drain(&t, order, st) == 1);
    CHECK(order[0] == 5 && st[0].op == DP_OFF);
    CHECK(t.n_dirty == 0 && t.c_shipped == 1);

    /* 200 writes over 80 targets can never need more than 80 slots. */
    dp_init(&t);
    for (int i = 0; i < 200; i++) dp_write(&t, i % 3 == 0, (uint8_t)(i % 64), DP_LEVEL, (uint8_t)i);
    /* i%3==0 → group (i%64)&15; distinct targets: 64 ballasts + 16 groups. */
    CHECK(t.n_dirty == DP_SLOTS);
    CHECK(t.c_written == 200 && t.c_overwritten == 200 - DP_SLOTS);
    CHECK(drain(&t, order, NULL) == DP_SLOTS);
    CHECK(t.n_dirty == 0);
}

static void test_drain_order(void) {
    dp_table_t t; dp_init(&t);
    dp_write(&t, false, 9, DP_OFF, 0);     /* stamp 1 */
    dp_write(&t, true,  3, DP_OFF, 0);     /* stamp 2 */
    dp_write(&t, false, 1, DP_OFF, 0);     /* stamp 3 */
    dp_write(&t, false, 9, DP_ON, 0);      /* 9 re-written: now newest */
    uint8_t order[DP_SLOTS];
    CHECK(drain(&t, order, NULL) == 3);
    CHECK(order[0] == dp_index(true, 3));
    CHECK(order[1] == 1);
    CHECK(order[2] == 9);
}

/* ── model bus: 64 ballasts, 16 groups with fixed membership ──────────────── */
typedef struct { uint8_t level; uint8_t last_active; } ballast_t;
static uint64_t s_member[DP_GROUPS];

static void apply_to(ballast_t *b, uint8_t op, uint8_t arc) {
    if (op == DP_OFF) b->level = 0;
    else if (op == DP_ON) b->level = b->last_active;
    else { b->level = arc; if (arc) b->last_active = arc; }
}

static void bus_apply(ballast_t *bus, uint8_t idx, const dp_slot_t *s) {
    if (idx < DP_BALLASTS) { apply_to(&bus[idx], s->op, s->arc); return; }
    uint64_t m = s_member[idx - DP_BALLASTS];
    for (int a = 0; a < 64; a++) if (m >> a & 1) apply_to(&bus[a], s->op, s->arc);
}

static void test_group_vs_member(void) {
    srand(14);
    for (int round = 0; round < 2000; round++) {
        for (int g = 0; g < 16; g++) {
            s_member[g] = 0;
            for (int a = 0; a < 64; a++) if (rand() % 6 == 0) s_member[g] |= (uint64_t)1 << a;
        }
        /* Two buses from the same start. The oracle executes EVERY command in
         * arrival order (an infinitely deep FIFO); the table coalesces. */
        ballast_t oracle[64], coal[64];
        for (int a = 0; a < 64; a++) {
            oracle[a].level = coal[a].level = (uint8_t)(rand() % 2 ? 200 : 0);
            oracle[a].last_active = coal[a].last_active = 200;
        }
        dp_table_t t; dp_init(&t);
        int n = 20 + rand() % 150;
        for (int i = 0; i < n; i++) {
            bool grp = rand() % 3 == 0;
            uint8_t tgt = (uint8_t)(grp ? rand() % 16 : rand() % 64);
            /* ON/OFF only: GOTO LAST ACTIVE depends on history a coalesced
             * stream deliberately skips, so levels are tested separately. */
            dp_op_t op = rand() % 2 ? DP_ON : DP_OFF;
            dp_slot_t s = { .op = (uint8_t)op };
            bus_apply(oracle, dp_index(grp, tgt), &s);
            dp_write(&t, grp, tgt, op, 0);
        }
        uint8_t order[DP_SLOTS]; dp_slot_t st[DP_SLOTS];
        int k = drain(&t, order, st);
        for (int i = 0; i < k; i++) bus_apply(coal, order[i], &st[i]);
        int bad = 0;
        for (int a = 0; a < 64; a++) if (oracle[a].level != coal[a].level) bad++;
        if (bad) { CHECK(bad == 0); printf("  round %d: %d ballasts differ\n", round, bad); return; }
    }

    /* Absolute levels, the same property. */
    srand(1414);
    for (int round = 0; round < 2000; round++) {
        for (int g = 0; g < 16; g++) {
            s_member[g] = 0;
            for (int a = 0; a < 64; a++) if (rand() % 6 == 0) s_member[g] |= (uint64_t)1 << a;
        }
        ballast_t oracle[64] = {0}, coal[64] = {0};
        dp_table_t t; dp_init(&t);
        int n = 20 + rand() % 150;
        for (int i = 0; i < n; i++) {
            bool grp = rand() % 3 == 0;
            uint8_t tgt = (uint8_t)(grp ? rand() % 16 : rand() % 64);
            uint8_t arc = (uint8_t)(rand() % 255);
            dp_slot_t s = { .op = DP_LEVEL, .arc = arc };
            bus_apply(oracle, dp_index(grp, tgt), &s);
            dp_write(&t, grp, tgt, DP_LEVEL, arc);
        }
        uint8_t order[DP_SLOTS]; dp_slot_t st[DP_SLOTS];
        int k = drain(&t, order, st);
        for (int i = 0; i < k; i++) bus_apply(coal, order[i], &st[i]);
        for (int a = 0; a < 64; a++) if (oracle[a].level != coal[a].level) {
            CHECK(oracle[a].level == coal[a].level); return; }
    }

    /* The named cases from the design. */
    dp_table_t t; dp_init(&t);
    dp_write(&t, true, 15, DP_OFF, 0);
    dp_write(&t, false, 4, DP_ON, 0);
    uint8_t order[DP_SLOTS];
    CHECK(drain(&t, order, NULL) == 2);
    CHECK(order[0] == dp_index(true, 15) && order[1] == 4);   /* 3.4 stays on */

    dp_init(&t);
    dp_write(&t, true, 15, DP_OFF, 0);
    dp_write(&t, false, 4, DP_ON, 0);
    dp_write(&t, true, 15, DP_OFF, 0);                         /* group again: newest */
    CHECK(drain(&t, order, NULL) == 2);
    CHECK(order[0] == 4 && order[1] == dp_index(true, 15));   /* all end off */
}

static void test_button(void) {
    dp_table_t t; dp_init(&t);
    /* Nothing else pending: the button's own stale slot is just dropped. */
    dp_write(&t, false, 7, DP_OFF, 0);
    CHECK(dp_button(&t, false, 7, DP_ON, 0) == false);
    CHECK(t.n_dirty == 0);
    /* Nothing pending at all: a no-op. */
    CHECK(dp_button(&t, false, 7, DP_ON, 0) == false);
    CHECK(t.n_dirty == 0 && t.c_button == 1);

    /* Siri "all off" is draining (group OFF pending) and someone presses the
     * switch for 3.4, a member: the press must be re-asserted AFTER the group. */
    dp_init(&t);
    dp_write(&t, true, 15, DP_OFF, 0);
    dp_write(&t, false, 4, DP_OFF, 0);
    CHECK(dp_button(&t, false, 4, DP_ON, 0) == true);
    uint8_t order[DP_SLOTS]; dp_slot_t st[DP_SLOTS];
    CHECK(drain(&t, order, st) == 2);
    CHECK(order[0] == dp_index(true, 15));
    CHECK(order[1] == 4 && st[1].op == DP_ON);
    CHECK(t.c_written == 2);   /* the re-assert is the button's, not a new command */

    /* A press on a group while member slots are pending: the group is newest. */
    dp_init(&t);
    dp_write(&t, false, 4, DP_ON, 0);
    dp_write(&t, false, 5, DP_ON, 0);
    CHECK(dp_button(&t, true, 15, DP_OFF, 0) == true);
    CHECK(drain(&t, order, st) == 3);
    CHECK(order[2] == dp_index(true, 15) && st[2].op == DP_OFF);
}

/* Membership edits: latest wins per (ballast, group); the list keeps the stamp
 * it got when it became non-empty, so it drains before a LATER group command. */
static void test_members(void) {
    dp_table_t t; dp_init(&t);
    dp_members(&t, 5, (1ull << 2) | (1ull << 9), 1ull << 4);   /* +2 +9 -4 */
    dp_members(&t, 5, 0, 1ull << 9);                          /* -9 cancels +9 */
    CHECK(t.n_dirty == 1);
    dp_write(&t, true, 5, DP_ON, 0);                           /* then group ON */
    dp_members(&t, 5, 1ull << 7, 0);                           /* rides the early stamp */
    CHECK(t.n_dirty == 2);

    uint8_t idx, a, d, b, n = 0;
    struct { uint8_t a, d; } out[8];
    while (dp_next(&t, &idx)) {
        if (idx >= DP_MEMBER_BASE) {
            CHECK(idx == DP_MEMBER_BASE + 5);
            dp_member_frame(&t, idx, &a, &d, &b);
        } else {
            dp_frame(idx, &t.slot[idx], &a, &d);
        }
        out[n].a = a; out[n].d = d; n++;
        dp_take(&t, idx);
    }
    CHECK(n == 5);
    CHECK(out[0].a == 0x05 && out[0].d == 0x65);   /* ADD 2 to G5      */
    CHECK(out[1].a == 0x09 && out[1].d == 0x75);   /* REMOVE 4         */
    CHECK(out[2].a == 0x0F && out[2].d == 0x65);   /* ADD 7 (rode along) */
    CHECK(out[3].a == 0x13 && out[3].d == 0x75);   /* REMOVE 9 (latest) */
    CHECK(out[4].a == 0x8B && out[4].d == 0x0A);   /* then G5 ON, last */
    CHECK(t.n_dirty == 0);

    /* Intent bookkeeping. */
    uint64_t m;
    CHECK(!dp_intent(&t, 3, &m));
    dp_note_intent(&t, 3, 0x30u);
    CHECK(dp_intent(&t, 3, &m) && m == 0x30u);
    CHECK(!dp_intent(&t, 4, &m));

    /* A button while a membership list drains counts it as "something older
     * pending", so the press is re-asserted after it. */
    dp_init(&t);
    dp_members(&t, 1, 1ull << 3, 0);
    CHECK(dp_button(&t, false, 3, DP_OFF, 0) == true);
}

static void test_frames(void) {
    uint8_t a, d;
    dp_slot_t off = { .op = DP_OFF }, on = { .op = DP_ON }, lvl = { .op = DP_LEVEL, .arc = 200 };
    dp_frame(5, &off, &a, &d);  CHECK(a == 0x0A && d == 0x00);   /* DAPC 0 */
    dp_frame(5, &on,  &a, &d);  CHECK(a == 0x0B && d == 0x0A);   /* GOTO LAST ACTIVE */
    dp_frame(5, &lvl, &a, &d);  CHECK(a == 0x0A && d == 200);    /* DAPC 200 */
    dp_frame(63, &lvl, &a, &d); CHECK(a == 0x7E && d == 200);
    uint8_t g3 = dp_index(true, 3);
    dp_frame(g3, &off, &a, &d); CHECK(a == 0x87 && d == 0x00);   /* group OFF command */
    dp_frame(g3, &on,  &a, &d); CHECK(a == 0x87 && d == 0x0A);
    dp_frame(g3, &lvl, &a, &d); CHECK(a == 0x86 && d == 200);    /* group DAPC */
    dp_frame(dp_index(true, 15), &off, &a, &d); CHECK(a == 0x9F);
}

int main(void) {
    test_overwrite();
    test_drain_order();
    test_group_vs_member();
    test_button();
    test_members();
    test_frames();
    if (s_fail) { printf("dali_pending_test: %d FAILED\n", s_fail); return 1; }
    printf("dali_pending_test: all passed\n");
    return 0;
}
