#pragma once
/*
 * dali_pending.h — latest-wins desired state for one DALI gateway (task 14).
 *
 * A burst of absolute commands (Siri "turn off all the lights" is ~65 of them in
 * a few ms) used to be dispatched one queued frame each into the scheduler's
 * 8-deep user class, and everything past the eighth was dropped with a WARN —
 * measured 2026-10-10 on the house: 57 of 75 DALI commands lost, nothing told
 * HA. Every such command is ABSOLUTE (off / on / level for one ballast or one
 * group), so only the newest per target matters. This table holds exactly that:
 * one slot per short address (64) and per group (16), overwritten by each new
 * command, drained by the scheduler as capacity frees up. It cannot overflow.
 *
 * ORDERING — why the oldest-written slot ships first, and why that is all the
 * group-vs-member handling there is. Each write stamps its slot with a
 * monotonically increasing sequence; the drain always takes the dirty slot with
 * the SMALLEST stamp. A ballast's final level is set by whichever of its own
 * slot and its groups' slots executes last, and in stamp order that is the one
 * written most recently — exactly what a FIFO that never overflowed would have
 * produced, minus the superseded duplicates. "group 3.15 OFF, then 3.4 ON" ships
 * in that order and 3.4 stays on; writing "group OFF" again afterwards moves the
 * group's slot to newest and everything ends off. No membership lookup is
 * involved, so a stale membership cache cannot make it wrong.
 *
 * Pure data structure: no SDK, no I/O, host-testable (test/dali_pending_test.c).
 * dali_queue.c owns one per gateway and drains it as a rung of its priority
 * ladder; lighting_actions.c is the only writer.
 */

#include <stdbool.h>
#include <stdint.h>

#define DP_BALLASTS  64u
#define DP_GROUPS    16u
#define DP_SLOTS     (DP_BALLASTS + DP_GROUPS)   /* index: ballast a = a, group g = 64+g */

typedef enum {
    DP_OFF   = 0,   /* ballast: DAPC 0;  group: OFF command          */
    DP_ON    = 1,   /* GOTO LAST ACTIVE LEVEL — restores, sets no level */
    DP_LEVEL = 2,   /* DAPC `arc`                                    */
} dp_op_t;

typedef struct {
    uint32_t stamp;   /* write order; meaningful only while dirty */
    uint8_t  op;      /* dp_op_t */
    uint8_t  arc;     /* DP_LEVEL only */
    bool     dirty;
} dp_slot_t;

typedef struct {
    dp_slot_t slot[DP_SLOTS];
    uint32_t  next_stamp;
    uint8_t   n_dirty;
    /* counters for `dq` */
    uint32_t  c_written;      /* commands written into the table */
    uint32_t  c_overwritten;  /* writes that replaced a not-yet-shipped one */
    uint32_t  c_shipped;      /* slots drained to the scheduler */
    uint32_t  c_button;       /* slots a button cleared or re-asserted */
} dp_table_t;

static inline uint8_t dp_index(bool is_group, uint8_t target) {
    return is_group ? (uint8_t)(DP_BALLASTS + (target & 0x0Fu)) : (uint8_t)(target & 0x3Fu);
}

void dp_init(dp_table_t *t);

/* Record the newest desired state for a target. Always succeeds. */
void dp_write(dp_table_t *t, bool is_group, uint8_t target, dp_op_t op, uint8_t arc);

/* Forget a target's pending state (its own slot only). */
void dp_clear(dp_table_t *t, bool is_group, uint8_t target);

/* A BUTTON has just commanded `target` directly, ahead of this table. Buttons
 * always win, so whatever was pending for the target is obsolete — but the
 * button skipped the queue, and an OLDER pending slot that is still to ship (a
 * group the ballast is in, or a member of the group the button drove) would
 * otherwise execute after it and undo the press. So: if nothing else is pending
 * on this gateway the target's slot is simply cleared; if anything else is, the
 * button's own state is re-written as the NEWEST slot, so it is re-asserted after
 * everything older. Returns true if it re-asserted (one extra, idempotent frame —
 * only ever during a drain). */
bool dp_button(dp_table_t *t, bool is_group, uint8_t target, dp_op_t op, uint8_t arc);

/* The slot to ship next: the dirty one with the oldest stamp. False if none. */
bool dp_next(const dp_table_t *t, uint8_t *idx);

/* Mark slot `idx` shipped (clean). */
void dp_take(dp_table_t *t, uint8_t idx);

/* The DALI forward frame for slot `idx` in state `s`: address byte + data byte,
 * using the same encodings the direct paths always have (ballast off = DAPC 0,
 * group off = the OFF command, on = GOTO LAST ACTIVE LEVEL, level = DAPC). */
void dp_frame(uint8_t idx, const dp_slot_t *s, uint8_t *addr, uint8_t *data);
