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
/* Membership edits occupy a further 16 indices in the same stamp order:
 * group g's pending ADD/REMOVE TO GROUP frames = DP_SLOTS + g. */
#define DP_MEMBER_BASE DP_SLOTS
#define DP_ALL       (DP_SLOTS + DP_GROUPS)

/* What a shipped index addressed — for the verification hook. */
typedef enum {
    DP_K_BALLAST = 0,   /* target = short address                       */
    DP_K_GROUP   = 1,   /* target = group number                        */
    DP_K_MEMBER  = 2,   /* target = the short address whose groups changed */
} dp_kind_t;

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

    /* Group membership edits (ADD / REMOVE TO GROUP), one pending list per
     * group. Latest wins per (ballast, group) pair: an add cancels a pending
     * remove of the same pair and vice versa. A group's list takes ONE stamp,
     * when it goes from empty to non-empty, and keeps it while it drains — so
     * every edit requested before a later command on the group (e.g. "set
     * members, then group ON") has gone out before that command does. Edits
     * added while it drains ride along early, which can only make such a
     * command see membership NEWER than at its request, never older. */
    uint64_t  m_add[DP_GROUPS];
    uint64_t  m_rem[DP_GROUPS];
    uint32_t  m_stamp[DP_GROUPS];
    /* The membership last REQUESTED for each group, valid per bit of
     * m_intent_valid. Edits are diffed against it as well as against the
     * measured cache, because the cache lags: a frame can have shipped but not
     * yet been read back, and a diff against the cache alone would miss
     * undoing it. */
    uint64_t  m_intent[DP_GROUPS];
    uint16_t  m_intent_valid;
    uint32_t  c_member_ops;   /* membership frames requested */
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

/* Queue membership changes for `group`: ballasts in `add` join it, ballasts in
 * `remove` leave it (the masks must be disjoint). Never blocks, never fails. */
void dp_members(dp_table_t *t, uint8_t group, uint64_t add, uint64_t remove);

/* Record / fetch the membership last requested for `group` (see m_intent). */
void dp_note_intent(dp_table_t *t, uint8_t group, uint64_t members);
bool dp_intent(const dp_table_t *t, uint8_t group, uint64_t *members);

/* The index to ship next: the dirty one with the oldest stamp, over level
 * slots and membership lists alike. False if none. */
bool dp_next(const dp_table_t *t, uint8_t *idx);

/* Mark index `idx` shipped. A level slot goes clean; a membership list loses
 * the one frame dp_member_frame() built (its lowest pending ballast) and stays
 * dirty, at its stamp, while any remain. */
void dp_take(dp_table_t *t, uint8_t idx);

/* The next frame of membership list `idx` (>= DP_MEMBER_BASE): ADD TO GROUP
 * (0x60|g) or REMOVE FROM GROUP (0x70|g) to the lowest pending ballast, command
 * form; these are SET-class, so the caller sends them twice. */
void dp_member_frame(const dp_table_t *t, uint8_t idx, uint8_t *addr, uint8_t *data,
                     uint8_t *ballast);

/* The DALI forward frame for slot `idx` in state `s`: address byte + data byte,
 * using the same encodings the direct paths always have (ballast off = DAPC 0,
 * group off = the OFF command, on = GOTO LAST ACTIVE LEVEL, level = DAPC). */
void dp_frame(uint8_t idx, const dp_slot_t *s, uint8_t *addr, uint8_t *data);
