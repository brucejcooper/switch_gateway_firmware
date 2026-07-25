/*
 * boot_decide_test.c — exhaustive host-side test for the bootloader's pure
 * decision function, bootloader/boot_decide.c. See that file (and
 * bootloader/boot_decide.h for the full rule table) for the policy being
 * tested; this file drives it across the whole state matrix named in the
 * task: no valid bc -> ACTIVE; PENDING -> COMMIT_STAGING; CONFIRMED ->
 * ACTIVE; attempts>max && !CONFIRMED -> ROLLBACK; plus the boundary at
 * attempts == max, PENDING's priority over the crash-loop check, several
 * max_boot_attempts policy values, and bc==NULL/bc_valid mismatches.
 *
 * Not part of the firmware build (see CLAUDE.md: host tests live under
 * test/ and are never added to the RP2040 CMake target). Links straight
 * against the real bootloader/boot_decide.c — no logic is reimplemented
 * here, only exercised.
 *
 * Build & run:
 *   cc -Wall -Wextra -I ../src -I ../bootloader \
 *       -o /tmp/boot_decide_test boot_decide_test.c ../bootloader/boot_decide.c \
 *       && /tmp/boot_decide_test
 */
#include "../bootloader/boot_decide.h"
#include "../src/partition_map.h"   /* BACKUP_SIZE — see bc_has_valid_backup() checks below */

#include <stdio.h>
#include <string.h>

static int fail = 0;
#define CHECK(cond) do { \
        if (!(cond)) { \
            printf("FAIL: %s (%s:%d)\n", #cond, __FILE__, __LINE__); \
            fail = 1; \
        } \
    } while (0)

static struct boot_control mk(uint32_t flags, uint32_t boot_attempts) {
    struct boot_control bc;
    memset(&bc, 0, sizeof bc);
    bc.magic = BC_MAGIC;
    bc.flags = flags;
    bc.boot_attempts = boot_attempts;
    return bc;
}

int main(void) {
    /* ── Rule 1: no valid record -> ACTIVE, regardless of what garbage *bc
     *    holds (bc_valid=false means the caller must not trust its contents
     *    at all) or whether bc is NULL outright. ──────────────────────── */
    {
        struct boot_control garbage = mk(BC_F_PENDING | BC_F_CONFIRMED, 999);
        CHECK(boot_decide(&garbage, false, 3) == BOOT_ACTIVE);
        CHECK(boot_decide(NULL, false, 3) == BOOT_ACTIVE);
    }

    /* ── Rule 2: PENDING -> COMMIT_STAGING, unconditionally of CONFIRMED or
     *    attempts (checked before the crash-loop rule). ─────────────────── */
    {
        CHECK(boot_decide(&(struct boot_control){0}, true, 3) != BOOT_COMMIT_STAGING);
        /* baseline: PENDING alone */
        struct boot_control bc = mk(BC_F_PENDING, 0);
        CHECK(boot_decide(&bc, true, 3) == BOOT_COMMIT_STAGING);

        /* PENDING + huge attempts (way past any budget) still commits */
        bc = mk(BC_F_PENDING, 1000);
        CHECK(boot_decide(&bc, true, 3) == BOOT_COMMIT_STAGING);

        /* PENDING + CONFIRMED both set (shouldn't normally happen, but
         * PENDING must still win) */
        bc = mk(BC_F_PENDING | BC_F_CONFIRMED, 0);
        CHECK(boot_decide(&bc, true, 3) == BOOT_COMMIT_STAGING);

        /* PENDING + CONFIRMED + huge attempts */
        bc = mk(BC_F_PENDING | BC_F_CONFIRMED, 1000);
        CHECK(boot_decide(&bc, true, 3) == BOOT_COMMIT_STAGING);
    }

    /* ── Rule 3: CONFIRMED (no PENDING) -> ACTIVE, regardless of attempts. */
    {
        struct boot_control bc = mk(BC_F_CONFIRMED, 0);
        CHECK(boot_decide(&bc, true, 3) == BOOT_ACTIVE);

        bc = mk(BC_F_CONFIRMED, 3);      /* == max */
        CHECK(boot_decide(&bc, true, 3) == BOOT_ACTIVE);

        bc = mk(BC_F_CONFIRMED, 4);      /* > max — CONFIRMED still wins */
        CHECK(boot_decide(&bc, true, 3) == BOOT_ACTIVE);

        bc = mk(BC_F_CONFIRMED, 1000000);
        CHECK(boot_decide(&bc, true, 3) == BOOT_ACTIVE);
    }

    /* ── Rules 4/5: no PENDING, no CONFIRMED -> pure attempts-vs-budget. ── */
    {
        unsigned max = 3;
        for (uint32_t attempts = 0; attempts <= max; attempts++) {
            struct boot_control bc = mk(0, attempts);
            CHECK(boot_decide(&bc, true, max) == BOOT_ACTIVE);
        }
        for (uint32_t attempts = max + 1; attempts <= max + 5; attempts++) {
            struct boot_control bc = mk(0, attempts);
            CHECK(boot_decide(&bc, true, max) == BOOT_ROLLBACK);
        }
    }

    /* ── Same rules 4/5, swept across several max_boot_attempts policy
     *    values, including the max_boot_attempts == 0 edge (any attempt
     *    beyond the very first unconfirmed boot rolls back immediately). ── */
    {
        unsigned maxes[] = { 0, 1, 2, 5, 10 };
        for (size_t m = 0; m < sizeof maxes / sizeof maxes[0]; m++) {
            unsigned max = maxes[m];
            struct boot_control at_max = mk(0, max);
            CHECK(boot_decide(&at_max, true, max) == BOOT_ACTIVE);

            struct boot_control one_over = mk(0, max + 1);
            CHECK(boot_decide(&one_over, true, max) == BOOT_ROLLBACK);
        }
    }

    /* ── boot_attempts == 0 with no flags at all (e.g. a freshly-written,
     *    all-zero record with neither PENDING nor CONFIRMED — shouldn't
     *    arise in practice via ota_stage.c, but boot_decide must still do
     *    something sane): well within any nonzero budget -> ACTIVE. ────── */
    {
        struct boot_control bc = mk(0, 0);
        CHECK(boot_decide(&bc, true, 0) == BOOT_ACTIVE);   /* 0 <= 0 */
        CHECK(boot_decide(&bc, true, 5) == BOOT_ACTIVE);
    }

    /* boot_decide() must never itself report BOOT_STUCK — that verdict is
     * reserved for the hardware layer (main()) reacting to a real fw_verify
     * failure, which this pure function has no way to observe. Sweep a wide
     * combination of inputs and assert STUCK never comes out. */
    {
        uint32_t flag_combos[] = { 0, BC_F_PENDING, BC_F_CONFIRMED,
                                    BC_F_PENDING | BC_F_CONFIRMED };
        for (size_t f = 0; f < sizeof flag_combos / sizeof flag_combos[0]; f++) {
            for (uint32_t attempts = 0; attempts <= 20; attempts++) {
                struct boot_control bc = mk(flag_combos[f], attempts);
                for (unsigned max = 0; max <= 10; max++) {
                    CHECK(boot_decide(&bc, true, max) != BOOT_STUCK);
                    CHECK(boot_decide(&bc, false, max) != BOOT_STUCK);
                }
            }
        }
    }

    /* ── bc_has_valid_backup(): TRUE-ROLLBACK PHASE pure gate. Exercises the
     *    exact (0, BACKUP_SIZE] range plus the BC_F_BACKUP_VALID flag gate,
     *    independent of everything else in the record (PENDING/CONFIRMED/
     *    attempts must not matter — this predicate is purely about the
     *    backup fields), and the bc==NULL case. ────────────────────────── */
    {
        /* Flag clear -> false regardless of backup_len. */
        struct boot_control bc = mk(0, 0);
        bc.backup_len = 1234;
        CHECK(!bc_has_valid_backup(&bc));

        /* Flag set but backup_len == 0 (the documented "nothing to roll back
         * to" sentinel from ota_stage.c) -> false. */
        bc = mk(BC_F_BACKUP_VALID, 0);
        bc.backup_len = 0;
        CHECK(!bc_has_valid_backup(&bc));

        /* Flag set, backup_len == 1 (smallest valid) -> true. */
        bc = mk(BC_F_BACKUP_VALID, 0);
        bc.backup_len = 1;
        CHECK(bc_has_valid_backup(&bc));

        /* Flag set, backup_len == BACKUP_SIZE (largest valid, inclusive) ->
         * true. */
        bc = mk(BC_F_BACKUP_VALID, 0);
        bc.backup_len = BACKUP_SIZE;
        CHECK(bc_has_valid_backup(&bc));

        /* Flag set, backup_len == BACKUP_SIZE + 1 (one past valid) ->
         * false. */
        bc = mk(BC_F_BACKUP_VALID, 0);
        bc.backup_len = BACKUP_SIZE + 1;
        CHECK(!bc_has_valid_backup(&bc));

        /* Flag set, backup_len absurdly large -> false. */
        bc = mk(BC_F_BACKUP_VALID, 0);
        bc.backup_len = 0xFFFFFFFFu;
        CHECK(!bc_has_valid_backup(&bc));

        /* PENDING/CONFIRMED/boot_attempts must not influence the verdict —
         * only the backup fields matter. */
        bc = mk(BC_F_BACKUP_VALID | BC_F_PENDING | BC_F_CONFIRMED, 999);
        bc.backup_len = BACKUP_SIZE;
        CHECK(bc_has_valid_backup(&bc));

        /* NULL -> false, doesn't crash. */
        CHECK(!bc_has_valid_backup(NULL));
    }

    if (!fail) printf("PASS: all boot_decide checks passed\n");
    return fail;
}
