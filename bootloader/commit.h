#pragma once
#include <stdbool.h>

#include "boot_control.h"
#include "boot_decide.h"

/*
 * commit.{h,c} — the bootloader's commit/rollback orchestration, factored
 * out of main.c (2026-08-17) so it can be linked and exercised by a host
 * test (test/boot_commit_test.c) as a SEQUENCE across simulated boots, the
 * same way boot_decide.c already is for the pure policy layer. Nothing here
 * changes bootloader BEHAVIOUR — this is a straight code move (plus a
 * host-test seam, matching the pattern src/boot_control.c / src/ota_stage.c
 * already use) so the real bootloader.elf built from bootloader/main.c +
 * this file is the same program it was before the move; only the
 * CMakeLists.txt file list changed, and main.c now calls through these
 * declarations instead of defining the functions itself.
 *
 * Host-test seam: on target this reads/writes the real STAGING/APP/BACKUP
 * flash regions (PART_XIP()) and verifies against the real FW_PUBKEY; under
 * -DBOOT_COMMIT_HOST_TEST it redirects both through indirections the host
 * test supplies — see commit.c's header comment for the two hooks
 * (bc_commit_test_region()/bc_commit_test_pubkey()).
 */

/* try_commit_staging()/try_rollback() — see commit.c for the full
 * boot_decide.h BOOT_COMMIT_STAGING/BOOT_ROLLBACK orchestration doc
 * comments (moved here verbatim from bootloader/main.c, unchanged). */
boot_action_t try_commit_staging(struct boot_control *bc);
boot_action_t try_rollback(struct boot_control *bc);

/*
 * boot_commit_run — everything main() does from bc_read() through settling
 * on a final BOOT_ACTIVE/BOOT_STUCK verdict: boot_decide(), the
 * COMMIT_STAGING/ROLLBACK dispatch, and the shared "about to boot an
 * unconfirmed/freshly-committed active image" bc_write() step (attempts
 * bump + the deferred BC_F_PENDING clear — see commit.c's
 * pending_clear_unwritten doc comment, moved here unchanged from main.c).
 *
 * Deliberately STOPS short of two things real main() still does on its own,
 * both hardware-only and orthogonal to this state machine: the `!bc_valid`
 * factory-path hardening (vector-table sanity check + watchdog-armed jump,
 * bootloader/README.md's "Hardware bring-up findings" #2/#3) and the actual
 * halt_blink()/jump_to_app() hand-off. Every scenario this function's
 * verdict matters for (a BC_F_PENDING commit, a crash-loop rollback, the
 * backup bookkeeping around either) requires bc_valid == true anyway, so
 * excluding the !bc_valid-only hardware block costs the host test nothing —
 * see bootloader/README.md's Fix-5 note (or the task this was built from)
 * for why a deeper factor was judged not worth the risk to the bootloader's
 * own hardware-only code.
 *
 * *bc_valid_out, if non-NULL, receives what bc_read() found (matching
 * main()'s own bc_valid local) — real main() still needs it for the
 * !bc_valid hardening check that runs after this returns.
 *
 * Calling this repeatedly against the same persisted boot-control state
 * (real flash on target; a host test's RAM seam) replays a SEQUENCE of real
 * boots exactly as the hardware would run them — including a boot that
 * "crashes" between this call and whatever main() would have done next,
 * simply by not calling it again. */
boot_action_t boot_commit_run(unsigned max_boot_attempts, bool *bc_valid_out);
