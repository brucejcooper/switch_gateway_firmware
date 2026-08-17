/*
 * boot_decide.c — see boot_decide.h for the full rule table and rationale.
 * Deliberately tiny and branchy rather than clever: this is the one function
 * in the whole OTA design where a subtle logic bug means bricking a device,
 * so it reads exactly like the rule list in the header, in the same order.
 */

#include "boot_decide.h"
#include "partition_map.h"   /* BACKUP_SIZE — see bc_has_valid_backup() below */

boot_action_t boot_decide(const struct boot_control *bc, bool bc_valid,
                           unsigned max_boot_attempts) {
    /* Rule 1: no valid record at all -> trust the active image. */
    if (!bc_valid || bc == NULL) return BOOT_ACTIVE;

    /* Rule 2: a verified staged image is waiting -> install it, unconditionally
     * of the active image's own confirm/attempt state. */
    if (bc->flags & BC_F_PENDING) return BOOT_COMMIT_STAGING;

    /* Rule 3: active image already proven good. */
    if (bc->flags & BC_F_CONFIRMED) return BOOT_ACTIVE;

    /* Rules 4/5: active image is unconfirmed — check the retry budget.
     * `>=`, not `>`: boot_attempts is bumped BEFORE each jump (main()'s
     * shared "about to boot an unconfirmed image" step), so by the time this
     * runs on the Nth boot of a given candidate, boot_attempts already holds
     * N-1 (0 on the very first attempt). max_boot_attempts=3 must mean the
     * candidate gets exactly 3 real boot attempts before a 4th boot rolls it
     * back — attempts 0,1,2 (the first three boots) still read `< 3` here
     * and get BOOT_ACTIVE; attempts==3 (about to be the FOURTH boot) is the
     * first value that reads `>= 3` and rolls back. A plain `>` here let
     * boot_attempts==max_boot_attempts (the Nth boot, not yet an (N+1)th)
     * through as BOOT_ACTIVE too, silently granting one extra attempt beyond
     * whatever the constant said — see MAX_BOOT_ATTEMPTS's own comment
     * (bootloader/main.c) and bootloader/README.md's rule list, which
     * document max_boot_attempts=3 as "three attempts" and must actually
     * mean it. */
    if (bc->boot_attempts >= max_boot_attempts) return BOOT_ROLLBACK;
    return BOOT_ACTIVE;
}

/* See boot_decide.h for the full rationale. */
bool bc_has_valid_backup(const struct boot_control *bc) {
    if (bc == NULL) return false;
    if (!(bc->flags & BC_F_BACKUP_VALID)) return false;
    if (bc->backup_len == 0 || bc->backup_len > BACKUP_SIZE) return false;
    return true;
}
