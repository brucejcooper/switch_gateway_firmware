#pragma once
#include <stdbool.h>

#include "boot_control.h"

/*
 * boot_decide — the PURE decision core of the bootloader. Given the current
 * boot_control record (src/boot_control.h) and a policy knob
 * (max_boot_attempts), decides what the bootloader should do next. This
 * function touches no hardware (no flash reads/writes, no fw_verify) and is
 * fully host-testable (test/boot_decide_test.c) — the bootloader's main()
 * (hardware layer, NOT host-tested) is only a thin wrapper that reads bc_read(),
 * calls this, and acts on the result. Keeping the branchy "what do we do"
 * logic here — separate from "how do we actually copy flash / verify a
 * signature / jump" — is what makes the interesting part of the boot policy
 * testable without hardware.
 *
 * ── The four actions ────────────────────────────────────────────────────
 *   BOOT_ACTIVE          Boot the active app slot (APP_OFF) as-is. The normal
 *                        case: either nothing has ever been staged, or the
 *                        active image is already confirmed good, or it is
 *                        unconfirmed but still within its retry budget.
 *   BOOT_COMMIT_STAGING  A verified-and-signed image is waiting in STAGING
 *                        (BC_F_PENDING). Copy it into the active slot, then
 *                        boot it. Takes priority over everything else — see
 *                        the rule ordering below.
 *   BOOT_ROLLBACK        The active image is unconfirmed AND has now failed
 *                        to confirm across more boots than the retry budget
 *                        allows: a crash loop. THIS PHASE STILL HAS ONLY ONE
 *                        RUN-ADDRESS APP SLOT (partition_map.h: a single
 *                        APP_OFF, no B slot the app itself links against),
 *                        but the bootloader's main() (try_rollback(), see
 *                        bootloader/README.md's "True rollback" section) now
 *                        performs a REAL restore instead of just refusing to
 *                        jump: it gates on bc_has_valid_backup() (below) —
 *                        was the outgoing image that this crash-looping one
 *                        replaced itself CONFIRMED, and so backed up into
 *                        BACKUP at commit time? — then fw_verify()s the
 *                        BACKUP image, copies BACKUP over the now-untrusted
 *                        active slot (XIP-safe), and re-verifies active
 *                        before persisting a fresh BC_F_CONFIRMED record and
 *                        booting it. boot_decide() itself still does none of
 *                        this — it has no access to flash contents — it only
 *                        reports the distinct BOOT_ROLLBACK verdict (rather
 *                        than collapsing it into BOOT_STUCK itself) so
 *                        main() knows to attempt that restore.
 *   BOOT_STUCK           Nothing is safely bootable. boot_decide() itself
 *                        NEVER returns this value — it has no access to flash
 *                        contents, so it cannot know whether an image
 *                        actually verifies. BOOT_STUCK is produced only by
 *                        the hardware layer (main()), when the image an
 *                        action points at (active or freshly-committed) fails
 *                        fw_verify(), or when the BOOT_ROLLBACK restore above
 *                        cannot complete safely — no valid backup
 *                        (bc_has_valid_backup() false, e.g. the very first
 *                        OTA over the factory image has nothing backed up
 *                        yet), the backup image itself fails fw_verify(), or
 *                        the restore copy's post-copy re-verify fails. Never
 *                        jump to an unverified image on any path. It is part
 *                        of this enum only so main()'s state variable and
 *                        boot_decide()'s return type share one vocabulary.
 *
 * ── Rules, in priority order (this is the exact, exhaustive policy) ─────
 *   1. !bc_valid                                        -> BOOT_ACTIVE
 *      No persisted boot-control record at all (first-ever boot before any
 *      OTA has happened, or both redundant sectors are corrupt). There is no
 *      evidence the active image is bad, so trust it.
 *   2. bc_valid && (flags & BC_F_PENDING)                -> BOOT_COMMIT_STAGING
 *      A staged image is waiting, regardless of the active image's own
 *      confirmed/attempt state. This is checked before the crash-loop rule
 *      on purpose: a fresh, successfully-staged update should always get its
 *      chance to install, even if the *previous* boot happened to be
 *      mid-crash-loop-detection.
 *   3. bc_valid && !(flags & BC_F_PENDING) && (flags & BC_F_CONFIRMED)
 *                                                        -> BOOT_ACTIVE
 *      Active image already proven good (ota_stage_confirm() /
 *      ota_mark_valid() ran) — boot it, no attempt bookkeeping needed.
 *   4. bc_valid && !(flags & BC_F_PENDING) && !(flags & BC_F_CONFIRMED)
 *        && boot_attempts >= max_boot_attempts            -> BOOT_ROLLBACK
 *      Unconfirmed AND already burned through the retry budget: crash loop.
 *   5. bc_valid && !(flags & BC_F_PENDING) && !(flags & BC_F_CONFIRMED)
 *        && boot_attempts < max_boot_attempts             -> BOOT_ACTIVE
 *      Unconfirmed but still within budget — try again. (main() is
 *      responsible for bumping boot_attempts and persisting it via bc_write()
 *      *before* jumping, so a crash this attempt is visible next boot; see
 *      bootloader/README.md.)
 *
 * boot_attempts and max_boot_attempts are compared with `>=`, so
 * max_boot_attempts really does mean "this many attempts": boot_attempts
 * holds the count of PRIOR attempts (bumped after each one, before the next
 * jump — see rule 5's parenthetical), so a fresh candidate reads 0 on its
 * first boot, 1 on its second, ..., max_boot_attempts-1 on its
 * max_boot_attempts'th (still BOOT_ACTIVE — `< max_boot_attempts`), and
 * max_boot_attempts on what would be attempt number max_boot_attempts+1
 * (BOOT_ROLLBACK — the candidate never gets that extra try). A plain `>`
 * here would let boot_attempts == max_boot_attempts itself through as one
 * more BOOT_ACTIVE, silently granting max_boot_attempts+1 real attempts —
 * see MAX_BOOT_ATTEMPTS's own comment (bootloader/main.c), which documents
 * exactly three and must mean it.
 */
typedef enum {
    BOOT_ACTIVE,
    BOOT_COMMIT_STAGING,
    BOOT_ROLLBACK,
    BOOT_STUCK,
} boot_action_t;

boot_action_t boot_decide(const struct boot_control *bc, bool bc_valid,
                           unsigned max_boot_attempts);

/*
 * bc_has_valid_backup — TRUE-ROLLBACK PHASE. Pure field-range check: does
 * *bc name a usable BACKUP region (partition_map.h) image? "Usable" here
 * means exactly BC_F_BACKUP_VALID is set AND backup_len is in
 * (0, BACKUP_SIZE] — the same range ota_stage.c promises to write it in
 * (see boot_control.h's BC_F_BACKUP_VALID doc), and the range the
 * hardware-layer copy loops in main.c need to stay inside both the BACKUP
 * and active slots.
 *
 * This does NOT prove the backup image itself verifies (fw_verify) — that
 * requires reading flash, out of scope for this pure/host-testable helper —
 * only that it is structurally worth spending a fw_verify() call on at all.
 * It is the single shared gate for both of main.c's backup touch points,
 * which must agree on what "usable" means before either ever reads/writes
 * flash:
 *   - try_commit_staging(): whether to back up the outgoing active image
 *     into BACKUP before overwriting active with the new staged image;
 *   - the BOOT_ROLLBACK handler: whether there is anything in BACKUP worth
 *     attempting to fw_verify() + restore at all.
 * Returns false (nothing usable) if bc is NULL.
 */
bool bc_has_valid_backup(const struct boot_control *bc);
