/*
 * commit.c — bootloader commit/rollback orchestration. See commit.h for why
 * this is a separate file from main.c (host-testability, test/
 * boot_commit_test.c) and the doc comment on boot_commit_run() for what it
 * covers vs. what stays in main.c. Every doc comment on
 * try_commit_staging()/try_rollback() below is unchanged from when this code
 * lived in bootloader/main.c — only the file it lives in, and the
 * PART_XIP()/FW_PUBKEY indirections described next, have changed.
 *
 * Host-test seam (matches src/boot_control.c's BOOT_CONTROL_HOST_TEST and
 * src/ota_stage.c's OTA_STAGE_HOST_TEST): every access that would otherwise
 * go through PART_XIP(STAGING_OFF/APP_OFF/BACKUP_OFF) or FW_PUBKEY is routed
 * through REGION_PTR()/VERIFY_PUBKEY, which redirect to RAM (a host test's
 * bc_commit_test_region()/bc_commit_test_pubkey()) under
 * -DBOOT_COMMIT_HOST_TEST and are plain passthroughs to the real flash
 * addresses/key otherwise. flash_copy_region() gets its own host stand-in
 * (a plain memcpy) for the same reason src/ota_stage.c's flash op needs
 * one: real flash_range_erase/program don't exist off-target.
 *
 * Runs with only one core active, no flash_safe_execute() — see
 * bootloader/README.md's "Flash layout" section for why (no second core to
 * lock out of concurrent XIP access here, unlike the app).
 */

#include "commit.h"

#include <string.h>

#include "partition_map.h"
#include "fw_image.h"

#ifdef BOOT_COMMIT_HOST_TEST

extern uint8_t *bc_commit_test_region(uint32_t off);
#define REGION_PTR(off) (bc_commit_test_region(off))

extern const uint8_t *bc_commit_test_pubkey(void);
#define VERIFY_PUBKEY (bc_commit_test_pubkey())

/* Host stand-in for the XIP-safe sector-by-sector flash copy below: both
 * REGION_PTR endpoints are plain RAM (the same flat array — see
 * test/boot_commit_test.c), so a straight memcpy reproduces the same net
 * effect (dst[0..len) == src[0..len) as it stood before the call) without
 * faking erase/program or sector granularity. This exercises the
 * SEQUENCING/gating logic in try_commit_staging()/try_rollback() below —
 * what commits when, in what order, behind which verify — which is what
 * this test suite is for; the actual RP2040 flash mechanics
 * (XIP-unavailable-during-erase/program, sector alignment) are a separate,
 * already hardware-validated concern (bootloader/README.md's "Hardware
 * bring-up findings"). */
static void flash_copy_region(uint32_t src_off, uint32_t dst_off, uint32_t len) {
    memcpy(REGION_PTR(dst_off), REGION_PTR(src_off), len);
}

#else /* !BOOT_COMMIT_HOST_TEST — target build */

/* partition_map.h defines its own FLASH_SECTOR_SIZE/FLASH_PAGE_SIZE as plain
 * numeric literals (host-compilable, see its header comment); undef before
 * pulling in the SDK's copies so the two identical-value definitions don't
 * produce a harmless-but-noisy macro-redefined warning — same pattern as
 * src/boot_control.c, src/ota_stage.c and bootloader/main.c. */
#undef FLASH_SECTOR_SIZE
#undef FLASH_PAGE_SIZE
#include "hardware/flash.h"

#define REGION_PTR(off) ((uint8_t *)PART_XIP(off))
#define VERIFY_PUBKEY (FW_PUBKEY)

/* ── generic XIP-safe flash-to-flash copy ─────────────────────────────────
 *
 * RP2040 XIP note (see CLAUDE.md's flash-write model / the task brief this
 * was built from): the flash bus cannot serve an execute-in-place READ of
 * *any* flash region — not just the one being touched — while an
 * erase/program is in flight. So for every sector: copy that sector's
 * source bytes out of `src_off` (an ordinary XIP-mapped memcpy, made while
 * flash is fully idle) into a RAM buffer FIRST, and only THEN erase +
 * reprogram the corresponding sector at `dst_off` from that RAM copy.
 * flash_range_erase/flash_range_program are pico-sdk functions that already
 * run from RAM (__no_inline_not_in_flash_func) for exactly this reason.
 *
 * Shared by every flash-region-to-flash-region copy this bootloader makes:
 * STAGING -> active (commit), active -> BACKUP (commit-time backup, true-
 * rollback phase), and BACKUP -> active (rollback restore, true-rollback
 * phase) — same sector-at-a-time RAM-buffered dance regardless of which two
 * regions are involved.
 *
 * `len` is the exact byte length of the signed artifact (image + trailer)
 * being copied — the caller is responsible for bounds-checking it against
 * both the source and destination regions' sizes before calling this, so
 * the sector math here can't walk off either region. `src_off`/`dst_off`
 * are absolute flash byte offsets (partition_map.h's *_OFF constants). */
static void flash_copy_region(uint32_t src_off, uint32_t dst_off, uint32_t len) {
    static uint8_t chunk[FLASH_SECTOR_SIZE];
    const uint8_t *src = REGION_PTR(src_off);
    uint32_t sectors = (len + FLASH_SECTOR_SIZE - 1) / FLASH_SECTOR_SIZE;

    for (uint32_t s = 0; s < sectors; s++) {
        uint32_t off = s * FLASH_SECTOR_SIZE;
        memcpy(chunk, src + off, FLASH_SECTOR_SIZE);           /* XIP read: flash fully idle here */
        flash_range_erase(dst_off + off, FLASH_SECTOR_SIZE);   /* flash busy: XIP unavailable during this call */
        flash_range_program(dst_off + off, chunk, FLASH_SECTOR_SIZE);
    }
}

#endif /* BOOT_COMMIT_HOST_TEST */

/* ── COMMIT_STAGING orchestration ─────────────────────────────────────────
 * Called only when boot_decide() returned BOOT_COMMIT_STAGING, i.e.
 * bc_valid is true and bc->flags has BC_F_PENDING set. Returns the action
 * main() should take next:
 *
 *   BOOT_ACTIVE  Full success only: the staged image verified, was copied
 *                into the active slot, and the active slot re-verified
 *                afterwards. bc->flags has BC_F_PENDING cleared in RAM
 *                (NOT written to flash yet — see the big comment in
 *                main() about why that single write is deferred to the
 *                shared "about to boot an unconfirmed active image" step).
 *   BOOT_STUCK   Anything went wrong: bad staged_len, the staged image
 *                itself fails fw_verify, a commit-time backup copy that
 *                doesn't itself verify (true-rollback phase, see below),
 *                or the post-copy re-verify of the active slot fails. In
 *                EVERY failure case bc is left completely untouched
 *                (BC_F_PENDING still set, nothing written) — see the
 *                important note below on why this never falls back to
 *                "just boot whatever was active before".
 *
 * Why a rejected/failed commit does NOT fall back to booting the old
 * active image: ota_stage_finish() (src/ota_stage.c, already-landed and
 * frozen) writes a *brand-new* boot_control record when staging completes
 * — flags=BC_F_PENDING, staged_len/staged_sha512 = the CANDIDATE's, with
 * everything else (including whatever staged_len the *previous* active
 * image was installed with) simply gone. So the instant BC_F_PENDING is
 * observed, bc->staged_len no longer names the active image's real
 * length — it names the candidate's. If that candidate then fails
 * verification here, there is no length left anywhere to re-verify the
 * (perfectly fine, untouched) active image against; guessing one would
 * violate "never run an unverified image" just as badly as skipping the
 * check outright. So every failure path here goes to BOOT_STUCK rather
 * than attempting a fallback boot with a number we know is wrong. In
 * practice this should be rare: ota_stage_finish() already ran this exact
 * fw_verify() over the staged image before ever writing BC_F_PENDING, so a
 * failure here means the staging region was corrupted/tampered with
 * *after* that check (or, post-copy, that the copy itself went wrong).
 *
 * TRUE-ROLLBACK PHASE — commit-time backup: BEFORE the outgoing active
 * image is overwritten, back it up into BACKUP so a future crash-looping
 * commit can be undone (see the BOOT_ROLLBACK handler below) instead of
 * halting. Gated on bc_has_valid_backup(bc) (boot_decide.c) — set only by
 * ota_stage.c when the outgoing image was itself CONFIRMED (see
 * ota_stage.h's "only ever back up a CONFIRMED outgoing image" rule). No
 * valid backup record (e.g. the very first OTA, over the trailer-less
 * factory image) -> skip the backup copy entirely; there is simply no
 * fallback yet for that image, same residual gap README.md documents for
 * the unconditional-jump factory-boot case. */
boot_action_t try_commit_staging(struct boot_control *bc) {
    if (bc->staged_len == 0 || bc->staged_len > STAGING_SIZE || bc->staged_len > APP_SIZE) {
        /* A structurally-valid boot_control record (magic+crc32 both
         * checked out) with a nonsensical staged_len shouldn't be
         * reachable in practice (ota_stage_finish() always writes a
         * staged_len that fits both regions), but a bootloader is exactly
         * the wrong place to trust "shouldn't happen". */
        return BOOT_STUCK;
    }

    const uint8_t *staging = REGION_PTR(STAGING_OFF);
    if (!fw_verify(staging, bc->staged_len, VERIFY_PUBKEY)) {
        /* Staging is corrupt, unsigned, or tampered with — see the big
         * comment above for why this can't safely fall back to the old
         * active image. The active slot itself has not been touched at
         * all yet (this check runs before any flash copy below). */
        return BOOT_STUCK;
    }

    const uint8_t *active = REGION_PTR(APP_OFF);

    /* ── interrupted-commit gate: has THIS commit already completed on a
     * previous, interrupted boot? ─────────────────────────────────────────
     *
     * A commit can be cut short after the staging->active copy below has
     * finished (and even after it re-verified) but before main()'s bc_write
     * persists BC_F_PENDING's clear — that write is deliberately deferred to
     * the shared "about to boot an unconfirmed active image" step in main(),
     * see this function's own doc comment. On the retry boot, bc_read()
     * comes back with BC_F_PENDING still set (the clear never reached
     * flash), boot_decide() returns BOOT_COMMIT_STAGING again, and this
     * function runs again from the top against a STAGING region and a
     * staged_len that are both completely unchanged — but ACTIVE now
     * already holds the candidate, not the old outgoing image the naive
     * (pre-this-fix) code below assumed.
     *
     * The naive code backed up unconditionally whenever bc_has_valid_backup()
     * was true, sizing that backup copy from bc->backup_len (the OLD
     * outgoing image's length). If backup_len happened to equal staged_len,
     * fw_verify(ACTIVE, backup_len) — checking whether ACTIVE holds a validly
     * signed image of that length — PASSED, because ACTIVE now holds the
     * candidate, which genuinely is a validly signed image of that length.
     * So the "verify the outgoing image before backing it up" check does NOT
     * close this hole: it happily overwrites the last-good BACKUP with a
     * second copy of the CANDIDATE, silently destroying the actual rollback
     * safety net. If the lengths differ, the backup copy instead reproduces
     * a torn image of the wrong length, permanently corrupting the same
     * safety net and landing in a stable BOOT_STUCK. Either way, the retry
     * boot must never blindly redo the backup step.
     *
     * The robust gate compares TRAILERS, not lengths: STAGING's trailer
     * (already proven-signed by the fw_verify() just above) and ACTIVE's
     * trailer at the same, candidate-sized offset. RFC 8032 Ed25519 signing
     * is deterministic, so two byte-identical trailers (magic, img_len,
     * sha512 AND sig) can only mean ACTIVE already holds this exact signed
     * candidate — forging a match without the private key is exactly as
     * hard as forging fw_verify() itself. A match therefore proves the
     * staging->active copy already ran to completion on a prior boot; a
     * mismatch proves it did not (or the active slot holds something else
     * entirely, torn or old).
     *
     * staged_len >= sizeof(struct fw_trailer) is already guaranteed by the
     * fw_verify() above (it fails closed on any total_len shorter than the
     * trailer — see fw_image.c — so this function would already have
     * returned BOOT_STUCK otherwise), and staged_len <= APP_SIZE is
     * guaranteed by the bounds check at the top of this function, so
     * trailer_off can't underflow and the read below can't walk off either
     * slot. Restated explicitly below anyway, belt-and-braces, matching this
     * file's "never trust shouldn't happen" rule (see the bounds check at
     * the top of this function). */
    if (bc->staged_len < sizeof(struct fw_trailer)) return BOOT_STUCK;
    uint32_t trailer_off = bc->staged_len - sizeof(struct fw_trailer);
    bool commit_already_done =
        memcmp(active + trailer_off, staging + trailer_off, sizeof(struct fw_trailer)) == 0;

    if (!commit_already_done) {
        /* ── commit-time backup (true-rollback phase) ─────────────────────
         * Staging verified and this is NOT a retry of an already-completed
         * commit — proceed as before, with one added gate. bc_has_valid_backup()
         * (boot_decide.c, pure/host-tested) checks exactly the
         * BC_F_BACKUP_VALID + backup_len-in-range contract ota_stage.c
         * promises; bc->backup_len is the OUTGOING image's own byte length
         * (NOT bc->staged_len, which by now names the *candidate* — see the
         * big comment above the function). */
        if (bc_has_valid_backup(bc)) {
            /* Extra gate this phase adds: only back up if ACTIVE itself
             * still verifies AS THE OUTGOING IMAGE (backup_len). Walk the
             * interruption points to see why this is exactly the right
             * condition, and why it's safe:
             *
             * (a) Cut during THIS backup copy (flash_copy_region(APP_OFF,
             *     BACKUP_OFF, ...) below, mid-loop): active has not been
             *     touched by this function at all yet (the backup copy
             *     strictly precedes the active-slot erase later in this
             *     function, on every code path) — so ACTIVE still holds the
             *     old outgoing image intact, fw_verify(ACTIVE, backup_len)
             *     passes, commit_already_done is false (ACTIVE's trailer is
             *     still the OLD one), and the backup copy correctly re-runs
             *     from scratch. BACKUP itself may be left part-old/part-new
             *     from the interrupted copy, but that's fine: it hasn't been
             *     trusted yet (BC_F_BACKUP_VALID / bc are untouched by this
             *     boot) and this re-run overwrites it sector-by-sector
             *     again before it's ever re-verified below.
             * (b) Cut during the staging->active copy further down (mid-
             *     loop): ACTIVE is now a TORN mix of old and new bytes —
             *     genuinely neither the outgoing image nor the candidate.
             *     Because the backup copy in (a) strictly precedes this
             *     point on every path through this function, if execution
             *     ever reached here on a PRIOR boot, the backup was either
             *     already made and verified then, or there was nothing to
             *     back up in the first place (bc_has_valid_backup() false) —
             *     either way there is nothing this boot needs to (re)do for
             *     BACKUP. commit_already_done is false here too (the torn
             *     ACTIVE's trailer, if the copy hasn't yet reached the tail
             *     sector, is still the OLD trailer, not the candidate's) —
             *     so without this extra fw_verify(ACTIVE, backup_len) gate,
             *     the naive backup-whenever-trailers-differ rule would copy
             *     this GARBAGE torn image over the good backup made in (a).
             *     fw_verify(ACTIVE, backup_len) correctly fails here (a torn
             *     mix of two different images is never a validly-signed
             *     image of either length), so the gate skips the backup —
             *     exactly right, since it was already made (or never
             *     needed) before this point was ever reachable.
             * (c) Cut after the staging->active copy completes AND its own
             *     post-copy fw_verify() (below) succeeds, but before
             *     main()'s deferred bc_write() persists BC_F_PENDING's
             *     clear: this is the commit_already_done == true case
             *     handled by the outer `if` above — this whole backup
             *     branch is skipped entirely, correctly, without even
             *     reaching this fw_verify(ACTIVE, backup_len) check.
             *
             * So: skip backup if trailers already match (case c, handled
             * above); else do the backup ONLY if ACTIVE still verifies as
             * the outgoing image (cases "never started this boot" and (a));
             * else (case (b), ACTIVE is a torn copy) skip it — it was
             * necessarily already made, or never needed, before the active
             * slot was first touched. */
            if (fw_verify(active, bc->backup_len, VERIFY_PUBKEY)) {
                flash_copy_region(APP_OFF, BACKUP_OFF, bc->backup_len);

                /* Re-verify the backup we just made — the only way this
                 * bootloader can detect a bad flash copy, since
                 * flash_range_erase/program report failure only via
                 * hard_assert on malformed *arguments*, not on a genuine
                 * write fault. Fails -> BOOT_STUCK immediately, WITHOUT
                 * having touched active at all (matches the acceptance
                 * rule: a backup-copy failure must never proceed to
                 * overwrite the still-good active image). The device is no
                 * worse off than before this boot attempt — active is
                 * untouched, staging is untouched, bc is untouched below
                 * (BC_F_PENDING still set) — just stuck rather than
                 * silently committing over an active image whose rollback
                 * safety net didn't actually get made. */
                if (!fw_verify(REGION_PTR(BACKUP_OFF), bc->backup_len, VERIFY_PUBKEY)) {
                    return BOOT_STUCK;
                }
            }
            /* else: ACTIVE is a torn copy from an interrupted staging->active
             * copy on a previous boot (interruption point (b) above) — skip
             * the backup; it was already made (or never applicable) before
             * active was first touched, so redoing it here would only
             * clobber the real backup with garbage. */
        }
        /* else: no valid backup record — nothing to back up (first-ever OTA,
         * or the outgoing image was never itself confirmed); proceed
         * straight to the commit below exactly as before this phase. */

        /* Staging verified (and, if applicable, the outgoing active image is
         * now safely backed up, or correctly left alone) — copy staging
         * into the active slot. From this point on the OLD active image is
         * being overwritten; a failure past here can no longer be undone by
         * re-reading the (about to be clobbered) active slot — only the
         * just-made BACKUP (if any) can help, and only on a LATER boot via
         * the BOOT_ROLLBACK path, once this candidate has been given its
         * fair chance and crash-looped. */
        flash_copy_region(STAGING_OFF, APP_OFF, bc->staged_len);
    }
    /* else: commit_already_done — this exact candidate was already copied
     * into ACTIVE (and, on the boot that did it, already re-verified below)
     * on a previous, interrupted boot; see interruption point (c) above.
     * Skip both the backup copy and the redundant staging->active copy
     * entirely and fall straight through to the re-verify immediately
     * below, which re-checks the (already-correct) active slot one more
     * time before clearing BC_F_PENDING. */

    if (!fw_verify(active, bc->staged_len, VERIFY_PUBKEY)) {
        /* The copy did not reproduce a verifiable image (flash write
         * error, power loss mid-copy, ...). Leave bc exactly as read
         * (BC_F_PENDING still set) — the next boot retries the same
         * commit from the untouched staging copy — and refuse to jump to
         * whatever is now sitting in the active slot. */
        return BOOT_STUCK;
    }

    bc->flags &= ~BC_F_PENDING;
    return BOOT_ACTIVE;
}

/* ── BOOT_ROLLBACK orchestration (true-rollback phase) ────────────────────
 * Called only when boot_decide() returned BOOT_ROLLBACK, i.e. the active
 * image is unconfirmed and has crash-looped past MAX_BOOT_ATTEMPTS
 * (boot_decide.h rules 4/5). Attempts to restore the previous known-good
 * image from BACKUP over the now-untrustworthy active slot instead of
 * simply refusing to boot. Returns the action main() should take next:
 *
 *   BOOT_ACTIVE  Full success only: BACKUP held a usable image, it
 *                verified, was copied into the active slot, the copy
 *                re-verified, and a fresh CONFIRMED boot_control record
 *                naming it was persisted. `*bc` is updated in place to
 *                match that persisted record, so main()'s subsequent
 *                "verify the active image before jumping" step re-verifies
 *                and then jumps to the RESTORED image, not the
 *                crash-looping one it replaced.
 *   BOOT_STUCK   No usable backup (bc_has_valid_backup() false — e.g. the
 *                very first-ever OTA had nothing to back up, or the image
 *                that crashed was never itself preceded by a CONFIRMED
 *                one), the backup image itself fails fw_verify(), or the
 *                restore copy's post-copy re-verify fails. Never jump to
 *                an unverified image — same rule as everywhere else in
 *                this bootloader. Every failure path here leaves bc/flash
 *                exactly as found: nothing here can make a bad situation
 *                actively worse than the pre-this-phase "just halt"
 *                behaviour. */
boot_action_t try_rollback(struct boot_control *bc) {
    if (!bc_has_valid_backup(bc)) {
        /* Nothing to roll back to — see bc_has_valid_backup()'s doc
         * (boot_decide.h) for exactly which cases land here. Same terminal
         * state as before this phase existed: refuse to jump. */
        return BOOT_STUCK;
    }

    const uint8_t *backup = REGION_PTR(BACKUP_OFF);
    if (!fw_verify(backup, bc->backup_len, VERIFY_PUBKEY)) {
        /* The backup doesn't verify (corrupt/tampered/never actually a
         * complete image) — refuse to restore an unverified image. The
         * active slot is untouched; this check runs before any copy. */
        return BOOT_STUCK;
    }

    /* Backup verified — restore it over the crash-looping active image,
     * XIP-safe (same RAM-buffered sector dance as every other flash copy
     * here; see flash_copy_region()'s header comment for why). */
    flash_copy_region(BACKUP_OFF, APP_OFF, bc->backup_len);

    const uint8_t *active = REGION_PTR(APP_OFF);
    if (!fw_verify(active, bc->backup_len, VERIFY_PUBKEY)) {
        /* The restore copy did not reproduce a verifiable image (flash
         * write error, power loss mid-copy, ...). BACKUP's own copy is
         * untouched by this failed attempt (only active was written), so a
         * *future* boot attempting the same rollback still has a real
         * backup to try again from — but *this* boot must not jump into
         * whatever is now sitting in active. */
        return BOOT_STUCK;
    }

    /* Restore verified end-to-end. This is now the active image, and by
     * construction it is the last image that was ever CONFIRMED — only a
     * CONFIRMED outgoing image is ever backed up in the first place (see
     * try_commit_staging()'s commit-time backup step / ota_stage.h's
     * "only ever back up a CONFIRMED outgoing image" rule) — so persist a
     * fresh record naming it directly CONFIRMED, with boot_attempts
     * zeroed: it already proved itself once before, and a bootloader-
     * verified restore from BACKUP is at least as strong evidence of
     * health as a fresh ota_stage_finish() commit, so there is no reason
     * to spend a retry budget re-confirming it. staged_len tracks the
     * restored image's length (so the "verify the active image before
     * jumping" step in main() keeps working completely unchanged), and
     * staged_sha512 is copied from the now-verified image's own trailer —
     * same "recompute from the trailer fw_verify() just proved" pattern
     * ota_stage_finish() uses, rather than carrying over anything stale
     * from the crash-looping record being replaced. */
    struct fw_trailer trailer;
    memcpy(&trailer, active + (bc->backup_len - sizeof trailer), sizeof trailer);

    struct boot_control restored;
    memset(&restored, 0, sizeof restored);
    restored.staged_len = bc->backup_len;
    memcpy(restored.staged_sha512, trailer.sha512, sizeof restored.staged_sha512);
    restored.flags = BC_F_CONFIRMED;
    restored.boot_attempts = 0;
    /* backup_len/BC_F_BACKUP_VALID intentionally left clear (the memset
     * above): BACKUP now holds the exact same image as active, not a
     * distinct fallback, so there is nothing left to roll back to until
     * the next confirmed-outgoing OTA repopulates it — ota_stage.c's own
     * rule, unaffected by this restore. This is the "rollback of the
     * rollback" gap: a second crash loop right after a restore has no
     * further fallback and halts, same as the first-ever-OTA gap
     * documented in README.md. */

    bc_write(&restored);   /* Return value intentionally unchecked — matches
                             * this bootloader's other bc_write() call site
                             * (main()'s attempt-bump step): a failed write
                             * here leaves the still-intact pre-restore
                             * record in place, so the *next* boot re-reads
                             * the same unconfirmed/over-budget state and
                             * retries this exact rollback again from the
                             * still-untouched BACKUP copy. */

    *bc = restored;
    return BOOT_ACTIVE;
}

/* See commit.h for what this covers and what it deliberately excludes
 * (moved from bootloader/main.c's int main() unchanged, except that it now
 * owns its own bc_read() and returns instead of falling through to the
 * !bc_valid hardware block / jump). */
boot_action_t boot_commit_run(unsigned max_boot_attempts, bool *bc_valid_out) {
    struct boot_control bc;
    bool bc_valid = bc_read(&bc);
    if (bc_valid_out) *bc_valid_out = bc_valid;

    boot_action_t action = boot_decide(&bc, bc_valid, max_boot_attempts);

    /* Set only when try_commit_staging() below clears BC_F_PENDING in `bc`
     * without itself writing it to flash (see that function's doc comment —
     * the write is deliberately deferred to the shared step at the bottom
     * of this function). Tracked as its own bool, rather than diffing
     * bc.flags before/after, so try_rollback()'s own already-persisted
     * record (see below) can never accidentally trigger a second, redundant
     * write here — only a REAL unwritten commit does. This function must
     * fold this into whichever write ends up happening below, or a PENDING
     * flag cleared only in RAM here can silently never reach flash: if the
     * active image the commit just produced happens to already be
     * BC_F_CONFIRMED (the crash-loop-restore case can't reach this, but a
     * hand-crafted/corrupted PENDING|CONFIRMED record could), the old code
     * only ever wrote on the `!(bc.flags & BC_F_CONFIRMED)` branch — so that
     * record would commit successfully and silently EVERY boot, never
     * actually persisting the clear and never bumping boot_attempts,
     * permanently disarming crash-loop rollback for whatever gets
     * committed. */
    bool pending_clear_unwritten = false;

    if (action == BOOT_COMMIT_STAGING) {
        action = try_commit_staging(&bc);
        if (action == BOOT_ACTIVE) pending_clear_unwritten = true;
    }

    if (action == BOOT_ROLLBACK) {
        /* boot_decide() only ever reaches this when the active image is
         * unconfirmed and has crash-looped past max_boot_attempts.
         * TRUE-ROLLBACK PHASE: try_rollback() attempts a real restore from
         * BACKUP (partition_map.h) instead of unconditionally halting —
         * see its doc comment above for the full success/failure story.
         * On success it also updates `bc` in place to the freshly-
         * persisted, CONFIRMED record naming the restored image, so the
         * BOOT_ACTIVE handling just below verifies and jumps to THAT
         * image. try_rollback() already wrote that record itself, so
         * pending_clear_unwritten stays false here — nothing left for the
         * write below to do for this path. */
        action = try_rollback(&bc);
    }

    if (action == BOOT_ACTIVE && bc_valid) {
        /* Verify the active image before jumping — "never run an
         * unverified image" (see README.md). bc.staged_len doubles as "the
         * currently-active image's length" once a commit has happened:
         * ota_stage_finish() sets it when staging, try_commit_staging()
         * above only ever clears BC_F_PENDING (never touches staged_len),
         * and ota_stage_confirm() (src/ota_stage.c) only ever touches
         * flags/boot_attempts — so it still names the active image's exact
         * signed length here, however many boots later. */
        if (!fw_verify(REGION_PTR(APP_OFF), bc.staged_len, VERIFY_PUBKEY)) {
            action = BOOT_STUCK;
        } else {
            /* Whatever RAM-only change try_commit_staging() made above
             * (pending_clear_unwritten) must reach flash on this boot no
             * matter which of the two conditions below actually holds —
             * see pending_clear_unwritten's own doc comment for why. */
            bool need_write = pending_clear_unwritten;
            if (!(bc.flags & BC_F_CONFIRMED)) {
                /* About to boot an unconfirmed image — either freshly
                 * committed just above, or a plain retry still within
                 * budget. Bump boot_attempts now, BEFORE jumping, so a
                 * crash during *this* attempt is visible to the crash-loop
                 * check on the next boot. */
                bc.boot_attempts += 1;
                need_write = true;
            }
            /* Common case (freshly committed and unconfirmed, or a plain
             * unconfirmed retry): both conditions land on the same single
             * bc_write() below — one write, one seq bump, for the whole
             * commit-and-boot sequence, exactly as before this fix. The
             * only new case is an already-CONFIRMED record whose PENDING
             * clear from try_commit_staging() would otherwise be lost: that
             * now still gets its own write here, just without a
             * boot_attempts bump (a confirmed image doesn't own a retry
             * budget). Already-confirmed with nothing pending
             * (need_write == false) still costs no write, same as before. */
            if (need_write) bc_write(&bc);
        }
    }
    /* action == BOOT_ACTIVE && !bc_valid: no boot_control record has ever
     * been written — the pre-OTA factory state boot_decide()'s rule 1
     * exists for. Left entirely to the caller (bootloader/main.c's real
     * main()): the vector-table sanity check + watchdog-armed jump that
     * follow are hardware-only and out of scope here — see commit.h. */

    return action;
}
