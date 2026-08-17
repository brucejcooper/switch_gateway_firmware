/*
 * boot_commit_test.c — host-side SEQUENCE test for the bootloader's
 * commit/rollback orchestration (bootloader/commit.c: try_commit_staging(),
 * try_rollback(), boot_commit_run()).
 *
 * Why this exists: boot_decide_test.c exhaustively covers the PURE decision
 * table (boot_decide.c) — "given this boot_control record, what should
 * happen" — but nothing before this file exercised try_commit_staging()/
 * try_rollback() as a SEQUENCE across simulated boots, including a boot
 * "crashing" partway through. Both real power-cut bugs this design has had
 * (the commit-retry backup clobber fixed by 9620301's trailer gate, and the
 * PENDING|CONFIRMED persist fixed by ae1f661) live exactly in that
 * sequencing, not in the pure decision table — a test that only calls each
 * function once, in isolation, can't see either one.
 *
 * Host-test seam: matches src/boot_control.c's BOOT_CONTROL_HOST_TEST and
 * src/ota_stage.c's OTA_STAGE_HOST_TEST pattern. bootloader/commit.c is
 * compiled with -DBOOT_COMMIT_HOST_TEST, which redirects every access that
 * would otherwise go through PART_XIP(STAGING_OFF/APP_OFF/BACKUP_OFF) to
 * bc_commit_test_region() below — one big RAM array (g_flash) standing in
 * for the WHOLE flash address space, indexed exactly like partition_map.h's
 * byte offsets, so STAGING/APP/BACKUP are just three windows into the same
 * buffer, same as on real hardware. fw_verify() calls are redirected from
 * the real (unusable — placeholder, no matching private key) FW_PUBKEY to
 * bc_commit_test_pubkey(), a freshly generated test keypair, same reasoning
 * as ota_stage_test.c's identical seam. boot_control.c is linked with its
 * own pre-existing -DBOOT_CONTROL_HOST_TEST seam (two RAM "sectors").
 *
 * Scope note: this test constructs boot_control PENDING/backup records
 * directly via bc_write() (mirroring, in a few lines, exactly what
 * ota_stage_finish() computes) rather than linking src/ota_stage.c and its
 * own OTA_STAGE_HOST_TEST seam. ota_stage_finish()'s bookkeeping is already
 * exhaustively covered by test/ota_stage_test.c; entangling two independent
 * host-test seams here would add risk (interacting #defines, two RAM
 * flash-stand-ins that must be kept in sync) for no additional coverage —
 * this file's job is the BOOTLOADER side of the contract, taking
 * ota_stage_finish()'s output shape as already proven.
 *
 * Build & run:
 *   cc -DBOOT_COMMIT_HOST_TEST -DBOOT_CONTROL_HOST_TEST -Wall -Wextra -std=c11 \
 *       -I ../bootloader -I ../src -I ../lib/monocypher \
 *       -o /tmp/boot_commit_test boot_commit_test.c \
 *       ../bootloader/commit.c ../bootloader/boot_decide.c \
 *       ../src/boot_control.c ../src/fw_image.c \
 *       ../lib/monocypher/monocypher.c ../lib/monocypher/monocypher-ed25519.c \
 *       && /tmp/boot_commit_test
 */
#include "../bootloader/commit.h"
#include "../bootloader/boot_decide.h"
#include "../src/boot_control.h"
#include "../src/fw_image.h"
#include "../src/partition_map.h"
#include "monocypher.h"
#include "monocypher-ed25519.h"

#include <stdio.h>
#include <string.h>
#include <stddef.h>

static int fail = 0;
#define CHECK(cond) do { \
        if (!(cond)) { \
            printf("FAIL: %s (%s:%d)\n", #cond, __FILE__, __LINE__); \
            fail = 1; \
        } \
    } while (0)

/* ── bootloader/commit.c's BOOT_COMMIT_HOST_TEST seam ────────────────────
 * One flat RAM array standing in for the entire 2 MB flash address space,
 * indexed exactly like partition_map.h's byte offsets (PART_XIP(off) on
 * target becomes &g_flash[off] here) — so STAGING_OFF/APP_OFF/BACKUP_OFF
 * are just three windows into the one buffer, matching how they really are
 * three windows into the one physical flash chip. */
static uint8_t g_flash[FLASH_TOTAL_SIZE];

uint8_t *bc_commit_test_region(uint32_t off) {
    return g_flash + off;
}

/* ── commit.c's pubkey seam (identical reasoning to ota_stage_test.c's) ── */
static uint8_t g_test_pk[32];
const uint8_t *bc_commit_test_pubkey(void) {
    return g_test_pk;
}

/* ── boot_control.c's pre-existing BOOT_CONTROL_HOST_TEST seam (identical
 * to boot_control_test.c / ota_stage_test.c) — two RAM buffers standing in
 * for the BOOTCTRL_A/B sectors. */
#define BC_SECTOR_BYTES 4096u
static uint8_t g_bc_sector[2][BC_SECTOR_BYTES];

const uint8_t *bc_test_sector_base(int idx) {
    return g_bc_sector[idx];
}
void bc_test_sector_write(int idx, const void *data, size_t len) {
    memset(g_bc_sector[idx], 0xFF, BC_SECTOR_BYTES);
    memcpy(g_bc_sector[idx], data, len);
}

/* ── helpers ─────────────────────────────────────────────────────────── */

static uint8_t g_sk[64], g_pk[32];

static void reset_world(void) {
    memset(g_flash, 0xFF, sizeof g_flash);
    memset(g_bc_sector, 0xFF, sizeof g_bc_sector);
}

static uint8_t *region(uint32_t off) { return bc_commit_test_region(off); }

/* Builds a total_len = img_len + sizeof(struct fw_trailer) signed artifact
 * into buf: img_len deterministic-but-salted bytes, followed by a correctly
 * signed trailer under g_sk. `salt` lets two images of the SAME img_len
 * still differ byte-for-byte (needed for the equal-length backup-clobber
 * scenario) — mirrors ota_stage_test.c's build_signed_image, plus salt. */
static uint32_t build_signed_image(uint8_t *buf, uint32_t img_len, uint8_t salt) {
    for (uint32_t i = 0; i < img_len; i++)
        buf[i] = (uint8_t)((i * 17u + 11u) ^ salt);

    struct fw_trailer t;
    t.magic   = FW_TRAILER_MAGIC;
    t.img_len = img_len;
    crypto_sha512(t.sha512, buf, img_len);
    crypto_ed25519_sign(t.sig, g_sk, (const uint8_t *)&t, offsetof(struct fw_trailer, sig));

    memcpy(buf + img_len, &t, sizeof t);
    return img_len + (uint32_t)sizeof t;
}

static void place(uint32_t off, const uint8_t *buf, uint32_t len) {
    memcpy(region(off), buf, len);
}

/* Seed "the device is already running this CONFIRMED image" directly: write
 * its bytes into ACTIVE and persist a matching CONFIRMED boot_control
 * record. Skips replaying the bootstrap/first-OTA history that got it
 * there — a legitimate simplification, same as ota_stage_test.c seeding
 * confirmed state directly for its own true-rollback-bookkeeping tests. */
static void seed_confirmed_active(const uint8_t *img, uint32_t total_len) {
    place(APP_OFF, img, total_len);
    struct fw_trailer t;
    memcpy(&t, img + total_len - sizeof t, sizeof t);

    struct boot_control bc;
    memset(&bc, 0, sizeof bc);
    bc.flags      = BC_F_CONFIRMED;
    bc.staged_len = total_len;
    memcpy(bc.staged_sha512, t.sha512, sizeof bc.staged_sha512);
    CHECK(bc_write(&bc));
}

/* Stage a candidate exactly like ota_stage_finish() does (src/ota_stage.c,
 * already exhaustively tested by ota_stage_test.c — reproduced here in
 * miniature per this file's header "Scope note"): write it into STAGING,
 * read whatever boot_control currently describes as ACTIVE, and set
 * BC_F_BACKUP_VALID/backup_len on the new PENDING record iff that outgoing
 * image is itself BC_F_CONFIRMED. */
static void stage_candidate(const uint8_t *img, uint32_t total_len) {
    place(STAGING_OFF, img, total_len);

    struct boot_control old;
    bool have_old = bc_read(&old);

    struct fw_trailer t;
    memcpy(&t, img + total_len - sizeof t, sizeof t);

    struct boot_control bc;
    memset(&bc, 0, sizeof bc);
    bc.flags      = BC_F_PENDING;
    bc.staged_len = total_len;
    memcpy(bc.staged_sha512, t.sha512, sizeof bc.staged_sha512);
    if (have_old && old.staged_len > 0 && (old.flags & BC_F_CONFIRMED)) {
        bc.flags |= BC_F_BACKUP_VALID;
        bc.backup_len = old.staged_len;
    }
    CHECK(bc_write(&bc));
}

/* ── (a) normal commit ──────────────────────────────────────────────────
 * Freshly-staged candidate, no prior history at all (bc_valid was false).
 * A single boot must commit it, verify it, and persist attempts=1 with
 * PENDING cleared — the baseline every other scenario below builds on. */
static void test_normal_commit(void) {
    reset_world();

    static uint8_t imgA[8192];
    uint32_t total_a = build_signed_image(imgA, 5000, 0xA1);
    stage_candidate(imgA, total_a);   /* no confirmed prior active -> no backup */

    bool bc_valid = false;
    boot_action_t action = boot_commit_run(3, &bc_valid);
    CHECK(action == BOOT_ACTIVE);
    CHECK(bc_valid == true);
    CHECK(memcmp(region(APP_OFF), imgA, total_a) == 0);

    struct boot_control bc;
    CHECK(bc_read(&bc));
    CHECK(!(bc.flags & BC_F_PENDING));
    CHECK(!(bc.flags & BC_F_CONFIRMED));
    CHECK(!(bc.flags & BC_F_BACKUP_VALID));   /* nothing was confirmed to back up */
    CHECK(bc.boot_attempts == 1);
    CHECK(bc.staged_len == total_a);
}

/* ── (b)/(c) commit interrupted after the staging->active copy but before
 * main()'s deferred bc_write() — retry must NOT clobber BACKUP (the
 * 9620301 trailer gate). Parameterized on whether X and the candidate B
 * happen to be the SAME length: that's exactly the historical bug case
 * (backup_len == staged_len let a naive length-only check misfire — see
 * try_commit_staging()'s doc comment in commit.c). */
static void test_interrupted_commit_no_clobber(bool equal_lengths) {
    reset_world();

    static uint8_t imgX[8192], imgB[8192];
    uint32_t total_x = build_signed_image(imgX, 6000, 0xB2);
    uint32_t total_b = build_signed_image(imgB, equal_lengths ? 6000 : 4000, 0xC3);
    CHECK((total_x == total_b) == equal_lengths);   /* sanity: got the case we asked for */

    seed_confirmed_active(imgX, total_x);
    stage_candidate(imgB, total_b);   /* X is CONFIRMED -> backup_len = total_x */

    struct boot_control bc_before;
    CHECK(bc_read(&bc_before));
    CHECK((bc_before.flags & BC_F_BACKUP_VALID) != 0);
    CHECK(bc_before.backup_len == total_x);

    /* Boot #1: call try_commit_staging() directly and DO NOT persist its
     * result — this is exactly "crashed after the copy, before the
     * deferred bc_write()" (see commit.c's pending_clear_unwritten doc
     * comment / bootloader/README.md's "interrupted-commit gate"). */
    struct boot_control bc1 = bc_before;
    boot_action_t a1 = try_commit_staging(&bc1);
    CHECK(a1 == BOOT_ACTIVE);
    CHECK(memcmp(region(APP_OFF), imgB, total_b) == 0);     /* candidate committed */
    CHECK(memcmp(region(BACKUP_OFF), imgX, total_x) == 0);  /* X backed up */

    static uint8_t backup_snapshot[8192];
    memcpy(backup_snapshot, region(BACKUP_OFF), total_x);

    /* Boot #2: nothing was ever persisted after boot #1, so a fresh
     * bc_read() (inside boot_commit_run()) still returns the OLD PENDING
     * record naming candidate B — the real retry-boot scenario. */
    bool bc_valid = false;
    boot_action_t a2 = boot_commit_run(3, &bc_valid);
    CHECK(a2 == BOOT_ACTIVE);
    CHECK(bc_valid == true);

    /* The whole point: BACKUP must be BYTE-IDENTICAL to what boot #1 made —
     * not re-copied (which, per the historical bug, could silently copy the
     * CANDIDATE over the real backup instead), not torn. */
    CHECK(memcmp(region(BACKUP_OFF), backup_snapshot, total_x) == 0);
    CHECK(memcmp(region(APP_OFF), imgB, total_b) == 0);

    struct boot_control bc2;
    CHECK(bc_read(&bc2));
    CHECK(!(bc2.flags & BC_F_PENDING));
    CHECK(bc2.boot_attempts == 1);   /* exactly one real boot's worth of bump */
}

/* ── (d) torn active copy (verify fails) — backup must be skipped ────────
 * Fabricates a mid-copy interruption at the staging->active step itself
 * (commit.c's interruption point (b)): ACTIVE ends up a mix of the NEW
 * candidate's head and the OLD outgoing image's tail (including its
 * trailer, so the trailer-based "already done" gate correctly reads false).
 * fw_verify(ACTIVE, backup_len) must fail on that torn mix, so the backup
 * step must be skipped entirely rather than copying garbage over BACKUP —
 * and the commit must still self-heal (the staging->active copy always
 * re-runs in full on a retry, there is no partial-resume). */
static void test_torn_active_skips_backup(void) {
    reset_world();

    static uint8_t imgX[8192], imgB[8192];
    uint32_t total_x = build_signed_image(imgX, 6000, 0xD4);
    uint32_t total_b = build_signed_image(imgB, 6000, 0xE5);   /* same length, different bytes */
    CHECK(total_x == total_b);

    seed_confirmed_active(imgX, total_x);
    stage_candidate(imgB, total_b);

    /* Tear ACTIVE: first half already B, second half (incl. the trailer)
     * still X — "the copy hasn't yet reached the tail sector". */
    uint32_t half = total_b / 2;
    memcpy(region(APP_OFF), imgB, half);
    memcpy(region(APP_OFF) + half, imgX + half, total_x - half);

    static uint8_t backup_snapshot[8192];
    memcpy(backup_snapshot, region(BACKUP_OFF), sizeof backup_snapshot);   /* still 0xFF: erased */

    bool bc_valid = false;
    boot_action_t action = boot_commit_run(3, &bc_valid);

    /* Backup must be untouched (skipped, not corrupted with the torn mix) — */
    CHECK(memcmp(region(BACKUP_OFF), backup_snapshot, sizeof backup_snapshot) == 0);
    /* — while the commit itself self-heals: the staging->active copy always
     * re-runs in full, so the tear is completely overwritten and the boot
     * still succeeds with a byte-exact candidate in ACTIVE. */
    CHECK(action == BOOT_ACTIVE);
    CHECK(bc_valid == true);
    CHECK(memcmp(region(APP_OFF), imgB, total_b) == 0);
}

/* ── (e) PENDING|CONFIRMED record — commit persists the clear (ae1f661) ──
 * A record with BOTH flags set shouldn't arise from ota_stage_finish() in
 * practice (it never sets BC_F_CONFIRMED), but a bootloader must not trust
 * "shouldn't happen" (see try_commit_staging()'s own bounds-check comment).
 * Before ae1f661, main() only ever wrote boot_control on the
 * `!(flags & BC_F_CONFIRMED)` branch, so a PENDING clear against an
 * ALREADY-CONFIRMED record silently never reached flash — permanently
 * disarming crash-loop rollback for whatever got committed. */
static void test_pending_confirmed_persists_clear(void) {
    reset_world();

    static uint8_t imgC[8192];
    uint32_t total_c = build_signed_image(imgC, 5000, 0xF6);
    place(STAGING_OFF, imgC, total_c);

    struct fw_trailer t;
    memcpy(&t, imgC + total_c - sizeof t, sizeof t);

    struct boot_control bc;
    memset(&bc, 0, sizeof bc);
    bc.flags        = BC_F_PENDING | BC_F_CONFIRMED;
    bc.staged_len   = total_c;
    bc.boot_attempts = 7;   /* arbitrary nonzero: proves CONFIRMED skips the bump below */
    memcpy(bc.staged_sha512, t.sha512, sizeof bc.staged_sha512);
    CHECK(bc_write(&bc));

    struct boot_control before;
    CHECK(bc_read(&before));
    uint32_t seq_before = before.seq;

    bool bc_valid = false;
    boot_action_t action = boot_commit_run(3, &bc_valid);
    CHECK(action == BOOT_ACTIVE);
    CHECK(memcmp(region(APP_OFF), imgC, total_c) == 0);

    struct boot_control after;
    CHECK(bc_read(&after));
    CHECK(!(after.flags & BC_F_PENDING));     /* the clear reached flash this time */
    CHECK((after.flags & BC_F_CONFIRMED) != 0);
    CHECK(after.boot_attempts == 7);          /* confirmed image: no attempt bump */
    CHECK(after.seq > seq_before);            /* and a write really did happen */
}

/* ── (f) crash-loop -> rollback restores BACKUP ──────────────────────────
 * W confirmed and backed up when X was staged over it; X never confirms.
 * With the Fix-4 off-by-one correction (boot_decide.c's `>=`), X gets
 * EXACTLY 3 real boots (attempts 0,1,2 all BOOT_ACTIVE) before the 4th
 * boot (attempts==3) rolls back — this doubles as a regression test for
 * that fix, at the orchestration level rather than boot_decide()'s pure
 * table. */
static void test_crash_loop_restores_backup(void) {
    reset_world();

    static uint8_t imgW[8192], imgX[8192];
    uint32_t total_w = build_signed_image(imgW, 5000, 0x17);
    uint32_t total_x = build_signed_image(imgX, 5500, 0x28);

    seed_confirmed_active(imgW, total_w);
    stage_candidate(imgX, total_x);   /* W confirmed -> backup_len = total_w */

    bool v; boot_action_t act;
    struct boot_control bc;

    act = boot_commit_run(3, &v);   /* boot A: commits X */
    CHECK(act == BOOT_ACTIVE);
    CHECK(bc_read(&bc));
    CHECK(bc.boot_attempts == 1);
    CHECK(memcmp(region(APP_OFF), imgX, total_x) == 0);
    CHECK(memcmp(region(BACKUP_OFF), imgW, total_w) == 0);   /* W backed up */

    act = boot_commit_run(3, &v);   /* boot B: attempts 1 -> 2, still within budget */
    CHECK(act == BOOT_ACTIVE);
    CHECK(bc_read(&bc));
    CHECK(bc.boot_attempts == 2);
    CHECK(memcmp(region(APP_OFF), imgX, total_x) == 0);   /* still X: no rollback yet */

    act = boot_commit_run(3, &v);   /* boot C: attempts 2 -> 3, still within budget */
    CHECK(act == BOOT_ACTIVE);
    CHECK(bc_read(&bc));
    CHECK(bc.boot_attempts == 3);
    CHECK(memcmp(region(APP_OFF), imgX, total_x) == 0);   /* still X */

    act = boot_commit_run(3, &v);   /* boot D: attempts==3 >= max -> ROLLBACK */
    CHECK(act == BOOT_ACTIVE);      /* resolved to a successful restore */
    CHECK(bc_read(&bc));
    CHECK(bc.flags == BC_F_CONFIRMED);   /* exactly confirmed; no PENDING/BACKUP_VALID */
    CHECK(bc.boot_attempts == 0);
    CHECK(bc.staged_len == total_w);
    CHECK(memcmp(region(APP_OFF), imgW, total_w) == 0);   /* W restored, not X */
}

int main(void) {
    uint8_t seed[32];
    for (int i = 0; i < 32; i++) seed[i] = (uint8_t)(i * 13 + 7);
    crypto_ed25519_key_pair(g_sk, g_pk, seed);   /* seed is wiped by this call */
    memcpy(g_test_pk, g_pk, sizeof g_test_pk);   /* commit.c verifies against this */

    test_normal_commit();
    test_interrupted_commit_no_clobber(false);   /* (b): different lengths */
    test_interrupted_commit_no_clobber(true);    /* (c): equal lengths (the historical bug case) */
    test_torn_active_skips_backup();             /* (d) */
    test_pending_confirmed_persists_clear();     /* (e) */
    test_crash_loop_restores_backup();           /* (f) */

    if (fail) {
        printf("FAILED\n");
        return 1;
    }
    printf("all boot_commit tests passed\n");
    return 0;
}
