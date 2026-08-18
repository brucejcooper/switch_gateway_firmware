/*
 * ota_stage_test.c — host-side test for src/ota_stage.c (core-0 OTA staging
 * writer).
 *
 * Not part of the firmware build (see CLAUDE.md: host tests live under test/
 * and are never added to the RP2040 CMake target). Compiles ota_stage.c
 * itself with -DOTA_STAGE_HOST_TEST (routing every access that would
 * otherwise go through PART_XIP(STAGING_OFF) to the g_staging RAM array
 * below, standing in for the real STAGING flash region) and boot_control.c
 * with -DBOOT_CONTROL_HOST_TEST (its own, pre-existing seam — see
 * boot_control_test.c), so bc_write()'s pending record lands in the two RAM
 * "sectors" here too. Links straight against the real vendored
 * lib/monocypher/{monocypher,monocypher-ed25519}.c and src/fw_image.c to
 * build a REAL signed image — no crypto is reimplemented here, only exercised
 * (same approach as fw_image_test.c).
 *
 * Build & run:
 *   cc -DOTA_STAGE_HOST_TEST -DBOOT_CONTROL_HOST_TEST -Wall -Wextra \
 *       -I ../src -I ../lib/monocypher \
 *       -o /tmp/ota_stage_test ota_stage_test.c \
 *       ../src/ota_stage.c ../src/boot_control.c ../src/fw_image.c \
 *       ../lib/monocypher/monocypher.c ../lib/monocypher/monocypher-ed25519.c \
 *       && /tmp/ota_stage_test
 */
#include "../src/ota_stage.h"
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

/* ── ota_stage.c's OTA_STAGE_HOST_TEST seam: a RAM array standing in for the
 * STAGING flash region (PART_XIP(STAGING_OFF) on target). Real STAGING_SIZE
 * is 576 KB (three-slot re-carve — see partition_map.h); every image this
 * test stages is far smaller, so a much smaller buffer is fine —
 * ota_stage.c's own bounds check is against the real STAGING_SIZE constant
 * regardless of this buffer's size. */
#define TEST_STAGING_CAP (64u * 1024u)
static uint8_t g_staging[TEST_STAGING_CAP];

uint8_t *ota_test_staging_base(void) {
    return g_staging;
}

/* ── ota_stage.c's pubkey seam: FW_PUBKEY (fw_image.c) is a fixed DEAD BEEF
 * placeholder, not a real Ed25519 point, so no signature can verify against
 * it. The host test supplies its own freshly generated test keypair's public
 * key instead (set by main() before the first ota_stage_finish() call). */
static uint8_t g_test_pk[32];
const uint8_t *ota_test_pubkey(void) {
    return g_test_pk;
}

/* ── boot_control.c's pre-existing BOOT_CONTROL_HOST_TEST seam (identical to
 * boot_control_test.c) — two RAM buffers standing in for the BOOTCTRL_A/B
 * sectors. */
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

static void erase_staging(void) {
    memset(g_staging, 0xFF, sizeof g_staging);
}
static void erase_bc(void) {
    memset(g_bc_sector, 0xFF, sizeof g_bc_sector);
}

/* Builds a total_len = img_len + sizeof(struct fw_trailer) signed artifact
 * into buf: img_len bytes of deterministic content, followed by a correctly
 * signed trailer (with the given build_ctr) under sk. Mirrors
 * fw_image_test.c's build_signed_image. */
static uint32_t build_signed_image(uint8_t *buf, uint32_t img_len, uint32_t build_ctr,
                                    const uint8_t sk[64]) {
    for (uint32_t i = 0; i < img_len; i++)
        buf[i] = (uint8_t)(i * 17u + 11u);

    struct fw_trailer t;
    t.magic     = FW_TRAILER_MAGIC;
    t.img_len   = img_len;
    t.build_ctr = build_ctr;
    crypto_sha512(t.sha512, buf, img_len);
    crypto_ed25519_sign(t.sig, sk, (const uint8_t *)&t, offsetof(struct fw_trailer, sig));

    memcpy(buf + img_len, &t, sizeof t);
    return img_len + (uint32_t)sizeof t;
}

/* Feed `buf[0..total_len)` into ota_stage_write() across several odd-sized
 * chunks, deliberately including sizes that don't align to the 256 B page or
 * 4 KB sector granularity (and one exactly sector-sized chunk), so a single
 * ota_stage_write() call — and the run as a whole — crosses both boundaries
 * repeatedly. Returns false (via CHECK) if any call fails. */
static void stream_in_chunks(const uint8_t *buf, uint32_t total_len) {
    static const uint32_t pattern[] = { 777, 4096, 13, 4083, 999, 1, 4095, 250 };
    uint32_t off = 0;
    int i = 0;
    while (off < total_len) {
        uint32_t want = pattern[i % (int)(sizeof pattern / sizeof pattern[0])];
        uint32_t remaining = total_len - off;
        uint32_t chunk = (want < remaining) ? want : remaining;
        CHECK(ota_stage_write(buf + off, chunk) == true);
        off += chunk;
        i++;
    }
}

int main(void) {
    uint8_t seed[32];
    for (int i = 0; i < 32; i++) seed[i] = (uint8_t)(i * 13 + 5);
    uint8_t sk[64], pk[32];
    crypto_ed25519_key_pair(sk, pk, seed);   /* seed is wiped by this call */
    memcpy(g_test_pk, pk, sizeof g_test_pk);   /* ota_stage.c verifies against this */

    static uint8_t g_image[24 * 1024];
    uint32_t img_len = 20000;
    uint32_t total_len = build_signed_image(g_image, img_len, 1, sk);
    CHECK(total_len < TEST_STAGING_CAP);
    CHECK(total_len > 4 * FLASH_SECTOR_SIZE);   /* must span several sectors */

    /* ── (a) valid image: full round trip ────────────────────────────── */
    erase_staging();
    erase_bc();
    ota_stage_begin();
    stream_in_chunks(g_image, total_len);
    CHECK(ota_stage_finish(total_len) == true);

    /* staged bytes match the input exactly */
    CHECK(memcmp(g_staging, g_image, total_len) == 0);

    /* the staged image itself verifies under our test pubkey */
    CHECK(fw_verify(g_staging, total_len, pk) == true);

    /* a pending boot_control record was written with the right metadata */
    struct boot_control bc;
    CHECK(bc_read(&bc) == true);
    CHECK((bc.flags & BC_F_PENDING) != 0);
    CHECK(bc.staged_len == total_len);
    struct fw_trailer trailer;
    CHECK(fw_trailer_at(g_image, total_len, &trailer));
    CHECK(memcmp(bc.staged_sha512, trailer.sha512, sizeof bc.staged_sha512) == 0);
    /* anti-rollback floor: no prior record at all -> floor 0, carried
     * forward unchanged (staging never raises it, only a commit does — see
     * ota_stage.c's ota_stage_finish()). */
    CHECK(bc.min_build_ctr == 0);

    /* ── (b) one-byte-corrupted image: verify fails, no record written ── */
    erase_staging();
    erase_bc();
    CHECK(bc_read(&bc) == false);   /* sanity: nothing valid before this sub-test */

    uint8_t bad_image[24 * 1024];
    memcpy(bad_image, g_image, total_len);
    bad_image[img_len / 2] ^= 0xFF;   /* flip a byte in the image body, not the trailer */

    ota_stage_begin();
    stream_in_chunks(bad_image, total_len);
    CHECK(ota_stage_finish(total_len) == false);

    /* no boot_control record was ever written by the failed finish() */
    CHECK(bc_read(&bc) == false);

    /* ── (c) true-rollback bookkeeping: stage+confirm image A, then stage
     * image B over it -> B's record backs up A's length ────────────────── */
    static uint8_t g_image_b[24 * 1024];
    uint32_t img_len_b = 8000;   /* deliberately a different length from A (20000) */
    uint32_t total_len_b = build_signed_image(g_image_b, img_len_b, 2, sk);
    CHECK(total_len_b != total_len);   /* sanity: A and B really are different sizes */

    erase_staging();
    erase_bc();

    /* Stage + confirm A (mirrors a normal OTA that booted and was proven
     * good) — this is the "outgoing active image" B's finish() will see.
     * ota_stage_confirm() is now a no-op on a PENDING record (see (e)
     * below), so simulate the bootloader's commit step in between — it
     * clears BC_F_PENDING and persists that before the app (and its
     * eventual confirm) ever runs — exactly like the real boot sequence:
     * ota_stage_finish() (PENDING) -> bootloader commit (PENDING cleared)
     * -> app boots -> ota_stage_confirm(). */
    ota_stage_begin();
    stream_in_chunks(g_image, total_len);
    CHECK(ota_stage_finish(total_len) == true);
    CHECK(bc_read(&bc) == true);
    bc.flags &= ~BC_F_PENDING;
    CHECK(bc_write(&bc) == true);
    ota_stage_confirm();
    CHECK(bc_read(&bc) == true);
    CHECK((bc.flags & BC_F_CONFIRMED) != 0);
    CHECK(bc.staged_len == total_len);

    /* Now stage B. A is CONFIRMED and describes a real image, so B's new
     * record must back it up. */
    ota_stage_begin();
    stream_in_chunks(g_image_b, total_len_b);
    CHECK(ota_stage_finish(total_len_b) == true);

    CHECK(bc_read(&bc) == true);
    CHECK((bc.flags & BC_F_PENDING) != 0);
    CHECK(bc.staged_len == total_len_b);
    CHECK((bc.flags & BC_F_BACKUP_VALID) != 0);
    CHECK(bc.backup_len == total_len);   /* A's length, preserved for the bootloader */

    /* ── (d) staging over a NON-confirmed active leaves backup_len=0 /
     * BC_F_BACKUP_VALID clear — an outgoing image that was never proven to
     * boot must never be treated as a rollback candidate ────────────────── */
    erase_staging();
    erase_bc();

    /* Stage A but do NOT confirm it — it sits PENDING, exactly like a
     * back-to-back OTA fired before the first one ever booted. */
    ota_stage_begin();
    stream_in_chunks(g_image, total_len);
    CHECK(ota_stage_finish(total_len) == true);
    CHECK(bc_read(&bc) == true);
    CHECK((bc.flags & BC_F_CONFIRMED) == 0);   /* sanity: still unconfirmed */

    /* Stage B over the still-unconfirmed A. */
    ota_stage_begin();
    stream_in_chunks(g_image_b, total_len_b);
    CHECK(ota_stage_finish(total_len_b) == true);

    CHECK(bc_read(&bc) == true);
    CHECK((bc.flags & BC_F_PENDING) != 0);
    CHECK(bc.staged_len == total_len_b);
    CHECK((bc.flags & BC_F_BACKUP_VALID) == 0);   /* nothing safe to roll back to */
    CHECK(bc.backup_len == 0);

    /* ── (e) confirm on a PENDING record is a no-op ──────────────────────
     * Reachable race (see ota_stage.c's ota_stage_confirm() comment): the
     * healthy-boot confirm one-shot in src/net/ota.c can fire against a
     * record ota_stage_finish() just wrote for a brand-new candidate,
     * before the post-install reboot actually happens. That record
     * describes STAGING, not whatever is running right now — confirming it
     * must be a no-op, not a write that sets BC_F_CONFIRMED on a PENDING
     * record (which would silently disarm crash-loop rollback for whatever
     * eventually gets committed — see bootloader/main.c). */
    erase_staging();
    erase_bc();

    ota_stage_begin();
    stream_in_chunks(g_image, total_len);
    CHECK(ota_stage_finish(total_len) == true);
    CHECK(bc_read(&bc) == true);
    CHECK((bc.flags & BC_F_PENDING) != 0);   /* sanity: freshly staged, still pending */
    uint32_t seq_before = bc.seq;

    ota_stage_confirm();

    CHECK(bc_read(&bc) == true);
    CHECK((bc.flags & BC_F_PENDING) != 0);      /* still pending */
    CHECK((bc.flags & BC_F_CONFIRMED) == 0);    /* NOT confirmed */
    CHECK(bc.seq == seq_before);                /* no flash write happened at all */

    /* ── (f) anti-rollback floor: ota_stage_finish() refuses a candidate
     * whose build_ctr is below the persisted floor, and carries the floor
     * FORWARD UNCHANGED (never raises it — that's the bootloader's job on
     * an actual commit, see bootloader/commit.c) into one that clears it ──
     * This is the app-side, fail-fast half of the anti-rollback pair — see
     * test/boot_commit_test.c for the bootloader's own, authoritative gate,
     * and for the rollback-preserves-the-floor scenario. */
    erase_staging();
    erase_bc();

    /* Seed a record with floor=10 directly, exactly as if a real commit had
     * already raised it that far (this test's job is ota_stage_finish()'s
     * own gate, not how the floor got there). */
    struct boot_control floor_seed;
    memset(&floor_seed, 0, sizeof floor_seed);
    floor_seed.flags = BC_F_CONFIRMED;
    floor_seed.staged_len = 1234;   /* arbitrary, unrelated to this sub-test */
    floor_seed.min_build_ctr = 10;
    CHECK(bc_write(&floor_seed) == true);

    /* candidate build_ctr (5) below the floor (10) -> refused outright, no
     * new record written at all (not even a rejected PENDING one — see
     * ota_stage_finish()'s early return). */
    static uint8_t g_low[24 * 1024];
    uint32_t total_low = build_signed_image(g_low, 3000, 5, sk);
    ota_stage_begin();
    stream_in_chunks(g_low, total_low);
    CHECK(ota_stage_finish(total_low) == false);

    CHECK(bc_read(&bc) == true);
    CHECK(bc.min_build_ctr == 10);              /* floor untouched */
    CHECK((bc.flags & BC_F_PENDING) == 0);      /* still the seeded CONFIRMED record */
    CHECK(bc.staged_len == 1234);               /* nothing about it changed */

    /* candidate build_ctr (10) exactly AT the floor -> accepted; the new
     * PENDING record carries the floor forward unchanged, not raised —
     * ota_stage_finish() never raises it, only a bootloader commit does. */
    static uint8_t g_at[24 * 1024];
    uint32_t total_at = build_signed_image(g_at, 3000, 10, sk);
    ota_stage_begin();
    stream_in_chunks(g_at, total_at);
    CHECK(ota_stage_finish(total_at) == true);

    CHECK(bc_read(&bc) == true);
    CHECK((bc.flags & BC_F_PENDING) != 0);
    CHECK(bc.staged_len == total_at);
    CHECK(bc.min_build_ctr == 10);              /* carried forward, not raised */

    if (fail) {
        printf("FAILED\n");
        return 1;
    }
    printf("all ota_stage tests passed\n");
    return 0;
}
