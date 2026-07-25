/*
 * boot_control_test.c — host-side test for src/boot_control.c.
 *
 * Not part of the firmware build (see CLAUDE.md: host tests live under test/
 * and are never added to the RP2040 CMake target). Compiles boot_control.c
 * itself with -DBOOT_CONTROL_HOST_TEST, which routes its sector reads/writes
 * through the two hook functions below instead of hardware/flash.h +
 * flash_safe_execute() — neither of which exist on the host. The two RAM
 * buffers here stand in for the BOOTCTRL_A/BOOTCTRL_B flash sectors, "erased"
 * (0xFF, matching real NOR flash) at the start of each scenario.
 *
 * Build & run:
 *   cc -DBOOT_CONTROL_HOST_TEST -Wall -Wextra -I ../src \
 *       -o /tmp/boot_control_test boot_control_test.c ../src/boot_control.c \
 *       && /tmp/boot_control_test
 */
#include "../src/boot_control.h"

#include <stdio.h>
#include <string.h>

#define SECTOR_BYTES 4096u

static uint8_t g_sector[2][SECTOR_BYTES];

/* ── the seam boot_control.c calls when built with -DBOOT_CONTROL_HOST_TEST ── */
const uint8_t *bc_test_sector_base(int idx) {
    return g_sector[idx];
}
void bc_test_sector_write(int idx, const void *data, size_t len) {
    memset(g_sector[idx], 0xFF, SECTOR_BYTES);  /* mimic a real sector erase */
    memcpy(g_sector[idx], data, len);
}

static int fail = 0;
#define CHECK(cond) do { \
        if (!(cond)) { \
            printf("FAIL: %s (%s:%d)\n", #cond, __FILE__, __LINE__); \
            fail = 1; \
        } \
    } while (0)

static void erase_all(void) {
    memset(g_sector, 0xFF, sizeof g_sector);
}

/* Write a structurally-valid record directly into a sector buffer, bypassing
 * bc_write() — this is what lets the test drive bc_read() in isolation. */
static void seed_sector(int idx, uint32_t seq, uint32_t flags) {
    struct boot_control rec;
    memset(&rec, 0, sizeof rec);
    rec.magic = BC_MAGIC;
    rec.seq   = seq;
    rec.flags = flags;
    rec.crc32 = crc32(&rec, offsetof(struct boot_control, crc32));
    memset(g_sector[idx], 0xFF, SECTOR_BYTES);
    memcpy(g_sector[idx], &rec, sizeof rec);
}

int main(void) {
    /* 1. crc32 known-answer test (the standard CRC-32/ISO-HDLC check string). */
    CHECK(crc32("123456789", 9) == 0xCBF43926u);

    /* 2. Both sectors erased -> bc_read fails and zeroes *out. */
    erase_all();
    struct boot_control rec;
    memset(&rec, 0x55, sizeof rec);  /* poison, so a false "success" is visible */
    CHECK(bc_read(&rec) == false);
    struct boot_control zero;
    memset(&zero, 0, sizeof zero);
    CHECK(memcmp(&rec, &zero, sizeof rec) == 0);

    /* 3. Two valid records, different seq -> bc_read picks the higher seq,
     *    regardless of which physical sector (A or B) holds it. */
    erase_all();
    seed_sector(0, 5, BC_F_CONFIRMED);
    seed_sector(1, 7, BC_F_PENDING);
    CHECK(bc_read(&rec) == true);
    CHECK(rec.seq == 7);
    CHECK(rec.flags == BC_F_PENDING);

    erase_all();
    seed_sector(0, 7, BC_F_PENDING);
    seed_sector(1, 5, BC_F_CONFIRMED);
    CHECK(bc_read(&rec) == true);
    CHECK(rec.seq == 7);
    CHECK(rec.flags == BC_F_PENDING);

    /* 4. A bad crc32 on the higher-seq sector is rejected; the still-valid
     *    lower-seq sector is returned instead. */
    erase_all();
    seed_sector(0, 5, BC_F_CONFIRMED);
    seed_sector(1, 7, BC_F_PENDING);
    g_sector[1][0] ^= 0xFF;  /* corrupt sector B's magic/crc-covered bytes */
    CHECK(bc_read(&rec) == true);
    CHECK(rec.seq == 5);

    /* 5. Both sectors have a bad crc32 -> bc_read fails and zeroes *out. */
    g_sector[0][0] ^= 0xFF;
    memset(&rec, 0x55, sizeof rec);
    CHECK(bc_read(&rec) == false);
    CHECK(memcmp(&rec, &zero, sizeof rec) == 0);

    /* 6. bc_write end-to-end: from erased flash, the first write lands in
     *    sector 0 with seq 1. Also covers backup_len/BC_F_BACKUP_VALID (the
     *    true-rollback fields carved out of the old reserved[4]) — they
     *    round-trip through bc_write()/bc_read() exactly like any other
     *    field, since they're just more struct bytes under the same crc32. */
    erase_all();
    struct boot_control in;
    memset(&in, 0, sizeof in);
    in.flags = BC_F_PENDING | BC_F_BACKUP_VALID;
    in.staged_len = 12345;
    memset(in.staged_sha512, 0xAB, sizeof in.staged_sha512);
    in.backup_len = 54321;
    CHECK(bc_write(&in) == true);
    CHECK(bc_read(&rec) == true);
    CHECK(rec.seq == 1);
    CHECK(rec.flags == (BC_F_PENDING | BC_F_BACKUP_VALID));
    CHECK(rec.staged_len == 12345);
    CHECK(memcmp(rec.staged_sha512, in.staged_sha512, sizeof rec.staged_sha512) == 0);
    CHECK(rec.backup_len == 54321);

    /* 7. Power-safety: bc_write always targets the STALE sector, so the
     *    sector holding the current latest record is byte-for-byte untouched
     *    by the write — a reset mid-write can't corrupt the last-known-good
     *    record. Snapshot the (now-latest) sector 0 before a second write,
     *    then confirm it is unchanged and the new record (seq 2) lives in
     *    the other sector. */
    uint8_t snapshot0[SECTOR_BYTES];
    memcpy(snapshot0, g_sector[0], SECTOR_BYTES);
    in.flags = BC_F_CONFIRMED;   /* backup_len (54321) carried over from test 6, unchanged */
    CHECK(bc_write(&in) == true);
    CHECK(memcmp(g_sector[0], snapshot0, SECTOR_BYTES) == 0);
    CHECK(bc_read(&rec) == true);
    CHECK(rec.seq == 2);
    CHECK(rec.flags == BC_F_CONFIRMED);
    CHECK(rec.backup_len == 54321);   /* backup_len survives a write that doesn't touch it */

    /* A third write flips back to sector 0 (now the stale one), leaving
     * sector 1 (seq 2) untouched. This one clears backup_len/BC_F_BACKUP_VALID
     * (the "nothing safe to roll back to" state) to confirm that path too. */
    uint8_t snapshot1[SECTOR_BYTES];
    memcpy(snapshot1, g_sector[1], SECTOR_BYTES);
    in.backup_len = 0;
    CHECK(bc_write(&in) == true);
    CHECK(memcmp(g_sector[1], snapshot1, SECTOR_BYTES) == 0);
    CHECK(bc_read(&rec) == true);
    CHECK(rec.seq == 3);
    CHECK((rec.flags & BC_F_BACKUP_VALID) == 0);
    CHECK(rec.backup_len == 0);

    if (!fail) printf("PASS: all boot_control checks passed\n");
    return fail;
}
