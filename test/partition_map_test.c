/*
 * partition_map_test.c — host-side compile/sanity check for src/partition_map.h.
 *
 * Not part of the firmware build (see CLAUDE.md instructions: host tests live
 * under test/ and are never added to the RP2040 CMake target). Compiles the
 * header standalone with the host cc so its _Static_assert()s run under a
 * plain toolchain too, then re-checks the same invariants at runtime for a
 * human-readable pass/fail instead of just a compiler abort.
 *
 * Build & run:
 *   cc -Wall -Wextra -I ../src -o /tmp/partition_map_test partition_map_test.c \
 *       && /tmp/partition_map_test
 */
#include "../src/partition_map.h"

#include <stdio.h>

int main(void) {
    int fail = 0;

#define CHECK(cond) do { \
        if (!(cond)) { \
            printf("FAIL: %s (%s:%d)\n", #cond, __FILE__, __LINE__); \
            fail = 1; \
        } \
    } while (0)

    /* Contiguous, non-overlapping, tile exactly to 0x200000. */
    CHECK(BOOTLOADER_OFF == 0x000000u);
    CHECK(BOOTLOADER_OFF + BOOTLOADER_SIZE == BOOTCTRL_A_OFF);
    CHECK(BOOTCTRL_A_OFF + BOOTCTRL_A_SIZE == BOOTCTRL_B_OFF);
    CHECK(BOOTCTRL_B_OFF + BOOTCTRL_B_SIZE == APP_OFF);
    CHECK(APP_OFF + APP_SIZE == STAGING_OFF);
    CHECK(STAGING_OFF + STAGING_SIZE == BACKUP_OFF);
    CHECK(BACKUP_OFF + BACKUP_SIZE == FS_OFF);
    CHECK(FS_OFF + FS_SIZE == FLASH_TOTAL_SIZE);
    CHECK(FLASH_TOTAL_SIZE == 0x00200000u);

    /* Sizes match the spec: three EQUAL 576 KB app-sized slots (true-rollback
     * phase re-carve — was one 896 KB active + one 832 KB staging slot). */
    CHECK(BOOTLOADER_SIZE == 0x00E000u);
    CHECK(BOOTCTRL_A_SIZE == 0x001000u);
    CHECK(BOOTCTRL_B_SIZE == 0x001000u);
    CHECK(APP_SIZE        == 0x090000u);
    CHECK(STAGING_SIZE    == 0x090000u);
    CHECK(BACKUP_SIZE     == 0x090000u);
    CHECK(FS_SIZE         == 0x040000u);
    CHECK(APP_SIZE == STAGING_SIZE && STAGING_SIZE == BACKUP_SIZE);
    CHECK(APP_OFF + 3u * APP_SIZE == FS_OFF);

    /* Offsets match the spec exactly. */
    CHECK(APP_OFF     == 0x010000u);
    CHECK(STAGING_OFF == 0x0A0000u);
    CHECK(BACKUP_OFF  == 0x130000u);

    /* littlefs region unchanged (must match pico_enable_filesystem SIZE). */
    CHECK(FS_OFF == 0x1C0000u);
    CHECK(FS_SIZE == 0x40000u);

    /* Every region is 4 KB-sector aligned. */
    CHECK(BOOTLOADER_OFF  % FLASH_SECTOR_SIZE == 0);
    CHECK(BOOTCTRL_A_OFF  % FLASH_SECTOR_SIZE == 0);
    CHECK(BOOTCTRL_B_OFF  % FLASH_SECTOR_SIZE == 0);
    CHECK(APP_OFF         % FLASH_SECTOR_SIZE == 0);
    CHECK(STAGING_OFF     % FLASH_SECTOR_SIZE == 0);
    CHECK(BACKUP_OFF      % FLASH_SECTOR_SIZE == 0);
    CHECK(FS_OFF          % FLASH_SECTOR_SIZE == 0);

    /* XIP helper. */
    CHECK(PART_XIP(APP_OFF) == 0x10010000u);
    CHECK(PART_XIP(STAGING_OFF) == 0x100A0000u);
    CHECK(PART_XIP(BACKUP_OFF) == 0x10130000u);
    CHECK(PART_XIP(FS_OFF) == 0x101C0000u);

    if (!fail) printf("PASS: all partition_map checks passed\n");
    return fail;
}
