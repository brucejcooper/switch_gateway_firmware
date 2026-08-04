#pragma once
#include <stdint.h>

/*
 * partition_map — the 2 MB flash layout for OTA (bootloader + A/B boot-control
 * + active/staging/backup app slots + the existing littlefs region), as byte
 * offsets from flash base (0x00000000). XIP addresses are flash base
 * 0x10000000 + offset (see PART_XIP() below).
 *
 * TRUE-ROLLBACK PHASE: carves the app-sized portion of flash (
 * one 896 KB active slot + one 832 KB staging slot) into THREE EQUAL 576 KB
 * slots — active, staging, and a new BACKUP region — so the bootloader can
 * restore the previous good image after a bad commit instead of halting
 * (BOOT_STUCK). The app still links ONCE, at the unchanged APP_OFF/XIP
 * 0x10010000 — this is not A/B dual-linking, just a third flash-only slot the
 * bootloader copies into/out of. BOOTLOADER, BOOTCTRL A/B, and the littlefs
 * region (FS_OFF/FS_SIZE) are all UNCHANGED. 576 KB per slot still dwarfs the
 * ~300 KB firmware image, just with less headroom than the old 896 KB active
 * slot had (traded for the new backup region — that's the whole point of
 * this phase).
 *
 *   bootloader     0x000000 .. 0x00E000  (56 KB)   reserved, not yet built
 *   boot-control A 0x00E000 .. 0x00F000  (4 KB)
 *   boot-control B 0x00F000 .. 0x010000  (4 KB)
 *   active app     0x010000 .. 0x0A0000  (576 KB)  XIP 0x10010000  [UNCHANGED OFFSET]
 *   staging        0x0A0000 .. 0x130000  (576 KB)  XIP 0x100A0000
 *   backup         0x130000 .. 0x1C0000  (576 KB)  XIP 0x10130000  [NEW]
 *   littlefs       0x1C0000 .. 0x200000  (256 KB)  XIP 0x101C0000  [UNCHANGED]
 */

#define FLASH_BASE_ADDR   0x10000000u   /* XIP base */
#define FLASH_TOTAL_SIZE  0x00200000u   /* 2 MB */
#define FLASH_SECTOR_SIZE 0x00001000u   /* 4 KB erase granularity */
#define FLASH_PAGE_SIZE   0x00000100u   /* 256 B program granularity */

/* XIP address for a given flash byte offset. */
#define PART_XIP(off) (FLASH_BASE_ADDR + (off))

#define BOOTLOADER_OFF 0x00000000u
#define BOOTLOADER_SIZE 0x0000E000u   /* 56 KB — reserved only, not yet built */

#define BOOTCTRL_A_OFF 0x0000E000u
#define BOOTCTRL_A_SIZE 0x00001000u  /* 4 KB */

#define BOOTCTRL_B_OFF 0x0000F000u
#define BOOTCTRL_B_SIZE 0x00001000u  /* 4 KB */

#define APP_OFF 0x00010000u
#define APP_SIZE 0x00090000u         /* 576 KB — was 896 KB pre-rollback; see header comment */

#define STAGING_OFF 0x000A0000u
#define STAGING_SIZE 0x00090000u    /* 576 KB — was 832 KB pre-rollback */

/* NEW (true-rollback phase): holds a copy of the previous CONFIRMED active
 * image, made by the bootloader just before it commits a new staged image
 * over the active slot. If the newly-committed image then fails to boot, the
 * bootloader restores BACKUP -> active instead of halting. Populated/consumed
 * only by the bootloader (bootloader/main.c); the app never writes it
 * directly — see boot_control.h's backup_len/BC_F_BACKUP_VALID for how the
 * app records what to back up. */
#define BACKUP_OFF 0x00130000u
#define BACKUP_SIZE 0x00090000u     /* 576 KB — same size as active/staging */

/* Existing littlefs region — MUST match pico_enable_filesystem(SIZE ...) in
 * CMakeLists.txt (currently 262144 = 0x40000) so existing config/mappings
 * survive. Do not move or resize without also updating that call. */
#define FS_OFF 0x001C0000u
#define FS_SIZE 0x00040000u          /* 256 KB */

/* ── (a) regions tile exactly, contiguously, without overlap, across the
 *      whole 2 MB flash ─────────────────────────────────────────────────── */
_Static_assert(BOOTLOADER_OFF == 0x00000000u,
               "bootloader must start at flash base");
_Static_assert(BOOTLOADER_OFF + BOOTLOADER_SIZE == BOOTCTRL_A_OFF,
               "boot-control A must immediately follow bootloader");
_Static_assert(BOOTCTRL_A_OFF + BOOTCTRL_A_SIZE == BOOTCTRL_B_OFF,
               "boot-control B must immediately follow boot-control A");
_Static_assert(BOOTCTRL_B_OFF + BOOTCTRL_B_SIZE == APP_OFF,
               "active app must immediately follow boot-control B");
_Static_assert(APP_OFF + APP_SIZE == STAGING_OFF,
               "staging must immediately follow the active app");
_Static_assert(STAGING_OFF + STAGING_SIZE == BACKUP_OFF,
               "backup must immediately follow staging");
_Static_assert(BACKUP_OFF + BACKUP_SIZE == FS_OFF,
               "littlefs must immediately follow backup");
_Static_assert(FS_OFF + FS_SIZE == FLASH_TOTAL_SIZE,
               "regions must tile exactly to the 2 MB flash size");

/* ── (a2) the three app-sized slots are exactly equal, and together with
 *        APP_OFF tile exactly up to FS_OFF — the "three equal slots below
 *        littlefs" invariant this phase's re-carve is built on ────────────── */
_Static_assert(APP_SIZE == STAGING_SIZE && STAGING_SIZE == BACKUP_SIZE,
               "active/staging/backup must be equal-sized slots");
_Static_assert(APP_OFF + 3u * APP_SIZE == FS_OFF,
               "three equal app-sized slots must tile exactly up to littlefs");

/* ── (b) littlefs region is unchanged from the pre-OTA layout ───────────── */
_Static_assert(FS_OFF == 0x001C0000u,
               "FS_OFF must match the existing littlefs placement (top 256 KB)");
_Static_assert(FS_SIZE == 0x00040000u,
               "FS_SIZE must match pico_enable_filesystem's SIZE in CMakeLists.txt");

/* ── (c) every region offset is 4 KB-sector aligned ──────────────────────── */
_Static_assert(BOOTLOADER_OFF  % FLASH_SECTOR_SIZE == 0, "bootloader offset must be sector-aligned");
_Static_assert(BOOTCTRL_A_OFF  % FLASH_SECTOR_SIZE == 0, "boot-control A offset must be sector-aligned");
_Static_assert(BOOTCTRL_B_OFF  % FLASH_SECTOR_SIZE == 0, "boot-control B offset must be sector-aligned");
_Static_assert(APP_OFF         % FLASH_SECTOR_SIZE == 0, "active app offset must be sector-aligned");
_Static_assert(STAGING_OFF     % FLASH_SECTOR_SIZE == 0, "staging offset must be sector-aligned");
_Static_assert(BACKUP_OFF      % FLASH_SECTOR_SIZE == 0, "backup offset must be sector-aligned");
_Static_assert(FS_OFF          % FLASH_SECTOR_SIZE == 0, "littlefs offset must be sector-aligned");
