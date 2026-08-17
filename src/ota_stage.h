#pragma once
#include <stdint.h>
#include <stdbool.h>

/*
 * ota_stage — core-0 staging writer for pull-based OTA. Appends downloaded
 * image bytes into the STAGING flash region (partition_map.h), erasing each
 * 4 KB sector lazily the first time a write crosses into it and buffering to
 * 256 B flash-page boundaries, and on ota_stage_finish() verifies the
 * complete staged image (fw_verify, fw_image.h) before committing a
 * BC_F_PENDING boot_control record (boot_control.h) for the (future)
 * bootloader to pick up.
 *
 * CROSS-CORE CONTRACT: every function here touches flash and must run ONLY
 * on core 0. Core 1 (network/download) never calls this API directly — it
 * hands bytes across via g_ota_buf below plus the IC_CMD_OTA_* / IC_EVT_OTA_*
 * intercore protocol (intercore.h); src/serial_protocol.c's ic_dispatch_cmd
 * is the sole caller of this module on core 0.
 */

/* Cross-core handoff buffer: core 1 (the sole writer) fills it only between
 * IC_EVT_OTA_ACKs; core 0 (the sole reader) reads it only while handling
 * IC_CMD_OTA_DATA. Lock-free by that strict single-owner-per-phase handoff —
 * see the ACK protocol description in intercore.h. Defined once in
 * ota_stage.c; both cores include this header. */
#define OTA_BUF_SZ 4096u
extern uint8_t g_ota_buf[OTA_BUF_SZ];

/* Reset the staging write offset to 0 — call once at the start of a new OTA
 * download, before any ota_stage_write(). If the current boot_control record
 * is still BC_F_PENDING (a previous ota_stage_finish() staged a verified
 * candidate that hasn't been committed by the bootloader yet — e.g. a
 * re-trigger during the post-success pre-reboot window), that record is
 * invalidated FIRST, before this session ever erases a byte of STAGING: a
 * dangling PENDING record naming a region this session is about to overwrite
 * would otherwise be verified as garbage on the next boot if this new
 * session itself fails partway. See ota_stage.c's ota_stage_begin() for the
 * full reasoning, including how it recovers the real ACTIVE-image length
 * from backup_len so the bootloader's own "about to boot ACTIVE" re-verify
 * step keeps working after the invalidation. */
void ota_stage_begin(void);

/* Append `len` bytes (from g_ota_buf, per the IC_CMD_OTA_DATA contract) at
 * the current staging offset. Erases each 4 KB sector lazily the first time
 * the write crosses into it (sector-aligned), buffers to 256 B page
 * boundaries and flash_range_programs full pages, all inside
 * flash_safe_execute() on target. Returns false on a flash error or if
 * offset + len would exceed STAGING_SIZE; either way the session is left
 * as-is — call ota_stage_abort() to reset. */
bool ota_stage_write(const uint8_t *data, uint32_t len);

/* Flush the final partial page (if any), then fw_verify() the staged image
 * — read back via PART_XIP(STAGING_OFF) — over the first total_len bytes
 * against FW_PUBKEY. On success, builds and bc_write()s a boot_control
 * record: BC_F_PENDING, staged_len = total_len, staged_sha512 copied from
 * the (now-verified) trailer. Returns true only if both the verification
 * and the bc_write() succeeded — a verify failure leaves NO boot_control
 * record written.
 *
 * True-rollback bookkeeping: before building that record, reads the CURRENT
 * boot_control (describing whatever is in the ACTIVE slot right now). If it
 * is valid, describes a real image (staged_len > 0), and that image is
 * BC_F_CONFIRMED, the new record also gets backup_len = that outgoing
 * image's length and BC_F_BACKUP_VALID set — telling a (future-phase)
 * bootloader it may safely copy ACTIVE -> BACKUP before committing this new
 * image. An outgoing image that is missing or not yet confirmed is never
 * backed up (backup_len = 0, BC_F_BACKUP_VALID clear) — a rollback candidate
 * must already have proven itself by booting to confirmation, not merely
 * having been staged. See ota_stage.c's ota_stage_finish() for the full
 * rule and its back-to-back-OTA edge case. */
bool ota_stage_finish(uint32_t total_len);

/* Discard the current staging session (resets the write offset; bytes
 * already flashed are left in place and will simply be overwritten from
 * offset 0 by the next ota_stage_begin()/ota_stage_write() session). */
void ota_stage_abort(void);

/* Mark the currently-active boot_control record confirmed: sets
 * BC_F_CONFIRMED and clears boot_attempts. A no-op if there is no valid
 * record (bc_read() fails). */
void ota_stage_confirm(void);
