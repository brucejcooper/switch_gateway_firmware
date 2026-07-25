#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/*
 * boot_control — the persistent hand-off record between the running app and
 * the (future) bootloader, double-buffered across the two 4 KB boot-control
 * sectors (BOOTCTRL_A_OFF/BOOTCTRL_B_OFF, partition_map.h).
 *
 * Power-safety model: the record is never updated in place. bc_write() always
 * erases+programs the STALE sector — the one that does NOT hold the current
 * latest record — and stamps it seq = latest_seq + 1, so a reset at any point
 * during the write leaves the other sector, still holding the previous fully-
 * valid record, untouched. bc_read() finds the current record by scanning
 * both sectors for a valid magic+crc32 and taking the higher seq; a half-
 * written sector (erase started but program interrupted, or vice versa) fails
 * its crc32 check and is simply ignored.
 */

#define BC_MAGIC 0x424F4F54u   /* "BOOT" */

/* flags */
#define BC_F_PENDING      (1u << 0)  /* staging holds a verified image to commit */
#define BC_F_CONFIRMED    (1u << 1)  /* active image has been proven good */
#define BC_F_BACKUP_VALID (1u << 2)  /* BACKUP region (partition_map.h) holds a
                                       * restorable image — a CONFIRMED image
                                       * that was active immediately before
                                       * the current staged/committed one;
                                       * backup_len is its byte length. Clear
                                       * whenever there is nothing safe to
                                       * roll back to (first-ever OTA over the
                                       * trailer-less factory image, or the
                                       * outgoing image was never itself
                                       * confirmed — see ota_stage.c). */

struct boot_control {
    uint32_t magic;              /* BC_MAGIC */
    uint32_t seq;                /* monotonically increasing; higher wins */
    uint32_t flags;              /* BC_F_* bitmask */
    uint32_t staged_len;         /* byte length of the image staged for commit */
    uint8_t  staged_sha512[64];  /* SHA-512 of the staged image (crypto_sha512) */
    uint32_t boot_attempts;      /* consecutive boot attempts of a pending image */
    /* Byte length of the signed image currently sitting in the BACKUP region,
     * valid only when BC_F_BACKUP_VALID is set (0 when clear). Repurposes one
     * word of the original uint32_t reserved[4] — struct size (104 B) and
     * crc32 coverage are unchanged by this rename. */
    uint32_t backup_len;
    uint32_t reserved[3];        /* zero; keeps the record growable in place */
    uint32_t crc32;              /* CRC-32 over every preceding byte of this struct */
};

/* Standard CRC-32 (reflected, poly 0xEDB88320, init 0xFFFFFFFF, output XORed
 * with 0xFFFFFFFF — the classic zlib/PNG/Ethernet variant).
 * crc32("123456789", 9) == 0xCBF43926. */
uint32_t crc32(const void *data, size_t len);

/* Find the current boot-control record: the highest-seq sector (of the two in
 * partition_map.h) whose magic and crc32 both check out. Returns true and
 * fills *out on success; on failure (both sectors erased/corrupt) zeroes *out
 * and returns false. */
bool bc_read(struct boot_control *out);

/* Persist *in as the new latest record. seq and crc32 in *in are ignored and
 * overwritten: seq is stamped to one past the current latest (1 if neither
 * sector currently holds a valid record), and crc32 is recomputed over the
 * result. The record is written to the sector NOT currently holding the
 * latest record (erase then program, via flash_safe_execute so core 1 is
 * locked out first) — the other sector is never touched, so a power loss
 * mid-write cannot corrupt the last-known-good record. Returns false if the
 * flash write failed. */
bool bc_write(const struct boot_control *in);
