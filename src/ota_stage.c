/*
 * ota_stage.c — core-0 OTA staging writer. See ota_stage.h for the API and
 * the cross-core contract (this file must run only on core 0).
 *
 * Host-test seam (matches src/boot_control.c's BOOT_CONTROL_HOST_TEST and
 * src/net/dns_resolve.c's DNS_RESOLVE_HOST_TEST): everything touching
 * hardware/flash.h + flash_safe_execute() lives only under
 * `#ifndef OTA_STAGE_HOST_TEST`. Compiled with -DOTA_STAGE_HOST_TEST, the
 * single hook below (ota_test_staging_base()) redirects every access that
 * would otherwise go through PART_XIP(STAGING_OFF) to a plain RAM array
 * supplied by the host test, standing in for the STAGING flash region; the
 * sector/page bookkeeping logic that actually matters (lazy erase, page
 * buffering, offset accounting) runs identically on host and target.
 */

#include "ota_stage.h"
#include "partition_map.h"
#include "boot_control.h"
#include "fw_image.h"

#include <string.h>

#ifdef OTA_STAGE_HOST_TEST
/* Host seam: STAGING is a plain RAM buffer (STAGING_SIZE bytes) owned by the
 * host test, standing in for the real flash region addressed by
 * PART_XIP(STAGING_OFF). */
extern uint8_t *ota_test_staging_base(void);

/* Host seam: fw_image.h's FW_PUBKEY is a fixed DEAD BEEF placeholder, not a
 * valid Ed25519 curve point — no private key can produce a signature that
 * verifies against it, so the host test cannot exercise a *successful*
 * ota_stage_finish() against the real constant (and casting away its `const`
 * to overwrite it in place is undefined behaviour — it typically lives in a
 * read-only-mapped section and would just crash). The host test instead
 * supplies its own freshly generated test keypair's public key here. This
 * indirection compiles away to nothing but the real FW_PUBKEY on target. */
extern const uint8_t *ota_test_pubkey(void);
#define OTA_VERIFY_PUBKEY (ota_test_pubkey())
#else
#define OTA_VERIFY_PUBKEY (FW_PUBKEY)
/* partition_map.h defines its own FLASH_SECTOR_SIZE/FLASH_PAGE_SIZE as plain
 * numeric literals (host-compilable); undef before pulling in the SDK's
 * copies so the two identical-value definitions don't warn (same reasoning
 * as boot_control.c). */
#undef FLASH_SECTOR_SIZE
#undef FLASH_PAGE_SIZE
#include "hardware/flash.h"
#include "pico/flash.h"
#endif

/* ── the cross-core handoff buffer (see ota_stage.h) ─────────────────────── */
uint8_t g_ota_buf[OTA_BUF_SZ];

/* ── staging-write session state (core 0 only; one session in flight) ────
 * Invariant: s_offset is always flash-page-aligned and is the number of
 * bytes already programmed into STAGING; s_buf_len (< FLASH_PAGE_SIZE) is
 * the number of further bytes buffered in s_page, not yet flushed. The
 * session's total logical length so far is s_offset + s_buf_len. */
static uint32_t s_offset;
static uint32_t s_buf_len;
static uint32_t s_erased_upto;   /* [0, s_erased_upto) of STAGING erased this session */
static uint8_t  s_page[FLASH_PAGE_SIZE];

static uint8_t *staging_base(void) {
#ifdef OTA_STAGE_HOST_TEST
    return ota_test_staging_base();
#else
    return (uint8_t *)PART_XIP(STAGING_OFF);
#endif
}

#ifndef OTA_STAGE_HOST_TEST
struct ota_flash_args {
    uint32_t offset;         /* absolute flash byte offset */
    const uint8_t *buf;
    uint32_t len;
    bool erase;
};

/* Runs under flash_safe_execute(): core 1 is the registered lockout victim
 * and is parked for the duration (see boot_control.c's identical pattern). */
static void ota_flash_op(void *param) {
    const struct ota_flash_args *a = param;
    if (a->erase) flash_range_erase(a->offset, FLASH_SECTOR_SIZE);
    flash_range_program(a->offset, a->buf, a->len);
}
#endif

/* Ensure the sector covering the page at STAGING-relative offset `off`
 * (page-aligned) is erased — lazily, the first time a write crosses into it
 * — then program `len` (<= FLASH_PAGE_SIZE) bytes there. Returns false on a
 * flash-safe-execute failure. */
static bool flush_page(uint32_t off, const uint8_t *buf, uint32_t len) {
    bool need_erase = (off >= s_erased_upto);

#ifdef OTA_STAGE_HOST_TEST
    if (need_erase) {
        memset(staging_base() + s_erased_upto, 0xFF, FLASH_SECTOR_SIZE);
        s_erased_upto += FLASH_SECTOR_SIZE;
    }
    memcpy(staging_base() + off, buf, len);
    return true;
#else
    if (need_erase) s_erased_upto += FLASH_SECTOR_SIZE;
    struct ota_flash_args args = {
        .offset = STAGING_OFF + off,
        .buf    = buf,
        .len    = len,
        .erase  = need_erase,
    };
    /* 4 KB erase (~40 ms typical on-board W25Q) + one 256 B page program;
     * 300 ms leaves ample margin without stalling the lockout handshake for
     * long if something is wrong (matches boot_control.c's reasoning). */
    int rc = flash_safe_execute(ota_flash_op, &args, 300);
    return rc == PICO_OK;
#endif
}

void ota_stage_begin(void) {
    s_offset = 0;
    s_buf_len = 0;
    s_erased_upto = 0;
}

bool ota_stage_write(const uint8_t *data, uint32_t len) {
    if (len == 0) return true;
    if (!data) return false;

    /* Bounds check against the whole session so far, before touching flash. */
    if ((uint64_t)s_offset + s_buf_len + len > STAGING_SIZE) return false;

    while (len > 0) {
        uint32_t space = FLASH_PAGE_SIZE - s_buf_len;
        uint32_t chunk = (len < space) ? len : space;
        memcpy(s_page + s_buf_len, data, chunk);
        s_buf_len += chunk;
        data += chunk;
        len  -= chunk;

        if (s_buf_len == FLASH_PAGE_SIZE) {
            if (!flush_page(s_offset, s_page, FLASH_PAGE_SIZE)) return false;
            s_offset += FLASH_PAGE_SIZE;
            s_buf_len = 0;
        }
    }
    return true;
}

bool ota_stage_finish(uint32_t total_len) {
    /* Flush the final partial page, if any. Bytes padded past the real
     * content never fall within [0, total_len) so fw_verify() below never
     * reads them; 0xFF matches the erased-flash pattern for tidiness. */
    if (s_buf_len > 0) {
        memset(s_page + s_buf_len, 0xFF, FLASH_PAGE_SIZE - s_buf_len);
        if (!flush_page(s_offset, s_page, FLASH_PAGE_SIZE)) return false;
        s_offset += FLASH_PAGE_SIZE;
        s_buf_len = 0;
    }
    if (total_len > s_offset) return false;   /* fewer bytes staged than claimed */

    const uint8_t *img = staging_base();
    if (!fw_verify(img, total_len, OTA_VERIFY_PUBKEY)) return false;

    /* fw_verify() already proved this trailer (magic/length/hash/signature
     * all check out), so its sha512 is exactly "recompute the digest of the
     * staged image" — copy it rather than re-hash. */
    struct fw_trailer trailer;
    memcpy(&trailer, img + (total_len - sizeof trailer), sizeof trailer);

    /* ── preserve the OUTGOING active image's length for the bootloader's
     * benefit (true-rollback phase) ──────────────────────────────────────
     *
     * This function runs with the CURRENT boot_control record still on
     * flash — it describes whatever is sitting in the ACTIVE slot right now,
     * not the image we are about to stage. If that outgoing image is a
     * known-good, fully CONFIRMED image, its length is exactly what a
     * (future-phase) bootloader needs to size the ACTIVE -> BACKUP copy it
     * makes just before committing THIS new staged image. Rule: only ever
     * back up a CONFIRMED outgoing image.
     *
     *   - bc_valid && staged_len>0 && BC_F_CONFIRMED -> the active image is
     *     real and proven good: backup_len = old staged_len, BC_F_BACKUP_VALID
     *     set on the NEW record.
     *   - anything else -> nothing safe to roll back to: backup_len = 0,
     *     BC_F_BACKUP_VALID clear. Two cases land here: (1) no valid record
     *     at all yet — the very first OTA, staged over the trailer-less
     *     factory image with nothing ever written to boot_control; (2) the
     *     outgoing record exists but is itself still PENDING/unconfirmed —
     *     it was never proven to boot, so backing it up would be no safer
     *     than the image replacing it.
     *
     * Back-to-back-OTA edge: if OTA #2 is staged while OTA #1 is still
     * PENDING (never confirmed), OTA #1's own backup_len/BC_F_BACKUP_VALID
     * (recorded when OTA #1 was staged, describing whatever was active
     * before IT) are simply superseded here — OTA #2's record starts this
     * decision fresh from whatever bc_read() returns right now, per the same
     * rule above, so it correctly lands in case (2) and backs up nothing.
     * There is no accumulation across staged-but-never-confirmed images. */
    struct boot_control old_bc;
    bool have_old = bc_read(&old_bc);
    bool back_up_outgoing = have_old && old_bc.staged_len > 0 &&
                            (old_bc.flags & BC_F_CONFIRMED) != 0;

    struct boot_control bc;
    memset(&bc, 0, sizeof bc);
    bc.flags = BC_F_PENDING;
    bc.staged_len = total_len;
    memcpy(bc.staged_sha512, trailer.sha512, sizeof bc.staged_sha512);
    if (back_up_outgoing) {
        bc.flags |= BC_F_BACKUP_VALID;
        bc.backup_len = old_bc.staged_len;
    }
    /* else: bc.backup_len stays 0 and BC_F_BACKUP_VALID stays clear, from the
     * memset above. */

    return bc_write(&bc);
}

void ota_stage_abort(void) {
    s_offset = 0;
    s_buf_len = 0;
    s_erased_upto = 0;
}

void ota_stage_confirm(void) {
    struct boot_control bc;
    if (!bc_read(&bc)) return;   /* nothing valid to confirm */
    /* Idempotent: already confirmed with a clean attempt count -> no needless
     * flash erase/program. This is called once per healthy boot, so without
     * this guard every boot would wear a boot-control sector for nothing. */
    if ((bc.flags & BC_F_CONFIRMED) && bc.boot_attempts == 0) return;
    bc.flags |= BC_F_CONFIRMED;
    bc.boot_attempts = 0;
    bc_write(&bc);
}
