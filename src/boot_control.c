/*
 * boot_control.c — persistent app<->bootloader hand-off record, double-
 * buffered across the two boot-control sectors (partition_map.h).
 *
 * The hardware-touching bits (hardware/flash.h, flash_safe_execute) live only
 * inside the `#ifndef BOOT_CONTROL_HOST_TEST` block below — a thin seam so
 * this file can be compiled unmodified for the host test
 * (test/boot_control_test.c compiles it with -DBOOT_CONTROL_HOST_TEST and
 * supplies bc_test_sector_base()/bc_test_sector_write(), two RAM buffers
 * standing in for the real sectors), while the seq-selection / crc / power-
 * safety logic that actually matters runs identically on host and target.
 */

#include "boot_control.h"
#include "partition_map.h"
#include <string.h>

#ifdef BOOT_CONTROL_HOST_TEST
/* Host seam: the two boot-control sectors are plain RAM buffers, set up by
 * the host test before calling bc_read()/bc_write(). idx is 0=A, 1=B. */
extern const uint8_t *bc_test_sector_base(int idx);
extern void bc_test_sector_write(int idx, const void *data, size_t len);
#else
/* partition_map.h defines its own FLASH_SECTOR_SIZE/FLASH_PAGE_SIZE as plain
 * numeric literals (so its _Static_assert()s stay host-compilable without the
 * SDK). Undef them before pulling in the SDK's copies so the two identical-
 * value definitions don't produce a harmless-but-noisy macro-redefined
 * warning; everything below uses the SDK's copies. */
#undef FLASH_SECTOR_SIZE
#undef FLASH_PAGE_SIZE
#include "hardware/flash.h"
#include "pico/flash.h"
#endif

#define BC_SECTOR_COUNT 2

_Static_assert(BOOTCTRL_A_SIZE == BOOTCTRL_B_SIZE,
               "boot-control sectors must be the same size");
_Static_assert(BOOTCTRL_A_SIZE == FLASH_SECTOR_SIZE,
               "boot-control sectors must equal the flash erase granularity");
_Static_assert(sizeof(struct boot_control) <= FLASH_PAGE_SIZE,
               "boot_control record must fit in a single flash program page");

/* ── CRC-32 (reflected, poly 0xEDB88320) ─────────────────────────────────── */
uint32_t crc32(const void *data, size_t len) {
    const uint8_t *p = data;
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= p[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 1u) ? (crc >> 1) ^ 0xEDB88320u : (crc >> 1);
    }
    return crc ^ 0xFFFFFFFFu;
}

/* ── sector access (real flash vs. host RAM buffers) ─────────────────────── */
#ifndef BOOT_CONTROL_HOST_TEST
static uint32_t sector_off(int idx) {
    return idx == 0 ? BOOTCTRL_A_OFF : BOOTCTRL_B_OFF;
}
#endif

static const uint8_t *sector_base(int idx) {
#ifdef BOOT_CONTROL_HOST_TEST
    return bc_test_sector_base(idx);
#else
    return (const uint8_t *)PART_XIP(sector_off(idx));
#endif
}

/* Reads sector idx and reports whether it holds a structurally valid record
 * (magic + crc32 both check out); *out is filled either way. */
static bool sector_valid(int idx, struct boot_control *out) {
    memcpy(out, sector_base(idx), sizeof *out);
    if (out->magic != BC_MAGIC) return false;
    return crc32(out, offsetof(struct boot_control, crc32)) == out->crc32;
}

/* Scans both sectors and returns the valid one with the higher seq. */
static bool find_latest(int *idx_out, struct boot_control *rec_out) {
    struct boot_control recs[BC_SECTOR_COUNT];
    bool valid[BC_SECTOR_COUNT];
    int best = -1;

    for (int i = 0; i < BC_SECTOR_COUNT; i++) {
        valid[i] = sector_valid(i, &recs[i]);
        if (valid[i] && (best < 0 || recs[i].seq > recs[best].seq))
            best = i;
    }
    if (best < 0) return false;
    if (idx_out) *idx_out = best;
    if (rec_out) *rec_out = recs[best];
    return true;
}

bool bc_read(struct boot_control *out) {
    if (find_latest(NULL, out)) return true;
    memset(out, 0, sizeof *out);
    return false;
}

#ifndef BOOT_CONTROL_HOST_TEST
struct bc_flash_args {
    uint32_t offset;
    const uint8_t *buf;
    size_t len;
};

/* Runs under flash_safe_execute(): core 1 (the registered lockout victim) is
 * parked for the duration, so this is safe against the XIP cache/flash being
 * read while it's being erased/reprogrammed. */
static void bc_flash_erase_program(void *param) {
    const struct bc_flash_args *a = param;
    flash_range_erase(a->offset, FLASH_SECTOR_SIZE);
    flash_range_program(a->offset, a->buf, a->len);
}
#endif

bool bc_write(const struct boot_control *in) {
    int latest_idx = 0;
    struct boot_control latest;
    bool have_latest = find_latest(&latest_idx, &latest);
    int stale_idx = have_latest ? (1 - latest_idx) : 0;

    struct boot_control rec = *in;
    rec.magic = BC_MAGIC;
    rec.seq   = have_latest ? latest.seq + 1u : 1u;
    rec.crc32 = crc32(&rec, offsetof(struct boot_control, crc32));

    /* Program a whole page: the record (104 B) is far smaller than
     * FLASH_PAGE_SIZE, so pad with zero rather than trying a sub-page program. */
    uint8_t page[FLASH_PAGE_SIZE];
    memset(page, 0, sizeof page);
    memcpy(page, &rec, sizeof rec);

#ifdef BOOT_CONTROL_HOST_TEST
    bc_test_sector_write(stale_idx, page, sizeof page);
    return true;
#else
    struct bc_flash_args args = {
        .offset = sector_off(stale_idx),
        .buf    = page,
        .len    = sizeof page,
    };
    /* 4 KB erase + one 256 B page program is well under a second on the
     * on-board W25Q; 1 s leaves ample margin without stalling the lockout
     * handshake for long if something is wrong. */
    int rc = flash_safe_execute(bc_flash_erase_program, &args, 1000);
    return rc == PICO_OK;
#endif
}
