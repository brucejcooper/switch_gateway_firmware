#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "config.h"

#define MAX_FIXTURES        24
#define MAX_BUTTONS          8    /* 7 physical + 1 spare, keep byte-aligned */
#define MAX_TARGETS_PER_BTN  8

typedef struct {
    uint8_t slave;   /* real Modbus slave id; 0 (broadcast) means unset/legacy */
    uint8_t addr;
} mapping_target_t;

/* Group encoding: when bit 7 of `addr` is set, the low nibble (0-15) is a DALI
 * group number; otherwise `addr` is a short ballast address (0-63). Layout is
 * byte-compatible with previous mappings.bin files (all old `addr` values had
 * bit 7 clear). Relay bindings never set the group bit. */
#define TARGET_GROUP_BIT       0x80u
#define TARGET_IS_GROUP(t)     (((t).addr & TARGET_GROUP_BIT) != 0)
#define TARGET_GROUP_NUM(t)    ((t).addr & 0x0Fu)
#define TARGET_BALLAST_ADDR(t) ((t).addr & 0x3Fu)
#define TARGET_MAKE_GROUP(g)   ((uint8_t)(TARGET_GROUP_BIT | ((g) & 0x0Fu)))

typedef struct {
    uint8_t           fixture;
    uint8_t           button;
    uint8_t           target_count;
    uint8_t           _pad;
    mapping_target_t  targets[MAX_TARGETS_PER_BTN];
} mapping_record_t;

extern mapping_record_t g_mappings[MAX_FIXTURES][MAX_BUTTONS];

bool mapping_load(void);
bool mapping_save(void);

const mapping_record_t *mapping_lookup(uint8_t fixture, uint8_t button);

bool mapping_set(uint8_t fixture, uint8_t button,
                 const mapping_target_t *targets, uint8_t count);
bool mapping_delete(uint8_t fixture, uint8_t button);

/* Serialise all non-empty mappings as JSON lines into buf.
 * Returns bytes written, or -1 if buffer too small. */
int mapping_serialise_all(char *buf, int buf_len);

/* ── Batch replace transaction ─────────────────────────────────────────────
 * Used by the spreadsheet sync tool to wipe and replace every binding
 * atomically. Sequence is begin → add* → commit (or abort).
 *
 *   begin   — clears scratch, starts a new transaction (idempotent).
 *   add     — stages one (fixture, button) entry into the scratch buffer.
 *             Validates ranges; on failure latches a sticky error flag so
 *             commit will refuse. Returns true if the entry was accepted.
 *   commit  — if no error latched, swaps scratch into g_mappings, saves to
 *             flash, ends the transaction. Returns false if no txn was
 *             active or an add had failed.
 *   abort   — discards scratch + clears error flag. Idempotent.
 *
 * Transaction state is RAM-only: a reboot mid-transaction leaves the live
 * mappings untouched. */
void mapping_txn_begin(void);
bool mapping_txn_add(uint8_t fixture, uint8_t button,
                     const mapping_target_t *targets, uint8_t count);
bool mapping_txn_commit(void);
void mapping_txn_abort(void);
bool mapping_txn_active(void);
bool mapping_txn_errored(void);

/* During commit, walk every (fixture, button) whose targets differ between
 * the old live mapping and the newly-committed one — caller emits the
 * required config_mapping events. Must be called inside commit (after the
 * swap) for the diff to reflect what changed. */
typedef void (*mapping_diff_cb_t)(uint8_t fixture, uint8_t button,
                                  const mapping_target_t *new_targets,
                                  uint8_t new_count, void *user);
void mapping_txn_for_each_change(mapping_diff_cb_t cb, void *user);
