#pragma once
/*
 * dali_state.h — the controller's DALI world model (task 05).
 *
 * This is the "controller owns all state" half of task 05. Every fact about a
 * ballast — presence, arc level, status, group membership — is learned here by
 * QUERYING THE BALLAST through the command queue, and held in the controller's
 * state cache. Nothing is read from a gateway register, because the gateway is
 * being reduced to a paced DALI pipe with no cache of its own.
 *
 * Replaces, one for one:
 *   gateway block0 (arc/status)  -> QUERY STATUS + QUERY ACTUAL LEVEL sweep
 *   gateway dirty bitmap         -> post-command verification marks (below)
 *   gateway present bitmap       -> presence evidence rules (below)
 *   gateway group masks          -> QUERY GROUPS 0-7 / 8-15 per ballast
 *
 * PRESENCE EVIDENCE RULES (overview.md principle 4 — absence needs proof):
 * only a CLEAN NO-ANSWER to a directed query counts against presence, and only
 * after DALI_ABSENT_STRIKES consecutive ones. A good answer restores presence
 * immediately. A collision, a bus fault or a dropped command proves NOTHING
 * about the ballast — it is evidence about the bus — so those FREEZE the
 * counters rather than counting as absence. Without that rule a noisy bus
 * slowly marks a healthy installation offline.
 *
 * PRIORITY: a ballast touched by a command is verified at DQ_PRIO_LEVEL_READ so
 * the user-visible state converges immediately; the untouched rest is swept at
 * DQ_PRIO_BACKGROUND. Both are self-limiting to one query in flight per slave,
 * so the sweep can never crowd out a button press.
 */

#include <stdbool.h>
#include <stdint.h>

/* Consecutive clean no-answers before a ballast is declared absent. */
#define DALI_ABSENT_STRIKES  2u

void dali_state_init(void);

/* Per-loop driver: issue at most one query per slave, verification first. */
void dali_state_tick(void);

/* Post-command verification. Mark what a command actually touched, so its real
 * outcome is read back rather than assumed:
 *   _mark_dirty       one ballast (a directed command)
 *   _mark_dirty_group a group's cached members (a group command)
 *   _mark_all         every present ballast (broadcast, or an unknown target) */
void dali_state_mark_dirty(uint8_t slave_id, uint8_t addr);
void dali_state_mark_dirty_group(uint8_t slave_id, uint8_t group);
void dali_state_mark_all(uint8_t slave_id);

/* Arm a full discovery pass for a slave: presence + groups + level/status for
 * every address (config follows via the existing config-query rotation). Used at
 * boot, after provisioning, and by the CLI `scan`. Non-blocking — progress is
 * made by dali_state_tick(); dali_state_scan_busy() reports completion. */
void dali_state_request_full_scan(uint8_t slave_id);

/* Re-interrogate ONE ballast completely, group membership included. Used after
 * an ADD/REMOVE FROM GROUP write so the membership cache is confirmed by a read
 * rather than left as the controller's prediction. */
void dali_state_mark_scan(uint8_t slave_id, uint8_t addr);
bool dali_state_scan_busy(uint8_t slave_id);

/* Dump per-slave sweep state and presence to the console (CLI `dstate`). */
void dali_state_debug_print(int slave_id);
