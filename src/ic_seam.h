#pragma once

/*
 * ic_seam.h — public interface to the core-0/core-1 intercore seam
 * (ic_seam.c): the single writer of g_config for all core-1 traffic (web UI
 * + Home Assistant writes), and the core-0 -> core-1 event forwarder.
 *
 * See ic_seam.c's own file-top comment for the split rationale (this used to
 * be buried at line ~2500 of serial_protocol.c, a 2700-line console file).
 * cli.h is the other half of that split.
 */

#include <stdint.h>
#include "mapping.h"   /* mapping_target_t — for emit_config_mapping() below */

/* Call once per main-loop iteration, AFTER cli_poll(): drains inbound
 * intercore commands (dispatch through ic_dispatch_cmd) and outbound lighting
 * events (convert via levt_to_ic and forward to core 1), plus the small
 * pollers that used to share serial_poll()'s loop body and must keep running
 * in the same relative order — see the comment above its definition. */
void ic_seam_pump(void);

/* Boot/ready announce + full mapping resync. Public because main.c's boot
 * sequence and the CLI's `resync` verb call them directly, alongside
 * ic_dispatch_cmd's own IC_CMD_RESYNC case. Names kept from the old
 * serial_protocol.h API — neither actually talks JSON over USB CDC (see the
 * .c file for what they really do). */
void serial_tx_hello(void);
void serial_tx_config_full(void);

/* ── Functions cli.c reaches into ic_seam.c for ─────────────────────────────
 * These build and push a {t,d,v} state report (or apply a {t,d,v} set) to
 * core 1 over the intercore event/command queues. They are primarily seam
 * functions — dali_apply_kv()'s return-code contract (1 ok / 0 unknown key /
 * -1 not present / -2 transport error) exists for ic_dispatch_cmd's
 * IC_CMD_SET_DALI case — that the CLI's bare-get and `dali`/`cfg`/`map` verbs
 * also call, so there is exactly one place that applies a DALI key=value or
 * builds a state_dali/state_group/state_cfg/config_mapping report. */
void emit_state_dali(uint8_t slave, uint8_t addr);
void emit_state_group(uint8_t slave, uint8_t group);
void emit_state_cfg(void);
void dali_group_set_members(uint8_t slave, uint8_t group, uint64_t new_mask);
uint64_t dali_group_members_requested(uint8_t slave, uint8_t group);

/* Register the blocking-wait hook (after dq_init). */
void ic_seam_init(void);
/* Move queued HA light commands into the desired-state tables now — for code
 * that blocks core 0 (see ic_seam.c). Safe to call from any wait loop. */
void ic_seam_absorb_light_cmds(void);
int  dali_apply_kv(uint8_t slave, uint8_t addr, const char *key, const char *val);

/* Emit a config_mapping line for one (fixture, button). Empty `targets`
 * (count == 0) clears the bridge's retained MQTT entry for that button. */
void emit_config_mapping(uint8_t fixture, uint8_t button,
                         const mapping_target_t *targets, uint8_t count);
