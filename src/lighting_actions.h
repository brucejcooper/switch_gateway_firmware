#pragma once
#include <stdint.h>
#include <stdbool.h>

/*
 * lighting_actions — bridge between button_fsm / serial_protocol and modbus.
 *
 * Each function enqueues a Modbus action.  The callback updates state_cache
 * and pushes a state_event for serial_poll() to emit to HA.
 *
 * These replace the Lua coroutine bindings that previously linked button
 * events to Modbus writes.
 */

/* Relay: on/off */
void lighting_set_relay(uint8_t channel, bool on);

/* Issue a raw DALI command via TinyModbus's RAW_CMD (or RAW_CMD_TWICE if
 * `twice` is true). Always pre-flushes the result register at FC04 0x0108
 * — TinyModbus rejects new raw writes with SLAVE_DEVICE_BUSY whenever a
 * stale terminal value is parked there, and *every* successful raw write
 * parks one. This helper is the only correct way for any caller to issue
 * a raw DALI command; never write FC06 0x1000/0x1001 directly. */
void dali_raw_arm(uint8_t slave_id, uint16_t value, bool twice);

/* DALI: on (goto last active) / off */
void lighting_set_on_off(uint8_t slave_id, uint8_t dali_addr, bool on);

/* DALI: step up or down one level (uses ballast fade rate) */
void lighting_dali_step(uint8_t slave_id, uint8_t dali_addr, bool up);

/* DALI: explicit arc level (DAPC) */
void lighting_dali_set_level(uint8_t slave_id, uint8_t dali_addr, uint8_t arc);

/* DALI group helpers — single bus frame per call. State updates land via the
 * normal per-member dirty-bit polling path; no cache update on send. */
void lighting_group_set_on_off(uint8_t slave_id, uint8_t group, bool on);
void lighting_group_step(uint8_t slave_id, uint8_t group, bool up);
void lighting_group_set_level(uint8_t slave_id, uint8_t group, uint8_t arc);

/*
 * Called from dirty_poll when a state change is confirmed by hardware.
 * Emits a state event for serial_poll() to publish.
 */
void lighting_emit_relay_state(uint8_t channel, bool on);
void lighting_emit_dali_state(uint8_t slave_id, uint8_t addr,
                               uint8_t arc, uint8_t status);
void lighting_emit_presence(uint8_t slave_id,
                             uint64_t appeared, uint64_t disappeared);
void lighting_emit_fault(uint8_t slave_id, uint8_t addr,
                          bool lamp_fail, bool power_cycle);
/* fix #6: emitted by dirty_poll_full_scan() so bridge can set availability */
void lighting_emit_scan_complete(uint8_t slave_id, uint64_t present_mask);

/* Recompute a group's derived (on, bri) from its present members and emit
 * a state_group event if it differs from the last emitted value. Called
 * automatically from lighting_emit_dali_state for every group containing
 * the ballast that changed. Idempotent — safe to call when state is
 * unchanged. */
void lighting_emit_group_state_if_changed(uint8_t slave_id, uint8_t group);

/* Re-emit the full state snapshot to the CDC port. Pushes hello + every
 * cached entity state + scan_complete + every mapping. Idempotent and
 * non-blocking — pushes events to the same ring buffer the dirty-poll
 * machinery uses. Safe to call from any context that already calls into
 * lighting_emit_*. */
void lighting_resync_emit(void);
