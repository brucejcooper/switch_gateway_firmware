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

/* DALI: on (goto last active) / off */
void lighting_set_on_off(uint8_t slave_id, uint8_t dali_addr, bool on);

/* DALI: step up or down one level (uses ballast fade rate) */
void lighting_dali_step(uint8_t slave_id, uint8_t dali_addr, bool up);

/* DALI: explicit arc level (DAPC) */
void lighting_dali_set_level(uint8_t slave_id, uint8_t dali_addr, uint8_t arc);

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
