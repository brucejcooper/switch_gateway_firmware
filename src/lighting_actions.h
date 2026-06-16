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

/* All command helpers below return true if the Modbus write was queued, false if
 * it was dropped (action queue full) or skipped (slave being provisioned). On
 * false they log the reason and do NOT update the predictive cache, so the cache
 * never claims a state the hardware was never told. Callers may ignore the result
 * (the drop is already logged) or react to it. */

/* Relay: on/off */
bool lighting_set_relay(uint8_t channel, bool on);

/* Decode one relay channel's on/off from a raw FC01 coil-status payload (the
 * coil-data bytes that follow the Modbus byte-count field). Honors
 * g_config.relay_byteswap: some boards (e.g. Waveshare Modbus RTU Relay 32CH)
 * return the coil-data bytes in reversed order, contrary to the Modbus spec, so
 * when the flag is set the byte index is mirrored within the payload. Bits
 * within a byte are spec-standard either way (LSB = lowest channel of the byte).
 * Returns false for channels beyond byte_count*8. */
bool lighting_relay_coil_on(const uint8_t *coil_bytes, uint8_t byte_count, uint8_t ch);

/* Issue a raw DALI command via TinyModbus's RAW_CMD (or RAW_CMD_TWICE if
 * `twice` is true). TinyModbus rejects new raw writes only while the
 * previous command is still in flight (raw_cmd_result high byte == 0xFF —
 * PENDING); a terminal IDLE/NAK/OK/ERR is silently overwritten by the new
 * write. The slave returns MB_ERR_DEVICE_BUSY on the PENDING case. Returns the
 * enqueue result. */
bool dali_raw_arm(uint8_t slave_id, uint16_t value, bool twice);

/* DALI: on (goto last active) / off */
bool lighting_set_on_off(uint8_t slave_id, uint8_t dali_addr, bool on);

/* DALI: step up or down one level (uses ballast fade rate) */
bool lighting_dali_step(uint8_t slave_id, uint8_t dali_addr, bool up);

/* DALI: explicit arc level (DAPC) */
bool lighting_dali_set_level(uint8_t slave_id, uint8_t dali_addr, uint8_t arc);

/* DALI group helpers — single bus frame per call. State updates land via the
 * normal per-member dirty-bit polling path; no cache update on send. */
bool lighting_group_set_on_off(uint8_t slave_id, uint8_t group, bool on);
bool lighting_group_step(uint8_t slave_id, uint8_t group, bool up);
bool lighting_group_set_level(uint8_t slave_id, uint8_t group, uint8_t arc);

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

/* Emitted by dirty_poll when a DALI gateway's reachability changes (online =
 * responding to Modbus, offline = repeated read failures). Drives per-gateway
 * availability in the bridge. */
void lighting_emit_gateway_state(uint8_t slave_id, bool online);

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
