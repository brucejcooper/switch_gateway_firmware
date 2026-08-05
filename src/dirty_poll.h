#pragma once
#include <stdint.h>
#include <stdbool.h>

void dirty_poll_init(void);
void dirty_poll_mark_active(uint8_t slave_id);

/* Call from main loop — checks timing, issues reads if due.
 * Uses modbus_get_input_registers() with callbacks (non-blocking). */
void dirty_poll_tick(void);

/* Arm a full DALI interrogation and return IMMEDIATELY. The main loop drives it;
 * state converges over the next few seconds for a live gateway, and never for an
 * absent one — which is the correct outcome and costs nothing. This is what boot
 * uses. */
void dirty_poll_arm_scan(uint8_t slave_id);

/* Re-emit each relay + DALI slave's CURRENT reachability as a gateway-state
 * event. Reachability is tracked here as a one-shot transition (online→offline
 * fires once), so resync must replay it: on every MQTT (re)connect core 1's
 * discovery re-seeds every gateway-availability topic "online", and without
 * this an already-offline board would stay falsely online. Called from
 * lighting_resync_emit(). */
void dirty_poll_emit_gateway_states(void);

/* Steady-state config-over-raw rotation: mark one present ballast per gateway
 * for a background raw config re-read this often. ~14 s × 64 ballasts ≈ 15 min
 * for full coverage — enough to catch an external recommissioning without
 * loading the bus. */
#define DALI_CFG_ROTATE_MS       14000u

/* The gateway's sysinfo block — the ONLY register range this module still reads.
 *
 * The gateway caches nothing, so live state comes from raw DALI queries via
 * dali_state.c; its cache addresses return ILLEGAL DATA ADDRESS. */
#define DALI_SYSINFO_START    0x0310u  /* FC04 input, 4 regs: uptime(2) boot_count reset_reason */
