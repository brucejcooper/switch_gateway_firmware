#pragma once
#include <stdint.h>
#include <stdbool.h>

void dirty_poll_init(void);
void dirty_poll_mark_active(uint8_t slave_id);

/* Call from main loop — checks timing, issues reads if due.
 * Uses modbus_get_input_registers() with callbacks (non-blocking). */
void dirty_poll_tick(void);

/* Blocking startup full scan — call once per slave at boot */
void dirty_poll_full_scan(uint8_t slave_id);

/* 60s background presence + status anomaly check */
void dirty_poll_background_check(uint8_t slave_id);

/* Re-emit each relay + DALI slave's CURRENT reachability as a gateway-state
 * event. Reachability is tracked here as a one-shot transition (online→offline
 * fires once), so resync must replay it: on every MQTT (re)connect core 1's
 * discovery re-seeds every gateway-availability topic "online", and without
 * this an already-offline board would stay falsely online. Called from
 * lighting_resync_emit(). */
void dirty_poll_emit_gateway_states(void);

#define DIRTY_POLL_INTERVAL_MS   100u    /* fast dirty-mask poll while active */
#define DIRTY_ACTIVE_WINDOW_MS   2000u   /* stay active this long after the last command/change */
#define DIRTY_SWEEP_MS           1000u   /* regular cheap dirty-mask sweep while idle */
#define BACKGROUND_CHECK_MS      60000u  /* heavy presence/group/fault scan */
/* Steady-state config-over-raw rotation (task 03, only when g_config.dali_raw_cfg):
 * mark one present ballast per gateway for a background raw config re-read this
 * often. ~14 s × 64 ballasts ≈ 15 min for full coverage — enough to catch an
 * external recommissioning without loading the bus. */
#define DALI_CFG_ROTATE_MS       14000u

/* TinyModbus register addresses (post task-1-flag-driven refactor).
 * Per-ballast banks (BLOCK0..BLOCK3) are FC03 holding registers; the
 * presence and dirty bitmasks are FC04 input registers. */
#define DALI_BLOCK0_START     0x0000u  /* FC03 holding */
#define DALI_BLOCK1_START     0x0040u  /* FC03 holding */
#define DALI_BLOCK2_START     0x0080u  /* FC03 holding */
#define DALI_BLOCK3_START     0x00C0u  /* FC03 holding */
#define DALI_PRESENT_START    0x0100u  /* FC04 input, 4 regs */
#define DALI_DIRTY_START      0x0104u  /* FC04 input, 4 regs, read-clears */
#define DALI_SYSINFO_START    0x0310u  /* FC04 input, 4 regs: uptime(2) boot_count reset_reason */
#define DALI_GROUP_MASK_START 0x0110u  /* FC03 holding, 4 regs/group * 16 groups = 64 regs */
#define DALI_SCAN_TRIGGER_REG 0x1F00u   /* relocated in task 04 (was 0x1010) */
#define DALI_MAX_CHUNK        16u
