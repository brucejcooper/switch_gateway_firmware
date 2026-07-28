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

/* Blocking full scan — waits for the interrogation to finish. ONLY for the CLI
 * `scan`, where the operator wants the result printed on return. Feeds the
 * watchdog (a scan on a populated bus runs well past 5 s) and bails out early if
 * the gateway stops answering, so an absent slave costs a moment rather than the
 * whole deadline. */
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
/* Steady-state config-over-raw rotation: mark one present ballast per gateway
 * for a background raw config re-read this often. ~14 s × 64 ballasts ≈ 15 min
 * for full coverage — enough to catch an external recommissioning without
 * loading the bus. */
#define DALI_CFG_ROTATE_MS       14000u

/* TinyModbus register addresses. Only BLOCK0 (live arc level / status) is read
 * from the gateway now — the config banks (min/max/power-on/sys-fail/fade,
 * formerly 0x0040/0x0080/0x00C0) were retired in task 04 and config is read via
 * raw DALI queries. Presence/dirty are FC04 input registers. */
#define DALI_BLOCK0_START     0x0000u  /* FC03 holding: status | brightness */
#define DALI_PRESENT_START    0x0100u  /* FC04 input, 4 regs */
#define DALI_DIRTY_START      0x0104u  /* FC04 input, 4 regs, read-clears */
#define DALI_SYSINFO_START    0x0310u  /* FC04 input, 4 regs: uptime(2) boot_count reset_reason */
#define DALI_GROUP_MASK_START 0x0110u  /* FC03 holding, 4 regs/group * 16 groups = 64 regs */
#define DALI_SCAN_TRIGGER_REG 0x1F00u   /* relocated in task 04 (was 0x1010) */
#define DALI_MAX_CHUNK        16u
