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

#define DIRTY_POLL_INTERVAL_MS   100u
#define DIRTY_ZERO_SETTLE_COUNT  2u
#define BACKGROUND_CHECK_MS      60000u

/* TinyModbus FC04 register addresses */
#define DALI_BLOCK0_START     0x0000u
#define DALI_BLOCK1_START     0x0040u
#define DALI_BLOCK2_START     0x0080u
#define DALI_BLOCK3_START     0x00C0u
#define DALI_PRESENT_START    0x0151u
#define DALI_DIRTY_START      0x0159u
#define DALI_SCAN_TRIGGER_REG 0x1010u
#define DALI_MAX_CHUNK        16u
