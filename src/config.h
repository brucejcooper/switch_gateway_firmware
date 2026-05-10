#pragma once
#include <stdint.h>
#include <stdbool.h>

/* Number of TinyModbus DALI gateways. Matches hardware: slaves 2 and 3.
 * The Waveshare relay board (slave 1) is handled separately via relay_slave_id.
 * Increase this and extend config_load()/config_save() to add a third gateway. */
#define MAX_DALI_SLAVES 2

typedef struct {
    uint16_t long_press_ms;        /* default 500 */
    uint16_t dim_repeat_ms;        /* default 200 */
    uint16_t reversal_window_ms;   /* default 400 */
    uint8_t  relay_slave_id;       /* default 1   */
    uint8_t  dali_slave_ids[MAX_DALI_SLAVES];
    uint8_t  dali_slave_count;
} global_config_t;

extern global_config_t g_config;

/* Load from /config.json; fills defaults if absent or corrupt.
 * Uses stdio fopen() — works with pico-vfs. */
bool config_load(void);

/* Persist current g_config to /config.json */
bool config_save(void);

void config_defaults(void);
