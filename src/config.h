#pragma once
#include <stdint.h>
#include <stdbool.h>

/* Slaves are addressed by their real Modbus address. The relay (Waveshare) and
 * DALI (TinyModbus) gateways are kept in separate config lists so the firmware
 * can dispatch operations to the right protocol via slave_kind(). Default
 * hardware: relay at slave 1, DALI gateways at slaves 2 and 3. */
#define MAX_RELAY_SLAVES 2
#define MAX_DALI_SLAVES  2

typedef struct {
    uint16_t long_press_ms;        /* default 500 */
    uint16_t dim_repeat_ms;        /* default 200 */
    uint16_t reversal_window_ms;   /* default 400 */
    uint8_t  relay_slave_ids[MAX_RELAY_SLAVES];
    uint8_t  relay_slave_count;
    uint8_t  dali_slave_ids[MAX_DALI_SLAVES];
    uint8_t  dali_slave_count;
} global_config_t;

extern global_config_t g_config;

typedef enum {
    SLAVE_KIND_UNKNOWN = 0,
    SLAVE_KIND_RELAY,
    SLAVE_KIND_DALI,
} slave_kind_t;

/* Resolve a Modbus slave id to its protocol kind via the configured lists.
 * Returns SLAVE_KIND_UNKNOWN for slave 0 (broadcast) and unconfigured ids. */
slave_kind_t slave_kind(uint8_t slave_id);

/* Load from /config.json; fills defaults if absent or corrupt.
 * Uses stdio fopen() — works with pico-vfs. Accepts both the legacy
 * "relay_slave":N scalar and the new "relay_slaves":[...] array shape. */
bool config_load(void);

/* Persist current g_config to /config.json */
bool config_save(void);

void config_defaults(void);
