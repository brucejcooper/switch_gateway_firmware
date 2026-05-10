/*
 * config.c — global configuration persistence.
 *
 * Stored as a simple text file /config.json so it can be inspected and
 * hand-edited if needed.  Uses stdio fopen() which is provided by pico-vfs,
 * exactly as the original modbus.c did for /modbus_config.txt.
 */

#include "config.h"
#include <stdio.h>
#include <string.h>

global_config_t g_config;

void config_defaults(void) {
    g_config.long_press_ms      = 500;
    g_config.dim_repeat_ms      = 200;
    g_config.reversal_window_ms = 400;
    g_config.relay_slave_id     = 1;
    g_config.dali_slave_ids[0]  = 2;
    g_config.dali_slave_ids[1]  = 3;
    g_config.dali_slave_count   = 2;
}

bool config_load(void) {
    FILE *fp = fopen("/config.json", "r");
    if (!fp) {
        config_defaults();
        return false;
    }

    unsigned lp = 500, dr = 200, rw = 400, rs = 1, ds0 = 2, ds1 = 3, dc = 2;
    int matched = fscanf(fp,
        " { \"long_press_ms\" : %u , \"dim_repeat_ms\" : %u ,"
        " \"reversal_window_ms\" : %u , \"relay_slave\" : %u ,"
        " \"dali_slaves\" : [ %u , %u ] , \"dali_count\" : %u }",
        &lp, &dr, &rw, &rs, &ds0, &ds1, &dc);
    fclose(fp);

    if (matched < 7) {
        config_defaults();
        return false;
    }

    g_config.long_press_ms      = (uint16_t)lp;
    g_config.dim_repeat_ms      = (uint16_t)dr;
    g_config.reversal_window_ms = (uint16_t)rw;
    g_config.relay_slave_id     = (uint8_t)rs;
    g_config.dali_slave_ids[0]  = (uint8_t)ds0;
    g_config.dali_slave_ids[1]  = (uint8_t)ds1;
    g_config.dali_slave_count   = (uint8_t)(dc <= MAX_DALI_SLAVES ? dc : MAX_DALI_SLAVES);
    return true;
}

bool config_save(void) {
    FILE *fp = fopen("/config.json", "w");
    if (!fp) return false;
    fprintf(fp,
        "{\"long_press_ms\":%u,\"dim_repeat_ms\":%u,"
        "\"reversal_window_ms\":%u,\"relay_slave\":%u,"
        "\"dali_slaves\":[%u,%u],\"dali_count\":%u}\n",
        g_config.long_press_ms, g_config.dim_repeat_ms,
        g_config.reversal_window_ms, g_config.relay_slave_id,
        g_config.dali_slave_ids[0], g_config.dali_slave_ids[1],
        g_config.dali_slave_count);
    fclose(fp);
    return true;
}
