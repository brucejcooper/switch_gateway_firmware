/*
 * config.c — global configuration persistence.
 *
 * Stored as a simple text file /config.json so it can be inspected and
 * hand-edited if needed.  Uses stdio fopen() which is provided by pico-vfs,
 * exactly as the original modbus.c did for /modbus_config.txt.
 */

#include "config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

global_config_t g_config;

slave_kind_t slave_kind(uint8_t slave_id) {
    if (slave_id == 0) return SLAVE_KIND_UNKNOWN;  /* broadcast */
    for (int i = 0; i < g_config.relay_slave_count; i++) {
        if (g_config.relay_slave_ids[i] == slave_id) return SLAVE_KIND_RELAY;
    }
    for (int i = 0; i < g_config.dali_slave_count; i++) {
        if (g_config.dali_slave_ids[i] == slave_id) return SLAVE_KIND_DALI;
    }
    return SLAVE_KIND_UNKNOWN;
}

void config_defaults(void) {
    g_config.long_press_ms       = 500;
    g_config.dim_repeat_ms       = 200;
    g_config.reversal_window_ms  = 400;
    g_config.relay_slave_ids[0]  = 1;
    g_config.relay_slave_count   = 1;
    g_config.dali_slave_ids[0]   = 2;
    g_config.dali_slave_ids[1]   = 3;
    g_config.dali_slave_count    = 2;
}

/* ── Minimal JSON key lookup (same flavour as serial_protocol.c) ─────────── */
static bool find_value(const char *json, const char *key, const char **out_p) {
    char needle[32];
    snprintf(needle, sizeof(needle), "\"%s\"", key);
    const char *kp = strstr(json, needle);
    if (!kp) return false;
    kp += strlen(needle);
    while (*kp == ' ' || *kp == '\t' || *kp == ':') kp++;
    *out_p = kp;
    return true;
}

static bool find_uint(const char *json, const char *key, unsigned *out) {
    const char *p;
    if (!find_value(json, key, &p)) return false;
    char *end = NULL;
    long v = strtol(p, &end, 10);
    if (end == p || v < 0) return false;
    *out = (unsigned)v;
    return true;
}

/* Parse a JSON array of small unsigned ints. Returns count parsed (0..max). */
static uint8_t find_uint_array(const char *json, const char *key,
                                uint8_t *out, uint8_t max) {
    const char *p;
    if (!find_value(json, key, &p)) return 0;
    if (*p != '[') return 0;
    p++;
    uint8_t count = 0;
    while (count < max) {
        while (*p == ' ' || *p == '\t') p++;
        if (*p == ']' || *p == '\0') break;
        char *end = NULL;
        long v = strtol(p, &end, 10);
        if (end == p || v < 0 || v > 255) break;
        out[count++] = (uint8_t)v;
        p = end;
        while (*p == ' ' || *p == '\t') p++;
        if (*p == ',') p++;
    }
    return count;
}

bool config_load(void) {
    config_defaults();

    FILE *fp = fopen("/config.json", "r");
    if (!fp) return false;

    char buf[512];
    size_t n = fread(buf, 1, sizeof(buf) - 1, fp);
    fclose(fp);
    buf[n] = '\0';

    unsigned u;
    if (find_uint(buf, "long_press_ms",      &u)) g_config.long_press_ms      = (uint16_t)u;
    if (find_uint(buf, "dim_repeat_ms",      &u)) g_config.dim_repeat_ms      = (uint16_t)u;
    if (find_uint(buf, "reversal_window_ms", &u)) g_config.reversal_window_ms = (uint16_t)u;

    /* Relay slaves: prefer new "relay_slaves":[...] array; fall back to the
     * legacy scalar "relay_slave":N for forward migration. */
    uint8_t rs[MAX_RELAY_SLAVES];
    uint8_t rc = find_uint_array(buf, "relay_slaves", rs, MAX_RELAY_SLAVES);
    if (rc > 0) {
        memcpy(g_config.relay_slave_ids, rs, rc);
        g_config.relay_slave_count = rc;
    } else if (find_uint(buf, "relay_slave", &u) && u > 0 && u < 248) {
        g_config.relay_slave_ids[0] = (uint8_t)u;
        g_config.relay_slave_count  = 1;
    }

    uint8_t ds[MAX_DALI_SLAVES];
    uint8_t dc = find_uint_array(buf, "dali_slaves", ds, MAX_DALI_SLAVES);
    if (dc > 0) {
        memcpy(g_config.dali_slave_ids, ds, dc);
        g_config.dali_slave_count = dc;
    }

    return true;
}

bool config_save(void) {
    FILE *fp = fopen("/config.json", "w");
    if (!fp) return false;
    fprintf(fp,
        "{\"long_press_ms\":%u,\"dim_repeat_ms\":%u,"
        "\"reversal_window_ms\":%u,\"relay_slaves\":[",
        g_config.long_press_ms, g_config.dim_repeat_ms,
        g_config.reversal_window_ms);
    for (int i = 0; i < g_config.relay_slave_count; i++) {
        fprintf(fp, "%s%u", i > 0 ? "," : "", g_config.relay_slave_ids[i]);
    }
    fprintf(fp, "],\"dali_slaves\":[");
    for (int i = 0; i < g_config.dali_slave_count; i++) {
        fprintf(fp, "%s%u", i > 0 ? "," : "", g_config.dali_slave_ids[i]);
    }
    fprintf(fp, "]}\n");
    fclose(fp);
    return true;
}
