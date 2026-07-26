/*
 * config.c — global configuration persistence (/config.json).
 *
 * One unified config file holds the controller, modbus, MQTT and network
 * settings (consolidating the former /modbus_baud.txt and /modbus_config.txt).
 * Parsed with jsmn (malloc-free, float-free); written with snprintf/fprintf.
 * Keys are flat (mqtt_host, net_ip, ...) for simple, robust, human-editable JSON.
 */

#include "config.h"
#include "log.h"
#include "jsmn.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

global_config_t g_config;

slave_kind_t slave_kind(uint8_t slave_id) {
    if (slave_id == 0) return SLAVE_KIND_UNKNOWN;  /* broadcast */
    for (int i = 0; i < g_config.relay_slave_count; i++)
        if (g_config.relay_slave_ids[i] == slave_id) return SLAVE_KIND_RELAY;
    for (int i = 0; i < g_config.dali_slave_count; i++)
        if (g_config.dali_slave_ids[i] == slave_id) return SLAVE_KIND_DALI;
    return SLAVE_KIND_UNKNOWN;
}

void config_defaults(void) {
    memset(&g_config, 0, sizeof g_config);
    g_config.long_press_ms       = 500;
    g_config.dim_repeat_ms       = 200;
    g_config.reversal_window_ms  = 400;
    g_config.gw_diag_interval_ms = 30000;
    g_config.relay_slave_ids[0]  = 1;
    g_config.relay_slave_count   = 1;
    g_config.dali_slave_ids[0]   = 2;
    g_config.dali_slave_ids[1]   = 3;
    g_config.dali_slave_count    = 2;
    g_config.baud                = 19200;
    g_config.de_mode             = 0;
    g_config.relay_byteswap      = false;
    g_config.mqtt_port           = 1883;
    strncpy(g_config.mqtt_prefix, "lighting", sizeof g_config.mqtt_prefix - 1);
    g_config.net_dhcp            = 1;
    strncpy(g_config.net_hostname, "lights", sizeof g_config.net_hostname - 1);
    strncpy(g_config.web_user, "admin", sizeof g_config.web_user - 1);
    strncpy(g_config.web_pass, "admin", sizeof g_config.web_pass - 1);
}

/* ── jsmn helpers ────────────────────────────────────────────────────────── */
static bool tok_eq(const char *js, const jsmntok_t *t, const char *s) {
    int len = t->end - t->start;
    return t->type == JSMN_STRING && (int)strlen(s) == len &&
           strncmp(js + t->start, s, len) == 0;
}
static int find_key(const char *js, const jsmntok_t *t, int n, const char *key) {
    for (int i = 1; i < n; i++) if (tok_eq(js, &t[i], key)) return i + 1;
    return -1;
}
static bool get_uint(const char *js, const jsmntok_t *t, int n, const char *key, unsigned *out) {
    int i = find_key(js, t, n, key);
    if (i < 0) return false;
    char b[16]; int len = t[i].end - t[i].start; if (len >= (int)sizeof b) len = (int)sizeof b - 1;
    memcpy(b, js + t[i].start, len); b[len] = '\0';
    char *e; long v = strtol(b, &e, 10);
    if (e == b || v < 0) return false;
    *out = (unsigned)v; return true;
}
static bool get_str(const char *js, const jsmntok_t *t, int n, const char *key, char *out, size_t cap) {
    int i = find_key(js, t, n, key);
    if (i < 0 || t[i].type != JSMN_STRING) return false;
    int len = t[i].end - t[i].start; if (len >= (int)cap) len = (int)cap - 1;
    memcpy(out, js + t[i].start, len); out[len] = '\0';
    return true;
}
static bool get_ip(const char *js, const jsmntok_t *t, int n, const char *key, uint8_t ip[4]) {
    char b[20];
    if (!get_str(js, t, n, key, b, sizeof b)) return false;
    unsigned a0, a1, a2, a3;
    if (sscanf(b, "%u.%u.%u.%u", &a0, &a1, &a2, &a3) != 4) return false;
    ip[0] = (uint8_t)a0; ip[1] = (uint8_t)a1; ip[2] = (uint8_t)a2; ip[3] = (uint8_t)a3;
    return true;
}
/* Parse a JSON array of small uints at key into out, returns count. */
static uint8_t get_u8_array(const char *js, const jsmntok_t *t, int n, const char *key,
                            uint8_t *out, uint8_t max) {
    int i = find_key(js, t, n, key);
    if (i < 0 || t[i].type != JSMN_ARRAY) return 0;
    int count = t[i].size; if (count > max) count = max;
    for (int k = 0; k < count; k++) {
        const jsmntok_t *e = &t[i + 1 + k];
        char b[8]; int len = e->end - e->start; if (len >= (int)sizeof b) len = (int)sizeof b - 1;
        memcpy(b, js + e->start, len); b[len] = '\0';
        out[k] = (uint8_t)strtol(b, NULL, 10);
    }
    return (uint8_t)count;
}

/* One-time import of the legacy modbus text files into g_config, then delete. */
static bool migrate_legacy_modbus(void) {
    bool changed = false;
    FILE *fp = fopen("/modbus_baud.txt", "r");
    if (fp) {
        unsigned long b = 0;
        if (fscanf(fp, "%lu", &b) == 1 && b > 0) { g_config.baud = (uint32_t)b; changed = true; }
        fclose(fp);
        remove("/modbus_baud.txt");
        LOG_INFO("config", "migrated /modbus_baud.txt -> baud=%lu", (unsigned long)g_config.baud);
    }
    fp = fopen("/modbus_config.txt", "r");
    if (fp) {
        char m[24] = {0};
        if (fgets(m, sizeof m, fp)) {
            /* legacy stored the DE-mode string; default 0 (when-tx) unless "always". */
            if (strstr(m, "always")) g_config.de_mode = 1;
            changed = true;
        }
        fclose(fp);
        remove("/modbus_config.txt");
        LOG_INFO("config", "migrated /modbus_config.txt -> de_mode=%u", g_config.de_mode);
    }
    return changed;
}

bool config_load(void) {
    config_defaults();

    FILE *fp = fopen("/config.json", "r");
    char buf[1024];
    size_t n = 0;
    if (fp) { n = fread(buf, 1, sizeof(buf) - 1, fp); fclose(fp); }
    buf[n] = '\0';

    if (n > 0) {
        jsmn_parser jp; jsmntok_t tk[96]; jsmn_init(&jp);
        int nt = jsmn_parse(&jp, buf, n, tk, 96);
        if (nt > 0) {
            unsigned u;
            if (get_uint(buf, tk, nt, "long_press_ms", &u))      g_config.long_press_ms      = (uint16_t)u;
            if (get_uint(buf, tk, nt, "dim_repeat_ms", &u))      g_config.dim_repeat_ms      = (uint16_t)u;
            if (get_uint(buf, tk, nt, "reversal_window_ms", &u)) g_config.reversal_window_ms = (uint16_t)u;
            if (get_uint(buf, tk, nt, "gw_diag_interval_ms", &u)) g_config.gw_diag_interval_ms = u;
            if (get_uint(buf, tk, nt, "baud", &u) && u > 0)      g_config.baud               = u;
            if (get_uint(buf, tk, nt, "de_mode", &u))            g_config.de_mode            = (uint8_t)u;
            if (get_uint(buf, tk, nt, "relay_byteswap", &u))     g_config.relay_byteswap     = u ? true : false;

            uint8_t rs[MAX_RELAY_SLAVES];
            uint8_t rc = get_u8_array(buf, tk, nt, "relay_slaves", rs, MAX_RELAY_SLAVES);
            if (rc > 0) { memcpy(g_config.relay_slave_ids, rs, rc); g_config.relay_slave_count = rc; }
            uint8_t ds[MAX_DALI_SLAVES];
            uint8_t dc = get_u8_array(buf, tk, nt, "dali_slaves", ds, MAX_DALI_SLAVES);
            if (dc > 0) { memcpy(g_config.dali_slave_ids, ds, dc); g_config.dali_slave_count = dc; }

            get_str(buf, tk, nt, "mqtt_host",   g_config.mqtt_host,   sizeof g_config.mqtt_host);
            get_str(buf, tk, nt, "mqtt_user",   g_config.mqtt_user,   sizeof g_config.mqtt_user);
            get_str(buf, tk, nt, "mqtt_pass",   g_config.mqtt_pass,   sizeof g_config.mqtt_pass);
            get_str(buf, tk, nt, "mqtt_prefix", g_config.mqtt_prefix, sizeof g_config.mqtt_prefix);
            if (get_uint(buf, tk, nt, "mqtt_port", &u) && u > 0) g_config.mqtt_port = (uint16_t)u;

            if (get_uint(buf, tk, nt, "net_dhcp", &u)) g_config.net_dhcp = u ? 1 : 0;
            get_ip(buf, tk, nt, "net_ip",   g_config.net_ip);
            get_ip(buf, tk, nt, "net_mask", g_config.net_mask);
            get_ip(buf, tk, nt, "net_gw",   g_config.net_gw);
            get_ip(buf, tk, nt, "net_dns",  g_config.net_dns);
            get_str(buf, tk, nt, "net_hostname", g_config.net_hostname, sizeof g_config.net_hostname);

            get_str(buf, tk, nt, "web_user", g_config.web_user, sizeof g_config.web_user);
            get_str(buf, tk, nt, "web_pass", g_config.web_pass, sizeof g_config.web_pass);

            get_str(buf, tk, nt, "fw_url", g_config.fw_url, sizeof g_config.fw_url);
        } else {
            LOG_WARN("config", "config.json parse failed (%d) — using defaults", nt);
        }
    }

    if (migrate_legacy_modbus()) config_save();
    return true;
}

bool config_save(void) {
    FILE *fp = fopen("/config.json", "w");
    if (!fp) return false;
    fprintf(fp,
        "{\"long_press_ms\":%u,\"dim_repeat_ms\":%u,\"reversal_window_ms\":%u,"
        "\"gw_diag_interval_ms\":%lu,"
        "\"baud\":%lu,\"de_mode\":%u,\"relay_byteswap\":%u,"
        "\"relay_slaves\":[",
        g_config.long_press_ms, g_config.dim_repeat_ms, g_config.reversal_window_ms,
        (unsigned long)g_config.gw_diag_interval_ms,
        (unsigned long)g_config.baud, g_config.de_mode, g_config.relay_byteswap ? 1u : 0u);
    for (int i = 0; i < g_config.relay_slave_count; i++)
        fprintf(fp, "%s%u", i ? "," : "", g_config.relay_slave_ids[i]);
    fprintf(fp, "],\"dali_slaves\":[");
    for (int i = 0; i < g_config.dali_slave_count; i++)
        fprintf(fp, "%s%u", i ? "," : "", g_config.dali_slave_ids[i]);
    fprintf(fp,
        "],\"mqtt_host\":\"%s\",\"mqtt_port\":%u,\"mqtt_user\":\"%s\","
        "\"mqtt_pass\":\"%s\",\"mqtt_prefix\":\"%s\","
        "\"net_dhcp\":%u,\"net_ip\":\"%u.%u.%u.%u\",\"net_mask\":\"%u.%u.%u.%u\","
        "\"net_gw\":\"%u.%u.%u.%u\",\"net_dns\":\"%u.%u.%u.%u\","
        "\"net_hostname\":\"%s\","
        "\"web_user\":\"%s\",\"web_pass\":\"%s\",\"fw_url\":\"%s\"}\n",
        g_config.mqtt_host, g_config.mqtt_port, g_config.mqtt_user,
        g_config.mqtt_pass, g_config.mqtt_prefix, g_config.net_dhcp,
        g_config.net_ip[0], g_config.net_ip[1], g_config.net_ip[2], g_config.net_ip[3],
        g_config.net_mask[0], g_config.net_mask[1], g_config.net_mask[2], g_config.net_mask[3],
        g_config.net_gw[0], g_config.net_gw[1], g_config.net_gw[2], g_config.net_gw[3],
        g_config.net_dns[0], g_config.net_dns[1], g_config.net_dns[2], g_config.net_dns[3],
        g_config.net_hostname,
        g_config.web_user, g_config.web_pass, g_config.fw_url);
    fclose(fp);
    return true;
}
