/*
 * config.c — global configuration persistence (/config.json).
 *
 * One unified config file holds the controller, modbus, MQTT and network
 * settings (consolidating the former /modbus_baud.txt and /modbus_config.txt).
 * Parsed with jsmn (malloc-free, float-free); written with snprintf/fprintf.
 * Keys are flat (mqtt_host, net_ip, ...) for simple, robust, human-editable JSON.
 */

#include "config.h"
#include "strfit.h"
#include "log.h"
#include "jsmn.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

global_config_t g_config;

#define CONFIG_PATH     "/config.json"
#define CONFIG_PATH_TMP "/config.json.tmp"

/* See config.h for the rule and why this lives here rather than in
 * serial_protocol.c. Whitelist, not a blacklist: every byte must be one of the
 * permitted characters, so anything else — dots, underscores, spaces, control
 * characters, high-bit/UTF-8 bytes — is rejected without having to enumerate
 * it. */
bool config_hostname_ok(const char *s) {
    if (!s) return false;
    size_t n = strlen(s);
    if (n < 1 || n > 31) return false;
    if (s[0] == '-' || s[n - 1] == '-') return false;
    for (size_t i = 0; i < n; i++) {
        char ch = s[i];
        bool good = (ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') ||
                    (ch >= '0' && ch <= '9') || ch == '-';
        if (!good) return false;
    }
    return true;
}

bool config_hostname_set(char *dst, size_t cap, const char *src) {
    /* str_set() carries the refuse-rather-than-truncate rule (strfit.h) and
     * the "leave dst untouched on failure" guarantee; the lowercasing is the
     * only thing specific to a hostname. config_hostname_ok() has already
     * bounded the length at 31, so str_set() cannot fail for any caller in
     * this firmware (they all pass a net_hostname-sized 32-byte buffer) — it
     * stays as the backstop if that ever stops being true. */
    if (!config_hostname_ok(src) || !str_set(dst, cap, src)) return false;
    for (size_t i = 0; dst[i]; i++) {
        char ch = dst[i];
        if (ch >= 'A' && ch <= 'Z') dst[i] = (char)(ch - 'A' + 'a');
    }
    return true;
}

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
    g_config.dali_sweep_interval_ms = 10000;
    g_config.relay_slave_ids[0]  = 1;
    g_config.relay_slave_count   = 1;
    g_config.dali_slave_ids[0]   = 2;
    g_config.dali_slave_ids[1]   = 3;
    g_config.dali_slave_count    = 2;
    g_config.baud                = 38400;
    g_config.de_mode             = 0;
    g_config.relay_byteswap      = false;
    g_config.mqtt_port           = 1883;
    /* str_set() for the compiled defaults too — not because a literal could
     * ever fail to fit, but so no strncpy(dst, src, sizeof dst - 1) is left in
     * the file for the next person to copy as the house pattern. */
    str_set(g_config.mqtt_prefix, sizeof g_config.mqtt_prefix, "lighting");
    g_config.net_dhcp            = 1;
    str_set(g_config.net_hostname, sizeof g_config.net_hostname, "lights");
    str_set(g_config.web_user, sizeof g_config.web_user, "admin");
    str_set(g_config.web_pass, sizeof g_config.web_pass, "admin");
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
/* Returns the element count, or -1 if the key is ABSENT (or not an array).
 * The distinction matters: an EMPTY array is a valid configuration — "this
 * installation has no relay board" / "no DALI gateways" — and must be
 * distinguishable from "the key isn't there, keep the default". Collapsing the
 * two made an empty list impossible to persist: it was settable at runtime and
 * silently reverted to the compiled default on the next boot. */
static int get_u8_array(const char *js, const jsmntok_t *t, int n, const char *key,
                        uint8_t *out, uint8_t max) {
    int i = find_key(js, t, n, key);
    if (i < 0 || t[i].type != JSMN_ARRAY) return -1;
    int count = t[i].size; if (count > max) count = max;
    for (int k = 0; k < count; k++) {
        const jsmntok_t *e = &t[i + 1 + k];
        char b[8]; int len = e->end - e->start; if (len >= (int)sizeof b) len = (int)sizeof b - 1;
        memcpy(b, js + e->start, len); b[len] = '\0';
        out[k] = (uint8_t)strtol(b, NULL, 10);
    }
    return count;
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

/* Load one string field STRICTLY. get_str() clamps to the destination's cap,
 * which destroys the evidence that a value was ever too long, so read into a
 * buffer larger than any config string and let str_set() judge whether it
 * fits. Values written by config_save() always fit by construction, so
 * LS_TOO_LONG only ever means a hand-edited or damaged file — but silently
 * substituting a shorter value there is exactly the failure this whole rule
 * exists to prevent. */
typedef enum { LS_ABSENT, LS_OK, LS_TOO_LONG } load_str_t;

static load_str_t load_str(const char *js, const jsmntok_t *t, int n,
                           const char *key, char *dst, size_t cap) {
    /* Comfortably larger than the biggest config string (fw_url[128]); a
     * value too long for THIS buffer is far too long for any field and is
     * still correctly refused below. */
    char tmp[192];
    if (!get_str(js, t, n, key, tmp, sizeof tmp)) return LS_ABSENT;
    return str_set(dst, cap, tmp) ? LS_OK : LS_TOO_LONG;
}

/* Ordinary field: too long -> keep the compiled default and say so. */
#define LOAD_STR(k, field)                                                      \
    do { if (load_str(buf, tk, nt, k, g_config.field,                           \
                      sizeof g_config.field) == LS_TOO_LONG)                    \
             LOG_WARN("cfg", k " in config exceeds %u chars — keeping default",  \
                      STR_MAX_LEN(sizeof g_config.field)); } while (0)

/* Credential: too long -> do NOT fall back to the compiled default. That
 * would silently re-open the web UI on admin/admin because a config file was
 * damaged, turning a corrupt file into an auth downgrade. Clear it instead:
 * check_auth() (http_server.c) refuses an empty credential, so the device
 * fails CLOSED and stays locked until the config is repaired. */
#define LOAD_CRED(k, field)                                                     \
    do { if (load_str(buf, tk, nt, k, g_config.field,                           \
                      sizeof g_config.field) == LS_TOO_LONG) {                  \
             g_config.field[0] = '\0';                                          \
             LOG_ERR("cfg", k " in config exceeds %u chars — cleared; web UI "  \
                     "stays locked until /config.json is repaired",             \
                     STR_MAX_LEN(sizeof g_config.field)); } } while (0)

bool config_load(void) {
    config_defaults();

    /* A power cut between config_save()'s fclose() and rename() leaves the new
     * config sitting in the .tmp file with the old CONFIG_PATH still intact
     * and authoritative — discard the orphaned .tmp so a later save doesn't
     * confuse it for anything meaningful (mirrors mapping_load(), mapping.c). */
    remove(CONFIG_PATH_TMP);

    FILE *fp = fopen(CONFIG_PATH, "r");
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
            if (get_uint(buf, tk, nt, "dali_sweep_interval_ms", &u)) g_config.dali_sweep_interval_ms = u;
            if (get_uint(buf, tk, nt, "baud", &u) && u > 0)      g_config.baud               = u;
            if (get_uint(buf, tk, nt, "de_mode", &u))            g_config.de_mode            = (uint8_t)u;
            if (get_uint(buf, tk, nt, "relay_byteswap", &u))     g_config.relay_byteswap     = u ? true : false;

            /* >= 0 means the key was present — including an empty list, which
             * legitimately means "no slaves of this kind". Only a missing key
             * leaves the compiled default in place. */
            uint8_t rs[MAX_RELAY_SLAVES];
            int rc = get_u8_array(buf, tk, nt, "relay_slaves", rs, MAX_RELAY_SLAVES);
            if (rc >= 0) { memcpy(g_config.relay_slave_ids, rs, (size_t)rc); g_config.relay_slave_count = (uint8_t)rc; }
            uint8_t ds[MAX_DALI_SLAVES];
            int dc = get_u8_array(buf, tk, nt, "dali_slaves", ds, MAX_DALI_SLAVES);
            if (dc >= 0) { memcpy(g_config.dali_slave_ids, ds, (size_t)dc); g_config.dali_slave_count = (uint8_t)dc; }

            LOAD_STR("mqtt_host",   mqtt_host);
            LOAD_STR("mqtt_user",   mqtt_user);
            LOAD_STR("mqtt_pass",   mqtt_pass);
            LOAD_STR("mqtt_prefix", mqtt_prefix);
            if (get_uint(buf, tk, nt, "mqtt_port", &u) && u > 0) g_config.mqtt_port = (uint16_t)u;

            if (get_uint(buf, tk, nt, "net_dhcp", &u)) g_config.net_dhcp = u ? 1 : 0;
            get_ip(buf, tk, nt, "net_ip",   g_config.net_ip);
            get_ip(buf, tk, nt, "net_mask", g_config.net_mask);
            get_ip(buf, tk, nt, "net_gw",   g_config.net_gw);
            get_ip(buf, tk, nt, "net_dns",  g_config.net_dns);
            /* Validate on the way IN, not just at the CLI/API: before this, a
             * hand-edited or damaged /config.json put arbitrary bytes straight
             * into net_hostname, and netif_init() copies that into HOST_NAME[]
             * and onto the wire as DHCP option 12. Read into a buffer sized
             * like load_str()'s so an over-long value is refused rather than
             * clamped into a valid-looking label; an invalid stored value keeps
             * the default from config_defaults() rather than failing the whole
             * load, consistent with how every other field here degrades. */
            { char hn[192];
              if (get_str(buf, tk, nt, "net_hostname", hn, sizeof hn) &&
                  !config_hostname_set(g_config.net_hostname, sizeof g_config.net_hostname, hn))
                  LOG_WARN("cfg", "net_hostname in config is not a valid label — using \"%s\"",
                           g_config.net_hostname); }

            LOAD_CRED("web_user", web_user);
            LOAD_CRED("web_pass", web_pass);

            LOAD_STR("fw_url", fw_url);
        } else {
            LOG_WARN("config", "config.json parse failed (%d) — using defaults", nt);
        }
    }

    if (migrate_legacy_modbus()) config_save();
    return true;
}

bool config_save(void) {
    /* Write the full config to a .tmp file first, then rename() it over the
     * live path — littlefs rename is a single atomic metadata commit, so a
     * power cut anywhere in this function leaves either the old config or
     * the new one intact, never a truncated one. The old fopen(path, "w")
     * destroyed the old config before the new one was durable: a cut
     * mid-write silently dropped whatever fields came after the truncation
     * point, reverting them to compiled defaults on the next boot —
     * including mqtt/network config, which bricks connectivity until
     * physical access (same defect fixed for mappings in cc19f7b). */
    FILE *fp = fopen(CONFIG_PATH_TMP, "w");
    if (!fp) return false;
    fprintf(fp,
        "{\"long_press_ms\":%u,\"dim_repeat_ms\":%u,\"reversal_window_ms\":%u,"
        "\"gw_diag_interval_ms\":%lu,"
        "\"dali_sweep_interval_ms\":%lu,"
        "\"baud\":%lu,\"de_mode\":%u,\"relay_byteswap\":%u,"
        "\"relay_slaves\":[",
        g_config.long_press_ms, g_config.dim_repeat_ms, g_config.reversal_window_ms,
        (unsigned long)g_config.gw_diag_interval_ms,
        (unsigned long)g_config.dali_sweep_interval_ms,
        (unsigned long)g_config.baud, g_config.de_mode, g_config.relay_byteswap ? 1u : 0u);
    /* A pending rewrite (see config.h) is persisted here, NOT applied to the
     * live relay_slave_ids/dali_slave_ids arrays (or their counts) — those stay
     * whatever they were at boot until the next config_load() re-reads this
     * file. */
    const uint8_t *rs = g_config.relay_slaves_pending ? g_config.pending_relay_slave_ids : g_config.relay_slave_ids;
    uint8_t rc = g_config.relay_slaves_pending ? g_config.pending_relay_slave_count : g_config.relay_slave_count;
    for (int i = 0; i < rc; i++)
        fprintf(fp, "%s%u", i ? "," : "", rs[i]);
    fprintf(fp, "],\"dali_slaves\":[");
    const uint8_t *ds = g_config.dali_slaves_pending ? g_config.pending_dali_slave_ids : g_config.dali_slave_ids;
    uint8_t dc = g_config.dali_slaves_pending ? g_config.pending_dali_slave_count : g_config.dali_slave_count;
    for (int i = 0; i < dc; i++)
        fprintf(fp, "%s%u", i ? "," : "", ds[i]);
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
    /* pico-vfs's littlefs backend exposes no separate fsync(); fclose()
     * reaches lfs_file_close(), which syncs the file's data to flash before
     * returning, so this is the actual durability boundary. */
    bool ok = true;
    if (fflush(fp) != 0) ok = false;
    if (fclose(fp) != 0) ok = false;
    if (!ok) return false;
    if (rename(CONFIG_PATH_TMP, CONFIG_PATH) != 0) {
        LOG_ERR("config", "save failed: rename %s -> %s", CONFIG_PATH_TMP, CONFIG_PATH);
        return false;
    }
    return true;
}
