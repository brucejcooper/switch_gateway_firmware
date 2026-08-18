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
#include "json_str.h"
#include "log.h"
#include "jsmn.h"
#include "jsmn_util.h"
#include "web_auth.h"   /* CFG_HEX validation + legacy web_pass migration */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

global_config_t g_config;

#define CONFIG_PATH     "/config.json"
#define CONFIG_PATH_TMP "/config.json.tmp"

/* See config.h for the rule and why this lives here rather than in
 * cli.c. Whitelist, not a blacklist: every byte must be one of the
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

/* ── Config field schema ─────────────────────────────────────────────────
 * One row per field = one place that knows its name, storage, and limits.
 * Before this table, the same rule was duplicated and drifted: ha_bridge.c's
 * CFG_NUM[] declared long_press_ms/dim_repeat_ms/etc.'s ranges, but only as a
 * min/max HINT for Home Assistant's number-entity widget — nothing in the
 * firmware actually enforced them. The same file's MQTT parse cast straight
 * into the u16 intercore field with `(uint16_t)jint(...)`, so a raw publish
 * of long_press_ms=70000 silently wrapped to 4464 and got stored. cli.c's
 * `cfg` verb used plain atoi() with no bound at all. Three copies of "what is
 * a legal long_press_ms", none of them agreeing, and only one of them (the HA
 * hint) written down anywhere.
 *
 * offset is offsetof(global_config_t, field) — the row IS the field's storage
 * location, so config_apply_u32()/config_apply_str() below can write through
 * a `(char *)&g_config + offset` cast instead of a hand-written switch over
 * every field name. size is the destination buffer's sizeof (strings only —
 * strfit.h's "fits or fails" cap); min/max are the enforced range (numerics
 * only). Making the ranges enforced for the first time (rather than just
 * advertised) is a deliberate behaviour change — see config_load()'s comment
 * and the schema task's report for the full list.
 *
 * Deliberately NOT in this table (existing hand-written code stays exactly as
 * it is): baud (modbus_set_baud() owns the UART divisor + live-apply logic),
 * de_mode (live-apply + self-persist), relay_slaves/dali_slaves (pending,
 * reboot-to-apply rewrite machinery — see the field comment in config.h),
 * net_ip/mask/gw/dns (dotted-quad quartets via jip(), not a single scalar). */
typedef enum {
    CFG_U16,
    CFG_U32,
    CFG_BOOL,
    CFG_STR,
    CFG_HOSTNAME,
    CFG_CRED,
    CFG_HEX,     /* lowercase-hex text of a fixed raw byte length — web_pass_hash/web_salt */
} cfg_kind_t;

typedef struct {
    const char *key;    /* the JSON/CLI field name — same string every layer uses */
    cfg_kind_t  kind;
    size_t      offset; /* offsetof(global_config_t, field) */
    size_t      size;   /* strings: sizeof(field), the strfit.h cap; numerics: unused */
    uint32_t    min, max; /* numerics only (CFG_U16/CFG_U32); ignored otherwise */
} cfg_field_t;

static const cfg_field_t CFG_SCHEMA[] = {
    /* numerics — min/max are what CFG_NUM[] (ha_bridge.c) has always
     * advertised to HA; they become enforced here for the first time. */
    { "long_press_ms",          CFG_U16, offsetof(global_config_t, long_press_ms),          0, 100,   5000   },
    { "dim_repeat_ms",          CFG_U16, offsetof(global_config_t, dim_repeat_ms),          0, 50,    2000   },
    { "reversal_window_ms",     CFG_U16, offsetof(global_config_t, reversal_window_ms),     0, 50,    2000   },
    { "gw_diag_interval_ms",    CFG_U32, offsetof(global_config_t, gw_diag_interval_ms),    0, 0,     600000 },
    { "dali_sweep_interval_ms", CFG_U32, offsetof(global_config_t, dali_sweep_interval_ms), 0, 0,     600000 },
    { "mqtt_port",              CFG_U16, offsetof(global_config_t, mqtt_port),              0, 1,     65535  },

    /* bool */
    { "relay_byteswap", CFG_BOOL, offsetof(global_config_t, relay_byteswap), 0, 0, 1 },

    /* ordinary strings — too long keeps the compiled default (LOAD_STR rule) */
    { "mqtt_host",   CFG_STR, offsetof(global_config_t, mqtt_host),   sizeof g_config.mqtt_host,   0, 0 },
    { "mqtt_user",   CFG_STR, offsetof(global_config_t, mqtt_user),   sizeof g_config.mqtt_user,   0, 0 },
    { "mqtt_pass",   CFG_STR, offsetof(global_config_t, mqtt_pass),   sizeof g_config.mqtt_pass,   0, 0 },
    { "mqtt_prefix", CFG_STR, offsetof(global_config_t, mqtt_prefix), sizeof g_config.mqtt_prefix, 0, 0 },
    { "fw_url",      CFG_STR, offsetof(global_config_t, fw_url),      sizeof g_config.fw_url,      0, 0 },

    /* hostname — config_hostname_set() owns the charset + lowercasing rule */
    { "net_hostname", CFG_HOSTNAME, offsetof(global_config_t, net_hostname), sizeof g_config.net_hostname, 0, 0 },

    /* credentials — too long fails CLOSED on load (LOAD_CRED rule below);
     * an empty value is refused on apply (see config_apply_str()). */
    { "web_user", CFG_CRED, offsetof(global_config_t, web_user), sizeof g_config.web_user, 0, 0 },

    /* web_pass_hash/web_salt — lowercase hex text of the Argon2id hash and
     * its salt (web_auth.h). Wrong length or non-hex content fails CLOSED on
     * load exactly like a CRED (config_load()'s CFG_HEX case below), for the
     * same reason: a damaged config must not silently reopen the web UI, and
     * there is no compiled-default hash to "fall back to" that wouldn't ALSO
     * have to be a real Argon2id output. Neither is ever written by a plain
     * config_apply_str() call from outside config.c — the only writers are
     * config_load()'s own legacy-password migration and the dedicated
     * hashing setter (ic_seam.c's IC_CMD_SET_AUTH); this row exists so both
     * still load/save/validate through the ONE schema-driven path every
     * other field does, instead of a hand-written third code path. */
    { "web_pass_hash", CFG_HEX, offsetof(global_config_t, web_pass_hash), sizeof g_config.web_pass_hash, 0, 0 },
    { "web_salt",      CFG_HEX, offsetof(global_config_t, web_salt),      sizeof g_config.web_salt,      0, 0 },
};
#define CFG_SCHEMA_N (sizeof CFG_SCHEMA / sizeof CFG_SCHEMA[0])

static const cfg_field_t *cfg_field_find(const char *key) {
    if (!key) return NULL;
    for (size_t i = 0; i < CFG_SCHEMA_N; i++)
        if (strcmp(CFG_SCHEMA[i].key, key) == 0) return &CFG_SCHEMA[i];
    return NULL;
}

cfg_apply_result_t config_apply_u32(const char *key, uint32_t value) {
    const cfg_field_t *f = cfg_field_find(key);
    if (!f) return CFG_APPLY_UNKNOWN_KEY;
    void *p = (char *)&g_config + f->offset;
    switch (f->kind) {
    case CFG_U16:
        if (value < f->min || value > f->max) return CFG_APPLY_OUT_OF_RANGE;
        *(uint16_t *)p = (uint16_t)value;
        return CFG_APPLY_OK;
    case CFG_U32:
        if (value < f->min || value > f->max) return CFG_APPLY_OUT_OF_RANGE;
        *(uint32_t *)p = value;
        return CFG_APPLY_OK;
    case CFG_BOOL:
        /* No range to enforce — any nonzero value is "on", same as the
         * hand-written `v ? true : false` every caller used before. */
        *(bool *)p = value ? true : false;
        return CFG_APPLY_OK;
    default:
        return CFG_APPLY_INVALID;  /* a string-kind key used with the numeric API */
    }
}

cfg_apply_result_t config_apply_str(const char *key, const char *value) {
    if (!value) return CFG_APPLY_INVALID;
    const cfg_field_t *f = cfg_field_find(key);
    if (!f) return CFG_APPLY_UNKNOWN_KEY;
    char *dst = (char *)&g_config + f->offset;
    switch (f->kind) {
    case CFG_STR:
        return str_set(dst, f->size, value) ? CFG_APPLY_OK : CFG_APPLY_TOO_LONG;
    case CFG_CRED:
        /* A blank credential is never a legitimate write — refusing it here
         * backstops every caller, the same way the both-or-neither pairing
         * in ic_seam.c's IC_CMD_SET_AUTH backstops a half-applied change. */
        if (!value[0]) return CFG_APPLY_INVALID;
        return str_set(dst, f->size, value) ? CFG_APPLY_OK : CFG_APPLY_TOO_LONG;
    case CFG_HOSTNAME:
        return config_hostname_set(dst, f->size, value) ? CFG_APPLY_OK : CFG_APPLY_INVALID;
    case CFG_HEX:
        /* explen = the raw byte count this field holds (size is sizeof the
         * hex-text field, i.e. 2*explen + 1 for the NUL). Only config.c's own
         * load/migration paths ever call this for "web_pass_hash"/"web_salt"
         * (see the schema row's comment) — checked here anyway, same as
         * every other kind, rather than trusted because today's callers
         * happen to behave. */
        if (!web_auth_hex_valid(value, (f->size - 1) / 2)) return CFG_APPLY_INVALID;
        return str_set(dst, f->size, value) ? CFG_APPLY_OK : CFG_APPLY_TOO_LONG;
    default:
        return CFG_APPLY_INVALID;  /* a numeric-kind key used with the string API */
    }
}

bool config_field_range(const char *key, uint32_t *min, uint32_t *max) {
    const cfg_field_t *f = cfg_field_find(key);
    if (!f || (f->kind != CFG_U16 && f->kind != CFG_U32)) return false;
    if (min) *min = f->min;
    if (max) *max = f->max;
    return true;
}

unsigned config_field_maxlen(const char *key) {
    const cfg_field_t *f = cfg_field_find(key);
    if (!f || (f->kind != CFG_STR && f->kind != CFG_CRED &&
               f->kind != CFG_HOSTNAME && f->kind != CFG_HEX)) return 0;
    return STR_MAX_LEN(f->size);
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
    /* Precomputed Argon2id("admin", WEB_AUTH_DEFAULT_SALT_HEX) — see
     * web_auth.c for the KDF parameters (nb_blocks=32, nb_passes=3,
     * nb_lanes=1, Argon2id). The salt is a FIXED, public constant rather
     * than a fresh random one: it is exactly as public as the factory
     * default password itself always was ("admin" is printed in this repo's
     * own README), so there is nothing to gain from randomising it, and a
     * literal here means the compiled default costs a str_set(), not an
     * Argon2 call, on every boot that hasn't yet loaded a real
     * /config.json. Regenerate both literals together if WEB_AUTH_NB_*
     * ever changes — trivial with a throwaway host program: call
     * web_auth_gen_salt() once to pick a new constant (or keep this one),
     * then web_auth_hash("admin", that salt, ..., 0) and hex-encode both. */
    str_set(g_config.web_salt, sizeof g_config.web_salt,
            "6c6967687473776974636864666c7430");   /* ASCII "lightswitchdflt0" */
    str_set(g_config.web_pass_hash, sizeof g_config.web_pass_hash,
            "05143c762a259de70d8e6e2eac4b0a857d816fa43071e968e24680f32d39bd8c");
}

/* ── jsmn helpers ────────────────────────────────────────────────────────── */
/* jtok_eq/jfind/jstr_get/jip are the shared helpers, jsmn_util.h — see that
 * header for the jstr_key()/get_str() clamp-bug history (e875365, 23761ba)
 * this file used to have its own copy of. get_uint() and get_u8_array() stay
 * here: neither is duplicated in http_server.c/ha_bridge.c (both parse
 * numeric fields with the simpler, unvalidated jint()/jint_key() instead —
 * config-load wants get_uint()'s stricter "reject, don't default, on
 * malformed input" contract, which is config.c-specific), and
 * get_u8_array() is tied to this file's MAX_RELAY_SLAVES/MAX_DALI_SLAVES
 * slave-list shape. */
static bool get_uint(const char *js, const jsmntok_t *t, int n, const char *key, unsigned *out) {
    int i = jfind(js, t, n, key);
    if (i < 0) return false;
    char b[16]; int len = t[i].end - t[i].start; if (len >= (int)sizeof b) len = (int)sizeof b - 1;
    memcpy(b, js + t[i].start, len); b[len] = '\0';
    char *e; long v = strtol(b, &e, 10);
    if (e == b || v < 0) return false;
    *out = (unsigned)v; return true;
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
    int i = jfind(js, t, n, key);
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

/* jstr_get_unescaped() (jsmn_util.h) is the "fits-or-fails, dst untouched
 * otherwise" extractor the CFG_STR/CFG_CRED/CFG_HOSTNAME cases below load
 * through — jsmn knows the token's true length, so it can measure the
 * DECODED length and compare directly against the destination field's own
 * cap, refusing before copying a byte. That replaces the old load_str()/
 * tmp[192] two-step (read into an oversized scratch buffer, then let
 * str_set() judge whether it fits the real field): with a four-state
 * extractor there is nothing left for a second step to do.
 * Values written by config_save() always fit AND are always validly escaped
 * by construction (json_escape(), config.c), so JSTR_TOO_LONG/JSTR_MALFORMED
 * here only ever mean a hand-edited or damaged file — but silently
 * substituting a shorter or differently-decoded value is exactly the failure
 * this whole rule (and e875365/23761ba, plus the config_save() corruption
 * story in json_str.h) exists to prevent. jstr_get_unescaped() is jstr_get()'s
 * unescaping twin (jsmn_util.h) — config.c is the one caller in this firmware
 * that needs the DECODED four-state contract on load; every other consumer of
 * a config string still goes through plain jstr_get()/jstr_set(). */

bool config_load(void) {
    config_defaults();

    /* Set by the legacy-plaintext-"web_pass" migration below (schema-loop
     * scope, but the resave decision at the bottom of this function needs
     * it) — see that block's own comment. */
    bool web_pass_migrated = false;

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

            /* Every CFG_SCHEMA field loads through this one loop instead of
             * one hand-written line per field — see the schema's own header
             * comment (above cfg_field_t, top of this file) for the drift
             * story that made that necessary. Each kind degrades exactly the
             * way it always did, with one deliberate addition (flagged
             * below):
             *   - numeric/bool: absent, or present-but-malformed (get_uint's
             *     own "reject, don't default" contract) -> keep the compiled
             *     default, no warning — unchanged.
             *     Present-but-OUT-OF-RANGE is NEW: the schema's ranges used
             *     to be only a UI hint (ha_bridge.c's CFG_NUM[]) and were
             *     never checked on load at all, so e.g. a hand-edited
             *     long_press_ms=70000 loaded verbatim. Now it is rejected
             *     like any other bad value — default kept, warning logged —
             *     which is a behaviour change worth watching for on upgrade
             *     (a config file already carrying an out-of-range value will
             *     start logging on every boot until it's corrected).
             *   - CFG_STR: too long (decoded) -> keep the default and warn
             *     (was LOAD_STR). A MALFORMED escape (dangling backslash,
             *     bad \uXXXX, a lone surrogate — jstr_get_unescaped()'s new
             *     4th state) degrades the SAME way: keep the default and
             *     warn. config_save() never writes a malformed escape, so
             *     reaching this means the file was hand-edited or damaged —
             *     exactly the case the too-long path already exists for.
             *   - CFG_CRED: too long (decoded) OR malformed -> do NOT fall
             *     back to the compiled default. That would silently re-open
             *     the web UI on admin/admin because a config file was
             *     damaged, turning a corrupt file into an auth downgrade.
             *     Clear it instead: check_auth() (http_server.c) refuses an
             *     empty credential, so the device fails CLOSED and stays
             *     locked until the config is repaired (was LOAD_CRED). A
             *     malformed escape gets exactly the same fail-closed
             *     treatment as too-long — a credential jstr_get_unescaped()
             *     can't make sense of must never be treated as "absent,
             *     keep default", for the same reason.
             *   - CFG_HOSTNAME: not a valid label (including too-long or a
             *     malformed escape) -> keep the default and warn. Reads into
             *     a 192-byte scratch buffer — generously larger than
             *     net_hostname's own 32 — so an over-long value is captured
             *     in FULL and correctly rejected as "too long" by
             *     config_hostname_ok(), rather than silently pre-clamped
             *     into a valid-looking 31-char prefix that would then
             *     wrongly pass. Writing straight into the live field would
             *     also leave an unvalidated raw value sitting there the
             *     instant config_hostname_ok() rejects it, in place of the
             *     safe default. (A malformed escape can't actually produce a
             *     valid hostname even if it decoded — config_hostname_ok()'s
             *     charset rejects raw backslashes/quotes/control bytes
             *     regardless — but jstr_get_unescaped() is checked first so
             *     the log names the real cause.)
             *   - CFG_HEX: too long, malformed escape, OR not exactly the
             *     expected count of hex digits -> fail CLOSED exactly like
             *     CFG_CRED (clear; see that bullet) rather than keep the
             *     compiled default. A field that doesn't decode to a real
             *     Argon2id hash/salt is not "absent, use the default" — it
             *     is either a hand-damaged file or (in web_pass_hash's case)
             *     the tail end of a migration that didn't complete, and in
             *     both cases silently falling back to the compiled
             *     admin/admin hash would be the same auth downgrade the
             *     CRED rule exists to prevent. */
            for (size_t si = 0; si < CFG_SCHEMA_N; si++) {
                const cfg_field_t *f = &CFG_SCHEMA[si];
                switch (f->kind) {
                case CFG_U16: case CFG_U32: case CFG_BOOL: {
                    if (!get_uint(buf, tk, nt, f->key, &u)) break;
                    cfg_apply_result_t r = config_apply_u32(f->key, u);
                    if (r == CFG_APPLY_OUT_OF_RANGE)
                        LOG_WARN("cfg", "%s in config is out of range (%lu..%lu) — keeping default",
                                 f->key, (unsigned long)f->min, (unsigned long)f->max);
                    break;
                }
                case CFG_STR: {
                    char *dst = (char *)&g_config + f->offset;
                    jstr_status_t s = jstr_get_unescaped(buf, tk, nt, f->key, dst, f->size);
                    if (s == JSTR_TOO_LONG)
                        LOG_WARN("cfg", "%s in config exceeds %u chars — keeping default",
                                 f->key, STR_MAX_LEN(f->size));
                    else if (s == JSTR_MALFORMED)
                        LOG_WARN("cfg", "%s in config has a malformed JSON escape — keeping default",
                                 f->key);
                    break;
                }
                case CFG_CRED: {
                    char *dst = (char *)&g_config + f->offset;
                    jstr_status_t s = jstr_get_unescaped(buf, tk, nt, f->key, dst, f->size);
                    if (s == JSTR_TOO_LONG) {
                        dst[0] = '\0';
                        LOG_ERR("cfg", "%s in config exceeds %u chars — cleared; web UI "
                                "stays locked until /config.json is repaired",
                                f->key, STR_MAX_LEN(f->size));
                    } else if (s == JSTR_MALFORMED) {
                        dst[0] = '\0';
                        LOG_ERR("cfg", "%s in config has a malformed JSON escape — cleared; web UI "
                                "stays locked until /config.json is repaired",
                                f->key);
                    }
                    break;
                }
                case CFG_HEX: {
                    char *dst = (char *)&g_config + f->offset;
                    jstr_status_t s = jstr_get_unescaped(buf, tk, nt, f->key, dst, f->size);
                    if (s == JSTR_OK && !web_auth_hex_valid(dst, (f->size - 1) / 2))
                        s = JSTR_MALFORMED;   /* right length category, wrong content — same fail-closed path */
                    if (s == JSTR_TOO_LONG) {
                        dst[0] = '\0';
                        LOG_ERR("cfg", "%s in config exceeds %u chars — cleared; web UI "
                                "stays locked until a new password is set",
                                f->key, STR_MAX_LEN(f->size));
                    } else if (s == JSTR_MALFORMED) {
                        dst[0] = '\0';
                        LOG_ERR("cfg", "%s in config is not valid hex — cleared; web UI "
                                "stays locked until a new password is set",
                                f->key);
                    }
                    break;
                }
                case CFG_HOSTNAME: {
                    char *dst = (char *)&g_config + f->offset;
                    char hn[192];
                    jstr_status_t hs = jstr_get_unescaped(buf, tk, nt, f->key, hn, sizeof hn);
                    bool bad = (hs == JSTR_TOO_LONG) || (hs == JSTR_MALFORMED) ||
                               (hs == JSTR_OK && !config_hostname_set(dst, f->size, hn));
                    if (bad)
                        LOG_WARN("cfg", "%s in config is not a valid label — using \"%s\"",
                                 f->key, dst);
                    break;
                }
                }
            }

            /* One-time migration: a pre-hash /config.json stores the web-UI
             * password in PLAINTEXT under the (now-retired) "web_pass" key.
             * If present, hash it now with a FRESH salt (point 2's rule:
             * generate a salt only when a password is actually being set —
             * migrating one counts), write web_pass_hash/web_salt, and set
             * web_pass_migrated so this function's caller re-saves once —
             * config_save() only ever emits the hashed fields, so "web_pass"
             * is never written back and this whole block runs at most once
             * per device. web_auth_hash() is called with retry_ms=0 (a
             * single attempt): core 1 cannot yet be inside a KDF call at
             * this point in boot (see web_auth.c's header comment — core 1
             * is still parked on the config-ready gate), so the shared lock
             * is uncontended by construction and a retry could never help.
             * Both the plaintext temporary and the raw hash are wiped right
             * after use, same as every other plaintext-candidate call site. */
            {
                char legacy[48];
                jstr_status_t ws = jstr_get_unescaped(buf, tk, nt, "web_pass", legacy, sizeof legacy);
                if (ws == JSTR_OK) {
                    web_pass_migrated = true;
                    if (legacy[0]) {
                        uint8_t salt[WEB_AUTH_SALT_LEN], hash[WEB_AUTH_HASH_LEN];
                        web_auth_gen_salt(salt);
                        if (web_auth_hash(legacy, salt, hash, 0)) {
                            char salt_hex[33], hash_hex[65];
                            web_auth_hex_encode(salt, sizeof salt, salt_hex);
                            web_auth_hex_encode(hash, sizeof hash, hash_hex);
                            str_set(g_config.web_salt,      sizeof g_config.web_salt,      salt_hex);
                            str_set(g_config.web_pass_hash, sizeof g_config.web_pass_hash, hash_hex);
                            LOG_INFO("cfg", "migrated legacy plaintext web_pass -> web_pass_hash");
                        } else {
                            /* Lock genuinely unavailable (should not happen — see
                             * above) or a future caller changes the retry budget:
                             * fail closed exactly like the CFG_HEX load-time rule,
                             * not "leave the compiled admin/admin default in place". */
                            g_config.web_pass_hash[0] = '\0';
                            LOG_ERR("cfg", "legacy web_pass migration failed — web UI stays "
                                    "locked until a new password is set");
                        }
                        web_auth_wipe(hash, sizeof hash);
                    } else {
                        /* An empty legacy web_pass was already the fail-closed
                         * marker under the old CFG_CRED rule — carry that
                         * forward as an empty hash rather than hashing "". */
                        g_config.web_pass_hash[0] = '\0';
                    }
                    web_auth_wipe(legacy, sizeof legacy);
                } else if (ws == JSTR_TOO_LONG || ws == JSTR_MALFORMED) {
                    /* Same fail-closed treatment the old CFG_CRED rule gave a
                     * damaged web_pass — clear rather than guess, and still
                     * drop the damaged legacy key by resaving. */
                    g_config.web_pass_hash[0] = '\0';
                    LOG_ERR("cfg", "legacy web_pass in config is damaged — cleared; web UI "
                            "stays locked until a new password is set");
                    web_pass_migrated = true;
                }
            }

            /* Fields NOT in CFG_SCHEMA (baud, de_mode, the slave lists, the
             * network quartets) — see the schema's own header comment for
             * why each stays hand-written; unchanged from before this
             * refactor. */
            if (get_uint(buf, tk, nt, "baud", &u) && u > 0)      g_config.baud    = u;
            if (get_uint(buf, tk, nt, "de_mode", &u))            g_config.de_mode = (uint8_t)u;

            /* >= 0 means the key was present — including an empty list, which
             * legitimately means "no slaves of this kind". Only a missing key
             * leaves the compiled default in place. */
            uint8_t rs[MAX_RELAY_SLAVES];
            int rc = get_u8_array(buf, tk, nt, "relay_slaves", rs, MAX_RELAY_SLAVES);
            if (rc >= 0) { memcpy(g_config.relay_slave_ids, rs, (size_t)rc); g_config.relay_slave_count = (uint8_t)rc; }
            uint8_t ds[MAX_DALI_SLAVES];
            int dc = get_u8_array(buf, tk, nt, "dali_slaves", ds, MAX_DALI_SLAVES);
            if (dc >= 0) { memcpy(g_config.dali_slave_ids, ds, (size_t)dc); g_config.dali_slave_count = (uint8_t)dc; }

            if (get_uint(buf, tk, nt, "net_dhcp", &u)) g_config.net_dhcp = u ? 1 : 0;
            jip(buf, tk, nt, "net_ip",   g_config.net_ip);
            jip(buf, tk, nt, "net_mask", g_config.net_mask);
            jip(buf, tk, nt, "net_gw",   g_config.net_gw);
            jip(buf, tk, nt, "net_dns",  g_config.net_dns);
        } else {
            LOG_WARN("config", "config.json parse failed (%d) — using defaults", nt);
        }
    }

    if (migrate_legacy_modbus() || web_pass_migrated) config_save();
    return true;
}

/* config_save() below is NOT schema-driven — its single fprintf is correct
 * and complete, and rewriting a hand-tuned format string into a loop bought
 * nothing but risk (see the task report for the full reasoning). This pins
 * the one thing cheap to check at compile time: the schema's field COUNT, so
 * an add/remove there — 16 fields today — forces a look at config_save()'s
 * fprintf instead of the two silently drifting apart. It cannot catch a
 * field that changes NAME or TYPE without changing the row count; that half
 * of save/schema consistency rests on review. */
_Static_assert(CFG_SCHEMA_N == 16,
    "CFG_SCHEMA changed size — update config_save()'s fprintf (config.c) to match");

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
    fprintf(fp, "],");

    /* Every string field from here on goes through json_escape() into this
     * one scratch buffer before being printed. Before this, config_save()
     * fprintf'd each field's raw bytes straight between the surrounding
     * quotes — a single `"` or `\` in, say, an MQTT password (set validly
     * over the CLI, which has never restricted the character set) corrupted
     * the whole document: jsmn_parse() fails on the next boot and
     * config_load() falls back to config_defaults() for EVERY field,
     * including web_user/web_pass — one bad byte in one field silently
     * reopened the web UI on admin/admin. See json_str.h's header comment
     * for the full story.
     *
     * One shared buffer, not one per field: each field is escaped and
     * printed immediately in its own fprintf() before the next field is
     * escaped into the same memory, so only one field's escaped text needs
     * to exist at a time. Sized for the largest field (fw_url[128]) at RFC
     * 8259's worst case — every byte expands to `\u00XX`, 6 bytes — so
     * 6*127+1 = 763 covers fw_url's real 127-char maximum with a little
     * headroom to 769; that covers every other, smaller string field a
     * fortiori. Lives on config_save's stack, which runs on core 0's main
     * loop — 769 bytes there is unremarkable.
     *
     * SAVE_ESC() escaping a field can't actually fail: every field already
     * satisfies its own strfit.h cap (str_set()/config_apply_str() enforce
     * that on every write path, long before it reaches here), and esc[] is
     * sized for the worst-case expansion of the largest of them. But per
     * this codebase's "never trust shouldn't-happen" rule (see e.g.
     * get_uint()'s handling of a malformed numeric token, above), a failure
     * here still aborts the save cleanly rather than assert or silently
     * write a short/wrong field: close and discard the half-written .tmp
     * file and return false WITHOUT renaming, so a failure here leaves
     * CONFIG_PATH exactly as it was — config_load() keeps reading the last
     * good file instead of ever seeing a truncated one. */
    char esc[6 * 128 + 1];
#define SAVE_ESC(field) \
    do { \
        if (!json_escape(esc, sizeof esc, (field))) { \
            LOG_ERR("config", "save failed: could not escape %s (bug — see json_str.h)", #field); \
            fclose(fp); remove(CONFIG_PATH_TMP); return false; \
        } \
    } while (0)

    SAVE_ESC(g_config.mqtt_host);
    fprintf(fp, "\"mqtt_host\":\"%s\",\"mqtt_port\":%u,", esc, g_config.mqtt_port);
    SAVE_ESC(g_config.mqtt_user);
    fprintf(fp, "\"mqtt_user\":\"%s\",", esc);
    SAVE_ESC(g_config.mqtt_pass);
    fprintf(fp, "\"mqtt_pass\":\"%s\",", esc);
    SAVE_ESC(g_config.mqtt_prefix);
    fprintf(fp, "\"mqtt_prefix\":\"%s\",", esc);
    fprintf(fp,
        "\"net_dhcp\":%u,\"net_ip\":\"%u.%u.%u.%u\",\"net_mask\":\"%u.%u.%u.%u\","
        "\"net_gw\":\"%u.%u.%u.%u\",\"net_dns\":\"%u.%u.%u.%u\",",
        g_config.net_dhcp,
        g_config.net_ip[0], g_config.net_ip[1], g_config.net_ip[2], g_config.net_ip[3],
        g_config.net_mask[0], g_config.net_mask[1], g_config.net_mask[2], g_config.net_mask[3],
        g_config.net_gw[0], g_config.net_gw[1], g_config.net_gw[2], g_config.net_gw[3],
        g_config.net_dns[0], g_config.net_dns[1], g_config.net_dns[2], g_config.net_dns[3]);
    SAVE_ESC(g_config.net_hostname);
    fprintf(fp, "\"net_hostname\":\"%s\",", esc);
    SAVE_ESC(g_config.web_user);
    fprintf(fp, "\"web_user\":\"%s\",", esc);
    /* web_pass_hash/web_salt are lowercase hex — json_escape() is a no-op for
     * that charset, but going through it anyway keeps this file's rule
     * ("every string field is escaped before it's printed") true without a
     * carved-out exception for these two. */
    SAVE_ESC(g_config.web_pass_hash);
    fprintf(fp, "\"web_pass_hash\":\"%s\",", esc);
    SAVE_ESC(g_config.web_salt);
    fprintf(fp, "\"web_salt\":\"%s\",", esc);
    SAVE_ESC(g_config.fw_url);
    fprintf(fp, "\"fw_url\":\"%s\"}\n", esc);
#undef SAVE_ESC
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
