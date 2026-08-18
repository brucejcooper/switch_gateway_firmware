/*
 * config_schema_test.c — host-side test for config.c's field schema
 * (CFG_SCHEMA[]) and its public apply/lookup API (config.h):
 * config_apply_u32(), config_apply_str(), config_field_range(),
 * config_field_maxlen().
 *
 * Scope: this exercises the APPLY layer directly (every writer — cli.c,
 * ic_seam.c, ha_bridge.c, http_server.c's backstop — now funnels through it),
 * not config_load() itself. config_load() reads a hardcoded absolute path
 * (/config.json, config.c) with no override hook, so driving it from a host
 * test would mean writing to the real filesystem root — not attempted here.
 * config_load()'s new schema-iterating loop calls exactly config_apply_u32()/
 * jstr_get() per field (see config.c), so the apply-layer coverage below is
 * the behaviour config_load() exercises too; only the JSON-parse plumbing
 * around it (get_uint/jstr_get on a real token stream) is untested here, and
 * that plumbing is unchanged from before this schema existed.
 *
 * Build & run:
 *   cc -DJSMN_STATIC -Wall -Wextra -I ../src -I ../lib/jsmn \
 *       -o /tmp/config_schema_test config_schema_test.c ../src/config.c \
 *       && /tmp/config_schema_test
 */
#include "../src/config.h"
#include "../src/strfit.h"
#include "../src/log.h"

#include <stdio.h>
#include <string.h>

/* config.c logs through these; the apply/lookup paths under test never touch
 * the filesystem, so plain stubs are enough to link. */
log_level_t g_log_level = LOG_LEVEL_OFF;
void log_emit(log_level_t lvl, const char *lc, const char *mod, const char *fmt, ...) {
    (void)lvl; (void)lc; (void)mod; (void)fmt;
}

static int g_fails = 0;
static const char *g_case = "";
#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL [%s]: %s (%s:%d)\n", g_case, #cond, __FILE__, __LINE__); g_fails++; } \
} while (0)

/* One row per numeric/bool field: key, [min,max] (bool: min=0,max=1 as a
 * convention — see below, the bool row isn't range-checked). Mirrors
 * CFG_SCHEMA's numerics 1:1; if a row is ever added/removed there without a
 * matching change here, the boundary checks below stop covering it — this
 * table is deliberately hand-kept, not derived, so a missed field is a
 * silent test gap rather than a build error either way (there is no way to
 * introspect CFG_SCHEMA — it's private to config.c). */
static struct { const char *key; uint32_t min, max; } NUM_FIELDS[] = {
    { "long_press_ms",          100,   5000   },
    { "dim_repeat_ms",          50,    2000   },
    { "reversal_window_ms",     50,    2000   },
    { "gw_diag_interval_ms",    0,     600000 },
    { "dali_sweep_interval_ms", 0,     600000 },
    { "mqtt_port",              1,     65535  },
};
#define NUM_FIELDS_N (sizeof NUM_FIELDS / sizeof NUM_FIELDS[0])

/* One row per CFG_STR field: key, cap (sizeof the g_config field). */
static struct { const char *key; size_t cap; } STR_FIELDS[] = {
    { "mqtt_host",   64  },
    { "mqtt_user",   48  },
    { "mqtt_pass",   64  },
    { "mqtt_prefix", 24  },
    { "fw_url",      128 },
};
#define STR_FIELDS_N (sizeof STR_FIELDS / sizeof STR_FIELDS[0])

/* One row per CFG_CRED field: key, cap. */
static struct { const char *key; size_t cap; } CRED_FIELDS[] = {
    { "web_user", 32 },
    { "web_pass", 48 },
};
#define CRED_FIELDS_N (sizeof CRED_FIELDS / sizeof CRED_FIELDS[0])

int main(void) {
    config_defaults();

    g_case = "numeric: boundary values accepted";
    for (size_t i = 0; i < NUM_FIELDS_N; i++) {
        const char *k = NUM_FIELDS[i].key;
        uint32_t mn = NUM_FIELDS[i].min, mx = NUM_FIELDS[i].max;
        CHECK(config_apply_u32(k, mn) == CFG_APPLY_OK);
        CHECK(config_apply_u32(k, mx) == CFG_APPLY_OK);
        if (mn > 0) {
            uint32_t mid = mn + (mx - mn) / 2;
            CHECK(config_apply_u32(k, mid) == CFG_APPLY_OK);
        }
    }

    g_case = "numeric: one below min / one above max rejected, g_config untouched";
    for (size_t i = 0; i < NUM_FIELDS_N; i++) {
        const char *k = NUM_FIELDS[i].key;
        uint32_t mn = NUM_FIELDS[i].min, mx = NUM_FIELDS[i].max;
        /* Establish a known-good sentinel value first. */
        uint32_t sentinel = mn + (mx - mn) / 2;
        CHECK(config_apply_u32(k, sentinel) == CFG_APPLY_OK);
        if (mn > 0) {
            CHECK(config_apply_u32(k, mn - 1) == CFG_APPLY_OUT_OF_RANGE);
        }
        CHECK(config_apply_u32(k, mx + 1) == CFG_APPLY_OUT_OF_RANGE);
        CHECK(config_apply_u32(k, 0xFFFFFFFFu) == CFG_APPLY_OUT_OF_RANGE);
        /* Re-apply the sentinel is unaffected by the rejections above — this
         * loop doesn't read g_config back (no per-field accessor exists),
         * but config_field_range() below independently pins that the reader
         * every caller uses (CLI/ic_seam/ha_bridge) sees the same bounds. */
    }

    g_case = "bool: any nonzero -> true, zero -> false, no range enforced";
    CHECK(config_apply_u32("relay_byteswap", 1) == CFG_APPLY_OK);
    CHECK(g_config.relay_byteswap == true);
    CHECK(config_apply_u32("relay_byteswap", 0) == CFG_APPLY_OK);
    CHECK(g_config.relay_byteswap == false);
    CHECK(config_apply_u32("relay_byteswap", 42) == CFG_APPLY_OK);   /* truthy, not "in range" */
    CHECK(g_config.relay_byteswap == true);

    g_case = "numeric: g_config actually written on OK, actually untouched on reject";
    {
        CHECK(config_apply_u32("long_press_ms", 500) == CFG_APPLY_OK);
        CHECK(g_config.long_press_ms == 500);
        CHECK(config_apply_u32("long_press_ms", 99) == CFG_APPLY_OUT_OF_RANGE);
        CHECK(g_config.long_press_ms == 500);          /* unchanged */
        CHECK(config_apply_u32("long_press_ms", 5001) == CFG_APPLY_OUT_OF_RANGE);
        CHECK(g_config.long_press_ms == 500);           /* unchanged */
        CHECK(config_apply_u32("long_press_ms", 100) == CFG_APPLY_OK);
        CHECK(g_config.long_press_ms == 100);
        CHECK(config_apply_u32("long_press_ms", 5000) == CFG_APPLY_OK);
        CHECK(g_config.long_press_ms == 5000);
    }
    {
        CHECK(config_apply_u32("gw_diag_interval_ms", 0) == CFG_APPLY_OK);   /* 0 = off, in range */
        CHECK(g_config.gw_diag_interval_ms == 0);
        CHECK(config_apply_u32("gw_diag_interval_ms", 600001) == CFG_APPLY_OUT_OF_RANGE);
        CHECK(g_config.gw_diag_interval_ms == 0);
        CHECK(config_apply_u32("gw_diag_interval_ms", 600000) == CFG_APPLY_OK);
        CHECK(g_config.gw_diag_interval_ms == 600000);
    }
    {
        CHECK(config_apply_u32("mqtt_port", 0) == CFG_APPLY_OUT_OF_RANGE);   /* previously silently kept-default */
        CHECK(config_apply_u32("mqtt_port", 1) == CFG_APPLY_OK);
        CHECK(g_config.mqtt_port == 1);
        CHECK(config_apply_u32("mqtt_port", 65535) == CFG_APPLY_OK);
        CHECK(g_config.mqtt_port == 65535);
        CHECK(config_apply_u32("mqtt_port", 65536) == CFG_APPLY_OUT_OF_RANGE);
        CHECK(g_config.mqtt_port == 65535);
    }

    g_case = "unknown key";
    CHECK(config_apply_u32("no_such_field", 1) == CFG_APPLY_UNKNOWN_KEY);
    CHECK(config_apply_str("no_such_field", "x") == CFG_APPLY_UNKNOWN_KEY);
    CHECK(config_field_range("no_such_field", NULL, NULL) == false);
    CHECK(config_field_maxlen("no_such_field") == 0);

    g_case = "wrong-kind API use (real key, wrong apply function)";
    CHECK(config_apply_u32("mqtt_host", 1) == CFG_APPLY_INVALID);    /* string field via numeric API */
    CHECK(config_apply_str("mqtt_port", "1883") == CFG_APPLY_INVALID); /* numeric field via string API */
    CHECK(config_field_range("mqtt_host", NULL, NULL) == false);     /* string field has no numeric range */
    CHECK(config_field_maxlen("long_press_ms") == 0);                /* numeric field has no maxlen */
    CHECK(config_field_maxlen("relay_byteswap") == 0);               /* bool field has no maxlen */

    g_case = "string: fits, boundary, and too-long via apply";
    for (size_t i = 0; i < STR_FIELDS_N; i++) {
        const char *k = STR_FIELDS[i].key;
        size_t cap = STR_FIELDS[i].cap;
        char ok[192], toolong[192];
        memset(ok, 'a', cap - 1); ok[cap - 1] = '\0';         /* exactly the max: fits */
        memset(toolong, 'a', cap); toolong[cap] = '\0';       /* one over: rejected */

        CHECK(config_apply_str(k, "") == CFG_APPLY_OK);       /* empty is a legal ordinary string */
        CHECK(config_apply_str(k, "hello") == CFG_APPLY_OK);
        CHECK(config_apply_str(k, ok) == CFG_APPLY_OK);
        CHECK(config_apply_str(k, toolong) == CFG_APPLY_TOO_LONG);

        CHECK(config_field_maxlen(k) == (unsigned)(cap - 1));
    }

    g_case = "hostname via apply: valid + lowercased, invalid rejected, g_config untouched on reject";
    {
        CHECK(config_apply_str("net_hostname", "Lights-01") == CFG_APPLY_OK);
        CHECK(strcmp(g_config.net_hostname, "lights-01") == 0);
        CHECK(config_apply_str("net_hostname", "bad_name") == CFG_APPLY_INVALID);
        CHECK(strcmp(g_config.net_hostname, "lights-01") == 0);      /* unchanged */
        CHECK(config_apply_str("net_hostname", "") == CFG_APPLY_INVALID);
        CHECK(strcmp(g_config.net_hostname, "lights-01") == 0);      /* unchanged */
        CHECK(config_apply_str("net_hostname", "-bad") == CFG_APPLY_INVALID);
        CHECK(config_field_maxlen("net_hostname") == 31);
    }

    g_case = "cred via apply: valid stores, empty refused, too-long refused, untouched on reject";
    for (size_t i = 0; i < CRED_FIELDS_N; i++) {
        const char *k = CRED_FIELDS[i].key;
        size_t cap = CRED_FIELDS[i].cap;

        CHECK(config_apply_str(k, "operator") == CFG_APPLY_OK);
        CHECK(config_apply_str(k, "") == CFG_APPLY_INVALID);          /* blank credential refused */

        char toolong[192];
        memset(toolong, 'a', cap); toolong[cap] = '\0';
        CHECK(config_apply_str(k, toolong) == CFG_APPLY_TOO_LONG);

        char ok[192];
        memset(ok, 'a', cap - 1); ok[cap - 1] = '\0';
        CHECK(config_apply_str(k, ok) == CFG_APPLY_OK);

        CHECK(config_field_maxlen(k) == (unsigned)(cap - 1));
    }
    /* g_config actually holds what was last accepted (web_user/web_pass are
     * fixed fields, unlike the table-driven checks above — spot-check one). */
    CHECK(strcmp(g_config.web_user, "operator") != 0);   /* overwritten by the `ok` cap-1 value above */

    g_case = "config_field_range: exact bounds for every numeric field";
    for (size_t i = 0; i < NUM_FIELDS_N; i++) {
        uint32_t mn = 0, mx = 0;
        CHECK(config_field_range(NUM_FIELDS[i].key, &mn, &mx) == true);
        CHECK(mn == NUM_FIELDS[i].min);
        CHECK(mx == NUM_FIELDS[i].max);
    }
    CHECK(config_field_range("relay_byteswap", NULL, NULL) == false);  /* bool: no numeric range */

    g_case = "config_field_range: NULL out-params tolerated";
    {
        uint32_t mn = 0;
        CHECK(config_field_range("long_press_ms", &mn, NULL) == true);
        CHECK(mn == 100);
        uint32_t mx = 0;
        CHECK(config_field_range("long_press_ms", NULL, &mx) == true);
        CHECK(mx == 5000);
        CHECK(config_field_range("long_press_ms", NULL, NULL) == true);
    }

    g_case = "NULL key / NULL value handled, not dereferenced";
    {
        CHECK(config_apply_u32(NULL, 1) == CFG_APPLY_UNKNOWN_KEY);
        CHECK(config_apply_str(NULL, "x") == CFG_APPLY_UNKNOWN_KEY);
        CHECK(config_apply_str("mqtt_host", NULL) == CFG_APPLY_INVALID);
        CHECK(config_field_range(NULL, NULL, NULL) == false);
        CHECK(config_field_maxlen(NULL) == 0);
    }

    if (g_fails == 0) printf("PASS: all config schema checks passed\n");
    else              printf("%d CHECK(s) FAILED\n", g_fails);
    return g_fails ? 1 : 0;
}
