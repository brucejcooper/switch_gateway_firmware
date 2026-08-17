/*
 * config_hostname_test.c — host-side test for config_hostname_ok() and
 * config_hostname_set() (src/config.c), the single DHCP/DNS hostname rule
 * shared by the CLI (`net set hostname=`), the web/API handler
 * (/api/net, http_server.c) and config_load() itself.
 *
 * The rule: one RFC-1123 label — 1..31 chars, letters/digits/hyphen only, not
 * leading or trailing a hyphen. It is a WHITELIST, so the interesting cases
 * are the characters that look plausible in a hostname and are not permitted
 * (dot, underscore, space) plus the ones that should never reach the wire at
 * all (control and high-bit bytes). config_hostname_set() additionally
 * lowercases, so what is stored, what goes out as DHCP option 12, and what the
 * router shows in local DNS are all the same string.
 *
 * Build & run:
 *   cc -DJSMN_STATIC -Wall -Wextra -I ../src -I ../lib/jsmn \
 *       -o /tmp/config_hostname_test config_hostname_test.c ../src/config.c \
 *       && /tmp/config_hostname_test
 */
#include "../src/config.h"
#include "../src/log.h"

#include <stdio.h>
#include <string.h>

/* config.c logs through these; the parse/validate paths under test never
 * touch the filesystem, so plain stubs are enough to link. */
log_level_t g_log_level = LOG_LEVEL_OFF;
void log_emit(log_level_t lvl, const char *lc, const char *mod, const char *fmt, ...) {
    (void)lvl; (void)lc; (void)mod; (void)fmt;
}

static int g_fails = 0;
static const char *g_case = "";
#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL [%s]: %s (%s:%d)\n", g_case, #cond, __FILE__, __LINE__); g_fails++; } \
} while (0)

/* Valid input => accepted, and stored lowercased. */
static void ok_as(const char *in, const char *want) {
    char dst[32];
    memset(dst, '@', sizeof dst);
    CHECK(config_hostname_ok(in));
    CHECK(config_hostname_set(dst, sizeof dst, in));
    CHECK(strcmp(dst, want) == 0);
}

/* Invalid input => rejected, and dst left exactly as it was. A caller that
 * ignores the return value must not end up with a half-written hostname. */
static void rejected(const char *in) {
    char dst[32];
    memcpy(dst, "lights", 7);
    CHECK(!config_hostname_ok(in));
    CHECK(!config_hostname_set(dst, sizeof dst, in));
    CHECK(strcmp(dst, "lights") == 0);
}

int main(void) {
    g_case = "accepted, lowercased";
    ok_as("lights", "lights");
    ok_as("a", "a");                          /* 1 char is a legal label */
    ok_as("a1", "a1");
    ok_as("light-switch", "light-switch");
    ok_as("0", "0");                          /* labels may start with a digit */
    ok_as("a-1-b", "a-1-b");
    /* Uppercase is legal in a label (case-insensitive, RFC 4343) but is
     * normalised on the way in — this is the "lowercase on accept" rule. */
    ok_as("Lights", "lights");
    ok_as("LIGHTS", "lights");
    ok_as("LiGhTs-01", "lights-01");

    g_case = "length bounds";
    {
        char max31[32], over32[33];
        memset(max31, 'a', 31); max31[31] = '\0';
        memset(over32, 'a', 32); over32[32] = '\0';
        ok_as(max31, max31);                  /* 31 fits net_hostname[32] */
        rejected(over32);                     /* 32 does not */
        rejected("");                         /* empty is not a label */
    }

    g_case = "hyphen placement";
    rejected("-lights");
    rejected("lights-");
    rejected("-");
    ok_as("a-b", "a-b");                      /* interior hyphens are fine */

    g_case = "characters that look plausible but are not permitted";
    rejected("lights.local");                 /* a dot makes it 2 labels */
    rejected("my_host");                      /* underscore: not RFC 1123 */
    rejected("light switch");                 /* space */
    rejected("lights!");
    rejected("lights/1");
    rejected("lights:80");
    rejected("light+s");
    rejected("light*s");
    rejected("\"lights\"");

    g_case = "bytes that must never reach the wire";
    rejected("light\tswitch");                /* control character */
    rejected("light\nswitch");                /* embedded newline */
    rejected("lights\x01");
    rejected("light\xc3\xa9s");               /* UTF-8 / high-bit bytes */
    rejected("light\x7f");

    g_case = "NULL and zero cap are handled, not dereferenced";
    {
        char dst[32] = "lights";
        CHECK(!config_hostname_ok(NULL));
        CHECK(!config_hostname_set(NULL, sizeof dst, "ok"));
        CHECK(!config_hostname_set(dst, 0, "ok"));
        CHECK(strcmp(dst, "lights") == 0);
    }

    /* http_server.c normalises /api/net's command buffer in place, so dst and
     * src alias. The copy runs strictly forward and never re-reads an index it
     * has written, which is what makes that safe. */
    g_case = "in-place normalisation (dst == src)";
    {
        char buf[32];
        snprintf(buf, sizeof buf, "%s", "LiGhTs-01");
        CHECK(config_hostname_set(buf, sizeof buf, buf));
        CHECK(strcmp(buf, "lights-01") == 0);

        snprintf(buf, sizeof buf, "%s", "bad_name");
        CHECK(!config_hostname_set(buf, sizeof buf, buf));
        CHECK(strcmp(buf, "bad_name") == 0);   /* untouched on reject */
    }

    /* A destination too small for an otherwise-valid label is REFUSED, not
     * truncated: storing a shorter, different hostname than the one that was
     * accepted would be a silent substitution. Unreachable from this firmware
     * (every caller passes 32 bytes and the rule caps at 31) — pinned so it
     * stays that way if a caller ever passes something smaller. */
    g_case = "destination too small is refused, not truncated";
    {
        char small[5] = "keep";
        CHECK(!config_hostname_set(small, sizeof small, "LIGHTSWITCH"));
        CHECK(strcmp(small, "keep") == 0);
        /* Exactly-fits and one-too-long, either side of the boundary. */
        char four[5] = "keep";
        CHECK(config_hostname_set(four, sizeof four, "ABCD"));
        CHECK(strcmp(four, "abcd") == 0);
        char four2[5] = "keep";
        CHECK(!config_hostname_set(four2, sizeof four2, "ABCDE"));
        CHECK(strcmp(four2, "keep") == 0);
    }

    if (g_fails == 0) printf("PASS: all config hostname checks passed\n");
    else              printf("%d CHECK(s) FAILED\n", g_fails);
    return g_fails ? 1 : 0;
}
