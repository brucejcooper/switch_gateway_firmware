/*
 * fwupdate_test.c — host-side test for src/net/fwupdate.c's pure function
 * (fw_json_get).
 *
 * Not part of the firmware build (see CLAUDE.md: host tests live under test/
 * and are never added to the RP2040 CMake target). Compiles fwupdate.c itself
 * with -DFWUPDATE_HOST_TEST, which compiles out the cooperative manifest-
 * check state machine (fwupdate_init/poll/announce/install/accessors — needs
 * the W5100S + pico-sdk + ota.h's ota_trigger()/ota_state(), none available
 * on host) and leaves fw_json_get() unmodified, exactly like
 * test/ota_test.c does for ota_parse_url().
 *
 * Build & run:
 *   cc -DFWUPDATE_HOST_TEST -Wall -Wextra -I ../src -I ../src/net \
 *       -o /tmp/fwupdate_test fwupdate_test.c ../src/net/fwupdate.c \
 *       && /tmp/fwupdate_test
 */
#include "../src/net/fwupdate.h"

#include <stdio.h>
#include <string.h>

static int fail = 0;
#define CHECK(cond) do { \
        if (!(cond)) { \
            printf("FAIL: %s (%s:%d)\n", #cond, __FILE__, __LINE__); \
            fail = 1; \
        } \
    } while (0)

/* A realistic manifest, keys in the "natural" order. */
static const char MANIFEST[] =
    "{\"version\":\"2.1.0\","
    "\"url\":\"http://ota.8bitcloud.com/light_switch.bin\","
    "\"title\":\"Light Switch 2.1.0\","
    "\"release_url\":\"https://git.8bitcloud.com/bruce/light_switch/releases/2.1.0\","
    "\"notes\":\"Fixes DALI group membership on boot\","
    "\"board\":\"w5100s_evb_pico\",\"size\":123456}";

/* ── (1) extract each documented string field ────────────────────────────── */
static void test_extract_all_fields(void) {
    char out[256];

    CHECK(fw_json_get(MANIFEST, "version", out, sizeof out));
    CHECK(strcmp(out, "2.1.0") == 0);

    CHECK(fw_json_get(MANIFEST, "url", out, sizeof out));
    CHECK(strcmp(out, "http://ota.8bitcloud.com/light_switch.bin") == 0);

    CHECK(fw_json_get(MANIFEST, "title", out, sizeof out));
    CHECK(strcmp(out, "Light Switch 2.1.0") == 0);

    CHECK(fw_json_get(MANIFEST, "release_url", out, sizeof out));
    CHECK(strcmp(out, "https://git.8bitcloud.com/bruce/light_switch/releases/2.1.0") == 0);

    CHECK(fw_json_get(MANIFEST, "notes", out, sizeof out));
    CHECK(strcmp(out, "Fixes DALI group membership on boot") == 0);
}

/* ── (2) a key absent from the buffer entirely -> false ──────────────────── */
static void test_missing_key(void) {
    char out[64];
    static const char BUF[] = "{\"version\":\"2.1.0\",\"url\":\"http://x/y.bin\"}";
    CHECK(fw_json_get(BUF, "title", out, sizeof out) == false);
    CHECK(fw_json_get(BUF, "release_url", out, sizeof out) == false);
    CHECK(fw_json_get(BUF, "notes", out, sizeof out) == false);
}

/* ── (3) an empty string value -> false ──────────────────────────────────── */
static void test_empty_value(void) {
    char out[64];
    static const char BUF[] = "{\"version\":\"2.1.0\",\"notes\":\"\",\"url\":\"http://x/y.bin\"}";
    CHECK(fw_json_get(BUF, "notes", out, sizeof out) == false);
    /* neighbours on either side of the empty value still parse fine */
    CHECK(fw_json_get(BUF, "version", out, sizeof out) && !strcmp(out, "2.1.0"));
    CHECK(fw_json_get(BUF, "url", out, sizeof out) && !strcmp(out, "http://x/y.bin"));
}

/* ── (4) keys in any order ────────────────────────────────────────────────── */
static void test_key_order_independence(void) {
    char out[128];
    static const char BUF[] =
        "{\"notes\":\"reordered\",\"url\":\"http://x/y.bin\","
        "\"title\":\"T\",\"version\":\"9.9.9\",\"release_url\":\"http://r\"}";
    CHECK(fw_json_get(BUF, "version", out, sizeof out) && !strcmp(out, "9.9.9"));
    CHECK(fw_json_get(BUF, "url", out, sizeof out) && !strcmp(out, "http://x/y.bin"));
    CHECK(fw_json_get(BUF, "title", out, sizeof out) && !strcmp(out, "T"));
    CHECK(fw_json_get(BUF, "release_url", out, sizeof out) && !strcmp(out, "http://r"));
    CHECK(fw_json_get(BUF, "notes", out, sizeof out) && !strcmp(out, "reordered"));
}

/* ── (5) a value substring must never be mistaken for a key ──────────────── */
static void test_no_value_substring_confusion(void) {
    char out[128];

    /* (5a) the *word* "version" appears inside another key's value (a url
     * path segment) — must not be picked up as the "version" key. The real
     * version key comes later in the buffer. */
    static const char BUF_A[] =
        "{\"url\":\"http://ota.8bitcloud.com/version/light_switch.bin\","
        "\"version\":\"3.0.0\"}";
    CHECK(fw_json_get(BUF_A, "version", out, sizeof out));
    CHECK(strcmp(out, "3.0.0") == 0);

    /* (5b) "url" as a suffix of another key's *name* ("release_url") must not
     * satisfy a lookup for the bare "url" key. */
    static const char BUF_B[] =
        "{\"release_url\":\"https://example/1\",\"title\":\"T\"}";
    CHECK(fw_json_get(BUF_B, "url", out, sizeof out) == false);

    /* (5c) the exact text 'url' appears, unquoted-and-uncoloned, inside a
     * notes value — must not satisfy a lookup for "url" either. The real
     * "url" key follows. */
    static const char BUF_C[] =
        "{\"notes\":\"see the release url field\",\"url\":\"http://real\"}";
    CHECK(fw_json_get(BUF_C, "url", out, sizeof out));
    CHECK(strcmp(out, "http://real") == 0);
}

/* ── (6) output-capacity and malformed-input edge cases ──────────────────── */
static void test_edge_cases(void) {
    char out[4];   /* too small for "2.1.0" (5 chars + NUL) */
    CHECK(fw_json_get(MANIFEST, "version", out, sizeof out) == false);

    CHECK(fw_json_get(NULL, "version", out, sizeof out) == false);
    CHECK(fw_json_get(MANIFEST, NULL, out, sizeof out) == false);
    CHECK(fw_json_get(MANIFEST, "version", NULL, sizeof out) == false);
    CHECK(fw_json_get(MANIFEST, "version", out, 0) == false);

    /* key present but value isn't a quoted string (a bare number) -> false */
    char big[64];
    CHECK(fw_json_get(MANIFEST, "size", big, sizeof big) == false);
}

int main(void) {
    test_extract_all_fields();
    test_missing_key();
    test_empty_value();
    test_key_order_independence();
    test_no_value_substring_confusion();
    test_edge_cases();

    if (fail) {
        printf("FAILED\n");
        return 1;
    }
    printf("all fwupdate tests passed\n");
    return 0;
}
