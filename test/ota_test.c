/*
 * ota_test.c — host-side test for src/net/ota.c's pure function
 * (ota_parse_url).
 *
 * Not part of the firmware build (see CLAUDE.md: host tests live under test/
 * and are never added to the RP2040 CMake target). Compiles ota.c itself
 * with -DOTA_HOST_TEST, which compiles out the cross-core/socket state
 * machine (ota_init/ota_trigger/ota_poll/ota_on_event — needs the W5100S +
 * pico-sdk + intercore's SPSC queues, none available on host) and leaves
 * ota_parse_url() unmodified, exactly like test/dns_resolve_test.c and
 * test/http_client_test.c do for their respective modules. ota.h pulls in
 * intercore.h (for ota_on_event()'s ic_event_t parameter), but that header
 * is itself plain structs/prototypes with no hardware dependency, so it
 * compiles cleanly on host too.
 *
 * Build & run:
 *   cc -DOTA_HOST_TEST -Wall -Wextra -I ../src -I ../src/net \
 *       -o /tmp/ota_test ota_test.c ../src/net/ota.c \
 *       && /tmp/ota_test
 */
#include "../src/net/ota.h"

#include <stdio.h>
#include <string.h>

static int fail = 0;
#define CHECK(cond) do { \
        if (!(cond)) { \
            printf("FAIL: %s (%s:%d)\n", #cond, __FILE__, __LINE__); \
            fail = 1; \
        } \
    } while (0)

/* ── (1) the canonical OTA URL: bare host, no port, real path ────────────── */
static void test_canonical(void) {
    char host[64], path[128]; uint16_t port = 0xFFFF;
    bool ok = ota_parse_url("http://ota.8bitcloud.com/light_switch.bin",
                             host, sizeof host, &port, path, sizeof path);
    CHECK(ok == true);
    CHECK(strcmp(host, "ota.8bitcloud.com") == 0);
    CHECK(port == 80);
    CHECK(strcmp(path, "/light_switch.bin") == 0);
}

/* ── (2) explicit port ────────────────────────────────────────────────────── */
static void test_explicit_port(void) {
    char host[64], path[128]; uint16_t port = 0;
    bool ok = ota_parse_url("http://192.168.1.50:8080/fw/light_switch.bin",
                             host, sizeof host, &port, path, sizeof path);
    CHECK(ok == true);
    CHECK(strcmp(host, "192.168.1.50") == 0);
    CHECK(port == 8080);
    CHECK(strcmp(path, "/fw/light_switch.bin") == 0);
}

/* ── (3) bare host, no path at all -> path defaults to "/" ────────────────── */
static void test_bare_host_no_path(void) {
    char host[64], path[128]; uint16_t port = 0;
    bool ok = ota_parse_url("http://ota.8bitcloud.com",
                             host, sizeof host, &port, path, sizeof path);
    CHECK(ok == true);
    CHECK(strcmp(host, "ota.8bitcloud.com") == 0);
    CHECK(port == 80);
    CHECK(strcmp(path, "/") == 0);
}

/* ── (4) bare host with just a trailing slash -> path is "/" too ─────────── */
static void test_bare_host_trailing_slash(void) {
    char host[64], path[128]; uint16_t port = 0;
    bool ok = ota_parse_url("http://ota.8bitcloud.com/",
                             host, sizeof host, &port, path, sizeof path);
    CHECK(ok == true);
    CHECK(strcmp(host, "ota.8bitcloud.com") == 0);
    CHECK(port == 80);
    CHECK(strcmp(path, "/") == 0);
}

/* ── (5) bare host + explicit port, no path ──────────────────────────────── */
static void test_bare_host_with_port(void) {
    char host[64], path[128]; uint16_t port = 0;
    bool ok = ota_parse_url("http://ota.8bitcloud.com:8081",
                             host, sizeof host, &port, path, sizeof path);
    CHECK(ok == true);
    CHECK(strcmp(host, "ota.8bitcloud.com") == 0);
    CHECK(port == 8081);
    CHECK(strcmp(path, "/") == 0);
}

/* ── (6) non-http schemes rejected ───────────────────────────────────────── */
static void test_non_http_scheme(void) {
    char host[64], path[128]; uint16_t port = 0;
    CHECK(ota_parse_url("https://ota.8bitcloud.com/x.bin", host, sizeof host, &port, path, sizeof path) == false);
    CHECK(ota_parse_url("ftp://ota.8bitcloud.com/x.bin", host, sizeof host, &port, path, sizeof path) == false);
    CHECK(ota_parse_url("ota.8bitcloud.com/x.bin", host, sizeof host, &port, path, sizeof path) == false);
    CHECK(ota_parse_url("", host, sizeof host, &port, path, sizeof path) == false);
}

/* ── (7) malformed hosts / ports ──────────────────────────────────────────── */
static void test_malformed(void) {
    char host[64], path[128]; uint16_t port = 0;

    /* empty host */
    CHECK(ota_parse_url("http://", host, sizeof host, &port, path, sizeof path) == false);
    CHECK(ota_parse_url("http:///path", host, sizeof host, &port, path, sizeof path) == false);
    /* empty port (bare colon) */
    CHECK(ota_parse_url("http://host:/path", host, sizeof host, &port, path, sizeof path) == false);
    /* non-numeric port */
    CHECK(ota_parse_url("http://host:abc/path", host, sizeof host, &port, path, sizeof path) == false);
    /* out-of-range port */
    CHECK(ota_parse_url("http://host:70000/path", host, sizeof host, &port, path, sizeof path) == false);
    CHECK(ota_parse_url("http://host:0/path", host, sizeof host, &port, path, sizeof path) == false);
    /* NULL args */
    CHECK(ota_parse_url(NULL, host, sizeof host, &port, path, sizeof path) == false);
    CHECK(ota_parse_url("http://host/path", NULL, sizeof host, &port, path, sizeof path) == false);
}

/* ── (8) capacity overflow rejected cleanly ──────────────────────────────── */
static void test_capacity(void) {
    char host[8], path[128]; uint16_t port = 0;
    /* "ota.8bitcloud.com" (18 chars) doesn't fit an 8-byte host buffer. */
    CHECK(ota_parse_url("http://ota.8bitcloud.com/x.bin", host, sizeof host, &port, path, sizeof path) == false);

    char host2[64], path2[8];
    /* "/light_switch.bin" (18 chars) doesn't fit an 8-byte path buffer. */
    CHECK(ota_parse_url("http://ota.8bitcloud.com/light_switch.bin", host2, sizeof host2, &port, path2, sizeof path2) == false);
}

int main(void) {
    test_canonical();
    test_explicit_port();
    test_bare_host_no_path();
    test_bare_host_trailing_slash();
    test_bare_host_with_port();
    test_non_http_scheme();
    test_malformed();
    test_capacity();

    if (fail) {
        printf("FAILED\n");
        return 1;
    }
    printf("all ota tests passed\n");
    return 0;
}
