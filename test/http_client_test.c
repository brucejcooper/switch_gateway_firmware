/*
 * http_client_test.c — host-side test for src/net/http_client.c's pure
 * function (http_parse_head).
 *
 * Not part of the firmware build (see CLAUDE.md: host tests live under test/
 * and are never added to the RP2040 CMake target). Compiles http_client.c
 * itself with -DHTTP_CLIENT_HOST_TEST, which compiles out the ioLibrary
 * socket state machine (httpc_get_start/httpc_poll — needs the W5100S +
 * pico-sdk, neither available on host) and leaves http_parse_head()
 * unmodified, exactly like test/dns_resolve_test.c does for
 * src/net/dns_resolve.c.
 *
 * Build & run:
 *   cc -DHTTP_CLIENT_HOST_TEST -Wall -Wextra -I ../src/net \
 *       -o /tmp/http_client_test http_client_test.c ../src/net/http_client.c \
 *       && /tmp/http_client_test
 */
#include "../src/net/http_client.h"

#include <stdio.h>
#include <string.h>

static int fail = 0;
#define CHECK(cond) do { \
        if (!(cond)) { \
            printf("FAIL: %s (%s:%d)\n", #cond, __FILE__, __LINE__); \
            fail = 1; \
        } \
    } while (0)

/* ── (1) a complete 200 with a body, delivered in one shot ──────────────── */
static void test_complete_200(void) {
    static const char resp[] = "HTTP/1.1 200 OK\r\nContent-Length: 5\r\n\r\nhello";
    int status = -999, body_off = -999;
    long content_len = -999;

    int r = http_parse_head(resp, (int)strlen(resp), &status, &content_len, &body_off);
    CHECK(r == 1);
    CHECK(status == 200);
    CHECK(content_len == 5);
    CHECK(body_off >= 0 && body_off < (int)sizeof resp);
    CHECK(resp[body_off] == 'h');
    CHECK(memcmp(resp + body_off, "hello", 5) == 0);
}

/* ── (2) a 404 with no body and no Content-Length ────────────────────────── */
static void test_404_no_body(void) {
    static const char resp[] = "HTTP/1.1 404 Not Found\r\nConnection: close\r\n\r\n";
    int status = -999, body_off = -999;
    long content_len = -999;

    int r = http_parse_head(resp, (int)strlen(resp), &status, &content_len, &body_off);
    CHECK(r == 1);
    CHECK(status == 404);
    CHECK(content_len == -1);                 /* absent */
    CHECK(body_off == (int)strlen(resp));      /* nothing after the header block */
}

/* ── (3) the same 200 response fed in two partial slices ─────────────────── */
static void test_split_across_reads(void) {
    static const char resp[] = "HTTP/1.1 200 OK\r\nContent-Length: 5\r\n\r\nhello";
    int total = (int)strlen(resp);
    int split = 10;   /* mid "HTTP/1.1 2|00 OK\r\n..." — well inside the headers */
    CHECK(split < total);

    int status = -999, body_off = -999;
    long content_len = -999;

    /* first call: only the first `split` bytes have arrived — headers not
     * complete yet (no CRLFCRLF present at all in this prefix) */
    int r1 = http_parse_head(resp, split, &status, &content_len, &body_off);
    CHECK(r1 == 0);

    /* second call: the rest of the response has now arrived too (simulating
     * accumulation into the same reassembly buffer) */
    int r2 = http_parse_head(resp, total, &status, &content_len, &body_off);
    CHECK(r2 == 1);
    CHECK(status == 200);
    CHECK(content_len == 5);
    CHECK(body_off == total - 5);
    CHECK(memcmp(resp + body_off, "hello", 5) == 0);
}

/* ── (4) a malformed status line ──────────────────────────────────────────── */
static void test_malformed_status_line(void) {
    static const char resp[] = "NOT HTTP AT ALL\r\nContent-Length: 5\r\n\r\nhello";
    int status = -999, body_off = -999;
    long content_len = -999;

    int r = http_parse_head(resp, (int)strlen(resp), &status, &content_len, &body_off);
    CHECK(r == -1);
}

/* ── bonus: a few edge cases beyond the required four ────────────────────── */
static void test_bonus_cases(void) {
    int status, body_off; long content_len;

    /* header-only case-insensitive match, mixed case, extra whitespace */
    static const char r1[] = "HTTP/1.1 200 OK\r\ncontent-LENGTH:   42  \r\n\r\n";
    CHECK(http_parse_head(r1, (int)strlen(r1), &status, &content_len, &body_off) == 1);
    CHECK(content_len == 42);
    CHECK(body_off == (int)strlen(r1));

    /* a status line with a 3-digit code but no reason phrase at all */
    static const char r2[] = "HTTP/1.1 204\r\n\r\n";
    CHECK(http_parse_head(r2, (int)strlen(r2), &status, &content_len, &body_off) == 1);
    CHECK(status == 204);
    CHECK(content_len == -1);

    /* empty buffer / zero length: not complete, not malformed */
    CHECK(http_parse_head("", 0, &status, &content_len, &body_off) == 0);

    /* a non-numeric status code is malformed */
    static const char r3[] = "HTTP/1.1 abc OK\r\n\r\n";
    CHECK(http_parse_head(r3, (int)strlen(r3), &status, &content_len, &body_off) == -1);
}

int main(void) {
    test_complete_200();
    test_404_no_body();
    test_split_across_reads();
    test_malformed_status_line();
    test_bonus_cases();

    if (fail) {
        printf("FAILED\n");
        return 1;
    }
    printf("all http_client tests passed\n");
    return 0;
}
