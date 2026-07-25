/*
 * dns_resolve_test.c — host-side test for src/net/dns_resolve.c's pure
 * functions (dns_build_query / dns_parse_response).
 *
 * Not part of the firmware build (see CLAUDE.md: host tests live under test/
 * and are never added to the RP2040 CMake target). Compiles dns_resolve.c
 * itself with -DDNS_RESOLVE_HOST_TEST, which compiles out the ioLibrary
 * socket state machine (dnsr_start/dnsr_poll — needs the W5100S + pico-sdk,
 * neither available on host) and leaves the two pure functions this test
 * exercises unmodified, exactly like test/boot_control_test.c does for
 * src/boot_control.c.
 *
 * Build & run:
 *   cc -DDNS_RESOLVE_HOST_TEST -Wall -Wextra -I ../src/net \
 *       -o /tmp/dns_resolve_test dns_resolve_test.c ../src/net/dns_resolve.c \
 *       && /tmp/dns_resolve_test
 */
#include "../src/net/dns_resolve.h"

#include <stdio.h>
#include <string.h>

static int fail = 0;
#define CHECK(cond) do { \
        if (!(cond)) { \
            printf("FAIL: %s (%s:%d)\n", #cond, __FILE__, __LINE__); \
            fail = 1; \
        } \
    } while (0)

/* ── (1) dns_build_query: header fields + exact QNAME label encoding ────── */
static void test_build_query(void) {
    uint8_t buf[300];
    uint16_t id = 0x1234;
    int n = dns_build_query(buf, sizeof buf, id, "ota.8bitcloud.com");
    CHECK(n > 0);

    /* Header: ID, flags (RD set, everything else 0), QDCOUNT=1, AN/NS/AR=0. */
    CHECK(buf[0] == 0x12 && buf[1] == 0x34);           /* ID */
    CHECK(buf[2] == 0x01);                              /* RD=1, QR/OPCODE/AA/TC=0 */
    CHECK(buf[3] == 0x00);
    CHECK(buf[4] == 0x00 && buf[5] == 0x01);            /* QDCOUNT = 1 */
    CHECK(buf[6] == 0x00 && buf[7] == 0x00);            /* ANCOUNT = 0 */
    CHECK(buf[8] == 0x00 && buf[9] == 0x00);            /* NSCOUNT = 0 */
    CHECK(buf[10] == 0x00 && buf[11] == 0x00);          /* ARCOUNT = 0 */

    /* QNAME: \x03ota\x08bitcloud... wait -- "8bitcloud" is one 9-byte label. */
    static const uint8_t expect_qname[] = {
        0x03, 'o', 't', 'a',
        0x09, '8', 'b', 'i', 't', 'c', 'l', 'o', 'u', 'd',
        0x03, 'c', 'o', 'm',
        0x00
    };
    CHECK((size_t)(n - 12 - 4) == sizeof expect_qname);   /* QNAME length */
    CHECK(memcmp(buf + 12, expect_qname, sizeof expect_qname) == 0);

    /* QTYPE=A(1), QCLASS=IN(1) immediately after QNAME. */
    int qt_off = 12 + (int)sizeof expect_qname;
    CHECK(buf[qt_off]     == 0x00 && buf[qt_off + 1] == 0x01);   /* QTYPE  = A */
    CHECK(buf[qt_off + 2] == 0x00 && buf[qt_off + 3] == 0x01);   /* QCLASS = IN */
    CHECK(qt_off + 4 == n);

    /* Overflow: a cap too small for the encoded query must fail cleanly. */
    CHECK(dns_build_query(buf, 12, id, "ota.8bitcloud.com") == -1);
    CHECK(dns_build_query(buf, n - 1, id, "ota.8bitcloud.com") == -1);

    /* Malformed hostnames rejected. */
    CHECK(dns_build_query(buf, sizeof buf, id, "") == -1);
    CHECK(dns_build_query(buf, sizeof buf, id, "a..b") == -1);
    CHECK(dns_build_query(buf, sizeof buf, id, "trailing.") == -1);
}

/* ── helpers to build canned response buffers ────────────────────────────── */

/* Appends a label-encoded name (dot-separated, no trailing dot) to buf at
 * *pos, advancing *pos. Used to build the (uncompressed) question name. */
static void put_name(uint8_t *buf, int *pos, const char *name) {
    const char *p = name;
    while (*p) {
        const char *dot = strchr(p, '.');
        size_t seglen = dot ? (size_t)(dot - p) : strlen(p);
        buf[(*pos)++] = (uint8_t)seglen;
        memcpy(buf + *pos, p, seglen);
        *pos += (int)seglen;
        p += seglen;
        if (*p == '.') p++; else break;
    }
    buf[(*pos)++] = 0x00;
}

static void put_u16(uint8_t *buf, int *pos, uint16_t v) {
    buf[(*pos)++] = (uint8_t)(v >> 8);
    buf[(*pos)++] = (uint8_t)v;
}
static void put_u32(uint8_t *buf, int *pos, uint32_t v) {
    buf[(*pos)++] = (uint8_t)(v >> 24);
    buf[(*pos)++] = (uint8_t)(v >> 16);
    buf[(*pos)++] = (uint8_t)(v >> 8);
    buf[(*pos)++] = (uint8_t)v;
}

/* ── (2) dns_parse_response: valid answer via a compression pointer ─────── */
static void test_parse_valid_with_compression(void) {
    uint8_t buf[128];
    int pos = 0;
    uint16_t id = 0xBEEF;

    put_u16(buf, &pos, id);
    buf[pos++] = 0x81; buf[pos++] = 0x80;      /* QR=1, RD=1, RA=1, RCODE=0 */
    put_u16(buf, &pos, 1);                      /* QDCOUNT */
    put_u16(buf, &pos, 1);                      /* ANCOUNT */
    put_u16(buf, &pos, 0);                      /* NSCOUNT */
    put_u16(buf, &pos, 0);                      /* ARCOUNT */

    int qname_off = pos;
    put_name(buf, &pos, "ota.8bitcloud.com");
    put_u16(buf, &pos, 1);                      /* QTYPE  = A */
    put_u16(buf, &pos, 1);                      /* QCLASS = IN */

    /* Answer: NAME is a compression pointer back at the question's QNAME
     * (0xC0 | high 6 bits of offset, low 8 bits of offset) -- this is the
     * mandatory case per the task: a real answer MUST exercise the pointer. */
    CHECK(qname_off < 0x4000);   /* fits in a 14-bit pointer */
    buf[pos++] = (uint8_t)(0xC0 | (qname_off >> 8));
    buf[pos++] = (uint8_t)(qname_off & 0xFF);
    put_u16(buf, &pos, 1);                      /* TYPE  = A */
    put_u16(buf, &pos, 1);                      /* CLASS = IN */
    put_u32(buf, &pos, 300);                    /* TTL */
    put_u16(buf, &pos, 4);                      /* RDLENGTH */
    uint8_t expect_ip[4] = { 203, 0, 113, 42 };
    memcpy(buf + pos, expect_ip, 4); pos += 4;

    uint8_t got_ip[4] = {0};
    bool ok = dns_parse_response(buf, pos, id, got_ip);
    CHECK(ok == true);
    CHECK(memcmp(got_ip, expect_ip, 4) == 0);
}

/* ── (3) rejections: wrong id, truncated, non-A-only answers ─────────────── */
static void test_rejections(void) {
    uint8_t buf[128];
    int pos = 0;
    uint16_t id = 0x4242;

    put_u16(buf, &pos, id);
    buf[pos++] = 0x81; buf[pos++] = 0x80;
    put_u16(buf, &pos, 1);
    put_u16(buf, &pos, 1);
    put_u16(buf, &pos, 0);
    put_u16(buf, &pos, 0);
    int qname_off = pos;
    put_name(buf, &pos, "ota.8bitcloud.com");
    put_u16(buf, &pos, 1);
    put_u16(buf, &pos, 1);
    buf[pos++] = (uint8_t)(0xC0 | (qname_off >> 8));
    buf[pos++] = (uint8_t)(qname_off & 0xFF);
    put_u16(buf, &pos, 1);
    put_u16(buf, &pos, 1);
    put_u32(buf, &pos, 300);
    put_u16(buf, &pos, 4);
    uint8_t ip[4] = { 10, 0, 0, 1 };
    memcpy(buf + pos, ip, 4); pos += 4;
    int full_len = pos;

    uint8_t out[4];

    /* (3a) wrong id */
    CHECK(dns_parse_response(buf, full_len, (uint16_t)(id ^ 0xFFFF), out) == false);

    /* (3b) truncated packet (cut mid-answer, and cut to less than a header) */
    CHECK(dns_parse_response(buf, full_len - 2, id, out) == false);
    CHECK(dns_parse_response(buf, 6, id, out) == false);

    /* (3c) a response with only non-A records (AAAA + CNAME, no A) must be
     * rejected even though it otherwise parses cleanly. */
    {
        uint8_t b2[160];
        int p2 = 0;
        put_u16(b2, &p2, id);
        b2[p2++] = 0x81; b2[p2++] = 0x80;
        put_u16(b2, &p2, 1);
        put_u16(b2, &p2, 2);          /* ANCOUNT = 2 (CNAME + AAAA) */
        put_u16(b2, &p2, 0);
        put_u16(b2, &p2, 0);
        int qn = p2;
        put_name(b2, &p2, "ota.8bitcloud.com");
        put_u16(b2, &p2, 1);
        put_u16(b2, &p2, 1);

        /* Answer 1: CNAME (type 5), points at the question name, arbitrary
         * RDATA (a name) -- not an A record. */
        b2[p2++] = (uint8_t)(0xC0 | (qn >> 8));
        b2[p2++] = (uint8_t)(qn & 0xFF);
        put_u16(b2, &p2, 5);           /* TYPE = CNAME */
        put_u16(b2, &p2, 1);           /* CLASS = IN */
        put_u32(b2, &p2, 300);
        int rdlen_off = p2;
        put_u16(b2, &p2, 0);           /* placeholder RDLENGTH */
        int rdata_start = p2;
        put_name(b2, &p2, "edge.8bitcloud.com");
        uint16_t rdlen = (uint16_t)(p2 - rdata_start);
        b2[rdlen_off] = (uint8_t)(rdlen >> 8); b2[rdlen_off + 1] = (uint8_t)rdlen;

        /* Answer 2: AAAA (type 28), 16-byte RDATA -- not an A record either. */
        b2[p2++] = (uint8_t)(0xC0 | (qn >> 8));
        b2[p2++] = (uint8_t)(qn & 0xFF);
        put_u16(b2, &p2, 28);          /* TYPE = AAAA */
        put_u16(b2, &p2, 1);
        put_u32(b2, &p2, 300);
        put_u16(b2, &p2, 16);
        memset(b2 + p2, 0xAB, 16); p2 += 16;

        CHECK(dns_parse_response(b2, p2, id, out) == false);
    }

    /* (3d) not a response (QR=0) rejected even with a well-formed answer section */
    {
        uint8_t b3[64];
        memcpy(b3, buf, (size_t)full_len);
        b3[2] = 0x01;   /* QR=0, RD=1 */
        CHECK(dns_parse_response(b3, full_len, id, out) == false);
    }

    /* (3e) non-zero RCODE (e.g. NXDOMAIN=3) rejected */
    {
        uint8_t b4[64];
        memcpy(b4, buf, (size_t)full_len);
        b4[3] = 0x03;
        CHECK(dns_parse_response(b4, full_len, id, out) == false);
    }
}

int main(void) {
    test_build_query();
    test_parse_valid_with_compression();
    test_rejections();

    if (fail) {
        printf("FAILED\n");
        return 1;
    }
    printf("all dns_resolve tests passed\n");
    return 0;
}
