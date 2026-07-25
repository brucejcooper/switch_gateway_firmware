/*
 * dns_resolve.c — cooperative UDP DNS A-record resolver (core 1). See header.
 *
 * dns_build_query()/dns_parse_response() are pure (no sockets, no globals) so
 * they're exercised directly in test/dns_resolve_test.c against canned byte
 * buffers; dnsr_start()/dnsr_poll() are a thin non-blocking wrapper around
 * them, matching the socket-state-machine cadence of ping.c/mqtt_app.c (one
 * bounded step per tick, socket number supplied by the caller, no LOG_* in
 * the poll hot path).
 *
 * Host-test seam (matches src/boot_control.c's BOOT_CONTROL_HOST_TEST):
 * everything below the pure functions touches the ioLibrary socket API and
 * pico-sdk's pico/time.h, neither of which exist on the host, so it's compiled
 * out under -DDNS_RESOLVE_HOST_TEST. The pure functions — the only thing the
 * host test exercises — are unmodified either way.
 */

#include "dns_resolve.h"

#include <string.h>

#ifndef DNS_RESOLVE_HOST_TEST
#include "socket.h"
#include "wizchip_conf.h"   /* Sn_MR_UDP, getSn_RX_RSR */

#include "pico/time.h"
#include <stdio.h>
#endif

#define DNS_LOCAL_PORT    50053u
#define DNS_SERVER_PORT   53u
#define DNS_PKT_CAP       320u    /* comfortably covers our query + typical replies */
#define DNS_ATTEMPT_MS    1500u   /* per-attempt timeout */
#define DNS_MAX_ATTEMPTS  2       /* one send + one retry -> ~3 s worst case */
#define DNS_MAX_HOSTNAME  128

/* ── pure: query builder ────────────────────────────────────────────────── */

int dns_build_query(uint8_t *buf, int cap, uint16_t id, const char *hostname) {
    if (!buf || !hostname || cap < 12 || hostname[0] == '\0') return -1;

    int pos = 0;
    buf[pos++] = (uint8_t)(id >> 8);
    buf[pos++] = (uint8_t)id;
    buf[pos++] = 0x01;   /* flags hi: QR=0, OPCODE=0(QUERY), AA=0, TC=0, RD=1 */
    buf[pos++] = 0x00;   /* flags lo: RA/Z/RCODE = 0 */
    buf[pos++] = 0x00; buf[pos++] = 0x01;   /* QDCOUNT = 1 */
    buf[pos++] = 0x00; buf[pos++] = 0x00;   /* ANCOUNT = 0 */
    buf[pos++] = 0x00; buf[pos++] = 0x00;   /* NSCOUNT = 0 */
    buf[pos++] = 0x00; buf[pos++] = 0x00;   /* ARCOUNT = 0 */

    /* QNAME: one length-prefixed label per dot-separated segment, root label
     * (0x00) to terminate. Rejects empty segments ("a..b", leading/trailing
     * dot) and segments over the DNS 63-byte label limit. */
    const char *p = hostname;
    while (*p) {
        const char *dot = strchr(p, '.');
        size_t seglen = dot ? (size_t)(dot - p) : strlen(p);
        if (seglen == 0 || seglen > 63) return -1;
        if (pos + 1 + (int)seglen > cap) return -1;
        buf[pos++] = (uint8_t)seglen;
        memcpy(buf + pos, p, seglen);
        pos += (int)seglen;
        p += seglen;
        if (*p == '.') {
            p++;
            if (*p == '\0') return -1;   /* trailing dot -> empty final label */
        } else {
            break;
        }
    }
    if (pos + 1 > cap) return -1;
    buf[pos++] = 0x00;   /* root label */

    if (pos + 4 > cap) return -1;
    buf[pos++] = 0x00; buf[pos++] = 0x01;   /* QTYPE  = A */
    buf[pos++] = 0x00; buf[pos++] = 0x01;   /* QCLASS = IN */

    return pos;
}

/* ── pure: response parser ──────────────────────────────────────────────── */

/* Walk a (possibly compressed) name starting at `off` and return the offset
 * just past it in the *original* stream (i.e. past the terminating zero
 * label, or past the 2-byte pointer that redirected elsewhere) — the caller
 * resumes parsing there. Returns -1 on any malformed/out-of-range/looping
 * name. Loop-bounded (RFC 1035 4.1.4 compression pointers must only point
 * backward, but we don't trust that — cap the walk instead). */
static int skip_name(const uint8_t *buf, int len, int off) {
    int end = -1;   /* position in the original stream once we've moved on */
    for (int guard = 0; guard < 128; guard++) {
        if (off < 0 || off >= len) return -1;
        uint8_t b = buf[off];
        if ((b & 0xC0) == 0xC0) {                 /* compression pointer */
            if (off + 1 >= len) return -1;
            if (end < 0) end = off + 2;
            int ptr = (int)((b & 0x3F) << 8) | buf[off + 1];
            if (ptr >= len) return -1;
            off = ptr;
            continue;
        }
        if (b & 0xC0) return -1;                  /* 0x40/0x80: reserved, invalid */
        if (b == 0) {                              /* root label: name ends here */
            if (end < 0) end = off + 1;
            return end;
        }
        off += 1 + b;
        if (off > len) return -1;
    }
    return -1;   /* pointer chain too long -> treat as malformed */
}

bool dns_parse_response(const uint8_t *buf, int len, uint16_t expect_id, uint8_t out_ip[4]) {
    if (!buf || !out_ip || len < 12) return false;

    uint16_t id = ((uint16_t)buf[0] << 8) | buf[1];
    if (id != expect_id) return false;

    uint8_t flags_hi = buf[2];
    uint8_t flags_lo = buf[3];
    if ((flags_hi & 0x80) == 0) return false;      /* QR bit: must be a response */
    if ((flags_lo & 0x0F) != 0) return false;      /* RCODE must be NOERROR */

    uint16_t qdcount = ((uint16_t)buf[4] << 8) | buf[5];
    uint16_t ancount = ((uint16_t)buf[6] << 8) | buf[7];
    if (ancount == 0) return false;

    int off = 12;
    for (uint16_t q = 0; q < qdcount; q++) {
        off = skip_name(buf, len, off);
        if (off < 0 || off + 4 > len) return false;
        off += 4;   /* QTYPE + QCLASS */
    }

    for (uint16_t a = 0; a < ancount; a++) {
        off = skip_name(buf, len, off);
        if (off < 0 || off + 10 > len) return false;
        uint16_t type  = ((uint16_t)buf[off]     << 8) | buf[off + 1];
        uint16_t class = ((uint16_t)buf[off + 2] << 8) | buf[off + 3];
        /* buf[off+4 .. off+7] = TTL, unused */
        uint16_t rdlen = ((uint16_t)buf[off + 8] << 8) | buf[off + 9];
        off += 10;
        if (rdlen > (uint16_t)(len - off)) return false;
        if (type == 1 && class == 1 && rdlen == 4) {   /* A / IN */
            memcpy(out_ip, buf + off, 4);
            return true;
        }
        off += rdlen;
    }
    return false;   /* no A record among the answers */
}

#ifndef DNS_RESOLVE_HOST_TEST
/* ── cooperative state machine (target only; see host-test seam above) ──── */

typedef enum { DNS_IDLE, DNS_OPEN, DNS_SEND, DNS_WAIT } dns_phase_t;

static struct {
    bool        active;
    dns_phase_t phase;
    uint8_t     sock;
    uint8_t     server_ip[4];
    char        hostname[DNS_MAX_HOSTNAME];
    uint16_t    id;
    uint8_t     attempt;
    uint32_t    t_ms;
} D;

static uint32_t now_ms(void) { return to_ms_since_boot(get_absolute_time()); }

static void dnsr_close(void) {
    close(D.sock);
    D.active = false;
    D.phase  = DNS_IDLE;
}

void dnsr_start(const uint8_t dns_server[4], const char *hostname, uint8_t sock) {
    if (D.active) close(D.sock);   /* abandon any in-flight job on this/another sock */
    memset(&D, 0, sizeof D);
    D.active = true;
    D.phase  = DNS_OPEN;
    memcpy(D.server_ip, dns_server, 4);
    snprintf(D.hostname, sizeof D.hostname, "%s", hostname ? hostname : "");
    D.sock    = sock;
    D.attempt = 0;
    /* Transaction id: not security-sensitive here (LAN-side resolver behind
     * the same UDP round-trip as DHCP), just needs to disambiguate this job
     * from a stale/foreign reply. Boot-time clock + a per-call salt is enough. */
    D.id = (uint16_t)(now_ms() ^ ((uintptr_t)hostname & 0xFFFFu) ^ 0xA5A5u);
}

static bool dnsr_send(void) {
    uint8_t pkt[DNS_PKT_CAP];
    int n = dns_build_query(pkt, sizeof pkt, D.id, D.hostname);
    if (n < 0) return false;
    int32_t r = sendto(D.sock, pkt, (uint16_t)n, D.server_ip, DNS_SERVER_PORT);
    if (r < 0) return false;
    D.attempt++;
    D.t_ms  = now_ms();
    D.phase = DNS_WAIT;
    return true;
}

dnsr_status_t dnsr_poll(uint8_t out_ip[4]) {
    if (!D.active) return DNSR_FAILED;
    uint32_t now = now_ms();

    switch (D.phase) {
    case DNS_OPEN: {
        close(D.sock);
        int8_t s = socket(D.sock, Sn_MR_UDP, DNS_LOCAL_PORT, SF_IO_NONBLOCK);
        if (s != D.sock) { dnsr_close(); return DNSR_FAILED; }
        D.phase = DNS_SEND;
        return DNSR_PENDING;
    }

    case DNS_SEND:
        if (!dnsr_send()) { dnsr_close(); return DNSR_FAILED; }
        return DNSR_PENDING;

    case DNS_WAIT: {
        /* Drain a few queued datagrams per tick looking for our reply; a
         * stray/foreign packet must not stall the resolve until its timeout. */
        for (int i = 0; i < 4 && getSn_RX_RSR(D.sock) > 0; i++) {
            uint8_t  rbuf[DNS_PKT_CAP];
            uint8_t  raddr[4];
            uint16_t rport = 0;
            int32_t  got = recvfrom(D.sock, rbuf, sizeof rbuf, raddr, &rport);
            if (got <= 0) break;
            if (memcmp(raddr, D.server_ip, 4) != 0) continue;   /* not our server */
            uint8_t ip[4];
            if (dns_parse_response(rbuf, (int)got, D.id, ip)) {
                memcpy(out_ip, ip, 4);
                dnsr_close();
                return DNSR_OK;
            }
            /* malformed / stale-id reply from the right server: keep waiting */
        }
        if (now - D.t_ms >= DNS_ATTEMPT_MS) {
            if (D.attempt < DNS_MAX_ATTEMPTS) { D.phase = DNS_SEND; return DNSR_PENDING; }
            dnsr_close();
            return DNSR_FAILED;
        }
        return DNSR_PENDING;
    }

    default:
        dnsr_close();
        return DNSR_FAILED;
    }
}
#endif /* !DNS_RESOLVE_HOST_TEST */
