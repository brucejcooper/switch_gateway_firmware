/*
 * http_client.c — cooperative non-blocking HTTP/1.1 GET client (core 1). See
 * header.
 *
 * http_parse_head() (and the small header-scan helpers it shares with the
 * chunked-encoding check) are pure — no sockets, no globals — so they're
 * exercised directly in test/http_client_test.c against canned byte buffers;
 * httpc_get_start()/httpc_poll() are a thin non-blocking wrapper around them,
 * matching the socket-state-machine cadence of ping.c/mqtt_app.c/
 * dns_resolve.c (one bounded step per tick, socket number supplied by the
 * caller, no LOG_* in the poll hot path).
 *
 * Host-test seam (matches src/boot_control.c's BOOT_CONTROL_HOST_TEST and
 * src/net/dns_resolve.c's DNS_RESOLVE_HOST_TEST): everything below the pure
 * functions touches the ioLibrary socket API and pico-sdk's pico/time.h,
 * neither of which exist on the host, so it's compiled out under
 * -DHTTP_CLIENT_HOST_TEST. The pure functions — the only thing the host test
 * exercises — are unmodified either way.
 */

#include "http_client.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#ifndef HTTP_CLIENT_HOST_TEST
#include "socket.h"
#include "wizchip_conf.h"   /* getSn_SR / getSn_IR / getSn_RX_RSR / getSn_TX_FSR / Sn_* */

#include "pico/time.h"
#include <stdio.h>
#endif

/* ── pure: header-block parsing ──────────────────────────────────────────── */

/* Case-insensitive search for header `name` within header block buf[0..hdr_len)
 * (status line + header lines, WITHOUT the final blank-line CRLF — see the
 * body_off/hdr_len derivation in http_parse_head()). Returns 1 and points
 * out_val/out_vlen at the value (leading spaces/tabs skipped) on a hit, else
 * 0. Tolerates the last header line lacking a trailing CRLF (that CRLF is the
 * first half of the terminating CRLFCRLF and so falls outside hdr_len). */
static int find_header_value(const char *buf, int hdr_len, const char *name,
                             const char **out_val, int *out_vlen) {
    /* skip the status line */
    int i = 0;
    while (i + 1 < hdr_len && !(buf[i] == '\r' && buf[i + 1] == '\n')) i++;
    if (i + 1 >= hdr_len) return 0;      /* no header lines at all */
    i += 2;

    size_t nlen = strlen(name);
    while (i < hdr_len) {
        int j = i;
        while (j + 1 < hdr_len && !(buf[j] == '\r' && buf[j + 1] == '\n')) j++;
        int line_end = (j + 1 < hdr_len) ? j : hdr_len;
        int linelen  = line_end - i;

        if ((size_t)linelen > nlen && buf[i + nlen] == ':') {
            size_t k = 0;
            for (; k < nlen; k++)
                if (tolower((unsigned char)buf[i + k]) != tolower((unsigned char)name[k])) break;
            if (k == nlen) {
                const char *v = buf + i + nlen + 1;
                int vl = linelen - (int)nlen - 1;
                while (vl > 0 && (*v == ' ' || *v == '\t')) { v++; vl--; }
                *out_val = v;
                *out_vlen = vl;
                return 1;
            }
        }
        if (j + 1 >= hdr_len) break;
        i = j + 2;
    }
    return 0;
}

int http_parse_head(const char *buf, int len, int *status, long *content_len, int *body_off) {
    if (!buf || len <= 0 || !status || !content_len || !body_off) return 0;

    /* find the CRLFCRLF that terminates the header block */
    int term = -1;
    for (int i = 0; i + 4 <= len; i++) {
        if (buf[i] == '\r' && buf[i + 1] == '\n' && buf[i + 2] == '\r' && buf[i + 3] == '\n') {
            term = i;
            break;
        }
    }
    if (term < 0) return 0;      /* header block not fully arrived yet */

    int hdr_len = term;          /* status line + headers, sans final CRLF */

    /* status line: "HTTP/<version> <3-digit-code> <reason>" */
    int line1_end = hdr_len;
    for (int i = 0; i + 1 < hdr_len; i++) {
        if (buf[i] == '\r' && buf[i + 1] == '\n') { line1_end = i; break; }
    }
    if (line1_end < 8 || strncmp(buf, "HTTP/", 5) != 0) return -1;

    int sp = -1;
    for (int i = 5; i < line1_end; i++) if (buf[i] == ' ') { sp = i; break; }
    if (sp < 0 || sp + 4 > line1_end) return -1;
    if (!isdigit((unsigned char)buf[sp + 1]) || !isdigit((unsigned char)buf[sp + 2]) ||
        !isdigit((unsigned char)buf[sp + 3]))
        return -1;

    *status = (buf[sp + 1] - '0') * 100 + (buf[sp + 2] - '0') * 10 + (buf[sp + 3] - '0');

    long cl = -1;
    const char *val; int vlen;
    if (find_header_value(buf, hdr_len, "Content-Length", &val, &vlen)) {
        char tmp[24];
        int n = vlen < (int)sizeof tmp - 1 ? vlen : (int)sizeof tmp - 1;
        memcpy(tmp, val, (size_t)n);
        tmp[n] = '\0';
        char *end = tmp;
        long v = strtol(tmp, &end, 10);
        if (end != tmp && v >= 0) cl = v;
    }
    *content_len = cl;
    *body_off = term + 4;
    return 1;
}

#ifndef HTTP_CLIENT_HOST_TEST
/* ── cooperative state machine (target only; see host-test seam above) ──── */

#define HTTPC_LOCAL_PORT       50100u
#define HTTPC_REQ_CAP          320u    /* "GET <path> HTTP/1.1\r\nHost: ...\r\n..." */
#define HTTPC_HDR_CAP          1024u   /* response header block reassembly */
#define HTTPC_CHUNK            1536u   /* bounded per-tick send/recv, ~1.5 KB */
#define HTTPC_CONNECT_TIMEOUT_MS 5000u
#define HTTPC_IO_TIMEOUT_MS      8000u

typedef enum {
    HC_IDLE, HC_CONNECTING, HC_SEND, HC_RECV_HEAD, HC_RECV_BODY, HC_DONE, HC_FAILED
} hc_phase_t;

/* Pure: does the completed header block buf[0..hdr_len) declare
 * "Transfer-Encoding: chunked"? Kept separate from http_parse_head() so that
 * function's signature/contract stays exactly as specified; hdr_len is the
 * same value http_parse_head() derives internally (body_off - 4). Lives here
 * (rather than beside find_header_value() above) because it's only used by
 * the state machine below — the host test exercises http_parse_head() only. */
static bool head_is_chunked(const char *buf, int hdr_len) {
    const char *val; int vlen;
    if (!find_header_value(buf, hdr_len, "Transfer-Encoding", &val, &vlen)) return false;
    if (vlen < 7) return false;                /* shorter than "chunked" */
    char tmp[32];
    int n = vlen < (int)sizeof tmp - 1 ? vlen : (int)sizeof tmp - 1;
    for (int i = 0; i < n; i++) tmp[i] = (char)tolower((unsigned char)val[i]);
    tmp[n] = '\0';
    return strstr(tmp, "chunked") != NULL;
}

static struct {
    bool       active;
    hc_phase_t phase;
    uint8_t    sock;
    uint8_t    ip[4];
    uint16_t   port;
    uint32_t   t_ms;              /* last activity in the current phase, for timeouts */

    char       req[HTTPC_REQ_CAP];
    uint16_t   req_len, req_sent;

    uint8_t    hdr[HTTPC_HDR_CAP];
    int        hdr_len;

    int        status_code;
    long       content_len;       /* -1 until a header with Content-Length parses */
    uint32_t   body_len;
    uint32_t   body_recv;
    uint8_t    chunk[HTTPC_CHUNK];

    httpc_body_cb on_body;
    void         *cb_ctx;
} H;

static uint32_t now_ms(void) { return to_ms_since_boot(get_absolute_time()); }

static void httpc_close_sock(void) {
    disconnect(H.sock);
    close(H.sock);
}

void httpc_get_start(const uint8_t ip[4], uint16_t port, const char *host, const char *path,
                     uint8_t sock, httpc_body_cb on_body, void *ctx) {
    if (H.active) httpc_close_sock();   /* abandon any in-flight job on this/another sock */
    memset(&H, 0, sizeof H);
    H.active      = true;
    H.sock        = sock;
    memcpy(H.ip, ip, 4);
    H.port        = port;
    H.on_body     = on_body;
    H.cb_ctx      = ctx;
    H.status_code = 0;
    H.content_len = -1;

    int n = snprintf(H.req, sizeof H.req,
        "GET %s HTTP/1.1\r\n"
        "Host: %s\r\n"
        "Connection: close\r\n"
        "User-Agent: light_switch/1\r\n\r\n",
        (path && *path) ? path : "/", host ? host : "");
    H.req_len  = (n > 0 && (size_t)n < sizeof H.req) ? (uint16_t)n : (uint16_t)(sizeof H.req - 1);
    H.req_sent = 0;

    close(H.sock);
    int8_t s = socket(H.sock, Sn_MR_TCP, HTTPC_LOCAL_PORT, SF_IO_NONBLOCK);
    if (s != (int8_t)H.sock) { H.phase = HC_FAILED; return; }
    int8_t cr = connect(H.sock, H.ip, H.port);
    if (cr != SOCK_OK && cr != SOCK_BUSY) { H.phase = HC_FAILED; return; }
    H.t_ms  = now_ms();
    H.phase = HC_CONNECTING;
}

httpc_status_t httpc_poll(void) {
    if (!H.active) return HTTPC_FAILED;
    uint32_t now = now_ms();

    switch (H.phase) {
    case HC_CONNECTING: {
        uint8_t ir = getSn_IR(H.sock);
        if (ir & Sn_IR_TIMEOUT) { setSn_IR(H.sock, Sn_IR_TIMEOUT); H.phase = HC_FAILED; break; }
        if (ir & Sn_IR_DISCON)  { setSn_IR(H.sock, Sn_IR_DISCON);  H.phase = HC_FAILED; break; }
        if (ir & Sn_IR_CON)       setSn_IR(H.sock, Sn_IR_CON);

        switch (getSn_SR(H.sock)) {
        case SOCK_ESTABLISHED: H.phase = HC_SEND; H.t_ms = now; break;
        case SOCK_CLOSE_WAIT:
        case SOCK_CLOSED:      H.phase = HC_FAILED; break;
        default:
            if (now - H.t_ms > HTTPC_CONNECT_TIMEOUT_MS) H.phase = HC_FAILED;
            break;
        }
        break;
    }

    case HC_SEND: {
        if (getSn_SR(H.sock) != SOCK_ESTABLISHED) { H.phase = HC_FAILED; break; }
        uint16_t freesz = getSn_TX_FSR(H.sock);
        uint16_t remain = H.req_len - H.req_sent;
        uint16_t want   = remain < freesz ? remain : freesz;
        if (want > HTTPC_CHUNK) want = HTTPC_CHUNK;
        if (want > 0) {
            int32_t s = send(H.sock, (uint8_t *)H.req + H.req_sent, want);
            if (s < 0 && s != SOCK_BUSY) { H.phase = HC_FAILED; break; }
            if (s > 0) { H.req_sent += (uint16_t)s; H.t_ms = now; }
        }
        if (H.req_sent >= H.req_len) { H.phase = HC_RECV_HEAD; H.t_ms = now; break; }
        if (now - H.t_ms > HTTPC_IO_TIMEOUT_MS) H.phase = HC_FAILED;
        break;
    }

    case HC_RECV_HEAD: {
        uint8_t ir = getSn_IR(H.sock);
        if (ir & Sn_IR_TIMEOUT) { setSn_IR(H.sock, Sn_IR_TIMEOUT); H.phase = HC_FAILED; break; }
        if (ir & Sn_IR_DISCON)    setSn_IR(H.sock, Sn_IR_DISCON);   /* drain first, see below */
        if (ir & Sn_IR_CON)       setSn_IR(H.sock, Sn_IR_CON);

        uint16_t rsr = getSn_RX_RSR(H.sock);
        if (rsr > 0) {
            int space = (int)sizeof H.hdr - H.hdr_len;
            if (space <= 0) { H.phase = HC_FAILED; break; }   /* header block too large */
            uint16_t want = rsr < (uint16_t)space ? rsr : (uint16_t)space;
            if (want > HTTPC_CHUNK) want = HTTPC_CHUNK;
            int32_t got = recv(H.sock, H.hdr + H.hdr_len, want);
            if (got < 0 && got != SOCK_BUSY) { H.phase = HC_FAILED; break; }
            if (got > 0) {
                H.hdr_len += (int)got;
                H.t_ms = now;
                int status, body_off; long cl;
                int pr = http_parse_head((const char *)H.hdr, H.hdr_len, &status, &cl, &body_off);
                if (pr < 0) { H.phase = HC_FAILED; break; }
                if (pr == 1) {
                    /* Static-file server: require Content-Length framing. A
                     * chunked (or framing-less) response fails cleanly here
                     * rather than being mis-parsed. */
                    if (cl < 0 || head_is_chunked((const char *)H.hdr, body_off - 4)) {
                        H.phase = HC_FAILED;
                        break;
                    }
                    H.status_code = status;
                    H.content_len = cl;
                    H.body_len    = (uint32_t)cl;
                    int leftover = H.hdr_len - body_off;
                    if (leftover > 0) {
                        if (H.on_body) H.on_body(H.hdr + body_off, (uint32_t)leftover, H.cb_ctx);
                        H.body_recv = (uint32_t)leftover;
                    }
                    H.phase = (H.body_recv >= H.body_len) ? HC_DONE : HC_RECV_BODY;
                    break;
                }
            }
        }
        {
            uint8_t sr = getSn_SR(H.sock);
            bool peer_gone = (sr == SOCK_CLOSE_WAIT || sr == SOCK_CLOSED);
            if (peer_gone && getSn_RX_RSR(H.sock) == 0) { H.phase = HC_FAILED; break; }
        }
        if (now - H.t_ms > HTTPC_IO_TIMEOUT_MS) H.phase = HC_FAILED;
        break;
    }

    case HC_RECV_BODY: {
        uint8_t ir = getSn_IR(H.sock);
        if (ir & Sn_IR_TIMEOUT) { setSn_IR(H.sock, Sn_IR_TIMEOUT); H.phase = HC_FAILED; break; }
        if (ir & Sn_IR_DISCON)    setSn_IR(H.sock, Sn_IR_DISCON);
        if (ir & Sn_IR_CON)       setSn_IR(H.sock, Sn_IR_CON);

        uint16_t rsr = getSn_RX_RSR(H.sock);
        if (rsr > 0) {
            uint32_t remain = H.body_len - H.body_recv;
            uint16_t want   = rsr;
            if (want > HTTPC_CHUNK) want = HTTPC_CHUNK;
            if ((uint32_t)want > remain) want = (uint16_t)remain;
            if (want > 0) {
                int32_t got = recv(H.sock, H.chunk, want);
                if (got < 0 && got != SOCK_BUSY) { H.phase = HC_FAILED; break; }
                if (got > 0) {
                    H.t_ms = now;
                    if (H.on_body) H.on_body(H.chunk, (uint32_t)got, H.cb_ctx);
                    H.body_recv += (uint32_t)got;
                }
            }
        }
        if (H.body_recv >= H.body_len) { H.phase = HC_DONE; break; }

        uint8_t sr = getSn_SR(H.sock);
        bool peer_gone = (sr == SOCK_CLOSE_WAIT || sr == SOCK_CLOSED);
        if (peer_gone && getSn_RX_RSR(H.sock) == 0) { H.phase = HC_FAILED; break; }  /* truncated */
        if (now - H.t_ms > HTTPC_IO_TIMEOUT_MS) H.phase = HC_FAILED;
        break;
    }

    case HC_IDLE:
    case HC_DONE:
    case HC_FAILED:
    default:
        break;
    }

    if (H.phase == HC_DONE)   { httpc_close_sock(); H.active = false; return HTTPC_DONE; }
    if (H.phase == HC_FAILED) { httpc_close_sock(); H.active = false; return HTTPC_FAILED; }
    return HTTPC_PENDING;
}

int httpc_status_code(void) { return H.status_code; }

uint32_t httpc_content_length(void) { return H.content_len < 0 ? 0u : (uint32_t)H.content_len; }

#endif /* !HTTP_CLIENT_HOST_TEST */
