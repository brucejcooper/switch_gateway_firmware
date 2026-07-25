/*
 * ota.c — core-1 pull-based OTA orchestrator. See ota.h for the API.
 *
 * Host-test seam (matches dns_resolve.c/http_client.c's pattern): the state
 * machine below needs the W5100S socket API (via netif.h/dns_resolve.h/
 * http_client.h) and intercore.h's queues, none of which exist on the host,
 * so it is compiled out under -DOTA_HOST_TEST. ota_parse_url() is pure (no
 * sockets, no globals) and unaffected either way — it's the only thing
 * test/ota_test.c exercises.
 */

#include "ota.h"

#include <string.h>
#include <stdlib.h>

/* ── pure: URL parser (host-testable; unconditional) ─────────────────────── */

bool ota_parse_url(const char *url, char *host, int host_cap, uint16_t *port, char *path, int path_cap) {
    if (!url || !host || host_cap <= 0 || !port || !path || path_cap <= 0) return false;

    static const char PFX[] = "http://";
    const int pfx_len = (int)(sizeof(PFX) - 1);
    if (strncmp(url, PFX, (size_t)pfx_len) != 0) return false;
    const char *p = url + pfx_len;
    if (*p == '\0' || *p == '/' || *p == ':') return false;   /* empty host */

    const char *slash = strchr(p, '/');
    const char *hp_end = slash ? slash : p + strlen(p);       /* end of host[:port] */
    const char *colon = memchr(p, ':', (size_t)(hp_end - p));
    const char *host_end = colon ? colon : hp_end;

    size_t hlen = (size_t)(host_end - p);
    if (hlen == 0 || hlen >= (size_t)host_cap) return false;

    uint16_t pt = 80;
    if (colon) {
        size_t plen = (size_t)(hp_end - (colon + 1));
        if (plen == 0 || plen > 5) return false;   /* no digits, or absurdly long */
        char pbuf[6];
        for (size_t i = 0; i < plen; i++) {
            char c = colon[1 + i];
            if (c < '0' || c > '9') return false;
            pbuf[i] = c;
        }
        pbuf[plen] = '\0';
        long v = strtol(pbuf, NULL, 10);
        if (v <= 0 || v > 65535) return false;
        pt = (uint16_t)v;
    }

    const char *pth = slash ? slash : "/";
    size_t path_len = strlen(pth);
    if (path_len == 0) { pth = "/"; path_len = 1; }
    if (path_len >= (size_t)path_cap) return false;

    /* All checks passed — commit to the caller's buffers. */
    memcpy(host, p, hlen);
    host[hlen] = '\0';
    memcpy(path, pth, path_len);
    path[path_len] = '\0';
    *port = pt;
    return true;
}

#ifndef OTA_HOST_TEST
/* ── cooperative state machine (target only; see host-test seam above) ──── */

#include "netif.h"
#include "dns_resolve.h"
#include "http_client.h"
#include "ota_stage.h"   /* g_ota_buf, OTA_BUF_SZ */

#include "pico/time.h"

/* Sole socket this module ever touches: the same revocable DHCP-socket-0 loan
 * ping.c/http_server.c use (netif.h). One #define, per the task brief. */
#define OTA_SOCK 0

/* Once the app has run this long past boot without crash-looping, it confirms
 * the active image so the bootloader's rollback counter disarms. Longer than the
 * 5 s watchdog period on purpose: a genuinely broken image reboots (watchdog /
 * fault) well before this fires, so it never gets confirmed and the bootloader
 * eventually halts it — while a healthy image (even with the broker down) always
 * reaches this and can't be bricked by a few power cycles. */
#define OTA_CONFIRM_UPTIME_MS   30000u

#define OTA_FLUSH_THRESH        1536u    /* one max http_client body chunk */
#define OTA_ACQUIRE_TIMEOUT_MS  5000u    /* bound on waiting for netif ready + socket 0 */
#define OTA_ACK_TIMEOUT_MS      2000u    /* bound on core0's IC_EVT_OTA_ACK per chunk */
#define OTA_STATUS_TIMEOUT_MS  10000u    /* bound on core0's IC_EVT_OTA_STATUS at end */

typedef enum {
    OS_IDLE = 0,
    OS_ACQUIRE_SOCK,   /* waiting for netif_ready() + netif_ping_acquire_sock0() */
    OS_RESOLVE,        /* dnsr_start/dnsr_poll in flight (skipped for a dotted-quad host) */
    OS_HTTP_START,      /* push IC_CMD_OTA_BEGIN, then httpc_get_start() */
    OS_DOWNLOADING,    /* pumping httpc_poll(); body cb fills g_ota_buf */
    OS_AWAIT_ACK,      /* IC_CMD_OTA_DATA sent; httpc paused until IC_EVT_OTA_ACK */
    OS_SEND_END,       /* push IC_CMD_OTA_END after the final flush's ACK */
    OS_AWAIT_STATUS,   /* waiting for IC_EVT_OTA_STATUS */
    OS_DONE,           /* terminal; state() holds SUCCESS/FAIL until the next trigger */
} ota_phase_t;

static struct {
    ota_state_t  state;
    ota_phase_t  phase;
    char         host[64];
    uint16_t     port;
    char         path[128];
    uint8_t      ip[4];
    bool         have_sock;
    bool         begin_sent;
    uint32_t     fill;                    /* bytes buffered in g_ota_buf, unsent */
    uint32_t     total_sent;              /* bytes handed to core 0 so far */
    bool         final_flush;             /* this AWAIT_ACK's flush was the last one */
    uint32_t     t_ms;                    /* phase-entry timestamp for the bounded waits */
} O;

/* Boot-lifetime (not per-job) one-shot: reset only in ota_init(), so a new
 * ota_trigger() never re-arms the healthy-boot confirmation. */
static bool s_confirmed;

static uint32_t now_ms(void) { return to_ms_since_boot(get_absolute_time()); }

/* "a.b.c.d" (four 0-255 decimal octets, nothing else) -> ip[4]. Anything else
 * (a real hostname, trailing garbage, out-of-range octet) returns false so
 * the caller falls back to DNS. */
static bool parse_dotted_quad(const char *s, uint8_t ip[4]) {
    const char *p = s;
    for (int i = 0; i < 4; i++) {
        char *end;
        long v = strtol(p, &end, 10);
        if (end == p || v < 0 || v > 255) return false;
        ip[i] = (uint8_t)v;
        p = end;
        if (i < 3) {
            if (*p != '.') return false;
            p++;
        }
    }
    return *p == '\0';
}

static void ota_release_sock(void) {
    if (O.have_sock) {
        netif_ping_release_sock0();
        O.have_sock = false;
    }
}

/* Any failure path: best-effort discard whatever core 0 may have staged
 * (harmless/no-op if no session was ever begun) and go terminal. */
static void ota_fail(void) {
    ic_cmd_t c; memset(&c, 0, sizeof c);
    c.type = IC_CMD_OTA_ABORT;
    intercore_push_cmd(&c);
    ota_release_sock();
    O.state = OTA_FAIL;
    O.phase = OS_DONE;
}

static void ota_succeed(void) {
    ota_release_sock();
    O.state = OTA_SUCCESS;
    O.phase = OS_DONE;
    ic_cmd_t c; memset(&c, 0, sizeof c);
    c.type = IC_CMD_REBOOT;
    intercore_push_cmd(&c);
}

/* httpc_get_start's on_body callback: append into g_ota_buf. Runs inside
 * httpc_poll() during OS_DOWNLOADING only — never while AWAIT_ACK holds the
 * buffer for core 0. No LOG_* here (hot byte-path). */
static void ota_body_cb(const uint8_t *data, uint32_t len, void *ctx) {
    (void)ctx;
    if (!data || len == 0) return;
    uint32_t space = OTA_BUF_SZ - O.fill;
    if (len > space) len = space;   /* defensive clamp; the OS_DOWNLOADING flush-
                                      * before-refill invariant (fill <= OTA_BUF_SZ -
                                      * OTA_FLUSH_THRESH going into each httpc_poll())
                                      * plus http_client's <=1.5KB chunks means this
                                      * never actually truncates. */
    memcpy(g_ota_buf + O.fill, data, len);
    O.fill += len;
}

void ota_init(void) {
    memset(&O, 0, sizeof O);
    O.state = OTA_IDLE;
    O.phase = OS_IDLE;
    s_confirmed = false;
}

void ota_trigger(const char *url) {
    if (O.state == OTA_RUNNING) return;   /* one job at a time */
    memset(&O, 0, sizeof O);
    if (!ota_parse_url(url, O.host, sizeof O.host, &O.port, O.path, sizeof O.path)) {
        O.state = OTA_FAIL;
        O.phase = OS_DONE;
        return;
    }
    O.state = OTA_RUNNING;
    O.phase = OS_ACQUIRE_SOCK;
    O.t_ms  = now_ms();
}

void ota_mark_valid(void) {
    ic_cmd_t c; memset(&c, 0, sizeof c);
    c.type = IC_CMD_OTA_CONFIRM;
    intercore_push_cmd(&c);
}

ota_state_t ota_state(void) { return O.state; }

int ota_progress_pct(void) {
    if (O.state != OTA_RUNNING) return 0;
    uint32_t total = httpc_content_length();
    if (total == 0) return 0;
    uint32_t done = O.total_sent + O.fill;
    if (done >= total) return 100;
    return (int)((done * 100u) / total);
}

void ota_on_event(const ic_event_t *e) {
    if (!e || O.state != OTA_RUNNING) return;
    switch (e->type) {
    case IC_EVT_OTA_ACK:
        if (O.phase != OS_AWAIT_ACK) return;
        O.phase = O.final_flush ? OS_SEND_END : OS_DOWNLOADING;
        return;
    case IC_EVT_OTA_STATUS:
        if (O.phase != OS_AWAIT_STATUS) return;
        if (e->ota_status.code == OTA_OK) ota_succeed();
        else ota_fail();
        return;
    default:
        return;
    }
}

void ota_poll(void) {
    /* Healthy-boot confirmation (one-shot). Disarms the bootloader's crash-loop
     * rollback for the running image once it has proven it can run stably past
     * boot. Idempotent downstream (ota_stage_confirm skips the write when already
     * confirmed). Independent of the download state machine below. */
    if (!s_confirmed && now_ms() > OTA_CONFIRM_UPTIME_MS) {
        s_confirmed = true;
        ota_mark_valid();
    }

    switch (O.phase) {
    case OS_IDLE:
    case OS_DONE:
        return;

    case OS_ACQUIRE_SOCK:
        if (!netif_ready() || !netif_ping_acquire_sock0()) {
            if (now_ms() - O.t_ms > OTA_ACQUIRE_TIMEOUT_MS) ota_fail();
            return;
        }
        O.have_sock = true;
        if (parse_dotted_quad(O.host, O.ip)) {
            O.phase = OS_HTTP_START;
        } else {
            uint8_t dns[4];
            netif_get_dns(dns);
            dnsr_start(dns, O.host, OTA_SOCK);
            O.phase = OS_RESOLVE;
        }
        return;

    case OS_RESOLVE: {
        uint8_t ip[4];
        dnsr_status_t s = dnsr_poll(ip);
        if (s == DNSR_PENDING) return;
        if (s == DNSR_FAILED) { ota_fail(); return; }
        memcpy(O.ip, ip, 4);
        O.phase = OS_HTTP_START;
        return;
    }

    case OS_HTTP_START: {
        if (!O.begin_sent) {
            ic_cmd_t c; memset(&c, 0, sizeof c);
            c.type = IC_CMD_OTA_BEGIN;
            if (!intercore_push_cmd(&c)) return;   /* retry next tick */
            O.begin_sent = true;
        }
        httpc_get_start(O.ip, O.port, O.host, O.path, OTA_SOCK, ota_body_cb, NULL);
        O.fill = 0;
        O.total_sent = 0;
        O.phase = OS_DOWNLOADING;
        return;
    }

    case OS_DOWNLOADING: {
        httpc_status_t st = httpc_poll();
        if (st == HTTPC_FAILED) { ota_fail(); return; }
        bool done = (st == HTTPC_DONE);

        if (done && O.fill == 0) { O.phase = OS_SEND_END; return; }

        if (O.fill > 0 && (OTA_BUF_SZ - O.fill < OTA_FLUSH_THRESH || done)) {
            ic_cmd_t c; memset(&c, 0, sizeof c);
            c.type = IC_CMD_OTA_DATA;
            c.ota_data.len = (uint16_t)O.fill;
            if (!intercore_push_cmd(&c)) return;   /* queue full: retry same flush */
            O.total_sent  += O.fill;
            O.fill         = 0;
            O.final_flush  = done;
            O.phase        = OS_AWAIT_ACK;
            O.t_ms         = now_ms();
        }
        return;
    }

    case OS_AWAIT_ACK:
        if (now_ms() - O.t_ms > OTA_ACK_TIMEOUT_MS) ota_fail();
        return;   /* otherwise: ota_on_event() advances the phase */

    case OS_SEND_END: {
        ic_cmd_t c; memset(&c, 0, sizeof c);
        c.type = IC_CMD_OTA_END;
        c.ota_end.total_len = O.total_sent;
        if (!intercore_push_cmd(&c)) return;   /* retry next tick */
        O.phase = OS_AWAIT_STATUS;
        O.t_ms  = now_ms();
        return;
    }

    case OS_AWAIT_STATUS:
        if (now_ms() - O.t_ms > OTA_STATUS_TIMEOUT_MS) ota_fail();
        return;   /* otherwise: ota_on_event() finishes the run */
    }
}

#endif /* !OTA_HOST_TEST */
