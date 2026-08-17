/*
 * fwupdate.c — core-1 pull-based-OTA manifest checker. See fwupdate.h for the
 * API.
 *
 * Host-test seam (matches ota.c/dns_resolve.c's pattern): the state machine
 * below needs the W5100S socket API (via netif.h/dns_resolve.h/http_client.h)
 * and ota.h's ota_state()/ota_trigger(), none of which exist/link on the
 * host, so it is compiled out under -DFWUPDATE_HOST_TEST. fw_json_get() is
 * pure (no sockets, no globals) and unaffected either way — it's the only
 * thing test/fwupdate_test.c exercises.
 */

#include "fwupdate.h"
#include "intercore.h"

#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/* ── pure: dumb "key":"value" extractor (host-testable; unconditional) ──── */

static bool is_ws(char c) { return c == ' ' || c == '\t' || c == '\n' || c == '\r'; }

bool fw_json_get(const char *buf, const char *key, char *out, int cap) {
    if (!buf || !key || !out || cap <= 0) return false;

    char pat[40];
    int pl = snprintf(pat, sizeof pat, "\"%s\"", key);
    if (pl <= 0 || pl >= (int)sizeof pat) return false;

    const char *cursor = buf;
    for (;;) {
        const char *hit = strstr(cursor, pat);
        if (!hit) return false;

        /* The quoted key must be followed (after optional whitespace) by a
         * ':' — this is what stops a bare occurrence of the same text inside
         * another key's *value* (which is never both quote-delimited and
         * colon-followed right there) from being mistaken for the key. */
        const char *p = hit + pl;
        while (is_ws(*p)) p++;
        if (*p != ':') { cursor = hit + 1; continue; }
        p++;
        while (is_ws(*p)) p++;
        if (*p != '"') { cursor = hit + 1; continue; }   /* only string values supported */
        p++;

        const char *end = strchr(p, '"');
        if (!end) return false;

        size_t vlen = (size_t)(end - p);
        if (vlen == 0) return false;                 /* empty value -> false */
        if (vlen >= (size_t)cap) return false;        /* wouldn't fit */

        memcpy(out, p, vlen);
        out[vlen] = '\0';
        return true;
    }
}

#ifndef FWUPDATE_HOST_TEST
/* ── cooperative state machine (target only; see host-test seam above) ──── */

#include "netif.h"
#include "dns_resolve.h"
#include "http_client.h"
#include "log.h"          /* LOG_WARN — abort reasons */
#include "ota.h"
#include "version.h"   /* FW_VERSION — generated from git at build time */

#include "pico/time.h"
#include <stdlib.h>

/* Sole socket this module ever touches: the same revocable DHCP-socket-0 loan
 * ota.c/ping.c/http_server.c use (netif.h). Never held at the same time as
 * ota.c's use of it — a check only starts while ota_state() != OTA_RUNNING,
 * and ota_trigger() takes priority over an in-flight check by simply not
 * caring about it (fwupdate holds the socket for, at most, a small manifest
 * fetch and always releases it in the same tick loop it's parked in). */
#define FW_SOCK 0

#ifndef FWUPDATE_MANIFEST_URL
#define FWUPDATE_MANIFEST_URL "http://ota.8bitcloud.com/light_switch.json"
#endif

#define FW_ACQUIRE_TIMEOUT_MS  5000u    /* bound on waiting for netif ready + socket 0 */
#define FW_RETRY_MS           60000u    /* after an aborted check, try again this soon */
#define FW_FETCH_TIMEOUT_MS   10000u    /* bound on the manifest GET completing */
#define FW_BODY_CAP            1024u    /* manifest is documented as <1 KB */

typedef enum {
    FS_IDLE = 0,        /* waiting for the interval or an announce() */
    FS_ACQUIRE_SOCK,    /* waiting for netif_ready() + netif_sock0_acquire() */
    FS_RESOLVE,         /* dnsr_start/dnsr_poll in flight (skipped for a dotted-quad host) */
    FS_HTTP_START,      /* httpc_get_start() about to fire */
    FS_FETCHING,        /* pumping httpc_poll(); body cb fills s_body */
} fw_phase_t;

static struct {
    fw_phase_t phase;
    char       host[64];
    uint16_t   port;
    char       path[128];
    uint8_t    ip[4];
    bool       have_sock;
    uint32_t   t_ms;         /* phase-entry timestamp for the bounded waits */
    uint32_t   body_len;
    bool       pending;      /* announce() forces the next eligible poll to check now */
    uint32_t   last_check_ms;
    /* A check that aborts must NOT wait out the full hourly interval before
     * trying again — pending is consumed on entry, so without this a single
     * transient (socket 0 lent to the web UI, DHCP renewal guard band) silently
     * discards the request and leaves `latest` stale for an hour. */
    uint32_t   retry_at_ms;
    bool       retry_armed;
    char       last_err[40]; /* why the last check aborted; "" once one succeeds */

    char installed[48];
    char latest[48];
    char latest_url[200];
    char title[64];
    char release_url[128];
    char notes[256];
} F;

static char s_body[FW_BODY_CAP];

static uint32_t now_ms(void) { return to_ms_since_boot(get_absolute_time()); }

/* "a.b.c.d" (four 0-255 decimal octets, nothing else) -> ip[4]. Anything else
 * (a real hostname, trailing garbage, out-of-range octet) returns false so
 * the caller falls back to DNS. (Small local copy of ota.c's helper — both
 * are file-local statics in independent modules.) */
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

static void fw_release_sock(void) {
    if (F.have_sock) {
        netif_sock0_release();
        F.have_sock = false;
    }
}

/* Every abort path lands here. It MUST say why: an aborted check leaves
 * installed/latest looking current, so a silent failure is indistinguishable
 * from "you are up to date" — which is exactly how a stuck socket-0 lend hid a
 * broken update path for ten minutes on the bench. */
static void fw_abort(const char *why) {
    LOG_WARN("fwup", "check aborted: %s (retry in %lus)", why,
             (unsigned long)(FW_RETRY_MS / 1000u));
    snprintf(F.last_err, sizeof F.last_err, "%s", why);
    fw_release_sock();
    F.phase       = FS_IDLE;
    F.retry_at_ms = now_ms() + FW_RETRY_MS;
    F.retry_armed = true;
}

/* httpc_get_start's on_body callback: append into s_body. Runs inside
 * httpc_poll() during FS_FETCHING only. No LOG_* here (hot byte-path). */
static void fw_body_cb(const uint8_t *data, uint32_t len, void *ctx) {
    (void)ctx;
    if (!data || len == 0) return;
    uint32_t space = (uint32_t)sizeof(s_body) - 1 - F.body_len;
    if (len > space) len = space;
    if (len == 0) return;
    memcpy(s_body + F.body_len, data, len);
    F.body_len += len;
}

/* Only commit version/url (the fields fwupdate_available()/install() rely
 * on) if BOTH are present — a malformed/partial response leaves the
 * last-known-good manifest state untouched rather than wiping it. The
 * optional descriptive fields are updated independently, best-effort. */
static void fw_parse_manifest(void) {
    char ver[sizeof F.latest];
    char url[sizeof F.latest_url];
    bool have_ver = fw_json_get(s_body, "version", ver, sizeof ver);
    bool have_url = fw_json_get(s_body, "url", url, sizeof url);
    if (have_ver && have_url) {
        memcpy(F.latest, ver, sizeof F.latest);
        memcpy(F.latest_url, url, sizeof F.latest_url);
    }

    char tmp[sizeof F.notes];
    if (fw_json_get(s_body, "title", tmp, sizeof F.title))
        memcpy(F.title, tmp, sizeof F.title);
    if (fw_json_get(s_body, "release_url", tmp, sizeof F.release_url))
        memcpy(F.release_url, tmp, sizeof F.release_url);
    if (fw_json_get(s_body, "notes", tmp, sizeof F.notes))
        memcpy(F.notes, tmp, sizeof F.notes);
}

void fwupdate_init(void) {
    memset(&F, 0, sizeof F);
    F.phase = FS_IDLE;
    F.pending = true;   /* check once as soon as the network comes up */
    strncpy(F.installed, FW_VERSION, sizeof F.installed - 1);
}

void fwupdate_announce(void) {
    F.pending = true;
}

void fwupdate_install(void) {
    if (F.latest_url[0] != '\0') ota_trigger(F.latest_url);
}

const char *fwupdate_installed(void)   { return F.installed; }
const char *fwupdate_latest(void)      { return F.latest; }
const char *fwupdate_latest_url(void)  { return F.latest_url; }
const char *fwupdate_title(void)       { return F.title; }
const char *fwupdate_release_url(void) { return F.release_url; }
const char *fwupdate_last_error(void) { return F.last_err; }
const char *fwupdate_notes(void)       { return F.notes; }

bool fwupdate_available(void) {
    return F.latest[0] != '\0' && strcmp(F.latest, F.installed) != 0;
}

/* ── status JSON (for GET /api/ota and the web UI) ───────────────────────── */
/* Clamped accumulating snprintf — keeps n <= cap so (cap - n) never goes
 * negative into vsnprintf. */
static int acc(char *buf, int cap, int n, const char *fmt, ...) {
    if (n < 0 || n >= cap) return cap;
    va_list ap; va_start(ap, fmt);
    int w = vsnprintf(buf + n, (size_t)(cap - n), fmt, ap);
    va_end(ap);
    if (w < 0) return n;
    n += w;
    return n > cap ? cap : n;
}
/* Append a JSON-escaped quoted string. Manifest fields are attacker-influenced
 * (fetched over plain HTTP), so escape quotes/backslashes/control chars rather
 * than trust them to be clean JSON. */
static int acc_str(char *buf, int cap, int n, const char *s) {
    if (n < 0 || n >= cap) return cap;
    buf[n++] = '"';
    for (; *s && n < cap - 7; s++) {
        unsigned char ch = (unsigned char)*s;
        switch (ch) {
        case '"': case '\\': buf[n++] = '\\'; buf[n++] = (char)ch; break;
        case '\n': buf[n++] = '\\'; buf[n++] = 'n'; break;
        case '\r': buf[n++] = '\\'; buf[n++] = 'r'; break;
        case '\t': buf[n++] = '\\'; buf[n++] = 't'; break;
        default:
            if (ch < 0x20) n += snprintf(buf + n, (size_t)(cap - n), "\\u%04x", ch);
            else buf[n++] = (char)ch;
        }
    }
    if (n < cap) buf[n++] = '"';
    return n > cap ? cap : n;
}

int fwupdate_status_json(char *buf, int cap) {
    const char *st = ota_state() == OTA_RUNNING ? "running"
                   : ota_state() == OTA_SUCCESS ? "success"
                   : ota_state() == OTA_FAIL    ? "fail" : "idle";
    int n = 0;
    n = acc(buf, cap, n, "{\"installed\":");
    n = acc_str(buf, cap, n, fwupdate_installed());
    n = acc(buf, cap, n, ",\"latest\":");
    n = acc_str(buf, cap, n, fwupdate_latest());
    n = acc(buf, cap, n, ",\"available\":%s,\"url\":", fwupdate_available() ? "true" : "false");
    n = acc_str(buf, cap, n, fwupdate_latest_url());
    n = acc(buf, cap, n, ",\"title\":");
    n = acc_str(buf, cap, n, fwupdate_title());
    n = acc(buf, cap, n, ",\"last_error\":");
    n = acc_str(buf, cap, n, fwupdate_last_error());
    /* Distinct from "last_error" above: that field is fwupdate.c's own —
     * why the last MANIFEST check aborted (DNS/socket/http status on the
     * small JSON fetch). This one is ota.c's — why the IMAGE install itself
     * failed (bad HTTP status/signature/flash write/timeout on the actual
     * firmware download+flash). Conflating them would make "last_error"
     * ambiguous about which phase failed; kept separate instead, populated
     * only when there's actually something to report. */
    n = acc(buf, cap, n, ",\"ota_error\":");
    n = acc_str(buf, cap, n, ota_state() == OTA_FAIL ? ota_last_error() : "");
    n = acc(buf, cap, n, ",\"release_url\":");
    n = acc_str(buf, cap, n, fwupdate_release_url());
    n = acc(buf, cap, n, ",\"notes\":");
    n = acc_str(buf, cap, n, fwupdate_notes());
    n = acc(buf, cap, n, ",\"state\":\"%s\",\"pct\":%d}", st, ota_progress_pct());
    return n;
}

void fwupdate_poll(void) {
    switch (F.phase) {
    case FS_IDLE: {
        bool due = F.pending
                || (F.retry_armed && (int32_t)(now_ms() - F.retry_at_ms) >= 0)
                || (now_ms() - F.last_check_ms >= FWCHECK_INTERVAL_S * 1000u);
        if (!due) return;
        if (ota_state() == OTA_RUNNING) return;   /* image download owns http_client + sock0 */
        if (!netif_ready()) return;

        /* Config override first (cfg fw_url=…, e.g. a bench device polling a
         * local manifest server), compiled-in default otherwise. */
        const char *manifest_url = intercore_snapshot()->fw_url[0]
                                       ? intercore_snapshot()->fw_url
                                       : FWUPDATE_MANIFEST_URL;
        if (!ota_parse_url(manifest_url, F.host, sizeof F.host, &F.port,
                            F.path, sizeof F.path)) {
            /* Malformed compiled-in URL — nothing to retry into, so just clear
             * the pending flag and wait out the normal interval rather than
             * spinning every tick. */
            F.pending = false;
            F.last_check_ms = now_ms();
            return;
        }
        F.pending = false;
        F.retry_armed = false;
        F.last_check_ms = now_ms();
        F.phase = FS_ACQUIRE_SOCK;
        F.t_ms  = now_ms();
        return;
    }

    case FS_ACQUIRE_SOCK:
        if (!netif_ready() || !netif_sock0_acquire()) {
            if (now_ms() - F.t_ms > FW_ACQUIRE_TIMEOUT_MS)
                fw_abort(netif_ready() ? "socket 0 unavailable" : "network not ready");
            return;
        }
        F.have_sock = true;
        if (parse_dotted_quad(F.host, F.ip)) {
            F.phase = FS_HTTP_START;
        } else {
            uint8_t dns[4];
            netif_get_dns(dns);
            dnsr_start(dns, F.host, FW_SOCK);
            F.phase = FS_RESOLVE;
        }
        return;

    case FS_RESOLVE: {
        uint8_t ip[4];
        dnsr_status_t s = dnsr_poll(ip);
        if (s == DNSR_PENDING) return;
        if (s == DNSR_FAILED) { fw_abort("dns lookup failed"); return; }
        memcpy(F.ip, ip, 4);
        F.phase = FS_HTTP_START;
        return;
    }

    case FS_HTTP_START:
        F.body_len = 0;
        httpc_get_start(F.ip, F.port, F.host, F.path, FW_SOCK, fw_body_cb, NULL);
        F.t_ms  = now_ms();
        F.phase = FS_FETCHING;
        return;

    case FS_FETCHING: {
        httpc_status_t st = httpc_poll();
        if (st == HTTPC_FAILED) { fw_abort("http request failed"); return; }
        if (st == HTTPC_PENDING) {
            if (now_ms() - F.t_ms > FW_FETCH_TIMEOUT_MS) fw_abort("manifest fetch timeout");
            return;
        }
        /* HTTPC_DONE */
        if (httpc_status_code() != 200) {
            /* A 404/500 was previously discarded in silence, leaving the last
             * good manifest in place and the entity reading "up to date". */
            char why[40];
            snprintf(why, sizeof why, "manifest http %d", httpc_status_code());
            fw_abort(why);
            return;
        }
        s_body[F.body_len < sizeof(s_body) ? F.body_len : sizeof(s_body) - 1] = '\0';
        fw_parse_manifest();
        F.last_err[0] = '\0';        /* a clean check clears the last failure */
        fw_release_sock();
        F.phase = FS_IDLE;
        return;
    }
    }
}

#endif /* !FWUPDATE_HOST_TEST */
