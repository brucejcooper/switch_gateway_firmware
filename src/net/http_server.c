/*
 * http_server.c — non-blocking embedded config web server (core 1). See header.
 *
 * Design notes:
 *  - One connection per socket, single request then close. The W5100S has no
 *    accept(): a socket sits in SOCK_LISTEN and auto-promotes to ESTABLISHED on
 *    a SYN, so each managed socket is its own little accept-serve-close loop.
 *  - Everything is poll-driven off the socket status register (getSn_SR) and the
 *    RX/TX free-size registers (getSn_RX_RSR / getSn_TX_FSR). We never block and
 *    send at most one bounded chunk per socket per tick, so a large transfer
 *    can't starve mqtt_app_tick() or trip the core-1 watchdog.
 *  - Responses are close-delimited (Connection: close). /api/state has no
 *    Content-Length and is streamed straight from web_state's cursor renderer.
 */

#include "http_server.h"
#include "web_assets.h"
#include "web_state.h"
#include "netif.h"
#include "intercore.h"
#include "log.h"
#include "cdc_out.h"       /* cdc_out_history() — GET /api/log */
#include "fwupdate.h"      /* fwupdate_status_json/announce/install — /api/ota */

#include "socket.h"
#include "wizchip_conf.h"   /* getSn_SR / getSn_RX_RSR / getSn_TX_FSR / Sn_* */
#include "jsmn.h"
#include "mapping.h"        /* mapping_target_t, TARGET_*, MAX_TARGETS_PER_BTN */

#include "pico/time.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#define HTTP_PORT        80
#define HTTP_SOCK_BUFSZ  2048u   /* per-socket TX/RX buffer (netif txsz/rxsz) */
#define HTTP_BUF         2048u    /* per-conn request reassembly / send staging */
#define RECV_TIMEOUT_MS  8000u
#define SEND_TIMEOUT_MS  8000u
#define DRAIN_TIMEOUT_MS 2000u

/* Managed sockets. Sockets 2-3 are always ours; socket 0 is lent opportunistically
 * by the netif DHCP arbiter (revocable bonus capacity). s_conn[2] is socket 0 and
 * is only pumped while netif_web_sock0_enabled(). */
#define N_BASE  2                 /* sockets 2,3 — always ours */
#define N_SOCK  (N_BASE + 1)      /* + socket 0 (opportunistic) */
static const uint8_t k_socks[N_SOCK] = { 2, 3, 0 };

typedef enum { CONN_LISTEN, CONN_RECV, CONN_SEND, CONN_DRAIN } conn_state_t;
typedef enum { RESP_MEM, RESP_STATE } resp_kind_t;
typedef enum { PH_HDR, PH_BODY } send_phase_t;

typedef struct {
    uint8_t  sn;
    conn_state_t st;
    uint32_t t_ms;            /* last activity for timeouts */

    /* request reassembly (during CONN_RECV) */
    uint16_t rx_len;
    bool     hdr_complete;
    uint16_t content_len;
    uint16_t body_start;      /* index of body in buf */

    /* response */
    resp_kind_t  resp;
    send_phase_t phase;
    /* staged bytes to send out of buf[] (header, then state chunks) */
    uint16_t out_len, out_sent;
    /* RESP_MEM body (points into flash/rodata) */
    const uint8_t *mem; uint32_t mem_len, mem_sent;
    /* RESP_STATE streaming */
    web_render_cursor_t cur;
    bool state_done;

    char buf[HTTP_BUF];
} conn_t;

static conn_t s_conn[N_SOCK];
static bool   s_sock0_active = false;   /* are we currently serving on socket 0? */

/* Live copy of the Basic-auth credentials. Seeded from the launch snapshot;
 * updated in place when the password is changed via POST /api/auth so the new
 * credentials take effect immediately (core 0 persists them for next boot). */
static char s_user[32];
static char s_pass[48];

static uint32_t now_ms(void) { return to_ms_since_boot(get_absolute_time()); }

/* ── socket lifecycle ───────────────────────────────────────────────────────*/
/* Count socket()/listen() failures here. arm_listen runs in the per-tick pump,
 * so per the hot-path rule we must NOT LOG_* — bump a counter, surfaced off the
 * hot path via http_server_arm_fail_count(). A non-zero, climbing count means a
 * web socket failed to (re)arm and that listener is dead. */
static uint32_t s_arm_fails = 0;
uint32_t http_server_arm_fail_count(void) { return s_arm_fails; }

/* Self-heal watchdog: if NEITHER base socket reaches LISTEN/ESTABLISHED for this
 * long while netif is otherwise up, the W5100S socket layer is wedged (arm_listen
 * keeps failing, so port 80 refuses) and only a full chip reset clears it — ask
 * netif to recover. s_last_listen_ms tracks the last tick a base socket was
 * healthy; it's also reset whenever netif isn't ready so the window only counts
 * contiguous wedged-while-up time. */
#define WEB_WEDGE_TIMEOUT_MS 30000u
static uint32_t s_last_listen_ms = 0;

/* GET /api/log snapshot. cdc_out_history() must be linearised into a stable buffer
 * because RESP_MEM streams the body pointer across many ticks while the live ring
 * keeps moving. One shared buffer → one /api/log stream at a time; a second
 * concurrent request gets 503 until the first connection is re-armed. */
#define LOG_SNAP_CAP 8192
static char    s_logbuf[LOG_SNAP_CAP];
static conn_t *s_logbuf_owner = NULL;

static void arm_listen(conn_t *c) {
    if (s_logbuf_owner == c) s_logbuf_owner = NULL;   /* release the log snapshot */
    close(c->sn);
    int8_t s = socket(c->sn, Sn_MR_TCP, HTTP_PORT, SF_IO_NONBLOCK);
    int8_t l = (s == c->sn) ? listen(c->sn) : (int8_t)-1;
    if (s != c->sn || l != SOCK_OK) s_arm_fails++;
    c->st = CONN_LISTEN;
    c->rx_len = 0; c->hdr_complete = false; c->content_len = 0; c->body_start = 0;
    c->out_len = c->out_sent = 0; c->mem = NULL; c->mem_len = c->mem_sent = 0;
    c->state_done = false;
}

void http_server_init(void) {
    const ic_snapshot_t *sn = intercore_snapshot();
    snprintf(s_user, sizeof s_user, "%s", sn && sn->web_user[0] ? sn->web_user : "admin");
    snprintf(s_pass, sizeof s_pass, "%s", sn && sn->web_pass[0] ? sn->web_pass : "admin");
    for (size_t i = 0; i < N_SOCK; i++) s_conn[i].sn = k_socks[i];
    /* Arm the always-ours sockets now; socket 0 is armed on demand by the tick. */
    for (size_t i = 0; i < N_BASE; i++) arm_listen(&s_conn[i]);
    s_last_listen_ms = now_ms();
    LOG_INFO("http", "config web server on :%d (sockets 2-3 +0 when DHCP idle)", HTTP_PORT);
}

/* ── tiny HTTP helpers ───────────────────────────────────────────────────────*/

/* Case-insensitive search for a header; returns a pointer to the value (leading
 * spaces skipped) within the NUL-terminated request, or NULL. */
static const char *find_header(const char *req, const char *name) {
    size_t nlen = strlen(name);
    for (const char *p = req; *p; ) {
        const char *eol = strstr(p, "\r\n");
        if (!eol || eol == p) break;                 /* end of headers */
        if ((size_t)(eol - p) > nlen && p[nlen] == ':' &&
            strncasecmp(p, name, nlen) == 0) {
            const char *v = p + nlen + 1;
            while (*v == ' ' || *v == '\t') v++;
            return v;
        }
        p = eol + 2;
    }
    return NULL;
}

static int b64val(char c) {
    if (c >= 'A' && c <= 'Z') return c - 'A';
    if (c >= 'a' && c <= 'z') return c - 'a' + 26;
    if (c >= '0' && c <= '9') return c - '0' + 52;
    if (c == '+') return 62;
    if (c == '/') return 63;
    return -1;
}
static int b64decode(const char *in, uint8_t *out, int outcap) {
    int n = 0, bits = 0, acc = 0;
    for (; *in && *in != '\r' && *in != '\n' && *in != ' '; in++) {
        int v = b64val(*in);
        if (v < 0) break;                /* padding '=' or invalid -> stop */
        acc = (acc << 6) | v; bits += 6;
        if (bits >= 8) { bits -= 8; if (n < outcap) out[n++] = (uint8_t)(acc >> bits); }
    }
    return n;
}

/* Validate Basic auth against the snapshot credentials. */
static bool check_auth(const char *req) {
    const char *v = find_header(req, "Authorization");
    if (!v || strncasecmp(v, "Basic ", 6) != 0) return false;
    uint8_t dec[96];
    int n = b64decode(v + 6, dec, sizeof dec - 1);
    if (n <= 0) return false;
    dec[n] = '\0';
    char want[96];
    int wl = snprintf(want, sizeof want, "%s:%s", s_user, s_pass);
    if (wl <= 0 || wl != n) return false;
    /* length-checked compare (not early-out on first diff) */
    uint8_t diff = 0;
    for (int i = 0; i < n; i++) diff |= (uint8_t)(dec[i] ^ (uint8_t)want[i]);
    return diff == 0;
}

/* ── response builders ───────────────────────────────────────────────────────*/

static void resp_mem(conn_t *c, const char *status, const char *extra_hdr,
                     const char *mime, const uint8_t *body, uint32_t len, bool gz) {
    c->out_len = (uint16_t)snprintf(c->buf, sizeof c->buf,
        "HTTP/1.1 %s\r\n"
        "Content-Type: %s\r\n"
        "Content-Length: %lu\r\n"
        "%s%s"
        "Connection: close\r\n\r\n",
        status, mime, (unsigned long)len,
        gz ? "Content-Encoding: gzip\r\n" : "",
        extra_hdr ? extra_hdr : "");
    c->out_sent = 0;
    c->resp = RESP_MEM; c->phase = PH_HDR;
    c->mem = body; c->mem_len = len; c->mem_sent = 0;
    c->st = CONN_SEND; c->t_ms = now_ms();
}

/* Small fixed text/JSON reply (body is a string literal in flash). */
static void resp_text(conn_t *c, const char *status, const char *mime, const char *body) {
    resp_mem(c, status, NULL, mime, (const uint8_t *)body, (uint32_t)strlen(body), false);
}

/* Dump the retained log scrollback (GET /api/log). Network path to the logs that
 * otherwise live only on USB-CDC — works even when USB has no reader or is wedged.
 * Snapshots under cdc_out's lock into s_logbuf, then streams it like any RESP_MEM
 * body. Auth is already enforced by route() before we get here. */
static void resp_log(conn_t *c) {
    if (s_logbuf_owner && s_logbuf_owner != c) {
        resp_text(c, "503 Service Unavailable", "text/plain", "log busy, retry\n");
        return;
    }
    int n = cdc_out_history(s_logbuf, sizeof s_logbuf);
    s_logbuf_owner = c;
    resp_mem(c, "200 OK", "Cache-Control: no-store\r\n", "text/plain; charset=utf-8",
             (const uint8_t *)s_logbuf, (uint32_t)n, false);
}

/* Small dynamic JSON reply. Unlike resp_text (whose body must be a stable
 * pointer valid across the multi-tick send), this copies the body INTO the
 * connection's own buf right after the header, so mem points at per-connection
 * storage — safe with concurrent connections, no shared buffer/owner guard. */
static void resp_json(conn_t *c, const char *status, const char *body, int blen) {
    int hlen = snprintf(c->buf, sizeof c->buf,
        "HTTP/1.1 %s\r\n"
        "Content-Type: application/json\r\n"
        "Content-Length: %d\r\n"
        "Cache-Control: no-store\r\n"
        "Connection: close\r\n\r\n",
        status, blen);
    if (hlen < 0) hlen = 0;
    if (hlen > (int)sizeof c->buf) hlen = (int)sizeof c->buf;
    if (blen > (int)sizeof c->buf - hlen) blen = (int)sizeof c->buf - hlen;
    memcpy(c->buf + hlen, body, (size_t)blen);
    c->out_len = (uint16_t)hlen; c->out_sent = 0;
    c->resp = RESP_MEM; c->phase = PH_HDR;
    c->mem = (const uint8_t *)(c->buf + hlen); c->mem_len = (uint32_t)blen; c->mem_sent = 0;
    c->st = CONN_SEND; c->t_ms = now_ms();
}

/* GET /api/ota — firmware/update status for the web Firmware tab and for
 * scripted/agent polling. Auth is already enforced by route(). */
static void resp_ota(conn_t *c) {
    char body[900];
    int n = fwupdate_status_json(body, sizeof body);
    resp_json(c, "200 OK", body, n);
}

static void resp_401(conn_t *c) {
    resp_mem(c, "401 Unauthorized",
             "WWW-Authenticate: Basic realm=\"lightswitch\"\r\n",
             "text/plain", (const uint8_t *)"auth required", 13, false);
}

static void resp_state(conn_t *c) {
    char etag[24];
    snprintf(etag, sizeof etag, "ETag: \"%lu\"\r\n", (unsigned long)web_state_seq());
    c->out_len = (uint16_t)snprintf(c->buf, sizeof c->buf,
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: application/json\r\n"
        "Cache-Control: no-store\r\n"
        "%s"
        "Connection: close\r\n\r\n", etag);
    c->out_sent = 0;
    c->resp = RESP_STATE; c->phase = PH_HDR;
    web_render_begin(&c->cur); c->state_done = false;
    c->st = CONN_SEND; c->t_ms = now_ms();
}

/* ── JSON request parsing (jsmn) + write dispatch ───────────────────────────*/

static bool jeq(const char *js, const jsmntok_t *t, const char *s) {
    int len = t->end - t->start;
    return t->type == JSMN_STRING && (int)strlen(s) == len &&
           strncmp(js + t->start, s, len) == 0;
}
static int jfind(const char *js, const jsmntok_t *t, int n, const char *key) {
    for (int i = 1; i < n; i++) if (jeq(js, &t[i], key)) return i + 1;
    return -1;
}
static long jint(const char *js, const jsmntok_t *t) {
    char b[16]; int len = t->end - t->start; if (len >= (int)sizeof b) len = (int)sizeof b - 1;
    memcpy(b, js + t->start, len); b[len] = '\0'; return strtol(b, NULL, 10);
}
static bool jhas(const char *js, const jsmntok_t *t, int n, const char *key) {
    return jfind(js, t, n, key) >= 0;
}
static int jint_key(const char *js, const jsmntok_t *t, int n, const char *key, int def) {
    int i = jfind(js, t, n, key); return i < 0 ? def : (int)jint(js, &t[i]);
}
static void jstr_key(const char *js, const jsmntok_t *t, int n, const char *key,
                     char *out, size_t cap) {
    out[0] = '\0';
    int i = jfind(js, t, n, key);
    if (i < 0) return;
    int len = t[i].end - t[i].start; if (len >= (int)cap) len = (int)cap - 1;
    memcpy(out, js + t[i].start, len); out[len] = '\0';
}
static bool jbool_key(const char *js, const jsmntok_t *t, int n, const char *key, bool def) {
    int i = jfind(js, t, n, key);
    if (i < 0) return def;
    char ch = js[t[i].start];
    return ch == 't' || ch == 'T' || ch == '1';
}
static bool jtri(const char *js, const jsmntok_t *t, int n, const char *key, ic_tri_t *out) {
    int i = jfind(js, t, n, key);
    if (i < 0) return false;
    if (jeq(js, &t[i], "toggle"))                          *out = IC_TRI_TOGGLE;
    else if (jeq(js, &t[i], "on") || jeq(js, &t[i], "true")) *out = IC_TRI_ON;
    else { char ch = js[t[i].start]; *out = (ch == 't' || ch == '1') ? IC_TRI_ON : IC_TRI_OFF; }
    return true;
}
static bool jip(const char *js, const jsmntok_t *t, int n, const char *key, uint8_t ip[4]) {
    char b[20]; jstr_key(js, t, n, key, b, sizeof b);
    unsigned a0, a1, a2, a3;
    if (sscanf(b, "%u.%u.%u.%u", &a0, &a1, &a2, &a3) != 4) return false;
    ip[0] = (uint8_t)a0; ip[1] = (uint8_t)a1; ip[2] = (uint8_t)a2; ip[3] = (uint8_t)a3;
    return true;
}
/* Walk a JSON targets array [{slave,addr|group},...] at value-token `ai`. */
static uint8_t parse_targets_at(const char *js, const jsmntok_t *t, int n, int ai,
                                mapping_target_t *out, uint8_t max) {
    if (ai < 0 || t[ai].type != JSMN_ARRAY) return 0;
    uint8_t c = 0; int i = ai + 1, count = t[ai].size;
    for (int el = 0; el < count && c < max && i < n; el++) {
        if (t[i].type != JSMN_OBJECT) break;
        int sz = t[i].size; i++;
        int sl = 0, addr = 0, grp = -1;
        for (int f = 0; f < sz && i + 1 < n; f++) {
            if      (jeq(js, &t[i], "slave")) sl   = (int)jint(js, &t[i + 1]);
            else if (jeq(js, &t[i], "addr"))  addr = (int)jint(js, &t[i + 1]);
            else if (jeq(js, &t[i], "group")) grp  = (int)jint(js, &t[i + 1]);
            i += 2;
        }
        out[c].slave = (uint8_t)sl;
        out[c].addr  = grp >= 0 ? TARGET_MAKE_GROUP(grp) : (uint8_t)(addr & 0x3F);
        c++;
    }
    return c;
}
/* Decode a JSON member array [0,1,5] at value-token `ai` to a 64-bit mask. */
static uint64_t parse_members_at(const char *js, const jsmntok_t *t, int n, int ai) {
    uint64_t mask = 0;
    if (ai < 0 || t[ai].type != JSMN_ARRAY) return 0;
    int count = t[ai].size, i = ai + 1;
    for (int el = 0; el < count && i < n; el++, i++) {
        int a = (int)jint(js, &t[i]);
        if (a >= 0 && a < 64) mask |= (1ull << a);
    }
    return mask;
}

static void resp_ok(conn_t *c) { resp_text(c, "200 OK", "application/json", "{\"ok\":true}"); }
static void push_cmd_resp(conn_t *c, const ic_cmd_t *cmd) {
    if (intercore_push_cmd(cmd)) resp_ok(c);
    else resp_text(c, "503 Service Unavailable", "application/json", "{\"error\":\"busy\"}");
}

static void handle_post(conn_t *c, const char *path, const char *body, int blen) {
    jsmn_parser jp; jsmntok_t tk[96]; jsmn_init(&jp);
    int nt = jsmn_parse(&jp, body, blen, tk, 96);
    if (nt < 1 || tk[0].type != JSMN_OBJECT) {
        resp_text(c, "400 Bad Request", "application/json", "{\"error\":\"bad json\"}");
        return;
    }
    ic_cmd_t cmd; memset(&cmd, 0, sizeof cmd);

    if (strcmp(path, "/api/reboot") == 0) {
        /* Out-of-band reboot: core 0 answers this via the watchdog, without
         * touching USB — the recovery path when the USB CLI/picotool are wedged.
         * The 200 goes out before core 0 acts; the socket then drops on reset. */
        cmd.type = IC_CMD_REBOOT;
        push_cmd_resp(c, &cmd); return;
    }
    /* OTA triggers. fwupdate lives on core 1 (same core as this server), so these
     * call it directly — no intercore. check = force a manifest re-check;
     * install = start the pull+flash of the last-known manifest (no-op until a
     * manifest has been fetched, and while an OTA is already running). */
    if (strcmp(path, "/api/ota/check") == 0)   { fwupdate_announce(); resp_ok(c); return; }
    if (strcmp(path, "/api/ota/install") == 0) { fwupdate_install();  resp_ok(c); return; }
    if (strcmp(path, "/api/scan") == 0) {
        cmd.type = IC_CMD_SCAN;
        cmd.scan.slave = (uint8_t)jint_key(body, tk, nt, "slave", 0);
        push_cmd_resp(c, &cmd); return;
    }
    if (strcmp(path, "/api/relay") == 0) {
        cmd.type = IC_CMD_SET_RELAY;
        cmd.set_relay.ch = (uint8_t)jint_key(body, tk, nt, "ch", 0);
        ic_tri_t on = IC_TRI_OFF; jtri(body, tk, nt, "on", &on);
        cmd.set_relay.on = on;
        push_cmd_resp(c, &cmd); return;
    }
    if (strcmp(path, "/api/dali") == 0) {
        cmd.type = IC_CMD_SET_DALI;
        cmd.set_dali.slave = (uint8_t)jint_key(body, tk, nt, "slave", 0);
        cmd.set_dali.addr  = (uint8_t)jint_key(body, tk, nt, "addr", 0);
        uint16_t f = 0; ic_tri_t on;
        if (jtri(body, tk, nt, "on", &on))   { cmd.set_dali.on = on; f |= IC_DALI_F_ON; }
        if (jhas(body, tk, nt, "bri"))       { cmd.set_dali.bri       = (uint8_t)jint_key(body, tk, nt, "bri", 0);       f |= IC_DALI_F_BRI; }
        if (jhas(body, tk, nt, "min"))       { cmd.set_dali.min       = (uint8_t)jint_key(body, tk, nt, "min", 0);       f |= IC_DALI_F_MIN; }
        if (jhas(body, tk, nt, "max"))       { cmd.set_dali.max       = (uint8_t)jint_key(body, tk, nt, "max", 0);       f |= IC_DALI_F_MAX; }
        if (jhas(body, tk, nt, "power_on"))  { cmd.set_dali.power_on  = (uint8_t)jint_key(body, tk, nt, "power_on", 0);  f |= IC_DALI_F_POWER_ON; }
        if (jhas(body, tk, nt, "sys_fail"))  { cmd.set_dali.sys_fail  = (uint8_t)jint_key(body, tk, nt, "sys_fail", 0);  f |= IC_DALI_F_SYS_FAIL; }
        if (jhas(body, tk, nt, "fade_rate")) { cmd.set_dali.fade_rate = (uint8_t)jint_key(body, tk, nt, "fade_rate", 0); f |= IC_DALI_F_FADE_RATE; }
        if (jhas(body, tk, nt, "fade_time")) { cmd.set_dali.fade_time = (uint8_t)jint_key(body, tk, nt, "fade_time", 0); f |= IC_DALI_F_FADE_TIME; }
        cmd.set_dali.fields = f;
        push_cmd_resp(c, &cmd); return;
    }
    if (strcmp(path, "/api/group") == 0) {
        cmd.type = IC_CMD_SET_GROUP;
        cmd.set_group.slave = (uint8_t)jint_key(body, tk, nt, "slave", 0);
        cmd.set_group.group = (uint8_t)jint_key(body, tk, nt, "group", 0);
        uint16_t f = 0; ic_tri_t on;
        if (jtri(body, tk, nt, "on", &on)) { cmd.set_group.on = on; f |= IC_GRP_F_ON; }
        if (jhas(body, tk, nt, "bri"))     { cmd.set_group.bri = (uint8_t)jint_key(body, tk, nt, "bri", 0); f |= IC_GRP_F_BRI; }
        int mi = jfind(body, tk, nt, "members");
        if (mi >= 0) { cmd.set_group.members = parse_members_at(body, tk, nt, mi); f |= IC_GRP_F_MEMBERS; }
        cmd.set_group.fields = f;
        push_cmd_resp(c, &cmd); return;
    }
    if (strcmp(path, "/api/cfg") == 0) {
        cmd.type = IC_CMD_SET_CFG; uint16_t f = 0;
        if (jhas(body, tk, nt, "long_press_ms"))      { cmd.set_cfg.long_press_ms      = (uint16_t)jint_key(body, tk, nt, "long_press_ms", 0);      f |= IC_CFG_F_LONG_PRESS; }
        if (jhas(body, tk, nt, "dim_repeat_ms"))      { cmd.set_cfg.dim_repeat_ms      = (uint16_t)jint_key(body, tk, nt, "dim_repeat_ms", 0);      f |= IC_CFG_F_DIM_REPEAT; }
        if (jhas(body, tk, nt, "reversal_window_ms")) { cmd.set_cfg.reversal_window_ms = (uint16_t)jint_key(body, tk, nt, "reversal_window_ms", 0); f |= IC_CFG_F_REVERSAL; }
        if (jhas(body, tk, nt, "gw_diag_interval_ms")) { cmd.set_cfg.gw_diag_interval_ms = (uint32_t)jint_key(body, tk, nt, "gw_diag_interval_ms", 0); f |= IC_CFG_F_GW_DIAG; }
        if (jhas(body, tk, nt, "dali_sweep_interval_ms")) { cmd.set_cfg.dali_sweep_interval_ms = (uint32_t)jint_key(body, tk, nt, "dali_sweep_interval_ms", 0); f |= IC_CFG_F_DALI_SWEEP; }
        if (jhas(body, tk, nt, "baud"))               { cmd.set_cfg.baud               = (uint32_t)jint_key(body, tk, nt, "baud", 0);               f |= IC_CFG_F_BAUD; }
        if (jhas(body, tk, nt, "de_mode"))            { cmd.set_cfg.de_mode            = (uint8_t)jint_key(body, tk, nt, "de_mode", 0);             f |= IC_CFG_F_DE_MODE; }
        if (jhas(body, tk, nt, "relay_byteswap"))     { cmd.set_cfg.relay_byteswap     = (uint8_t)jint_key(body, tk, nt, "relay_byteswap", 0);      f |= IC_CFG_F_RELAY_BYTESWAP; }
        cmd.set_cfg.fields = f;
        push_cmd_resp(c, &cmd); return;
    }
    if (strcmp(path, "/api/net") == 0) {
        cmd.type = IC_CMD_SET_NETCFG; uint16_t f = 0;
        if (jhas(body, tk, nt, "dhcp")) { cmd.set_netcfg.dhcp = jbool_key(body, tk, nt, "dhcp", true); f |= IC_NET_F_DHCP; }
        if (jip(body, tk, nt, "ip",   cmd.set_netcfg.ip))   f |= IC_NET_F_IP;
        if (jip(body, tk, nt, "mask", cmd.set_netcfg.mask)) f |= IC_NET_F_MASK;
        if (jip(body, tk, nt, "gw",   cmd.set_netcfg.gw))   f |= IC_NET_F_GW;
        if (jip(body, tk, nt, "dns",  cmd.set_netcfg.dns))  f |= IC_NET_F_DNS;
        if (jhas(body, tk, nt, "hostname")) { jstr_key(body, tk, nt, "hostname", cmd.set_netcfg.hostname, sizeof cmd.set_netcfg.hostname); f |= IC_NET_F_HOSTNAME; }
        cmd.set_netcfg.fields = f;
        push_cmd_resp(c, &cmd); return;
    }
    if (strcmp(path, "/api/mqtt") == 0) {
        cmd.type = IC_CMD_SET_MQTTCFG; uint16_t f = 0;
        if (jhas(body, tk, nt, "host"))   { jstr_key(body, tk, nt, "host",   cmd.set_mqttcfg.host,   sizeof cmd.set_mqttcfg.host);   f |= IC_MQTT_F_HOST; }
        if (jhas(body, tk, nt, "port"))   { cmd.set_mqttcfg.port = (uint16_t)jint_key(body, tk, nt, "port", 1883);                   f |= IC_MQTT_F_PORT; }
        if (jhas(body, tk, nt, "user"))   { jstr_key(body, tk, nt, "user",   cmd.set_mqttcfg.user,   sizeof cmd.set_mqttcfg.user);   f |= IC_MQTT_F_USER; }
        if (jhas(body, tk, nt, "pass"))   { jstr_key(body, tk, nt, "pass",   cmd.set_mqttcfg.pass,   sizeof cmd.set_mqttcfg.pass);   f |= IC_MQTT_F_PASS; }
        if (jhas(body, tk, nt, "prefix")) { jstr_key(body, tk, nt, "prefix", cmd.set_mqttcfg.prefix, sizeof cmd.set_mqttcfg.prefix); f |= IC_MQTT_F_PREFIX; }
        cmd.set_mqttcfg.fields = f;
        push_cmd_resp(c, &cmd); return;
    }
    if (strcmp(path, "/api/auth") == 0) {
        char u[32] = {0}, p[48] = {0};
        jstr_key(body, tk, nt, "user", u, sizeof u);
        jstr_key(body, tk, nt, "pass", p, sizeof p);
        if (!u[0] || !p[0]) {
            resp_text(c, "400 Bad Request", "application/json",
                      "{\"error\":\"user and pass required\"}");
            return;
        }
        cmd.type = IC_CMD_SET_AUTH;
        snprintf(cmd.set_auth.user, sizeof cmd.set_auth.user, "%s", u);
        snprintf(cmd.set_auth.pass, sizeof cmd.set_auth.pass, "%s", p);
        /* Apply to the live credentials now so the change is effective for the
         * next request; core 0 persists it for the next boot. */
        snprintf(s_user, sizeof s_user, "%s", u);
        snprintf(s_pass, sizeof s_pass, "%s", p);
        push_cmd_resp(c, &cmd); return;
    }
    if (strcmp(path, "/api/mapping") == 0) {
        int fx = jint_key(body, tk, nt, "fixture", -1);
        int bt = jint_key(body, tk, nt, "button", -1);
        if (fx < 0 || bt < 0) {
            resp_text(c, "400 Bad Request", "application/json",
                      "{\"error\":\"fixture and button required\"}");
            return;
        }
        mapping_target_t tg[MAX_TARGETS_PER_BTN];
        int ti = jfind(body, tk, nt, "targets");
        uint8_t tc = ti >= 0 ? parse_targets_at(body, tk, nt, ti, tg, MAX_TARGETS_PER_BTN) : 0;
        if (tc == 0) {
            cmd.type = IC_CMD_DEL_MAPPING;
            cmd.del_mapping.fixture = (uint8_t)fx; cmd.del_mapping.button = (uint8_t)bt;
        } else {
            cmd.type = IC_CMD_SET_MAPPING;
            cmd.set_mapping.fixture = (uint8_t)fx; cmd.set_mapping.button = (uint8_t)bt;
            cmd.set_mapping.count = tc;
            memcpy(cmd.set_mapping.targets, tg, (size_t)tc * sizeof(mapping_target_t));
        }
        push_cmd_resp(c, &cmd); return;
    }
    if (strcmp(path, "/api/provision") == 0) {
        uint8_t slave = (uint8_t)jint_key(body, tk, nt, "slave", 0);
        if (jbool_key(body, tk, nt, "abort", false)) {
            cmd.type = IC_CMD_PROVISION_ABORT; cmd.provision_abort.slave = slave;
            push_cmd_resp(c, &cmd); return;
        }
        char mode[12]; jstr_key(body, tk, nt, "mode", mode, sizeof mode);
        cmd.type = IC_CMD_PROVISION_START;
        cmd.provision_start.slave = slave;
        cmd.provision_start.mode  = strcmp(mode, "new") == 0     ? IC_PROV_UNADDRESSED
                                  : strcmp(mode, "address") == 0 ? IC_PROV_ADDRESS
                                                                 : IC_PROV_ALL;
        cmd.provision_start.target = (uint8_t)jint_key(body, tk, nt, "target", 0);
        push_cmd_resp(c, &cmd); return;
    }
    resp_text(c, "404 Not Found", "application/json", "{\"error\":\"unknown endpoint\"}");
}

/* ── routing ─────────────────────────────────────────────────────────────────*/

static void route(conn_t *c) {
    /* buf is NUL-terminated request text; body (if any) follows the headers. */
    char method[8] = {0}, path[160] = {0};
    const char *sp1 = strchr(c->buf, ' ');
    const char *sp2 = sp1 ? strchr(sp1 + 1, ' ') : NULL;
    if (!sp1 || !sp2) { resp_text(c, "400 Bad Request", "text/plain", "bad request"); return; }
    size_t ml = (size_t)(sp1 - c->buf); if (ml >= sizeof method) ml = sizeof method - 1;
    memcpy(method, c->buf, ml);
    size_t pl = (size_t)(sp2 - sp1 - 1); if (pl >= sizeof path) pl = sizeof path - 1;
    memcpy(path, sp1 + 1, pl);
    /* strip query string for routing */
    char *q = strchr(path, '?'); if (q) *q = '\0';

    if (!check_auth(c->buf)) { resp_401(c); return; }

    bool is_get  = strcmp(method, "GET") == 0;
    bool is_post = strcmp(method, "POST") == 0;

    if (is_get && strcmp(path, "/api/state") == 0) { resp_state(c); return; }
    if (is_get && strcmp(path, "/api/log") == 0)   { resp_log(c);   return; }
    if (is_get && strcmp(path, "/api/ota") == 0)   { resp_ota(c);   return; }

    if (is_post && strncmp(path, "/api/", 5) == 0) {
        int blen = c->content_len;
        if (blen > 1024) {
            resp_text(c, "413 Payload Too Large", "application/json",
                      "{\"error\":\"body too large\"}");
            return;
        }
        /* Copy the body out before resp_* overwrites buf with the response. */
        char body[1025];
        memcpy(body, c->buf + c->body_start, (size_t)blen);
        body[blen] = '\0';
        handle_post(c, path, body, blen);
        return;
    }

    if (is_get) {
        const web_asset_t *a = web_assets_lookup(path);
        if (a) { resp_mem(c, "200 OK", NULL, a->mime, a->data, a->len, a->gzip); return; }
        resp_text(c, "404 Not Found", "text/plain", "not found");
        return;
    }

    resp_text(c, "405 Method Not Allowed", "text/plain", "method not allowed");
}

/* ── per-connection pump ─────────────────────────────────────────────────────*/

static void do_recv(conn_t *c) {
    uint16_t rsr = getSn_RX_RSR(c->sn);
    if (rsr > 0) {
        uint16_t room = (uint16_t)(sizeof c->buf - 1 - c->rx_len);
        if (room == 0) {                         /* request too big for us */
            resp_text(c, "413 Payload Too Large", "text/plain", "too large");
            return;
        }
        uint16_t want = rsr < room ? rsr : room;
        int32_t got = recv(c->sn, (uint8_t *)c->buf + c->rx_len, want);
        if (got == SOCK_BUSY) return;
        if (got < 0) { arm_listen(c); return; }
        c->rx_len += (uint16_t)got;
        c->buf[c->rx_len] = '\0';
        c->t_ms = now_ms();
    }

    if (!c->hdr_complete) {
        const char *end = strstr(c->buf, "\r\n\r\n");
        if (end) {
            c->hdr_complete = true;
            c->body_start = (uint16_t)(end + 4 - c->buf);
            const char *cl = find_header(c->buf, "Content-Length");
            c->content_len = cl ? (uint16_t)atoi(cl) : 0;
        }
    }
    if (c->hdr_complete) {
        uint16_t body_have = c->rx_len - c->body_start;
        if (body_have >= c->content_len) route(c);  /* full request in hand */
    }
}

/* Send as much of the currently-staged buffer as TX allows. Returns true when
 * the staged region (out_sent..out_len) is fully flushed. */
static bool flush_staged(conn_t *c) {
    while (c->out_sent < c->out_len) {
        uint16_t freesz = getSn_TX_FSR(c->sn);
        if (freesz == 0) return false;
        uint16_t n = (uint16_t)(c->out_len - c->out_sent);
        if (n > freesz) n = freesz;
        int32_t s = send(c->sn, (uint8_t *)c->buf + c->out_sent, n);
        if (s == SOCK_BUSY) return false;
        if (s < 0) { arm_listen(c); return false; }
        c->out_sent += (uint16_t)s;
        c->t_ms = now_ms();
        break;   /* one send per tick: stay fair to MQTT */
    }
    return c->out_sent >= c->out_len;
}

static void do_send(conn_t *c) {
    if (c->phase == PH_HDR) {
        if (!flush_staged(c)) return;
        c->phase = PH_BODY;
    }
    if (c->resp == RESP_MEM) {
        while (c->mem_sent < c->mem_len) {
            uint16_t freesz = getSn_TX_FSR(c->sn);
            if (freesz == 0) return;
            uint32_t rem = c->mem_len - c->mem_sent;
            uint16_t n = (uint16_t)(rem < freesz ? rem : freesz);
            int32_t s = send(c->sn, (uint8_t *)(uintptr_t)(c->mem + c->mem_sent), n);
            if (s == SOCK_BUSY) return;
            if (s < 0) { arm_listen(c); return; }
            c->mem_sent += (uint32_t)s;
            c->t_ms = now_ms();
            break;   /* one send per tick */
        }
        if (c->mem_sent >= c->mem_len) { c->st = CONN_DRAIN; c->t_ms = now_ms(); }
        return;
    }
    /* RESP_STATE: stream the cursor renderer one chunk at a time. */
    if (c->out_sent >= c->out_len) {
        if (c->state_done) { c->st = CONN_DRAIN; c->t_ms = now_ms(); return; }
        bool done = false;
        size_t n = web_render_next(&c->cur, c->buf, sizeof c->buf, &done);
        c->out_len = (uint16_t)n; c->out_sent = 0; c->state_done = done;
        if (n == 0 && done) { c->st = CONN_DRAIN; c->t_ms = now_ms(); return; }
    }
    flush_staged(c);
}

static void pump(conn_t *c) {
    uint8_t sr = getSn_SR(c->sn);

    /* Reap remote close at any point. */
    if (sr == SOCK_CLOSE_WAIT || sr == SOCK_CLOSED) {
        if (c->st != CONN_LISTEN || sr == SOCK_CLOSED) { arm_listen(c); return; }
    }

    switch (c->st) {
    case CONN_LISTEN:
        if (sr == SOCK_ESTABLISHED) {
            c->st = CONN_RECV; c->t_ms = now_ms();
            c->rx_len = 0; c->hdr_complete = false; c->buf[0] = '\0';
        } else if (sr == SOCK_CLOSED) {
            arm_listen(c);
        }
        break;
    case CONN_RECV:
        if (sr != SOCK_ESTABLISHED) { arm_listen(c); break; }
        do_recv(c);
        if (c->st == CONN_RECV && now_ms() - c->t_ms > RECV_TIMEOUT_MS) arm_listen(c);
        break;
    case CONN_SEND:
        if (sr != SOCK_ESTABLISHED) { arm_listen(c); break; }
        do_send(c);
        if (c->st == CONN_SEND && now_ms() - c->t_ms > SEND_TIMEOUT_MS) arm_listen(c);
        break;
    case CONN_DRAIN:
        /* Wait for the TX buffer to flush (peer-acked) before closing, so the
         * response isn't truncated by an RST; bounded by a timeout. */
        if (getSn_TX_FSR(c->sn) >= HTTP_SOCK_BUFSZ - 4 ||
            now_ms() - c->t_ms > DRAIN_TIMEOUT_MS) {
            disconnect(c->sn);
            arm_listen(c);
        }
        break;
    }
}

void http_server_tick(void) {
    /* Acquire/relinquish the opportunistic socket 0 per the netif DHCP arbiter.
     * netif_tick() runs before us each loop, so the flag is current; when it
     * drops, netif has (or is about to) reclaim socket 0 for a DHCP renewal, so
     * we simply stop pumping it — netif owns closing/reopening it. */
    bool en = netif_web_sock0_enabled();
    if (en && !s_sock0_active) {
        arm_listen(&s_conn[N_BASE]);     /* socket 0 */
        s_sock0_active = true;
    } else if (!en && s_sock0_active) {
        s_sock0_active = false;
    }

    for (size_t i = 0; i < N_BASE; i++) pump(&s_conn[i]);
    if (s_sock0_active) pump(&s_conn[N_BASE]);

    /* Self-heal: detect a wedged web-socket layer and ask netif for a chip reset.
     * Only count time while netif is ready (don't fight its own bring-up/recovery),
     * and treat any base socket in LISTEN or ESTABLISHED as healthy. No LOG here —
     * hot path; netif logs the recovery. */
    uint32_t now = now_ms();
    if (!netif_ready()) {
        s_last_listen_ms = now;
    } else {
        bool healthy = false;
        for (size_t i = 0; i < N_BASE; i++) {
            uint8_t sr = getSn_SR(s_conn[i].sn);
            if (sr == SOCK_LISTEN || sr == SOCK_ESTABLISHED) { healthy = true; break; }
        }
        if (healthy) {
            s_last_listen_ms = now;
        } else if (now - s_last_listen_ms > WEB_WEDGE_TIMEOUT_MS) {
            netif_request_recovery();
            s_last_listen_ms = now;   /* give the reset time before re-checking */
        }
    }
}
