#pragma once
/*
 * http_client.h — cooperative non-blocking HTTP/1.1 GET client (core 1).
 *
 * Pull-based OTA fetches manifest/firmware files from a plain HTTP static
 * file server. http_parse_head() is pure (no sockets, no globals) and fully
 * host-testable against canned byte buffers, tolerating the header block
 * arriving fragmented across multiple reads; httpc_get_start()/httpc_poll()
 * are a thin non-blocking wrapper around it, matching the socket-state-
 * machine cadence of mqtt_app.c/http_server.c/dns_resolve.c: one bounded
 * send/recv per tick, socket number supplied by the caller (this module
 * never grabs a socket itself), no LOG_* in httpc_poll()'s hot path.
 *
 * Only Content-Length-framed responses are supported (the OTA server serves
 * static files). A chunked (or framing-less) response is a clean HTTPC_FAILED
 * rather than a best-effort/mis-parse.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ── pure, host-testable ─────────────────────────────────────────────────── */

/* Scan buf[0..len) for a complete HTTP/1.1 response header block (terminated
 * by CRLFCRLF). Returns:
 *    1  the block is complete: *status holds the numeric status code (e.g.
 *       200), *content_len holds the parsed Content-Length value (-1 if that
 *       header is absent), *body_off holds the index of the first body byte
 *       in buf.
 *    0  the header block is not yet complete — call again once more bytes
 *       have arrived (safe to call repeatedly against a growing buffer; a
 *       prior 0 result does not consume or invalidate buf).
 *   -1  the status line is malformed (does not start with
 *       "HTTP/<version> <3-digit-code>").
 * Header name matching is case-insensitive. buf need not be NUL-terminated. */
int http_parse_head(const char *buf, int len, int *status, long *content_len, int *body_off);

/* ── cooperative state machine ───────────────────────────────────────────── */

typedef void (*httpc_body_cb)(const uint8_t *data, uint32_t len, void *ctx);
typedef enum { HTTPC_PENDING, HTTPC_DONE, HTTPC_FAILED } httpc_status_t;

/* Start (or restart) a GET job against ip:port using caller-owned TCP socket
 * `sock` (the caller allocates/borrows it — this module never grabs a socket
 * on its own; see the netif socket-0 arbiter / ping.c / dns_resolve.c for the
 * same convention). `on_body` is invoked from httpc_poll() with successive
 * bounded chunks of the response body (zero or more calls before the job
 * reaches a terminal state) — never with header bytes. Starting a new job
 * while one is in flight abandons the old one (closes its socket) and begins
 * the new one. */
void httpc_get_start(const uint8_t ip[4], uint16_t port, const char *host, const char *path,
                     uint8_t sock, httpc_body_cb on_body, void *ctx);

/* Advance the client one step. Never blocks: connects/sends/receives across
 * ticks, doing at most one bounded (<=~1.5 KB) send or recv action per call
 * so a large transfer can't starve mqtt_app_tick()/http_server_tick() or trip
 * the core-1 watchdog. Completes once Content-Length bytes have been
 * delivered to on_body; a response with no Content-Length (or with
 * Transfer-Encoding: chunked) fails cleanly (HTTPC_FAILED) instead of being
 * mis-parsed. Closes the socket on any terminal state (HTTPC_DONE or
 * HTTPC_FAILED). Safe (and a no-op, returning HTTPC_FAILED) to call with no
 * job started. */
httpc_status_t httpc_poll(void);

/* Parsed response status code (e.g. 200/404); 0 before a header is parsed. */
int httpc_status_code(void);

/* Parsed Content-Length in bytes; 0 if none was seen. */
uint32_t httpc_content_length(void);
