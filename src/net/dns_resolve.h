#pragma once
/*
 * dns_resolve.h — cooperative UDP DNS A-record resolver (core 1).
 *
 * mqtt_app.h notes hostname resolution as a follow-up ("ioLibrary DNS_run
 * blocks, unusable in the cooperative core-1 loop without a cooperative
 * resolver"). This is that resolver: build/parse are pure functions (no
 * sockets, no globals, fully host-testable against canned byte buffers), and
 * a thin state machine drives a caller-owned UDP socket across ticks.
 *
 * Like every net/ module, this one does NOT allocate or borrow a socket
 * itself — the caller (e.g. the netif socket-0 arbiter, same as ping.c) picks
 * an idle socket number and passes it in. Never blocks; no LOG_* in
 * dnsr_poll() (core-1 hot-path rule, see CLAUDE.md).
 */

#include <stdint.h>
#include <stdbool.h>

/* ── pure, host-testable functions ──────────────────────────────────────── */

/* Build a standard DNS query: one question, QNAME = label-encoded
 * `hostname`, QTYPE=A(1), QCLASS=IN(1), RD (recursion desired) set. Writes
 * into buf (capacity `cap` bytes) and returns the total byte length, or -1 if
 * `hostname` is empty, has an empty/over-length (>63) label, or the encoded
 * query would not fit in `cap`. */
int dns_build_query(uint8_t *buf, int cap, uint16_t id, const char *hostname);

/* Parse a DNS response `buf` (`len` bytes): the transaction id must equal
 * `expect_id`, the QR bit must mark it a response, RCODE must be 0, and it
 * must carry at least one A(1)/IN(1) answer resource record (name-compression
 * pointers, RFC 1035 4.1.4, are followed when walking record names). On
 * success writes the first A record's 4-byte address to out_ip and returns
 * true. Returns false on any malformation, truncation, id mismatch,
 * non-response, non-zero RCODE, or absence of an A record. */
bool dns_parse_response(const uint8_t *buf, int len, uint16_t expect_id, uint8_t out_ip[4]);

/* ── cooperative state machine ──────────────────────────────────────────── */

typedef enum { DNSR_PENDING, DNSR_OK, DNSR_FAILED } dnsr_status_t;

/* Start (or restart) a resolve job: `hostname` against `dns_server`, using
 * caller-owned UDP socket `sock` (the caller allocates/borrows it — this
 * module never grabs a socket on its own). Call dnsr_poll() every core-1 tick
 * afterward until it returns a terminal status. Starting a new job while one
 * is in flight abandons the old one (closes its socket) and begins the new. */
void dnsr_start(const uint8_t dns_server[4], const char *hostname, uint8_t sock);

/* Advance the resolver one step. Never blocks: opens/sends/receives the
 * socket across ticks, retries once on a ~1.5 s per-attempt timeout (~3 s
 * total), and releases (closes) the socket on any terminal outcome. Returns
 * DNSR_PENDING while still in flight; DNSR_OK once `out_ip` has been filled
 * in with the resolved address; DNSR_FAILED on timeout/error/malformed reply.
 * Safe (and a no-op, returning DNSR_FAILED) to call with no job started. */
dnsr_status_t dnsr_poll(uint8_t out_ip[4]);
