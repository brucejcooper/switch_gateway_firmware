#pragma once
/*
 * ota.h — core-1 pull-based OTA orchestrator.
 *
 * Downloads a signed firmware image over plain HTTP (dns_resolve.h +
 * http_client.h) and hands the bytes to core 0 across the g_ota_buf
 * cross-core handoff (ota_stage.h) via the IC_CMD_OTA_* / IC_EVT_OTA_*
 * intercore protocol (intercore.h) — core 0 is the sole flash writer, this
 * module is the network-side state machine + the cross-core glue and never
 * touches flash itself. See CLAUDE.md / the task brief for the full
 * cross-core contract.
 *
 * Cooperative, like every net/ module: ota_poll() does at most one bounded
 * step per call (a socket-acquire attempt, a dnsr_poll()/httpc_poll() step,
 * or a single intercore_push_cmd()) and never blocks; no LOG_* in the
 * byte-path (ota_body_cb, the OS_DOWNLOADING case). It borrows the same
 * revocable DHCP-socket-0 loan ping.c/http_server.c use (OTA_SOCK, a single
 * #define below) for the whole session — resolve, then the HTTP GET.
 *
 * One job at a time: ota_trigger() while OTA_RUNNING is a no-op, matching
 * ping_submit()'s "busy" convention rather than aborting an in-flight
 * download.
 */

#include <stdint.h>
#include <stdbool.h>
#include "intercore.h"   /* ic_event_t — for ota_on_event() below */

typedef enum { OTA_IDLE, OTA_RUNNING, OTA_SUCCESS, OTA_FAIL } ota_state_t;

/* Reset all OTA state to idle. Call once at core-1 startup (alongside
 * netif_init()/mqtt_app_init()/ha_bridge_init()), before the first
 * ota_poll(). */
void ota_init(void);

/* Begin a firmware download from an http:// URL. A no-op while a run is
 * already OTA_RUNNING, OR while a just-succeeded install is still waiting for
 * its post-install reboot to actually land — call ota_state() first if you
 * need to know whether the request will be honoured. url is parsed
 * (ota_parse_url()) and copied into internal storage immediately, so the
 * caller's buffer need not outlive this call. A URL that fails to parse
 * leaves state() == OTA_FAIL right away (no socket touched). */
void ota_trigger(const char *url);

/* Advance the OTA state machine one step. Call every core-1 tick, alongside
 * netif_tick()/mqtt_app_tick()/ha_bridge_tick(). Never blocks. */
void ota_poll(void);

/* Tell core 0 the currently-running image is healthy: pushes
 * IC_CMD_OTA_CONFIRM, which clears the rollback arm (boot_control's
 * BC_F_PENDING -> BC_F_CONFIRMED, boot_attempts reset). Best-effort (a
 * dropped push just means the confirm is missing until the next call) — call
 * once after boot, once the caller is satisfied the new image is good. */
void ota_mark_valid(void);

/* Feed one core0->core1 event to the OTA state machine. Call from
 * ha_bridge's event drain for IC_EVT_OTA_ACK and IC_EVT_OTA_STATUS — the two
 * event types this module owns (ha_bridge routes them here instead of
 * web_state_on_event/ha_bridge_publish_event, the same way it special-cases
 * IC_EVT_PING_REQ). A no-op for any other event type or with no OTA run
 * active, so it is harmless to call unconditionally on every popped event
 * too. */
void ota_on_event(const ic_event_t *e);

/* Current state. OTA_IDLE until the first ota_trigger(); OTA_RUNNING for the
 * whole resolve/download/flash/verify pipeline; OTA_SUCCESS/OTA_FAIL are
 * terminal until the next ota_trigger() (a successful run also pushes
 * IC_CMD_REBOOT — retried from ota_poll() until it's actually delivered, see
 * ota.c's s_reboot_pending — so in practice the device reboots before
 * anything polls OTA_SUCCESS for long; ota_trigger() itself refuses a new
 * job for as long as that reboot is still pending). */
ota_state_t ota_state(void);

/* Why the current/last run ended in OTA_FAIL — a short, human-readable
 * reason ("http status 404", "bad signature", "ack timeout", ...), or "" if
 * the run hasn't failed. Mirrors fwupdate_last_error(); valid until the next
 * ota_trigger(). */
const char *ota_last_error(void);

/* Download progress 0..100, based on bytes handed to core 0 so far vs.
 * httpc_content_length(). 0 outside OTA_RUNNING or before the response
 * Content-Length header has been parsed. */
int ota_progress_pct(void);

/* ── pure, host-testable ─────────────────────────────────────────────────── */

/* Parse "http://host[:port]/path" into host (NUL-terminated into
 * host[host_cap]), port (uint16_t; defaults to 80 when absent), and path
 * (NUL-terminated into path[path_cap]; defaults to "/" when the URL has none,
 * e.g. "http://host"). Returns false — leaving host, port and path all
 * untouched — for a non-"http" scheme, a missing/empty host, a malformed or
 * out-of-range port, or any component that would not fit its output
 * capacity. */
bool ota_parse_url(const char *url, char *host, int host_cap, uint16_t *port, char *path, int path_cap);
