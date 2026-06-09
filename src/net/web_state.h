#pragma once
/*
 * web_state.h — core-1 read model for the embedded config web UI.
 *
 * Core 1 holds no domain state of its own; ha_bridge translates events straight
 * to MQTT and forgets them. The web UI, however, needs to *show* current state
 * while polling, and must stay fresh even when MQTT is down (its whole reason to
 * exist is commissioning a device before it reaches Home Assistant). So this
 * module observes the same core0->core1 event stream and keeps a small static
 * snapshot of relays, ballasts, groups, gateways, mappings, provisioning and the
 * controller config, which GET /api/state renders.
 *
 * Fed from ha_bridge's event drain (see ha_bridge_tick) so a single pop updates
 * both the MQTT bridge and this cache. Pure data — no sockets, no flash.
 */

#include "intercore.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

void web_state_init(void);

/* Fold one inter-core event into the cache. Cheap; safe to call for every event
 * regardless of MQTT state. */
void web_state_on_event(const ic_event_t *e);

/* Monotonic change counter — bumps on every mutation. The HTTP layer uses it as
 * a weak ETag so an unchanged poll can answer 304 Not Modified. */
uint32_t web_state_seq(void);

/* ── Cursor-based JSON renderer for GET /api/state ──────────────────────────
 * Emits whole JSON items per call so the HTTP layer can stream the document in
 * bounded TX chunks without splitting a value across a W5100S socket boundary.
 * The caller's buffer must be at least WEB_STATE_MIN_CHUNK bytes. */
#define WEB_STATE_MIN_CHUNK 1024u

typedef struct {
    uint8_t  sec;       /* section index (internal) */
    uint8_t  si;        /* DALI slave index (within the "dali" object) */
    uint8_t  sub;       /* sub-phase within a slave (open/ballasts/groups/close) */
    uint16_t i;         /* item index within the current array */
    bool     started;   /* have we emitted an item in the current array? */
    uint32_t seq;       /* snapshot seq this render began at (for diagnostics) */
} web_render_cursor_t;

void web_render_begin(web_render_cursor_t *cur);

/* Append the next whole units to buf (up to cap). Returns bytes written and sets
 * *done true once the full document has been emitted. cap must be >=
 * WEB_STATE_MIN_CHUNK. Returns 0 with *done=false only if cap is too small. */
size_t web_render_next(web_render_cursor_t *cur, char *buf, size_t cap, bool *done);
