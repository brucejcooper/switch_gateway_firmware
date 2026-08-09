#pragma once
/*
 * fwupdate.h — core-1 pull-based-OTA manifest checker.
 *
 * Periodically GETs a small JSON manifest (FWUPDATE_MANIFEST_URL) describing
 * the latest published firmware — mirroring the brewery ESP32 fleet's
 * fwupdate.h/MQTT-'update'-entity pattern, adapted to this stack (no
 * esp_mqtt_client; ha_bridge, a separate task, will publish these accessors
 * to HA and route the Install button to fwupdate_install()). Manifest shape
 * (extra keys tolerated/ignored):
 *   { "version": "<latest>", "url": "http://ota.8bitcloud.com/light_switch.bin",
 *     "title": "...", "release_url": "...", "notes": "..." }
 *
 * Cooperative, like every net/ module: fwupdate_poll() does at most one
 * bounded step per call (a socket-acquire attempt, a dnsr_poll()/httpc_poll()
 * step) and never blocks; no LOG_* in the byte-path. It borrows the same
 * revocable DHCP-socket-0 loan ota.c/ping.c/http_server.c use, and only runs
 * a check when ota_state() != OTA_RUNNING — http_client has a single static
 * instance, so a manifest GET and an image download must never overlap.
 *
 * This module never touches MQTT or intercore itself: it just fetches +
 * parses the manifest and exposes the result via accessors, and
 * fwupdate_install() hands the URL to ota_trigger(). Publishing state /
 * wiring the HA 'update' entity is ha_bridge's job.
 */

#include <stdbool.h>

/* How often fwupdate_poll() re-checks the manifest on its own, in seconds.
 * fwupdate_announce() (call on every MQTT (re)connect) forces an immediate
 * check regardless of this timer, matching the brewery behaviour. */
#ifndef FWCHECK_INTERVAL_S
#define FWCHECK_INTERVAL_S 3600u
#endif

/* Reset all fwupdate state (installed version, no known latest yet) and arm
 * an immediate first check. Call once at core-1 startup, alongside
 * ota_init()/ha_bridge_init(), before the first fwupdate_poll(). */
void fwupdate_init(void);

/* Advance the manifest-check state machine one step. Call every core-1 tick,
 * alongside ota_poll(). Never blocks. A no-op whenever ota_state() ==
 * OTA_RUNNING (image download owns http_client + socket 0) or the network
 * isn't up yet — the due check is simply deferred to the next eligible tick. */
void fwupdate_poll(void);

/* Force an immediate manifest check on the next eligible fwupdate_poll(),
 * bypassing FWCHECK_INTERVAL_S. Call on every MQTT (re)connect. Cheap/
 * idempotent to call repeatedly — it just sets a flag. */
void fwupdate_announce(void);

/* Begin an OTA of the last-known latest firmware (ota_trigger(latest_url)).
 * A no-op if no manifest has been fetched successfully yet (fwupdate_latest_url()
 * is empty) — check fwupdate_available() first. Does not itself check
 * ota_state(); ota_trigger() already no-ops while OTA_RUNNING. */
void fwupdate_install(void);

/* Currently-installed version (FW_VERSION), stable for the process lifetime. */
const char *fwupdate_installed(void);

/* Last-known latest version from the manifest; empty ("") until the first
 * successful check. */
const char *fwupdate_latest(void);

/* Last-known firmware image URL from the manifest; empty until the first
 * successful check. This is what fwupdate_install() passes to ota_trigger(). */
const char *fwupdate_latest_url(void);

/* Optional manifest fields, empty ("") if absent from the last successful
 * response or before the first successful check. */
const char *fwupdate_title(void);
const char *fwupdate_release_url(void);

/* Why the last manifest check aborted ("" if the last one succeeded). An
 * aborted check leaves installed/latest looking current, so this is the only
 * way to tell "up to date" apart from "we never managed to ask". Surfaced in
 * GET /api/ota as `last_error`. */
const char *fwupdate_last_error(void);
const char *fwupdate_notes(void);

/* True once a manifest has been fetched and its version differs (any
 * difference, not ordering — matches the brewery fleet's strcmp() convention)
 * from fwupdate_installed(). False before the first successful check. */
bool fwupdate_available(void);

/* Render the OTA/update status as a JSON object into buf: installed, latest,
 * available, url, title, release_url, notes, state (idle/running/success/fail),
 * pct. Returns bytes written. Backs GET /api/ota and the web Firmware tab. */
int fwupdate_status_json(char *buf, int cap);

/* ── pure, host-testable ─────────────────────────────────────────────────── */

/* Dumb "key":"value" string extractor (mirrors the brewery fleet's
 * json_get()): scans buf for the literal quoted key `"key"` immediately
 * followed (after optional whitespace) by ':', optional whitespace, then a
 * quoted string value, and copies that value (NUL-terminated) into
 * out[0..cap). Not a real JSON parser — no escape handling, no nesting
 * awareness — but the anchor on the quoted key + following colon means a
 * plain substring occurring inside another key's *value* (e.g. the word
 * "version" appearing inside a url's path) is never mistaken for the key: it
 * is never both quote-delimited AND colon-followed at that position. Returns
 * false (out left untouched) if the key is absent, its value is empty, its
 * value isn't a quoted string, or the value would not fit in cap. Keys may
 * appear in any order and elsewhere in buf need not be well-formed. */
bool fw_json_get(const char *buf, const char *key, char *out, int cap);
