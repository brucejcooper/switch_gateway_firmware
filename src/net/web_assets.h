#pragma once
/*
 * web_assets.h — static web UI assets compiled into flash.
 *
 * The Vite single-file build is gzipped and emitted as C arrays by
 * cmake/gen_assets.cmake (see Phase 5) into build/generated/web_assets.c, which
 * provides the table below. A hand-written placeholder serves a stub page until
 * that pipeline is wired up. Assets are served straight from flash (XIP) — no
 * RAM copy, no filesystem — with Content-Encoding: gzip when `gzip` is set.
 */

#include <stddef.h>
#include <stdbool.h>

typedef struct {
    const char          *path;   /* request path, e.g. "/" or "/index.html" */
    const char          *mime;   /* Content-Type */
    const unsigned char *data;   /* asset bytes (possibly gzip-compressed) */
    unsigned             len;
    bool                 gzip;   /* data is gzip-compressed */
} web_asset_t;

/* Exact-path lookup; "/" is aliased to "/index.html". Returns NULL if absent. */
const web_asset_t *web_assets_lookup(const char *path);
