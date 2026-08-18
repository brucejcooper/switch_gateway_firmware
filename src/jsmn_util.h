#pragma once
/*
 * jsmn_util.h — the small helper layer over jsmn tokens (jtok_eq/jfind/jint/
 * string extraction), written once instead of three times.
 *
 * WHY THIS EXISTS. config.c, http_server.c and ha_bridge.c each parse jsmn
 * tokens for a flat JSON object and each grew their own near-identical
 * key-lookup + value-extraction helpers (tok_eq/find_key/get_str in config.c;
 * jeq/jfind/jstr_key in http_server.c; jtok_eq/jfind/jint in ha_bridge.c).
 * That triplication already caused a real bug, not just a maintenance
 * annoyance: http_server.c's jstr_key() clamped an over-long string to its
 * destination's cap instead of rejecting it — commit e875365, found on the
 * bench as a 32-char hostname POSTed to /api/net answering {"ok":true} and
 * silently storing 31 'a's, a value the caller never asked for and the CLI
 * rejects outright. The fix was scoped to jstr_key(); config.c's get_str()
 * had the IDENTICAL clamp-not-reject flaw on the config-load path and needed
 * a second, separate fix (commit 23761ba) because the two functions were
 * never the same function to begin with — fixing one text did not fix the
 * other's copy of the same bug. One shared extractor removes that whole class
 * of "fixed here, forgot to fix there" bug: fix it once, every caller gets it.
 *
 * Header-only static inline, same reasoning as strfit.h (read that header
 * first — this one builds on it): it's a few dozen lines, and per-TU
 * JSMN_STATIC (set at the CMake target level) just works when this header is
 * included wherever jsmn.h already is, with no separate source-list entry to
 * remember in either firmware target.
 *
 * jfind()'s reconciliation (the risky part — read this before changing it):
 * config.c's find_key() returned `i + 1` unconditionally once it matched a
 * key token, even when i+1 == n (no token exists there); http_server.c's and
 * ha_bridge.c's jfind() both guarded that same return with `(i + 1 < n) ? i +
 * 1 : -1`. The bounds check is LOAD-BEARING, not belt-and-braces: this
 * project compiles jsmn WITHOUT JSMN_STRICT, and in lenient mode a dangling
 * key genuinely parses with a positive token count — measured on this
 * machine with this jsmn: `{"key"}` -> n=2, `{"key":}` -> n=2, and
 * `{"a":1,"key"}` -> n=4, the last token being the valueless key each time.
 * (Only a hard-truncated `{"key"` fails with n<0.) So jfind("key") CAN match
 * at i == n-1 on real, positively-parsed input, and the old unguarded
 * find_key() would then have handed its caller index n — a read of an
 * uninitialised jsmntok_t. Reaching it needed a malformed document at a
 * find_key() call site (a hand-damaged /config.json, e.g. a trailing
 * `,"key"}` from a truncated edit), which is exactly the input config_load()
 * exists to survive. Unifying on the guarded version therefore FIXED a
 * latent out-of-bounds read in config-load, it did not merely tidy a
 * redundant check — and the guard must never be "simplified" away on the
 * theory that jsmn wouldn't produce such a token stream. It does. There is a
 * regression case pinning this in test/config_hostname_test.c. Both jfind() and
 * jhas()/jint_key() below walk from index 1 checking every string token
 * (not just object-level keys, no depth tracking) exactly as all three
 * originals did — correct for the flat, non-nested objects every call site
 * here actually parses, but a caller with a nested object whose value happens
 * to equal the key string being searched for would get a false match. None of
 * today's callers hits that case; if a future one does, it needs its own
 * depth-aware walk, not a change here.
 */
#include "jsmn.h"
#include "strfit.h"
#include "json_str.h"  /* json_escape()/json_unescape() — jstr_*_unescaped() below */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>    /* sscanf() — jip() */
#include <stdlib.h>
#include <string.h>

/* Does token `t` hold the string `s` (exact length + content match)? */
static inline bool jtok_eq(const char *js, const jsmntok_t *t, const char *s) {
    int len = t->end - t->start;
    return t->type == JSMN_STRING && (int)strlen(s) == len &&
           strncmp(js + t->start, s, len) == 0;
}

/* Index of `key`'s VALUE token among a flat object's tokens, or -1 if the key
 * is absent. Walks every token from index 1 (index 0 is the enclosing object)
 * looking for a string token equal to `key`, and returns the token right
 * after it — see the file header for why this is safe unguarded against
 * jsmn_parse()'s own precondition, and why it does not need to be
 * depth-aware for any caller today. */
static inline int jfind(const char *js, const jsmntok_t *t, int n, const char *key) {
    for (int i = 1; i < n; i++)
        if (jtok_eq(js, &t[i], key)) return (i + 1 < n) ? i + 1 : -1;
    return -1;
}

/* Is `key` present at all? */
static inline bool jhas(const char *js, const jsmntok_t *t, int n, const char *key) {
    return jfind(js, t, n, key) >= 0;
}

/* Parse a token's text as a decimal integer. No validation beyond strtol's
 * own (a non-numeric token parses as 0) — callers that need to distinguish
 * "absent" from "present but zero" use jint_key()'s separate `def`, and
 * callers that need to reject malformed input outright (config.c's numeric
 * config fields) use their own stricter get_uint(), not this. */
static inline long jint(const char *js, const jsmntok_t *t) {
    char b[16];
    int len = t->end - t->start;
    if (len < 0) return 0;
    if (len >= (int)sizeof b) len = (int)sizeof b - 1;
    memcpy(b, js + t->start, (size_t)len);
    b[len] = '\0';
    return strtol(b, NULL, 10);
}

/* jint(), defaulted when `key` is absent. */
static inline int jint_key(const char *js, const jsmntok_t *t, int n, const char *key, int def) {
    int i = jfind(js, t, n, key);
    return i < 0 ? def : (int)jint(js, &t[i]);
}

/* Truthy test for a bool-ish token ("true"/"1"/anything starting 't'/'T'),
 * defaulted when `key` is absent. Matches the JSON booleans/1-0 numbers the
 * web API and HA both send. */
static inline bool jbool_key(const char *js, const jsmntok_t *t, int n, const char *key, bool def) {
    int i = jfind(js, t, n, key);
    if (i < 0) return def;
    char ch = js[t[i].start];
    return ch == 't' || ch == 'T' || ch == '1';
}

/* The STRICT string extractor — the canonical one, use this for any new
 * caller. Copies `key`'s value into dst[cap], or fails — leaving dst
 * COMPLETELY UNTOUCHED — when the key is missing, its value isn't a JSON
 * string, or it does not fit. jsmn already knows the token's true length, so
 * the fits-or-fails check (strfit.h's rule) costs nothing and happens before
 * any byte is copied: no truncate-then-discover, the way jstr_clamped() (and
 * the get_str()/jstr_key() this replaced) used to. This is the fix for
 * e875365/23761ba applied at the one place both bugs lived: the JSON
 * extraction. */
static inline bool jstr_set(const char *js, const jsmntok_t *t, int n, const char *key,
                            char *dst, size_t cap) {
    int i = jfind(js, t, n, key);
    if (i < 0 || t[i].type != JSMN_STRING) return false;
    int len = t[i].end - t[i].start;
    if (len < 0 || (size_t)len >= cap) return false;
    memcpy(dst, js + t[i].start, (size_t)len);
    dst[len] = '\0';
    return true;
}

/* jstr_set()'s UNESCAPING twin — same strict contract (dst COMPLETELY
 * UNTOUCHED unless the key is present, is a JSON string, and its DECODED
 * value fits cap), but the bytes stored are what json_unescape() decodes the
 * token span to, not the raw span jstr_set() copies verbatim. Use this at
 * any boundary where the JSON string may legitimately contain a `"`, a `\`,
 * a control character, or a `\uXXXX` escape and the caller wants the actual
 * value, not jsmn's still-escaped token text — see json_str.h's header
 * comment for why jstr_set() alone was a silent-transcription bug (a
 * browser-sent `ab"cd` was being stored as the six bytes `ab\"cd`).
 * Length policy: the DECODED length is checked against cap, because the
 * decoded form is what actually gets stored — a 70-byte escaped span for a
 * 40-byte string still fits a 48-byte field. For any value that contains no
 * escapes, decoded length == raw length, so this behaves identically to
 * jstr_set() for every caller migrating from it — no change for the common
 * case, only for the one that used to be silently wrong. */
static inline bool jstr_set_unescaped(const char *js, const jsmntok_t *t, int n, const char *key,
                                      char *dst, size_t cap) {
    int i = jfind(js, t, n, key);
    if (i < 0 || t[i].type != JSMN_STRING) return false;
    int len = t[i].end - t[i].start;
    if (len < 0) return false;
    return json_unescape(dst, cap, js + t[i].start, (size_t)len);
}

/* The OLD clamping string extractor, renamed from the original jstr_key() so
 * its truncating nature is visible at every call site instead of hiding
 * behind an innocuous name — that hidden truncation is exactly what e875365
 * was. NOT for new callers: use jstr_set(). Kept only for two bounded-parse
 * callers in http_server.c where a truncated result just fails the
 * subsequent parse/compare rather than being stored or trusted —
 * (1) jip()'s dotted-quad scratch buffer (char[20]: a clamped value simply
 * fails sscanf's "%u.%u.%u.%u" match, same as a malformed one), and
 * (2) the provisioning "mode" string (char[12]: a clamped value just fails
 * every strcmp() against the known mode names and falls through to the
 * IC_PROV_ALL default, same as any other unrecognised value). In both cases
 * the clamped string is never persisted or echoed back as if it were what
 * the caller sent, so the silent-substitution failure this file exists to
 * prevent does not apply. On absence/wrong-type/empty, writes dst[0] = '\0'
 * (unlike jstr_set(), which leaves dst untouched on failure) — callers here
 * rely on that to get a defined empty string to sscanf/strcmp against. */
static inline void jstr_clamped(const char *js, const jsmntok_t *t, int n, const char *key,
                                char *dst, size_t cap) {
    dst[0] = '\0';
    int i = jfind(js, t, n, key);
    if (i < 0) return;
    int len = t[i].end - t[i].start;
    if (len < 0) { dst[0] = '\0'; return; }
    if (len >= (int)cap) len = (int)cap - 1;
    memcpy(dst, js + t[i].start, (size_t)len);
    dst[len] = '\0';
}

/* Dotted-quad "a.b.c.d" at `key` -> ip[4]. Built on jstr_clamped() into a
 * scratch buffer sized generously for any legal dotted-quad (max 15 chars +
 * NUL); a value too long to fit just fails the sscanf below, same as a
 * malformed one — see jstr_clamped()'s own comment for why that's fine here.
 * Shared by config.c (loading net_ip/mask/gw/dns from /config.json) and
 * http_server.c (POST /api/net) — both parsed an identical clamp-then-sscanf
 * dotted-quad before this file existed. */
static inline bool jip(const char *js, const jsmntok_t *t, int n, const char *key, uint8_t ip[4]) {
    char b[20];
    jstr_clamped(js, t, n, key, b, sizeof b);
    unsigned a0, a1, a2, a3;
    if (sscanf(b, "%u.%u.%u.%u", &a0, &a1, &a2, &a3) != 4) return false;
    ip[0] = (uint8_t)a0; ip[1] = (uint8_t)a1; ip[2] = (uint8_t)a2; ip[3] = (uint8_t)a3;
    return true;
}

/* Three-state string getter for config-LOAD use, where "key absent" (fine,
 * keep the compiled default) must be distinguishable from "key present but
 * too long for the destination field" (not fine — warn and either keep the
 * default or, for a credential, fail closed; see config.c's LOAD_STR/
 * LOAD_CRED). jstr_set() collapses both into a single `false` and so cannot
 * drive that distinction; this is jstr_set()'s three-way twin, built the same
 * fits-or-fails way (dst untouched unless the result is JSTR_OK). "Absent"
 * also covers a present-but-wrong-type value (e.g. a JSON number where a
 * string was expected) — config.c's original get_str() treated that the same
 * way, and config-load degrading to "keep the default" for a malformed field
 * is consistent with how every other field there degrades. */
typedef enum { JSTR_ABSENT, JSTR_OK, JSTR_TOO_LONG, JSTR_MALFORMED } jstr_status_t;

static inline jstr_status_t jstr_get(const char *js, const jsmntok_t *t, int n,
                                     const char *key, char *dst, size_t cap) {
    int i = jfind(js, t, n, key);
    if (i < 0 || t[i].type != JSMN_STRING) return JSTR_ABSENT;
    int len = t[i].end - t[i].start;
    if (len < 0) return JSTR_ABSENT;
    if ((size_t)len >= cap) return JSTR_TOO_LONG;
    memcpy(dst, js + t[i].start, (size_t)len);
    dst[len] = '\0';
    return JSTR_OK;
}

/* jstr_get()'s UNESCAPING twin, for the same config-LOAD use as jstr_get()
 * (see its own comment above) but where the stored value went through
 * json_escape() on the way out — i.e. every string field this file now
 * saves via config_save(). Adds a 4th state jstr_get()'s three cannot
 * express: a present, JSON-string-typed value whose escape sequences are
 * broken (dangling backslash, bad \uXXXX, a lone surrogate) — JSTR_MALFORMED,
 * distinct from JSTR_TOO_LONG so a caller can log the right diagnosis
 * instead of "too long" for a field that decodes short but decodes wrong.
 * config.c's config_load() treats JSTR_MALFORMED the same way it treats
 * JSTR_TOO_LONG for each field kind (CFG_STR: warn + keep default; CFG_CRED:
 * clear + fail closed, never fall back to the compiled admin/admin default —
 * a damaged escape in a credential is exactly the "config file is damaged"
 * case that rule exists for; CFG_HOSTNAME: warn + keep default). Absent, or
 * present-but-wrong-type, is still JSTR_ABSENT — unchanged from jstr_get().
 * Length policy matches jstr_set_unescaped(): the cap check is against the
 * DECODED length, since that's what's actually stored. */
static inline jstr_status_t jstr_get_unescaped(const char *js, const jsmntok_t *t, int n,
                                               const char *key, char *dst, size_t cap) {
    int i = jfind(js, t, n, key);
    if (i < 0 || t[i].type != JSMN_STRING) return JSTR_ABSENT;
    int len = t[i].end - t[i].start;
    if (len < 0) return JSTR_ABSENT;
    size_t need = json_unescape_core(NULL, 0, js + t[i].start, (size_t)len);
    if (need == (size_t)-1) return JSTR_MALFORMED;
    if (need >= cap) return JSTR_TOO_LONG;
    if (!json_unescape(dst, cap, js + t[i].start, (size_t)len)) return JSTR_MALFORMED; /* can't happen, see json_unescape()'s own comment */
    return JSTR_OK;
}
