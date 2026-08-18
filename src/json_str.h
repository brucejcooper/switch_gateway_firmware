#pragma once
/*
 * json_str.h — RFC 8259 §7 JSON string escape/unescape, exact and total: any
 * byte sequence a config field can hold round-trips through both directions.
 *
 * WHY THIS EXISTS. config_save() writes every string field as a raw %s
 * between quotes:
 *
 *     fprintf(fp, "\"mqtt_pass\":\"%s\"", g_config.mqtt_pass);
 *
 * A password set via the CLI as `ab"cd` lands in the file as
 * `"mqtt_pass":"ab"cd"` — the embedded quote closes the JSON string three
 * characters early, and everything after it (every field that follows in the
 * fprintf, i.e. most of the file) becomes trailing garbage. jsmn_parse()
 * fails outright on the whole document, config_load() falls back to
 * config_defaults() for every field, and that includes web_user/web_pass —
 * so one legitimate CLI write silently reopens the web UI on admin/admin on
 * the next boot. A single unescaped byte in one field is a config wipe *and*
 * an auth downgrade.
 *
 * The asymmetry that makes BOTH directions this file's job: jsmn is a
 * tokenizer, not a decoder. jsmn_parse_string() (lib/jsmn/jsmn.h) validates
 * that an escape sequence is well-formed — a backslash is followed by one of
 * the allowed letters, or by `u` and exactly 4 hex digits — but it does not
 * decode it. The token span jsmn hands back still contains the raw source
 * bytes, backslashes included: a browser-sent `ab"cd` arrives across the
 * wire as `ab\"cd` (the browser's own JSON.stringify escaped it correctly),
 * and jstr_set() (jsmn_util.h, before this file) copied that span verbatim
 * — so what got stored was the six bytes `ab\"cd`, literal backslash and
 * all, not the four the user typed. Not corruption (the backslash is a
 * legal password character to jsmn), but a silent transcription error: the
 * broker/UI sees a different string than the one entered. jsmn validates
 * escapes on the way in; nothing decoded them on the way out. That is the
 * gap these two functions close.
 *
 * json_escape() is config_save()'s replacement for the bare field in the
 * fprintf format string — escape into a scratch buffer first, then print the
 * scratch. json_unescape() is jstr_get_unescaped()/jstr_set_unescaped()'s
 * decode step (jsmn_util.h) — every config-JSON string boundary (config.c's
 * load, http_server.c's /api/mqtt, /api/auth, /api/net) goes through one of
 * those, never through this header directly.
 *
 * Deliberately OUT of scope: ha_bridge.c's MQTT payload parsers (bindings,
 * light state) are a separate wire contract that Home Assistant, not this
 * firmware, defines and already exercises in production — changing their
 * decode path risks breaking working entity behaviour for a bug class HA's
 * own JSON encoder doesn't hit in practice (HA escapes correctly; the bug
 * here is *our* encoder, not our decoder of HA's traffic). Left untouched on
 * purpose; do not fold it into this pass.
 *
 * Header-only static inline, same reasoning as strfit.h and jsmn_util.h
 * (read strfit.h first if this is new): a few dozen lines, no allocation,
 * and per-TU JSMN_STATIC just works with no separate source-list entry to
 * remember in either firmware target.
 *
 * Contracts, both functions:
 *   - No allocation. No aliasing: dst and src MUST NOT overlap (the decode
 *     walks src while writing dst as it goes in the real, committing pass —
 *     see json_unescape_core() below — so a caller cannot decode in place
 *     the way config_hostname_set() decodes hostnames in place). No call
 *     site in this firmware needs in-place escape/unescape; if one ever
 *     does, it needs its own copy-through-scratch step, not a change here.
 *   - Fits-or-fails, dst COMPLETELY UNTOUCHED on failure — the same rule
 *     strfit.h's str_set() and jsmn_util.h's jstr_set() already enforce for
 *     every other config string path. Both functions here get this by
 *     construction, not by promise: each runs a non-writing pass first (measure
 *     the exact output size / validate every escape) and only touches dst in
 *     a second pass once that size is known to fit cap. A caller that ignores
 *     the return value can never end up with a half-written field.
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Decode 4 ASCII hex digits at s[0..3] into *out. Case-insensitive, per
 * \uXXXX (RFC 8259 §7). Returns false on any non-hex-digit byte. */
static inline bool json_hex4(const char *s, uint32_t *out) {
    uint32_t v = 0;
    for (int i = 0; i < 4; i++) {
        char c = s[i];
        uint32_t d;
        if      (c >= '0' && c <= '9') d = (uint32_t)(c - '0');
        else if (c >= 'a' && c <= 'f') d = (uint32_t)(c - 'a' + 10);
        else if (c >= 'A' && c <= 'F') d = (uint32_t)(c - 'A' + 10);
        else return false;
        v = (v << 4) | d;
    }
    *out = v;
    return true;
}

/* Escape `src` (a NUL-terminated C string — a config field's live value, not
 * a jsmn token span) into dst[cap] as the exact bytes to place between a
 * JSON document's quotes, per RFC 8259 §7:
 *   - `"` and `\` become `\"` and `\\`.
 *   - backspace/form-feed/newline/CR/tab become `\b` `\f` `\n` `\r` `\t`.
 *   - every other byte < 0x20 becomes `\u00XX` (2-digit uppercase-agnostic
 *     hex, lowercase emitted here — RFC 8259 accepts either case on input,
 *     and this is output).
 *   - every other byte — 0x20..0x7E, DEL (0x7F), and every byte >= 0x80 of a
 *     UTF-8-encoded field — passes through untouched. This file does not
 *     interpret UTF-8; it escapes JSON-significant bytes only and leaves
 *     multi-byte sequences exactly as they arrived, which is correct because
 *     none of their bytes are JSON-significant (all >= 0x80 by construction).
 *
 * Returns false — dst untouched — if the escaped form (plus its NUL) does
 * not fit cap, or if dst/src is NULL or cap is 0. No allocation: the first
 * pass measures the exact escaped length so the second pass either fits
 * completely or is never started. */
static inline bool json_escape(char *dst, size_t cap, const char *src) {
    if (!dst || cap == 0 || !src) return false;

    size_t need = 0;
    for (const unsigned char *p = (const unsigned char *)src; *p; p++) {
        unsigned char ch = *p;
        switch (ch) {
        case '"': case '\\':
        case '\b': case '\f': case '\n': case '\r': case '\t':
            need += 2;
            break;
        default:
            need += (ch < 0x20) ? 6 : 1;
            break;
        }
        if (need >= cap) return false;   /* bail early; also bounds the loop */
    }
    if (need >= cap) return false;       /* room for need bytes + NUL */

    static const char hex[] = "0123456789abcdef";
    char *d = dst;
    for (const unsigned char *p = (const unsigned char *)src; *p; p++) {
        unsigned char ch = *p;
        switch (ch) {
        case '"':  *d++ = '\\'; *d++ = '"';  break;
        case '\\': *d++ = '\\'; *d++ = '\\'; break;
        case '\b': *d++ = '\\'; *d++ = 'b';  break;
        case '\f': *d++ = '\\'; *d++ = 'f';  break;
        case '\n': *d++ = '\\'; *d++ = 'n';  break;
        case '\r': *d++ = '\\'; *d++ = 'r';  break;
        case '\t': *d++ = '\\'; *d++ = 't';  break;
        default:
            if (ch < 0x20) {
                *d++ = '\\'; *d++ = 'u'; *d++ = '0'; *d++ = '0';
                *d++ = hex[(ch >> 4) & 0xF];
                *d++ = hex[ch & 0xF];
            } else {
                *d++ = (char)ch;
            }
            break;
        }
    }
    *d = '\0';
    return true;
}

/* Shared decode walk for json_unescape(): consumes src[0..len) once,
 * interpreting escapes exactly as described there, and either just counts
 * the decoded output (dst == NULL — the validate pass) or writes it
 * (dst != NULL, cap the destination's real size — the commit pass). One
 * function, not two copies of the same escape table that could drift.
 *
 * Returns the decoded byte count (NOT including a NUL — caller appends it)
 * on success. Returns (size_t)-1 on malformed input (dangling backslash,
 * unknown escape letter, bad \uXXXX hex, a lone UTF-16 surrogate) — checked
 * identically whether or not dst is NULL, so the validate pass and the
 * commit pass agree on what counts as malformed — or, when dst != NULL, if
 * the output would not fit cap - 1 bytes (room reserved for the NUL). */
static inline size_t json_unescape_core(char *dst, size_t cap, const char *src, size_t len) {
    size_t si = 0, di = 0;
    while (si < len) {
        unsigned char ch = (unsigned char)src[si];

        if (ch != '\\') {
            if (dst && di >= cap - 1) return (size_t)-1;
            if (dst) dst[di] = (char)ch;
            di++; si++;
            continue;
        }

        if (si + 1 >= len) return (size_t)-1;   /* dangling backslash at end of span */
        char esc = src[si + 1];
        char lit;
        switch (esc) {
        case '"':  lit = '"';  break;
        case '\\': lit = '\\'; break;
        case '/':  lit = '/';  break;
        case 'b':  lit = '\b'; break;
        case 'f':  lit = '\f'; break;
        case 'n':  lit = '\n'; break;
        case 'r':  lit = '\r'; break;
        case 't':  lit = '\t'; break;
        case 'u':  lit = 0;    break;   /* handled below, not a single-byte literal */
        default:   return (size_t)-1;   /* jsmn already rejects this at parse time,
                                          * but this file trusts no caller's input
                                          * over its own checks (see strfit.h's rule) */
        }

        if (esc != 'u') {
            if (dst && di >= cap - 1) return (size_t)-1;
            if (dst) dst[di] = lit;
            di++; si += 2;
            continue;
        }

        /* \uXXXX — jsmn guaranteed 4 hex digits follow at parse time, but this
         * span may not be the same bytes jsmn validated (defence in depth), so
         * re-check the bound and the hex here too. */
        if (si + 6 > len) return (size_t)-1;
        uint32_t cp;
        if (!json_hex4(src + si + 2, &cp)) return (size_t)-1;
        si += 6;

        if (cp >= 0xD800 && cp <= 0xDBFF) {
            /* High surrogate: must be immediately followed by a low surrogate
             * \uXXXX — that is the only legal way UTF-16 (and so JSON) spells
             * a codepoint above the BMP (e.g. an emoji). */
            if (si + 6 > len || src[si] != '\\' || src[si + 1] != 'u') return (size_t)-1;
            uint32_t lo;
            if (!json_hex4(src + si + 2, &lo)) return (size_t)-1;
            if (lo < 0xDC00 || lo > 0xDFFF) return (size_t)-1;   /* not a low surrogate */
            si += 6;
            cp = 0x10000u + ((cp - 0xD800u) << 10) + (lo - 0xDC00u);
        } else if (cp >= 0xDC00 && cp <= 0xDFFF) {
            return (size_t)-1;   /* lone low surrogate — never valid on its own */
        }

        unsigned nbytes = cp < 0x80u    ? 1 :
                          cp < 0x800u   ? 2 :
                          cp < 0x10000u ? 3 : 4;
        if (dst && di + nbytes > cap - 1) return (size_t)-1;
        if (dst) {
            switch (nbytes) {
            case 1:
                dst[di] = (char)cp;
                break;
            case 2:
                dst[di]     = (char)(0xC0 | (cp >> 6));
                dst[di + 1] = (char)(0x80 | (cp & 0x3F));
                break;
            case 3:
                dst[di]     = (char)(0xE0 | (cp >> 12));
                dst[di + 1] = (char)(0x80 | ((cp >> 6) & 0x3F));
                dst[di + 2] = (char)(0x80 | (cp & 0x3F));
                break;
            default:
                dst[di]     = (char)(0xF0 | (cp >> 18));
                dst[di + 1] = (char)(0x80 | ((cp >> 12) & 0x3F));
                dst[di + 2] = (char)(0x80 | ((cp >> 6) & 0x3F));
                dst[di + 3] = (char)(0x80 | (cp & 0x3F));
                break;
            }
        }
        di += nbytes;
    }
    return di;
}

/* Decode a JSON string TOKEN SPAN — src[0..len), not NUL-terminated (this is
 * jsmn's start/end, the raw bytes between the quotes jsmn already stripped)
 * — into dst[cap] as the original bytes the value represents:
 *   - `\"` `\\` `\/` `\b` `\f` `\n` `\r` `\t` become their one-byte literal.
 *   - `\uXXXX` becomes UTF-8: 1 byte for U+0000..U+007F, 2 bytes for
 *     U+0080..U+07FF, 3 bytes for any other BMP codepoint, or 4 bytes when
 *     paired as a UTF-16 surrogate pair (`😀` -> the 4-byte UTF-8
 *     encoding of an emoji-range codepoint).
 *   - any other byte (jsmn accepted it unescaped in the span, including a
 *     raw control byte or a UTF-8 continuation byte >= 0x80 — see this
 *     file's header comment on jsmn's validate-but-don't-decode split) is
 *     copied through unchanged.
 *
 * Fails — dst untouched, returns false — on a dangling backslash, an escape
 * letter outside the RFC 8259 set, malformed `\uXXXX` hex, a lone UTF-16
 * surrogate, or if the decoded output does not fit cap - 1 bytes. Decoded
 * output is never longer than the input span (every multi-byte UTF-8
 * encoding above replaces a 6-byte `\uXXXX` or 12-byte surrogate pair), but
 * this checks the real bound rather than trusting that arithmetic — see the
 * file header's "never trust shouldn't-happen" note. */
static inline bool json_unescape(char *dst, size_t cap, const char *src, size_t len) {
    if (!dst || cap == 0 || !src) return false;

    size_t need = json_unescape_core(NULL, 0, src, len);
    if (need == (size_t)-1 || need >= cap) return false;

    size_t written = json_unescape_core(dst, cap, src, len);
    if (written == (size_t)-1 || written != need) return false;   /* can't happen: identical input/cap as the pass above */
    dst[written] = '\0';
    return true;
}
