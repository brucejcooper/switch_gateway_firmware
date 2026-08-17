#pragma once
/*
 * strfit.h — "copy a string into a fixed field, or fail" — the one rule for
 * every externally-supplied string that lands in a fixed-size config field.
 *
 * WHY THIS EXISTS. Three separate layers each silently truncated to fit, and
 * none of them could tell a caller it had happened:
 *
 *   - http_server.c's jstr_key() clamps to its cap (`if (len >= cap) len =
 *     cap - 1`). Measured on the bench: POSTing a 32-char hostname to
 *     /api/net answered {"ok":true} and stored 31 chars — a value the caller
 *     never asked for, and one the CLI rejects outright.
 *   - config.c's get_str() clamps the same way on load.
 *   - the CLI's strncpy(dst, src, sizeof dst - 1) truncates, and four sites
 *     did not even write the terminating NUL, relying on the unstated
 *     invariant that the field's last byte is never non-zero.
 *
 * A too-long value is an ERROR THE CALLER MUST REPORT, not a shorter value
 * substituted behind their back: a truncated password or MQTT prefix still
 * saves, still reports success, and then simply does not work — with nothing
 * anywhere saying why. Refusing is both safer and far easier to diagnose.
 *
 * Header-only static inline deliberately: it is a few lines, and the two
 * executables (light_switch, light_switch_ota) keep separate source lists, so
 * a header cannot be added to one and forgotten in the other.
 */
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

/* Copy `src` into dst[cap] (NUL included), or fail if it does not fit.
 *
 * Returns false — leaving dst COMPLETELY UNTOUCHED — when either pointer is
 * NULL, cap is 0, or strlen(src) >= cap. Leaving dst untouched matters: a
 * caller that ignores the result must not end up with a half-written field.
 *
 * dst == src is safe (the copy runs strictly forward, writing index i only
 * after reading it, and never re-reading a written index), though no caller
 * relies on it today. */
static inline bool str_set(char *dst, size_t cap, const char *src) {
    if (!dst || cap == 0 || !src) return false;
    size_t n = strlen(src);
    if (n >= cap) return false;
    for (size_t i = 0; i < n; i++) dst[i] = src[i];
    dst[n] = '\0';
    return true;
}

/* The largest string a field of `cap` bytes can hold, for error messages that
 * name the real limit ("max 23") instead of leaving the user to guess. */
#define STR_MAX_LEN(cap) ((unsigned)((cap) - 1u))
