/*
 * json_str_test.c — host-side test for json_str.h's json_escape()/
 * json_unescape() (RFC 8259 §7 JSON string escape pair) and jsmn_util.h's
 * jstr_set_unescaped()/jstr_get_unescaped(), the extractors built on them.
 *
 * Coverage: every byte 0x00..0xFF escapes and round-trips; the RFC's named
 * short escapes and \uXXXX (including 2/3-byte UTF-8 and a surrogate pair)
 * decode exactly; malformed input (lone surrogate, bad hex, dangling
 * backslash, unknown escape letter) is rejected, not guessed at; fits-or-
 * fails holds at the exact boundary in both directions; and a nasty corpus
 * round-trips through the REAL pipeline — json_escape() -> jsmn_parse() of a
 * {"k":"<escaped>"} document -> jstr_get_unescaped() — the actual path
 * config_save()/config_load() and the HTTP API now use.
 *
 * Build & run:
 *   cc -DJSMN_STATIC -Wall -Wextra -I ../src -I ../lib/jsmn \
 *       -o /tmp/json_str_test json_str_test.c \
 *       && /tmp/json_str_test
 */
#include "../src/json_str.h"
#include "../src/jsmn_util.h"
#include "../lib/jsmn/jsmn.h"

#include <stdio.h>
#include <string.h>

static int g_fails = 0;
static const char *g_case = "";
#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL [%s]: %s (%s:%d)\n", g_case, #cond, __FILE__, __LINE__); g_fails++; } \
} while (0)

/* ── json_escape() / json_unescape() round-trip on raw bytes ──────────────*/

/* Escape `src` (n bytes, NOT necessarily NUL-free) into esc[], reparse it as
 * a bare JSON string token via jsmn (the same tokenizer config_load()/
 * http_server.c actually use), unescape the token span, and confirm the
 * result is byte-for-byte src. Exercises json_escape()+json_unescape()
 * together AND jsmn's own "validates the escapes we emit" behaviour. */
static void round_trip(const unsigned char *src, size_t n) {
    char in[512];
    CHECK(n < sizeof in);
    memcpy(in, src, n); in[n] = '\0';

    char esc[6 * 512 + 1];
    CHECK(json_escape(esc, sizeof esc, in));

    char doc[6 * 512 + 8];
    int dlen = snprintf(doc, sizeof doc, "{\"k\":\"%s\"}", esc);
    CHECK(dlen > 0 && (size_t)dlen < sizeof doc);

    jsmn_parser jp; jsmntok_t tk[8]; jsmn_init(&jp);
    int nt = jsmn_parse(&jp, doc, (size_t)dlen, tk, 8);
    CHECK(nt > 0);

    char back[512];
    jstr_status_t st = jstr_get_unescaped(doc, tk, nt, "k", back, sizeof back);
    CHECK(st == JSTR_OK);
    CHECK(memcmp(back, src, n) == 0 && back[n] == '\0');
}

static void every_byte(void) {
    g_case = "every byte 0x00..0xff escapes and round-trips";
    for (int b = 0; b <= 0xFF; b++) {
        if (b == 0) continue;   /* NUL can't appear in a C-string src; not a
                                  * legal case for these NUL-terminated-input
                                  * functions — see json_str.h's contract */
        unsigned char one[1] = { (unsigned char)b };
        round_trip(one, 1);
    }
    /* And a run of every non-NUL byte together, to catch any interaction
     * between adjacent escape forms. */
    unsigned char all[255];
    int k = 0;
    for (int b = 1; b <= 0xFF; b++) all[k++] = (unsigned char)b;
    round_trip(all, (size_t)k);
}

static void named_escapes_exact_form(void) {
    g_case = "named short escapes produce the exact RFC 8259 form";
    struct { char in; const char *want; } T[] = {
        { '"',  "\\\"" }, { '\\', "\\\\" },
        { '\b', "\\b" },  { '\f', "\\f" }, { '\n', "\\n" },
        { '\r', "\\r" },  { '\t', "\\t" },
    };
    for (size_t i = 0; i < sizeof T / sizeof T[0]; i++) {
        char in[2] = { T[i].in, '\0' };
        char esc[16];
        CHECK(json_escape(esc, sizeof esc, in));
        CHECK(strcmp(esc, T[i].want) == 0);
    }
    /* Other control bytes -> \u00XX, lowercase hex. */
    struct { unsigned char in; const char *want; } U[] = {
        { 0x01, "\\u0001" },   /* 0x00 excluded — no NUL in a C-string src */
        { 0x1F, "\\u001f" },
        { 0x0B, "\\u000b" },   /* vertical tab: control, but not one of the named 6 */
    };
    for (size_t i = 0; i < sizeof U / sizeof U[0]; i++) {
        char in[2] = { (char)U[i].in, '\0' };
        char esc[16];
        CHECK(json_escape(esc, sizeof esc, in));
        CHECK(strcmp(esc, U[i].want) == 0);
    }
    /* Ordinary bytes, DEL, and a high-bit UTF-8 byte pass through untouched. */
    char esc[16];
    CHECK(json_escape(esc, sizeof esc, "a")); CHECK(strcmp(esc, "a") == 0);
    CHECK(json_escape(esc, sizeof esc, "\x7f")); CHECK(strcmp(esc, "\x7f") == 0);
    CHECK(json_escape(esc, sizeof esc, "\xc3\xa9")); CHECK(strcmp(esc, "\xc3\xa9") == 0);
}

/* ── \uXXXX decode: BMP widths + surrogate pair ────────────────────────────*/

static void unicode_decode_widths(void) {
    g_case = "\\uXXXX decode: 1/2/3-byte UTF-8 and a surrogate pair";
    char dst[16];

    /* U+0041 'A' -> 1-byte UTF-8 (ASCII). */
    CHECK(json_unescape(dst, sizeof dst, "\\u0041", 6));
    CHECK(strcmp(dst, "A") == 0);

    /* U+00E9 'é' -> 2-byte UTF-8 (0xC3 0xA9). */
    CHECK(json_unescape(dst, sizeof dst, "\\u00e9", 6));
    CHECK((unsigned char)dst[0] == 0xC3 && (unsigned char)dst[1] == 0xA9 && dst[2] == '\0');

    /* U+4E2D '中' -> 3-byte UTF-8 (0xE4 0xB8 0xAD). */
    CHECK(json_unescape(dst, sizeof dst, "\\u4e2d", 6));
    CHECK((unsigned char)dst[0] == 0xE4 && (unsigned char)dst[1] == 0xB8 &&
          (unsigned char)dst[2] == 0xAD && dst[3] == '\0');

    /* U+1F600 (😀) as a UTF-16 surrogate pair 😀 -> 4-byte UTF-8
     * (0xF0 0x9F 0x98 0x80). */
    CHECK(json_unescape(dst, sizeof dst, "\\ud83d\\ude00", 12));
    CHECK((unsigned char)dst[0] == 0xF0 && (unsigned char)dst[1] == 0x9F &&
          (unsigned char)dst[2] == 0x98 && (unsigned char)dst[3] == 0x80 && dst[4] == '\0');

    /* Round-trip: escape emoji-containing text via json_escape()'s "pass
     * through >= 0x80 untouched" rule (it does not re-encode as \uXXXX —
     * the RFC does not require it and this file never emits one), and
     * confirm json_unescape() on the RAW UTF-8 bytes is a straight copy. */
    char esc[32];
    CHECK(json_escape(esc, sizeof esc, "\xf0\x9f\x98\x80"));   /* raw 😀 UTF-8 */
    CHECK(strcmp(esc, "\xf0\x9f\x98\x80") == 0);                /* untouched */
    char back[8];
    CHECK(json_unescape(back, sizeof back, esc, strlen(esc)));
    CHECK(memcmp(back, "\xf0\x9f\x98\x80", 4) == 0 && back[4] == '\0');
}

/* ── rejects ────────────────────────────────────────────────────────────*/

static void rejects(void) {
    g_case = "json_unescape rejects malformed input";
    char dst[16];
    /* Lone high surrogate, no low surrogate following. */
    CHECK(!json_unescape(dst, sizeof dst, "\\ud83d", 6));
    /* Lone low surrogate on its own. */
    CHECK(!json_unescape(dst, sizeof dst, "\\ude00", 6));
    /* High surrogate followed by something that isn't a low surrogate. */
    CHECK(!json_unescape(dst, sizeof dst, "\\ud83d\\u0041", 12));
    /* High surrogate followed by a non-escape. */
    CHECK(!json_unescape(dst, sizeof dst, "\\ud83dXX", 8));
    /* Bad hex digits. */
    CHECK(!json_unescape(dst, sizeof dst, "\\u00zz", 6));
    CHECK(!json_unescape(dst, sizeof dst, "\\uXXXX", 6));
    /* Truncated \u (fewer than 4 hex digits available in the span). */
    CHECK(!json_unescape(dst, sizeof dst, "\\u12", 4));
    /* Dangling backslash at end of span. */
    CHECK(!json_unescape(dst, sizeof dst, "abc\\", 4));
    /* Unknown escape letter (RFC 8259 §7 permits only " \ / b f n r t u). */
    CHECK(!json_unescape(dst, sizeof dst, "\\q", 2));
    CHECK(!json_unescape(dst, sizeof dst, "\\1", 2));

    g_case = "json_escape/json_unescape reject NULL/zero-cap, dst untouched";
    char d[8] = "keep";
    CHECK(!json_escape(NULL, 8, "x"));
    CHECK(!json_escape(d, 0, "x"));
    CHECK(!json_escape(d, sizeof d, NULL));
    CHECK(strcmp(d, "keep") == 0);
    CHECK(!json_unescape(NULL, 8, "x", 1));
    CHECK(!json_unescape(d, 0, "x", 1));
    CHECK(!json_unescape(d, sizeof d, NULL, 1));
    CHECK(strcmp(d, "keep") == 0);
}

/* ── fits-or-fails at the exact boundary, both directions ─────────────────*/

static void boundary_escape(void) {
    g_case = "json_escape: fits-or-fails at the exact boundary";
    /* "ab" escapes to "ab" (2 bytes) — needs cap 3 (2 + NUL). */
    char d[16] = "sentinel";
    CHECK(!json_escape(d, 2, "ab"));           /* one short — must fail */
    CHECK(strcmp(d, "sentinel") == 0);         /* untouched */
    CHECK(json_escape(d, 3, "ab"));            /* exact fit */
    CHECK(strcmp(d, "ab") == 0);

    /* A quote escapes to 2 bytes (\"), needs cap 3. */
    memcpy(d, "sentinel", 9);   /* d[16] */
    CHECK(!json_escape(d, 3, "\"\""));         /* 2 quotes -> 4 bytes, needs cap 5 */
    CHECK(strcmp(d, "sentinel") == 0);
    CHECK(json_escape(d, 5, "\"\""));
    CHECK(strcmp(d, "\\\"\\\"") == 0);

    /* A control byte escapes to 6 bytes (\u00XX), needs cap 7. */
    memcpy(d, "sentinel", 9);   /* d[16] */
    char in1[2] = { 0x01, '\0' };
    CHECK(!json_escape(d, 6, in1));            /* one short */
    CHECK(strcmp(d, "sentinel") == 0);
    CHECK(json_escape(d, 7, in1));
    CHECK(strcmp(d, "\\u0001") == 0);
}

static void boundary_unescape(void) {
    g_case = "json_unescape: fits-or-fails at the exact boundary";
    char d[16] = "sentinel";
    /* "ab" decodes to 2 bytes, needs cap 3. */
    CHECK(!json_unescape(d, 2, "ab", 2));
    CHECK(strcmp(d, "sentinel") == 0);
    CHECK(json_unescape(d, 3, "ab", 2));
    CHECK(strcmp(d, "ab") == 0);

    /* é decodes to 2 bytes, needs cap 3. */
    memcpy(d, "sentinel", 9);   /* d[16] */
    CHECK(!json_unescape(d, 2, "\\u00e9", 6));
    CHECK(strcmp(d, "sentinel") == 0);
    CHECK(json_unescape(d, 3, "\\u00e9", 6));
    CHECK((unsigned char)d[0] == 0xC3 && (unsigned char)d[1] == 0xA9);

    /* Surrogate pair decodes to 4 bytes, needs cap 5. */
    memcpy(d, "sentinel", 9);   /* d[16] */
    CHECK(!json_unescape(d, 4, "\\ud83d\\ude00", 12));
    CHECK(strcmp(d, "sentinel") == 0);
    CHECK(json_unescape(d, 5, "\\ud83d\\ude00", 12));
}

/* ── jsmn_util.h extractors: jstr_set_unescaped()/jstr_get_unescaped() ───*/

static void jsmn_extractors(void) {
    g_case = "jstr_set_unescaped: decodes, fits-or-fails, absent/wrong-type";
    const char *doc = "{\"a\":\"ab\\\"cd\",\"b\":\"back\\\\slash\",\"c\":42}";
    jsmn_parser jp; jsmntok_t tk[16]; jsmn_init(&jp);
    int nt = jsmn_parse(&jp, doc, strlen(doc), tk, 16);
    CHECK(nt > 0);

    char d[16];
    CHECK(jstr_set_unescaped(doc, tk, nt, "a", d, sizeof d));
    CHECK(strcmp(d, "ab\"cd") == 0);
    CHECK(jstr_set_unescaped(doc, tk, nt, "b", d, sizeof d));
    CHECK(strcmp(d, "back\\slash") == 0);
    CHECK(!jstr_set_unescaped(doc, tk, nt, "c", d, sizeof d));   /* not a string */
    CHECK(!jstr_set_unescaped(doc, tk, nt, "nope", d, sizeof d)); /* absent */
    /* Too-small destination — untouched on failure. */
    char small[3] = "xy";
    CHECK(!jstr_set_unescaped(doc, tk, nt, "a", small, sizeof small));
    CHECK(strcmp(small, "xy") == 0);

    g_case = "jstr_get_unescaped: four states";
    char big[64];
    CHECK(jstr_get_unescaped(doc, tk, nt, "a", big, sizeof big) == JSTR_OK);
    CHECK(strcmp(big, "ab\"cd") == 0);
    CHECK(jstr_get_unescaped(doc, tk, nt, "nope", big, sizeof big) == JSTR_ABSENT);
    CHECK(jstr_get_unescaped(doc, tk, nt, "c", big, sizeof big) == JSTR_ABSENT);   /* wrong type */

    char tiny[3];
    CHECK(jstr_get_unescaped(doc, tk, nt, "a", tiny, sizeof tiny) == JSTR_TOO_LONG);

    const char *baddoc = "{\"m\":\"bad\\udead\"}";   /* lone high surrogate */
    jsmn_init(&jp);
    int bn = jsmn_parse(&jp, baddoc, strlen(baddoc), tk, 16);
    CHECK(bn > 0);
    char out[32];
    CHECK(jstr_get_unescaped(baddoc, tk, bn, "m", out, sizeof out) == JSTR_MALFORMED);
}

/* ── the real pipeline: json_escape -> jsmn_parse -> jstr_get_unescaped ──*/

static void real_pipeline_corpus(void) {
    g_case = "real pipeline round-trip: a nasty corpus";
    const char *corpus[] = {
        "ab\"cd",
        "back\\slash",
        "tab\ttab",
        "line1\nline2",
        "carriage\rreturn",
        "\xc3\xa9\xc3\xa8\xc3\xaa",   /* éèê, raw UTF-8 */
        "\x01",                       /* a single control byte */
        "",                            /* empty string */
        "plain ascii, nothing special",
        "quote\"then\\backslash\"again",
    };
    for (size_t i = 0; i < sizeof corpus / sizeof corpus[0]; i++) {
        char esc[512];
        CHECK(json_escape(esc, sizeof esc, corpus[i]));

        char doc[600];
        int dlen = snprintf(doc, sizeof doc, "{\"k\":\"%s\"}", esc);
        CHECK(dlen > 0 && (size_t)dlen < sizeof doc);

        jsmn_parser jp; jsmntok_t tk[8]; jsmn_init(&jp);
        int nt = jsmn_parse(&jp, doc, (size_t)dlen, tk, 8);
        CHECK(nt > 0);

        char back[512];
        CHECK(jstr_get_unescaped(doc, tk, nt, "k", back, sizeof back) == JSTR_OK);
        CHECK(strcmp(back, corpus[i]) == 0);
    }
}

int main(void) {
    every_byte();
    named_escapes_exact_form();
    unicode_decode_widths();
    rejects();
    boundary_escape();
    boundary_unescape();
    jsmn_extractors();
    real_pipeline_corpus();

    if (g_fails == 0) printf("PASS: all json_str checks passed\n");
    else               printf("%d CHECK(s) FAILED\n", g_fails);
    return g_fails ? 1 : 0;
}
