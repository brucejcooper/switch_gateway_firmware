/*
 * web_auth_test.c — host-side test for web_auth.c: the Argon2id KDF wrapper,
 * its hex codec, the constant-time compare, and the busy/retry contract the
 * shared work area's lock makes to every caller (config.c's migration,
 * ic_seam.c's IC_CMD_SET_AUTH, net/http_server.c's check_auth() and POST
 * /api/auth handler — see web_auth.h's header comment for all four).
 *
 * Runs against the HOST_TEST lock stand-in (web_auth.c's WEB_AUTH_HOST_TEST
 * branch) — a plain flag standing in for the RP2040 hardware spinlock, since
 * there is no real second core to contend it with on the host. That branch's
 * whole job is to give the SAME pass/fail SHAPE the target lock gives
 * (successful acquire when free, `false` when busy, never a torn/partial
 * hash) — the target-only lock acquisition itself (spin_try_lock_unsafe())
 * is not something a single-threaded host process can exercise, so it isn't
 * what this file is pinning.
 *
 * Build & run:
 *   cc -DWEB_AUTH_HOST_TEST -Wall -Wextra -I ../src -I ../lib/monocypher \
 *       -o /tmp/web_auth_test web_auth_test.c ../src/web_auth.c \
 *       ../lib/monocypher/monocypher.c && /tmp/web_auth_test
 */
#include "../src/web_auth.h"

#include <stdio.h>
#include <string.h>

/* The host-test busy-injection hook (web_auth.c's WEB_AUTH_HOST_TEST branch). */
extern bool web_auth_test_force_busy;

static int g_fails = 0;
static const char *g_case = "";
#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL [%s]: %s (%s:%d)\n", g_case, #cond, __FILE__, __LINE__); g_fails++; } \
} while (0)

int main(void) {
    web_auth_init();

    g_case = "hash: set->verify roundtrip — correct password verifies";
    {
        uint8_t salt[WEB_AUTH_SALT_LEN], hash[WEB_AUTH_HASH_LEN], check[WEB_AUTH_HASH_LEN];
        web_auth_gen_salt(salt);
        CHECK(web_auth_hash("correct horse battery staple", salt, hash, 0));
        CHECK(web_auth_hash("correct horse battery staple", salt, check, 0));
        CHECK(web_auth_verify_equal(hash, check));
    }

    g_case = "hash: wrong password refuses (different hash, verify_equal false)";
    {
        uint8_t salt[WEB_AUTH_SALT_LEN], want[WEB_AUTH_HASH_LEN], got[WEB_AUTH_HASH_LEN];
        web_auth_gen_salt(salt);
        CHECK(web_auth_hash("admin", salt, want, 0));
        CHECK(web_auth_hash("not-admin", salt, got, 0));
        CHECK(!web_auth_verify_equal(want, got));
    }

    g_case = "hash: same password, different salt -> different hash";
    {
        uint8_t salt_a[WEB_AUTH_SALT_LEN], salt_b[WEB_AUTH_SALT_LEN];
        uint8_t hash_a[WEB_AUTH_HASH_LEN], hash_b[WEB_AUTH_HASH_LEN];
        memset(salt_a, 0x11, sizeof salt_a);
        memset(salt_b, 0x22, sizeof salt_b);
        CHECK(web_auth_hash("admin", salt_a, hash_a, 0));
        CHECK(web_auth_hash("admin", salt_b, hash_b, 0));
        CHECK(!web_auth_verify_equal(hash_a, hash_b));
    }

    g_case = "hash: deterministic for the same password+salt";
    {
        uint8_t salt[WEB_AUTH_SALT_LEN] = { 0 };
        uint8_t h1[WEB_AUTH_HASH_LEN], h2[WEB_AUTH_HASH_LEN];
        CHECK(web_auth_hash("repeat", salt, h1, 0));
        CHECK(web_auth_hash("repeat", salt, h2, 0));
        CHECK(web_auth_verify_equal(h1, h2));
    }

    g_case = "salt: two draws are not the same 16 bytes";
    {
        uint8_t a[WEB_AUTH_SALT_LEN], b[WEB_AUTH_SALT_LEN];
        web_auth_gen_salt(a);
        web_auth_gen_salt(b);
        CHECK(memcmp(a, b, sizeof a) != 0);
    }

    g_case = "busy: a caller that loses the race gets false back, output untouched";
    {
        uint8_t salt[WEB_AUTH_SALT_LEN] = { 0 };
        uint8_t hash[WEB_AUTH_HASH_LEN];
        memset(hash, 0xAA, sizeof hash);
        web_auth_test_force_busy = true;
        CHECK(web_auth_hash("admin", salt, hash, 0) == false);
        /* Untouched: web_auth_hash() must not write a partial/torn hash on
         * the busy path — this is what a real cross-core collision on the
         * target relies on (the loser must never publish garbage). */
        uint8_t untouched[WEB_AUTH_HASH_LEN];
        memset(untouched, 0xAA, sizeof untouched);
        CHECK(memcmp(hash, untouched, sizeof hash) == 0);
        web_auth_test_force_busy = false;
        /* Freed again: the SAME call now succeeds — proves "busy" was a
         * transient contention signal, not a permanently wedged lock. */
        CHECK(web_auth_hash("admin", salt, hash, 0) == true);
    }

    g_case = "fast_digest: same user:pass -> same digest, constant-time-equal";
    {
        uint8_t d1[WEB_AUTH_HASH_LEN], d2[WEB_AUTH_HASH_LEN];
        web_auth_fast_digest("admin", "hunter2", d1);
        web_auth_fast_digest("admin", "hunter2", d2);
        CHECK(web_auth_verify_equal(d1, d2));
    }

    g_case = "fast_digest: different password -> different digest";
    {
        uint8_t d1[WEB_AUTH_HASH_LEN], d2[WEB_AUTH_HASH_LEN];
        web_auth_fast_digest("admin", "hunter2", d1);
        web_auth_fast_digest("admin", "hunter3", d2);
        CHECK(!web_auth_verify_equal(d1, d2));
    }

    g_case = "fast_digest: different user, same password -> different digest";
    {
        /* Binds the cache to the WHOLE "user:pass" candidate, not just the
         * password — a username change with an unchanged password must miss
         * the fast-path cache too (http_server.c's check_auth() relies on
         * this: it never separately re-checks the username against a stale
         * cache hit). */
        uint8_t d1[WEB_AUTH_HASH_LEN], d2[WEB_AUTH_HASH_LEN];
        web_auth_fast_digest("admin", "hunter2", d1);
        web_auth_fast_digest("root",  "hunter2", d2);
        CHECK(!web_auth_verify_equal(d1, d2));
    }

    g_case = "hex: encode/decode roundtrip";
    {
        uint8_t raw[WEB_AUTH_HASH_LEN];
        for (size_t i = 0; i < sizeof raw; i++) raw[i] = (uint8_t)(i * 7 + 3);
        char hex[2 * WEB_AUTH_HASH_LEN + 1];
        web_auth_hex_encode(raw, sizeof raw, hex);
        CHECK(strlen(hex) == 2 * WEB_AUTH_HASH_LEN);
        uint8_t back[WEB_AUTH_HASH_LEN] = { 0 };
        CHECK(web_auth_hex_decode(hex, back, sizeof back));
        CHECK(memcmp(raw, back, sizeof raw) == 0);
    }

    g_case = "hex: decode rejects wrong length and non-hex content";
    {
        uint8_t out[4];
        CHECK(!web_auth_hex_decode("aabbcc", out, 4));        /* too short (6 chars, need 8) */
        CHECK(!web_auth_hex_decode("aabbccddee", out, 4));    /* too long */
        CHECK(!web_auth_hex_decode("aabbccdg", out, 4));      /* 'g' not hex */
        CHECK(web_auth_hex_decode("aabbccdd", out, 4));       /* exactly right */
    }

    g_case = "hex_valid: charset + exact-length check (config.c's CFG_HEX gate)";
    {
        CHECK(web_auth_hex_valid("0123456789abcdef", 8));
        CHECK(web_auth_hex_valid("0123456789ABCDEF", 8));     /* uppercase accepted */
        CHECK(!web_auth_hex_valid("0123456789abcde", 8));     /* one short */
        CHECK(!web_auth_hex_valid("0123456789abcdef0", 8));   /* one long */
        CHECK(!web_auth_hex_valid("0123456789abcdeg", 8));    /* 'g' not hex */
        CHECK(!web_auth_hex_valid("", 8));                    /* empty */
        CHECK(web_auth_hex_valid("", 0));                     /* zero-length field, degenerate but valid */
    }

    g_case = "wipe: safe to call, buffer still usable afterwards";
    {
        char buf[16] = "sensitive-data!";
        web_auth_wipe(buf, sizeof buf);
        /* Not asserting the exact zeroed content (crypto_wipe's contract is
         * "the compiler can't optimise this away", not a documented output
         * value this test should pin) — just that the call is safe and the
         * buffer is still writable/comparable afterwards. */
        strcpy(buf, "ok");
        CHECK(strcmp(buf, "ok") == 0);
    }

    if (g_fails == 0) printf("PASS: all web_auth checks passed\n");
    else              printf("%d CHECK(s) FAILED\n", g_fails);
    return g_fails ? 1 : 0;
}
