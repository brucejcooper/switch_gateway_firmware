/*
 * web_auth.c — Argon2id KDF for the web-UI password, its hex codec, and the
 * thin Monocypher wrappers (web_auth.h explains why they're funnelled
 * through here rather than every caller including monocypher.h directly).
 *
 * ── Sizing the work area ─────────────────────────────────────────────────
 * Checked against build/light_switch.elf.map before this change: .data
 * 0x1d78 (7,544 B) + .bss 0x16df8 (93,688 B) + the 2 KB .heap reservation =
 * ~103,280 B of the RP2040's 256 KB (262,144 B) general-RAM region — about
 * 158,864 B (~155 KB) of headroom before this file adds anything. A 32 KB
 * work area (WEB_AUTH_NB_BLOCKS=32, 1 KiB/block per the Argon2 spec) spends
 * about a fifth of that headroom and leaves ~123 KB free — comfortably
 * inside the "prefer 32 KB if comfortably available, floor 8" budget, so the
 * fallback (salted iterated BLAKE2b) was not needed. nb_passes=3 is
 * Monocypher's own stated floor for Argon2i/Argon2id CPU hardness;
 * nb_lanes=1 because Monocypher runs every lane on a single thread
 * regardless (see its own comment above the loop in crypto_argon2(),
 * monocypher.c) — more lanes would only shrink the per-lane memory for the
 * same 32 KB total, not add real parallelism, so lane count stays at
 * Monocypher's minimum. CRYPTO_ARGON2_ID (Argon2id) is used because
 * monocypher.h (this vendored 4.x) exposes algorithm selection.
 *
 * ── The shared-work-area lock (see web_auth.h's header for who calls what) ─
 * Exactly one static work area exists, used by every KDF call on EITHER
 * core. It is guarded by ONE RP2040 hardware spinlock, claimed once via
 * spin_lock_claim_unused() so nothing else in the SDK can collide with it.
 * spin_try_lock_unsafe()/spin_unlock_unsafe() (not the blocking/IRQ-safe
 * spin_lock_blocking()) are used deliberately: an Argon2 pass over 32 KB
 * takes tens to a few hundred milliseconds on a 133 MHz Cortex-M0+, which is
 * NOT the "very short lived" critical section hardware spinlocks are meant
 * for (see the SDK's own comment on spin_lock_unsafe_blocking()) — holding
 * one that long WITH interrupts disabled would stall whichever core holds
 * it far longer than acceptable. The "unsafe" variants only provide mutual
 * exclusion between the two cores' access to the shared buffer; they do not
 * touch PRIMASK, so neither core's own interrupt-driven work stalls while a
 * hash runs.
 *
 * Boot-time (config_load()'s legacy-password migration) needs no retry at
 * all: core 1 is parked in core1_entry()'s `while (!s_config_ready)` loop
 * from core1_launch() until core1_config_ready() (main.c), which is called
 * long after config_load() returns — core 1 cannot possibly be inside a KDF
 * call yet, so the lock is uncontended by construction at that point, and
 * migration calls web_auth_hash() with retry_ms=0 (a single attempt) purely
 * for API uniformity, not because it might need to wait.
 *
 * Steady-state, two call sites can contend the lock, both rare-by-nature:
 *   - core 0's IC_CMD_SET_AUTH handler (ic_seam.c) hashes a new password for
 *     PERSISTENCE, once per password change.
 *   - core 1's POST /api/auth handler (http_server.c) hashes the SAME new
 *     password again, once, to update its own live credential immediately —
 *     these two are triggered by the same user action and so are the most
 *     likely pair to actually collide in practice.
 *   - core 1's check_auth() SLOW PATH (cache miss) verifies a candidate —
 *     but only on the first request after boot or after a credential
 *     change; every other request hits the BLAKE2b fast-path cache and never
 *     touches the lock at all (see web_auth_fast_digest() and
 *     http_server.c's check_auth()).
 * Each caller picks its own retry budget for what it can afford to block:
 * the two setters (persistence and live-update) pass a bounded budget (a
 * couple of seconds is generous against a KDF call that takes at most a few
 * hundred milliseconds) so a same-instant collision self-resolves instead of
 * failing a user-initiated password change; check_auth()'s slow path passes
 * 0 (a single non-blocking attempt) because it runs on every request and
 * must never stall the core-1 tick loop — losing the race there just fails
 * THIS ONE request closed (a 401, not a 200 for a wrong password — see
 * check_auth()'s own comment), and the very next poll (~1.5 s later, or
 * immediately if it's a human retrying) tries again.
 */
#include "web_auth.h"
#include "monocypher.h"
#include <string.h>
#include <stdio.h>

/* See the file header for the sizing rationale. */
#define WEB_AUTH_NB_BLOCKS 32u   /* 32 * 1024 = 32,768-byte work area */
#define WEB_AUTH_NB_PASSES 3u
#define WEB_AUTH_NB_LANES  1u
static uint8_t s_work_area[WEB_AUTH_NB_BLOCKS * 1024u];

/* Run one Argon2id pass into `hash` using the shared work area; wipes the
 * work area afterwards (it holds intermediate material derived from the
 * candidate password, same hygiene reason every plaintext gets wiped).
 * Shared by both the target and host-test lock implementations below. */
static void kdf_run(const char *password, const uint8_t salt[WEB_AUTH_SALT_LEN],
                    uint8_t hash[WEB_AUTH_HASH_LEN]) {
    crypto_argon2_config cfg = {
        .algorithm = CRYPTO_ARGON2_ID,
        .nb_blocks = WEB_AUTH_NB_BLOCKS,
        .nb_passes = WEB_AUTH_NB_PASSES,
        .nb_lanes  = WEB_AUTH_NB_LANES,
    };
    crypto_argon2_inputs in = {
        .pass      = (const uint8_t *)password,
        .salt      = salt,
        .pass_size = (uint32_t)strlen(password),
        .salt_size = WEB_AUTH_SALT_LEN,
    };
    crypto_argon2(hash, WEB_AUTH_HASH_LEN, s_work_area, cfg, in, crypto_argon2_no_extras);
    crypto_wipe(s_work_area, sizeof s_work_area);
}

#ifndef WEB_AUTH_HOST_TEST
#include "pico/rand.h"
#include "hardware/sync.h"
#include "hardware/sync/spin_lock.h"
#include "pico/time.h"

static spin_lock_t *s_lock;

void web_auth_init(void) {
    int n = spin_lock_claim_unused(true);
    s_lock = spin_lock_init((uint)n);
}

void web_auth_gen_salt(uint8_t salt[WEB_AUTH_SALT_LEN]) {
    uint64_t a = get_rand_64(), b = get_rand_64();
    memcpy(salt,     &a, 8);
    memcpy(salt + 8, &b, 8);
    web_auth_wipe(&a, sizeof a);
    web_auth_wipe(&b, sizeof b);
}

static bool kdf_try(const char *password, const uint8_t salt[WEB_AUTH_SALT_LEN],
                    uint8_t hash[WEB_AUTH_HASH_LEN]) {
    if (!spin_try_lock_unsafe(s_lock)) return false;
    kdf_run(password, salt, hash);
    spin_unlock_unsafe(s_lock);
    return true;
}

bool web_auth_hash(const char *password, const uint8_t salt[WEB_AUTH_SALT_LEN],
                   uint8_t hash[WEB_AUTH_HASH_LEN], uint32_t retry_ms) {
    if (kdf_try(password, salt, hash)) return true;
    if (retry_ms == 0) return false;
    absolute_time_t dl = make_timeout_time_ms(retry_ms);
    while (absolute_time_diff_us(get_absolute_time(), dl) > 0) {
        if (kdf_try(password, salt, hash)) return true;
    }
    return false;
}
#else
/* Host-test seam (matches src/boot_control.c's BOOT_CONTROL_HOST_TEST
 * pattern): no real cross-core contention exists on the host (one thread),
 * so a plain flag stands in for the hardware spinlock, and get_rand_64()
 * (pico_rand, target-only) is stood in by rand() — NOT cryptographically
 * secure, but salt uniqueness/format is all a host test needs to pin; no
 * target build ever takes this path. web_auth_test_force_busy is a test
 * hook (test/web_auth_test.c) that proves a caller which loses the race
 * gets `false` back rather than a torn/partial hash. */
#include <stdlib.h>
static bool s_locked = false;
bool web_auth_test_force_busy = false;

void web_auth_init(void) { s_locked = false; }

void web_auth_gen_salt(uint8_t salt[WEB_AUTH_SALT_LEN]) {
    for (size_t i = 0; i < WEB_AUTH_SALT_LEN; i++) salt[i] = (uint8_t)rand();
}

static bool kdf_try(const char *password, const uint8_t salt[WEB_AUTH_SALT_LEN],
                    uint8_t hash[WEB_AUTH_HASH_LEN]) {
    if (s_locked || web_auth_test_force_busy) return false;
    s_locked = true;
    kdf_run(password, salt, hash);
    s_locked = false;
    return true;
}

bool web_auth_hash(const char *password, const uint8_t salt[WEB_AUTH_SALT_LEN],
                   uint8_t hash[WEB_AUTH_HASH_LEN], uint32_t retry_ms) {
    (void)retry_ms;   /* single thread on the host: a retry can never help */
    return kdf_try(password, salt, hash);
}
#endif

void web_auth_fast_digest(const char *user, const char *password,
                          uint8_t digest[WEB_AUTH_HASH_LEN]) {
    /* Sized generously for the real fields (web_user[32] + ':' + a
     * candidate password up to ic_cmd_t.set_auth.pass's 47 usable chars). */
    char combined[96];
    int n = snprintf(combined, sizeof combined, "%s:%s", user, password);
    if (n < 0) n = 0;
    if (n > (int)sizeof combined) n = (int)sizeof combined;
    crypto_blake2b(digest, WEB_AUTH_HASH_LEN, (const uint8_t *)combined, (size_t)n);
    web_auth_wipe(combined, sizeof combined);
}

bool web_auth_verify_equal(const uint8_t a[WEB_AUTH_HASH_LEN], const uint8_t b[WEB_AUTH_HASH_LEN]) {
    return crypto_verify32(a, b) == 0;
}

void web_auth_wipe(void *p, size_t n) {
    crypto_wipe(p, n);
}

/* ── hex codec ───────────────────────────────────────────────────────────*/

static int hex_nibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

void web_auth_hex_encode(const uint8_t *raw, size_t n, char *out) {
    static const char hex[] = "0123456789abcdef";
    for (size_t i = 0; i < n; i++) {
        out[2 * i]     = hex[(raw[i] >> 4) & 0xF];
        out[2 * i + 1] = hex[raw[i] & 0xF];
    }
    out[2 * n] = '\0';
}

bool web_auth_hex_decode(const char *hex, uint8_t *out, size_t n) {
    for (size_t i = 0; i < n; i++) {
        int hi = hex_nibble(hex[2 * i]);
        int lo = hi < 0 ? -1 : hex_nibble(hex[2 * i + 1]);
        if (hi < 0 || lo < 0) return false;
        out[i] = (uint8_t)((hi << 4) | lo);
    }
    return hex[2 * n] == '\0';
}

bool web_auth_hex_valid(const char *hex, size_t explen) {
    size_t i = 0;
    for (; hex[i] != '\0'; i++) {
        if (i >= 2 * explen) return false;
        if (hex_nibble(hex[i]) < 0) return false;
    }
    return i == 2 * explen;
}
