#pragma once
/*
 * web_auth.h — the ONE file that knows about Monocypher on behalf of the
 * web-UI password: the Argon2id KDF (config.h's web_pass_hash/web_salt),
 * its hex codec, and thin constant-time-compare/wipe wrappers so every other
 * caller (config.c, ic_seam.c, net/http_server.c) can hash/verify/scrub a
 * credential without including monocypher.h itself. That containment is
 * deliberate, not just tidiness: the KDF's static work area (below) is a
 * single shared resource touched from BOTH cores, and keeping every access
 * to it behind one small API is what makes the concurrency argument
 * checkable in one place instead of scattered across three files.
 *
 * ── Why the work area is shared, not per-core ───────────────────────────
 * RP2040 has one physical address space — "core 0's RAM" and "core 1's RAM"
 * is not a thing, only "core 0's CODE" and "core 1's CODE" running
 * concurrently over the same memory. A 32 KB Argon2 work area costs 32 KB
 * once either way; giving each core its own copy would spend 64 KB to solve
 * a problem (concurrent access) that a lock solves for free. See web_auth.c
 * for the actual lock and the argument that a bounded try-lock is safe here.
 *
 * ── Call sites (who calls what, and when) ───────────────────────────────
 *   - config_load() (core 0, boot, single-threaded — see main.c: core 1 is
 *     parked until core1_config_ready(), called long after config_load()
 *     returns): migrates a legacy plaintext "web_pass" to a fresh hash+salt.
 *   - ic_seam.c's IC_CMD_SET_AUTH (core 0, main loop): hashes a new
 *     candidate password from POST /api/auth for PERSISTENCE.
 *   - net/http_server.c's POST /api/auth handler (core 1): hashes the SAME
 *     candidate again, separately, to update its own live in-RAM credential
 *     immediately (mirrors the pre-hash code's "apply now, core 0 persists
 *     for next boot" behaviour) — see that file for why a second hash (and
 *     therefore a second, different salt) is fine rather than plumbing the
 *     one core 0 computed back over the intercore queue.
 *   - net/http_server.c's check_auth() (core 1, EVERY request): verifies a
 *     candidate. Hot path — see its own comment for the BLAKE2b fast-path
 *     cache that keeps this off the KDF on all but the first request after
 *     a credential change.
 */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define WEB_AUTH_SALT_LEN 16   /* raw bytes */
#define WEB_AUTH_HASH_LEN 32   /* raw bytes */

/* Claim and initialise the shared spinlock. Call ONCE, on core 0, before
 * core1_launch() — i.e. before core 1 can possibly exist to race a KDF call.
 * (Everything else below is safe to call from either core after that.) */
void web_auth_init(void);

/* Fill salt[WEB_AUTH_SALT_LEN] with fresh random bytes. Call ONLY when
 * setting a NEW password — never to re-derive an existing one, or every
 * verify would need the caller to already know today's salt from somewhere
 * other than storage. */
void web_auth_gen_salt(uint8_t salt[WEB_AUTH_SALT_LEN]);

/* Derive hash[WEB_AUTH_HASH_LEN] = Argon2id(password, salt) using the ONE
 * shared static work area, under the shared try-lock (web_auth.c).
 * `password` is a NUL-terminated plaintext candidate — never stored, wipe it
 * yourself with web_auth_wipe() once done.
 *
 * Returns false (hash[] left untouched) if the work area is busy — a
 * concurrent call is in flight on the other core — and stays busy for the
 * whole of `retry_ms` (0 = a single non-blocking attempt, for a hot-path
 * caller that must never block; a caller that can afford to wait out the
 * rare cross-core collision passes a bounded budget, e.g. a second or two —
 * see the call sites listed above for which is which and why each choice is
 * safe for its own budget). Never blocks past `retry_ms`. */
bool web_auth_hash(const char *password, const uint8_t salt[WEB_AUTH_SALT_LEN],
                   uint8_t hash[WEB_AUTH_HASH_LEN], uint32_t retry_ms);

/* Cheap post-verification cache digest: BLAKE2b-256 of the candidate
 * "user:pass" bytes. Pure — no lock, no work area, microseconds not tens of
 * milliseconds — used by check_auth()'s fast path so a device that has
 * already run the full KDF once for the current credentials doesn't have to
 * run it again on every ~1.5 s web-UI poll. Lives here (not local to
 * http_server.c) so it can never quietly diverge from what web_auth_hash()
 * considers "the candidate". */
void web_auth_fast_digest(const char *user, const char *password,
                          uint8_t digest[WEB_AUTH_HASH_LEN]);

/* Constant-time equality (Monocypher's crypto_verify32, inverted to a normal
 * bool: true means equal). Use for EVERY hash/digest comparison a credential
 * check makes — a cheap-path early-out that behaves differently from the
 * slow path on a near-miss is exactly the kind of side channel this exists
 * to avoid. */
bool web_auth_verify_equal(const uint8_t a[WEB_AUTH_HASH_LEN], const uint8_t b[WEB_AUTH_HASH_LEN]);

/* Zero `n` bytes at `p` in a way the compiler cannot optimise away (thin
 * wrapper over Monocypher's crypto_wipe()). Call on every plaintext
 * candidate and KDF temporary once you're done with it. */
void web_auth_wipe(void *p, size_t n);

/* ── hex codec — web_pass_hash/web_salt are stored as lowercase hex text ───
 * Plain hex, no JSON escaping needed (the charset [0-9a-f] contains nothing
 * `json_escape()` would ever touch), but they flow through the same schema
 * string machinery as every other config field (config.c's CFG_HEX kind) so
 * config_load()/config_save() don't need a third code path. */

/* Encode `n` raw bytes as 2n lowercase hex chars + NUL into out[2n+1]. */
void web_auth_hex_encode(const uint8_t *raw, size_t n, char *out);

/* Decode exactly `n` bytes (2n hex chars, case-insensitive) from `hex` into
 * out[n]. Returns false — out left untouched — unless hex is EXACTLY 2n hex
 * digits followed by NUL (wrong length, a non-hex-digit byte, or a NUL
 * before 2n chars are all rejected, not silently short-read). */
bool web_auth_hex_decode(const char *hex, uint8_t *out, size_t n);

/* Charset+length check only (no output buffer needed) — config.c's CFG_HEX
 * schema kind uses this to validate a loaded/applied hex string in place.
 * explen is the RAW byte count the field is meant to hold (16 for a salt, 32
 * for a hash); `hex` must be exactly 2*explen hex digits, nothing more or
 * less. */
bool web_auth_hex_valid(const char *hex, size_t explen);
