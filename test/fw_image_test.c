/*
 * fw_image_test.c — host-side test for src/fw_image.c (OTA image trailer
 * verification: SHA-512 + standard RFC 8032 Ed25519 via vendored
 * Monocypher).
 *
 * Not part of the firmware build (see CLAUDE.md: host tests live under
 * test/ and are never added to the RP2040 CMake target). Links straight
 * against the real vendored lib/monocypher/monocypher.c,
 * lib/monocypher/monocypher-ed25519.c and src/fw_image.c — no crypto is
 * reimplemented here, only exercised.
 *
 * lib/monocypher's CORE monocypher.c/.h (verified byte-identical to the
 * upstream v4.0.2 release) does not expose crypto_sha512 or standard
 * Ed25519 — those live in Monocypher's OPTIONAL extension pair
 * monocypher-ed25519.{c,h}, official upstream files from the very same
 * v4.0.2 release, now vendored verbatim alongside the core library (see
 * fw_image.h's header comment for the full rationale). fw_image.c uses
 * crypto_sha512 for the digest and crypto_ed25519_sign/_check for the
 * signature — literal, interoperable RFC 8032 Ed25519.
 *
 * Test (a) below is a known-answer test against REAL, PUBLISHED test
 * vectors (not values derived from this code): NIST's SHA-512("abc")
 * vector, and RFC 8032 Section 7.1 "TEST 2" for Ed25519 (fixed seed,
 * public key, one-byte message, and expected signature). Both prove the
 * vendored primitives are standard-conformant, independent of anything
 * fw_image.c does. Tests (b)/(c) cover fw_verify()'s own logic (valid
 * image accepted; corrupted image/hash/signature rejected) using a locally
 * generated keypair, exactly as specified.
 *
 * Build & run:
 *   cc -Wall -Wextra -I ../src -I ../lib/monocypher \
 *       -o /tmp/fw_image_test fw_image_test.c ../src/fw_image.c \
 *       ../lib/monocypher/monocypher.c ../lib/monocypher/monocypher-ed25519.c \
 *       && /tmp/fw_image_test
 */
#include "../src/fw_image.h"
#include "monocypher.h"
#include "monocypher-ed25519.h"

#include <stdio.h>
#include <string.h>
#include <stddef.h>

static int fail = 0;
#define CHECK(cond) do { \
        if (!(cond)) { \
            printf("FAIL: %s (%s:%d)\n", #cond, __FILE__, __LINE__); \
            fail = 1; \
        } \
    } while (0)

/* ── helpers ─────────────────────────────────────────────────────────── */

static void hex_to_bytes(const char *hex, uint8_t *out, size_t out_len) {
    for (size_t i = 0; i < out_len; i++) {
        unsigned v;
        sscanf(hex + 2 * i, "%2x", &v);
        out[i] = (uint8_t)v;
    }
}

/* Builds a total_len = img_len + sizeof(struct fw_trailer) buffer: img_len
 * bytes of deterministic "image" content, followed by a correctly signed
 * trailer under (sk, pk). Caller-owned buf must be >= that total length. */
static uint32_t build_signed_image(uint8_t *buf, uint32_t img_len,
                                    const uint8_t sk[64], const uint8_t pk[32]) {
    for (uint32_t i = 0; i < img_len; i++)
        buf[i] = (uint8_t)(i * 31u + 7u);   /* arbitrary but deterministic */

    struct fw_trailer t;
    t.magic   = FW_TRAILER_MAGIC;
    t.img_len = img_len;
    crypto_sha512(t.sha512, buf, img_len);
    crypto_ed25519_sign(t.sig, sk, (const uint8_t *)&t, offsetof(struct fw_trailer, sig));
    (void)pk; /* pk is embedded in sk per crypto_ed25519_key_pair; kept for clarity/signature symmetry */

    memcpy(buf + img_len, &t, sizeof t);
    return img_len + (uint32_t)sizeof t;
}

/* ── (a) known-answer tests against REAL published test vectors ────────
 * (a1) SHA-512("abc") — the canonical NIST/FIPS 180-4 test vector. Proves
 * crypto_sha512 (the digest fw_image.c relies on) is standard SHA-512.
 * (a2) RFC 8032 Section 7.1 "TEST 2" for Ed25519 — a fixed 32-byte seed,
 * its corresponding public key, a 1-byte message, and the expected
 * signature, all lifted verbatim from the RFC. Proves crypto_ed25519_sign
 * reproduces the exact published signature bit-for-bit, and that
 * crypto_ed25519_check accepts it — i.e. the vendored Ed25519
 * implementation is standard RFC 8032 Ed25519, not a Monocypher-only
 * variant. */
static void test_known_answer_vectors(void) {
    /* (a1) SHA-512("abc") */
    {
        const uint8_t msg[3] = { 'a', 'b', 'c' };
        const char *expect_hex =
            "ddaf35a193617abacc417349ae20413"
            "112e6fa4e89a97ea20a9eeee64b55d39"
            "a2192992a274fc1a836ba3c23a3feebb"
            "d454d4423643ce80e2a9ac94fa54ca49f";
        uint8_t expect[64];
        hex_to_bytes(expect_hex, expect, sizeof expect);

        uint8_t got[64];
        crypto_sha512(got, msg, sizeof msg);

        CHECK(memcmp(got, expect, sizeof got) == 0);
    }

    /* (a2) RFC 8032 7.1 TEST 2 (Ed25519) */
    {
        const char *seed_hex =
            "4ccd089b28ff96da9db6c346ec114e0f"
            "5b8a319f35aba624da8cf6ed4fb8a6fb";
        const char *pk_hex =
            "3d4017c3e843895a92b70aa74d1b7ebc"
            "9c982ccf2ec4968cc0cd55f12af4660c";
        const uint8_t msg[1] = { 0x72 };
        const char *sig_hex =
            "92a009a9f0d4cab8720e820b5f642540"
            "a2b27b5416503f8fb3762223ebdb69da"
            "085ac1e43e15996e458f3613d0f11d8c"
            "387b2eaeb4302aeeb00d291612bb0c00";

        uint8_t seed[32], pk[32], expect_sig[64];
        hex_to_bytes(seed_hex, seed, sizeof seed);
        hex_to_bytes(pk_hex, pk, sizeof pk);
        hex_to_bytes(sig_hex, expect_sig, sizeof expect_sig);

        /* crypto_ed25519_sign wants the 64-byte "expanded" secret key
         * (seed || public_key), same layout crypto_ed25519_key_pair
         * produces; the RFC's 32-byte "SECRET KEY" is just the seed. */
        uint8_t sk[64];
        memcpy(sk, seed, 32);
        memcpy(sk + 32, pk, 32);

        uint8_t got_sig[64];
        crypto_ed25519_sign(got_sig, sk, msg, sizeof msg);
        CHECK(memcmp(got_sig, expect_sig, sizeof expect_sig) == 0);

        /* and the published signature must verify against the published
         * public key/message via crypto_ed25519_check */
        CHECK(crypto_ed25519_check(expect_sig, pk, msg, sizeof msg) == 0);
    }
}

/* ── (b) valid image + trailer -> fw_verify() true ──────────────────── */
static void test_valid_image_verifies(void) {
    uint8_t seed[32];
    for (int i = 0; i < 32; i++) seed[i] = (uint8_t)(i * 7 + 3);
    uint8_t sk[64], pk[32];
    crypto_ed25519_key_pair(sk, pk, seed);   /* seed is wiped by this call */

    uint8_t buf[4096];
    uint32_t img_len = 1000;
    uint32_t total_len = build_signed_image(buf, img_len, sk, pk);

    CHECK(fw_verify(buf, total_len, pk) == true);
}

/* ── (c) tamper detection: image / sha512 / sig each independently ─── */
static void test_tamper_detection(void) {
    uint8_t seed[32];
    for (int i = 0; i < 32; i++) seed[i] = (uint8_t)(i * 5 + 1);
    uint8_t sk[64], pk[32];
    crypto_ed25519_key_pair(sk, pk, seed);

    uint8_t buf[4096];
    uint32_t img_len = 500;
    uint32_t total_len = build_signed_image(buf, img_len, sk, pk);

    /* sanity: unmodified buffer verifies before each tamper sub-case */
    CHECK(fw_verify(buf, total_len, pk) == true);

    /* flip a byte in the image */
    {
        uint8_t tampered[4096];
        memcpy(tampered, buf, total_len);
        tampered[img_len / 2] ^= 0xFF;
        CHECK(fw_verify(tampered, total_len, pk) == false);
    }

    /* flip a byte in the trailer's sha512 field */
    {
        uint8_t tampered[4096];
        memcpy(tampered, buf, total_len);
        size_t sha_off = img_len + offsetof(struct fw_trailer, sha512);
        tampered[sha_off] ^= 0xFF;
        CHECK(fw_verify(tampered, total_len, pk) == false);
    }

    /* flip a byte in the trailer's sig field */
    {
        uint8_t tampered[4096];
        memcpy(tampered, buf, total_len);
        size_t sig_off = img_len + offsetof(struct fw_trailer, sig);
        tampered[sig_off] ^= 0xFF;
        CHECK(fw_verify(tampered, total_len, pk) == false);
    }

    /* bonus: wrong public key must also fail */
    {
        uint8_t wrong_pk[32];
        memcpy(wrong_pk, pk, sizeof wrong_pk);
        wrong_pk[0] ^= 0xFF;
        CHECK(fw_verify(buf, total_len, wrong_pk) == false);
    }

    /* bonus: corrupted magic must also fail */
    {
        uint8_t tampered[4096];
        memcpy(tampered, buf, total_len);
        size_t magic_off = img_len + offsetof(struct fw_trailer, magic);
        tampered[magic_off] ^= 0xFF;
        CHECK(fw_verify(tampered, total_len, pk) == false);
    }

    /* bonus: total_len shorter than the real artifact must fail */
    CHECK(fw_verify(buf, total_len - 1, pk) == false);
}

int main(void) {
    test_known_answer_vectors();
    test_valid_image_verifies();
    test_tamper_detection();

    if (fail) {
        printf("FAILED\n");
        return 1;
    }
    printf("all fw_image tests passed\n");
    return 0;
}
