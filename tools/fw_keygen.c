/*
 * fw_keygen.c — host-side ed25519 keypair generator for the OTA image
 * signing scheme (see src/fw_image.h / tools/fw_sign.c). Links straight
 * against the same vendored Monocypher used on-device and by fw_sign —
 * no crypto is reimplemented here.
 *
 * Output:
 *   - writes the 64-byte "expanded" secret key (seed || public_key, the
 *     layout crypto_ed25519_key_pair produces and crypto_ed25519_sign
 *     consumes) to <privkey_out> — KEEP THIS FILE SECRET, never commit it.
 *   - prints the 32-byte public key to stdout, both as raw hex and as a
 *     ready-to-paste C array literal for src/fw_image.h's FW_PUBKEY[32].
 *
 * Usage:
 *   fw_keygen <privkey_out.bin>
 *
 * Build:
 *   cc -Wall -Wextra -I ../lib/monocypher -o fw_keygen fw_keygen.c \
 *       ../lib/monocypher/monocypher.c ../lib/monocypher/monocypher-ed25519.c
 */
#include "monocypher.h"
#include "monocypher-ed25519.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Host-only: reads randomness from the OS CSPRNG for the 32-byte seed.
 * Never used on the RP2040 (this file never builds into the firmware). */
static int fill_random(uint8_t *buf, size_t len) {
    FILE *f = fopen("/dev/urandom", "rb");
    if (!f) return 0;
    size_t n = fread(buf, 1, len, f);
    fclose(f);
    return n == len;
}

int main(int argc, char **argv) {
    if (argc != 2) {
        fprintf(stderr, "usage: %s <privkey_out.bin>\n", argv[0]);
        return 2;
    }

    uint8_t seed[32];
    if (!fill_random(seed, sizeof seed)) {
        fprintf(stderr, "error: could not read randomness from /dev/urandom\n");
        return 1;
    }

    uint8_t sk[64];   /* expanded secret key: seed || public_key */
    uint8_t pk[32];
    crypto_ed25519_key_pair(sk, pk, seed);   /* wipes seed internally */

    FILE *out = fopen(argv[1], "wb");
    if (!out) {
        fprintf(stderr, "error: could not open %s for writing\n", argv[1]);
        crypto_wipe(sk, sizeof sk);
        return 1;
    }
    size_t written = fwrite(sk, 1, sizeof sk, out);
    fclose(out);
    if (written != sizeof sk) {
        fprintf(stderr, "error: short write to %s\n", argv[1]);
        crypto_wipe(sk, sizeof sk);
        return 1;
    }

    printf("Private key (64 bytes, seed||pubkey) written to: %s\n", argv[1]);
    printf("KEEP THIS FILE SECRET. Do not commit it to git.\n\n");

    printf("Public key (hex): ");
    for (int i = 0; i < 32; i++) printf("%02x", pk[i]);
    printf("\n\n");

    printf("Public key as a C array literal for src/fw_image.h's FW_PUBKEY[32]:\n");
    printf("const uint8_t FW_PUBKEY[32] = {\n");
    for (int i = 0; i < 32; i += 8) {
        printf("   ");
        for (int j = i; j < i + 8 && j < 32; j++) printf(" 0x%02X,", pk[j]);
        printf("\n");
    }
    printf("};\n");

    crypto_wipe(sk, sizeof sk);
    return 0;
}
