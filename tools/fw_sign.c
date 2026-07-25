/*
 * fw_sign.c — host-side OTA image signer. Reads a raw app .bin, appends a
 * struct fw_trailer (see src/fw_image.h — the SAME struct/offsets are
 * reused here, not re-derived), and writes <in>.signed.bin, an artifact
 * that src/fw_image.c's fw_verify() accepts as-is.
 *
 * Wire format / signing convention (must match fw_image.h exactly):
 *   [image bytes][struct fw_trailer]
 *   trailer.magic    = FW_TRAILER_MAGIC
 *   trailer.img_len  = size of the input .bin
 *   trailer.sha512   = crypto_sha512(image bytes)
 *   trailer.sig      = crypto_ed25519_sign(privkey, magic|img_len|sha512)
 *                       i.e. over the trailer's own bytes up to (not
 *                       including) sig itself — offsetof(fw_trailer, sig)
 *                       bytes, straight off the in-memory struct, exactly
 *                       as fw_verify() re-derives it on the device side.
 *
 * Usage:
 *   fw_sign <in.bin> <privkey.bin(64)> <out.signed.bin>
 *
 * privkey.bin is the 64-byte "expanded" secret key (seed || public_key)
 * produced by fw_keygen / crypto_ed25519_key_pair — the same layout
 * crypto_ed25519_sign expects.
 *
 * Build:
 *   cc -Wall -Wextra -I ../src -I ../lib/monocypher -o fw_sign fw_sign.c \
 *       ../src/fw_image.c ../lib/monocypher/monocypher.c \
 *       ../lib/monocypher/monocypher-ed25519.c
 */
#include "../src/fw_image.h"
#include "monocypher.h"
#include "monocypher-ed25519.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>

static uint8_t *read_whole_file(const char *path, long *out_len) {
    FILE *f = fopen(path, "rb");
    if (!f) return NULL;
    if (fseek(f, 0, SEEK_END) != 0) { fclose(f); return NULL; }
    long len = ftell(f);
    if (len < 0) { fclose(f); return NULL; }
    if (fseek(f, 0, SEEK_SET) != 0) { fclose(f); return NULL; }

    uint8_t *buf = malloc((size_t)len);
    if (!buf) { fclose(f); return NULL; }
    size_t n = fread(buf, 1, (size_t)len, f);
    fclose(f);
    if (n != (size_t)len) { free(buf); return NULL; }

    *out_len = len;
    return buf;
}

int main(int argc, char **argv) {
    if (argc != 4) {
        fprintf(stderr, "usage: %s <in.bin> <privkey.bin(64)> <out.signed.bin>\n", argv[0]);
        return 2;
    }
    const char *in_path  = argv[1];
    const char *key_path = argv[2];
    const char *out_path = argv[3];

    long img_len_l = 0;
    uint8_t *img = read_whole_file(in_path, &img_len_l);
    if (!img) {
        fprintf(stderr, "error: could not read input image %s\n", in_path);
        return 1;
    }
    if (img_len_l <= 0 || img_len_l > (long)UINT32_MAX) {
        fprintf(stderr, "error: input image size %ld out of range\n", img_len_l);
        free(img);
        return 1;
    }
    uint32_t img_len = (uint32_t)img_len_l;

    long key_len_l = 0;
    uint8_t *sk = read_whole_file(key_path, &key_len_l);
    if (!sk) {
        fprintf(stderr, "error: could not read private key %s\n", key_path);
        free(img);
        return 1;
    }
    if (key_len_l != 64) {
        fprintf(stderr, "error: private key %s must be exactly 64 bytes (got %ld)\n",
                key_path, key_len_l);
        free(img);
        free(sk);
        return 1;
    }

    /* Build the trailer exactly per fw_image.h's convention. */
    struct fw_trailer t;
    t.magic   = FW_TRAILER_MAGIC;
    t.img_len = img_len;
    crypto_sha512(t.sha512, img, img_len);
    crypto_ed25519_sign(t.sig, sk, (const uint8_t *)&t, offsetof(struct fw_trailer, sig));

    FILE *out = fopen(out_path, "wb");
    if (!out) {
        fprintf(stderr, "error: could not open %s for writing\n", out_path);
        free(img);
        crypto_wipe(sk, (size_t)key_len_l);
        free(sk);
        return 1;
    }
    size_t w1 = fwrite(img, 1, img_len, out);
    size_t w2 = fwrite(&t, 1, sizeof t, out);
    fclose(out);

    crypto_wipe(sk, (size_t)key_len_l);
    free(sk);

    if (w1 != img_len || w2 != sizeof t) {
        fprintf(stderr, "error: short write to %s\n", out_path);
        free(img);
        return 1;
    }

    printf("signed: %s (%u bytes image + %zu byte trailer = %lu bytes total) -> %s\n",
           in_path, img_len, sizeof t, (unsigned long)(img_len + sizeof t), out_path);

    free(img);
    return 0;
}
