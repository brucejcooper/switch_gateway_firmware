/*
 * fw_sign_e2e_test.c — end-to-end host test proving tools/fw_sign.c and the
 * on-device verifier src/fw_image.c agree: it invokes the actual compiled
 * `fw_sign` CLI (not a reimplementation) against tools/dev_privkey.bin to
 * sign a fake app image, then links the REAL src/fw_image.c — compiled with
 * its real baked-in FW_PUBKEY (the public half of dev_privkey.bin) — and
 * calls the real fw_verify() over the signed output.
 *
 * Not part of the firmware build (see CLAUDE.md: host tests live under
 * test/ and are never added to the RP2040 CMake target).
 *
 * Build & run (run from the test/ directory; builds fw_sign first, then
 * this test, then executes it):
 *   cc -Wall -Wextra -I ../src -I ../lib/monocypher \
 *       -o /tmp/fw_sign ../tools/fw_sign.c ../src/fw_image.c \
 *       ../lib/monocypher/monocypher.c ../lib/monocypher/monocypher-ed25519.c \
 *   && cc -Wall -Wextra -I ../src -I ../lib/monocypher \
 *       -o /tmp/fw_sign_e2e_test fw_sign_e2e_test.c ../src/fw_image.c \
 *       ../lib/monocypher/monocypher.c ../lib/monocypher/monocypher-ed25519.c \
 *   && /tmp/fw_sign_e2e_test
 *
 * (tools/dev_privkey.bin must already exist — generated once via
 * `tools/fw_keygen dev_privkey.bin`; see tools/README.md.)
 */
#include "../src/fw_image.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int fail = 0;
#define CHECK(cond) do { \
        if (!(cond)) { \
            printf("FAIL: %s (%s:%d)\n", #cond, __FILE__, __LINE__); \
            fail = 1; \
        } \
    } while (0)

#define FAKE_APP_PATH   "/tmp/fw_sign_e2e_fake_app.bin"
#define SIGNED_PATH     "/tmp/fw_sign_e2e_signed.bin"
#define PRIVKEY_PATH    "../tools/dev_privkey.bin"
#define FW_SIGN_BIN     "/tmp/fw_sign"

static uint8_t *read_whole_file(const char *path, long *out_len) {
    FILE *f = fopen(path, "rb");
    if (!f) return NULL;
    fseek(f, 0, SEEK_END);
    long len = ftell(f);
    fseek(f, 0, SEEK_SET);
    uint8_t *buf = malloc((size_t)len);
    if (!buf) { fclose(f); return NULL; }
    size_t n = fread(buf, 1, (size_t)len, f);
    fclose(f);
    if (n != (size_t)len) { free(buf); return NULL; }
    *out_len = len;
    return buf;
}

int main(void) {
    /* (1) Write a deterministic fake "app" image — content doesn't matter,
     * fw_sign/fw_verify only care about it being some byte sequence. */
    {
        FILE *f = fopen(FAKE_APP_PATH, "wb");
        CHECK(f != NULL);
        if (f) {
            uint8_t buf[3000];
            for (size_t i = 0; i < sizeof buf; i++) buf[i] = (uint8_t)(i * 17u + 5u);
            CHECK(fwrite(buf, 1, sizeof buf, f) == sizeof buf);
            fclose(f);
        }
    }

    /* (2) Invoke the REAL fw_sign CLI (built at /tmp/fw_sign per the header
     * comment's build command) against the dev private key. build_ctr is
     * just an arbitrary positive test value here — this test is about
     * signing/verification agreement, not the anti-rollback logic itself
     * (see test/ota_stage_test.c / test/boot_commit_test.c for that). */
    {
        char cmd[512];
        snprintf(cmd, sizeof cmd, "%s %s %s %s %s", FW_SIGN_BIN, FAKE_APP_PATH,
                  PRIVKEY_PATH, "7", SIGNED_PATH);
        int rc = system(cmd);
        CHECK(rc == 0);
    }

    /* (3) Read the signed artifact back and run the REAL fw_verify() (real
     * src/fw_image.c, real baked-in FW_PUBKEY = dev_privkey.bin's public
     * half) over it. Must accept. */
    long total_len = 0;
    uint8_t *signed_buf = read_whole_file(SIGNED_PATH, &total_len);
    CHECK(signed_buf != NULL);
    if (signed_buf) {
        CHECK(fw_verify(signed_buf, (uint32_t)total_len, FW_PUBKEY) == true);

        /* (4) Corrupt one byte of the image portion and confirm fw_verify()
         * now rejects it (proves the tool's output is actually checked
         * end-to-end, not just structurally well-formed). */
        uint8_t *tampered = malloc((size_t)total_len);
        CHECK(tampered != NULL);
        if (tampered) {
            memcpy(tampered, signed_buf, (size_t)total_len);
            tampered[0] ^= 0xFF;
            CHECK(fw_verify(tampered, (uint32_t)total_len, FW_PUBKEY) == false);
            free(tampered);
        }
        free(signed_buf);
    }

    if (fail) {
        printf("FAILED\n");
        return 1;
    }
    printf("all fw_sign_e2e tests passed\n");
    return 0;
}
