# OTA signing tools

Host-side tools for the pull-based OTA scheme (see `src/fw_image.h`,
`src/net/ota.c`, `src/net/fwupdate.c`). None of these build with the RP2040
firmware (`make -C build`) — they're plain host binaries/scripts, built with
the macOS/Linux system `cc`, and link the same vendored
`lib/monocypher/{monocypher.c,monocypher-ed25519.c}` the firmware uses so the
crypto is identical on both sides.

## Files

- `fw_keygen.c` — generates an ed25519 keypair. Writes the 64-byte private
  key (seed||pubkey) to a file, prints the 32-byte public key as hex and as
  a C array literal ready to paste into `src/fw_image.h`'s `FW_PUBKEY[32]`.
- `fw_sign.c` — signs a raw app `.bin`: computes SHA-512, builds a
  `struct fw_trailer` (reusing `src/fw_image.h`'s struct/offsets directly —
  it does not re-derive the layout), signs `magic|img_len|sha512` with
  ed25519, appends the trailer, writes `<out>.signed.bin`. This is the exact
  artifact `src/fw_image.c`'s `fw_verify()` accepts.
- `fw_manifest.sh` — given a version string + a signed `.bin` + the URL it
  will be published at, emits the manifest JSON (`version`, `board`, `url`,
  `size`, `sha256`, `title`, `release_url`, `notes`) matching the shape
  `src/net/fwupdate.h` documents/parses.
- `dev_privkey.bin` — a **development-only** ed25519 private key generated
  once with `fw_keygen` (2026-07-05). **Gitignored — never commit it.** Its
  matching public key is baked into `src/fw_image.h`/`fw_image.c`'s
  `FW_PUBKEY[32]` (clearly commented as a dev key). Rotate before shipping
  to production: run `fw_keygen` again, replace `FW_PUBKEY[32]`, reflash
  every device that must trust the new key.

## Building the tools

```sh
cd tools
cc -Wall -Wextra -I ../lib/monocypher -o fw_keygen fw_keygen.c \
    ../lib/monocypher/monocypher.c ../lib/monocypher/monocypher-ed25519.c

cc -Wall -Wextra -I ../src -I ../lib/monocypher -o fw_sign fw_sign.c \
    ../src/fw_image.c ../lib/monocypher/monocypher.c \
    ../lib/monocypher/monocypher-ed25519.c
```

`fw_manifest.sh` needs no build (needs `shasum` or `sha256sum` on PATH,
present on macOS/Linux by default).

## Generating a (new) signing key

Only needed once per key generation/rotation — do **not** run this for every
release, that would invalidate `FW_PUBKEY[32]` on every already-flashed
device.

```sh
./fw_keygen new_privkey.bin
# paste the printed FW_PUBKEY[32] array into src/fw_image.h, replacing the
# existing one, and update its comment to say which key/date it is.
```

## Release flow: build -> sign -> manifest -> publish

```sh
# 1. Build the RELOCATED app (opt-in target, links at APP_OFF/0x10010000
#    with no boot2 — see ../bootloader/README.md's "The relocated app build"
#    section for why this, and NOT plain `light_switch`, is the correct
#    input here: `light_switch` stays linked at the standalone 0x10000000
#    origin so it's still directly flashable/boot2-at-0x0 on its own).
cmake -S .. -B ../build_ota -DOTA_APP_LAYOUT=ON
make -C ../build_ota light_switch_ota
arm-none-eabi-objcopy -O binary ../build_ota/light_switch_ota.elf ../build_ota/light_switch_ota.bin

# 2. Sign it with the dev (or production) private key
./fw_sign ../build_ota/light_switch_ota.bin dev_privkey.bin ../build_ota/light_switch_ota.signed.bin

# 3. Generate the manifest
./fw_manifest.sh 2.1.0 ../build_ota/light_switch_ota.signed.bin \
    http://ota.8bitcloud.com/light_switch.bin \
    "Light Switch 2.1.0" \
    https://example.com/releases/2.1.0 \
    "Release notes here" \
    > ../build_ota/light_switch.json

# 4. Publish both files at the URLs fwupdate.c polls:
#      http://ota.8bitcloud.com/light_switch.bin   <- build_ota/light_switch_ota.signed.bin
#      http://ota.8bitcloud.com/light_switch.json  <- build_ota/light_switch.json
```

`fwupdate.c` on the device polls `light_switch.json`, compares `version`
against its own `FW_VERSION`, and if newer, downloads `url` into staging;
`fw_verify()` (baked-in `FW_PUBKEY`) is the actual trust check performed
on-device before the bootloader will ever mark the staged image bootable —
the manifest's `sha256` field is informational only, not part of that trust
chain.

## Testing

`test/fw_sign_e2e_test.c` proves the tool and the on-device verifier agree:
it builds a fake app `.bin`, shells out to a built `fw_sign` (or links the
same signing logic — see that file's header comment) using
`tools/dev_privkey.bin`, then runs the real `src/fw_image.c` `fw_verify()`
compiled with the real baked-in `FW_PUBKEY` over the signed output and
asserts `true`, then corrupts one byte and asserts `false`.
