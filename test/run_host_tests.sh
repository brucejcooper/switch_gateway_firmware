#!/usr/bin/env bash
#
# run_host_tests.sh — builds and runs every host-side test under test/.
#
# Each test/*_test.c file documents its own exact `cc ... && /tmp/<name>`
# build-and-run command in its header comment (see the file for the "why").
# This script does not parse those comments at runtime (fragile); instead
# the TESTS table below is a hand-transcribed, greppable mirror of each
# test's documented command — defines, include dirs, and the source files
# it links against. If a test's build recipe changes, update its row here
# to match the header comment.
#
# Usage: ./run_host_tests.sh   (from anywhere; the script cds to its own dir)
#
# Exit status: 0 if every test builds and passes, 1 otherwise.

set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")"

CC="${CC:-cc}"

# All 16 tests currently compile warning-free under -Wall -Wextra, so
# warnings are treated as build failures (-Werror) to keep it that way.
# If a future test needs to add a warning-producing header/pattern, drop
# -Werror for that one row and note it here instead of loosening this for
# everyone.
CFLAGS_COMMON="-Wall -Wextra -Werror"

BUILD_DIR="$(mktemp -d)"
trap 'rm -rf "$BUILD_DIR"' EXIT

# ─────────────────────────────────────────────────────────────────────────
# Per-test table: name, defines+includes (as documented in each test's own
# "Build & run" header comment), and the source files it links against.
# Three parallel indexed arrays (not an associative array — the system
# /bin/bash on this box is 3.2, which predates `declare -A`, and `env bash`
# resolves to it because /usr/local/bin precedes /opt/homebrew/bin in
# PATH). Order matches `ls test/*_test.c`; keep all three arrays in lockstep
# when adding/removing a row.
# ─────────────────────────────────────────────────────────────────────────

TEST_NAMES=()
TEST_FLAGS=()
TEST_SOURCES=()

TEST_NAMES+=("boot_commit_test")
TEST_FLAGS+=("-DBOOT_COMMIT_HOST_TEST -DBOOT_CONTROL_HOST_TEST -std=c11 -I ../bootloader -I ../src -I ../lib/monocypher")
TEST_SOURCES+=("boot_commit_test.c ../bootloader/commit.c ../bootloader/boot_decide.c ../src/boot_control.c ../src/fw_image.c ../lib/monocypher/monocypher.c ../lib/monocypher/monocypher-ed25519.c")

TEST_NAMES+=("boot_control_test")
TEST_FLAGS+=("-DBOOT_CONTROL_HOST_TEST -I ../src")
TEST_SOURCES+=("boot_control_test.c ../src/boot_control.c")

TEST_NAMES+=("boot_decide_test")
TEST_FLAGS+=("-I ../src -I ../bootloader")
TEST_SOURCES+=("boot_decide_test.c ../bootloader/boot_decide.c")

TEST_NAMES+=("config_hostname_test")
TEST_FLAGS+=("-DJSMN_STATIC -DWEB_AUTH_HOST_TEST -I ../src -I ../lib/jsmn -I ../lib/monocypher")
TEST_SOURCES+=("config_hostname_test.c ../src/config.c ../src/web_auth.c ../lib/monocypher/monocypher.c")

TEST_NAMES+=("config_schema_test")
TEST_FLAGS+=("-DJSMN_STATIC -DWEB_AUTH_HOST_TEST -I ../src -I ../lib/jsmn -I ../lib/monocypher")
TEST_SOURCES+=("config_schema_test.c ../src/config.c ../src/web_auth.c ../lib/monocypher/monocypher.c")

TEST_NAMES+=("web_auth_test")
TEST_FLAGS+=("-DWEB_AUTH_HOST_TEST -I ../src -I ../lib/monocypher")
TEST_SOURCES+=("web_auth_test.c ../src/web_auth.c ../lib/monocypher/monocypher.c")

TEST_NAMES+=("dns_resolve_test")
TEST_FLAGS+=("-DDNS_RESOLVE_HOST_TEST -I ../src/net")
TEST_SOURCES+=("dns_resolve_test.c ../src/net/dns_resolve.c")

TEST_NAMES+=("fw_image_test")
TEST_FLAGS+=("-I ../src -I ../lib/monocypher")
TEST_SOURCES+=("fw_image_test.c ../src/fw_image.c ../lib/monocypher/monocypher.c ../lib/monocypher/monocypher-ed25519.c")

# fw_sign_e2e_test is the one exception to "no /tmp/<name> literals": the
# test source itself (fw_sign_e2e_test.c) hardcodes FW_SIGN_BIN as
# "/tmp/fw_sign" and shells out to it via system() — that path is baked
# into the .c file, not a build option, so the helper `fw_sign` CLI it
# execs really must be built to literal /tmp/fw_sign. See run_fw_sign_e2e
# below. The test *binary* itself still builds into $BUILD_DIR like everyone
# else.
TEST_NAMES+=("fw_sign_e2e_test")
TEST_FLAGS+=("-I ../src -I ../lib/monocypher")
TEST_SOURCES+=("fw_sign_e2e_test.c ../src/fw_image.c ../lib/monocypher/monocypher.c ../lib/monocypher/monocypher-ed25519.c")
FW_SIGN_HELPER_SOURCES="../tools/fw_sign.c ../src/fw_image.c ../lib/monocypher/monocypher.c ../lib/monocypher/monocypher-ed25519.c"

TEST_NAMES+=("fwupdate_test")
TEST_FLAGS+=("-DFWUPDATE_HOST_TEST -I ../src -I ../src/net")
TEST_SOURCES+=("fwupdate_test.c ../src/net/fwupdate.c")

TEST_NAMES+=("http_client_test")
TEST_FLAGS+=("-DHTTP_CLIENT_HOST_TEST -I ../src/net")
TEST_SOURCES+=("http_client_test.c ../src/net/http_client.c")

TEST_NAMES+=("json_str_test")
TEST_FLAGS+=("-DJSMN_STATIC -I ../src -I ../lib/jsmn")
TEST_SOURCES+=("json_str_test.c")

TEST_NAMES+=("mbq_client_test")
TEST_FLAGS+=("-DMBQ_CLIENT_HOST_TEST -I ../src")
TEST_SOURCES+=("mbq_client_test.c ../src/mbq_client.c")

TEST_NAMES+=("ota_stage_test")
TEST_FLAGS+=("-DOTA_STAGE_HOST_TEST -DBOOT_CONTROL_HOST_TEST -I ../src -I ../lib/monocypher")
TEST_SOURCES+=("ota_stage_test.c ../src/ota_stage.c ../src/boot_control.c ../src/fw_image.c ../lib/monocypher/monocypher.c ../lib/monocypher/monocypher-ed25519.c")

TEST_NAMES+=("ota_test")
TEST_FLAGS+=("-DOTA_HOST_TEST -I ../src -I ../src/net")
TEST_SOURCES+=("ota_test.c ../src/net/ota.c")

TEST_NAMES+=("partition_map_test")
TEST_FLAGS+=("-I ../src")
TEST_SOURCES+=("partition_map_test.c")

# ─────────────────────────────────────────────────────────────────────────

pass_count=0
fail_count=0
fail_names=()

build_and_run() {
    local idx="$1"
    local name="${TEST_NAMES[$idx]}"
    local bin="$BUILD_DIR/$name"
    # word-split TEST_FLAGS/TEST_SOURCES on purpose, they're flag/file lists
    # shellcheck disable=SC2086
    if ! $CC $CFLAGS_COMMON ${TEST_FLAGS[$idx]} -o "$bin" ${TEST_SOURCES[$idx]} \
            > "$BUILD_DIR/$name.build.log" 2>&1; then
        echo "FAIL: $name (build error)"
        cat "$BUILD_DIR/$name.build.log"
        fail_count=$((fail_count + 1))
        fail_names+=("$name")
        return
    fi
    if "$bin" > "$BUILD_DIR/$name.run.log" 2>&1; then
        echo "PASS: $name"
        pass_count=$((pass_count + 1))
    else
        echo "FAIL: $name (test failure)"
        cat "$BUILD_DIR/$name.run.log"
        fail_count=$((fail_count + 1))
        fail_names+=("$name")
    fi
}

# fw_sign_e2e_test needs the real fw_sign CLI built first, at the literal
# path /tmp/fw_sign its source hardcodes (see the comment on
# TEST_FLAGS[fw_sign_e2e_test] above). Requires tools/dev_privkey.bin to
# already exist (tools/fw_keygen dev_privkey.bin — see tools/README.md).
run_fw_sign_e2e() {
    local idx="$1"
    if [ ! -f ../tools/dev_privkey.bin ]; then
        echo "FAIL: fw_sign_e2e_test (missing ../tools/dev_privkey.bin — run tools/fw_keygen first)"
        fail_count=$((fail_count + 1))
        fail_names+=("fw_sign_e2e_test")
        return
    fi
    # shellcheck disable=SC2086
    if ! $CC $CFLAGS_COMMON -I ../src -I ../lib/monocypher -o /tmp/fw_sign \
            $FW_SIGN_HELPER_SOURCES > "$BUILD_DIR/fw_sign_helper.build.log" 2>&1; then
        echo "FAIL: fw_sign_e2e_test (fw_sign helper build error)"
        cat "$BUILD_DIR/fw_sign_helper.build.log"
        fail_count=$((fail_count + 1))
        fail_names+=("fw_sign_e2e_test")
        return
    fi
    build_and_run "$idx"
}

for i in "${!TEST_NAMES[@]}"; do
    if [ "${TEST_NAMES[$i]}" = "fw_sign_e2e_test" ]; then
        run_fw_sign_e2e "$i"
    else
        build_and_run "$i"
    fi
done

echo
echo "== $pass_count/${#TEST_NAMES[@]} passed =="
if [ "$fail_count" -gt 0 ]; then
    echo "FAILED: ${fail_names[*]}"
    exit 1
fi
exit 0
