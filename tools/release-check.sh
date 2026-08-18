#!/usr/bin/env bash
#
# release-check.sh — the single pre-release gate for the controller firmware.
#
# Run this before tagging or publishing any OTA release (i.e. before the
# "Ongoing: release an update" steps in CLAUDE.md / tools/README.md). It
# exists because the release flow has several footguns that are each easy to
# get right individually and easy to forget under pressure — a dirty tree
# silently baking "-dirty" into FW_VERSION, a manifest `version` that doesn't
# match the image's FW_VERSION (CLAUDE.md's "#1 footgun" — HA never reports
# "up to date"), a warning that slipped into `src/` or `bootloader/`, or a
# release that touches boot code with a stale/unbuilt bootloader binary. This
# script runs every check that catches those, in one command, fail-fast, with
# a clear PASS/FAIL per stage and a final summary.
#
# Stages (in order — see run_stage_* functions below for the "why" of each):
#   1. git cleanliness        — refuse a dirty superproject tree
#   2. host tests              — test/run_host_tests.sh (16 tests)
#   3. standalone build        — build/ light_switch, zero warnings
#   4. OTA build+sign+manifest — build_ota/ light_switch_ota, signed, and a
#                                 manifest dry-run with the version/size
#                                 cross-checks CLAUDE.md calls out
#   5. bootloader build        — bootloader/build/, zero warnings
#   6. hardware e2e (opt-in)   — test/ha_e2e.py against the bench device +
#                                 dev broker; OFF by default (see stage 6)
#
# Usage:
#   tools/release-check.sh                 # stages 1-5, no hardware touched
#   tools/release-check.sh --with-e2e      # also runs stage 6 (needs the
#                                           # bench controller + dev HA/broker
#                                           # up — see CLAUDE.md's serial/
#                                           # test-rig notes)
#   tools/release-check.sh --allow-dirty   # skip stage 1 (you know the
#                                           # resulting FW_VERSION will say
#                                           # "-dirty" — fine for a scratch
#                                           # build, never for a release)
#
# Exit status: 0 if every stage that ran passed, 1 otherwise. Runs from
# anywhere (cd's to the repo root itself, like tools/bench_flash_ota.sh).

set -euo pipefail

cd "$(dirname "$0")/.."
ROOT="$(pwd)"

CC="${CC:-cc}"

ALLOW_DIRTY=0
WITH_E2E=0

usage() {
    cat <<'EOF'
usage: tools/release-check.sh [--allow-dirty] [--with-e2e]

  --allow-dirty   skip the git-cleanliness gate (stage 1). The resulting
                  build's FW_VERSION will contain "-dirty" — never do this
                  for a real release, only for exercising this script.
  --with-e2e      also run stage 6, the hardware end-to-end test
                  (test/ha_e2e.py). Needs the bench controller + dev HA/
                  broker up. OFF by default: this script must be safe to
                  run unattended without touching real hardware.
EOF
}

for arg in "$@"; do
    case "$arg" in
        --allow-dirty) ALLOW_DIRTY=1 ;;
        --with-e2e) WITH_E2E=1 ;;
        -h|--help) usage; exit 0 ;;
        *)
            echo "unknown argument: $arg" >&2
            usage >&2
            exit 2
            ;;
    esac
done

# ─────────────────────────────────────────────────────────────────────────
# Bookkeeping: two parallel indexed arrays (not an associative array — see
# test/run_host_tests.sh's header comment for why: the system /bin/bash on
# this box is 3.2, which predates `declare -A`). record() appends a row;
# cleanup() (below) prints them all as the final summary on every exit path,
# pass or fail, via an EXIT trap.
# ─────────────────────────────────────────────────────────────────────────

STAGE_NAMES=()
STAGE_STATUS=()

record() {
    STAGE_NAMES+=("$1")
    STAGE_STATUS+=("$2")
}

WORK_DIR="$(mktemp -d)"

cleanup() {
    local rc=$?
    rm -rf "$WORK_DIR"
    echo
    echo "== release-check summary =="
    for i in "${!STAGE_NAMES[@]}"; do
        printf '  %-32s %s\n' "${STAGE_NAMES[$i]}" "${STAGE_STATUS[$i]}"
    done
    if [ "$rc" -eq 0 ]; then
        echo "RESULT: PASS"
    else
        echo "RESULT: FAIL"
    fi
    exit "$rc"
}
trap cleanup EXIT

fail() {
    echo "FAIL: $*"
    exit 1
}

filesize() {
    # Portable BSD/GNU stat, same idiom as fw_manifest.sh.
    stat -f%z "$1" 2>/dev/null || stat -c%s "$1"
}

# Any 'warning:' line from a compiler invocation whose source path is under
# THIS repo's own src/ or bootloader/ directories is ours and fails the
# gate. Vendored code (lib/*) is excluded on purpose rather than by an
# explicit exclude list: Monocypher and ioLibrary are already built with -w
# (see CMakeLists.txt and bootloader/CMakeLists.txt's
# set_source_files_properties(... "-w")), and nothing else under lib/
# (pico-sdk, pico-vfs, coreMQTT) is ever compiled with our -Wall -Wextra —
# so in practice a 'warning:' from our own translation units is the only
# thing this should ever match.
#
# CMake invokes the compiler with absolute source paths in this tree
# (verified against compile_commands.json), so matching is anchored to
# "$ROOT/src/" and "$ROOT/bootloader/" specifically via bash glob
# (case-pattern) matching, NOT a bare grep for the substring "/src/" —
# pico-sdk's own layout has a nested lib/pico-sdk/src/rp2_common/... tree,
# so a loose substring match would false-positive on any pico-sdk warning
# (caught by hand-testing this function against a synthetic log before
# relying on it).
check_no_warnings() {
    local log="$1" label="$2"
    local line warnings=""
    while IFS= read -r line; do
        case "$line" in
            "$ROOT/src/"*": warning:"*|"$ROOT/bootloader/"*": warning:"*)
                warnings="${warnings}${line}"$'\n'
                ;;
        esac
    done < <(grep ': warning:' "$log" || true)
    if [ -n "$warnings" ]; then
        printf '%s' "$warnings"
        fail "$label produced warning(s) in src/ or bootloader/ — zero warnings tolerated (see above)"
    fi
}

# ─────────────────────────────────────────────────────────────────────────
# Stage 1 — git cleanliness
# ─────────────────────────────────────────────────────────────────────────
run_stage_git_clean() {
    echo "== [1/6] GIT CLEANLINESS =="
    if [ "$ALLOW_DIRTY" = "1" ]; then
        echo "SKIP (--allow-dirty): the FW_VERSION built from this tree WILL contain \"-dirty\""
        record "1 git cleanliness" "SKIP (--allow-dirty)"
        return
    fi
    # Mirror exactly what `git describe --dirty` itself checks (verified by
    # hand: `git describe --dirty` internally runs `git diff-index --quiet
    # HEAD --`), rather than the broader `git status --porcelain`. The two
    # differ in a way that matters here:
    #   - untracked files do NOT make `git describe --dirty` dirty, so they
    #     must not fail this gate either (this file itself is untracked
    #     until the parent session commits it — `git status --porcelain`
    #     would wrongly flag that as a release blocker);
    #   - modified TRACKED files do, and are the real failure case.
    # --ignore-submodules=dirty ignores uncommitted content inside a
    # submodule's working tree — lib/ioLibrary_Driver intentionally carries
    # two build-time patches applied there at CMake configure time
    # (CLAUDE.md "ioLibrary recv() #173" / "DHCP configurable hostname" —
    # see CMakeLists.txt's iolib_patch machinery) — while still catching the
    # case that DOES matter: a submodule pointing at a different commit
    # (gitlink) than what's recorded in the superproject index.
    git update-index -q --refresh || true
    local tracked_dirty
    tracked_dirty="$(git diff-index --name-only HEAD --ignore-submodules=dirty --)"
    if [ -n "$tracked_dirty" ]; then
        echo "$tracked_dirty"
        fail "tracked file(s) modified vs HEAD (listed above) — FW_VERSION embeds 'git describe --dirty', so a release built now ships as \"...-dirty\" (indistinguishable from any other dirty build). Commit first, or pass --allow-dirty to override."
    fi

    local untracked
    untracked="$(git ls-files --others --exclude-standard)"
    if [ -n "$untracked" ]; then
        echo "$untracked"
        echo "NOTE: untracked file(s) present (listed above). These do NOT affect FW_VERSION, but they also won't be part of the commit this release is built from — commit them first if they belong in the release."
    fi

    echo "PASS: no tracked-file changes vs HEAD (FW_VERSION will not carry '-dirty')"
    record "1 git cleanliness" "PASS"
}

# ─────────────────────────────────────────────────────────────────────────
# Stage 2 — host tests
# ─────────────────────────────────────────────────────────────────────────
run_stage_host_tests() {
    echo "== [2/6] HOST TESTS =="
    if ! ./test/run_host_tests.sh; then
        fail "host tests failed (see output above) — test/run_host_tests.sh must be all-green before release"
    fi
    record "2 host tests" "PASS"
}

# ─────────────────────────────────────────────────────────────────────────
# Stage 3 — standalone build (build/, the default `light_switch` target)
# ─────────────────────────────────────────────────────────────────────────
run_stage_standalone_build() {
    echo "== [3/6] STANDALONE BUILD =="
    local log="$WORK_DIR/build.log"
    if ! { cmake -S . -B build && make -C build; } >"$log" 2>&1; then
        cat "$log"
        fail "standalone build failed (build/, target light_switch)"
    fi
    check_no_warnings "$log" "standalone build"
    echo "PASS: standalone build (build/light_switch.uf2)"
    record "3 standalone build" "PASS"
}

# ─────────────────────────────────────────────────────────────────────────
# Stage 4 — OTA build + sign + manifest dry-run (build_ota/, light_switch_ota)
# ─────────────────────────────────────────────────────────────────────────
run_stage_ota_build() {
    echo "== [4/6] OTA BUILD + SIGN + MANIFEST DRY-RUN =="
    local log="$WORK_DIR/build_ota.log"
    if ! { cmake -S . -B build_ota -DOTA_APP_LAYOUT=ON && make -C build_ota light_switch_ota; } \
            >"$log" 2>&1; then
        cat "$log"
        fail "OTA build failed (build_ota/, target light_switch_ota)"
    fi
    check_no_warnings "$log" "OTA build"

    arm-none-eabi-objcopy -O binary build_ota/light_switch_ota.elf build_ota/light_switch_ota.bin

    local ver
    ver="$(sed -n 's/.*FW_VERSION "\(.*\)".*/\1/p' build_ota/generated/version.h)"
    case "$ver" in
        *-dirty)
            # Should be unreachable — stage 1 already refused a dirty tree
            # (or the caller explicitly accepted one via --allow-dirty). If
            # this ever fires with stage 1 having passed, something is wrong
            # with the git-describe/build ordering, not just a dirty tree.
            fail "built image version is '$ver' (dirty) — should be unreachable after stage 1 passed; investigate"
            ;;
    esac

    if [ ! -x tools/fw_sign ]; then
        echo "tools/fw_sign not built yet — building it (see tools/README.md)"
        "$CC" -Wall -Wextra -I src -I lib/monocypher -o tools/fw_sign tools/fw_sign.c \
            src/fw_image.c lib/monocypher/monocypher.c lib/monocypher/monocypher-ed25519.c
    fi

    if [ ! -f tools/dev_privkey.bin ]; then
        echo "WARN: tools/dev_privkey.bin absent — skipping sign + manifest dry-run"
        echo "      (generate one: tools/fw_keygen dev_privkey.bin — see tools/README.md)"
        echo "PASS: OTA build (sign/manifest skipped, no dev key)"
        record "4 OTA build+sign+manifest" "PASS (sign skipped, no key)"
        return
    fi

    # Read-only: this is a dry-run gate, not a real release, so it signs with
    # whatever tools/fw_build_counter currently holds and does NOT bump it —
    # bumping is a real release's job (tools/README.md's release flow).
    if [ ! -f tools/fw_build_counter ]; then
        fail "tools/fw_build_counter missing — see tools/README.md's Anti-rollback build counter section"
    fi
    local build_ctr
    build_ctr="$(cat tools/fw_build_counter)"

    tools/fw_sign build_ota/light_switch_ota.bin tools/dev_privkey.bin "$build_ctr" \
        build_ota/light_switch_ota.signed.bin >/dev/null

    tools/fw_manifest.sh "$ver" build_ota/light_switch_ota.signed.bin \
        http://ota.8bitcloud.com/light_switch.bin "Light Switch $ver" "" \
        "release-check.sh dry-run manifest" > build_ota/light_switch.json

    # The #1 OTA footgun (CLAUDE.md "Version gotcha"): the device treats
    # manifest.version != its own FW_VERSION as "update available" by plain
    # string inequality. A mismatch here either leaves a phantom update
    # forever (hand-picked version that never matches what gets installed)
    # or never converges on "up to date" after install. Re-read both the
    # manifest and version.h from disk rather than trusting $ver, so this
    # check would still catch a bug in fw_manifest.sh itself.
    local manifest_ver manifest_size actual_size
    manifest_ver="$(jq -r .version build_ota/light_switch.json)"
    manifest_size="$(jq -r .size build_ota/light_switch.json)"
    actual_size="$(filesize build_ota/light_switch_ota.signed.bin)"

    if [ "$manifest_ver" != "$ver" ]; then
        fail "manifest version '$manifest_ver' != image FW_VERSION '$ver' — HA would never converge on 'up to date' (or never offer the update at all)"
    fi
    if [ "$manifest_size" != "$actual_size" ]; then
        fail "manifest size $manifest_size != actual signed binary size $actual_size bytes"
    fi

    echo "PASS: OTA build + sign + manifest (version $ver, ${actual_size} B)"
    record "4 OTA build+sign+manifest" "PASS (version $ver)"
}

# ─────────────────────────────────────────────────────────────────────────
# Stage 5 — bootloader build (bootloader/build/, separate top-level project)
# ─────────────────────────────────────────────────────────────────────────
run_stage_bootloader_build() {
    echo "== [5/6] BOOTLOADER BUILD =="
    local log="$WORK_DIR/bootloader_build.log"
    if ! { cmake -S bootloader -B bootloader/build && make -C bootloader/build; } \
            >"$log" 2>&1; then
        cat "$log"
        fail "bootloader build failed (bootloader/build/)"
    fi
    check_no_warnings "$log" "bootloader build"
    echo "PASS: bootloader build (bootloader/build/bootloader.uf2)"
    record "5 bootloader build" "PASS"
}

# ─────────────────────────────────────────────────────────────────────────
# Stage 6 — hardware end-to-end (opt-in, OFF by default)
# ─────────────────────────────────────────────────────────────────────────
run_stage_hardware_e2e() {
    echo "== [6/6] HARDWARE E2E (optional) =="
    if [ "$WITH_E2E" != "1" ]; then
        echo "SKIPPED — opt-in only (pass --with-e2e). Needs the bench controller"
        echo "flashed and running, plus the dev HA + MQTT broker up (see CLAUDE.md's"
        echo "serial/test-rig notes) before test/ha_e2e.py can round-trip anything."
        record "6 hardware e2e" "SKIPPED (opt-in --with-e2e)"
        return
    fi

    if [ -n "${RELEASE_CHECK_E2E_DRYRUN:-}" ]; then
        # Internal test hook ONLY, for exercising this script's own
        # --with-e2e argument wiring without touching the bench device or
        # broker. Never set this for a real release check.
        echo "[dry-run] RELEASE_CHECK_E2E_DRYRUN set — would exec: .venv/bin/python test/ha_e2e.py"
        record "6 hardware e2e" "DRY-RUN (RELEASE_CHECK_E2E_DRYRUN set)"
        return
    fi

    echo "Running test/ha_e2e.py against the bench device + dev broker..."
    if ! .venv/bin/python test/ha_e2e.py; then
        fail "hardware e2e failed (test/ha_e2e.py) — see output above"
    fi
    echo "PASS: hardware e2e"
    record "6 hardware e2e" "PASS"
}

# ─────────────────────────────────────────────────────────────────────────

run_stage_git_clean
run_stage_host_tests
run_stage_standalone_build
run_stage_ota_build
run_stage_bootloader_build
run_stage_hardware_e2e

exit 0
