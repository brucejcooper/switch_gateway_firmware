#!/usr/bin/env bash
# fw_manifest.sh — emit the pull-based-OTA manifest JSON for a signed
# light_switch firmware image, matching the brewery fleet's manifest shape
# plus this project's fields (see src/net/fwupdate.h's documented shape and
# tools/README.md for the full publish flow).
#
# Manifest fields:
#   version      - firmware version string (arg 1)
#   board        - always "light_switch"
#   url          - where the signed .bin will be published (arg 3)
#   size         - byte size of the signed .bin (measured, not user-supplied)
#   sha256       - sha256 of the signed .bin (informational only — the
#                  fw_trailer's ed25519 signature inside the .bin is the
#                  actual trust root that fw_verify() checks on-device;
#                  this field is a convenience for humans/scripts, e.g.
#                  for a download integrity spot-check)
#   title        - optional human title (arg 4, may be empty)
#   release_url  - optional release notes URL (arg 5, may be empty)
#   notes        - optional free-text notes (arg 6, may be empty)
#
# Usage:
#   fw_manifest.sh <version> <signed.bin> <publish_url> [title] [release_url] [notes]
#
# Example:
#   tools/fw_manifest.sh 2.1.0 build/light_switch.signed.bin \
#       http://ota.8bitcloud.com/light_switch.bin \
#       "Light Switch 2.1.0" \
#       https://example.com/releases/2.1.0 \
#       "Bug fixes and OTA support" \
#       > light_switch.json
#
# Output goes to stdout — redirect to light_switch.json for publishing.

set -euo pipefail

if [ "$#" -lt 3 ] || [ "$#" -gt 6 ]; then
    echo "usage: $0 <version> <signed.bin> <publish_url> [title] [release_url] [notes]" >&2
    exit 2
fi

VERSION="$1"
SIGNED_BIN="$2"
URL="$3"
TITLE="${4:-}"
RELEASE_URL="${5:-}"
NOTES="${6:-}"

if [ ! -f "$SIGNED_BIN" ]; then
    echo "error: signed bin not found: $SIGNED_BIN" >&2
    exit 1
fi

# Byte size — portable BSD/GNU stat.
if stat -f%z "$SIGNED_BIN" >/dev/null 2>&1; then
    SIZE=$(stat -f%z "$SIGNED_BIN")          # BSD/macOS stat
else
    SIZE=$(stat -c%s "$SIGNED_BIN")          # GNU stat
fi

# sha256 — try shasum (macOS default) then sha256sum (Linux).
if command -v shasum >/dev/null 2>&1; then
    SHA256=$(shasum -a 256 "$SIGNED_BIN" | awk '{print $1}')
elif command -v sha256sum >/dev/null 2>&1; then
    SHA256=$(sha256sum "$SIGNED_BIN" | awk '{print $1}')
else
    echo "error: need shasum or sha256sum on PATH" >&2
    exit 1
fi

# Minimal JSON string escaper: backslash and double-quote only (title/
# release_url/notes are short human-supplied strings; this is not a
# general-purpose JSON encoder, matching the firmware's own "dumb but
# sufficient" JSON handling in fwupdate.c/fw_json_get()).
json_escape() {
    printf '%s' "$1" | sed 's/\\/\\\\/g; s/"/\\"/g'
}

VERSION_J=$(json_escape "$VERSION")
URL_J=$(json_escape "$URL")
TITLE_J=$(json_escape "$TITLE")
RELEASE_URL_J=$(json_escape "$RELEASE_URL")
NOTES_J=$(json_escape "$NOTES")

cat <<EOF
{
  "version": "${VERSION_J}",
  "board": "light_switch",
  "url": "${URL_J}",
  "size": ${SIZE},
  "sha256": "${SHA256}",
  "title": "${TITLE_J}",
  "release_url": "${RELEASE_URL_J}",
  "notes": "${NOTES_J}"
}
EOF
