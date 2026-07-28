#!/usr/bin/env bash
# bench_flash_ota.sh — reflash the OTA-layout bench device over USB.
#
# The bench controller runs the bootloader + relocated app layout, but has no
# network, so the normal pull-OTA path is unavailable. Flashing the app slot
# directly is not enough on its own: for a VALID boot-control record the
# bootloader fw_verify()s the active slot against the LENGTH RECORDED IN THAT
# RECORD (bootloader/main.c, BOOT_ACTIVE && bc_valid), so a freshly built app of
# a different size fails verification and lands in BOOTSEL. So we also erase the
# two boot-control sectors, which puts the bootloader on its factory (!bc_valid)
# path: vector-table sanity check + watchdog-armed jump, no length to disagree
# with.
#
# Cost of that erase: BC_F_BACKUP_VALID is cleared, so the NEXT real OTA has no
# rollback fallback (exactly like the first-ever OTA over a factory image). Fine
# for a bench device; do NOT use this on a production unit — use a real OTA.
#
# Usage: tools/bench_flash_ota.sh      (run from the controller/ directory)
set -euo pipefail

cd "$(dirname "$0")/.."

echo "== build (relocated app)"
cmake -S . -B build_ota -DOTA_APP_LAYOUT=ON >/dev/null
make -C build_ota light_switch_ota >/dev/null
arm-none-eabi-objcopy -O binary build_ota/light_switch_ota.elf build_ota/light_switch_ota.bin
tools/fw_sign build_ota/light_switch_ota.bin tools/dev_privkey.bin \
              build_ota/light_switch_ota.signed.bin >/dev/null
VER=$(sed -n 's/.*FW_VERSION "\(.*\)".*/\1/p' build_ota/generated/version.h)
echo "   version $VER  ($(stat -f%z build_ota/light_switch_ota.signed.bin) B signed)"

# Park in BOOTSEL — but ONLY if the device is running the app. `picotool reboot
# -f -u` HANGS when the device is already in BOOTSEL (there is no application to
# force out of), which silently strands a flash half-done: the board sits parked
# with no app and no serial port, looking like a boot failure. `picotool info`
# without -f succeeds only in BOOTSEL, so it is the probe.
if picotool info >/dev/null 2>&1; then
    echo "== already in BOOTSEL"
else
    echo "== park in BOOTSEL"
    picotool reboot -f -u >/dev/null
    sleep 2
fi

echo "== load app @ 0x10010000"
picotool load build_ota/light_switch_ota.signed.bin -o 0x10010000 >/dev/null

echo "== erase boot-control (0x0E000-0x10000) -> factory boot path"
picotool erase -r 0x1000E000 0x10010000 >/dev/null

echo "== reboot"
picotool reboot >/dev/null
sleep 4
echo "   done — $VER should be live"
