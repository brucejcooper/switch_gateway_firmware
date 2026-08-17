cmake_minimum_required(VERSION 3.13)

# ── Standalone pico-sdk project for the OTA bootloader ──────────────────────
#
# This is a SEPARATE top-level CMake project from the controller's own
# CMakeLists.txt (../CMakeLists.txt) — it is never add_subdirectory()'d into
# the main 'light_switch' build, and building it does not touch that build
# tree at all (separate binary dir, e.g. bootloader/build). See
# bootloader/README.md for the full layout/boot-flow story and, importantly,
# what is and is NOT validated without real hardware.
#
# Same board as the main firmware (WIZnet W5100S-EVB-Pico / RP2040) so we
# pick up the right LED pin, clock config, and default boot_stage2 for the
# actual on-board flash chip — even though this bootloader never touches the
# W5100S/SPI0 itself.
set(PICO_BOARD wiznet_w5100s_evb_pico)
set(PICO_SDK_PATH ${CMAKE_CURRENT_LIST_DIR}/../lib/pico-sdk)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# --- Flash-size correctness note (do NOT set PICO_FLASH_SIZE_BYTES here) ---
# A previous version of this file set `PICO_FLASH_SIZE_BYTES "0xE000"` here
# to shrink the *linker's* FLASH region to the bootloader partition. That is
# WRONG: setting this CMake variable before pico_sdk_init() makes the board
# header treat it as a user override (see pico-sdk's cmake/generic_board.cmake
# -> PICO_BOARD_CMAKE_OVERRIDES), which pico_base_headers' CMakeLists.txt
# then promotes into a GLOBAL `target_compile_definitions(pico_base_headers
# INTERFACE PICO_FLASH_SIZE_BYTES=0xE000)` — and pico_base_headers is an
# INTERFACE dependency of essentially everything, including hardware_flash.c.
# hardware_flash.c's flash_range_erase()/flash_range_program() do
# `hard_assert(flash_offs + count <= PICO_FLASH_SIZE_BYTES)`, so with that
# global define in place this bootloader would panic('Hard assert') the
# first time it erased/programmed anything at or past BOOTCTRL_A_OFF/APP_OFF
# (0xE000/0x10000) — i.e. every real commit-staging or bc_write() call on
# actual hardware. Confirmed via bootloader/build/compile_commands.json:
# hardware_flash/flash.c was compiled with `-DPICO_FLASH_SIZE_BYTES=0xE000`
# even though nothing in this file names that translation unit.
#
# The fix: leave PICO_FLASH_SIZE_BYTES alone (it defaults to the board
# header's true 2 MB chip size, wiznet_w5100s_evb_pico.h) and instead
# constrain *where this executable's own code+data may be linked* with a
# dedicated linker script, memmap_bootloader.ld (set further down via
# pico_set_linker_script, after the target exists) — a byte-for-byte copy of
# pico-sdk's memmap_default.ld except its FLASH MEMORY region is hardcoded
# to ORIGIN=0x10000000 LENGTH=0xE000 (BOOTLOADER_OFF/BOOTLOADER_SIZE from
# ../src/partition_map.h) instead of `INCLUDE "pico_flash_region.ld"`. That
# region assignment is a link-time-only constraint on section placement; it
# has no effect on any C-visible macro, so hardware_flash.c's bounds check
# keeps seeing the chip's real 2 MB and never panics on an in-bounds
# APP_OFF/STAGING_OFF/BOOTCTRL_*_OFF access. See memmap_bootloader.ld's own
# header comment and README.md for the full story.

include(${PICO_SDK_PATH}/pico_sdk_init.cmake)

project(bootloader C CXX ASM)
set(CMAKE_C_STANDARD 11)

pico_sdk_init()

add_executable(bootloader
    main.c
    boot_decide.c
    commit.c
    ${CMAKE_CURRENT_LIST_DIR}/../src/boot_control.c
    ${CMAKE_CURRENT_LIST_DIR}/../src/fw_image.c
    ${CMAKE_CURRENT_LIST_DIR}/../lib/monocypher/monocypher.c
    ${CMAKE_CURRENT_LIST_DIR}/../lib/monocypher/monocypher-ed25519.c
)

target_include_directories(bootloader PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/../src
    ${CMAKE_CURRENT_LIST_DIR}/../lib/monocypher
)

# Constrain this executable's own code+data to BOOTLOADER_OFF..+BOOTLOADER_SIZE
# (0x0..0xE000) — see the big comment above for why this is a linker script,
# not a PICO_FLASH_SIZE_BYTES CMake variable. KEEP memmap_bootloader.ld's FLASH
# LENGTH in sync with BOOTLOADER_SIZE in ../src/partition_map.h; growing past
# it fails the link with a "region `FLASH' overflowed" error.
pico_set_linker_script(bootloader ${CMAKE_CURRENT_LIST_DIR}/memmap_bootloader.ld)

# Vendored third-party: don't subject Monocypher to our -Wall -Wextra (same
# as the main firmware's CMakeLists.txt).
set_source_files_properties(
    ${CMAKE_CURRENT_LIST_DIR}/../lib/monocypher/monocypher.c
    ${CMAKE_CURRENT_LIST_DIR}/../lib/monocypher/monocypher-ed25519.c
    PROPERTIES COMPILE_OPTIONS "-w")

# pico_runtime (NOT pico_stdlib): crt0 + boot2 + the standard linker-script
# wiring, runtime bring-up (clocks etc.) — but none of pico_stdlib's stdio
# plumbing. This bootloader runs single-core with NO USB and NO network by
# design (see README.md); it only needs hardware_flash (erase/program),
# hardware_gpio (the halt/error LED), and hardware_structs (SCB->VTOR for
# the app handoff).
target_link_libraries(bootloader PRIVATE
    pico_runtime
    pico_bootrom
    hardware_flash
    hardware_gpio
    hardware_structs
    hardware_watchdog
)

# No stdio anywhere in this image (belt-and-braces on top of not linking
# pico_stdio_usb/pico_stdio_uart at all): matches the "no USB/network" rule.
pico_enable_stdio_usb(bootloader 0)
pico_enable_stdio_uart(bootloader 0)

pico_add_extra_outputs(bootloader)

target_compile_options(bootloader PRIVATE -Wall -Wextra -Wno-unused-parameter)
