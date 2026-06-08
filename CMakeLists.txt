cmake_minimum_required(VERSION 3.13)

# Board: WIZnet W5100S-EVB-Pico (RP2040). The board header pins PICO_PLATFORM=rp2040,
# the W5100S SPI0 pins (SCK18/MOSI19/MISO16/CS17/RST20/INT21), 2 MB flash + W25Q080
# stage2. (Was a bare Pico 2 / rp2350 before the on-board-bridge work.)
set(PICO_BOARD wiznet_w5100s_evb_pico)
set(PICO_SDK_PATH "lib/pico-sdk")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(${PICO_SDK_PATH}/pico_sdk_init.cmake)

project(light_switch C CXX ASM)
set(CMAKE_C_STANDARD 11)

pico_sdk_init()

add_executable(light_switch
    src/main.c
    src/intercore.c
    src/modbus.c
    src/button_actions.c
    src/button_fsm.c
    src/state_cache.c
    src/config.c
    src/mapping.c
    src/lighting_actions.c
    src/dirty_poll.c
    src/bus_discovery.c
    src/serial_protocol.c
    src/serial_protocol_provision.c
    src/dali_provision.c
    src/button_fsm_emit.c
    src/log.c
)

add_subdirectory(lib/pico-vfs)

target_include_directories(light_switch PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src
    ${CMAKE_BINARY_DIR}/generated      # version.h (git-derived FW_VERSION)
)

# ── FW_VERSION = base + git describe (tag/hash + -dirty), refreshed each build ──
set(FW_BASE_VERSION "2.0.0")
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/generated)
set(GEN_VERSION_ARGS
    -DSRC_DIR=${CMAKE_CURRENT_LIST_DIR}
    -DOUT=${CMAKE_BINARY_DIR}/generated/version.h
    -DBASE_VERSION=${FW_BASE_VERSION}
    -P ${CMAKE_CURRENT_LIST_DIR}/cmake/gen_version.cmake)
execute_process(COMMAND ${CMAKE_COMMAND} ${GEN_VERSION_ARGS})   # exist at configure time
add_custom_target(gen_version ALL
    COMMAND ${CMAKE_COMMAND} ${GEN_VERSION_ARGS}
    BYPRODUCTS ${CMAKE_BINARY_DIR}/generated/version.h
    COMMENT "Updating version.h from git")
add_dependencies(light_switch gen_version)

target_link_libraries(light_switch PRIVATE
    pico_stdlib
    hardware_pio
    hardware_uart
    hardware_gpio
    hardware_timer
    hardware_watchdog
    hardware_flash
    pico_flash
    pico_multicore     # core 1 network bridge
    pico_util          # queue_t (inter-core SPSC queues)
    pico_time
)

# jsmn — single-header JSON tokenizer used by core 1 (malloc-free, float-free).
target_include_directories(light_switch PRIVATE ${CMAKE_CURRENT_LIST_DIR}/lib/jsmn)

# USB CDC for serial protocol (replaces Lua REPL)
pico_enable_stdio_usb(light_switch 1)
pico_enable_stdio_uart(light_switch 0)

# Filesystem — same config as original
pico_enable_filesystem(light_switch
    SIZE 262144        # 256 KB
    AUTO_INIT 0
    MAX_FAT_VOLUME 1
    MAX_MOUNTPOINT 2
)

# PIO button grid
pico_generate_pio_header(light_switch ${CMAKE_CURRENT_LIST_DIR}/src/buttongrid.pio)

pico_add_extra_outputs(light_switch)

target_compile_options(light_switch PRIVATE -Wall -Wextra -Wno-unused-parameter)
