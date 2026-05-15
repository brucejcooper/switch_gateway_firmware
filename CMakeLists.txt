cmake_minimum_required(VERSION 3.13)

set(PICO_PLATFORM "rp2350")
set(PICO_SDK_PATH "lib/pico-sdk")

include(${PICO_SDK_PATH}/pico_sdk_init.cmake)

project(light_switch C CXX ASM)
set(CMAKE_C_STANDARD 11)

pico_sdk_init()

add_executable(light_switch
    src/main.c
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
)

target_link_libraries(light_switch PRIVATE
    pico_stdlib
    hardware_pio
    hardware_uart
    hardware_gpio
    hardware_timer
    hardware_watchdog
    hardware_flash
    pico_flash
    pico_util          # queue_t
    pico_time
)

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
