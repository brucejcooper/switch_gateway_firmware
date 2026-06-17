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
    src/core1.c
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
    src/cdc_out.c
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

# ── Embed the Vite single-file config web UI into flash (requires Node/npm) ──
# The web app under web/ is built to one self-contained index.html, then
# web/tools/gen-assets.mjs gzips it and emits generated/web_assets.c (the
# web_assets_lookup table served by src/net/http_server.c). Rebuilds only when a
# web source changes (custom-command DEPENDS), so the firmware dev loop isn't
# slowed by a Vite run every make.
set(WEB_DIR ${CMAKE_CURRENT_LIST_DIR}/web)
set(GEN_WEB_ASSETS ${CMAKE_BINARY_DIR}/generated/web_assets.c)
find_program(NPM_EXECUTABLE npm)
if(NOT NPM_EXECUTABLE)
    message(FATAL_ERROR
        "npm not found, but the config web UI is built into the firmware. Install "
        "Node.js (the web/ Vite build is required), or build a Node-free variant by "
        "compiling a hand-written web_assets.c instead of the generated one.")
endif()
file(GLOB_RECURSE WEB_SRCS CONFIGURE_DEPENDS
    ${WEB_DIR}/src/*
    ${WEB_DIR}/index.html
    ${WEB_DIR}/package.json
    ${WEB_DIR}/vite.config.js
    ${WEB_DIR}/tools/gen-assets.mjs)
add_custom_command(
    OUTPUT ${GEN_WEB_ASSETS}
    COMMAND ${NPM_EXECUTABLE} --prefix ${WEB_DIR} install --no-audit --no-fund --silent
    COMMAND ${CMAKE_COMMAND} -E env GEN_OUT=${GEN_WEB_ASSETS}
            ${NPM_EXECUTABLE} --prefix ${WEB_DIR} run build --silent
    DEPENDS ${WEB_SRCS}
    COMMENT "Building web UI (Vite) + embedding gzipped assets"
    VERBATIM)
set_source_files_properties(${GEN_WEB_ASSETS} PROPERTIES GENERATED TRUE)

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

# jsmn — single-header JSON tokenizer (malloc-free, float-free). JSMN_STATIC so
# each TU that includes it (config.c on core 0, ha_bridge.c on core 1) gets its
# own copy — no duplicate-symbol link error.
target_include_directories(light_switch PRIVATE ${CMAKE_CURRENT_LIST_DIR}/lib/jsmn)
target_compile_definitions(light_switch PRIVATE JSMN_STATIC)

# ── WIZnet W5100S networking (ioLibrary_Driver) — core 1 only ───────────────
set(IOLIB ${CMAKE_CURRENT_LIST_DIR}/lib/ioLibrary_Driver)

# Build-time patch: ioLibrary recv() returns SOCK_BUSY before checking whether
# data is actually present in non-block mode (upstream issue #173), which leaves
# the MQTT CONNACK unread forever. We keep the submodule pristine and apply the
# one-line fix here, idempotently, so it's repeatable and trivially removable
# once upstream fixes #173. Reverse-check first to detect an already-patched
# tree (re-configures, dirty checkouts); a forward-apply failure is non-fatal
# (warns) so an upstream fix that obsoletes the patch doesn't break the build.
set(IOLIB_RECV_PATCH ${CMAKE_CURRENT_LIST_DIR}/patches/0001-ioLibrary-recv-nonblock-issue-173.patch)
execute_process(
    COMMAND git apply --reverse --check ${IOLIB_RECV_PATCH}
    WORKING_DIRECTORY ${IOLIB}
    RESULT_VARIABLE _iolib_patch_rev ERROR_QUIET OUTPUT_QUIET)
if(_iolib_patch_rev EQUAL 0)
    message(STATUS "ioLibrary recv() #173 patch: already applied")
else()
    execute_process(
        COMMAND git apply ${IOLIB_RECV_PATCH}
        WORKING_DIRECTORY ${IOLIB}
        RESULT_VARIABLE _iolib_patch_app ERROR_QUIET OUTPUT_QUIET)
    if(_iolib_patch_app EQUAL 0)
        message(STATUS "ioLibrary recv() #173 patch: applied")
    else()
        message(WARNING "ioLibrary recv() #173 patch did not apply — upstream may "
            "have fixed #173. Verify Ethernet/socket.c recv() non-block handling "
            "and remove patches/0001-ioLibrary-recv-nonblock-issue-173.patch if so.")
    endif()
endif()

# Build-time patch: ioLibrary's DHCP client hardcodes the option-12 hostname to
# "WIZnet" + a 6-hex-char MAC suffix in a fixed 7-byte array. We want a clean,
# configurable hostname (so the router registers e.g. "lights" in local DNS), so
# this patch widens HOST_NAME[] to 32 bytes (netif.c overwrites it from config)
# and drops the MAC suffix. Same pristine-submodule + idempotent-apply approach
# as the recv patch above; remove both the patch file and this block to revert.
set(IOLIB_HOSTNAME_PATCH ${CMAKE_CURRENT_LIST_DIR}/patches/0002-dhcp-configurable-hostname.patch)
execute_process(
    COMMAND git apply --reverse --check ${IOLIB_HOSTNAME_PATCH}
    WORKING_DIRECTORY ${IOLIB}
    RESULT_VARIABLE _iolib_host_rev ERROR_QUIET OUTPUT_QUIET)
if(_iolib_host_rev EQUAL 0)
    message(STATUS "ioLibrary DHCP hostname patch: already applied")
else()
    execute_process(
        COMMAND git apply ${IOLIB_HOSTNAME_PATCH}
        WORKING_DIRECTORY ${IOLIB}
        RESULT_VARIABLE _iolib_host_app ERROR_QUIET OUTPUT_QUIET)
    if(_iolib_host_app EQUAL 0)
        message(STATUS "ioLibrary DHCP hostname patch: applied")
    else()
        message(WARNING "ioLibrary DHCP hostname patch did not apply — verify "
            "Internet/DHCP/dhcp.c HOST_NAME handling and "
            "patches/0002-dhcp-configurable-hostname.patch.")
    endif()
endif()

target_sources(light_switch PRIVATE
    ${IOLIB}/Ethernet/wizchip_conf.c
    ${IOLIB}/Ethernet/socket.c
    ${IOLIB}/Ethernet/W5100S/w5100s.c
    ${IOLIB}/Internet/DHCP/dhcp.c
    ${CMAKE_CURRENT_LIST_DIR}/src/net/wizchip_glue.c
    ${CMAKE_CURRENT_LIST_DIR}/src/net/netif.c
    ${CMAKE_CURRENT_LIST_DIR}/src/net/mqtt_app.c
    ${CMAKE_CURRENT_LIST_DIR}/src/net/ha_bridge.c
    ${CMAKE_CURRENT_LIST_DIR}/src/net/web_state.c
    ${CMAKE_CURRENT_LIST_DIR}/src/net/http_server.c
    ${CMAKE_CURRENT_LIST_DIR}/src/net/ping.c
    ${GEN_WEB_ASSETS}                  # generated: gzipped Vite UI (web/)
)

# ── coreMQTT (FreeRTOS standalone; static, malloc/float-free) ───────────────
include(${CMAKE_CURRENT_LIST_DIR}/lib/coreMQTT/mqttFilePaths.cmake)
target_sources(light_switch PRIVATE ${MQTT_SOURCES} ${MQTT_SERIALIZER_SOURCES})
target_include_directories(light_switch PRIVATE ${MQTT_INCLUDE_PUBLIC_DIRS})
target_include_directories(light_switch PRIVATE
    ${IOLIB}/Ethernet
    ${IOLIB}/Ethernet/W5100S
    ${IOLIB}/Internet/DHCP
    ${CMAKE_CURRENT_LIST_DIR}/src/net
)
target_compile_definitions(light_switch PRIVATE _WIZCHIP_=W5100S)
target_link_libraries(light_switch PRIVATE hardware_spi pico_unique_id)

# Vendored third-party: don't subject ioLibrary to our -Wall -Wextra.
set_source_files_properties(
    ${IOLIB}/Ethernet/wizchip_conf.c
    ${IOLIB}/Ethernet/socket.c
    ${IOLIB}/Ethernet/W5100S/w5100s.c
    ${IOLIB}/Internet/DHCP/dhcp.c
    PROPERTIES COMPILE_OPTIONS "-w")

# USB CDC for serial protocol (replaces Lua REPL)
pico_enable_stdio_usb(light_switch 1)
pico_enable_stdio_uart(light_switch 0)

# Make stdout non-blocking: with the timeout at 0, stdio_usb_out_chars writes
# only what the CDC TX FIFO can take and returns immediately instead of spinning
# until the host drains it. This is what stops a full FIFO (e.g. a `log dbg`
# firehose with a slow/absent reader) from stalling a core past the 5 s watchdog.
# Log output is additionally buffered + dropped-with-a-count via src/cdc_out.c;
# this define also covers the CLI's direct printf responses.
target_compile_definitions(light_switch PRIVATE PICO_STDIO_USB_STDOUT_TIMEOUT_US=0)

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
