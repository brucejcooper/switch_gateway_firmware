cmake_minimum_required(VERSION 3.13)

set(PICO_PLATFORM "rp2350")
set(PICO_SDK_PATH "lib/pico-sdk")


include(${PICO_SDK_PATH}/pico_sdk_init.cmake)
# include(FreeRTOS_Kernel_import.cmake)



add_compile_definitions(LUA_PATH_VAR="/")
# add_compile_definitions(LUAI_ASSERT)

project(light_switch)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()



add_executable(light_switch
    src/main.c
    src/modbus.c
    src/button_actions.c
    src/lualoop.c

    lib/lua-5.4.8/src/lapi.c 
    lib/lua-5.4.8/src/lcode.c 
    lib/lua-5.4.8/src/lctype.c 
    lib/lua-5.4.8/src/ldebug.c 
    lib/lua-5.4.8/src/ldo.c 
    lib/lua-5.4.8/src/ldump.c 
    lib/lua-5.4.8/src/lfunc.c 
    lib/lua-5.4.8/src/lgc.c 
    lib/lua-5.4.8/src/llex.c 
    lib/lua-5.4.8/src/lmem.c 
    lib/lua-5.4.8/src/lobject.c 
    lib/lua-5.4.8/src/lopcodes.c 
    lib/lua-5.4.8/src/lparser.c 
    lib/lua-5.4.8/src/lstate.c 
    lib/lua-5.4.8/src/lstring.c 
    lib/lua-5.4.8/src/ltable.c 
    lib/lua-5.4.8/src/ltm.c 
    lib/lua-5.4.8/src/lundump.c 
    lib/lua-5.4.8/src/lvm.c 
    lib/lua-5.4.8/src/lzio.c
    lib/lua-5.4.8/src/lauxlib.c 
    lib/lua-5.4.8/src/lbaselib.c 
    lib/lua-5.4.8/src/lcorolib.c 
    lib/lua-5.4.8/src/ldblib.c 
    lib/lua-5.4.8/src/liolib.c 
    lib/lua-5.4.8/src/lmathlib.c 
    lib/lua-5.4.8/src/loadlib.c 
    lib/lua-5.4.8/src/loslib.c 
    lib/lua-5.4.8/src/lstrlib.c 
    lib/lua-5.4.8/src/ltablib.c 
    lib/lua-5.4.8/src/lutf8lib.c 
    lib/lua-5.4.8/src/linit.c
)
add_subdirectory(lib/pico-vfs)

target_include_directories(light_switch PRIVATE 
    ${CMAKE_CURRENT_LIST_DIR}/src 
    ${CMAKE_CURRENT_LIST_DIR}/lib/lua-5.4.8/src
    ${CMAKE_CURRENT_LIST_DIR}/lib/littlefs
)



# pull in common dependencies
target_link_libraries(light_switch PRIVATE 
    pico_stdlib 
    hardware_pio 
    pico_flash
    # FreeRTOS-Kernel-Heap3
    # blockdevice_heap
    # filesystem_littlefs
    # filesystem_vfs
)

pico_enable_stdio_usb(light_switch 1)
pico_enable_stdio_uart(light_switch 0)
pico_enable_filesystem(light_switch
    SIZE 262144         # 256KB for filesystem
    AUTO_INIT 0         # Auto-initialize
    MAX_FAT_VOLUME 1    # Support 2 FAT volumes
    MAX_MOUNTPOINT 2    # Support 4 mount points
)


if (PICO_CYW43_SUPPORTED)
    target_link_libraries(light_switch pico_cyw43_arch_none)
endif()

pico_generate_pio_header(light_switch ${CMAKE_CURRENT_LIST_DIR}/src/buttongrid.pio)

pico_add_extra_outputs(light_switch)

