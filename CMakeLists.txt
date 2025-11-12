cmake_minimum_required(VERSION 3.13)

set(PICO_PLATFORM "rp2350")
set(PICO_SDK_PATH "lib/pico-sdk")


include(${PICO_SDK_PATH}/pico_sdk_init.cmake)
include(FreeRTOS_Kernel_import.cmake)



project(light_switch)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()



add_executable(light_switch
    src/main.c
    src/modbus.c
    src/button_actions.c
    src/entity_status.c
    src/connection_tracking.c
    src/dali_provision.c
    lib/cJSON/cJSON.c
    )

add_subdirectory(lib/pico-lfs)

target_include_directories(light_switch PRIVATE 
    ${CMAKE_CURRENT_LIST_DIR}/src 
    ${CMAKE_CURRENT_LIST_DIR}/lib/cJSON
)



# pull in common dependencies
target_link_libraries(light_switch pico_stdlib hardware_pio pico_multicore  pico_async_context_freertos FreeRTOS-Kernel-Heap4)

pico_enable_stdio_usb(light_switch 1)
pico_enable_stdio_uart(light_switch 0)

target_compile_definitions(light_switch PRIVATE
  LFS_THREADSAFE=1
  LFS_NO_DEBUG=1
)


if (PICO_CYW43_SUPPORTED)
    target_link_libraries(light_switch pico_cyw43_arch_none)
endif()

pico_generate_pio_header(light_switch ${CMAKE_CURRENT_LIST_DIR}/src/buttongrid.pio)

pico_add_extra_outputs(light_switch)