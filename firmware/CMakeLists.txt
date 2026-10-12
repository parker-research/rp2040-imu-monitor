cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(rp2040_imu_monitor C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# RP2040-Zero is an RP2040 board.
set(PICO_BOARD pico)

pico_sdk_init()

# ---------------------------------------------------------------------------
# User-tunable build options (override with e.g. `cmake -DREPORT_INTERVAL_MS=100 ..`).
# ---------------------------------------------------------------------------
set(REPORT_INTERVAL_MS 250 CACHE STRING
    "Default interval between JSONL sample reports (ms). Can also be changed at runtime over serial.")
set(LSM303_LEGACY_VARIANT "AUTO" CACHE STRING
    "How to resolve the DLH-vs-DLHC ambiguity (both use accel 0x19 + mag 0x1E): AUTO, DLH, or DLHC.")
set_property(CACHE LSM303_LEGACY_VARIANT PROPERTY STRINGS AUTO DLH DLHC)
option(STATUS_LED_GRB_ORDER
    "WS2812 colour order is GRB (standard WS2812B). Set OFF for LEDs wired as RGB order." ON)

set(CMAKE_EXECUTABLE_SUFFIX ".elf")
add_executable(rp2040_imu_monitor
    main.c
    lsm303.c
    status_led.c
)

pico_generate_pio_header(rp2040_imu_monitor ${CMAKE_CURRENT_SOURCE_DIR}/ws2812.pio)

# Generate build_info.h at make-time (note: not at cmake configure time).
add_custom_target(generate_build_info ALL
    COMMAND ${CMAKE_COMMAND}
        -DOUTPUT_FILE=${CMAKE_CURRENT_BINARY_DIR}/build_info.h
        -DSOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
        -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/generate_build_info.cmake
    BYPRODUCTS ${CMAKE_CURRENT_BINARY_DIR}/build_info.h
    COMMENT "Generating build_info.h"
    VERBATIM
)
target_include_directories(rp2040_imu_monitor PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
add_dependencies(rp2040_imu_monitor generate_build_info)

target_compile_definitions(rp2040_imu_monitor PRIVATE
    DEFAULT_REPORT_INTERVAL_MS=${REPORT_INTERVAL_MS}
    LSM303_LEGACY_VARIANT_${LSM303_LEGACY_VARIANT}=1
    STATUS_LED_GRB_ORDER=$<BOOL:${STATUS_LED_GRB_ORDER}>
)

target_compile_options(rp2040_imu_monitor PRIVATE -O2 -Wall -Wextra)

target_link_libraries(rp2040_imu_monitor
    pico_stdlib
    hardware_adc
    hardware_gpio
    hardware_i2c
    hardware_pio
    hardware_watchdog
)

# USB-CDC for serial; disable UART stdio so we don't burn pins.
pico_enable_stdio_usb(rp2040_imu_monitor  1)
pico_enable_stdio_uart(rp2040_imu_monitor 0)

# Produce .uf2 output to flash.
pico_add_bin_output(rp2040_imu_monitor)
pico_add_uf2_output(rp2040_imu_monitor)
