cmake_minimum_required(VERSION 3.16)
project(sw2_transport C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Strict warnings
add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)

# Release builds: optimise; Debug builds: keep symbols + sanitizers
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_options(-g -fsanitize=address,undefined)
    add_link_options(-fsanitize=address,undefined)
else()
    add_compile_options(-O2)
endif()

# --------------------------------------------------------------------------
# Dependencies
# --------------------------------------------------------------------------

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

# BlueZ / libbluetooth (for raw HCI / L2CAP ATT sockets)
find_library(BLUETOOTH_LIB bluetooth REQUIRED)

# --------------------------------------------------------------------------
# Transport static library
# --------------------------------------------------------------------------

add_library(sw2_transport STATIC
    src/transport_usb.c
    src/transport_ble.c
)

target_include_directories(sw2_transport
    PUBLIC  include
    PRIVATE ${LIBUSB_INCLUDE_DIRS}
)

target_link_libraries(sw2_transport
    PRIVATE ${LIBUSB_LIBRARIES}
    PRIVATE ${BLUETOOTH_LIB}
)

# --------------------------------------------------------------------------
# Minimal smoke-test binary  (optional, remove when building as a subdir)
# --------------------------------------------------------------------------

option(SW2_BUILD_TOOLS "Build the sw2ctl test tool" ON)

if(SW2_BUILD_TOOLS)
    add_executable(sw2ctl tools/sw2ctl.c)
    target_link_libraries(sw2ctl PRIVATE sw2_transport)
endif()