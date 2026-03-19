cmake_minimum_required(VERSION 3.10)
project(ConHid)
set(CMAKE_C_STANDARD 11)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

add_executable(conhid main.c initializer.c controller.c virtualdevice.c transport.c calibration.c)

target_link_libraries(conhid ${LIBUSB_LIBRARIES} pthread)
target_include_directories(conhid PUBLIC ${LIBUSB_INCLUDE_DIRS})