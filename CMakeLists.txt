cmake_minimum_required(VERSION 3.10)
project(ConHidCpp)

set(CMAKE_CXX_STANDARD 17)

# Find libusb (since you'll need it for the pump)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

add_executable(conhid_test main.cpp initializer.cpp controller.cpp virtualdevice.cpp)

# Link the libraries
target_link_libraries(conhid_test ${LIBUSB_LIBRARIES})
target_include_directories(conhid_test PUBLIC ${LIBUSB_INCLUDE_DIRS})