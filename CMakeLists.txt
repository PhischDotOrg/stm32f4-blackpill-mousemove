#-
# $Copyright$
#
cmake_minimum_required(VERSION 3.12)
project(stm32f4-blackpill-mousemove)

###############################################################################
# Enable C++17 Support according to:
#
# https://crascit.com/2015/03/28/enabling-cxx11-in-cmake/
#
# Google Test requires > C++11.
###############################################################################
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

###############################################################################
# Include Unit Testing / Code Coverage Support
###############################################################################
include(common/UnitTest.cmake)
include(common/Coverage.cmake)

###############################################################################
# Project-specific configuration
###############################################################################
set(TARGET_NAME     firmware)
set(TARGET_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/UsbDescriptors.cpp
)

set(STM32_BOARD   STM32F4_BlackPill     CACHE STRING "STM32 Board Type")

# FIXME Adding this breaks the Hostbuild / Test Cases for USB
# This is b/c the USB_PRINTF resolves to g_uart.printf() and the
# g_uart Symbol is not defined.
add_definitions("-DUSB_DEBUG")

###############################################################################
# Include a common CMake File that contains the stuff which would normally go
# in a top-level CMake File.
###############################################################################
add_subdirectory(common)
