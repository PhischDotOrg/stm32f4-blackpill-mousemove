#-
# $Copyright$
#
cmake_minimum_required(VERSION 3.12)
project(stm32f4-minimal)

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
)

set(STM32_BOARD   STM32F4_Discovery    CACHE STRING "STM32 Board Type")

###############################################################################
# Include a common CMake File that contains the stuff which would normally go
# in a top-level CMake File.
###############################################################################
add_subdirectory(common)
