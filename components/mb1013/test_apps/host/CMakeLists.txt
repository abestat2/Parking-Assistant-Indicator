# Host tests for the mb1013 component (Tiers 1 and 2). Build with IDF_TARGET=linux
# on Linux, macOS or WSL; the RMT driver and FreeRTOS queue calls are CMock mocks.
cmake_minimum_required(VERSION 3.22)

set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/../.."
    "$ENV{IDF_PATH}/tools/mocks/driver/"
    "$ENV{IDF_PATH}/tools/mocks/freertos/")
set(COMPONENTS main)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(mb1013_host_test)
