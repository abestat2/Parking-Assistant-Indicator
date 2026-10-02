# Host tests for the vl53l1x component (Tiers 1 and 2). Build with IDF_TARGET=linux
# on Linux, macOS or WSL; GPIO and FreeRTOS calls are CMock mocks and the sensor is simulated.
cmake_minimum_required(VERSION 3.22)

set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/../.."
    "${CMAKE_CURRENT_LIST_DIR}/../../../i2c_bus"
    "$ENV{IDF_PATH}/tools/mocks/driver/"
    "$ENV{IDF_PATH}/tools/mocks/freertos/")
set(COMPONENTS main)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(vl53l1x_host_test)
