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

# COVERAGE=1 instruments the vl53l1x component (including the ST ULD) for gcov; see run_host_tests.sh
if("$ENV{COVERAGE}" STREQUAL "1")
    idf_component_get_property(vl53l1x_lib vl53l1x COMPONENT_LIB)
    target_compile_options(${vl53l1x_lib} PRIVATE --coverage -O0)
    target_link_options(${CMAKE_PROJECT_NAME}.elf PRIVATE --coverage)
endif()
