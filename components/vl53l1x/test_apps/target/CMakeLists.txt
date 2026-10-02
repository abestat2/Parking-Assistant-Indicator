# On-target tests for the vl53l1x component (Tiers 1 and 3). Runs on an ESP32-C3.
cmake_minimum_required(VERSION 3.22)

set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/../.."
    "${CMAKE_CURRENT_LIST_DIR}/../../../i2c_bus")
set(COMPONENTS main)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(vl53l1x_target_test)
