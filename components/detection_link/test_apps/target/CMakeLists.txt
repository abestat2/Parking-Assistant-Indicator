# On-target tests for the detection_link component (Tiers 1 and 3). Runs on an ESP32-C3.
cmake_minimum_required(VERSION 3.22)

set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../..")
set(COMPONENTS main)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(detection_link_target_test)
