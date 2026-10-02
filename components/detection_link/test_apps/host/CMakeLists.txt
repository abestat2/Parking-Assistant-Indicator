# Host tests for the detection_link component (Tiers 1 and 2). Build with IDF_TARGET=linux
# on Linux, macOS or WSL; ESP-NOW and its Wi-Fi dependencies are CMock mocks.
cmake_minimum_required(VERSION 3.22)

set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/../.."
    "$ENV{IDF_PATH}/tools/mocks/esp_wifi/"
    "$ENV{IDF_PATH}/tools/mocks/esp_netif/"
    "$ENV{IDF_PATH}/tools/mocks/esp_event/"
    "$ENV{IDF_PATH}/tools/mocks/lwip/")
set(COMPONENTS main)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(detection_link_host_test)
