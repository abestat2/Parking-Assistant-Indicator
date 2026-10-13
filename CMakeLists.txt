cmake_minimum_required(VERSION 3.16)

# Shared components from git submodules (git submodule update --init)
list(APPEND EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/external/ESP-NOW-Transport")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(parking_assistant)