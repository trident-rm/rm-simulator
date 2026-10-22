# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
# Optional offline CLI build; third-party sources are supplied by the caller.
cmake_minimum_required(VERSION 3.20)
project(rm_openswx_extractor LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(OPENPARASOLID_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
set(OPENSWX_SOURCE "" CACHE PATH "Path to a checked openswx source checkout")
if(NOT EXISTS "${OPENSWX_SOURCE}/libopenswx/CMakeLists.txt")
  message(FATAL_ERROR "Set OPENSWX_SOURCE to the openswx checkout")
endif()
add_subdirectory("${OPENSWX_SOURCE}/libopenswx" openswx)
find_package(nlohmann_json CONFIG REQUIRED)
add_executable(swx_dump "${OPENSWX_SOURCE}/asmbox/src/linux/swx_dump.cc")
target_link_libraries(swx_dump PRIVATE openswx nlohmann_json::nlohmann_json)
