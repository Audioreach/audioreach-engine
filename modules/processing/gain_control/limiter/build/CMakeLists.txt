#[[
   @file CMakeLists.txt

   @brief

   @copyright
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear
]]
cmake_minimum_required(VERSION 3.10)

add_library(limiter STATIC IMPORTED GLOBAL)

set(lib_abs_path ${CMAKE_CURRENT_SOURCE_DIR}/../bin/arm/liblimiter.a)

set_target_properties(limiter PROPERTIES IMPORTED_LOCATION ${lib_abs_path})
set_property(GLOBAL APPEND PROPERTY GLOBAL_SPF_LIBS_LIST limiter)
