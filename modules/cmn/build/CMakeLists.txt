#[[
   @file CMakeLists.txt

   @brief

   @copyright
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear
]]
cmake_minimum_required(VERSION 3.10)

#Include directories
include_directories(
                    ${LIB_ROOT}/api
                   )

#Add the sub directories
add_subdirectory(${LIB_ROOT}/common/build common)
