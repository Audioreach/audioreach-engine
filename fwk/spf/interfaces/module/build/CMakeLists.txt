#[[
   @file CMakeLists.txt

   @brief

   @copyright
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear

]]
cmake_minimum_required(VERSION 3.10)

#Add the sub directories
add_subdirectory(../capi_cmn/build capi_cmn)
add_subdirectory(../capi_libraries/build capi_libraries)
add_subdirectory(../shared_lib_api/build shared_lib_api)
