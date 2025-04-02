#[[
   @file CMakeLists.txt

   @brief

   @copyright
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause
]]
cmake_minimum_required(VERSION 3.10)

#Include directories
include_directories(
                    ${LIB_ROOT}/api
                    ../lib/inc
                    ../../../../../fwk/spf/interfaces/module/shared_lib_api/inc/generic
                    ../../../../cmn/common/utils/inc
                   )

#Add the sub directories
add_subdirectory(../lib/build lib)
