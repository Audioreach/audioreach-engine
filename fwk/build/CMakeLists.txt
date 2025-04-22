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
                    ../spf/dls/inc
                    ../spf/interfaces/fwk/api
                    ../spf/utils/lpi_pool/inc
                    ../spf/utils/list/inc
                    ../platform/core_drv/platform_cfg/inc
                    ../platform/core_drv/inc
                    ../platform/core_drv/voice_timer/inc
                    ../platform/init/amdb/inc
                    ../platform/private/api
                   )

#Add the sub directories
add_subdirectory(../platform/build platform)
add_subdirectory(../spf/build spf)

#Build utest folder only when DISABLE_UNIT_TEST flag is disabled
if(NOT DISABLE_UNIT_TEST)
   add_subdirectory(../utest/build utest)
endif()
