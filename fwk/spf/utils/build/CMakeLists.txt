#[[
   @file CMakeLists.txt

   @brief

   @copyright
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear

]]
cmake_minimum_required(VERSION 3.10)

#Add the sub directories
add_subdirectory(../cmn/build cmn)
add_subdirectory(../interleaver/build interleaver)
add_subdirectory(../list/build list)
add_subdirectory(../lpi_pool/build lpi_pool)
add_subdirectory(../watchdog_svc/build watchdog_svc)
