#[[
   @file CMakeLists.txt

   @brief

   @copyright
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear

]]
cmake_minimum_required(VERSION 3.10)

#Include direectories
include_directories(
                    ../core/src
                   )

#Add the sub directories
add_subdirectory(../core/build core)
add_subdirectory(../ext/build ext)
