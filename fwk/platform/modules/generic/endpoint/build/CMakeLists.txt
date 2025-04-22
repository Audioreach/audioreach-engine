#[[
   @file CMakeLists.txt

   @brief

   @copyright
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear

]]

#Include directories
include_directories(
                    ../alsa_device/api
                   )

add_subdirectory(../alsa_device/build alsa_device)
