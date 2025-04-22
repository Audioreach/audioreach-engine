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
                    ../../core/src
                   )

#Add the sub directories
add_subdirectory(../ctrl_port/build ctrl_port)
add_subdirectory(../duty_cycle/build duty_cycle)
add_subdirectory(../island_exit/build island_exit)
add_subdirectory(../offload/build offload)
add_subdirectory(../path_delay/build path_delay)
add_subdirectory(../prof/build prof)
add_subdirectory(../soft_timer_fwk_ext/build soft_timer_fwk_ext)
add_subdirectory(../voice/build voice)
add_subdirectory(../global_shmem_msg/build cu_global_shmem_msg_ext)
