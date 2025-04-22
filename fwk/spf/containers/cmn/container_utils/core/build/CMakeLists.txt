#[[
   @file CMakeLists.txt

   @brief

   @copyright
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear

]]
cmake_minimum_required(VERSION 3.10)

#Include directories
set (lib_incs_list
     ${LIB_ROOT}/inc
    )

#Add the source files
set (lib_srcs_list
     ${LIB_ROOT}/src/cu.c
     ${LIB_ROOT}/src/cu_buf_util.c
     ${LIB_ROOT}/src/cu_cmd_handler.c
     ${LIB_ROOT}/src/cu_data_handler.c
     ${LIB_ROOT}/src/cu_data_msg_handler.c
     ${LIB_ROOT}/src/cu_events.c
     ${LIB_ROOT}/src/cu_ext_port_util.c
     ${LIB_ROOT}/src/cu_gpr_if.c
     ${LIB_ROOT}/src/cu_island.c
     ${LIB_ROOT}/src/cu_pm.c
     ${LIB_ROOT}/src/cu_state_handler.c
     ${LIB_ROOT}/src/cu_utils.c
     ${LIB_ROOT}/src/cu_utils_island.c
     ${LIB_ROOT}/src/cu_prebuffer.c
     ${LIB_ROOT}/src/cu_utils_qsh_audio_island.c
    )

#Add the compiler flags
set (lib_flgs_list
     -Wno-address-of-packed-member
    )

#Call spf_build_static_library to generate the static library
spf_build_static_library(container_utils_core
                         "${lib_incs_list}"
                         "${lib_srcs_list}"
                         "${lib_defs_list}"
                         "${lib_flgs_list}"
                         "${lib_link_libs_list}"
                        )
