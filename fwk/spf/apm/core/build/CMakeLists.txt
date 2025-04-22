#[[
   @file CMakeLists.txt

   @brief

   @copyright
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear

]]
cmake_minimum_required(VERSION 3.10)

#Add the souerce files
set (lib_srcs_list
     ${LIB_ROOT}/src/apm.c
     ${LIB_ROOT}/src/apm_cmd_sequencer.c
     ${LIB_ROOT}/src/apm_cmd_utils.c
     ${LIB_ROOT}/src/apm_gpr_cmd_handler.c
     ${LIB_ROOT}/src/apm_gpr_cmd_parser.c
     ${LIB_ROOT}/src/apm_gpr_if.c
     ${LIB_ROOT}/src/apm_graph_db.c
     ${LIB_ROOT}/src/apm_msg_rsp_handler.c
     ${LIB_ROOT}/src/apm_msg_utils.c
    )

#Call spf_build_static_library to generate the static library
spf_build_static_library(APM_CORE
                         "${lib_incs_list}"
                         "${lib_srcs_list}"
                         "${lib_defs_list}"
                         "${lib_flgs_list}"
                         "${lib_link_libs_list}"
                        )
