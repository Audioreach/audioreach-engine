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
     ${LIB_ROOT}/src/gen_cntr.c
     ${LIB_ROOT}/src/gen_cntr_buf_util.c
     ${LIB_ROOT}/src/gen_cntr_cmd_handler.c
     ${LIB_ROOT}/src/gen_cntr_data_handler.c
     ${LIB_ROOT}/src/gen_cntr_data_handler_island.c
     ${LIB_ROOT}/src/gen_cntr_data_msg_handler.c
     ${LIB_ROOT}/src/gen_cntr_island.c
     ${LIB_ROOT}/src/gen_cntr_pm.c
     ${LIB_ROOT}/src/gen_cntr_st_handler.c
     ${LIB_ROOT}/src/gen_cntr_st_handler_island.c
     ${LIB_ROOT}/src/gen_cntr_trigger_policy.c
     ${LIB_ROOT}/src/gen_cntr_trigger_policy_island.c
     ${LIB_ROOT}/src/gen_cntr_utils.c
     ${LIB_ROOT}/src/gen_cntr_utils_island.c
     ${LIB_ROOT}/src/gen_cntr_utils_md_island.c
     ${LIB_ROOT}/src/gen_cntr_fwk_extn_utils.c
     ${LIB_ROOT}/src/gen_cntr_fwk_extn_utils_island.c
    )

#Call spf_build_static_library to generate the static library
spf_build_static_library(gen_cntr_main
                         "${lib_incs_list}"
                         "${lib_srcs_list}"
                         "${lib_defs_list}"
                         "${lib_flgs_list}"
                         "${lib_link_libs_list}"
                        )
