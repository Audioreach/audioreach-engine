#[[
   @file CMakeLists.txt

   @brief

   @copyright
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear

]]
cmake_minimum_required(VERSION 3.10)

#Add the source files
set(lib_srcs_list
     ${LIB_ROOT}/src/apm_proxy_vcpm_cmd_rsp_handler.c
     ${LIB_ROOT}/src/apm_proxy_vcpm_utils.c
    )

set(lib_incs_list
   ${LIB_ROOT}/src/apm_proxy_vcpm_utils_i.h
   ${LIB_ROOT}/inc/apm_proxy_vcpm_utils.h
)

include_directories(
   ../vcpm/inc/
)

add_subdirectory(../vcpm/build vcpm)

#Call spf_build_static_library to generate the static library
spf_build_static_library(vcpm_prxy_mgr
                         "${lib_incs_list}"
                         "${lib_srcs_list}"
                         "${lib_defs_list}"
                         "${lib_flgs_list}"
                         "${lib_link_libs_list}"
                        )
