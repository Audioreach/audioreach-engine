#[[
   @file CMakeLists.txt

   @brief

   @copyright
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear

]]
cmake_minimum_required(VERSION 3.10)

#Add the source files
set (lib_srcs_list
     ${LIB_ROOT}/stub_src/apm_proxy_vcpm_stub.c
    )

#Call spf_build_static_library to generate the static library
spf_build_static_library(vcpm_prxy_mgr
                         "${lib_incs_list}"
                         "${lib_srcs_list}"
                         "${lib_defs_list}"
                         "${lib_flgs_list}"
                         "${lib_link_libs_list}"
                        )
