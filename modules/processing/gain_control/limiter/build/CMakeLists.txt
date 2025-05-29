#[[
   @file CMakeLists.txt

   @brief

   @copyright
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear
]]
cmake_minimum_required(VERSION 3.10)

set(lib_incs_list
   ${LIB_ROOT}/api
   ${LIB_ROOT}/lib/inc
   ${PROJECT_SRC_DIR}/modules/cmn/common/utils/inc
)

set(lib_srcs_list
   ${LIB_ROOT}/lib/src/limiter.c
   ${LIB_ROOT}/lib/src/limiter24.c
)

#Call spf_build_static_library to generate the static library
spf_build_static_library(limiter
                         "${lib_incs_list}"
                         "${lib_srcs_list}"
                         "${lib_defs_list}"
                         "${lib_flgs_list}"
                         "${lib_link_libs_list}"
                        )
