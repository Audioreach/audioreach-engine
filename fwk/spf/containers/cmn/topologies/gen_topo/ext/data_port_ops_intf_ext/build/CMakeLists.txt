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
     ${LIB_ROOT}/src/gen_topo_data_port_ops_intf_ext.c
    )

#Call spf_build_static_library to generate the static library
spf_build_static_library(gen_topo_data_port_ops_intf_ext
                         "${lib_incs_list}"
                         "${lib_srcs_list}"
                         "${lib_defs_list}"
                         "${lib_flgs_list}"
                         "${lib_link_libs_list}"
                        )
