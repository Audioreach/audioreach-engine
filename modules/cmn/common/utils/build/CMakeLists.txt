#[[
   @file CMakeLists.txt

   @brief

   @copyright
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause
]]
cmake_minimum_required(VERSION 3.10)

#Include directories
set (lib_incs_list
     ${LIB_ROOT}/inc
    )

#Add the source files
set (lib_srcs_list
     ${LIB_ROOT}/src/exp10.c
     ${LIB_ROOT}/src/log10.c
     ${LIB_ROOT}/src/basic_op.c
     ${LIB_ROOT}/src/audio_basic_op_ext.c
     ${LIB_ROOT}/src/crossfade.c
     ${LIB_ROOT}/src/audio_buffer.cpp
     ${LIB_ROOT}/src/audio_buffer32.c
     ${LIB_ROOT}/src/iir_tdf2.c
     ${LIB_ROOT}/src/simple_mm.c
     ${LIB_ROOT}/src/divide_qx.c
     ${LIB_ROOT}/src/basic_math.c
     ${LIB_ROOT}/src/audio_panner.cpp
     ${LIB_ROOT}/src/audio_buffer32_island.c
     ${LIB_ROOT}/src/divide_qx_island.c
     ${LIB_ROOT}/src/mathlib.c
     ${LIB_ROOT}/src/audio_complex_basic_op.c
     #${LIB_ROOT}/src/filterDesign.c
     ${LIB_ROOT}/src/util.c
     ${LIB_ROOT}/src/audio_delay32.c
     ${LIB_ROOT}/src/clips.c
     ${LIB_ROOT}/src/apiir_df1_opt.c
    )

#Call spf_build_static_library to generate the static library
spf_build_static_library(mod_utils
                         "${lib_incs_list}"
                         "${lib_srcs_list}"
                         "${lib_defs_list}"
                         "${lib_flgs_list}"
                         "${lib_link_libs_list}"
                        )
