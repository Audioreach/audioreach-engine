#[[
   @file CMakeLists.txt

   @brief

   @copyright
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear
]]
cmake_minimum_required(VERSION 3.10)

set(fir_sources
    ${LIB_ROOT}/capi/src/capi_fir_filter.cpp
    ${LIB_ROOT}/capi/src/capi_fir_filter_utils.cpp
    ${LIB_ROOT}/capi/src/capi_fir_filter_utils_v2.cpp
    ${LIB_ROOT}/capi/src/capi_fir_filter_xfade_utils.cpp
    ${LIB_ROOT}/lib/src/fir_lib.c
    ${LIB_ROOT}/lib/src/fir_lib_process.c
)

set(fir_includes
    ${LIB_ROOT}/api
    ${LIB_ROOT}/capi/inc
    ${LIB_ROOT}/capi/src
    ${LIB_ROOT}/lib/inc
    ${LIB_ROOT}/lib/src
    ../../../../cmn/common/utils/inc
)

spf_module_sources(
   KCONFIG     CONFIG_FIR_FILTER
   NAME        fir
   MAJOR_VER   1
   MINOR_VER   0
   AMDB_ITYPE  "capi"
   AMDB_MTYPE  "PP"
   AMDB_MID "0x07001022"
   AMDB_TAG "capi_fir"
   AMDB_MOD_NAME  "MODULE_ID_FIR_FILTER"
   SRCS     ${fir_sources}
   INCLUDES ${fir_includes}
   H2XML_HEADERS  "${LIB_ROOT}/api/api_fir.h"
   CFLAGS      ""
)
