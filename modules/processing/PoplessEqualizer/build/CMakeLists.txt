#[[
   @file CMakeLists.txt

   @brief

   @copyright
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear
]]
cmake_minimum_required(VERSION 3.10)

set(popless_equalizer_sources
    ${LIB_ROOT}/capi/src/capi_popless_equalizer.cpp
    ${LIB_ROOT}/capi/src/capi_popless_equalizer_kpps.cpp
    ${LIB_ROOT}/capi/src/capi_popless_equalizer_utils.cpp
    ${LIB_ROOT}/lib/src/equalizer.c
    ${LIB_ROOT}/lib/src/equalizer_filter_design.c
    ${LIB_ROOT}/lib/src/equalizer_preset.c
    ${LIB_ROOT}/lib/src/tangent.c
)

set(popless_equalizer_includes
    ${LIB_ROOT}/api
    ${LIB_ROOT}/capi/inc
    ${LIB_ROOT}/capi/src
    ${LIB_ROOT}/lib/inc
    ${LIB_ROOT}/lib/src
    ${PROJECT_SOURCE_DIR}/modules/cmn/api
    ${PROJECT_SOURCE_DIR}/modules/cmn/common/internal_api
    ${PROJECT_SOURCE_DIR}/modules/processing/filters/multi_stage_iir/lib/inc
)

spf_module_sources(
   KCONFIG     CONFIG_POPLESS_EQUALIZER
   NAME        popless_equalizer
   MAJOR_VER   1
   MINOR_VER   0
   AMDB_ITYPE  "capi"
   AMDB_MTYPE  "PP"
   AMDB_MID    "0x07001045"
   AMDB_TAG    "capi_p_eq"
   AMDB_MOD_NAME  "MODULE_ID_POPLESS_EQUALIZER"
   AMDB_FMT_ID1   "MEDIA_FMT_ID_PCM"
   SRCS     ${popless_equalizer_sources}
   INCLUDES ${popless_equalizer_includes}
   H2XML_HEADERS  "${LIB_ROOT}/api/popless_equalizer_api.h"
   CFLAGS      ""
)
