#[[
   @file CMakeLists.txt

   @brief

   @copyright
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear
]]
cmake_minimum_required(VERSION 3.10)

set(bass_boost_sources
    ${LIB_ROOT}/capi/src/capi_bassboost.cpp
    ${LIB_ROOT}/capi/src/capi_bassboost_kpps.cpp
    ${LIB_ROOT}/capi/src/capi_bassboost_utils.cpp
    ${LIB_ROOT}/lib/src/bassboost.c
)

set(bass_boost_includes
    ${LIB_ROOT}/api
    ${LIB_ROOT}/capi/inc
    ${LIB_ROOT}/capi/src
    ${LIB_ROOT}/lib/inc
    ${LIB_ROOT}/lib/src
    ${PROJECT_SOURCE_DIR}/modules/cmn/common/utils/inc
    ${PROJECT_SOURCE_DIR}/modules/processing/filters/multi_stage_iir/lib/inc
    ${PROJECT_SOURCE_DIR}/modules/processing/gain_control/limiter/lib/inc
    ${PROJECT_SOURCE_DIR}/modules/processing/gain_control/drc/lib/inc
    ${PROJECT_SOURCE_DIR}/fwk/spf/interfaces/module/shared_lib_api/inc/generic
)

spf_module_sources(
   KCONFIG     CONFIG_BASS_BOOST
   NAME        bass_boost
   MAJOR_VER   1
   MINOR_VER   0
   AMDB_ITYPE  "capi"
   AMDB_MTYPE  "PP"
   AMDB_MID "0x07001062"
   AMDB_TAG "capi_bassboost"
   AMDB_MOD_NAME  "MODULE_ID_BASS_BOOST"
   SRCS     ${bass_boost_sources}
   INCLUDES ${bass_boost_includes}
   H2XML_HEADERS  "${LIB_ROOT}/api/api_bassboost.h"
   CFLAGS      ""
)
