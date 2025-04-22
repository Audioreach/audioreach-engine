#[[
   @file CMakeLists.txt

   @brief

   @copyright
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear
]]
cmake_minimum_required(VERSION 3.10)

set(msiir_sources
    ${LIB_ROOT}/capi/src/capi_multistageiir.cpp
    ${LIB_ROOT}/capi/src/capi_multistageiir_utils.cpp
    ${LIB_ROOT}/capi/src/capi_multistageiir_utils_v2.cpp
    ${LIB_ROOT}/lib/src/CMultiStageIIR.c
    ${LIB_ROOT}/lib/src/msiir.c
)

set(msiir_includes
    ${LIB_ROOT}/api
    ${LIB_ROOT}/capi/inc
    ${LIB_ROOT}/capi/src
    ${LIB_ROOT}/lib/inc
    ${LIB_ROOT}/lib/src
    ../../../../cmn/common/utils/inc/
    ../../../../../fwk/spf/interfaces/module/shared_lib_api/inc/generic
)

spf_module_sources(
   KCONFIG     CONFIG_MSIIR
   NAME        msiir
   MAJOR_VER   1
   MINOR_VER   0
   AMDB_ITYPE  "capi"
   AMDB_MTYPE  "PP"
   AMDB_MID "0x07001014"
   AMDB_TAG "capi_multistageiir"
   AMDB_MOD_NAME  "MODULE_ID_MSIIR"
   SRCS     ${msiir_sources}
   INCLUDES ${msiir_includes}
   H2XML_HEADERS  "${LIB_ROOT}/api/api_msiir.h"
   CFLAGS      ""
)
