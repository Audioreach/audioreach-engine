#[[
   @file CMakeLists.txt

   @brief

   @copyright
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause
]]
cmake_minimum_required(VERSION 3.10)

set(drc_sources
    ${LIB_ROOT}/capi/src/capi_drc.cpp
    ${LIB_ROOT}/capi/src/capi_drc_utils.cpp
    ${LIB_ROOT}/lib/src/drc_lib.c
    ${LIB_ROOT}/lib/src/drc_lib_opt.c
)

set(drc_includes
    ${LIB_ROOT}/api
    ${LIB_ROOT}/capi/inc
    ${LIB_ROOT}/capi/src
    ${LIB_ROOT}/lib/inc
    ${LIB_ROOT}/lib/src
    ../../../../cmn/common/utils/inc
    ../../../../cmn/api
    ../../../../cmn/common/internal_api
)

spf_module_sources(
   KCONFIG     CONFIG_DRC
   NAME        drc
   MAJOR_VER   1
   MINOR_VER   0
   AMDB_ITYPE  "capi"
   AMDB_MTYPE  "PP"
   AMDB_MID "0x07001066"
   AMDB_TAG "capi_drc"
   AMDB_MOD_NAME  "MODULE_ID_DRC"
   AMDB_FMT_ID1   "MEDIA_FMT_ID_PCM"
   SRCS     ${drc_sources}
   INCLUDES ${drc_includes}
   H2XML_HEADERS  "../api/api_drc.h"
   CFLAGS      ""
)
