#[[
   @file CMakeLists.txt

   @brief

   @copyright
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear
]]
cmake_minimum_required(VERSION 3.10)

set(chmixer_sources
    ${LIB_ROOT}/capi/src/capi_chmixer.cpp
    ${LIB_ROOT}/capi/src/capi_chmixer_utils.cpp
    ${LIB_ROOT}/lib/src/ChannelMixerLib.c
    ${LIB_ROOT}/lib/src/ChannelMixerLib_island.c
    ${LIB_ROOT}/lib/src/ChannelMixerRemapRules.c
)

set(chmixer_includes
    ${LIB_ROOT}/api
    ${LIB_ROOT}/capi/inc
    ${LIB_ROOT}/capi/src
    ${LIB_ROOT}/lib/inc
    ${LIB_ROOT}/lib/src
    ../../../cmn/common/utils/inc
)

spf_module_sources(
   KCONFIG     CONFIG_CHMIXER
   NAME        chmixer
   MAJOR_VER   1
   MINOR_VER   0
   AMDB_ITYPE  "capi"
   AMDB_MTYPE  "PP"
   AMDB_MID "0x07001013"
   AMDB_TAG "capi_chmixer"
   AMDB_MOD_NAME  "MODULE_ID_CHMIXER"
   SRCS     ${chmixer_sources}
   INCLUDES ${chmixer_includes}
   H2XML_HEADERS  "${LIB_ROOT}/api/chmixer_api.h"
   CFLAGS      ""
)
