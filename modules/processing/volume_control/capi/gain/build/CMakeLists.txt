#[[
   @file CMakeLists.txt

   @brief

   @copyright
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear
]]
cmake_minimum_required(VERSION 3.10)

set(gain_sources
    ${LIB_ROOT}/src/capi_gain.c
    ${LIB_ROOT}/src/capi_gain_utils.c
	../../../lib/src/apply_gain.c
)

set(gain_includes
    ${LIB_ROOT}/api
    ${LIB_ROOT}/inc
    ${LIB_ROOT}/src
    ../../../lib/inc
    ../../../../../cmn/common/utils/inc
)
spf_module_sources(
   KCONFIG     CONFIG_GAIN
   NAME        gain
   MAJOR_VER   1
   MINOR_VER   0
   AMDB_ITYPE  "capi"
   AMDB_MTYPE  "PP"
   AMDB_MID "0x07001002"
   AMDB_TAG "capi_gain"
   AMDB_MOD_NAME  "MODULE_ID_GAIN"
   SRCS     ${gain_sources}
   INCLUDES ${gain_includes}
   H2XML_HEADERS  "${LIB_ROOT}/api/gain_api.h"
   CFLAGS      ""
)

