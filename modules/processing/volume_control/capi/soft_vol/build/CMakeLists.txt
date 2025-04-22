#[[
   @file CMakeLists.txt

   @brief

   @copyright
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear
]]
cmake_minimum_required(VERSION 3.10)

set(soft_vol_sources
    ${LIB_ROOT}/src/capi_soft_vol.cpp
    ${LIB_ROOT}/src/capi_soft_vol_island.cpp
    ${LIB_ROOT}/src/capi_soft_vol_utils.cpp
    ${LIB_ROOT}/src/capi_soft_vol_utils_v2.cpp
	../../../lib/src/softvolumecontrols.cpp
	../../../lib/src/softvolumecontrols_island.cpp
)

set(soft_vol_includes
    ${LIB_ROOT}/api
    ${LIB_ROOT}/inc
    ${LIB_ROOT}/src
    ../../../lib/inc
	../../../../../cmn/common/internal_api
    ../../../../../cmn/common/utils/inc
)

spf_module_sources(
   KCONFIG     CONFIG_SOFT_VOL
   NAME        soft_vol
   MAJOR_VER   1
   MINOR_VER   0
   AMDB_ITYPE  "capi"
   AMDB_MTYPE  "PP"
   AMDB_MID "0x0700101B"
   AMDB_TAG "capi_soft_vol"
   AMDB_MOD_NAME  "MODULE_ID_VOL_CTRL"
   SRCS     ${soft_vol_sources}
   INCLUDES ${soft_vol_includes}
   H2XML_HEADERS  "${LIB_ROOT}/api/soft_vol_api.h"
   CFLAGS      ""
)
