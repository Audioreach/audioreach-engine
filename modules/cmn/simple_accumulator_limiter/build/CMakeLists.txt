#[[
   @file CMakeLists.txt

   @brief

   @copyright
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear
]]
cmake_minimum_required(VERSION 3.10)

set(sal_sources
    ${LIB_ROOT}/capi/src/capi_sal.cpp
    ${LIB_ROOT}/capi/src/capi_sal_island.cpp
    ${LIB_ROOT}/capi/src/capi_sal_md_utils_island.cpp
    ${LIB_ROOT}/capi/src/capi_sal_port_utils.cpp
    ${LIB_ROOT}/capi/src/capi_sal_utils.cpp
)

set(sal_includes
   ${LIB_ROOT}/api
   ${LIB_ROOT}/capi/inc
   ${LIB_ROOT}/capi/src
   ${PROJECT_SOURCE_DIR}/modules/processing/gain_control/limiter/api
   ${PROJECT_SOURCE_DIR}/modules/processing/gain_control/limiter/lib/inc
)

spf_module_sources(
   KCONFIG     CONFIG_SAL
   NAME        sal
   MAJOR_VER   1
   MINOR_VER   0
   AMDB_ITYPE  "capi"
   AMDB_MTYPE  "PP"
   AMDB_MID "0x07001010"
   AMDB_TAG "capi_sal"
   AMDB_MOD_NAME  "MODULE_ID_SAL"
   SRCS     ${sal_sources}
   INCLUDES ${sal_includes}
   H2XML_HEADERS  "${LIB_ROOT}/api/sal_api.h"
   CFLAGS      ""
)
