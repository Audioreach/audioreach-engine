#[[
   @file CMakeLists.txt

   @brief

   @copyright
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear
]]
cmake_minimum_required(VERSION 3.10)

set(shoebox_reverb_includes
    ${LIB_ROOT}/api
    ${LIB_ROOT}/inc
    ${PROJECT_SOURCE_DIR}/modules/cmn/common/utils/inc
)

spf_module_sources(
   KCONFIG     CONFIG_SHOEBOX
   NAME        shoebox
   MAJOR_VER   1
   MINOR_VER   0
   AMDB_ITYPE  "capi"
   AMDB_MTYPE  "pp"
   AMDB_MID    "0x07001058"
   AMDB_TAG    "capi_shoebox"
   AMDB_MOD_NAME  "MODULE_ID_SHOEBOX"
   INCLUDES    ${shoebox_reverb_includes}
   H2XML_HEADERS  "${LIB_ROOT}/api/shoebox_api.h"
   CFLAGS      ""
   STATIC_LIB_PATH "${LIB_ROOT}/bin/arm/libshoebox_reverb.a"
)

spf_module_sources(
   KCONFIG     CONFIG_REVERB
   NAME        reverb
   MAJOR_VER   1
   MINOR_VER   0
   AMDB_ITYPE  "capi"
   AMDB_MTYPE  "pp"
   AMDB_MID    "0x07001059"
   AMDB_TAG    "capi_shoebox"
   AMDB_MOD_NAME  "MODULE_ID_REVERB"
   INCLUDES    ${shoebox_reverb_includes}
   H2XML_HEADERS  "${LIB_ROOT}/api/api_reverb.h"
   CFLAGS      ""
   STATIC_LIB_PATH "${LIB_ROOT}/bin/arm/libshoebox_reverb.a"
)

