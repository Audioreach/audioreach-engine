#[[
   @file CMakeLists.txt

   @brief

   @copyright
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause

   // clang-format off
   $Header: $
   // clang-format on
]]
cmake_minimum_required(VERSION 3.10)

#Include directories
set(iir_mbdrc_includes
                    ${LIB_ROOT}/api
                    ${LIB_ROOT}/inc
                    ../../../../cmn/common/utils/inc
                    ../../drc/lib/inc
                    ../../limiter/inc
                   )

spf_module_sources(
   KCONFIG     CONFIG_IIR_MBDRC
   NAME        iir_mbdrc
   MAJOR_VER   1
   MINOR_VER   0
   AMDB_ITYPE  "capi"
   AMDB_MTYPE  "pp"
   AMDB_MID    "0x07001017"
   AMDB_TAG    "capi_iir_mbdrc"
   AMDB_MOD_NAME  "MODULE_ID_IIR_MBDRC"
   INCLUDES    ${iir_mbdrc_includes}
   H2XML_HEADERS  "${LIB_ROOT}/api/mbdrc_api.h"
   CFLAGS      ""
   STATIC_LIB_PATH "${LIB_ROOT}/bin/arm/libiir_mbdrc.a"
)
