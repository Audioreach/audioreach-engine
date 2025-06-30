#[[
   @file CMakeLists.txt

   @brief

   @copyright
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear
]]
cmake_minimum_required(VERSION 3.10)

set(virtualizer_includes
    ${LIB_ROOT}/api
    ${LIB_ROOT}/capi/inc
    ${LIB_ROOT}/capi/src
    ${LIB_ROOT}/lib/inc
    ${LIB_ROOT}/lib/src
    ${PROJECT_SOURCE_DIR}/modules/processing/gain_control/limiter/lib/inc
    ${PROJECT_SOURCE_DIR}/modules/processing/filters/multi_stage_iir/lib/inc
    ${PROJECT_SOURCE_DIR}/modules/cmn/common/utils/inc
)

spf_module_sources(
   KCONFIG     CONFIG_VIRTUALIZER
   NAME        virtualizer
   MAJOR_VER   1
   MINOR_VER   0
   AMDB_ITYPE  "capi"
   AMDB_MTYPE  "pp"
   AMDB_MID    "0x07001064"
   AMDB_TAG    "capi_virtualizer"
   AMDB_MOD_NAME  "MODULE_ID_VIRTUALIZER"
   INCLUDES    ${virtualizer_includes}
   H2XML_HEADERS  "${LIB_ROOT}/api/Virtualizer_api.h"
   CFLAGS      ""
   STATIC_LIB_PATH "${LIB_ROOT}/bin/arm/libvirtualizer.a"
)
