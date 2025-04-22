#[[
   @file CMakeLists.txt

   @brief

   @copyright
   Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear
]]
cmake_minimum_required(VERSION 3.10)

set(iir_resampler_includes
   ${LIB_ROOT}/inc
)

spf_module_sources(
   KCONFIG     CONFIG_IIR_RESAMPLER
   NAME        iir_resampler
   MAJOR_VER   1
   MINOR_VER   0
   AMDB_ITYPE  "capi"
   AMDB_MTYPE  "pp"
   AMDB_MID    "0x07001018"
   AMDB_TAG    "capi_iir_resampler"
   AMDB_MOD_NAME  "MODULE_ID_IIR_RESAMPLER"
   INCLUDES    ${iir_resampler_includes}
   H2XML_HEADERS  "${LIB_ROOT}/api/iir_resampler_api.h"
   CFLAGS      ""
   STATIC_LIB_PATH "${LIB_ROOT}/bin/arm/libiir_resampler.a"
)