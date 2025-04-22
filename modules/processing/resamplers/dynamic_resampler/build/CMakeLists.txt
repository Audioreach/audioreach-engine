#[[
   @file CMakeLists.txt

   @brief

   @copyright
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear
]]
cmake_minimum_required(VERSION 3.10)

#Include directories
set(dynamic_resampler_includes
	${LIB_ROOT}/inc
)

spf_module_sources(
   KCONFIG     CONFIG_DYNAMIC_RESAMPLER
   NAME        dynamic_resampler
   MAJOR_VER   1
   MINOR_VER   0
   AMDB_ITYPE  "capi"
   AMDB_MTYPE  "pp"
   AMDB_MID    "0x07001016"
   AMDB_TAG    "capi_dynamic_resampler"
   AMDB_MOD_NAME  "MODULE_ID_DYNAMIC_RESAMPLER"
   INCLUDES    ${dynamic_resampler_includes}
   H2XML_HEADERS  "${LIB_ROOT}/api/dynamic_resampler_api.h"
   CFLAGS      ""
   STATIC_LIB_PATH "${LIB_ROOT}/bin/arm/libdynamic_resampler.a"
)


