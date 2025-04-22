#[[
   @file CMakeLists.txt

   @brief

   @copyright
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear
]]

cmake_minimum_required(VERSION 3.10)

set(latency_sources
	${LIB_ROOT}capi/src/capi_latency_utils.c
	${LIB_ROOT}capi/src/capi_latency.c
	${LIB_ROOT}capi/src/capi_latency_utils_v2.c
	${LIB_ROOT}lib/src/latency_buf_utils.c
)


set(latency_includes
	${LIB_ROOT}capi/inc
	${LIB_ROOT}capi/src
	${LIB_ROOT}lib/inc
	${LIB_ROOT}api
)

spf_module_sources(
	KCONFIG		CONFIG_LATENCY
	NAME		latency
	MAJOR_VER	1
	MINOR_VER	0
	AMDB_ITYPE	"capi"
	AMDB_MTYPE	"generic"
	AMDB_MID	"0x0700101C"
	AMDB_TAG	"capi_latency"
	AMDB_MOD_NAME	"MODULE_ID_LATENCY"
	SRCS		${latency_sources}
	INCLUDES	${latency_includes}
	H2XML_HEADERS	"${LIB_ROOT}api/latency_api.h"
	CFLAGS		""
)
