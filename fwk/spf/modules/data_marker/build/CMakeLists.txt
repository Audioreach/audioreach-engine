#[[
   @file CMakeLists.txt

   @brief

   @copyright
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear

]]
cmake_minimum_required(VERSION 3.10)

set(data_marker_sources
	${LIB_ROOT}capi/src/capi_data_marker_md_utils.c
	${LIB_ROOT}capi/src/capi_data_marker.c
)

set(data_marker_includes
	${LIB_ROOT}capi/inc
	${LIB_ROOT}capi/src
	${LIB_ROOT}api
)

spf_module_sources(
	KCONFIG		CONFIG_DATA_MARKER
	NAME		data_marker
	MAJOR_VER	1
	MINOR_VER	0
	AMDB_ITYPE	"capi"
	AMDB_MTYPE	"generic"
	AMDB_MID	"0x0700106A"
	AMDB_TAG	"capi_data_marker"
	AMDB_MOD_NAME	"MODULE_ID_DATA_MARKER"
	SRCS		${data_marker_sources}
	INCLUDES	${data_marker_includes}
	H2XML_HEADERS	"${LIB_ROOT}api/data_marker_api.h"
	CFLAGS		""
)
