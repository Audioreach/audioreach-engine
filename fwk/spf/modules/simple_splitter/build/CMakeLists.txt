#[[
   @file CMakeLists.txt

   @brief

   @copyright
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear

]]
cmake_minimum_required(VERSION 3.10)

set(splitter_sources
	${LIB_ROOT}capi/src/capi_splitter_island.c
	${LIB_ROOT}capi/src/capi_splitter_utils.c
	${LIB_ROOT}capi/src/capi_splitter.c
	${LIB_ROOT}capi/src/capi_splitter_md_island.c
)

set(splitter_includes
	${LIB_ROOT}capi/inc
	${LIB_ROOT}capi/src
	${LIB_ROOT}api
)

spf_module_sources(
	KCONFIG		CONFIG_SPLITTER
	NAME		splitter
	MAJOR_VER	1
	MINOR_VER	0
	AMDB_ITYPE	"capi"
	AMDB_MTYPE	"generic"
	AMDB_MID	"0x07001011"
	AMDB_TAG	"capi_splitter"
	AMDB_MOD_NAME	"MODULE_ID_SPLITTER"
	SRCS		${splitter_sources}
	INCLUDES	${splitter_includes}
	H2XML_HEADERS	"${LIB_ROOT}api/splitter_api.h"
	CFLAGS		""
)
