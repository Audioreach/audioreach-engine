#[[
   @file CMakeLists.txt

   @brief

   @copyright
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear

]]
cmake_minimum_required(VERSION 3.10)

set(mux_demux_sources
	${LIB_ROOT}capi/src/capi_mux_demux_island.c
	${LIB_ROOT}capi/src/capi_mux_demux_utils.c
	${LIB_ROOT}capi/src/capi_mux_demux.c
)

set(mux_demux_includes
	${LIB_ROOT}capi/inc
	${LIB_ROOT}capi/src
	${LIB_ROOT}api
)

spf_module_sources(
	KCONFIG		CONFIG_MUX_DEMUX
	NAME		mux_demux
	MAJOR_VER	1
	MINOR_VER	0
	AMDB_ITYPE	"capi"
	AMDB_MTYPE	"generic"
	AMDB_MID	"0x07001098"
	AMDB_TAG	"capi_mux_demux"
	AMDB_MOD_NAME	"MODULE_ID_MUX_DEMUX"
	SRCS		${mux_demux_sources}
	INCLUDES	${mux_demux_includes}
	H2XML_HEADERS	"${LIB_ROOT}api/mux_demux_api.h"
	CFLAGS		""
)
