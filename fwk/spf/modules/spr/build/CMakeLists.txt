#[[
   @file CMakeLists.txt

   @brief

   @copyright
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear

]]
cmake_minimum_required(VERSION 3.10)

set(spr_sources
	${LIB_ROOT}capi/src/capi_spr_avsync_utils.c
	${LIB_ROOT}capi/src/capi_spr_avsync_utils_island.c
	${LIB_ROOT}capi/src/capi_spr_data_utils_island.c
	${LIB_ROOT}capi/src/capi_spr_imc_utils.c
	${LIB_ROOT}capi/src/capi_spr_imc_utils_island.c
	${LIB_ROOT}capi/src/capi_spr_int_buf_utils.c
	${LIB_ROOT}capi/src/capi_spr_int_buf_utils_island.c
	${LIB_ROOT}capi/src/capi_spr_path_delay.c
	${LIB_ROOT}capi/src/capi_spr_path_delay_island.c
	${LIB_ROOT}capi/src/capi_spr_utils.c
	${LIB_ROOT}capi/src/capi_spr_utils_island.c
	${LIB_ROOT}capi/src/capi_spr.c
	${LIB_ROOT}lib/src/cicular_buffer_island.c
	${LIB_ROOT}lib/src/spr_lib_island.c
)

set(spr_includes
	${LIB_ROOT}capi/inc
	${LIB_ROOT}capi/src
	${LIB_ROOT}lib/inc
	${LIB_ROOT}api
)

spf_module_sources(
	KCONFIG		CONFIG_SPR
	NAME		spr
	MAJOR_VER	1
	MINOR_VER	0
	AMDB_ITYPE	"capi"
	AMDB_MTYPE	"generic"
	AMDB_MID	"0x07001032"
	AMDB_TAG	"capi_spr"
	AMDB_MOD_NAME	"MODULE_ID_SPR"
	SRCS		${spr_sources}
	INCLUDES	${spr_includes}
	H2XML_HEADERS	"${LIB_ROOT}api/spr_api.h"
	CFLAGS		""
)
