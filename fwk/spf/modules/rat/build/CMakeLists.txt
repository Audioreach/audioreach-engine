#[[
   @file CMakeLists.txt

   @brief

   @copyright
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear

]]
cmake_minimum_required(VERSION 3.10)

set(rate_adapted_timer_sources
	${LIB_ROOT}capi/mimo_rat/src/capi_rat_island.c
	${LIB_ROOT}capi/mimo_rat/src/capi_rat_port_utils.c
	${LIB_ROOT}capi/mimo_rat/src/capi_rat_utils.c
	${LIB_ROOT}capi/mimo_rat/src/capi_rat.c
)

set(rate_adapted_timer_includes
	${LIB_ROOT}capi/inc
	${LIB_ROOT}capi/siso_rat/inc
	${LIB_ROOT}capi/mimo_rat/src
	${LIB_ROOT}capi/mimo_rat/inc
	${LIB_ROOT}api
)

spf_module_sources(
	KCONFIG		CONFIG_SISO_RAT
	NAME		rate_adapted_timer
	MAJOR_VER	1
	MINOR_VER	0
	AMDB_ITYPE	"capi"
	AMDB_MTYPE	"generic"
	AMDB_MID	"0x07001041"
	AMDB_TAG	"capi_siso_rat"
	AMDB_MOD_NAME	"MODULE_ID_RATE_ADAPTED_TIMER"
	SRCS		${rate_adapted_timer_sources}
	INCLUDES	${rate_adapted_timer_includes}
	H2XML_HEADERS	"${LIB_ROOT}api/rate_adapted_timer_api.h"
	CFLAGS		""
)
