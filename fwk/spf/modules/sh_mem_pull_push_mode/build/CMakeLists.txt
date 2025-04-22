#[[
   @file CMakeLists.txt

   @brief

   @copyright
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear

]]
cmake_minimum_required(VERSION 3.10)

set(sh_mem_pull_push_mode_sources
	${LIB_ROOT}src/capi_sh_mem_pull_push_mode.c
	${LIB_ROOT}src/pull_push_mode.c
)

set(sh_mem_pull_push_mode_includes
	${LIB_ROOT}inc
	${LIB_ROOT}src
	${LIB_ROOT}api
)

spf_module_sources(
	KCONFIG		CONFIG_SH_MEM_PULL_MODE
	NAME		sh_mem_pull_mode
	MAJOR_VER	1
	MINOR_VER	0
	AMDB_ITYPE	"capi"
	AMDB_MTYPE	"generic"
	AMDB_MID	"0x07001006"
	AMDB_TAG	"capi_pull_mode"
	AMDB_MOD_NAME	"MODULE_ID_SH_MEM_PULL_MODE"
	SRCS		${sh_mem_pull_push_mode_sources}
	INCLUDES	${sh_mem_pull_push_mode_includes}
	H2XML_HEADERS	"${LIB_ROOT}api/sh_mem_pull_push_mode_api.h"
	CFLAGS		""
)

# Added to generate json entry for sh_mem_push_mode.
spf_module_sources(
	KCONFIG		CONFIG_SH_MEM_PUSH_MODE
	NAME		sh_mem_push_mode
	MAJOR_VER	1
	MINOR_VER	0
	AMDB_ITYPE	"capi"
	AMDB_MTYPE	"generic"
	AMDB_MID	"0x07001007"
	AMDB_TAG	"capi_push_mode"
	AMDB_MOD_NAME	"MODULE_ID_SH_MEM_PUSH_MODE"
	SRCS		""
	INCLUDES	""
	H2XML_HEADERS	"${LIB_ROOT}api/sh_mem_pull_push_mode_api.h"
	CFLAGS		""
)
