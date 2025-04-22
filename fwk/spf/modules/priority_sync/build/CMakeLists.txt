#[[
   @file CMakeLists.txt

   @brief

   @copyright
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear

]]
cmake_minimum_required(VERSION 3.10)

set(priority_sync_sources
	${LIB_ROOT}capi/src/capi_priority_sync.c
	${LIB_ROOT}capi/src/capi_priority_sync_control_utils.c
	${LIB_ROOT}capi/src/capi_priority_sync_data_utils.c
	${LIB_ROOT}capi/src/capi_priority_sync_event_utils.c
	${LIB_ROOT}capi/src/capi_priority_sync_ts.c
)

set(priority_sync_includes
	${LIB_ROOT}capi/inc
	${LIB_ROOT}capi/src
	${LIB_ROOT}api
)

spf_module_sources(
	KCONFIG		CONFIG_PRIORITY_SYNC
	NAME		priority_sync
	MAJOR_VER	1
	MINOR_VER	0
	AMDB_ITYPE	"capi"
	AMDB_MTYPE	"generic"
	AMDB_MID	"0x07001028"
	AMDB_TAG	"capi_priority_sync"
	AMDB_MOD_NAME	"MODULE_ID_PRIORITY_SYNC"
	SRCS		${priority_sync_sources}
	INCLUDES	${priority_sync_includes}
	H2XML_HEADERS	"${LIB_ROOT}api/priority_sync_api.h"
	CFLAGS		""
)
