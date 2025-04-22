#[[
   @file CMakeLists.txt

   @brief

   @copyright
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear

]]
cmake_minimum_required(VERSION 3.10)

set(sync_sources
	${LIB_ROOT}capi/src/capi_sync.c
	${LIB_ROOT}capi/src/capi_sync_control_utils.c
	${LIB_ROOT}capi/src/capi_sync_data_utils.c
	${LIB_ROOT}capi/src/capi_sync_event_utils.c
	${LIB_ROOT}capi/src/capi_sync_port_utils.c
	${LIB_ROOT}capi/src/capi_sync_process_ec_mode.c
	${LIB_ROOT}capi/src/capi_sync_process_generic.c
	${LIB_ROOT}capi/src/capi_sync_utils.c
)

set(sync_includes
	${LIB_ROOT}capi/inc
	${LIB_ROOT}api
)

spf_module_sources(
	KCONFIG		CONFIG_SYNC
	NAME		sync
	MAJOR_VER	1
	MINOR_VER	0
	AMDB_ITYPE	"capi"
	AMDB_MTYPE	"generic"
	AMDB_MID	"0x07001038"
	AMDB_TAG	"capi_sync"
	AMDB_MOD_NAME	"MODULE_ID_SYNC"
	SRCS		${sync_sources}
	INCLUDES	${sync_includes}
	H2XML_HEADERS	"${LIB_ROOT}api/sync_api.h"
	CFLAGS		""
)
