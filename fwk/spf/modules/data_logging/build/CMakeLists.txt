#[[
   @file CMakeLists.txt

   @brief

   @copyright
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear

]]
cmake_minimum_required(VERSION 3.10)

set(data_logging_sources
	${LIB_ROOT}capi/src/capi_data_logging.c
	${LIB_ROOT}capi/src/capi_data_logging_island.c
	${LIB_ROOT}capi/src/capi_data_logging_config_utils.c
)

set(data_logging_includes
	${LIB_ROOT}capi/inc
	${LIB_ROOT}capi/src
	${LIB_ROOT}api
)

spf_module_sources(
	KCONFIG		CONFIG_DATA_LOGGING
	NAME		data_logging
	MAJOR_VER	1
	MINOR_VER	0
	AMDB_ITYPE	"capi"
	AMDB_MTYPE	"generic"
	AMDB_MID	"0x0700101a"
	AMDB_TAG	"capi_data_logging"
	AMDB_MOD_NAME	"MODULE_ID_DATA_LOGGING"
	SRCS		${data_logging_sources}
	INCLUDES	${data_logging_includes}
	H2XML_HEADERS	"${LIB_ROOT}api/data_logging_api.h"
	CFLAGS		"-Wno-address-of-packed-member"
)
