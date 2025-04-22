# Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

set(gain_sources
	capi/src/capi_gain_module.c
	capi/src/capi_gain_module_utils.c
	lib/src/gain_module_lib.c
)

set(gain_includes
	capi/inc
	lib/inc
	api
	capi/src
	lib/src
)

spf_module_sources(
	KCONFIG		CONFIG_GAIN
	NAME		gain
	MAJOR_VER	1
	MINOR_VER	0
	AMDB_ITYPE	"capi"
	AMDB_MTYPE	"generic"
	AMDB_MID	"0x0700106F"
	AMDB_TAG	"capi_gain_module"
	AMDB_MOD_NAME	"MODULE_ID_GAIN_MODULE"
	AMDB_FMT_ID1	"MEDIA_FMT_ID_EXAMPLE"
	SRCS		${gain_sources}
	INCLUDES	${gain_includes}
	H2XML_HEADERS	api/gain_module_api.h
	CFLAGS		""
)
