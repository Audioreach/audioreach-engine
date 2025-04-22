# Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

set(echo_cancellation_sources
	capi/src/capi_ecns_metadata_utils.c
	capi/src/capi_ecns_utils.c
	capi/src/capi_ecns_data_port_utils.c
	capi/src/capi_ecns.c
)
set(echo_cancellation_includes
	capi/src
	capi/inc
	api
)

spf_module_sources(
	KCONFIG		CONFIG_ECHO_CANCELLATION
	NAME		echo_cancellation
	MAJOR_VER	1
	MINOR_VER	0
	AMDB_ITYPE	"capi"
	AMDB_MTYPE	"encoder"
	AMDB_MID	"0x0700109A"
	AMDB_TAG	"capi_ecns"
	AMDB_MOD_NAME	"MODULE_ID_ECNS"
	AMDB_FMT_ID1	"MEDIA_FMT_ID_EXAMPLE"
	SRCS		${echo_cancellation_sources}
	INCLUDES	${echo_cancellation_includes}
	H2XML_HEADERS	"api/ecns_calibration_api.h"
	CFLAGS		""
)
