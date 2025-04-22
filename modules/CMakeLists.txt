# Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

#Include directories
include_directories(
                    cmn/common/utils/inc/
                    cmn/api
                    cmn/common/internal_api/imcl/api
                    ${LIB_ROOT}/fwk/spf/utils/list/inc
                    ${LIB_ROOT}/fwk/spf/interfaces/module/capi_cmn/ctrl_port/inc
                    ${LIB_ROOT}/fwk/spf/containers/cmn/topologies/topo_utils/inc
                    ${LIB_ROOT}/fwk/spf/utils/lpi_pool/inc
                    ${LIB_ROOT}/fwk/spf/containers/cmn/graph_utils/inc
                   )

#Add the sub directories
add_subdirectory(cmn/build cmn)

if(CONFIG_ENCODER)
	add_subdirectory(examples/encoder)
endif()
if(CONFIG_ECHO_CANCELLATION)
	add_subdirectory(examples/echo_cancellation)
endif()
if(CONFIG_EXAMPLE_GAIN)
	add_subdirectory(examples/gain)
endif()
if(CONFIG_CHMIXER)
    add_subdirectory(processing/channel_mixer/build)
endif()
if(CONFIG_MSIIR)
    add_subdirectory(processing/filters/multi_stage_iir/build)
endif()
if(CONFIG_PCM_CNV)
    add_subdirectory(cmn/pcm_mf_cnv/build)
endif()
if(CONFIG_DRC)
    add_subdirectory(processing/gain_control/drc/build)
endif()
if(CONFIG_IIR_MBDRC)
    add_subdirectory(processing/gain_control/limiter/build)
    add_subdirectory(processing/gain_control/iir_mbdrc/build)
endif()
if(CONFIG_GAIN)
    add_subdirectory(processing/volume_control/capi/gain/build)
endif()
if(CONFIG_SOFT_VOL)
    add_subdirectory(processing/volume_control/capi/soft_vol/build)
endif()
