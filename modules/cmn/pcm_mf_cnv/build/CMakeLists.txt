#[[
   @file CMakeLists.txt

   @brief

   @copyright
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear-Clear
]]
cmake_minimum_required(VERSION 3.10)

set(pcm_cnv_sources
   ${LIB_ROOT}/capi/pcm_cnv/src/capi_pcm_mf_cnv.cpp
   ${LIB_ROOT}/capi/pcm_cnv/src/capi_pcm_mf_cnv_island.cpp
   ${LIB_ROOT}/capi/pcm_cnv/src/capi_pcm_mf_cnv_utils.cpp
   ${LIB_ROOT}/capi/pcm_cnv/src/capi_pcm_mf_cnv_utils_island.cpp
   ${LIB_ROOT}/lib/src/pc_converter.cpp
   ${LIB_ROOT}/lib/src/pc_converter_island.cpp
   ${LIB_ROOT}/lib/src/pc_init.cpp
   ${LIB_ROOT}/lib/src/pc_process.cpp
   ${LIB_ROOT}/lib/src/pc_process_island.cpp
   ${LIB_ROOT}/lib/src/pc_float/pc_float.cpp
   )

include_directories(
     ${LIB_ROOT}/capi/pcm_cnv/api
     ${LIB_ROOT}/capi/pcm_cnv/inc
     ${LIB_ROOT}/capi/pcm_cnv/src
     ${LIB_ROOT}/capi/mfc/api
     ${LIB_ROOT}/capi/mfc/inc
     ${LIB_ROOT}/lib/inc
     ../../../audio/pcm_encoder/api
     ../../../audio/pcm_encoder/inc
     ../../../audio/pcm_decoder/api
     ../../../audio/pcm_decoder/inc
     ../../../processing/channel_mixer/lib/inc
     ../../../processing/channel_mixer/api/
     ../../../processing/resamplers/dynamic_resampler/inc
     ../../../processing/resamplers/iir_resampler/inc
     ../../../../fwk/spf/utils/cmn/inc
     ../../../../fwk/spf/utils/interleaver/inc
     ../../../../fwk/spf/utils/lpi_pool/inc
     ../../../../fwk/spf/interfaces/fwk/api
     ../../../../fwk/spf/containers/cmn/graph_utils/inc
)

add_library(pcm_cnv ${pcm_cnv_sources})

#Link dynamic_resampler and iir_resampler binaries to pcm_cnv library
add_library(dynamic_resampler STATIC IMPORTED GLOBAL)
set(lib_abs_path ${CMAKE_CURRENT_SOURCE_DIR}/../../../processing/resamplers/dynamic_resampler/bin/arm/libdynamic_resampler.a)
set_target_properties(dynamic_resampler PROPERTIES IMPORTED_LOCATION ${lib_abs_path})
target_link_libraries(pcm_cnv INTERFACE dynamic_resampler)

add_library(iir_resampler STATIC IMPORTED GLOBAL)
set(lib_abs_path ${CMAKE_CURRENT_SOURCE_DIR}/../../../processing/resamplers/iir_resampler/bin/arm/libiir_resampler.a)
set_target_properties(iir_resampler PROPERTIES IMPORTED_LOCATION ${lib_abs_path})
target_link_libraries(pcm_cnv INTERFACE iir_resampler)

set_property(GLOBAL APPEND PROPERTY GLOBAL_SPF_LIBS_LIST pcm_cnv)

set(pcm_cnv_includes
     ${LIB_ROOT}/capi/pcm_cnv/api
     ${LIB_ROOT}/capi/pcm_cnv/inc
     ${LIB_ROOT}/capi/pcm_cnv/src
     ${LIB_ROOT}/capi/mfc/api
     ${LIB_ROOT}/capi/mfc/inc
     ${LIB_ROOT}/lib/inc
     ../../../audio/pcm_encoder/api
     ../../../audio/pcm_encoder/inc
     ../../../audio/pcm_decoder/api
     ../../../audio/pcm_decoder/inc
     ../../../processing/channel_mixer/lib/inc
     ../../../processing/channel_mixer/api/
     ../../../processing/resamplers/dynamic_resampler/inc
     ../../../processing/resamplers/iir_resampler/inc
     ../../../../fwk/spf/utils/cmn/inc
     ../../../../fwk/spf/utils/interleaver/inc
     ../../../../fwk/spf/utils/lpi_pool/inc
     ../../../../fwk/spf/interfaces/fwk/api
     ../../../../fwk/spf/containers/cmn/graph_utils/inc
     )

spf_module_sources(
   KCONFIG     CONFIG_PCM_CNV
   NAME        pcm_cnv
   MAJOR_VER   1
   MINOR_VER   0
   AMDB_ITYPE  "capi"
   AMDB_MTYPE  "PP"
   AMDB_MID "0x07001003"
   AMDB_TAG "capi_pcm_cnv"
   AMDB_MOD_NAME  "MODULE_ID_PCM_CNV"
   SRCS     ${pcm_cnv_sources}
   INCLUDES ${pcm_cnv_includes}
   H2XML_HEADERS  "${LIB_ROOT}/capi/pcm_cnv/api/pcm_converter_api.h"
   CFLAGS      ""
)

spf_module_sources(
     KCONFIG        CONFIG_PCM_DECODER
     NAME      pcm_decoder
     MAJOR_VER 1
     MINOR_VER 0
     AMDB_ITYPE     "capi"
     AMDB_MTYPE     "decoder"
     AMDB_MID  "0x07001005"
     AMDB_TAG  "capi_pcm_dec"
     AMDB_MOD_NAME  "MODULE_ID_PCM_DEC"
     AMDB_FMT_ID1   "MEDIA_FMT_ID_PCM"
     SRCS      ${pcm_cnv_sources}
     INCLUDES  ${pcm_cnv_includes}
     H2XML_HEADERS  "../../../audio/pcm_decoder/api/pcm_decoder_api.h"
     CFLAGS         ""
)

spf_module_sources(
     KCONFIG        CONFIG_PCM_ENCODER
     NAME      pcm_encoder
     MAJOR_VER 1
     MINOR_VER 0
     AMDB_ITYPE     "capi"
     AMDB_MTYPE     "encoder"
     AMDB_MID  "0x07001004"
     AMDB_TAG  "capi_pcm_enc"
     AMDB_MOD_NAME  "MODULE_ID_PCM_ENC"
     AMDB_FMT_ID1   "MEDIA_FMT_ID_PCM"
     SRCS      ${pcm_cnv_sources}
     INCLUDES  ${pcm_cnv_includes}
     H2XML_HEADERS  "../../../audio/pcm_encoder/api/pcm_encoder_api.h"
     CFLAGS         ""
)

spf_module_sources(
   KCONFIG     CONFIG_MFC
   NAME        mfc
   MAJOR_VER   1
   MINOR_VER   0
   AMDB_ITYPE  "capi"
   AMDB_MTYPE  "PP"
   AMDB_MID "0x07001015"
   AMDB_TAG "capi_mfc"
   AMDB_MOD_NAME  "MODULE_ID_MFC"
   SRCS        ${pcm_cnv_sources}
   INCLUDES    ${pcm_cnv_includes}
   H2XML_HEADERS  "${LIB_ROOT}/capi/mfc/api/mfc_api.h"
   CFLAGS      ""
)
