#ifndef PCM_DECODER_API_H_
#define PCM_DECODER_API_H_
/*==============================================================================
 @file pcm_decoder_api.h
 @brief This file contains PCM decoder APIs

================================================================================
Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
SPDX-License-Identifier: BSD-3-Clause
==============================================================================*/

/*------------------------------------------------------------------------------
   Include files
------------------------------------------------------------------------------*/
#include "chmixer_common_api.h"
#include "module_cmn_api.h"
#include "common_enc_dec_api.h"

/*# @h2xml_title1          {PCM Decoder Module API}
    @h2xml_title_agile_rev {PCM Decoder Module API}
    @h2xml_title_date      {March 26, 2019} */

/*------------------------------------------------------------------------------
   Defines
------------------------------------------------------------------------------*/

/** @ingroup ar_spf_mod_decoder_mod
    Enumerates the input port ID for the MFC (#MODULE_ID_MFC) module. */
#define PCM_DEC_DATA_INPUT_PORT   0x2

/** @ingroup ar_spf_mod_decoder_mod
    Enumerates the output port ID for the MFC module. */
#define PCM_DEC_DATA_OUTPUT_PORT  0x1

/*------------------------------------------------------------------------------
   Module
------------------------------------------------------------------------------*/

/** @ingroup ar_spf_mod_decoder_mod
    Identifier for the module used as the decoder for PCM use cases.

    This module can be used to convert the properties of PCM stream:
    endianness, interleaving, bit width, number of channels, and so on. It
    cannot be used to convert sampling rate. The module has only one input and
    one output port.

    The input media format for this module is propagated from the Write Shared
    Memory Endpoint module and cannot be configured.

    @subhead4{Supported media format ID}

    The module decodes data to the following format ID:
    - Data format       : #DATA_FORMAT_FIXED_POINT @lstsp1
    - fmt_id            : #MEDIA_FMT_ID_PCM @lstsp1
    - Sample rates      : 1..384 kHz @lstsp1
    - Number of channels: 1..32 @lstsp1
    - Bit width: @lstsep
       - 16 (bits per sample 16 and Q15) @lstsp2
       - 24 (bits per sample 24 and Q23, bits per sample 32 and Q23, Q27, or
             Q31) @lstsp2
       - 32 (bits per sample 32 and Q31) @lstsp1
    - Interleaving: @lstsep
       - Interleaved @lstsp2
       - De-interleaved unpacked @lstsp2
       - De-interleaved packed @lstsp1
    - Endianness: little, big
*/
#define MODULE_ID_PCM_DEC              0x07001005

/*# @h2xmlm_module             {"MODULE_ID_PCM_DEC", MODULE_ID_PCM_DEC}
    @h2xmlm_displayName        {"PCM Decoder"}
    @h2xmlm_description        {ID for the PCM Decoder module. It can be used
                                to convert the properties of PCM stream:
                                endianness, interleaving, bit width, number of
                                channels, and so on. It cannot be used to
                                convert sampling rate. For more details,
                                see AudioReach Signal Processing Framework (SPF) API Reference.}
    @h2xmlm_dataMaxInputPorts  {1}
    @h2xmlm_dataMaxOutputPorts {1}
    @h2xmlm_dataInputPorts     {IN=PCM_DEC_DATA_INPUT_PORT}
    @h2xmlm_dataOutputPorts    {OUT=PCM_DEC_DATA_OUTPUT_PORT}
    @h2xmlm_supportedContTypes {APM_CONTAINER_TYPE_GC}
    @h2xmlm_isOffloadable      {true}
    @h2xmlm_stackSize          {4096}

    @{                      <-- Start of the Module -->
    @h2xml_Select           {"param_id_pcm_output_format_cfg_t"}
    @h2xmlm_InsertParameter
    @h2xml_Select           {"payload_pcm_output_format_cfg_t"}
    @h2xmlm_InsertParameter
    @h2xml_Select           {param_id_pcm_output_format_cfg_t::data_format}
    @h2xmle_rangeEnum       {pcm_data_format}
    @h2xml_Select           {param_id_chmixer_coeff_t}
    @h2xmlm_InsertParameter
    @h2xml_Select           {chmixer_coeff_t}
    @h2xmlm_InsertStructure
    @}                      <-- End of the Module --> */


#endif // PCM_DECODER_API_H_
