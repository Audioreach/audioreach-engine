#ifndef PCM_CONVERTER_API_H_
#define PCM_CONVERTER_API_H_
/*==============================================================================
 @file pcm_converter_api.h
 @brief This file contains PCM converter APIs

================================================================================
 Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 SPDX-License-Identifier: BSD-3-Clause-Clear
==============================================================================*/

/*------------------------------------------------------------------------
  Include files
  -----------------------------------------------------------------------*/
#include "chmixer_common_api.h"
#include "module_cmn_api.h"
#include "common_enc_dec_api.h"

/*# @h2xml_title1          {PCM Converter Module API}
    @h2xml_title_agile_rev {PCM Converter Module API}
    @h2xml_title_date      {March 26, 2019} */

/*------------------------------------------------------------------------------
   Defines
------------------------------------------------------------------------------*/

/** @ingroup ar_spf_mod_pcm_conv_mod
    Enumerates the input port ID for the MFC module (#MODULE_ID_MFC). */
#define PCM_CNV_DATA_INPUT_PORT   0x2

/** @ingroup ar_spf_mod_pcm_conv_mod
    Enumerates the output port ID for the MFC module. */
#define PCM_CNV_DATA_OUTPUT_PORT  0x1

/*------------------------------------------------------------------------------
   Module
------------------------------------------------------------------------------*/

/** @ingroup ar_spf_mod_pcm_conv_mod
    Identifier for the PCM Converter module, which is used to convert the
    properties of a PCM stream: endianness, interleaving, bit width,
    number of channels, and so on. It cannot be used to convert the sampling
    rate. This module has only one input port and one output port.

    @subhead4{Supported parameter IDs}
    - #PARAM_ID_PCM_OUTPUT_FORMAT_CFG @lstsp1
    - #PARAM_ID_REMOVE_INITIAL_SILENCE @lstsp1
    - #PARAM_ID_REMOVE_TRAILING_SILENCE

    @subhead4{Supported input media format ID}
    - Data format       : #DATA_FORMAT_FIXED_POINT and #DATA_FORMAT_FLOATING_POINT @lstsp1
    - fmt_id            : Don't care @lstsp1
    - Sample rates      : 1..384 kHz @lstsp1
    - Number of channels: 1 to 128 (for certain products this module supports only 32 channels) @lstsp1
    - Bit width: @lstsep
       - 16 (bits per sample 16 and Q15) @lstsp2
       - 24 (bits per sample 24 and Q23, bits per sample 32 and Q23, Q27, or
             Q31) @lstsp2
       - 32 (bits per sample 32 and Q31) @lstsp1
       - 64 (only for floating point data)
    - Interleaving: @lstsep
       - Interleaved @lstsp2
       - De-interleaved unpacked @lstsp2
       - De-interleaved packed @lstsp1
    - Endianness: little, big
 */
#define MODULE_ID_PCM_CNV              0x07001003

/*# @h2xmlm_module             {"MODULE_ID_PCM_CNV", MODULE_ID_PCM_CNV}
    @h2xmlm_displayName        {"PCM Converter"}
    @h2xmlm_modSearchKeys	   {channel mixer, byte converter, endianness, Audio}
    @h2xmlm_description        {ID for the module used to convert the
                                properties of a PCM stream: endianness,
                                interleaving, bit width, number of channels,data format converter,
                                and so on. It cannot be used to convert the
                                sampling rate. \n
                                The module has only one input port and one
                                output port. For more details, see AudioReach Signal Processing Framework (SPF) API Reference.}
    @h2xmlm_dataMaxInputPorts  {1}
    @h2xmlm_dataMaxOutputPorts {1}
    @h2xmlm_dataInputPorts     {IN = PCM_CNV_DATA_INPUT_PORT}
    @h2xmlm_dataOutputPorts    {OUT = PCM_CNV_DATA_OUTPUT_PORT}
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


#endif // PCM_CONVERTER_API_H_
