#ifndef PCM_ENCODER_API_H_
#define PCM_ENCODER_API_H_
/*==============================================================================
 @file pcm_encoder_api.h
 @brief This file contains PCM encoder APIs

================================================================================
Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
SPDX-License-Identifier: BSD-3-Clause
==============================================================================*/

/*------------------------------------------------------------------------
  Include files
  -----------------------------------------------------------------------*/
#include "chmixer_common_api.h"
#include "module_cmn_api.h"
#include "common_enc_dec_api.h"

/*# @h2xml_title1          {PCM Encoder Module API}
    @h2xml_title_agile_rev {PCM Encoder Module API}
    @h2xml_title_date      {March 26, 2019} */

/*------------------------------------------------------------------------------
   Defines
------------------------------------------------------------------------------*/

/** @ingroup ar_spf_mod_encoder_mod
    Enumerates the input port ID for the MFC module (#MODULE_ID_MFC). */
#define PCM_ENC_DATA_INPUT_PORT   0x2

/** @ingroup ar_spf_mod_encoder_mod
    Enumerates the output port ID for the MFC module. */
#define PCM_ENC_DATA_OUTPUT_PORT  0x1

/*------------------------------------------------------------------------------
   Parameter ID
------------------------------------------------------------------------------*/

/** @ingroup ar_spf_mod_encoder_mod
    Identifier for the parameter used to set the frame size of the PCM encoder
    module.

    The PCM encoder frame size is determined by the performance mode of the
    container. For low latency, the frame size is 1 ms; for low power, the
    frame size is 5 ms.

    This parameter ID can be used if the client is required to specify a
    fixed frame size for the PCM encoder.

    @msgpayload
    param_id_pcm_encoder_frame_size_t
*/
#define PARAM_ID_PCM_ENCODER_FRAME_SIZE                   0x0800136B

/*# @h2xmlp_parameter   {"PARAM_ID_PCM_ENCODER_FRAME_SIZE",
                          PARAM_ID_PCM_ENCODER_FRAME_SIZE}
    @h2xmlp_description {ID for the parameter used to set the frame size of
                         the PCM encoder module. \n
                         The PCM encoder frame size is determined by the
                         performance mode of the container. For low latency,
                         the frame size is 1 ms; for low power, the frame size
                         is 5 ms. \n
                         This parameter ID can be used if the client is
                         required to specify a fixed frame size for the PCM
                         encoder.}
    @h2xmlp_toolPolicy   {Calibration} */

/** @ingroup ar_spf_mod_encoder_mod
    Payload of the #PARAM_ID_PCM_ENCODER_FRAME_SIZE parameter.
*/
#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
struct param_id_pcm_encoder_frame_size_t
{
    uint32_t frame_size_type;
   /**< Specifies whether the frame size of the PCM encoder module is in
        samples or microseconds.

       @valuesbul
        - 0 -- frame_size_not_specified (Default)
        - 1 -- frame_size_in_samples
        - 2 -- frame_size_in_us @tablebulletend */

   /*#< @h2xmle_description {Specifies whether the frame size of the PCM
                             encoder module is in samples or microseconds.}
        @h2xmle_default     {0}
        @h2xmle_rangeList   {"frame_size_not_specified"=0;
                             "frame_size_in_samples"=1;
                             "frame_size_in_us"=2} */

    uint32_t frame_size_in_samples;
   /**< Frame size of the PCM encoder is in samples.

        This value is used when frame_size_type is set to 1 and this value is
        greater than zero.

        We recommend limiting the frame size to 100 ms (or 384 kHz * 100 ms =
        38400 samples). */

   /*#< @h2xmle_description {Frame size of the PCM encoder is in samples.
                             This value is used when frame_size_type is set
                             to 1 and this value is greater than 0. \n
                             We recommend limiting the frame size to 100 ms
                             (or 384 kHz * 100 ms = 38400 samples).}
        @h2xmle_default     {0}
        @h2xmle_range       {0..38400} */

    uint32_t frame_size_in_us;
   /**< Frame size of the PCM encoder is in microseconds.

        This parameter value is used when frame_size_type is set to 2 and this
        value is greater than 0. */

   /*#< @h2xmle_description {Frame size of the PCM encoder is in microseconds.
                             This parameter value is used when frame_size_type
                             is set to 2 and this value is greater than 0.}
        @h2xmle_default     {0}
        @h2xmle_range       {0..100000} */
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;
typedef struct param_id_pcm_encoder_frame_size_t param_id_pcm_encoder_frame_size_t;

/*------------------------------------------------------------------------------
   Module
------------------------------------------------------------------------------*/

/** @ingroup ar_spf_mod_encoder_mod
    Identifier for the module used as the encoder for PCM use cases.

    This module can be used to convert the properties of a PCM stream, such as
    endianness, interleaving, bit width, number of channels, and so on. It
    cannot be used to convert the sampling rate.

    The module has only one input and one output port.

    @note1hang The module does not support #PARAM_ID_ENCODER_OUTPUT_CONFIG
               because PARAM_ID_PCM_OUTPUT_FORMAT_CFG covers the supported
               controls.

    @subhead4{Supported parameter ID}
    - #PARAM_ID_PCM_OUTPUT_FORMAT_CFG

    @subhead4{Supported input media format ID}
    - Data format       : #DATA_FORMAT_FIXED_POINT @lstsp1
    - fmt_id            : Don't care @lstsp1
    - Sample rates      : 1..384 kHz @lstsp1
    - Number of channels: 1..32 @lstsp1
    - Bit width: @lstsep
       - 16 (bits per sample 16 and Q15) @lstsp2
       - 24 (bits per sample 24 and Q23, bits per sample 32 and Q23, Q27, or
             Q31) @lstsp2
       - 32 (bits per sample 32 and Q31) @newpage
    - Interleaving: @lstsep
       - Interleaved @lstsp2
       - De-interleaved unpacked @lstsp2
       - De-interleaved packed @lstsp1
    - Endianness: little
 */
#define MODULE_ID_PCM_ENC              0x07001004

/*# @h2xmlm_module             {"MODULE_ID_PCM_ENC", MODULE_ID_PCM_ENC}
    @h2xmlm_displayName        {"PCM Encoder"}
    @h2xmlm_modSearchKeys	   {Encoder}
    @h2xmlm_description        {ID for the module used as the encoder for PCM
                                use cases. \n
                                This module can be used to convert the
                                properties of a PCM stream, such as
                                endianness, interleaving, bit width, number of
                                channels, and so on. It cannot be used to
                                convert the sampling rate. For more details,
                                see AudioReach Signal Processing Framework (SPF) API Reference.}
    @h2xmlm_dataMaxInputPorts  {1}
    @h2xmlm_dataMaxOutputPorts {1}
    @h2xmlm_dataInputPorts     {IN=PCM_ENC_DATA_INPUT_PORT}
    @h2xmlm_dataOutputPorts    {OUT=PCM_ENC_DATA_OUTPUT_PORT}
    @h2xmlm_supportedContTypes {APM_CONTAINER_TYPE_GC}
    @h2xmlm_isOffloadable      {false}
    @h2xmlm_stackSize          {4096}

    @{                      <-- Start of the Module -->
    @h2xml_Select           {"param_id_pcm_output_format_cfg_t"}
    @h2xmlm_InsertParameter
    @h2xml_Select           {"payload_pcm_output_format_cfg_t"}
    @h2xmlm_InsertParameter
    @h2xml_Select           {param_id_pcm_output_format_cfg_t::data_format}
    @h2xmle_rangeEnum       {pcm_data_format}
    @h2xml_Select           {"param_id_pcm_encoder_frame_size_t"}
    @h2xmlm_InsertParameter
    @h2xml_Select           {param_id_chmixer_coeff_t}
    @h2xmlm_InsertParameter
    @h2xml_Select           {chmixer_coeff_t}
    @h2xmlm_InsertStructure
    @}                      <-- End of the Module --> */


#endif // PCM_ENCODER_API_H_
