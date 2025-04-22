#ifndef COMMON_ENC_DEC_API_H
#define COMMON_ENC_DEC_API_H
/**
 * \file common_enc_dec_api.h
 * \brief
 *    This file contains media format IDs and definitions
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "module_cmn_api.h"

/** @addtogroup ar_spf_mod_encdec_mods
    These APIs are used by all encoder and decoders, including the PCM Decoder
    and PCM Encoder modules.
*/

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/*# @h2xml_title1           {Common APIs for Encoders and Decoders}
    @h2xml_title_agile_rev  {Common APIs for Encoders and Decoders}
    @h2xml_title_date       {August 13, 2018} */

/*# @h2xmlx_xmlNumberFormat {int} */

/** @ingroup ar_spf_mod_encdec_mods
    Identifier for the parameter used to set the real module ID for the
    placeholder modules.

    @msgpayload
    param_id_placeholder_real_module_id_t
 */
#define PARAM_ID_REAL_MODULE_ID 0x0800100B

/*# @h2xmlp_parameter   {"PARAM_ID_REAL_MODULE_ID", PARAM_ID_REAL_MODULE_ID}
    @h2xmlp_description {ID for the parameter used to set the real module ID
                         for the placeholder modules.}
    @h2xmlp_toolPolicy  {Calibration} */

/** @ingroup ar_spf_mod_encdec_mods
    Payload of the #PARAM_ID_REAL_MODULE_ID parameter.
 */
#include "spf_begin_pack.h"
struct param_id_placeholder_real_module_id_t
{
   uint32_t real_module_id;
   /**< Identifier of the real module ID to be used in place of a placeholder
        module. For example, MODULE_ID_AAC_DEC or MODULE_ID_AAC_ENC. */

   /*#< @h2xmle_description {ID for the real module ID to be used in place of
                             a placeholder module. For example,
                             MODULE_ID_AAC_DEC or MODULE_ID_AAC_ENC.}
        @h2xmle_default     {0}
        @h2xmle_range       {0..0xFFFFFFFF}
        @h2xmle_policy      {Basic} */
}
#include "spf_end_pack.h"
;
typedef struct param_id_placeholder_real_module_id_t param_id_placeholder_real_module_id_t;

/** @ingroup ar_spf_mod_encdec_mods
    Identifier for the parameter used by #MODULE_ID_PLACEHOLDER_ENCODER and
    #MODULE_ID_PLACEHOLDER_DECODER to reset the placeholder module.

    After setting the real module ID to the placeholder using
    #PARAM_ID_REAL_MODULE_ID, the module is no longer a placeholder. Any Set or
    Get command goes directly to the CAPI.

    The reset involves destroying the current real module ID. After resetting,
    the PARAM_ID_REAL_MODULE_ID can be sent again. After reset, the module
    regains its placeholder state. @newpage
 */
#define PARAM_ID_RESET_PLACEHOLDER_MODULE 0x08001173

/** @ingroup ar_spf_mod_encdec_mods
    Identifier for the Placeholder Encoder module, which alleviates the need
    to create a graph for every encoder where a placeholder module is used.

    The graph definition contains the placeholder module ID. It is replaced by
    the real encoder module before the graph is started (if enabled).

    @subhead4{Supported parameter IDs}
    - #PARAM_ID_REAL_MODULE_ID -- Used to replace the placeholder module
      with the real module. @lstsep
      - The placeholder module can be replaced only once. @lstsp2
      - When a Set parameter is issued to the real module before
        PARAM_ID_REAL_MODULE_ID is set, the Set parameter is cached.
        @lstsp1
    - #PARAM_ID_RESET_PLACEHOLDER_MODULE -- Used to set a new real module ID.
      @lstsp1
    - #PARAM_ID_MODULE_ENABLE -- Used to enable or disable the module. When
      disabled, the module is bypassed.

    @subhead4{Supported media formats}
    - Formats that the encoder modules support.
 */
#define MODULE_ID_PLACEHOLDER_ENCODER 0x07001008

/*# @h2xmlm_module             {"MODULE_ID_PLACEHOLDER_ENCODER",
                                 MODULE_ID_PLACEHOLDER_ENCODER}
    @h2xmlm_displayName        {"Placeholder Encoder"}
    @h2xmlm_modSearchKeys	   {Encoder, Audio, Voice}
    @h2xmlm_description        {ID for the module used to alleviate the need
                                for creating a graph for every encoder where a
                                placeholder module is used. For more details,
                                see AudioReach Signal Processing Framework (SPF) API Reference.}
    @h2xmlm_dataMaxInputPorts  {1}
    @h2xmlm_dataMaxOutputPorts {1}
    @h2xmlm_dataOutputPorts    {OUT=1}
    @h2xmlm_dataInputPorts     {IN=2}
    @h2xmlm_supportedContTypes {APM_CONTAINER_TYPE_GC}
    @h2xmlm_isOffloadable      {true}
    @h2xmlm_stackSize          {1024}
    @h2xmlm_toolPolicy         {Calibration}

    @{                      <-- Start of the Module -->
    @h2xml_Select           {"param_id_placeholder_real_module_id_t"}
    @h2xmlm_InsertParameter
    @h2xml_Select           {"param_id_module_enable_t"}
    @h2xmlm_InsertParameter
    @h2xmlp_toolPolicy      {Calibration}
    @h2xml_Select           {param_id_module_enable_t::enable}
    @h2xmle_default         {1}
    @}                      <-- End of the Module --> */

/** @ingroup ar_spf_mod_encdec_mods
    Identifier for the Placeholder Decoder module, which alleviates the need
    to create a graph for every decoder where a placeholder module is used.

    The graph definition contains the placeholder module ID. It is replaced by
    the real decoder module before the graph is started (if enabled).

    @subhead4{Supported parameter IDs}
    - #PARAM_ID_REAL_MODULE_ID -- Used to replace the placeholder module with
      the real module. @lstsep
      - The placeholder module can be replaced only once. @lstsp2
      - When a Set parameter is issued to the real module before
        PARAM_ID_REAL_MODULE_ID is set, the Set parameter is cached.
        @lstsp1
    - #PARAM_ID_RESET_PLACEHOLDER_MODULE -- used to set a new real module ID.
      @lstsp1
    - #PARAM_ID_MODULE_ENABLE -- used to enable or disable the module. When
      disabled, module is bypassed.

    @subhead4{Supported data format}
    - #DATA_FORMAT_RAW_COMPRESSED

    @subhead4{Supported media formats}
    - Formats that the decoder modules support. @newpage
 */
#define MODULE_ID_PLACEHOLDER_DECODER 0x07001009

/*# @h2xmlm_module             {"MODULE_ID_PLACEHOLDER_DECODER",
                                 MODULE_ID_PLACEHOLDER_DECODER}
    @h2xmlm_displayName        {"Placeholder Decoder"}
    @h2xmlm_modSearchKeys	   {Decoder, Audio, Voice}
    @h2xmlm_description        {ID for the module used to alleviate the need
                                for creating a graph for every decoder where a
                                placeholder module is used. For more details,
                                see AudioReach Signal Processing Framework (SPF) API Reference.}
    @h2xmlm_dataMaxInputPorts  {1}
    @h2xmlm_dataMaxOutputPorts {1}
    @h2xmlm_dataOutputPorts    {OUT=1}
    @h2xmlm_dataInputPorts     {IN=2}
    @h2xmlm_supportedContTypes {APM_CONTAINER_TYPE_GC}
    @h2xmlm_stackSize          {1024}
    @h2xmlm_toolPolicy         {Calibration}

    @{                      <-- Start of the Module -->
    @h2xml_Select           {"param_id_placeholder_real_module_id_t"}
    @h2xmlm_InsertParameter
    @h2xml_Select           {"param_id_module_enable_t"}
    @h2xmlm_InsertParameter
    @h2xmlp_toolPolicy      {Calibration}
    @h2xml_Select           {param_id_module_enable_t::enable}
    @h2xmle_default         {1}
    @}                      <-- End of the Module --> */

/*Enums defined for supported values of data_format field in param_id_pcm_output_format_cfg_t
Modules need to import enums based on supported values of data format*/
/* Enum for data format fixed point, default for all modules */
enum pcm_data_format
{
   INVALID_VAL             = 0,
   FIXED_POINT_DATA_FORMAT = DATA_FORMAT_FIXED_POINT,
   FLOATING_POINT_DATA_FORMAT = DATA_FORMAT_FLOATING_POINT,
};

/** @ingroup ar_spf_mod_encdec_mods
    Identifier for the parameter that configures the PCM output format of the
    PCM encoder, PCM decoder, or PCM converter.

    Following are example scenarios that require this parameter:
    - PCM decoder upstream of the PCM converter:
       - The parameter must be set on both the decoder and converter.
       - The converter acts as backup for the decoder.
       - If the decoder is not configured for 24-bit output and it outputs 16
         bits, there is no benefit in using the converter to get 24 bits.
    - PCM converter upstream of PCM encoder:
       - The parameter must be set only on the converter.
    - PCM converter use cases in Generic and Specialized containers.

    Following is the overall structure:

    @code
    param_id_pcm_output_format_cfg_t pcfg;
    uint8_t payload[payload_size]
    @endcode @vertspace{6}

    Following is the overall structure when fmt_id = #MEDIA_FMT_ID_PCM
    and data_format = #DATA_FORMAT_FIXED_POINT:

    @code
    {
       payload_pcm_output_format_cfg_t cfg;
       uint8_t channel_mapping[num_channels];
       uint8_t 32bit_padding[if_any];
    }
    @endcode

    @msgpayload
    param_id_pcm_output_format_cfg_t \n
    @indent{12pt} payload_pcm_output_format_cfg_t
    @par
    If configuration of payload_pcm_output_format_cfg_t is inconsistent, the
    result is an error. Examples:
    - If the bit width is 16, the Q format cannot be 27. @lstsp1
    - If the bit width, bits per sample, alignment, and Q format are related.
      Thus, if one is native, the others must also be native. If one is unset,
      the others must be unset. @newpage
 */
#define PARAM_ID_PCM_OUTPUT_FORMAT_CFG 0x08001008

/*# @h2xmlp_parameter   {"PARAM_ID_PCM_OUTPUT_FORMAT_CFG",
                          PARAM_ID_PCM_OUTPUT_FORMAT_CFG}
    @h2xmlp_description {Identifier for the parameter that configures the PCM
                         output format of the PCM encoder, PCM decoder, or PCM
                         converter. For more details, see AudioReach Signal Processing Framework (SPF) API Reference.}
    @h2xmlp_toolPolicy  {Calibration} */

/** @ingroup ar_spf_mod_encdec_mods
    Payload of the #PARAM_ID_PCM_OUTPUT_FORMAT_CFG parameter.
 */
#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
struct param_id_pcm_output_format_cfg_t
{
   uint32_t data_format;
   /**< Format of the PCM output data.

        @valuesbul
        - #INVALID_VALUE (Default)
        - #DATA_FORMAT_FIXED_POINT
        - #DATA_FORMAT_FLOATING_POINT @tablebulletend */

   /*#< @h2xmle_description {Format of the data}
        @h2xmle_rangeEnum   {pcm_data_format}
        @h2xmle_policy      {Basic} */

   uint32_t fmt_id;
   /**< Identifier for the data stream format.

        @valuesbul
        - #INVALID_VALUE
        - #MEDIA_FMT_ID_PCM @tablebulletend */

   /*#< @h2xmle_description {ID for the data stream format.}
        @h2xmle_default     {0}
        @h2xmle_rangeList   {"INVALID_VALUE"=0,
                             "Media format ID of PCM"=MEDIA_FMT_ID_PCM}
        @h2xmle_policy      {Basic} */

   uint32_t payload_size;
/**< Size of the payload that immediately follows this structure.

     The payload size does not include bytes added for 32-bit alignment. */

/*#< @h2xmle_description {Size of the payload that immediately follows this
                          structure. The payload size does not include
                          bytes added for 32-bit alignment.}
     @h2xmle_default     {0}
     @h2xmle_range       {0..0xFFFFFFFF}
     @h2xmle_policy      {Basic} */

#if defined(__H2XML__)
   uint8_t payload[0];
/**< Payload for PCM output format configuration of size payload_size.

     The payload structure varies depending on the combination of the
     data_format and fmt_id fields. For example, if data_format =
     #DATA_FORMAT_FIXED_POINT or #DATA_FORMAT_FLOATING_POINT and fmt_id = #MEDIA_FMT_ID_PCM, the payload is
     payload_pcm_output_format_cfg_t. */

/*#< @h2xmle_description       {Payload for PCM output format configuration
                                of size payload_size. \n
                                The payload structure varies depending on
                                the combination of the data_format and
                                fmt_id fields. For example, if data_format
                                is DATA_FORMAT_FIXED_POINT or #DATA_FORMAT_FLOATING_POINT and fmt_id is
                                MEDIA_FMT_ID_PCM, the payload is
                                payload_pcm_output_format_cfg_t.}
    @h2xmle_policy            {Basic}
    @h2xmle_variableArraySize {payload_size} */
#endif
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;
typedef struct param_id_pcm_output_format_cfg_t param_id_pcm_output_format_cfg_t;

/*# @h2xmlp_subStruct
    @h2xmlp_description {Payload for PARAM_ID_PCM_OUTPUT_FORMAT_CFG when fmt_id
                         = MEDIA_FMT_ID_PCM and data_format is fixed point or floating point.
                         For more details, see AudioReach Signal Processing Framework (SPF) API Reference.}
    @h2xmlp_toolPolicy  {Calibration} */

/** @ingroup ar_spf_mod_encdec_mods
    Payload of the #PARAM_ID_PCM_OUTPUT_FORMAT_CFG when fmt_id =
    #MEDIA_FMT_ID_PCM and data_format = #DATA_FORMAT_FIXED_POINT or #DATA_FORMAT_FLOATING_POINT.
 */

#ifdef PROD_SPECIFIC_MAX_CH
#define MAX_NUM_CHANNELS_V2 128
#else
#define MAX_NUM_CHANNELS_V2 32
#endif

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
struct payload_pcm_output_format_cfg_t
{
   int16_t bit_width;
   /**< Bit width of each sample.

        @valuesbul
        - #PARAM_VAL_UNSET
        - #PARAM_VAL_NATIVE
        - #PARAM_VAL_INVALID
        - #BIT_WIDTH_16
        - #BIT_WIDTH_24
        - #BIT_WIDTH_32 - Valid for fixed and floating point data
        - #BIT_WIDTH_64 - Valid only for floating point data @tablebulletend */

   /*#< @h2xmle_description {Bit width of each sample (such as 16, 24,32 or 64
                             bits.}
        @h2xmle_default     {0}
        @h2xmle_rangeList   {"PARAM_VAL_UNSET"=-2;
                             "PARAM_VAL_NATIVE"=-1;
                             "PARAM_VAL_INVALID"=0;
                             "BIT_WIDTH_16"=16;
                             "BIT_WIDTH_24"=24;
                             "BIT_WIDTH_32"=32;
                             "BIT_WIDTH_64"=64}
        @h2xmle_policy      {Basic} */

   int16_t alignment;
   /**< Indicates the alignment of bits_per_sample in sample_word_size.

        This field is relevant only when bit_width is 24 and bits_per_sample
        is 32.
        - For bit_width 24, bits_per_sample 32, and q_factor 23, alignment
          should be PCM_LSB_ALIGNED.
        - For bit_width 24, bits_per_sample 32, and q_factor 31, alignment
          should be PCM_MSB_ALIGNED. @tablebulletend */

   /*#< @h2xmle_description {Indicates the alignment of bits_per_sample in
                             sample_word_size. This field is relevant only when
                             bit_width is 24 and bits_per_sample is 32.
                             - For bit_width 24, bits_per_sample 32, and
                               q_factor 23, alignment should be
                               PCM_LSB_ALIGNED.
                             - For bit_width 24, bits_per_sample 32, and
                               q_factor 31, alignment should be
                               PCM_MSB_ALIGNED.}
        @h2xmle_default     {0}
        @h2xmle_rangeList   {"PARAM_VAL_UNSET"=-2;
                             "PARAM_VAL_NATIVE"=-1;
                             "PARAM_VAL_INVALID"=0;
                             "PCM_LSB_ALIGNED"=1;
                             "PCM_MSB_ALIGNED"=2}
        @h2xmle_policy      {Basic} */

   int16_t bits_per_sample;
   /**< Bits required to store one sample.

        For example:
        - 16-bits per sample always contain 16-bit samples.
        - 24-bits per sample always contain 24-bit samples.
        - 32-bits per sample have the following cases:
           - If bit width = 24 and alignment = PCM_LSB_ALIGNED, 24-bit samples
             are placed in the lower 24 bits of a 32-bit word. The upper bits
             might be sign-extended.
           - If bit width = 24 and alignment = PCM_MSB_ALIGNED, 24-bit samples
             are placed in the upper 24 bits of a 32-bit word. The lower bits
             might not be set to 0.
           - If bit width = 32, 32-bit samples are placed in the 32-bit
             words.
        - 64-bits per sample always contain 64-bit samples. @tablebulletend */

   /*#< @h2xmle_description {Bits required to store one sample. For example: \n
                             - 16-bits per sample always contain 16-bit
                               samples.
                             - 24-bits per sample always contain 24-bit
                               samples.
                             - 32-bits per sample have the following cases:
                                - If bit width = 24 and alignment =
                                  PCM_LSB_ALIGNED, 24-bit samples are placed in
                                  the lower 24 bits of a 32-bit word. The upper
                                  bits might be sign-extended.
                                - If bit width = 24 and alignment =
                                  PCM_MSB_ALIGNED, 24-bit samples are placed in
                                  the upper 24 bits of a 32-bit word. The lower
                                  bits might not be set to 0.
                                - If bit width = 32, 32-bit samples are placed
                                  in the 32-bit words.
                                - 64-bits per sample is valid only for floating point data format}
        @h2xmle_default     {0}
        @h2xmle_rangeList   {"PARAM_VAL_UNSET"=-2;
                             "PARAM_VAL_NATIVE"=-1;
                             "PARAM_VAL_INVALID"=0;
                             "16"=16;
                             "24"=24;
                             "32"=32;
                             "64"=64}
        @h2xmle_policy      {Basic} */

   int16_t q_factor;
   /**< Indicates the Q factor of the PCM data:
        - Q15 for 16-bit_width signed data.
        - Q23 for 24-bit_width signed packed (24-bits_per_sample) data.
        - Q27 for LSB-aligned, 24-bit_width unpacked (32-bits_per_sample)
          signed data used internally to the SPF.
        - Q31 for MSB-aligned, 24-bit_width unpacked (32 bits_per_sample)
          signed data.
        - Q23 for LSB-aligned, 24-bit_width unpacked (32 bits_per_sample)
          signed data.
        - Q31 for 32-bit_width signed data.
        - Not applicable for floating point data format @tablebulletend */

   /*#< @h2xmle_description {Indicates the Q factor of the PCM data: \n
                             - Q15 for 16-bit_width signed data
                             - Q23 for 24-bit_width signed packed
                               (24-bits_per_sample) data
                             - Q27 for LSB-aligned, 24-bit_width unpacked
                               (32-bits_per_sample) signed data used internally
                               to the SPF
                             - Q31 for MSB-aligned, 24-bit_width unpacked
                               (32 bits_per_sample) signed data
                             - Q23 for LSB-aligned, 24-bit_width unpacked
                               (32 bits_per_sample) signed data
                              -Q31 for 32-bit_width signed data}
        @h2xmle_default     {0}
        @h2xmle_rangeList   {"PARAM_VAL_UNSET"=-2;
                             "PARAM_VAL_NATIVE"=-1;
                             "PARAM_VAL_INVALID"=0;
                             "Q15"=15;
                             "Q23"=23;
                             "Q27"=27;
                             "Q31"=31}
        @h2xmle_policy      {Basic} */

   int16_t endianness;
   /**< Indicates whether PCM samples are stored in little endian or big endian
        format.

        @valuesbul
        - #PARAM_VAL_UNSET
        - #PARAM_VAL_NATIVE
        - #PARAM_VAL_INVALID (Default)
        - #PCM_LITTLE_ENDIAN
        - #PCM_BIG_ENDIAN @tablebulletend */

   /*#< @h2xmle_description {Indicates whether PCM samples are stored in little
                             endian or big endian format.}
        @h2xmle_default     {0}
        @h2xmle_rangeList   {"PARAM_VAL_UNSET"=-2;
                             "PARAM_VAL_NATIVE"=-1;
                             "PARAM_VAL_INVALID"=0;
                             "PCM_LITTLE_ENDIAN"=1;
                             "PCM_BIG_ENDIAN"=2}
        @h2xmle_policy      {Basic} */

   int16_t interleaved;
   /**< Indicates whether the data is interleaved.

        @valuesbul
        - #PARAM_VAL_UNSET
        - #PARAM_VAL_NATIVE
        - #PARAM_VAL_INVALID (Default)
        - #PCM_INTERLEAVED
        - #PCM_DEINTERLEAVED_PACKED
        - #PCM_DEINTERLEAVED_UNPACKED @tablebulletend */

   /*#< @h2xmle_description {Indicates whether the data is interleaved.}
        @h2xmle_default     {0}
        @h2xmle_rangeList   {"PARAM_VAL_UNSET"=-2;
                             "PARAM_VAL_NATIVE"=-1;
                             "PARAM_VAL_INVALID"=0;
                             "PCM_INTERLEAVED"=1;
                             "PCM_DEINTERLEAVED_PACKED"=2;
                             "PCM_DEINTERLEAVED_UNPACKED"=3}
        @h2xmle_policy      {Basic} */

   int16_t reserved;
   /**< Reserved for alignment; must be set to 0. */

   /*#< @h2xmle_description {Reserved for alignment; must be set to 0.}
        @h2xmle_default     {0}
        @h2xmle_policy      {Basic} */

   int16_t num_channels;
   /**< Number of channels in the array.

        @values -2 through MAX_NUM_CHANNELS_V2 (Default = 0) */

   /*#< @h2xmle_description {Number of channels.}
     @h2xmle_default     {0}
        @h2xmle_range       {-2..MAX_NUM_CHANNELS_V2}
     @h2xmle_policy      {Basic} */

#if defined(__H2XML__)
   uint8_t channel_mapping[0];
   /**< Array of channel mappings of size num_channels.

     Channel[i] mapping describes channel i. Each element i of the array
     describes channel i inside the buffer where i is less than
     num_channels.

     An unused channel is set to 0. */

   /*#< @h2xmle_description       {Array of channel mappings of size
                                num_channels. \n
                                Channel[i] mapping describes channel i. Each
                                element i of the array describes channel i
                                inside the buffer where i is less than
                                num_channels. An unused channel is set to 0.}
     @h2xmle_variableArraySize {num_channels}
   @h2xmle_rangeEnum         {pcm_channel_map}
   @h2xmle_default           {1}
     @h2xmle_policy            {Basic} */
#endif
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;
typedef struct payload_pcm_output_format_cfg_t payload_pcm_output_format_cfg_t;

/** @ingroup ar_spf_mod_encdec_mods
    Identifier for the parameter that sets the bit rate on the Encoder module.

    @msgpayload
    param_id_enc_bitrate_param_t
 */
#define PARAM_ID_ENC_BITRATE 0x08001052

/*# @h2xmlp_parameter   {"PARAM_ID_ENC_BITRATE", PARAM_ID_ENC_BITRATE}
    @h2xmlp_description {ID for the parameter that sets the bit rate on the
                         Encoder module.}
    @h2xmlp_toolPolicy  {Calibration} */

/** @ingroup ar_spf_mod_encdec_mods
    Payload of the #PARAM_ID_ENC_BITRATE parameter.
 */
#include "spf_begin_pack.h"
struct param_id_enc_bitrate_param_t
{
   uint32_t bitrate;
   /**< Defines the bit rate in bits per second. */

   /*#< @h2xmle_description {Defines the bit rate in bits per second.}
        @h2xmle_range       {1..1536000}
        @h2xmle_default     {1} */
}
#include "spf_end_pack.h"
;
typedef struct param_id_enc_bitrate_param_t param_id_enc_bitrate_param_t;

/** @ingroup ar_spf_mod_encdec_mods
    Identifier for the parameter that configures the frame control (AAC encoder
    frame size) on the Encoder module.

    @msgpayload
    param_id_enc_frame_size_control_t
 */
#define PARAM_ID_ENC_FRAME_SIZE_CONTROL 0x08001053

/*# @h2xmlp_parameter   {"PARAM_ID_ENC_FRAME_SIZE_CONTROL",
                          PARAM_ID_ENC_FRAME_SIZE_CONTROL}
    @h2xmlp_description {ID for the parameter that configures the frame
                         control (AAC encoder frame size) on the Encoder
                         module.}
    @h2xmlp_toolPolicy  {Calibration} */

/** @ingroup ar_spf_mod_encdec_mods
    Payload of the #PARAM_ID_ENC_FRAME_SIZE_CONTROL parameter.
 */
#include "spf_begin_pack.h"
struct param_id_enc_frame_size_control_t
{
   uint32_t frame_size_control_type;
   /**< Type of frame size control.

        @valuesbul
        - 0 -- MTU_SIZE
        - 1 -- PEAK_BIT_RATE
        - 2 -- BIT_RATE_MODE @tablebulletend */

   /*#< @h2xmle_description {Type of frame size control.}
        @h2xmle_rangeList   {"MTU_SIZE"=0;
                             "PEAK_BIT_RATE"=1;
                             "BIT_RATE_MODE"=2}
        @h2xmle_policy      {Basic} */

   uint32_t frame_size_control_value;
   /**< Value of the frame size control.

       @tblsubhdbul{MTU\_SIZE}
        - MTU size in bytes per the connected Bluetooth sink device.
        - The MTU size is expected to be@ge the frame size.

       @tblsubhdbul{MPEAK\_BIT\_RATE}
        - Peak bit rate in bits/second.
        - For example, 96000 if 96 kbps is the peak bitrate to be set.
        - The peak bit rate setting is assumed to be > the configured bitrate.

       @tblsubhdbul{MBIT\_RATE\_MODE}
        - Bit rate modes are Constant Bit Rate (CBR), Variable Bit Rate (VBR),
          and so on.
        - For example:
           - 0 -- CBR (Default)
           - 1 -- VBR @tablebulletend */

   /*#< @h2xmle_description {Value of the frame size control (MTU_SIZE,
                             PEAK_BIT_RATE, BIT_RATE_MODE.}
        @h2xmle_policy      {Basic} */
}
#include "spf_end_pack.h"
;
typedef struct param_id_enc_frame_size_control_t param_id_enc_frame_size_control_t;

/** @ingroup ar_spf_mod_encdec_mods
    Identifier for the parameter that is sent to a decoder to remove initial
    silence (zeros) introduced by the decoder.

    The parameter must be sent before the first data buffer is received by the
    decoder. Otherwise, no initial zeros are removed.

    If the parameter is sent multiple times before the decoder receives the
    first data buffer, only the most recent value is considered.

    The amount of initial silence is expected to be smaller than one frame.
    If the amount is larger than one frame, the entire first frame is dropped,
    but the rest of the initial zeros remain.

    @msgpayload
    param_id_remove_initial_silence_t
*/
#define PARAM_ID_REMOVE_INITIAL_SILENCE 0x0800114B

/*# @h2xmlp_parameter   {"PARAM_ID_REMOVE_INITIAL_SILENCE",
                          PARAM_ID_REMOVE_INITIAL_SILENCE}
    @h2xmlp_description {ID for the parameter that is sent to a decoder to
                         remove initial silence (zeros) introduced by the
                         decoder. \n
                         The parameter must be sent before the first data
                         buffer is received by the decoder. Otherwise, no
                         initial zeros will be removed. If the parameter is
                         sent multiple times before the decoder receives the
                         first data buffer, only the most recent value is
                         considered. \n
                         The amount of initial silence is expected to be
                         smaller than one frame. If the amount is larger than
                         one frame, the entire first frame is dropped, but the
                         rest of the initial zeros remain.}
    @h2xmlp_toolPolicy  {Calibration} */

/** @ingroup ar_spf_mod_encdec_mods
    Payload of the #PARAM_ID_REMOVE_INITIAL_SILENCE parameter.
 */
#include "spf_begin_pack.h"
struct param_id_remove_initial_silence_t
{
   uint32_t samples_per_ch_to_remove;
   /**< Number of samples per channel to remove. */

   /*#< @h2xmle_description {Number of samples per channel to remove.}
        @h2xmle_default     {0.}
        @h2xmle_range       {0..0xFFFFFFFF}
        @h2xmle_policy      {Basic} */
}
#include "spf_end_pack.h"
;
typedef struct param_id_remove_initial_silence_t param_id_remove_initial_silence_t;

/** @ingroup ar_spf_mod_encdec_mods
    Identifier for the parameter that is sent to a decoder to remove trailing
    silence (zeros) introduced by the decoder.

    The parameter must be sent before the last data buffer is received by the
    decoder. Otherwise, no trailing zeros will be removed.

    If this parameter is sent multiple times before the last data buffer is
    received by the decoder, only the most recent value is considered.

    The amount of trailing silence is expected to be smaller than one frame.
    If the amount is larger than one frame, the entire last frame will be
    dropped, but the rest of the trailing zeros will remain.

    @msgpayload
    param_id_remove_trailing_silence_t
 */
#define PARAM_ID_REMOVE_TRAILING_SILENCE 0x0800115D

/*# @h2xmlp_parameter   {"PARAM_ID_REMOVE_TRAILING_SILENCE",
                          PARAM_ID_REMOVE_TRAILING_SILENCE}
    @h2xmlp_description {Identifier for the parameter that is sent to a
                         decoder to remove trailing silence (zeros) introduced
                         by the decoder. \n
                         The parameter must be sent before the last data
                         buffer is received by the decoder. Otherwise, no
                         trailing zeros will be removed. If this parameter is
                         sent multiple times before the last data buffer is
                         received by the decoder, only the most recent value
                         is considered. \n
                         The amount of trailing silence is expected to be
                         smaller than one frame. If the amount is larger than
                         one frame, the entire last frame will be dropped, but
                         the rest of the trailing zeros will remain.}
    @h2xmlp_toolPolicy  {Calibration} */

/** @ingroup ar_spf_mod_encdec_mods
    Payload of the #PARAM_ID_REMOVE_TRAILING_SILENCE parameter.
 */
#include "spf_begin_pack.h"
struct param_id_remove_trailing_silence_t
{
   uint32_t samples_per_ch_to_remove;
   /**< Number of samples per channel to remove. */

   /*#< @h2xmle_description {Number of samples per channel to remove.}
        @h2xmle_default     {0.}
        @h2xmle_range       {0..0xFFFFFFFF}
        @h2xmle_policy      {Basic} */
}
#include "spf_end_pack.h"
;
typedef struct param_id_remove_trailing_silence_t param_id_remove_trailing_silence_t;

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* COMMON_ENC_DEC_API_H */
