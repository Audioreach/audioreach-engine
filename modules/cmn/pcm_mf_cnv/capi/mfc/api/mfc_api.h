#ifndef MFC_API_H
#define MFC_API_H

/*==============================================================================
 @file mfc_api.h
 @brief This file contains MFC API
==============================================================================*/

/*===========================================================================
  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
  SPDX-License-Identifier: BSD-3-Clause-Clear
======================================================================== */

/*------------------------------------------------------------------------------
   Includes
------------------------------------------------------------------------------*/
#include "chmixer_common_api.h"
#include "module_cmn_api.h"

/*# @h2xml_title1          {Media Format Converter (MFC) Module API}
    @h2xml_title_agile_rev {Media Format Converter (MFC) Module API}
    @h2xml_title_date      {December 11, 2018} */

/*------------------------------------------------------------------------------
   Defines
------------------------------------------------------------------------------*/

/** @ingroup ar_spf_mod_mfc_mod
    Identifier for the input port of the MFC module (#MODULE_ID_MFC). */
#define MFC_DATA_INPUT_PORT   0x2

/** @ingroup ar_spf_mod_mfc_mod
    Identifier for the output port of the MFC module. */
#define MFC_DATA_OUTPUT_PORT  0x1

/*------------------------------------------------------------------------------
   API's
------------------------------------------------------------------------------*/

/** @ingroup ar_spf_mod_mfc_mod
    Identifier for the parameter that configures the resampler in the MFC
    module to be either an IIR resampler or FIR resampler.

    For an FIR resampler, clients can set additional FIR resampler
    configuration fields.

    @msgpayload
    param_id_mfc_resampler_cfg_t
*/
#define PARAM_ID_MFC_RESAMPLER_CFG                0x0800104D

/*# @h2xmlp_parameter   {"PARAM_ID_MFC_RESAMPLER_CFG",
                          PARAM_ID_MFC_RESAMPLER_CFG}
    @h2xmlp_description {ID for the parameter that configures the resampler in
                         the MFC module (MODULE_ID_MFC)to be either an IIR
                         resampler, FIR resampler or IIR preferred resampler.}
    @h2xmlp_toolPolicy  {Calibration; RTC} */

/** @ingroup ar_spf_mod_mfc_mod
    Payload for #PARAM_ID_MFC_RESAMPLER_CFG in the Media Format Converter
    Module.
*/
#include "spf_begin_pack.h"
struct param_id_mfc_resampler_cfg_t
{
   uint32_t resampler_type;
   /**< Specifies the type of resampler to use.

        @valuesbul
        - 0 -- FIR resampler (Default)
        - 1 -- IIR resampler
        - 2 -- IIR resampler preferred: Uses IIR when possible, if not will fallback on FIR
        */

   /*#< @h2xmle_description {Specifies the resampler type to be used in MFC. \n
                             If IIR is selected, the use_hw_rs, dynamic_mode, and delay_type fields
                             are not applicable and are ignored.\n

        					 If FIR resampler or IIR preferred is selected, the use_hw_rs field is applicable,
        					 it is cached and used.\n
        
        					 If FIR resampler is selected dynamic_mode, and delay_type fields are also applicable
        					 and they are cached and used.\n

							 If the input or output sampling rates are fractional:
        						- IIR Preferred Resampler will fallback on FIR
        						- IIR Resampler will NOT fallback on FIR, module will disable itself \n
                            }
        @h2xmle_rangeList   {"FIR resampler"=0;
                             "IIR Resampler"=1;
                             "IIR Resampler Preferred"=2}
        @h2xmle_default     {0} */

   uint32_t use_hw_rs;
   /**< Specifies whether to use the hardware or software resampler for the
        FIR resampler.

        @valuesbul
        - 0 -- Software Resampler (Default)
        - 1 -- Hardware Resampler

       @contcell
        If the hardware resampler is selected, the dynamic_mode and delay_type
        values are saved but they are ignored.
         - The hardware resampler can be created only if the chip supports it.
         - If hardware resampler creation fails for any reason, the software
           resampler is created with a saved dynamic mode, and the client sets
           the delay type.

        If the software resampler is selected, the dynamic_mode and delay_type
        fields are saved and used.

        A Get parameter query on this API will return the actual resampler
        being used. It is not simply what the client set. */

   /*#< @h2xmle_description {Specifies whether to use the hardware or software
                             resampler for the FIR resampler or IIR preferred resampler. \n
                             If the hardware resampler is selected, the
                             dynamic_mode and delay_type values are saved but
                             they are ignored. \n
                             If the software resampler is selected, these
                             fields are saved and used. \n
                             For more details, see AudioReach Signal Processing Framework (SPF) API Reference.}
        @h2xmle_rangeList   {"Software Resampler"=0,
                             "Hardware Resampler"=1}
        @h2xmle_default     {0} */

   uint16_t dynamic_mode;
   /**< Specifies the operation mode for the FIR resampler.

        @valuesbul
        - 0 -- Generic resampling (Default)
        - 1 -- Dynamic resampling

        This dynamic_mode value is considered only if the software resampler
        is used. */

   /*#< @h2xmle_description {Specifies the operation mode for the FIR
                             resampler. This dynamic_mode value is
                             considered only if the software resampler is
                             used.}
        @h2xmle_rangeList   {"Generic resampling"=0;
                             "Dynamic resampling"=1}
        @h2xmle_default     {0} */

   uint16_t delay_type;
   /**< Specifies the delay type for the FIR resampler.

        @valuesbul
        - 0 -- High delay with smooth transition (Default)
        - 1 -- Low delay with visible transitional phase distortion

        This value is considered only if the software resampler is used and
        the dynamic_mode value is  set to 1. */

   /*#< @h2xmle_description {Specifies the delay type for the FIR resampler.
                             This value is considered only if the software
                             resampler is used and the dynamic_mode value is 
                             set to 1.}
        @h2xmle_rangeList   {"High delay with smooth transition"=0;
                             "Low delay with visible transitional phase
                              distortion"=1}
        @h2xmle_default     {0} */
}
#include "spf_end_pack.h"
;
typedef struct param_id_mfc_resampler_cfg_t param_id_mfc_resampler_cfg_t;


/** @ingroup ar_spf_mod_mfc_mod
    Identifier for the output media format parameter used by #MODULE_ID_MFC.

    @msgpayload
    param_id_mfc_output_media_fmt_t @newpage
*/
#define PARAM_ID_MFC_OUTPUT_MEDIA_FORMAT            0x08001024

/*# @h2xmlp_parameter   {"PARAM_ID_MFC_OUTPUT_MEDIA_FORMAT",
                          PARAM_ID_MFC_OUTPUT_MEDIA_FORMAT}
    @h2xmlp_description {ID for the output media format parameter used by the
                         MFC module (MODULE_ID_MFC).}
    @h2xmlp_toolPolicy  {Calibration; RTC}
    @h2xmlp_maxSize     {72} */

/** @ingroup ar_spf_mod_mfc_mod
    Payload of the #PARAM_ID_MFC_OUTPUT_MEDIA_FORMAT parameter in the Media
    Format Converter Module. Following this structure is the variable payload
    for channel_map.
 */
#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
struct param_id_mfc_output_media_fmt_t
{
   int32_t sampling_rate;
   /**< Sampling rate in samples per second.

        @valuesbul
        - If the IIR resampler type is used:
           - #PARAM_VAL_NATIVE (Default)
           - #PARAM_VAL_UNSET
           - 8, 16, 24, 32, 48 kHz

        - If the FIR resampler type is used, all values are allowed:
           - #PARAM_VAL_NATIVE (Default)
           - #PARAM_VAL_UNSET
           - 8, 11.025, 12 kHz, 16, 22.05, 24, 32, 44.1, 48, 88.2, 96, 176.4,
             192, 352.8 kHz, 384 kHz @tablebulletend */

   /*#< @h2xmle_description {Sampling rate in samples per second. \n
                             If the IIR resampler type is used, only the
                             following sample rates are allowed:
                             PARAM_VAL_NATIVE and PARAM_VAL_UNSET; and 8, 16,
                             24, 32, and 48 kHz. \n
                             If the FIR resampler type is used, all values are
                             allowed.}
        @h2xmle_rangeList   {"PARAM_VAL_UNSET"= -2;
                             "PARAM_VAL_NATIVE"= -1;
                             "8 kHz"=8000;
                             "11.025 kHz"=11025;
                             "12 kHz"=12000;
                             "16 kHz"=16000;
                             "22.05 kHz"=22050;
                             "24 kHz"=24000;
                             "32 kHz"=32000;
                             "44.1 kHz"=44100;
                             "48 kHz"=48000;
                             "88.2 kHz"=88200;
                             "96 kHz"=96000;
                             "176.4 kHz"=176400;
                             "192 kHz"=192000;
                             "352.8 kHz"=352800;
                             "384 kHz"=384000}
        @h2xmle_default     {-1} */

   int16_t bit_width;
   /**< Bit width of the audio samples.

        @valuesbul
        - #PARAM_VAL_NATIVE
        - #PARAM_VAL_UNSET
        - #BIT_WIDTH_16
        - #BIT_WIDTH_24
        - #BIT_WIDTH_32

        Samples with a bit width of 16 (Q15 format) are stored in 16-bit
        words. Samples with a bit width 24 bits (Q27 format) or 32 bits (Q31
        format) are stored in 32-bit words. */

   /*#< @h2xmle_description {Bit width of the audio samples. \n
                             Samples with a bit width of 16 (Q15 format) are
                             stored in 16-bit words. \n
                             Samples with a bit width 24 bits (Q27 format) or
                             32 bits (Q31 format) are stored in 32-bit words.}
        @h2xmle_rangeList   {"PARAM_VAL_NATIVE"= -1;
                             "PARAM_VAL_UNSET"= -2;
                             "BIT_WIDTH_16"=16;
                             "BIT_WIDTH_24"=24;
                             "BIT_WIDTH_32"=32}
        @h2xmle_default     {-1} */

   int16_t num_channels;
   /**< Number of channels in the array.

        @values -2 through 32 */

   /*#< @h2xmle_description {Number of channels.}
        @h2xmle_range       {-2..MODULE_CMN_MAX_CHANNEL}
        @h2xmle_default     {-1} */

   uint16_t channel_type[0];
   /**< Array of channel mappings.

        Specify a channel mapping for each output channel. If the number of
        channels is not a multiple of four, zero padding must be added to the
        array to align the packet to a multiple of 32 bits.

        If num_channels is set to PARAM_VAL_NATIVE (-1) or PARAM_VAL_UNSET(-2),
        this field is ignored. */

   /*#< @h2xmle_description       {Array of channel mappings. Specify a channel
                                   mapping for each output channel. \n
                                   If the number of channels is not a multiple
                                   of four, zero padding must be added to array
                                   to align the packet to a multiple of 32
                                   bits . \n
                                   If num_channels is set to PARAM_VAL_NATIVE
                                   (-1) or PARAM_VAL_UNSET(-2), this field is
                                   ignored.}
        @h2xmle_variableArraySize {num_channels}
        @h2xmle_rangeEnum         {pcm_channel_map}
        @h2xmle_default           {1} */
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;
typedef struct param_id_mfc_output_media_fmt_t param_id_mfc_output_media_fmt_t;


/*------------------------------------------------------------------------------
   Module
------------------------------------------------------------------------------*/

/** @ingroup ar_spf_mod_mfc_mod
    Identifier for the Media Format Converter (MFC) module.

    @subhead4{Supported parameter IDs}
    - #PARAM_ID_MFC_OUTPUT_MEDIA_FORMAT @lstsp1
    - #PARAM_ID_MFC_RESAMPLER_CFG @lstsp1
    - #PARAM_ID_CHMIXER_COEFF

    Resampling is supported only for an integer sampling rate ratio, for
    example, 48 kHz and 8 kHz.

    @subhead4{Supported input media format ID}
    - Data Format          : #DATA_FORMAT_FIXED_POINT @lstsp1
    - fmt_id               : Don't care @lstsp1
    - Sample Rates         : >0 to 384 kHz @lstsp1
    - Number of channels   : 1 to 128 (for certain products this module supports only 32 channels) @lstsp1
    - Channel type         : 0..128 @lstsp1
    - Bits per sample      : 16, 32 @lstsp1
    - Q format             : 15, 27, 31 @lstsp1
    - Interleaving         : De-interleaved unpacked @lstsp1
    - Signed/unsigned      : Don't care
*/
#define MODULE_ID_MFC                               0x07001015

/*# @h2xmlm_module             {"MODULE_ID_MFC", MODULE_ID_MFC}
    @h2xmlm_displayName        {"MFC"}
    @h2xmlm_modSearchKeys	   {resampler, channel mixer, byte converter, Audio, Voice}
    @h2xmlm_description        {ID for the Media Format Converter (MFC) module.
                                For more details, see AudioReach Signal Processing Framework (SPF) API Reference.}
    @h2xmlm_dataMaxInputPorts  {1}
    @h2xmlm_dataInputPorts     {IN=MFC_DATA_INPUT_PORT}
    @h2xmlm_dataOutputPorts    {OUT=MFC_DATA_OUTPUT_PORT}
    @h2xmlm_dataMaxOutputPorts {1}
    @h2xmlm_supportedContTypes {APM_CONTAINER_TYPE_SC, APM_CONTAINER_TYPE_GC}
    @h2xmlm_isOffloadable      {true}
    @h2xmlm_stackSize          {4096}

    @{                      <-- Start of the Module -->
    @h2xml_Select           {param_id_mfc_resampler_cfg_t}
    @h2xmlm_InsertParameter
    @h2xml_Select           {param_id_mfc_output_media_fmt_t}
    @h2xmlm_InsertParameter
    @h2xml_Select           {param_id_chmixer_coeff_t}
    @h2xmlm_InsertParameter
    @h2xml_Select           {chmixer_coeff_t}
    @h2xmlm_InsertStructure
    @}                      <-- End of the Module --> */


#endif // MFC_API_H
