#ifndef CHMIXER_COMMON_API
#define CHMIXER_COMMON_API

/*==============================================================================
 @file chmixer_common_api.h
 @brief This file contains common parameters
==============================================================================*/

/*=======================================================================
Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
SPDX-License-Identifier: BSD-3-Clause-Clear
=========================================================================*/

#include "ar_defs.h"
#include "media_fmt_api.h"
#include "module_cmn_api.h"

/*==============================================================================
   Constants
==============================================================================*/

/*# @h2xmlp_subStruct */

/** @ingroup ar_spf_mod_chmixer_mod
    Part of the payload of #PARAM_ID_CHMIXER_COEFF.
*/
#include "spf_begin_pack.h"
struct chmixer_coeff_t
{
   uint16_t num_output_channels;
   /**< Number of output channels in the array.

        @values 1 through 128 (for some products it is 32) (Default = 1) */

   /*#< @h2xmle_description {Number of output channels.}
        @h2xmle_range       {1..MODULE_CMN_MAX_CHANNEL}
        @h2xmle_default     {1} */

   uint16_t num_input_channels;
   /**< Number of input channels in the array.

        @values 1 through 128 (for some products it is 32) (Default = 1)

        This value is followed by the payload consisting of the output channel
        map, input channel map, and coefficients. */

   /*#< @h2xmle_description {Number of input channels. This value is followed
                             by the payload consisting of the output channel
                             map, input channel map, and coefficients.}
        @h2xmle_default     {1}
        @h2xmle_range       {1..MODULE_CMN_MAX_CHANNEL} */

#ifdef __H2XML__
   uint16_t out_chmap[0];
   /**< Array of channel maps for the output channels. */

   /*#< @h2xmle_description       {Array of channel maps for the output
                                   channels.}
        @h2xmle_variableArraySize {num_output_channels}
        @h2xmle_rangeEnum         {pcm_channel_map}
        @h2xmle_default           {1} */

   uint16_t in_chmap[0];
   /**< Array of channel maps for the input channels. */

   /*#< @h2xmle_description       {Array of channel maps for the input
                                   channels.}
        @h2xmle_variableArraySize {num_input_channels}
        @h2xmle_rangeEnum         {pcm_channel_map}
        @h2xmle_default           {1} */

   uint16_t coeff[0];
   /**< Array of channel mixer coefficients: [num_output_channels]
        [num_input_channels]. */

   /*#< @h2xmle_description       {Array of channel mixer coefficients:
                                   [num_output_channels][num_input_channels].}
        @h2xmle_variableArraySize {"num_output_channels*num_input_channels"} */
#endif
}
#include "spf_end_pack.h"
;
typedef struct chmixer_coeff_t chmixer_coeff_t;


/*==============================================================================
   API definitions
==============================================================================*/

/** @ingroup ar_spf_mod_chmixer_mod
    Identifier for the coefficient parameter that #MODULE_ID_MFC uses to
    configure the channel mixer weighting coefficients.

    @msgpayload
    param_id_chmixer_coeff_t \n
    @indent{12pt} Immediately following this structure is a payload with the
                  (number of coefficients) * (variable-sized \n
    @indent{12pt}  chmixer coefficient arrays). \n
    @indent{12pt} chmixer_coeff_t
    @par
    The payload with (number of coefficients) * (variable-sized chmixer
    coefficient arrays) comprises the following:
    - num_output_channels (permitted values: 1..128) (for certain products this module supports only 32 channels)
    - num_input_channels (permitted values: 1..128) (for certain products this module supports only 32 channels)
    - num_output_channels entries of output channel mapping (permitted values:
      1 to 128) (for certain products it is 1 to 32)
    - num_input_channels entries of input channel mapping (permitted values:
      1 to 128) (for certain products it is 1 to 32)
    - num_output_channels (rows) * num_input_channels (columns) entries of
      channel mixer weighting coefficients in Q14 format (permitted values:
      0 to 16384) @lb{2}
      For example: @vertspace{-3}
       - Unity (1)      - 0x4000 -> 16384 @lstsp2
       - 3 dB (0.707)   - 0x2D44 -> 11588 @lstsp2
       - 6 dB (0.5)     - 0x2000 -> 8192 @lstsp2
       - 0              - 0x0    -> 0
    - Optional 16-bit zero padding if the entire combination of 1, 2, 3, 4,
      and 5 is not a multiple of 32 bits. @vertspace{-3}
        - Padding is required so the entire payload is aligned to 32&nbsp;bits
          (permitted value: 0) @lstsp2
        - Number of coefficients

    @subhead4{Rules for using Set parameters}
    -# Through a single Set parameter, a client can configure any number of
       coefficient sets. For example, the client can configure eight set
       parameters as follows:
        - num_coeff_tbls      = 8 @lstsp1
        - num_output_channels = 2 @lb{0.5}
          num_input_channels  = 2 @lb{0.5}
          out_chmap[]         = FL, FR @lb{0.5}
          in_chmap[]          = FL, FR @lb{0.5}
          coeff[0] set #0 [2][2] @lstsp1
        - num_output_channels = 2 @lb{0.5}
          num_input_channels  = 5 @lb{0.5}
          out_chmap[]         = 2, FL, FR @lb{0.5}
          in_chmap[]          =  L, FR, FC, LS, RS @lb{0.5}
          coeff[0] set #1 [2][5] @lstsp1
        - .. @lstsp1
        - num_output_channels = 1 @lb{0.5}
          num_input_channels  = 2 @lb{0.5}
          out_chmap[]         = FL @lb{0.5}
          in_chmap[]          = FL, FR @lb{0.5}
          coeff[0] set #7 [1][2] @lstsp1
    -# When the client sends a new coefficient mapping, the previous values are
       overwritten.
    -# If multiple rows contain duplicate entries, the higher index rule (which
       is to be set later) is selected and applied, if applicable.
    -# If the client does not explicitly provide the rule, the module reverts
       to default coefficients (built-in coefficients).
        - For example, if the input or output media types do not match the Set
          parameter rule provided. @lstsp1
        - The default coefficients are based on the Recommendation ITU-R BS.775
          specification.
*/
#define PARAM_ID_CHMIXER_COEFF 0x0800101F

/*# @h2xmlp_parameter   {"PARAM_ID_CHMIXER_COEFF", PARAM_ID_CHMIXER_COEFF}
    @h2xmlp_description {ID for the parameter that configures the channel
                         mixer weighting coefficients. For more details,
                         including rules for using Set parameters, see AudioReach Signal Processing Framework (SPF) API Reference.}
    @h2xmlp_toolPolicy  {Calibration} */

/** @ingroup ar_spf_mod_chmixer_mod
    Payload for #PARAM_ID_CHMIXER_COEFF.
*/
#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
struct param_id_chmixer_coeff_t
{
   uint32_t num_coeff_tbls;
   /**< Number of coefficient tables in the chmixer_coeff_tbl array. */

   /*#< @h2xmle_description {Number of coefficient tables in the array.}
        @h2xmle_range       {0..255}
        @h2xmle_default     {0} */

   chmixer_coeff_t chmixer_coeff_tbl[0];
   /**< Array of channel mixer coefficient tables (of size num_coeff_tbls). */

   /*#< @h2xmle_description       {Array of channel mixer coefficient tables
                                   (of size num_coeff_tbls).}
        @h2xmle_variableArraySize {num_coeff_tbls} */
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;
typedef struct param_id_chmixer_coeff_t param_id_chmixer_coeff_t;


#endif //CHMIXER_COMMON_API
