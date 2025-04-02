#ifndef MBDRC_API_H
#define MBDRC_API_H
/*==============================================================================
  @file mbdrc_api.h
  @brief This file contains MBDRC parameters
==============================================================================*/

/* =========================================================================
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause
   ========================================================================== */
 
/*------------------------------------------------------------------------
 * Include files
 * -----------------------------------------------------------------------*/
#include "module_cmn_api.h"

/**
     @h2xml_title1          {Multiband Dynamic Range Control (MBDRC) API}
     @h2xml_title_agile_rev  {Multiband Dynamic Range Control (MBDRC) API}
     @h2xml_title_date      {October 10, 2018}
  */
/**
   @h2xmlx_xmlNumberFormat {int}
*/

/*==============================================================================
   Constants
==============================================================================*/
#define CAPI_IIR_MBDRC_MAX_PORT          1 

#define CAPI_IIR_MBDRC_STACK_SIZE        4096

#define IIR_MAX_COEFFS_PER_BAND             10

#define IIR_MBDRC_MAX_BANDS                 10

#define MODULE_ID_IIR_MBDRC                                    0x07001017

#ifdef PROD_SPECIFIC_MAX_CH
#define CAPI_IIR_MBDRC_MAX_CHANNELS 128
#else
#define CAPI_IIR_MBDRC_MAX_CHANNELS 32
#endif        

/** 
    @h2xmlm_module       {"MODULE_ID_IIR_MBDRC", 
                          MODULE_ID_IIR_MBDRC}
    @h2xmlm_displayName  {"IIR MBDRC"}
    @h2xmlm_modSearchKeys{drc, Audio}
    @h2xmlm_toolPolicy   {Calibration}
    @h2xmlm_description  {Multiband Dynamic Range Control Module \n

  -  This module supports the following parameter IDs: \n
  - #PARAM_ID_IIR_MBDRC_CONFIG_PARAMS \n
  - #PARAM_ID_IIR_MBDRC_FILTER_XOVER_FREQS \n
  - #PARAM_ID_MODULE_ENABLE \n

*   Supported Input Media Format: \n
*  - Data Format          : FIXED_POINT \n
*  - fmt_id               : Don't care \n
*  - Sample Rates         : Any \n
*  - Number of channels   : 1 to 128 (for certain products this module supports only 32 channels) \n
*  - Channel type         : 1 to 128 \n
*  - Bits per sample      : 16, 32 \n
*  - Q format             : 15, 27 \n
*  - Interleaving         : de-interleaved unpacked \n
*  - Signed/unsigned      : Signed }

    @h2xmlm_dataMaxInputPorts    {CAPI_IIR_MBDRC_MAX_PORT}
    @h2xmlm_dataInputPorts       {IN=2}
    @h2xmlm_dataMaxOutputPorts   {CAPI_IIR_MBDRC_MAX_PORT}
    @h2xmlm_dataOutputPorts      {OUT=1}
    @h2xmlm_supportedContTypes  {APM_CONTAINER_TYPE_SC, APM_CONTAINER_TYPE_GC}
    @h2xmlm_isOffloadable        {true}
    @h2xmlm_stackSize            {CAPI_IIR_MBDRC_STACK_SIZE}
    @h2xmlm_ToolPolicy           {Calibration}
    @{                   <-- Start of the Module -->
*/

/*  ID of the MBDRC Configuration parameter used by MODULE_ID_IIR_MBDRC.*/
#define PARAM_ID_IIR_MBDRC_CONFIG_PARAMS                         0x08001028

#include "spf_begin_pack.h"
/** @h2xmlp_subStruct */
struct subband_drc_config_params_t
{
   
   int16_t drc_mode;
   /**< @h2xmle_description {Specifies whether DRC mode is bypassed for subbands.}
        @h2xmle_rangeList   {"Disable"=0; "Enable"=1}
      @h2xmle_default     {1} */
      
   int16_t drc_linked_flag;
   /**< @h2xmle_description {Specifies whether all stereo channels have the same applied dynamics
        or if they process their dynamics independently.}
        @h2xmle_rangeList   {"Not linked,channels process the dynamics independently" = 0;
                             "Linked,channels have the same applied dynamics" = 1}
          @h2xmle_default     {1}                      */

   int16_t drc_down_sample_level;
   /**< @h2xmle_description {DRC down sample level.}
        @h2xmle_default     {1}
      @h2xmle_range    {1..16}*/

   uint16_t drc_rms_time_avg_const;
   /**< @h2xmle_description {RMS signal energy time-averaging constant.}
      @h2xmle_default     {298}
    @h2xmle_range       {0..65535}
      @h2xmle_dataFormat  {Q16}    */

   uint16_t drc_makeup_gain;
   /**< @h2xmle_description {DRC makeup gain in decibels.}
        @h2xmle_default     {4096}
        @h2xmle_range       {258..64917}
      @h2xmle_dataFormat  {Q12}    */

   /* Down expander settings */

   int16_t down_expdr_threshold;
   /**< @h2xmle_description {Down expander threshold.}
                             Its value must be: (<down_cmpsr_threshold)}
        @h2xmle_default     {3877}
        @h2xmle_range       {0..11560}
        @h2xmle_dataFormat  {Q7}  */

   int16_t down_expdr_slope;
   /**< @h2xmle_description {Down expander slope.}
        @h2xmle_default     {65434}
        @h2xmle_range       {-32768..0}
        @h2xmle_dataFormat  {Q8}  */
      
   uint16_t down_expdr_hysteresis;
   /**< @h2xmle_description {Down expander hysteresis constant.}
        @h2xmle_default     {18855}
        @h2xmle_range       {1..32690}
        @h2xmle_dataFormat  {Q14}  */

   uint32_t down_expdr_attack;
   /**< @h2xmle_description {Down expander attack constant.}
        @h2xmle_default     {15690611}
        @h2xmle_range       {0..2147483648}
        @h2xmle_dataFormat  {Q31}  */

   uint32_t down_expdr_release;
   /**< @h2xmle_description {Down expander release constant.}
        @h2xmle_default     {39011832}
        @h2xmle_range       {0..2147483648}
        @h2xmle_dataFormat  {Q31}  */
      
   int32_t down_expdr_min_gain_db;
   /**< @h2xmle_description {Down expander minimum gain.}
      @h2xmle_default     {-50331648}
        @h2xmle_range       {-805306368..0}
        @h2xmle_dataFormat  {Q23}  */

   /* Up compressor settings */

   int16_t up_cmpsr_threshold;
   /**< @h2xmle_description {Up compressor threshold.
                             Its value must be:
                             (&gt;down_expdr_threshold) &amp;&amp; (&lt;down_cmpsr_threshold)}
        @h2xmle_default     {3877}
        @h2xmle_range       {0..11560}
        @h2xmle_dataFormat  {Q7}  */
      
   uint16_t up_cmpsr_slope;
   /**< @h2xmle_description {Up compressor slope.}
        @h2xmle_range       {0..64880}
      @h2xmle_default     {0}
        @h2xmle_dataFormat  {Q8}  */

   uint32_t up_cmpsr_attack;
   /**< @h2xmle_description {Up compressor attack constant.}
        @h2xmle_default     {7859688}
        @h2xmle_range       {0..2147483648}
        @h2xmle_dataFormat  {Q31}  */

   uint32_t up_cmpsr_release;
   /**< @h2xmle_description {Up compressor release constant.}
        @h2xmle_default     {7859688}
        @h2xmle_range       {0..2147483648}
        @h2xmle_dataFormat  {Q31}  */
      


   uint16_t up_cmpsr_hysteresis;
   /**< @h2xmle_description {Up compressor hysteresis constant.}
        @h2xmle_default     {18855}
        @h2xmle_range       {1..32690}
        @h2xmle_dataFormat  {Q14}  */

   /* Down compressor settings */

   int16_t down_cmpsr_threshold;
   /**< @h2xmle_description {Down compressor threshold.
                             Its value must be: (&gt;up_cmpsr_threshold)}
        @h2xmle_default     {9637}
        @h2xmle_range       {0..11560}
        @h2xmle_dataFormat  {Q7}  */

   uint16_t down_cmpsr_slope;
   /**< @h2xmle_description {Down compressor slope.}
      @h2xmle_default     {62259}
      @h2xmle_range       {0..64880}
        @h2xmle_dataFormat  {Q8}  */
      
   uint16_t down_cmpsr_hysteresis;
   /**< @h2xmle_description {Down compressor hysteresis constant.}
        @h2xmle_default     {18855}
        @h2xmle_range       {1..32690}
        @h2xmle_dataFormat  {Q14}  */
      
   uint32_t down_cmpsr_attack;
   /**< @h2xmle_description {Down compressor attack constant.}
        @h2xmle_default     {77314964}
        @h2xmle_range       {0..2147483648}
        @h2xmle_dataFormat  {Q31}  */

   uint32_t down_cmpsr_release;
   /**< @h2xmle_description {Down compressor release constant.}
        @h2xmle_default     {1574244}
        @h2xmle_range       {0..2147483648}
        @h2xmle_dataFormat  {Q31}  */

      
}
#include "spf_end_pack.h"
;
/* DRC configuration structure for each sub-band of an MBDRC module. */
typedef struct subband_drc_config_params_t subband_drc_config_params_t;


#include "spf_begin_pack.h"
/** @h2xmlp_subStruct */
struct iir_filter_config_params_t
{
   uint32_t num_even_stages; 
   /**< @h2xmle_description {even filter stages;}
         @h2xmle_default     {3}
         @h2xmle_range       {0..3}
          */
   uint32_t num_odd_stages; 
   /**< @h2xmle_description {odd filter stages}
         @h2xmle_default     {2}
         @h2xmle_range       {0..2}
          */

   /* according to allpass biquad filter's transfer function:         */
   /* H_ap(z) = (b0 + b1*z^-1 + z^-2)/(1 + b1*z^-1 + b0*z^-2)         */
   /* only b0 and b1 needs to be saved                            */
   /* Besides, the 3-rd stage of even allpass filter is 1st order IIR */
   /* H_even_3rd(z) = (b0 + z^-1)/(1 + b0*z^-1)                       */
   /* only b0 needs to be saved                              */
   /* iir_coeffs store order is:                            */
   /* [b0_even1 b1_even1 | b0_even2 b1_even2 | b0_even3 b1_even3 | ...*/
   /* ...b0_odd1 b1_odd1 | b0_odd2 b1_odd2]                           */
   /* if num_even_stages are 2 , then b0_even3 = 0 and  b1_even3= 0   */
   int32_t iir_coeffs[IIR_MAX_COEFFS_PER_BAND];
   /**< @h2xmle_description {IIR filter coefficients. \n
                            -# IIR_MAX_COEFFS_PER_BAND = 10}
          */

}
#include "spf_end_pack.h"
;  

typedef struct iir_filter_config_params_t iir_filter_config_params_t;

#include "spf_begin_pack.h"
/** @h2xmlp_subStruct */
struct limiter_config_param_t
{
   
    int32_t limiter_threshold;
   /**< @h2xmle_description  {Threshold in decibels for the limiter output.
                     \n For 16bit use case: limiter threshold is [-96dB  0dB].
                     \n For 24bit use case: limiter threshold is [-162dB  24dB].
                     \n For true 32bit use case: limiter threshold is [-162dB  0dB].
                     \n If a value out of this range is configured, it will be automatically limited to the upper bound or low bound in the DSP processing.}
        @h2xmle_default      {93945856}
      @h2xmle_range        {0..2127207634}
        @h2xmle_dataFormat   {Q27} */

   int32_t limiter_makeup_gain;
   /**< @h2xmle_description  {Makeup gain in decibels for the limiter output.}
        @h2xmle_default      {256}
      @h2xmle_range        {1..32228}
        @h2xmle_dataFormat   {Q8}  */

   int32_t limiter_gc;
   /**< @h2xmle_description  {Limiter gain recovery coefficient.}
        @h2xmle_range        {0..32767}
        @h2xmle_default      {32440}
        @h2xmle_dataFormat   {Q15}  */

   int32_t limiter_max_wait;
   /**< @h2xmle_description  {Maximum limiter waiting time in seconds(Q15 format)}
        @h2xmle_range        {0..328}
        @h2xmle_default      {82}
        @h2xmle_dataFormat   {Q15}  */
      
   uint32_t gain_attack;      
   /**< @h2xmle_description  {Limiter gain attack time in seconds (Q31 format)}
        @h2xmle_default      {188099735}
      @h2xmle_range        {0..2147483648}
        @h2xmle_dataFormat   {Q31}  */
      
   uint32_t gain_release;     
   /**< @h2xmle_description  {Limiter gain release time in second (Q31 format)}
        @h2xmle_default      {32559427}
      @h2xmle_range        {0..2147483648}
        @h2xmle_dataFormat   {Q31}  */
      
   uint32_t attack_coef;     
   /**< @h2xmle_description  {Limiter gain attack time speed coef}
        @h2xmle_default      {32768}
      @h2xmle_range        {32768..3276800}
        @h2xmle_dataFormat   {Q15}  */
      
   uint32_t release_coef;     
   /**< @h2xmle_description  {Limiter gain release time speed coef}
        @h2xmle_default      {32768}
      @h2xmle_range        {32768..3276800}
        @h2xmle_dataFormat   {Q15}  */
      
   int32_t hard_threshold;   
   /**< @h2xmle_description  {Hard Threshold in decibels for the limiter output.
                     \n For 16bit use case: limiter hard threshold is [-96dB  0dB].
                     \n For 24bit use case: limiter hard threshold is [-162dB  24dB].
                     \n For true 32bit use case: limiter hard threshold is [-162dB  0dB].
                     \n If a value out of this range is configured, it will be automatically limited to the upper bound or low bound in the DSP processing.}
        @h2xmle_default      {93945856}
      @h2xmle_range        {0..2127207634}
        @h2xmle_dataFormat   {Q27} */
}
#include "spf_end_pack.h"
;  

typedef struct limiter_config_param_t limiter_config_param_t;

/* Structure for the configuration parameters for an MBDRC module. */
#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
/** @h2xmlp_subStruct */
struct iir_mbdrc_per_ch_config_params_t
{
   uint32_t channel_mask_lsb;
   /**< @h2xmle_description  {Lower 32 bits of the channel mask. Each bit corresponds to channel map 
                             from 1 (PCM_CHANNEL_L) to 31 (PCM_CHANNEL_LW). 
                             Bit 0 is reserved and must be set to zero.\n
                             -#A set bit indicates that the following config is applied to corresponding channel-maps. 
                             Bit position of the channel-map is obtained by left shifting (1 (left shift) Channel_map) 
                             }
      @h2xmle_default        {0xFFFFFFFE} */
   

   uint32_t channel_mask_msb;
   /**< @h2xmle_description  {Upper 32 bits of the channel mask. Each bit corresponds to channel map 
                             from 32 (PCM_CHANNEL_RW) to 63 (PCM_CUSTOM_CHANNEL_MAP_16).\n
                             -#A set bit indicates that the following config is applied to corresponding channel-maps. 
                             Bit position of the channel-map is obtained by left shifting (1 (left shift) (Channel_map - 32)) 
                             }
      @h2xmle_default        {0xFFFFFFFF}  */
                      
                      
   limiter_config_param_t limiter;
   /**< @h2xmle_description  {...}
        @h2xmle_default      {0}
      @h2xmlx_expandStructs {false} */ 

   subband_drc_config_params_t subband_drc[0];
   /**< @h2xmle_description  { Following this structure is the sub-band payload,
                               This sub-band structure must be repeated for each band.
                               After this DRC structure is configured for valid bands, the next MBDRC
                               setparams expects the sequence of sub-band MBDRC filter coefficients (the
                               length depends on the number of bands) plus the mute flag for that band
                               plus uint16 padding.
                               }
        @h2xmle_variableArraySize {"iir_mbdrc_config_params_t::num_bands"}   
        */  
#ifdef __H2XML__
   iir_filter_config_params_t iir_filter[0];
   /**< @h2xmle_description  {...}
        @h2xmle_default      {0}
        @h2xmle_variableArraySize {"(iir_mbdrc_config_params_t::num_bands)- 1"} */ 

   int32_t mute_flag[0];
   /**< @h2xmle_description  {...}
        @h2xmle_default      {0}
        @h2xmle_variableArraySize {"iir_mbdrc_config_params_t::num_bands"} */ 
#endif //H@XML        
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;

typedef struct iir_mbdrc_per_ch_config_params_t iir_mbdrc_per_ch_config_params_t;

/** @h2xmlp_parameter   {"PARAM_ID_IIR_MBDRC_CONFIG_PARAMS", PARAM_ID_IIR_MBDRC_CONFIG_PARAMS} 
    @h2xmlp_description {Used to configure a device}
    @h2xmlp_toolPolicy  {NO_SUPPORT}  */

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"

struct iir_mbdrc_config_params_t
{
   uint32_t num_bands;
   /**< @h2xmle_description  {Number of bands.}
        @h2xmle_default      {1}
        @h2xmle_range        {1..10}  */  
      
      
   uint32_t limiter_mode;
   /**< @h2xmle_description {Specifies whether Limiter mode is bypassed for subbands.}
      @h2xmle_default      {1}
        @h2xmle_rangeList   {"Disable"=0; "Enable"=1} */
      
   uint32_t limiter_delay;
   /**< @h2xmle_description  {Limiter delay in seconds.
                              Range: 0 to 20 ms. Default 8 ms}
        @h2xmle_range        {0..655}
        @h2xmle_default      {262}
        @h2xmle_dataFormat   {Q15}  */ 
      
   uint32_t limiter_history_winlen;
   /**< @h2xmle_description  {Length of history window.
                             Range: 0 to 100 ms. Default 8 ms }
        @h2xmle_range        {0..3276}
        @h2xmle_default      {262}
        @h2xmle_dataFormat   {Q15}  */
      
      
   uint32_t num_config;
   /**< @h2xmle_description  {Specifies the different sets of mbdrc configurations.}
        @h2xmle_range        {1..63}
        @h2xmle_default      {1} */
      
   uint32_t drc_delay[0];   
   /**< @h2xmle_description {DRC delay in samples.
                             range 0 to 100 ms. default 1ms at 48KHz}
        @h2xmle_range       {0..38400}
          @h2xmle_variableArraySize {num_bands}
        @h2xmle_default      {48}*/
#ifdef __H2XML__
   iir_mbdrc_per_ch_config_params_t config_data[0];
   /**< @h2xmle_description {Specifies the different sets of mbdrc configurations}
        @h2xmle_variableArraySize {num_config}
        @h2xmle_default      {0} */ 
#endif //H@XML   
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;

typedef struct iir_mbdrc_config_params_t iir_mbdrc_config_params_t;

/** ID of the MBDRC Configuration parameter used by #MODULE_ID_IIR_MBDRC.*/
#define PARAM_ID_IIR_MBDRC_FILTER_XOVER_FREQS             0x08001029

/* Structure for the MBDRC filter cross over frequencies parameter
 * for an MBDRC module. */
#include "spf_begin_pack.h"
/* Payload of the PARAM_ID_IIR_MBDRC_FILTER_XOVER_FREQS parameter used
 by the MBDRC module.
 */
 /** @h2xmlp_subStruct */
struct iir_mbdrc_per_ch_filter_xover_freqs_t
{
   
   uint32_t channel_mask_lsb;
   /**< @h2xmle_description  {Lower 32 bits of the channel mask. Each bit corresponds to channel map 
                             from 1 (PCM_CHANNEL_L) to 31 (PCM_CHANNEL_LW). 
                             Bit 0 is reserved and must be set to zero.\n
                             -#A set bit indicates that the following config is applied to corresponding channel-maps. 
                             Bit position of the channel-map is obtained by left shifting (1 (left shift) Channel_map) 
                             }
      @h2xmle_default        {0xFFFFFFFE} */
   

   uint32_t channel_mask_msb;
   /**< @h2xmle_description  {Upper 32 bits of the channel mask. Each bit corresponds to channel map 
                             from 32 (PCM_CHANNEL_RW) to 63 (PCM_CUSTOM_CHANNEL_MAP_16).\n
                             -#A set bit indicates that the following config is applied to corresponding channel-maps. 
                             Bit position of the channel-map is obtained by left shifting (1 (left shift) (Channel_map - 32)) 
                             }
      @h2xmle_default        {0xFFFFFFFF}  */
   
   uint32_t iir_mbdrc_cross_over_freqs[IIR_MBDRC_MAX_BANDS-1];
   /**< @h2xmle_description  {Array of filter crossover frequencies. Based on Band number n,
                              filter_xover_freqs[MBDRC_MAX_BANDS-1] has (n-1) 
                              crossover frequencies and the rest(if any) are ignored.
                              } */
}
#include "spf_end_pack.h"
;
typedef struct iir_mbdrc_per_ch_filter_xover_freqs_t iir_mbdrc_per_ch_filter_xover_freqs_t;

/** @h2xmlp_parameter   {"PARAM_ID_IIR_MBDRC_FILTER_XOVER_FREQS", PARAM_ID_IIR_MBDRC_FILTER_XOVER_FREQS} 
    @h2xmlp_description {Used to configure a device}
    @h2xmlp_toolPolicy  {NO_SUPPORT}  */
#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
struct iir_mbdrc_filter_xover_freqs_t
{
   uint32_t num_config;
   /**< @h2xmle_description  {Specifies the different sets of mbdrc configurations.}
        @h2xmle_range        {1..63}
        @h2xmle_default      {1} */
      
   iir_mbdrc_per_ch_filter_xover_freqs_t iir_mbdrc_per_ch_cross_over_freqs[0];
   /**< @h2xmle_description {Specifies the different sets of mbdrc configurations}
        @h2xmle_variableArraySize {num_config}
        @h2xmle_default      {1} */
      
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;
typedef struct iir_mbdrc_filter_xover_freqs_t iir_mbdrc_filter_xover_freqs_t;

/*  ID of the MBDRC Configuration parameter used by MODULE_ID_IIR_MBDRC.*/
#define PARAM_ID_IIR_MBDRC_CONFIG_PARAMS_V2 0x8001A86

/* Structure for the configuration parameters for an MBDRC module. */
#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
/** @h2xmlp_subStruct */
struct iir_mbdrc_per_ch_config_params_v2_t
{
   uint32_t channel_type_group_mask;
   /**< @h2xmle_description  {Indicates the mask for channel_type_mask_list array.
                             Each bit in channel_type_group_mask corresponds to a channel group.
                             Read as
                             Bit 0 corresponds to channel group 1, which includes channel map for channels 1-31.
                             Bit 1 corresponds to channel group 2, which includes channel map for channels 32-63.
                             Bit 2 corresponds to channel group 3, which includes channel map for channels 64-95.
                             Bit 3 corresponds to channel group 4, which includes channel map for channels 96-127.
                             Bit 4 corresponds to channel group 5, which includes channel map for channels 128-159.
                             A set bit (1) in channel_type_group_mask indicates that the channels in that channel group are configured.
                             }
    @h2xmle_range            {0..31}
    @h2xmle_default          {0x00000003} */

   uint32_t channel_type_mask_list[0];
   /**< @h2xmle_description  {An array used to configure the channels for different channel groups. The array size depends on the number of
                             bits set in channel_type_group_mask.\n
                             For group 1, each bit of channel_type_mask_list corresponds to channel map from 1 (PCM_CHANNEL_L) to 31 (PCM_CHANNEL_LW).\n
                             Bit 0 of group 1 channel_type_mask_list is reserved and must always be set to zero.\n
                             For any other group, each bit of channel_type_mask_list corresponds to channel map from [32(group_no -1) to 32(group_no)-1].\n
                             Bit position of the channel-map for channel_type_mask_list of defined group is obtained by left shifting (1 (left shift) Channel_map%32
                             }
    @h2xmle_variableArraySizeFunction {GET_SET_BITS_COUNT, channel_type_group_mask}
    @h2xmle_copySrcList      {channel_mask_lsb, channel_mask_msb}
    @h2xmle_defaultList      {0xfffffffe, 0xffffffff}  */

   limiter_config_param_t limiter;  
   /**< @h2xmle_description  {limiter payload}
        @h2xmle_copySrc      {limiter}
        @h2xmle_default      {0}
      @h2xmlx_expandStructs {false} */

   subband_drc_config_params_t subband_drc[0]; 
   /**< @h2xmle_description  { Following this structure is the sub-band payload,
                               This sub-band structure must be repeated for each band.
                               After this DRC structure is configured for valid bands, the next MBDRC
                               setparams expects the sequence of sub-band MBDRC filter coefficients (the
                               length depends on the number of bands) plus the mute flag for that band
                               plus uint16 padding.
                               }
        @h2xmle_copySrc      {subband_drc}
        @h2xmle_variableArraySize {"iir_mbdrc_config_params_v2_t::num_bands"}
        */
#ifdef __H2XML__
   iir_filter_config_params_t iir_filter[0]; 
   /**< @h2xmle_description  {sub-band payload, this sub-band structure is repeated for each band}
        @h2xmle_copySrc      {iir_filter}
        @h2xmle_default      {0}
        @h2xmle_variableArraySize {"(iir_mbdrc_config_params_v2_t::num_bands)- 1"} */

   int32_t mute_flag[0];
   /**< @h2xmle_description  {Species mute flag for the band}
        @h2xmle_copySrc      {mute_flag}
        @h2xmle_default      {0}
        @h2xmle_variableArraySize {"iir_mbdrc_config_params_v2_t::num_bands"} */
#endif //H@XML
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;

typedef struct iir_mbdrc_per_ch_config_params_v2_t iir_mbdrc_per_ch_config_params_v2_t;

/** @h2xmlp_parameter   {"PARAM_ID_IIR_MBDRC_CONFIG_PARAMS_V2", PARAM_ID_IIR_MBDRC_CONFIG_PARAMS_V2}
    @h2xmlp_copySrc     {0x08001028}
    @h2xmlp_description {Used to configure a device}
    @h2xmlp_toolPolicy  {RTC, Calibration}  */
#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"

struct iir_mbdrc_config_params_v2_t
{
   uint32_t num_bands;
   /**< @h2xmle_description  {Number of bands.}
        @h2xmle_copySrc      {num_bands}
        @h2xmle_default      {1}
        @h2xmle_range        {1..10}  */


   uint32_t limiter_mode;
   /**< @h2xmle_description {Specifies whether Limiter mode is bypassed for subbands.}
        @h2xmle_copySrc      {limiter_mode}
      @h2xmle_default      {1}
        @h2xmle_rangeList   {"Disable"=0; "Enable"=1} */

   uint32_t limiter_delay;
   /**< @h2xmle_description  {Limiter delay in seconds.
                              Range: 0 to 20 ms. Default 8 ms}
        @h2xmle_copySrc      {limiter_delay}
        @h2xmle_range        {0..655}
        @h2xmle_default      {262}
        @h2xmle_dataFormat   {Q15}  */

   uint32_t limiter_history_winlen;
   /**< @h2xmle_description  {Length of history window.
                             Range: 0 to 100 ms. Default 8 ms }
        @h2xmle_copySrc      {limiter_history_winlen}
        @h2xmle_range        {0..3276}
        @h2xmle_default      {262}
        @h2xmle_dataFormat   {Q15}  */


   uint32_t num_config;
   /**< @h2xmle_description  {Specifies the different sets of mbdrc configurations.}
        @h2xmle_copySrc      {num_config}
        @h2xmle_range        {1..MODULE_CMN_MAX_CHANNEL}
        @h2xmle_default      {1} */

   uint32_t drc_delay[0];
   /**< @h2xmle_description {DRC delay in samples.
                             range 0 to 100 ms. default 1ms at 48KHz}
        @h2xmle_copySrc     {drc_delay}
        @h2xmle_range       {0..38400}
        @h2xmle_variableArraySize {num_bands}
        @h2xmle_default      {48}*/

   iir_mbdrc_per_ch_config_params_v2_t config_data[0];
   /**< @h2xmle_description {Specifies the different sets of mbdrc configurations}
        @h2xmle_variableArraySize {num_config}
        @h2xmle_default      {0} */

}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;

typedef struct iir_mbdrc_config_params_t iir_mbdrc_config_params_t;

/** ID of the MBDRC Configuration parameter used by #MODULE_ID_IIR_MBDRC.*/
#define PARAM_ID_IIR_MBDRC_FILTER_XOVER_FREQS_V2 0x8001A87

/* Structure for the MBDRC filter cross over frequencies parameter
 * for an MBDRC module. */
#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
/* Payload of the PARAM_ID_IIR_MBDRC_FILTER_XOVER_FREQS_V2 parameter used
 by the MBDRC module.
 */
 /** @h2xmlp_subStruct */
struct iir_mbdrc_per_ch_filter_xover_freqs_v2_t
{

   uint32_t channel_type_group_mask;
   /**< @h2xmle_description  {Indicates the mask for channel_type_mask_list array.
                             Each bit in channel_type_group_mask corresponds to a channel group.
                             Read as
                             Bit 0 corresponds to channel group 1, which includes channel map for channels 1-31.
                             Bit 1 corresponds to channel group 2, which includes channel map for channels 32-63.
                             Bit 2 corresponds to channel group 3, which includes channel map for channels 64-95.
                             Bit 3 corresponds to channel group 4, which includes channel map for channels 96-127.
                             Bit 4 corresponds to channel group 5, which includes channel map for channels 128-159.
                             A set bit (1) in channel_type_group_mask indicates that the channels in that channel group are configured.
                             }
    @h2xmle_range            {0..31}
    @h2xmle_default          {0x00000003} */


   uint32_t channel_type_mask_list[0];
   /**< @h2xmle_description  {An array used to configure the channels for different channel groups. The array size depends on the number of
                             bits set in channel_type_group_mask.\n
                             For group 1, each bit of channel_type_mask_list corresponds to channel map from 1 (PCM_CHANNEL_L) to 31 (PCM_CHANNEL_LW).\n
                             Bit 0 of group 1 channel_type_mask_list is reserved and must always be set to zero.\n
                             For any other group, each bit of channel_type_mask_list corresponds to channel map from [32(group_no -1) to 32(group_no)-1].\n
                             Bit position of the channel-map for channel_type_mask_list of defined group is obtained by left shifting (1 (left shift) Channel_map%32
                             }
    @h2xmle_variableArraySizeFunction {GET_SET_BITS_COUNT, channel_type_group_mask}
    @h2xmle_copySrcList      {channel_mask_lsb, channel_mask_msb}
    @h2xmle_defaultList      {0xfffffffe, 0xffffffff}  */

   uint32_t iir_mbdrc_cross_over_freqs[IIR_MBDRC_MAX_BANDS-1];
   /**< @h2xmle_description  {Array of filter crossover frequencies. Based on Band number n,
                              filter_xover_freqs[MBDRC_MAX_BANDS-1] has (n-1)
                              crossover frequencies and the rest(if any) are ignored.
                              }  
        @h2xmle_copySrc      {iir_mbdrc_cross_over_freqs}*/
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;
typedef struct iir_mbdrc_per_ch_filter_xover_freqs_v2_t iir_mbdrc_per_ch_filter_xover_freqs_v2_t;

/** @h2xmlp_parameter   {"PARAM_ID_IIR_MBDRC_FILTER_XOVER_FREQS_V2", PARAM_ID_IIR_MBDRC_FILTER_XOVER_FREQS_V2}
    @h2xmlp_copySrc     {0x08001029}
    @h2xmlp_description {Used to configure a device} 
    @h2xmlp_toolPolicy  {RTC, Calibration}  */
#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
struct iir_mbdrc_filter_xover_freqs_v2_t
{
   uint32_t num_config;
   /**< @h2xmle_description  {Specifies the different sets of mbdrc configurations.}
        @h2xmle_copySrc      {num_config}
        @h2xmle_range        {1..MODULE_CMN_MAX_CHANNEL}
        @h2xmle_default      {1} */

#ifdef __H2XML__
   iir_mbdrc_per_ch_filter_xover_freqs_v2_t iir_mbdrc_per_ch_cross_over_freqs[0];
   /**< @h2xmle_description {Specifies the different sets of mbdrc configurations}
        @h2xmle_variableArraySize {num_config}
        @h2xmle_default      {1} */
#endif
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;
typedef struct iir_mbdrc_filter_xover_freqs_v2_t iir_mbdrc_filter_xover_freqs_v2_t;


/**
  @h2xml_Select         {param_id_module_enable_t}
  @h2xmlm_InsertParameter
*/

/** @}                   <-- End of the Module -->*/

#endif
