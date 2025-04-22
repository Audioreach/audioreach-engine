#ifndef API_MSIIR_H
#define API_MSIIR_H
/*==============================================================================
  @file api_msiir.h
  @brief This file contains API structures and param for the IIR Filter Module
==============================================================================*/

/*=======================================================================
    Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
    SPDX-License-Identifier: BSD-3-Clause-Clear
=========================================================================*/

 /*------------------------------------------------------------------------
 * Include files
 * -----------------------------------------------------------------------*/
#include "module_cmn_api.h"

/*==============================================================================
   Constants
==============================================================================*/
// stack size measured for 8 channel 5 stage
#define CAPI_MSIIR_STACK_SIZE            3072

#define CAPI_MSIIR_MAX_IN_PORTS          1

#define CAPI_MSIIR_MAX_OUT_PORTS         1

/** Maximum number of channels for the multichannel IIR tuning filter. */
#ifdef PROD_SPECIFIC_MAX_CH
#define IIR_TUNING_FILTER_MAX_CHANNELS_V2 128
#else
#define IIR_TUNING_FILTER_MAX_CHANNELS_V2 32
#endif

#define MODULE_ID_MSIIR                             0x07001014
/**
     @h2xml_title1          {Multi-Stage IIR (MSIIR) Filter Module API}
     @h2xml_title_agile_rev {Multi-Stage IIR (MSIIR) Filter Module API}
     @h2xml_title_date      {July 31, 2018}
  */

/** @h2xmlm_module       {"MODULE_ID_MSIIR", MODULE_ID_MSIIR}
    @h2xmlm_displayName    {"MS-IIR Filter"}
    @h2xmlm_modSearchKeys {filter, Audio}
  @h2xmlm_description  {ID of the Multi-Channel Multi-Stage IIR Tuning Filter module.\n

    - This module supports the following parameter IDs\n
     - #PARAM_ID_MSIIR_TUNING_FILTER_ENABLE\n
     - #PARAM_ID_MSIIR_TUNING_FILTER_PREGAIN\n
     - #PARAM_ID_MSIIR_TUNING_FILTER_CONFIG_PARAMS\n
     - #PARAM_ID_MODULE_ENABLE\n
*
*  - Supported Input Media Format:\n
*  - Data Format          : FIXED_POINT\n
*  - fmt_id               : Don't care\n
*  - Sample Rates         : Any\n
*  - Number of channels   : 1 to 128 (for certain products this module supports only 32 channels)\n
*  - Channel type         : 1 to 128\n
*  - Bits per sample      : 16, 32\n
*  - Q format             : 15, 27\n
*  - Interleaving         : de-interleaved unpacked\n
*  - Signed/unsigned      : Signed }

    @h2xmlm_dataMaxInputPorts    {CAPI_MSIIR_MAX_IN_PORTS}
    @h2xmlm_dataInputPorts       {IN=2}
    @h2xmlm_dataMaxOutputPorts   {CAPI_MSIIR_MAX_OUT_PORTS}
    @h2xmlm_dataOutputPorts      {OUT=1}
    @h2xmlm_supportedContTypes  {APM_CONTAINER_TYPE_SC, APM_CONTAINER_TYPE_GC}
    @h2xmlm_isOffloadable        {true}
    @h2xmlm_stackSize            {CAPI_MSIIR_STACK_SIZE}
    @h2xmlm_ToolPolicy           {Calibration}

    @{                   <-- Start of the Module -->
 */


/* ID of the Multichannel IIR Tuning Filter Enable parameters used by
    MODULE_ID_MSIIR.
 */
#define PARAM_ID_MSIIR_TUNING_FILTER_ENABLE        0x08001020

/* Structure for holding one channel type - IIR enable pair. */

/*
 This structure immediately follows the param_id_msiir_enable_t
 structure.
 */
/** @h2xmlp_subStruct */
#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"

struct param_id_msiir_ch_enable_t
{
   uint32_t channel_mask_lsb;
   /**< @h2xmle_description  {Lower 32 bits of the channel mask. Each bit corresponds to channel map
                             from 1 (PCM_CHANNEL_L) to 31 (PCM_CHANNEL_LW).
                             Bit 0 is reserved and must be set to zero.\n
                             -#A set bit indicates that the filter corresponding to the set channel-maps are enabled.
                             Bit position of the channel-map is obtained by left shifting (1 (left shift) Channel_map)
                             }
    @h2xmle_default          {0xFFFFFFFE} */


   uint32_t channel_mask_msb;
   /**< @h2xmle_description  {Upper 32 bits of the channel mask. Each bit corresponds to channel map
                             from 32 (PCM_CHANNEL_RW) to 63 (PCM_CUSTOM_CHANNEL_MAP_16).\n
                             -#A set bit indicates that the filter corresponding to the set channel-maps are enabled.
                             Bit position of the channel-map is obtained by left shifting (1 (left shift) (Channel_map - 32))
                             }
    @h2xmle_default         {0xFFFFFFFF}  */

   uint32_t enable_flag;
   /**< @h2xmle_description  {Specifies whether the above channels are enabled.}
        @h2xmle_rangeList    {"Disable"=0;
                              "Enable"=1}
        @h2xmle_default      {0} */
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;

typedef struct param_id_msiir_ch_enable_t param_id_msiir_ch_enable_t;

/** @h2xmlp_parameter   {"PARAM_ID_MSIIR_TUNING_FILTER_ENABLE", PARAM_ID_MSIIR_TUNING_FILTER_ENABLE}
    @h2xmlp_description {ID of the Multichannel IIR Tuning Filter Enable parameters used by
    MODULE_ID_MSIIR.\n Payload of the PARAM_ID_MSIIR_TUNING_FILTER_ENABLE
 channel type/IIR enable pairs used by the Multiple Channel IIR Tuning
 Filter module.\n
 This structure immediately follows the param_id_msiir_enable_t
 structure.\n}
    @h2xmlp_toolPolicy  {NO_SUPPORT} 
 @h2xmlx_expandStructs  {false}
   */
#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
/* Payload of the PARAM_ID_MSIIR_TUNING_FILTER_ENABLE
 parameters used by the Multiple Channel IIR Tuning Filter module.
 */
struct param_id_msiir_enable_t
{
   uint32_t num_config;
   /**< @h2xmle_description  {Number of configurations for which enable flags are provided.\n}
        @h2xmle_range    {1..63}
        @h2xmle_default {1}  */

   param_id_msiir_ch_enable_t enable_flag_settings[0];
   /**< @h2xmle_description  {multi_ch_iir_cfg.}
        @h2xmle_variableArraySize {num_config}
        @h2xmle_default {0} */
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;

/* Structure for the multichannel IIR enable command */
typedef struct param_id_msiir_enable_t param_id_msiir_enable_t;


/* ID of the Multiple Channel IIR Tuning Filter Pregain parameters used by
 */
#define PARAM_ID_MSIIR_TUNING_FILTER_PREGAIN 0x08001021

/* Structure for holding one channel type - IIR pregain pair. */

/** @h2xmlp_subStruct */
#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"

struct param_id_msiir_ch_pregain_t
{
   uint32_t channel_mask_lsb;
   /**< @h2xmle_description  {Lower 32 bits of the channel mask. Each bit corresponds to channel map
                             from 1 (PCM_CHANNEL_L) to 31 (PCM_CHANNEL_LW).
                             Bit 0 is reserved and must be set to zero.\n
                             -#A set bit indicates that the pregain is set on the corresponding channel-maps.
                             Bit position of the channel-map is obtained by left shifting (1 (left shift) Channel_map)
                             }
    @h2xmle_default          {0xFFFFFFFE} */


   uint32_t channel_mask_msb;
   /**< @h2xmle_description  {Upper 32 bits of the channel mask. Each bit corresponds to channel map
                             from 32 (PCM_CHANNEL_RW) to 63 (PCM_CUSTOM_CHANNEL_MAP_16).\n
                             -#A set bit indicates that the pregain is set on the corresponding channel-maps.
                             Bit position of the channel-map is obtained by left shifting (1 (left shift) (Channel_map - 32))
                             }
    @h2xmle_default         {0xFFFFFFFF}  */

   int32_t pregain;
   /**< @h2xmle_description  {Pregain of the above channels (in Q27 format).}
        @h2xmle_default {0x08000000}
      @h2xmle_dataFormat  {Q27} */

}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
   ;

typedef struct param_id_msiir_ch_pregain_t param_id_msiir_ch_pregain_t;

/** @h2xmlp_parameter   {"PARAM_ID_MSIIR_TUNING_FILTER_PREGAIN", PARAM_ID_MSIIR_TUNING_FILTER_PREGAIN}
    @h2xmlp_description {Payload of the PARAM_ID_MSIIR_TUNING_FILTER_PREGAIN
 channel type/IIR pregain pairs used by the Multiple Channel IIR Tuning
 Filter module.\n
 This structure immediately follows the param_id_msiir_pregain__cfg_t
 structure.\n}
    @h2xmlp_toolPolicy  {NO_SUPPORT} 
  @h2xmlx_expandStructs {false} */
#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"

/* Payload of the PARAM_ID_MSIIR_TUNING_FILTER_PREGAIN
 parameters used by the Multiple Channel IIR Tuning Filter module.
 */
struct param_id_msiir_pregain_t
{
   uint32_t num_config;
   /**< @h2xmle_description  {Number of pregain configurations provided}
        @h2xmle_range    {1..63}
        @h2xmle_default {1}       */

   param_id_msiir_ch_pregain_t pre_gain_settings[0];
   /**< @h2xmle_description  {pre_gain_settings.}
        @h2xmle_variableArraySize {num_config} */
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;

/* Structure for the multichannel IIR pregain command */
typedef struct param_id_msiir_pregain_t param_id_msiir_pregain_t;


#define PARAM_ID_MSIIR_TUNING_FILTER_CONFIG_PARAMS 0x08001022

/** @h2xmlp_subStruct */

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
struct param_id_msiir_ch_filter_config_t
{
   uint32_t channel_mask_lsb;
   /**< @h2xmle_description  {Lower 32 bits of the channel mask. Each bit corresponds to channel map
                             from 1 (PCM_CHANNEL_L) to 31 (PCM_CHANNEL_LW).
                             Bit 0 is reserved and must be set to zero.\n
                             -#A set bit indicates that the filter corresponding to the set channel-maps are configured.
                             Bit position of the channel-map is obtained by left shifting (1 (left shift) Channel_map)
                             }
    @h2xmle_default          {0xFFFFFFFE} */


   uint32_t channel_mask_msb;
   /**< @h2xmle_description  {Upper 32 bits of the channel mask. Each bit corresponds to channel map
                             from 32 (PCM_CHANNEL_RW) to 63 (PCM_CUSTOM_CHANNEL_MAP_16).\n
                             -#A set bit indicates that the filter corresponding to the set channel-maps are configured.
                             Bit position of the channel-map is obtained by left shifting (1 (left shift) (Channel_map - 32))
                             }
    @h2xmle_default         {0xFFFFFFFF}  */

   uint16_t reserved;
   /**< @h2xmle_description {Clients must set this field to 0.}
   @h2xmle_rangeList {"0"=0}
   @h2xmle_readOnly  {true}
        */

   uint16_t num_biquad_stages;
   /**< @h2xmle_description {Number of biquad-IIR bands.}
        @h2xmle_range  {0..20}
   @h2xmle_default {1}
        */

   int32_t filter_coeffs[0];
   /**< @h2xmle_description  {filter_coeffs}
        @h2xmlx_expandArray  {true}
        @h2xmle_dataFormat   {Q30}
        @h2xmle_defaultList  {0x40000000, 0, 0, 0, 0}
   @h2xmle_policy {advanced}
        @h2xmle_variableArraySize  { "5*num_biquad_stages" } */

#if defined(__H2XML__) || defined(DOXYGEN_ONLY)
   int16_t num_shift_factor[0];
/**< @h2xmle_description  {num_shift_factor}
     @h2xmlx_expandArray  {true}
     @h2xmle_default      {0x02}
@h2xmle_policy {advanced}
     @h2xmle_variableArraySize  {"num_biquad_stages" } */
#endif // __H2XML__ || DOXYGEN_ONLY
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;

typedef struct param_id_msiir_ch_filter_config_t param_id_msiir_ch_filter_config_t;

/** @h2xmlp_parameter   {"PARAM_ID_MSIIR_TUNING_FILTER_CONFIG_PARAMS", PARAM_ID_MSIIR_TUNING_FILTER_CONFIG_PARAMS}
    @h2xmlp_description {Payload of the #PARAM_ID_MSIIR_TUNING_FILTER_CONFIG_PARAMS
 parameters used by the Multichannel IIR Tuning Filter module.\nThis structure is followed by the multichannel IIR
 filter coefficients as follows\n
    - Channel type/configuration pairs - See the Payload format table.\n
    - Sequence of int32 filter_coeffs - Five coefficients for each band, each
      in int32 format in the order of b0, b1, b2, a1, a2.\n
    - Sequence of int16 num_shift_factor - One int16 per band. The numerator
      shift factor is related to the Q factor of the filter coefficients b0,
      b1, b2.\n\n
    There must be one data sequence per channel.\n
    If the number of bands is odd, there must be an extra 16-bit padding by
    the end of the numerator shift factors. This extra 16-bit padding makes
    the entire structure 32-bit aligned. The padding bits must be all zeros.\n}
    @h2xmlp_toolPolicy  {NO_SUPPORT} 
     @h2xmlx_expandStructs  {false}
   */
#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"

/* Payload of the #PARAM_ID_MSIIR_TUNING_FILTER_CONFIG_PARAMS
 parameters used by the Multichannel IIR Tuning Filter module.
 */
struct param_id_msiir_config_t
{
   uint32_t num_config;
   /**< @h2xmle_description  {Number of channels for which enable flags are provided.}
        @h2xmle_range    {1..63}
        @h2xmle_default {1}     */

   param_id_msiir_ch_filter_config_t multi_ch_iir_cfg[0];
   /**< @h2xmle_description  {multi_ch_iir_cfg.}
        @h2xmle_variableArraySize {num_config} */
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;

/* Structure for the multichannel IIR config params */
typedef struct param_id_msiir_config_t param_id_msiir_config_t;

/* ID of the Multichannel IIR Tuning Filter Enable parameters used by
    MODULE_ID_MSIIR.
 */
#define PARAM_ID_MSIIR_TUNING_FILTER_ENABLE_V2 0x8001A8A

/* Structure for holding one channel type - IIR enable pair. */

/*
 This structure immediately follows the param_id_msiir_enable_t
 structure.
 */
/** @h2xmlp_subStruct */
#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"

typedef struct param_id_msiir_ch_enable_v2_t
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
   uint32_t enable_flag;
   /**< @h2xmle_description  {Specifies whether the above channels are enabled.}
        @h2xmle_copySrc      {enable_flag}
        @h2xmle_rangeList    {"Disable"=0;
                              "Enable"=1}
        @h2xmle_default      {0} */
} param_id_msiir_ch_enable_v2_t
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;

/** @h2xmlp_parameter   {"PARAM_ID_MSIIR_TUNING_FILTER_ENABLE_V2", PARAM_ID_MSIIR_TUNING_FILTER_ENABLE_V2}
    @h2xmlp_copySrc     {0x08001020}
    @h2xmlp_description {ID of the Multichannel IIR Tuning Filter Enable parameters used by
    MODULE_ID_MSIIR.\n Payload of the PARAM_ID_MSIIR_TUNING_FILTER_ENABLE_V2
 channel type/IIR enable pairs used by the Multiple Channel IIR Tuning
 Filter module.\n
 This structure immediately follows the param_id_msiir_enable_v2_t
 structure.\n}
    @h2xmlp_toolPolicy  {RTC, Calibration}
 @h2xmlx_expandStructs  {false}
   */
#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
/* Payload of the PARAM_ID_MSIIR_TUNING_FILTER_ENABLE_V2
 parameters used by the Multiple Channel IIR Tuning Filter module.
 */
struct param_id_msiir_enable_v2_t
{
   uint32_t num_config;
   /**< @h2xmle_description  {Number of configurations for which enable flags are provided.}
        @h2xmle_copySrc      {num_config}
        @h2xmle_range    {1..MODULE_CMN_MAX_CHANNEL}
        @h2xmle_default {1}  */
#ifdef __H2XML__
   param_id_msiir_ch_enable_v2_t enable_flag_settings[0];
   /**< @h2xmle_description  {multi_ch_iir_cfg.}
        @h2xmle_variableArraySize {num_config}
        @h2xmle_default {0} */
#endif
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;

/* Structure for the multichannel IIR enable command */
typedef struct param_id_msiir_enable_v2_t param_id_msiir_enable_v2_t;


/* ID of the Multiple Channel IIR Tuning Filter Pregain parameters used by
 */
#define PARAM_ID_MSIIR_TUNING_FILTER_PREGAIN_V2 0x8001A8B

/* Structure for holding one channel type - IIR pregain pair. */

/** @h2xmlp_subStruct */
#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"

typedef struct param_id_msiir_ch_pregain_v2_t
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

   int32_t pregain;
   /**< @h2xmle_description  {Pregain of the above channels (in Q27 format).}
        @h2xmle_copySrc      {pregain}
        @h2xmle_default {0x08000000}
      @h2xmle_dataFormat  {Q27} */

} param_id_msiir_ch_pregain_v2_t
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
   ;

/** @h2xmlp_parameter   {"PARAM_ID_MSIIR_TUNING_FILTER_PREGAIN_V2", PARAM_ID_MSIIR_TUNING_FILTER_PREGAIN_V2}
    @h2xmlp_copySrc     {0x08001021}
    @h2xmlp_description {Payload of the PARAM_ID_MSIIR_TUNING_FILTER_PREGAIN_V2
 channel type/IIR pregain pairs used by the Multiple Channel IIR Tuning
 Filter module.\n
 This structure immediately follows the param_id_msiir_pregain_cfg_v2_t
 structure.\n}
    @h2xmlp_toolPolicy  {RTC, Calibration}
  @h2xmlx_expandStructs {false} */
#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"

/* Payload of the PARAM_ID_MSIIR_TUNING_FILTER_PREGAIN_V2
 parameters used by the Multiple Channel IIR Tuning Filter module.
 */
struct param_id_msiir_pregain_v2_t
{
   uint32_t num_config;
   /**< @h2xmle_description  {Number of pregain configurations provided}
        @h2xmle_copySrc      {num_config}
        @h2xmle_range    {1..MODULE_CMN_MAX_CHANNEL}
        @h2xmle_default {1}       */
#ifdef __H2XML__
   param_id_msiir_ch_pregain_v2_t pre_gain_settings[0];
   /**< @h2xmle_description  {pre_gain_settings.}
        @h2xmle_variableArraySize {num_config} */
#endif
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;
/* Structure for the multichannel IIR pregain command */
typedef struct param_id_msiir_pregain_v2_t param_id_msiir_pregain_v2_t;


#define PARAM_ID_MSIIR_TUNING_FILTER_CONFIG_PARAMS_V2 0x8001A8C

/** @h2xmlp_subStruct */

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
typedef struct param_id_msiir_ch_filter_config_v2_t
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

   uint16_t reserved;
   /**< @h2xmle_description {Clients must set this field to 0.}
        @h2xmle_copySrc      {reserved}
   @h2xmle_rangeList {"0"=0}
   @h2xmle_readOnly  {true}
        */

   uint16_t num_biquad_stages;
   /**< @h2xmle_description {Number of biquad-IIR bands.}
        @h2xmle_copySrc      {num_biquad_stages}
        @h2xmle_range  {0..20}
   @h2xmle_default {1}
        */

   int32_t filter_coeffs[0];
   /**< @h2xmle_description  {filter_coeffs}
        @h2xmle_copySrc      {filter_coeffs}
        @h2xmlx_expandArray  {true}
        @h2xmle_dataFormat   {Q30}
        @h2xmle_defaultList  {0x40000000, 0, 0, 0, 0}
   @h2xmle_policy {advanced}
        @h2xmle_variableArraySize  { "5*num_biquad_stages" } */

#if defined(__H2XML__) || defined(DOXYGEN_ONLY)
   int16_t num_shift_factor[0];
/**< @h2xmle_description  {num_shift_factor}
     @h2xmle_copySrc      {num_shift_factor}
     @h2xmlx_expandArray  {true}
     @h2xmle_default      {0x02}
     @h2xmle_policy {advanced}
     @h2xmle_variableArraySize  {"num_biquad_stages" } */
#endif // __H2XML__ || DOXYGEN_ONLY
} param_id_msiir_ch_filter_config_v2_t
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;

/** @h2xmlp_parameter   {"PARAM_ID_MSIIR_TUNING_FILTER_CONFIG_PARAMS_V2", PARAM_ID_MSIIR_TUNING_FILTER_CONFIG_PARAMS_V2}
    @h2xmlp_copySrc     {0x08001022}
    @h2xmlp_description {Payload of the #PARAM_ID_MSIIR_TUNING_FILTER_CONFIG_PARAMS_V2
 parameters used by the Multichannel IIR Tuning Filter module.\nThis structure is followed by the multichannel IIR
 filter coefficients as follows\n
    - Channel type/configuration pairs - See the Payload format table.\n
    - Sequence of int32 filter_coeffs - Five coefficients for each band, each
      in int32 format in the order of b0, b1, b2, a1, a2.\n
    - Sequence of int16 num_shift_factor - One int16 per band. The numerator
      shift factor is related to the Q factor of the filter coefficients b0,
      b1, b2.\n\n
    There must be one data sequence per channel.\n
    If the number of bands is odd, there must be an extra 16-bit padding by
    the end of the numerator shift factors. This extra 16-bit padding makes
    the entire structure 32-bit aligned. The padding bits must be all zeros.\n}
    @h2xmlp_toolPolicy  {RTC, Calibration}
     @h2xmlx_expandStructs  {false}
   */
#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"

/* Payload of the #PARAM_ID_MSIIR_TUNING_FILTER_CONFIG_PARAMS_V2
 parameters used by the Multichannel IIR Tuning Filter module.
 */
struct param_id_msiir_config_v2_t
{
   uint32_t num_config;
   /**< @h2xmle_description  {Number of channels for which enable flags are provided.}
        @h2xmle_copySrc  {num_config}
        @h2xmle_range    {1..MODULE_CMN_MAX_CHANNEL}
        @h2xmle_default {1}     */
#ifdef __H2XML__
   param_id_msiir_ch_filter_config_v2_t multi_ch_iir_cfg[0];
   /**< @h2xmle_description  {multi_ch_iir_cfg.}
        @h2xmle_variableArraySize {num_config} */
#endif
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;
/* Structure for the multichannel IIR config params */
typedef struct param_id_msiir_config_v2_t param_id_msiir_config_v2_t;



/**
  @h2xml_Select         {param_id_msiir_enable_t}
   @h2xmlm_InsertParameter
*/

/**
   @h2xml_Select          {param_id_msiir_pregain_t}
   @h2xmlm_InsertParameter

*/

/**
   @h2xml_Select          {param_id_msiir_config_t}
   @h2xmlm_InsertParameter
*/

/**
   @h2xml_Select          {param_id_msiir_ch_pregain_t}
   @h2xmlm_InsertParameter
*/

/**
   @h2xml_Select          {param_id_msiir_ch_enable_t}
   @h2xmlm_InsertParameter
*/

/**
   @h2xml_Select          {param_id_msiir_ch_filter_config_t}
   @h2xmlm_InsertParameter
*/

/**
   @h2xml_Select          {param_id_module_enable_t}
   @h2xmlm_InsertParameter
*/
/**
  @h2xml_Select         {param_id_msiir_enable_v2_t}
   @h2xmlm_InsertParameter
*/

/**
   @h2xml_Select          {param_id_msiir_pregain_v2_t}
   @h2xmlm_InsertParameter

*/

/**
   @h2xml_Select          {param_id_msiir_config_v2_t}
   @h2xmlm_InsertParameter
*/
/**
   @h2xml_Select          {param_id_msiir_ch_pregain_v2_t}
   @h2xmlm_InsertParameter
*/

/**
   @h2xml_Select          {param_id_msiir_ch_enable_v2_t}
   @h2xmlm_InsertParameter
*/

/**
   @h2xml_Select          {param_id_msiir_ch_filter_config_v2_t}
   @h2xmlm_InsertParameter
*/
/** @}                   <-- End of the Module -->*/

#endif // API_MSIIR_H
