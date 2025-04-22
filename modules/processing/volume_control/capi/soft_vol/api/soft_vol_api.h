#ifndef _SOFT_VOL_API_H_
#define _SOFT_VOL_API_H_

/*==============================================================================
  @file soft_vol_api.h
  @brief This file contains Soft Vol Module APIs

  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
  SPDX-License-Identifier: BSD-3-Clause-Clear
==============================================================================*/

/*------------------------------------------------------------------------
 * Include files
 * -----------------------------------------------------------------------*/
#include "module_cmn_api.h"
#include "imcl_p_eq_vol_api.h"
#include "imcl_module_gain_api.h"
#include "imcl_mute_api.h"

/** 
    @h2xml_title1           {Volume Control Module API}
    @h2xml_title_agile_rev  {Volume Control Module API}
    @h2xml_title_date       {Aug 22, 2018} */

/*==============================================================================
   Defines
==============================================================================*/
/** Supported parameters for a soft stepping linear ramping curve. */
#define PARAM_VOL_CTRL_RAMPINGCURVE_LINEAR 0

/** Exponential ramping curve. */
#define PARAM_VOL_CTRL_RAMPINGCURVE_EXP 1

/** Logarithmic ramping curve.  */
#define PARAM_VOL_CTRL_RAMPINGCURVE_LOG 2

/** Fractional exponent ramping curve.*/
#define PARAM_VOL_CTRL_RAMPINGCURVE_FRAC_EXP 3

#ifdef PROD_SPECIFIC_MAX_CH
#define VOLUME_CONTROL_MAX_CHANNELS 128
#else
#define VOLUME_CONTROL_MAX_CHANNELS 32	
#endif

/** Module ID for SOFT VOL module */
#define MODULE_ID_VOL_CTRL 0x0700101B

/* Input port ID of Vol Ctrl module */
#define VOL_CTRL_DATA_INPUT_PORT   0x2

/* Output port ID of Vol Ctrl module */
#define VOL_CTRL_DATA_OUTPUT_PORT  0x1

/*------------------------------------------------------------------------------
   Module
------------------------------------------------------------------------------*/
/**
    @h2xmlm_module       {"MODULE_ID_VOL_CTRL",
                          MODULE_ID_VOL_CTRL}
    @h2xmlm_displayName  {"Volume Control"}
    @h2xmlm_modSearchKeys{gain, Audio}
    @h2xmlm_description  {- ID of the volume control module.\n
                          - This module supports the following parameter IDs:\n
                          - #PARAM_ID_VOL_CTRL_MASTER_GAIN\n
                          - #PARAM_ID_VOL_CTRL_MASTER_MUTE\n
                          - #PARAM_ID_VOL_CTRL_GAIN_RAMP_PARAMETERS\n
                          - #PARAM_ID_VOL_CTRL_MUTE_RAMP_PARAMETERS\n
                          - #PARAM_ID_VOL_CTRL_MULTICHANNEL_GAIN\n
                          - #PARAM_ID_VOL_CTRL_MULTICHANNEL_MUTE\n
                          - Supported Input Media Format: \n
                          - Data Format          : FIXED \n
                          - fmt_id               : Don't care \n
                          - Sample Rates         : Don't care \n
                          - Number of channels   : 1 to 128 (for certain products this module supports only 32 channels) \n
                          - Channel type         : 1 to 128 \n
                          - Bits per sample      : 16, 32 \n
                          - Q format             : Don't care \n
                          - Interleaving         : de-interleaved unpacked \n
                          - Signed/unsigned      : Signed \n
                          -All parameter IDs are device independent.\n}

    @h2xmlm_dataMaxInputPorts        { 1 }
    @h2xmlm_dataInputPorts           { IN = VOL_CTRL_DATA_INPUT_PORT}
    @h2xmlm_dataOutputPorts          { OUT= VOL_CTRL_DATA_OUTPUT_PORT}
    @h2xmlm_dataMaxOutputPorts       { 1 }
	@h2xmlm_supportedContTypes      {APM_CONTAINER_TYPE_SC, APM_CONTAINER_TYPE_GC}
    @h2xmlm_isOffloadable            {true}
	@h2xmlm_stackSize                { 2048 }
	@h2xmlm_ctrlDynamicPortIntent    { "Gain Info IMCL" = INTENT_ID_GAIN_INFO, maxPorts= INFINITE }
    @h2xmlm_ctrlDynamicPortIntent  { "Mute IMCL" = INTENT_ID_MUTE, maxPorts= INFINITE }
    @h2xmlm_ctrlDynamicPortIntent  { "Popless Equalizer to Soft Volume for headroom control" = INTENT_ID_P_EQ_VOL_HEADROOM, maxPorts= 1 }
    @{                   <-- Start of the Module --> */

/*==============================================================================
   API definitions
==============================================================================*/
/* Param-id for communicating headroom requirement to the Volume Control module through
 * the service.  Currently, this is used only by the Popless Equalizer by raising CAPI_EVENT_HEADROOM event */
#ifndef PARAM_ID_ALGORITHMIC_HEADROOM
#define PARAM_ID_ALGORITHMIC_HEADROOM (0x0800103A)
#endif

/* ID of the Master Gain parameter used by MODULE_ID_VOL_CTRL. */
#define PARAM_ID_VOL_CTRL_MASTER_GAIN 0x08001035

/** @h2xmlp_parameter   {"PARAM_ID_VOL_CTRL_MASTER_GAIN", PARAM_ID_VOL_CTRL_MASTER_GAIN}
    @h2xmlp_description {Specifies the master gain}
    @h2xmlp_toolPolicy  {Calibration; RTC}*/

/* Payload of the PARAM_ID_VOL_CTRL_MASTER_GAIN parameter used
 by the Volume Control module */
 /* Structure for the master gain parameter for a volume control module. */
#include "spf_begin_pack.h"
struct volume_ctrl_master_gain_t
{
   uint16_t master_gain;
/**< @h2xmle_description  {Specifies linear master gain in Q13 format\n}
     @h2xmle_dataFormat   {Q13}
     @h2xmle_default      {0x2000} */

   uint16_t reserved;
/**< @h2xmle_description  {Clients must set this field to 0.\n}
     @h2xmle_rangeList    {"0" = 0}
     @h2xmle_default      {0}     */
}
#include "spf_end_pack.h"
;

/* Structure type def for above payload. */
typedef struct volume_ctrl_master_gain_t volume_ctrl_master_gain_t;



/** ID of the mute Configuration parameter used by MODULE_ID_VOL_CTRL. */
#define PARAM_ID_VOL_CTRL_MASTER_MUTE 0x08001036
/** @h2xmlp_parameter   {"PARAM_ID_VOL_CTRL_MASTER_MUTE", PARAM_ID_VOL_CTRL_MASTER_MUTE}
    @h2xmlp_description {Configures the mute flag}
    @h2xmlp_toolPolicy  {Calibration; RTC}*/

#include "spf_begin_pack.h"

/* Payload of the PARAM_ID_VOL_CTRL_MASTER_MUTE parameter used
 by the Volume Control module */
 /* Structure for the mute configuration parameter for a
 volume control module. */
struct volume_ctrl_master_mute_t
{
   uint32_t mute_flag;
/**< @h2xmle_description {Specifies whether mute is enabled}
     @h2xmle_rangeList   {"Disable"= 0;
                          "Enable"=1}
     @h2xmle_default     {0}  */
}
#include "spf_end_pack.h"
;

/* Structure type def for above payload. */
typedef struct volume_ctrl_master_mute_t volume_ctrl_master_mute_t;



/* ID of the Soft Stepping gain ramp parameters used by MODULE_ID_VOL_CTRL. */
#define PARAM_ID_VOL_CTRL_GAIN_RAMP_PARAMETERS 0x08001037
/** @h2xmlp_parameter   {"PARAM_ID_VOL_CTRL_GAIN_RAMP_PARAMETERS", PARAM_ID_VOL_CTRL_GAIN_RAMP_PARAMETERS}
    @h2xmlp_toolPolicy  {RTC; Calibration}
    @h2xmlp_description {Specifies Soft Stepping gain ramp parameters} */


/* Structure for holding soft stepping volume parameters. */
#include "spf_begin_pack.h"
struct volume_ctrl_gain_ramp_params_t
{
   uint32_t period_ms;
/**< @h2xmle_description  { Specifies period in milliseconds}
     @h2xmle_range        {0..15000}
     @h2xmle_default      {0} */

   uint32_t step_us;
/**< @h2xmle_description  { Specifies step in microseconds}
     @h2xmle_range        {0..15000000}
     @h2xmle_default      {0} */

   uint32_t ramping_curve;
/**< @h2xmle_description  {Specifies ramping curve type.\n
                         -Supported Values for ramping curve:\n
                         ->Linear ramping curve\n
                         -PARAM_VOL_CTRL_RAMPINGCURVE_LINEAR - 0 \n
                         ->Exponential ramping curve\n
                         -PARAM_VOL_CTRL_RAMPINGCURVE_EXP    - 1 \n
                         ->Logarithmic ramping curve\n
                         -PARAM_VOL_CTRL_RAMPINGCURVE_LOG    - 2 \n
                         ->Fractional exponent ramping curve\n
                         -PARAM_VOL_CTRL_RAMPINGCURVE_FRAC_EXP - 3  }

     @h2xmle_rangeList   {"PARAM_VOL_CTRL_RAMPINGCURVE_LINEAR" = 0;
                         "PARAM_VOL_CTRL_RAMPINGCURVE_EXP" = 1;
                         "PARAM_VOL_CTRL_RAMPINGCURVE_LOG" = 2;
                         "PARAM_VOL_CTRL_RAMPINGCURVE_FRAC_EXP" = 3}
    @h2xmle_default     {0}   */
}
#include "spf_end_pack.h"
;

/* Structure type def for above payload. */
typedef struct volume_ctrl_gain_ramp_params_t volume_ctrl_gain_ramp_params_t;



/* ID of the Soft Stepping mute ramp parameters used by MODULE_ID_VOL_CTRL. */
#define PARAM_ID_VOL_CTRL_MUTE_RAMP_PARAMETERS 0x0800103D
/** @h2xmlp_parameter   {"PARAM_ID_VOL_CTRL_MUTE_RAMP_PARAMETERS", PARAM_ID_VOL_CTRL_MUTE_RAMP_PARAMETERS}
    @h2xmlp_description {Specifies Soft Stepping mute ramp parameters }
    @h2xmlp_toolPolicy  {RTC;Calibration} */

/* Structure for holding soft stepping volume parameters. */
#include "spf_begin_pack.h"
struct volume_ctrl_mute_ramp_params_t
{
   uint32_t period_ms;
/**< @h2xmle_description  { Specifies period in milliseconds.\n }
     @h2xmle_range        {0..15000}
     @h2xmle_default      {0} */

   uint32_t step_us;
/**< @h2xmle_description  { Specifies step in microseconds.\n}
     @h2xmle_range        {0..15000000}
     @h2xmle_default      {0} */

   uint32_t ramping_curve;
/**< @h2xmle_description  { Specifies ramping curve type.\n
                          -Supported Values for ramping curve:\n
                          ->Linear ramping curve\n
                          -PARAM_VOL_CTRL_RAMPINGCURVE_LINEAR - 0 \n
                          ->Exponential ramping curve\n
                          -PARAM_VOL_CTRL_RAMPINGCURVE_EXP    - 1 \n
                          ->Logarithmic ramping curve\n
                          -PARAM_VOL_CTRL_RAMPINGCURVE_LOG    - 2 \n
                          ->Fractional exponent ramping curve\n
                          -PARAM_VOL_CTRL_RAMPINGCURVE_FRAC_EXP - 3  }

     @h2xmle_rangeList   {"PARAM_VOL_CTRL_RAMPINGCURVE_LINEAR" = 0;
                          "PARAM_VOL_CTRL_RAMPINGCURVE_EXP" = 1;
                          "PARAM_VOL_CTRL_RAMPINGCURVE_LOG" = 2;
                          "PARAM_VOL_CTRL_RAMPINGCURVE_FRAC_EXP" = 3}
     @h2xmle_default     {0}   */
}
#include "spf_end_pack.h"
;

/* Structure type def for above payload. */
typedef struct volume_ctrl_mute_ramp_params_t volume_ctrl_mute_ramp_params_t;



/* Structure for holding one channel type - gain pair. */
/** @h2xmlp_subStruct */
#include "spf_begin_pack.h"
struct volume_ctrl_channels_gain_config_t
{
   uint32_t channel_mask_lsb;
   /**< @h2xmle_description  {Lower 32 bits of the channel mask. Each bit corresponds to channel map 
                             from 1 (PCM_CHANNEL_L) to 31 (PCM_CHANNEL_LW). 
                             Bit 0 is reserved and must be set to zero.\n
                             -#A set bit indicates that the gain is set on the corresponding channel-maps. 
                             Bit position of the channel-map is obtained by left shifting (1 (left shift) Channel_map) 
                             }
    @h2xmle_default          {0xFFFFFFFE} */
   

   uint32_t channel_mask_msb;
   /**< @h2xmle_description  {Upper 32 bits of the channel mask. Each bit corresponds to channel map 
                             from 32 (PCM_CHANNEL_RW) to 63 (PCM_CUSTOM_CHANNEL_MAP_16).\n
                             -#A set bit indicates that the gain is set on the corresponding channel-maps.
                             Bit position of the channel-map is obtained by left shifting (1 (left shift) (Channel_map - 32)) 
                             }
    @h2xmle_default         {0xFFFFFFFF}  */

   uint32_t gain;
   /**< @h2xmle_description  {Gain value for the above channels in Q28 format}
        @h2xmle_dataFormat   {Q28}
        @h2xmle_default      {0x10000000} */
} 
#include "spf_end_pack.h"
;

/* Structure type def for above payload. */
typedef struct volume_ctrl_channels_gain_config_t volume_ctrl_channels_gain_config_t;


/* ID of the Multi-channel Volume Control parameters used by #MODULE_ID_VOL_CTRL. */
#define PARAM_ID_VOL_CTRL_MULTICHANNEL_GAIN 0x08001038
/** @h2xmlp_parameter   {"PARAM_ID_VOL_CTRL_MULTICHANNEL_GAIN",PARAM_ID_VOL_CTRL_MULTICHANNEL_GAIN}
    @h2xmlp_description { Payload of the multi channel type gain pairs used by the Volume Control module}
    @h2xmlp_toolPolicy  {NO_SUPPORT}
    @h2xmlp_maxSize     {760}*/

/* Structure for the multichannel gain command */
#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
struct volume_ctrl_multichannel_gain_t
{
   uint32_t num_config;
   /**< @h2xmle_description  {Number of channels-gain configurations provided}
        @h2xmle_range        {1..63}
        @h2xmle_default      {1}  */

   volume_ctrl_channels_gain_config_t gain_data[0];
   /**< @h2xmle_description  {Payload consisting of all channels-gain pairs }
       @h2xmle_variableArraySize {num_config}	*/

}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;

/* Structure type def for above payload. */
typedef struct volume_ctrl_multichannel_gain_t volume_ctrl_multichannel_gain_t;

/*   @h2xml_Select					{volume_ctrl_channels_gain_config_t}
     @h2xmlm_InsertParameter */



/* Structure for holding one channel type - mute pair. */
/** @h2xmlp_subStruct */
#include "spf_begin_pack.h"
struct volume_ctrl_channels_mute_config_t
{
   uint32_t channel_mask_lsb;
   /**< @h2xmle_description  {Lower 32 bits of the channel mask. Each bit corresponds to channel map 
                             from 1 (PCM_CHANNEL_L) to 31 (PCM_CHANNEL_LW). 
                             Bit 0 is reserved and must be set to zero.\n
                             -#A set bit indicates whether mute is enabled on the corresponding channel-maps. 
                             Bit position of the channel-map is obtained by left shifting (1 (left shift) Channel_map) 
                             }
    @h2xmle_default          {0xFFFFFFFE} */
   

   uint32_t channel_mask_msb;
   /**< @h2xmle_description  {Upper 32 bits of the channel mask. Each bit corresponds to channel map 
                             from 32 (PCM_CHANNEL_RW) to 63 (PCM_CUSTOM_CHANNEL_MAP_16).\n
                             -#A set bit indicates whether mute is enabled on the corresponding channel-maps.
                             Bit position of the channel-map is obtained by left shifting (1 (left shift) (Channel_map - 32)) 
                             }
    @h2xmle_default         {0xFFFFFFFF}  */

   uint32_t mute;
   /**< @h2xmle_description  { Specifies mute for the above channels }
        @h2xmle_rangeList    {"Disable"= 0;
                              "Enable"=1}
        @h2xmle_default      {0}
                              */
}
#include "spf_end_pack.h"
;

/* Structure type def for above payload. */
typedef struct volume_ctrl_channels_mute_config_t volume_ctrl_channels_mute_config_t;

/* ID of the Multichannel Mute Configuration parameters used by #MODULE_ID_VOL_CTRL. */
#define PARAM_ID_VOL_CTRL_MULTICHANNEL_MUTE 0x08001039
/** @h2xmlp_parameter   {"PARAM_ID_VOL_CTRL_MULTICHANNEL_MUTE", PARAM_ID_VOL_CTRL_MULTICHANNEL_MUTE}
    @h2xmlp_description {Payload of the PARAM_ID_VOL_CTRL_MULTICHANNEL_MUTE channel type/mute
                         setting pairs used by the Volume Control module}
    @h2xmlp_toolPolicy  {NO_SUPPORT}
    @h2xmlp_maxSize     {760} */

/* Structure for the multichannel mute command */
#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
struct volume_ctrl_multichannel_mute_t
{
   uint32_t num_config;
   /**< @h2xmle_description  {Number of channels-mute configurations provided}
        @h2xmle_range        {1..63}
        @h2xmle_default      {1}		*/

   volume_ctrl_channels_mute_config_t mute_data[0];
   /**< @h2xmle_description  {Array of channels-mute setting pairs}
        @h2xmle_variableArraySize {num_config}*/
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;

/* Structure type def for above payload. */
typedef struct volume_ctrl_multichannel_mute_t volume_ctrl_multichannel_mute_t;

/*   @h2xml_Select					{volume_ctrl_channels_mute_config_t}
     @h2xmlm_InsertParameter */


/* Structure for holding one channel type - gain pair. */
/** @h2xmlp_subStruct */
#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
struct volume_ctrl_channels_gain_config_v2_t
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

   uint32_t gain;
   /**< @h2xmle_description  {Gain value for the above channels in Q28 format}
        @h2xmle_copySrc      {gain}
        @h2xmle_dataFormat   {Q28}
        @h2xmle_default      {0x10000000} */
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;

/* Structure type def for above payload. */
typedef struct volume_ctrl_channels_gain_config_v2_t volume_ctrl_channels_gain_config_v2_t;


/* ID of the Multi-channel Volume Control parameters used by #MODULE_ID_VOL_CTRL. */
#define PARAM_ID_VOL_CTRL_MULTICHANNEL_GAIN_V2 0x8001A88
/** @h2xmlp_parameter   {"PARAM_ID_VOL_CTRL_MULTICHANNEL_GAIN_V2",PARAM_ID_VOL_CTRL_MULTICHANNEL_GAIN_V2}
    @h2xmlp_copySrc     {0x08001038}
    @h2xmlp_description { Payload of the multi channel type gain pairs used by the Volume Control module}
    @h2xmlp_toolPolicy  {Calibration; RTC}*/

/* Structure for the multichannel gain command */
#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
struct volume_ctrl_multichannel_gain_v2_t
{
   uint32_t num_config;
   /**< @h2xmle_description  {Number of channels-gain configurations provided}
        @h2xmle_copySrc      {num_config}
        @h2xmle_range        {1..MODULE_CMN_MAX_CHANNEL}
        @h2xmle_default      {1}  */

   volume_ctrl_channels_gain_config_v2_t gain_config[0];
   /**< @h2xmle_description  {Payload consisting of all channels-gain pairs }
       @h2xmle_variableArraySize {num_config}    */

}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;

/* Structure type def for above payload. */
typedef struct volume_ctrl_multichannel_gain_v2_t volume_ctrl_multichannel_gain_v2_t;

/*   @h2xml_Select                    {volume_ctrl_multichannel_gain_v2_t}
     @h2xmlm_InsertParameter */



/* Structure for holding one channel type - mute pair. */
/** @h2xmlp_subStruct */
#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
struct volume_ctrl_channels_mute_config_v2_t
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

   uint32_t mute;
   /**< @h2xmle_description  { Specifies mute for the above channels }
        @h2xmle_copySrc      {mute}
        @h2xmle_rangeList    {"Disable"= 0;
                              "Enable"=1}
        @h2xmle_default      {0}
                              */
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;

/* Structure type def for above payload. */
typedef struct volume_ctrl_channels_mute_config_v2_t volume_ctrl_channels_mute_config_v2_t;

/* ID of the Multichannel Mute Configuration parameters used by #MODULE_ID_VOL_CTRL. */
#define PARAM_ID_VOL_CTRL_MULTICHANNEL_MUTE_V2 0x8001A89
/** @h2xmlp_parameter   {"PARAM_ID_VOL_CTRL_MULTICHANNEL_MUTE_V2", PARAM_ID_VOL_CTRL_MULTICHANNEL_MUTE_V2}
    @h2xmlp_copySrc     {0x08001039}
    @h2xmlp_description {Payload of the PARAM_ID_VOL_CTRL_MULTICHANNEL_MUTE_V2 channel type/mute
                         setting pairs used by the Volume Control module}
    @h2xmlp_toolPolicy  {Calibration; RTC} */

/* Structure for the multichannel mute command */
#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
struct volume_ctrl_multichannel_mute_v2_t
{
   uint32_t num_config;
   /**< @h2xmle_description  {Number of channels-mute configurations provided}
        @h2xmle_copySrc      {num_config}
        @h2xmle_range        {1..MODULE_CMN_MAX_CHANNEL}
        @h2xmle_default      {1}        */

   volume_ctrl_channels_mute_config_v2_t mute_config[0];
   /**< @h2xmle_description  {Array of channels-mute setting pairs}
        @h2xmle_variableArraySize {num_config}*/
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;

/* Structure type def for above payload. */
typedef struct volume_ctrl_multichannel_mute_v2_t volume_ctrl_multichannel_mute_v2_t;

/*   @h2xml_Select                    {volume_ctrl_channels_mute_config_v2_t}
     @h2xmlm_InsertParameter */



/** @}                   <-- End of the Module -->*/
#endif //_SOFT_VOL_API_H_
