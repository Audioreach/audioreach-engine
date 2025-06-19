/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef VIRTUALIZER_CALIB_H
#define VIRTUALIZER_CALIB_H
/*==============================================================================
  @file Virtualizer_calib.h
  @brief This file contains VIRTUALIZER API
==============================================================================*/

#include "module_cmn_api.h"

/** @h2xml_title1           {Virtualizer Module}
    @h2xml_title_agile_rev  {Virtualizer Module}
    @h2xml_title_date       {May 31,2019} */

/*------------------------------------------------------------------------------
   Module
------------------------------------------------------------------------------*/
/*==============================================================================
   Constants
==============================================================================*/


/* ID of the Virtualizer Strength parameter used by MODULE_ID_VIRTUALIZER. */
#define PARAM_ID_VIRTUALIZER_STRENGTH                    0x08001136

/* Structure for the strength parameter of VIRTUALIZER module. */
typedef struct param_id_virtualizer_strength_t param_id_virtualizer_strength_t;
/** @h2xmlp_parameter   {"PARAM_ID_VIRTUALIZER_STRENGTH", PARAM_ID_VIRTUALIZER_STRENGTH}
    @h2xmlp_description {Specifies the virtualizer strength } 
	@h2xmlp_toolPolicy              {Calibration}*/

#include "spf_begin_pack.h"

/* Payload of the PARAM_ID_VIRTUALIZER_STRENGTH parameter used by the Virtualizer module. */
struct param_id_virtualizer_strength_t
{
   uint32_t strength;
   /**< @h2xmle_description  {Specifies the virtualizer strength.\n }
         @h2xmle_range       {0..1000}
         @h2xmle_default     {1000}*/

}
#include "spf_end_pack.h"
;


/* ID of the Virtualizer Out Type parameter used by MODULE_ID_VIRTUALIZER.*/
#define PARAM_ID_VIRTUALIZER_OUT_TYPE                    0x08001137

/* Structure for the out type parameter of VIRTUALIZER module. */
typedef struct param_id_virtualizer_out_type_t param_id_virtualizer_out_type_t;
/** @h2xmlp_parameter   {"PARAM_ID_VIRTUALIZER_OUT_TYPE", PARAM_ID_VIRTUALIZER_OUT_TYPE}
    @h2xmlp_description {Specifies the output device type of the virtualizer.}
	@h2xmlp_toolPolicy              {Calibration} */

/* Payload of the PARAM_ID_VIRTUALIZER_OUT_TYPE parameter used by the Virtualizer module. */
#include "spf_begin_pack.h"

struct param_id_virtualizer_out_type_t
{
   uint32_t out_type;
   /**< @h2xmle_description  { Specifies the output device type of the virtualizer.\n}
        @h2xmle_rangeList        {"Headphone"= 0;
                                   "Desktop speakers"=1}
    @h2xmle_default         {0}    */

}
#include "spf_end_pack.h"
;


/* ID of the Virtualizer Gain Adjust parameter used by MODULE_ID_VIRTUALIZER. */
#define PARAM_ID_VIRTUALIZER_GAIN_ADJUST                 0x08001138


/* Structure for the strength parameter of VIRTUALIZER module. */
typedef struct param_id_virtualizer_gain_adjust_t param_id_virtualizer_gain_adjust_t;
/** @h2xmlp_parameter   {"PARAM_ID_VIRTUALIZER_GAIN_ADJUST", PARAM_ID_VIRTUALIZER_GAIN_ADJUST}
    @h2xmlp_description {Specifies the overall gain adjustment of virtualizer outputs.}
	@h2xmlp_toolPolicy              {Calibration}  */

#include "spf_begin_pack.h"

/* Payload of the PARAM_ID_VIRTUALIZER_GAIN_ADJUST parameter used by the Virtualizer module. */

struct param_id_virtualizer_gain_adjust_t
{
   int32_t gain_adjust;
   /**< @h2xmle_description  { Specifies the overall gain adjustment of virtualizer outputs, in the unit 'millibels'.\n}
        @h2xmle_default     {0}
        @h2xmle_range       {-600..600}
        @@h2xmle_default  {0} */

}
#include "spf_end_pack.h"
;

#define PARAM_ID_VIRTUALIZER_OUT_CH_CFG                 0x08001139

typedef struct param_id_virtualizer_out_ch_cfg_t param_id_virtualizer_out_ch_cfg_t;
/** @h2xmlp_parameter   {"PARAM_ID_VIRTUALIZER_OUT_CH_CFG", PARAM_ID_VIRTUALIZER_OUT_CH_CFG}
    @h2xmlp_description {Specifies the output channel configuration of the module.}
	@h2xmlp_toolPolicy              {Calibration}  */

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"

struct param_id_virtualizer_out_ch_cfg_t
{
  int16_t num_channels;
  /**< @h2xmle_description  { Specifies number of channels \n
                              - Sender will have to pad accordingly to make sure it is 4 byte aligned}
       @h2xmle_range     {1..2}
       @h2xmle_default      {2}  */
  uint16_t channel_map[0];
  /**< @h2xmle_description  { Payload consisting of channel mapping information for all the channels. \n
                              The channel mapping is specific for specific number of channels \n
                              for 1 : [PCM_CHANNEL_C] \n
                              for 2 : [PCM_CHANNEL_L, PCM_CHANNEL_R] \n }
       @h2xmle_range        {1..3}
       @h2xmle_defaultList    {1,2}
       @h2xmle_variableArraySize {num_channels}*/
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;


/* Global unique Module ID definition
   Module library is independent of this number, it defined here for static
   loading purpose only */

#define MODULE_ID_VIRTUALIZER                            0x07001064
/**
    @h2xmlm_module       {"MODULE_ID_VIRTUALIZER",
                          MODULE_ID_VIRTUALIZER}
    @h2xmlm_displayName  {"Virtualizer"}
    @h2xmlm_modSearchKeys{effects, Audio}
    @h2xmlm_description  {ID of the VIRTUALIZER module.\n

    . This module supports the following parameter IDs:\n
    .           1. PARAM_ID_MODULE_ENABLE \n
    .           2. PARAM_ID_VIRTUALIZER_STRENGTH \n
    .           3. PARAM_ID_VIRTUALIZER_OUT_TYPE \n
    .           4. PARAM_ID_VIRTUALIZER_GAIN_ADJUST \n
    .           5. PARAM_ID_VIRTUALIZER_OUT_CH_CFG \n
    \n
    . Supported Containers: \n
    .           1. APM_CONTAINER_TYPE_ID_GC (or) \n
    .           2. APM_CONTAINER_TYPE_ID_SC 
    \n
    . Supported Input Media Format:      \n
    .           Data Format          : FIXED_POINT \n
    .           fmt_id               : MEDIA_FMT_ID_PCM \n
    .           Sample Rates         : Standard sample rates between 8kHz and 192kHz \n
    .           Number of channels   : 1,2,6,8  \n
    .           Channel type         : Supported channel mapping based on number of channels is given below.  \n
    .           Bits per sample      : 16,32 \n
    .           Q format             : Q15 for bps = 16, Q27 for bps = 32 \n
    .           Interleaving         : Deinterleaved Unpacked \n
    .           Signed/unsigned      : Signed  \n
    \n
    . Supported Input Channel Mapping based on number of input channels: \n
    .           1:  mono  [C] \n
    .           2:  stereo  [L, R] \n
    .           6:  5.1 channel  [L, R, C, LFE, LS, RS] or [L, R, C, LFE, LB, RB] \n
    .           8:  7.1 channel  [L, R, C, LFE, LS, RS, LB, RB] \n
    \n
    . Supported Output Channel Mapping based on number of output channels: \n
    .           1: mono [C] \n
    .           2: stereo [L, R] \n
    \n
   }
    @h2xmlm_toolPolicy              {Calibration}
    @h2xmlm_dataMaxInputPorts        { 1 }
    @h2xmlm_dataInputPorts           { IN=2}
    @h2xmlm_dataOutputPorts          { OUT=1}
    @h2xmlm_dataMaxOutputPorts       { 1 }
	@h2xmlm_supportedContTypes      {APM_CONTAINER_TYPE_GC, APM_CONTAINER_TYPE_SC}
    @h2xmlm_isOffloadable            {true}
	@h2xmlm_stackSize           {4096}
    @h2xmlm_toolPolicy              {Calibration}

   @{                   <-- Start of the Module -->
      @h2xml_Select        {"param_id_module_enable_t"}
      @h2xmlm_InsertParameter
      @h2xml_Select        {"param_id_virtualizer_strength_t"}
      @h2xmlm_InsertParameter
      @h2xmlp_toolPolicy   {Calibration}
	  @h2xml_Select        {"param_id_virtualizer_out_type_t"}
      @h2xmlm_InsertParameter
      @h2xmlp_toolPolicy   {Calibration}
	  @h2xml_Select        {"param_id_virtualizer_gain_adjust_t"}
      @h2xmlm_InsertParameter
      @h2xmlp_toolPolicy   {Calibration}
	  @h2xml_Select        {"param_id_virtualizer_out_ch_cfg_t"}
      @h2xmlm_InsertParameter
      @h2xmlp_toolPolicy   {Calibration}
   @}                   <-- End of the Module -->*/


#endif

