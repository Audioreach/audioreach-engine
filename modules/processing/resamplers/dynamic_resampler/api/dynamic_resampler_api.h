#ifndef DYNAMIC_RESAMPLER_API_H
#define DYNAMIC_RESAMPLER_API_H

/*==============================================================================
  @file dynamic_resampler_api.h
  @brief This file contains RESAMPLER API
==============================================================================*/

/* ========================================================================
  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
  SPDX-License-Identifier: BSD-3-Clause-Clear
===========================================================================*/

/*------------------------------------------------------------------------
 * Include files
 * -----------------------------------------------------------------------*/
#include "module_cmn_api.h"

/** @h2xml_title1           {Dynamic Resampler Module API}
    @h2xml_title_agile_rev  {Dynamic Resampler Module API}
    @h2xml_title_date       {September, 2018} */

/*------------------------------------------------------------------------------
   Defines
------------------------------------------------------------------------------*/
/* Input port ID of Dynamic Resampler module */
#define DYN_RS_DATA_INPUT_PORT   0x2

/* Output port ID of Dynamic Resampler module */
#define DYN_RS_DATA_OUTPUT_PORT  0x1

/*------------------------------------------------------------------------------
   Module
------------------------------------------------------------------------------*/
#define MODULE_ID_DYNAMIC_RESAMPLER   0x07001016
/**
    @h2xmlm_module       {"MODULE_ID_DYNAMIC_RESAMPLER",
                          MODULE_ID_DYNAMIC_RESAMPLER}
    @h2xmlm_displayName  {"Dynamic Resampler"}
    @h2xmlm_modSearchKeys{resampler, Audio}
    @h2xmlm_description  {- Dynamic Resampler module.\n
                          - This module converts audio sampling rate and uses FIR filters to do it. \n
                          - Therefore the SNR is higher than IIR filters but - at the cost of higher Delay.\n
                          -\n
                          - Supports following params:
                          - PARAM_ID_DYNAMIC_RESAMPLER_CONFIG
                          - PARAM_ID_DYNAMIC_RESAMPLER_OUT_CFG
                          - \n
                          - Supported Input Media Format: \n
                          - Data Format          : FIXED_POINT \n
                          - fmt_id               : Don't care \n
                          - Sample Rates         : >0 to 384 kHz \n
                          - Number of channels   : 1 to 128 (for certain products this module supports only 32 channels) \n
                          - Channel type         : Don't care \n
                          - Bits per sample      : 16, 32 \n
                          - Q format             : Don't care \n
                          - Interleaving         : de-interleaved unpacked \n
                          - Signed/unsigned      : Don't care }

	@h2xmlm_dataMaxInputPorts        { 1 }
    @h2xmlm_dataInputPorts           { IN  = DYN_RS_DATA_INPUT_PORT}
    @h2xmlm_dataOutputPorts          { OUT = DYN_RS_DATA_OUTPUT_PORT}
    @h2xmlm_dataMaxOutputPorts       { 1 }
	@h2xmlm_supportedContTypes      {APM_CONTAINER_TYPE_SC}
    @h2xmlm_isOffloadable            {true}
	@h2xmlm_stackSize                { 4096 }
    @{                   <-- Start of the Module -->
*/

/* ID of the Dynamic Resampler configuration parameter used by MODULE_ID_DYNAMIC_RESAMPLER. */
#define PARAM_ID_DYNAMIC_RESAMPLER_CONFIG   0x08001025

/* Structure payload for: PARAM_ID_DYNAMIC_RESAMPLER_CONFIG*/
/** @h2xmlp_parameter   {"PARAM_ID_DYNAMIC_RESAMPLER_CONFIG", PARAM_ID_DYNAMIC_RESAMPLER_CONFIG}
    @h2xmlp_description { Structure for setting the configuration for the Dynamic Resampler }
    @h2xmlp_toolPolicy  {Calibration; RTC}  */

#include "spf_begin_pack.h"
struct param_id_dynamic_resampler_config_t
{
   uint32_t use_hw_rs;
   /**< @h2xmle_description  {Specifies whether to use the hardware or software resampler for DYNAMIC RESAMPLER \n
                              -> If the client wants to use hardware resampler, use_hw_rs should
                              be set to 1 and dynamic_mode and delay_type values will be saved but ignored. \n
                              -> Hardware resampler can be created only if the chip supports it \n
                              -> If hardware resampler creation fails for any reason, sw resampler will
                              be created with saved dynamic mode and delay type set by client \n
							  -\n
                              -> If use_hw_rs flag is set to 0, software resampler is to be used,
                              and dynamic_mode and delay_type params come into play \n
							  -\n
                              -> Get param query on this api will return the actual resampler being used.
                              It is not simply what the client set}

        @h2xmle_rangeList    {"Use sw resampler" = 0,
                              "Use hw resampler" = 1}
        @h2xmle_default      {0}  */

   uint16_t dynamic_mode;
   /**< @h2xmle_description  {Specifies the mode of operation for the DYNAMIC RESAMPLER\n
                              -> This dynamic_mode value has significance only if sw resampler is used.}
        @h2xmle_rangeList    {"Generic resampling"= 0;
                              "Dynamic resampling"=1}
        @h2xmle_default      {0}  */


   uint16_t delay_type;
   /**< @h2xmle_description  {Specifies the delay type for the DYNAMIC RESAMPLER mode\n
                              -> This delay_type value has significance only if sw resampler is used
                              AND the dynamic_mode value is set to 1. }
        @h2xmle_rangeList    {"High delay with smooth transition"= 0;
                              "Low delay with visible transitional phase distortion"=1}
        @h2xmle_default      {0}  */
}
#include "spf_end_pack.h"
;
/* Structure type def for above payload. */
typedef struct param_id_dynamic_resampler_config_t param_id_dynamic_resampler_config_t;



/* Set param for sending output sample rate */
#define PARAM_ID_DYNAMIC_RESAMPLER_OUT_CFG       0x08001034

/** @h2xmlp_parameter   {"PARAM_ID_DYNAMIC_RESAMPLER_OUT_CFG", PARAM_ID_DYNAMIC_RESAMPLER_OUT_CFG}
    @h2xmlp_description {Specifies the output sample rate to be set.}
    @h2xmlp_toolPolicy  {Calibration; RTC}  */

#include "spf_begin_pack.h"
struct param_id_dynamic_resampler_out_cfg_t
{
   int32_t sampling_rate;
   /**< @h2xmle_description  {Specifies the output sample rate\n
                              ->Ranges from -2 to 384KHz where 
                              -2 is PARAM_VAL_UNSET and -1 is PARAM_VAL_NATIVE}
        @h2xmle_rangelist    {"PARAM_VAL_UNSET" = -2;
                             "PARAM_VAL_NATIVE" =-1;
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
        @h2xmle_default      {-1}           */
}
#include "spf_end_pack.h"
;
/* Structure type def for above payload. */
typedef struct param_id_dynamic_resampler_out_cfg_t param_id_dynamic_resampler_out_cfg_t;


/** @}                   <-- End of the Module -->*/
#endif //DYNAMIC_RESAMPLER_API_H
