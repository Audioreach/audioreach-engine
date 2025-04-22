#ifndef CHMIXER_API_H
#define CHMIXER_API_H

/*==============================================================================
  @file chmixer_api.h
  @brief This file contains CHMIXER API
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

/** @h2xml_title1           {Channel Mixer Module API}
    @h2xml_title_agile_rev  {Channel Mixer Module API}
    @h2xml_title_date       {February 9, 2018} */

/*------------------------------------------------------------------------------
   Defines
------------------------------------------------------------------------------*/
/* Input port ID of chmixer module */
#define CHMIXER_DATA_INPUT_PORT   0x2

/* Output port ID of chmixer module */
#define CHMIXER_DATA_OUTPUT_PORT  0x1

/*------------------------------------------------------------------------------
   Parameters
------------------------------------------------------------------------------*/
/* ID of the Coefficient parameter used by MODULE_ID_CHMIXER to configure the channel mixer weighting coefficients. */
#define PARAM_ID_CHMIXER_OUT_CH_CFG  0x0800103B

/** @h2xmlp_parameter   {"PARAM_ID_CHMIXER_OUT_CH_CFG", PARAM_ID_CHMIXER_OUT_CH_CFG}
    @h2xmlp_description {Configures the channel mixer output config}
    @h2xmlp_toolPolicy  {RTC; Calibration}
    @h2xmlp_maxSize     {68}*/

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
struct param_id_chmixer_out_ch_cfg_t
{
	int16_t num_channels;
	/**< @h2xmle_description  { Specifies number of channels \n
	                            - Sender will have to pad accordingly to make sure it is 4 byte aligned}
	     @h2xmle_range        {-2..MODULE_CMN_MAX_CHANNEL}
	     @h2xmle_default      {-1}  */
	uint16_t channel_map[0];
	/**< @h2xmle_description  { Payload consisting of channel mapping information for all the channels \n 
	                            ->If num_channels field is set to PARAM_VAL_NATIVE (-1) or PARAM_VAL_UNSET(-2)
							  this field will be ignored} 
         @h2xmle_variableArraySize {num_channels}
         @h2xmle_rangeEnum    {pcm_channel_map}
         @h2xmle_default      {1}*/
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;

/* Structure type def for above payload. */
typedef struct param_id_chmixer_out_ch_cfg_t param_id_chmixer_out_ch_cfg_t;

/*------------------------------------------------------------------------------
   Module
------------------------------------------------------------------------------*/

#define MODULE_ID_CHMIXER 0x07001013
/**
    @h2xmlm_module       {"MODULE_ID_CHMIXER",
                          MODULE_ID_CHMIXER}
    @h2xmlm_displayName  {"Channel Mixer"}
    @h2xmlm_modSearchKeys{channel mixer,Audio}
    @h2xmlm_description  {- ID of the Channel Mixer module. This module upmixes or downmixes
    audio channels based on configured coefficients\n
                          - Supported Input Media Format:     \n
                          - Data Format          : FIXED_POINT \n
                          - fmt_id               : Don't care\n
                          - Sample Rates         : >0 \n
                          - Number of channels   : 1 to 128\n
                          - Channel type         : 0 to 128\n
                          - Bits per sample      : 16, 32\n
                          - Q format             : Don't care\n
                          - Interleaving         : de-interleaved unpacked\n
                          - Signed/unsigned      : Signed}
    @h2xmlm_dataInputPorts       { IN = CHMIXER_DATA_INPUT_PORT}
    @h2xmlm_dataOutputPorts      { OUT = CHMIXER_DATA_OUTPUT_PORT}
    @h2xmlm_dataMaxInputPorts    { 1 }
    @h2xmlm_dataMaxOutputPorts    { 1 }
	@h2xmlm_supportedContTypes  { APM_CONTAINER_TYPE_SC, APM_CONTAINER_TYPE_GC}
    @h2xmlm_isOffloadable        {true}
	@h2xmlm_stackSize            { 2048 }
    @{                   <-- Start of the Module -->  
    
	@h2xml_Select		 {param_id_chmixer_out_ch_cfg_t}
    @h2xmlm_InsertParameter

    @h2xml_Select		 {param_id_chmixer_coeff_t}
    @h2xmlm_InsertParameter

    @h2xml_Select		 {chmixer_coeff_t}
    @h2xmlm_InsertStructure
    @}                   <-- End of the Module -->*/
#endif // CHMIXER_API_H
