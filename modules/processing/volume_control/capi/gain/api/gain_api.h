#ifndef _GAIN_API_H_
#define _GAIN_API_H_

/*==============================================================================
  @file gain_api.h
  @brief This file contains Gain module APIs

  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
  SPDX-License-Identifier: BSD-3-Clause-Clear
==============================================================================*/

 /*------------------------------------------------------------------------
 * Include files
 * -----------------------------------------------------------------------*/
#include "module_cmn_api.h"

/** @h2xml_title1          {Gain API}
     @h2xml_title_agile_rev {Gain API}
     @h2xml_title_date      {July 15, 2018} */

/*------------------------------------------------------------------------------
   Defines
------------------------------------------------------------------------------*/
/** @ingroup ar_spf_mod_gain_macros
    Input port ID of the Gain module. */
#define GAIN_DATA_INPUT_PORT   0x2

/** @ingroup ar_spf_mod_gain_macros
    Output port ID of the Gain module. */
#define GAIN_DATA_OUTPUT_PORT  0x1

/** @ingroup ar_spf_mod_gain_macros
    Stack size of the Gain module. */
#define GAIN_STACK_SIZE 4096
/*==============================================================================
   Module
==============================================================================*/

/** @ingroup ar_spf_mod_gain_macros
    Gain module.

    @subhead4{Supported parameter IDs}
    - PARAM_ID_GAIN

    @subhead4{Supported input media format ID}
    - Data Format          : FIXED_POINT @lstsp1
    - fmt_id               : Don't care @lstsp1
    - Sample Rates         : Don't care @lstsp1
    - Number of channels   : 1 to 128 (for certain products this module supports only 32 channels) @lstsp1
    - Channel type         : Don't care @lstsp1
    - Bits per sample      : 16, 32 @lstsp1
    - Q format             : Don't care @lstsp1
    - Interleaving         : De-interleaved unpacked @lstsp1
    - Signed/unsigned      : Signed @lstsp1
 */
#define MODULE_ID_GAIN 			0x07001002
/** @h2xmlm_module       {"MODULE_ID_GAIN",
                           MODULE_ID_GAIN}
    @h2xmlm_displayName  {"Gain"}
    @h2xmlm_modSearchKeys{gain, Audio}
	@h2xmlm_description  {Gain Module \n
                          - Supports following params:
                          - PARAM_ID_GAIN
                          - \n
                          - Supported Input Media Format:
                          - Data Format          : FIXED_POINT
                          - fmt_id               : Don't care
                          - Sample Rates         : Don't care
                          - Number of channels   : 1 to 128 (for certain products this module supports only 32 channels)
                          - Channel type         : Don't care
                          - Bits per sample      : 16, 32
                          - Q format             : Don't care
                          - Interleaving         : de-interleaved unpacked
                          - Signed/unsigned      : Signed}
    @h2xmlm_dataMaxInputPorts        { 1 }
    @h2xmlm_dataInputPorts           { IN  = GAIN_DATA_INPUT_PORT}
    @h2xmlm_dataOutputPorts          { OUT = GAIN_DATA_OUTPUT_PORT}
    @h2xmlm_dataMaxOutputPorts       { 1 }
	@h2xmlm_supportedContTypes      {APM_CONTAINER_TYPE_SC, APM_CONTAINER_TYPE_GC}
    @h2xmlm_isOffloadable            {true}
	@h2xmlm_stackSize                { GAIN_STACK_SIZE }
    @{                   <-- Start of the Module -->

    @h2xml_Select        {"param_id_module_enable_t"}
    @h2xmlm_InsertParameter

*/

/*==============================================================================
   API definitions
==============================================================================*/
/** @ingroup ar_spf_mod_gain_macros
    ID of the parameter used to set the gain. */
#define PARAM_ID_GAIN           0x08001006

/** @h2xmlp_parameter   {"PARAM_ID_GAIN",
                         PARAM_ID_GAIN}
    @h2xmlp_description {Configures the gain}
    @h2xmlp_toolPolicy  {Calibration; RTC} */

#include "spf_begin_pack.h"
/** @ingroup ar_spf_mod_gain_macros
    Payload for parameter param_id_gain_cfg_t. */
struct param_id_gain_cfg_t
{
      uint16_t gain;
      /**< Linear gain (in Q13 format). */
      /**< @h2xmle_description {Linear gain (in Q13 format)}
           @h2xmle_dataFormat  {Q13}
           @h2xmle_displayType {dbtextbox} 
           @h2xmle_default     {0x2000} */

	   uint16_t reserved;
       /**< Reserved. Clients must set this field to 0. */
       /**< @h2xmle_description {Clients must set this field to 0.}
	        @h2xmle_rangeList   {"0"=0}
	        @h2xmle_visibility	{hide} */
}
#include "spf_end_pack.h"
;
/* Structure type def for above payload. */
typedef struct param_id_gain_cfg_t param_id_gain_cfg_t;

/**  @}                   <-- End of the Module -->*/

#endif //_GAIN_API_H_
