#ifndef _DUTY_CYCLING_BUF_MODULE_H
#define _DUTY_CYCLING_BUF_MODULE_H
/**
 * \file duty_cycling_buf_api.h
 * \brief 
 *  	 API file for duty cycling buffering Module
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "apm_graph_properties.h"
#include "module_cmn_api.h"

/*==============================================================================
   Constants
==============================================================================*/

/* Input port ID of DUTY_CYCLING_BUF */
#define DUTY_CYCLING_BUF_DATA_INPUT_PORT   0x2

/* Output port ID of DUTY_CYCLING_BUF */
#define DUTY_CYCLING_BUF_DATA_OUTPUT_PORT  0x1

/* Max number of input ports of DUTY_CYCLING_BUF */
#define DUTY_CYCLING_BUF_DATA_MAX_INPUT_PORTS 0x1

/* Max number of output ports of DUTY_CYCLING_BUF */
#define DUTY_CYCLING_BUF_DATA_MAX_OUTPUT_PORTS 0x1

/* Stack size of DUTY_CYCLING_BUF */
#define DUTY_CYCLING_BUF_STACK_SIZE 2048


/*==============================================================================
   Param ID
==============================================================================*/

/*==============================================================================
   Module
==============================================================================*/



#define PARAM_ID_DUTY_CYCLING_BUF_CONFIG                 0x08001376

/** @h2xmlp_parameter   {"PARAM_ID_DUTY_CYCLING_BUF_CONFIG",
                          PARAM_ID_DUTY_CYCLING_BUF_CONFIG}
    @h2xmlp_description { This parameter is sent to the buffering module to configure buffer sizes
                      .   This parameter also helps in deciding island entry and exiting criteria}
    @h2xmlp_toolPolicy  { Calibration } */

#include "spf_begin_pack.h"
struct param_id_duty_cycling_buf_config
{
   uint32_t buffer_size_in_ms;
   /**< @h2xmle_description { Size of internal buffer in milliseconds. Value is dependent on duty cycling period 
                             (i.e., how long module intends to stay in island and non-island). For fractional 
							  sampling rate, calculation would lead to the buffer size slightly lower than 
							  the actual needed buffer size.For example, depending on the clock if system 
							  is in non-island for 20ms and in island for 80ms buffer size would be 100ms
							 }
        @h2xmle_default     { 0 }
        @h2xmle_range       { 0..0xFFFFFFFF }
        @h2xmle_policy      { Basic } */
   uint32_t lower_threshold_in_ms;
   /**< @h2xmle_description { Lower threshold decides when the module must exit island. 
                             This value must be tuned based on upstream processing time & downstream frame size, 
							 island exit time(for example say 1ms). Higher the value, lesser is the duration for 
							 which system stays in island. If value too low it may cause downstream (real-time) 
							 to underrun. For example, if upstream is 5ms and downstream is 1ms, recommended value 
							 is 5ms+1ms(island exit time). For 5ms upstream and 10ms downstream, recommended value 
							 is 10ms+1ms(island exit time)}
        @h2xmle_default     { 5 }
        @h2xmle_range       { 0..0xFFFFFFFF }
        @h2xmle_policy      { Basic } */
    
        
}
#include "spf_end_pack.h"
;


typedef struct param_id_duty_cycling_buf_config param_id_duty_cycling_buf_config;    

/** @h2xml_title1           {DUTY_CYCLING_BUF API}
    @h2xml_title_agile_rev  {DUTY_CYCLING_BUF API}
    @h2xml_title_date       {May 10, 2021} */

/**
 * Module ID for Duty cycling buffering module
 */
#define MODULE_ID_DUTY_CYCLING_BUF  0x070010DC

/**
    @h2xmlm_module              {"Duty cycling buffering module", MODULE_ID_DUTY_CYCLING_BUF}
    @h2xmlm_displayName         {"Duty cycling buffering module"}
	@h2xmlm_modSearchKeys	    {VoiceUI}
    @h2xmlm_description         { This module's main purpose is to buffer data inside the module to enable entry and exit from island }
    @h2xmlm_dataMaxInputPorts   {DUTY_CYCLING_BUF_DATA_MAX_INPUT_PORTS}
    @h2xmlm_dataMaxOutputPorts  {DUTY_CYCLING_BUF_DATA_MAX_OUTPUT_PORTS}
    @h2xmlm_dataInputPorts      {IN=DUTY_CYCLING_BUF_DATA_INPUT_PORT}
    @h2xmlm_dataOutputPorts     {OUT=DUTY_CYCLING_BUF_DATA_OUTPUT_PORT}
    @h2xmlm_supportedContTypes {APM_CONTAINER_TYPE_GC}
    @h2xmlm_isOffloadable       {false}
    @h2xmlm_stackSize           {DUTY_CYCLING_BUF_STACK_SIZE}
    @h2xmlm_toolPolicy          {Calibration}
    @{                          <-- Start of the Module -->
    @h2xml_Select      { param_id_module_enable_t }
    @h2xmlm_InsertParameter
    @h2xml_Select              {param_id_module_enable_t::enable}
    @h2xmle_default            {1}
    @h2xmlm_toolPolicy           {Calibration}
    
    @h2xml_Select      { param_id_duty_cycling_buf_config }
    @h2xmlm_InsertParameter
    @}                          <-- End of the Module -->
*/

#endif //_DUTY_CYCLING_BUF_MODULE_H
