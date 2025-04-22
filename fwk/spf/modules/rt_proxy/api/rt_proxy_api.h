#ifndef __RT_PROXY_H
#define __RT_PROXY_H
/**
 * \file rt_proxy_api.h
 * \brief 
 *  	 This file contains module published by RT Proxy CAPI intialization.
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "ar_defs.h"
#include "module_cmn_api.h"
#include "imcl_fwk_intent_api.h"

/**     
  @h2xml_title1          {Real Time Proxy Module}
  @h2xml_title_agile_rev {Real Time Proxy Module}
  @h2xml_title_date      {Jun 4, 2019}
*/

/** @ingroup ar_spf_mod_rt_proxy_macros
    Maximum number of input ports supported by the RT Proxy module. */
#define CAPI_RT_PROXY_MAX_INPUT_PORTS         (1)

/** @ingroup ar_spf_mod_rt_proxy_macros
    Maximum number of output ports supported by the RT Proxy module. */
#define CAPI_RT_PROXY_MAX_OUTPUT_PORTS        (1)

/** @ingroup ar_spf_mod_rt_proxy_macros
    Maximum number of control ports supported by the RT Proxy module. */
#define CAPI_RT_PROXY_MAX_CTRL_PORTS			 (1)

/*==============================================================================
   Constants
==============================================================================*/

/** @ingroup ar_spf_mod_rt_proxy_macros
    Configuration for the RT Proxy module. */
#define PARAM_ID_RT_PROXY_CONFIG         0x08001157

/*==============================================================================
   Type Definitions
==============================================================================*/


/** @h2xmlp_parameter   {"PARAM_ID_RT_PROXY_CONFIG",
                          PARAM_ID_RT_PROXY_CONFIG}
    @h2xmlp_description { This param ID is used to configure RT proxy module. } */
#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"

/** @ingroup ar_spf_mod_rt_proxy_macros
    Configures the RT proxy module. */
struct param_id_rt_proxy_config_t
{
   uint32_t jitter_allowance_in_ms;
   /**< It defines the range of jitter for client buffers.
        Range is considered as +/- jitter_allowance_in_ms. */

   /**< @h2xmle_description { It defines the range of jitter for client buffers.
                              Rang is considered as +/- jitter_allowance_in_ms }
        @h2xmle_default     { 1 } */

   uint32_t client_frame_size_in_ms;
   /**< Read/Write client buffer frame size. */

   /**< @h2xmle_description { Its read/write client buffer frame size.}
        @h2xmle_default     { 0 } */

   uint32_t low_watermark_in_ms;
   /**< Low watermark event is sent to the client if the buffer size
        reaches less than this threshold. */

   /**< @h2xmle_description { Low watermark event is sent to the client if the buffer size
                              reaches less than this threshold. }
        @h2xmle_default     { 0 } */

   uint32_t high_watermark_in_ms;
   /**< High watermark event is sent to the client if the buffer size
        crosses this threshold. */

   /**< @h2xmle_description { High watermark event is sent to the client if the buffer size
                              crosses this threshold. }
        @h2xmle_default     { 0xFFFFFFFF } */
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;

/* Structure to define the RT proxy configuration.*/
typedef struct param_id_rt_proxy_config_t param_id_rt_proxy_config_t;

/*------------------------------------------------------------------------------
   Module
------------------------------------------------------------------------------*/

/** @ingroup ar_spf_mod_rt_proxy_macros
    This module acts a bridge RT source and RT sink. */
#define MODULE_ID_RT_PROXY_RX                     0x07001065

/**
    @h2xmlm_module         {"MODULE_ID_RT_PROXY_RX", MODULE_ID_RT_PROXY_RX}
    @h2xmlm_displayName    {"Real Time Proxy RX"}

    @h2xmlm_description    { This module acts a bridge RT source and RT sink. }
    @h2xmlm_dataMaxInputPorts    { 1 }
    @h2xmlm_dataMaxOutputPorts   { 1 }
    @h2xmlm_ctrlDynamicPortIntent  { "RT proxy - RAT" = INTENT_ID_TIMER_DRIFT_INFO,
								   	   maxPorts= 1 }
    @h2xmlm_supportedContTypes  { APM_CONTAINER_TYPE_GC }
    @h2xmlm_isOffloadable        {false}
    @h2xmlm_stackSize            { 1024 }
    @{                     <-- Start of the Module -->

    @h2xml_Select      {param_id_rt_proxy_config_t}
    @h2xmlm_InsertParameter

 @}                     <-- End of the Module --> */

/*------------------------------------------------------------------------------
   Module
------------------------------------------------------------------------------*/

/** @ingroup ar_spf_mod_rt_proxy_macros
    This module acts a bridge RT client sink and RT source on DSP. */
#define MODULE_ID_RT_PROXY_TX                     0x0700106B

/**
    @h2xmlm_module         {"MODULE_ID_RT_PROXY_TX", MODULE_ID_RT_PROXY_TX}
    @h2xmlm_displayName    {"Real Time Proxy TX"}

    @h2xmlm_description    { This module acts a bridge RT client sink and RT source on DSP. }
    @h2xmlm_dataMaxInputPorts    { 1 }
    @h2xmlm_dataMaxOutputPorts   { 1 }
    @h2xmlm_ctrlDynamicPortIntent  { "RT proxy - RAT" = INTENT_ID_TIMER_DRIFT_INFO,
								   	   maxPorts= 1 }
    @h2xmlm_supportedContTypes  { APM_CONTAINER_TYPE_GC }
    @h2xmlm_stackSize            { 1024 }
    @{                     <-- Start of the Module -->

    @h2xml_Select      {param_id_rt_proxy_config_t}
    @h2xmlm_InsertParameter

 @}                     <-- End of the Module --> */

#endif /*__RT_PROXY_H*/
