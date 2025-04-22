
/**
 * \file demuxer_api.h
 * \brief 
 *  	 demuxer_api.h: This file contains the Module Id, Param IDs and configuration
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef DEMUXER_API_H
#define DEMUXER_API_H

 /*------------------------------------------------------------------------
 * Include files
 * -----------------------------------------------------------------------*/

#include "module_cmn_api.h"
#define DEMUXER_MAX_INPUT_PORTS    1
#define CAPI_DEMUXER_STACK_SIZE  512

/**
   @h2xmlx_xmlNumberFormat {int}
*/

 
/*==============================================================================
   Param ID
==============================================================================*/

#define PARAM_ID_DEMUXER_OUT_CONFIG 0x08001373

/*==============================================================================
   Param structure defintions
==============================================================================*/
/** @h2xmlp_subStruct
    @h2xmlp_description  { This structure sets channel indices from input which has to be taken for corresponding output port. }
    */

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
struct demuxer_out_config_t
{
   uint32_t output_port_id;
   /**< @h2xmle_description {output port id.}
    */

   uint32_t num_channels;
/**< @h2xmle_description {number of channels in the corresponding output port}
     @h2xmle_default     {1}
     @h2xmle_range       {1..MODULE_CMN_MAX_CHANNEL} 
   */
#ifdef __H2XML__
   uint16_t input_channel_index[0];
/**< @h2xmle_description   {channel index corresponding to input from which output has to be taken}
     @h2xmlx_expandStructs {true}
     @h2xmle_variableArraySize  { num_channels}
     @h2xmle_default     {0}
 */
#endif

#ifdef __H2XML__
   uint16_t channel_type[0];
/**< @h2xmle_description   {channel type to be raised in output media format corresponding to each channel ino utput port}
     @h2xmlx_expandStructs {true}
     @h2xmle_variableArraySize	{ num_channels}
     @h2xmle_default	 {0}
 */
#endif
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;

typedef struct demuxer_out_config_t demuxer_out_config_t;

/** @h2xmlp_parameter   {"PARAM_ID_DEMUXER_OUT_CONFIG", PARAM_ID_DEMUXER_OUT_CONFIG}
    @h2xmlp_description {This parameter is used to know how
                           to de-mux/split the input and send to output}
    @h2xmlp_toolPolicy  {Calibration} */

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
struct param_id_demuxer_out_config_t
{
   uint32_t num_out_ports;
/**< @h2xmle_description {Number of output ports of the for which configuration
                      is sent }
 */
#ifdef __H2XML__
   demuxer_out_config_t out_cfg_arr[0];
/**< @h2xmle_description  		{An array of output port configuration}
     @h2xmlx_expandStructs 		{false}
     @h2xmle_variableArraySize  	{num_out_ports}
 */
#endif
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;

typedef struct param_id_demuxer_out_config_t param_id_demuxer_out_config_t;
 
#define MODULE_ID_DEMUXER 0x070010DB

/** @h2xml_title1          {Module demuxer}
    @h2xml_title_agile_rev {Module demuxer}
     @h2xml_title_date     {May 20, 2021}
  */

/**
    @h2xmlm_module       {"MODULE_ID_DEMUXER",MODULE_ID_DEMUXER}
    @h2xmlm_displayName  {" Demuxer"}
    @h2xmlm_description  {ID of the DEMUX module. This module is used to split/demux the
    streams on the input to all the output ports based on configuration \n
   }
    @h2xmlm_dataMaxInputPorts    {DEMUXER_MAX_INPUT_PORTS}
    @h2xmlm_dataMaxOutputPorts   {INFINITE}
    @h2xmlm_dataInputPorts       {IN=2}
    @h2xmlm_supportedContTypes   {APM_CONTAINER_TYPE_WC,APM_CONTAINER_TYPE_GC}
    @h2xmlm_isOffloadable        {false}
    @h2xmlm_stackSize            {CAPI_DEMUXER_STACK_SIZE}
    @h2xmlm_ToolPolicy           {Calibration}    

	 @{                   <-- Start of the Module -->
    @h2xml_Select					{param_id_demuxer_out_config_t}
    @h2xmlm_InsertParameter
    @h2xml_Select					{demuxer_out_config_t}
    @h2xmlm_InsertParameter
    @}                   <-- End of the Module -->*/


#endif /* DEMUXER_API_H */
