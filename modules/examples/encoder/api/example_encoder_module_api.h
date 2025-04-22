#ifndef _EXAMPLE_ENCODER_MODULE_API_H_
#define _EXAMPLE_ENCODER_MODULE_API_H_
/**
 * \file example_encoder_module_api.c
 *  
 * \brief
 *  
 *     Example Encoder Module
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/*==============================================================================
   Include Files
==============================================================================*/

#include "media_fmt_api.h"
#include "common_enc_dec_api.h"

/** @h2xml_title1           {EXAMPLE Encoder Module API}
    @h2xml_title_agile_rev  {EXAMPLE Encoder Module API}
    @h2xml_title_date       {October 10, 2019} */

/*==============================================================================
   Constants
==============================================================================*/

/* Input port ID of EXAMPLE module */
#define EXAMPLE_ENC_MODULE_DATA_INPUT_PORT   0x2

/* Output port ID of EXAMPLE module */
#define EXAMPLE_ENC_MODULE_DATA_OUTPUT_PORT  0x1

/* Max number of input ports of EXAMPLE module */
#define EXAMPLE_ENC_MODULE_DATA_MAX_INPUT_PORTS 0x1

/* Max number of output ports of EXAMPLE module */
#define EXAMPLE_ENC_MODULE_DATA_MAX_OUTPUT_PORTS 0x1

/* Stack size of EXAMPLE module */
#define EXAMPLE_ENC_MODULE_STACK_SIZE 4096

/* Media format ID the example module encodes to*/
#define MEDIA_FMT_ID_EXAMPLE 1


/*==============================================================================
   Structs
==============================================================================*/
/* This struct is the payload following param_id_encoder_output_config_t  */
typedef struct example_enc_cfg_t example_enc_cfg_t;
/** @h2xmlp_subStruct
    @h2xmlp_description {Payload for configuring the example encoder module}
    @h2xmlp_toolPolicy  {Calibration} */

#include "spf_begin_pack.h"
struct example_enc_cfg_t
{
   uint32_t abc;
   /**<
        @h2xmle_description {abc}
        @h2xmle_rangeList   {"0"=0;
                             "1"=1;
                             "2"=2}
        @h2xmle_default     {0}
        @h2xmle_policy      {Basic} */

   uint32_t xyz;
   /**<
        @h2xmle_description {xyz}
        @h2xmle_range       {0..2}
        @h2xmle_default     {0}
        @h2xmle_policy      {Basic} */
}
#include "spf_end_pack.h"
;

/*==============================================================================
   Module
==============================================================================*/
/**
 * Module ID for EXAMPLE encoder.
 * Encodes to data of format-id = MEDIA_FMT_ID_EXAMPLE
 */
#define MODULE_ID_EXAMPLE_ENC 0x0700109B

/**
    @h2xmlm_module              {"MODULE_ID_EXAMPLE_ENC", MODULE_ID_EXAMPLE_ENC}
    @h2xmlm_displayName         {"Example - Encoder"}
    @h2xmlm_description         {This module is used as an example encoder.\n
                                 - This module has only one input and one output port.\n
                                 - Payload for param_id_encoder_output_config_t : example_enc_cfg_t\n
                                 - Encodes data to format-id = MEDIA_FMT_ID_EXAMPLE\n
                                 - Supports following params:\n
                                 -- PARAM_ID_ENCODER_OUTPUT_CONFIG\n
                                 -- PARAM_ID_ENC_BITRATE\n
                                 - Supported Input Media Format:\n
                                 -- Data Format : FIXED_POINT\n
                                 -- fmt_id : Don't care\n
                                 -- Sample Rates: \n
                                 -- Number of channels : 2\n
                                 -- Bit Width : 16\n
                                 -- Bits Per Sample : 16\n
                                 -- Q format : Q15\n
                                 -- Interleaving : interleaved\n}
    @h2xmlm_dataMaxInputPorts   {EXAMPLE_ENC_MODULE_DATA_MAX_INPUT_PORTS}
    @h2xmlm_dataMaxOutputPorts  {EXAMPLE_ENC_MODULE_DATA_MAX_OUTPUT_PORTS}
    @h2xmlm_dataInputPorts      {IN=EXAMPLE_ENC_MODULE_DATA_INPUT_PORT}
    @h2xmlm_dataOutputPorts     {OUT=EXAMPLE_ENC_MODULE_DATA_OUTPUT_PORT}
    @h2xmlm_supportedContTypes {APM_CONTAINER_TYPE_GC}
    @h2xmlm_stackSize           {EXAMPLE_ENC_MODULE_STACK_SIZE}
    @h2xmlm_toolPolicy          {Calibration}
    @{                          <-- Start of the Module -->
    @h2xml_Select               {"param_id_encoder_output_config_t"}
    @h2xmlm_InsertParameter
    @h2xml_Select               {param_id_encoder_output_config_t::fmt_id}
    @h2xmle_rangeList           {"Media format ID of EXAMPLE"=MEDIA_FMT_ID_EXAMPLE}
    @h2xmle_default             {0}
    @h2xml_Select               {"example_enc_cfg_t"}
    @h2xmlm_InsertStructure
    @h2xml_Select               {"param_id_enc_bitrate_param_t"}
    @h2xmlm_InsertParameter
    @}                          <-- End of the Module -->
*/

#endif //_EXAMPLE_ENCODER_MODULE_API_H_
