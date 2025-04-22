#ifndef __RTM_LOGGING_API_H__
#define __RTM_LOGGING_API_H__
/**
 * \file rtm_logging_api.h
 * \brief 
 *  	 This file contains audio rtm logging parameters.
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "ar_defs.h"

/*==============================================================================
   Param ID
==============================================================================*/

#define PARAM_ID_RTM_LOGGING_ENABLE                 0x080010A6

/*==============================================================================
   Param structure defintions
==============================================================================*/

/** @h2xmlp_parameter   {"PARAM_ID_RTM_LOGGING_ENABLE", PARAM_ID_RTM_LOGGING_ENABLE}
    @h2xmlp_description {Parameter used to enable/disable RTM logging in audio modules.}
    @h2xmlp_toolPolicy  {Calibration; RTC} */
	
#include "spf_begin_pack.h"
struct rtm_logging_enable_payload_t
{
   uint32_t enable_rtm_logging;
   /**< @h2xmle_description  {Param to enable RTM logging for any Audio module}
        @h2xmle_rangeList    {"Disable" = 0;
                              "Enable"  = 1}
        @h2xmle_policy       {Basic}
        @h2xmle_default      {0} */

   uint32_t log_code;
   /**< @h2xmle_description {logging code}
        @h2xmle_default     {0x184B}
        @h2xmle_rangeList   {"VoiceProc_RTM_Log_Code" = 0x184B;
                             "AudProc_RTM_Log_Code"  = 0x19B3}
        @h2xmle_policy      {Basic} */
}
#include "spf_end_pack.h"
;
/* Type definition for the above structure.*/
typedef struct rtm_logging_enable_payload_t rtm_logging_enable_payload_t;

/* Currently no RTM header is logged as part of RTM data blob, hence a default value is
 * used for RTM header version. */
#define RTM_HEADER_VERSION_0 0

#include "spf_begin_pack.h"
struct rtm_logging_header_t
{
   uint32_t version;
   /**< @h2xmle_description {version of RTM header}
         @h2xmle_default     {0x0}
         @h2xmle_range       {0..4294967295}  */
}
#include "spf_end_pack.h"
;
/* Type definition for the above structure.*/
typedef struct rtm_logging_header_t rtm_logging_header_t;

/* Immediately following this structure is the rtm logging param data struct which consists of module id, param id ,
 * param size and reserved fields.*/

/** @h2xmlp_description { Common structure used for RTM logging of all monitoring parameters}
    @h2xmlp_toolPolicy  {RTM}
    @h2xmlp_readOnly    {true}*/
#include "spf_begin_pack.h"
struct rtm_logging_param_data_t
{
   uint32_t module_instance_id;
   /**< @h2xmle_description {instance ID of the module that logs the RTM data}
         @h2xmle_default     {0x0}
         @h2xmle_range       {0..4294967295}  */

   uint32_t param_id;
   /**< @h2xmle_description {param ID of the parameter that is being logged}
         @h2xmle_default     {0x0}
         @h2xmle_range       {0..4294967295}  */

   uint16_t param_size;
   /**< @h2xmle_description {size of the param data which is followed by this structure}
         @h2xmle_default     {0x0}
         @h2xmle_range       {0..65536}  */

   uint16_t reserved;
   /**< @h2xmle_description {reserved parameter}
         @h2xmle_default     {0x0}
         @h2xmle_range       {0..65536}  */
}
#include "spf_end_pack.h"
;
/* Type definition for the above structure.*/
typedef struct rtm_logging_param_data_t rtm_logging_param_data_t;

/*==============================================================================
   Param ID
==============================================================================*/

#define PARAM_ID_RTM_PCM_LOGGING                     0x080010A7

/*==============================================================================
   Param structure defintions
==============================================================================*/

/** @h2xmlp_description { To query RTM PCM logging.\n
                          Payload size of this parameter is dynamic and depends on pcm_size.
                          The header size for version 0 param is 48 bytes \n.
                          payload size -- (48 +  pcm_size) bytes}
    @h2xmlp_toolPolicy  {RTM}
    @h2xmlp_readOnly    {true}*/
#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
struct rtm_pcm_logging_payload_t
{

    uint32_t                      pcm_size;
	/**< @h2xmle_description {total size of PCM data in Bytes}
         @h2xmle_default     {0x0}
         @h2xmle_range       {0..4294967295}  */

    uint32_t                      log_tap_id;
	/**< @h2xmle_description {tap ID for PCM data}
         @h2xmle_default     {0x0}
         @h2xmle_range       {0..4294967295}  */

    uint32_t                      sampling_rate;
	/**< @h2xmle_description {sampling rate of the PCM data}
         @h2xmle_default     {0x1f40}
         @h2xmle_range       {0..384000}  */

    uint16_t                      num_channels;
	/**< @h2xmle_description {Number of channels in the PCM stream getting logged}
         @h2xmle_default     {0x1}
         @h2xmle_range       {1..32}  */

    uint8_t                       pcm_width;
	/**< @h2xmle_description {pcm width of the PCM data}
         @h2xmle_rangeList    {"bitwidth 16bit"=16;
                               "bitwidth 24bit"=24;
                              "bitwidth 24bit"=32}
         @h2xmle_default     {0x10}  */

    uint8_t                       interleaved;
	/**< @h2xmle_description {specifies whether PCM data packing is interleaved or deinterleaved}
         @h2xmle_rangeList    {"INTERLEAVED"=0;
                               "DEINTERLEAVED_PACKED"=1;
                              "DEINTERLEAVED_UNPACKED"=2}
         @h2xmle_default     {0x0}  */

    uint8_t                       channel_map[32];
	/**< @h2xmle_description {Channel map of each channel in the PCM stream getting logged}
         @h2xmle_default     {0x1}
         @h2xmle_range       {1..63}  */
#if defined(__H2XML__)
    int8_t                         payload[0];
	/**< @h2xmle_description {Array of PCM data}

		 @h2xmle_variableArraySize  {"pcm_size"}
		 @h2xmle_policy       {Basic} */
#endif

}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;
/* Type definition for the above structure.*/
typedef struct rtm_pcm_logging_payload_t rtm_pcm_logging_payload_t;

#endif  // __RTM_LOGGING_PARAM_CALIB_H__
