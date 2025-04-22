/**
 * \file sh_mem_push_lab_api.h
 * \brief
 *  	 This file contains CAPI push lab module APIs
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef _PUSH_LAB_API_H_
#define _PUSH_LAB_API_H_

/*------------------------------------------------------------------------
 * Include files
 * -----------------------------------------------------------------------*/
#include "ar_defs.h"
#include "apm_container_api.h"
#include "media_fmt_api.h"
#include "imcl_fwk_intent_api.h"

/**
     @h2xml_title1          {Shared memory push lab module}
     @h2xml_title_agile_rev {Shared memory push lab module}
     @h2xml_title_date      {Dec 9, 2020}
  */
// TODO: check if required
#define SH_MEM_PUSH_LAB_MAX_INPUT_PORTS 0x1

#define SH_MEM_PUSH_LAB_MAX_OUTPUT_PORTS 0x1

#define SH_MEM_PUSH_LAB_STACK_SIZE_REQUIREMENT 4096

/**
   @h2xmlx_xmlNumberFormat {int}
*/

/*==============================================================================
   Event ID
==============================================================================*/

/**
 * Watermark event ID from push lab module to the client.
 *
 * Payload of the configuration in APM_CMD_REGISTER_MODULE_EVENTS: event_cfg_sh_mem_push_lab_watermark_level_t
 * Payload of the event APM_EVENT_MODULE_TO_CLIENT: event_sh_mem_push_lab_watermark_level_t
 */
#define EVENT_ID_SH_MEM_PUSH_LAB_WATERMARK 0x0800130E

/*==============================================================================
     Event ID
  ==============================================================================*/

  /** @h2xmlp_parameter   {"EVENT_ID_SH_MEM_PUSH_LAB_WATERMARK", EVENT_ID_SH_MEM_PUSH_LAB_WATERMARK}
    @h2xmlp_description {Periodic event sent from module to client to inform about write data
					   location in the Shared buffer}
    @h2xmlp_toolPolicy  {NO_SUPPORT} */

#include "spf_begin_pack.h"

struct event_cfg_sh_mem_push_lab_watermark_level_t
{

   uint32_t start_index;
   /**< @h2xmle_description {Start index of Keyword}
        @h2xmle_policy      {Basic}
        @h2xmle_default     {0}
        @h2xmle_range       { 0..0xFFFFFFFF }
        @h2xmle_policy      {Basic}*/

   uint32_t current_write_position_index;
   /**< @h2xmle_description {Current Write position index of Keyword}
        @h2xmle_policy      {Basic}
        @h2xmle_default     {0}
        @h2xmle_range       { 0..0xFFFFFFFF }
        @h2xmle_policy      {Basic}*/
}

#include "spf_end_pack.h"
;

/* Structure type def for above payload. */
typedef struct event_cfg_sh_mem_push_lab_watermark_level_t event_cfg_sh_mem_push_lab_watermark_level_t;

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
struct event_cfg_sh_mem_push_lab_watermark_t
{
   uint32_t start_index;
   /**< @h2xmle_description {Start index of Keyword}
        @h2xmle_policy      {Basic}
        @h2xmle_default     {0}
        @h2xmle_range       { 0..0xFFFFFFFF }
        @h2xmle_policy      {Basic}*/

   uint32_t current_write_position_index;
   /**< @h2xmle_description {Current Write position index}
        @h2xmle_policy      {Basic}
        @h2xmle_default     {0}
        @h2xmle_range       { 0..0xFFFFFFFF }
        @h2xmle_policy      {Basic}*/
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;

/* Structure type def for above payload. */
typedef struct event_cfg_sh_mem_push_lab_watermark_t event_cfg_sh_mem_push_lab_watermark_t;

/*==============================================================================
   Param ID
==============================================================================*/

/* Parameter id to be used to configure shared mem info for push lab module.

   Before issuing this set-param, APM_CMD_SHARED_MEM_MAP_REGIONS must be
   issued and memories for shared circular buffer must be mapped.

   Shared circular buffer must be mapped as uncached.

   Payload struct sh_mem_push_lab_cfg_t
 */
#define PARAM_ID_SH_MEM_PUSH_LAB_CFG 0x0800130C

/*==============================================================================
   Param structure defintions
==============================================================================*/

/** @h2xmlp_parameter   {"PARAM_ID_SH_MEM_PUSH_LAB_CFG", PARAM_ID_SH_MEM_PUSH_LAB_CFG}
    @h2xmlp_description {Parameter used to configure shared circular buffer for the module.}
    @h2xmlp_toolPolicy  {NO_SUPPORT} */

#include "spf_begin_pack.h"
struct sh_mem_push_lab_cfg_t
{
   uint32_t shared_circ_buf_addr_lsw;
   /**< @h2xmle_description {Lower 32 bits of the address of the shared circular buffer.}
        @h2xmle_default     {0}
        @h2xmle_range       { 0..0xFFFFFFFF }
        @h2xmle_policy      {Basic} */

   uint32_t shared_circ_buf_addr_msw;
   /**< @h2xmle_description {Upper 32 bits of the address of the shared circular buffer.}
        @h2xmle_default     {0}
        @h2xmle_range       { 0..0xFFFFFFFF }
        @h2xmle_policy      {Basic} */

   uint32_t shared_circ_buf_size;
   /**< @h2xmle_description {Number of bytes in the shared circular buffer.
                             The value must be an integral multiple of the number of (bits per sample * number of
      channels)}
        @h2xmle_default     {0}
        @h2xmle_range       { 0..0xFFFFFFFF }
        @h2xmle_policy      {Basic} */

   uint32_t circ_buf_mem_map_handle;
   /**< @h2xmle_description {Unique identifier for the shared memory address of shared circular buffer.
                             The spf returns this handle through #APM_CMD_RSP_SHARED_MEM_MAP_REGIONS.}
        @h2xmle_default     {0}
        @h2xmle_range       { 0..0xFFFFFFFF }
        @h2xmle_policy      {Basic} */
}
#include "spf_end_pack.h"
;

/* Structure type def for above payload. */
typedef struct sh_mem_push_lab_cfg_t sh_mem_push_lab_cfg_t;

/* Parameter id to be used to configure watermark event period for push lab module.

   Payload struct sh_mem_push_lab_watermark_period_t
 */
#define PARAM_ID_SH_MEM_PUSH_LAB_WATERMARK_PERIOD 0x0800130D

/*==============================================================================
   Param structure defintions
==============================================================================*/

/** @h2xmlp_parameter   {"PARAM_ID_SH_MEM_PUSH_LAB_WATERMARK_PERIOD", PARAM_ID_SH_MEM_PUSH_LAB_WATERMARK_PERIOD}
    @h2xmlp_description {Parameter used to configure watermark event interval for the module.}
    @h2xmlp_toolPolicy  {Calibration} */

#include "spf_begin_pack.h"
struct sh_mem_push_lab_watermark_period_t
{
   uint32_t watermark_period_in_ms;
   /**< @h2xmle_description {Gives the interval for watermark period in ms.}
        @h2xmle_default     {20}
        @h2xmle_range       { 0..0xFFFFFFFF }
        @h2xmle_policy      {Basic} */
}
#include "spf_end_pack.h"
;

/* Structure type def for above payload. */
typedef struct sh_mem_push_lab_watermark_period_t sh_mem_push_lab_watermark_period_t;

/*
 * ID of the Shared Memory Push Lab Module
 *
 * Supported Input Media Format:
 *  - Data Format          : FIXED_POINT
 *  - fmt_id               : Don't care
 *  - Sample Rates         : 1-384 kHz
 *  - Number of channels   : 1-32
 *  - Bit Width            : 16 (bits per sample 16 and Q15),
 *                         : 24 (bits per sample 24 and Q23, bits per sample 32 and Q23 or Q27 or Q31),
 *                         : 32 (bits per sample 32 and Q31)
 *  - Interleaving         : interleaved, deinterleaved unpacked, deinterleaved packed.
 *  - Endianess            : little, big
 */
#define MODULE_ID_SH_MEM_PUSH_LAB 0x070010D0
/**
    @h2xmlm_module         {"MODULE_ID_SH_MEM_PUSH_LAB", MODULE_ID_SH_MEM_PUSH_LAB}
    @h2xmlm_displayName    {"Shared Memory Push Lab"}
    @h2xmlm_description    {
                            This module is used to read data from spf to the host through shared circular buffer
						    mechanism in voice activation usecases. This module has only one static input port
						    with ID 2 and one output port. Push Lab module pushes the data from upstream to the
						    shared buffer and upon detection event notification from detection engine, sends
						    watermark events to client at regular configurable intervals indicating start of
						    keyword and current write position in the buffer. On Gate close intimation from
						    Voice Wakeup, detection module stops sending watermark events to client. It can be
						    used both as an intermediate module or a sink module. \n
							 * Supported Input Media Format	\n
							 *  - Data Format: FIXED_POINT	\n
							 *  - fmt_id: Don't care	\n
							 *  - Sample Rates: 1-384 kHz	\n
							 *  - No. of channels: 1-32	\n
							 *  - Bit Width: 16, 24, 32	\n
							 *  - Interleaving: interleaved, deinterleaved unpacked, deinterleaved packed.	\n
							 *  - Endianess: little, big	\n
							 *  - Signed/unsigned: Any	\n
                            }
    @h2xmlm_dataMaxInputPorts    {SH_MEM_PUSH_LAB_MAX_INPUT_PORTS}
    @h2xmlm_dataInputPorts       {IN = 2}
    @h2xmlm_dataMaxOutputPorts   {SH_MEM_PUSH_LAB_MAX_OUTPUT_PORTS}
    @h2xmlm_supportedContTypes  { APM_CONTAINER_TYPE_WC,APM_CONTAINER_TYPE_GC}
    @h2xmlm_stackSize            { SH_MEM_PUSH_LAB_STACK_SIZE_REQUIREMENT }
    @h2xmlm_ctrlDynamicPortIntent   { "DAM-DE Control" = INTENT_ID_AUDIO_DAM_DETECTION_ENGINE_CTRL,
                                      maxPorts= 1 }
    @h2xmlm_toolPolicy     {Calibration}
    @{                     <-- Start of the Module -->

    @h2xml_Select          {sh_mem_push_lab_cfg_t}
    @h2xmlm_InsertParameter
    @h2xml_Select          {sh_mem_push_lab_watermark_period_t}
    @h2xmlm_InsertParameter
    @h2xml_Select          {event_cfg_sh_mem_push_lab_watermark_level_t}
    @h2xmlm_InsertParameter
    @}                     <-- End of the Module -->
*/

#endif // _PUSH_LAB_API_H_
