#ifndef _SMART_SYNC_API_H_
#define _SMART_SYNC_API_H_
/**
 * \file smart_sync_api.h
 * \brief 
 *  	 This file contains Smart Sync module APIs
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "ar_defs.h"
#include "apm_graph_properties.h"

/** @h2xml_title1          {Smart Sync Module API}
    @h2xml_title_agile_rev {Smart Sync Module API}
    @h2xml_title_date      {June 06, 2018}   */

/*==============================================================================
   Defines
==============================================================================*/

#define SMART_SYNC_MAX_IN_PORTS 2
#define SMART_SYNC_MAX_OUT_PORTS 2

#define SMART_SYNC_PRIMARY_IN_PORT_ID 0x2
#define SMART_SYNC_PRIMARY_OUT_PORT_ID 0x1

#define SMART_SYNC_SECONDARY_IN_PORT_ID 0x4
#define SMART_SYNC_SECONDARY_OUT_PORT_ID 0x3

#define SMART_SYNC_MAX_CHANNELS 8

#define SMART_SYNC_STACK_SIZE 4096

/*==============================================================================
   API definitions
==============================================================================*/

/*==============================================================================
   Param ID
==============================================================================*/

/* Custom param id to be sent by VCPM to set voice processing info of smart sync
 * module. Voice processing info includes voc proc start tick, proc start samples and
 * vfr cycle duration */
#define PARAM_ID_SMART_SYNC_VOICE_PROC_INFO            0x08001027

/*==============================================================================
   Type definitions
==============================================================================*/
/* Structure defined for above Property  */
#include "spf_begin_pack.h"
struct smart_sync_voice_proc_info_t
{
   uint32_t is_subscribe;
   /**< Flag to indicate whether to subscribe/unsubscribe with Voice Timer. */

   uint32_t voice_proc_start_offset_us;
   /**< voice processing start tick in microseconds */

   uint32_t voice_proc_start_samples_us;
   /**< minimum number of samples (in microseconds) required to start processing at
    *   voice processing start tick */

   uint32_t vfr_cycle_duration_ms;
   /**< vfr cycle duration in milliseconds
        @values : {20, 40} */

   uint32_t vsid;
   /* TODO(CG): add comments for vsid and vfr mode */

   uint32_t vfr_mode;
   
   uint32_t path_delay_us;
}
#include "spf_end_pack.h"
;

typedef struct smart_sync_voice_proc_info_t smart_sync_voice_proc_info_t;

#define MODULE_ID_SMART_SYNC 0x0700103C
/** @h2xmlm_module            {"MODULE_ID_SMART_SYNC", MODULE_ID_SMART_SYNC}
  @h2xmlm_displayName         {"Smart Sync"}
  @h2xmlm_modSearchKeys       {Voice}
  @h2xmlm_description         {- Voice call Synchronziation Module\n
                               - Supported Input Media Format: \n
                               - Data Format          : FIXED_POINT, \n
                               - fmt_id               : MEDIA_FMT_ID_PCM \n
                               - Sample Rates         : 8, 11.025, 12, 16, 22.05, 24, 32, 44.1, 48, \n
                               -                        88.2, 96, 176.4, 192, 352.8, 384 kHz \n
                               - Number of channels   : 1 to 8 \n
                               - Channel type         : Don't care \n
                               - Bit Width            : 16 (bits per sample 16 and Q15), \n
                               -                      : 32 (bits per sample 32 and Q31)  \n
                               - Q format             : Q15, Q31 \n
                               - Interleaving         : Deinterleaved unpacked \n
                               - Endianess            : little, big \n
                               - Primary and secondary ports are allowed to have different media formats. The only \n
                               - restriction is that if one port runs at a fractional rate, the other port must also \n
                               - run at a fractional rate (use cases where ports have variable input frame sizes are \n
                               - not supported).}

    @h2xmlm_dataMaxInputPorts   {SMART_SYNC_MAX_IN_PORTS}
    @h2xmlm_dataInputPorts      {PRIMARY_IN = SMART_SYNC_PRIMARY_IN_PORT_ID;
                                 SECONDARY_IN = SMART_SYNC_SECONDARY_IN_PORT_ID}
    @h2xmlm_dataMaxOutputPorts  {SMART_SYNC_MAX_OUT_PORTS}
    @h2xmlm_dataOutputPorts     {PRIMARY_OUT = SMART_SYNC_PRIMARY_OUT_PORT_ID;
                                 SECONDARY_OUT = SMART_SYNC_SECONDARY_OUT_PORT_ID}

    @h2xmlm_supportedContTypes  {APM_CONTAINER_TYPE_SC}
    @h2xmlm_isOffloadable        {false}
    @h2xmlm_stackSize           { SMART_SYNC_STACK_SIZE }
    @{                     <-- Start of the Module -->
    @}                     <-- End of the Module -->
*/

#endif //_SMART_SYNC_API_H_
