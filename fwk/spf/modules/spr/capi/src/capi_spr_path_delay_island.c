/*==========================================================================
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 * =========================================================================*/

/**
 *   \file capi_spr_path_delay.c
 *   \brief
 *        This file contains CAPI implementation of SPR path delay
 */

#include "capi_spr_i.h"
#include "capi_fwk_extns_signal_triggered_module.h"
#include "capi_intf_extn_path_delay.h"
#include "imcl_fwk_intent_api.h"

/*------------------------------------------------------------------------------
  Function name: spr_aggregate_path_delay
   Aggregates the path delay for the given SPR output port. Invoked per process
   call before render decision and when the path delay response is received.
 * ------------------------------------------------------------------------------*/
uint32_t spr_aggregate_path_delay(capi_spr_t *me_ptr, spr_output_port_info_t *out_port_info_ptr)
{
   if (!out_port_info_ptr->path_delay.delay_us_pptr)
   {
      return 0;
   }

   uint32_t delay = 0;
   for (uint32_t i = 0; i < out_port_info_ptr->path_delay.num_delay_ptrs; i++)
   {
      if (out_port_info_ptr->path_delay.delay_us_pptr[i])
      {
         delay += *(out_port_info_ptr->path_delay.delay_us_pptr[i]);
      }
   }

#ifdef DEBUG_SPR_MODULE
   SPR_MSG_ISLAND(me_ptr->miid,
           DBG_LOW_PRIO,
           "PATH_DELAY: aggregated delay out port %lu is %lu us",
           out_port_info_ptr->port_id,
           delay);
#endif

   bool_t OUTPUT = FALSE;
   // Compare primary array index with output port array index
   if (me_ptr->primary_output_arr_idx ==
       spr_get_arr_index_from_port_index(me_ptr, out_port_info_ptr->port_index, OUTPUT))
   {
      spr_avsync_set_ds_delay(me_ptr->avsync_ptr, delay);
   }

   return delay;
}
