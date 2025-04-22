/**
 * \file capi_gapless_event_utils.c
 * \brief
 *     Implementation of utility functions for capi event handling (kpps, bandwidth, any other events, etc).
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "capi_gapless_i.h"

/**
 * Function to get KPPS numbers.
 */
static uint32_t gapless_get_kpps(capi_gapless_t *me_ptr)
{
   return GAPLESS_KPPS;
}

/**
 * Function to raise various events of the gapless module.
 */
capi_err_t gapless_raise_event(capi_gapless_t *me_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   if (NULL == me_ptr->cb_info.event_cb)
   {
      AR_MSG(DBG_ERROR_PRIO, "Event callback is not set. Unable to raise events!");
      CAPI_SET_ERROR(capi_result, CAPI_EUNSUPPORTED);
      return capi_result;
   }

   // The gapless module can't be disabled.
   me_ptr->events_config.enable = TRUE;
   me_ptr->events_config.kpps   = gapless_get_kpps(me_ptr);

   capi_result |= capi_cmn_update_kpps_event(&me_ptr->cb_info, me_ptr->events_config.kpps);
   capi_result |=
      capi_cmn_update_bandwidth_event(&me_ptr->cb_info, me_ptr->events_config.code_bw, me_ptr->events_config.data_bw);
   capi_result |= capi_cmn_update_algo_delay_event(&me_ptr->cb_info, me_ptr->events_config.delay_in_us);
   capi_result |= gapless_raise_process_event(me_ptr);
   return capi_result;
}

/**
 * Gapless module is always enabled, so this should at most raise enable once and otherwise do nothing.
 */
capi_err_t gapless_raise_process_event(capi_gapless_t *me_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   uint32_t   enable      = TRUE;

   if (me_ptr->events_config.enable != enable)
   {
      capi_result = capi_cmn_update_process_check_event(&me_ptr->cb_info, enable);
      if (CAPI_EOK == capi_result)
      {
         me_ptr->events_config.enable = enable;
      }
   }
   return capi_result;
}

/**
 * Raises the early eos event for this EOS metadata on this input port. Checks that we didn't already send
 * early EOS with this metadata before sending.
 *
 * Note that even in pass-through mode, we are raising early eos as long as the client is registered for
 * the early eos event. However it will be sent too late since there is no delay buffer created.
 */
capi_err_t gapless_raise_early_eos_event(capi_gapless_t *        me_ptr,
                                         capi_gapless_in_port_t *in_port_ptr,
                                         capi_stream_data_v2_t * in_sdata_ptr,
                                         module_cmn_md_t *       md_ptr)
{
   capi_err_t                         result = CAPI_EOK;
   capi_event_info_t                  event_info;
   capi_event_data_to_dsp_client_v2_t data_to_dsp_client_event;
   event_id_gapless_early_eos_event_t early_eos_event;
   uint32_t                           in_index = in_port_ptr->cmn.index;

   // Nothing to do if the client isn't registered.
   if (!(me_ptr->client_registered))
   {
      AR_MSG(DBG_MED_PRIO, "Not raising early eos event, client not regeistered!");
      return result;
   }

   // Sanity checks.
   if (NULL == me_ptr->cb_info.event_cb)
   {
      AR_MSG(DBG_ERROR_PRIO, "Event callback is not set. Unable to raise early eos event!");
      CAPI_SET_ERROR(result, CAPI_EUNSUPPORTED);
      return result;
   }

   if (MODULE_CMN_MD_ID_EOS != md_ptr->metadata_id)
   {
      AR_MSG(DBG_ERROR_PRIO, "Early Eos called for metadata with non-EOS md_id 0x%lx", md_ptr->metadata_id);
      return CAPI_EFAILED;
   }

   // TODO(claguna): Error if it's an internal EOS.

   // Get EOS metadata.
   uint32_t is_out_band = md_ptr->metadata_flag.is_out_of_band;

   module_cmn_md_eos_t *eos_metadata_ptr =
      is_out_band ? (module_cmn_md_eos_t *)md_ptr->metadata_ptr : (module_cmn_md_eos_t *)&(md_ptr->metadata_buf);

   // We wouldn't expect EOS on the non-active input port unless the file was shorter than the delay buffer. That isn't
   // a normal case.
   if (in_index != me_ptr->active_in_port_index)
   {
      AR_MSG(DBG_ERROR_PRIO, "Warning: EOS appeared on port which is not active.");
   }

   // Sanity checks on metadata structure.
   if ((!eos_metadata_ptr) || (!md_ptr->tracking_ptr))
   {
      AR_MSG(DBG_ERROR_PRIO, "Error: EOS md ptr 0x%lx or core ptr is NULL.", eos_metadata_ptr);
      return CAPI_EFAILED;
   }

   // src_module_iid = write shared memory end point module id, as stored in the EOS metadata.
   module_cmn_md_tracking_payload_t *tracking_ptr = (module_cmn_md_tracking_payload_t*)md_ptr->tracking_ptr;
   early_eos_event.src_module_iid = tracking_ptr->src_port;

   // Populate event fields and send the event.
   data_to_dsp_client_event.event_id                = EVENT_ID_EARLY_EOS;
   data_to_dsp_client_event.payload.actual_data_len = sizeof(event_id_gapless_early_eos_event_t);
   data_to_dsp_client_event.payload.max_data_len    = sizeof(event_id_gapless_early_eos_event_t);
   data_to_dsp_client_event.payload.data_ptr        = (int8_t *)&early_eos_event;
   data_to_dsp_client_event.dest_address            = me_ptr->event_dest_address;
   data_to_dsp_client_event.token                   = me_ptr->event_token;

   event_info.port_info.is_input_port = TRUE;
   event_info.port_info.is_valid      = TRUE;
   event_info.port_info.port_index    = in_index;

   event_info.payload.actual_data_len = sizeof(capi_event_data_to_dsp_client_v2_t);
   event_info.payload.data_ptr        = (int8_t *)&data_to_dsp_client_event;
   event_info.payload.max_data_len    = sizeof(capi_event_data_to_dsp_client_v2_t);

   result = me_ptr->cb_info.event_cb(me_ptr->cb_info.event_context, CAPI_EVENT_DATA_TO_DSP_CLIENT_V2, &event_info);
   if (CAPI_EOK != result)
   {
      AR_MSG(DBG_ERROR_PRIO, "Failed to raise early eos event.");
   }
   else
   {
      AR_MSG(DBG_HIGH_PRIO,
             "Raised early eos event on input port index %ld, src module id 0x%lx, metadata ptr 0x%lx.",
             in_index,
			 tracking_ptr->src_port,
             md_ptr);
   }

   return result;
}
