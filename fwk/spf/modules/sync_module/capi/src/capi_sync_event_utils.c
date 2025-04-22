/**
 * \file capi_sync_event_utils.c
 * \brief
 *     Implementation of utility functions for capi event handling (kpps, bandwidth, any other events, etc).
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "capi_sync_i.h"
#include "module_cmn_api.h"

const uint32_t CAPI_SYNC_BW   = 1 * 1024 * 1024;
const uint32_t CAPI_SYNC_KPPS = 30;

/* =========================================================================
 * Static function declarations
 * =========================================================================*/
static uint32_t capi_sync_get_kpps(capi_sync_t *me_ptr);

/* =========================================================================
 * Function definitions
 * =========================================================================*/

/**
 * Function to get kpps.
 */
static uint32_t capi_sync_get_kpps(capi_sync_t *me_ptr)
{
   uint64_t kpps = 0;
   for (uint32_t i = 0; i < me_ptr->num_in_ports; i++)
   {
      if (capi_sync_media_fmt_is_valid(&me_ptr->in_port_info_ptr[i]) &&
          (SYNC_PORT_INDEX_INVALID != me_ptr->in_port_info_ptr[i].cmn.conn_index))
      {
         // bytes per second
         uint64_t bps = me_ptr->in_port_info_ptr[i].media_fmt.format.sampling_rate *
                        me_ptr->in_port_info_ptr[i].media_fmt.format.num_channels *
                        (me_ptr->in_port_info_ptr[i].media_fmt.format.bits_per_sample >> 3);

         // copying from input to internal and from internal to output buffer.
         // considering four bytes are copied per packet then packets_per_second =  2*(bps/4)
         kpps += ((bps >> 1) / 1000);
      }
   }
   return (uint32_t)kpps;
}

static uint32_t capi_sync_get_bw(capi_sync_t *me_ptr)
{
   uint64_t bw = 0;
   for (uint32_t i = 0; i < me_ptr->num_in_ports; i++)
   {
      if (capi_sync_media_fmt_is_valid(&me_ptr->in_port_info_ptr[i]) &&
          (SYNC_PORT_INDEX_INVALID != me_ptr->in_port_info_ptr[i].cmn.conn_index))
      {
         uint64_t data_rate = me_ptr->in_port_info_ptr[i].media_fmt.format.sampling_rate *
                              me_ptr->in_port_info_ptr[i].media_fmt.format.num_channels *
                              (me_ptr->in_port_info_ptr[i].media_fmt.format.bits_per_sample >> 3);
         bw += (data_rate * 4); // input to internal and internal to output
      }
   }
   return (uint32_t)bw;
}

static capi_err_t capi_sync_raise_out_port_active_inactive_event(capi_sync_t *me_ptr,
                                                                 bool_t       is_inactive,
                                                                 uint32_t     out_port_idx)
{
   capi_err_t capi_result = CAPI_EOK;
   capi_event_info_t event_info;
   event_info.port_info.is_valid = FALSE;
   // Package the fwk event within the data_to_dsp capi event.
   capi_event_data_to_dsp_service_t                  evt = { 0 };
   fwk_extn_sync_event_id_data_port_activity_state_t event;
   event.is_inactive           = is_inactive;
   event.out_port_index        = out_port_idx;
   evt.param_id                = FWK_EXTN_SYNC_EVENT_ID_DATA_PORT_ACTIVITY_STATE;
   evt.token                   = 0;
   evt.payload.actual_data_len = sizeof(fwk_extn_sync_event_id_data_port_activity_state_t);
   evt.payload.data_ptr        = (int8_t *)&event;
   evt.payload.max_data_len    = sizeof(fwk_extn_sync_event_id_data_port_activity_state_t);

   event_info.payload.actual_data_len = sizeof(capi_event_data_to_dsp_service_t);
   event_info.payload.data_ptr        = (int8_t *)&evt;
   event_info.payload.max_data_len    = sizeof(capi_event_data_to_dsp_service_t);
   capi_result = me_ptr->cb_info.event_cb(me_ptr->cb_info.event_context, CAPI_EVENT_DATA_TO_DSP_SERVICE, &event_info);
   if (CAPI_FAILED(capi_result))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "capi sync: Failed to raise out_port_active_inactive_event with result 0x%x out port idx = %ld"
             "is_inactive = %lu",
             capi_result,
             out_port_idx,
             is_inactive);
      return capi_result;
   }

   AR_MSG(DBG_MED_PRIO,
          "capi sync: raised out_port_active_inactive_event with result 0x%x out port idx = %ld",
          out_port_idx,
          is_inactive);
   return capi_result;
}

capi_err_t capi_sync_check_raise_out_port_active_inactive_event(capi_sync_t *me_ptr)
{
   AR_MSG(DBG_MED_PRIO, "Handling capi_sync_raise_out_port_active_inactive_event");

   if (NULL == me_ptr->cb_info.event_cb)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi sync: capi event callback is not set, unable to raise active/inactive port event!");
      return CAPI_EBADPARAM;
   }

   for (uint32_t i = 0; i < me_ptr->num_out_ports; i++)
   {
      capi_sync_out_port_t *out_port_ptr = &(me_ptr->out_port_info_ptr[i]);
      if (CAPI_PORT_STATE_CLOSED != out_port_ptr->cmn.state)
      {
         capi_sync_cmn_port_t *conn_in_cmn_port_ptr =
            capi_sync_get_port_cmn_from_port_id(me_ptr, out_port_ptr->cmn.conn_port_id, TRUE /*is_input*/);

         // check if the connected input port is closed. that points to this output port being inactive
         if ((!conn_in_cmn_port_ptr) ||
             (conn_in_cmn_port_ptr && (CAPI_PORT_STATE_CLOSED == conn_in_cmn_port_ptr->state)))
         {
            capi_sync_raise_out_port_active_inactive_event(me_ptr, TRUE /*is_inactive*/, i);
         }
         else
         {
            capi_sync_raise_out_port_active_inactive_event(me_ptr, FALSE /*is_inactive*/, i);
         }
      }
   }

   return AR_EOK;
}

/**
 * Function to send the toggle threshold change event.
 */
capi_err_t capi_sync_raise_event_toggle_threshold(capi_sync_t *me_ptr, bool_t enable_threshold)
{
   capi_err_t        capi_result = CAPI_EOK;
   capi_event_info_t event_info;
   bool_t            prev_thresh_state;
   event_info.port_info.is_valid = FALSE;

   if (NULL == me_ptr->cb_info.event_cb)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi sync: capi event callback is not set, unable to raise enable threshold event!");
      return CAPI_EBADPARAM;
   }

   // If threshold is being disabled, it can result in a process call in the call back context without updating the
   // state of the threshold in the module me_ptr. When the process is called again, the module raises the event again
   // and again leading to nested process calls and crashes.
   prev_thresh_state             = me_ptr->threshold_is_disabled;
   me_ptr->threshold_is_disabled = !enable_threshold;

   if (me_ptr->threshold_is_disabled)
   {
      // Enable the module if threshold is disabled, so that module can do syncing.
      capi_sync_raise_enable_disable_event(me_ptr, TRUE);
   }

   // Package the fwk event within the data_to_dsp capi event.
   capi_event_data_to_dsp_service_t                    evt = { 0 };
   fwk_extn_sync_event_id_enable_threshold_buffering_t event;
   event.enable_threshold_buffering = enable_threshold;

   evt.param_id                = FWK_EXTN_SYNC_EVENT_ID_ENABLE_THRESHOLD_BUFFERING;
   evt.token                   = 0;
   evt.payload.actual_data_len = sizeof(fwk_extn_sync_event_id_enable_threshold_buffering_t);
   evt.payload.data_ptr        = (int8_t *)&event;
   evt.payload.max_data_len    = sizeof(fwk_extn_sync_event_id_enable_threshold_buffering_t);

   event_info.payload.actual_data_len = sizeof(capi_event_data_to_dsp_service_t);
   event_info.payload.data_ptr        = (int8_t *)&evt;
   event_info.payload.max_data_len    = sizeof(capi_event_data_to_dsp_service_t);
   capi_result = me_ptr->cb_info.event_cb(me_ptr->cb_info.event_context, CAPI_EVENT_DATA_TO_DSP_SERVICE, &event_info);
   if (CAPI_FAILED(capi_result))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "capi sync: Failed to raise enable threshold event with result 0x%x enable = %ld",
             capi_result,
             enable_threshold);
      me_ptr->threshold_is_disabled = prev_thresh_state;
      return capi_result;
   }

   AR_MSG(DBG_MED_PRIO, "capi sync: raised event for enable threshold, enable = %ld!", enable_threshold);

   return capi_result;
}

void capi_sync_raise_enable_disable_event(capi_sync_t *me_ptr, bool_t is_enable)
{
   if (me_ptr->events_config.enable != is_enable)
   {
      me_ptr->events_config.enable = is_enable;

      AR_MSG(DBG_MED_PRIO, "capi sync: is_enabled %d", is_enable);

      if (me_ptr->is_mimo_process_state_intf_ext_supported)
      {
         capi_event_callback_info_t *cb_info_ptr = &me_ptr->cb_info;

         intf_extn_event_id_mimo_module_process_state_t event_payload;
         event_payload.is_disabled = !is_enable;

         /* Create event */
         capi_event_data_to_dsp_service_t to_send;
         to_send.param_id                = INTF_EXTN_EVENT_ID_MIMO_MODULE_PROCESS_STATE;
         to_send.payload.actual_data_len = sizeof(intf_extn_event_id_mimo_module_process_state_t);
         to_send.payload.max_data_len    = sizeof(intf_extn_event_id_mimo_module_process_state_t);
         to_send.payload.data_ptr        = (int8_t *)&event_payload;

         /* Create event info */
         capi_event_info_t event_info;
         event_info.port_info.is_input_port = FALSE;
         event_info.port_info.is_valid      = FALSE;
         event_info.payload.actual_data_len = sizeof(to_send);
         event_info.payload.max_data_len    = sizeof(to_send);
         event_info.payload.data_ptr        = (int8_t *)&to_send;

         (void)cb_info_ptr->event_cb(cb_info_ptr->event_context, CAPI_EVENT_DATA_TO_DSP_SERVICE, &event_info);
      }
   }
}

/**
 * Function to raise various events of the sync.
 */
capi_err_t capi_sync_raise_event(capi_sync_t *me_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   if (NULL == me_ptr->cb_info.event_cb)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi sync:  Event callback is not set. Unable to raise events!");
      CAPI_SET_ERROR(capi_result, CAPI_EUNSUPPORTED);
      return capi_result;
   }

   me_ptr->events_config.kpps        = capi_sync_get_kpps(me_ptr);
   me_ptr->events_config.data_bw     = capi_sync_get_bw(me_ptr);
   me_ptr->events_config.delay_in_us = 0;

   capi_result |= capi_cmn_update_kpps_event(&me_ptr->cb_info, me_ptr->events_config.kpps);
   capi_result |=
      capi_cmn_update_bandwidth_event(&me_ptr->cb_info, me_ptr->events_config.code_bw, me_ptr->events_config.data_bw);
   capi_result |= capi_cmn_update_algo_delay_event(&me_ptr->cb_info, me_ptr->events_config.delay_in_us);
   return capi_result;
}
