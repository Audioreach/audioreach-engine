/**
 * \file capi_sync_utils.c
 * \brief
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "capi_sync_i.h"
#include "module_cmn_api.h"

/**
 * Initialize sync configuration
 */
void capi_sync_init_config(capi_sync_t *me_ptr, capi_sync_mode_t mode)
{
   me_ptr->mode                  = mode;         // Start with equal priority inputs
   me_ptr->synced_state          = STATE_SYNCED; // State is interpreted as synced till ports are started
   me_ptr->threshold_is_disabled = FALSE;        // Begins with threshold enabled which is default container behavior.
   me_ptr->events_config.enable  = TRUE;         // enabled by default. will try to disable from process context.
}

/**
 *  Handle change in input threshold
 */
capi_err_t capi_sync_ext_input_threshold_change(capi_sync_t *me_ptr)
{
   capi_err_t capi_result = CAPI_EOK;

   if (NULL == me_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi sync : process ext input threshold received null ptr");
      return CAPI_EBADPARAM;
   }

   for (uint32_t i = 0; i < me_ptr->num_in_ports; i++)
   {
      capi_sync_in_port_t *in_port_ptr = &(me_ptr->in_port_info_ptr[i]);

      if (SYNC_PORT_INDEX_INVALID == in_port_ptr->cmn.self_index)
      {
         continue;
      }

      if (capi_sync_media_fmt_is_valid(in_port_ptr))
      {
         capi_result |= capi_sync_calc_threshold_bytes(me_ptr, in_port_ptr);
         capi_result |= capi_sync_allocate_port_buffer(me_ptr, in_port_ptr);
      }

      in_port_ptr->is_threshold_disabled = TRUE;
      in_port_ptr->is_output_sent_once   = FALSE;
   }

   bool_t should_disable_thresh = capi_sync_should_disable_thresh(me_ptr);

   AR_MSG(DBG_MED_PRIO,
          "capi sync process ext input thresh change: Current Thresh state = %d, me_ptr thresh %d",
          should_disable_thresh,
          me_ptr->threshold_is_disabled);

   if (me_ptr->threshold_is_disabled != should_disable_thresh)
   {

      if (FALSE == should_disable_thresh)
      {
         me_ptr->synced_state = STATE_SYNCED;
      }
      else
      {
         me_ptr->synced_state = STATE_STARTING;
      }

      capi_result |= capi_sync_raise_event_toggle_threshold(me_ptr, !should_disable_thresh);
   }

   return capi_result;
}

/**
 *  Calculate threshold bytes for a given input port. This is exercised during
 *  input media format change and input threshold change
 */
capi_err_t capi_sync_calc_threshold_bytes(capi_sync_t *me_ptr, capi_sync_in_port_t *in_port_ptr)
{
   capi_err_t capi_result = CAPI_EOK;

   // RR: Assume only 1ms cushion for now. Figure out how else to handle
   in_port_ptr->peer_frame_size_us = SYNC_UPSTREAM_FRAME_SIZE_US;

   uint32_t us      = me_ptr->module_config.frame_len_us;
   uint32_t peer_us = in_port_ptr->peer_frame_size_us;
   uint32_t sr      = in_port_ptr->media_fmt.format.sampling_rate;
   uint32_t bps     = in_port_ptr->media_fmt.format.bits_per_sample;

   // Calculate threshold in bytes per channel based on operating threshold &
   // frame size in bytes per channel based on upstream peer frame size
   // Round to integer samples.
   in_port_ptr->threshold_bytes_per_ch = capi_cmn_us_to_bytes_per_ch(us, sr, bps);

   // Round to integer samples.
   in_port_ptr->frame_size_bytes_per_ch = capi_cmn_us_to_bytes_per_ch(peer_us, sr, bps);

   AR_MSG(DBG_MED_PRIO,
          "capi sync process input index %ld new threshold: %ld bytes per ch, frame size %ld bytes per ch",
          in_port_ptr->cmn.self_index,
          in_port_ptr->threshold_bytes_per_ch,
          in_port_ptr->frame_size_bytes_per_ch);

   return capi_result;
}
