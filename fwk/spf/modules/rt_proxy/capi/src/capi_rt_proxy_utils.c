/**
 *   \file capi_rt_proxy_utils.c
 *   \brief
 *        This file contains CAPI implementation of RT Proxy module.
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "capi_rt_proxy_i.h"

/*==============================================================================
   Public Function Implementation
==============================================================================*/

// TODO: Currently dropping all metadata, need to implement metadata in circ buf.
capi_err_t capi_rt_proxy_propagate_metadata(capi_rt_proxy_t *me_ptr, capi_stream_data_t *input)
{
   capi_err_t             capi_result   = CAPI_EOK;
   capi_stream_data_v2_t *in_stream_ptr = (capi_stream_data_v2_t *)input;

   /* need to check if the stream version is v2 */
   if (CAPI_STREAM_V2 == in_stream_ptr->flags.stream_data_version) // stream version v2
   {
      // return if metadata list is NULL.
      if (in_stream_ptr->metadata_list_ptr)
      {
         AR_MSG(DBG_ERROR_PRIO, "Warning: Dropping metadata on input port");
         capi_result |= capi_rt_proxy_destroy_md_list(me_ptr, &in_stream_ptr->metadata_list_ptr);
      }

      // EOF will be dropped from the module.
      if (in_stream_ptr->flags.end_of_frame)
      {
         AR_MSG(DBG_MED_PRIO, "Warning: EOF is dropped on input port");
         in_stream_ptr->flags.end_of_frame = FALSE;
      }
   }

   return capi_result;
}

/**
 * Calls metadata_destroy on each node in the passed in metadata list.
 */
capi_err_t capi_rt_proxy_destroy_md_list(capi_rt_proxy_t *me_ptr, module_cmn_md_list_t **md_list_pptr)
{
   module_cmn_md_list_t *next_ptr = NULL;
   for (module_cmn_md_list_t *node_ptr = *md_list_pptr; node_ptr;)
   {
      bool_t IS_DROPPED_TRUE = TRUE;
      next_ptr               = node_ptr->next_ptr;
      if (me_ptr->metadata_handler.metadata_destroy)
      {
         me_ptr->metadata_handler.metadata_destroy(me_ptr->metadata_handler.context_ptr,
                                                   node_ptr,
                                                   IS_DROPPED_TRUE,
                                                   md_list_pptr);
      }
      else
      {
         AR_MSG(DBG_ERROR_PRIO, "ffns: Error: metadata handler not provided, can't drop metadata.");
         return CAPI_EFAILED;
      }
      node_ptr = next_ptr;
   }

   return CAPI_EOK;
}

capi_err_t rt_proxy_vaildate_and_cache_input_media_format(capi_rt_proxy_t *me_ptr, capi_buf_t *buf_ptr)
{
   capi_media_fmt_v2_t *media_fmt_ptr = (capi_media_fmt_v2_t *)buf_ptr->data_ptr;

   // compute the actual size of the mf.
   uint32_t actual_mf_size = media_fmt_ptr->format.num_channels * sizeof(uint16_t);
   actual_mf_size += sizeof(capi_set_get_media_format_t) + sizeof(capi_standard_data_format_v2_t);

   // validate the MF payload
   if (buf_ptr->actual_data_len < actual_mf_size)
   {
      AR_MSG(DBG_ERROR_PRIO, "rt_proxy: Invalid media format size %d", buf_ptr->actual_data_len);
      return CAPI_ENEEDMORE;
   }

   // TODO: Validate media format fields.

   // media format into local po
   memscpy(&me_ptr->operating_mf, sizeof(capi_media_fmt_v2_t), media_fmt_ptr, actual_mf_size);

   // Set media format received to TRUE.
   me_ptr->is_input_mf_received = TRUE;

   return CAPI_EOK;
}

// raise capi events on receiving the input and output port configurations.
capi_err_t rt_proxy_raise_output_media_format_event(capi_rt_proxy_t *me_ptr, capi_media_fmt_v2_t *mf_info_ptr)
{
   capi_err_t        result = CAPI_EOK;
   capi_event_info_t event_info;

   event_info.port_info.is_valid      = true;
   event_info.port_info.is_input_port = false;
   event_info.port_info.port_index    = 0;

   event_info.payload.actual_data_len = sizeof(capi_media_fmt_v2_t);
   event_info.payload.data_ptr        = (int8_t *)mf_info_ptr;
   event_info.payload.max_data_len    = sizeof(capi_media_fmt_v2_t);

   result = me_ptr->event_cb_info.event_cb(me_ptr->event_cb_info.event_context,
                                           CAPI_EVENT_OUTPUT_MEDIA_FORMAT_UPDATED_V2,
                                           &event_info);
   if (CAPI_EOK != result)
   {
      AR_MSG(DBG_ERROR_PRIO, "rt_proxy: Failed to raise event for output media format");
   }

   return result;
}

// raise capi events on receiving the input and output port configurations.
capi_err_t rt_proxy_raise_mpps_and_bw_events(capi_rt_proxy_t *me_ptr)
{
   // Need to profile this.
   return CAPI_EOK;
}

// raise capi events on receiving the input and output port configurations.
capi_err_t rt_proxy_raise_threshold_events(capi_rt_proxy_t *me_ptr)
{
   capi_err_t capi_result = CAPI_EOK;

   // raise input and output thresholds
   uint32_t in_thresh_in_bytes  = 0;
   uint32_t out_thresh_in_bytes = 0;

   if (me_ptr->is_tx_module)
   {
      in_thresh_in_bytes = capi_cmn_us_to_bytes_per_ch(me_ptr->cfg.client_frame_size_in_ms * 1000,
                                                       me_ptr->operating_mf.format.sampling_rate,
                                                       me_ptr->operating_mf.format.bits_per_sample);
      out_thresh_in_bytes = capi_cmn_us_to_bytes_per_ch(me_ptr->frame_duration_in_us * 1000,
                                                        me_ptr->operating_mf.format.sampling_rate,
                                                        me_ptr->operating_mf.format.bits_per_sample);
   }
   else
   {
      in_thresh_in_bytes = capi_cmn_us_to_bytes_per_ch(me_ptr->frame_duration_in_us,
                                                       me_ptr->operating_mf.format.sampling_rate,
                                                       me_ptr->operating_mf.format.bits_per_sample);
      out_thresh_in_bytes = capi_cmn_us_to_bytes_per_ch(me_ptr->cfg.client_frame_size_in_ms * 1000,
                                                        me_ptr->operating_mf.format.sampling_rate,
                                                        me_ptr->operating_mf.format.bits_per_sample);
   }

   AR_MSG(DBG_HIGH_PRIO, "input_thresh=%lu, output_thresh=%lu", in_thresh_in_bytes, out_thresh_in_bytes);

   capi_result |=
      capi_cmn_update_port_data_threshold_event(&me_ptr->event_cb_info, in_thresh_in_bytes, TRUE /* is_input*/, 0);

   capi_result |=
      capi_cmn_update_port_data_threshold_event(&me_ptr->event_cb_info, out_thresh_in_bytes, FALSE /* is_input*/, 0);

   return capi_result;
}
