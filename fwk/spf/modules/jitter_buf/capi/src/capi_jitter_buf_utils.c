/**
 *   \file capi_jitter_buf_utils.c
 *   \brief
 *        This file contains CAPI implementation of Jitter Buf module.
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "capi_jitter_buf_i.h"

/*==============================================================================
   Public Function Implementation
==============================================================================*/

/* TODO: Pending - Currently dropping all metadata, need to implement metadata in circ buf. */
capi_err_t capi_jitter_buf_propagate_metadata(capi_jitter_buf_t *me_ptr, capi_stream_data_t *input)
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
         capi_result |= capi_jitter_buf_destroy_md_list(me_ptr, &in_stream_ptr->metadata_list_ptr);
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

/* Calls metadata_destroy on each node in the passed in metadata list.  */
capi_err_t capi_jitter_buf_destroy_md_list(capi_jitter_buf_t *me_ptr, module_cmn_md_list_t **md_list_pptr)
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
         AR_MSG(DBG_ERROR_PRIO, "jitter_buf: Error: metadata handler not provided, can't drop metadata.");
         return CAPI_EFAILED;
      }
      node_ptr = next_ptr;
   }

   return CAPI_EOK;
}

/* Set size of circular buffer based on the configuration received */
capi_err_t capi_jitter_buf_set_size(capi_jitter_buf_t *me_ptr, bool_t is_debug)
{
   capi_err_t result = CAPI_EOK;

   if (me_ptr->driver_hdl.stream_buf)
   {
      /* If the circular buffer is already configured and initialized to
       * param values but debug size is received then destroy and reinit
       * only if this is due to debug param id. Otherwise return. */
      if (is_debug)
      {
         if (AR_EOK != (result = jitter_buf_driver_deinit(me_ptr)))
         {
            AR_MSG(DBG_ERROR_PRIO, "jitter_buf: Cannot destroy driver. ");
            return CAPI_EFAILED;
         }
      }
      else
      {
         return result;
      }
   }

   /* If debug value is present that takes precedence */
   if (me_ptr->debug_size_ms)
   {
      me_ptr->jitter_allowance_in_ms = me_ptr->debug_size_ms;
   }

   /* If previously disabled enable it back if it needs to be */
   if (is_debug && (!me_ptr->event_config.process_state) && me_ptr->debug_size_ms)
   {
      result = capi_cmn_update_process_check_event(&me_ptr->event_cb_info, TRUE);
      if (result)
      {
         AR_MSG(DBG_ERROR_PRIO, "jitter_buf: Cannot enable the module. ");
         return result;
      }
      me_ptr->event_config.process_state = TRUE;
   }

   /* If jitter size is zero disable module and return*/
   if (me_ptr->jitter_allowance_in_ms == 0)
   {
      /* If config received is zero then module is disabled */
      result                             = capi_cmn_update_process_check_event(&me_ptr->event_cb_info, FALSE);
      me_ptr->event_config.process_state = FALSE;
      return result;
   }

   /* If calibration and media format are set then initialize the driver.
    * we need calibration to create buffers. */

   if (!me_ptr->driver_hdl.stream_buf)
   {
      ar_result_t result = AR_EOK;
      if (AR_EOK != (result = jitter_buf_driver_init(me_ptr)))
      {
         AR_MSG(DBG_ERROR_PRIO, "jitter_buf: Cannot intialize the driver. ");
         return CAPI_EFAILED;
      }
   }
   else
   {
      /* if driver is already initialized just re-calibrate the driver. */
       if (CAPI_EOK != (result = ar_result_to_capi_err(jitter_buf_calibrate_driver(me_ptr))))
       {
          AR_MSG(DBG_ERROR_PRIO, "jitter_buf: Failed calibrating the driver with error = %lx", result);
          return result;
       }
   }

   return CAPI_EOK;
}

/* Check validate and store media format */
capi_err_t capi_jitter_buf_vaildate_and_cache_input_mf(capi_jitter_buf_t *me_ptr, capi_buf_t *buf_ptr)
{
   capi_media_fmt_v2_t *media_fmt_ptr = (capi_media_fmt_v2_t *)buf_ptr->data_ptr;

   // compute the actual size of the mf.
   uint32_t actual_mf_size = media_fmt_ptr->format.num_channels * sizeof(uint16_t);
   actual_mf_size += sizeof(capi_set_get_media_format_t) + sizeof(capi_standard_data_format_v2_t);

   // validate the MF payload
   if (buf_ptr->actual_data_len < actual_mf_size)
   {
      AR_MSG(DBG_ERROR_PRIO, "jitter_buf: Invalid media format size %d", buf_ptr->actual_data_len);
      return CAPI_ENEEDMORE;
   }

   /*  TODO: Pending - Validate media format fields. Only deinterleaved ?*/

   memscpy(&me_ptr->operating_mf, sizeof(capi_media_fmt_v2_t), media_fmt_ptr, actual_mf_size);

   /* Set media format received to TRUE. */
   me_ptr->is_input_mf_received = TRUE;

   return CAPI_EOK;
}

/* Raise capi output media format after receiving input media format */
capi_err_t capi_jitter_buf_raise_output_mf_event(capi_jitter_buf_t *me_ptr, capi_media_fmt_v2_t *mf_info_ptr)
{
   capi_err_t        result = CAPI_EOK;
   capi_event_info_t event_info;

   event_info.port_info.is_valid      = TRUE;
   event_info.port_info.is_input_port = FALSE;
   event_info.port_info.port_index    = 0;

   event_info.payload.actual_data_len = sizeof(capi_media_fmt_v2_t);
   event_info.payload.data_ptr        = (int8_t *)mf_info_ptr;
   event_info.payload.max_data_len    = sizeof(capi_media_fmt_v2_t);

   result = me_ptr->event_cb_info.event_cb(me_ptr->event_cb_info.event_context,
                                           CAPI_EVENT_OUTPUT_MEDIA_FORMAT_UPDATED_V2,
                                           &event_info);
   if (CAPI_EOK != result)
   {
      AR_MSG(DBG_ERROR_PRIO, "jitter_buf: Failed to raise event for output media format");
   }

   return result;
}

/* Raise kpps and bw depending on input media format */
/* TODO: Pending - Check values */
capi_err_t capi_jitter_buf_raise_kpps_bw_event(capi_jitter_buf_t *me_ptr)
{
   capi_err_t capi_result = CAPI_EOK;

   if (!me_ptr->is_input_mf_received)
   {
      return capi_result;
   }

   uint32_t kpps, data_bw;

   if (me_ptr->operating_mf.format.num_channels <= 8)
   {
      kpps    = JITTER_BUF_KPPS;
      data_bw = JITTER_BUF_BW;
   }
   else if (me_ptr->operating_mf.format.num_channels <= 16)
   {
      kpps    = JITTER_BUF_KPPS_16;
      data_bw = JITTER_BUF_BW_16;
   }
   else
   {
      kpps    = JITTER_BUF_KPPS_32;
      data_bw = JITTER_BUF_BW_32;
   }

   if (kpps != me_ptr->event_config.kpps)
   {
      capi_result |= capi_cmn_update_kpps_event(&me_ptr->event_cb_info, kpps);
      if (!capi_result)
      {
         me_ptr->event_config.kpps = kpps;
      }
   }

   if (data_bw != me_ptr->event_config.data_bw)
   {
      capi_result |= capi_cmn_update_bandwidth_event(&me_ptr->event_cb_info, 0, data_bw);
      if (!capi_result)
      {
         me_ptr->event_config.data_bw = data_bw;
      }
   }

   return capi_result;
}

/* Jitter buffer is by default driven based on output availability. Only if output is read
 * input is written if it is available at that time. If not, the input gets buffered
 * up in congestion buffering module.
 * If input buffering mode is enabled (ICMD) then input is written if it is available.*/
capi_err_t capi_jitter_buf_change_trigger_policy(capi_jitter_buf_t *me_ptr)
{
   capi_err_t result = AR_EOK;

   if (NULL == me_ptr->policy_chg_cb.change_data_trigger_policy_cb_fn)
   {
      return result;
   }

   fwk_extn_port_trigger_affinity_t input_group1  = { FWK_EXTN_PORT_TRIGGER_AFFINITY_NONE };
   fwk_extn_port_trigger_affinity_t output_group1 = { FWK_EXTN_PORT_TRIGGER_AFFINITY_PRESENT };

   fwk_extn_port_trigger_group_t triggerable_groups[1];
   triggerable_groups[0].in_port_grp_affinity_ptr  = &input_group1;
   triggerable_groups[0].out_port_grp_affinity_ptr = &output_group1;

   fwk_extn_port_nontrigger_policy_t input_group2  = { FWK_EXTN_PORT_NON_TRIGGER_OPTIONAL };
   fwk_extn_port_nontrigger_policy_t output_group2 = { FWK_EXTN_PORT_NON_TRIGGER_INVALID };

   fwk_extn_port_nontrigger_group_t nontriggerable_group[1];
   nontriggerable_group[0].in_port_grp_policy_ptr  = &input_group2;
   nontriggerable_group[0].out_port_grp_policy_ptr = &output_group2;

   /* If input buffering is to be done at input trigger then need to mark optional triggerable.*/
   if (JBM_BUFFER_INPUT_AT_INPUT_TRIGGER == me_ptr->input_buffer_mode)
   {
      input_group1 = FWK_EXTN_PORT_TRIGGER_AFFINITY_PRESENT;
      input_group2 = FWK_EXTN_PORT_NON_TRIGGER_INVALID;
   }

   // By default set the mode to RT, when the write arrives then make it FTRT.
   result = me_ptr->policy_chg_cb.change_data_trigger_policy_cb_fn(me_ptr->policy_chg_cb.context_ptr,
                                                                   nontriggerable_group,
                                                                   FWK_EXTN_PORT_TRIGGER_POLICY_OPTIONAL,
                                                                   1,
                                                                   triggerable_groups);
   return result;
}
