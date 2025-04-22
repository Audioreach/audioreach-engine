/**
 *   \file capi_bt_buf_module_utils.c
 *   \brief
 *        This file contains utilities for duty cycling buffering module
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "module_cmn_api.h"
#include "capi_duty_cycling_buf_utils.h"

/*
 * Module needs data trigger in STM container in order to consume input FTRT.
 */
capi_err_t capi_duty_cycling_buf_raise_event_data_trigger_in_st_cntr(capi_duty_cycling_buf_t *me_ptr)
{
   capi_err_t                                  result = CAPI_EOK;
   capi_buf_t                                  payload;
   fwk_extn_event_id_data_trigger_in_st_cntr_t event;

   if (NULL == me_ptr->event_cb_info.event_cb)
   {
      AR_MSG(DBG_ERROR_PRIO, "Event callback is not set. Unable to raise events!");
      CAPI_SET_ERROR(result, CAPI_EUNSUPPORTED);
      return result;
   }

   event.is_enable             = TRUE;
   event.needs_input_triggers  = TRUE;
   event.needs_output_triggers = FALSE;

   payload.data_ptr        = (int8_t *)&event;
   payload.actual_data_len = payload.max_data_len = sizeof(event);

   result =
      capi_cmn_raise_data_to_dsp_svc_event(&me_ptr->event_cb_info, FWK_EXTN_EVENT_ID_DATA_TRIGGER_IN_ST_CNTR, &payload);

   if (CAPI_FAILED(result))
   {
      AR_MSG(DBG_ERROR_PRIO, "capi bt_buf_module: Failed to raise event to enable data_trigger.");
      return result;
   }
   else
   {
      AR_MSG(DBG_HIGH_PRIO, "capi bt_buf_module: raised event to enable data_trigger in ST container");
   }

   return result;
}

/*

Trigger policy scheme for the module
High level expected behaviour
- Module should send output only during signal trigger
  so that output will be sent to RAT only during signal trigger
- If input is sent, store it in the circular buffer
- Buf module informs Duty Cycling Manager 'DCM' at say 't1' that it is ready to enter island
  and DCM sends command to all the containers and container may be processing some data
  at that time. So sometime is elapsed inbetween container sending response to DCM inturn causing
  some delay to enter the island(say at 't2' , t2>t1). Till t2 if buf module process input it may cause overflow.
  So block input as soon buffering is done without waiting for ack from DCM. So that even if upstream
  sends some data it will be in dataq of container and module need not handle any overflows
- For exiting island - As soon as lower threshold is reached change tgp

Data Trigger:
While buffering:- Listen to input and block output
After buffering:- Block input and Block output

Signal Trigger:
While buffering:- Listen to input OR output - tgp optional
After buffering:- Block input and  output is mandatory to process

*/

capi_err_t capi_duty_cycling_buf_update_tgp_while_buffering(capi_duty_cycling_buf_t *me_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   uint32_t   ONE_IP_PORT = 1;
   uint32_t   ONE_OP_PORT = 1;
   uint32_t   ONE_GROUP   = 1;

   fwk_extn_port_trigger_affinity_t  inp_affinity[ONE_IP_PORT];
   fwk_extn_port_nontrigger_policy_t inp_non_tgp[ONE_IP_PORT];
   fwk_extn_port_trigger_affinity_t  out_affinity[ONE_OP_PORT];
   fwk_extn_port_nontrigger_policy_t out_non_tgp[ONE_OP_PORT];

   AR_MSG_ISLAND(DBG_HIGH_PRIO, " tgp_while_buffering i.e., in non island");

   if (NULL == me_ptr->tgp.tg_policy_cb.change_data_trigger_policy_cb_fn)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi bt_buf_module:  callback is not set. Unable to raise trigger policy!");
      return capi_result;
   }

   fwk_extn_port_trigger_group_t triggerable_group = {.in_port_grp_affinity_ptr  = inp_affinity,
                                                      .out_port_grp_affinity_ptr = out_affinity };
   fwk_extn_port_nontrigger_group_t nontriggerable_group = {.in_port_grp_policy_ptr  = inp_non_tgp,
                                                            .out_port_grp_policy_ptr = out_non_tgp };

   // while_buffering for data trigger -listen to input
   inp_affinity[0] = FWK_EXTN_PORT_TRIGGER_AFFINITY_PRESENT;
   inp_non_tgp[0]  = FWK_EXTN_PORT_NON_TRIGGER_INVALID;

   // while_buffering for data trigger Output is blocked.
   out_affinity[0] = FWK_EXTN_PORT_TRIGGER_AFFINITY_NONE;
   out_non_tgp[0]  = FWK_EXTN_PORT_NON_TRIGGER_BLOCKED;

   capi_result = me_ptr->tgp.tg_policy_cb.change_data_trigger_policy_cb_fn(me_ptr->tgp.tg_policy_cb.context_ptr,
                                                                           &nontriggerable_group,
                                                                           FWK_EXTN_PORT_TRIGGER_POLICY_MANDATORY,
                                                                           ONE_GROUP,
                                                                           &triggerable_group);
   if (CAPI_SUCCEEDED(capi_result))
   {
      AR_MSG(DBG_HIGH_PRIO, "while buffering: data trigger policy updated.");
   }

   // while_buffering for signal trigger -listen to input
   inp_affinity[0] = FWK_EXTN_PORT_TRIGGER_AFFINITY_PRESENT;
   inp_non_tgp[0]  = FWK_EXTN_PORT_NON_TRIGGER_INVALID;

   // while_buffering for signal trigger listen to output
   out_affinity[0] = FWK_EXTN_PORT_TRIGGER_AFFINITY_PRESENT;
   out_non_tgp[0]  = FWK_EXTN_PORT_NON_TRIGGER_INVALID;
   // listen to input or output - so make optional

   //Review again - to change optional to mandatory
   capi_result = me_ptr->tgp.tg_policy_cb.change_signal_trigger_policy_cb_fn(me_ptr->tgp.tg_policy_cb.context_ptr,
                                                                             &nontriggerable_group,
                                                                             FWK_EXTN_PORT_TRIGGER_POLICY_OPTIONAL,
                                                                             ONE_GROUP,
                                                                             &triggerable_group);
   if (CAPI_SUCCEEDED(capi_result))
   {
      AR_MSG(DBG_HIGH_PRIO, "while buffering: signal trigger policy updated.");
   }

   me_ptr->tgp_status = DUTY_CYCLING_WHILE_BUFFERING;

   return capi_result;
}

capi_err_t capi_duty_cycling_buf_update_tgp_after_buffering(capi_duty_cycling_buf_t *me_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   uint32_t   ONE_IP_PORT = 1;
   uint32_t   ONE_OP_PORT = 1;
   uint32_t   ONE_GROUP   = 1;

   fwk_extn_port_trigger_affinity_t  inp_affinity[ONE_IP_PORT];
   fwk_extn_port_nontrigger_policy_t inp_non_tgp[ONE_IP_PORT];
   fwk_extn_port_trigger_affinity_t  out_affinity[ONE_OP_PORT];
   fwk_extn_port_nontrigger_policy_t out_non_tgp[ONE_OP_PORT];

   AR_MSG_ISLAND(DBG_HIGH_PRIO, " tgp_after_buffering i.e., ready to enter island");

   if (NULL == me_ptr->tgp.tg_policy_cb.change_data_trigger_policy_cb_fn)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi bt_buf_module:  callback is not set. Unable to raise trigger policy!");
      return capi_result;
   }

   fwk_extn_port_trigger_group_t triggerable_group = {.in_port_grp_affinity_ptr  = inp_affinity,
                                                      .out_port_grp_affinity_ptr = out_affinity };
   fwk_extn_port_nontrigger_group_t nontriggerable_group = {.in_port_grp_policy_ptr  = inp_non_tgp,
                                                            .out_port_grp_policy_ptr = out_non_tgp };

   // after_buffering for data trigger Input is blocked.
   inp_affinity[0] = FWK_EXTN_PORT_TRIGGER_AFFINITY_NONE;
   inp_non_tgp[0]  = FWK_EXTN_PORT_NON_TRIGGER_BLOCKED;

   // after_buffering for data trigger Output is blocked
   out_affinity[0] = FWK_EXTN_PORT_TRIGGER_AFFINITY_NONE;
   out_non_tgp[0]  = FWK_EXTN_PORT_NON_TRIGGER_BLOCKED;

   capi_result = me_ptr->tgp.tg_policy_cb.change_data_trigger_policy_cb_fn(me_ptr->tgp.tg_policy_cb.context_ptr,
                                                                           &nontriggerable_group,
                                                                           FWK_EXTN_PORT_TRIGGER_POLICY_MANDATORY,
                                                                           ONE_GROUP,
                                                                           &triggerable_group);
   if (CAPI_SUCCEEDED(capi_result))
   {
      AR_MSG(DBG_HIGH_PRIO, "after buffering: data trigger policy updated.");
   }

   // after_buffering for signal trigger - block input.
   inp_affinity[0] = FWK_EXTN_PORT_TRIGGER_AFFINITY_NONE;
   inp_non_tgp[0]  = FWK_EXTN_PORT_NON_TRIGGER_BLOCKED;

   // after_buffering for signal trigger - listen output
   out_affinity[0] = FWK_EXTN_PORT_TRIGGER_AFFINITY_PRESENT;
   out_non_tgp[0]  = FWK_EXTN_PORT_NON_TRIGGER_INVALID;

   capi_result = me_ptr->tgp.tg_policy_cb.change_signal_trigger_policy_cb_fn(me_ptr->tgp.tg_policy_cb.context_ptr,
                                                                             &nontriggerable_group,
                                                                             FWK_EXTN_PORT_TRIGGER_POLICY_MANDATORY,
                                                                             ONE_GROUP,
                                                                             &triggerable_group);
   if (CAPI_SUCCEEDED(capi_result))
   {
      AR_MSG(DBG_HIGH_PRIO, "after buffering: signal trigger policy updated.");
   }

   me_ptr->tgp_status = DUTY_CYCLING_AFTER_BUFFERING;

   return capi_result;
}

bool_t capi_duty_cycling_buf_is_supported_media_type(capi_media_fmt_v2_t *format_ptr)
{
   if (CAPI_FIXED_POINT != format_ptr->header.format_header.data_format)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "duty_cycling_buf: Unsupported data format %lu",
             (uint32_t)format_ptr->header.format_header.data_format);
      return FALSE;
   }

   if (format_ptr->format.data_interleaving != CAPI_DEINTERLEAVED_UNPACKED)
   {
      AR_MSG(DBG_ERROR_PRIO, "duty_cycling_buf: Interleaved data is not supported.");
      return FALSE;
   }

   if (format_ptr->format.num_channels > CAPI_MAX_CHANNELS_V2)
   {
      AR_MSG(DBG_ERROR_PRIO, "duty_cycling_buf: Unsupported mf num_channels= %d,", format_ptr->format.num_channels);
      return FALSE;
   }

   return TRUE;
}

capi_err_t capi_duty_cycling_buf_set_prop_input_media_fmt(capi_duty_cycling_buf_t *me_ptr, capi_prop_t *prop_ptr)
{
   capi_err_t capi_result = CAPI_EOK;

   capi_buf_t *payload_ptr = &(prop_ptr->payload);

   AR_MSG(DBG_HIGH_PRIO,
          "Received input media format with payload port idx %ld, port info is_valid %ld",
          prop_ptr->port_info.port_index,
          prop_ptr->port_info.is_valid);

   if (sizeof(capi_media_fmt_v2_t) > payload_ptr->actual_data_len)
   {
      AR_MSG(DBG_ERROR_PRIO, "Input Media Format Bad param size %lu", payload_ptr->actual_data_len);
      CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
      return capi_result;
   }

   if (!prop_ptr->port_info.is_valid)
   {
      AR_MSG(DBG_ERROR_PRIO, "Media format port info is invalid");
      CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      return capi_result;
   }

   capi_media_fmt_v2_t *data_ptr = (capi_media_fmt_v2_t *)(payload_ptr->data_ptr);

   if (!capi_duty_cycling_buf_is_supported_media_type(data_ptr))
   {
      AR_MSG(DBG_ERROR_PRIO, "Media format is unsupported");
      CAPI_SET_ERROR(capi_result, CAPI_EFAILED);
      return capi_result;
   }

   uint32_t port_index = prop_ptr->port_info.port_index;
   if (port_index != DEFAULT_PORT_INDEX)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "Received port index %u expected port index %u ",
             port_index,
             (uint32_t)DEFAULT_PORT_INDEX);
      CAPI_SET_ERROR(capi_result, CAPI_EFAILED);
      return capi_result;
   }

   // Copy to local me_ptr
   me_ptr->in_media_fmt         = *data_ptr;
   me_ptr->is_input_mf_received = TRUE;

   AR_MSG(DBG_HIGH_PRIO,
          "input media format sampling_rate %u num_channels %u bits_per_sample %u",
          me_ptr->in_media_fmt.format.sampling_rate,
          me_ptr->in_media_fmt.format.num_channels,
          me_ptr->in_media_fmt.format.bits_per_sample);

   capi_result |= duty_cycling_buf_check_create_circular_buffer(me_ptr);

   // Set the media format to the ciccular buffer if the media format changed.
   if (capi_duty_cycling_buf_does_circular_buffer_exist(me_ptr))
   {

      AR_MSG(DBG_LOW_PRIO, "circular buffer is present ");

      capi_media_fmt_v2_t cur_media_fmt;
      memset(&cur_media_fmt, 0, sizeof(capi_media_fmt_v2_t));

      capi_result |= capi_duty_cycling_buf_get_circular_buffer_media_fmt(me_ptr, &cur_media_fmt);

      if (CAPI_FAILED(capi_result))
      {
         AR_MSG(DBG_ERROR_PRIO, "Get circular buffer media fmt returned bad result %d", capi_result);
         return capi_result;
      }
      else
      {
         if (!capi_cmn_media_fmt_equal(&cur_media_fmt, &(me_ptr->in_media_fmt)))
         {

            AR_MSG(DBG_HIGH_PRIO, "circular buffer already exists but MF not same ");
            capi_result |= capi_duty_cycling_buf_set_circular_buffer_media_fmt(me_ptr,
                                                                               &(me_ptr->in_media_fmt),
                                                                               me_ptr->cntr_frame_size_us);
         }
      }
   }
   else
   {
   }

   capi_duty_cycling_buf_island_entry_exit_util(me_ptr);

   return capi_result;
}

// Function being called from multiple places.
// in_media_fmt is initialised with CAPI_DATA_FORMAT_INVALID_VAL
//
capi_err_t duty_cycling_buf_check_create_circular_buffer(capi_duty_cycling_buf_t *me_ptr)
{
   capi_err_t result = CAPI_EOK;

   if ((!(me_ptr->is_buf_cfg_received)) || (0 == me_ptr->cntr_frame_size_us))
   {
      AR_MSG(DBG_MED_PRIO,
             "Circular buffer not yet created due to missing configuration "
             "is_buf_cfg_received %u, cntr_frame_size_us %ld. Continuing to wait for configuration.",
             me_ptr->is_buf_cfg_received,
             me_ptr->cntr_frame_size_us);
      return result;
   }

   // Take the max of container frame size and lower threshold recieved in set param
   me_ptr->lower_threshold_in_ms =
      MAX(me_ptr->lower_threshold_in_ms_rcvd_in_set_param, (me_ptr->cntr_frame_size_us / 1000));
   if (!capi_duty_cycling_buf_is_supported_media_type(&(me_ptr->in_media_fmt)))
   {
      AR_MSG(DBG_MED_PRIO, "Unsupported media format");
      return CAPI_EUNSUPPORTED;
   }

   if (!capi_duty_cycling_buf_does_circular_buffer_exist(me_ptr))
   {
      /*
      During BT LPI playback usecase upstream (i.e., input )runs in FTRT mode where as output is in RT mode
      there may be chance that signal triggers(which is the trigger to drain circular buffer)
      might not be coming while we accumulate data in circular buffer (since input is FTRT) ,
      Exit from island happens if lower_threshold_in_ms worth of data is in circular buffer.
      which means the remaining size left in the buffer is only (buffer_size_in_ms-lower_threshold_in_ms = 'x' say)
      The current logic to enter island next time is based on module receiving buffer_size_in_ms worth of data.
      since 'x' ms data is already present inside the buffer at the time of exit.
      If there is a delay in signal trigger, to avoid overrun (data dropping) when the buffer is full
      buffer_size_in_ms+lower_threshold_in_ms is used to create circular buffer
      */
      result |=
         capi_duty_cycling_buf_create_circular_buffer(me_ptr,
                                                      &(me_ptr->in_media_fmt),
                                                      (me_ptr->buffer_size_in_ms + me_ptr->lower_threshold_in_ms));

      if (CAPI_FAILED(result))
      {
         AR_MSG(DBG_MED_PRIO,
                "Failed to create circular buffer at input port  with size %ld ms. result 0x%lx",
                me_ptr->buffer_size_in_ms,
                result);
         return result;
      }

      AR_MSG(DBG_MED_PRIO,
             "Created circular buffer at input port  with size %ld ms.",
             (me_ptr->buffer_size_in_ms + me_ptr->lower_threshold_in_ms));

      result |= capi_duty_cycling_buf_set_circular_buffer_media_fmt(me_ptr,
                                                                    &(me_ptr->in_media_fmt),
                                                                    me_ptr->cntr_frame_size_us);
   }

   return result;
}

capi_err_t capi_duty_cycling_buf_create_circular_buffer(capi_duty_cycling_buf_t *me_ptr,
                                                        capi_media_fmt_v2_t *    media_fmt_ptr,
                                                        uint32_t                 size_ms)
{
   ar_result_t result = AR_EOK;

   if (me_ptr->stream_drv_ptr)
   {
      return AR_EOK;
   }

   AR_MSG(DBG_LOW_PRIO, "creating circular buffer");

   // All the input arguments are same for all channel buffers except buffer ID.
   spf_circ_buf_alloc_inp_args_t inp_args;
   memset(&inp_args, 0, sizeof(spf_circ_buf_alloc_inp_args_t));
   inp_args.preferred_chunk_size = DUTY_CYCLING_BUF_PREFERRED_CHUNK_SIZE;
   inp_args.heap_id              = (POSAL_HEAP_ID)me_ptr->heap_info.heap_id;
   inp_args.metadata_handler     = &me_ptr->metadata_handler;
   // Unique ID. using me_ptr
   inp_args.buf_id                = (uint32_t)me_ptr;
   inp_args.cb_info.event_cb      = duty_cycling_buf_circular_buffer_event_cb;
   inp_args.cb_info.event_context = (void *)me_ptr;

   // Creates circular buffers and returns handle
   if (AR_DID_FAIL(result |= spf_circ_buf_init(&me_ptr->stream_drv_ptr, &inp_args)))
   {
      AR_MSG(DBG_ERROR_PRIO, "circular buffer create failed during buf init");
      return AR_EFAILED;
   }

   // register the writer handle with the driver.
   if (AR_DID_FAIL(result |= spf_circ_buf_register_writer_client(me_ptr->stream_drv_ptr,
                                                                 size_ms * 1000, // us
                                                                 &me_ptr->writer_handle)))
   {
      AR_MSG(DBG_ERROR_PRIO, "circular buffer writer reg failed");
      return AR_EFAILED;
   }

   // register the reader with the driver handle.
   if (AR_DID_FAIL(result |= spf_circ_buf_register_reader_client(me_ptr->stream_drv_ptr,
                                                                 0, // not requesting any resize
                                                                 &me_ptr->reader_handle)))
   {
      AR_MSG(DBG_ERROR_PRIO, "circular buffer reader reg failed");
      return AR_EFAILED;
   }

   AR_MSG(DBG_MED_PRIO, "circular buffer Creation success ");
   return result;
}

capi_err_t duty_cycling_buf_circular_buffer_event_cb(void *          context_ptr,
                                                     spf_circ_buf_t *circ_buf_ptr,
                                                     uint32_t        event_id,
                                                     void *          event_info_ptr)
{
   capi_err_t               result = CAPI_EOK;
   capi_duty_cycling_buf_t *me_ptr = (capi_duty_cycling_buf_t *)context_ptr;

   if (NULL == circ_buf_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "NULL circ buf ptr in circ buf callback");
      return CAPI_EFAILED;
   }

   AR_MSG(DBG_HIGH_PRIO, "circular_buffer_event_cb event_id 0x%lX ", event_id);

   if (event_id == SPF_CIRC_BUF_EVENT_ID_OUTPUT_MEDIA_FORMAT)
   {
      capi_media_fmt_v2_t *mf_info_ptr   = (capi_media_fmt_v2_t *)event_info_ptr;
      bool_t               IS_INPUT_PORT = FALSE;

      me_ptr->out_media_fmt            = *mf_info_ptr;
      me_ptr->cir_buffer_raised_out_mf = TRUE;

      result |=
         capi_cmn_output_media_fmt_event_v2(&me_ptr->event_cb_info, mf_info_ptr, IS_INPUT_PORT, DEFAULT_PORT_INDEX);

      AR_MSG(DBG_HIGH_PRIO,
             "circular buf raised output media format sampling_rate %u num_channels %u bits_per_sample %u",
             mf_info_ptr->format.sampling_rate,
             mf_info_ptr->format.num_channels,
             mf_info_ptr->format.bits_per_sample);

	  capi_duty_cycling_buf_island_entry_exit_util(me_ptr);		 
   }

   return result;
}

capi_err_t capi_duty_cycling_buf_set_circular_buffer_media_fmt(capi_duty_cycling_buf_t *me_ptr,
                                                               capi_media_fmt_v2_t *    media_fmt_ptr,
                                                               uint32_t                 cntr_frame_size_us)
{
   AR_MSG(DBG_HIGH_PRIO, "set_circular_buffer_media_fmt");
   // set writers media format on the circular buffer
   return spf_circ_buf_set_media_format(me_ptr->writer_handle, media_fmt_ptr, cntr_frame_size_us);
}

capi_err_t capi_duty_cycling_buf_get_circular_buffer_media_fmt(capi_duty_cycling_buf_t *me_ptr,
                                                               capi_media_fmt_v2_t *    ret_mf_ptr)
{
   if ((!me_ptr->writer_handle) || (!ret_mf_ptr))
   {
      AR_MSG(DBG_ERROR_PRIO, "circular_buffer: null arg for  get_circular_buffer_media_fmt");
      return CAPI_EFAILED;
   }

   // get the current media format of the reader client.
   return spf_circ_buf_get_media_format(me_ptr->reader_handle, ret_mf_ptr);
}

capi_err_t capi_duty_cycling_buf_island_entry_exit_util(capi_duty_cycling_buf_t *me_ptr)
{
   capi_err_t result = CAPI_EOK;

   if ((!(me_ptr->cntr_frame_size_received)) || (!(me_ptr->is_input_mf_received)) ||
       (!(me_ptr->cir_buffer_raised_out_mf)))
   {
      AR_MSG(DBG_MED_PRIO,
             "cntr_frame_size_received %u, is_input_mf_received %u. cir_buffer_raised_out_mf %u Continuing to wait",
             me_ptr->cntr_frame_size_received,
             me_ptr->is_input_mf_received,
             me_ptr->cir_buffer_raised_out_mf);
      return result;
   }

   me_ptr->expected_input_bytes = capi_cmn_us_to_bytes_per_ch(me_ptr->cntr_frame_size_us,
                                                              me_ptr->in_media_fmt.format.sampling_rate,
                                                              me_ptr->in_media_fmt.format.bits_per_sample);

   me_ptr->expected_output_bytes = capi_cmn_us_to_bytes_per_ch(me_ptr->cntr_frame_size_us,
                                                               me_ptr->out_media_fmt.format.sampling_rate,
                                                               me_ptr->out_media_fmt.format.bits_per_sample);

   return result;
}

/*

Set_param on input:
Event to be raised from output -
Module has the capability to underrun i.e., to fill zeroes and
send output when ever signal trigger comes even if circular buffer has no data
set RT flag as TRUE even if upstream is NRT - This is independent of module
enabled or disabled

Set_param on output:
Event to be raised from input -
Even though DS may be RT in current usecase (placed in st container)
module to set  RT flag as FALSE. this will help in SPR working in non-timer mode

Caching of rt flag can be avoided based on assumption that module
is always placed in ST container i.e., downstream of this module will always be RT

Printing a warning

*/




capi_err_t capi_duty_cycling_buf_set_data_port_property(capi_duty_cycling_buf_t *me_ptr, capi_buf_t *payload_ptr)
{
   capi_err_t capi_result = CAPI_EOK;

   if (NULL == payload_ptr->data_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "Set port property, received null buffer");
      CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      return capi_result;
   }

   // Validate length.
   if (sizeof(intf_extn_param_id_is_rt_port_property_t) > payload_ptr->actual_data_len)
   {
      AR_MSG(DBG_ERROR_PRIO, "Set port property, Bad param size %lu", payload_ptr->actual_data_len);
      CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
      return capi_result;
   }

   intf_extn_param_id_is_rt_port_property_t *data_ptr =
      (intf_extn_param_id_is_rt_port_property_t *)(payload_ptr->data_ptr);

   AR_MSG(DBG_HIGH_PRIO,
          "data port property set param "
          "is_input %lu, port index %lu, is_rt %lu",
          data_ptr->is_input,
          data_ptr->port_index,
          data_ptr->is_rt);

   capi_buf_t                               event_payload;
   intf_extn_param_id_is_rt_port_property_t event = *data_ptr;
   event_payload.data_ptr                         = (int8_t *)&event;
   event_payload.actual_data_len = event_payload.max_data_len = sizeof(event);

   // If set param is on input send event on output and vice versa
   event.is_input = (data_ptr->is_input) ? FALSE : TRUE;

   if (data_ptr->is_input)
   {
      event.is_rt = TRUE; // RT flag is true to be sent from output
   }
   else
   {

      if (!data_ptr->is_rt)
      {
         AR_MSG(DBG_ERROR_PRIO,
                "WARNING: Downstream is expected to be RT "
                "but received is_rt %lu on output port during set_param",
                data_ptr->is_rt);
      }

      // Update only if module is enabled
      if (me_ptr->events_config.enable)
      {
         event.is_rt = FALSE; // RT flag is false
      }
   }

   capi_result = capi_cmn_raise_data_to_dsp_svc_event(&me_ptr->event_cb_info,
                                                      INTF_EXTN_EVENT_ID_IS_RT_PORT_PROPERTY,
                                                      &event_payload);

   return capi_result;
}

/*
- Assumption is module is at beginning of subgraph.
- If upstream(subgraph from different container) gets closed and opened,
  input port will be closed and opened again.
- Whenever input port is opened
  If module is enabled - Send NRT flag from input port

- No need to handle rt flag porpagation explicitly from output port based on port
  operation.Only once during init send rt flag as TRUE from output port.
- keeping under #if 0. Framrwork might handle by querying module  

*/

#if 0
capi_err_t capi_duty_cycling_buf_set_param_port_op(capi_duty_cycling_buf_t *me_ptr, capi_buf_t *payload_ptr)
{
   capi_err_t capi_result = CAPI_EOK;

   if (NULL == payload_ptr->data_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "Set property port operation, received null buffer");
      CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      return capi_result;
   }

   // Validate length.
   if (sizeof(intf_extn_data_port_operation_t) > payload_ptr->actual_data_len)
   {
      AR_MSG(DBG_ERROR_PRIO, "Set property for port operation, Bad param size %lu", payload_ptr->actual_data_len);
      CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
      return capi_result;
   }

   intf_extn_data_port_operation_t *data_ptr = (intf_extn_data_port_operation_t *)(payload_ptr->data_ptr);

   if ((sizeof(intf_extn_data_port_operation_t) + (data_ptr->num_ports * sizeof(intf_extn_data_port_id_idx_map_t))) >
       payload_ptr->actual_data_len)
   {
      AR_MSG(DBG_ERROR_PRIO, "Set property for port operation, Bad param size %lu", payload_ptr->actual_data_len);
      CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
      return capi_result;
   }

   uint32_t max_ports = data_ptr->is_input_port ? DUTY_CYCLING_BUF_MAX_INPUT_PORTS : DUTY_CYCLING_BUF_MAX_OUTPUT_PORTS;

   // For each port in the operation payload.
   for (uint32_t iter = 0; iter < data_ptr->num_ports; iter++)
   {

      uint32_t port_id    = data_ptr->id_idx[iter].port_id;
      uint32_t port_index = data_ptr->id_idx[iter].port_index;

      AR_MSG(DBG_HIGH_PRIO,
             "port id = %lu, idx = %lu is_input_port %u opcode %u",
             port_id,
             port_index,
             data_ptr->is_input_port,
             (uint32_t)data_ptr->opcode);

      // Validate port index doesn't go out of bounds.
      if (port_index >= max_ports || (port_index != DEFAULT_PORT_INDEX))
      {
         AR_MSG(DBG_ERROR_PRIO,
                "Bad parameter in id-idx map on port %lu in payload, port_index = %lu, "
                "is_input = %lu, max ports = %d",
                iter,
                port_index,
                data_ptr->is_input_port,
                max_ports);

         CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
         return capi_result;
      }

      switch ((uint32_t)data_ptr->opcode)
      {
         case INTF_EXTN_DATA_PORT_OPEN:
         {

            if (data_ptr->is_input_port)
            {
               bool is_rt = TRUE;
               if (me_ptr->events_config.enable)
               {
                  is_rt = FALSE;
               }
               capi_result |= capi_duty_cycling_buf_raise_rt_port_prop_event(me_ptr,
                                                                             TRUE ,
                                                                             is_rt ,
                                                                             DEFAULT_PORT_INDEX);
            }

            break;
         }
         case INTF_EXTN_DATA_PORT_CLOSE:
         case INTF_EXTN_DATA_PORT_START:
         case INTF_EXTN_DATA_PORT_STOP:
         {
            break;
         }
         default:
         {
            AR_MSG(DBG_HIGH_PRIO, "Port operation opcode %lu. Not supported.", data_ptr->opcode);
            CAPI_SET_ERROR(capi_result, CAPI_EUNSUPPORTED);
            break;
         }
      }
   }
   return capi_result;
}

#endif
