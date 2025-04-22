/**
 *   \file capi_duty_cycling_buf_island.c
 *   \brief
 *        This file contains CAPI implementation of duty cycling buffering module
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "capi_duty_cycling_buf.h"
#include "capi_duty_cycling_buf_utils.h"
#include "posal_power_mgr.h"

/*==============================================================================
   Local Defines
==============================================================================*/

static capi_vtbl_t vtbl = { capi_duty_cycling_buf_process,        capi_duty_cycling_buf_end,
                            capi_duty_cycling_buf_set_param,      capi_duty_cycling_buf_get_param,
                            capi_duty_cycling_buf_set_properties, capi_duty_cycling_buf_get_properties };

capi_vtbl_t *capi_duty_cycling_buf_get_vtbl()
{
   return &vtbl;
}

/*------------------------------------------------------------------------
  Function name: capi_duty_cycling_buf_process
  Processes an input buffer and generates an output buffer.
 * -----------------------------------------------------------------------*/
capi_err_t capi_duty_cycling_buf_process(capi_t *capi_ptr, capi_stream_data_t *input[], capi_stream_data_t *output[])
{
   capi_err_t            capi_result                  = CAPI_EOK;
   spf_circ_buf_result_t circ_buf_result              = SPF_CIRCBUF_SUCCESS;
   uint32_t              data_rcvd_in_current_process = 0;
   uint32_t              data_pulled_from_circ_buffer = 0;

   if (NULL == capi_ptr)
   {
      AR_MSG_ISLAND(DBG_ERROR_PRIO, "CAPI pointer cannot be null");
      return CAPI_EFAILED;
   }

   capi_duty_cycling_buf_t *me_ptr = (capi_duty_cycling_buf_t *)(capi_ptr);
   if (!capi_duty_cycling_buf_does_circular_buffer_exist(me_ptr))
   {
      AR_MSG_ISLAND(DBG_ERROR_PRIO, "Circular buffer doesn't exist");
      return CAPI_EFAILED;
   }

   // Function checks erasure on inptu being false also inaddition to actual data length
   bool_t is_in_provided                 = capi_duty_cycling_buf_in_has_data(input);
   bool_t is_out_provided                = capi_duty_cycling_buf_out_has_space(output);
   bool_t is_circular_buf_empty          = capi_duty_cycling_buf_is_circular_buffer_empty(me_ptr);
   bool_t is_partial_data_present_in_out = capi_duty_cycling_buf_out_has_partial_data(output);

   if (is_partial_data_present_in_out)
   {
      AR_MSG_ISLAND(DBG_ERROR_PRIO, "Partial data in output. offsets not being handled");
   }

   bool_t ALLOW_OVERFLOW_FALSE = FALSE;

   if (is_in_provided)
   {

      // Since module blocked input for both signal and data triggers
      // we should not receive input in DUTY_CYCLING_AFTER_BUFFERING
      // printing  message
      if (me_ptr->current_status == DUTY_CYCLING_AFTER_BUFFERING)
      {
         AR_MSG_ISLAND(DBG_HIGH_PRIO, " Received input data despite blocking input ");
      }

      circ_buf_result = spf_circ_buf_write_one_frame(me_ptr->writer_handle,
                                                     (capi_stream_data_t *)input[DEFAULT_PORT_INDEX],
                                                     ALLOW_OVERFLOW_FALSE);

      if (SPF_CIRCBUF_FAIL == circ_buf_result)
      {
         AR_MSG_ISLAND(DBG_HIGH_PRIO, "Failed while writing data to the circular buffer.");
         return CAPI_EFAILED;
      }

      // If container sends frame size worth of bytes in general module receives this most of the time,
      // dont do any caculation.mainly to avoid division in every process call
      if (input[DEFAULT_PORT_INDEX]->buf_ptr[0].actual_data_len == me_ptr->expected_input_bytes)
      {
         data_rcvd_in_current_process = me_ptr->cntr_frame_size_us;
      }
      else
      {
         data_rcvd_in_current_process = (capi_cmn_bytes_to_us(input[DEFAULT_PORT_INDEX]->buf_ptr[0].actual_data_len,
                                                              me_ptr->in_media_fmt.format.sampling_rate,
                                                              me_ptr->in_media_fmt.format.bits_per_sample,
                                                              1,
                                                              NULL));
      }

      me_ptr->data_rcvd_in_ms_while_buffering += data_rcvd_in_current_process;
      me_ptr->data_remaining_in_ms_in_circ_buf += data_rcvd_in_current_process;
   }

   if (is_out_provided)
   {
      /*If output is provided but circular buffer is empty fill zeroes and send*/
      if (is_circular_buf_empty)
      {
         capi_result = capi_duty_cycling_buf_underrun(me_ptr, output);
      }
      else /*circular buffer is not empty. So read from cir buffer*/
      {
         circ_buf_result =
            spf_circ_buf_read_one_frame(me_ptr->reader_handle, (capi_stream_data_t *)output[DEFAULT_PORT_INDEX]);

         if (SPF_CIRCBUF_UNDERRUN == circ_buf_result)
         {
            return CAPI_ENEEDMORE;
         }
         else if (SPF_CIRCBUF_FAIL == circ_buf_result)
         {
            AR_MSG_ISLAND(DBG_HIGH_PRIO, "Failed reading data from the circular buffer.");
            return CAPI_EFAILED;
         }

         // Since we have read from circular buffer make erasure flag as FALSE;
         output[DEFAULT_PORT_INDEX]->flags.erasure = FALSE;

         if (me_ptr->expected_output_bytes == output[DEFAULT_PORT_INDEX]->buf_ptr[0].actual_data_len)
         {
            data_pulled_from_circ_buffer = me_ptr->cntr_frame_size_us;
         }
         else
         {

            data_pulled_from_circ_buffer = (capi_cmn_bytes_to_us(output[DEFAULT_PORT_INDEX]->buf_ptr[0].actual_data_len,
                                                                 me_ptr->out_media_fmt.format.sampling_rate,
                                                                 me_ptr->out_media_fmt.format.bits_per_sample,
                                                                 1,
                                                                 NULL));
         }

         me_ptr->data_remaining_in_ms_in_circ_buf -= data_pulled_from_circ_buffer;
      }
   }

   // Only if module is enabled, interaction with DCM etc is needed
   if (me_ptr->events_config.enable)
   {
      if (me_ptr->current_status == DUTY_CYCLING_AFTER_BUFFERING)
      {
         if (me_ptr->data_remaining_in_ms_in_circ_buf <= (me_ptr->lower_threshold_in_ms * 1000))
         {
            posal_island_trigger_island_exit();
            me_ptr->payload.dcm_island_change_request = SPF_MSG_CMD_DCM_REQ_FOR_ISLAND_EXIT;
            me_ptr->payload.signal_ptr                = NULL;
            capi_result |= capi_duty_cycling_buf_update_tgp_while_buffering(me_ptr);
            // vote for non-island based on DCM API
            capi_result |= posal_power_mgr_send_command(me_ptr->payload.dcm_island_change_request,
                                                        &me_ptr->payload,
                                                        sizeof(dcm_island_control_payload_t));
            me_ptr->current_status = DUTY_CYCLING_WHILE_BUFFERING;
         }
      }
      else
      {
         if (me_ptr->data_rcvd_in_ms_while_buffering >= (me_ptr->buffer_size_in_ms * 1000))
         {
            // We have buffered enough data. Change trigger policy before calling DCM
            // Data Trigger: Block input and Block output
            // Signal Trigger:Block input and listen to output+
            capi_result |= capi_duty_cycling_buf_update_tgp_after_buffering(me_ptr);
            // vote for island based on DCM API
            me_ptr->payload.dcm_island_change_request = SPF_MSG_CMD_DCM_REQ_FOR_UNBLOCK_ISLAND_ENTRY;
            me_ptr->payload.signal_ptr                = NULL;
            capi_result |= posal_power_mgr_send_command(me_ptr->payload.dcm_island_change_request,
                                                        &me_ptr->payload,
                                                        sizeof(dcm_island_control_payload_t));
            // reset data received  to 0
            me_ptr->data_rcvd_in_ms_while_buffering = 0;
            me_ptr->current_status                  = DUTY_CYCLING_AFTER_BUFFERING;
         }
      }
   }

   return capi_result;
}

bool_t capi_duty_cycling_buf_is_circular_buffer_empty(capi_duty_cycling_buf_t *        me_ptr)
{
   if (capi_duty_cycling_buf_does_circular_buffer_exist(me_ptr))
   {
      bool_t is_empty = FALSE;
      spf_circ_buf_driver_is_buffer_empty(me_ptr->reader_handle, &is_empty);
      return is_empty;
   }
   else
   {
      // Buffer doesn't exist, so it's empty.
      return TRUE;
   }
   return FALSE;
}

bool_t capi_duty_cycling_buf_is_circular_buffer_full(capi_duty_cycling_buf_t *        me_ptr)
{
   bool_t is_full = FALSE;

   if (capi_duty_cycling_buf_does_circular_buffer_exist(me_ptr))
   {
      spf_circ_buf_driver_is_buffer_full((me_ptr)->reader_handle, &is_full);
   }

   return is_full;
}

bool_t capi_duty_cycling_buf_does_circular_buffer_exist(capi_duty_cycling_buf_t *        me_ptr)
{
   if (me_ptr->stream_drv_ptr)
   {
      return TRUE;
   }
   return FALSE;
}

void capi_duty_cycling_buf_get_unread_bytes(capi_duty_cycling_buf_t *        me_ptr,uint32_t *unread_bytes)
{
   if (capi_duty_cycling_buf_does_circular_buffer_exist(me_ptr))
   {
      spf_circ_buf_get_unread_bytes(me_ptr->reader_handle, unread_bytes);
   }

   return ;
}

capi_err_t capi_duty_cycling_buf_underrun(capi_duty_cycling_buf_t *me_ptr, capi_stream_data_t *output[])
{
   capi_err_t result = CAPI_EOK;

   if (!capi_duty_cycling_buf_out_has_space(output))
   {
      AR_MSG_ISLAND(DBG_HIGH_PRIO, "not producing zeros, no space in output");
      return result;
   }

   uint32_t frame_size_per_ch = capi_cmn_us_to_bytes_per_ch(me_ptr->cntr_frame_size_us,
                                                            me_ptr->out_media_fmt.format.sampling_rate,
                                                            me_ptr->out_media_fmt.format.bits_per_sample);
   uint32_t num_channels      = me_ptr->out_media_fmt.format.num_channels;

   if (frame_size_per_ch > output[DEFAULT_PORT_INDEX]->buf_ptr[0].max_data_len)
   {
      AR_MSG_ISLAND(DBG_HIGH_PRIO, "output space is less than container frame size");
      frame_size_per_ch = MIN(frame_size_per_ch, output[DEFAULT_PORT_INDEX]->buf_ptr[0].max_data_len);
   }

   for (uint32_t ch = 0; ch < num_channels; ch++)
   {
      memset(output[DEFAULT_PORT_INDEX]->buf_ptr[ch].data_ptr, 0, frame_size_per_ch);
      output[DEFAULT_PORT_INDEX]->buf_ptr[ch].actual_data_len = frame_size_per_ch;
   }
   AR_MSG_ISLAND(DBG_HIGH_PRIO, "underrun: module is filling zeros");
   output[DEFAULT_PORT_INDEX]->flags.erasure = TRUE;

   return result;
}
