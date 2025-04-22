/**
 * \file capi_sync_process_ec_mode.c
 * \brief
 *      * mplementation of utility functions for module port handling
 *  
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "capi_sync_i.h"
#include "module_cmn_api.h"

/* =========================================================================
 * Static function declarations
 * =========================================================================*/
static capi_err_t ec_sync_process_bypass(capi_sync_t *       me_ptr,
                                         capi_stream_data_t *input[],
                                         capi_stream_data_t *output[]);

static capi_err_t ec_sync_process_only_sec_active(capi_sync_t *       me_ptr,
                                                  capi_stream_data_t *input[],
                                                  capi_stream_data_t *output[]);

static capi_err_t capi_sync_pass_through_data(capi_sync_t *        me_ptr,
                                              capi_stream_data_t * input[],
                                              capi_stream_data_t * output[],
                                              capi_sync_in_port_t *in_port_ptr);

/* =========================================================================
 * Function definitions
 * =========================================================================*/

capi_err_t ec_sync_mode_process(capi_sync_t *me_ptr, capi_stream_data_t *input[], capi_stream_data_t *output[])
{
   capi_err_t capi_result        = CAPI_EOK;
   bool_t     ENABLE_THRESHOLD   = TRUE;
   bool_t     CHECK_PROCESS_BUF  = TRUE;
   bool_t     CHECK_INTERNAL_BUF = FALSE;
   bool_t     data_gap_found     = FALSE;
   bool_t     any_data_found     = FALSE;

   capi_sync_in_port_t *pri_in_port_ptr = capi_sync_get_in_port_from_port_id(me_ptr, SYNC_EC_PRIMARY_IN_PORT_ID);
   capi_sync_in_port_t *sec_in_port_ptr = capi_sync_get_in_port_from_port_id(me_ptr, SYNC_EC_SECONDARY_IN_PORT_ID);

   if (!pri_in_port_ptr || !sec_in_port_ptr)
   {
      return CAPI_EFAILED;
   }

   // If threshold is not configured, EC sync behaves a pass through module
   if (!me_ptr->module_config.frame_len_us)
   {
      return ec_sync_process_bypass(me_ptr, input, output);
   }

   capi_result |= capi_sync_validate_io_bufs(me_ptr, input, output, &any_data_found);

   if (CAPI_FAILED(capi_result))
   {
      return capi_result;
   }

   // Handle the case where no data was sent. In this case we need to only handle data flow gap and return.
   if (!any_data_found)
   {
#ifdef CAPI_SYNC_DEBUG
      AR_MSG(DBG_MED_PRIO, "capi sync process(): No data sent. Handling data gap flag and returning.");
#endif

      capi_result |= capi_sync_handle_dfg(me_ptr, input, output, &data_gap_found);
      return capi_result;
   }

   // If primary port is not running, then buffer only secondary, but do not send out any data
   if (!capi_sync_is_path_running(me_ptr, pri_in_port_ptr->cmn.self_index))
   {
      return ec_sync_process_only_sec_active(me_ptr, input, output);
   }

   if (STATE_SYNCED == me_ptr->synced_state)
   {
      bool_t send_secondary = FALSE;

      // No data gap was found.

      // Check that threshold was met, and if not then return without consuming.
      // If threshold is not met on primary port.
      if (!capi_sync_port_meets_threshold(me_ptr, pri_in_port_ptr, input, CHECK_PROCESS_BUF))
      {
         // Don't consume any input, mark input as 0 length.
         capi_sync_mark_input_unconsumed(me_ptr, input, pri_in_port_ptr);
         capi_sync_mark_input_unconsumed(me_ptr, input, sec_in_port_ptr);

         AR_MSG(DBG_MED_PRIO,
                "capi sync process(): in steady state but did not receive threshold amount of data. "
                "Not consuming any data, and returning result 0x%lx.",
                capi_result);
         return capi_result;
      }

      // Only check secondary if secondary is running.
      if (capi_sync_is_path_running(me_ptr, sec_in_port_ptr->cmn.self_index))
      {
         if (!capi_sync_port_meets_threshold(me_ptr, sec_in_port_ptr, input, CHECK_PROCESS_BUF))
         {
            // Don't consume any input, mark input as 0 length.
            capi_sync_mark_input_unconsumed(me_ptr, input, pri_in_port_ptr);
            capi_sync_mark_input_unconsumed(me_ptr, input, sec_in_port_ptr);
            AR_MSG(DBG_MED_PRIO,
                   "capi sync process(): in steady state but did not receive threshold amount of data. "
                   "Not consuming any data, and returning result 0x%lx.",
                   capi_result);
            return capi_result;
         }
      }

      send_secondary = capi_sync_is_path_running(me_ptr, sec_in_port_ptr->cmn.self_index);

#ifdef CAPI_SYNC_DEBUG
      AR_MSG(DBG_MED_PRIO,
             "capi sync process(): steady state, doing pass-through. secondary port is running? %ld.",
             send_secondary);
#endif

      // Pass through both ports by buffering and then sending from buffer. Need to do this in case
      // the secondary port has data in its buffer.
      capi_result |= capi_sync_buffer_new_data(me_ptr, input, pri_in_port_ptr);

      if (send_secondary)
         capi_result |= capi_sync_buffer_new_data(me_ptr, input, sec_in_port_ptr);

      // Even if secondary isn't running, we need to send it if data flow gap is pending.
      send_secondary |= sec_in_port_ptr->pending_data_flow_gap;

      if (send_secondary)
         capi_result |= capi_sync_send_buffered_data(me_ptr, output, me_ptr->synced_state, sec_in_port_ptr, input);

      capi_result |= capi_sync_handle_dfg(me_ptr, input, output, &data_gap_found);
      return capi_result;
   }

   // At this point in the function we are starting. Threshold is disabled.

   // Check for port stop. if so we should drop all data and return.
   // TODO(claguna): If port stop on primary, still need to pass through primary.
   capi_result |= capi_sync_handle_dfg(me_ptr, input, output, &data_gap_found);
   if (data_gap_found)
   {
#ifdef CAPI_SYNC_DEBUG
      AR_MSG(DBG_MED_PRIO, "capi sync process(): port was stopped during sync stage. Dropping data and stopping.");
#endif
      return capi_result;
   }

// At this point there is no data gap. So standard starting handling is below.

#ifdef CAPI_SYNC_DEBUG
   AR_MSG(DBG_MED_PRIO, "capi sync process(): sync stage, buffering data.");
#endif

   // Buffer new data.
   bool_t send_secondary = capi_sync_is_path_running(me_ptr, sec_in_port_ptr->cmn.self_index);
   if (send_secondary)
   {
      capi_result |= capi_sync_buffer_new_data(me_ptr, input, sec_in_port_ptr);
   }
   capi_result |= capi_sync_buffer_new_data(me_ptr, input, pri_in_port_ptr);

   // Send data if we meet the threshold on primary port. If not, we just buffer data and wait for more data.
   if (capi_sync_port_meets_threshold(me_ptr, pri_in_port_ptr, input, CHECK_INTERNAL_BUF))
   {

#ifdef CAPI_SYNC_DEBUG
      AR_MSG(DBG_MED_PRIO,
             "capi sync process(): sync stage, threshold met. Sending out data and moving to steady state.");
#endif

      // Send out buffered data, padding initial zeros to partial secondary.
      capi_sync_send_buffered_data(me_ptr, output, me_ptr->synced_state, pri_in_port_ptr, input);
      capi_sync_send_buffered_data(me_ptr, output, me_ptr->synced_state, sec_in_port_ptr, input);
      me_ptr->synced_state = STATE_SYNCED;

      sec_in_port_ptr->is_threshold_disabled = FALSE;

      // Enable threshold.
      capi_result |= capi_sync_raise_event_toggle_threshold(me_ptr, ENABLE_THRESHOLD);
   }

   return capi_result;
}

static capi_err_t ec_sync_process_bypass(capi_sync_t *me_ptr, capi_stream_data_t *input[], capi_stream_data_t *output[])
{
   capi_err_t capi_result    = CAPI_EOK;
   bool_t     data_gap_found = FALSE;

   capi_sync_in_port_t *pri_in_port_ptr = capi_sync_get_in_port_from_port_id(me_ptr, SYNC_EC_PRIMARY_IN_PORT_ID);
   capi_sync_in_port_t *sec_in_port_ptr = capi_sync_get_in_port_from_port_id(me_ptr, SYNC_EC_SECONDARY_IN_PORT_ID);

   if (!pri_in_port_ptr || !sec_in_port_ptr)
   {
      return CAPI_EFAILED;
   }

   // If primary port isn't started, then just drop data on the secondary path.
   if (!capi_sync_input_has_data(input, pri_in_port_ptr->cmn.self_index))
   {
#ifdef CAPI_SYNC_DEBUG
      AR_MSG(DBG_MED_PRIO, "capi sync process(): Threshold not set, and no primary data. Dropping secondary data.");
#endif

      return capi_result;
   }

#ifdef CAPI_SYNC_DEBUG
   AR_MSG(DBG_ERROR_PRIO, "capi sync process(): Threshold not set, passing through data.");
#endif

   // Pass through data on channels where data is present.
   capi_result |= capi_sync_pass_through_data(me_ptr, input, output, pri_in_port_ptr);

   // If secondary input exists
   // RR: Why not same check as line 224?
   if (input[sec_in_port_ptr->cmn.self_index] && input[sec_in_port_ptr->cmn.self_index]->buf_ptr)
   {

      capi_result |= capi_sync_pass_through_data(me_ptr, input, output, sec_in_port_ptr);
   }
   capi_result |= capi_sync_handle_dfg(me_ptr, input, output, &data_gap_found);

   return capi_result;
}

static capi_err_t ec_sync_process_only_sec_active(capi_sync_t *       me_ptr,
                                                  capi_stream_data_t *input[],
                                                  capi_stream_data_t *output[])
{
   capi_err_t capi_result = CAPI_EOK;

   if (STATE_SYNCED == me_ptr->synced_state)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi sync process(): state is synced but primary port is not yet running.");
      return CAPI_EFAILED;
   }

   capi_sync_in_port_t *sec_in_port_ptr = capi_sync_get_in_port_from_port_id(me_ptr, SYNC_EC_SECONDARY_IN_PORT_ID);

   if (!sec_in_port_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi sync only_sec_active() error, encountered null sec_in_port_ptr");
      return CAPI_EFAILED;
   }

   // If only secondary is running, we should simply continue to buffer data on secondary. If we overflow,
   // then drop initial data as we should never send out secondary without primary.
   if (capi_sync_is_path_running(me_ptr, sec_in_port_ptr->cmn.self_index))
   {
      capi_result |= capi_sync_buffer_new_data(me_ptr, input, sec_in_port_ptr);

      // Handle data gap. If we get a stop, we should immediately propagate the data flow gap and
      // drop secondary data. No need to send data content discontinuity since in secondary-only case no
      // data was sent downstream since data flow start anyways.
   }

   return capi_result;
}

/**
 * Pass through data from passed in input index to passed in output index.
 */
static capi_err_t capi_sync_pass_through_data(capi_sync_t *        me_ptr,
                                              capi_stream_data_t * input[],
                                              capi_stream_data_t * output[],
                                              capi_sync_in_port_t *in_port_ptr)
{
   capi_err_t capi_result          = CAPI_EOK;
   uint32_t   bytes_to_copy_per_ch = 0;
   //uint32_t   bytes_copied_per_ch  = 0;

   uint32_t in_index     = in_port_ptr->cmn.self_index;
   uint32_t out_index    = in_port_ptr->cmn.conn_index;
   uint32_t num_channels = in_port_ptr->media_fmt.format.num_channels;

   // Validate that port indices are present.
   if ((!(input[in_index] && input[in_index]->buf_ptr)) || (!(output[out_index] && output[out_index]->buf_ptr)))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "capi sync process(): input or output buffers not present input idx: %ld, in ptr: 0x%lx, "
             "output idx: %ld, out ptr 0x%lx.",
             in_index,
             input[in_index]->buf_ptr,
             out_index,
             output[out_index]->buf_ptr);
      return CAPI_EFAILED;
   }

   // Don't access output out of bounds, only copy present input.
   bytes_to_copy_per_ch = MIN(input[in_index]->buf_ptr[0].actual_data_len, output[out_index]->buf_ptr[0].max_data_len);

   for (uint32_t ch = 0; ch < num_channels; ch++)
   {
     memscpy(output[out_index]->buf_ptr[ch].data_ptr,
                                    output[out_index]->buf_ptr[ch].max_data_len,
                                    input[in_index]->buf_ptr[ch].data_ptr,
                                    bytes_to_copy_per_ch);

      output[out_index]->buf_ptr[ch].actual_data_len = bytes_to_copy_per_ch;
      input[in_index]->buf_ptr[ch].actual_data_len   = bytes_to_copy_per_ch;
   }

   // Copy flags/timestamp through correct port.
   output[out_index]->flags = input[in_index]->flags;
   if (input[in_index]->flags.is_timestamp_valid)
   {
      output[out_index]->timestamp = input[in_index]->timestamp;
   }

   return capi_result;
}
