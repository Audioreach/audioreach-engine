/**
 * \file capi_sync_data_utils.c
 * \brief
 *     Implementation of utility functions for capi data handling (process function, data buffers, data processing, etc).
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
static capi_err_t generic_sync_mark_port_as_stopped(capi_sync_t *me_ptr, uint32_t in_index);
static bool_t capi_sync_move_eos_dfg_to_end(capi_sync_t *me_ptr, uint32_t end_offset, capi_stream_data_v2_t *sdata_ptr);
static void capi_sync_adjust_md_after_prop(capi_sync_t *          me_ptr,
                                           capi_stream_data_v2_t *sdata_ptr,
                                           uint32_t               samples_consumed_from_internal_buf);

/* =========================================================================
 * Function definitions
 * =========================================================================*/

/**
 * Checks each input port for presence of data flow gap. If found:
 * 1. Sets port state to stop.
 * 2. Drops internally buffered data on that port.
 * 3. Propagates data flow gap on output.
 */
capi_err_t capi_sync_handle_dfg(capi_sync_t *       me_ptr,
                                capi_stream_data_t *input[],
                                capi_stream_data_t *output[],
                                bool_t *            data_gap_found_ptr)
{
   capi_err_t capi_result = CAPI_EOK;

   *data_gap_found_ptr = FALSE;

   for (uint32_t idx = 0; idx < me_ptr->num_in_ports; idx++)
   {
      // No need for NULL check since the loop runs till num_in_ports and all ports are init with default state
      capi_sync_in_port_t *in_port_ptr = capi_sync_get_in_port_from_index(me_ptr, idx);
      if (!in_port_ptr)
      {
         AR_MSG(DBG_ERROR_PRIO, "capi sync handle_data_gap_flag() error, encountered null in_port_ptr");
         return CAPI_EFAILED;
      }

      if (CAPI_PORT_STATE_CLOSED != in_port_ptr->cmn.state)
      {
         if (in_port_ptr->pending_data_flow_gap)
         {
            *data_gap_found_ptr = TRUE;
            capi_result |= capi_sync_in_port_stop(me_ptr, in_port_ptr, output);
         }
      }
   }

   return capi_result;
}

/**
 * Helper function to allocate capi buffer memory for the port buffer.
 */
capi_err_t capi_sync_allocate_port_buffer(capi_sync_t *me_ptr, capi_sync_in_port_t *in_port_ptr)
{
   capi_err_t capi_result  = CAPI_EOK;
   uint32_t   num_channels = in_port_ptr->media_fmt.format.num_channels;

   // If it already exists, free the buffer.
   if (in_port_ptr->int_bufs_ptr)
   {
      capi_sync_deallocate_port_buffer(me_ptr, in_port_ptr);
   }

   // Allocate buffers to fit the threshold plus one extra upstream frame.
   uint32_t buf_size_bytes = in_port_ptr->frame_size_bytes_per_ch + in_port_ptr->threshold_bytes_per_ch;

   // Allocate and zero memory. Memory is allocated in one contiguous block, starting out with
   // capi_buf_t structures and followed by data.
   uint32_t mem_size = (sizeof(capi_buf_t) + buf_size_bytes) * num_channels;

   AR_MSG(DBG_MED_PRIO,
          "capi sync: allocating input port buffer for size %d, port index = %ld",
          buf_size_bytes,
          in_port_ptr->cmn.self_index);

   in_port_ptr->int_bufs_ptr = (capi_buf_t *)posal_memory_malloc(mem_size, (POSAL_HEAP_ID)me_ptr->heap_info.heap_id);

   if (NULL == in_port_ptr->int_bufs_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi v2 sync: Couldn't allocate memory for input port buffer.");
      return CAPI_ENOMEMORY;
   }

   memset(in_port_ptr->int_bufs_ptr, 0, sizeof(capi_buf_t) * num_channels);

   // Struct memory is already addressable from the bufs_ptr array. Use mem_ptr to setup channel pointers
   // starting at address following capi_buf_t structs.
   int8_t *mem_ptr = ((int8_t *)(in_port_ptr->int_bufs_ptr)) + (sizeof(capi_buf_t) * num_channels);

   for (uint32_t ch = 0; ch < num_channels; ch++)
   {
      capi_buf_t *buf_ptr      = &in_port_ptr->int_bufs_ptr[ch];
      buf_ptr->actual_data_len = 0;
      buf_ptr->max_data_len    = buf_size_bytes;
      buf_ptr->data_ptr        = mem_ptr;

      // Move mem_ptr to next channel.
      mem_ptr += buf_size_bytes;
   }
   return capi_result;
}

/**
 * Helper function to free capi buffer memory for the port buffer.
 */
void capi_sync_deallocate_port_buffer(capi_sync_t *me_ptr, capi_sync_in_port_t *in_port_ptr)
{
   if (!in_port_ptr)
   {
      return;
   }
   if (in_port_ptr->int_bufs_ptr)
   {
      posal_memory_free(in_port_ptr->int_bufs_ptr);
      in_port_ptr->int_bufs_ptr = NULL;
   }
}

/**
 * Check if passed in port has threshold amount of data buffered. There is an additional
 * amount of padding available which is equal to the upstream frame length (assumed to be 1ms for EC mode)
 * Boolean passed to either check in the process buffer (input) or in the me_ptr internal buffer.
 */
bool_t capi_sync_port_meets_threshold(capi_sync_t *        me_ptr,
                                      capi_sync_in_port_t *in_port_ptr,
                                      capi_stream_data_t * input[],
                                      bool_t               check_process_buf)
{
   capi_buf_t *buf_ptr  = NULL;
   uint32_t    in_index = in_port_ptr->cmn.self_index;

   buf_ptr = check_process_buf ? input[in_index]->buf_ptr : in_port_ptr->int_bufs_ptr;

   if (!buf_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi sync process(): ERROR input port not provided : %ld", in_index);
      return FALSE;
   }

   // Assume all channels have same amount of data buffered.
   return buf_ptr->actual_data_len >= in_port_ptr->threshold_bytes_per_ch;
}

/**
 * Mark capi input as un-consumed
 */
capi_err_t capi_sync_mark_input_unconsumed(capi_sync_t *        me_ptr,
                                           capi_stream_data_t * input[],
                                           capi_sync_in_port_t *in_port_ptr)
{
   capi_err_t capi_result  = CAPI_EOK;
   uint32_t   in_index     = in_port_ptr->cmn.self_index;
   uint32_t   num_channels = in_port_ptr->media_fmt.format.num_channels;

   // Validate that port indices are present. Otherwise skip for this port.
   if ((!(input[in_index] && input[in_index]->buf_ptr)))
   {
      AR_MSG(DBG_MED_PRIO, "capi sync: input %ld not present, can't be marked as unconsumed.", in_index);
      return capi_result;
   }

#ifdef CAPI_SYNC_DEBUG
   AR_MSG(DBG_MED_PRIO, "capi sync: input data %ld marked as unconsumed.", in_index);
#endif

   for (uint32_t ch = 0; ch < num_channels; ch++)
   {
      input[in_index]->buf_ptr[ch].actual_data_len = 0;
   }

   return capi_result;
}

/**
 * Buffers data on input into port buffers. Find port indices into input arg based on id-idx mapping.
 * This checks if threshold is exceeded on either port and errors if so.
 * If exceeded on secondary we need to drop old data and print. If exceeded on input we need to
 * send out and then later buffer remaining input. TODO(claguna): check if required.
 */
capi_err_t capi_sync_buffer_new_data(capi_sync_t *me_ptr, capi_stream_data_t *input[], capi_sync_in_port_t *in_port_ptr)
{
   capi_err_t             capi_result  = CAPI_EOK;
   uint32_t               in_index     = in_port_ptr->cmn.self_index;
   capi_stream_data_v2_t *input_v2_ptr = (capi_stream_data_v2_t *)input[in_index];

   if (CAPI_STREAM_V2 != input[in_index]->flags.stream_data_version)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi sync process(): stream_version is not v2.");
      return CAPI_EFAILED;
   }

   if (!in_port_ptr->int_bufs_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi sync process(): input buffer not allocated for port = %ld", in_index);
      return CAPI_EFAILED;
   }

   if (!(input[in_index] && input[in_index]->buf_ptr))
   {
      AR_MSG(DBG_ERROR_PRIO, "capi sync process(): input not present for port = %ld ", in_index);
      return CAPI_EFAILED;
   }

   uint32_t num_channels                   = in_port_ptr->media_fmt.format.num_channels;
   uint32_t in_bytes_before                = input[in_index]->buf_ptr[0].actual_data_len;
   uint32_t bytes_in_int_buf_before_per_ch = in_port_ptr->int_bufs_ptr[0].actual_data_len;

   // Copy data from capi_input to local buffer.
   for (uint32_t ch = 0; ch < num_channels; ch++)
   {
      int8_t *write_ptr = in_port_ptr->int_bufs_ptr[ch].data_ptr + in_port_ptr->int_bufs_ptr[ch].actual_data_len;
      // if there is just one input port then do not buffer extra data.
      uint32_t max_data_len = (1 == me_ptr->num_opened_in_ports) ? in_port_ptr->threshold_bytes_per_ch
                                                                 : in_port_ptr->int_bufs_ptr[ch].max_data_len;
      uint32_t write_size = (max_data_len > in_port_ptr->int_bufs_ptr[ch].actual_data_len)
                               ? max_data_len - in_port_ptr->int_bufs_ptr[ch].actual_data_len
                               : 0;
      uint32_t input_size = input[in_index]->buf_ptr[ch].actual_data_len;
      uint32_t copy_size  = memscpy(write_ptr, write_size, input[in_index]->buf_ptr[ch].data_ptr, input_size);

      in_port_ptr->int_bufs_ptr[ch].actual_data_len += copy_size;

      // If process is unable to consume input, drop this data only for EC secondary port case
      // Otherwise simply mark the other bytes as not consumed
      if (copy_size != input_size)
      {
         if (SYNC_EC_SECONDARY_IN_PORT_ID == in_port_ptr->cmn.self_port_id)
         {
            uint32_t drop_size = input[in_index]->buf_ptr[ch].actual_data_len - copy_size;

#ifdef CAPI_SYNC_DEBUG
            AR_MSG(DBG_ERROR_PRIO,
                   "capi sync process(): Overflowed buffer on secondary path. This is expected if only "
                   "secondary path is running. Scooting over data to make room for new data, and dropping oldest "
                   "data. Amount dropped = %ld",
                   drop_size);
#endif

            // First move data over to make room for remaining data.
            write_ptr          = in_port_ptr->int_bufs_ptr[ch].data_ptr;
            write_size         = in_port_ptr->int_bufs_ptr[ch].max_data_len;
            int8_t * read_ptr  = in_port_ptr->int_bufs_ptr[ch].data_ptr + drop_size;
            uint32_t read_size = in_port_ptr->int_bufs_ptr[ch].actual_data_len - drop_size;
            memsmove(write_ptr, write_size, read_ptr, read_size);
            in_port_ptr->int_bufs_ptr[ch].actual_data_len -= drop_size;

            // Now copy the remaining data.
            write_ptr  = in_port_ptr->int_bufs_ptr[ch].data_ptr + in_port_ptr->int_bufs_ptr[ch].actual_data_len;
            write_size = in_port_ptr->int_bufs_ptr[ch].max_data_len - in_port_ptr->int_bufs_ptr[ch].actual_data_len;
            read_ptr   = input[in_index]->buf_ptr[ch].data_ptr + copy_size;
            read_size  = drop_size;
            memsmove(write_ptr, write_size, read_ptr, read_size);

            // TODO(claguna): Drop metadata if needed.
         }
         else
         {
            if (me_ptr->num_opened_in_ports > 1)
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "capi sync process(): Error: Can't buffer all input on port : %ld! input amt = "
                      "%ld bytes, able to buffer %ld bytes. Remaining bytes marked as unconsumed.",
                      in_port_ptr->cmn.self_index,
                      input[in_index]->buf_ptr[ch].actual_data_len,
                      copy_size);
            }

            input[in_index]->buf_ptr[ch].actual_data_len = copy_size;
         }

      } // if copy_size != input_size

   } // for loop over num channels

   if (me_ptr->metadata_handler.metadata_propagate)
   {
      uint32_t                   ALGO_DELAY_ZERO = 0; // Sync has no algo delay.
      intf_extn_md_propagation_t input_md_info;
      memset(&input_md_info, 0, sizeof(input_md_info));
      input_md_info.df                          = in_port_ptr->media_fmt.header.format_header.data_format;
      input_md_info.len_per_ch_in_bytes         = input[in_index]->buf_ptr[0].actual_data_len; //bytes consumed from capi buffer
      input_md_info.initial_len_per_ch_in_bytes = in_bytes_before; //initial bytes in capi buffer
      input_md_info.buf_delay_per_ch_in_bytes   = 0; //no pre-padding
      input_md_info.bits_per_sample             = in_port_ptr->media_fmt.format.bits_per_sample;
      input_md_info.sample_rate                 = in_port_ptr->media_fmt.format.sampling_rate;

      intf_extn_md_propagation_t output_md_info;
      memscpy(&output_md_info, sizeof(output_md_info), &input_md_info, sizeof(input_md_info));
      output_md_info.initial_len_per_ch_in_bytes = bytes_in_int_buf_before_per_ch; //bytes in internal buffer before copy
      output_md_info.len_per_ch_in_bytes         = input[in_index]->buf_ptr[0].actual_data_len; //bytes copied in internal buffer

      capi_result |= me_ptr->metadata_handler.metadata_propagate(me_ptr->metadata_handler.context_ptr,
                                                                 input_v2_ptr,
                                                                 &(in_port_ptr->int_stream),
                                                                 NULL,
                                                                 ALGO_DELAY_ZERO,
                                                                 &input_md_info,
                                                                 &output_md_info);

#ifdef CAPI_SYNC_DEBUG
      AR_MSG(DBG_HIGH_PRIO,
             "capi sync md_prop(): in_index = %d input consumed = %d algo delay = %d buff data before = %d after "
             "prop eos in %d eos out %d",
             in_port_ptr->cmn.self_index,
             input[in_index]->buf_ptr[0].actual_data_len,
             ALGO_DELAY_ZERO,
             bytes_in_int_buf_before_per_ch,
             input_v2_ptr->flags.marker_eos,
             in_port_ptr->int_stream.flags.marker_eos);
#endif
   }

   if (input[in_index]->flags.marker_eos)
   {
      AR_MSG(DBG_HIGH_PRIO, "capi sync process(): EOS marker received for port idx %d", in_port_ptr->cmn.self_index);
      in_port_ptr->pending_eos = TRUE;
      in_port_ptr->is_eos_rcvd = TRUE;
   }

   // Copy timestamp to input port if it is valid and we haven't stored one yet.
   if ((!(in_port_ptr->is_timestamp_valid)) && input[in_index]->flags.is_timestamp_valid && input[in_index]->buf_ptr[0].actual_data_len)
   {
      uint32_t  NUM_CH_1       = 1;
      uint64_t *FRACT_TIME_PTR = NULL;
      int64_t   adjust_ts      = capi_cmn_bytes_to_us(bytes_in_int_buf_before_per_ch,
                                               in_port_ptr->media_fmt.format.sampling_rate,
                                               in_port_ptr->media_fmt.format.bits_per_sample,
                                               NUM_CH_1,
                                               FRACT_TIME_PTR);

      in_port_ptr->buffer_timestamp   = input[in_index]->timestamp - adjust_ts;
      in_port_ptr->is_timestamp_valid = TRUE;

#ifdef CAPI_SYNC_VERBOSE
      AR_MSG(DBG_MED_PRIO,
             "capi sync debug input index %d, storing new buffer timestamp %ld with adjustment -(%d)",
             in_index,
             in_port_ptr->buffer_timestamp,
             adjust_ts);
#endif
   }

   // if eof is set in the input and all the data was consumed, consume eof and move it to internal list
   if (input[in_index]->flags.end_of_frame && (in_bytes_before == input[in_index]->buf_ptr[0].actual_data_len))
   {
      input[in_index]->flags.end_of_frame        = FALSE;
      in_port_ptr->int_stream.flags.end_of_frame = TRUE;

#ifdef CAPI_SYNC_VERBOSE
      AR_MSG(DBG_MED_PRIO, "capi sync debug input index %d, storing eof in internal buffer", in_index);
#endif
   }

#ifdef CAPI_SYNC_DEBUG
   AR_MSG(DBG_MED_PRIO,
          "capi sync process(): input index %d, capiv2 input consumed %d internal buf fill length %d ",
          in_index,
          input[in_index]->buf_ptr[0].actual_data_len,
          in_port_ptr->int_bufs_ptr[0].actual_data_len);
#endif

   return capi_result;
}

/**
 * Send all buffered data through output. The primary port must have threshold amount
 * of data buffered when calling this function. Also pads secondary data with zeros up
 * to the threshold length such that both primary and secondary have threshold amount
 * of data. If starting, zeros are padded at the beginning of secondary data to
 * synchronize most recently received data, if stopping, zeros are padded at the end of
 * secondary data.
 *
 * TODO(claguna): How to store metadata when buffering input data? How to copy flags?
 */
capi_err_t capi_sync_send_buffered_data(capi_sync_t *        me_ptr,
                                        capi_stream_data_t * output[],
                                        capi_sync_state_t    synced_state,
                                        capi_sync_in_port_t *in_port_ptr,
                                        capi_stream_data_t * input[])
{
   capi_err_t            capi_result  = CAPI_EOK;
   capi_sync_out_port_t *out_port_ptr = capi_sync_get_out_port_from_index(me_ptr, in_port_ptr->cmn.conn_index);

   if (!out_port_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi sync send_buffered_data() error, encountered null out_port_ptr");
      return CAPI_EFAILED;
   }

   uint32_t out_index = out_port_ptr->cmn.self_index;
   uint32_t in_index  = in_port_ptr->cmn.self_index;

   capi_stream_data_v2_t *output_v2_ptr = (capi_stream_data_v2_t *)output[out_index];

   if (CAPI_STREAM_V2 != output[out_index]->flags.stream_data_version)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi sync process(): stream_version is not v2.");
      return CAPI_EFAILED;
   }

   if (!in_port_ptr->int_bufs_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi sync process(): input buffer not allocated for %d", in_port_ptr->cmn.self_index);
      return CAPI_EFAILED;
   }

   if (!output[out_index]->buf_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "capi sync process(): output not present for port: index = %ld, ptr = 0x%lx",
             out_index,
             output[out_index]->buf_ptr);
      return CAPI_EFAILED;
   }

   if ((output[out_index]->buf_ptr[0].max_data_len < in_port_ptr->threshold_bytes_per_ch) &&
       (!in_port_ptr->pending_eof))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "capi sync process(): can't send buffered data, less than threshold amt was provided on primary "
             "output: %ld of %ld bytes per ch during EOF.",
             output[out_index]->buf_ptr[0].max_data_len,
             in_port_ptr->threshold_bytes_per_ch);
      return CAPI_EFAILED;
   }

   uint32_t num_channels                 = in_port_ptr->media_fmt.format.num_channels;
   uint32_t bytes_in_int_buf_before_copy = in_port_ptr->int_bufs_ptr[0].actual_data_len;
   uint32_t read_size = MIN(in_port_ptr->int_bufs_ptr[0].actual_data_len, in_port_ptr->threshold_bytes_per_ch);

   // Setup timestamps to be copied to the output buffers
   // If we are forcefully flushing the data due to eof before sync has been achieved, set TS to invalid.
   // Else, this will lead to TS discontinuity during sync
   int64_t in_ts          = in_port_ptr->buffer_timestamp;
   bool_t  in_ts_is_valid = (STATE_STARTING == synced_state && in_port_ptr->is_threshold_disabled &&
                            in_port_ptr->int_stream.flags.end_of_frame)
                              ? FALSE
                              : in_port_ptr->is_timestamp_valid;

   uint32_t num_zeroes_per_ch        = 0;
   int64_t  ts_adjust                = 0;
   uint32_t wr_offset_per_ch         = 0;
   bool_t   pushed_zeros_before_data = FALSE;

   // If this input has not yet synchronized, there might be a need to prepend zeroes
   // In this process call, if EOF is detected and inputs are not yet sync'ed, skip prepending zeroes
   if (STATE_STARTING == synced_state && in_port_ptr->is_threshold_disabled &&
       !in_port_ptr->int_stream.flags.end_of_frame)
   {
      uint64_t *FRACT_TIME_PTR_NULL = NULL;
      uint32_t  NUM_CH_1            = 1;

      // Avoid negative numbers in case of in_port_ptr->pending_eos, and avoid overflowing output buffer.
      num_zeroes_per_ch =
         (in_port_ptr->threshold_bytes_per_ch > read_size) ? (in_port_ptr->threshold_bytes_per_ch - read_size) : 0;
      num_zeroes_per_ch = MIN(num_zeroes_per_ch, output[out_index]->buf_ptr[0].max_data_len);
      ts_adjust         = capi_cmn_bytes_to_us(num_zeroes_per_ch,
                                       in_port_ptr->media_fmt.format.sampling_rate,
                                       in_port_ptr->media_fmt.format.bits_per_sample,
                                       NUM_CH_1,
                                       FRACT_TIME_PTR_NULL);
   }

   if (num_zeroes_per_ch)
   {
      pushed_zeros_before_data = TRUE;

      AR_MSG(DBG_MED_PRIO,
             "capi sync process(): prepending %ld bytes per channel of initial zeros on output port idx %ld adj ts "
             "%ld",
             num_zeroes_per_ch,
             out_index,
             ts_adjust);

      // Fill zeroes in the output
      for (uint32_t ch = 0; ch < num_channels; ch++)
      {
         int8_t *write_ptr = output[out_index]->buf_ptr[ch].data_ptr;
         memset(write_ptr, 0, num_zeroes_per_ch);
         output[out_index]->buf_ptr[ch].actual_data_len = num_zeroes_per_ch;
      }

      wr_offset_per_ch += num_zeroes_per_ch;

      // Negative timestamps can occur if zeros are prepended for sync'ing at timestamp 0
      if (in_ts_is_valid)
      {
         in_ts -= ts_adjust;
      }
   }

   uint32_t bytes_consumed_from_int_buf = 0;
   uint32_t bytes_copied_from_int_buf = 0;

   // Copy data from internal buffer to output.
  // bool_t shift_size_1_ch = 0;
   for (uint32_t ch = 0; ch < num_channels; ch++)
   {
      int8_t *read_ptr = in_port_ptr->int_bufs_ptr[ch].data_ptr;
      int8_t *wr_ptr   = output[out_index]->buf_ptr[ch].data_ptr + wr_offset_per_ch;

      uint32_t wr_size   = output[out_index]->buf_ptr[ch].max_data_len - wr_offset_per_ch;
      uint32_t copy_size = memscpy(wr_ptr, wr_size, read_ptr, read_size);

      bytes_copied_from_int_buf = copy_size;

      // If EOS is pending, then mark all data is consumed
      // Typically the sync module would always buffer threshold worth data except when paired with DM modules
      if (in_port_ptr->pending_eos || in_port_ptr->int_stream.flags.marker_eos)
      {
         bytes_consumed_from_int_buf = in_port_ptr->int_bufs_ptr[ch].actual_data_len;
      }
      else
      {
         bytes_consumed_from_int_buf = copy_size;
      }

      // If there is left over in the input, move to the beginning of the internal buffer
      if (bytes_consumed_from_int_buf < in_port_ptr->int_bufs_ptr[ch].actual_data_len && copy_size)
      {
         int8_t * dst_ptr  = in_port_ptr->int_bufs_ptr[ch].data_ptr;
         uint32_t dst_size = in_port_ptr->int_bufs_ptr[ch].max_data_len;
         int8_t * src_ptr  = in_port_ptr->int_bufs_ptr[ch].data_ptr + copy_size;
         uint32_t src_size = in_port_ptr->int_bufs_ptr[ch].actual_data_len - copy_size;

         uint32_t bytes_copied = memscpy(dst_ptr, dst_size, src_ptr, src_size);

         in_port_ptr->int_bufs_ptr[ch].actual_data_len = bytes_copied;

         //shift_size_1_ch = copy_size;

         // Re-sync to the next buffer timestamp
         in_port_ptr->is_timestamp_valid = FALSE;
         in_port_ptr->buffer_timestamp   = 0;
      }
      else if (copy_size)
      {
         in_port_ptr->int_bufs_ptr[ch].actual_data_len = 0;

         // If there is no data remaining, clear the timestamp.
         in_port_ptr->is_timestamp_valid = FALSE;
         in_port_ptr->buffer_timestamp   = 0;
      }

      output[out_index]->buf_ptr[ch].actual_data_len += copy_size;
   }

   // If input was fully consumed, check if trailing zeroes need to be filled for
   // pending data gap flow
   if (0 == in_port_ptr->int_bufs_ptr[0].actual_data_len && bytes_consumed_from_int_buf)
   {
      // Avoid negative numbers in case of in_port_ptr->pending_eos.
      uint32_t num_trailing_zeroes = (in_port_ptr->threshold_bytes_per_ch > output[out_index]->buf_ptr[0].actual_data_len) ?
         (in_port_ptr->threshold_bytes_per_ch - output[out_index]->buf_ptr[0].actual_data_len) : 0;

      if (num_trailing_zeroes)
      {
         if (in_port_ptr->pending_eos)
         {
            for (uint32_t ch = 0; ch < num_channels; ch++)
            {
               uint32_t out_offset = output[out_index]->buf_ptr[ch].actual_data_len;
               int8_t * out_ptr    = output[out_index]->buf_ptr[ch].data_ptr + out_offset;
               uint32_t zero_size  = output[out_index]->buf_ptr[ch].max_data_len - out_offset;

               zero_size = MIN(zero_size, num_trailing_zeroes);

               memset(out_ptr, 0, zero_size);

               AR_MSG(DBG_HIGH_PRIO,
                      "capi sync process(): appending %d trailing zeroes on output port idx %d pending data flow "
                      "gap %d pending eos %d ",
                      num_trailing_zeroes,
                      out_index,
                      in_port_ptr->pending_data_flow_gap,
                      in_port_ptr->pending_eos);

               output[out_index]->buf_ptr[ch].actual_data_len += zero_size;
            }
         }
         else
         {
#ifdef CAPI_SYNC_VERBOSE
            AR_MSG(DBG_HIGH_PRIO,
                   "capi sync process(): NOT appending %d trailing zeroes on output port idx %ld pending data flow "
                   "gap %d pending eos %d ",
                   num_trailing_zeroes,
                   out_index,
                   in_port_ptr->pending_data_flow_gap,
                   in_port_ptr->pending_eos);
#endif
         }
      }
   }

   // Amount of data generated for helper function shouldn't include zeros.
   bool_t sent_any_eos = FALSE;

   if (me_ptr->metadata_handler.metadata_propagate)
   {
      uint32_t                   ALGO_DELAY_ZERO = 0;
      intf_extn_md_propagation_t input_md_info;
      memset(&input_md_info, 0, sizeof(input_md_info));
      input_md_info.df                          = in_port_ptr->media_fmt.header.format_header.data_format;
      input_md_info.len_per_ch_in_bytes         = bytes_consumed_from_int_buf;
      input_md_info.initial_len_per_ch_in_bytes = bytes_in_int_buf_before_copy;
      input_md_info.bits_per_sample             = in_port_ptr->media_fmt.format.bits_per_sample;
      input_md_info.sample_rate                 = in_port_ptr->media_fmt.format.sampling_rate;

      intf_extn_md_propagation_t output_md_info;
      memscpy(&output_md_info, sizeof(output_md_info), &input_md_info, sizeof(input_md_info));
      // If we prepended zeros, this counts as output buffer initial size.
      output_md_info.initial_len_per_ch_in_bytes = pushed_zeros_before_data ? num_zeroes_per_ch : 0;
      output_md_info.len_per_ch_in_bytes         = bytes_copied_from_int_buf;

      capi_result |= me_ptr->metadata_handler.metadata_propagate(me_ptr->metadata_handler.context_ptr,
                                                                 &(in_port_ptr->int_stream),
                                                                 output_v2_ptr,
                                                                 NULL,
                                                                 ALGO_DELAY_ZERO,
                                                                 &input_md_info,
                                                                 &output_md_info);

      sent_any_eos = output_v2_ptr->flags.marker_eos;

      capi_sync_adjust_md_after_prop(me_ptr,
                                     &(in_port_ptr->int_stream),
                                     capi_cmn_bytes_to_samples_per_ch(bytes_consumed_from_int_buf,
                                                                      in_port_ptr->media_fmt.format.bits_per_sample,
                                                                      1));

      // Move EOS/DFG after any trailing zeros.
      uint32_t out_bytes_all_ch = output[out_index]->buf_ptr[0].actual_data_len * num_channels;
      uint32_t end_offset       = capi_cmn_bytes_to_samples_per_ch(out_bytes_all_ch,
                                                             in_port_ptr->media_fmt.format.bits_per_sample,
                                                             num_channels);
      capi_sync_move_eos_dfg_to_end(me_ptr, end_offset, output_v2_ptr);

#ifdef CAPI_SYNC_DEBUG
      AR_MSG(DBG_HIGH_PRIO,
             "capi sync md_prop(): out_index = %d out gen = %d data from int buf = %d after prop eos in %d eos out "
             "%d",
             out_port_ptr->cmn.self_index,
             output[out_index]->buf_ptr[0].actual_data_len,
             bytes_consumed_from_int_buf,
             in_port_ptr->int_stream.flags.marker_eos,
             output_v2_ptr->flags.marker_eos);
#endif
   }

   // For topologies with SYNC and DM modules together, output might have more space as compared to the
   // threshold configured. Check for threshold instead.
   if (!in_port_ptr->is_output_sent_once &&
       (output[out_index]->buf_ptr[0].actual_data_len == in_port_ptr->threshold_bytes_per_ch))
   {
      AR_MSG(DBG_HIGH_PRIO, "capi sync process(): port index %d first output", in_port_ptr->cmn.self_index);
      in_port_ptr->is_output_sent_once = TRUE;
   }

#ifdef CAPI_SYNC_VERBOSE
   AR_MSG(DBG_HIGH_PRIO,
          "capi sync debug before output index %d capi ts %ld capi is_ts_valid %d",
          out_index,
          output[out_index]->timestamp,
          output[out_index]->flags.is_timestamp_valid);
#endif

   // Assign timestamps always. This should be done before dropping data, which
   // resets the timestamps/is_valid fields.
   output[out_index]->timestamp                = in_ts;
   output[out_index]->flags.is_timestamp_valid = in_ts_is_valid;

#ifdef CAPI_SYNC_VERBOSE
   AR_MSG(DBG_HIGH_PRIO,
          "capi sync debug after output index %d capi ts %ld capi is_ts_valid %d",
          out_index,
          output[out_index]->timestamp,
          output[out_index]->flags.is_timestamp_valid);
#endif

   if (sent_any_eos)
   {
      generic_sync_mark_port_as_stopped(me_ptr, in_port_ptr->cmn.self_index);

      AR_MSG(DBG_HIGH_PRIO, "capi sync process(): Propagting EOS to output idx = %d ", out_index);

      output[out_index]->flags.marker_eos = TRUE;
      in_port_ptr->pending_eos            = FALSE;
      in_port_ptr->is_eos_rcvd            = TRUE;

      // EOS has propagated from input to output, so set marker_eos on output and clear on input.
      input[in_index]->flags.marker_eos   = FALSE;
      output[out_index]->flags.marker_eos = TRUE;

      in_port_ptr->int_stream.flags.end_of_frame = FALSE;
      input[in_index]->flags.end_of_frame        = FALSE;
      output[out_index]->flags.end_of_frame      = TRUE;

#ifdef CAPI_SYNC_VERBOSE
      AR_MSG(DBG_MED_PRIO, "capi sync debug input index %d, propagating eof to output %d", in_index, out_index);
#endif
   }
   else
   {
      in_port_ptr->pending_eos = FALSE;
      in_port_ptr->is_eos_rcvd = FALSE;
   }

   // If there is EOF in the internal stream and the all the data in the internal buffer has been consumed, propagate
   // EOF
   if (in_port_ptr->int_stream.flags.end_of_frame && (bytes_in_int_buf_before_copy == bytes_consumed_from_int_buf))
   {
#ifdef CAPI_SYNC_VERBOSE
      AR_MSG(DBG_MED_PRIO, "capi sync debug input index %d, propagating eof to output %d", in_index, out_index);
#endif
      in_port_ptr->int_stream.flags.end_of_frame = FALSE;
      input[in_index]->flags.end_of_frame        = FALSE;
      output[out_index]->flags.end_of_frame      = TRUE;
   }

   // If the ports have not yet sync'ed, check if output is sent once before disabling this port's threshold requirement
   // This is to properly handle the case where EOF is received before ports have sync'ed. If we flush the data
   // from the internal buffers to sync again, then we should not disable threshold.
   if (STATE_STARTING == synced_state && in_port_ptr->is_threshold_disabled && in_port_ptr->is_output_sent_once)
   {
      in_port_ptr->is_threshold_disabled = FALSE;
      AR_MSG(DBG_HIGH_PRIO,
             "capi sync process(): port index %d moving to thresh enable state",
             in_port_ptr->cmn.self_index);
   }

   // If at any point, the output length is not threshold worth data, set eof on the output
   if (output[out_index]->buf_ptr[0].actual_data_len &&
       (output[out_index]->buf_ptr[0].actual_data_len < in_port_ptr->threshold_bytes_per_ch))
   {
      //#ifdef CAPI_SYNC_VERBOSE
      AR_MSG(DBG_MED_PRIO,
             "capi sync debug input index %d, setting eof on output %d due to partial frame",
             in_index,
             out_index);
      //#endif
      output[out_index]->flags.end_of_frame = TRUE;
   }

   /**
    * If timestamp disc is detected on output then drop the data.
    * If data is sent with discontinuous timestamp then it causes hang at SAL input. SAL will first receive EOF due to
    * ts disc while it receives valid data on other ports. After SAL process, data from this output port will be copied
    * to the SAL input port. Now SYNC and SAL will have trigger on one connected ports, this will cause hang.
    */

   uint32_t output_buf_duration_in_us = capi_cmn_bytes_to_us(output[out_index]->buf_ptr[0].actual_data_len,
                                                             in_port_ptr->media_fmt.format.sampling_rate,
                                                             in_port_ptr->media_fmt.format.bits_per_sample,
                                                             1,
                                                             NULL);
   if (output[out_index]->flags.end_of_frame)
   {
      // if output port has EOF then invalidate the timestamp for next time
      out_port_ptr->is_ts_valid = FALSE;
   }
   else if (!out_port_ptr->is_ts_valid)
   {
      if (in_ts_is_valid)
      {
         out_port_ptr->is_ts_valid        = TRUE;
         out_port_ptr->expected_timestamp = in_ts + output_buf_duration_in_us;
      }
   }
   else
   {
      bool_t disc = FALSE;
      if (!in_ts_is_valid)
      {
         disc = TRUE;
      }
      else
      {
         int64_t ts_diff = in_ts - out_port_ptr->expected_timestamp;
         ts_diff         = (ts_diff < 0) ? (0 - ts_diff) : ts_diff;
         if (ts_diff >= 1000)
         {
            disc = TRUE;
         }
      }

      if (disc)
      {
         AR_MSG(DBG_HIGH_PRIO,
                "capi sync process(): dropping buffered data on index %d, output data len %d due to ts disc. ",
                out_index,
                output[out_index]->buf_ptr[0].actual_data_len);
         for (int i = 0; i < output[out_index]->bufs_num; i++)
         {
            output[out_index]->buf_ptr[i].actual_data_len = 0;
         }
         output[out_index]->flags.end_of_frame = TRUE;
         out_port_ptr->is_ts_valid             = FALSE;
      }
      else
      {
         out_port_ptr->is_ts_valid        = TRUE;
         out_port_ptr->expected_timestamp = in_ts + output_buf_duration_in_us;
      }
   }

#ifdef CAPI_SYNC_DEBUG
   AR_MSG(DBG_HIGH_PRIO,
          "capi sync process(): send buffered data on index %d, output data len %d internal buf length %d ",
          out_index,
          output[out_index]->buf_ptr[0].actual_data_len,
          in_port_ptr->int_bufs_ptr[0].actual_data_len);
#endif

   return capi_result;
}

/**
 * Clear all internally buffered data on the given input port
 */
capi_err_t capi_sync_clear_buffered_data(capi_sync_t *me_ptr, capi_sync_in_port_t *in_port_ptr)
{
   capi_err_t capi_result = CAPI_EOK;

   if (!in_port_ptr)
   {
      AR_MSG(DBG_LOW_PRIO, "capi sync: Warning NULL In port Pointer ");
      return capi_result;
   }

   capi_buf_t *bufs_ptr = in_port_ptr->int_bufs_ptr;

   // Nothing to do if buffer wasn't allocated yet.
   if (bufs_ptr)
   {
      uint32_t num_channels = in_port_ptr->media_fmt.format.num_channels;

      // Skip dropping data if buffers are empty.
      if ((0 < num_channels) && !bufs_ptr[0].actual_data_len)
      {
         AR_MSG(DBG_LOW_PRIO, "capi sync: Found no data in internal buffers while dropping data.");
      }
      else
      {
         if (bufs_ptr[0].actual_data_len)
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "capi sync: Warning! Dropping %d bytes per channel input index %d",
                   bufs_ptr[0].actual_data_len,
                   in_port_ptr->cmn.self_index);
         }

         // Drop data by setting actual data length back to zero.
         for (uint32_t ch = 0; ch < num_channels; ch++)
         {
            bufs_ptr[ch].actual_data_len = 0;
         }
      }
   }
   else
   {
#ifdef CAPI_SYNC_VERBOSE
      AR_MSG(DBG_LOW_PRIO,
             "capi sync: no data to drop for port [%d] with buffer not allocated yet",
             in_port_ptr->cmn.self_index);
#endif
   }

   // Reset timestamp.
   in_port_ptr->is_timestamp_valid = FALSE;
   in_port_ptr->buffer_timestamp   = 0;

   module_cmn_md_list_t *next_ptr = NULL;

   // Clear all buffered metadata in the internal stream data.
   for (module_cmn_md_list_t *node_ptr = in_port_ptr->int_stream.metadata_list_ptr; node_ptr;)
   {
      bool_t IS_DROPPED_TRUE = TRUE;
      next_ptr               = node_ptr->next_ptr;
      if (me_ptr->metadata_handler.metadata_destroy)
      {
         me_ptr->metadata_handler.metadata_destroy(me_ptr->metadata_handler.context_ptr,
                                                   node_ptr,
                                                   IS_DROPPED_TRUE,
                                                   &in_port_ptr->int_stream.metadata_list_ptr);
      }
      else
      {
         AR_MSG(DBG_ERROR_PRIO, "capi sync: Error: metadata handler not provided, can't drop metadata.");
         return CAPI_EFAILED;
      }
      node_ptr = next_ptr;
   }

   return capi_result;
}

/**
 * Utility to move the generic sync module ports to stop as part of data process.
 * This is invoked when EOS is received at the input port.
 */
// RR: Discuss what can happen if we call the same function as in_port_stop for data gap handling
static capi_err_t generic_sync_mark_port_as_stopped(capi_sync_t *me_ptr, uint32_t in_index)
{
   capi_err_t capi_result = CAPI_EOK;

   if (MODE_EC_PRIO_INPUT == me_ptr->mode)
   {
      return capi_result;
   }

   capi_sync_in_port_t *in_port_ptr = capi_sync_get_in_port_from_index(me_ptr, in_index);
   if (!in_port_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi sync mark_port_as_stopped() error, encountered null in_port_ptr");
      return CAPI_EFAILED;
   }

   if (CAPI_PORT_STATE_STOPPED == in_port_ptr->cmn.state)
   {
      AR_MSG(DBG_LOW_PRIO, "capi sync port %d already in stop state. ignoring", in_port_ptr->cmn.self_index);
      return CAPI_EOK;
   }

   in_port_ptr->will_start_rcvd     = FALSE;
   in_port_ptr->cmn.state           = CAPI_PORT_STATE_STOPPED;
   in_port_ptr->is_output_sent_once = FALSE;

   capi_sync_out_port_t *conn_out_port_ptr = capi_sync_get_out_port_from_index(me_ptr, in_port_ptr->cmn.conn_index);
   if (conn_out_port_ptr)
   {
      conn_out_port_ptr->is_ts_valid = FALSE;
   }

   AR_MSG(DBG_LOW_PRIO, "capi sync mark_port_as_stopped() %d", in_port_ptr->cmn.self_index);

   return capi_result;
}

/**
 * Utility function to calculate the connected mask given incoming mask & type of port
 */
uint32_t generic_sync_calc_conn_mask(capi_sync_t *me_ptr, uint32_t incoming_mask, bool_t is_in_to_out)
{
   uint32_t ret_mask = 0;

   if (is_in_to_out)
   {
      while (incoming_mask)
      {
         uint32_t index = s32_get_lsb_s32(incoming_mask);
         capi_sync_clear_bit(&incoming_mask, index);

         capi_sync_in_port_t *in_port_ptr = capi_sync_get_in_port_from_index(me_ptr, index);
         if (in_port_ptr && (SYNC_PORT_INDEX_INVALID != in_port_ptr->cmn.conn_index))
         {
            capi_sync_set_bit(&ret_mask, in_port_ptr->cmn.conn_index);
         }
      }
   }
   else
   {
      while (incoming_mask)
      {
         uint32_t index = s32_get_lsb_s32(incoming_mask);
         capi_sync_clear_bit(&incoming_mask, index);

         capi_sync_out_port_t *out_port_ptr = capi_sync_get_out_port_from_index(me_ptr, index);
         if (out_port_ptr && (SYNC_PORT_INDEX_INVALID != out_port_ptr->cmn.conn_index))
         {
            capi_sync_set_bit(&ret_mask, out_port_ptr->cmn.conn_index);
         }
      }
   }

   return ret_mask;
}

/**
 * Utility function to validate the incoming capiv2 buffers for the sync module
 */
capi_err_t capi_sync_validate_io_bufs(capi_sync_t *       me_ptr,
                                      capi_stream_data_t *input[],
                                      capi_stream_data_t *output[],
                                      bool_t *            does_any_port_have_data)
{
   *does_any_port_have_data = FALSE;

   capi_sync_proc_info_t *proc_ctx_ptr = &(me_ptr->proc_ctx);

#ifdef CAPI_SYNC_DEBUG
   AR_MSG(DBG_MED_PRIO, "capi sync process(): Process IO bufs.");
#endif

   for (uint32_t i = 0; i < me_ptr->num_in_ports; i++)
   {
      capi_sync_in_port_t *in_port_ptr = &(me_ptr->in_port_info_ptr[i]);
      uint32_t             in_index    = in_port_ptr->cmn.self_index;

      // if this input is not active/conn output is not active for any reason then ignore
      if ((SYNC_PORT_INDEX_INVALID == in_port_ptr->cmn.self_index) ||
          (SYNC_PORT_INDEX_INVALID == in_port_ptr->cmn.conn_index))
      {
         continue;
      }

      // Identify inputs that have dfg at the input.
      if (capi_sync_sdata_has_dfg(me_ptr, (capi_stream_data_v2_t *)input[in_index]))
      {
         AR_MSG(DBG_HIGH_PRIO,
                "capi sync process(): DFG marker found on input port idx %d",
                in_port_ptr->cmn.self_index);
         in_port_ptr->pending_data_flow_gap = TRUE;

         if (!input[in_index]->flags.end_of_frame)
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "capi sync process(): EOF marker NOT received with DFG for port idx %d. Updating by self",
                   in_port_ptr->cmn.self_index);
            in_port_ptr->pending_eof = TRUE;
         }
      }

      if (input[in_index]->flags.marker_eos)
      {
         AR_MSG(DBG_HIGH_PRIO, "capi sync process(): EOS marker received for port idx %d", in_port_ptr->cmn.self_index);
         in_port_ptr->pending_eos = TRUE;

         if (!input[in_index]->flags.end_of_frame)
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "capi sync process(): EOF marker NOT received with EOS for port idx %d. Updating by self",
                   in_port_ptr->cmn.self_index);
            in_port_ptr->pending_eof = TRUE;
         }
      }

      if (input[in_index]->flags.end_of_frame)
      {
         AR_MSG(DBG_HIGH_PRIO, "capi sync process(): EOF marker received for port idx %d", in_port_ptr->cmn.self_index);
         in_port_ptr->pending_eof = TRUE;
      }

      bool_t is_data_on_port = capi_sync_input_has_data(input, in_index);
      *does_any_port_have_data |= is_data_on_port;

      // If the connected output has space in the capiv2 buffer, then add it to the proc mask
      if (capi_sync_output_has_space(output, in_port_ptr->cmn.conn_index))
      {
         capi_sync_set_bit(&proc_ctx_ptr->capi_data_avail_mask.output, in_port_ptr->cmn.conn_index);
      }
      else
      {
         capi_sync_clear_bit(&proc_ctx_ptr->capi_data_avail_mask.output, in_port_ptr->cmn.conn_index);
      }

      // For all stopped input ports, if data was received move those input ports to start
      if (is_data_on_port)
      {
         if (CAPI_PORT_STATE_STOPPED == in_port_ptr->cmn.state)
         {
            if (!in_port_ptr->int_bufs_ptr)
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "capi sync process(): Error: Port %d state moved to started yet internal buffer is not"
                      "allocated",
                      in_index);
               return CAPI_EFAILED;
            }

            in_port_ptr->cmn.state = CAPI_PORT_STATE_STARTED;

#ifdef CAPI_SYNC_DEBUG
            AR_MSG(DBG_MED_PRIO,
                   "capi sync process(): input port idx %d received first data after stop. "
                   "Moving to start state.",
                   in_index);
#endif
            me_ptr->synced_state             = STATE_STARTING;
            in_port_ptr->is_output_sent_once = FALSE;
         }

         in_port_ptr->is_eos_rcvd = FALSE;
      }
      else
      {
         capi_sync_clear_bit(&proc_ctx_ptr->capi_data_avail_mask.input, in_index);

         // This can happen only if eos was received previously and moved to internal list and force process
         // is called with no data.
         // RR: Is this sufficient or need to check int_md_list_ptr for MD? What about DFG?
         if (in_port_ptr->pending_eof && in_port_ptr->int_bufs_ptr && (in_port_ptr->int_bufs_ptr->actual_data_len > 0))
         {
            AR_MSG(DBG_MED_PRIO,
                   "capi sync process(): input port idx %d pending eof set with no input data and has %d bytes in "
                   "internal buffer. ",
                   in_index,
                   in_port_ptr->int_bufs_ptr->actual_data_len);
            *does_any_port_have_data |= TRUE;
         }
      }

   } // for input ports loop

   return CAPI_EOK;
}

/*------------------------------------------------------------------------
  Function name: sync_module_process
  Processes an input buffer and generates an output buffer.
 * -----------------------------------------------------------------------*/
capi_err_t sync_module_process(capi_sync_t *me_ptr, capi_stream_data_t *input[], capi_stream_data_t *output[])
{
   capi_err_t capi_result = CAPI_EOK;

   if (MODE_EC_PRIO_INPUT == me_ptr->mode)
   {
      capi_result |= ec_sync_mode_process(me_ptr, input, output);
   }
   else
   {
      capi_result |= generic_sync_mode_process(me_ptr, input, output);
   }

   return capi_result;
}

/**
 * Utility function to identify the synced input ports for the generic mode
 */
uint32_t generic_sync_calc_synced_input_mask(capi_sync_t *me_ptr)
{
   uint32_t mask  = 0;
   uint32_t count = 0;

   // Expected to be called only by generic_sync for now
   if (MODE_ALL_EQUAL_PRIO_INPUT != me_ptr->mode)
   {
      return mask;
   }

   for (uint32_t i = 0; i < me_ptr->num_in_ports; i++)
   {
      capi_sync_in_port_t *in_port_ptr = &(me_ptr->in_port_info_ptr[i]);

      // If the part is started & has sent output at least once without pending eos
      // no need to check for index here since the port would never be sync'ed without a valid output connection
      if ((CAPI_PORT_STATE_STARTED == in_port_ptr->cmn.state) && (FALSE == in_port_ptr->is_threshold_disabled) &&
          (in_port_ptr->is_output_sent_once) && (FALSE == in_port_ptr->is_eos_rcvd))
      {
         capi_sync_set_bit(&mask, in_port_ptr->cmn.self_index);
         count++;
      }
#ifdef CAPI_SYNC_VERBOSE
      else
      {
         AR_MSG(DBG_HIGH_PRIO,
                "capi sync debug input index %d state %d is_thresh_disabled %d is_output_sent_once %d is_eos_rcvd %d",
                in_port_ptr->cmn.self_index,
                in_port_ptr->cmn.state,
                in_port_ptr->is_threshold_disabled,
                in_port_ptr->is_output_sent_once,
                in_port_ptr->is_eos_rcvd);
      }
#endif
   }

   me_ptr->proc_ctx.synced_ports_mask = mask;
   me_ptr->proc_ctx.num_synced        = count;

   return mask;
}
/**
 * Utility function to identify the input ports waiting to sync for the generic mode
 */
uint32_t generic_sync_calc_waiting_input_mask(capi_sync_t *me_ptr)
{
   uint32_t mask  = 0;
   uint32_t count = 0;

   // Expected to be called only by generic_sync for now
   if (MODE_ALL_EQUAL_PRIO_INPUT != me_ptr->mode)
   {
      return mask;
   }

   for (uint32_t i = 0; i < me_ptr->num_in_ports; i++)
   {
      capi_sync_in_port_t *in_port_ptr = &(me_ptr->in_port_info_ptr[i]);

      // If the port is started & hasn't yet delivered output once
      if ((CAPI_PORT_STATE_STARTED == in_port_ptr->cmn.state) && (FALSE == in_port_ptr->is_eos_rcvd) &&
          ((TRUE == in_port_ptr->is_threshold_disabled) || (FALSE == in_port_ptr->is_output_sent_once)))
      {
         if (SYNC_PORT_INDEX_INVALID != in_port_ptr->cmn.conn_index)
         {
            capi_sync_set_bit(&mask, in_port_ptr->cmn.self_index);
            count++;
         }
         else
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "capi sync input index %d is ready to sync but has invalid output index %d ",
                   in_port_ptr->cmn.self_index,
                   in_port_ptr->cmn.conn_index);
         }
      }
#ifdef CAPI_SYNC_VERBOSE
      else
      {
         AR_MSG(DBG_HIGH_PRIO,
                "capi sync debug input index %d state %d is_thresh_disabled %d is_output_sent_once %d is_eos_rcvd %d",
                in_port_ptr->cmn.self_index,
                in_port_ptr->cmn.state,
                in_port_ptr->is_threshold_disabled,
                in_port_ptr->is_output_sent_once,
                in_port_ptr->is_eos_rcvd);
      }
#endif
   }

   me_ptr->proc_ctx.ready_to_sync_mask = mask;
   me_ptr->proc_ctx.num_ready_to_sync  = count;

   return mask;
}

/**
 * Utility function to identify the input ports waiting to sync for the generic mode
 */
uint32_t generic_sync_calc_stopped_input_mask(capi_sync_t *me_ptr, capi_stream_data_t *input[])
{
   uint32_t mask  = 0;
   uint32_t count = 0;

   // Expected to be called only by generic_sync for now
   if (MODE_ALL_EQUAL_PRIO_INPUT != me_ptr->mode)
   {
      return mask;
   }

   for (uint32_t i = 0; i < me_ptr->num_in_ports; i++)
   {
      capi_sync_in_port_t *in_port_ptr = &(me_ptr->in_port_info_ptr[i]);

      // If EOS/DFG follow each other, the port moves to stopped state as part of handling the first one
      // As a result, it is possible that the next metadata gets stuck inside the internal list.
      // Add this to the stopped port list so that the metadata can be flushed out
      if (in_port_ptr && (CAPI_PORT_STATE_STOPPED == in_port_ptr->cmn.state) &&
          (SYNC_PORT_INDEX_INVALID != in_port_ptr->cmn.self_index) &&
          (SYNC_PORT_INDEX_INVALID != in_port_ptr->cmn.conn_index))
      {
         uint32_t in_index = in_port_ptr->cmn.self_index;

         if (input && input[in_index] && (CAPI_STREAM_V2 == input[in_index]->flags.stream_data_version) &&
             (capi_sync_sdata_has_any_md((capi_stream_data_v2_t *)input[in_index])))
         {
            capi_sync_set_bit(&mask, in_port_ptr->cmn.self_index);
            count++;
         }
      }
   }

   me_ptr->proc_ctx.stopped_ports_mask = mask;
   me_ptr->proc_ctx.num_stopped        = count;

   return mask;
}

/**
 * Utility function to identify the if sync module should disable threshold
 */
bool_t capi_sync_should_disable_thresh(capi_sync_t *me_ptr)
{
   bool_t should_disable_threshold = FALSE;

   for (uint32_t i = 0; i < me_ptr->num_in_ports; i++)
   {
      capi_sync_in_port_t *in_port_ptr = capi_sync_get_in_port_from_index(me_ptr, i);
      if (!in_port_ptr)
      {
         AR_MSG(DBG_ERROR_PRIO, "capi sync should_disable_thresh() error, encountered null in_port_ptr");
         return CAPI_EFAILED;
      }

      /* Even if one input has disabled the threshold, break and maintain the state */
      if ((CAPI_PORT_STATE_STARTED == in_port_ptr->cmn.state) && (TRUE == in_port_ptr->is_threshold_disabled))
      {
         //#ifdef CAPI_SYNC_DEBUG
         AR_MSG(DBG_MED_PRIO,
                "capi sync index %d is started & has requested for thresh disable ",
                in_port_ptr->cmn.self_index);
         //#endif
         should_disable_threshold = TRUE;
         break;
      }
   }

   return should_disable_threshold;
}

/**
 * Utility function to handle metadata for the sync module. This should only be called when
 * there is no input data to push metadata to the output.
 */
capi_err_t capi_sync_pass_through_metadata_single_port(capi_sync_t *        me_ptr,
                                                       capi_sync_in_port_t *in_port_ptr,
                                                       capi_stream_data_t * output[],
                                                       capi_stream_data_t * input[])
{
   capi_err_t             result                = CAPI_EOK;
   uint32_t               in_index              = in_port_ptr->cmn.self_index;
   uint32_t               out_index             = in_port_ptr->cmn.conn_index;
   capi_stream_data_v2_t *in_stream_ptr         = (capi_stream_data_v2_t *)input[in_index];
   capi_stream_data_v2_t *out_stream_ptr        = (capi_stream_data_v2_t *)output[out_index];
   bool_t                 sent_any_eos          = FALSE;
   bool_t                 prev_eos_out          = FALSE;
   uint32_t               ALGO_DELAY_ZERO       = 0;
   module_cmn_md_list_t * DUMMY_INT_MD_LIST_PTR = NULL;

   if (!in_stream_ptr || !out_stream_ptr)
   {
      return result;
   }

   if (input[in_index]->buf_ptr && input[in_index]->buf_ptr[0].actual_data_len)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi sync : Only supposed to pass through metadata in no-input cases.");
      return CAPI_EFAILED;
   }

   // Process meta data only if stream version v2
   if (!((CAPI_STREAM_V2 == in_stream_ptr->flags.stream_data_version)))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "capi sync : Expected sdata version %d received sdata version %d ",
             CAPI_STREAM_V2,
             in_stream_ptr->flags.stream_data_version);
      return result;
   }

#ifdef CAPI_SYNC_DEBUG
   AR_MSG(DBG_LOW_PRIO, "capi sync : Attempt metadata pass through from input %d to output %d", in_index, out_index);
#endif

   // Cache old eos value. Assumed output side eos is not used in metadata_propagate.
   prev_eos_out                     = out_stream_ptr->flags.marker_eos;
   out_stream_ptr->flags.marker_eos = FALSE;

   intf_extn_md_propagation_t input_md_info;
   memset(&input_md_info, 0, sizeof(input_md_info));
   input_md_info.df                          = in_port_ptr->media_fmt.header.format_header.data_format;
   input_md_info.len_per_ch_in_bytes         = 0;
   input_md_info.initial_len_per_ch_in_bytes = 0;
   input_md_info.bits_per_sample             = in_port_ptr->media_fmt.format.bits_per_sample;
   input_md_info.sample_rate                 = in_port_ptr->media_fmt.format.sampling_rate;

   intf_extn_md_propagation_t output_md_info;
   memscpy(&output_md_info, sizeof(output_md_info), &input_md_info, sizeof(input_md_info));
   output_md_info.initial_len_per_ch_in_bytes = 0;

   if (me_ptr->metadata_handler.metadata_propagate)
   {
      result |=
         me_ptr->metadata_handler.metadata_propagate(me_ptr->metadata_handler.context_ptr,
                                                     in_stream_ptr,
                                                     out_stream_ptr,
                                                     &(DUMMY_INT_MD_LIST_PTR), // Won't be used since algo delay is 0.
                                                     ALGO_DELAY_ZERO,
                                                     &input_md_info,
                                                     &output_md_info);
   }

   // After propagation, check if any eos moved to the output on this process call.
   sent_any_eos = out_stream_ptr->flags.marker_eos;

   // Add back prev_eos_out, if it existed. Assumes function never clears this from the output side.
   out_stream_ptr->flags.marker_eos |= prev_eos_out;
   out_stream_ptr->flags.end_of_frame = in_stream_ptr->flags.end_of_frame;
   in_stream_ptr->flags.end_of_frame  = FALSE;

   if (sent_any_eos)
   {
      generic_sync_mark_port_as_stopped(me_ptr, in_port_ptr->cmn.self_index);
   }

   // RR: EOF & flags?
   return result;
}

/**
 * Checks if the sdata_ptr has any DFG metadata in it by looping through the metadata list.
 * TODO(claguna): Does this belong in a common place? If so, md_id to find could be an argument.
 */
bool_t capi_sync_sdata_has_dfg(capi_sync_t *me_ptr, capi_stream_data_v2_t *sdata_ptr)
{
   bool_t has_dfg = FALSE;

   if (!sdata_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi sync capi_sync_sdata_has_dfg sdata_ptr was NULL, returning FALSE.");
      return FALSE;
   }

   module_cmn_md_list_t *list_ptr = sdata_ptr->metadata_list_ptr;
   while (list_ptr)
   {
      module_cmn_md_t *md_ptr = list_ptr->obj_ptr;
      if (MODULE_CMN_MD_ID_DFG == md_ptr->metadata_id)
      {
         has_dfg = TRUE;
         break;
      }

      list_ptr = list_ptr->next_ptr;
   }

   return has_dfg;
}

// RR: TODO Attempt merge with capi_sync_sdata_has_dfg
bool_t capi_sync_sdata_has_any_md(capi_stream_data_v2_t *sdata_ptr)
{

   if (!sdata_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi sync capi_sync_sdata_has_any_md sdata_ptr was NULL, returning FALSE.");
      return FALSE;
   }

   if (sdata_ptr->metadata_list_ptr)
   {
      return TRUE;
   }

   return FALSE;
}

static void capi_sync_adjust_md_after_prop(capi_sync_t *          me_ptr,
                                           capi_stream_data_v2_t *sdata_ptr,
                                           uint32_t               samples_consumed_from_internal_buf)
{
   module_cmn_md_list_t *list_ptr = sdata_ptr->metadata_list_ptr;
   while (list_ptr)
   {
      module_cmn_md_t *md_ptr = list_ptr->obj_ptr;

      if (md_ptr->offset < samples_consumed_from_internal_buf)
      {
         md_ptr->offset = 0;
      }
      else
      {
         md_ptr->offset -= samples_consumed_from_internal_buf;
      }

      list_ptr = list_ptr->next_ptr;
   }
}
/**
 * If we append trailing zeros, EOS/DFG needs to remain at the end of the data otherwise the fwk will remove them. This
 * function assigns all |flushing eos/dfg in the sdata_ptr's md_list|'s offsets to the passed in end_offset.
 */
static bool_t capi_sync_move_eos_dfg_to_end(capi_sync_t *me_ptr, uint32_t end_offset, capi_stream_data_v2_t *sdata_ptr)
{
   bool_t has_dfg = FALSE;

   if (!sdata_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi sync capi_sync_move_eos_dfg_to_end sdata_ptr was NULL, returning FALSE.");
      return FALSE;
   }

   module_cmn_md_list_t *list_ptr = sdata_ptr->metadata_list_ptr;
   while (list_ptr)
   {
      module_cmn_md_t *md_ptr = list_ptr->obj_ptr;
      if (MODULE_CMN_MD_ID_DFG == md_ptr->metadata_id)
      {
         AR_MSG(DBG_ERROR_PRIO,
                "MD_DBG: DFG moved to output port, moving to end of output. offset was %lu, now %lu. node_ptr 0x%lx, "
                "md_ptr 0x%lx.",
                md_ptr->offset,
                end_offset,
                list_ptr,
                md_ptr);

         md_ptr->offset = end_offset;
      }
      else if (MODULE_CMN_MD_ID_EOS == md_ptr->metadata_id)
      {
         module_cmn_md_eos_t *eos_metadata_ptr = 0;
         uint32_t             is_out_band      = md_ptr->metadata_flag.is_out_of_band;
         if (is_out_band)
         {
            eos_metadata_ptr = (module_cmn_md_eos_t *)md_ptr->metadata_ptr;
         }
         else
         {
            eos_metadata_ptr = (module_cmn_md_eos_t *)&(md_ptr->metadata_buf);
         }

         if (MODULE_CMN_MD_EOS_FLUSHING == eos_metadata_ptr->flags.is_flushing_eos)
         {
            AR_MSG(DBG_MED_PRIO,
                   "MD_DBG: flushing EOS moved to output port, moving to end of output. offset was %lu, now %lu. "
                   "node_ptr 0x%lx, md_ptr 0x%lx.",
                   md_ptr->offset,
                   end_offset,
                   list_ptr,
                   md_ptr);
            md_ptr->offset = end_offset;
         }
      }

      list_ptr = list_ptr->next_ptr;
   }

   return has_dfg;
}
