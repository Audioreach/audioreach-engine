/**
 * \file capi_smart_sync_ts_disc_utils.c
 * \brief
 *       Implementation of smart sync timestamp discontinuity handling utilities.
 *
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "capi_smart_sync_i.h"

/* =========================================================================
 * Static function declarations
 * =========================================================================*/

/**
 * Adds the specified number of zeros to the end of the circular buffer.
 */
static capi_err_t capi_smart_sync_buffer_zeros(capi_smart_sync_t *me_ptr, uint32_t zeros_bytes_per_ch, bool_t is_primary)
{
   capi_err_t capi_result = CAPI_EOK;

   uint32_t in_index = is_primary ? me_ptr->primary_in_port_info.cmn.index : me_ptr->secondary_in_port_info.cmn.index;
   capi_smart_sync_in_port_t *in_port_ptr =
      is_primary ? &me_ptr->primary_in_port_info : &me_ptr->secondary_in_port_info;

   if (SMART_SYNC_PORT_INDEX_INVALID == in_index) // port is not opened
   {
      return CAPI_EOK;
   }

   if (!in_port_ptr->circ_buf.bufs_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi smart sync: internal circular buffer for proc samples not allocated! ");
      return CAPI_EFAILED;
   }

   // Nothing to buffer.
   if (0 == zeros_bytes_per_ch)
   {
      return capi_result;
   }

   uint32_t write_index          = in_port_ptr->circ_buf.write_index;
   uint32_t bytes_written_per_ch = 0;

   // Write zeros to circ buffer.
   for (uint32_t ch = 0; ch < in_port_ptr->media_fmt.format.num_channels; ch++)
   {
      bytes_written_per_ch          = 0;
      uint32_t bytes_to_write       = 0;
      uint32_t circ_buf_size        = in_port_ptr->circ_buf.max_data_len_per_ch;
      uint32_t rem_lin_size         = 0;
      uint32_t zeros_remaining_size = zeros_bytes_per_ch;
      write_index                   = in_port_ptr->circ_buf.write_index;

      int8_t *circ_buf_start_addr = in_port_ptr->circ_buf.bufs_ptr + (ch * in_port_ptr->circ_buf.max_data_len_per_ch);
      int8_t *write_ptr           = NULL;

      while (zeros_remaining_size > 0)
      {
         write_ptr      = (int8_t *)(circ_buf_start_addr + write_index);
         rem_lin_size   = circ_buf_size - write_index;
         bytes_to_write = MIN(zeros_remaining_size, rem_lin_size);

         memset(write_ptr, 0, bytes_to_write);

         zeros_remaining_size -= bytes_to_write;
         bytes_written_per_ch += bytes_to_write;

         write_index += bytes_to_write;

         if (write_index >= circ_buf_size)
         {
            write_index = 0;
         }
      }
   }

   in_port_ptr->circ_buf.write_index = write_index;
   in_port_ptr->circ_buf.actual_data_len_per_ch += bytes_written_per_ch;

   capi_result |= capi_smart_sync_circ_buf_adjust_for_overflow(me_ptr, is_primary, write_index);

#ifdef SMART_SYNC_DEBUG_HIGH
   AR_MSG(DBG_HIGH_PRIO,
          "capi smart sync: after buffering new data on input idx %ld, actual_data_len_per_ch: %d, "
          "max_data_len_per_ch: %d ",
          in_port_ptr->cmn.index,
          in_port_ptr->circ_buf.actual_data_len_per_ch,
          in_port_ptr->circ_buf.max_data_len_per_ch);
#endif

   return capi_result;
}

capi_err_t capi_smart_sync_check_handle_ts_disc(capi_smart_sync_t * me_ptr,
                                                capi_stream_data_t *input[],
                                                capi_stream_data_t *output[],
                                                bool_t *            skip_processing_ptr,
                                                bool_t *            eof_found_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   if ((NULL == skip_processing_ptr) || (NULL == eof_found_ptr))
   {
      AR_MSG(DBG_ERROR_PRIO, "capi smart sync: received bad skip_processing_ptr");
      return (CAPI_EFAILED);
   }

   *skip_processing_ptr = FALSE;
   *eof_found_ptr       = FALSE;

   if(me_ptr->disable_ts_disc_handling)
   {
      return capi_result;
   }

   uint32_t PRIMARY_PATH_IDX = 0;
   uint32_t NUM_PATHS        = 2;
   for (uint32_t path_idx = 0; path_idx < NUM_PATHS; path_idx++)
   {
      bool_t   is_primary = (PRIMARY_PATH_IDX == path_idx);
      uint32_t in_index =
         is_primary ? me_ptr->primary_in_port_info.cmn.index : me_ptr->secondary_in_port_info.cmn.index;

      if (SMART_SYNC_PORT_INDEX_INVALID == in_index) // port is not opened
      {
         continue;
      }

      // Any EOF besides eof coming with EOS is considered as timestamp discontinuity which needs to be 'resynced'.
      if ((input && input[in_index] && input[in_index]->flags.end_of_frame && (!input[in_index]->flags.marker_eos)))
      {
         AR_MSG(DBG_MED_PRIO,
                "capi smart sync: Detected timestamp discontinuity on port %ld (0 primary 1 secondary) from EOF.",
                is_primary);

         // Since we will now handle EOF, clearing from the input.
         input[in_index]->flags.end_of_frame = FALSE;
         *eof_found_ptr                      = TRUE;
      }
   }

   if (!*eof_found_ptr)
   {
      return capi_result;
   }

   // In the case of eof coming during sync stage, we can continue as normal.
   if ((SMART_SYNC_STATE_BEFORE_FIRST_PROC_TICK == me_ptr->state))
   {
      AR_MSG(DBG_MED_PRIO,
             "capi smart sync: EOF found before proc tick, ignoring.");

      *skip_processing_ptr = FALSE;
   }
   // If we receive two EOF's back to back, give up and do conventional resync.
   else if (me_ptr->received_eof)
   {
      capi_result |= capi_smart_sync_resync_module_state(me_ptr);
   }
   else
   {
      // Otherwise, we need to calculate the next expected timestamp. Then in the next process call, we will
      // compare to next actual timestamp for zero padding. Then we can move to first_vfr state to buffer data
      // and send when a threshold of data has arrived.
      for (uint32_t path_idx = 0; path_idx < NUM_PATHS; path_idx++)
      {
         bool_t                     is_primary = (PRIMARY_PATH_IDX == path_idx);
         capi_smart_sync_in_port_t *cur_in_port_info_ptr =
            is_primary ? &me_ptr->primary_in_port_info : &me_ptr->secondary_in_port_info;

         if ((SMART_SYNC_PORT_INDEX_INVALID == cur_in_port_info_ptr->cmn.index) ||
             (CAPI_PORT_STATE_STARTED != cur_in_port_info_ptr->cmn.state)) // port is not opened and started
         {
            continue;
         }

         if (cur_in_port_info_ptr->is_ts_valid)
         {
            uint64_t *FRACT_TIME_PTR_NULL = NULL;
            // If no data was provided, use cached output buf timestamp (got updated to next frame at end of previous
            // process call).
            int64_t  base_ts            = cur_in_port_info_ptr->cur_out_buf_timestamp_us;
            uint32_t input_bytes_per_ch = 0;

            if ((input) && (input[cur_in_port_info_ptr->cmn.index]))
            {
               if (input[cur_in_port_info_ptr->cmn.index]->flags.is_timestamp_valid)
               {
                  base_ts = input[cur_in_port_info_ptr->cmn.index]->timestamp;
               }
               if (input[cur_in_port_info_ptr->cmn.index]->buf_ptr)
               {
                  input_bytes_per_ch = input[cur_in_port_info_ptr->cmn.index]->buf_ptr[0].actual_data_len;
               }
            }

            cur_in_port_info_ptr->next_expected_ts =
               base_ts + (int64_t)capi_cmn_bytes_to_us(input_bytes_per_ch,
                                                       cur_in_port_info_ptr->media_fmt.format.sampling_rate,
                                                       cur_in_port_info_ptr->media_fmt.format.bits_per_sample,
                                                       1, // num_channels = 1 since bytes is already specified per_ch.
                                                       FRACT_TIME_PTR_NULL);

            cur_in_port_info_ptr->is_next_expected_ts_valid = TRUE;

            AR_MSG(DBG_MED_PRIO,
                   "capi smart sync: calculated next_expected_ts msw %ld lsw %ld for is_primary %ld.",
                   (uint32_t)(cur_in_port_info_ptr->next_expected_ts >> 32),
                   (uint32_t)(cur_in_port_info_ptr->next_expected_ts),
                   is_primary);
         }
      }

      // Fall into processing case to buffer input to the circular buffer and potentially output a frame if threshold is
      // met.
      *skip_processing_ptr = FALSE;
      me_ptr->state        = SMART_SYNC_STATE_SYNCING;
      me_ptr->received_eof = TRUE;

      // Disable the threshold. This is needed because otherwise we will receive too much data in the next
      // process call. Example: 6ms + EOF came. Next process call could have 20ms of data, then smart sync
      // has 26 ms of data total...too much. We only needed 14ms new data.
      capi_result |= capi_smart_sync_raise_event_toggle_threshold_n_sync_state(me_ptr, FALSE);
   }

   return capi_result;
}

/**
 * In case of timestamp discontinuity, this function:
 * 1. Checks the difference between expected and actual incoming timestamp
 * 2. If that difference is small, pads zeros to the circular buffer to maintain continuous timestamps.
 * 3. If the difference is large or negative, it reverts to a resync behavior to get back to a synchronized
 *    voice timeline by the next proc tick. This fallback behavior has potential to drop 2 20ms frames in case
 *    of CDRX hence we attempt to manually fix the discontinuity with padding zeros in step 2 first.
 */
capi_err_t capi_smart_sync_check_pad_ts_disc_zeros(capi_smart_sync_t * me_ptr,
                                                   capi_stream_data_t *input[],
                                                   capi_stream_data_t *output[])
{
   capi_err_t capi_result = CAPI_EOK;

   // No handling needed if we didn't receive EOF on the last process call.
   if(!me_ptr->received_eof)
   {
      return capi_result;
   }

   me_ptr->received_eof = FALSE;

   // If there is no expected timestamp, then there was no timestamp discontinuity. Return early.
   if ((!me_ptr->primary_in_port_info.is_next_expected_ts_valid) &&
       (!me_ptr->secondary_in_port_info.is_next_expected_ts_valid))
   {
      return capi_result;
   }

   int64_t pri_zeros_to_pad_us = 0;
   int64_t sec_zeros_to_pad_us = 0;
   int64_t  TS_THRESHOLD_US     = 5000;

   uint32_t PRIMARY_PATH_IDX = 0;
   uint32_t NUM_PATHS        = 2;
   for (uint32_t path_idx = 0; path_idx < NUM_PATHS; path_idx++)
   {
      bool_t                     is_primary = (PRIMARY_PATH_IDX == path_idx);
      capi_smart_sync_in_port_t *cur_in_port_info_ptr =
         is_primary ? &me_ptr->primary_in_port_info : &me_ptr->secondary_in_port_info;

      // Check how many zeros to pad on the path.
      if ((SMART_SYNC_PORT_INDEX_INVALID != cur_in_port_info_ptr->cmn.index) &&
          (CAPI_PORT_STATE_STARTED == cur_in_port_info_ptr->cmn.state))
      {
         uint32_t in_index = cur_in_port_info_ptr->cmn.index;
         if ((input) && (input[in_index]) && input[in_index]->flags.is_timestamp_valid)
         {
            if (is_primary)
            {
               if (me_ptr->primary_in_port_info.is_next_expected_ts_valid)
               {
                  pri_zeros_to_pad_us = input[in_index]->timestamp - me_ptr->primary_in_port_info.next_expected_ts;
               }
            }
            else
            {
               if (me_ptr->secondary_in_port_info.is_next_expected_ts_valid)
               {
                  sec_zeros_to_pad_us = input[in_index]->timestamp - me_ptr->secondary_in_port_info.next_expected_ts;
               }
            }
         }
      }
   }

   if ((0 <= pri_zeros_to_pad_us) && (TS_THRESHOLD_US >= pri_zeros_to_pad_us) && (0 <= sec_zeros_to_pad_us) &&
       (TS_THRESHOLD_US >= sec_zeros_to_pad_us))
   {
      bool_t PRIMARY_PATH   = TRUE;
      bool_t SECONDARY_PATH = FALSE;

      uint32_t pri_zeros_bytes_per_ch =
         capi_cmn_us_to_bytes_per_ch(pri_zeros_to_pad_us,
                                     me_ptr->primary_in_port_info.media_fmt.format.sampling_rate,
                                     me_ptr->primary_in_port_info.media_fmt.format.bits_per_sample);
      uint32_t sec_zeros_bytes_per_ch =
         capi_cmn_us_to_bytes_per_ch(sec_zeros_to_pad_us,
                                     me_ptr->secondary_in_port_info.media_fmt.format.sampling_rate,
                                     me_ptr->secondary_in_port_info.media_fmt.format.bits_per_sample);

      AR_MSG(DBG_HIGH_PRIO,
             "capi smart sync: padding zeros to make discontinuous data continuous. primary %ld us/%ld bytes per ch, secondary %ld us/%ld bytes per ch.",
             (uint32_t)pri_zeros_to_pad_us,
             pri_zeros_bytes_per_ch,
             (uint32_t)sec_zeros_to_pad_us,
             sec_zeros_bytes_per_ch);

      capi_result |= capi_smart_sync_buffer_zeros(me_ptr, pri_zeros_bytes_per_ch, PRIMARY_PATH);
      capi_result |= capi_smart_sync_buffer_zeros(me_ptr, sec_zeros_bytes_per_ch, SECONDARY_PATH);
   }
   else
   {
      AR_MSG(DBG_HIGH_PRIO,
             "capi smart sync: doing total ts resync - zero padding amount too large or negative: primary %ld us, "
             "secondary %ld us.",
             (uint32_t)pri_zeros_to_pad_us,
             (uint32_t)sec_zeros_to_pad_us);

      // Timestamp difference of at least one port is negative or too large - revert to total ts resync.
      capi_result |= capi_smart_sync_resync_module_state(me_ptr);
   }

   // Clear expected timestamps.
   me_ptr->primary_in_port_info.is_next_expected_ts_valid   = FALSE;
   me_ptr->primary_in_port_info.next_expected_ts            = 0;
   me_ptr->secondary_in_port_info.is_next_expected_ts_valid = FALSE;
   me_ptr->secondary_in_port_info.next_expected_ts          = 0;

   return capi_result;
}