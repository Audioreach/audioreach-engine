/**
 * \file capi_priority_sync_ts.c
 * \brief
 *       Implement timestamp based sync.
 *
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "capi_priority_sync_i.h"

//drop the data from the secondary path and adjust timestamp accordingly.
static void priority_ts_sync_drop_data_from_sec(capi_priority_sync_t *me_ptr, uint32_t bytes_to_drop_per_ch)
{
   capi_priority_sync_in_port_t *in_port_ptr = &me_ptr->secondary_in_port_info;
   for (uint32_t ch = 0; ch < in_port_ptr->media_fmt.format.num_channels; ch++)
   {

      int8_t *write_ptr = in_port_ptr->int_stream.buf_ptr[ch].data_ptr;
      int8_t *read_ptr  = in_port_ptr->int_stream.buf_ptr[ch].data_ptr + bytes_to_drop_per_ch;

      // move data ahead
      memsmove(write_ptr,
               in_port_ptr->int_stream.buf_ptr[ch].max_data_len,
               read_ptr,
               in_port_ptr->int_stream.buf_ptr[ch].actual_data_len - bytes_to_drop_per_ch);

      in_port_ptr->int_stream.buf_ptr[ch].actual_data_len -= bytes_to_drop_per_ch;
   }

   // Adjust timestamp
   if (in_port_ptr->int_stream.flags.is_timestamp_valid && bytes_to_drop_per_ch)
   {
      uint64_t *FRACT_TIME_PTR_NULL = NULL;
      uint32_t  NUM_CH_1            = 1;
      uint32_t  drop_us             = capi_cmn_bytes_to_us(bytes_to_drop_per_ch,
                                              in_port_ptr->media_fmt.format.sampling_rate,
                                              in_port_ptr->media_fmt.format.bits_per_sample,
                                              NUM_CH_1,
                                              FRACT_TIME_PTR_NULL);

      in_port_ptr->int_stream.timestamp += drop_us;
   }

   PS_MSG(me_ptr->miid,
          DBG_HIGH_PRIO,
          "Port index %d, data_dropped_per_ch %lu, adjusted_ts_lsw %lu",
          in_port_ptr->cmn.index,
          bytes_to_drop_per_ch,
          (uint32_t)in_port_ptr->int_stream.timestamp);
}

/* function to pad zeros and adjust timestamp accordingly
 * zeros are calculated by the caller based on the current timestamp.
 * zeros are appended only to the path where data-flow is active.
 * less zeros are appended if not enough space available in internal buffer.
 */
static void priority_ts_sync_pad_zeros(capi_priority_sync_t *me_ptr,
                                       uint32_t              pri_zeros_to_pad_us,
                                       uint32_t              sec_zeros_to_pad_us)
{
   uint32_t PRIMARY_PATH_LOOP_IDX = 0;
   uint32_t NUM_PATHS             = 2;

   for (uint32_t path_idx = 0; path_idx < NUM_PATHS; path_idx++)
   {
      bool_t                        is_primary = (path_idx == PRIMARY_PATH_LOOP_IDX);
      capi_priority_sync_in_port_t *in_port_ptr =
         is_primary ? &me_ptr->primary_in_port_info : &me_ptr->secondary_in_port_info;
      bool_t is_path_active = capi_priority_sync_is_path_running(me_ptr, is_primary);

      uint32_t zeros_to_pad_us = (is_primary) ? pri_zeros_to_pad_us : sec_zeros_to_pad_us;

      if (!is_path_active || !zeros_to_pad_us)
      {
         continue;
      }

      uint32_t zeros_to_pad_per_ch = capi_cmn_us_to_bytes_per_ch(zeros_to_pad_us,
                                                                 in_port_ptr->media_fmt.format.sampling_rate,
                                                                 in_port_ptr->media_fmt.format.bits_per_sample);
      uint32_t bytes_aval_in_buf =
         in_port_ptr->int_stream.buf_ptr[0].max_data_len - in_port_ptr->int_stream.buf_ptr[0].actual_data_len;

      zeros_to_pad_per_ch = MIN(zeros_to_pad_per_ch, bytes_aval_in_buf);

      //if zeros are less than threshold then it means that the valid data is being sent in this call.
      //so move to synced state.
      if (zeros_to_pad_per_ch < in_port_ptr->threshold_bytes_per_ch)
      {
         in_port_ptr->data_continous = TRUE;
      }

      for (uint32_t ch = 0; ch < in_port_ptr->media_fmt.format.num_channels; ch++)
      {

         int8_t *write_ptr = in_port_ptr->int_stream.buf_ptr[ch].data_ptr + zeros_to_pad_per_ch;
         int8_t *read_ptr  = in_port_ptr->int_stream.buf_ptr[ch].data_ptr;

         // move data ahead
         memsmove(write_ptr,
                  in_port_ptr->int_stream.buf_ptr[ch].max_data_len - zeros_to_pad_per_ch,
                  read_ptr,
                  in_port_ptr->int_stream.buf_ptr[ch].actual_data_len);

         // set zeros
         memset(in_port_ptr->int_stream.buf_ptr[ch].data_ptr, 0, zeros_to_pad_per_ch);

         in_port_ptr->int_stream.buf_ptr[ch].actual_data_len += zeros_to_pad_per_ch;
      }

      // Adjust timestamp
      if (in_port_ptr->int_stream.flags.is_timestamp_valid && zeros_to_pad_per_ch)
      {
         uint64_t *FRACT_TIME_PTR_NULL = NULL;
         uint32_t  NUM_CH_1            = 1;
         uint32_t  zeros_us            = capi_cmn_bytes_to_us(zeros_to_pad_per_ch,
                                                  in_port_ptr->media_fmt.format.sampling_rate,
                                                  in_port_ptr->media_fmt.format.bits_per_sample,
                                                  NUM_CH_1,
                                                  FRACT_TIME_PTR_NULL);

         in_port_ptr->int_stream.timestamp -= zeros_us;
      }

      PS_MSG(me_ptr->miid,
             DBG_HIGH_PRIO,
             "Port index %d, zeros_padded_per_ch %lu, adjusted_ts_lsw %lu",
             in_port_ptr->cmn.index,
             zeros_to_pad_per_ch,
             (uint32_t)in_port_ptr->int_stream.timestamp);

      // don't need to adjust metadata, it is not stored during syncing state.
   }
}

/**
 * helper function to send data to the output.
 * If just secondary is active then data is dropped.
 * also moves to the synced state if not already moved
 */
static void priority_ts_sync_send_data(capi_priority_sync_t *me_ptr, capi_stream_data_t *output[])
{
   bool_t                        PRIMARY_PATH             = TRUE;
   bool_t                        SECONDARY_PATH           = FALSE;

   // check the data flow state of both paths
   bool_t is_primary_active   = capi_priority_sync_is_path_running(me_ptr, PRIMARY_PATH);
   bool_t is_secondary_active = capi_priority_sync_is_path_running(me_ptr, SECONDARY_PATH);
   bool_t is_secondary_ready = (DATA_PORT_STATE_STARTED == me_ptr->secondary_out_port_info.cmn.state) &&
                               capi_priority_sync_media_fmt_is_valid(me_ptr, SECONDARY_PATH);

   //if primary is active then send data to the capi output buffer
   if (is_primary_active)
   {
      capi_priority_sync_send_buffered_data(me_ptr, output, me_ptr->synced_state);
   }

   //drop the data from secondary if
   //1. primary in not active and just secondary is active. or
   //2. secondary output port is not started
   if (is_secondary_active && (!is_primary_active || !is_secondary_ready))
   {
      capi_priority_sync_in_port_t *secondary_input_port_ptr = &me_ptr->secondary_in_port_info;

      // if only secondary is active then drop threshold amount of data.
      uint32_t drop_size = MIN(secondary_input_port_ptr->threshold_bytes_per_ch,
                               secondary_input_port_ptr->int_stream.buf_ptr[0].actual_data_len);

      // drop data from internal buffer
      for (uint32_t ch = 0; ch < secondary_input_port_ptr->media_fmt.format.num_channels; ch++)
      {
         // Write to the beginning of the buffer.
         int8_t * write_ptr  = secondary_input_port_ptr->int_stream.buf_ptr[ch].data_ptr;
         uint32_t write_size = secondary_input_port_ptr->int_stream.buf_ptr[ch].max_data_len;
         int8_t * read_ptr   = secondary_input_port_ptr->int_stream.buf_ptr[ch].data_ptr + drop_size;
         uint32_t read_size  = secondary_input_port_ptr->int_stream.buf_ptr[ch].actual_data_len - drop_size;

         memscpy(write_ptr, write_size, read_ptr, read_size);

         secondary_input_port_ptr->int_stream.buf_ptr[ch].actual_data_len -= drop_size;
      }

      PS_MSG(me_ptr->miid, DBG_HIGH_PRIO, "data-dropped from secondary path. %lu bytes", drop_size);

      // invalidate timestamp so that it gets updated with new data.
      secondary_input_port_ptr->int_stream.flags.is_timestamp_valid = FALSE;

      // destroy metadata as no data sent when just secondary is active.
      capi_priority_sync_destroy_md_list(me_ptr, &(secondary_input_port_ptr->int_stream.metadata_list_ptr));
   }

   //print timestamp after sending data.
   {
      uint32_t PRIMARY_OUT_PORT_INDEX   = me_ptr->primary_out_port_info.cmn.index;
      uint32_t SECONDARY_OUT_PORT_INDEX = me_ptr->secondary_out_port_info.cmn.index;
      PS_MSG(me_ptr->miid,
             DBG_HIGH_PRIO,
             "data-sent. primary timestamp (is_valid) [%lu, %lu], secondary timestamp(is_valid) [%lu, %lu]",
             is_primary_active ? (uint32_t)output[PRIMARY_OUT_PORT_INDEX]->timestamp : 0,
             is_primary_active ? output[PRIMARY_OUT_PORT_INDEX]->flags.is_timestamp_valid : 0,
             (is_secondary_active && is_secondary_ready) ? (uint32_t)output[SECONDARY_OUT_PORT_INDEX]->timestamp : 0,
             (is_secondary_active && is_secondary_ready) ? output[SECONDARY_OUT_PORT_INDEX]->flags.is_timestamp_valid
                                                         : 0);
   }

   // if data is sent then check and move to synced state.
   if (PRIORITY_SYNC_STATE_SYNCED != me_ptr->synced_state)
   {
      if (is_primary_active && !me_ptr->primary_in_port_info.data_continous)
      {
         // if synced state is not achieved even after sending the data then it means more zeros padding required in
         // next call.  so to avoid timestamp discontinuity, invalidate timestamp.
         output[me_ptr->primary_out_port_info.cmn.index]->flags.is_timestamp_valid = FALSE;
      }

      // if primary is active then it should move to the data continuous state.
      bool_t is_primary_synced =
         !is_primary_active || (is_primary_active && me_ptr->primary_in_port_info.data_continous);

      // if secondary is active then it should move to the data continous state.
      bool_t is_secondary_synced =
         !is_secondary_active || (is_secondary_active && me_ptr->secondary_in_port_info.data_continous);

      if (is_primary_synced && is_secondary_synced)
      {
         me_ptr->synced_state = PRIORITY_SYNC_STATE_SYNCED;
         // Enable threshold.
         bool_t ENABLE_THRESHOLD = TRUE;
         capi_priority_sync_raise_event_toggle_threshold(me_ptr, ENABLE_THRESHOLD);
         (void)capi_priority_sync_handle_tg_policy(me_ptr);
      }
   }
}

static capi_err_t capi_priority_sync_mark_input_consumed(capi_priority_sync_t *me_ptr,
                                                         capi_stream_data_t *  input[],
                                                         bool_t                is_primary)
{
   capi_err_t capi_result = CAPI_EOK;

   uint32_t in_index = is_primary ? me_ptr->primary_in_port_info.cmn.index : me_ptr->secondary_in_port_info.cmn.index;
   capi_priority_sync_in_port_t *in_port_ptr =
      is_primary ? &me_ptr->primary_in_port_info : &me_ptr->secondary_in_port_info;
   // handle only started ports.
   if (DATA_PORT_STATE_STARTED == in_port_ptr->cmn.state)
   {
      capi_priority_sync_destroy_md_list(me_ptr, &(((capi_stream_data_v2_t *)input[in_index])->metadata_list_ptr));
      input[in_index]->flags.marker_eos   = FALSE;
      input[in_index]->flags.end_of_frame = FALSE;
   }

   return capi_result;
}

/**
 * Function to handle error during process.
 * Moves to syncing state.
 */
static void priority_ts_sync_handle_process_err(capi_priority_sync_t *me_ptr,
                                                capi_stream_data_t *  input[],
                                                bool_t                mark_buffer_unconsumed)
{
   bool_t PRIMARY_PATH   = TRUE;
   bool_t SECONDARY_PATH = FALSE;
   if (mark_buffer_unconsumed)
   {
      capi_priority_sync_mark_input_unconsumed(me_ptr, input, PRIMARY_PATH);
      capi_priority_sync_mark_input_unconsumed(me_ptr, input, SECONDARY_PATH);
   }
   else
   {
      capi_priority_sync_mark_input_consumed(me_ptr, input, PRIMARY_PATH);
      capi_priority_sync_mark_input_consumed(me_ptr, input, SECONDARY_PATH);
   }

   if (PRIORITY_SYNC_STATE_STARTING != me_ptr->synced_state)
   {
      me_ptr->synced_state = PRIORITY_SYNC_STATE_STARTING;

      // destroy metadata from internal buffer as md propagation is not supported in syncing state.
      capi_priority_sync_destroy_md_list(me_ptr, &(me_ptr->primary_in_port_info.int_stream.metadata_list_ptr));
      capi_priority_sync_destroy_md_list(me_ptr, &(me_ptr->secondary_in_port_info.int_stream.metadata_list_ptr));

      // Disable threshold.
      bool_t DISABLE_THRESHOLD = FALSE;
      capi_priority_sync_raise_event_toggle_threshold(me_ptr, DISABLE_THRESHOLD);
   }
}

capi_err_t priority_ts_sync_process(capi_t *_pif, capi_stream_data_t *input[], capi_stream_data_t *output[])
{
   capi_err_t            capi_result = CAPI_EOK;
   capi_priority_sync_t *me_ptr      = (capi_priority_sync_t *)_pif;
   bool_t                PRIMARY_PATH       = TRUE;
   bool_t                SECONDARY_PATH     = FALSE;

   uint32_t PRIMARY_IN_PORT_INDEX = me_ptr->primary_in_port_info.cmn.index;
   uint32_t SECONDARY_IN_PORT_INDEX = me_ptr->secondary_in_port_info.cmn.index;
   uint32_t PRIMARY_OUT_PORT_INDEX = me_ptr->primary_out_port_info.cmn.index;
   uint32_t SECONDARY_OUT_PORT_INDEX = me_ptr->secondary_out_port_info.cmn.index;
   capi_priority_sync_in_port_t *primary_input_port_ptr = &me_ptr->primary_in_port_info;
   capi_priority_sync_in_port_t *secondary_input_port_ptr = &me_ptr->secondary_in_port_info;
   capi_priority_sync_out_port_t *primary_output_port_ptr = &me_ptr->primary_out_port_info;
   capi_priority_sync_out_port_t *secondary_output_port_ptr = &me_ptr->secondary_out_port_info;

   bool_t any_data_found = FALSE;
   if (CAPI_EOK != (capi_result = priority_sync_check_for_started_ports(me_ptr, input, output, &any_data_found)))
   {
      PS_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Unexpected error.");
      // Don't consume any input, mark input as 0 length.
      bool_t BUFFER_CONSUMED = FALSE;
      priority_ts_sync_handle_process_err(me_ptr, input, BUFFER_CONSUMED);
      return capi_result;
   }

   // check the data flow state of both paths
   bool_t is_primary_active   = capi_priority_sync_is_path_running(me_ptr, PRIMARY_PATH);
   bool_t is_secondary_active = capi_priority_sync_is_path_running(me_ptr, SECONDARY_PATH);

   bool_t is_primary_ready = (DATA_PORT_STATE_STARTED == primary_output_port_ptr->cmn.state) &&
                             capi_priority_sync_media_fmt_is_valid(me_ptr, PRIMARY_PATH);

   bool_t is_secondary_ready = (DATA_PORT_STATE_STARTED == secondary_output_port_ptr->cmn.state) &&
                               capi_priority_sync_media_fmt_is_valid(me_ptr, SECONDARY_PATH);

   // if output port is started then output buffer should have at least threshold amount of space
   // also return if both path are at-gap or not started.
   if ((is_primary_ready &&
        (output[PRIMARY_OUT_PORT_INDEX]->buf_ptr[0].max_data_len < primary_input_port_ptr->threshold_bytes_per_ch)) ||
       (is_secondary_ready && (output[SECONDARY_OUT_PORT_INDEX]->buf_ptr[0].max_data_len <
                               secondary_input_port_ptr->threshold_bytes_per_ch)) ||
       (!is_primary_active && !is_secondary_active))
   {
      PS_MSG(me_ptr->miid, DBG_ERROR_PRIO, "need more data.");
      // Don't consume any input, mark input as 0 length.
      bool_t BUFFER_UNCONSUMED = TRUE;
      priority_ts_sync_handle_process_err(me_ptr, input, BUFFER_UNCONSUMED);
      return CAPI_ENEEDMORE;
   }

   switch (me_ptr->synced_state)
   {
      case PRIORITY_SYNC_STATE_SYNCED:
      {
         // check if capi buffer has threshold amount of data.
         bool_t pri_capi_buf_meet_threshold =
            is_primary_active &&
            (input[PRIMARY_IN_PORT_INDEX]->buf_ptr->actual_data_len >= primary_input_port_ptr->threshold_bytes_per_ch);
         bool_t sec_capi_buf_meet_threshold =
            is_secondary_active &&
            (input[SECONDARY_IN_PORT_INDEX]->buf_ptr->actual_data_len >= secondary_input_port_ptr->threshold_bytes_per_ch);

         // if threshold amount of data is not provided and EOF is also not set then return NEEDMORE.
         // in synced state fwk must provide threshold amount of data on all active ports.
         if ((is_primary_active &&
              (!pri_capi_buf_meet_threshold && !input[PRIMARY_IN_PORT_INDEX]->flags.end_of_frame)) ||
             (is_secondary_active &&
              (!sec_capi_buf_meet_threshold && !input[SECONDARY_IN_PORT_INDEX]->flags.end_of_frame)))
         {
            PS_MSG(me_ptr->miid,
                   DBG_ERROR_PRIO,
                   "insufficient bytes on input port in synced state! primary %lu, secondary %lu.",
                   is_primary_active ? input[PRIMARY_IN_PORT_INDEX]->buf_ptr->actual_data_len : 0,
                   is_secondary_active ? input[SECONDARY_IN_PORT_INDEX]->buf_ptr->actual_data_len : 0);

            // Don't consume any input, mark input as 0 length.
            bool_t BUFFER_UNCONSUMED = TRUE;
            priority_ts_sync_handle_process_err(me_ptr, input, BUFFER_UNCONSUMED);
            return CAPI_ENEEDMORE;
         }

         /* In synced state if EOF is received then first buffer it internally and if there is sufficient data then send
          * it out before propagating EOF. This is different from non-ts based priority synch logic where if EOF is
          * present then data is dropped immediately. This is because in that case there is no
          * buffering on primary path.
          *
          * Since we may have some internally buffered data in this case therefore it is good
          * to check if threshold can be satisfied with partial data (which came with EOF)
          */

         // cache data/md/eof/timestamp to the internal buffer.
         capi_priority_sync_buffer_new_data(me_ptr, input, is_primary_active, is_secondary_active);

         if (is_primary_active)
         {
            /* case 1: only primary is active
             * case 2: primary and secondary both are active
             **/

            bool_t pri_int_buf_threshold_met = (primary_input_port_ptr->int_stream.buf_ptr[0].actual_data_len >=
                                                primary_input_port_ptr->threshold_bytes_per_ch);

            //internal buffers are considered full if there is an EOF
            bool_t pri_int_buf_full = primary_input_port_ptr->flags.proc_ctx_has_eof ||
                                      (primary_input_port_ptr->int_stream.buf_ptr[0].actual_data_len ==
                                       primary_input_port_ptr->int_stream.buf_ptr[0].max_data_len);

            bool_t sec_int_buf_full =
               is_secondary_active && (secondary_input_port_ptr->flags.proc_ctx_has_eof ||
                                       secondary_input_port_ptr->int_stream.buf_ptr[0].actual_data_len ==
                                          secondary_input_port_ptr->int_stream.buf_ptr[0].max_data_len);

            /* if primary internal buffer has threshold amount of data and one of the internal buffer is completely
             * filled then send the data to the output.
             * If there is no EOF then threshold must be satisfied because in synced state capi buffer itself should
             * have threshold amount of data.
             * Buffer-full check is also added to ensure that there is PRIORITY_SYNC_TS_SYNC_WINDOW_US buffering in one
             * path. This additional buffering is useful to do timestamp based synchronization if one path goes to
             * at-gap and then starts again. Without this window synchronization can be achieved only in one direction.
             */
            if (pri_int_buf_threshold_met && (pri_int_buf_full || sec_int_buf_full))
            {
               priority_ts_sync_send_data(me_ptr, output);
            }
            else
            {
               // this may happen if EOF was cached in internal buffer and threshold didn't meet.
               me_ptr->synced_state = PRIORITY_SYNC_STATE_STARTING;
            }

            /* handle EOF and related metadata (EOS/DFG) now.
             * case1: EOF is stuck in internal buffer with some partial data.
             * 		Drop the data, propagate EOF (EOS/DFG) to capi output.
             * 		mark discontinuity on the path
             * case2: EOF is already propagated as part of priority_ts_sync_send_data
             * 		mark discontinuity on the path
             * */
            capi_priority_sync_check_handle_flow_gap(me_ptr, input, output);
         }
         else if (is_secondary_active)
         {
            /* Case: only secondary path is active. primary is in at-gap state.
             * data is not sent on the output, there is no point is sending data without primary path.
             * threshold amount of data is dropped to keep the consisting timing.
             */

            if (secondary_input_port_ptr->flags.proc_ctx_has_eof)
            {
               // handle EOF by clearing the internal buffer and marking data discontinuous
               capi_priority_sync_check_handle_flow_gap(me_ptr, input, output);

               // don't need to send eof as data is not being sent.
               output[SECONDARY_OUT_PORT_INDEX]->flags.end_of_frame = FALSE;
            }
            else
            {
               bool_t sec_int_buf_full = secondary_input_port_ptr->int_stream.buf_ptr[0].actual_data_len ==
                                         secondary_input_port_ptr->int_stream.buf_ptr[0].max_data_len;

               /* Checking for full internal buffer to make sure that there is PRIORITY_SYNC_TS_SYNC_WINDOW_US buffering
                * */
               if (sec_int_buf_full)
               {
                  priority_ts_sync_send_data(me_ptr, output);
               }
               else
               {
                  // Usually shouldn't happen
                  me_ptr->synced_state = PRIORITY_SYNC_STATE_STARTING;
                  PS_MSG(me_ptr->miid,
                         DBG_ERROR_PRIO,
                         "Error! secondary path internal buffer is not full in synced state.");
               }
            }
         }

         //if data becomes discontinuous on any port then move to syncing state.
         if ((is_primary_active && !primary_input_port_ptr->data_continous) ||
             (is_secondary_active && !secondary_input_port_ptr->data_continous))
         {
            // Begin synchronizing primary and secondary data.
            me_ptr->synced_state = PRIORITY_SYNC_STATE_STARTING;
            PS_MSG(me_ptr->miid, DBG_HIGH_PRIO, "Coming out of synced state due to data discontinuity");
         }

         //disable threshold for syncing
         if (PRIORITY_SYNC_STATE_STARTING == me_ptr->synced_state)
         {
            //destroy metadata from internal buffer as md propagation is not supported in syncing state.
            capi_priority_sync_destroy_md_list(me_ptr, &(me_ptr->primary_in_port_info.int_stream.metadata_list_ptr));
            capi_priority_sync_destroy_md_list(me_ptr, &(me_ptr->secondary_in_port_info.int_stream.metadata_list_ptr));

            // Disable threshold.
            bool_t DISABLE_THRESHOLD = FALSE;
            capi_priority_sync_raise_event_toggle_threshold(me_ptr, DISABLE_THRESHOLD);
         }
         break;
      }
      case PRIORITY_SYNC_STATE_STARTING:
      {
         // 1. First handle metadata.
         // if there is no EOF then drop all the metadata. not handling metadata during syncing state for
         // simplicity.
         // if there is EOF due to EOS/DFG then propagate metadata to output and handle at-gap

         uint32_t PRIMARY_PATH_LOOP_IDX = 0;
         uint32_t NUM_PATHS             = 2;

         for (uint32_t path_idx = 0; path_idx < NUM_PATHS; path_idx++)
         {
            bool_t                        is_primary = (path_idx == PRIMARY_PATH_LOOP_IDX);
            capi_priority_sync_in_port_t *in_port_ptr =
               is_primary ? &me_ptr->primary_in_port_info : &me_ptr->secondary_in_port_info;
            uint32_t in_index = in_port_ptr->cmn.index;

            if (capi_priority_sync_is_path_running(me_ptr, is_primary))
            {
               // destroy metadata during syncing state (if no EOF).
               if (!in_port_ptr->flags.proc_ctx_has_eof &&
                   (CAPI_STREAM_V2 == input[in_port_ptr->cmn.index]->flags.stream_data_version))
               {
                  capi_stream_data_v2_t *input_stream_ptr = (capi_stream_data_v2_t *)input[in_index];
                  capi_priority_sync_destroy_md_list(me_ptr, &(input_stream_ptr->metadata_list_ptr));
               }
            }
        }

        //2. Buffer data into internal buffer
        capi_priority_sync_buffer_new_data(me_ptr, input, is_primary_active, is_secondary_active);

        // in case of EOF during syncing state, return.
        if (primary_input_port_ptr->flags.proc_ctx_has_eof || secondary_input_port_ptr->flags.proc_ctx_has_eof)
        {
           //handle at-gap if EOS/DFG is propagated.
           capi_priority_sync_check_handle_flow_gap(me_ptr, input, output);

           //input data dropped!
           return CAPI_EOK;
        }

        //3. synchronize based on the timestamp if
        // 1. timestmap is valid on both path
        // 2. at least one path in not in data continuity (so that zeros can be added or data can be dropped to synchronize)
        if (is_primary_active && primary_input_port_ptr->int_stream.flags.is_timestamp_valid && is_secondary_active &&
            secondary_input_port_ptr->int_stream.flags.is_timestamp_valid &&
            (!primary_input_port_ptr->data_continous || !secondary_input_port_ptr->data_continous))
        {
           int64_t pri_timestamp_to_sync = primary_input_port_ptr->int_stream.timestamp;
           int64_t sec_timestamp_to_sync = secondary_input_port_ptr->int_stream.timestamp;

           // time different between both path, this will give us the amount of delay which should be added in leading
           // path to synchronize it with trailing path.
           int64_t diff_us =
              MAX(pri_timestamp_to_sync, sec_timestamp_to_sync) - MIN(pri_timestamp_to_sync, sec_timestamp_to_sync);

           //Identify amount of zeros to pad/or data to drop in each path to time synchronize
           uint32_t pri_zeros_to_pad_us = 0;
           uint32_t sec_zeros_to_pad_us = 0;
           uint32_t sec_data_to_drop_us = 0;

           // add delay in the path which is leading
           if (pri_timestamp_to_sync > sec_timestamp_to_sync)
           {
              pri_zeros_to_pad_us = diff_us;
           }
           else
           {
              sec_zeros_to_pad_us = diff_us;
           }

           if (!primary_input_port_ptr->data_continous && !secondary_input_port_ptr->data_continous)
           {
              if (diff_us <= PRIORITY_SYNC_TS_SYNC_WINDOW_US)
              {
                 // adjust the delay such that the leading path has 5ms delay and
                 // trailing path has 5-diff_us delay. this is to ensure that we at least add 5ms delay in one path.
                 pri_zeros_to_pad_us += (PRIORITY_SYNC_TS_SYNC_WINDOW_US - diff_us);
                 sec_zeros_to_pad_us += (PRIORITY_SYNC_TS_SYNC_WINDOW_US - diff_us);
              }
           }
           else
           {
              // if data-continuity is set then can;t add more zeros on that port.
              if (primary_input_port_ptr->data_continous && pri_zeros_to_pad_us > 0)
              {
                 // if can't pad zeros in the primary path then try to drop data from secondary path to synchronize.
                 sec_data_to_drop_us = pri_zeros_to_pad_us;
                 pri_zeros_to_pad_us = 0;
              }
           }

           if (sec_data_to_drop_us)
           {
              /* Sync wasn't achieved by padding zeros may need to drop the data from secondary path.
               * Data drop can only be done
               * 1. Either when primary internal buffer is full and we must move to synced state.
               * 2. Or secondary internal buffer is full and waiting for more data on primary path. */

              bool_t sec_int_buf_threshold_met = (secondary_input_port_ptr->int_stream.buf_ptr[0].actual_data_len >=
                                                  secondary_input_port_ptr->threshold_bytes_per_ch);

              bool_t pri_int_buf_full = (primary_input_port_ptr->int_stream.buf_ptr[0].actual_data_len ==
                                         primary_input_port_ptr->int_stream.buf_ptr[0].max_data_len);

              bool_t sec_int_buf_full = (secondary_input_port_ptr->int_stream.buf_ptr[0].actual_data_len ==
                                         secondary_input_port_ptr->int_stream.buf_ptr[0].max_data_len);

              uint32_t sec_bytes_to_drop_per_ch = 0;

              /**
               * 1. If primary internal buffer is full then can not wait further and must move to synced state in this
               * call.Try to drop the data from secondary path to synchronize but ensure threshold amount of data
               * is present after dropping
               * 2. If secondary internal buffer is full then can drop the data as long as threshold amount of data is
               * still present in the buffer.
               */

              // If primary internal buffer is full then has to go into synced state and send data in this call.
              secondary_input_port_ptr->data_continous = pri_int_buf_full ? TRUE : FALSE;

              if ((pri_int_buf_full && sec_int_buf_threshold_met) || sec_int_buf_full)
              {
                 // if primary buffer is full and secondary port has more than threshold amount of data, then can not
                 // wait further so try to drop the data as much as possible now and then move to SYNC state.

                 uint32_t extra_bytes_in_sec_int_buf_per_ch =
                    secondary_input_port_ptr->int_stream.buf_ptr[0].actual_data_len -
                    secondary_input_port_ptr->threshold_bytes_per_ch;

                 sec_bytes_to_drop_per_ch =
                    capi_cmn_us_to_bytes_per_ch(sec_data_to_drop_us,
                                                secondary_input_port_ptr->media_fmt.format.sampling_rate,
                                                secondary_input_port_ptr->media_fmt.format.bits_per_sample);

                 if(extra_bytes_in_sec_int_buf_per_ch >= sec_bytes_to_drop_per_ch)
                 {
                   //required data is dropped so can move to synced state.
                   secondary_input_port_ptr->data_continous = TRUE;
                 }
                 else
                 {
                    sec_bytes_to_drop_per_ch = extra_bytes_in_sec_int_buf_per_ch;
                 }

                 priority_ts_sync_drop_data_from_sec(me_ptr, sec_bytes_to_drop_per_ch);
              }
           }
           else if (pri_zeros_to_pad_us || sec_zeros_to_pad_us)
           {
              // pad zeros to synchronize and adjust timestamp
              priority_ts_sync_pad_zeros(me_ptr, pri_zeros_to_pad_us, sec_zeros_to_pad_us);
           }
           else
           {
              // sync is done,  no need to pad zeros or to drop data.
              primary_input_port_ptr->data_continous   = TRUE;
              secondary_input_port_ptr->data_continous = TRUE;
           }
        }

        //4. send data to output and move to synced state.
        if (is_primary_active && is_secondary_active)
        {
          //if both paths are active then move to synced state when
          //1. both have threshold amount of data.
          //2. and one path has full internal buffer (as we added 5ms delay)

          // assuming that synchronization by padding zeros is already done.
          // and since we added 5ms zeros in one of path therefore that path will have full internal buffer by the time
          // it caches threshold amount of actual data.

          bool_t pri_int_buf_threshold_met =
             (primary_input_port_ptr->int_stream.buf_ptr[0].actual_data_len >= primary_input_port_ptr->threshold_bytes_per_ch);

          bool_t sec_int_buf_threshold_met =
             (secondary_input_port_ptr->int_stream.buf_ptr[0].actual_data_len >= secondary_input_port_ptr->threshold_bytes_per_ch);

          bool_t pri_int_buf_full =
             (primary_input_port_ptr->int_stream.buf_ptr[0].actual_data_len == primary_input_port_ptr->int_stream.buf_ptr[0].max_data_len);

          bool_t sec_int_buf_full =
             (secondary_input_port_ptr->int_stream.buf_ptr[0].actual_data_len == secondary_input_port_ptr->int_stream.buf_ptr[0].max_data_len);

          if (pri_int_buf_threshold_met && sec_int_buf_threshold_met && (pri_int_buf_full || sec_int_buf_full))
          {
             // Send out buffered data and move to synced state.
             priority_ts_sync_send_data(me_ptr, output);
          }
        }
        else
        {
           // if only one path is active then move to synced state when
           // 1. threshold amount of data in buffer if data is not continuous (pad zeros to make buffer full)
           // 2. or buffer is full if data is continuous

           capi_priority_sync_in_port_t *in_port_ptr =
              (is_primary_active) ? primary_input_port_ptr : secondary_input_port_ptr;

           bool_t int_buf_threshold_met =
              (in_port_ptr->int_stream.buf_ptr[0].actual_data_len >= in_port_ptr->threshold_bytes_per_ch);

           if (!in_port_ptr->data_continous && int_buf_threshold_met)
           {
              //pad 5ms zeros, add delay now so that it can be used to synchronize later with other path
              priority_ts_sync_pad_zeros(me_ptr, PRIORITY_SYNC_TS_SYNC_WINDOW_US, PRIORITY_SYNC_TS_SYNC_WINDOW_US);

              in_port_ptr->data_continous = TRUE;
           }

           bool_t int_buf_full =
              (in_port_ptr->int_stream.buf_ptr[0].actual_data_len == in_port_ptr->int_stream.buf_ptr[0].max_data_len);

           if (in_port_ptr->data_continous && int_buf_full)
           {
              // Send out buffered data and move to synced state.
              priority_ts_sync_send_data(me_ptr, output);
           }
        }

        break;
      }
      default:
      {
         PS_MSG(me_ptr->miid, DBG_MED_PRIO, "unexpected port state %ld. returning error.");
         return CAPI_EFAILED;
      }
   }

   // If threshold is disabled then return ENEEDMORE so that fwk continues buffering
   capi_result = (me_ptr->threshold_is_disabled) ? CAPI_ENEEDMORE : CAPI_EOK;
   return capi_result;
}
