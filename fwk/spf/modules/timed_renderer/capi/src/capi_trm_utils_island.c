/**
 *   \file capi_trm_utils_island.c
 *   \brief
 *        This file contains utilities implementation of Timed Renderer Module
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "capi_trm_utils.h"

static capi_err_t capi_trm_move_metadata_from_input_to_loc_buf(capi_trm_t *               me_ptr,
                                                               capi_trm_circ_buf_frame_t *frame_ptr,
                                                               capi_stream_data_t *       in_sdata_ptr,
                                                               uint32_t                   write_offset,
                                                               uint32_t                   in_inital_len_per_ch_in_bytes,
                                                               uint32_t                   len_consumed_per_ch_in_bytes);

static capi_err_t capi_trm_propagate_metadata_from_local_buf_to_output(capi_trm_t *               me_ptr,
                                                                       capi_trm_circ_buf_frame_t *frame_ptr,
                                                                       capi_stream_data_t *       out_sdata_ptr,
                                                                       uint32_t initial_read_frame_offset,
                                                                       uint32_t prev_in_actual_data_len,
                                                                       uint32_t len_consumed_from_frame,
                                                                       uint32_t initial_bytes_in_output,
                                                                       capi_trm_propped_flags_t *propped_flags_ptr,
                                                                       bool_t                    is_first_op_write);

static capi_err_t capi_trm_get_first_timestamp(capi_trm_t *me_ptr,
                                               uint64_t *  first_timestamp_ptr,
                                               bool_t *    first_timestamp_valid_ptr);
static capi_err_t capi_trm_prefill_zeros(capi_trm_t *me_ptr, capi_stream_data_t *output[]);

/*==========================================================================
  FUNCTIONS
========================================================================== */
void capi_trm_clear_ttr(capi_trm_t *me_ptr)
{
   AR_MSG_ISLAND(DBG_HIGH_PRIO, "capi_time_renderer: clearing ttr, which was %ld", me_ptr->curr_ttr);

   me_ptr->render_decision    = CAPI_TRM_DECISION_PENDING;
   me_ptr->curr_ttr           = 0;
   me_ptr->first_ttr_received = FALSE;
   capi_trm_update_tgp_before_sync(me_ptr);
}

/*------------------------------------------------------------------------
  Function name: capi_trm_drop_all_metadata
  DESCRIPTION: Drops all metadata at the input port
  -----------------------------------------------------------------------*/
capi_err_t capi_trm_drop_all_metadata(capi_trm_t *me_ptr, capi_stream_data_t *input[])
{
   capi_err_t result = CAPI_EOK;

   if (input[0] == NULL)
   {
      return result;
   }

   // clear all flags.
   input[0]->flags.end_of_frame = 0;
   input[0]->flags.marker_eos   = 0;
   input[0]->flags.erasure      = 0;
#if 0
   if (CAPI_STREAM_V2 != input[0]->flags.stream_data_version)
   {
      AR_MSG_ISLAND(DBG_ERROR_PRIO, "capi_time_renderer: stream version must be 2");
      result |= CAPI_EFAILED;
      return result;
   }
#endif
   capi_stream_data_v2_t *in_stream_ptr = (capi_stream_data_v2_t *)input[0];

   // No metadata in stream
   if ((!in_stream_ptr) || (!in_stream_ptr->metadata_list_ptr))
   {
      return result;
   }

   posal_island_trigger_island_exit();
   capi_trm_metadata_destroy_handler(in_stream_ptr, me_ptr);
   return result;
}

/*------------------------------------------------------------------------
  Function name: capi_trm_adjust_offset
  DESCRIPTION: adjust all metadata with an offset
  -----------------------------------------------------------------------*/
static capi_err_t capi_trm_adjust_offset(module_cmn_md_list_t *md_list_ptr,
                                         uint32_t              bytes_consumed_per_ch,
                                         uint16_t              bits_per_sample)
{
   capi_err_t result = CAPI_EOK;

   uint32_t bytes_per_sample = CAPI_CMN_BITS_TO_BYTES(bits_per_sample);
   uint32_t samples_consumed = bytes_consumed_per_ch / bytes_per_sample;

   if (md_list_ptr)
   {
      module_cmn_md_list_t *node_ptr = md_list_ptr;
      while (node_ptr)
      {
         module_cmn_md_t *md_ptr = node_ptr->obj_ptr;

#ifdef TRM_DEBUG
         AR_MSG_ISLAND(DBG_HIGH_PRIO,
                    "capi_timed_rendere: update offset of md_ptr 0x%x md_id 0x%08lX with offset %lu by samples "
                    "consumed "
                    "%lu ",
                    md_ptr,
                    md_ptr->metadata_id,
                    md_ptr->offset,
                    samples_consumed);
#endif
         md_ptr->offset -= MIN(md_ptr->offset, samples_consumed);

         node_ptr = node_ptr->next_ptr;
      }
   }

   return result;
}

/*------------------------------------------------------------------------
  Function name: capi_trm_handle_metadata_b4_process
  DESCRIPTION: Search for and cache TTR, and destroy TTR.
  -----------------------------------------------------------------------*/
capi_err_t capi_trm_handle_metadata_b4_process(capi_trm_t *        me_ptr,
                                               capi_stream_data_t *input[],
                                               capi_stream_data_t *output[],
                                               bool_t *            is_resync_ptr)
{
   capi_err_t result = CAPI_EOK;

   if (input[0] == NULL)
   {
      return result;
   }

#if 0
   if (CAPI_STREAM_V2 != input[0]->flags.stream_data_version)
   {
      AR_MSG_ISLAND(DBG_ERROR_PRIO, "capi_timed_renderer: stream version must be 1");
      result |= CAPI_EFAILED;
      return result;
   }
#endif

   capi_stream_data_v2_t *in_stream_ptr = (capi_stream_data_v2_t *)input[0];

   // No metadata in stream
   if ((!in_stream_ptr) || (!in_stream_ptr->metadata_list_ptr))
   {
      return result;
   }

   posal_island_trigger_island_exit();
   capi_trm_metadata_b4_process_nlpi(me_ptr, in_stream_ptr, &is_resync_ptr);
   return result;
}

capi_err_t capi_trm_buffer_input_data(capi_trm_t *me_ptr, capi_stream_data_t *input[])
{
   capi_err_t result = CAPI_EOK;

   if (!capi_trm_in_has_data(input))
   {
      AR_MSG_ISLAND(DBG_ERROR_PRIO, "capi timed renderer: input not present");
      return CAPI_EFAILED;
   }

   if (input[0]->flags.erasure)
   {
#ifdef TRM_DEBUG
      AR_MSG_ISLAND(DBG_HIGH_PRIO, "capi timed renderer: input has erasure, not buffering (dropping instead).");
#endif

      // Since all input is consumed, clear erasure flag.
      input[0]->flags.erasure = FALSE;
      return result;
   }

   /* During sample slipping/stuffing from upstream, one sample less or more can come.
      if input is exactly 2ms then there is no drift correction and TRM should consume only 1ms.
      if input is less than 2ms and more than 1ms then there is drift correction, so cache everything
      so that prebuffer inside TRM is replenished.*/
   if (input[0]->buf_ptr[0].actual_data_len >= 2 * me_ptr->held_input_buf.frame_len_per_ch)
   {
      for (int i = 0; i < input[0]->bufs_num; i++)
      {
         input[0]->buf_ptr[i].actual_data_len = me_ptr->held_input_buf.frame_len_per_ch;
      }
   }

   uint32_t input_actual_data_len = input[0]->buf_ptr[0].actual_data_len;
   uint32_t read_offset           = 0;

   if (me_ptr->input_bytes_to_drop_per_ch && input_actual_data_len)
   {
      // TODO: dropping all mds for simplicity.
      capi_trm_drop_all_metadata(me_ptr, input);

      if (input_actual_data_len <= me_ptr->input_bytes_to_drop_per_ch)
      {
         AR_MSG_ISLAND(DBG_ERROR_PRIO, "capi timed renderer: Dropping the complete input buffer.");
         me_ptr->input_bytes_to_drop_per_ch -= input_actual_data_len;
         return CAPI_EOK;
      }
      else
      {
         read_offset = me_ptr->input_bytes_to_drop_per_ch;
         me_ptr->input_bytes_to_drop_per_ch = 0;
         AR_MSG_ISLAND(DBG_ERROR_PRIO,
                    "capi timed renderer: Dropping the %ld bytes per channel from input buffer.",
                    read_offset);
      }
   }

#ifdef TRM_DEBUG
   AR_MSG_ISLAND(DBG_HIGH_PRIO,
              "capi timed renderer: before write - held input buffer read index %d, write index %d  ",
              me_ptr->held_input_buf.read_index,
              me_ptr->held_input_buf.write_index);
#endif

   do // do while loop to buffer if there is any eos/eof without any data.
   {
      uint32_t frame_len_per_ch = me_ptr->held_input_buf.frame_len_per_ch;
      uint32_t bytes_copied     = 0;

      uint32_t frame_index  = me_ptr->held_input_buf.write_index;
      uint32_t num_channels = me_ptr->media_format.format.num_channels;

      //it is possible that the same frame is being read so need to adjust the write-offset.
      uint32_t write_offset = me_ptr->held_input_buf.frame_ptr[frame_index].read_offset +
                              me_ptr->held_input_buf.frame_ptr[frame_index].actual_data_len_per_ch;

#ifdef TRM_DEBUG
      AR_MSG_ISLAND(DBG_HIGH_PRIO,
                 "capi timed renderer: before write index %d, write offset %d, actual_data_len_per_ch: %lu, "
                 "input_actual_data_len: %lu, flags: "
                 "0x%lx, input_to_drop %d",
                 me_ptr->held_input_buf.write_index,
                 write_offset,
                 me_ptr->held_input_buf.frame_ptr[frame_index].actual_data_len_per_ch,
                 input_actual_data_len,
                 input[0]->flags.word,
                 me_ptr->input_bytes_to_drop_per_ch);
#endif

      // Check for overflow and fail if so.
      if ((me_ptr->held_input_buf.write_index == me_ptr->held_input_buf.read_index) &&
          (frame_len_per_ch == write_offset))
      {
         /* if read offset is non-zero
          * then there is small space available from 0 to read offset, but we are still considering buffer full to avoid
          * complexity in managing circular buffer.*/
         AR_MSG_ISLAND(DBG_ERROR_PRIO,
                       "capi timed renderer: Overflow on held buffer, read idx %ld, write idx %ld, read offset %ld, "
                       "held "
                       "buffer actual data len "
                       "at that idx %ld",
                       me_ptr->held_input_buf.read_index,
                       me_ptr->held_input_buf.write_index,
                       me_ptr->held_input_buf.frame_ptr[frame_index].read_offset,
                       me_ptr->held_input_buf.frame_ptr[frame_index].actual_data_len_per_ch);

         capi_trm_free_held_metadata(me_ptr, &(me_ptr->held_input_buf.frame_ptr[frame_index]), NULL, TRUE);
         return CAPI_EFAILED;
      }

      for (uint32_t ch = 0; ch < num_channels; ch++)
      {
         int8_t *read_ptr  = input[0]->buf_ptr[ch].data_ptr + read_offset;
         int8_t *write_ptr = me_ptr->held_input_buf.frame_ptr[frame_index].sdata.buf_ptr[ch].data_ptr + write_offset;

         bytes_copied =
            memscpy(write_ptr, (frame_len_per_ch - write_offset), read_ptr, (input_actual_data_len - read_offset));

         me_ptr->held_input_buf.actual_data_len_all_ch += bytes_copied;
      }

      me_ptr->held_input_buf.frame_ptr[frame_index].actual_data_len_per_ch += bytes_copied;

#ifdef TRM_DEBUG
      AR_MSG_ISLAND(DBG_HIGH_PRIO,
                 "capi timed renderer:%ld bytes per ch written from input to held buffer frame index %d, actual data "
                 "len per ch %d",
                 bytes_copied,
                 frame_index,
                 me_ptr->held_input_buf.frame_ptr[frame_index].actual_data_len_per_ch);
#endif

      capi_trm_move_metadata_from_input_to_loc_buf(me_ptr,
                                                   &(me_ptr->held_input_buf.frame_ptr[frame_index]),
                                                   input[0],
                                                   write_offset,
                                                   (input_actual_data_len - read_offset),
                                                   bytes_copied);

      read_offset += bytes_copied;
      write_offset += bytes_copied;

      if (write_offset >= frame_len_per_ch)
      {
         // Move to next frame.
         me_ptr->held_input_buf.write_index++;
         if (NUM_FRAMES_IN_LOCAL_BUFFER == me_ptr->held_input_buf.write_index)
         {
            me_ptr->held_input_buf.write_index = 0;
         }
      }
   } while (read_offset < input_actual_data_len);

#ifdef TRM_DEBUG
   AR_MSG_ISLAND(DBG_HIGH_PRIO,
              "capi timed renderer: after write - held input buffer read index %d, write index %d held buf "
              "actual_data_len_all_ch %ld ",
              me_ptr->held_input_buf.read_index,
              me_ptr->held_input_buf.write_index,
              me_ptr->held_input_buf.actual_data_len_all_ch);
#endif

   return result;
}

capi_err_t capi_trm_buffer_zeros(capi_trm_t *me_ptr, uint32_t zeros_to_pad_bytes_per_ch)
{
   capi_err_t result = CAPI_EOK;

#ifdef TRM_DEBUG
   AR_MSG_ISLAND(DBG_HIGH_PRIO,
                 "capi timed renderer: before write - held input buffer read index %d, write index %d  ",
                 me_ptr->held_input_buf.read_index,
                 me_ptr->held_input_buf.write_index);
#endif

   uint32_t frame_len_per_ch = me_ptr->held_input_buf.frame_len_per_ch;
   uint32_t num_channels     = me_ptr->media_format.format.num_channels;

   while (zeros_to_pad_bytes_per_ch)
   {
      uint32_t zeros_padded_bytes_per_ch = 0;
      uint32_t frame_index               = me_ptr->held_input_buf.write_index;

      // it is possible that the same frame is being read so need to adjust the write-offset.
      uint32_t write_offset = me_ptr->held_input_buf.frame_ptr[frame_index].read_offset +
                              me_ptr->held_input_buf.frame_ptr[frame_index].actual_data_len_per_ch;

      // Check for overflow and fail if so.
      if ((me_ptr->held_input_buf.write_index == me_ptr->held_input_buf.read_index) &&
          (frame_len_per_ch == write_offset))
      {
         /* if read offset is non-zero
          * then there is small space available from 0 to read offset, but we are still considering buffer full to avoid
          * complexity in managing circular buffer.*/
         AR_MSG_ISLAND(DBG_ERROR_PRIO,
                       "capi timed renderer: Overflow on held buffer, read idx %ld, write idx %ld, read offset %ld, "
                       "held "
                       "buffer actual data len "
                       "at that idx %ld",
                       me_ptr->held_input_buf.read_index,
                       me_ptr->held_input_buf.write_index,
                       me_ptr->held_input_buf.frame_ptr[frame_index].read_offset,
                       me_ptr->held_input_buf.frame_ptr[frame_index].actual_data_len_per_ch);
         return CAPI_EFAILED;
      }

#ifdef TRM_DEBUG
      AR_MSG_ISLAND(DBG_HIGH_PRIO,
                    "capi timed renderer: before write index %d, actual_data_len_per_ch: %lu, zeros_to_pad_per_ch: %lu",
                    me_ptr->held_input_buf.write_index,
                    me_ptr->held_input_buf.frame_ptr[frame_index].actual_data_len_per_ch,
                    zeros_to_pad_bytes_per_ch);
#endif
      for (uint32_t ch = 0; ch < num_channels; ch++)
      {
         int8_t *write_ptr = me_ptr->held_input_buf.frame_ptr[frame_index].sdata.buf_ptr[ch].data_ptr + write_offset;

         zeros_padded_bytes_per_ch = MIN((frame_len_per_ch - write_offset), zeros_to_pad_bytes_per_ch);
         memset(write_ptr, 0, zeros_padded_bytes_per_ch);

         me_ptr->held_input_buf.actual_data_len_all_ch += zeros_padded_bytes_per_ch;
      }

      me_ptr->held_input_buf.frame_ptr[frame_index].actual_data_len_per_ch += zeros_padded_bytes_per_ch;

      if (0 == write_offset)
      {
         me_ptr->held_input_buf.frame_ptr[frame_index].sdata.flags.word                = 0;
         me_ptr->held_input_buf.frame_ptr[frame_index].sdata.flags.stream_data_version = CAPI_STREAM_V2;
      }

      write_offset += zeros_padded_bytes_per_ch;

      zeros_to_pad_bytes_per_ch -= zeros_padded_bytes_per_ch;

#ifdef TRM_DEBUG
      AR_MSG_ISLAND(DBG_HIGH_PRIO,
                    "capi timed renderer: %ld bytes per ch zeros written to held buffer at frame %d, actual data len "
                    "per "
                    "ch %d",
                    zeros_padded_bytes_per_ch,
                    frame_index,
                    me_ptr->held_input_buf.frame_ptr[frame_index].actual_data_len_per_ch);
#endif

      if (write_offset >= frame_len_per_ch)
      {
         // Move to next frame.
         me_ptr->held_input_buf.write_index++;
         if (NUM_FRAMES_IN_LOCAL_BUFFER == me_ptr->held_input_buf.write_index)
         {
            me_ptr->held_input_buf.write_index = 0;
         }
      }
   }
#ifdef TRM_DEBUG
   AR_MSG_ISLAND(DBG_HIGH_PRIO,
                 "capi timed renderer: after write - held input buffer read index %d, write index %d held buf "
                 "actual_data_len_all_ch %ld ",
                 me_ptr->held_input_buf.read_index,
                 me_ptr->held_input_buf.write_index,
                 me_ptr->held_input_buf.actual_data_len_all_ch);
#endif

   return result;
}

/**
 * Get timestamp of the first frame buffered in the held buffer. This should only be used at the beginning
 * of a stream, while outputting the prebuffer to help calculate the timestamp of the prebuffer data.
 */
static capi_err_t capi_trm_get_first_timestamp(capi_trm_t *me_ptr,
                                               uint64_t *  first_timestamp_ptr,
                                               bool_t *    first_timestamp_valid_ptr)
{
   capi_err_t result = CAPI_EOK;

   if (NULL == first_timestamp_ptr || NULL == first_timestamp_valid_ptr)
   {
      return CAPI_EFAILED;
   }

   *first_timestamp_valid_ptr = FALSE;
   *first_timestamp_ptr       = 0;

   if (NULL == me_ptr->held_input_buf.frame_ptr)
   {
#ifdef TRM_DEBUG
      AR_MSG_ISLAND(DBG_ERROR_PRIO, "capi timed renderer: held buffer is not yet allocated!");
#endif

      return CAPI_EFAILED;
   }

   // Exit early if there's no buffered data.
   if (0 == me_ptr->held_input_buf.actual_data_len_all_ch)
   {
#ifdef TRM_DEBUG
      AR_MSG_ISLAND(DBG_ERROR_PRIO, "capi timed renderer: can't get first timestamp, held buffer is empty");
#endif

      return result;
   }

   uint32_t frame_index = me_ptr->held_input_buf.read_index;

   *first_timestamp_valid_ptr = me_ptr->held_input_buf.frame_ptr[frame_index].sdata.flags.is_timestamp_valid;
   *first_timestamp_ptr       = me_ptr->held_input_buf.frame_ptr[frame_index].sdata.timestamp;

   return result;
}

/**
 * Pads output with initial zeros. Assumes output is empty.
 */
static capi_err_t capi_trm_prefill_zeros(capi_trm_t *me_ptr, capi_stream_data_t *output[])
{
   capi_err_t result                        = CAPI_EOK;
   uint32_t   num_channels                  = me_ptr->media_format.format.num_channels;
   uint32_t   cur_zeros_to_pad_us           = MIN(me_ptr->frame_size_us, me_ptr->remaining_zeros_to_pad_us);
   uint32_t   cur_zeros_to_pad_bytes_per_ch = capi_cmn_us_to_bytes_per_ch(cur_zeros_to_pad_us,
                                                                        me_ptr->media_format.format.sampling_rate,
                                                                        me_ptr->media_format.format.bits_per_sample);

   if (0 == cur_zeros_to_pad_us)
   {
      return result;
   }

   if (cur_zeros_to_pad_bytes_per_ch > output[0]->buf_ptr[0].max_data_len)
   {
      AR_MSG_ISLAND(DBG_HIGH_PRIO,
                 "capi timed renderer: bad zero padding calculation, zeros to pad %ld bytes per ch is greater than "
                 "output max data len %ld bytes per ch",
                 cur_zeros_to_pad_bytes_per_ch,
                 output[0]->buf_ptr[0].max_data_len);
      return CAPI_EFAILED;
   }

   // Pad zeros.
   for (uint32_t ch = 0; ch < num_channels; ch++)
   {
      memset(output[0]->buf_ptr[ch].data_ptr, 0, cur_zeros_to_pad_bytes_per_ch);
      output[0]->buf_ptr[ch].actual_data_len = cur_zeros_to_pad_bytes_per_ch;
   }

   uint64_t first_timestamp       = 0;
   bool_t   first_timestamp_valid = FALSE;
   capi_trm_get_first_timestamp(me_ptr, &first_timestamp, &first_timestamp_valid);

   // Extrapolate timestamp from the first valid data.
   if (first_timestamp_valid)
   {
      output[0]->flags.is_timestamp_valid = TRUE;
      output[0]->timestamp                = first_timestamp - me_ptr->remaining_zeros_to_pad_us;
   }
   else
   {
      output[0]->flags.is_timestamp_valid = FALSE;
   }

   me_ptr->remaining_zeros_to_pad_us -= cur_zeros_to_pad_us;

#ifdef TRM_DEBUG
   AR_MSG_ISLAND(DBG_HIGH_PRIO,
              "capi timed renderer: padded %lu us zeros, %lu us remaining",
              cur_zeros_to_pad_us,
              me_ptr->remaining_zeros_to_pad_us);
#endif

   return result;
}

// Assumes that the held buffer frame is empty when we copy into it.
capi_err_t capi_trm_render_data_from_held_input_buffer(capi_trm_t *              me_ptr,
                                                       capi_stream_data_t *      output[],
                                                       capi_trm_propped_flags_t *propped_flags_ptr)
{
   capi_err_t result                     = CAPI_EOK;
   uint32_t   total_bytes_to_copy_per_ch = 0;
   uint32_t   num_channels               = me_ptr->media_format.format.num_channels;
   uint32_t   frame_size_bytes_per_ch    = capi_cmn_us_to_bytes_per_ch(me_ptr->frame_size_us,
                                                                  me_ptr->media_format.format.sampling_rate,
                                                                  me_ptr->media_format.format.bits_per_sample);

   // Set to TRUE if output actual data len is 0 before padding zeros.
   bool_t is_first_op_write = FALSE;

   // Only produce data if output was provided.
   if (!capi_trm_out_has_space(output))
   {
#ifdef TRM_DEBUG
      AR_MSG_ISLAND(DBG_HIGH_PRIO, "capi timed renderer: not producing output, output not present");
#endif
      return result;
   }

   // Exit early if there's no buffered data.
   if (0 == me_ptr->held_input_buf.actual_data_len_all_ch)
   {
      AR_MSG_ISLAND(DBG_ERROR_PRIO, "capi timed renderer: can't render from held buffer, held buffer is empty");
      return result;
   }
#ifdef TRM_DEBUG
   AR_MSG_ISLAND(DBG_HIGH_PRIO,
              "capi timed renderer: Producing output, held input buffer read index %d, write index %d before copy held "
              "buffer actual_data_len_all_ch %ld ",
              me_ptr->held_input_buf.read_index,
              me_ptr->held_input_buf.write_index,
              me_ptr->held_input_buf.actual_data_len_all_ch);
#endif
   is_first_op_write = (0 == output[0]->buf_ptr[0].actual_data_len);

   result |= capi_trm_prefill_zeros(me_ptr, output);

   if (frame_size_bytes_per_ch <= output[0]->buf_ptr[0].actual_data_len)
   {
#ifdef TRM_DEBUG
      AR_MSG_ISLAND(DBG_HIGH_PRIO,
                 "capi timed renderer: Padded a full frame of zeros, don't need to read from held input buffer.");
#endif
      return result;
   }

   // Render remaining data from the held input buffer into the output, up to the output frame size.
   total_bytes_to_copy_per_ch = frame_size_bytes_per_ch - output[0]->buf_ptr[0].actual_data_len;

   // Continue reading while there's room to write (and data to read, checked at end of loop).
   while (total_bytes_to_copy_per_ch)
   {
      uint32_t cur_bytes_copied          = 0;
      uint32_t frame_index               = me_ptr->held_input_buf.read_index;
      uint32_t initial_read_frame_offset = me_ptr->held_input_buf.frame_ptr[frame_index].read_offset;
      uint32_t prev_in_actual_data_len   = me_ptr->held_input_buf.frame_ptr[frame_index].actual_data_len_per_ch;
      uint32_t prev_out_buf_actual_len   = output[0]->buf_ptr[0].actual_data_len;
      capi_trm_propped_flags_t cur_propped_flags;
      cur_propped_flags.marker_eos = FALSE;

      if (output[0]->flags.end_of_frame)
      {
#ifdef TRM_DEBUG
         AR_MSG_ISLAND(DBG_HIGH_PRIO, "capi timed renderer: output has EOF, can't copy any more data");
#endif
         break;
      }

      if (total_bytes_to_copy_per_ch > (me_ptr->held_input_buf.actual_data_len_all_ch / num_channels))
      {
         AR_MSG_ISLAND(DBG_ERROR_PRIO,
                    "capi timed renderer: Not enough buffered data to produce full frame, needed %ld (per ch), have "
                    "%ld (per ch). Outputting what we have.",
                    total_bytes_to_copy_per_ch,
                    me_ptr->held_input_buf.actual_data_len_all_ch / num_channels);
         total_bytes_to_copy_per_ch = me_ptr->held_input_buf.actual_data_len_all_ch / num_channels;
      }

#ifdef TRM_DEBUG
      AR_MSG_ISLAND(DBG_HIGH_PRIO,
                 "capi timed renderer: before read index %d, prev_in_actual_data_len: %lu, total_bytes_to_copy_per_ch: "
                 "%lu, prev_out_buf_actual_len:%lu, frame flags 0x%lx",
                 me_ptr->held_input_buf.read_index,
                 prev_in_actual_data_len,
                 total_bytes_to_copy_per_ch,
                 prev_out_buf_actual_len,
                 me_ptr->held_input_buf.frame_ptr[frame_index].sdata.flags.word);
#endif

      for (uint32_t ch = 0; ch < num_channels; ch++)
      {
         int8_t *read_ptr = me_ptr->held_input_buf.frame_ptr[frame_index].sdata.buf_ptr[ch].data_ptr +
                            me_ptr->held_input_buf.frame_ptr[frame_index].read_offset;
         int8_t *write_ptr = output[0]->buf_ptr[ch].data_ptr + output[0]->buf_ptr[ch].actual_data_len;

         cur_bytes_copied = memscpy(write_ptr, total_bytes_to_copy_per_ch, read_ptr, prev_in_actual_data_len);

         output[0]->buf_ptr[ch].actual_data_len += cur_bytes_copied;
         me_ptr->held_input_buf.actual_data_len_all_ch -= cur_bytes_copied;
      }

      me_ptr->held_input_buf.frame_ptr[frame_index].read_offset += cur_bytes_copied;
      me_ptr->held_input_buf.frame_ptr[frame_index].actual_data_len_per_ch -= cur_bytes_copied;
      total_bytes_to_copy_per_ch -= cur_bytes_copied;

      capi_trm_propagate_metadata_from_local_buf_to_output(me_ptr,
                                                           &(me_ptr->held_input_buf.frame_ptr[frame_index]),
                                                           output[0],
                                                           initial_read_frame_offset,
                                                           prev_in_actual_data_len,
                                                           cur_bytes_copied,
                                                           prev_out_buf_actual_len,
                                                           &cur_propped_flags,
                                                           is_first_op_write);

      propped_flags_ptr->marker_eos |= cur_propped_flags.marker_eos;

#ifdef TRM_DEBUG
      AR_MSG_ISLAND(DBG_HIGH_PRIO,
                 "capi timed renderer: data read from frame idx %d, bytes copied %d, frame read offset %d out flags "
                 "0x%lx",
                 frame_index,
                 cur_bytes_copied,
                 me_ptr->held_input_buf.frame_ptr[frame_index].read_offset,
                 output[0]->flags.word);
#endif

      // Move to next frame when this frame is exhausted.
      if (0 == me_ptr->held_input_buf.frame_ptr[frame_index].actual_data_len_per_ch)
      {
         me_ptr->held_input_buf.frame_ptr[frame_index].read_offset = 0;
         me_ptr->held_input_buf.read_index++;
         if (NUM_FRAMES_IN_LOCAL_BUFFER == me_ptr->held_input_buf.read_index)
         {
            me_ptr->held_input_buf.read_index = 0;
         }
      }

      // If we were unable to copy any bytes, that implies that we exhausted the held buffer (or ran out of bytes to
      // copy). Stop copying.
      if ((0 == cur_bytes_copied))
      {
#ifdef TRM_DEBUG
         AR_MSG_ISLAND(DBG_HIGH_PRIO, "capi timed renderer: no data copied in this loop, breaking");
#endif
         break;
      }

      // If we copied any data to output, next writes will not be the first write to output.
      // (At this point we can assume 0 != cur_bytes_copied)
      is_first_op_write = FALSE;
   }

   // If the circular buffer is now empty, make sure the write pointer moves to the read index. This
   // might not be true if we were halfway through a frame when the circular buffer was emptied.
   if ((0 == me_ptr->held_input_buf.actual_data_len_all_ch) &&
       (me_ptr->held_input_buf.read_index != me_ptr->held_input_buf.write_index))
   {

      me_ptr->remaining_zeros_to_pad_us = 2 * me_ptr->frame_size_us;

#ifdef TRM_DEBUG
      AR_MSG_ISLAND(DBG_HIGH_PRIO,
                    "capi timed renderer: updating write index from %ld to read index %ld and restoring prebuffer to "
                    "%ld us",
                    me_ptr->held_input_buf.write_index,
                    me_ptr->held_input_buf.read_index,
                    me_ptr->remaining_zeros_to_pad_us);
#endif

      me_ptr->held_input_buf.write_index = me_ptr->held_input_buf.read_index;
   }

#ifdef TRM_DEBUG
   AR_MSG_ISLAND(DBG_HIGH_PRIO,
              "capi timed renderer: after read - held input buffer read index %d, write index %d  ",
              me_ptr->held_input_buf.read_index,
              me_ptr->held_input_buf.write_index);
#else
   if (20 <= me_ptr->process_count)
   {
      AR_MSG_ISLAND(DBG_HIGH_PRIO,
                 "capi timed renderer: after read - held input buffer read index %d, write index %d  ",
                 me_ptr->held_input_buf.read_index,
                 me_ptr->held_input_buf.write_index);
      me_ptr->process_count = 0;
   }
#endif

   return result;
}

static capi_err_t capi_trm_move_metadata_from_input_to_loc_buf(capi_trm_t *               me_ptr,
                                                               capi_trm_circ_buf_frame_t *frame_ptr,
                                                               capi_stream_data_t *       in_sdata_ptr,
                                                               uint32_t                   write_offset,
                                                               uint32_t                   in_inital_len_per_ch_in_bytes,
                                                               uint32_t                   len_consumed_per_ch_in_bytes)
{
   capi_err_t result = CAPI_EOK;

   // If sdata pointer is not given return.
   if (!in_sdata_ptr)
   {
      return result;
   }

#if 0
   if (CAPI_STREAM_V2 != in_sdata_ptr->flags.stream_data_version)
   {
#ifdef TRM_DEBUG
      AR_MSG_ISLAND(DBG_ERROR_PRIO,
                 "capi timed renderer: Invalid stream data version %d  ",
                 frame_ptr->sdata.flags.stream_data_version);
#endif
      return CAPI_EFAILED;
   }
#endif

   // if metadata is not present in the input frame return. do nothing.
   capi_stream_data_v2_t *in_sdata_v2_ptr = (capi_stream_data_v2_t *)in_sdata_ptr;

   bool_t input_had_eos           = in_sdata_ptr->flags.marker_eos;
   bool_t is_input_fully_consumed = (in_inital_len_per_ch_in_bytes == len_consumed_per_ch_in_bytes);

   /*
    * Iterate through input buffer metadata list and check if the metadata needs to buffered
    */
   if (NULL != in_sdata_v2_ptr->metadata_list_ptr)
   {
      // Although we report some algo delay, it is handed as buffering delay from within the module.
      uint32_t                   ALGO_DELAY_ZERO = 0;
      intf_extn_md_propagation_t input_md_info;
      memset(&input_md_info, 0, sizeof(input_md_info));
      input_md_info.df              = me_ptr->media_format.header.format_header.data_format;
      input_md_info.bits_per_sample = me_ptr->media_format.format.bits_per_sample;
      input_md_info.sample_rate     = me_ptr->media_format.format.sampling_rate;

      input_md_info.len_per_ch_in_bytes         = len_consumed_per_ch_in_bytes;
      input_md_info.initial_len_per_ch_in_bytes = in_inital_len_per_ch_in_bytes;
      input_md_info.buf_delay_per_ch_in_bytes   = 0;

      intf_extn_md_propagation_t output_md_info;
      memscpy(&output_md_info, sizeof(output_md_info), &input_md_info, sizeof(input_md_info));
      output_md_info.len_per_ch_in_bytes         = len_consumed_per_ch_in_bytes;
      output_md_info.initial_len_per_ch_in_bytes = write_offset;
      output_md_info.buf_delay_per_ch_in_bytes   = 0;

      // TODO: check capi error.
      me_ptr->metadata_handler.metadata_propagate(me_ptr->metadata_handler.context_ptr,
                                                  in_sdata_v2_ptr,   // capi sdata v2 ptr
                                                  &frame_ptr->sdata, // cur wr frame sdata
                                                  NULL,              // int md list not needed with 0 algo delay
                                                  ALGO_DELAY_ZERO,   // algo delay
                                                  &input_md_info,
                                                  &output_md_info);

      capi_trm_adjust_offset(in_sdata_v2_ptr->metadata_list_ptr,
                             len_consumed_per_ch_in_bytes,
                             me_ptr->media_format.format.bits_per_sample);
   }
   else
   {
#ifdef TRM_DEBUG
      AR_MSG_ISLAND(DBG_HIGH_PRIO, "capi timed renderer: empty metadata list");
#endif
   }

   // --- Copy stream flags ---

   // These flags should be copied since they apply to all data in the input buffer.
   // Flags not copied here:
   // 1. end_of_frame - handled separately below.
   // 2. marker_eos   - handled by metadata_propagate.

   // When copying from input to held buffer, always copy to a new held buffer. So no need
   // to worry about aggregating flags.
   frame_ptr->sdata.flags.marker_1            = in_sdata_ptr->flags.marker_1;
   frame_ptr->sdata.flags.marker_2            = in_sdata_ptr->flags.marker_2;
   frame_ptr->sdata.flags.marker_3            = in_sdata_ptr->flags.marker_3;
   frame_ptr->sdata.flags.erasure             = in_sdata_ptr->flags.erasure;
   frame_ptr->sdata.flags.stream_data_version = in_sdata_ptr->flags.stream_data_version;
   frame_ptr->sdata.flags.is_timestamp_valid  = in_sdata_ptr->flags.is_timestamp_valid;

   frame_ptr->sdata.timestamp = in_sdata_ptr->timestamp;

   // eof propagation
   // EOF propagation during EOS: propagate only once input EOS goes to output.
   if (input_had_eos)
   {
      if (frame_ptr->sdata.flags.marker_eos && !in_sdata_ptr->flags.marker_eos)
      {
         AR_MSG_ISLAND(DBG_HIGH_PRIO, "EOS reached TRM held buffer");
         me_ptr->held_input_buf.is_eos_buffered = TRUE;

         if (in_sdata_ptr->flags.end_of_frame)
         {
            in_sdata_ptr->flags.end_of_frame    = FALSE;
            frame_ptr->sdata.flags.end_of_frame = TRUE;
            AR_MSG_ISLAND(DBG_HIGH_PRIO, "EOF was propagated to internal buffer");
         }
      }
   }
   else
   {
      // Otherwise, propagate EOF if input was fully consumed.
      if (is_input_fully_consumed)
      {
         if (in_sdata_v2_ptr->flags.end_of_frame)
         {
            in_sdata_v2_ptr->flags.end_of_frame = FALSE;
            frame_ptr->sdata.flags.end_of_frame = TRUE;
            AR_MSG_ISLAND(DBG_HIGH_PRIO, "EOF was propagated to internal buffer");
         }
      }
   }

#ifdef TRM_DEBUG
   AR_MSG_ISLAND(DBG_HIGH_PRIO,
              "capi timed renderer: buffering flags 0x%lx, timestamp %ld input has_eos= %d, frame has_eos= %d ",
              (uint32_t)frame_ptr->sdata.flags.word,
              (uint32_t)frame_ptr->sdata.timestamp,
              in_sdata_v2_ptr->flags.marker_eos,
              frame_ptr->sdata.flags.marker_eos);
#endif

   return CAPI_EOK;
}

capi_err_t capi_trm_free_held_metadata(capi_trm_t *               me_ptr,
                                       capi_trm_circ_buf_frame_t *frame_ptr,
                                       module_cmn_md_list_t **    stream_associated_md_list_pptr,
                                       bool_t                     force_free)
{
   capi_err_t result = CAPI_EOK;

   // metadata overrun needs to be printed as a warning.
   if (frame_ptr->sdata.metadata_list_ptr)
   {
      // TODO: Destroy only sample associated and Buffer associated metadata. Stream associated metadata must
      //       be carried forward for overrun. Currently force_free is set TRUE for overrun cases as well.

      module_cmn_md_list_t *cur_node_ptr = frame_ptr->sdata.metadata_list_ptr;
      while (cur_node_ptr)
      {
         // get current metadata node object.
         module_cmn_md_t *md_ptr = (module_cmn_md_t *)cur_node_ptr->obj_ptr;

         // Check if metadata can be destoryed or not
         bool_t is_stream_associated = FALSE;
         bool_t need_to_be_dropped   = FALSE; /* by default metadata is rendered during destroy. */
         if (MODULE_CMN_MD_ID_EOS == md_ptr->metadata_id)
         {

#ifdef TRM_DEBUG
            AR_MSG_ISLAND(DBG_HIGH_PRIO, "capi timed renderer: EOS found.");
#endif
            is_stream_associated = TRUE;
         }
         else
         {
            AR_MSG_ISLAND(DBG_HIGH_PRIO,
                       "capi timed renderer: Unknown metadata found dropping it. metadata_id:  0x%x ",
                       md_ptr->metadata_id);
            need_to_be_dropped = TRUE;
         }

         // Destroyed if,
         //   1. Force free is true.
         //        OR
         //   2. If its NOT stream associated metadata.
         if (force_free || !is_stream_associated)
         {
            capi_err_t capi_res = me_ptr->metadata_handler.metadata_destroy(me_ptr->metadata_handler.context_ptr,
                                                                            cur_node_ptr,
                                                                            need_to_be_dropped, // dropping metadata
                                                                            &frame_ptr->sdata.metadata_list_ptr);
            if (CAPI_FAILED(capi_res))
            {
               AR_MSG_ISLAND(DBG_ERROR_PRIO,
                          "capi timed renderer: Failed in destroying metadata 0x%lx",
                          md_ptr->metadata_id);
            }

            // when a node is destoyed init again to the head of the list.
            cur_node_ptr = frame_ptr->sdata.metadata_list_ptr;
            continue;
         }

         // Retained if,
         //  1. If its IS stream associated metadata.
         //             &&
         //  2. only if force is False.
         if (!force_free && is_stream_associated)
         {
            if (stream_associated_md_list_pptr)
            {
               // for the last reader just move metadata node from circular buffer to reader client handle.
               spf_list_move_node_to_another_list((spf_list_node_t **)stream_associated_md_list_pptr,
                                                  (spf_list_node_t *)cur_node_ptr,
                                                  (spf_list_node_t **)&frame_ptr->sdata.metadata_list_ptr);
            }

            // when a node is destroyed init again to the head of the list.
            cur_node_ptr = frame_ptr->sdata.metadata_list_ptr;
            continue;
         }

         LIST_ADVANCE(cur_node_ptr);
      }
   }

   return result;
}

/**
 * Removes all data and metadata from the held buffer.
 */
capi_err_t capi_trm_flush_held_buffer(capi_trm_t *me_ptr, module_cmn_md_list_t **    stream_associated_md_list_pptr, bool_t force_free_md)
{
   capi_err_t result = CAPI_EOK;
   uint32_t dropped_data = 0;

   if (me_ptr->held_input_buf.frame_ptr)
   {
      for (uint32_t i = 0; i < NUM_FRAMES_IN_LOCAL_BUFFER; i++)
      {
         capi_trm_circ_buf_frame_t *cur_frame_ptr = &(me_ptr->held_input_buf.frame_ptr[i]);
         result |= capi_trm_free_held_metadata(me_ptr, cur_frame_ptr, stream_associated_md_list_pptr, force_free_md);

         dropped_data += cur_frame_ptr->actual_data_len_per_ch;
         // cur_frame_ptr->sdata.buf_ptr actual/max data length are ignored in this implementation.
         cur_frame_ptr->sdata.flags.word       = 0;
         cur_frame_ptr->actual_data_len_per_ch = 0;
         cur_frame_ptr->read_offset            = 0;
      }
   }

   AR_MSG_ISLAND(DBG_HIGH_PRIO,
              "capi timed renderer: Flushing internal buffer, before flush - read idx %ld write idx %ld, dropped_data %lu bytes per ch",
              me_ptr->held_input_buf.read_index,
              me_ptr->held_input_buf.write_index,
              dropped_data);

   me_ptr->held_input_buf.read_index             = 0;
   me_ptr->held_input_buf.write_index            = 0;
   me_ptr->held_input_buf.is_eos_buffered        = FALSE;
   me_ptr->held_input_buf.actual_data_len_all_ch = 0;
   return result;
}

/* Propagates metadata from circular buffer to output sdata.
 * 1. initial_bytes_in_output
 *
 * */
static capi_err_t capi_trm_propagate_metadata_from_local_buf_to_output(capi_trm_t *               me_ptr,
                                                                       capi_trm_circ_buf_frame_t *frame_ptr,
                                                                       capi_stream_data_t *       out_sdata_ptr,
                                                                       uint32_t initial_read_frame_offset,
                                                                       uint32_t prev_in_actual_data_len,
                                                                       uint32_t len_consumed_from_frame,
                                                                       uint32_t initial_bytes_in_output,
                                                                       capi_trm_propped_flags_t *propped_flags_ptr,
                                                                       bool_t                    is_first_op_write)
{
   capi_err_t result = CAPI_EOK;

#ifdef TRM_DEBUG
   AR_MSG_ISLAND(DBG_HIGH_PRIO, "capi timed renderer: entered output metadata propagation");
#endif

   if (NULL == propped_flags_ptr)
   {
      return CAPI_EFAILED;
   }

   // if the read client doesn't pass sdata, do nothing.
   if (NULL == out_sdata_ptr)
   {
      return result;
   }

#if 0
   // if the output sdata is not stream v2, metadata propagation is not supported.
   if ((CAPI_STREAM_V2 != out_sdata_ptr->flags.stream_data_version))
   {
#ifdef TRM_DEBUG
      AR_MSG_ISLAND(DBG_ERROR_PRIO,
                 "capi timed renderer: Invalid stream data version %d  ",
                 frame_ptr->sdata.flags.stream_data_version);
#endif
      return CAPI_EFAILED;
   }
#endif

   bool_t is_input_fully_consumed  = (prev_in_actual_data_len == len_consumed_from_frame);
   bool_t input_had_eos            = frame_ptr->sdata.flags.marker_eos;
   bool_t output_had_eos           = out_sdata_ptr->flags.marker_eos;
   out_sdata_ptr->flags.marker_eos = FALSE;

   // if the current reader clients has metadata associated with the current frame propagate it to the output.
   if (frame_ptr->sdata.metadata_list_ptr)
   {
      // Although we report some algo delay, it is handed as buffering delay from within the module.
      uint32_t                   ALGO_DELAY_ZERO  = 0;
      capi_stream_data_v2_t *    out_sdata_v2_ptr = (capi_stream_data_v2_t *)out_sdata_ptr;
      intf_extn_md_propagation_t input_md_info;
      memset(&input_md_info, 0, sizeof(input_md_info));
      input_md_info.df                          = me_ptr->media_format.header.format_header.data_format;
      input_md_info.len_per_ch_in_bytes         = len_consumed_from_frame;
      input_md_info.initial_len_per_ch_in_bytes = prev_in_actual_data_len;
      input_md_info.buf_delay_per_ch_in_bytes   = 0;
      input_md_info.bits_per_sample             = me_ptr->media_format.format.bits_per_sample;
      input_md_info.sample_rate                 = me_ptr->media_format.format.sampling_rate;

      intf_extn_md_propagation_t output_md_info;
      memscpy(&output_md_info, sizeof(output_md_info), &input_md_info, sizeof(input_md_info));
      output_md_info.len_per_ch_in_bytes         = len_consumed_from_frame;
      output_md_info.initial_len_per_ch_in_bytes = initial_bytes_in_output;
      output_md_info.buf_delay_per_ch_in_bytes   = 0;

      // propagate metadata from reader frame handle to output stream data pointer.
      capi_err_t capi_res = me_ptr->metadata_handler.metadata_propagate(me_ptr->metadata_handler.context_ptr,
                                                                        &frame_ptr->sdata, // circ buf frame
                                                                                           // sdata
                                                                        out_sdata_v2_ptr,  // capi sdata v2 ptr
                                                                        NULL, // algo delay 0 -> no int md list needed.
                                                                        ALGO_DELAY_ZERO,
                                                                        &input_md_info,
                                                                        &output_md_info);
      if (CAPI_FAILED(capi_res))
      {
         AR_MSG_ISLAND(DBG_ERROR_PRIO,
                    "capi timed renderer: failed in propagating metadata to output, err code 0x%lx.",
                    capi_res);
         return capi_res;
      }

      capi_trm_adjust_offset(frame_ptr->sdata.metadata_list_ptr,
                             len_consumed_from_frame,
                             me_ptr->media_format.format.bits_per_sample);
   }
   else
   {
#ifdef TRM_DEBUG
      AR_MSG_ISLAND(DBG_HIGH_PRIO, "capi timed renderer: empty metadata list");
#endif
   }

   propped_flags_ptr->marker_eos = out_sdata_ptr->flags.marker_eos;
   out_sdata_ptr->flags.marker_eos |= output_had_eos;

   // --- Copy stream flags ---

   // These flags should be copied since they apply to all data in the input buffer.
   // Flags not copied here:
   // 1. end_of_frame - handled separately below.
   // 2. marker_eos   - handled by metadata_propagate.

   // If copying to initially empty output, reassign over flags (previous flag values should be ignored).
   // Since we ignore flags when padding zeros, this bool needs to be true even if output is partially padded
   // with zeros.
   if (is_first_op_write)
   {
      out_sdata_ptr->flags.marker_1            = frame_ptr->sdata.flags.marker_1;
      out_sdata_ptr->flags.marker_2            = frame_ptr->sdata.flags.marker_2;
      out_sdata_ptr->flags.marker_3            = frame_ptr->sdata.flags.marker_3;
      out_sdata_ptr->flags.erasure             = frame_ptr->sdata.flags.erasure;
      out_sdata_ptr->flags.stream_data_version = frame_ptr->sdata.flags.stream_data_version;
   }
   // If copying to partially filled output, use |= to save previous flag versions.
   else
   {
      out_sdata_ptr->flags.marker_1 |= frame_ptr->sdata.flags.marker_1;
      out_sdata_ptr->flags.marker_2 |= frame_ptr->sdata.flags.marker_2;
      out_sdata_ptr->flags.marker_3 |= frame_ptr->sdata.flags.marker_3;
      out_sdata_ptr->flags.erasure |= frame_ptr->sdata.flags.erasure;
      out_sdata_ptr->flags.stream_data_version |= frame_ptr->sdata.flags.stream_data_version;
   }

   // Timestamp copy is only needed if output is empty. Otherwise we assume timestamps are continuous and
   // combine in the output buffer.
   if (0 == initial_bytes_in_output)
   {
      if (frame_ptr->sdata.flags.is_timestamp_valid)
      {
         uint32_t  NUM_CH_ONE          = 1; // read offset is already per ch.
         uint64_t *FRACT_TIME_PTR_NULL = NULL;

         // Offset input timestamp by the initial read offset.
         uint64_t input_ts =
            frame_ptr->sdata.timestamp + capi_cmn_bytes_to_us(initial_read_frame_offset,
                                                              me_ptr->media_format.format.sampling_rate,
                                                              me_ptr->media_format.format.bits_per_sample,
                                                              NUM_CH_ONE,
                                                              FRACT_TIME_PTR_NULL);

         out_sdata_ptr->timestamp                = input_ts;
         out_sdata_ptr->flags.is_timestamp_valid = frame_ptr->sdata.flags.is_timestamp_valid;
      }
   }

   // eof propagation
   // EOF propagation during EOS: propagate only once input EOS goes to output.
   if (input_had_eos)
   {
      if (out_sdata_ptr->flags.marker_eos && !frame_ptr->sdata.flags.marker_eos)
      {
         if (frame_ptr->sdata.flags.end_of_frame)
         {
            frame_ptr->sdata.flags.end_of_frame = FALSE;
            out_sdata_ptr->flags.end_of_frame   = TRUE;
            AR_MSG_ISLAND(DBG_HIGH_PRIO, "EOF was propagated to output");
         }
      }
   }
   else
   {
      // Otherwise, propagate EOF if input was fully consumed.
      if (is_input_fully_consumed)
      {
         if (frame_ptr->sdata.flags.end_of_frame)
         {
            frame_ptr->sdata.flags.end_of_frame = FALSE;
            out_sdata_ptr->flags.end_of_frame   = TRUE;
            AR_MSG_ISLAND(DBG_HIGH_PRIO, "EOF was propagated to output");
         }
      }
   }

   return CAPI_EOK;
}
