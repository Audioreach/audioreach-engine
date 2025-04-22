/**
 * \file spf_circular_buffer_island.c
 * \brief
 *    This file contains implementation of circular buffering
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/*==============================================================================
   Headers and Includes.
==============================================================================*/

#include "spf_circular_buffer_i.h"

/*==============================================================================
   Public Function Implementation
==============================================================================*/

spf_circ_buf_result_t spf_circ_buf_read_one_frame(spf_circ_buf_client_t *rd_client_ptr,
                                                  capi_stream_data_t *   out_sdata_ptr)
{
   return _circ_buf_read_one_frame(rd_client_ptr, out_sdata_ptr);
}

/*
 * Read one container frame from the circular buffer.
 */
spf_circ_buf_result_t _circ_buf_read_one_frame(spf_circ_buf_client_t *rd_client_ptr, capi_stream_data_t *out_sdata_ptr)
{
   spf_circ_buf_result_t res = SPF_CIRCBUF_SUCCESS;
   if ((NULL == rd_client_ptr->rw_pos.chunk_node_ptr) || (NULL == rd_client_ptr->circ_buf_ptr) ||
       (NULL == out_sdata_ptr))
   {
#ifdef DEBUG_CIRC_BUF_UTILS
      AR_MSG(DBG_ERROR_PRIO, "read: Failed. Invalid input params");
#endif
      return SPF_CIRCBUF_FAIL;
   }
   capi_buf_t *          out_ptr          = out_sdata_ptr->buf_ptr;
   spf_circ_buf_chunk_t *cur_rd_chunk_ptr = (spf_circ_buf_chunk_t *)rd_client_ptr->rw_pos.chunk_node_ptr->obj_ptr;
   if (NULL == out_ptr->data_ptr)
   {
#ifdef DEBUG_CIRC_BUF_UTILS
      AR_MSG(DBG_HIGH_PRIO,
             "read: buf_id: 0x%lx NULL read buffers received, nothing to read",
             rd_client_ptr->circ_buf_ptr->id);
#endif
      return res;
   }

   // cache bytes requested to read
   uint32_t bytes_req_to_read = out_ptr[DEFAULT_CH_IDX].max_data_len;

   // reset output sdata before reading data
   out_ptr[DEFAULT_CH_IDX].actual_data_len  = 0;
   out_sdata_ptr->flags.word                = 0;
   out_sdata_ptr->flags.stream_data_version = 1;
   out_sdata_ptr->timestamp                 = 0;

#ifdef DEBUG_CIRC_BUF_UTILS
   AR_MSG(DBG_HIGH_PRIO, "read: bytes_to_read= %u ; unread_bytes = %u", bytes_req_to_read, rd_client_ptr->unread_bytes);
#endif

   // If there is not data in the current frame, try to copy only the metadata and release it.
   // If output sdata pointer is non NULL and the current frame has only metadata its possible to propagate only
   // metadata.
   if (rd_client_ptr->unread_bytes == 0)
   {
      // IF sdata is non NULL, we can just possibly output only metadata
      spf_circ_buf_frame_t *cur_ch0_rd_frm_ptr = get_frame_ptr(&rd_client_ptr->rw_pos, DEFAULT_CH_IDX);

#ifdef DEBUG_CIRC_BUF_UTILS
      AR_MSG(DBG_HIGH_PRIO,
             "read: buf_id: 0x%lx possible underrun. cur_frame_ptr: 0x%lx actual_data_len: %lu , metadata_list: 0x%lx",
             rd_client_ptr->circ_buf_ptr->id,
             cur_ch0_rd_frm_ptr,
             cur_ch0_rd_frm_ptr->actual_data_len,
             cur_ch0_rd_frm_ptr->sdata.metadata_list_ptr);
#endif

      if (cur_ch0_rd_frm_ptr->actual_data_len == 0 && cur_ch0_rd_frm_ptr->sdata.metadata_list_ptr)
      {
#ifdef DEBUG_CIRC_BUF_UTILS
         AR_MSG(DBG_HIGH_PRIO, "read: buf_id: 0x%lx Propagating only metadata.", rd_client_ptr->circ_buf_ptr->id);
#endif

         // Clone sdata from the new read frame to readers handle
         _circ_buf_clone_sdata_from_cur_frame_to_reader_hdl(cur_ch0_rd_frm_ptr, rd_client_ptr, (out_sdata_ptr != NULL));

         // TODO: Propagate metdata from buffer frame to output frame.
         res |= _circular_buffer_propagate_metadata_to_output_stream(rd_client_ptr, out_ptr, out_sdata_ptr, 0, 0, 0, 0);

         // TODO: get stream associated metadata in the cur frame and copy it to next frame position
         //       in the read handle. Currently freeing any metadata left in the current reader pos,
         //       "ideally this should not happen."
         _circ_buf_free_sdata(rd_client_ptr->circ_buf_ptr,
                              &rd_client_ptr->rw_pos.readers_sdata,
                              TRUE /* force_free*/,
                              NULL /* return list of stream associated metadata */);

         /* Advance to the next frame */
         res |= _circ_buf_advance_to_next_frame(rd_client_ptr->circ_buf_ptr, &rd_client_ptr->rw_pos);
         return res;
      }
      else // if the current read frame doesnt have metadata also underrun.
      {
#ifdef DEBUG_CIRC_BUF_UTILS
         AR_MSG(DBG_HIGH_PRIO,
                "read: buf_id: 0x%lx current frame doesnt have metadata, underrun detected.",
                rd_client_ptr->circ_buf_ptr->id);
#endif
         return SPF_CIRCBUF_UNDERRUN;
      }
   }

   /* Check if the current chunk has different mf from the readers operating media format. */
   bool_t did_mf_change = _circ_buf_check_and_handle_change_in_mf(rd_client_ptr, cur_rd_chunk_ptr->mf);
   if (TRUE == did_mf_change)
   {
#ifdef DEBUG_CIRC_BUF_UTILS
      AR_MSG(DBG_HIGH_PRIO, "read: mf changed. not reading from the current frame");
#endif
      return res;
   }

   if (bytes_req_to_read > rd_client_ptr->unread_bytes)
   {
#ifdef DEBUG_CIRC_BUF_UTILS
      AR_MSG(DBG_HIGH_PRIO,
             "read: warning: circ buf possible underrun in next read, bytes to read = %u, unread bytes = %u",
             bytes_req_to_read,
             rd_client_ptr->unread_bytes);
#endif
      bytes_req_to_read = rd_client_ptr->unread_bytes;
   }

   /************* LOOP to read data from circular buffer **************/
   uint32_t bytes_left_to_read = bytes_req_to_read;
   while (bytes_left_to_read > 0)
   {
      spf_circ_buf_frame_t *cur_ch0_frame_ptr = get_frame_ptr(&rd_client_ptr->rw_pos, DEFAULT_CH_IDX);

#ifdef DEBUG_CIRC_BUF_UTILS
      AR_MSG(DBG_HIGH_PRIO,
             "read_loop: buf_id: 0x%lx, cur_frame_ptr: 0x%x, frame_actual_data_len: %lu cur_frame_offset: %lu, "
             "bytes_left_to_read: %lu",
             rd_client_ptr->circ_buf_ptr->id,
             cur_ch0_frame_ptr,
             cur_ch0_frame_ptr->actual_data_len,
             rd_client_ptr->rw_pos.frame_offset,
             bytes_left_to_read);
#endif

#ifdef DEBUG_CIRC_BUF_UTILS_VERBOSE
      AR_MSG(DBG_HIGH_PRIO,
             "read_loop: buf_id: 0x%lx, chunk_id: 0x%lx, chunk_ptr: 0x%x, frame_position: %lu chunk_size: %lu",
             rd_client_ptr->circ_buf_ptr->id,
             cur_rd_chunk_ptr->id,
             cur_rd_chunk_ptr,
             rd_client_ptr->rw_pos.frame_position,
             cur_rd_chunk_ptr->size);
#endif

      /* Check if the current chunk has different mf from the readers operating media format. */
      if (rd_client_ptr->operating_mf != cur_rd_chunk_ptr->mf)
      {
#ifdef DEBUG_CIRC_BUF_UTILS
         AR_MSG(DBG_HIGH_PRIO,
                "read: mf changed, not reading data further from the buffer. bytes_left_to_read: %lu",
                bytes_left_to_read);
#endif
         return res;
      }

      // if there is no data/metadata to read in the current frame move to the next frame.
      if (NULL == cur_ch0_frame_ptr->sdata.metadata_list_ptr && 0 == cur_ch0_frame_ptr->actual_data_len)
      {
#ifdef DEBUG_CIRC_BUF_UTILS_VERBOSE
         AR_MSG(DBG_HIGH_PRIO,
                "read_loop: buf_id: 0x%lx, Skipping read frame. No data/metadata buffered to read in this chunk_ptr: "
                "0x%x frame_position 0x%lx.",
                rd_client_ptr->circ_buf_ptr->id,
                rd_client_ptr->rw_pos.frame_position,
                cur_ch0_frame_ptr);
#endif
         _circ_buf_advance_to_next_frame(rd_client_ptr->circ_buf_ptr, &rd_client_ptr->rw_pos);
         continue;
      }

      /*
       *
       * ******* Get initial state of current read frame and output buffer ******************
       *
       */
      // output buffer state
      uint32_t initial_bytes_in_output = out_ptr->actual_data_len;

      // current read frame state
      uint32_t initial_read_frame_offset = rd_client_ptr->rw_pos.frame_offset;
      uint32_t len_consumed_from_frame   = 0;
      if (rd_client_ptr->rw_pos.frame_offset > cur_ch0_frame_ptr->actual_data_len)
      {
         return SPF_CIRCBUF_FAIL;
      }

      uint32_t initial_bytes_available_to_read =
         cur_ch0_frame_ptr->actual_data_len - rd_client_ptr->rw_pos.frame_offset;

      // if data is being read for the first time from the frame clone metadata from buffer frame to
      // reader's handle.
      if (initial_read_frame_offset == 0)
      {
         // Clone sdata from the new read frame to readers handle
         _circ_buf_clone_sdata_from_cur_frame_to_reader_hdl(cur_ch0_frame_ptr, rd_client_ptr, (out_sdata_ptr != NULL));
      }

      // check if the data can be further copied from buffer frame to output buffer.
      bool_t need_to_continue = _circ_buf_can_read_data_further(rd_client_ptr, out_sdata_ptr, initial_bytes_in_output);
      if (FALSE == need_to_continue)
      {

#ifdef DEBUG_CIRC_BUF_UTILS_VERBOSE
         AR_MSG(DBG_HIGH_PRIO,
                "read_loop: buf_id: 0x%lx, chunk_id: 0x%lx, chunk_ptr: 0x%x, frame_position:%lu  chunk_size: %lu",
                rd_client_ptr->circ_buf_ptr->id,
                cur_rd_chunk_ptr->id,
                cur_rd_chunk_ptr,
                rd_client_ptr->rw_pos.frame_position,
                cur_rd_chunk_ptr->size);
#endif
         break;
      }

      // compute leng of data consumed.
      if (bytes_left_to_read < initial_bytes_available_to_read) // reading partial frame
      {
         len_consumed_from_frame = bytes_left_to_read;
      }
      else
      {
         len_consumed_from_frame = initial_bytes_available_to_read;
      }

      /* Copy data from channel buffers to capi output stream sdata .*/
      spf_circ_buf_chunk_t *cur_chunk_ptr = (spf_circ_buf_chunk_t *)rd_client_ptr->rw_pos.chunk_node_ptr->obj_ptr;
      for (uint32_t ch_idx = 0; ch_idx < cur_chunk_ptr->num_channels; ch_idx++)
      {
         spf_circ_buf_frame_t *cur_ch_frame_ptr = get_frame_ptr(&rd_client_ptr->rw_pos, ch_idx);
         memscpy(&out_ptr[ch_idx].data_ptr[bytes_req_to_read - bytes_left_to_read],
                 len_consumed_from_frame,
                 &cur_ch_frame_ptr->data[rd_client_ptr->rw_pos.frame_offset],
                 len_consumed_from_frame);

         // updates bytes read till now.
         out_ptr[ch_idx].actual_data_len += len_consumed_from_frame;
      }

      rd_client_ptr->rw_pos.frame_offset += len_consumed_from_frame;

      bytes_left_to_read = bytes_left_to_read - len_consumed_from_frame;

      // update unread bytes from the reader client.
      rd_client_ptr->unread_bytes -= len_consumed_from_frame;

      // Propagate metdata from buffer frame to output frame.
      res |= _circular_buffer_propagate_metadata_to_output_stream(rd_client_ptr,
                                                                  out_ptr,
                                                                  out_sdata_ptr,
                                                                  initial_read_frame_offset,
                                                                  initial_bytes_available_to_read,
                                                                  len_consumed_from_frame,
                                                                  initial_bytes_in_output);

      // Free metadata if the frame is exhausted. Move to the next read frame. only for ch_idx == 0
      if (len_consumed_from_frame == initial_bytes_available_to_read)
      {
         // TODO: get stream associated metadata in the cur frame and copy it to next frame position
         //       in the read handle. Currently freeing any metadata left in the current reader pos,
         //       ideally this should not happen.

         _circ_buf_free_sdata(rd_client_ptr->circ_buf_ptr,
                              &rd_client_ptr->rw_pos.readers_sdata,
                              TRUE /* force_free*/,
                              NULL /* return list of stream associated metadata */);

         // Advance to the next frame
         res |= _circ_buf_advance_to_next_frame(rd_client_ptr->circ_buf_ptr, &rd_client_ptr->rw_pos);
      }

#ifdef DEBUG_CIRC_BUF_UTILS
      AR_MSG(DBG_HIGH_PRIO,
             "read_loop: buf_id: 0x%lx, next_frame_ptr: 0x%x, next_frame_offset: %lu, bytes_left_to_read: %lu unread_bytes: %lu",
             rd_client_ptr->circ_buf_ptr->id,
             cur_ch0_frame_ptr,
             rd_client_ptr->rw_pos.frame_offset,
             bytes_left_to_read,
             rd_client_ptr->unread_bytes);
#endif
   }

   return res;
}

/* Clones metadata from the current frame to reader handle.
 *
 * 1. Each reader needs to have reference to metadata its going to propagate to output.
 *
 * 2. Last reader need not clone, it can just move the metadata from buffer frame to reader handle.
 * */
spf_circ_buf_result_t _circ_buf_clone_sdata_from_cur_frame_to_reader_hdl(spf_circ_buf_frame_t * cur_rd_frame_ptr,
                                                                         spf_circ_buf_client_t *rd_client_ptr,
                                                                         bool_t                 is_output_sdata_v2)
{
   spf_circ_buf_result_t result       = SPF_CIRCBUF_SUCCESS;
   spf_circ_buf_t *      circ_buf_ptr = rd_client_ptr->circ_buf_ptr;

#ifdef DEBUG_CIRC_BUF_UTILS_METADATA
   AR_MSG(DBG_HIGH_PRIO,
          "clone_metadata: buf_id: 0x%lx cloning sdata from cur frame 0x%lx to reader 0x%lx ",
          circ_buf_ptr->id,
          cur_rd_frame_ptr,
          rd_client_ptr->id);
#endif

   /* increment the frame read ref counter,
    * Every reader touching the frame will increment the ref count.
    * This is used to free metadata when the last reader touches the frame.
    * Ref count will be reset when the writer writes new data.
    */
   cur_rd_frame_ptr->reader_ref_count++;

   /*
    * *********** Clone Flags and timestamp *******************
    */
   {

#ifdef DEBUG_CIRC_BUF_UTILS_METADATA
      AR_MSG(DBG_HIGH_PRIO,
             "clone_metadata: buf_id: 0x%lx Cloning flags 0x%lx timestamp %ld to reader handle 0x%lx ",
             circ_buf_ptr->id,
             cur_rd_frame_ptr->sdata.flags.word,
             cur_rd_frame_ptr->sdata.timestamp,
             rd_client_ptr->id);
#endif

      // Copy flags and timestamp of the current frame to readers handle.
      // TODO: check if this is correct.
      rd_client_ptr->rw_pos.readers_sdata.flags.word = cur_rd_frame_ptr->sdata.flags.word;

      // Copy flags and timestamp of the current frame to readers handle.
      rd_client_ptr->rw_pos.readers_sdata.timestamp = cur_rd_frame_ptr->sdata.timestamp;
   }

   /*
    * *********** Clone metadata list *******************
    */
   if (FALSE == is_output_sdata_v2)
   {
      // if output doesn't support sdata v2, no need to propagate metadata.

      return result;
   }

   // if current read frame doesnt have metadata do nothing.
   if (!cur_rd_frame_ptr->sdata.metadata_list_ptr || !circ_buf_ptr->metadata_handler)
   {
      return result;
   }

#ifdef DEBUG_CIRC_BUF_UTILS_METADATA
   print_frame_metadata(cur_rd_frame_ptr);
#endif

   /*    TODO: There may be a case where all the readers are done reading and metadata has propagated out from the
               module. If a new reader clients registers with circular buffer will not see metadata since its
               already propagated.

        Solution: metadata is freed only when the writer overflows. in that case metadata is consistent to the
        buffered data. But metadata will get stuck until writer overwrites, even after all reads have finished reading.
    */

   // Iterate through the metadata list in the buffer frame and clone the nodes to readers handle.
   bool_t is_clone = FALSE;
   if (cur_rd_frame_ptr->reader_ref_count < circ_buf_ptr->num_read_clients)
   {
      is_clone = TRUE;
   }

   result = _circ_buf_clone_or_move_metadata_list(circ_buf_ptr,
                                                  &cur_rd_frame_ptr->sdata.metadata_list_ptr,
                                                  &rd_client_ptr->rw_pos.readers_sdata.metadata_list_ptr,
                                                  is_clone);

   return result;
}

spf_circ_buf_result_t _circ_buf_clone_or_move_metadata_list(spf_circ_buf_t *       circ_buf_ptr,
                                                            module_cmn_md_list_t **src_list_pptr,
                                                            module_cmn_md_list_t **dst_list_pptr,
                                                            bool_t                 is_clone)
{

   spf_circ_buf_result_t result = SPF_CIRCBUF_SUCCESS;

   module_cmn_md_list_t *cur_md_node_ptr = *src_list_pptr;
   while (cur_md_node_ptr)
   {
      module_cmn_md_t *md_ptr = (module_cmn_md_t *)cur_md_node_ptr->obj_ptr;

      if (MODULE_CMN_MD_ID_EOS == md_ptr->metadata_id)
      {

#ifdef DEBUG_CIRC_BUF_UTILS_METADATA
         AR_MSG(DBG_HIGH_PRIO, "clone_metadata: buf_id: 0x%lx Propagating out EOS  ", circ_buf_ptr->id);
#endif
      }
      else if (MODULE_CMN_MD_ID_DFG == md_ptr->metadata_id)
      {

#ifdef DEBUG_CIRC_BUF_UTILS_METADATA
         AR_MSG(DBG_HIGH_PRIO, "clone_metadata: buf_id: 0x%x Propagating out DFG", circ_buf_ptr->id);
#endif
      }

      // If last reader is reading the frame from circular buffer then no need to clone.
      // Only for last reader ref count is equal to num_read_clients.
      if (is_clone)
      {
#ifdef DEBUG_CIRC_BUF_UTILS_METADATA
         AR_MSG(DBG_HIGH_PRIO,
                "clone_metadata: buf_id: 0x%x Cloning  metadata_id: 0x%x",
                circ_buf_ptr->id,
                md_ptr->metadata_id);
#endif
         capi_heap_id_t heap_id = { .heap_id = (uint32_t)circ_buf_ptr->heap_id };
         capi_err_t     capi_res =
            circ_buf_ptr->metadata_handler->metadata_clone(circ_buf_ptr->metadata_handler->context_ptr,
                                                           md_ptr,
                                                           dst_list_pptr,
                                                           heap_id);
         if (CAPI_FAILED(capi_res))
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "clone_metadata: buf_id: 0x%x Failed cloning metadata 0x%lx",
                   circ_buf_ptr->id,
                   md_ptr->metadata_id);
            return SPF_CIRCBUF_FAIL;
         }
      }
      else // last reader just moves the list from one frame to reader's handle
      {
#ifdef DEBUG_CIRC_BUF_UTILS_METADATA
         AR_MSG(DBG_HIGH_PRIO,
                "clone_metadata: buf_id: 0x%x Moving metadata 0x%lx",
                circ_buf_ptr->id,
                md_ptr->metadata_id);
#endif
         // for the last reader just move metadata node from circular buffer to reader client handle.
         spf_list_move_node_to_another_list((spf_list_node_t **)dst_list_pptr,
                                            (spf_list_node_t *)cur_md_node_ptr,
                                            (spf_list_node_t **)src_list_pptr);

         // when a cur_node_ptr is moved it becomes invalid, so init again to the head of the list.
         cur_md_node_ptr = *src_list_pptr;
         continue;
      }

      // advance to the next metadata list node
      LIST_ADVANCE(cur_md_node_ptr);
   }

   return result;
}

/* Propagates metadata from circular buffer to output sdata.
 * 1. initial_bytes_in_output
 *
 * */
spf_circ_buf_result_t _circular_buffer_propagate_metadata_to_output_stream(spf_circ_buf_client_t *rd_client_ptr,
                                                                           capi_buf_t *           out_buf_ptr,
                                                                           capi_stream_data_t *   out_sdata_ptr,
                                                                           uint32_t initial_read_frame_offset,
                                                                           uint32_t initial_bytes_available_to_read,
                                                                           uint32_t len_consumed_from_frame,
                                                                           uint32_t initial_bytes_in_output)
{

   spf_circ_buf_t *      circ_buf_ptr = rd_client_ptr->circ_buf_ptr;
   spf_circ_buf_chunk_t *rd_chunk_ptr = (spf_circ_buf_chunk_t *)rd_client_ptr->rw_pos.chunk_node_ptr->obj_ptr;

#ifdef DEBUG_CIRC_BUF_UTILS_METADATA
   AR_MSG(DBG_HIGH_PRIO,
          "propagate_metadata: entered output metadata propagation buf_id: 0x%lx ",
          rd_client_ptr->circ_buf_ptr->id);
#endif

   // if the read client doesn't pass sdata, do nothing.
   if (NULL == out_sdata_ptr)
   {
      return SPF_CIRCBUF_SUCCESS;
   }

   // if the output sdata is not stream v2, metadata propagation is not supported.
   bool_t is_output_sdata_v2 = FALSE;
   if ((CAPI_STREAM_V2 == out_sdata_ptr->flags.stream_data_version))
   {
      is_output_sdata_v2 = TRUE;
   }

   { // begin code block: flag propagation

      // Scenarios to propagate flags from a read frame.
      // In all the scenarios, if the output already has data its assumed that flags in read frame and output sdata
      // to be same. If they are not same, read() would have returned earlier & it wouldn't have reached
      // propagation stage.
      //
      // Currently only, is_timestamp_valid, marker_EOS, EOF and Erasure flags need to be propagated.
      //
      //  Case 1: Read entire frame from beginning to end.
      //       -> copy all the flags to output, if output already has data also we can copy. Since flags are same.
      //
      //  Case 2: Read data from a read_offset to end of the frame.
      //       -> copy all the flags to output, if output already has data also we can copy. Since flags are same.
      //
      //  Case 3: Read partial data from beginning of the frame.
      //       -> Propagate only flags corresponding to partial data i.e only Erasure/marker EOS.
      //       -> Marker EOS will propagated by metadata_propagate()
      //       -> EOF is associated with last sample, so it will not be propagated now.
      //
      //  Case 4: Read partial data from a read_offset.
      //       -> Propagate only flags corresponding to partial data i.e only Erasure/marker EOS.
      //       -> Marker EOS will propagated by metadata_propagate()
      //       -> EOF is not propagated
      //
      //  Case 5: actual_data_len of current frame is 'zero'
      //       -> Propagate all flags and metadata from frame to o/p

      // Timestamp is updated only when the data is written first time into capi's output buffer.
      if (0 == initial_bytes_in_output)
      {
         // Update the output buffer timestamp based on the current read offset.
         if (rd_client_ptr->rw_pos.readers_sdata.flags.is_timestamp_valid)
         {
            out_sdata_ptr->timestamp = rd_client_ptr->rw_pos.readers_sdata.timestamp +
                                       capi_cmn_bytes_to_us(initial_read_frame_offset,
                                                            rd_chunk_ptr->mf->sampling_rate,
                                                            rd_chunk_ptr->mf->bits_per_sample,
                                                            NUM_CH_PER_CIRC_BUF /* Num channels per circular buffer */,
                                                            NULL);

            out_sdata_ptr->flags.is_timestamp_valid = TRUE;
         }
      }
      else // if there was data in the o/p
      {
         // in this case, flags in the o/p and current read frame are expected to be same before reading data.
         // And also timestamp is retained as it is, no need to check for TS because frame work already checks
      }

#ifdef DEBUG_CIRC_BUF_UTILS_METADATA
      AR_MSG(DBG_HIGH_PRIO,
             "propagate_metadata: buf_id: 0x%lx Frame is exhausted. out sdata flags 0x%lx, output ts: %lu, frame "
             "flags 0x%lx init_bytes_op: %lu init_bytes_avail_to_read: %lu, initial_read_frame_offset:%lu bytes_read: "
             "%lu ",
             rd_client_ptr->circ_buf_ptr->id,
             out_sdata_ptr->flags.word,
             out_sdata_ptr->timestamp,
             rd_client_ptr->rw_pos.readers_sdata.flags.word,
             initial_bytes_in_output,
             initial_bytes_available_to_read,
             initial_read_frame_offset,
             len_consumed_from_frame);
#endif

      if (len_consumed_from_frame == initial_bytes_available_to_read)
      {

         // Handles case 1,2,5 -> propagate all the flags from frame to o/p.
         out_sdata_ptr->flags.word                      = rd_client_ptr->rw_pos.readers_sdata.flags.word;
         rd_client_ptr->rw_pos.readers_sdata.flags.word = 0;
      }
      else // partial read
      {
         // Handles case 3 & 4
         // Copy only erasure
         out_sdata_ptr->flags.erasure = rd_client_ptr->rw_pos.readers_sdata.flags.erasure;
      }

      // reset stream data version as it was earlier.
      out_sdata_ptr->flags.stream_data_version = is_output_sdata_v2;

   } // end code block: flag propagation

   // if the current reader clients has metadata associated with the current frame propagate it to the output.
   if (is_output_sdata_v2 && rd_client_ptr->rw_pos.readers_sdata.metadata_list_ptr)
   {

#ifdef DEBUG_CIRC_BUF_UTILS_METADATA
      print_frame_metadata_list(&rd_client_ptr->rw_pos.readers_sdata);
#endif

      capi_stream_data_v2_t *    out_sdata_v2_ptr = (capi_stream_data_v2_t *)out_sdata_ptr;
      uint32_t                   ALGO_DELAY_ZERO  = 0; // Sync has no algo delay.
      intf_extn_md_propagation_t input_md_info;
      memset(&input_md_info, 0, sizeof(input_md_info));
      input_md_info.df                          = CAPI_FIXED_POINT;
      input_md_info.len_per_ch_in_bytes         = len_consumed_from_frame;
      input_md_info.initial_len_per_ch_in_bytes = initial_bytes_available_to_read;
      input_md_info.buf_delay_per_ch_in_bytes   = 0;
      input_md_info.bits_per_sample             = rd_chunk_ptr->mf->bits_per_sample;
      input_md_info.sample_rate                 = rd_chunk_ptr->mf->sampling_rate;

      intf_extn_md_propagation_t output_md_info;
      memscpy(&output_md_info, sizeof(output_md_info), &input_md_info, sizeof(input_md_info));
      output_md_info.len_per_ch_in_bytes         = len_consumed_from_frame;
      output_md_info.initial_len_per_ch_in_bytes = initial_bytes_in_output; // check if its handled.
      output_md_info.buf_delay_per_ch_in_bytes   = 0;

      // propagate metadata from reader frame handle to output stream data pointer.
      capi_err_t capi_res =
         circ_buf_ptr->metadata_handler->metadata_propagate(circ_buf_ptr->metadata_handler->context_ptr,
                                                            &rd_client_ptr->rw_pos.readers_sdata, // circ buf frame
                                                                                                  // sdata
                                                            out_sdata_v2_ptr,                     // capi sdata v2 ptr
                                                            NULL,
                                                            ALGO_DELAY_ZERO,
                                                            &input_md_info,
                                                            &output_md_info);
      if (CAPI_FAILED(capi_res))
      {
         AR_MSG(DBG_ERROR_PRIO,
                "propagate_metadata: buf_id: 0x%x Reader 0x%x failed in propagating metadata to output.",
                rd_client_ptr->circ_buf_ptr->id,
                rd_client_ptr->id);
         return SPF_CIRCBUF_FAIL;
      }

      // Adjust the remaining metadata offsets based on the data consumed
      module_cmn_md_list_t *list_ptr = rd_client_ptr->rw_pos.readers_sdata.metadata_list_ptr;
      while (list_ptr)
      {
         module_cmn_md_t *md_ptr = (module_cmn_md_t *)list_ptr->obj_ptr;

         // Decrement remaining metadata offset w.r.t to new reader's offset position.
         md_ptr->offset = md_ptr->offset - (len_consumed_from_frame / (rd_chunk_ptr->mf->bits_per_sample / 8));

         LIST_ADVANCE(list_ptr);
      }
   }

#ifdef DEBUG_CIRC_BUF_UTILS_METADATA
   // print remaining metadata in the readers frame
   print_frame_metadata_list(&rd_client_ptr->rw_pos.readers_sdata);
#endif

   return SPF_CIRCBUF_SUCCESS;
}

/* Frees metadata from the given frame of a chunk. */
spf_circ_buf_result_t _circ_buf_free_sdata(spf_circ_buf_t *       circ_buf_ptr,
                                           capi_stream_data_v2_t *sdata_ptr,
                                           bool_t                 force_free,
                                           module_cmn_md_list_t **stream_associated_md_list_pptr)
{
   spf_circ_buf_result_t res = SPF_CIRCBUF_SUCCESS;

   // metadata overrun needs to be printed as a warning.
   if (sdata_ptr->metadata_list_ptr && circ_buf_ptr->metadata_handler)
   {
      // TODO: Destroy only sample associated and Buffer associated metadata. Stream associated metadata must
      //       be carried forward for overrun. Currently force_free is set TRUE for overrun cases as well.

      module_cmn_md_list_t *cur_node_ptr = sdata_ptr->metadata_list_ptr;
      while (cur_node_ptr)
      {
         // get current metadata node object.
         module_cmn_md_t *md_ptr = (module_cmn_md_t *)cur_node_ptr->obj_ptr;

         // Check if metadata can be destoryed or not
         bool_t is_stream_associated = FALSE;
         bool_t need_to_be_dropped   = FALSE; /* by default metadata is rendered during destroy. */
         if (MODULE_CMN_MD_ID_EOS == md_ptr->metadata_id)
         {

#ifdef DEBUG_CIRC_BUF_UTILS_METADATA
            AR_MSG(DBG_HIGH_PRIO, "free_metadata: buf_id: 0x%x EOS found.", circ_buf_ptr->id);
#endif
            is_stream_associated = TRUE;
         }
         else
         {
            AR_MSG(DBG_HIGH_PRIO,
                   "free_metadata: buf_id: 0x%x Unknown metadata found dropping it. metadata_id:  0x%x ",
                   circ_buf_ptr->id,
                   md_ptr->metadata_id);
            need_to_be_dropped = TRUE;
         }

         // Destroyed if,
         //   1. Force free is true.
         //        OR
         //   2. If its NOT stream associated metadata.
         if (force_free || !is_stream_associated)
         {
            capi_err_t capi_res =
               circ_buf_ptr->metadata_handler->metadata_destroy(circ_buf_ptr->metadata_handler->context_ptr,
                                                                cur_node_ptr,
                                                                need_to_be_dropped, // dropping metadata
                                                                &sdata_ptr->metadata_list_ptr);
            if (CAPI_FAILED(capi_res))
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "free_metadata: buf_id: 0x%x Failed in destroyig metadata 0x%lx",
                      circ_buf_ptr->id,
                      md_ptr->metadata_id);
            }

            // when a node is destoyed init again to the head of the list.
            cur_node_ptr = sdata_ptr->metadata_list_ptr;
            continue;
         }

         // Retained if,
         //  1. If its IS stream associated metadata.
         //  			&&
         //  2. only if force is False.
         if (!force_free && is_stream_associated)
         {
            if (stream_associated_md_list_pptr)
            {
               // for the last reader just move metadata node from circular buffer to reader client handle.
               spf_list_move_node_to_another_list((spf_list_node_t **)stream_associated_md_list_pptr,
                                                  (spf_list_node_t *)cur_node_ptr,
                                                  (spf_list_node_t **)&sdata_ptr->metadata_list_ptr);
            }

            // when a node is destroyed init again to the head of the list.
            cur_node_ptr = sdata_ptr->metadata_list_ptr;
            continue;
         }

         LIST_ADVANCE(cur_node_ptr);
      }
   }

   // memset frame stream header.
   memset(sdata_ptr, 0, sizeof(capi_stream_data_v2_t));

   return res;
}

// advances to next frame in the circular buffer.
//    1. If the current position is in last frame of the chunk, moves to the first frame of next chunk.
//    2. IF the next frame is in same chunk, just updates the chunk frame position.
spf_circ_buf_result_t _circ_buf_advance_to_next_frame(spf_circ_buf_t *circ_buf_ptr, spf_circ_buf_position_t *pos)
{
   if ((NULL == circ_buf_ptr) || (NULL == pos))
   {
      return SPF_CIRCBUF_FAIL;
   }
   spf_circ_buf_chunk_t *prev_chunk_ptr = (spf_circ_buf_chunk_t *)pos->chunk_node_ptr->obj_ptr;

   uint32_t next_frame_position = pos->frame_position + GET_CHUNK_FRAME_SIZE(prev_chunk_ptr->frame_size);

#ifdef DEBUG_CIRC_BUF_UTILS_VERBOSE
   AR_MSG(DBG_HIGH_PRIO,
          "adv_next_frame: buf_id: 0x%x, cur_chunk_ptr: 0x%x, chunk_size: %lu "
          "next_frame_position: %lu, ",
          circ_buf_ptr->id,
          prev_chunk_ptr,
          prev_chunk_ptr->size,
          next_frame_position);
#endif

   // If the next frame's end address is beyond chunk size it becomes invalid
   // In that case advance to the next frame in the next chunk.
   // Else, move to the next frame in the same chunk.
   uint32_t next_frame_position_end = next_frame_position + GET_CHUNK_FRAME_SIZE(prev_chunk_ptr->frame_size);
   if (next_frame_position_end > prev_chunk_ptr->size)
   {
      // Move to the next chunk in the list if it exist.
      // If the next pointer doesn't exist, it means the node is the tail so assign the head as the chunk pointer
      spf_circ_buf_result_t res = _circ_buf_next_chunk_node(circ_buf_ptr, &pos->chunk_node_ptr);
      if (res != SPF_CIRCBUF_SUCCESS)
      {
         return SPF_CIRCBUF_FAIL;
      }

      pos->frame_position = 0;
      pos->frame_offset   = 0;
   }
   else // if the next frame addr is valid, return is as the next frame pointer,
   {
      // move to the next frame in the same chunk
      pos->frame_position = next_frame_position;
      pos->frame_offset   = 0;
   }

#ifdef DEBUG_CIRC_BUF_UTILS_VERBOSE
   spf_circ_buf_chunk_t *temp_chunk_ptr = (spf_circ_buf_chunk_t *)pos->chunk_node_ptr->obj_ptr;
   AR_MSG(DBG_HIGH_PRIO,
          "adv_next_frame: new chunk_ptr: 0x%x, frame_position: %lu, frame_offset: %lu ",
          temp_chunk_ptr,
          pos->frame_position,
          pos->frame_offset);
#endif

   return SPF_CIRCBUF_SUCCESS;
}

spf_circ_buf_result_t _circ_buf_next_chunk_node(spf_circ_buf_t *circ_buf_ptr, spf_list_node_t **chunk_ptr_ptr)
{
   if ((NULL == chunk_ptr_ptr) || (NULL == *chunk_ptr_ptr) || (NULL == circ_buf_ptr))
   {
      return SPF_CIRCBUF_FAIL;
   }

   // Move to the next chunk in the list if it exist. If the next pointer doesnt exist, it means the node is the tail
   // so assign the head as the chunk pointer

   if ((*chunk_ptr_ptr)->next_ptr)
   {
      (*chunk_ptr_ptr) = (*chunk_ptr_ptr)->next_ptr;
   }
   else
   {
      (*chunk_ptr_ptr) = circ_buf_ptr->head_chunk_ptr;
   }

   return SPF_CIRCBUF_SUCCESS;
}

/* Can be checked before reading data */
bool_t _circ_buf_check_and_handle_change_in_mf(spf_circ_buf_client_t *rd_client_ptr, spf_circ_buf_mf_info_t *new_mf)
{
   spf_circ_buf_t *circ_buf_ptr  = rd_client_ptr->circ_buf_ptr;
   bool_t          did_mf_change = FALSE;

   // check if the readers operating mf is different from chunk media format.
   if (rd_client_ptr->operating_mf != new_mf)
   {

#ifdef DEBUG_CIRC_BUF_UTILS
      AR_MSG(DBG_HIGH_PRIO,
             "media_fmt_changed: buf_id: 0x%lx, detected chg in media format. prev_mf: 0x%lx, "
             "new_mf: 0x%lx",
             rd_client_ptr->circ_buf_ptr->id,
             rd_client_ptr->operating_mf,
             new_mf);
#endif
      // Remove the previos operation mf handle if any exists and reassign
      // mf based on the current chunk position.
      if (rd_client_ptr->operating_mf)
      {
         decr_mf_ref_count(circ_buf_ptr, &rd_client_ptr->operating_mf);
      }

      // incerement reader mf referene count
      rd_client_ptr->operating_mf = new_mf;
      incr_mf_ref_count(circ_buf_ptr, rd_client_ptr->operating_mf);

      // Raise output media format event since there is a change in mf
      if (rd_client_ptr->operating_mf && circ_buf_ptr->cb_info.event_cb)
      {
         capi_media_fmt_v2_t mf;
         mf.header.format_header.data_format = CAPI_FIXED_POINT;

         mf.format.bitstream_format  = MEDIA_FMT_ID_PCM;
         mf.format.bits_per_sample   = rd_client_ptr->operating_mf->bits_per_sample;
         mf.format.data_is_signed    = rd_client_ptr->operating_mf->data_is_signed;
         mf.format.data_interleaving = CAPI_DEINTERLEAVED_UNPACKED;
         mf.format.num_channels      = rd_client_ptr->operating_mf->num_channels;
         mf.format.q_factor          = rd_client_ptr->operating_mf->q_factor;
         mf.format.sampling_rate     = rd_client_ptr->operating_mf->sampling_rate;

         for (uint32_t ch_idx = 0; ch_idx < rd_client_ptr->operating_mf->num_channels; ch_idx++)
         {
            mf.channel_type[ch_idx] = rd_client_ptr->operating_mf->channel_type[ch_idx];
         }

         circ_buf_ptr->cb_info.event_cb(circ_buf_ptr->cb_info.event_context,
                                        circ_buf_ptr,
                                        SPF_CIRC_BUF_EVENT_ID_OUTPUT_MEDIA_FORMAT,
                                        (void *)&mf);
      }

      did_mf_change = TRUE;
      return did_mf_change;
   }

   return did_mf_change;
}

/* Checks if the data can be read from the current frame to the capi output buffer, this function gets called in the
 * loop so it can handle so,
 *		1. If the output sdata has already end_of_frame/marker_eos set we cannot read data from the buffer anymore.
 *		2.
 * */
bool_t _circ_buf_can_read_data_further(spf_circ_buf_client_t *rd_client_ptr,
                                       capi_stream_data_t *   out_sdata_ptr,
                                       uint32_t               initial_bytes_in_output)
{
   bool_t can_read_further = TRUE;

   if (out_sdata_ptr)
   {
      // check if any flags are set on the output sdata.
      if (out_sdata_ptr->flags.end_of_frame || out_sdata_ptr->flags.marker_eos)
      {
         can_read_further &= FALSE;
      }

      // if there is already data in output, then check if the current read frame has same flags as output.
      if (initial_bytes_in_output && (out_sdata_ptr->flags.word != rd_client_ptr->rw_pos.readers_sdata.flags.word))
      {
         can_read_further &= FALSE;
      }
   }

   return can_read_further;
}

spf_circ_buf_result_t spf_circ_buf_driver_is_buffer_empty(spf_circ_buf_client_t *rd_hdl_ptr, bool_t *is_empty_ptr)
{
   bool_t is_empty = FALSE;
   if (!rd_hdl_ptr || !is_empty_ptr || !(rd_hdl_ptr->circ_buf_ptr))
   {
      return SPF_CIRCBUF_FAIL;
   }

   spf_circ_buf_client_t *wr_hdl_ptr = rd_hdl_ptr->circ_buf_ptr->wr_client_ptr;

   if (!wr_hdl_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "is_buffer_empty not yet implemented for no write ptr case.");

      // TODO: This function currently doesn't work if there's no writers (case of only metadata buffered). For this
      // we need to keep track of unread_frames.
      return SPF_CIRCBUF_FAIL;
   }

   if (wr_hdl_ptr->rw_pos.chunk_node_ptr == rd_hdl_ptr->rw_pos.chunk_node_ptr &&
       wr_hdl_ptr->rw_pos.frame_position == rd_hdl_ptr->rw_pos.frame_position)
   {
      // Read/Write pointers will be equal if the buffer is empty. If the buffer is empty, unread bytes is 0 (otherwise
      // buffer is full).
      is_empty = (0 == rd_hdl_ptr->unread_bytes);
   }

   *is_empty_ptr = is_empty;
   return SPF_CIRCBUF_SUCCESS;
}

/* Read one MTU frame from buffer into the ouput at a time - currently used */
spf_circ_buf_result_t spf_circ_buf_raw_read_one_frame(spf_circ_buf_raw_client_t *rd_client_ptr,
                                                      capi_stream_data_t *       out_sdata_ptr)
{
   return _circ_buf_raw_read_one_frame(rd_client_ptr, out_sdata_ptr);
}

/* Read one raw MTU frame ata at time to the output buffer */
spf_circ_buf_result_t _circ_buf_raw_read_one_frame(spf_circ_buf_raw_client_t *rd_client_ptr,
                                                   capi_stream_data_t *       out_sdata_ptr)
{
   spf_circ_buf_result_t res = SPF_CIRCBUF_SUCCESS;
   if ((NULL == rd_client_ptr->rw_pos.chunk_node_ptr) || (NULL == rd_client_ptr->circ_buf_raw_ptr) ||
       (NULL == out_sdata_ptr))
   {
#ifdef DEBUG_CIRC_BUF_UTILS
      AR_MSG(DBG_ERROR_PRIO, "read: Failed. Invalid input params");
#endif
      return SPF_CIRCBUF_FAIL;
   }
   capi_buf_t *out_ptr = out_sdata_ptr->buf_ptr;

#ifdef DEBUG_CIRC_BUF_UTILS_VERBOSE
   spf_circ_buf_chunk_t *cur_rd_chunk_ptr = (spf_circ_buf_chunk_t *)rd_client_ptr->rw_pos.chunk_node_ptr->obj_ptr;
#endif

   if (NULL == out_ptr->data_ptr)
   {
#ifdef DEBUG_CIRC_BUF_UTILS
      AR_MSG(DBG_HIGH_PRIO,
             "read: buf_id: 0x%lx NULL read buffers received, nothing to read",
             rd_client_ptr->circ_buf_raw_ptr->id);
#endif
      return res;
   }

   // reset output sdata before reading data
   out_ptr[DEFAULT_CH_IDX].actual_data_len  = 0;
   out_sdata_ptr->flags.word                = 0;
   out_sdata_ptr->flags.stream_data_version = 1;
   out_sdata_ptr->timestamp                 = 0;

   // If there is not data in the current frame, try to copy only the metadata and release it.
   // If output sdata pointer is non NULL and the current frame has only metadata its possible to propagate only
   // metadata.
   if (rd_client_ptr->unread_bytes == 0)
   {
      // IF sdata is non NULL, we can just possibly output only metadata
      spf_circ_buf_frame_t *cur_ch0_rd_frm_ptr = get_frame_ptr(&rd_client_ptr->rw_pos, DEFAULT_CH_IDX);

#ifdef DEBUG_CIRC_BUF_UTILS
      AR_MSG(DBG_HIGH_PRIO,
             "read: buf_id: 0x%lx possible underrun. cur_frame_ptr: 0x%lx actual_data_len: %lu , "
             "metadata_list: 0x%lx",
             rd_client_ptr->circ_buf_raw_ptr->id,
             cur_ch0_rd_frm_ptr,
             cur_ch0_rd_frm_ptr->actual_data_len,
             cur_ch0_rd_frm_ptr->sdata.metadata_list_ptr);
#endif

      if (cur_ch0_rd_frm_ptr->actual_data_len == 0 && cur_ch0_rd_frm_ptr->sdata.metadata_list_ptr)
      {
#ifdef DEBUG_CIRC_BUF_UTILS
         AR_MSG(DBG_HIGH_PRIO, "read: buf_id: 0x%lx Propagating only metadata.", rd_client_ptr->circ_buf_raw_ptr->id);
#endif

         // Clone sdata from the new read frame to readers handle
         _circ_buf_raw_clone_sdata_from_cur_frame_to_reader_hdl(cur_ch0_rd_frm_ptr,
                                                                rd_client_ptr,
                                                                (out_sdata_ptr != NULL));

         // TODO: Propagate metdata from buffer frame to output frame.
         res |=
            _circular_buffer_raw_propagate_metadata_to_output_stream(rd_client_ptr, out_ptr, out_sdata_ptr, 0, 0, 0, 0);

         // TODO: get stream associated metadata in the cur frame and copy it to next frame position
         //       in the read handle. Currently freeing any metadata left in the current reader pos,
         //       "ideally this should not happen."
         _circ_buf_raw_free_sdata(rd_client_ptr->circ_buf_raw_ptr,
                                  &rd_client_ptr->rw_pos.readers_sdata,
                                  TRUE /* force_free*/,
                                  NULL /* return list of stream associated metadata */);

         /* Advance to the next frame */
         res |= _circ_buf_raw_advance_to_next_frame(rd_client_ptr->circ_buf_raw_ptr, &rd_client_ptr->rw_pos);
         return res;
      }
      else // if the current read frame doesnt have metadata also underrun.
      {
#ifdef DEBUG_CIRC_BUF_UTILS
         AR_MSG(DBG_HIGH_PRIO,
                "read: buf_id: 0x%lx current frame doesnt have metadata, underrun detected.",
                rd_client_ptr->circ_buf_raw_ptr->id);
#endif
         return SPF_CIRCBUF_UNDERRUN;
      }
   }

   /************* For raw compressed read one frame of actual data len per read**************/

   spf_circ_buf_frame_t *cur_ch0_frame_ptr = get_frame_ptr(&rd_client_ptr->rw_pos, DEFAULT_CH_IDX);
#ifdef DEBUG_CIRC_BUF_UTILS
   AR_MSG(DBG_HIGH_PRIO,
          "read_loop: buf_id: 0x%lx, cur_frame_ptr: 0x%x, frame_actual_data_len: %lu "
          "cur_frame_offset: %lu, unread bytes = %u",
          rd_client_ptr->circ_buf_raw_ptr->id,
          cur_ch0_frame_ptr,
          cur_ch0_frame_ptr->actual_data_len,
          rd_client_ptr->rw_pos.frame_offset,
          rd_client_ptr->unread_bytes);

   AR_MSG(DBG_HIGH_PRIO,
          "read_loop: Unread bytes updated to unread bytes = %u / %u = %u and cur buf #"
          " frame = %u",
          rd_client_ptr->unread_bytes,
          rd_client_ptr->unread_bytes_max,
          rd_client_ptr->unread_num_frames,
          cur_ch0_frame_ptr->num_encoded_frames_in_cur_buf);

#endif

#ifdef DEBUG_CIRC_BUF_UTILS_VERBOSE
   AR_MSG(DBG_HIGH_PRIO,
          "read_loop: buf_id: 0x%lx, chunk_id: 0x%lx, chunk_ptr: 0x%x, frame_position: %lu "
          "chunk_size: %lu",
          rd_client_ptr->circ_buf_raw_ptr->id,
          cur_rd_chunk_ptr->id,
          cur_rd_chunk_ptr,
          rd_client_ptr->rw_pos.frame_position,
          cur_rd_chunk_ptr->size);
#endif
   if (NULL == cur_ch0_frame_ptr->sdata.metadata_list_ptr && 0 == cur_ch0_frame_ptr->actual_data_len)
   {
#ifdef DEBUG_CIRC_BUF_UTILS_VERBOSE
      AR_MSG(DBG_HIGH_PRIO,
             "read_loop: buf_id: 0x%lx, Skipping read frame. No data/metadata buffered to read in this chunk_ptr: "
             "0x%x frame_position 0x%lx.",
             rd_client_ptr->circ_buf_raw_ptr->id,
             rd_client_ptr->rw_pos.frame_position,
             cur_ch0_frame_ptr);
#endif
      _circ_buf_raw_advance_to_next_frame(rd_client_ptr->circ_buf_raw_ptr, &rd_client_ptr->rw_pos);
      // TODO: CB CHECK ONCE to return?
   }

   // output buffer state
   uint32_t initial_bytes_in_output = out_ptr->actual_data_len;

   // current read frame state
   uint32_t initial_read_frame_offset       = rd_client_ptr->rw_pos.frame_offset;
   uint32_t len_consumed_from_frame         = 0;
   uint32_t initial_bytes_available_to_read = cur_ch0_frame_ptr->actual_data_len - rd_client_ptr->rw_pos.frame_offset;
   uint32_t num_frames_read                 = cur_ch0_frame_ptr->num_encoded_frames_in_cur_buf;

   // if data is being read for the first time from the frame clone metadata from buffer frame to
   // reader's handle.
   if (initial_read_frame_offset == 0)
   {
      // Clone sdata from the new read frame to readers handle
      _circ_buf_raw_clone_sdata_from_cur_frame_to_reader_hdl(cur_ch0_frame_ptr, rd_client_ptr, (out_sdata_ptr != NULL));
   }

   // check if the data can be further copied from buffer frame to output buffer.
   bool_t need_to_continue = _circ_buf_raw_can_read_data_further(rd_client_ptr, out_sdata_ptr, initial_bytes_in_output);
   if (FALSE == need_to_continue)
   {

#ifdef DEBUG_CIRC_BUF_UTILS_VERBOSE
      AR_MSG(DBG_HIGH_PRIO,
             "read_loop: buf_id: 0x%lx, chunk_id: 0x%lx, chunk_ptr: 0x%x, "
             "frame_position:%lu  chunk_size: %lu",
             rd_client_ptr->circ_buf_raw_ptr->id,
             cur_rd_chunk_ptr->id,
             cur_rd_chunk_ptr,
             rd_client_ptr->rw_pos.frame_position,
             cur_rd_chunk_ptr->size);
#endif
      // TODO: CB CHECK ONCE
      return SPF_CIRCBUF_SUCCESS;
   }

   len_consumed_from_frame = initial_bytes_available_to_read;

   /* Copy data from channel buffers to capi output stream sdata .*/
   spf_circ_buf_chunk_t *cur_chunk_ptr = (spf_circ_buf_chunk_t *)rd_client_ptr->rw_pos.chunk_node_ptr->obj_ptr;

   for (uint32_t ch_idx = 0; ch_idx < cur_chunk_ptr->num_channels; ch_idx++)
   {
      spf_circ_buf_frame_t *cur_ch_frame_ptr = get_frame_ptr(&rd_client_ptr->rw_pos, ch_idx);
      memscpy(&out_ptr[ch_idx].data_ptr[initial_bytes_in_output],
              len_consumed_from_frame,
              &cur_ch_frame_ptr->data[rd_client_ptr->rw_pos.frame_offset],
              len_consumed_from_frame);

      // updates bytes read till now.
      out_ptr[ch_idx].actual_data_len += len_consumed_from_frame;
   }

   rd_client_ptr->rw_pos.frame_offset += len_consumed_from_frame;

   // update unread bytes from the reader client.
   rd_client_ptr->unread_bytes -= len_consumed_from_frame;
   rd_client_ptr->unread_num_frames -= num_frames_read;
   rd_client_ptr->unread_bytes_max -= rd_client_ptr->circ_buf_raw_ptr->wr_raw_client_ptr->raw_frame_len;

   // Propagate metdata from buffer frame to output frame.

   res |= _circular_buffer_raw_propagate_metadata_to_output_stream(rd_client_ptr,
                                                                   out_ptr,
                                                                   out_sdata_ptr,
                                                                   initial_read_frame_offset,
                                                                   initial_bytes_available_to_read,
                                                                   len_consumed_from_frame,
                                                                   initial_bytes_in_output);

   // Free metadata if the frame is exhausted. Move to the next read frame. only for ch_idx == 0
   if (len_consumed_from_frame == initial_bytes_available_to_read)
   {
      // TODO: get stream associated metadata in the cur frame and copy it to next frame position
      //       in the read handle. Currently freeing any metadata left in the current reader pos,
      //       ideally this should not happen.

      _circ_buf_raw_free_sdata(rd_client_ptr->circ_buf_raw_ptr,
                               &rd_client_ptr->rw_pos.readers_sdata,
                               TRUE /* force_free*/,
                               NULL /* return list of stream associated metadata */);

      // Advance to the next frame
      res |= _circ_buf_raw_advance_to_next_frame(rd_client_ptr->circ_buf_raw_ptr, &rd_client_ptr->rw_pos);
   }

   return res;
}

/* Clone metadata from buffer to the output stream */
spf_circ_buf_result_t _circ_buf_raw_clone_sdata_from_cur_frame_to_reader_hdl(spf_circ_buf_frame_t *cur_rd_frame_ptr,
                                                                             spf_circ_buf_raw_client_t *rd_client_ptr,
                                                                             bool_t is_output_sdata_v2)
{
   spf_circ_buf_result_t result           = SPF_CIRCBUF_SUCCESS;
   spf_circ_buf_raw_t *  circ_buf_raw_ptr = rd_client_ptr->circ_buf_raw_ptr;

#ifdef DEBUG_CIRC_BUF_UTILS_METADATA
   AR_MSG(DBG_HIGH_PRIO,
          "clone_metadata: buf_id: 0x%lx cloning sdata from cur frame 0x%lx to reader 0x%lx ",
          circ_buf_raw_ptr->id,
          cur_rd_frame_ptr,
          rd_client_ptr->id);
#endif

   /* increment the frame read ref counter,
    * Every reader touching the frame will increment the ref count.
    * This is used to free metadata when the last reader touches the frame.
    * Ref count will be reset when the writer writes new data.
    */
   cur_rd_frame_ptr->reader_ref_count++;

   /*
    * *********** Clone Flags and timestamp *******************
    */
   {

#ifdef DEBUG_CIRC_BUF_UTILS_METADATA
      AR_MSG(DBG_HIGH_PRIO,
             "clone_metadata: buf_id: 0x%lx Cloning flags 0x%lx timestamp %ld to reader handle 0x%lx ",
             circ_buf_raw_ptr->id,
             cur_rd_frame_ptr->sdata.flags.word,
             cur_rd_frame_ptr->sdata.timestamp,
             rd_client_ptr->id);
#endif

      // Copy flags and timestamp of the current frame to readers handle.
      // TODO: check if this is correct.
      rd_client_ptr->rw_pos.readers_sdata.flags.word = cur_rd_frame_ptr->sdata.flags.word;

      // Copy flags and timestamp of the current frame to readers handle.
      rd_client_ptr->rw_pos.readers_sdata.timestamp = cur_rd_frame_ptr->sdata.timestamp;
   }

   /*
    * *********** Clone metadata list *******************
    */
   if (FALSE == is_output_sdata_v2)
   {
      // if output doesn't support sdata v2, no need to propagate metadata.

      return result;
   }

   // if current read frame doesnt have metadata do nothing.
   if (!cur_rd_frame_ptr->sdata.metadata_list_ptr || !circ_buf_raw_ptr->metadata_handler)
   {
      return result;
   }

#ifdef DEBUG_CIRC_BUF_UTILS_METADATA
   print_frame_metadata(cur_rd_frame_ptr);
#endif

   // Iterate through the metadata list in the buffer frame and clone the nodes to readers handle.
   bool_t is_clone = FALSE;
   if (cur_rd_frame_ptr->reader_ref_count < circ_buf_raw_ptr->num_read_clients)
   {
      is_clone = TRUE;
   }

   result = _circ_buf_raw_clone_or_move_metadata_list(circ_buf_raw_ptr,
                                                      &cur_rd_frame_ptr->sdata.metadata_list_ptr,
                                                      &rd_client_ptr->rw_pos.readers_sdata.metadata_list_ptr,
                                                      is_clone);

   return result;
}

/* Move / clone metadata in the buffer*/
spf_circ_buf_result_t _circ_buf_raw_clone_or_move_metadata_list(spf_circ_buf_raw_t *   circ_buf_raw_ptr,
                                                                module_cmn_md_list_t **src_list_pptr,
                                                                module_cmn_md_list_t **dst_list_pptr,
                                                                bool_t                 is_clone)
{

   spf_circ_buf_result_t result = SPF_CIRCBUF_SUCCESS;

   module_cmn_md_list_t *cur_md_node_ptr = *src_list_pptr;
   while (cur_md_node_ptr)
   {
      module_cmn_md_t *md_ptr = (module_cmn_md_t *)cur_md_node_ptr->obj_ptr;

      if (MODULE_CMN_MD_ID_EOS == md_ptr->metadata_id)
      {

#ifdef DEBUG_CIRC_BUF_UTILS_METADATA
         AR_MSG(DBG_HIGH_PRIO, "clone_metadata: buf_id: 0x%lx Propagating out EOS  ", circ_buf_raw_ptr->id);
#endif
      }
      else if (MODULE_CMN_MD_ID_DFG == md_ptr->metadata_id)
      {

#ifdef DEBUG_CIRC_BUF_UTILS_METADATA
         AR_MSG(DBG_HIGH_PRIO, "clone_metadata: buf_id: 0x%x Propagating out DFG", circ_buf_raw_ptr->id);
#endif
      }

      // If last reader is reading the frame from circular buffer then no need to clone.
      // Only for last reader ref count is equal to num_read_clients.
      if (is_clone)
      {
#ifdef DEBUG_CIRC_BUF_UTILS_METADATA
         AR_MSG(DBG_HIGH_PRIO,
                "clone_metadata: buf_id: 0x%x Cloning  metadata_id: 0x%x",
                circ_buf_raw_ptr->id,
                md_ptr->metadata_id);
#endif
         capi_heap_id_t heap_id = { .heap_id = (uint32_t)circ_buf_raw_ptr->heap_id };
         capi_err_t     capi_res =
            circ_buf_raw_ptr->metadata_handler->metadata_clone(circ_buf_raw_ptr->metadata_handler->context_ptr,
                                                               md_ptr,
                                                               dst_list_pptr,
                                                               heap_id);

         if (CAPI_FAILED(capi_res))
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "clone_metadata: buf_id: 0x%x Failed cloning metadata 0x%lx",
                   circ_buf_raw_ptr->id,
                   md_ptr->metadata_id);
            return SPF_CIRCBUF_FAIL;
         }
      }
      else // last reader just moves the list from one frame to reader's handle
      {
#ifdef DEBUG_CIRC_BUF_UTILS_METADATA
         AR_MSG(DBG_HIGH_PRIO,
                "clone_metadata: buf_id: 0x%x Moving metadata 0x%lx",
                circ_buf_raw_ptr->id,
                md_ptr->metadata_id);
#endif
         // for the last reader just move metadata node from circular buffer to reader client handle.
         spf_list_move_node_to_another_list((spf_list_node_t **)dst_list_pptr,
                                            (spf_list_node_t *)cur_md_node_ptr,
                                            (spf_list_node_t **)src_list_pptr);

         // when a cur_node_ptr is moved it becomes invalid, so init again to the head of the list.
         cur_md_node_ptr = *src_list_pptr;
         continue;
      }

      // advance to the next metadata list node
      LIST_ADVANCE(cur_md_node_ptr);
   }

   return result;
}

/* Propagate metadata to output stream */
spf_circ_buf_result_t _circular_buffer_raw_propagate_metadata_to_output_stream(spf_circ_buf_raw_client_t *rd_client_ptr,
                                                                               capi_buf_t *               out_buf_ptr,
                                                                               capi_stream_data_t *       out_sdata_ptr,
                                                                               uint32_t initial_read_frame_offset,
                                                                               uint32_t initial_bytes_available_to_read,
                                                                               uint32_t len_consumed_from_frame,
                                                                               uint32_t initial_bytes_in_output)
{

   spf_circ_buf_raw_t *circ_buf_raw_ptr = rd_client_ptr->circ_buf_raw_ptr;
   // spf_circ_buf_chunk_t *rd_chunk_ptr = (spf_circ_buf_chunk_t *)rd_client_ptr->rw_pos.chunk_node_ptr->obj_ptr;

#ifdef DEBUG_CIRC_BUF_UTILS_METADATA
   AR_MSG(DBG_HIGH_PRIO,
          "propagate_metadata: entered output metadata propagation buf_id: 0x%lx ",
          rd_client_ptr->circ_buf_raw_ptr->id);
#endif

   // if the read client doesn't pass sdata, do nothing.
   if (NULL == out_sdata_ptr)
   {
      return SPF_CIRCBUF_SUCCESS;
   }

   // if the output sdata is not stream v2, metadata propagation is not supported.
   bool_t is_output_sdata_v2 = FALSE;
   if ((CAPI_STREAM_V2 == out_sdata_ptr->flags.stream_data_version))
   {
      is_output_sdata_v2 = TRUE;
   }

   { // begin code block: flag propagation

      // Scenarios to propagate flags from a read frame.
      // In all the scenarios, if the output already has data its assumed that flags in read frame and output sdata
      // to be same. If they are not same, read() would have returned earlier & it wouldn't have reached
      // propagation stage.
      //
      // Currently only, is_timestamp_valid, marker_EOS, EOF and Erasure flags need to be propagated.
      //
      //  Case 1: Read entire frame from beginning to end.
      //       -> copy all the flags to output, if output already has data also we can copy. Since flags are same.
      //
      //  Case 2: Read data from a read_offset to end of the frame.
      //       -> copy all the flags to output, if output already has data also we can copy. Since flags are same.
      //
      //  Case 3: Read partial data from beginning of the frame.
      //       -> Propagate only flags corresponding to partial data i.e only Erasure/marker EOS.
      //       -> Marker EOS will propagated by metadata_propagate()
      //       -> EOF is associated with last sample, so it will not be propagated now.
      //
      //  Case 4: Read partial data from a read_offset.
      //       -> Propagate only flags corresponding to partial data i.e only Erasure/marker EOS.
      //       -> Marker EOS will propagated by metadata_propagate()
      //       -> EOF is not propagated
      //
      //  Case 5: actual_data_len of current frame is 'zero'
      //       -> Propagate all flags and metadata from frame to o/p

      // Timestamp is updated only when the data is written first time into capi's output buffer.
      if (0 == initial_bytes_in_output)
      {
         // Update the output buffer timestamp based on the current read offset.
         if (rd_client_ptr->rw_pos.readers_sdata.flags.is_timestamp_valid)
         {
            out_sdata_ptr->timestamp = rd_client_ptr->rw_pos.readers_sdata.timestamp;

            out_sdata_ptr->flags.is_timestamp_valid = TRUE;
         }
      }
      else // if there was data in the o/p
      {
         // in this case, flags in the o/p and current read frame are expected to be same before reading data.
         // And also timestamp is retained as it is, no need to check for TS because frame work already checks
      }

#ifdef DEBUG_CIRC_BUF_UTILS_METADATA
      AR_MSG(DBG_HIGH_PRIO,
             "propagate_metadata: buf_id: 0x%lx Frame is exhausted. out sdata flags 0x%lx, output ts: %lu, frame "
             "flags 0x%lx init_bytes_op: %lu init_bytes_avail_to_read: %lu, initial_read_frame_offset:%lu bytes_read: "
             "%lu ",
             rd_client_ptr->circ_buf_raw_ptr->id,
             out_sdata_ptr->flags.word,
             out_sdata_ptr->timestamp,
             rd_client_ptr->rw_pos.readers_sdata.flags.word,
             initial_bytes_in_output,
             initial_bytes_available_to_read,
             initial_read_frame_offset,
             len_consumed_from_frame);
#endif

      if (len_consumed_from_frame == initial_bytes_available_to_read)
      {

         // Handles case 1,2,5 -> propagate all the flags from frame to o/p.
         out_sdata_ptr->flags.word                      = rd_client_ptr->rw_pos.readers_sdata.flags.word;
         rd_client_ptr->rw_pos.readers_sdata.flags.word = 0;
      }
      else // partial read
      {
         // Handles case 3 & 4
         // Copy only erasure
         out_sdata_ptr->flags.erasure = rd_client_ptr->rw_pos.readers_sdata.flags.erasure;
      }

      // reset stream data version as it was earlier.
      out_sdata_ptr->flags.stream_data_version = is_output_sdata_v2;

   } // end code block: flag propagation

   // if the current reader clients has metadata associated with the current frame propagate it to the output.
   if (is_output_sdata_v2 && rd_client_ptr->rw_pos.readers_sdata.metadata_list_ptr)
   {

#ifdef DEBUG_CIRC_BUF_UTILS_METADATA
      print_frame_metadata_list(&rd_client_ptr->rw_pos.readers_sdata);
#endif

      capi_stream_data_v2_t *    out_sdata_v2_ptr = (capi_stream_data_v2_t *)out_sdata_ptr;
      uint32_t                   ALGO_DELAY_ZERO  = 0; // Sync has no algo delay.
      intf_extn_md_propagation_t input_md_info;
      memset(&input_md_info, 0, sizeof(input_md_info));
      input_md_info.df                          = CAPI_RAW_COMPRESSED;
      input_md_info.len_per_ch_in_bytes         = len_consumed_from_frame;
      input_md_info.initial_len_per_ch_in_bytes = initial_bytes_available_to_read;

      intf_extn_md_propagation_t output_md_info;
      memscpy(&output_md_info, sizeof(output_md_info), &input_md_info, sizeof(input_md_info));
      output_md_info.len_per_ch_in_bytes         = len_consumed_from_frame;
      output_md_info.initial_len_per_ch_in_bytes = initial_bytes_in_output; // check if its handled.

      // propagate metadata from reader frame handle to output stream data pointer.
      capi_err_t capi_res =
         circ_buf_raw_ptr->metadata_handler->metadata_propagate(circ_buf_raw_ptr->metadata_handler->context_ptr,
                                                                &rd_client_ptr->rw_pos.readers_sdata, // circ buf frame
                                                                                                      // sdata
                                                                out_sdata_v2_ptr, // capi sdata v2 ptr
                                                                NULL,
                                                                ALGO_DELAY_ZERO,
                                                                &input_md_info,
                                                                &output_md_info);
      if (CAPI_FAILED(capi_res))
      {
         AR_MSG(DBG_ERROR_PRIO,
                "propagate_metadata: buf_id: 0x%x Reader 0x%x failed in propagating metadata to output.",
                rd_client_ptr->circ_buf_raw_ptr->id,
                rd_client_ptr->id);
         return SPF_CIRCBUF_FAIL;
      }

      // Adjust the remaining metadata offsets based on the data consumed
      module_cmn_md_list_t *list_ptr = rd_client_ptr->rw_pos.readers_sdata.metadata_list_ptr;
      while (list_ptr)
      {
         module_cmn_md_t *md_ptr = (module_cmn_md_t *)list_ptr->obj_ptr;

         // Decrement remaining metadata offset w.r.t to new reader's offset position.
         md_ptr->offset = md_ptr->offset - len_consumed_from_frame;
         // TODO: CB CHECK ONCE

         LIST_ADVANCE(list_ptr);
      }
   }

#ifdef DEBUG_CIRC_BUF_UTILS_METADATA
   // print remaining metadata in the readers frame
   print_frame_metadata_list(&rd_client_ptr->rw_pos.readers_sdata);
#endif

   return SPF_CIRCBUF_SUCCESS;
}

/* Frees metadata from the given frame of a chunk. */
spf_circ_buf_result_t _circ_buf_raw_free_sdata(spf_circ_buf_raw_t *   circ_buf_raw_ptr,
                                               capi_stream_data_v2_t *sdata_ptr,
                                               bool_t                 force_free,
                                               module_cmn_md_list_t **stream_associated_md_list_pptr)
{
   spf_circ_buf_result_t res = SPF_CIRCBUF_SUCCESS;

   // metadata overrun needs to be printed as a warning.
   if (sdata_ptr->metadata_list_ptr && circ_buf_raw_ptr->metadata_handler)
   {
      // TODO: Destroy only sample associated and Buffer associated metadata. Stream associated metadata must
      //       be carried forward for overrun. Currently force_free is set TRUE for overrun cases as well.

      module_cmn_md_list_t *cur_node_ptr = sdata_ptr->metadata_list_ptr;
      while (cur_node_ptr)
      {
         // get current metadata node object.
         module_cmn_md_t *md_ptr = (module_cmn_md_t *)cur_node_ptr->obj_ptr;

         // Check if metadata can be destoryed or not
         bool_t is_stream_associated = FALSE;
         bool_t need_to_be_dropped   = FALSE; /* by default metadata is rendered during destroy. */
         if (MODULE_CMN_MD_ID_EOS == md_ptr->metadata_id)
         {

#ifdef DEBUG_CIRC_BUF_UTILS_METADATA
            AR_MSG(DBG_HIGH_PRIO, "free_metadata: buf_id: 0x%x EOS found.", circ_buf_raw_ptr->id);
#endif
            is_stream_associated = TRUE;
         }
         else
         {
            AR_MSG(DBG_HIGH_PRIO,
                   "free_metadata: buf_id: 0x%x Unknown metadata found dropping it. metadata_id:  0x%x ",
                   circ_buf_raw_ptr->id,
                   md_ptr->metadata_id);
            need_to_be_dropped = TRUE;
         }

         // Destroyed if,
         //   1. Force free is true.
         //        OR
         //   2. If its NOT stream associated metadata.
         if (force_free || !is_stream_associated)
         {
            capi_err_t capi_res =
               circ_buf_raw_ptr->metadata_handler->metadata_destroy(circ_buf_raw_ptr->metadata_handler->context_ptr,
                                                                    cur_node_ptr,
                                                                    need_to_be_dropped, // dropping metadata
                                                                    &sdata_ptr->metadata_list_ptr);
            if (CAPI_FAILED(capi_res))
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "free_metadata: buf_id: 0x%x Failed in destroyig metadata 0x%lx",
                      circ_buf_raw_ptr->id,
                      md_ptr->metadata_id);
            }

            // when a node is destoyed init again to the head of the list.
            cur_node_ptr = sdata_ptr->metadata_list_ptr;
            continue;
         }

         // Retained if,
         //  1. If its IS stream associated metadata.
         //          &&
         //  2. only if force is False.
         if (!force_free && is_stream_associated)
         {
            if (stream_associated_md_list_pptr)
            {
               // for the last reader just move metadata node from circular buffer to reader client handle.
               spf_list_move_node_to_another_list((spf_list_node_t **)stream_associated_md_list_pptr,
                                                  (spf_list_node_t *)cur_node_ptr,
                                                  (spf_list_node_t **)&sdata_ptr->metadata_list_ptr);
            }

            // when a node is destroyed init again to the head of the list.
            cur_node_ptr = sdata_ptr->metadata_list_ptr;
            continue;
         }

         LIST_ADVANCE(cur_node_ptr);
      }
   }

   // memset frame stream header.
   memset(sdata_ptr, 0, sizeof(capi_stream_data_v2_t));

   return res;
}

/* advances to next frame in the circular buffer.
 * 1. If the current position is in last frame of the chunk, moves to the first frame of next chunk.
 * 2. If the next frame is in same chunk, just updates the chunk frame position. */
spf_circ_buf_result_t _circ_buf_raw_advance_to_next_frame(spf_circ_buf_raw_t *     circ_buf_raw_ptr,
                                                          spf_circ_buf_position_t *pos)
{
   if ((NULL == circ_buf_raw_ptr) || (NULL == pos))
   {
      return SPF_CIRCBUF_FAIL;
   }
   spf_circ_buf_chunk_t *prev_chunk_ptr = (spf_circ_buf_chunk_t *)pos->chunk_node_ptr->obj_ptr;

   uint32_t next_frame_position = pos->frame_position + GET_CHUNK_FRAME_SIZE(prev_chunk_ptr->frame_size);

#ifdef DEBUG_CIRC_BUF_UTILS_VERBOSE
   AR_MSG(DBG_HIGH_PRIO,
          "adv_next_frame: buf_id: 0x%x, cur_chunk_ptr: 0x%x, chunk_size: %lu "
          "next_frame_position: %lu, ",
          circ_buf_raw_ptr->id,
          prev_chunk_ptr,
          prev_chunk_ptr->size,
          next_frame_position);
#endif

   // If the next frame's end address is beyond chunk size it becomes invalid
   // In that case advance to the next frame in the next chunk.
   // Else, move to the next frame in the same chunk.
   uint32_t next_frame_position_end = next_frame_position + GET_CHUNK_FRAME_SIZE(prev_chunk_ptr->frame_size);
   if (next_frame_position_end > prev_chunk_ptr->size)
   {
      // Move to the next chunk in the list if it exist.
      // If the next pointer doesn't exist, it means the node is the tail so assign the head as the chunk pointer
      spf_circ_buf_result_t res = _circ_buf_raw_next_chunk_node(circ_buf_raw_ptr, &pos->chunk_node_ptr);
      if (res != SPF_CIRCBUF_SUCCESS)
      {
         return SPF_CIRCBUF_FAIL;
      }

      pos->frame_position = 0;
      pos->frame_offset   = 0;
   }
   else // if the next frame addr is valid, return is as the next frame pointer,
   {
      // move to the next frame in the same chunk
      pos->frame_position = next_frame_position;
      pos->frame_offset   = 0;
   }

#ifdef DEBUG_CIRC_BUF_UTILS_VERBOSE
   spf_circ_buf_chunk_t *temp_chunk_ptr = (spf_circ_buf_chunk_t *)pos->chunk_node_ptr->obj_ptr;
   AR_MSG(DBG_HIGH_PRIO,
          "adv_next_frame: new chunk_ptr: 0x%x, frame_position: %lu, frame_offset: %lu ",
          temp_chunk_ptr,
          pos->frame_position,
          pos->frame_offset);
#endif

   return SPF_CIRCBUF_SUCCESS;
}

/* Get the next chunk node pointer */
spf_circ_buf_result_t _circ_buf_raw_next_chunk_node(spf_circ_buf_raw_t *circ_buf_raw_ptr,
                                                    spf_list_node_t **  chunk_ptr_ptr)
{
   if ((NULL == chunk_ptr_ptr) || (NULL == *chunk_ptr_ptr) || (NULL == circ_buf_raw_ptr))
   {
      return SPF_CIRCBUF_FAIL;
   }

   /* Move to the next chunk in the list if it exist. If the next pointer doesnt exist, it means the node
    * is the tail so assign the head as the chunk pointer */

   if ((*chunk_ptr_ptr)->next_ptr)
   {
      (*chunk_ptr_ptr) = (*chunk_ptr_ptr)->next_ptr;
   }
   else
   {
      (*chunk_ptr_ptr) = circ_buf_raw_ptr->head_chunk_ptr;
   }

   return SPF_CIRCBUF_SUCCESS;
}

/* Check if more raw data frames can be written into output */
bool_t _circ_buf_raw_can_read_data_further(spf_circ_buf_raw_client_t *rd_client_ptr,
                                           capi_stream_data_t *       out_sdata_ptr,
                                           uint32_t                   initial_bytes_in_output)
{
   bool_t can_read_further = TRUE;

   if (out_sdata_ptr)
   {
      // check if any flags are set on the output sdata.
      if (out_sdata_ptr->flags.end_of_frame || out_sdata_ptr->flags.marker_eos)
      {
         can_read_further &= FALSE;
      }

      // if there is already data in output, then check if the current read frame has same flags as output.
      if (initial_bytes_in_output && (out_sdata_ptr->flags.word != rd_client_ptr->rw_pos.readers_sdata.flags.word))
      {
         can_read_further &= FALSE;
      }
   }

   return can_read_further;
}

/* Check if raw circular buffer is empty - has no unread data */
spf_circ_buf_result_t spf_circ_buf_raw_driver_is_buffer_empty(spf_circ_buf_raw_client_t *rd_hdl_ptr,
                                                              bool_t *                   is_empty_ptr)
{
   bool_t is_empty = FALSE;
   if (!rd_hdl_ptr || !is_empty_ptr || !(rd_hdl_ptr->circ_buf_raw_ptr))
   {
      return SPF_CIRCBUF_FAIL;
   }

   spf_circ_buf_raw_client_t *wr_hdl_ptr = rd_hdl_ptr->circ_buf_raw_ptr->wr_raw_client_ptr;

   if (!wr_hdl_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "is_buffer_empty not yet implemented for no write ptr case.");

      /* This function currently doesn't work if there's no writers (case of only metadata buffered).
       * For this we need to keep track of unread_frames. */
      return SPF_CIRCBUF_FAIL;
   }

   if (wr_hdl_ptr->rw_pos.chunk_node_ptr == rd_hdl_ptr->rw_pos.chunk_node_ptr &&
       wr_hdl_ptr->rw_pos.frame_position == rd_hdl_ptr->rw_pos.frame_position)
   {
      /* Read/Write pointers will be equal if the buffer is empty. If the buffer is empty,
       * unread bytes is 0 (otherwise buffer is full).  */
      is_empty = (0 == rd_hdl_ptr->unread_bytes);
   }

   *is_empty_ptr = is_empty;
   return SPF_CIRCBUF_SUCCESS;
}
