/**
 * \file spf_circular_buffer_stream_utils.c
 * \brief
 *    This file contains implementation of circular buffering utilities.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/*==============================================================================
   Headers and Includes.
==============================================================================*/

#include "ar_msg.h"
#include "spf_list_utils.h"
#include "spf_circular_buffer_i.h"

/*==============================================================================
   Local utility Functions
==============================================================================*/

/*==============================================================================
   Public Function Implementation
==============================================================================*/

spf_circ_buf_result_t _move_metadata_from_input_sdata_to_cur_wr_frame(spf_circ_buf_client_t *wr_client_ptr,
                                                                      capi_buf_t *           in_buf_ptr,
                                                                      capi_stream_data_t *   in_sdata_ptr,
                                                                      uint32_t len_consumed_per_ch_in_bytes,
                                                                      uint32_t in_initial_len_per_ch_in_bytes)
{
   spf_circ_buf_result_t res           = SPF_CIRCBUF_SUCCESS;
   spf_circ_buf_chunk_t *chunk_ptr     = (spf_circ_buf_chunk_t *)wr_client_ptr->rw_pos.chunk_node_ptr->obj_ptr;
   spf_circ_buf_frame_t *ch0_frame_ptr = get_frame_ptr(&wr_client_ptr->rw_pos, DEFAULT_CH_IDX);
   spf_circ_buf_t *      circ_buf_ptr  = wr_client_ptr->circ_buf_ptr;
   bool_t                input_had_eos = FALSE;
   bool_t                is_input_fully_consumed = FALSE;
   // If sdata pointer is not given return.
   if (!in_sdata_ptr)
   {
      return res;
   }

   /*
    * Copy stream flags
    */
   {
      // TODO: Do we need to check and prevent copying some metadata ?
      ch0_frame_ptr->sdata.flags.word = in_sdata_ptr->flags.word;

      // Copy Timestamp
      ch0_frame_ptr->sdata.timestamp = in_sdata_ptr->timestamp;

#ifdef DEBUG_CIRC_BUF_UTILS_METADATA
      AR_MSG(DBG_HIGH_PRIO,
             "buffer_metadata: buf_id: 0x%lx buffering flags 0x%lx, timestamp %ld  ",
             circ_buf_ptr->id,
             ch0_frame_ptr->sdata.flags.word,
             ch0_frame_ptr->sdata.timestamp);
#endif
   }

   if (CAPI_STREAM_V2 != in_sdata_ptr->flags.stream_data_version)
   {
#ifdef DEBUG_CIRC_BUF_UTILS_METADATA
      AR_MSG(DBG_HIGH_PRIO,
             "buffer_metadata: buf_id: 0x%lx Invalid stream data version %d  ",
             circ_buf_ptr->id,
             ch0_frame_ptr->sdata.flags.stream_data_version);
#endif
      return res;
   }

   input_had_eos           = in_sdata_ptr->flags.marker_eos;
   is_input_fully_consumed = (len_consumed_per_ch_in_bytes == in_initial_len_per_ch_in_bytes);

   // if metadata is not present in the input frame return. do nothing.
   capi_stream_data_v2_t *in_sdata_v2_ptr = (capi_stream_data_v2_t *)in_sdata_ptr;
   if ((NULL == in_sdata_v2_ptr->metadata_list_ptr) || (NULL == circ_buf_ptr->metadata_handler))
   {
#ifdef DEBUG_CIRC_BUF_UTILS_METADATA
      AR_MSG(DBG_HIGH_PRIO,
             "buffer_metadata: buf_id: 0x%lx empty metadata list 0x%lx handler 0x%lx  ",
             circ_buf_ptr->id,
             in_sdata_v2_ptr->metadata_list_ptr,
             circ_buf_ptr->metadata_handler);
#endif
   }
   else
   {
#ifdef DEBUG_CIRC_BUF_UTILS_METADATA
      print_frame_metadata_list(in_sdata_v2_ptr);
#endif

      uint32_t                   ALGO_DELAY_ZERO = 0; // Sync has no algo delay.
      intf_extn_md_propagation_t input_md_info;       // capi input buffer info
      memset(&input_md_info, 0, sizeof(input_md_info));
      input_md_info.df                          = CAPI_FIXED_POINT; // assuming fixed point data
      input_md_info.len_per_ch_in_bytes         = len_consumed_per_ch_in_bytes;
      input_md_info.initial_len_per_ch_in_bytes = in_initial_len_per_ch_in_bytes;
      input_md_info.buf_delay_per_ch_in_bytes   = 0;
      input_md_info.bits_per_sample             = chunk_ptr->mf->bits_per_sample;
      input_md_info.sample_rate                 = chunk_ptr->mf->sampling_rate;

      intf_extn_md_propagation_t output_md_info; // frame thats buffered current input
      memscpy(&output_md_info, sizeof(output_md_info), &input_md_info, sizeof(input_md_info));
      output_md_info.len_per_ch_in_bytes = len_consumed_per_ch_in_bytes;
      output_md_info.initial_len_per_ch_in_bytes =
         0; // this should be always zero, since writes are always on new frame.
      output_md_info.buf_delay_per_ch_in_bytes = 0;

      // TODO: check capi error.
      circ_buf_ptr->metadata_handler->metadata_propagate(circ_buf_ptr->metadata_handler->context_ptr,
                                                         in_sdata_v2_ptr,       // capi sdata v2 ptr
                                                         &ch0_frame_ptr->sdata, // cur wr frame sdata
                                                         NULL,
                                                         ALGO_DELAY_ZERO,
                                                         &input_md_info,
                                                         &output_md_info);

#ifdef DEBUG_CIRC_BUF_UTILS_METADATA
      AR_MSG(DBG_HIGH_PRIO,
             "buffer_metadata: input has_eos= %d, frame has_eos= %d",
             in_sdata_v2_ptr->flags.marker_eos,
             ch0_frame_ptr->sdata.flags.marker_eos);
#endif

#ifdef DEBUG_CIRC_BUF_UTILS_VERBOSE
      print_frame_metadata(ch0_frame_ptr);
#endif
   }

   // eof propagation
   // EOF propagation during EOS: propagate only once input EOS goes to output.
   if (input_had_eos)
   {
      if (ch0_frame_ptr->sdata.flags.marker_eos && !in_sdata_ptr->flags.marker_eos)
      {
         if (in_sdata_ptr->flags.end_of_frame)
         {
            in_sdata_ptr->flags.end_of_frame        = FALSE;
            ch0_frame_ptr->sdata.flags.end_of_frame = TRUE;
            AR_MSG(DBG_HIGH_PRIO, "EOF was propagated");
         }
      }
   }
   else
   {
      if (is_input_fully_consumed)
      {
         if (in_sdata_ptr->flags.end_of_frame)
         {
            in_sdata_ptr->flags.end_of_frame        = FALSE;
            ch0_frame_ptr->sdata.flags.end_of_frame = TRUE;
            AR_MSG(DBG_HIGH_PRIO, "EOF was propagated");
         }
      }
   }

   return SPF_CIRCBUF_SUCCESS;
}

// Function to free the chunk.
//  1. Iterates through each frame and frees any metadata if present.
spf_circ_buf_result_t _circ_buf_free_chunk_metadata(spf_circ_buf_t *circ_buf_ptr, spf_circ_buf_chunk_t *chunk_ptr)
{
   spf_circ_buf_result_t res           = SPF_CIRCBUF_SUCCESS;
   spf_circ_buf_frame_t *cur_frame_ptr = (spf_circ_buf_frame_t *)chunk_ptr->buf_ptr[DEFAULT_CH_IDX];

   // loop through all frames in chunk
   while (is_a_valid_frame_address(circ_buf_ptr, chunk_ptr, cur_frame_ptr))
   {
      if (cur_frame_ptr->sdata.metadata_list_ptr)
      {
         res |= _circ_buf_free_frame_metadata(circ_buf_ptr, cur_frame_ptr, TRUE /*is_force_free*/, NULL);
      }

      cur_frame_ptr = (spf_circ_buf_frame_t *)GET_NEXT_FRAME_ADDRESS(chunk_ptr, cur_frame_ptr);
   }

   return res;
}

/* Frees metadata from the given frame of a chunk. */
spf_circ_buf_result_t _circ_buf_free_frame_metadata(spf_circ_buf_t *       circ_buf_ptr,
                                                    spf_circ_buf_frame_t * frame_ptr,
                                                    bool_t                 force_free,
                                                    module_cmn_md_list_t **stream_associated_md_list_pptr)
{
   spf_circ_buf_result_t res = SPF_CIRCBUF_SUCCESS;

   _circ_buf_free_sdata(circ_buf_ptr, &frame_ptr->sdata, force_free, stream_associated_md_list_pptr);

   /* Sanity check, should not ideally happen */
   if (frame_ptr->sdata.metadata_list_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "free_frame_metadata: Possible memleak. Couldn't free metadata frame_ptr: 0x%lx",
             frame_ptr);
   }

   // memset frame stream header.
   memset(frame_ptr, 0, sizeof(spf_circ_buf_frame_t));

   return res;
}

#ifdef DEBUG_CIRC_BUF_UTILS_METADATA

void print_frame_metadata_list(capi_stream_data_v2_t *in_sdata_v2_ptr)
{
   /*
    * Iterate through input buffer metadata list and print metadata
    */
   AR_MSG(DBG_HIGH_PRIO, "print_md_list:  Print metadata list 0x%lx", in_sdata_v2_ptr->metadata_list_ptr);

   module_cmn_md_list_t *cur_node_ptr = in_sdata_v2_ptr->metadata_list_ptr;
   while (cur_node_ptr)
   {
      module_cmn_md_t *md_ptr = (module_cmn_md_t *)cur_node_ptr->obj_ptr;
      if (MODULE_CMN_MD_ID_EOS == md_ptr->metadata_id)
      {
         AR_MSG(DBG_HIGH_PRIO, "print_md_list:  has EOS ");
      }
      else if (MODULE_CMN_MD_ID_DFG == md_ptr->metadata_id)
      {
         AR_MSG(DBG_HIGH_PRIO, "print_md_list:  has DFG");
      }
      else
      {
         AR_MSG(DBG_HIGH_PRIO, "print_md_list:  has Metadata id 0x%lx", md_ptr->metadata_id);
      }

      LIST_ADVANCE(cur_node_ptr);
   }
}

void print_frame_metadata(spf_circ_buf_frame_t *frame_ptr)
{
   if (frame_ptr->sdata.flags.end_of_frame)
   {
      AR_MSG(DBG_HIGH_PRIO,
             "print_frame_metadata: frame_ptr: 0x%lx end_of_frame: 0x%lx is set. ",
             frame_ptr,
             frame_ptr->sdata.flags.end_of_frame);
   }

   if (frame_ptr->sdata.metadata_list_ptr)
   {
      AR_MSG(DBG_HIGH_PRIO, "print_frame_metadata: frame_ptr: 0x%lx has a metadata list. ", frame_ptr);
   }
}
#endif

/* Move metadata from input into the raw buffer */
spf_circ_buf_result_t _move_metadata_from_input_sdata_to_cur_wr_raw_frame(spf_circ_buf_raw_client_t *wr_raw_client_ptr,
                                                                          capi_buf_t *               in_buf_ptr,
                                                                          capi_stream_data_t *       in_sdata_ptr,
                                                                          uint32_t len_consumed_per_ch_in_bytes,
                                                                          uint32_t in_initial_len_per_ch_in_bytes)
{
   spf_circ_buf_result_t res = SPF_CIRCBUF_SUCCESS;

   spf_circ_buf_frame_t *ch0_frame_ptr           = get_frame_ptr(&wr_raw_client_ptr->rw_pos, DEFAULT_CH_IDX);
   spf_circ_buf_raw_t *  circ_buf_raw_ptr        = wr_raw_client_ptr->circ_buf_raw_ptr;
   bool_t                input_had_eos           = FALSE;
   bool_t                is_input_fully_consumed = FALSE;
   // If sdata pointer is not given return.
   if (!in_sdata_ptr)
   {
      return res;
   }

   /*
    * Copy stream flags
    */
   {
      // TODO: Do we need to check and prevent copying some metadata ?
      ch0_frame_ptr->sdata.flags.word = in_sdata_ptr->flags.word;

      // Copy Timestamp
      ch0_frame_ptr->sdata.timestamp = in_sdata_ptr->timestamp;

#ifdef DEBUG_CIRC_BUF_UTILS_METADATA
      AR_MSG(DBG_HIGH_PRIO,
             "buffer_metadata: buf_id: 0x%lx buffering flags 0x%lx, timestamp %ld  ",
             circ_buf_raw_ptr->id,
             ch0_frame_ptr->sdata.flags.word,
             ch0_frame_ptr->sdata.timestamp);
#endif
   }

   if (CAPI_STREAM_V2 != in_sdata_ptr->flags.stream_data_version)
   {
#ifdef DEBUG_CIRC_BUF_UTILS_METADATA
      AR_MSG(DBG_HIGH_PRIO,
             "buffer_metadata: buf_id: 0x%lx Invalid stream data version %d  ",
             circ_buf_raw_ptr->id,
             ch0_frame_ptr->sdata.flags.stream_data_version);
#endif
      return res;
   }

   input_had_eos           = in_sdata_ptr->flags.marker_eos;
   is_input_fully_consumed = (len_consumed_per_ch_in_bytes == in_initial_len_per_ch_in_bytes);

   // if metadata is not present in the input frame return. do nothing.
   capi_stream_data_v2_t *in_sdata_v2_ptr = (capi_stream_data_v2_t *)in_sdata_ptr;
   if ((NULL == in_sdata_v2_ptr->metadata_list_ptr) || (NULL == circ_buf_raw_ptr->metadata_handler))
   {
#ifdef DEBUG_CIRC_BUF_UTILS_METADATA
      AR_MSG(DBG_HIGH_PRIO,
             "buffer_metadata: buf_id: 0x%lx empty metadata list 0x%lx handler 0x%lx  ",
             circ_buf_raw_ptr->id,
             in_sdata_v2_ptr->metadata_list_ptr,
             circ_buf_raw_ptr->metadata_handler);
#endif
   }
   else
   {
#ifdef DEBUG_CIRC_BUF_UTILS_METADATA
      print_frame_metadata_list(in_sdata_v2_ptr);
#endif

      uint32_t                   ALGO_DELAY_ZERO = 0; // Sync has no algo delay.
      intf_extn_md_propagation_t input_md_info;       // capi input buffer info
      memset(&input_md_info, 0, sizeof(input_md_info));
      input_md_info.df                          = CAPI_RAW_COMPRESSED; // assuming fixed point data
      input_md_info.len_per_ch_in_bytes         = len_consumed_per_ch_in_bytes;
      input_md_info.initial_len_per_ch_in_bytes = in_initial_len_per_ch_in_bytes;
      input_md_info.buf_delay_per_ch_in_bytes   = 0;

      intf_extn_md_propagation_t output_md_info; // frame thats buffered current input
      memscpy(&output_md_info, sizeof(output_md_info), &input_md_info, sizeof(input_md_info));
      output_md_info.len_per_ch_in_bytes = len_consumed_per_ch_in_bytes;
      output_md_info.initial_len_per_ch_in_bytes =
         0; // this should be always zero, since writes are always on new frame.
      output_md_info.buf_delay_per_ch_in_bytes = 0;

      // TODO: check capi error.
      circ_buf_raw_ptr->metadata_handler->metadata_propagate(circ_buf_raw_ptr->metadata_handler->context_ptr,
                                                             in_sdata_v2_ptr,       // capi sdata v2 ptr
                                                             &ch0_frame_ptr->sdata, // cur wr frame sdata
                                                             NULL,
                                                             ALGO_DELAY_ZERO,
                                                             &input_md_info,
                                                             &output_md_info);

#ifdef DEBUG_CIRC_BUF_UTILS_METADATA
      AR_MSG(DBG_HIGH_PRIO,
             "buffer_metadata: input has_eos= %d, frame has_eos= %d",
             in_sdata_v2_ptr->flags.marker_eos,
             ch0_frame_ptr->sdata.flags.marker_eos);
#endif

#ifdef DEBUG_CIRC_BUF_UTILS_VERBOSE
      print_frame_metadata(ch0_frame_ptr);
#endif
   }

   // eof propagation
   // EOF propagation during EOS: propagate only once input EOS goes to output.
   if (input_had_eos)
   {
      if (ch0_frame_ptr->sdata.flags.marker_eos && !in_sdata_ptr->flags.marker_eos)
      {
         if (in_sdata_ptr->flags.end_of_frame)
         {
            in_sdata_ptr->flags.end_of_frame        = FALSE;
            ch0_frame_ptr->sdata.flags.end_of_frame = TRUE;
            AR_MSG(DBG_HIGH_PRIO, "EOF was propagated");
         }
      }
   }
   else
   {
      if (is_input_fully_consumed)
      {
         if (in_sdata_ptr->flags.end_of_frame)
         {
            in_sdata_ptr->flags.end_of_frame        = FALSE;
            ch0_frame_ptr->sdata.flags.end_of_frame = TRUE;
            AR_MSG(DBG_HIGH_PRIO, "EOF was propagated");
         }
      }
   }

   return SPF_CIRCBUF_SUCCESS;
}

/* Free metadata of a given raw chunk */
spf_circ_buf_result_t _circ_buf_raw_free_chunk_metadata(spf_circ_buf_raw_t *  circ_buf_raw_ptr,
                                                        spf_circ_buf_chunk_t *chunk_ptr)
{
   spf_circ_buf_result_t res = SPF_CIRCBUF_SUCCESS;

   spf_circ_buf_frame_t *cur_frame_ptr = (spf_circ_buf_frame_t *)chunk_ptr->buf_ptr[DEFAULT_CH_IDX];

   // loop through all frames in chunk
   while (is_a_valid_frame_address((spf_circ_buf_t *)circ_buf_raw_ptr, chunk_ptr, cur_frame_ptr))
   {
      if (cur_frame_ptr->sdata.metadata_list_ptr)
      {
         res |= _circ_buf_raw_free_frame_metadata(circ_buf_raw_ptr, cur_frame_ptr, TRUE /*is_force_free*/, NULL);
      }

      cur_frame_ptr = (spf_circ_buf_frame_t *)GET_NEXT_FRAME_ADDRESS(chunk_ptr, cur_frame_ptr);
   }

   return res;
}

/* Frees metadata from the given frame of a chunk. */
spf_circ_buf_result_t _circ_buf_raw_free_frame_metadata(spf_circ_buf_raw_t *   circ_buf_raw_ptr,
                                                        spf_circ_buf_frame_t * frame_ptr,
                                                        bool_t                 force_free,
                                                        module_cmn_md_list_t **stream_associated_md_list_pptr)
{
   spf_circ_buf_result_t res = SPF_CIRCBUF_SUCCESS;

   _circ_buf_raw_free_sdata(circ_buf_raw_ptr, &frame_ptr->sdata, force_free, stream_associated_md_list_pptr);

   /* Sanity check, should not ideally happen */
   if (frame_ptr->sdata.metadata_list_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "free_frame_metadata: Possible memleak. Couldn't free metadata frame_ptr: 0x%lx",
             frame_ptr);
   }

   // memset frame stream header.
   memset(frame_ptr, 0, sizeof(spf_circ_buf_frame_t));

   return res;
}
