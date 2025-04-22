/**
 * \file spf_circular_buffer.c
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

/* Initializes the Circular buffer */
spf_circ_buf_result_t spf_circ_buf_init(spf_circ_buf_t **circ_buf_pptr, spf_circ_buf_alloc_inp_args_t *inp_args)
{

   if ((NULL == circ_buf_pptr) || (0 == inp_args->preferred_chunk_size))
   {
#ifdef DEBUG_CIRC_BUF_UTILS
      AR_MSG(DBG_ERROR_PRIO, "init: Bad init params. Allocation failed.");
#endif
      return SPF_CIRCBUF_FAIL;
   }

   spf_circ_buf_t *circ_buf_ptr = (spf_circ_buf_t *)posal_memory_malloc(sizeof(spf_circ_buf_t), inp_args->heap_id);
   if (!circ_buf_ptr)
   {
#ifdef DEBUG_CIRC_BUF_UTILS
      AR_MSG(DBG_ERROR_PRIO, "init: Allocation failed.");
#endif
      return SPF_CIRCBUF_FAIL;
   }
   memset(circ_buf_ptr, 0, sizeof(spf_circ_buf_t));

   *circ_buf_pptr = circ_buf_ptr;

   if (circ_buf_ptr->head_chunk_ptr || circ_buf_ptr->rd_client_list_ptr || circ_buf_ptr->wr_client_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "init: Allocation failed. Buffer already intialized.");
      return SPF_CIRCBUF_FAIL;
   }
   memset(circ_buf_ptr, 0, sizeof(spf_circ_buf_t));

   /* Initialize the read and write chunks to the head of the chunk list */
   circ_buf_ptr->heap_id            = inp_args->heap_id;
   circ_buf_ptr->id                 = inp_args->buf_id;
   circ_buf_ptr->write_byte_counter = 0;
   circ_buf_ptr->num_read_clients   = 0;
   circ_buf_ptr->wr_client_ptr      = NULL;
   circ_buf_ptr->metadata_handler   = inp_args->metadata_handler;
   circ_buf_ptr->cb_info            = inp_args->cb_info;

   // cache preferred chunk size and compute actual chunk size.
   circ_buf_ptr->preferred_chunk_size = inp_args->preferred_chunk_size;

   AR_MSG(DBG_HIGH_PRIO,
          "alloc: Allocated Circ buf size us = %lu, preferred_chunk_size: %lu ",
          circ_buf_ptr->preferred_chunk_size,
          circ_buf_ptr->num_chunks);

   return SPF_CIRCBUF_SUCCESS;
}

spf_circ_buf_result_t spf_circ_buf_register_reader_client(spf_circ_buf_t *        circ_buf_ptr,
                                                          uint32_t                req_base_buffer_size_in_us,
                                                          spf_circ_buf_client_t **client_hdl_pptr)
{
   // Return if the pointers are null or if the num_write_clients is
   if ((NULL == circ_buf_ptr) || (NULL == client_hdl_pptr))
   {
#ifdef DEBUG_CIRC_BUF_UTILS
      AR_MSG(DBG_HIGH_PRIO,
             "register_read_client: Failed to register. circ_buf_struct_ptr = 0x%lx, client_hdl_pptr=0x%x ",
             circ_buf_ptr,
             client_hdl_pptr);
#endif

      return SPF_CIRCBUF_FAIL;
   }

   spf_circ_buf_client_t *client_hdl_ptr =
      (spf_circ_buf_client_t *)posal_memory_malloc(sizeof(spf_circ_buf_client_t), circ_buf_ptr->heap_id);
   if (!client_hdl_ptr)
   {
#ifdef DEBUG_CIRC_BUF_UTILS
      AR_MSG(DBG_ERROR_PRIO, "register_read_client: Allocation failed.");
#endif
      return SPF_CIRCBUF_FAIL;
   }
   memset(client_hdl_ptr, 0, sizeof(spf_circ_buf_client_t));

   *client_hdl_pptr = client_hdl_ptr;

   // Init base params.
   client_hdl_ptr->is_read_client              = TRUE; // is read client
   client_hdl_ptr->circ_buf_ptr                = circ_buf_ptr;
   client_hdl_ptr->init_base_buffer_size_in_us = req_base_buffer_size_in_us;
   client_hdl_ptr->dynamic_resize_req_in_us    = 0;

   // Add the client to the client list.
   spf_list_insert_tail(&circ_buf_ptr->rd_client_list_ptr, client_hdl_ptr, circ_buf_ptr->heap_id, TRUE /* use_pool*/);
   circ_buf_ptr->num_read_clients++;

   // Resize the circular buffer based on new client request.
   _circ_buf_client_resize(circ_buf_ptr);

   // Resets the read/write position of the client.
   _circ_buf_read_client_reset(client_hdl_ptr);

#ifdef DEBUG_CIRC_BUF_UTILS
   AR_MSG(DBG_HIGH_PRIO,
          "register_read_client: Done. req_alloc_size = %lu, circ buf sz = %lu, num_read_clients= %u ",
          req_base_buffer_size_in_us,
          circ_buf_ptr->circ_buf_size_bytes,
          circ_buf_ptr->num_read_clients);
#endif

   return SPF_CIRCBUF_SUCCESS;
}

spf_circ_buf_result_t spf_circ_buf_register_writer_client(spf_circ_buf_t *        circ_buf_ptr,
                                                          uint32_t                req_base_buffer_size_in_us,
                                                          spf_circ_buf_client_t **client_hdl_pptr)
{
   // Return if the pointers are null or if the num_write_clients is
   if ((NULL == circ_buf_ptr) || (NULL == client_hdl_pptr))
   {
#ifdef DEBUG_CIRC_BUF_UTILS
      AR_MSG(DBG_HIGH_PRIO,
             "register_write_client: Failed to register. circ_buf_struct_ptr = 0x%lx, client_hdl_pptr=0x%x ",
             circ_buf_ptr,
             client_hdl_pptr);
#endif

      return SPF_CIRCBUF_FAIL;
   }

   // if a writer is already registered error out.
   if (circ_buf_ptr->wr_client_ptr)
   {
#ifdef DEBUG_CIRC_BUF_UTILS
      AR_MSG(DBG_HIGH_PRIO,
             "register_write_client: buf_idx=0x%x Failed to register to writer. Already a writer client exist.",
             circ_buf_ptr->id);
#endif
      return SPF_CIRCBUF_FAIL;
   }

   spf_circ_buf_client_t *client_hdl_ptr =
      (spf_circ_buf_client_t *)posal_memory_malloc(sizeof(spf_circ_buf_client_t), circ_buf_ptr->heap_id);
   if (!client_hdl_ptr)
   {
#ifdef DEBUG_CIRC_BUF_UTILS
      AR_MSG(DBG_ERROR_PRIO, "register_write_client: Allocation failed.");
#endif
      return SPF_CIRCBUF_FAIL;
   }
   memset(client_hdl_ptr, 0, sizeof(spf_circ_buf_client_t));

   *client_hdl_pptr = client_hdl_ptr;

   // Init base params.
   client_hdl_ptr->is_read_client              = FALSE; //
   client_hdl_ptr->circ_buf_ptr                = circ_buf_ptr;
   client_hdl_ptr->init_base_buffer_size_in_us = req_base_buffer_size_in_us;
   client_hdl_ptr->dynamic_resize_req_in_us    = 0;

   circ_buf_ptr->wr_client_ptr = client_hdl_ptr;

   // Create buffer with (base buffer size + write_client_requested_buf_size) + max_of(read_client_requests)
   _circ_buf_client_resize(circ_buf_ptr);

   // Resets the read/write position of the client.
   _circ_buf_write_client_reset(client_hdl_ptr);

#ifdef DEBUG_CIRC_BUF_UTILS
   AR_MSG(DBG_HIGH_PRIO,
          "register_write_client: circ buf sz = %lu, req_base_buffer_size_in_us= %lu",
          circ_buf_ptr->circ_buf_size_bytes,
          req_base_buffer_size_in_us);
#endif

   return SPF_CIRCBUF_SUCCESS;
}

spf_circ_buf_result_t spf_circ_buf_read_client_resize(spf_circ_buf_client_t *rd_client_ptr,
                                                      uint32_t               req_buffer_resize_in_us)
{
   if (NULL == rd_client_ptr)
   {
      return SPF_CIRCBUF_FAIL;
   }

   // Resize request, appends to the current size.
   rd_client_ptr->dynamic_resize_req_in_us = req_buffer_resize_in_us;

#ifdef DEBUG_CIRC_BUF_UTILS
   AR_MSG(DBG_HIGH_PRIO,
          "resize: Received client resize request. buf_id = 0x%x, req_buffer_resize_in_us: %u",
          rd_client_ptr->circ_buf_ptr->id,
          req_buffer_resize_in_us);
#endif

   // Request to resize the circ buf
   return _circ_buf_client_resize(rd_client_ptr->circ_buf_ptr);
}

spf_circ_buf_result_t spf_circ_buf_read_data(spf_circ_buf_client_t *rd_client_ptr, capi_stream_data_t *out_sdata_ptr)
{
   return _circ_buf_read_one_frame(rd_client_ptr, out_sdata_ptr);
}

/*
 * Adjusts the read pointer
 * Full documentation in circ_buf_utils.h
 */
spf_circ_buf_result_t spf_circ_buf_read_adjust(spf_circ_buf_client_t *rd_client_ptr,
                                               uint32_t               read_offset,
                                               uint32_t *             actual_unread_bytes_ptr)
{
   if (FALSE == rd_client_ptr->is_read_client)
   {
      return SPF_CIRCBUF_FAIL;
   }

#ifdef DEBUG_CIRC_BUF_UTILS
   AR_MSG(DBG_HIGH_PRIO,
          "circ_buf_read_adjust: Requested read adjust, read_offset= %u, unread_bytes=%u ",
          read_offset,
          rd_client_ptr->unread_bytes);
#endif

   if (read_offset > rd_client_ptr->unread_bytes)
   {
      read_offset = rd_client_ptr->unread_bytes;
   }

   if (actual_unread_bytes_ptr)
   {
      *actual_unread_bytes_ptr = read_offset;
   }

   return _circ_buf_read_advance(rd_client_ptr, rd_client_ptr->unread_bytes - read_offset);
}

/*
 * Dealloc and Deinit the circular buffer
 * Full documentation in circ_buf_utils.h
 */
spf_circ_buf_result_t spf_circ_buf_deinit(spf_circ_buf_t **circ_buf_pptr)
{
   spf_circ_buf_t *circ_buf_ptr = *circ_buf_pptr;

   // Iteraate through each chunk and destroy the chunk
   spf_circ_buf_chunk_t *cur_chunk_ptr =
      (spf_circ_buf_chunk_t *)spf_list_pop_head(&circ_buf_ptr->head_chunk_ptr, TRUE /* pool_used */);
   while (cur_chunk_ptr)
   {
      _circ_buf_free_chunk(circ_buf_ptr, cur_chunk_ptr);

      // pop next chunk pointer
      cur_chunk_ptr = (spf_circ_buf_chunk_t *)spf_list_pop_head(&circ_buf_ptr->head_chunk_ptr, TRUE /* pool_used */);
   }

   // Reset the write client handle.
   if (circ_buf_ptr->wr_client_ptr)
   {
      _circ_buf_destroy_client_hdl(circ_buf_ptr, &circ_buf_ptr->wr_client_ptr);
   }

   // Reset all the read client handles.
   spf_circ_buf_client_t *temp_rd_client_ptr =
      (spf_circ_buf_client_t *)spf_list_pop_head(&circ_buf_ptr->rd_client_list_ptr, TRUE /* pool_used */);
   while (temp_rd_client_ptr)
   {
      // destory client handle
      _circ_buf_destroy_client_hdl(circ_buf_ptr, &temp_rd_client_ptr);

      temp_rd_client_ptr =
         (spf_circ_buf_client_t *)spf_list_pop_head(&circ_buf_ptr->rd_client_list_ptr, TRUE /* pool_used */);
   }

   // Reset all the client node handles.
   if (NULL != circ_buf_ptr->rd_client_list_ptr)
   {
      spf_list_delete_list_and_free_objs(&circ_buf_ptr->rd_client_list_ptr, TRUE /* pool_used */);
   }

   if (NULL != circ_buf_ptr->mf_list_ptr)
   {
      spf_list_delete_list_and_free_objs(&circ_buf_ptr->mf_list_ptr, TRUE /* pool_used */);
   }

   memset(circ_buf_ptr, 0, sizeof(spf_circ_buf_t));

   posal_memory_free(circ_buf_ptr);

   *circ_buf_pptr = NULL;

   return SPF_CIRCBUF_SUCCESS;
}

spf_circ_buf_result_t spf_circ_buf_write_data(spf_circ_buf_client_t *wr_handle,
                                              capi_stream_data_t *   in_sdata_ptr,
                                              bool_t                 allow_overflow)
{
   return _circ_buf_write_data(wr_handle, in_sdata_ptr, allow_overflow, NULL, NULL);
}

spf_circ_buf_result_t spf_circ_buf_memset(spf_circ_buf_client_t *wr_client_ptr,
                                          bool_t                 allow_overflow,
                                          uint32_t               num_bytes,
                                          uint32_t               memset_value)
{
   return _circ_buf_write_data(wr_client_ptr, NULL, allow_overflow, &memset_value, &num_bytes);
}

spf_circ_buf_result_t spf_circ_buf_deregister_client(spf_circ_buf_client_t **client_hdl_pptr)
{
   if ((NULL == client_hdl_pptr) || (NULL == *client_hdl_pptr) || (NULL == (*client_hdl_pptr)->circ_buf_ptr))
   {
#ifdef DEBUG_CIRC_BUF_UTILS
      AR_MSG(DBG_HIGH_PRIO, "circ_buf_deregister_client: null arguments. Failed to De-register.");
#endif
      return SPF_CIRCBUF_FAIL;
   }
   spf_circ_buf_client_t *client_hdl_ptr = *client_hdl_pptr;
   spf_circ_buf_t *       circ_buf_ptr   = client_hdl_ptr->circ_buf_ptr;
   bool_t                 is_read_client = client_hdl_ptr->is_read_client;

   // Resize the buffer based on the new client list.
   if (is_read_client)
   {
      // Delete only the list node corresponding to the client.
      // Client memory must not be deallocated though.
      spf_list_find_delete_node(&circ_buf_ptr->rd_client_list_ptr, client_hdl_ptr, TRUE /*pool_used*/);

      if (circ_buf_ptr->num_read_clients > 0)
      {
         circ_buf_ptr->num_read_clients--;
      }

      // Resize the buffer based on the updated client list.
      _circ_buf_client_resize(circ_buf_ptr);
   }
   else
   {
      circ_buf_ptr->wr_client_ptr = NULL;
   }

#ifdef DEBUG_CIRC_BUF_UTILS
   AR_MSG(DBG_HIGH_PRIO,
          "circ_buf_deregister_client: Done. is_read_client = %d, circ buf size = %d, "
          "num_read_clients=%lu ",
          is_read_client,
          circ_buf_ptr->circ_buf_size_bytes,
          circ_buf_ptr->num_read_clients);
#endif

   // destroy any dynamic memory associated with the client handle.
   _circ_buf_destroy_client_hdl(circ_buf_ptr, &client_hdl_ptr);

   return SPF_CIRCBUF_SUCCESS;
}

spf_circ_buf_result_t spf_circ_buf_write_one_frame(spf_circ_buf_client_t *wr_handle,
                                                   capi_stream_data_t *   sdata_ptr,
                                                   bool_t                 allow_overflow)
{
   return _circ_buf_write_one_frame(wr_handle, sdata_ptr, allow_overflow, NULL);
}

spf_circ_buf_result_t spf_circ_buf_set_media_format(spf_circ_buf_client_t *wr_handle,
                                                    capi_media_fmt_v2_t *  inp_mf,
                                                    uint32_t               container_frame_size_in_us)
{
   ar_result_t           ar_res = AR_EOK;
   spf_circ_buf_result_t result = SPF_CIRCBUF_SUCCESS;

   if (!wr_handle)
   {
      AR_MSG(DBG_ERROR_PRIO, "Null write pointer ");
      return SPF_CIRCBUF_FAIL;
   }

   spf_circ_buf_t *circ_buf_ptr = wr_handle->circ_buf_ptr;

   /* STEP: 1*/
   // validate and cache media format into the writers mf handle.
   if (!_circ_buf_is_valid_mf(wr_handle, inp_mf))
   {
      AR_MSG(DBG_HIGH_PRIO, "Invalid input media format ");
      return SPF_CIRCBUF_FAIL;
   }

   // validate and copy media format.
   if (_is_mf_unchanged(wr_handle, inp_mf) && (container_frame_size_in_us == wr_handle->container_frame_size_in_us))
   {
      AR_MSG(DBG_HIGH_PRIO,
             "Nothing to do. inp mf and container frame size = %lu is unchanged.",
             container_frame_size_in_us);
      return SPF_CIRCBUF_SUCCESS;
   }
   wr_handle->container_frame_size_in_us = container_frame_size_in_us;

   /* STEP: 2*/
   // Update writer's media format if necessary.
   // validate and copy media format.
   spf_circ_buf_mf_info_t *wr_mf_ptr = NULL;
   if (!_is_mf_unchanged(wr_handle, inp_mf))
   {

      wr_mf_ptr = (spf_circ_buf_mf_info_t *)posal_memory_malloc(sizeof(spf_circ_buf_mf_info_t), circ_buf_ptr->heap_id);
      if (!wr_mf_ptr)
      {
#ifdef DEBUG_CIRC_BUF_UTILS
         AR_MSG(DBG_HIGH_PRIO,
                "register_client: buf_idx=0x%x Failed to register writer, malloc failed.",
                circ_buf_ptr->id);
#endif
         return SPF_CIRCBUF_FAIL;
      }
      memset(wr_mf_ptr, 0, sizeof(spf_circ_buf_mf_info_t));

      AR_MSG(DBG_MED_PRIO, "Media format has changed, new_mf_node_ptr = 0x%lx ", wr_mf_ptr);

      // cache input mf
      wr_mf_ptr->bits_per_sample = inp_mf->format.bits_per_sample;
      wr_mf_ptr->data_is_signed  = inp_mf->format.data_is_signed;
      wr_mf_ptr->sampling_rate   = inp_mf->format.sampling_rate;
      wr_mf_ptr->q_factor        = inp_mf->format.q_factor;
      wr_mf_ptr->num_channels    = inp_mf->format.num_channels;

      for (uint32_t ch_idx = 0; ch_idx < wr_mf_ptr->num_channels; ch_idx++)
      {
         wr_mf_ptr->channel_type[ch_idx] = inp_mf->format.channel_type[ch_idx];
      }

      // Push mf node into mf list.
      // MFs of all the data buffered is pushed to this queu
      ar_res =
         spf_list_insert_tail(&circ_buf_ptr->mf_list_ptr, (void *)wr_mf_ptr, circ_buf_ptr->heap_id, TRUE /* use_pool*/);
      if (AR_EOK != ar_res)
      {

#ifdef DEBUG_CIRC_BUF_UTILS
         AR_MSG(DBG_MED_PRIO, "Failed to push the mf node to the list ");
#endif
         posal_memory_free(wr_mf_ptr);
         return SPF_CIRCBUF_FAIL;
      }

      {
         // decerement writer mf ref count for previous handle.
         if (wr_handle->operating_mf)
         {
            decr_mf_ref_count(circ_buf_ptr, &wr_handle->operating_mf);
         }

         // incerement writer mf referene count
         wr_handle->operating_mf = wr_mf_ptr;
         incr_mf_ref_count(circ_buf_ptr, wr_mf_ptr);
      }
   }
   else
   {
      wr_mf_ptr = wr_handle->operating_mf;
   }

   /*STEP-3*/
   // TODO: if there are no chunks, allocate the chunks and assign readers and writers.
   // all the readers need to raise MF.
   if (NULL == circ_buf_ptr->head_chunk_ptr)
   {
      // Resize the buffer based on the updated client list.
      // Resize function will take care of initalizing readers and writers positions.
      result = _circ_buf_client_resize(circ_buf_ptr);
      return result;
   }

   /* STEP-4: Compute number of chunks info */
   uint32_t num_chunks               = 0;
   uint32_t actual_chunk_size        = 0;
   uint32_t first_chunk_size         = 0;
   uint32_t frame_data_size_in_bytes = 0;
   uint32_t num_additional_frames    = 0;
   _circ_buf_compute_chunk_info(circ_buf_ptr,
                                wr_handle->operating_mf,
                                circ_buf_ptr->circ_buf_size_in_us,
                                &num_additional_frames,
                                &frame_data_size_in_bytes,
                                &num_chunks,
                                &actual_chunk_size,
                                &first_chunk_size);

   uint32_t num_chunks_to_recreate = 0;
   uint32_t num_chunks_to_destroy  = 0;
   uint32_t num_chunks_to_add      = 0;
   if (num_chunks > circ_buf_ptr->num_chunks)
   {
      // mark all chunks to recreate
      num_chunks_to_recreate = circ_buf_ptr->num_chunks;
      // add new chunks after write pointer
      num_chunks_to_add = num_chunks - circ_buf_ptr->num_chunks;
   }
   else if (num_chunks == circ_buf_ptr->num_chunks)
   {
      // mark all chunks to recreate
      num_chunks_to_recreate = circ_buf_ptr->num_chunks;

      // The first chunk size may be different from all chunks, so
      // we need to make sure that the first chunk is added allocated at set mf context itself,
      // its not possible to recreate during time.
      num_chunks_to_add     = 1;
      num_chunks_to_destroy = 1;
      num_chunks_to_recreate -= 1;
   }
   else
   {
      // mark chunks to delete after write pointer
      num_chunks_to_recreate = num_chunks;

      num_chunks_to_destroy = circ_buf_ptr->num_chunks - num_chunks;

      // The first chunk size may be different from all chunks, so
      // we need to make sure that the first chunk is added allocated at set mf context itself,
      // its not possible to recreate during time.
      num_chunks_to_add = 1;
      num_chunks_to_destroy += 1;
      num_chunks_to_recreate -= 1;
   }

   AR_MSG(DBG_MED_PRIO,
          "set_mf: prev_num_chunks: %lu, new_req_num_chunks: %lu, num_chunks_to_recreate: %lu , num_chunks_to_add: %lu "
          " num_chunks_to_destroy: %lu ",
          circ_buf_ptr->num_chunks,
          num_chunks,
          num_chunks_to_recreate,
          num_chunks_to_add,
          num_chunks_to_destroy);

   /* STEP: 5*/
   /* If there are existing chunks recreate the chunks. */
   // mark all chunk to be recreated.
   spf_list_node_t *chunk_list_ptr = (spf_list_node_t *)wr_handle->rw_pos.chunk_node_ptr;
   while (chunk_list_ptr && num_chunks_to_destroy > 0)
   {
      spf_circ_buf_chunk_t *cur_chunk_ptr = (spf_circ_buf_chunk_t *)chunk_list_ptr->obj_ptr;

#ifdef DEBUG_CIRC_BUF_UTILS
      AR_MSG(DBG_HIGH_PRIO, "set_mf: mark to destroy. cur_chunk_ptr = 0x%lx", cur_chunk_ptr);
#endif

      // mark the chunk as to recreate
      cur_chunk_ptr->flags.recreate = FALSE; // both the flags cannot be true so clear other when one is set.
      cur_chunk_ptr->flags.destroy  = TRUE;

      num_chunks_to_destroy--;
      _circ_buf_next_chunk_node(circ_buf_ptr, &chunk_list_ptr);
   }

   while (chunk_list_ptr && num_chunks_to_recreate > 0)
   {
      spf_circ_buf_chunk_t *cur_chunk_ptr = (spf_circ_buf_chunk_t *)chunk_list_ptr->obj_ptr;

#ifdef DEBUG_CIRC_BUF_UTILS
      AR_MSG(DBG_HIGH_PRIO, "set_mf: mark to recreate. cur_chunk_ptr = 0x%lx", cur_chunk_ptr);
#endif

      // mark the chunk as to recreate
      cur_chunk_ptr->flags.destroy  = FALSE; // both the flags cannot be true so clear other when one is set.
      cur_chunk_ptr->flags.recreate = TRUE;

      num_chunks_to_recreate--;
      _circ_buf_next_chunk_node(circ_buf_ptr, &chunk_list_ptr);
   }

   /** STEP: 6
    * Handle corner case: If there is a reader ahead of writer position in the current writers chunk.
    * Then move unread data/metadata in the current chunk to a new chunk and then move readers to the newly created
    * chunk.
    *
    * This will make sure there is no reader ahead of writers chunk, before new mf chunks are inserted.
    */
   bool_t                  move_unread_data_to_a_new_chunk = FALSE;
   spf_circ_buf_mf_info_t *rd_mf_ptr                       = NULL;
   if (wr_handle->rw_pos.chunk_node_ptr)
   {
      spf_circ_buf_chunk_t *cur_wr_chunk_ptr = (spf_circ_buf_chunk_t *)wr_handle->rw_pos.chunk_node_ptr->obj_ptr;
#ifdef DEBUG_CIRC_BUF_UTILS
      AR_MSG(DBG_HIGH_PRIO,
             "set_mf: Need to move writer to next chunk. cur_chunk_ptr = 0x%lx, frame_position: %lu",
             cur_wr_chunk_ptr,
             wr_handle->rw_pos.frame_position);
#endif

      // check if there is any readers ahead of writer in this chunk, check amount of unread data
      for (spf_list_node_t *rd_client_list_ptr = wr_handle->circ_buf_ptr->rd_client_list_ptr;
           (NULL != rd_client_list_ptr);
           LIST_ADVANCE(rd_client_list_ptr))
      {
         spf_circ_buf_client_t *cur_rd_client_ptr = (spf_circ_buf_client_t *)rd_client_list_ptr->obj_ptr;

         // check if the chunk pointer is same.
         if (cur_rd_client_ptr->rw_pos.chunk_node_ptr->obj_ptr != (void *)cur_wr_chunk_ptr)
         {
            // not same chunk.
            continue;
         }

         // if the reader is after writer in current chunk, reader must be moved to begining of the next chunk.
         // if the reader is behind writer in the current chunk.
         if (cur_rd_client_ptr->rw_pos.frame_position >= wr_handle->rw_pos.frame_position &&
             (cur_rd_client_ptr->unread_bytes > 0))
         {
            move_unread_data_to_a_new_chunk = TRUE;
            rd_mf_ptr                       = cur_rd_client_ptr->operating_mf;
#ifdef DEBUG_CIRC_BUF_UTILS
            AR_MSG(DBG_HIGH_PRIO,
                   "set_mf: Reader must be moved to next chunk. cur_chunk_ptr = 0x%lx, frame_position: %lu "
                   "cur_wr_frame_pos: %lu",
                   cur_wr_chunk_ptr,
                   cur_rd_client_ptr->rw_pos.frame_position,
                   wr_handle->rw_pos.frame_position);
#endif
         }
      }

      /* if reader is found, then corner case is detected. move readers to a new chunk*/
      if (move_unread_data_to_a_new_chunk)
      {
         // create one new chunk and copy data after current writer position to the new chunk.
         // and mark the chunk for destroy. once all the data is read from the chunk buffer will be destroyed.
         // Note that the new chunk is created with old media format and will be inserted after current writer position.
         _circ_buf_add_chunks(circ_buf_ptr,
                              1,
                              cur_wr_chunk_ptr->num_frames,
                              cur_wr_chunk_ptr->frame_size,
                              rd_mf_ptr, // prev media format
                              cur_wr_chunk_ptr->size,
                              cur_wr_chunk_ptr->size);

         spf_list_node_t *new_chunk_node_ptr = wr_handle->rw_pos.chunk_node_ptr;
         _circ_buf_next_chunk_node(circ_buf_ptr, &new_chunk_node_ptr);

         spf_circ_buf_chunk_t *new_chunk_ptr = (spf_circ_buf_chunk_t *)new_chunk_node_ptr->obj_ptr;
         new_chunk_ptr->flags.destroy =
            TRUE; /* mark it to destroy, will be destroyed after data is read from the chunk */

         // move data from prev chunk to new chunk
         spf_circ_buf_position_t src_chunk_pos;
         memset(&src_chunk_pos, 0, sizeof(spf_circ_buf_position_t));
         src_chunk_pos.chunk_node_ptr = wr_handle->rw_pos.chunk_node_ptr;
         src_chunk_pos.frame_position = wr_handle->rw_pos.frame_position;

         spf_circ_buf_position_t dest_chunk_pos;
         memset(&dest_chunk_pos, 0, sizeof(spf_circ_buf_position_t));
         dest_chunk_pos.chunk_node_ptr = new_chunk_node_ptr;
         dest_chunk_pos.frame_position = wr_handle->rw_pos.frame_position;

         // move data from prev chunk to new chunk
         uint32_t cur_chunk_frame_size = cur_wr_chunk_ptr->frame_size;
         // copy until the src chunk is same the current writer chunks
         while (src_chunk_pos.chunk_node_ptr == wr_handle->rw_pos.chunk_node_ptr)
         {
            for (uint32_t ch_idx = 0; ch_idx < new_chunk_ptr->num_channels; ch_idx++)
            {
               spf_circ_buf_frame_t *src_ch_frame_ptr  = get_frame_ptr(&src_chunk_pos, ch_idx);
               spf_circ_buf_frame_t *dest_ch_frame_ptr = get_frame_ptr(&dest_chunk_pos, ch_idx);

               // copy frame info and data in the frame
               memscpy(dest_ch_frame_ptr, sizeof(spf_circ_buf_frame_t), src_ch_frame_ptr, sizeof(spf_circ_buf_frame_t));
               memscpy(&dest_ch_frame_ptr->data[0],
                       cur_chunk_frame_size,
                       &src_ch_frame_ptr->data[0],
                       cur_chunk_frame_size);

#ifdef DEBUG_CIRC_BUF_UTILS
               AR_MSG(DBG_HIGH_PRIO,
                      "copied_data: ch_idx=%lu dest_ch_frame_ptr: 0x%lx, src_ch_frame_ptr: 0x%lx, actual_data_len: %lu",
                      ch_idx,
                      dest_ch_frame_ptr,
                      src_ch_frame_ptr,
                      src_ch_frame_ptr->actual_data_len);
#endif

#ifdef DEBUG_CIRC_BUF_UTILS
               AR_MSG(DBG_HIGH_PRIO,
                      "copied_data: ch_idx=%lu reader_ref_count:%lu, src_flags:0x%lx, src_md_list:0x%lx",
                      ch_idx,
                      src_ch_frame_ptr->reader_ref_count,
                      src_ch_frame_ptr->sdata.flags.word,
                      src_ch_frame_ptr->sdata.metadata_list_ptr);
#endif
               // reset src frame header
               memset(src_ch_frame_ptr, 0, sizeof(spf_circ_buf_frame_t));
            }

            // Advance the src and dest frame positions
            _circ_buf_advance_to_next_frame(circ_buf_ptr, &src_chunk_pos);
            _circ_buf_advance_to_next_frame(circ_buf_ptr, &dest_chunk_pos);
         }

         // update the readers chunk node as the next chunk, no need to update other field
         for (spf_list_node_t *rd_client_list_ptr = wr_handle->circ_buf_ptr->rd_client_list_ptr;
              (NULL != rd_client_list_ptr);
              LIST_ADVANCE(rd_client_list_ptr))
         {
            spf_circ_buf_client_t *cur_rd_client_ptr = (spf_circ_buf_client_t *)rd_client_list_ptr->obj_ptr;

            // check if the chunk pointer is same.
            if (cur_rd_client_ptr->rw_pos.chunk_node_ptr->obj_ptr != (void *)cur_wr_chunk_ptr)
            {
               // not same chunk.
               continue;
            }

            // if the reader is after writer in current chunk, reader must be moved to begining of the next chunk.
            // if the reader is behind writer in the current chunk.
            if (cur_rd_client_ptr->rw_pos.frame_position >= wr_handle->rw_pos.frame_position)
            {
               cur_rd_client_ptr->rw_pos.chunk_node_ptr = new_chunk_node_ptr;
#ifdef DEBUG_CIRC_BUF_UTILS
               AR_MSG(DBG_HIGH_PRIO,
                      "set_mf: Reader moved to chunk cur_chunk_ptr = 0x%lx, frame_position: %lu, unread bytes %lu",
                      cur_rd_client_ptr->rw_pos.chunk_node_ptr->obj_ptr,
                      cur_rd_client_ptr->rw_pos.frame_position,
                      cur_rd_client_ptr->unread_bytes);
#endif
            }
         }
      }
   }

   /* STEP: 7*/
   // Create the chunks based on new media format and insert add after current writers position.
   _circ_buf_add_chunks(circ_buf_ptr,
                        num_chunks_to_add,
                        num_additional_frames,
                        frame_data_size_in_bytes,
                        wr_handle->operating_mf,
                        first_chunk_size,
                        actual_chunk_size);

   /* STEP: 8*/
   // The next frame writer writes must begin in the new chunk. Since new mf can be buffered only starting from next
   // chunk. On the next write,
   //    1. if the chunk has recreate flag set, writer will recreate the chunk based on new mf and write new data.
   //    2. if the chunk has destroy flag set. It will destroy the chunk and move to the next chunk.
   // writing new mf data. so update write chunk pointer to next chunk if necessary.
   // Drop data/metadata in frames following writer in current chunk and move writer to next chunk.
   while (wr_handle->rw_pos.frame_position != 0)
   {
      spf_circ_buf_frame_t *cur_wr_frame_ptr = get_frame_ptr(&wr_handle->rw_pos, DEFAULT_CH_IDX);

      // reset the frame
      _circ_buf_free_frame_metadata(wr_handle->circ_buf_ptr, cur_wr_frame_ptr, TRUE, NULL);

      //  Move reader to next frame.
      _circ_buf_advance_to_next_frame(wr_handle->circ_buf_ptr, &wr_handle->rw_pos);
   }

#ifdef DEBUG_CIRC_BUF_UTILS
   AR_MSG(DBG_HIGH_PRIO,
          "set_mf: Writer moved to chunk cur_chunk_ptr = 0x%lx, frame_position: %lu, unread bytes %lu",
          wr_handle->rw_pos.chunk_node_ptr->obj_ptr,
          wr_handle->rw_pos.frame_position,
          wr_handle->unread_bytes);
#endif

   /* STEP: 9*/
   // If any readers are empty, we can set their operating media format immediately.
   for (spf_list_node_t *rd_client_list_ptr = wr_handle->circ_buf_ptr->rd_client_list_ptr; (NULL != rd_client_list_ptr);
        LIST_ADVANCE(rd_client_list_ptr))
   {
      spf_circ_buf_client_t *cur_rd_client_ptr = (spf_circ_buf_client_t *)rd_client_list_ptr->obj_ptr;
      bool_t                 is_reader_empty   = FALSE;
      spf_circ_buf_driver_is_buffer_empty(cur_rd_client_ptr, &is_reader_empty);

      // Check if the reader is in the same chunk as the writer and there's still data to read.
      if (is_reader_empty)
      {
         _circ_buf_check_and_handle_change_in_mf(cur_rd_client_ptr, wr_handle->operating_mf);
      }
   }

   return result;
}

spf_circ_buf_result_t _circ_buf_check_and_recreate(spf_circ_buf_client_t *wr_handle,
                                                   bool_t                 allow_overflow,
                                                   bool_t *               did_recreate_ptr)
{
   spf_circ_buf_result_t result = AR_EOK;
   if (!wr_handle || !did_recreate_ptr)
   {
      return result;
   }

   spf_circ_buf_t *circ_buf_ptr = wr_handle->circ_buf_ptr;

   /* Does the chunk need to be recreated */
   spf_list_node_t *     cur_wr_chunk_node_ptr = wr_handle->rw_pos.chunk_node_ptr;
   spf_circ_buf_chunk_t *cur_wr_chunk_ptr      = (spf_circ_buf_chunk_t *)wr_handle->rw_pos.chunk_node_ptr->obj_ptr;

   // nothing to
   if (!cur_wr_chunk_ptr->flags.destroy && !cur_wr_chunk_ptr->flags.recreate)
   {
      *did_recreate_ptr = FALSE;
      return result;
   }

   // If there are any readers that are still in this chunk, and it's allow_overflow, detect as overflow and
   // recreate later.
   if (!allow_overflow)
   {
      for (spf_list_node_t *rd_client_list_ptr = wr_handle->circ_buf_ptr->rd_client_list_ptr;
           (NULL != rd_client_list_ptr);
           LIST_ADVANCE(rd_client_list_ptr))
      {
         spf_circ_buf_client_t *cur_rd_client_ptr = (spf_circ_buf_client_t *)rd_client_list_ptr->obj_ptr;
         bool_t                 is_reader_empty   = FALSE;
         spf_circ_buf_driver_is_buffer_empty(cur_rd_client_ptr, &is_reader_empty);

         // Check if the reader is in the same chunk as the writer and there's still data to read.
         if ((cur_rd_client_ptr->rw_pos.chunk_node_ptr->obj_ptr == (void *)cur_wr_chunk_ptr) && (!is_reader_empty))
         {
            *did_recreate_ptr = FALSE;
            return SPF_CIRCBUF_OVERRUN;
         }
      }
   }

   *did_recreate_ptr = TRUE;

   AR_MSG(DBG_HIGH_PRIO,
          "check_recreate: buf_id: 0x%lx chunk_ptr:0x%lx is_destroy: 0x%x, is_recreate: 0x%x",
          circ_buf_ptr->id,
          cur_wr_chunk_ptr,
          cur_wr_chunk_ptr->flags.destroy,
          cur_wr_chunk_ptr->flags.recreate);

   if (cur_wr_chunk_ptr->flags.destroy || cur_wr_chunk_ptr->flags.recreate)
   {
      // Free chunk metadata if any exists
      _circ_buf_free_chunk_metadata(wr_handle->circ_buf_ptr, cur_wr_chunk_ptr);

      // check if there is any readers in the chunk, remove the dropped data from unread bytes.
      for (spf_list_node_t *rd_client_list_ptr = wr_handle->circ_buf_ptr->rd_client_list_ptr;
           (NULL != rd_client_list_ptr);
           LIST_ADVANCE(rd_client_list_ptr))
      {
         spf_circ_buf_client_t *cur_rd_client_ptr = (spf_circ_buf_client_t *)rd_client_list_ptr->obj_ptr;

         // check if the chunk pointer is same.
         if (cur_rd_client_ptr->rw_pos.chunk_node_ptr->obj_ptr != (void *)cur_wr_chunk_ptr)
         {
            // not same chunk.
            continue;
         }

         // If the reader is empty, don't move it.
         bool_t is_reader_empty = FALSE;
         result |= spf_circ_buf_driver_is_buffer_empty(cur_rd_client_ptr, &is_reader_empty);
         if (is_reader_empty)
         {
            continue;
         }

#ifdef DEBUG_CIRC_BUF_UTILS
         AR_MSG(DBG_HIGH_PRIO,
                "recreate: buf_id: 0x%lx rd client overrun: 0x%x, rd frame_pos: %lu, total unread_bytes:%lu ",
                circ_buf_ptr->id,
                cur_rd_client_ptr->id,
                cur_rd_client_ptr->rw_pos.frame_position,
                cur_rd_client_ptr->unread_bytes);
#endif

         // Drop metadata in the current readers handle.
         spf_circ_buf_frame_t *cur_frame_ptr = get_frame_ptr(&cur_rd_client_ptr->rw_pos, DEFAULT_CH_IDX);
         result |= _circ_buf_free_sdata(wr_handle->circ_buf_ptr, &cur_rd_client_ptr->rw_pos.readers_sdata, TRUE, NULL);

         // 2. Deduct data thats being overun from readers unreadbytes.
         uint32_t unread_bytes_left_in_chunk = 0;

         // Keep moving the read pointer until it hits the next chunk
         if (circ_buf_ptr->num_chunks > 1)
         {
            while (cur_rd_client_ptr->rw_pos.chunk_node_ptr->obj_ptr == (void *)cur_wr_chunk_ptr)
            {

               // 1. Deduct data thats being overun from readers unreadbytes.
               unread_bytes_left_in_chunk += cur_frame_ptr->actual_data_len - cur_rd_client_ptr->rw_pos.frame_offset;

               // 2. Move reader to next frame.
               _circ_buf_advance_to_next_frame(cur_rd_client_ptr->circ_buf_ptr, &cur_rd_client_ptr->rw_pos);
            }

            // decrement unread bytes left in the chunk from total unread byte count.
            if (unread_bytes_left_in_chunk < cur_rd_client_ptr->unread_bytes)
            {
               cur_rd_client_ptr->unread_bytes -= unread_bytes_left_in_chunk;
            }
            else
            {
               cur_rd_client_ptr->unread_bytes = 0;
            }
         }
         else // circ_buf_ptr->num_chunks <= 1, one chunk case all data is lost
         {
            cur_rd_client_ptr->unread_bytes = 0;
         }
      }
   }

   // Recreate the chunk if flag is set.
   if (cur_wr_chunk_ptr->flags.recreate)
   {
      // Update chunk number.
      circ_buf_ptr->num_chunks--;

      // Update the current state of circular buffer in bytes.
      circ_buf_ptr->circ_buf_size_bytes -= cur_wr_chunk_ptr->num_frames * cur_wr_chunk_ptr->frame_size;

      // TODO: destroy the chunk.
      _circ_buf_free_chunk(circ_buf_ptr, cur_wr_chunk_ptr);

      // get actual chunk size based on writers media format
      uint32_t actual_chunk_size        = 0;
      uint32_t frame_data_size_in_bytes = 0;
      _circ_buf_compute_chunk_info(circ_buf_ptr,
                                   wr_handle->operating_mf,
                                   circ_buf_ptr->circ_buf_size_in_us,
                                   NULL,
                                   &frame_data_size_in_bytes,
                                   NULL,
                                   &actual_chunk_size,
                                   NULL);

      // allocate chunk function
      spf_circ_buf_chunk_t *new_chunk_ptr =
         _circ_buf_allocate_chunk(circ_buf_ptr, frame_data_size_in_bytes, actual_chunk_size, wr_handle->operating_mf);
      if (!new_chunk_ptr)
      {
         // free the temp list
         return SPF_CIRCBUF_FAIL;
      }

      // Update chunk number.
      circ_buf_ptr->num_chunks++;

      // Update the current state of circular buffer in bytes.
      circ_buf_ptr->circ_buf_size_bytes += (new_chunk_ptr->num_frames * new_chunk_ptr->frame_size);

      wr_handle->rw_pos.chunk_node_ptr->obj_ptr = new_chunk_ptr;
      wr_handle->rw_pos.frame_offset            = 0;
      wr_handle->rw_pos.frame_position          = 0;
   }
   else if (cur_wr_chunk_ptr->flags.destroy)
   {
      // move the writer to next chunk
      spf_list_node_t *next_chunk_ptr = wr_handle->rw_pos.chunk_node_ptr;
      _circ_buf_next_chunk_node(circ_buf_ptr, &next_chunk_ptr);

      // Update chunk number.
      circ_buf_ptr->num_chunks--;

      // Update the current state of circular buffer in bytes.
      circ_buf_ptr->circ_buf_size_bytes -= cur_wr_chunk_ptr->num_frames * cur_wr_chunk_ptr->frame_size;

      // Frees chunk metadata, mf ptr, and also chunk_ptr memory
      _circ_buf_free_chunk(circ_buf_ptr, cur_wr_chunk_ptr);

      spf_list_find_delete_node(&circ_buf_ptr->head_chunk_ptr, cur_wr_chunk_ptr, TRUE);

      // if this is the only chunk getting in the circular buffer, no chunks will be left
      // hence check if any chunks are present before setting writers position.
      if (circ_buf_ptr->head_chunk_ptr)
      {
         wr_handle->rw_pos.chunk_node_ptr = next_chunk_ptr;
         wr_handle->rw_pos.frame_offset   = 0;
         wr_handle->rw_pos.frame_position = 0;
      }
      else
      {
         memset(&wr_handle->rw_pos, 0, sizeof(spf_circ_buf_position_t));
      }
   }

   // even at this point if read pointer is sill on deprecrated chunk
   // it means that its one chunk case, because if there is only one chunk move the reader to next chunk will still
   // point to the same chunk. In that case initalize the readers position to writers.
   // It can also mean that reader is empty. In this case we also need to sync to the writer position.
   for (spf_list_node_t *rd_client_list_ptr = wr_handle->circ_buf_ptr->rd_client_list_ptr; (NULL != rd_client_list_ptr);
        LIST_ADVANCE(rd_client_list_ptr))
   {
      spf_circ_buf_client_t *cur_rd_client_ptr = (spf_circ_buf_client_t *)rd_client_list_ptr->obj_ptr;

      // move to next reader if the pointer is NULL.
      if (!cur_rd_client_ptr->rw_pos.chunk_node_ptr)
      {
         continue;
      }

      // check if the chunk pointer is same.
      if (cur_rd_client_ptr->rw_pos.chunk_node_ptr != (void *)cur_wr_chunk_node_ptr)
      {
         // not same chunk.
         continue;
      }

      cur_rd_client_ptr->rw_pos.chunk_node_ptr = wr_handle->rw_pos.chunk_node_ptr;
      cur_rd_client_ptr->rw_pos.frame_offset   = wr_handle->rw_pos.frame_offset;
      cur_rd_client_ptr->rw_pos.frame_position = wr_handle->rw_pos.frame_position;
   }

   return result;
}

/*
 * Get output media format, data read following this get param is expected to be of this media format.
 */
spf_circ_buf_result_t spf_circ_buf_get_media_format(spf_circ_buf_client_t *rd_client_ptr, capi_media_fmt_v2_t *mf_ptr)
{
   mf_ptr->header.format_header.data_format = CAPI_FIXED_POINT;

   mf_ptr->format.bits_per_sample   = rd_client_ptr->operating_mf->bits_per_sample;
   mf_ptr->format.data_is_signed    = rd_client_ptr->operating_mf->data_is_signed;
   mf_ptr->format.data_interleaving = CAPI_DEINTERLEAVED_UNPACKED;
   mf_ptr->format.num_channels      = rd_client_ptr->operating_mf->num_channels;
   mf_ptr->format.q_factor          = rd_client_ptr->operating_mf->q_factor;
   mf_ptr->format.sampling_rate     = rd_client_ptr->operating_mf->sampling_rate;

   for (uint32_t ch_idx = 0; ch_idx < rd_client_ptr->operating_mf->num_channels; ch_idx++)
   {
      mf_ptr->channel_type[ch_idx] = rd_client_ptr->operating_mf->channel_type[ch_idx];
   }

   return SPF_CIRCBUF_SUCCESS;
}

spf_circ_buf_result_t spf_circ_buf_driver_is_buffer_full(spf_circ_buf_client_t *rd_hdl_ptr, bool_t *is_full_ptr)
{
   bool_t is_full = FALSE;
   if (!rd_hdl_ptr || !is_full_ptr || !(rd_hdl_ptr->circ_buf_ptr))
   {
      return SPF_CIRCBUF_FAIL;
   }

   // Default to is_full is FALSE.

   spf_circ_buf_client_t *wr_hdl_ptr = rd_hdl_ptr->circ_buf_ptr->wr_client_ptr;

   if (!wr_hdl_ptr)
   {
      // Can't be full if there are no writers.
      *is_full_ptr = is_full;
      return SPF_CIRCBUF_SUCCESS;
   }

   if (wr_hdl_ptr->rw_pos.chunk_node_ptr == rd_hdl_ptr->rw_pos.chunk_node_ptr &&
       wr_hdl_ptr->rw_pos.frame_position == rd_hdl_ptr->rw_pos.frame_position)
   {
      // Read/Write pointers will be equal if the buffer is full or empty. If the buffer is empty, unread bytes would be
      // 0.
      is_full = (0 != rd_hdl_ptr->unread_bytes);
   }

   *is_full_ptr = is_full;
   return SPF_CIRCBUF_SUCCESS;
}

spf_circ_buf_result_t spf_circ_buf_get_unread_bytes(spf_circ_buf_client_t *rd_handle, uint32_t *unread_bytes)
{
   if (!rd_handle || !unread_bytes)
   {
      return SPF_CIRCBUF_FAIL;
   }

   *unread_bytes = rd_handle->unread_bytes;
   return SPF_CIRCBUF_SUCCESS;
}

/* Init raw circular buffer based on input parameters, sizes and MTU length */
spf_circ_buf_result_t spf_circ_buf_raw_init(spf_circ_buf_raw_t **              circ_buf_raw_pptr,
                                            spf_circ_buf_raw_alloc_inp_args_t *inp_args)
{

   if ((NULL == circ_buf_raw_pptr) || (0 == inp_args->preferred_chunk_size))
   {
#ifdef DEBUG_CIRC_BUF_UTILS
      AR_MSG(DBG_ERROR_PRIO, "init: Bad init params. Allocation failed.");
#endif
      return SPF_CIRCBUF_FAIL;
   }

   spf_circ_buf_raw_t *circ_buf_raw_ptr =
      (spf_circ_buf_raw_t *)posal_memory_malloc(sizeof(spf_circ_buf_raw_t), inp_args->heap_id);
   if (!circ_buf_raw_ptr)
   {
#ifdef DEBUG_CIRC_BUF_UTILS
      AR_MSG(DBG_ERROR_PRIO, "init: Allocation failed.");
#endif
      return SPF_CIRCBUF_FAIL;
   }
   memset(circ_buf_raw_ptr, 0, sizeof(spf_circ_buf_raw_t));

   *circ_buf_raw_pptr = circ_buf_raw_ptr;

   if (circ_buf_raw_ptr->head_chunk_ptr || circ_buf_raw_ptr->rd_client_list_ptr || circ_buf_raw_ptr->wr_raw_client_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "init: Allocation failed. Buffer already intialized.");
      return SPF_CIRCBUF_FAIL;
   }

   memset(circ_buf_raw_ptr, 0, sizeof(spf_circ_buf_raw_t));

   /* Initialize the read and write chunks to the head of the chunk list */
   circ_buf_raw_ptr->heap_id                      = inp_args->heap_id;
   circ_buf_raw_ptr->id                           = inp_args->buf_id;
   circ_buf_raw_ptr->circ_buf_raw_size_max        = inp_args->max_raw_circ_buf_size_bytes;
   circ_buf_raw_ptr->max_raw_circ_buf_bytes_limit = inp_args->max_raw_circ_buf_bytes_limit;
   circ_buf_raw_ptr->num_frames_max               = inp_args->max_num_frames;
   circ_buf_raw_ptr->write_byte_counter           = 0;
   circ_buf_raw_ptr->num_read_clients             = 0;
   circ_buf_raw_ptr->wr_raw_client_ptr            = NULL;
   circ_buf_raw_ptr->metadata_handler             = inp_args->metadata_handler;

   // cache preferred chunk size and compute actual chunk size.
   circ_buf_raw_ptr->preferred_chunk_size = inp_args->preferred_chunk_size;

   AR_MSG(DBG_HIGH_PRIO,
          "Initialized preferred_chunk_size: %lu num_chunk = %u",
          circ_buf_raw_ptr->preferred_chunk_size,
          circ_buf_raw_ptr->num_chunks);

   return SPF_CIRCBUF_SUCCESS;
}

/* Register and reset raw data reader client */
spf_circ_buf_result_t spf_circ_buf_raw_register_reader_client(spf_circ_buf_raw_t *        circ_buf_raw_ptr,
                                                              uint32_t                    req_base_buffer_size_bytes,
                                                              spf_circ_buf_raw_client_t **client_hdl_pptr)
{
   // Return if the pointers are null or if the num_write_clients is
   if ((NULL == circ_buf_raw_ptr) || (NULL == client_hdl_pptr))
   {
#ifdef DEBUG_CIRC_BUF_UTILS
      AR_MSG(DBG_HIGH_PRIO,
             "register_read_client: Failed to register. circ_buf_struct_ptr = 0x%lx, client_hdl_pptr=0x%x ",
             circ_buf_raw_ptr,
             client_hdl_pptr);
#endif

      return SPF_CIRCBUF_FAIL;
   }

   spf_circ_buf_raw_client_t *client_hdl_ptr =
      (spf_circ_buf_raw_client_t *)posal_memory_malloc(sizeof(spf_circ_buf_raw_client_t), circ_buf_raw_ptr->heap_id);
   if (!client_hdl_ptr)
   {
#ifdef DEBUG_CIRC_BUF_UTILS
      AR_MSG(DBG_ERROR_PRIO, "register_read_client: Allocation failed.");
#endif
      return SPF_CIRCBUF_FAIL;
   }
   memset(client_hdl_ptr, 0, sizeof(spf_circ_buf_raw_client_t));

   *client_hdl_pptr = client_hdl_ptr;

   // Init base params.
   client_hdl_ptr->is_read_client        = TRUE; // is read client
   client_hdl_ptr->circ_buf_raw_ptr      = circ_buf_raw_ptr;
   client_hdl_ptr->init_base_buffer_size = req_base_buffer_size_bytes;

   // Add the client to the client list.
   spf_list_insert_tail(&circ_buf_raw_ptr->rd_client_list_ptr,
                        client_hdl_ptr,
                        circ_buf_raw_ptr->heap_id,
                        TRUE /* use_pool*/);
   circ_buf_raw_ptr->num_read_clients++;

   // Resize the circular buffer based on new client request.
   _circ_buf_raw_client_resize(circ_buf_raw_ptr);

   // Resets the read/write position of the client.
   _circ_buf_raw_read_client_reset(client_hdl_ptr);

#ifdef DEBUG_CIRC_BUF_UTILS
   AR_MSG(DBG_HIGH_PRIO,
          "register_read_client: Done. req_alloc_size = %lu, circ buf sz = %lu, num_read_clients= %u ",
          req_base_buffer_size_bytes,
          circ_buf_raw_ptr->circ_buf_size_bytes,
          circ_buf_raw_ptr->num_read_clients);
#endif

   return SPF_CIRCBUF_SUCCESS;
}

/* Register and reset raw data writer client */
spf_circ_buf_result_t spf_circ_buf_raw_register_writer_client(spf_circ_buf_raw_t *        circ_buf_raw_ptr,
                                                              uint32_t                    req_base_buffer_size_bytes,
                                                              spf_circ_buf_raw_client_t **client_hdl_pptr)
{
   /* Return if the pointers are null or if the num_write_clients is */
   if ((NULL == circ_buf_raw_ptr) || (NULL == client_hdl_pptr))
   {
#ifdef DEBUG_CIRC_BUF_UTILS
      AR_MSG(DBG_HIGH_PRIO,
             "register_write_client: Failed to register. circ_buf_struct_ptr = 0x%lx, client_hdl_pptr=0x%x ",
             circ_buf_raw_ptr,
             client_hdl_pptr);
#endif

      return SPF_CIRCBUF_FAIL;
   }

   /* if a writer is already registered error out.*/
   if (circ_buf_raw_ptr->wr_raw_client_ptr)
   {
#ifdef DEBUG_CIRC_BUF_UTILS
      AR_MSG(DBG_HIGH_PRIO,
             "register_write_client: buf_idx=0x%x Failed to register to writer. Already a writer client exist.",
             circ_buf_raw_ptr->id);
#endif
      return SPF_CIRCBUF_FAIL;
   }

   spf_circ_buf_raw_client_t *client_hdl_ptr =
      (spf_circ_buf_raw_client_t *)posal_memory_malloc(sizeof(spf_circ_buf_raw_client_t), circ_buf_raw_ptr->heap_id);
   if (!client_hdl_ptr)
   {
#ifdef DEBUG_CIRC_BUF_UTILS
      AR_MSG(DBG_ERROR_PRIO, "register_write_client: Allocation failed.");
#endif
      return SPF_CIRCBUF_FAIL;
   }
   memset(client_hdl_ptr, 0, sizeof(spf_circ_buf_raw_client_t));

   *client_hdl_pptr = client_hdl_ptr;

   // Init base params.
   client_hdl_ptr->is_read_client        = FALSE; //
   client_hdl_ptr->circ_buf_raw_ptr      = circ_buf_raw_ptr;
   client_hdl_ptr->init_base_buffer_size = req_base_buffer_size_bytes;

   circ_buf_raw_ptr->wr_raw_client_ptr = client_hdl_ptr;

   // Create buffer with (base buffer size + write_client_requested_buf_size) + max_of(read_client_requests)
   _circ_buf_raw_client_resize(circ_buf_raw_ptr);

   // Resets the read/write position of the client.
   _circ_buf_raw_write_client_reset(client_hdl_ptr);

#ifdef DEBUG_CIRC_BUF_UTILS
   AR_MSG(DBG_HIGH_PRIO,
          "register_write_raw_client: circ buf sz = %lu, req_base_buffer_size = %lu",
          circ_buf_raw_ptr->circ_buf_size_bytes,
          req_base_buffer_size_bytes);
#endif

   return SPF_CIRCBUF_SUCCESS;
}

/* Deinit and destroy raw circular buffer and clients*/
spf_circ_buf_result_t spf_circ_buf_raw_deinit(spf_circ_buf_raw_t **circ_buf_raw_pptr)
{
   spf_circ_buf_raw_t *circ_buf_raw_ptr = *circ_buf_raw_pptr;

   AR_MSG(DBG_MED_PRIO, "deinit circ raw buf ptr %u", circ_buf_raw_ptr);

   // Iteraate through each chunk and destroy the chunk
   spf_circ_buf_chunk_t *cur_chunk_ptr =
      (spf_circ_buf_chunk_t *)spf_list_pop_head(&circ_buf_raw_ptr->head_chunk_ptr, TRUE /* pool_used */);

   AR_MSG(DBG_MED_PRIO, "free chunk");
   while (cur_chunk_ptr)
   {
      _circ_buf_raw_free_chunk(circ_buf_raw_ptr, cur_chunk_ptr);

      // pop next chunk pointer
      cur_chunk_ptr =
         (spf_circ_buf_chunk_t *)spf_list_pop_head(&circ_buf_raw_ptr->head_chunk_ptr, TRUE /* pool_used */);
   }
   AR_MSG(DBG_MED_PRIO, "freed_chunk");

   // Reset the write client handle.
   AR_MSG(DBG_MED_PRIO, "free write client");
   if (circ_buf_raw_ptr->wr_raw_client_ptr)
   {
      _circ_buf_raw_destroy_client_hdl(circ_buf_raw_ptr, &circ_buf_raw_ptr->wr_raw_client_ptr);
   }
   AR_MSG(DBG_MED_PRIO, "freed write client");

   // Reset all the read client handles.
   spf_circ_buf_raw_client_t *temp_rd_client_ptr =
      (spf_circ_buf_raw_client_t *)spf_list_pop_head(&circ_buf_raw_ptr->rd_client_list_ptr, TRUE /* pool_used */);
   AR_MSG(DBG_MED_PRIO, "free read client");
   while (temp_rd_client_ptr)
   {
      // destory client handle
      _circ_buf_raw_destroy_client_hdl(circ_buf_raw_ptr, &temp_rd_client_ptr);

      temp_rd_client_ptr =
         (spf_circ_buf_raw_client_t *)spf_list_pop_head(&circ_buf_raw_ptr->rd_client_list_ptr, TRUE /* pool_used */);
   }
   AR_MSG(DBG_MED_PRIO, "freed read client");

   AR_MSG(DBG_MED_PRIO, "free read client list");
   // Reset all the client node handles.
   if (NULL != circ_buf_raw_ptr->rd_client_list_ptr)
   {
      spf_list_delete_list_and_free_objs(&circ_buf_raw_ptr->rd_client_list_ptr, TRUE /* pool_used */);
   }
   AR_MSG(DBG_MED_PRIO, "freed read client list");

   AR_MSG(DBG_MED_PRIO, "free circ raw buf %u", circ_buf_raw_ptr);
   memset(circ_buf_raw_ptr, 0, sizeof(spf_circ_buf_raw_t));

   AR_MSG(DBG_MED_PRIO, "memset circ raw buf");
   posal_memory_free(circ_buf_raw_ptr);
   AR_MSG(DBG_MED_PRIO, "freed circ raw buf");

   *circ_buf_raw_pptr = NULL;

   return SPF_CIRCBUF_SUCCESS;
}

/* Memset circular raw buffer */
spf_circ_buf_result_t spf_circ_buf_raw_memset(spf_circ_buf_raw_client_t *wr_client_ptr,
                                              bool_t                     allow_overflow,
                                              uint32_t                   num_bytes,
                                              uint32_t                   memset_value)
{
   return _circ_buf_raw_write_data(wr_client_ptr, NULL, allow_overflow, &memset_value, &num_bytes);
}

/* Write one MTU frame into buffer from input at a time - currently used */
spf_circ_buf_result_t spf_circ_buf_raw_write_one_frame(spf_circ_buf_raw_client_t *wr_handle,
                                                       capi_stream_data_t *       sdata_ptr,
                                                       bool_t                     allow_overflow)
{
   return _circ_buf_raw_write_one_frame(wr_handle, sdata_ptr, allow_overflow, NULL);
}

/* Validate and set media format of the circular buffer */
spf_circ_buf_result_t spf_circ_buf_raw_set_media_format(spf_circ_buf_raw_client_t *wr_raw_handle,
                                                        capi_cmn_raw_media_fmt_t * inp_mf)
{
   spf_circ_buf_result_t result = SPF_CIRCBUF_SUCCESS;

   if (!wr_raw_handle)
   {
      AR_MSG(DBG_ERROR_PRIO, "Null write pointer ");
      return SPF_CIRCBUF_FAIL;
   }

   /* validate and cache media format  */
   if (!_circ_buf_raw_is_valid_mf(wr_raw_handle, inp_mf))
   {
      AR_MSG(DBG_HIGH_PRIO, "Invalid input media format ");
      return SPF_CIRCBUF_FAIL;
   }

   return result;
}

/* Used to create the circular buffer for the first time. Later on it is expanded with expand requests */
spf_circ_buf_result_t spf_circ_buf_raw_resize(spf_circ_buf_raw_client_t *wr_raw_handle, uint32_t raw_frame_len)
{
   spf_circ_buf_result_t result = SPF_CIRCBUF_SUCCESS;

   if (!wr_raw_handle)
   {
      AR_MSG(DBG_ERROR_PRIO, "Null write pointer ");
      return SPF_CIRCBUF_FAIL;
   }

   spf_circ_buf_raw_t *circ_buf_raw_ptr = wr_raw_handle->circ_buf_raw_ptr;

   if (raw_frame_len == circ_buf_raw_ptr->wr_raw_client_ptr->raw_frame_len)
   {
      AR_MSG(DBG_ERROR_PRIO, "Nothing to do. ");
      return SPF_CIRCBUF_SUCCESS;
   }

   circ_buf_raw_ptr->wr_raw_client_ptr->raw_frame_len = raw_frame_len;

   /* At this point we should have gotten the size for the buffer as well
    * as the raw frame length so we can create the raw circular buffer. */
   uint32_t frame_data_size_in_bytes    = 0;
   uint32_t actual_chunk_size           = 0;
   uint32_t actual_num_frames_per_chunk = 0;
   _circ_buf_raw_compute_chunk_info(circ_buf_raw_ptr,
                                    circ_buf_raw_ptr->circ_buf_size_bytes,
                                    &frame_data_size_in_bytes,
                                    &actual_chunk_size,
                                    &actual_num_frames_per_chunk);

   /* Creates the circular buffer */
   /* In add chunks we also reset the write and read clients
    * for the first time the buffer is create */
   result = _circ_buf_raw_client_resize(circ_buf_raw_ptr);

   return result;
}

/* Get the number if unread bytes (actual data len) / unread bytes (max data len) */
spf_circ_buf_result_t spf_circ_buf_raw_get_unread_bytes(spf_circ_buf_raw_client_t *rd_handle,
                                                        uint32_t *                 unread_bytes,
                                                        uint32_t *                 unread_bytes_max)
{
   if (!rd_handle || !unread_bytes || !unread_bytes_max)
   {
      return SPF_CIRCBUF_FAIL;
   }

   *unread_bytes     = rd_handle->unread_bytes;
   *unread_bytes_max = rd_handle->unread_bytes_max;
   return SPF_CIRCBUF_SUCCESS;
}
