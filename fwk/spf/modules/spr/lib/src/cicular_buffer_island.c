/**
 *   \file cicular_buffer.c
 *   \brief
 *        This file contains implementation of circular buffering
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "circular_buffer.h"
#include "ar_msg.h"
#include "spf_list_utils.h"

/*==============================================================================
   Local utility Functions
==============================================================================*/

//#define DEBUG_CIRC_BUF_UTILS

#ifndef ALIGN_8_BYTES
#define ALIGN_8_BYTES(a) (((a) & (~(uint32_t)0x7)) + 8)
#endif

// Write data as requested by the write client.
static circbuf_result_t _circ_buf_write_util(circ_buf_client_t *wr_client_ptr,
                                             int8_t *           inp_ptr,
                                             uint32_t           write_value,
                                             uint32_t           bytes_to_write,
                                             uint32_t           is_valid_timestamp,
                                             int64_t            timestamp);

// Utility to add chunks based on the increased buffer size.
static circbuf_result_t _circ_buf_add_chunks(circ_buf_t *circ_buf_ptr, uint32_t new_buf_size);

// Utility to remove chunks based on the decreased buffer size.
static circbuf_result_t _circ_buf_remove_chunks(circ_buf_t *circ_buf_ptr, uint32_t new_buf_size);

// Utility to resize the buffer based on the update read clients.
static circbuf_result_t _circ_buf_client_resize(circ_buf_t *circ_buf_ptr);

// Function detects the overflow in the read clients of the buffer and adjusts the buffer accordingly
static circbuf_result_t _circ_buf_detect_and_handle_overflow(circ_buf_t *circ_buf_ptr, uint32_t bytes_to_write);

// Utility to advance the given chunk node to the next chunk in the list. It wraps around to the head
// chunk if the input chunk is the tail of the list.
static circbuf_result_t _circ_buf_next_chunk_node(circ_buf_t *circ_buf_ptr, spf_list_node_t **chunk_ptr_ptr);

// Utility to reset the client r/w chunk position based on current buffer state.
static circbuf_result_t _circ_buf_read_client_reset(circ_buf_client_t *rd_client_ptr);

// Utility to reset the client write chunk position based on current buffer state.
static circbuf_result_t _circ_buf_write_client_reset(circ_buf_client_t *wr_client_ptr);

// Helper function to advance clients read/write position forward by given bytes.
static circbuf_result_t _circ_buf_position_shift_forward(circ_buf_t *      circ_buf_ptr,
                                                         spf_list_node_t **rd_chunk_list_pptr,
                                                         uint32_t *        rw_chunk_offset_ptr,
                                                         uint32_t          bytes_to_advance);

#ifdef DEBUG_CIRC_BUF_UTILS
static void print_client_chunk_positions(circ_buf_t *circ_buf_ptr);
#endif

// Utility to check if the buffer is corrupted.
// static circbuf_result_t _circ_buf_check_if_corrupted(circ_buf_t *frag_circ_buf_ptr);

/*==============================================================================
   Local Function Implementation
==============================================================================*/

/*==============================================================================
   Public Function Implementation
==============================================================================*/

/* Initializes the Circular buffer */
circbuf_result_t circ_buf_alloc(circ_buf_t *  circ_buf_ptr,
                                uint32_t      circ_buf_size,
                                uint32_t      preferred_chunk_size,
                                uint32_t      ch_id, // TODO: buf id everywhere
                                POSAL_HEAP_ID heap_id)
{
   if ((NULL == circ_buf_ptr) || (0 == preferred_chunk_size))
   {
#ifdef DEBUG_CIRC_BUF_UTILS
      AR_MSG_ISLAND(DBG_ERROR_PRIO, "CIRC_BUF: Bad init params. Allocation failed.");
#endif
      return CIRCBUF_FAIL;
   }

   // TODO: THis check is not needed to be removed later.
   //   if (circ_buf_ptr->circ_buf_size != circ_buf_size || circ_buf_ptr->preferred_chunk_size != preferred_chunk_size)
   //   {
   //      /* Free the existing Buffer */
   //      circ_buf_free(circ_buf_ptr);
   //   }
   memset(circ_buf_ptr, 0, sizeof(circ_buf_t));

   /* Initialize the read and write chunks to the head of the chunk list */
   circ_buf_ptr->preferred_chunk_size = preferred_chunk_size;
   circ_buf_ptr->heap_id              = heap_id;
   circ_buf_ptr->id                   = ch_id;

   circ_buf_ptr->write_byte_counter = 0;
   circ_buf_ptr->num_read_clients   = 0;
   circ_buf_ptr->wr_client_ptr      = NULL;
   circ_buf_ptr->max_req_buf_resize = 0;

   // allocate chunks based on the requested circular buffer size.
   if (CIRCBUF_SUCCESS != _circ_buf_add_chunks(circ_buf_ptr, circ_buf_size))
   {
#ifdef DEBUG_CIRC_BUF_UTILS
      AR_MSG_ISLAND(DBG_ERROR_PRIO, "CIRC_BUF: Chunk allocation failed.");
#endif
      return CIRCBUF_FAIL;
   }

#ifdef DEBUG_CIRC_BUF_UTILS
   AR_MSG_ISLAND(DBG_HIGH_PRIO,
          "CIRC_BUF: Allocated Circ buf size = %lu, preferred_chunk_size: %lu, num_chunks: %lu",
          circ_buf_ptr->circ_buf_size,
          circ_buf_ptr->preferred_chunk_size,
          circ_buf_ptr->num_chunks);
#endif

   return CIRCBUF_SUCCESS;
}

circbuf_result_t circ_buf_register_client(circ_buf_t *       circ_buf_struct_ptr,
                                          bool_t             is_read_client,
                                          uint32_t           req_base_buffer_size,
                                          circ_buf_client_t *client_hdl_ptr)
{
   // Return if the pointers are null or if the num_write_clients is
   if ((NULL == circ_buf_struct_ptr) || (NULL == client_hdl_ptr))
   {
#ifdef DEBUG_CIRC_BUF_UTILS
      AR_MSG_ISLAND(DBG_HIGH_PRIO,
             "circ_buf_register_client: Failed to register. is_read_client = %d, wr_client_ptr=0x%x ",
             is_read_client,
             circ_buf_struct_ptr->wr_client_ptr);
#endif

      return CIRCBUF_FAIL;
   }

   if (!is_read_client && (circ_buf_struct_ptr->wr_client_ptr))
   {
#ifdef DEBUG_CIRC_BUF_UTILS
      AR_MSG_ISLAND(DBG_HIGH_PRIO, "circ_buf_register_client: Failed to register writer client");
#endif
      return CIRCBUF_FAIL;
   }

   // Init base params.
   client_hdl_ptr->is_read_client       = is_read_client;
   client_hdl_ptr->circ_buf_ptr         = circ_buf_struct_ptr;
   client_hdl_ptr->req_base_buffer_size = req_base_buffer_size;
   client_hdl_ptr->req_buffer_resize    = 0;

   if (is_read_client)
   {
      // Add the client to the client list.
      spf_list_insert_tail(&circ_buf_struct_ptr->rd_client_list_ptr,
                           client_hdl_ptr,
                           circ_buf_struct_ptr->heap_id,
                           TRUE /* use_pool*/);
      circ_buf_struct_ptr->num_read_clients++;

      // Resize the circular buffer based on new client request.
      _circ_buf_client_resize(circ_buf_struct_ptr);

      // Resets the read/write position of the client.
      _circ_buf_read_client_reset(client_hdl_ptr);
#ifdef DEBUG_CIRC_BUF_UTILS
      print_client_chunk_positions(circ_buf_struct_ptr);
#endif
   }
   else
   {
      circ_buf_struct_ptr->wr_client_ptr = client_hdl_ptr;

      // Resets the read/write position of the client.
      _circ_buf_write_client_reset(client_hdl_ptr);
   }

#ifdef DEBUG_CIRC_BUF_UTILS
   AR_MSG_ISLAND(DBG_HIGH_PRIO,
          "circ_buf_register_client: Done. is_read_client = %d, req_alloc_size = %lu, circ buf sz = %lu, "
          "num_read_clients= %u ",
          is_read_client,
          req_base_buffer_size,
          circ_buf_struct_ptr->circ_buf_size,
          circ_buf_struct_ptr->num_read_clients);
#endif

   return CIRCBUF_SUCCESS;
}

circbuf_result_t circ_buf_read_client_resize(circ_buf_client_t *rd_client_ptr,
                                             uint32_t           req_buffer_resize,
                                             uint32_t           is_register)
{
   if (NULL == rd_client_ptr)
   {
      return CIRCBUF_FAIL;
   }

   if (is_register)
   {
      // Resize request, appends to the curretn size. TODO: Revisit check if it has to be updated or appended.
      rd_client_ptr->req_buffer_resize = req_buffer_resize;
   }
   else
   {
      rd_client_ptr->req_buffer_resize = 0;
   }

#ifdef DEBUG_CIRC_BUF_UTILS
   AR_MSG_ISLAND(DBG_HIGH_PRIO,
          "CIRC_BUF: Received client resize request. buf = 0x%x, alloc_request_size: %u, is_register: %d",
          rd_client_ptr->circ_buf_ptr,
          req_buffer_resize,
          is_register);
#endif

   // Request to resize the circ buf
   return _circ_buf_client_resize(rd_client_ptr->circ_buf_ptr);
}

circbuf_result_t circ_buf_deregister_client(circ_buf_client_t *client_hdl_ptr)
{
   if ((NULL == client_hdl_ptr) || (NULL == client_hdl_ptr->circ_buf_ptr))
   {
#ifdef DEBUG_CIRC_BUF_UTILS
      AR_MSG_ISLAND(DBG_HIGH_PRIO, "circ_buf_deregister_client: null arguments. Failed to De-register.");
#endif
      return CIRCBUF_FAIL;
   }

   circ_buf_t *circ_buf_struct_ptr = client_hdl_ptr->circ_buf_ptr;
   bool_t      is_read_client      = client_hdl_ptr->is_read_client;

   // Resize the buffer based on the new client list.
   if (is_read_client)
   {
      // Delete only the list node corresponding to the client.
      // Client memory must not be deallocated though.
      spf_list_find_delete_node(&circ_buf_struct_ptr->rd_client_list_ptr, client_hdl_ptr, TRUE /*pool_used*/);
      circ_buf_struct_ptr->num_read_clients--;

      // Resize the buffer based on the updated client list.
      _circ_buf_client_resize(circ_buf_struct_ptr);
   }
   else
   {
      circ_buf_struct_ptr->wr_client_ptr = NULL;
   }

#ifdef DEBUG_CIRC_BUF_UTILS
   AR_MSG_ISLAND(DBG_HIGH_PRIO,
          "circ_buf_deregister_client: Done. is_read_client = %d, circ buf size = %d, "
          "num_read_clients=%lu ",
          is_read_client,
          circ_buf_struct_ptr->circ_buf_size,
          circ_buf_struct_ptr->num_read_clients);
#endif

   // Reset the client pointer.
   memset(client_hdl_ptr, 0, sizeof(circ_buf_client_t));

   return CIRCBUF_SUCCESS;
}

/*
 * Read samples from the Circular buffer
 * Full documentation in circ_buf_utils.h
 */
circbuf_result_t circ_buf_read(circ_buf_client_t *rd_client_ptr,
                               int8_t *           out_ptr,
                               uint32_t           bytes_to_read,
                               uint32_t *         num_bytes_read_ptr)
{
   if ((NULL == out_ptr) || (NULL == rd_client_ptr->rw_chunk_node_ptr) || (NULL == rd_client_ptr->circ_buf_ptr) ||
       (NULL == num_bytes_read_ptr))
   {
#ifdef DEBUG_CIRC_BUF_UTILS
      AR_MSG_ISLAND(DBG_ERROR_PRIO, "CIRC_BUF: Read failed. Invalid input params. Rcvd bytes to read = %u", bytes_to_read);
#endif
      return CIRCBUF_FAIL;
   }

   circbuf_result_t result = CIRCBUF_SUCCESS;

// TODO: revisit.
//   if (CIRCBUF_FAIL == circ_buf_check_if_corrupted(rd_client_ptr->circ_buf_handle))
//   {
//#ifdef DEBUG_CIRC_BUF_UTILS
//      AR_MSG_ISLAND(DBG_ERROR_PRIO, "Circ buf corrupted");
//#endif
//      return CIRCBUF_FAIL;
//   }

#ifdef DEBUG_CIRC_BUF_UTILS
   AR_MSG_ISLAND(DBG_HIGH_PRIO,
          "CIRC_BUF: Circ Buf read, bytes_to_read= %u ; un_read_bytes = %u",
          bytes_to_read,
          rd_client_ptr->unread_bytes);
#endif

   if (0 == rd_client_ptr->unread_bytes) // TODO: revisit - check if un read bytes is correct.
   {
#ifdef DEBUG_CIRC_BUF_UTILS
      AR_MSG_ISLAND(DBG_ERROR_PRIO,
             "Circ Buf underrun, bytes to read = %u, unread bytes = %u",
             bytes_to_read,
             rd_client_ptr->unread_bytes);

      *num_bytes_read_ptr = 0;
#endif
      return CIRCBUF_UNDERRUN;
   }

   // For SPR use cases, when partial data is encountered in the circular buffer, do not drop it.
   // Drain the circular buffer and then append trailing zeroes to make one full frame in the CAPI.
   // In this case, do not set the erasure flag on the output. TODO: erasure as MD instead of flag?
   //
   // The decision to not drop the partial data is based on the following scenarios
   // 1. When SPR is used in compressed cases, the partial data might contain frame info.
   // 2. The data might correspond to a ramp down scenario and cannot be dropped.
   if (bytes_to_read > rd_client_ptr->unread_bytes) // TODO: revisit - check if un read bytes is correct.
   {
#ifdef DEBUG_CIRC_BUF_UTILS
      AR_MSG_ISLAND(DBG_ERROR_PRIO,
             "Circ Buf partial underrun, bytes to read = %u, unread bytes = %u",
             bytes_to_read,
             rd_client_ptr->unread_bytes);

#endif
      // Drain whatever is possible
      bytes_to_read = rd_client_ptr->unread_bytes;
      result        = CIRCBUF_UNDERRUN;
   }

   uint32_t          counter             = bytes_to_read;
   spf_list_node_t **rd_chunk_ptr_ptr    = &rd_client_ptr->rw_chunk_node_ptr;
   uint32_t *        rw_chunk_offset_ptr = &rd_client_ptr->rw_chunk_offset;
   while (counter > 0)
   {
      chunk_buffer_t *chunk_ptr = (chunk_buffer_t *)(*rd_chunk_ptr_ptr)->obj_ptr;

#ifdef DEBUG_CIRC_BUF_UTILS
      AR_MSG_ISLAND(DBG_HIGH_PRIO,
             "circ_buf_read: buf_id: 0x%x, chunk:%d, chunk_ptr: 0x%x, offset: %lu, buf_size: %lu, counter: %lu",
             rd_client_ptr->circ_buf_ptr->id,
             chunk_ptr->id,
             chunk_ptr,
             rd_client_ptr->rw_chunk_offset,
             chunk_ptr->size,
             counter);
#endif

      /* If the bytes to read is less than the remaining bytes in the chunk */
      if (counter < (chunk_ptr->size - *rw_chunk_offset_ptr))
      {
         /* Copy the less than chunk size counter */
         *rw_chunk_offset_ptr +=
            memscpy(&out_ptr[bytes_to_read - counter], counter, chunk_ptr->buffer_ptr + *rw_chunk_offset_ptr, counter);
         counter = 0;
      }
      else
      {
         /* Write after the previously read bytes (bytes_to_read - counter) */
         /* Assumes that out_ptr is trusted input with sufficient length */
         counter -= memscpy(&out_ptr[bytes_to_read - counter],
                            counter,
                            chunk_ptr->buffer_ptr + *rw_chunk_offset_ptr,
                            chunk_ptr->size - *rw_chunk_offset_ptr);

         /* Advance to next chunk */
         _circ_buf_next_chunk_node(rd_client_ptr->circ_buf_ptr, rd_chunk_ptr_ptr);
         *rw_chunk_offset_ptr = 0;
      }
   }

   // Update the client's unread bytes counter.
   rd_client_ptr->unread_bytes -= bytes_to_read;
   *num_bytes_read_ptr = bytes_to_read;

   return result;
}

/*
 * Write samples to the Circular buffer
 * Note: Since there is no space in circular buffer for writing samples_to_write
 * number of samples, create space by flushing samples_to_write-free_samples
 * number of samples
 * Full documentation in circ_buf_utils.h
 */
circbuf_result_t circ_buf_write(circ_buf_client_t *wr_client_ptr,
                                int8_t *           inp_ptr,
                                uint32_t           bytes_to_write,
                                uint32_t           is_valid_timestamp,
                                int64_t            timestamp)
{
   return _circ_buf_write_util(wr_client_ptr, inp_ptr, 0, bytes_to_write, is_valid_timestamp, timestamp);
}

/*
 * Helper function to advance clients read/write position forward by given bytes.
 * Full documentation in circ_buf_utils.h
 */
static circbuf_result_t _circ_buf_position_shift_forward(circ_buf_t *      circ_buf_ptr,
                                                         spf_list_node_t **rd_chunk_list_pptr,
                                                         uint32_t *        rw_chunk_offset_ptr,
                                                         uint32_t          bytes_to_advance)
{

   // Fail if bytes to shift forward is greater than circular buffer size.
   if ((bytes_to_advance > circ_buf_ptr->circ_buf_size) || (NULL == rd_chunk_list_pptr) ||
       (NULL == rw_chunk_offset_ptr))
   {
      return CIRCBUF_FAIL;
   }

   // No need to shift if bytes to advance is zero or circ buf size.
   if (bytes_to_advance == circ_buf_ptr->circ_buf_size || (0 == bytes_to_advance))
   {
      return CIRCBUF_SUCCESS;
   }

   uint32_t counter = bytes_to_advance;
   while (counter > 0)
   {
      chunk_buffer_t *chunk_ptr = (chunk_buffer_t *)(*rd_chunk_list_pptr)->obj_ptr;

      // If the bytes to shift is less than the remaining bytes in the chunk
      if (counter < (chunk_ptr->size - *rw_chunk_offset_ptr))
      {
         // Copy the less than chunk size counter
         *rw_chunk_offset_ptr += counter;
         counter = 0;
      }
      else
      {
         // Decrement the bytes in the chunk
         counter -= (chunk_ptr->size - *rw_chunk_offset_ptr);

         // Move to next chunk
         _circ_buf_next_chunk_node(circ_buf_ptr, rd_chunk_list_pptr);
         *rw_chunk_offset_ptr = 0;
      }
   }

   return CIRCBUF_SUCCESS;
}

/*
 * Helper function to advance read pointer by some amount
 * Full documentation in circ_buf_utils.h
 */
static circbuf_result_t _circ_buf_read_advance(circ_buf_client_t *rd_client_ptr, uint32_t bytes_to_advance)
{
   circbuf_result_t result = CIRCBUF_SUCCESS;

   if ((NULL == rd_client_ptr) || (NULL == rd_client_ptr->rw_chunk_node_ptr))
   {
#ifdef DEBUG_CIRC_BUF_UTILS
      AR_MSG_ISLAND(DBG_ERROR_PRIO,
             "_circ_buf_read_advance: Reader client position not intialized.",
             bytes_to_advance,
             rd_client_ptr->unread_bytes);
#endif
      return CIRCBUF_FAIL;
   }

   if (bytes_to_advance > rd_client_ptr->unread_bytes)
   {
#ifdef DEBUG_CIRC_BUF_UTILS
      AR_MSG_ISLAND(DBG_ERROR_PRIO,
             "_circ_buf_read_advance: circ_buf_read_advance fail, needed_size = %u, unread bytes = %u",
             bytes_to_advance,
             rd_client_ptr->unread_bytes);
#endif
      return CIRCBUF_FAIL;
   }

#ifdef DEBUG_CIRC_BUF_UTILS
   AR_MSG_ISLAND(DBG_HIGH_PRIO,
          "circ_buf_read_advance: needed_size = %u, unread bytes = %u",
          bytes_to_advance,
          rd_client_ptr->unread_bytes);
#endif

   result = _circ_buf_position_shift_forward(rd_client_ptr->circ_buf_ptr,
                                             &rd_client_ptr->rw_chunk_node_ptr,
                                             &rd_client_ptr->rw_chunk_offset,
                                             bytes_to_advance);
   if (CIRCBUF_FAIL == result)
   {
      return result;
   }

   // Update the client's unread bytes counter.
   rd_client_ptr->unread_bytes -= bytes_to_advance;

#ifdef DEBUG_CIRC_BUF_UTILS
   AR_MSG_ISLAND(DBG_HIGH_PRIO,
          "circ_buf_read_advance: Done. current read position is (chunk= 0x%x, offset= %u), unread_bytes=%u",
          rd_client_ptr->rw_chunk_node_ptr,
          rd_client_ptr->rw_chunk_offset,
          rd_client_ptr->unread_bytes);
#endif

   return result;
}

/*
 * Adjusts the read pointer
 * Full documentation in circ_buf_utils.h
 */
circbuf_result_t circ_buf_read_adjust(circ_buf_client_t *rd_client_ptr,
                                      uint32_t           read_offset,
                                      uint32_t *         actual_unread_bytes_ptr)
{
   if (FALSE == rd_client_ptr->is_read_client)
   {
      return CIRCBUF_FAIL;
   }

#ifdef DEBUG_CIRC_BUF_UTILS
   AR_MSG_ISLAND(DBG_HIGH_PRIO,
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
circbuf_result_t circ_buf_free(circ_buf_t *circ_buf_ptr)
{
   // Gk list delete frees all the list nodes and chunk buffer memory as well.
   if (NULL != circ_buf_ptr->head_chunk_ptr)
   {
      spf_list_delete_list_and_free_objs(&circ_buf_ptr->head_chunk_ptr, TRUE /* pool_used */);
   }

   // Reset the write client handle.
   if (circ_buf_ptr->wr_client_ptr)
   {
      memset(circ_buf_ptr->wr_client_ptr, 0, sizeof(circ_buf_client_t));
   }

   // Reset all the read client handles.
   circ_buf_client_t *temp_rd_client_ptr =
      (circ_buf_client_t *)spf_list_pop_head(&circ_buf_ptr->rd_client_list_ptr, TRUE /* pool_used */);
   while (temp_rd_client_ptr)
   {
      memset(temp_rd_client_ptr, 0, sizeof(circ_buf_client_t));

      temp_rd_client_ptr =
         (circ_buf_client_t *)spf_list_pop_head(&circ_buf_ptr->rd_client_list_ptr, TRUE /* pool_used */);
   }

   // Delete all the client node handles.
   if (NULL != circ_buf_ptr->rd_client_list_ptr)
   {
      spf_list_delete_list(&circ_buf_ptr->rd_client_list_ptr, TRUE /* pool_used */);
   }

   memset(circ_buf_ptr, 0, sizeof(circ_buf_t));

   return CIRCBUF_SUCCESS;
}

/*
 * TODO: Revisit.
 */
circbuf_result_t circ_buf_memset(circ_buf_t *frag_circ_buf_ptr, uint8_t write_value, uint32_t num_bytes)
{
   return _circ_buf_write_util(frag_circ_buf_ptr->wr_client_ptr, NULL, write_value, num_bytes, 0, 0);
}

/*
 * Resets the client read chunk position based on current buffer state.
 * Full documentation in circ_buf_utils.h
 */
static circbuf_result_t _circ_buf_read_client_reset(circ_buf_client_t *client_ptr)
{
   circbuf_result_t result       = CIRCBUF_SUCCESS;
   circ_buf_t *     circ_buf_ptr = client_ptr->circ_buf_ptr;
   // Set the read position same as write position
   if (client_ptr->is_read_client)
   {
      circ_buf_client_t *wr_client_ptr = circ_buf_ptr->wr_client_ptr;
      if (wr_client_ptr)
      {
         // Initialize the read position same as the current write position.
         client_ptr->rw_chunk_node_ptr = wr_client_ptr->rw_chunk_node_ptr;
         client_ptr->rw_chunk_offset   = wr_client_ptr->rw_chunk_offset;

         // If the write byte counter is equal to buf size, then read & write have same position.
         if (circ_buf_ptr->write_byte_counter > circ_buf_ptr->circ_buf_size)
         {
            client_ptr->unread_bytes = circ_buf_ptr->circ_buf_size;
         }
         else if(circ_buf_ptr->write_byte_counter == circ_buf_ptr->circ_buf_size)
         {
            /* When new output port was added during runtime, it should not read the old/stale data from circular buffer.
               So making reader_ptr->unread_bytes = 0.

               Note: As of now SPR is using circular as a scratch buffer only and it will hold only one frame size data.
               Which is expected to be consumed in every process call.
               If there are any changes in SPR circular buffer size, this change has to be reviewed */
            client_ptr->unread_bytes = 0;
         }
         else
         {
            client_ptr->unread_bytes = circ_buf_ptr->write_byte_counter;

            // Move the read position behind the write position by write_byte_counter bytes.
            // TODO: update comment.
            result = _circ_buf_position_shift_forward(circ_buf_ptr,
                                                      &client_ptr->rw_chunk_node_ptr,
                                                      &client_ptr->rw_chunk_offset,
                                                      circ_buf_ptr->circ_buf_size - circ_buf_ptr->write_byte_counter);
         }
      }
      else
      {
         // Initialize the read position same as the current write position.
         client_ptr->rw_chunk_node_ptr = NULL;
         client_ptr->rw_chunk_offset   = 0;
      }
   }

   return result;
}

/*
 * Resets the client write chunk position based on current buffer state.
 * Full documentation in circ_buf_utils.h
 */
static circbuf_result_t _circ_buf_write_client_reset(circ_buf_client_t *client_ptr)
{
   if (!client_ptr)
   {
      return CIRCBUF_SUCCESS;
   }

   circ_buf_t *circ_buf_ptr = client_ptr->circ_buf_ptr;

   // Reset to head chunk if the write client is not registered yet.
   client_ptr->rw_chunk_node_ptr    = circ_buf_ptr->head_chunk_ptr;
   client_ptr->rw_chunk_offset      = 0;
   circ_buf_ptr->write_byte_counter = 0;

   return CIRCBUF_SUCCESS;
}

/*
 *  Utility to write data to fragmented circular buffer.
 */
static circbuf_result_t _circ_buf_write_util(circ_buf_client_t *wr_client_ptr,
                                             int8_t *           inp_ptr,
                                             uint32_t           write_value,
                                             uint32_t           bytes_to_write,
                                             uint32_t           is_valid_timestamp,
                                             int64_t            timestamp)
{
   circbuf_result_t result = CIRCBUF_SUCCESS;

   // Check write client validity
   if ((NULL == wr_client_ptr) || (NULL == wr_client_ptr->rw_chunk_node_ptr) || (NULL == wr_client_ptr->circ_buf_ptr))
   {
#ifdef DEBUG_CIRC_BUF_UTILS
      AR_MSG_ISLAND(DBG_ERROR_PRIO, "_circ_buf_write_util: Writer client position not set.");
#endif
      return CIRCBUF_FAIL;
   }

   circ_buf_t *   circ_buf_ptr  = wr_client_ptr->circ_buf_ptr;
   const uint32_t circ_buf_size = wr_client_ptr->circ_buf_ptr->circ_buf_size;

   if ((bytes_to_write > circ_buf_size))
   {
#ifdef DEBUG_CIRC_BUF_UTILS
      AR_MSG_ISLAND(DBG_ERROR_PRIO,
             "_circ_buf_write_util: Received invalid bytes to write = %u, max buffer size = %lu ",
             bytes_to_write,
             circ_buf_size);
#endif
      return CIRCBUF_FAIL;
   }

   //   if (CIRCBUF_FAIL == circ_buf_check_if_corrupted(wr_client_ptr))
   //   {
   //#ifdef DEBUG_CIRC_BUF_UTILS
   //      AR_MSG_ISLAND(DBG_ERROR_PRIO, "Circ buf corrupted");
   //#endif
   //      return CIRCBUF_FAIL;
   //   }

   // Detect the move the read pointers in case of overflows.
   result = _circ_buf_detect_and_handle_overflow(circ_buf_ptr, bytes_to_write);

   // Get pointer to circular buffer params
   uint32_t          counter             = bytes_to_write;
   spf_list_node_t **wr_chunk_ptr_ptr    = &wr_client_ptr->rw_chunk_node_ptr;
   uint32_t *        wr_chunk_offset_ptr = &wr_client_ptr->rw_chunk_offset;
   while (counter > 0)
   {
      // get the chunk buffer address and the offset
      chunk_buffer_t *chunk_ptr = (chunk_buffer_t *)(*wr_chunk_ptr_ptr)->obj_ptr;

#ifdef DEBUG_CIRC_BUF_UTILS
      AR_MSG_ISLAND(DBG_HIGH_PRIO,
             "_circ_buf_write_util: buf_id=0x%x chunk= %d, chunk_ptr= 0x%x, offset= %lu, chunk_size= %lu, counter= "
             "%lu ",
             circ_buf_ptr->id,
             chunk_ptr->id,
             chunk_ptr,
             wr_client_ptr->rw_chunk_offset,
             chunk_ptr->size,
             counter);
#endif

      // If the bytes to read is less than the remaining bytes in the chunk
      if (counter < (chunk_ptr->size - wr_client_ptr->rw_chunk_offset))
      {
         /* Copy the less than chunk size counter.
          * In case of Context hub usecase, write to un-cached address. */
         if (NULL != inp_ptr)
         {
            memscpy(chunk_ptr->buffer_ptr + *wr_chunk_offset_ptr, counter, &inp_ptr[bytes_to_write - counter], counter);
         }
         else
         {
            memset(chunk_ptr->buffer_ptr + *wr_chunk_offset_ptr, write_value, counter);
         }

         wr_client_ptr->rw_chunk_offset += counter;

         counter = 0;
      }
      else
      {
         if (NULL != inp_ptr)
         {
            memscpy(chunk_ptr->buffer_ptr + *wr_chunk_offset_ptr,
                    (chunk_ptr->size - *wr_chunk_offset_ptr),
                    &inp_ptr[bytes_to_write - counter],
                    (chunk_ptr->size - *wr_chunk_offset_ptr));
         }
         else
         {
            memset(chunk_ptr->buffer_ptr + *wr_chunk_offset_ptr, write_value, chunk_ptr->size - *wr_chunk_offset_ptr);
         }

         counter -= (chunk_ptr->size - *wr_chunk_offset_ptr);

         // Advance to next chunk
         _circ_buf_next_chunk_node(wr_client_ptr->circ_buf_ptr, wr_chunk_ptr_ptr);
         *wr_chunk_offset_ptr = 0;
      }
   }

   // Update the write byte count in the circular buffer handle.
   circ_buf_ptr->write_byte_counter += bytes_to_write;
   if (circ_buf_ptr->write_byte_counter > circ_buf_size)
   {
      circ_buf_ptr->write_byte_counter = circ_buf_size;
   }

   // Update the timestamp of the latest sample in the buffer.
   // TODO: check timestamp discontinuity, update the timestamp to the last sample thats written.
   // circ_buf_ptr->timestamp          = timestamp;
   // circ_buf_ptr->is_valid_timestamp = is_valid_timestamp;

   // Iterate through all the reader clients and update unread byte count.
   for (spf_list_node_t *rd_client_list_ptr = circ_buf_ptr->rd_client_list_ptr;
        (NULL != rd_client_list_ptr);
        LIST_ADVANCE(rd_client_list_ptr))
   {
      circ_buf_client_t *temp_rd_client_ptr = (circ_buf_client_t *)rd_client_list_ptr->obj_ptr;

      // Update the unread byte count,
      // Saturate un_read_bytes count on buffer overflow. This can happen during steady state. TODO: do we have to
      // print over flow messages ?
      temp_rd_client_ptr->unread_bytes += bytes_to_write;
      if (temp_rd_client_ptr->unread_bytes > circ_buf_size)
      {
#ifdef DEBUG_CIRC_BUF_UTILS
         AR_MSG_ISLAND(DBG_HIGH_PRIO,
                "_circ_buf_write_util: Buffer overflow detected, buf_id=0x%x, rd_client_id=0x%x ",
                temp_rd_client_ptr->circ_buf_ptr->id,
                temp_rd_client_ptr);
#endif
         temp_rd_client_ptr->unread_bytes = circ_buf_size;
      }
   }

   return result;
}

static circbuf_result_t _circ_buf_add_chunks(circ_buf_t *circ_buf_ptr, uint32_t additional_size)
{
   uint32_t num_additional_chunks = 0;
   uint32_t last_chunk_size       = 0;
   uint32_t preferred_chunk_size  = circ_buf_ptr->preferred_chunk_size;
   uint32_t prev_circ_buf_size    = circ_buf_ptr->circ_buf_size;

#ifdef DEBUG_CIRC_BUF_UTILS
   AR_MSG_ISLAND(DBG_HIGH_PRIO,
          "CIRC_BUF: Add chunks. buf_id = 0x%x, additional_size: %u, prev_circ_buf_size: %u, prev_num_chunks: %u ",
          circ_buf_ptr->id,
          additional_size,
          circ_buf_ptr->circ_buf_size,
          circ_buf_ptr->num_chunks);
#endif

   if (additional_size == 0)
   {
      return CIRCBUF_SUCCESS;
   }

   // First allocate the array of chunk addresses, then allocate the individual buffer chunks
   last_chunk_size = (additional_size % preferred_chunk_size);

   if (0 == last_chunk_size)
   {
      // All chunks are same size
      num_additional_chunks = additional_size / preferred_chunk_size;
      last_chunk_size       = preferred_chunk_size;
   }
   else
   {
      // Last chunk size differs
      num_additional_chunks = additional_size / preferred_chunk_size + 1;
   }

   // Create the additional new chunks and add them to a temporary list
   spf_list_node_t *temp_new_chunk_list_ptr = NULL;
   uint32_t         chunk_index             = circ_buf_ptr->num_chunks;
   for (uint32_t iter = 0; iter < num_additional_chunks; ++iter)
   {
      uint32_t chunk_size = (num_additional_chunks - 1 == iter) ? last_chunk_size : preferred_chunk_size;

      uint32_t alloc_size = chunk_size + sizeof(chunk_buffer_t);

      chunk_buffer_t *buf_ptr =
         (chunk_buffer_t *)posal_memory_malloc(sizeof(uint8_t) * alloc_size, circ_buf_ptr->heap_id);

      if (NULL == buf_ptr)
      {
#ifdef DEBUG_CIRC_BUF_UTILS
         AR_MSG_ISLAND(DBG_ERROR_PRIO, "Circular Buffer allocation is failed ");
#endif
         return CIRCBUF_FAIL;
      }
      memset(buf_ptr, 0, alloc_size);

      buf_ptr->id         = chunk_index++;
      buf_ptr->size       = chunk_size;
      buf_ptr->buffer_ptr = (int8_t *)buf_ptr + sizeof(chunk_buffer_t);

      // Push the buffer to the tail of the list
      // TODO: check error
      spf_list_insert_tail(&temp_new_chunk_list_ptr, (void *)buf_ptr, circ_buf_ptr->heap_id, TRUE /* use_pool*/);
   }

   // Return failure if the new chunks list could not be created.
   if (NULL == temp_new_chunk_list_ptr)
   {
      return CIRCBUF_FAIL;
   }

   // Insert the chunk list after the current write chunk exists, else merge the new list
   // with the previous list.
   if (circ_buf_ptr->wr_client_ptr && circ_buf_ptr->wr_client_ptr->rw_chunk_node_ptr)
   {
      spf_list_node_t *cur_wr_chunk_ptr  = circ_buf_ptr->wr_client_ptr->rw_chunk_node_ptr;
      spf_list_node_t *next_wr_chunk_ptr = cur_wr_chunk_ptr->next_ptr;

      cur_wr_chunk_ptr->next_ptr        = temp_new_chunk_list_ptr;
      temp_new_chunk_list_ptr->prev_ptr = cur_wr_chunk_ptr;

      // Get the tail node of the new chunk list.
      spf_list_node_t *temp_tail_chunk_ptr = NULL;
      spf_list_get_tail_node(temp_new_chunk_list_ptr, &temp_tail_chunk_ptr);
      if (NULL == temp_tail_chunk_ptr)
      {
#ifdef DEBUG_CIRC_BUF_UTILS
         AR_MSG_ISLAND(DBG_ERROR_PRIO, "CIRC_BUF: Tail node not found.");
#endif
         return CIRCBUF_FAIL;
      }

      temp_tail_chunk_ptr->next_ptr = next_wr_chunk_ptr;
      if (next_wr_chunk_ptr)
      {
         next_wr_chunk_ptr->prev_ptr = temp_tail_chunk_ptr;
      }
   }
   else
   {
      // Else merge the new chunk list with the previous chunk list.
      spf_list_merge_lists(&circ_buf_ptr->head_chunk_ptr, &temp_new_chunk_list_ptr);
   }

   // Update chunk number.
   circ_buf_ptr->num_chunks = circ_buf_ptr->num_chunks + num_additional_chunks;
   circ_buf_ptr->circ_buf_size += additional_size;

   // Reset the write client position if we are adding the chunks for first time.
   if ((0 == prev_circ_buf_size) && (circ_buf_ptr->circ_buf_size > 0))
   {
      // Reset write client position first.
      _circ_buf_write_client_reset(circ_buf_ptr->wr_client_ptr);

      // Check and update the read positions to avoid reading from the removed chunks.s
      // Reset read client position. Must make sure valid write client position is set before calling this function
      // since read position is dependent on current write position.
      // TODO: check if needs to be removed.
      for (spf_list_node_t *client_list_ptr = circ_buf_ptr->rd_client_list_ptr; (NULL != client_list_ptr);
           LIST_ADVANCE(client_list_ptr))
      {
         circ_buf_client_t *temp_client_ptr = (circ_buf_client_t *)client_list_ptr->obj_ptr;

         _circ_buf_read_client_reset(temp_client_ptr);
      }
   }

#ifdef DEBUG_CIRC_BUF_UTILS
   AR_MSG_ISLAND(DBG_HIGH_PRIO,
          "CIRC_BUF: Adding chunks. buf_id = 0x%x, additional_size: %u, new_circ_buf_size: %u, num_chunks: %u ",
          circ_buf_ptr->id,
          additional_size,
          circ_buf_ptr->circ_buf_size,
          circ_buf_ptr->num_chunks);
#endif

   return CIRCBUF_SUCCESS;
}

static circbuf_result_t _circ_buf_remove_chunks(circ_buf_t *circ_buf_ptr, uint32_t removable_size)
{
   uint32_t preferred_chunk_size = circ_buf_ptr->preferred_chunk_size;

#ifdef DEBUG_CIRC_BUF_UTILS
   AR_MSG_ISLAND(DBG_HIGH_PRIO,
          "CIRC_BUF: Remove chunks. buf_id = 0x%x, removable_size: %u, new_circ_buf_size: %u, num_chunks: %u ",
          circ_buf_ptr->id,
          removable_size,
          circ_buf_ptr->circ_buf_size,
          circ_buf_ptr->num_chunks);
#endif

   if (removable_size == 0)
   {
      return CIRCBUF_SUCCESS;
   }

   while (removable_size && (removable_size >= preferred_chunk_size))
   {
      spf_list_node_t *temp_ptr = NULL;

      // Add after the current write chunks pointer if a write client exist.
      if (circ_buf_ptr->wr_client_ptr && circ_buf_ptr->wr_client_ptr->rw_chunk_node_ptr)
      {
         spf_list_node_t *cur_wr_chunk_ptr = circ_buf_ptr->wr_client_ptr->rw_chunk_node_ptr;

         if (cur_wr_chunk_ptr->next_ptr)
         {
            temp_ptr = cur_wr_chunk_ptr->next_ptr;
         }
      }

      // If the current write node is the tail, assign the head node as temp
      if (NULL == temp_ptr)
      {
         temp_ptr = circ_buf_ptr->head_chunk_ptr;
      }

      chunk_buffer_t *rem_chunk_ptr = (chunk_buffer_t *)temp_ptr->obj_ptr;

      removable_size -= rem_chunk_ptr->size;

      // Update circular buffer size.
      circ_buf_ptr->circ_buf_size -= rem_chunk_ptr->size;
      circ_buf_ptr->num_chunks--;

      // Delete the node from the list
      spf_list_delete_node_and_free_obj(&temp_ptr, &circ_buf_ptr->head_chunk_ptr, TRUE /* pool_used */);
   }

   // Reset write position if the new circular buffer size is zero
   if ((0 == circ_buf_ptr->circ_buf_size) && circ_buf_ptr->wr_client_ptr)
   {
      _circ_buf_write_client_reset(circ_buf_ptr->wr_client_ptr);
   }

   // Adjust write byte count based on new size
   if (circ_buf_ptr->write_byte_counter > circ_buf_ptr->circ_buf_size)
   {
      circ_buf_ptr->write_byte_counter = circ_buf_ptr->circ_buf_size;
   }

   // Check and update the read positions to avoid reading from the removed chunks.
   for (spf_list_node_t *client_list_ptr = circ_buf_ptr->rd_client_list_ptr; (NULL != client_list_ptr);
        LIST_ADVANCE(client_list_ptr))
   {
      circ_buf_client_t *temp_client_ptr = (circ_buf_client_t *)client_list_ptr->obj_ptr;

      if ((temp_client_ptr->unread_bytes > circ_buf_ptr->circ_buf_size) ||
          (temp_client_ptr->unread_bytes > circ_buf_ptr->write_byte_counter))
      {
         _circ_buf_read_client_reset(temp_client_ptr);
      }
   }

#ifdef DEBUG_CIRC_BUF_UTILS
   AR_MSG_ISLAND(DBG_HIGH_PRIO,
          "CIRC_BUF: Remove chunks. buf_id = 0x%x, removable_size: %u, new_circ_buf_size: %u, num_chunks: %u ",
          circ_buf_ptr->id,
          removable_size,
          circ_buf_ptr->circ_buf_size,
          circ_buf_ptr->num_chunks);
#endif

   return CIRCBUF_SUCCESS;
}

static circbuf_result_t _circ_buf_client_resize(circ_buf_t *circ_buf_ptr)
{
   circbuf_result_t result = CIRCBUF_SUCCESS;
   if (NULL == circ_buf_ptr)
   {
      return CIRCBUF_FAIL;
   }

   // Find the max alloc size from the current client list.
   uint32_t max_client_req_size = 0;

   for (spf_list_node_t *client_list_ptr = circ_buf_ptr->rd_client_list_ptr; (NULL != client_list_ptr);
        LIST_ADVANCE(client_list_ptr))
   {
      circ_buf_client_t *temp_client_ptr = (circ_buf_client_t *)client_list_ptr->obj_ptr;

      if (NULL == temp_client_ptr)
      {
         return CIRCBUF_FAIL;
      }

      if (!temp_client_ptr->is_read_client)
      {
         continue;
      }

      uint32_t total_client_requested_size = temp_client_ptr->req_base_buffer_size + temp_client_ptr->req_buffer_resize;

      if (total_client_requested_size > max_client_req_size)
      {
         max_client_req_size = total_client_requested_size;
      }
   }

#ifdef DEBUG_CIRC_BUF_UTILS
   AR_MSG_ISLAND(DBG_HIGH_PRIO,
          "CIRC_BUF: Found max of resize requests. buf_id = 0x%x, prev_max_req_alloc_size: %u, max_req_alloc_size: %u",
          circ_buf_ptr->id,
          circ_buf_ptr->max_req_buf_resize,
          max_client_req_size);
#endif

   if (max_client_req_size == circ_buf_ptr->max_req_buf_resize)
   {
      return CIRCBUF_SUCCESS;
   }

   if (max_client_req_size > circ_buf_ptr->max_req_buf_resize)
   {
      result = _circ_buf_add_chunks(circ_buf_ptr, max_client_req_size - circ_buf_ptr->max_req_buf_resize);
   }
   else
   {
      result = _circ_buf_remove_chunks(circ_buf_ptr, circ_buf_ptr->max_req_buf_resize - max_client_req_size);
   }

   circ_buf_ptr->max_req_buf_resize = max_client_req_size;

   return result;
}

// Function detects the overflow in the read clients of the buffer and adjusts the buffer accordingly
static circbuf_result_t _circ_buf_detect_and_handle_overflow(circ_buf_t *circ_buf_ptr, uint32_t bytes_to_write)
{
   circbuf_result_t result = CIRCBUF_SUCCESS;

   for (spf_list_node_t *rd_client_list_ptr = circ_buf_ptr->rd_client_list_ptr; (NULL != rd_client_list_ptr);
        LIST_ADVANCE(rd_client_list_ptr))
   {
      circ_buf_client_t *temp_rd_client_ptr = (circ_buf_client_t *)rd_client_list_ptr->obj_ptr;

      uint32_t free_bytes = circ_buf_ptr->circ_buf_size - temp_rd_client_ptr->unread_bytes;
      if (bytes_to_write > free_bytes)
      {
         /* Since there is no space in circular buffer for writing samples_to_write
          number of samples, create space by flushing samples_to_write-free_samples
          number of samples */
         _circ_buf_read_advance(temp_rd_client_ptr, bytes_to_write - free_bytes);

// TODO: print error if the gate is opened and overrun happens.
// add a a flag to each reader check if the overrun happened when gate is opened.
#ifdef DEBUG_CIRC_BUF_UTILS
         AR_MSG_ISLAND(DBG_HIGH_PRIO,
                "CIRC_BUF: Detected Overrun. client: 0x%x, bytes to write = %u, free_bytes = %u",
                (uint32_t)temp_rd_client_ptr,
                bytes_to_write,
                free_bytes);
#endif
         result = CIRCBUF_OVERRUN;
      }
   }

   return result;
}

static circbuf_result_t _circ_buf_next_chunk_node(circ_buf_t *circ_buf_ptr, spf_list_node_t **chunk_ptr_ptr)
{
   if ((NULL == chunk_ptr_ptr) || (NULL == *chunk_ptr_ptr) || (NULL == circ_buf_ptr))
   {
      return CIRCBUF_FAIL;
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

   return CIRCBUF_SUCCESS;
}

circbuf_result_t circ_buf_query_unread_bytes(circ_buf_client_t *rd_client_ptr, uint32_t *unread_bytes_ptr)
{
   if ((NULL == unread_bytes_ptr) || (NULL == rd_client_ptr->rw_chunk_node_ptr) ||
       (NULL == rd_client_ptr->circ_buf_ptr) || (FALSE == rd_client_ptr->is_read_client))
   {
#ifdef DEBUG_CIRC_BUF_UTILS
      AR_MSG_ISLAND(DBG_ERROR_PRIO, "CIRC_BUF: Query unread bytes failed. Invalid input params");
#endif
      return CIRCBUF_FAIL;
   }

   *unread_bytes_ptr = rd_client_ptr->unread_bytes;

#ifdef DEBUG_CIRC_BUF_UTILS
   AR_MSG_ISLAND(DBG_HIGH_PRIO, "CIRC_BUF: Circ Buf read, un_read_bytes = %u", rd_client_ptr->unread_bytes);
#endif

   return CIRCBUF_SUCCESS;
}

///*
// * This function checks if the gap between write_index and read_index is valid
// * Full documentation in circ_buf_utils.h
// */
// static circbuf_result_t _circ_buf_check_if_corrupted(circ_buf_t *frag_circ_buf_ptr)
//{
//   //   int32_t  write_read_gap       = 0;
//   //   uint32_t preferred_chunk_size = frag_circ_buf_ptr->preferred_chunk_size;
//   //   uint32_t circ_buf_read_offset =
//   //      frag_circ_buf_ptr->read_index.chunk_index * preferred_chunk_size +
//   //      frag_circ_buf_ptr->read_index.rw_chunk_offset;
//   //   uint32_t circ_buf_write_offset = frag_circ_buf_ptr->write_index.chunk_index * preferred_chunk_size +
//   //                                    frag_circ_buf_ptr->write_index.rw_chunk_offset;
//   //
//   //   write_read_gap = circ_buf_write_offset - circ_buf_read_offset;
//   //   /* Circ buf read/write is corrupted if the number of bytes between the
//   //    * circ_buf_write_offset and the circ_buf_read_offset is greater than the
//   //    * total circ_buf_size */
//   //   if (write_read_gap < 0)
//   //   {
//   //      write_read_gap += frag_circ_buf_ptr->circ_buf_size;
//   //      if (write_read_gap < 0)
//   //      {
//   //#ifdef DEBUG_CIRC_BUF_UTILS
//   //         AR_MSG_ISLAND(DBG_ERROR_PRIO, "Circ buf read/write corrupted");
//   //#endif
//   //         return CIRCBUF_FAIL;
//   //      }
//   //   }
//   //   if (write_read_gap > (int32_t)frag_circ_buf_ptr->circ_buf_size)
//   //   {
//   //#ifdef DEBUG_CIRC_BUF_UTILS
//   //      AR_MSG_ISLAND(DBG_ERROR_PRIO, "Circ buf read/write corrupted");
//   //#endif
//   //      return CIRCBUF_FAIL;
//   //   }
//   return CIRCBUF_SUCCESS;
//}
#ifdef DEBUG_CIRC_BUF_UTILS
static void print_client_chunk_positions(circ_buf_t *circ_buf_ptr)
{

   for (spf_list_node_t *client_list_ptr = circ_buf_ptr->rd_client_list_ptr; (NULL != client_list_ptr);
        LIST_ADVANCE(client_list_ptr))
   {

      circ_buf_client_t *rd_client_ptr = (circ_buf_client_t *)client_list_ptr->obj_ptr;
      circ_buf_client_t *wr_client_ptr = rd_client_ptr->circ_buf_ptr->wr_client_ptr;

      AR_MSG_ISLAND(DBG_HIGH_PRIO,
             "Reader client:0x%X, current read position is rd(chunk= 0x%x, offset= %u), unread_bytes=%u,"
             "wr(chunk= 0x%x, offset= %u,  write_byte_counter:%d)",
             rd_client_ptr,
             rd_client_ptr->rw_chunk_node_ptr,
             rd_client_ptr->rw_chunk_offset,
             rd_client_ptr->unread_bytes,
             wr_client_ptr->rw_chunk_node_ptr,
             wr_client_ptr->rw_chunk_offset,
             circ_buf_ptr->write_byte_counter);
   }
}
#endif
