/**
 *   \file spr_lib.c
 *   \brief
 *        This file contains implementation of SPR driver
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "spr_lib.h"
#include "circular_buffer.h"
#include "ar_msg.h"
#include "spf_list_utils.h"

#define MAX_CHANNELS_PER_STREAM CAPI_MAX_CHANNELS_V2

//#define DEBUG_SPR_DRIVER

#define MAX_64 0xFFFFFFFFFFFFFFFF

#define MEM_ALLOC_8_BYTE_ALIGNED 8

#ifndef ALIGN_8_BYTES
#define ALIGN_8_BYTES(a) (((a) & (~(uint32_t)0x7)) + 8)
#endif
/*==============================================================================
   Local Declarations
==============================================================================*/

/*==============================================================================
   Local Function Implementation
==============================================================================*/
circ_buf_t *_spr_find_circ_buf(spr_driver_t *drv_ptr, uint32_t ch_id);

ar_result_t _spr_stream_writer_free_util(spr_driver_t *        drv_ptr,
                                         spr_stream_writer_t **writer_handle_pptr,
                                         bool_t                free_list_node);

ar_result_t _spr_stream_reader_free_util(spr_driver_t *        drv_ptr,
                                         spr_stream_reader_t **reader_handle,
                                         bool_t                free_list_node);

/*==============================================================================
   Public Function Implementation
==============================================================================*/
ar_result_t spr_driver_init(POSAL_HEAP_ID heap_id, spr_driver_t **drv_ptr_ptr_ptr, uint32_t driver_instance_id)
{
   if (NULL == drv_ptr_ptr_ptr)
   {
#ifdef DEBUG_SPR_DRIVER
      AR_MSG_ISLAND(DBG_ERROR_PRIO, "SPR_DRIVER: Init failed. bad input params.");
#endif
      return AR_EBADPARAM;
   }

   // TODO: For Elite, check if the driver instance ID already exists and return the already created handle.

   // Allocate memory for the driver handle
   *drv_ptr_ptr_ptr =
      (spr_driver_t *)posal_memory_aligned_malloc(sizeof(spr_driver_t), MEM_ALLOC_8_BYTE_ALIGNED, heap_id);
   if (NULL == *drv_ptr_ptr_ptr)
   {
#ifdef DEBUG_SPR_DRIVER
      AR_MSG_ISLAND(DBG_ERROR_PRIO, "SPR_DRIVER: Init failed. No memory.");
#endif
      return AR_ENOMEMORY;
   }
   memset(*drv_ptr_ptr_ptr, 0, sizeof(spr_driver_t));

   (*drv_ptr_ptr_ptr)->heap_id            = heap_id;
   (*drv_ptr_ptr_ptr)->driver_instance_id = driver_instance_id; // TODO: meant for elite, check if it can be

   return AR_EOK;
}

ar_result_t spr_driver_deinit(spr_driver_t **drv_ptr_pptr, uint32_t driver_instance_id)
{
   ar_result_t result = AR_EOK;
   if (NULL == drv_ptr_pptr)
   {
#ifdef DEBUG_SPR_DRIVER
      AR_MSG_ISLAND(DBG_ERROR_PRIO, "spr_driver_deinit: Failed. bad input params.");
#endif
      return AR_EBADPARAM;
   }

   // First Reader have to be freed and then writers can be freed.
   // Iterate through the readers list and free the readers.
   spr_stream_reader_t *temp_strm_rd_ptr =
      (spr_stream_reader_t *)spf_list_pop_head(&(*drv_ptr_pptr)->stream_reader_list, TRUE /* pool_used */);
   while (temp_strm_rd_ptr)
   {
      // Free the stream reader circular buffers. But don't free the handle memory.
      result = _spr_stream_reader_free_util(*drv_ptr_pptr, &temp_strm_rd_ptr, FALSE);
      if (result != AR_EOK)
      {
#ifdef DEBUG_SPR_DRIVER
         AR_MSG_ISLAND(DBG_ERROR_PRIO,
                "spr_driver_deinit: Failed. Couldn't free the stream reader, 0x%x. ",
                temp_strm_rd_ptr->id);
#endif
      }

      // Free the stream Reader memory.
      posal_memory_aligned_free(temp_strm_rd_ptr);

      // Pop the next element from the list.
      temp_strm_rd_ptr =
         (spr_stream_reader_t *)spf_list_pop_head(&(*drv_ptr_pptr)->stream_reader_list, TRUE /* pool_used */);
   }

   // Iterate through the writers list and free the writers.
   spr_stream_writer_t *temp_strm_wr_ptr =
      (spr_stream_writer_t *)spf_list_pop_head(&(*drv_ptr_pptr)->stream_writer_list, TRUE /* pool_used */);
   while (temp_strm_wr_ptr)
   {
      // Free the stream writer circular buffers. But don't free the handle memory.
      result = _spr_stream_writer_free_util(*drv_ptr_pptr, &temp_strm_wr_ptr, FALSE);
      if (result != AR_EOK)
      {
#ifdef DEBUG_SPR_DRIVER
         AR_MSG_ISLAND(DBG_ERROR_PRIO,
                "spr_driver_deinit: Failed. Couldn't free the stream writer, 0x%x ",
                temp_strm_wr_ptr->id);
#endif
      }

      // Free the stream writer memory.
      posal_memory_aligned_free(temp_strm_wr_ptr);

      // Pop the next element from the list.
      temp_strm_wr_ptr =
         (spr_stream_writer_t *)spf_list_pop_head(&(*drv_ptr_pptr)->stream_writer_list, TRUE /* pool_used */);
   }

   // Free the stream Reader memory.
   posal_memory_aligned_free(*drv_ptr_pptr);
   *drv_ptr_pptr = NULL;

   return AR_EOK;
}

ar_result_t spr_driver_set_chunk_size(spr_driver_t *drv_ptr, uint32_t preferred_chunk_size)
{
   if (drv_ptr)
   {
      drv_ptr->preferred_chunk_size = preferred_chunk_size;
#ifdef DEBUG_SPR_DRIVER
      AR_MSG_ISLAND(DBG_ERROR_PRIO, "spr_driver_set_chunk_size: set the preferred chunk size = %lu", preferred_chunk_size);
#endif
   }

   return AR_EOK;
}

ar_result_t spr_stream_writer_create(spr_driver_t *        drv_ptr,
                                     uint32_t              base_buffer_size,
                                     uint32_t              pre_buffering_delay_in_bytes,
                                     uint32_t              num_channels,
                                     uint32_t *            ch_id_arr,
                                     spr_stream_writer_t **writer_handle_pptr)
{
   ar_result_t result = AR_EOK;
   if ((NULL == drv_ptr) || (NULL == writer_handle_pptr))
   {
#ifdef DEBUG_SPR_DRIVER
      AR_MSG_ISLAND(DBG_ERROR_PRIO, "SPR_DRIVER: Writer allocation failed. Bad input params.");
#endif
      return AR_EBADPARAM;
   }

   // Validate the init params
   // TODO: remove MAX_CHs
   if ((num_channels > MAX_CHANNELS_PER_STREAM) || (0 == num_channels) || (NULL == ch_id_arr))
   {
#ifdef DEBUG_SPR_DRIVER
      AR_MSG_ISLAND(DBG_ERROR_PRIO, "SPR_DRIVER: Writer allocation failed. num channels=%d not supported. ", num_channels);
#endif
      return AR_EBADPARAM;
   }

   // Allocate memory for the stream writer handle.
   uint32_t handle_size = ALIGN_8_BYTES(sizeof(spr_stream_writer_t));
   handle_size += (ALIGN_8_BYTES(sizeof(circ_buf_t)) + ALIGN_8_BYTES(sizeof(circ_buf_client_t))) * num_channels;

   int8_t *blob_ptr = (int8_t *)posal_memory_aligned_malloc(handle_size, MEM_ALLOC_8_BYTE_ALIGNED, drv_ptr->heap_id);
   if (NULL == blob_ptr)
   {
      return AR_ENOMEMORY;
   }
   memset(blob_ptr, 0, handle_size);

   // TODO: do 8 byte alignment for alloc and ptr assign.
   // Init handle fields.
   *writer_handle_pptr                                 = (spr_stream_writer_t *)blob_ptr;
   (*writer_handle_pptr)->base_buffer_size             = base_buffer_size;
   (*writer_handle_pptr)->pre_buffering_delay_in_bytes = pre_buffering_delay_in_bytes;
   (*writer_handle_pptr)->num_channels                 = num_channels;
   blob_ptr += ALIGN_8_BYTES(sizeof(spr_stream_writer_t));

   // Assign the circ buf array pointer
   (*writer_handle_pptr)->circ_buf_arr_ptr = (circ_buf_t *)(blob_ptr);
   blob_ptr += (ALIGN_8_BYTES(sizeof(circ_buf_t))) * num_channels;

   // Assign circ buf writer handle array pointer
   (*writer_handle_pptr)->wr_client_arr_ptr = (circ_buf_client_t *)(blob_ptr);

   // Create circular buffer and get the writer client handles for each channel.
   spr_stream_writer_t *str_wr_ptr = *writer_handle_pptr;

   // Add the stream writer handle to the driver list.
   spf_list_insert_tail(&drv_ptr->stream_writer_list, str_wr_ptr, drv_ptr->heap_id, TRUE /* use_pool*/);

   // Update number writers and assign writer ID.
   str_wr_ptr->id = drv_ptr->num_writers++;

   for (uint32_t iter = 0; iter < num_channels; iter++)
   {
      circbuf_result_t buf_res = CIRCBUF_SUCCESS;

      // Return failure if the given Channel ID is already used.
      // Each channel id must be unique across all the writers.
      uint32_t ch_id = ch_id_arr[iter];
      if (NULL != _spr_find_circ_buf(drv_ptr, ch_id))
      {
#ifdef DEBUG_SPR_DRIVER
         AR_MSG_ISLAND(DBG_ERROR_PRIO, "SPR_DRIVER: writer channel id = 0x%x is already used.", ch_id);
#endif
         spr_stream_writer_destroy(drv_ptr, writer_handle_pptr);
         return AR_EFAILED;
      }

      // TODO: Remoce fatal errors from the debug flag.
      // Allocate circular buffer and get the buffer handle.
      buf_res = circ_buf_alloc(&str_wr_ptr->circ_buf_arr_ptr[iter],
                               str_wr_ptr->base_buffer_size,
                               drv_ptr->preferred_chunk_size,
                               ch_id,
                               drv_ptr->heap_id);
      if (buf_res != CIRCBUF_SUCCESS)
      {
#ifdef DEBUG_SPR_DRIVER
         AR_MSG_ISLAND(DBG_ERROR_PRIO, "SPR_DRIVER: Circular buffer allocation failed. iter = %d", iter);
#endif
         spr_stream_writer_destroy(drv_ptr, writer_handle_pptr);
         return AR_EFAILED;
      }

      // Register the writer clients with the circular buffer and get the handles.
      result =
         circ_buf_register_client(&str_wr_ptr->circ_buf_arr_ptr[iter], FALSE, 0, &str_wr_ptr->wr_client_arr_ptr[iter]);
      if (buf_res != CIRCBUF_SUCCESS)
      {
#ifdef DEBUG_SPR_DRIVER
         AR_MSG_ISLAND(DBG_ERROR_PRIO, "SPR_DRIVER: Writer client registration failed. iter = %d", iter);
#endif
         spr_stream_writer_destroy(drv_ptr, writer_handle_pptr);
         return AR_EFAILED;
      }
   }

#ifdef DEBUG_SPR_DRIVER
   AR_MSG_ISLAND(DBG_HIGH_PRIO, "SPR_DRIVER: Writer Allocation successful num_channel =%d", num_channels);
#endif

   return result;
}

ar_result_t spr_stream_reader_create(spr_driver_t *        drv_ptr,
                                     uint32_t              downstream_setup_duration_in_bytes,
                                     uint32_t              num_channels,
                                     uint32_t *            ch_id_arr_ptr,
                                     spr_stream_reader_t **reader_handle_pptr)
{
   ar_result_t result = AR_EOK;
   if ((NULL == drv_ptr) || (NULL == reader_handle_pptr))
   {
#ifdef DEBUG_SPR_DRIVER
      AR_MSG_ISLAND(DBG_ERROR_PRIO, "SPR_DRIVER: Reader allocation failed. Bad input params.");
#endif
      return AR_EBADPARAM;
   }

   // Validate the init params
   if ((num_channels > MAX_CHANNELS_PER_STREAM) || (0 == num_channels))
   {
      return AR_EBADPARAM;
   }

   // Allocate memory for the stream reader handle.
   uint32_t handle_size = ALIGN_8_BYTES(sizeof(spr_stream_reader_t));
   handle_size += ALIGN_8_BYTES(sizeof(circ_buf_client_t)) * num_channels;

   int8_t *blob_ptr = (int8_t *)posal_memory_aligned_malloc(handle_size, MEM_ALLOC_8_BYTE_ALIGNED, drv_ptr->heap_id);
   if (NULL == blob_ptr)
   {
      return AR_ENOMEMORY;
   }
   memset(blob_ptr, 0, handle_size);

   // Init handle fields.
   *reader_handle_pptr                          = (spr_stream_reader_t *)blob_ptr;
   (*reader_handle_pptr)->req_base_bufffer_size = downstream_setup_duration_in_bytes;
   (*reader_handle_pptr)->num_channels          = num_channels;
   blob_ptr += ALIGN_8_BYTES(sizeof(spr_stream_reader_t));

   // Assign reader client array pointer.
   (*reader_handle_pptr)->rd_client_arr_ptr = (circ_buf_client_t *)blob_ptr;

   // Create circular buffer and get the writer client handles for each channel.
   spr_stream_reader_t *str_rd_ptr = *reader_handle_pptr;

   // TODO: Compute the relative sync offset and adjust the read pointer accordingly while reading the data from chs.
   str_rd_ptr->rel_sync_offset_in_bytes = 0;

   for (uint32_t iter = 0; iter < num_channels; iter++)
   {
      circbuf_result_t buf_res = CIRCBUF_SUCCESS;
      uint32_t         ch_id   = ch_id_arr_ptr[iter];

      // Find the channel buffer using the ch_id
      circ_buf_t *circ_buf_ptr = _spr_find_circ_buf(drv_ptr, ch_id);
      if (NULL == circ_buf_ptr)
      {
         result |= AR_ENOMEMORY;

#ifdef DEBUG_SPR_DRIVER
         AR_MSG_ISLAND(DBG_ERROR_PRIO, "SPR_DRIVER: Channel buffer ch_id=%d couldn't be found. iter = %d", ch_id, iter);
#endif
         result |= spr_stream_reader_destroy(drv_ptr, reader_handle_pptr);
         return result;
      }

      // Register read client for with the circular buffer.
      buf_res = circ_buf_register_client(circ_buf_ptr,
                                         TRUE,
                                         downstream_setup_duration_in_bytes,
                                         &str_rd_ptr->rd_client_arr_ptr[iter]);
      if (buf_res != CIRCBUF_SUCCESS)
      {
         result |= AR_EFAILED;

#ifdef DEBUG_SPR_DRIVER
         AR_MSG_ISLAND(DBG_ERROR_PRIO, "SPR_DRIVER: Reader client regsitration failed. ch_id=%d, iter = %d", ch_id, iter);
#endif
         result |= spr_stream_reader_destroy(drv_ptr, reader_handle_pptr);
         return result;
      }
   }

   // Add the stream Reader handle to the driver list.
   spf_list_insert_tail(&drv_ptr->stream_reader_list, *reader_handle_pptr, drv_ptr->heap_id, TRUE /* use_pool*/);

   // Update num readers and assign reader ID.
   (*reader_handle_pptr)->id = drv_ptr->num_readers++;

#ifdef DEBUG_SPR_DRIVER
   AR_MSG_ISLAND(DBG_HIGH_PRIO, "SPR_DRIVER: Reader Allocation successful num_channel =%d", num_channels);
#endif

   return AR_EOK;
}
// TODO: pass num_bufs, and update commnets for timestamp
// check max_data_len - actual_data_lenth
ar_result_t spr_stream_write(spr_stream_writer_t *writer_handle,
                             capi_buf_t *         input_buf_arr,
                             uint32_t             is_valid_timestamp,
                             int64_t              timestamp)
{
   ar_result_t      result       = AR_EOK;
   circbuf_result_t circ_buf_res = CIRCBUF_SUCCESS;
   // Iterate through all the channel write clients and write the input data
   for (uint32_t iter = 0; iter < writer_handle->num_channels; iter++)
   {
      circ_buf_res = circ_buf_write(&writer_handle->wr_client_arr_ptr[iter],
                                    input_buf_arr[iter].data_ptr,
                                    input_buf_arr[iter].actual_data_len,
                                    is_valid_timestamp,
                                    timestamp);

      if ((CIRCBUF_SUCCESS != circ_buf_res) && (CIRCBUF_OVERRUN != circ_buf_res))
      {
#ifdef DEBUG_SPR_DRIVER
         AR_MSG_ISLAND(DBG_ERROR_PRIO,
                "SPR_DRIVER: Stream writer buffering failed. ch_id=%d, iter = %d",
                writer_handle->wr_client_arr_ptr[iter].circ_buf_ptr->id,
                iter);
#endif
         // TODO: Need to continue or break ?
         result |= AR_EFAILED;
      }
#ifdef DEBUG_SPR_DRIVER
      AR_MSG_ISLAND(DBG_HIGH_PRIO,
             "SPR_DRIVER: Stream writer buffering done. ch_id=%d, iter = %d, actual_data_len = %d",
             writer_handle->wr_client_arr_ptr[iter].circ_buf_ptr->id,
             iter,
             input_buf_arr[iter].actual_data_len);
#endif
   }
#ifdef DEBUG_SPR_DRIVER
   AR_MSG_ISLAND(DBG_HIGH_PRIO,
          "SPR_DRIVER: Stream writer buffering done. actual_data_len = %d Timestamp %ld",
          input_buf_arr[0].actual_data_len,
          (uint32_t)timestamp);
#endif

   return result;
}

ar_result_t spr_stream_read(spr_stream_reader_t *reader_handle, capi_buf_t *output_buf_arr)
{
   ar_result_t      result     = AR_EOK;
   circbuf_result_t buf_result = CIRCBUF_SUCCESS;
   // Iterate through all the channel write clients and write the input data
   for (uint32_t iter = 0; iter < reader_handle->num_channels; iter++)
   {
      // If the output buffer is full return, else fill rest of the buffer.
      if (output_buf_arr[iter].actual_data_len == output_buf_arr[iter].max_data_len)
      {

#ifdef DEBUG_SPR_DRIVER
         AR_MSG_ISLAND(DBG_MED_PRIO,
                "SPR_DRIVER: Stream reader, output buffer full. ch_id=%d, iter = %d, actual_data_len=%d ",
                reader_handle->rd_client_arr_ptr[iter].circ_buf_ptr->id,
                iter,
                output_buf_arr[iter].actual_data_len);
#endif

         return AR_EOK;
      }

      uint32_t num_bytes_read = 0;
      buf_result              = circ_buf_read(&reader_handle->rd_client_arr_ptr[iter],
                                 output_buf_arr[iter].data_ptr + output_buf_arr[iter].actual_data_len,
                                 output_buf_arr[iter].max_data_len - output_buf_arr[iter].actual_data_len,
                                 &num_bytes_read);

      if (CIRCBUF_SUCCESS == buf_result)
      {
         output_buf_arr[iter].actual_data_len = output_buf_arr[iter].max_data_len;
      }
      else if (CIRCBUF_UNDERRUN == buf_result)
      {
#ifdef DEBUG_SPR_DRIVER
         AR_MSG_ISLAND(DBG_ERROR_PRIO,
                "SPR_DRIVER: Stream reader under run on ch_id=%d, iter = %d space left in output %d",
                reader_handle->rd_client_arr_ptr[iter].circ_buf_ptr->id,
                iter,
                output_buf_arr[iter].actual_data_len);
#endif
         output_buf_arr[iter].actual_data_len += num_bytes_read;
         result |= AR_ENEEDMORE;
         continue;
      }
      else
      {
#ifdef DEBUG_SPR_DRIVER
         AR_MSG_ISLAND(DBG_ERROR_PRIO,
                "SPR_DRIVER: Stream reader, read data failed.  ch_id=%d, iter = %d",
                reader_handle->rd_client_arr_ptr[iter].circ_buf_ptr->id,
                iter);
#endif
         return AR_EFAILED;
      }

#ifdef DEBUG_SPR_DRIVER
      AR_MSG_ISLAND(DBG_ERROR_PRIO,
             "SPR_DRIVER: Stream reader, data read done. ch_id=%d, iter = %d, actual_data_len=%d ",
             reader_handle->rd_client_arr_ptr[iter].circ_buf_ptr->id,
             iter,
             output_buf_arr[iter].actual_data_len);
#endif
   }

#ifdef DEBUG_SPR_DRIVER
   uint32_t num_unread_bytes = 0;
   spr_circ_buf_query_unread_bytes(reader_handle->rd_client_arr_ptr, &num_unread_bytes);
   AR_MSG_ISLAND(DBG_HIGH_PRIO,
          "SPR_DRIVER: Stream reader, data read done. error_code %x, actual_data_len=%d unread_bytes = %ld",
          result,
          output_buf_arr[0].actual_data_len,
          num_unread_bytes);

#endif

   return result;
}

ar_result_t spr_stream_read_adjust(spr_stream_reader_t *reader_handle,
                                   uint32_t             read_offset,
                                   uint32_t             bytes_per_ms,
                                   uint32_t *           actual_read_offset)
{
   if (NULL == reader_handle)
   {
#ifdef DEBUG_SPR_DRIVER
      AR_MSG_ISLAND(DBG_ERROR_PRIO, "SPR_DRIVER: Stream Reader adjust failed. Bad input params.");
#endif
      return AR_EFAILED;
   }

   // Find the most oldest time stamp among all the circular buffers.
   int64_t oldest_timestamp = MAX_64;
   for (uint32_t iter = 0; iter < reader_handle->num_channels; iter++)
   {
      // Check if its the latest time stamp
      if ((reader_handle->rd_client_arr_ptr[iter].circ_buf_ptr->is_valid_timestamp == TRUE) &&
          oldest_timestamp > reader_handle->rd_client_arr_ptr[iter].circ_buf_ptr->timestamp)
      {
         oldest_timestamp = reader_handle->rd_client_arr_ptr[iter].circ_buf_ptr->timestamp;
      }
   }

   ar_result_t result = AR_EOK;
   // Iterate through all the Readers channel buffers and adjust the read pointer.
   uint32_t min_unread_bytes = 0xFFFFFFFF;
   for (uint32_t iter = 0; iter < reader_handle->num_channels; iter++)
   {

      uint32_t           ch_un_read_bytes = 0;
      circ_buf_client_t *rd_client_ptr    = &reader_handle->rd_client_arr_ptr[iter];

      // TODO: check if timestamp not valid, and print msg saying channels may not be synchronized.
      uint32_t sync_offset_in_bytes = 0;
      if (rd_client_ptr->circ_buf_ptr->is_valid_timestamp)
      {
         int64_t sync_offset_in_ms = rd_client_ptr->circ_buf_ptr->timestamp - oldest_timestamp;
         if (sync_offset_in_ms > 0)
         {
            sync_offset_in_bytes = sync_offset_in_ms * bytes_per_ms;
         }
      }

      if (CIRCBUF_SUCCESS != circ_buf_read_adjust(rd_client_ptr, read_offset + sync_offset_in_bytes, &ch_un_read_bytes))
      {
#ifdef DEBUG_SPR_DRIVER
         AR_MSG_ISLAND(DBG_ERROR_PRIO,
                "SPR_DRIVER: Stream reader, read adjust failed. ch_id=%d, iter = %d",
                rd_client_ptr->circ_buf_ptr->id,
                iter);
#endif
         // TODO: Need to continue or break ?
         result |= AR_EFAILED;
      }

#ifdef DEBUG_SPR_DRIVER
      AR_MSG_ISLAND(DBG_ERROR_PRIO,
             "SPR_DRIVER: Stream reader, read adjust successful. ch_id=%d, ch_un_read_bytes = %lu, min_unread_bytes "
             "=%lu",
             rd_client_ptr->circ_buf_ptr->id,
             ch_un_read_bytes,
             min_unread_bytes);
#endif

      if (ch_un_read_bytes < min_unread_bytes)
      {
         min_unread_bytes = ch_un_read_bytes;
      }
   }

   if (actual_read_offset)
   {
      *actual_read_offset = min_unread_bytes;
   }

   AR_MSG_ISLAND(DBG_MED_PRIO,
          "SPR_DRIVER: Stream reader, read adjust done. read_offset = %d, actual_read_offset= %lu",
          read_offset,
          actual_read_offset);

   return result;
}

ar_result_t spr_stream_reader_req_resize(spr_stream_reader_t *reader_handle,
                                         uint32_t             requested_alloc_size,
                                         uint32_t             is_register)
{
   ar_result_t result = AR_EOK;

   if (NULL == reader_handle)
   {
#ifdef DEBUG_SPR_DRIVER
      AR_MSG_ISLAND(DBG_ERROR_PRIO, "SPR_DRIVER: Stream Reader resize failed. Bad input params.");
#endif
      return AR_EFAILED;
   }

   // Iterate through all the Readers channel buffers and request to resize the buffer.
   for (uint32_t iter = 0; iter < reader_handle->num_channels; iter++)
   {
      if (CIRCBUF_SUCCESS !=
          circ_buf_read_client_resize(&reader_handle->rd_client_arr_ptr[iter], requested_alloc_size, is_register))
      {
#ifdef DEBUG_SPR_DRIVER
         AR_MSG_ISLAND(DBG_ERROR_PRIO,
                "SPR_DRIVER: Stream reader, read adjust failed. ch_id=%d, iter = %d",
                reader_handle->rd_client_arr_ptr[iter].circ_buf_ptr->id,
                iter);
#endif
         // TODO: Need to continue or break ?
         result |= AR_EFAILED;
      }
   }

   AR_MSG_ISLAND(DBG_MED_PRIO,
          "SPR_DRIVER: Stream reader, resize done. requested_alloc_size: %lu, is_register: %lu",
          requested_alloc_size,
          is_register);

   return result;
}

ar_result_t spr_stream_reader_get_unread_bytes(spr_stream_reader_t *reader_handle, uint32_t *unread_bytes_per_ch_ptr)
{
   if (!reader_handle || !unread_bytes_per_ch_ptr)
   {
      return AR_EBADPARAM;
   }

   ar_result_t      result     = AR_EOK;
   circbuf_result_t buf_result = CIRCBUF_SUCCESS;

   buf_result = circ_buf_query_unread_bytes(&reader_handle->rd_client_arr_ptr[0], unread_bytes_per_ch_ptr);

   if (CIRCBUF_SUCCESS != buf_result)
   {

#ifdef DEBUG_SPR_DRIVER
      AR_MSG_ISLAND(DBG_ERROR_PRIO, "SPR_DRIVER: Stream reader, read unread bytes failed with result 0x%x", buf_result);
#endif
      return AR_EFAILED;
   }

   return result;
}

ar_result_t spr_stream_writer_destroy(spr_driver_t *drv_ptr, spr_stream_writer_t **writer_handle)
{
   return _spr_stream_writer_free_util(drv_ptr, writer_handle, TRUE);
}

ar_result_t _spr_stream_writer_free_util(spr_driver_t *        drv_ptr,
                                         spr_stream_writer_t **writer_handle_pptr,
                                         bool_t                free_list_node)
{
   if ((NULL == drv_ptr) || (NULL == writer_handle_pptr) || (NULL == *writer_handle_pptr))
   {
      return AR_EBADPARAM;
   }

   spr_stream_writer_t *strm_wr_ptr = *writer_handle_pptr;

   // De-register all the buffer clients and de allocate the memory
   for (uint32_t i = 0; i < strm_wr_ptr->num_channels; i++)
   {
      // De register if the valid cir_buf_handle is present.
      if (strm_wr_ptr->circ_buf_arr_ptr[i].head_chunk_ptr && strm_wr_ptr->wr_client_arr_ptr[i].circ_buf_ptr)
      {
#ifdef DEBUG_SPR_DRIVER
         AR_MSG_ISLAND(DBG_ERROR_PRIO,
                "_spr_stream_writer_free_util: De registering writer client for wr_id=%d, ch_id=%d",
                strm_wr_ptr->id,
                strm_wr_ptr->circ_buf_arr_ptr[i].id);
#endif
         circ_buf_deregister_client(&strm_wr_ptr->wr_client_arr_ptr[i]);
      }

      // TODO: THis check is redundant.
      if (strm_wr_ptr->circ_buf_arr_ptr[i].head_chunk_ptr)
      {
         circ_buf_free(&strm_wr_ptr->circ_buf_arr_ptr[i]);
      }
   }

   if (free_list_node)
   {
      // Remove the stream writer node from the driver list.
      spf_list_find_delete_node(&drv_ptr->stream_writer_list, *writer_handle_pptr, TRUE /*pool_used*/);

      // Free the stream writer memory.
      posal_memory_aligned_free(*writer_handle_pptr);
      *writer_handle_pptr = NULL;

      // Decrement number of writers.
      drv_ptr->num_writers--;
   }

   return AR_EOK;
}

ar_result_t spr_stream_reader_destroy(spr_driver_t *drv_ptr, spr_stream_reader_t **reader_handle)
{
   return _spr_stream_reader_free_util(drv_ptr, reader_handle, TRUE);
}

ar_result_t _spr_stream_reader_free_util(spr_driver_t *        drv_ptr,
                                         spr_stream_reader_t **reader_handle,
                                         bool_t                free_list_node)
{
   if ((NULL == drv_ptr) || (NULL == reader_handle) || (NULL == *reader_handle))
   {
      return AR_EBADPARAM;
   }

   // De-register all the buffer clients and de allocate the memory
   for (uint32_t i = 0; i < (*reader_handle)->num_channels; i++)
   {
      if ((*reader_handle)->rd_client_arr_ptr[i].circ_buf_ptr)
      {
         circ_buf_deregister_client(&(*reader_handle)->rd_client_arr_ptr[i]);
      }
   }

   if (free_list_node)
   {
      // Remove the stream writer node from the driver list.
      spf_list_find_delete_node(&drv_ptr->stream_reader_list, *reader_handle, TRUE /*pool_used*/);

      // Free the stream writer memory.
      posal_memory_aligned_free(*reader_handle);
      *reader_handle = NULL;

      // Decrement number of readers.
      drv_ptr->num_readers--;
   }

   return AR_EOK;
}

circ_buf_t *_spr_find_circ_buf(spr_driver_t *drv_ptr, uint32_t ch_id)
{
   // Iterate through the writers list and get the
   spf_list_node_t *strm_wr_list_ptr = drv_ptr->stream_writer_list;
   while (strm_wr_list_ptr)
   {
      spr_stream_writer_t *temp_strm_wr_ptr = (spr_stream_writer_t *)strm_wr_list_ptr->obj_ptr;

      for (uint32_t iter = 0; iter < temp_strm_wr_ptr->num_channels; iter++)
      {
         if (ch_id == temp_strm_wr_ptr->circ_buf_arr_ptr[iter].id)
         {
#ifdef DEBUG_SPR_DRIVER
            AR_MSG_ISLAND(DBG_HIGH_PRIO,
                   "_spr_find_circ_buf: Found circular buffer. wr_id = %d, ch_id=0x%x",
                   temp_strm_wr_ptr->id,
                   ch_id);
#endif
            return &(temp_strm_wr_ptr->circ_buf_arr_ptr[iter]);
         }
      }

      // Move to the next stream writer.
      LIST_ADVANCE(strm_wr_list_ptr);
   }

   return NULL;
}
