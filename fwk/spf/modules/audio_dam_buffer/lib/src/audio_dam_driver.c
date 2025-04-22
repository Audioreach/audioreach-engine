/**
 *   \file audio_dam_driver.c
 *   \brief
 *        This file contains implementation of Audio Dam buffer driver
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "audio_dam_driver.h"
#include "audio_dam_driver_i.h"
#include "circular_buffer_i.h"

#define MAX_CHANNELS_PER_STREAM 32

#ifdef DEBUG_CIRC_BUF_UTILS
#define DEBUG_AUDIO_DAM_DRIVER
#endif

#define MEM_ALLOC_8_BYTE_ALIGNED 8

#define ALIGN_8_BYTES(a) (((a) & (~(uint32_t)0x7)) + 8)
/*==============================================================================
   Local Declarations
==============================================================================*/

/*==============================================================================
   Local Function Implementation
==============================================================================*/
circ_buf_t *_audio_dam_find_circ_buf(audio_dam_driver_t *drv_ptr, uint32_t ch_id);

ar_result_t _audio_dam_stream_writer_free_util(audio_dam_driver_t *        drv_ptr,
                                               audio_dam_stream_writer_t **writer_handle_pptr,
                                               bool_t                      free_list_node);

ar_result_t _audio_dam_stream_reader_free_util(audio_dam_driver_t *        drv_ptr,
                                               audio_dam_stream_reader_t **reader_handle,
                                               bool_t                      free_list_node);

/*==============================================================================
   Public Function Implementation
==============================================================================*/
ar_result_t audio_dam_driver_init(audio_dam_init_args_t *init_args_ptr, audio_dam_driver_t *drv_ptr)
{
   if (NULL == drv_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "DAM_DRIVER: Init failed. bad input params.");
      return AR_EBADPARAM;
   }

   memset(drv_ptr, 0, sizeof(audio_dam_driver_t));

   drv_ptr->heap_id              = init_args_ptr->heap_id;
   drv_ptr->iid                  = init_args_ptr->iid;
   drv_ptr->preferred_chunk_size = init_args_ptr->preferred_chunk_size;

#ifdef DEBUG_AUDIO_DAM_DRIVER
   DAM_MSG(drv_ptr->iid,
           DBG_ERROR_PRIO,
           "DAM_DRIVER: Init done! heap_id %lu preferred_chunk_size %lu",
           drv_ptr->heap_id ,
           drv_ptr->preferred_chunk_size);
#endif

   return AR_EOK;
}

ar_result_t audio_dam_driver_deinit(audio_dam_driver_t *drv_ptr)
{
   ar_result_t result = AR_EOK;
   if (NULL == drv_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "audio_dam_driver_deinit: Failed. bad input params.");
      return AR_EBADPARAM;
   }

   // First Reader have to be freed and then writers can be freed.
   // Iterate through the readers list and free the readers.
   audio_dam_stream_reader_t *temp_strm_rd_ptr =
      (audio_dam_stream_reader_t *)spf_list_pop_head(&drv_ptr->stream_reader_list, TRUE /* pool_used */);
   while (temp_strm_rd_ptr)
   {
      // Free the stream reader circular buffers. But don't free the handle memory.
      result = _audio_dam_stream_reader_free_util(drv_ptr, &temp_strm_rd_ptr, FALSE);
      if (result != AR_EOK)
      {
         DAM_MSG(drv_ptr->iid,
                 DBG_ERROR_PRIO,
                 "audio_dam_driver_deinit: Failed. Couldn't free the stream reader, 0x%x. ",
                 temp_strm_rd_ptr->id);
      }

      // Free the stream Reader memory.
      posal_memory_aligned_free(temp_strm_rd_ptr);

      // Pop the next element from the list.
      temp_strm_rd_ptr =
         (audio_dam_stream_reader_t *)spf_list_pop_head(&drv_ptr->stream_reader_list, TRUE /* pool_used */);
   }

   // Iterate through the writers list and free the writers.
   audio_dam_stream_writer_t *temp_strm_wr_ptr =
      (audio_dam_stream_writer_t *)spf_list_pop_head(&drv_ptr->stream_writer_list, TRUE /* pool_used */);
   while (temp_strm_wr_ptr)
   {
      // Free the stream writer circular buffers. But don't free the handle memory.
      result = _audio_dam_stream_writer_free_util(drv_ptr, &temp_strm_wr_ptr, FALSE);
      if (result != AR_EOK)
      {
         DAM_MSG(drv_ptr->iid,
                 DBG_ERROR_PRIO,
                 "audio_dam_driver_deinit: Failed. Couldn't free the stream writer, 0x%x ",
                 temp_strm_wr_ptr->id);
      }

      // Free the stream writer memory.
      posal_memory_aligned_free(temp_strm_wr_ptr);

      // Pop the next element from the list.
      temp_strm_wr_ptr =
         (audio_dam_stream_writer_t *)spf_list_pop_head(&drv_ptr->stream_writer_list, TRUE /* pool_used */);
   }

   if (drv_ptr->ch_frame_scratch_buf_ptr)
   {
      posal_memory_free(drv_ptr->ch_frame_scratch_buf_ptr);
   }

   memset(drv_ptr, 0, sizeof(audio_dam_driver_t));

   return AR_EOK;
}

ar_result_t audio_dam_stream_writer_create(audio_dam_driver_t *        drv_ptr,
                                           uint32_t                    base_buffer_size,
                                           uint32_t                    num_channels,
                                           uint32_t *                  ch_id_arr,
                                           audio_dam_stream_writer_t **writer_handle_pptr)
{
   ar_result_t result = AR_EOK;
   if ((NULL == drv_ptr) || (NULL == writer_handle_pptr))
   {
      AR_MSG(DBG_ERROR_PRIO, "DAM_DRIVER: Writer allocation failed. Bad input params.");
      return AR_EBADPARAM;
   }

   // Validate the init params
   if ((num_channels > MAX_CHANNELS_PER_STREAM) || (0 == num_channels) || (NULL == ch_id_arr))
   {
      DAM_MSG(drv_ptr->iid,
              DBG_ERROR_PRIO,
              "DAM_DRIVER: Writer allocation failed. num channels=%d not supported. ",
              num_channels);
      return AR_EBADPARAM;
   }

   // Allocate memory for the stream writer handle.
   uint32_t handle_size = ALIGN_8_BYTES(sizeof(audio_dam_stream_writer_t));
   handle_size += (ALIGN_8_BYTES(sizeof(circ_buf_t)) + ALIGN_8_BYTES(sizeof(circ_buf_client_t))) * num_channels;

   int8_t *blob_ptr = (int8_t *)posal_memory_aligned_malloc(handle_size, MEM_ALLOC_8_BYTE_ALIGNED, drv_ptr->heap_id);
   if (NULL == blob_ptr)
   {
      return AR_ENOMEMORY;
   }
   memset(blob_ptr, 0, handle_size);

   // Init handle fields.
   *writer_handle_pptr                     = (audio_dam_stream_writer_t *)blob_ptr;
   (*writer_handle_pptr)->base_buffer_size = base_buffer_size;
   (*writer_handle_pptr)->num_channels     = num_channels;
   (*writer_handle_pptr)->driver_ptr       = drv_ptr;
   blob_ptr += ALIGN_8_BYTES(sizeof(audio_dam_stream_writer_t));

   // Assign the circ buf array pointer
   (*writer_handle_pptr)->circ_buf_arr_ptr = (circ_buf_t *)(blob_ptr);
   blob_ptr += (ALIGN_8_BYTES(sizeof(circ_buf_t))) * num_channels;

   // Assign circ buf writer handle array pointer
   (*writer_handle_pptr)->wr_client_arr_ptr = (circ_buf_client_t *)(blob_ptr);

   // Create circular buffer and get the writer client handles for each channel.
   audio_dam_stream_writer_t *str_wr_ptr = *writer_handle_pptr;

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
      if (NULL != _audio_dam_find_circ_buf(drv_ptr, ch_id))
      {
         DAM_MSG(drv_ptr->iid, DBG_ERROR_PRIO, "DAM_DRIVER: writer channel id = 0x%x is already used.", ch_id);
         audio_dam_stream_writer_destroy(drv_ptr, writer_handle_pptr);
         return AR_EFAILED;
      }

      // Allocate circular buffer and get the buffer handle.
      buf_res = circ_buf_alloc(&str_wr_ptr->circ_buf_arr_ptr[iter],
                               str_wr_ptr->base_buffer_size,
                               drv_ptr->preferred_chunk_size,
                               ch_id,
                               drv_ptr->heap_id);
      if (buf_res != CIRCBUF_SUCCESS)
      {
         DAM_MSG(drv_ptr->iid, DBG_ERROR_PRIO, "DAM_DRIVER: Circular buffer allocation failed. iter = %d", iter);
         audio_dam_stream_writer_destroy(drv_ptr, writer_handle_pptr);
         return AR_EFAILED;
      }

      // Register the writer clients with the circular buffer and get the handles.
      result = circ_buf_register_client(&str_wr_ptr->circ_buf_arr_ptr[iter],
                                        FALSE,
                                        drv_ptr->heap_id,
                                        0,
                                        &str_wr_ptr->wr_client_arr_ptr[iter]);
      if (buf_res != CIRCBUF_SUCCESS)
      {
         DAM_MSG(drv_ptr->iid, DBG_ERROR_PRIO, "DAM_DRIVER: Writer client registration failed. iter = %d", iter);
         audio_dam_stream_writer_destroy(drv_ptr, writer_handle_pptr);
         return AR_EFAILED;
      }
   }

#ifdef DEBUG_AUDIO_DAM_DRIVER
   DAM_MSG(drv_ptr->iid, DBG_HIGH_PRIO, "DAM_DRIVER: Writer Allocation successful num_channel =%d", num_channels);
#endif

   return result;
}

static ar_result_t audio_dam_stream_reader_virt_mode_util_(
   audio_dam_driver_t                            *drv_ptr,
   uint32_t                                       num_channels,
   param_id_audio_dam_imcl_virtual_writer_info_t *virt_wr_cfg_ptr,
   audio_dam_stream_reader_t                    **reader_handle_pptr)
{
   if (num_channels > virt_wr_cfg_ptr->num_channels)
   {
      DAM_MSG(drv_ptr->iid,
              DBG_ERROR_PRIO,
              "DAM_DRIVER: Reader create failed: num_channels invalid reader's num_channels=%lu > max %lu",
              num_channels,
              virt_wr_cfg_ptr->num_channels);
      return AR_EFAILED;
   }

   uint32_t handle_size = ALIGN_8_BYTES(sizeof(audio_dam_stream_reader_t)) +
                          ALIGN_8_BYTES(sizeof(audio_dam_stream_reader_virtual_buf_info_t)) +
                          ((sizeof(capi_buf_t) * virt_wr_cfg_ptr->num_channels));

   int8_t *blob_ptr = (int8_t *)posal_memory_aligned_malloc(handle_size, MEM_ALLOC_8_BYTE_ALIGNED, drv_ptr->heap_id);
   if (NULL == blob_ptr)
   {
      return AR_EFAILED;
   }
   memset(blob_ptr, 0, handle_size);

   // Init handle fields.
   *reader_handle_pptr                 = (audio_dam_stream_reader_t *)blob_ptr;
   (*reader_handle_pptr)->num_channels = num_channels;
   (*reader_handle_pptr)->driver_ptr   = drv_ptr;

   blob_ptr += ALIGN_8_BYTES(sizeof(audio_dam_stream_reader_t));

   (*reader_handle_pptr)->virt_buf_ptr = (audio_dam_stream_reader_virtual_buf_info_t *)blob_ptr;
   blob_ptr += ALIGN_8_BYTES(sizeof(audio_dam_stream_reader_virtual_buf_info_t));

   (*reader_handle_pptr)->virt_buf_ptr->out_scratch_bufs = (capi_buf_t *)blob_ptr;

   /** client is expected to alloc memory for this handle, no need to malloc and copy*/
   (*reader_handle_pptr)->virt_buf_ptr->cfg_ptr = virt_wr_cfg_ptr;

   return AR_EOK;
}

static ar_result_t audio_dam_stream_reader_circ_buf_create_util_(audio_dam_driver_t *drv_ptr,
                                                                 POSAL_HEAP_ID       chunk_heap_id,
                                                                 uint32_t            downstream_setup_duration_in_us,
                                                                 uint32_t            num_channels,
                                                                 uint32_t *          ch_id_arr_ptr,
                                                                 audio_dam_stream_reader_t **reader_handle_pptr)
{
   uint32_t downstream_setup_duration_in_bytes =
      audio_dam_compute_buffer_size_in_bytes(drv_ptr, downstream_setup_duration_in_us);

   // Check if the input channel IDs are valid.
   bool_t ch_doesnt_exist = FALSE;
   for (uint32_t iter = 0; iter < num_channels; iter++)
   {
      // Find the channel buffer using the ch_id
      circ_buf_t *circ_buf_ptr = _audio_dam_find_circ_buf(drv_ptr, ch_id_arr_ptr[iter]);
      if (NULL == circ_buf_ptr)
      {
         ch_doesnt_exist = TRUE;
         DAM_MSG(drv_ptr->iid,
                 DBG_MED_PRIO,
                 "DAM_DRIVER: Warning! Reader not ready to create: ch_id= 0x%x doesn't exist.",
                 ch_id_arr_ptr[iter]);
      }
   }

   // Return if any of the channel ID doesn't exist.
   if (ch_doesnt_exist)
   {
#ifdef DEBUG_AUDIO_DAM_DRIVER
      DAM_MSG(drv_ptr->iid, DBG_ERROR_PRIO, "DAM_DRIVER: Reader allocation failed.");
#endif
      return AR_ENOTREADY;
   }

   // Allocate memory for the stream reader handle.
   uint32_t handle_size = ALIGN_8_BYTES(sizeof(audio_dam_stream_reader_t));
   handle_size += ALIGN_8_BYTES(sizeof(circ_buf_client_t *) * num_channels); // size of pointer array
   handle_size += ALIGN_8_BYTES(sizeof(circ_buf_client_t)) * num_channels;   // size of elements for the array.

   int8_t *blob_ptr = (int8_t *)posal_memory_aligned_malloc(handle_size, MEM_ALLOC_8_BYTE_ALIGNED, drv_ptr->heap_id);
   if (NULL == blob_ptr)
   {
      return AR_EFAILED;
   }
   memset(blob_ptr, 0, handle_size);

   // Init handle fields.
   *reader_handle_pptr                          = (audio_dam_stream_reader_t *)blob_ptr;
   (*reader_handle_pptr)->req_base_bufffer_size = downstream_setup_duration_in_bytes;
   (*reader_handle_pptr)->num_channels          = num_channels;
   (*reader_handle_pptr)->driver_ptr            = drv_ptr;
   blob_ptr += ALIGN_8_BYTES(sizeof(audio_dam_stream_reader_t));

   // Assign reader client array pointer.
   (*reader_handle_pptr)->rd_client_ptr_arr = (circ_buf_client_t **)blob_ptr;
   blob_ptr += ALIGN_8_BYTES(sizeof(circ_buf_client_t *) * num_channels);

   for (uint32_t idx = 0; idx < num_channels; idx++)
   {
      (*reader_handle_pptr)->rd_client_ptr_arr[idx] = (circ_buf_client_t *)blob_ptr;
      blob_ptr += ALIGN_8_BYTES(sizeof(circ_buf_client_t));
   }

   // Create circular buffer and get the writer client handles for each channel.
   audio_dam_stream_reader_t *str_rd_ptr = *reader_handle_pptr;

   for (uint32_t iter = 0; iter < num_channels; iter++)
   {
      circbuf_result_t buf_res = CIRCBUF_SUCCESS;
      uint32_t         ch_id   = ch_id_arr_ptr[iter];

      // Find the channel buffer using the ch_id
      circ_buf_t *circ_buf_ptr = _audio_dam_find_circ_buf(drv_ptr, ch_id);

      // Register read client for with the circular buffer.
      buf_res = circ_buf_register_client(circ_buf_ptr,
                                         TRUE,
                                         chunk_heap_id,
                                         downstream_setup_duration_in_bytes,
                                         str_rd_ptr->rd_client_ptr_arr[iter]);
      if (buf_res != CIRCBUF_SUCCESS)
      {
#ifdef DEBUG_AUDIO_DAM_DRIVER
         DAM_MSG(drv_ptr->iid,
                 DBG_ERROR_PRIO,
                 "DAM_DRIVER: Reader client regsitration failed. ch_id=%d, iter = %d",
                 ch_id,
                 iter);
#endif
         return AR_EFAILED;
      }
   }
   return AR_EOK;
}

ar_result_t audio_dam_stream_reader_create(audio_dam_driver_t *drv_ptr,
                                           POSAL_HEAP_ID       chunk_heap_id,
                                           uint32_t            downstream_setup_duration_in_us,
                                           uint32_t            num_channels,
                                           uint32_t *          ch_id_arr_ptr,
                                           param_id_audio_dam_imcl_virtual_writer_info_t *virt_wr_cfg_ptr,
                                           audio_dam_stream_reader_t **                   reader_handle_pptr)
{
   ar_result_t result = AR_EOK;
   if ((NULL == drv_ptr) || (NULL == reader_handle_pptr))
   {
#ifdef DEBUG_AUDIO_DAM_DRIVER
      AR_MSG(DBG_ERROR_PRIO, "DAM_DRIVER: Reader allocation failed. Bad input params.");
#endif
      return AR_EBADPARAM;
   }

   // Validate the init params
   if ((num_channels > MAX_CHANNELS_PER_STREAM) || (0 == num_channels) || (NULL == ch_id_arr_ptr))
   {
      return AR_EBADPARAM;
   }

   if (virt_wr_cfg_ptr)
   {
      result = audio_dam_stream_reader_virt_mode_util_(drv_ptr, num_channels, virt_wr_cfg_ptr, reader_handle_pptr);
   }
   else
   {
      result = audio_dam_stream_reader_circ_buf_create_util_(drv_ptr,
                                                             chunk_heap_id,
                                                             downstream_setup_duration_in_us,
                                                             num_channels,
                                                             ch_id_arr_ptr,
                                                             reader_handle_pptr);
   }

   /** check if writer creation is successful*/
   if (AR_FAILED(result))
   {
      if (AR_ENOTREADY == result)
      {
         return AR_ENOTREADY;
      }
      result = audio_dam_stream_reader_destroy(drv_ptr, reader_handle_pptr);
      return result;
   }

   // Add the stream Reader handle to the driver list.
   spf_list_insert_tail(&drv_ptr->stream_reader_list, *reader_handle_pptr, drv_ptr->heap_id, TRUE /* use_pool*/);

   // Update num readers and assign reader ID.
   (*reader_handle_pptr)->id = drv_ptr->num_readers++;

#ifdef DEBUG_AUDIO_DAM_DRIVER
   DAM_MSG(drv_ptr->iid, DBG_HIGH_PRIO, "DAM_DRIVER: Reader Allocation successful num_channel =%d", num_channels);
#endif
   return AR_EOK;
}

ar_result_t audio_dam_stream_reader_ch_order_sort(audio_dam_stream_reader_t *reader_handle,
                                                  uint32_t                   num_chs_to_output,
                                                  uint32_t *                 ch_ids_to_output)
{
   if (!reader_handle)
   {
      AR_MSG(DBG_ERROR_PRIO, "DAM_DRIVER: Stream reader handle is NULL");
      return AR_EFAILED;
   }

   if (audio_dam_driver_is_virtual_writer_mode(reader_handle))
   {
      AR_MSG(DBG_HIGH_PRIO, "DAM_DRIVER: Warning! Stream reader channel order sorting unsupported in virt mode");
      return AR_EOK;
   }

   // Check if all the channel IDs in the ch_ids_to_output array exist in stream reader.
   for (uint32_t new_idx = 0; new_idx < num_chs_to_output; new_idx++)
   {
      uint32_t ch_id = ch_ids_to_output[new_idx];

      // Check if the ch_id exist in the stream reader channels list.
      bool_t   found   = FALSE;
      uint32_t old_idx = 0;
      for (old_idx = 0; (old_idx < reader_handle->num_channels) && !found; old_idx++)
      {
         if (!reader_handle->rd_client_ptr_arr[old_idx]->circ_buf_ptr)
         {
            continue;
         }

         if (ch_id == reader_handle->rd_client_ptr_arr[old_idx]->circ_buf_ptr->id)
         {
            found = TRUE;
         }
      }

      if (!found)
      {
         DAM_MSG(reader_handle->driver_ptr->iid,
                 DBG_ERROR_PRIO,
                 "DAM_DRIVER: Stream reader channel sort failed. ch_id=%d doesn't exist.",
                 ch_id);
         return AR_EFAILED;
      }
   }

   for (uint32_t new_idx = 0; new_idx < num_chs_to_output; new_idx++)
   {
      uint32_t ch_id = ch_ids_to_output[new_idx];

      // Find the channel position in the stream reader list.
      bool_t   found   = FALSE;
      uint32_t old_idx = 0;
      for (old_idx = 0; old_idx < reader_handle->num_channels; old_idx++)
      {
         if (!reader_handle->rd_client_ptr_arr[old_idx]->circ_buf_ptr)
         {
            continue;
         }

         if (ch_id == reader_handle->rd_client_ptr_arr[old_idx]->circ_buf_ptr->id)
         {
            DAM_MSG(reader_handle->driver_ptr->iid,
                    DBG_HIGH_PRIO,
                    "DAM_DRIVER: ch_id=0x%x index is changed from %lu to %lu",
                    reader_handle->rd_client_ptr_arr[old_idx]->circ_buf_ptr->id,
                    old_idx,
                    new_idx);
            found = TRUE;
            break;
         }
      }

      if (found && (new_idx != old_idx))
      {
         // Swap channel from "idx" position to "out_loop" position in array.
         circ_buf_client_t *temp                   = reader_handle->rd_client_ptr_arr[new_idx];
         reader_handle->rd_client_ptr_arr[new_idx] = reader_handle->rd_client_ptr_arr[old_idx];
         reader_handle->rd_client_ptr_arr[old_idx] = temp;
      }
   }

   return AR_EOK;
}

ar_result_t audio_dam_stream_reader_req_resize(audio_dam_stream_reader_t *reader_handle,
                                               uint32_t                   resize_in_us,
                                               POSAL_HEAP_ID              heap_id)
{
   ar_result_t result = AR_EOK;

   if (NULL == reader_handle)
   {
#ifdef DEBUG_AUDIO_DAM_DRIVER
      AR_MSG(DBG_ERROR_PRIO, "DAM_DRIVER: Stream Reader resize failed. Bad input params.");
#endif
      return AR_EFAILED;
   }

   /** ignore resize in virtual writer mode*/
   if (audio_dam_driver_is_virtual_writer_mode(reader_handle))
   {
      return AR_EOK;
   }

   uint32_t requested_alloc_size_in_bytes =
      audio_dam_compute_buffer_size_in_bytes(reader_handle->driver_ptr, resize_in_us);

   // Iterate through all the Readers channel buffers and request to resize the buffer.
   for (uint32_t iter = 0; iter < reader_handle->num_channels; iter++)
   {
      if (!reader_handle->rd_client_ptr_arr[iter]->circ_buf_ptr)
      {
         continue;
      }
      if (CIRCBUF_SUCCESS !=
          circ_buf_read_client_resize(reader_handle->rd_client_ptr_arr[iter], requested_alloc_size_in_bytes, heap_id))
      {
#ifdef DEBUG_AUDIO_DAM_DRIVER
         DAM_MSG(reader_handle->driver_ptr->iid,
                 DBG_ERROR_PRIO,
                 "DAM_DRIVER: Stream reader, resize failed. ch_id=%d, iter = %d, result 0x%lx",
                 reader_handle->rd_client_ptr_arr[iter]->circ_buf_ptr->id,
                 iter);
#endif
         return AR_EFAILED;
      }
   }

   DAM_MSG(reader_handle->driver_ptr->iid,
           DBG_HIGH_PRIO,
           "DAM_DRIVER: Stream reader resize done. requested_size:(%lu us, %lu bytes)",
           resize_in_us,
           requested_alloc_size_in_bytes);

   return result;
}

ar_result_t audio_dam_stream_writer_destroy(audio_dam_driver_t *drv_ptr, audio_dam_stream_writer_t **writer_handle)
{
   return _audio_dam_stream_writer_free_util(drv_ptr, writer_handle, TRUE);
}

ar_result_t _audio_dam_stream_writer_free_util(audio_dam_driver_t *        drv_ptr,
                                               audio_dam_stream_writer_t **writer_handle_pptr,
                                               bool_t                      free_list_node)
{
   if ((NULL == drv_ptr) || (NULL == writer_handle_pptr) || (NULL == *writer_handle_pptr))
   {
      return AR_EBADPARAM;
   }

   /** Note that in virtual writer mode writer handle is expected to null */

   audio_dam_stream_writer_t *strm_wr_ptr = *writer_handle_pptr;

   // De-register all the buffer clients and de allocate the memory
   for (uint32_t i = 0; i < strm_wr_ptr->num_channels; i++)
   {
      // De register if the valid cir_buf_handle is present.
      if (strm_wr_ptr->circ_buf_arr_ptr[i].head_chunk_ptr && strm_wr_ptr->wr_client_arr_ptr[i].circ_buf_ptr)
      {
#ifdef DEBUG_AUDIO_DAM_DRIVER
         AR_MSG(DBG_HIGH_PRIO,
                "_audio_dam_stream_writer_free_util: De registering writer client for wr_id=%d, ch_id=%d",
                strm_wr_ptr->id,
                strm_wr_ptr->circ_buf_arr_ptr[i].id);
#endif
         circ_buf_deregister_client(&strm_wr_ptr->wr_client_arr_ptr[i]);
      }

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

ar_result_t audio_dam_stream_reader_destroy(audio_dam_driver_t *drv_ptr, audio_dam_stream_reader_t **reader_handle)
{
   return _audio_dam_stream_reader_free_util(drv_ptr, reader_handle, TRUE);
}

ar_result_t _audio_dam_stream_reader_free_util(audio_dam_driver_t *        drv_ptr,
                                               audio_dam_stream_reader_t **reader_handle,
                                               bool_t                      free_list_node)
{
   if ((NULL == drv_ptr) || (NULL == reader_handle) || (NULL == *reader_handle))
   {
      return AR_EBADPARAM;
   }

   /** ignore resize in virtual writer mode*/
   if (audio_dam_driver_is_virtual_writer_mode(*reader_handle))
   {
      // do not free this ptr, its expected to be freed by the client
      (*reader_handle)->virt_buf_ptr->cfg_ptr = NULL;

      // dont free the virt buf handle, its freed as part fo reader handle free
      (*reader_handle)->virt_buf_ptr = NULL;
   }
   else
   {
      // De-register all the buffer clients and de allocate the memory
      for (uint32_t i = 0; i < (*reader_handle)->num_channels; i++)
      {
         if ((*reader_handle)->rd_client_ptr_arr[i]->circ_buf_ptr)
         {
            circ_buf_deregister_client((*reader_handle)->rd_client_ptr_arr[i]);
         }
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

circ_buf_t *_audio_dam_find_circ_buf(audio_dam_driver_t *drv_ptr, uint32_t ch_id)
{
   // Iterate through the writers list and get the
   spf_list_node_t *strm_wr_list_ptr = drv_ptr->stream_writer_list;
   while (strm_wr_list_ptr)
   {
      audio_dam_stream_writer_t *temp_strm_wr_ptr = (audio_dam_stream_writer_t *)strm_wr_list_ptr->obj_ptr;

      for (uint32_t iter = 0; iter < temp_strm_wr_ptr->num_channels; iter++)
      {
         if (ch_id == temp_strm_wr_ptr->circ_buf_arr_ptr[iter].id)
         {
#ifdef DEBUG_AUDIO_DAM_DRIVER
            AR_MSG(DBG_HIGH_PRIO,
                   "_audio_dam_find_circ_buf: Found circular buffer. wr_id = %d, ch_id=0x%x",
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

ar_result_t audio_dam_set_raw_compressed_mf(audio_dam_driver_t *drv_ptr,
                                            uint32_t            frame_max_data_len_in_us,
                                            uint32_t            frame_max_data_len_in_bytes)
{
   drv_ptr->is_raw_compressed = TRUE;

   // get frame length and etc from Input media format.
   drv_ptr->frame_max_data_len_in_us    = frame_max_data_len_in_us;
   drv_ptr->frame_max_data_len_in_bytes = frame_max_data_len_in_bytes;

   DAM_MSG(drv_ptr->iid,
           DBG_HIGH_PRIO,
           "frame_max_data_len in_us:%lu in_bytes:%lu",
           frame_max_data_len_in_us,
           frame_max_data_len_in_bytes);

   if (drv_ptr->ch_frame_scratch_buf_ptr)
   {
      posal_memory_free(drv_ptr->ch_frame_scratch_buf_ptr);
      drv_ptr->ch_frame_scratch_buf_ptr = NULL;
   }

   uint32_t scratch_frame_total_len = AUDIO_DAM_GET_TOTAL_FRAME_LEN(drv_ptr->frame_max_data_len_in_bytes);
   if (NULL ==
       (drv_ptr->ch_frame_scratch_buf_ptr = (int8_t *)posal_memory_malloc(scratch_frame_total_len, drv_ptr->heap_id)))
   {
      DAM_MSG(drv_ptr->iid, DBG_ERROR_PRIO, "Failed allocating scrtach memory for raw compressed ");
      return AR_ENOMEMORY;
   }

   return AR_EOK;
}

ar_result_t audio_dam_set_pcm_mf(audio_dam_driver_t *drv_ptr, uint32_t sampling_rate, uint32_t bytes_per_sample)
{
   drv_ptr->is_raw_compressed = FALSE;

   // get frame length and etc from Input media format.
   drv_ptr->frame_max_data_len_in_us    = 0;
   drv_ptr->frame_max_data_len_in_bytes = 0;

   drv_ptr->sampling_rate    = sampling_rate;
   drv_ptr->bytes_per_sample = bytes_per_sample;

   drv_ptr->bytes_per_one_ms = (sampling_rate / 1000) * bytes_per_sample;

   return AR_EOK;
}
