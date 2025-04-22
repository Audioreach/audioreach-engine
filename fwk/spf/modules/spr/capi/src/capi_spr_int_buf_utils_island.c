/*==========================================================================
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 * =========================================================================*/

/**
 *   \file capi_spr_int_buf_utils.c
 *   \brief
 *        This file contains CAPI implementation of Splitter Renderer Module internal buffer utilities
 */

#include "capi_spr_i.h"

/*==============================================================================
   Local Function forward declaration
==============================================================================*/
static uint64_t spr_calc_capi_stream_size_in_us(capi_stream_data_v2_t *strm_ptr, capi_media_fmt_v2_t *media_fmt_ptr);
static capi_err_t capi_spr_overwrite_int_buf_data(capi_spr_t *           me_ptr,
                                                  spr_int_buffer_t *     int_buf_ptr,
                                                  capi_stream_data_v2_t *input_ptr,
                                                  capi_media_fmt_v2_t *  media_fmt_ptr);
/*==============================================================================
   Function Implementation
==============================================================================*/

/*------------------------------------------------------------------------------
  Function name: capi_spr_overwrite_int_buf_data
   When the amount of data held inside SPR exceeds the configured value, this
   function is used to drop the data at the head of the int buffer list to store
   the new data. This is not expected to kick in unless the hold buffer duration
   is insufficient.
 * ------------------------------------------------------------------------------*/
static capi_err_t capi_spr_overwrite_int_buf_data(capi_spr_t *           me_ptr,
                                                  spr_int_buffer_t *     int_buf_ptr,
                                                  capi_stream_data_v2_t *input_ptr,
                                                  capi_media_fmt_v2_t *  media_fmt_ptr)
{
   capi_err_t result          = CAPI_EOK;
   bool_t     CHECK_HEAD_NODE = TRUE;
   bool_t     DROP_DATA       = TRUE;

   uint64_t cur_buffer_us = spr_calc_capi_stream_size_in_us(input_ptr, media_fmt_ptr);
   uint64_t total_drop_us = 0;
   while (cur_buffer_us)
   {
      capi_stream_data_v2_t *drop_strm_ptr =
         (capi_stream_data_v2_t *)capi_spr_get_list_head_obj_ptr(int_buf_ptr->buf_list_ptr);

      uint64_t drop_buffer_us = spr_calc_capi_stream_size_in_us(drop_strm_ptr, media_fmt_ptr);
      total_drop_us += drop_buffer_us;

      result =
         capi_spr_destroy_int_buf_node(me_ptr, int_buf_ptr, drop_strm_ptr, CHECK_HEAD_NODE, media_fmt_ptr, DROP_DATA);

      if (cur_buffer_us >= drop_buffer_us)
      {
         cur_buffer_us -= drop_buffer_us;
      }
      else
      {
         cur_buffer_us = 0;
      }
   }

   SPR_MSG_ISLAND(me_ptr->miid,
           DBG_HIGH_PRIO,
           "Dropping data %d (in us) to accommodate buffer %d (in us)",
           total_drop_us,
           spr_calc_capi_stream_size_in_us(input_ptr, media_fmt_ptr));

   return result;
}

/*------------------------------------------------------------------------------
  Function name: capi_spr_get_list_head_obj_ptr
  Returns the obj pointer stored in the head node of the list
 * ------------------------------------------------------------------------------*/
void *capi_spr_get_list_head_obj_ptr(void *list_ptr)
{
   if (list_ptr)
   {
      return ((spf_list_node_t *)list_ptr)->obj_ptr;
   }

   return NULL;
}

/*------------------------------------------------------------------------------
  Function name: capi_spr_destroy_int_buf_node
  Destroys an internal buffer node and attempts to check if the incoming stream
  node is the head node of the list or not
 * ------------------------------------------------------------------------------*/
capi_err_t capi_spr_destroy_int_buf_node(capi_spr_t *         me_ptr,
                                         spr_int_buffer_t *   int_buf_ptr,
                                         void *               buf_node_ptr,
                                         bool_t               check_if_head_node,
                                         capi_media_fmt_v2_t *media_fmt_ptr,
                                         bool_t               drop_data)
{
   if (!buf_node_ptr || !int_buf_ptr)
   {
      return CAPI_EOK;
   }

   // internal buffer lists exist only when avsync is enabled & the hold buffer is configured
   if (!(is_spr_avsync_enabled(me_ptr->avsync_ptr) && (spr_avsync_is_hold_buf_configured(me_ptr->avsync_ptr))))
   {
      SPR_MSG_ISLAND(me_ptr->miid,
              DBG_ERROR_PRIO,
              "attempting to destroy int_buf node when avsync_enabled = %d and hold_buf_configured = %d",
              is_spr_avsync_enabled(me_ptr->avsync_ptr),
              spr_avsync_is_hold_buf_configured(me_ptr->avsync_ptr));
      return CAPI_EUNSUPPORTED;
   }

   capi_stream_data_v2_t *strm_ptr  = (capi_stream_data_v2_t *)buf_node_ptr;
   spf_list_node_t **     list_pptr = &int_buf_ptr->buf_list_ptr;
   bool_t                 DROP_MD   = TRUE;

   // This is set to TRUE only from the process context to check if the input processed is from the hold buffer list or
   // not
   if (check_if_head_node)
   {
      bool_t POOL_USED = TRUE;

      if (!spr_int_buf_is_head_node(*list_pptr, buf_node_ptr))
      {
#ifdef SPR_INT_BUF_DEBUG
         SPR_MSG_ISLAND(me_ptr->miid, DBG_LOW_PRIO, "Incoming node 0x%p not head of the provided list", buf_node_ptr);
#endif
         return CAPI_EOK;
      }

      // pop the list explicitly to de-allocate the node memory
      (void)spf_list_pop_head(list_pptr, POOL_USED);

#ifdef SPR_INT_BUF_DEBUG
      SPR_MSG_ISLAND(me_ptr->miid, DBG_LOW_PRIO, "Incoming node 0x%p head of the provided list", buf_node_ptr);
#endif
   }

   if (strm_ptr->metadata_list_ptr)
   {
      SPR_MSG_ISLAND(me_ptr->miid, DBG_HIGH_PRIO, "warning: int buffer node being destroyed with metadata");
   }

#ifdef SPR_INT_BUF_DEBUG
   SPR_MSG_ISLAND(me_ptr->miid, DBG_LOW_PRIO, "destroy int_buf_obj_ptr 0x%X", strm_ptr);

   if (int_buf_ptr->is_used_for_hold)
   {
      SPR_MSG_ISLAND(me_ptr->miid,
              DBG_LOW_PRIO,
              "Destroying cached int buffer with TS %ld from hold list",
              strm_ptr->timestamp);
   }
   else
   {
      SPR_MSG_ISLAND(me_ptr->miid,
              DBG_LOW_PRIO,
              "Destroying cached int buffer with TS %ld from int list",
              strm_ptr->timestamp);
   }

#endif

   uint64_t cur_buffer_us = spr_calc_capi_stream_size_in_us(strm_ptr, media_fmt_ptr);

   if (int_buf_ptr->cur_fill_size_us >= cur_buffer_us)
   {
      int_buf_ptr->cur_fill_size_us -= cur_buffer_us;
   }
   if (me_ptr->avsync_ptr->data_held_us >= cur_buffer_us)
   {
      me_ptr->avsync_ptr->data_held_us -= cur_buffer_us;
   }

#ifdef SPR_INT_BUF_DEBUG
   SPR_MSG_ISLAND(me_ptr->miid, DBG_LOW_PRIO, "int_buf cur fill size %d", int_buf_ptr->cur_fill_size_us);
#endif

   // Free memory pointers allocated for int buf list node.
   // Check free metadata if any
   spr_handle_metadata(me_ptr, (capi_stream_data_t **)&strm_ptr, NULL, DROP_MD);

   if (strm_ptr->buf_ptr)
   {
      if (drop_data)
      {
         SPR_MSG_ISLAND(me_ptr->miid,
                 DBG_LOW_PRIO,
                 "dropping data:  %d in bytes per ch from int buffer",
                 strm_ptr->buf_ptr->actual_data_len);
      }

      for (uint32_t num_chan = 0; num_chan < strm_ptr->bufs_num; num_chan++)
      {
         capi_buf_t *drop_buf_ptr = &(strm_ptr->buf_ptr[num_chan]);
         posal_memory_free(drop_buf_ptr->data_ptr);
      }

      posal_memory_free(strm_ptr->buf_ptr);
   }

   posal_memory_free(strm_ptr);

   strm_ptr = NULL;

   return CAPI_EOK;
}

/*------------------------------------------------------------------------------
  Function name: capi_spr_move_int_buf_node
   Moves the int buffer node from one list to another. Used when a node from
   the cached media format list has to be held.
 * ------------------------------------------------------------------------------*/
capi_err_t capi_spr_move_int_buf_node(spf_list_node_t *    node_ptr,
                                      spr_int_buffer_t *   src_buf_ptr,
                                      spr_int_buffer_t *   dst_buf_ptr,
                                      capi_media_fmt_v2_t *media_fmt_ptr)
{
   capi_err_t result = CAPI_EOK;

   if (!(node_ptr && src_buf_ptr && dst_buf_ptr && media_fmt_ptr))
   {
      return CAPI_EBADPARAM;
   }

   capi_stream_data_v2_t *strm_ptr   = (capi_stream_data_v2_t *)(node_ptr->obj_ptr);
   uint64_t               buf_dur_us = spr_calc_capi_stream_size_in_us(strm_ptr, media_fmt_ptr);

   spf_list_move_node_to_another_list((spf_list_node_t **)&dst_buf_ptr->buf_list_ptr,
                                      node_ptr,
                                      (spf_list_node_t **)&src_buf_ptr->buf_list_ptr);

   src_buf_ptr->cur_fill_size_us -= buf_dur_us;
   dst_buf_ptr->cur_fill_size_us += buf_dur_us;

   return result;
}

/*------------------------------------------------------------------------------
  Function name: capi_spr_calc_stream_size_in_us
  Returns the obj pointer stored in the head node of the list
 * ------------------------------------------------------------------------------*/
static uint64_t spr_calc_capi_stream_size_in_us(capi_stream_data_v2_t *strm_ptr, capi_media_fmt_v2_t *media_fmt_ptr)
{
   uint64_t dur_us = 0;
   if (!(strm_ptr && strm_ptr->buf_ptr && media_fmt_ptr))
   {
      return dur_us;
   }

   uint32_t  NUM_CH_1       = 1;
   uint64_t *FRACT_TIME_PTR = NULL;

   dur_us = capi_cmn_bytes_to_us(strm_ptr->buf_ptr[0].actual_data_len,
                                 media_fmt_ptr->format.sampling_rate,
                                 media_fmt_ptr->format.bits_per_sample,
                                 NUM_CH_1,
                                 FRACT_TIME_PTR);

   return dur_us;
}

/*------------------------------------------------------------------------------
  Function name: capi_spr_create_int_buf_node
  Creates an internal buffer node to store the incoming capi stream
 * ------------------------------------------------------------------------------*/
capi_err_t capi_spr_create_int_buf_node(capi_spr_t *           me_ptr,
                                        spr_int_buffer_t *     int_buf_ptr,
                                        capi_stream_data_v2_t *input_ptr,
                                        POSAL_HEAP_ID          heap_id,
                                        capi_media_fmt_v2_t *  media_fmt_ptr)
{
   if (!int_buf_ptr || !input_ptr || !media_fmt_ptr || !me_ptr)
   {
      AR_MSG_ISLAND(DBG_ERROR_PRIO, "capi_spr: received invalid params to create int_buf node");
      return CAPI_EBADPARAM;
   }

   if (!(is_spr_avsync_enabled(me_ptr->avsync_ptr) && (spr_avsync_is_hold_buf_configured(me_ptr->avsync_ptr))))
   {
      SPR_MSG_ISLAND(me_ptr->miid,
              DBG_ERROR_PRIO,
              "attempting to create int_buf node when avsync_enabled = %d and hold_buf_configured = %d",
              is_spr_avsync_enabled(me_ptr->avsync_ptr),
              spr_avsync_is_hold_buf_configured(me_ptr->avsync_ptr));
      return CAPI_EUNSUPPORTED;
   }

   avsync_t * avsync_ptr    = me_ptr->avsync_ptr;
   uint64_t   cur_buffer_us = spr_calc_capi_stream_size_in_us(input_ptr, media_fmt_ptr);
   capi_err_t result        = CAPI_EOK;
   bool_t     POOL_USED     = TRUE;

   if ((avsync_ptr->data_held_us + cur_buffer_us) > avsync_ptr->client_config.hold_buf_duration_us)
   {
      SPR_MSG_ISLAND(me_ptr->miid,
              DBG_HIGH_PRIO,
              "required duration %ld is more than configured hold buffer "
              "size %ld. Dropping data",
              avsync_ptr->data_held_us + cur_buffer_us,
              avsync_ptr->client_config.hold_buf_duration_us);

      result |= capi_spr_overwrite_int_buf_data(me_ptr, int_buf_ptr, input_ptr, media_fmt_ptr);
   }

   capi_stream_data_v2_t *int_buf_obj_ptr        = NULL;
   spf_list_node_t **     int_buf_list_head_pptr = &int_buf_ptr->buf_list_ptr;

   // Allocate capi stream node
   if (NULL == (int_buf_obj_ptr = (capi_stream_data_v2_t *)posal_memory_malloc(sizeof(capi_stream_data_v2_t), heap_id)))
   {
      SPR_MSG_ISLAND(me_ptr->miid, DBG_ERROR_PRIO, "Failed to allocate stream data struct for int buf node");
      return CAPI_ENOMEMORY;
   }

   int_buf_obj_ptr->metadata_list_ptr = NULL;
   int_buf_obj_ptr->buf_ptr           = NULL;
   int_buf_obj_ptr->bufs_num          = input_ptr->bufs_num;
   int_buf_obj_ptr->flags             = input_ptr->flags;
   int_buf_obj_ptr->timestamp         = input_ptr->timestamp;

   // Merge metadata lists from input stream
   spf_list_merge_lists((spf_list_node_t **)&int_buf_obj_ptr->metadata_list_ptr,
                        (spf_list_node_t **)&input_ptr->metadata_list_ptr);

   int_buf_obj_ptr->buf_ptr = (capi_buf_t *)posal_memory_malloc(input_ptr->bufs_num * sizeof(capi_buf_t), heap_id);
   if (!int_buf_obj_ptr->buf_ptr)
   {
      SPR_MSG_ISLAND(me_ptr->miid, DBG_ERROR_PRIO, "Failed to allocate capi buf data struct for hold node");
      posal_memory_free(int_buf_obj_ptr);
      int_buf_obj_ptr = NULL;
      return CAPI_ENOMEMORY;
   }

#ifdef SPR_INT_BUF_DEBUG
   SPR_MSG_ISLAND(me_ptr->miid, DBG_LOW_PRIO, "alloc int_buf_obj_ptr 0x%X", int_buf_obj_ptr);
#endif

   for (uint32_t num_chan = 0; num_chan < int_buf_obj_ptr->bufs_num; num_chan++)
   {
      capi_buf_t *capi_in_ch_buf_ptr = &(input_ptr->buf_ptr[num_chan]);
      capi_buf_t *int_in_ch_buf_ptr  = &(int_buf_obj_ptr->buf_ptr[num_chan]);
#ifdef SPR_INT_BUF_DEBUG
      SPR_MSG_ISLAND(me_ptr->miid,
              DBG_LOW_PRIO,
              "Input capi v2 buf ptr 0x%X, int_capi_buf 0x%X, "
              "data_len = %d",
              capi_in_ch_buf_ptr,
              int_in_ch_buf_ptr,
              capi_in_ch_buf_ptr->actual_data_len);
#endif
      memscpy((void *)int_in_ch_buf_ptr, sizeof(capi_buf_t), (void *)capi_in_ch_buf_ptr, sizeof(capi_buf_t));

      if (!int_in_ch_buf_ptr->actual_data_len)
      {
         int_in_ch_buf_ptr->data_ptr        = NULL;
         int_in_ch_buf_ptr->actual_data_len = 0;
         SPR_MSG_ISLAND(me_ptr->miid,
                 DBG_ERROR_PRIO,
                 "warning: received buffer with data len %d TS %ld",
                 capi_in_ch_buf_ptr->actual_data_len,
                 input_ptr->timestamp);
      }
      else
      {
         int_in_ch_buf_ptr->data_ptr =
            (int8_t *)posal_memory_malloc(int_in_ch_buf_ptr->actual_data_len * sizeof(int8_t), heap_id);

         if (!int_in_ch_buf_ptr->data_ptr)
         {
            SPR_MSG_ISLAND(me_ptr->miid, DBG_ERROR_PRIO, "Failed to allocate capi data buffer");

            int_in_ch_buf_ptr = &(int_buf_obj_ptr->buf_ptr[num_chan]);
            bool_t DROP_MD    = TRUE;
            spr_handle_metadata(me_ptr, (capi_stream_data_t **)&int_buf_obj_ptr, NULL, DROP_MD);

            // Destroy the buffers allocated so far
            while (num_chan--)
            {
               posal_memory_free(int_in_ch_buf_ptr->data_ptr);
               int_in_ch_buf_ptr = &(int_buf_obj_ptr->buf_ptr[num_chan]);
            }
            posal_memory_free(int_buf_obj_ptr->buf_ptr);
            posal_memory_free(int_buf_obj_ptr);
            int_buf_obj_ptr = NULL;

            return CAPI_ENOMEMORY;
         }

         memscpy((void *)int_in_ch_buf_ptr->data_ptr,
                 int_in_ch_buf_ptr->actual_data_len,
                 (void *)capi_in_ch_buf_ptr->data_ptr,
                 capi_in_ch_buf_ptr->actual_data_len);
      }
   } // for loop over num channels

   // Add the allocated node to the list
   if (AR_EOK != (result = spf_list_insert_tail(int_buf_list_head_pptr, (void *)int_buf_obj_ptr, heap_id, POOL_USED)))
   {
      uint32_t num_chan = 0;
      while (num_chan < input_ptr->bufs_num)
      {
         posal_memory_free(int_buf_obj_ptr->buf_ptr[num_chan].data_ptr);
         num_chan++;
      }

      posal_memory_free(int_buf_obj_ptr->buf_ptr);
      posal_memory_free(int_buf_obj_ptr);

      return CAPI_EFAILED;
   }

   avsync_ptr->data_held_us += cur_buffer_us;
   int_buf_ptr->cur_fill_size_us += cur_buffer_us;

#ifdef SPR_INT_BUF_DEBUG
   if (int_buf_ptr->is_used_for_hold)
   {
      SPR_MSG_ISLAND(me_ptr->miid,
              DBG_MED_PRIO,
              "Added TS %ld to the hold list. int_buf_duration = %ld overall_data_held = %ld",
              int_buf_obj_ptr->timestamp,
              int_buf_ptr->cur_fill_size_us,
              avsync_ptr->data_held_us);
   }
   else
   {
      SPR_MSG_ISLAND(me_ptr->miid,
              DBG_MED_PRIO,
              "Added TS %ld to the int list. curr_duration = %ld overall data held = %ld",
              int_buf_obj_ptr->timestamp,
              int_buf_ptr->cur_fill_size_us,
              avsync_ptr->data_held_us);
   }

#endif

   return result;
}
