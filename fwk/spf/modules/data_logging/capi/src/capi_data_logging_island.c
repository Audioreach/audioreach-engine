/**
 *   \file capi_data_logging_island.c
 *   \brief
 *        This file contains CAPI implementation of Data logging module
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#include "capi_data_logging_i.h"
#include "posal.h"

capi_vtbl_t capi_data_log_vtbl = { capi_data_logging_process,        capi_data_logging_end,
                                   capi_data_logging_set_param,      capi_data_logging_get_param,
                                   capi_data_logging_set_properties, capi_data_logging_get_properties };


capi_err_t capi_data_logging_process(capi_t *capi_ptr, capi_stream_data_t *input[], capi_stream_data_t *output[])
{

   /* Data logging module is allowed to be placed in container with LPI heap.
    *    Data logging is skipped if the island mode is active.
    *    Data logging module will prevent island entry if host container is in island and island logging is enabled.
    *    Audio playback cntr maybe LPI, but we never enter island. so in such cases also logging can be done as island
    *     status will be non-island.
    *
    * Known-issue in incr_log_id: If EOS occurs while we are in island and we don't call capi_data_logging_process_nlpi,
    * then log-id is not incremented.
    */
   capi_err_t           result     = CAPI_EOK;
   capi_data_logging_t *me_ptr     = (capi_data_logging_t *)((capi_ptr));
   bool_t               do_logging = !posal_island_get_island_status();

   if (me_ptr->is_process_state &&
       do_logging) // if this is true, we are guaranteed to be not in LPI mode due to above checks
   {
#ifdef SIM
      bool_t is_in_island = posal_island_get_island_status();
      if (is_in_island && me_ptr->forced_logging) // If island logging is allowed exit island else return
      {
         capi_cmn_raise_island_vote_event(&me_ptr->event_cb_info, CAPI_CMN_ISLAND_VOTE_EXIT);
      }
#endif

      /* For voiceUI use case, if forced_logging is False, and is_in_island is False at this time, we will not enter
       * island until this thread goes to island. and the call to capi_data_logging_process_nlpi will not cause crash.
       * */
      capi_data_logging_process_nlpi(me_ptr, input, output);
   }

   // if output buffer is provided.
   // even if logging is not done, input data needs to be copied to output & output actual len be set.
   if (output && (NULL != output[0]) && (NULL != output[0]->buf_ptr) && (0 != output[0]->buf_ptr[0].max_data_len))
   {
      bool_t is_inplace = (input[0]->buf_ptr[0].data_ptr == output[0]->buf_ptr[0].data_ptr);

      if (!is_inplace && (output[0]->buf_ptr[0].max_data_len < input[0]->buf_ptr[0].actual_data_len) &&
          (me_ptr->media_format.format.num_channels > 1) &&
          (CAPI_DEINTERLEAVED_PACKED == me_ptr->media_format.format.data_interleaving))
      {
         uint32_t out_max_data_len_per_ch =
            output[0]->buf_ptr[0].max_data_len / me_ptr->media_format.format.num_channels;
         uint32_t in_actual_data_len_per_ch =
            input[0]->buf_ptr[0].actual_data_len / me_ptr->media_format.format.num_channels;

         uint32_t bytes_to_copy_per_ch = MIN(out_max_data_len_per_ch, in_actual_data_len_per_ch);

         for (uint32_t i = 0; i < me_ptr->media_format.format.num_channels; i++)
         {
            memscpy(output[0]->buf_ptr[0].data_ptr + (i * bytes_to_copy_per_ch),
                    bytes_to_copy_per_ch,
                    input[0]->buf_ptr[0].data_ptr + (i * in_actual_data_len_per_ch),
                    bytes_to_copy_per_ch);
         }

         output[0]->buf_ptr[0].actual_data_len = bytes_to_copy_per_ch * me_ptr->media_format.format.num_channels;
         input[0]->buf_ptr[0].actual_data_len  = bytes_to_copy_per_ch * me_ptr->media_format.format.num_channels;

#ifdef DATA_LOGGING_DBG
         AR_MSG_ISLAND(DBG_HIGH_PRIO,
                       "CAPI Data Logging: out buf actual data length: %d ",
                       output[0]->buf_ptr[0].actual_data_len);
#endif
      }
      else if(CAPI_DEINTERLEAVED_UNPACKED_V2 == me_ptr->media_format.format.data_interleaving)
      {
         uint32_t max_data_len_per_buf    = output[0]->buf_ptr[0].max_data_len;
         uint32_t actual_data_len_per_buf = input[0]->buf_ptr[0].actual_data_len;
         uint32_t copy_length             = MIN(actual_data_len_per_buf, max_data_len_per_buf);
         for (uint32_t i = 0; i < input[0]->bufs_num; i++)
         {
            if (!is_inplace)
            {
#ifdef DATA_LOGGING_DBG
               AR_MSG_ISLAND(DBG_HIGH_PRIO,
                             "CAPI Data Logging: input and output data buffers data ptrs are "
                             "not same!");
#endif
               memscpy(output[0]->buf_ptr[i].data_ptr,
                       copy_length,
                       input[0]->buf_ptr[i].data_ptr,
                       copy_length);
            }
#ifdef DATA_LOGGING_DBG
            AR_MSG_ISLAND(DBG_HIGH_PRIO,
                          "CAPI Data Logging: out buf actual data length: %d ",
                          copy_length);
#endif
         }
         output[0]->buf_ptr[0].actual_data_len = copy_length;
         input[0]->buf_ptr[0].actual_data_len  = copy_length;
      }
      else
      {
         for (uint32_t i = 0; i < input[0]->bufs_num; i++)
         {
            if (!is_inplace)
            {
#ifdef DATA_LOGGING_DBG
               AR_MSG_ISLAND(DBG_HIGH_PRIO,
                             "CAPI Data Logging: input and output data buffers data ptrs are "
                             "not same!");
#endif
               memscpy(output[0]->buf_ptr[i].data_ptr,
                       output[0]->buf_ptr[i].max_data_len,
                       input[0]->buf_ptr[i].data_ptr,
                       input[0]->buf_ptr[i].actual_data_len);
            }

            uint32_t length = MIN(input[0]->buf_ptr[i].actual_data_len, output[0]->buf_ptr[i].max_data_len);
            output[0]->buf_ptr[i].actual_data_len = length;
            input[0]->buf_ptr[i].actual_data_len  = length;

#ifdef DATA_LOGGING_DBG
            AR_MSG_ISLAND(DBG_HIGH_PRIO,
                          "CAPI Data Logging: out buf actual data length: %d ",
                          output[0]->buf_ptr[i].actual_data_len);
#endif
         }
      }
   }

   return result;
}
