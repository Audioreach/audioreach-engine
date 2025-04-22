/**
 * \file cu_buf_util.c
 * \brief
 *     This file contains utility functions for common buffer handling
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "cu_i.h"

/**
 * Utility function to initialize a queue, add it to the container's channel, and
 * set its cb handler (gets called when data is pushed to the queue). The queue will
 * be initialized at the locateion dest_ptr, so there is no allocation.
 */
ar_result_t cu_init_queue(cu_base_t *        base_ptr,
                            char *             data_q_name,
                            uint32_t           num_elements,
                            uint32_t           bit_mask,
                            cu_queue_handler_t q_func_ptr,
                            posal_channel_t    channel_ptr,
                            posal_queue_t **   q_pptr,
                            void *             dest_ptr,
                            POSAL_HEAP_ID      heap_id)
{
   INIT_EXCEPTION_HANDLING
   ar_result_t result = AR_EOK;
   uint32_t    min_elements =
      (num_elements <= CU_MIN_QUEUE_ELEMENTS_TO_PRE_ALLOC) ? num_elements : CU_MIN_QUEUE_ELEMENTS_TO_PRE_ALLOC;

   VERIFY(result, q_pptr);

   // The queue should not be created multiple times, so return fail if the
   // queue is already present.
   if (*q_pptr)
   {
      CU_MSG(base_ptr->gu_ptr->log_id, DBG_HIGH_PRIO, "data q already present");
      return AR_EFAILED;
   }

   posal_queue_init_attr_t q_attr;
   posal_queue_attr_init(&q_attr);
   posal_queue_attr_set_heap_id(&q_attr, heap_id);
   posal_queue_attr_set_max_nodes(&q_attr, num_elements);
   posal_queue_attr_set_prealloc_nodes(&q_attr, min_elements);
   posal_queue_attr_set_name(&q_attr, data_q_name);

   if (NULL == dest_ptr)
   {
      CU_MSG(base_ptr->gu_ptr->log_id, DBG_ERROR_PRIO, "Invalid destination pointer for queue");
      return AR_EFAILED;
   }

   if (AR_DID_FAIL(result = posal_queue_init((posal_queue_t *)dest_ptr, &q_attr)))
   {
      CU_MSG(base_ptr->gu_ptr->log_id, DBG_ERROR_PRIO, "Failed to init message queues, result: %lu", result);
      return result;
   }
   *q_pptr = dest_ptr; // if initialization is a success go ahead and set the q ptr to the dest


   if (q_func_ptr)
   {
      cu_set_handler_for_bit_mask(base_ptr, bit_mask, q_func_ptr);
   }

   /* Add queue to the channel only if valid bitmask and channel is passed in the arguments */
   if ((bit_mask > 0) && (NULL != channel_ptr))
   {
      if (AR_DID_FAIL(result = posal_channel_addq(channel_ptr, *q_pptr, bit_mask)))
      {
         CU_MSG(base_ptr->gu_ptr->log_id, DBG_ERROR_PRIO, "AR_DID_FAIL to add mqs to channel, result = %d", result);
         return result;
      }
   }
   else
   {
      CU_MSG(base_ptr->gu_ptr->log_id,
             DBG_ERROR_PRIO,
             "Queue not added to the channel, bit_mask: 0x%lX, ch_ptr: 0x%lX ",
             bit_mask,
             channel_ptr);
   }

   CATCH(result, CU_MSG_PREFIX, base_ptr->gu_ptr->log_id)
   {
   }
   return result;
}

ar_result_t cu_handle_prebuffer(cu_base_t *            base_ptr,
                                gu_ext_out_port_t *    ext_out_port_gu_ptr,
                                spf_msg_data_buffer_t *cur_out_buf_ptr,
                                uint32_t               nominal_frame_size)
{
   ar_result_t result = AR_EOK;

   int64_t  timestamp    = 0;
   uint32_t otp_buf_size = 0;

   cu_ext_out_port_t *ext_out_port_ptr =
      (cu_ext_out_port_t *)((uint8_t *)ext_out_port_gu_ptr + base_ptr->ext_out_port_cu_offset);

   bool_t is_ts_valid = cu_get_bits(cur_out_buf_ptr->flags,
                                    DATA_BUFFER_FLAG_TIMESTAMP_VALID_MASK,
                                    DATA_BUFFER_FLAG_TIMESTAMP_VALID_SHIFT);
   INIT_EXCEPTION_HANDLING

   if (!ext_out_port_gu_ptr->downstream_handle.spf_handle_ptr)
   {
      CU_MSG(base_ptr->gu_ptr->log_id, DBG_ERROR_PRIO, "Downstream not connected. Prebuffer not sent");
      return AR_EOK;
   }

   otp_buf_size =
      icb_get_frame_len_samples(&ext_out_port_ptr->icb_info.icb.otp, ext_out_port_ptr->media_fmt.pcm.sample_rate) *
      (ext_out_port_ptr->media_fmt.pcm.bits_per_sample >> 3) * ext_out_port_ptr->media_fmt.pcm.num_channels;

   if (is_ts_valid)
   {
      uint32_t total_size = 0;
      total_size          = otp_buf_size + (ext_out_port_ptr->icb_info.icb.num_reg_prebufs * nominal_frame_size);

      timestamp =
         cur_out_buf_ptr->timestamp - (int64_t)topo_bytes_to_us(total_size, &ext_out_port_ptr->media_fmt, NULL);
   }

   uint32_t msg_opcode = SPF_MSG_DATA_BUFFER;
   uint32_t total_size = GET_SPF_INLINE_DATABUF_REQ_SIZE(otp_buf_size);

   if ((ext_out_port_ptr->media_fmt.data_format == SPF_DEINTERLEAVED_RAW_COMPRESSED) &&
       (0 != ext_out_port_ptr->media_fmt.deint_raw.bufs_num))
   {
      msg_opcode = SPF_MSG_DATA_BUFFER_V2;
      total_size = GET_SPF_DATABUF_V2_REQ_SIZE(ext_out_port_ptr->media_fmt.deint_raw.bufs_num, otp_buf_size);
   }

   spf_msg_t msg;

   if (0 != otp_buf_size)
   {
      if (AR_DID_FAIL(spf_msg_create_msg(&msg,
                                         &total_size,
                                         msg_opcode,
                                         NULL,
                                         NULL,
                                         ext_out_port_gu_ptr->downstream_handle.spf_handle_ptr,
                                         base_ptr->heap_id)))
      {
         CU_MSG(base_ptr->gu_ptr->log_id, DBG_ERROR_PRIO, "Fail to create OTP one time prebuffer");
         THROW(result, AR_EFAILED);
      }

      spf_msg_header_t *header_ptr = header_ptr = (spf_msg_header_t *)(msg.payload_ptr);

      module_cmn_md_list_t **out_md_list_pptr  = NULL;
      uint32_t *             flags_ptr         = NULL;
      int64_t *              out_timestamp_ptr = NULL;

      if (msg_opcode == SPF_MSG_DATA_BUFFER_V2)
      {
         spf_msg_data_buffer_v2_t *out_buf_v2_ptr = (spf_msg_data_buffer_v2_t *)&header_ptr->payload_start;
         spf_msg_single_buf_v2_t * data_buf_info  = (spf_msg_single_buf_v2_t *)(out_buf_v2_ptr + 1);

         for (uint32_t b = 0; b < out_buf_v2_ptr->bufs_num; b++)
         {
            data_buf_info[b].actual_size = otp_buf_size;
            data_buf_info[b].max_size    = otp_buf_size;
         }

         out_md_list_pptr  = &out_buf_v2_ptr->metadata_list_ptr;
         flags_ptr         = &out_buf_v2_ptr->flags;
         out_timestamp_ptr = &out_buf_v2_ptr->timestamp;
      }
      else
      {
         spf_msg_data_buffer_t *out_buf_ptr = (spf_msg_data_buffer_t *)&header_ptr->payload_start;
         out_buf_ptr->max_size              = otp_buf_size;
         out_buf_ptr->actual_size           = out_buf_ptr->max_size;

         memset(&out_buf_ptr->data_buf[0], 0, out_buf_ptr->max_size);

         out_md_list_pptr  = &out_buf_ptr->metadata_list_ptr;
         flags_ptr         = &out_buf_ptr->flags;
         out_timestamp_ptr = &out_buf_ptr->timestamp;
      }

      *out_md_list_pptr = NULL;
      *flags_ptr        = 0;

      // mark the prebuffer flag
      cu_set_bits(flags_ptr, TRUE, DATA_BUFFER_FLAG_PREBUFFER_MARK_MASK, DATA_BUFFER_FLAG_PREBUFFER_MARK_SHIFT);

      if (is_ts_valid)
      {
         *out_timestamp_ptr = timestamp;
         cu_set_bits(flags_ptr,
                     is_ts_valid,
                     DATA_BUFFER_FLAG_TIMESTAMP_VALID_MASK,
                     DATA_BUFFER_FLAG_TIMESTAMP_VALID_SHIFT);

         timestamp += (int64_t)topo_bytes_to_us(otp_buf_size, &ext_out_port_ptr->media_fmt, NULL);
      }

      result = posal_queue_push_back(ext_out_port_gu_ptr->downstream_handle.spf_handle_ptr->q_ptr,
                                     (posal_queue_element_t *)&msg);

      if (AR_DID_FAIL(result))
      {
         CU_MSG(base_ptr->gu_ptr->log_id, DBG_ERROR_PRIO, "Failed to deliver OTP dowstream. Freeing");
         spf_msg_return_msg(&msg);
         THROW(result, result);
      }

      CU_MSG(base_ptr->gu_ptr->log_id,
             DBG_HIGH_PRIO,
             "pushed (one time prebuffer) OTP downstream 0x%p. buf-len, total expected = %lu, given = %lu, "
             "otp_buf_size = %lu,Current bit mask  0x%x",
             msg.payload_ptr,
             total_size,
             header_ptr->payload_size,
             otp_buf_size,
             base_ptr->curr_chan_mask);
   }
#if 0
   else
   {
      CU_MSG(base_ptr->gu_ptr->log_id, DBG_HIGH_PRIO, "Not pushing (one time prebuffer) OTP, buf size 0");
   }
#endif

   // bufQ poll and send the regular prebuffers
   for (uint32_t i = 0; i < ext_out_port_ptr->icb_info.icb.num_reg_prebufs; i++)
   {
      posal_bufmgr_node_t buf_node     = { 0 };
      bool_t              poll_success = FALSE;

      while (posal_channel_poll_inline(base_ptr->channel_ptr, ext_out_port_ptr->bit_mask))
      {
         // Take next buffer off the q
         if (AR_DID_FAIL(result = posal_queue_pop_front(ext_out_port_gu_ptr->this_handle.q_ptr,
                                                        (posal_queue_element_t *)&(buf_node))))
         {
            CU_MSG(base_ptr->gu_ptr->log_id, DBG_ERROR_PRIO, "Failure on Out prebuffer Pop ");
            THROW(result, result);
         }
         if (NULL != buf_node.buf_ptr)
         {
#ifdef BUF_CNT_DEBUGGING
            ext_out_port_ptr->buf_recv_cnt++;
#endif
            poll_success = TRUE;
         }
         break;
      }

      if (poll_success)
      {
         spf_msg_t *data_msg_ptr;

         module_cmn_md_list_t **out_md_list_pptr  = NULL;
         uint32_t *             out_flags_ptr     = NULL;
         int64_t *              out_timestamp_ptr = NULL;
         spf_msg_header_t *     header_ptr        = NULL;
         if (msg_opcode == SPF_MSG_DATA_BUFFER_V2)
         {
            header_ptr = (spf_msg_header_t *)(buf_node.buf_ptr);

            spf_msg_data_buffer_v2_t *out_buf_v2_ptr = (spf_msg_data_buffer_v2_t *)&header_ptr->payload_start;
            spf_msg_single_buf_v2_t * data_buf_info  = (spf_msg_single_buf_v2_t *)(out_buf_v2_ptr + 1);

            for (uint32_t b = 0; b < out_buf_v2_ptr->bufs_num; b++)
            {
               if (data_buf_info[b].max_size < nominal_frame_size)
               {
                  CU_MSG(base_ptr->gu_ptr->log_id,
                         DBG_ERROR_PRIO,
                         "Module 0x%lX, 0x%lX, Prebuffer max size (%lu) is smaller than frame size (%lu). ERROR!",
                         ext_out_port_gu_ptr->int_out_port_ptr->cmn.module_ptr->module_instance_id,
                         ext_out_port_gu_ptr->int_out_port_ptr->cmn.id,
                         data_buf_info[b].max_size,
                         nominal_frame_size);
                  nominal_frame_size = data_buf_info[b].max_size;
               }
            }

            for (uint32_t b = 0; b < out_buf_v2_ptr->bufs_num; b++)
            {
               memset(data_buf_info[b].data_ptr, 0, nominal_frame_size);
               data_buf_info[b].actual_size = nominal_frame_size;
            }

            out_md_list_pptr  = &out_buf_v2_ptr->metadata_list_ptr;
            out_flags_ptr     = &out_buf_v2_ptr->flags;
            out_timestamp_ptr = &out_buf_v2_ptr->timestamp;
         }
         else
         {
            header_ptr                         = (spf_msg_header_t *)(buf_node.buf_ptr);
            spf_msg_data_buffer_t *out_buf_ptr = (spf_msg_data_buffer_t *)&header_ptr->payload_start;

            if (out_buf_ptr->max_size < nominal_frame_size)
            {
               CU_MSG(base_ptr->gu_ptr->log_id,
                      DBG_ERROR_PRIO,
                      "Module 0x%lX, 0x%lX, Prebuffer max size (%lu) is smaller than frame size (%lu). ERROR!",
                      ext_out_port_gu_ptr->int_out_port_ptr->cmn.module_ptr->module_instance_id,
                      ext_out_port_gu_ptr->int_out_port_ptr->cmn.id,
                      out_buf_ptr->max_size,
                      nominal_frame_size);
               nominal_frame_size = out_buf_ptr->max_size;
            }

            memset(&out_buf_ptr->data_buf[0], 0, nominal_frame_size);

            out_buf_ptr->actual_size = nominal_frame_size;

            out_md_list_pptr  = &out_buf_ptr->metadata_list_ptr;
            out_flags_ptr     = &out_buf_ptr->flags;
            out_timestamp_ptr = &out_buf_ptr->timestamp;
         }

         *out_md_list_pptr = NULL;
         *out_flags_ptr    = 0;

         data_msg_ptr =
            spf_msg_convt_buf_node_to_msg(&buf_node, msg_opcode, NULL, NULL, 0, &ext_out_port_gu_ptr->this_handle);

         // mark the prebuffer flag
         cu_set_bits(out_flags_ptr, TRUE, DATA_BUFFER_FLAG_PREBUFFER_MARK_MASK, DATA_BUFFER_FLAG_PREBUFFER_MARK_SHIFT);

         if (is_ts_valid)
         {
            cu_set_bits(out_flags_ptr,
                        is_ts_valid,
                        DATA_BUFFER_FLAG_TIMESTAMP_VALID_MASK,
                        DATA_BUFFER_FLAG_TIMESTAMP_VALID_SHIFT);
            *out_timestamp_ptr = timestamp;
            timestamp += (int64_t)topo_bytes_to_us(nominal_frame_size, &ext_out_port_ptr->media_fmt, NULL);
         }

         result = posal_queue_push_back(ext_out_port_gu_ptr->downstream_handle.spf_handle_ptr->q_ptr,
                                        (posal_queue_element_t *)data_msg_ptr);
         if (AR_DID_FAIL(result))
         {
            CU_MSG(base_ptr->gu_ptr->log_id, DBG_ERROR_PRIO, "Failed to deliver prebuffer dowstream. Dropping");
            spf_msg_return_msg(data_msg_ptr);
         }

         CU_MSG(base_ptr->gu_ptr->log_id,
                DBG_HIGH_PRIO,
                "Module 0x%lX, 0x%lX, pushed prebuffer downstream 0x%p of size %lu bytes. Current bit mask 0x%x",
                ext_out_port_gu_ptr->int_out_port_ptr->cmn.module_ptr->module_instance_id,
                ext_out_port_gu_ptr->int_out_port_ptr->cmn.id,
                buf_node.buf_ptr,
                nominal_frame_size,
                base_ptr->curr_chan_mask);
      }
      else
      {
         CU_MSG(base_ptr->gu_ptr->log_id,
                DBG_ERROR_PRIO,
                "Channel poll on prebuffer failed, Q_ptr =  0x%p. Current bit mask 0x%x",
                ext_out_port_gu_ptr->this_handle.q_ptr,
                base_ptr->curr_chan_mask);
      }
   }

   CATCH(result, CU_MSG_PREFIX, base_ptr->gu_ptr->log_id)
   {
   }
   ext_out_port_ptr->icb_info.is_prebuffer_sent = TRUE;
   return result;
}
