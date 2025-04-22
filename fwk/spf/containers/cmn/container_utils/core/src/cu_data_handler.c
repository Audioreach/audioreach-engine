/**
 * \file cu_data_handler.c
 * \brief
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "cu_i.h"

/* =======================================================================
Public Function Definitions
========================================================================== */

/**
 * Pop an input data message from the external input port data queue, and
 * store it in the external input port.
 */
ar_result_t cu_get_input_data_msg(cu_base_t *me_ptr, gu_ext_in_port_t *gu_ext_in_port_ptr)
{
   ar_result_t result;

   cu_ext_in_port_t *ext_in_port_ptr =
      (cu_ext_in_port_t *)((uint8_t *)gu_ext_in_port_ptr + me_ptr->ext_in_port_cu_offset);

   if (ext_in_port_ptr->input_data_q_msg.payload_ptr)
   {
      CU_MSG(me_ptr->gu_ptr->log_id,
             DBG_ERROR_PRIO,
             "Already holding on to an input data command. "
             "Cannot get another command until this is freed");
      return AR_EUNEXPECTED;
   }

   result = posal_queue_pop_front(gu_ext_in_port_ptr->this_handle.q_ptr,
                                  (posal_queue_element_t *)&(ext_in_port_ptr->input_data_q_msg));
   if (AR_EOK != result)
   {
      ext_in_port_ptr->input_data_q_msg.payload_ptr = NULL;
      return result;
   }

#ifdef BUF_CNT_DEBUGGING
   if (SPF_MSG_DATA_BUFFER == ext_in_port_ptr->input_data_q_msg.msg_opcode)
   {
      ext_in_port_ptr->buf_recv_cnt++;
   }
#endif

   return result;
}

/**
 * Return an output buffer to its return queue.
 */
void cu_return_out_buf(cu_base_t *me_ptr, gu_ext_out_port_t *gu_ext_port_ptr)
{
   cu_ext_out_port_t *ext_port_ptr = (cu_ext_out_port_t *)((uint8_t *)gu_ext_port_ptr + me_ptr->ext_out_port_cu_offset);

   spf_msg_header_t *header = (spf_msg_header_t *)(ext_port_ptr->out_bufmgr_node.buf_ptr);
   if (NULL != header)
   {
      if (ext_port_ptr->out_bufmgr_node.return_q_ptr)
      {
#ifdef BUF_CNT_DEBUGGING
         if (ext_port_ptr->buf_recv_cnt > 1)
         {
            ext_port_ptr->buf_recv_cnt--;
         }
#endif
         spf_msg_t msg = {.payload_ptr = ext_port_ptr->out_bufmgr_node.buf_ptr, .msg_opcode = 0 };
         spf_msg_return_msg(&msg);
      }

      // The output buffer was returned, stop holding it.
      ext_port_ptr->out_bufmgr_node.buf_ptr = NULL;
   }
}

/**
 * Tasks to be done after handling any data path command.
 * - DATA_BUFFER opcode
 *    - Let actual_size to zero since all data has been consumed.
 *    - Increment buf_done_cnt.
 * - Ack the data message.
 * - Release the data message (set held message payload to NULL).
 */
ar_result_t cu_free_input_data_cmd(cu_base_t *me_ptr, gu_ext_in_port_t *gu_ext_in_port_ptr, ar_result_t status)
{
   INIT_EXCEPTION_HANDLING
   ar_result_t result = AR_EOK;

   VERIFY(result, me_ptr->cntr_vtbl_ptr->destroy_all_metadata);

   cu_ext_in_port_t *ext_in_port_ptr =
      (cu_ext_in_port_t *)((uint8_t *)gu_ext_in_port_ptr + me_ptr->ext_in_port_cu_offset);

   // If there is no held input data msg, there is nothing to free.
   if (!ext_in_port_ptr->input_data_q_msg.payload_ptr)
   {
      return result;
   }

   module_cmn_md_list_t *metadata_list_ptr = NULL;

   switch (ext_in_port_ptr->input_data_q_msg.msg_opcode)
   {
      case SPF_MSG_DATA_BUFFER:
      {

         spf_msg_header_t      *header_ptr = (spf_msg_header_t *)(ext_in_port_ptr->input_data_q_msg.payload_ptr);
         spf_msg_data_buffer_t *buf_ptr    = (spf_msg_data_buffer_t *)&header_ptr->payload_start;
         buf_ptr->actual_size              = 0;

         metadata_list_ptr          = buf_ptr->metadata_list_ptr;
         buf_ptr->metadata_list_ptr = NULL;
         break;
      }
      case SPF_MSG_DATA_BUFFER_V2:
      {
         spf_msg_header_t         *header_ptr = (spf_msg_header_t *)(ext_in_port_ptr->input_data_q_msg.payload_ptr);
         spf_msg_data_buffer_v2_t *buf_ptr    = (spf_msg_data_buffer_v2_t *)&header_ptr->payload_start;

         spf_msg_single_buf_v2_t *data_buf_info = (spf_msg_single_buf_v2_t *)(buf_ptr + 1);

         for (uint32_t b = 0; b < buf_ptr->bufs_num; b++)
         {
            data_buf_info[b].actual_size = 0;
         }

         metadata_list_ptr          = buf_ptr->metadata_list_ptr;
         buf_ptr->metadata_list_ptr = NULL;
         break;
      }
      default:
      {
         /* Not metadata list ptr not applicable */
         CU_MSG(me_ptr->gu_ptr->log_id,
                DBG_HIGH_PRIO,
                "Unsupported message opcode = 0x%lx.",
                ext_in_port_ptr->input_data_q_msg.msg_opcode);
      }
   }

#ifdef BUF_CNT_DEBUGGING
   ext_in_port_ptr->buf_done_cnt++;
#endif

   if (metadata_list_ptr)
   {
      bool_t IS_DROPPED_TRUE = TRUE;

      CU_MSG(me_ptr->gu_ptr->log_id,
             DBG_ERROR_PRIO,
             "Metadata found while freeing input data msg on external input port idx = %ld, miid = 0x%lx. "
             "Destroying.",
             gu_ext_in_port_ptr->int_in_port_ptr->cmn.index,
             gu_ext_in_port_ptr->int_in_port_ptr->cmn.module_ptr->module_instance_id);

      // Raising with the module at this external input port even though data didn't quite reach the module.
      // The only use of module here is for the raise_eos callback.
      TRY(result,
          me_ptr->cntr_vtbl_ptr->destroy_all_metadata(me_ptr->gu_ptr->log_id,
                                                      (void *)gu_ext_in_port_ptr->int_in_port_ptr->cmn.module_ptr,
                                                      &metadata_list_ptr,
                                                      IS_DROPPED_TRUE));
   }

   spf_msg_ack_msg(&ext_in_port_ptr->input_data_q_msg, status);

   // Set payload to NULL to indicate we are not holding on to any input data msg.
   ext_in_port_ptr->input_data_q_msg.payload_ptr = NULL;

   CATCH(result, CU_MSG_PREFIX, me_ptr->gu_ptr->log_id)
   {
   }

   return result;
}
