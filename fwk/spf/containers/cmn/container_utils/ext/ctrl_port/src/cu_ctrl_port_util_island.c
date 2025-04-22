/**
 * \file cu_ctrl_port_util_island.c
 * \brief
 *     This file contains container utility functions for control port handling in island (internal and external).
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "cu_i.h"
#include "cu_ctrl_port_i.h"
#include "capi_intf_extn_imcl.h"


ar_result_t cu_send_ctrl_polling_spf_msg(spf_msg_t *msg_ptr, spf_handle_t *dst_handle_ptr)
{
   ar_result_t    result     = AR_EOK;
   posal_queue_t *data_q_ptr = dst_handle_ptr->q_ptr;
   if (NULL == data_q_ptr)
   {
      AR_MSG_ISLAND(DBG_ERROR_PRIO,
                    "spf_msg_send_data() failed: dst handle's queue is NULL. Message "
                    "opcode: 0x%lx",
                    msg_ptr->msg_opcode);
      return AR_EFAILED;
   }

   result = posal_queue_push_back(data_q_ptr, (posal_queue_element_t *)msg_ptr);
   if (AR_DID_FAIL(result))
   {
      AR_MSG_ISLAND(DBG_ERROR_PRIO,
                    "spf_msg_send_data() failed: failed to push to the dst handle's "
                    "queue. Message opcode: 0x%lx",
                    msg_ptr->msg_opcode);
      return AR_EFAILED;
   }
   return AR_EOK;
}

static ar_result_t cu_prep_and_send_outgoing_ctrl_msg(cu_base_t *               me_ptr,
                                                      spf_msg_header_t *        header_ptr,
                                                      spf_handle_t *            dest_handle_ptr,
                                                      capi_buf_t *              buf_ptr,
                                                      imcl_outgoing_data_flag_t flags,
                                                      bool_t                    is_intra_cntr_msg,
                                                      uint32_t                  peer_port_id,
                                                      void *                    self_port_hdl,
                                                      void *                    peer_port_hdl,
                                                      bool_t                    is_recurring)
{
   ar_result_t result = AR_EOK;
   // Reinterpret the node itself as a spf_msg_t.
   spf_msg_t msg;
   msg.payload_ptr = header_ptr;

   // Need to be returned to the buffer queue.
   if (!flags.should_send)
   {
      CU_MSG_ISLAND(me_ptr->gu_ptr->log_id,
             DBG_HIGH_PRIO,
             " Buffer not ready, returning to the queue. is_recurring= %lu, result= %d",
             is_recurring,
             result);

      spf_msg_return_msg(&msg);
      return result;
   }

   if (flags.is_trigger)
   {
      msg.msg_opcode = SPF_MSG_CMD_CTRL_PORT_TRIGGER_MSG;
   }
   else
   {
      msg.msg_opcode = SPF_MSG_CTRL_PORT_POLLING_MSG;
   }

   // update the actual size of control message
   spf_msg_ctrl_port_msg_t *ctrl_buf_ptr = (spf_msg_ctrl_port_msg_t *)&header_ptr->payload_start[0];
   ctrl_buf_ptr->actual_size             = buf_ptr->actual_data_len;

   ctrl_buf_ptr->is_intra_cntr           = is_intra_cntr_msg;
   ctrl_buf_ptr->src_intra_cntr_port_hdl = self_port_hdl;
   ctrl_buf_ptr->dst_intra_cntr_port_hdl = peer_port_hdl;

   // populate the peer port ID
   intf_extn_param_id_imcl_incoming_data_t *payload_hdr_ptr =
      (intf_extn_param_id_imcl_incoming_data_t *)&ctrl_buf_ptr->data_buf[0];

   payload_hdr_ptr->port_id = peer_port_id;

   if (flags.is_trigger)
   {
      result = cu_send_ctrl_trigger_spf_msg(&msg, dest_handle_ptr);
   }
   else
   {
      result = cu_send_ctrl_polling_spf_msg(&msg, dest_handle_ptr);
   }

   if (AR_DID_FAIL(result))
   {
      CU_MSG_ISLAND(me_ptr->gu_ptr->log_id,
             DBG_ERROR_PRIO,
             " Failed to push the buffer to peer control port queue %d",
             result);
      spf_msg_return_msg(&msg);
   }
   return result;
}

static ar_result_t cu_ctrl_port_process_outgoing_buf(cu_base_t *               me_ptr,
                                                     gu_ctrl_port_t *          gu_ctrl_port_ptr,
                                                     capi_buf_t *              buf_ptr,
                                                     imcl_outgoing_data_flag_t flags)
{
   ar_result_t result = AR_EOK;

   cu_int_ctrl_port_t *ctrl_port_ptr =
      (cu_int_ctrl_port_t *)((uint8_t *)(gu_ctrl_port_ptr) + me_ptr->int_ctrl_port_cu_offset);

   if ((gu_ctrl_port_ptr->ext_ctrl_port_ptr &&
        NULL == gu_ctrl_port_ptr->ext_ctrl_port_ptr->peer_handle.spf_handle_ptr) ||
       (gu_ctrl_port_ptr->peer_ctrl_port_ptr && NULL == gu_ctrl_port_ptr->peer_ctrl_port_ptr->module_ptr) ||
       (NULL == gu_ctrl_port_ptr->ext_ctrl_port_ptr && NULL == gu_ctrl_port_ptr->peer_ctrl_port_ptr))
   {
      CU_MSG_ISLAND(me_ptr->gu_ptr->log_id,
                    DBG_ERROR_PRIO,
                    "No Peer Ctrl port for ctrl port ID 0x%lx",
                    gu_ctrl_port_ptr->id);

      // mark to return
      flags.should_send = FALSE;
      result            = AR_EFAILED;
   }

   uint32_t peer_port_id = 0;
   if (gu_ctrl_port_ptr->ext_ctrl_port_ptr)
   {
      peer_port_id = gu_ctrl_port_ptr->ext_ctrl_port_ptr->peer_handle.port_id;
   }
   else if (gu_ctrl_port_ptr->peer_ctrl_port_ptr)
   {
      peer_port_id = gu_ctrl_port_ptr->peer_ctrl_port_ptr->id;
   }

   // add the intf extn header to the actual size to get the actual size of outgoing intent param size.
   buf_ptr->actual_data_len += sizeof(intf_extn_param_id_imcl_incoming_data_t);

   // Get spf_msg_header_t* from the intent buf pointer.
   spf_msg_header_t *header_ptr = (spf_msg_header_t *)((int8_t *)buf_ptr->data_ptr - INTENT_BUFFER_OFFSET);

   bool_t is_recurring = FALSE;
   bool_t is_intra_container = (gu_ctrl_port_ptr->ext_ctrl_port_ptr)? FALSE: TRUE;

   if (header_ptr->return_q_ptr == ctrl_port_ptr->buffer_q_ptr)
   {
      // If the return Q of the buffer is same as the control ports buffer q then its recurring,
      // else its one time buffer
      // return q is populated when module request for buffer.
      is_recurring = TRUE;
   }

   result |=
      cu_prep_and_send_outgoing_ctrl_msg(me_ptr,
                                         header_ptr,
                                         cu_ctrl_port_get_dst_handle(me_ptr, gu_ctrl_port_ptr),
                                         buf_ptr,
                                         flags,
                                         is_intra_container,
                                         peer_port_id,
                                         is_intra_container ? (void *)gu_ctrl_port_ptr : NULL,
                                         is_intra_container ? (void *)(gu_ctrl_port_ptr->peer_ctrl_port_ptr) : NULL,
                                         is_recurring);

#if defined(VERBOSE_LOGGING) || defined(CTRL_LINK_DEBUG)
   CU_MSG_ISLAND(me_ptr->gu_ptr->log_id,
                 DBG_MED_PRIO,
                 " Module 0x%lX, 0x%lX: Sent ctrl msg to Module 0x%lX, 0x%lX, is_recurring= %lu, max_size= %lu, "
                 "actual_size= %lu, result 0x%lx",
                 gu_ctrl_port_ptr->module_ptr->module_instance_id,
                 gu_ctrl_port_ptr->id,
                 is_intra_container ? gu_ctrl_port_ptr->peer_ctrl_port_ptr->module_ptr->module_instance_id
                                    : gu_ctrl_port_ptr->ext_ctrl_port_ptr->peer_handle.module_instance_id,
                 is_intra_container ? gu_ctrl_port_ptr->peer_ctrl_port_ptr->id
                                    : gu_ctrl_port_ptr->ext_ctrl_port_ptr->peer_handle.port_id,
                 is_recurring,
                 buf_ptr->max_data_len,
                 buf_ptr->actual_data_len,
                 result);
#endif

   return result;
}

static ar_result_t cu_alloc_populate_one_time_ctrl_msg(cu_base_t *   me_ptr,
                                                       capi_buf_t *  buf_ptr,
                                                       bool_t        is_intra_cntr_msg,
                                                       void *        self_port_hdl,
                                                       void *        peer_port_hdl,
                                                       POSAL_HEAP_ID operating_heap_id,
                                                       uint32_t      token,
                                                       spf_handle_t *dst_handle_ptr)
{
   uint32_t out_buf_size = buf_ptr->actual_data_len + sizeof(intf_extn_param_id_imcl_incoming_data_t);
   spf_msg_t one_time_msg = {0};

   // Update the required for the data ptr and msg header.
   uint32_t req_size = GET_SPF_INLINE_CTRLBUF_REQ_SIZE(out_buf_size);

   if (POSAL_IS_ISLAND_HEAP_ID(operating_heap_id))
   {
      one_time_msg.payload_ptr = spf_lpi_pool_get_node(req_size);
      if (!one_time_msg.payload_ptr)
      {
         CU_MSG_ISLAND(me_ptr->gu_ptr->log_id, DBG_ERROR_PRIO, "Fail to create one time intent buffer message");
         return AR_EFAILED;
      }
   }
   else
   {
      // Get onetime msg from the buffer manager.
      if (AR_DID_FAIL(spf_msg_create_msg(&one_time_msg,
                                         &req_size,
                                         SPF_MSG_CMD_CTRL_PORT_TRIGGER_MSG, // opcode is updated based on trigger mode
                                                                            // when sending.
                                         NULL,
                                         NULL,
                                         dst_handle_ptr,
                                         operating_heap_id)))
      {
         CU_MSG_ISLAND(me_ptr->gu_ptr->log_id, DBG_ERROR_PRIO, "Fail to create one time intent buffer message");
         return AR_EFAILED;
      }
   }

   // Reinterpret node's buffer as a header.
   spf_msg_header_t *header_ptr = (spf_msg_header_t *)one_time_msg.payload_ptr;
   header_ptr->rsp_handle_ptr   = NULL;
   header_ptr->token.token_data = token;
   header_ptr->rsp_result       = 0;
   header_ptr->dst_handle_ptr   = dst_handle_ptr;
   // header_ptr->return_q_ptr is buffer manager q and its populated in spf_msg_create_msg()

   // Update the max size of the data buffer.
   spf_msg_ctrl_port_msg_t *ctrl_msg_ptr = (spf_msg_ctrl_port_msg_t *)&header_ptr->payload_start[0];

   ctrl_msg_ptr->is_intra_cntr           = is_intra_cntr_msg;
   ctrl_msg_ptr->src_intra_cntr_port_hdl = self_port_hdl;
   ctrl_msg_ptr->dst_intra_cntr_port_hdl = peer_port_hdl;
   ctrl_msg_ptr->max_size                = out_buf_size;

   // Get the intent buffer pointer that needs to be passed to module from the header pointer.
   buf_ptr->data_ptr     = (int8_t *)((int8_t *)header_ptr + INTENT_BUFFER_OFFSET);
   buf_ptr->max_data_len = out_buf_size - sizeof(intf_extn_param_id_imcl_incoming_data_t);
   return AR_EOK;
}

static ar_result_t cu_ctrl_port_get_one_time_buf(cu_base_t *     me_ptr,
                                                 gu_ctrl_port_t *gu_ctrl_port_ptr,
                                                 capi_buf_t *    buf_ptr)
{
   ar_result_t result = AR_EOK;
   if (!buf_ptr->actual_data_len)
   {
      CU_MSG_ISLAND(me_ptr->gu_ptr->log_id, DBG_ERROR_PRIO, " Requesting for zero size buffer. ");
      return AR_EFAILED;
   }

   cu_int_ctrl_port_t *ctrl_port_ptr =
      (cu_int_ctrl_port_t *)((uint8_t *)(gu_ctrl_port_ptr) + me_ptr->int_ctrl_port_cu_offset);

   bool_t is_intra_container = (gu_ctrl_port_ptr->ext_ctrl_port_ptr) ? FALSE : TRUE;
   result                    = cu_alloc_populate_one_time_ctrl_msg(me_ptr,
                                                buf_ptr,
                                                is_intra_container,
                                                is_intra_container ? (void *)gu_ctrl_port_ptr : NULL, // src handle
                                                is_intra_container ? (void *)(gu_ctrl_port_ptr->peer_ctrl_port_ptr)
                                                                   : NULL, // peer handle
                                                gu_ctrl_port_ptr->operating_heap_id,
                                                ctrl_port_ptr->outgoing_intent_cnt++,
                                                cu_ctrl_port_get_dst_handle(me_ptr, gu_ctrl_port_ptr));

   return result;
}

/*Load buffers to the port's buffer queue, with dst handle*/
ar_result_t cu_create_ctrl_bufs_util(cu_base_t *me_ptr, gu_ctrl_port_t *gu_ctrl_port_ptr, uint32_t new_num_bufs)
{
   INIT_EXCEPTION_HANDLING
   ar_result_t result             = AR_EOK;
   bool_t      is_inter_proc_link = (gu_ctrl_port_ptr->ext_ctrl_port_ptr)
                                  ? gu_is_domain_id_remote(gu_ctrl_port_ptr->ext_ctrl_port_ptr->peer_domain_id)
                                  : FALSE;

   cu_int_ctrl_port_t *ctrl_port_ptr =
      (cu_int_ctrl_port_t *)((uint8_t *)(gu_ctrl_port_ptr) + me_ptr->int_ctrl_port_cu_offset);

   if (new_num_bufs > CU_MAX_INTENT_Q_ELEMENTS)
   {
      CU_MSG(me_ptr->gu_ptr->log_id, DBG_ERROR_PRIO, "Not enough intent bufQ elements");
      return AR_ENORESOURCE;
   }

   if (ctrl_port_ptr->buf_size <= 1)
   {
      return AR_EOK;
   }

   CU_MSG(me_ptr->gu_ptr->log_id,
          DBG_LOW_PRIO,
          " Creating %lu outgoing intent buffers of size %lu; Self Qptr = 0x%lx",
          (new_num_bufs - ctrl_port_ptr->num_bufs),
          ctrl_port_ptr->buf_size,
          ctrl_port_ptr->buffer_q_ptr);

   TRY(result,
       spf_svc_create_and_push_ctrl_msg_bufs_to_buf_queue(ctrl_port_ptr->buffer_q_ptr,
                                                          ctrl_port_ptr->buf_size,
                                                          new_num_bufs,
                                                          cu_ctrl_port_get_dst_handle(me_ptr, gu_ctrl_port_ptr),
                                                          is_inter_proc_link,
                                                          gu_ctrl_port_ptr->operating_heap_id,
                                                          &ctrl_port_ptr->num_bufs));

   CATCH(result, CU_MSG_PREFIX, me_ptr->gu_ptr->log_id)
   {
   }

   return result;
}

static ar_result_t cu_ctrl_port_get_recurring_buf(cu_base_t *     me_ptr,
                                                  gu_ctrl_port_t *gu_ctrl_port_ptr,
                                                  capi_buf_t *    buf_ptr)
{
   ar_result_t              result           = AR_EOK;
   spf_msg_header_t *       header_ptr       = NULL;
   spf_msg_ctrl_port_msg_t *ctrl_msg_hdr_ptr = NULL;

   cu_int_ctrl_port_t *ctrl_port_ptr =
      (cu_int_ctrl_port_t *)((uint8_t *)(gu_ctrl_port_ptr) + me_ptr->int_ctrl_port_cu_offset);

   if (!ctrl_port_ptr->num_bufs || !ctrl_port_ptr->buf_size)
   {
      CU_MSG_ISLAND(me_ptr->gu_ptr->log_id,
                    DBG_ERROR_PRIO,
                    "0 num buf/num_size; No recurring bufs requested by module 0x%lx",
                    gu_ctrl_port_ptr->module_ptr->module_instance_id);
      return AR_EFAILED;
   }

   // poll until we get a buf that doesn't get recreated
   uint32_t loop_count = 0;
   while (NULL == header_ptr)
   {
      spf_msg_t recurring_msg = { 0 };
      // Take next buffer off the q
      if (AR_DID_FAIL(result = posal_queue_pop_front(ctrl_port_ptr->buffer_q_ptr, (posal_queue_element_t*) &recurring_msg)))
      {
         CU_MSG_ISLAND(me_ptr->gu_ptr->log_id, DBG_ERROR_PRIO, "Failure on Outgoing intent Buffer Pop ");
         return result;
      }

#ifdef VERBOSE_LOGGING
      CU_MSG_ISLAND(me_ptr->gu_ptr->log_id, DBG_LOW_PRIO, "outgoing intent buffer  0x%p", recurring_msg.payload_ptr);
#endif // VERBOSE_LOGGING

      header_ptr                   = (spf_msg_header_t *)recurring_msg.payload_ptr;
      header_ptr->return_q_ptr     = ctrl_port_ptr->buffer_q_ptr;
      header_ptr->rsp_handle_ptr   = NULL;
      header_ptr->token.token_data = ctrl_port_ptr->outgoing_intent_cnt++;
      header_ptr->rsp_result       = 0;
      header_ptr->dst_handle_ptr   = cu_ctrl_port_get_dst_handle(me_ptr, gu_ctrl_port_ptr);

      ctrl_msg_hdr_ptr = (spf_msg_ctrl_port_msg_t *)&header_ptr->payload_start[0];

      // if buf size doesn't match then recreate.
      if (ctrl_msg_hdr_ptr->max_size != ctrl_port_ptr->buf_size)
      {
         // Need to always exit island here due to memory operations
         // Can exit temporarily instead of voting snce it is mem free operation
         if (me_ptr->cntr_vtbl_ptr->exit_island_temporarily)
         {
            me_ptr->cntr_vtbl_ptr->exit_island_temporarily((void *)me_ptr);
         }

         // Free the popped buffer of old size.
         posal_memory_free(header_ptr);
         ctrl_port_ptr->num_bufs--;
         header_ptr = NULL;

         result = cu_create_ctrl_bufs_util(me_ptr,
                                           gu_ctrl_port_ptr,
                                           ctrl_port_ptr->num_bufs + 1); // one buf was destroyed, so create one more.

         if (AR_DID_FAIL(result))
         {
            CU_MSG_ISLAND(me_ptr->gu_ptr->log_id, DBG_ERROR_PRIO, "Intent Buffer recreate failed %d", result);
         }
      }
      // if buf got recreated, go back to loop.

      loop_count++;
      if (loop_count > 1000)
      {
         CU_MSG_ISLAND(me_ptr->gu_ptr->log_id, DBG_ERROR_PRIO, "Warning: possible infinite loop.", result);
         result = AR_EFAILED;
         break;
      }
   }

   if (header_ptr)
   {
      // Update capi buffer pointer with the return buffer.
      buf_ptr->data_ptr        = (int8_t *)((int8_t *)header_ptr + INTENT_BUFFER_OFFSET);
      buf_ptr->max_data_len    = ctrl_msg_hdr_ptr->max_size - sizeof(intf_extn_param_id_imcl_incoming_data_t);
      buf_ptr->actual_data_len = 0;
   }

#ifdef VERBOSE_LOGGING
   CU_MSG_ISLAND(me_ptr->gu_ptr->log_id,
                 DBG_HIGH_PRIO,
                 "module intent buffer  0x%p, max_size:: %lu ",
                 buf_ptr->data_ptr,
                 ctrl_msg_hdr_ptr->max_size);
#endif // VERBOSE_LOGGING

   return result;
}


ar_result_t cu_handle_imcl_event(cu_base_t *cu_ptr, gu_module_t *module_ptr, capi_event_info_t *event_info_ptr)
{
   ar_result_t                             result  = AR_EOK;
   capi_buf_t *                            payload = &event_info_ptr->payload;
   capi_event_get_data_from_dsp_service_t *dsp_event_ptr =
      (capi_event_get_data_from_dsp_service_t *)(payload->data_ptr);
   gu_ctrl_port_t *ctrl_port_ptr = NULL;
   int32_t buffer_allocated = 0; //1: buffer is allocated for control port, -1: buffer is returned by control port

   switch (dsp_event_ptr->param_id)
   {
      case INTF_EXTN_EVENT_ID_IMCL_GET_RECURRING_BUF:
      {
         if (dsp_event_ptr->payload.actual_data_len < sizeof(event_id_imcl_get_recurring_buf_t))
         {
            CU_MSG_ISLAND(cu_ptr->gu_ptr->log_id,
                   DBG_ERROR_PRIO,
                   "Module 0x%lX: Error in callback function. The actual size %lu is less than the required size "
                   "%lu for id %lu.",
                   module_ptr->module_instance_id,
                   payload->actual_data_len,
                   sizeof(event_id_imcl_get_recurring_buf_t),
                   (uint32_t)(dsp_event_ptr->param_id));
            return AR_ENEEDMORE;
         }

         event_id_imcl_get_recurring_buf_t *data_ptr =
            (event_id_imcl_get_recurring_buf_t *)(dsp_event_ptr->payload.data_ptr);

         // Get the ctrl port handle from control port ID. Takes care of island exiting if required.
         ctrl_port_ptr = (gu_ctrl_port_t *)gu_find_ctrl_port_by_id(module_ptr, data_ptr->port_id);
         if (!ctrl_port_ptr)
         {
            CU_MSG_ISLAND(cu_ptr->gu_ptr->log_id,
                   DBG_ERROR_PRIO,
                   "Module 0x%lX: Invalid control port id",
                   module_ptr->module_instance_id,
                   data_ptr->port_id);
            return AR_EFAILED;
         }

         result = cu_ctrl_port_get_recurring_buf(cu_ptr, ctrl_port_ptr, &data_ptr->buf);

         if (AR_SUCCEEDED(result))
         {
            buffer_allocated = 1; // buffer is assigned to module
         }

         break;
      }
      case INTF_EXTN_EVENT_ID_IMCL_GET_ONE_TIME_BUF:
      {
         if (dsp_event_ptr->payload.actual_data_len < sizeof(event_id_imcl_get_one_time_buf_t))
         {
            CU_MSG_ISLAND(cu_ptr->gu_ptr->log_id,
                   DBG_ERROR_PRIO,
                   "Module 0x%lX: Error in callback function. The actual size %lu is less than the required size "
                   "%lu for id %lu.",
                   module_ptr->module_instance_id,
                   payload->actual_data_len,
                   sizeof(event_id_imcl_get_one_time_buf_t),
                   (uint32_t)(dsp_event_ptr->param_id));
            return AR_ENEEDMORE;
         }

         event_id_imcl_get_one_time_buf_t *data_ptr =
            (event_id_imcl_get_one_time_buf_t *)(dsp_event_ptr->payload.data_ptr);

         // Get the ctrl port handle from control port ID.
         ctrl_port_ptr = (gu_ctrl_port_t *)gu_find_ctrl_port_by_id(module_ptr, data_ptr->port_id);
         if (ctrl_port_ptr)
         {
            result = cu_ctrl_port_get_one_time_buf(cu_ptr, ctrl_port_ptr, &data_ptr->buf);

            if (AR_SUCCEEDED(result))
            {
               buffer_allocated = 1; // buffer is assigned to module
            }
         }
         else
         {
            CU_MSG_ISLAND(cu_ptr->gu_ptr->log_id,
                   DBG_ERROR_PRIO,
                   "Module 0x%lX: no control port id 0x%lX",
                   module_ptr->module_instance_id,
                   data_ptr->port_id);
            return AR_EFAILED;
         }

         break;
      }
      case INTF_EXTN_EVENT_ID_IMCL_OUTGOING_DATA:
      {
         if (dsp_event_ptr->payload.actual_data_len < sizeof(event_id_imcl_outgoing_data_t))
         {
            CU_MSG_ISLAND(cu_ptr->gu_ptr->log_id,
                   DBG_ERROR_PRIO,
                   "Module 0x%lX: Error in callback function. The actual size %lu is less than the required size "
                   "%lu for id 0x%lX.",
                   module_ptr->module_instance_id,
                   payload->actual_data_len,
                   sizeof(event_id_imcl_outgoing_data_t),
                   (uint32_t)(dsp_event_ptr->param_id));
            return AR_ENEEDMORE;
         }

         event_id_imcl_outgoing_data_t *data_ptr = (event_id_imcl_outgoing_data_t *)(dsp_event_ptr->payload.data_ptr);

         if (!data_ptr->buf.data_ptr)
         {
            CU_MSG_ISLAND(cu_ptr->gu_ptr->log_id, DBG_ERROR_PRIO, "IMCL: Received NULL Ptr as message payload");
            return AR_EFAILED;
         }

         // Get the ctrl port handle from control port ID.
         ctrl_port_ptr = (gu_ctrl_port_t *)gu_find_ctrl_port_by_id(module_ptr, data_ptr->port_id);
         if (!ctrl_port_ptr)
         {
            CU_MSG_ISLAND(cu_ptr->gu_ptr->log_id,
                   DBG_ERROR_PRIO,
                   "Module 0x%lX: Invalid control port id",
                   module_ptr->module_instance_id,
                   data_ptr->port_id);
            return AR_EFAILED;
         }
         gu_ext_ctrl_port_t *gu_ext_ctrl_port_ptr = ctrl_port_ptr->ext_ctrl_port_ptr;
         if (gu_ext_ctrl_port_ptr && gu_is_domain_id_remote(gu_ext_ctrl_port_ptr->peer_domain_id))
         {
            if (cu_ptr->cntr_vtbl_ptr->exit_island_temporarily)
            {
               cu_ptr->cntr_vtbl_ptr->exit_island_temporarily((void *)cu_ptr);
            }

            cu_ctrl_port_process_inter_proc_outgoing_buf(cu_ptr,
                                                         gu_ext_ctrl_port_ptr,
                                                         &data_ptr->buf,
                                                         data_ptr->flags,
                                                         module_ptr->module_instance_id,
                                                         gu_ext_ctrl_port_ptr->peer_handle.port_id);
         }
         else
         {
            result = cu_ctrl_port_process_outgoing_buf(cu_ptr, ctrl_port_ptr, &data_ptr->buf, data_ptr->flags);
         }

         buffer_allocated = -1; //buffer is returned.

         break;
      }
      default:
      {
         if (cu_ptr->cntr_vtbl_ptr->exit_island_temporarily)
         {
            cu_ptr->cntr_vtbl_ptr->exit_island_temporarily((void *)cu_ptr);
         }

         return cu_handle_imcl_event_non_island(cu_ptr, module_ptr, event_info_ptr);
      }
   }

   // if container is in island and control port is in non-island then update the number of non-island buffers.
   // this will be used to prevent island entry while control port buffer is being used.
   if (ctrl_port_ptr && POSAL_IS_ISLAND_HEAP_ID(cu_ptr->heap_id) &&
       !POSAL_IS_ISLAND_HEAP_ID(ctrl_port_ptr->operating_heap_id))
   {
      int32_t num_active_buffers = cu_ptr->active_non_island_num_buffers_for_ctrl_port; // typecast as signed number.
      num_active_buffers += buffer_allocated;

      cu_ptr->active_non_island_num_buffers_for_ctrl_port = (num_active_buffers > 0) ? num_active_buffers : 0;
      CU_MSG_ISLAND(cu_ptr->gu_ptr->log_id,
                    DBG_HIGH_PRIO,
                    "number of active non-island control port buffers %lu, used %d [1:used,-1:returned] by MID 0x%x",
                    cu_ptr->active_non_island_num_buffers_for_ctrl_port,
                    buffer_allocated,
                    module_ptr->module_instance_id);

      CU_SET_ONE_FWK_EVENT_FLAG(cu_ptr, reevaluate_island_vote);
   }

   return result;
}
