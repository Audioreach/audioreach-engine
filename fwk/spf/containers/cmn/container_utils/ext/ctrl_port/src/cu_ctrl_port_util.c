/**
 * \file cu_ctrl_port_util.c
 * \brief
 *     This file contains container utility functions for control port handling (internal and external).
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "cu_i.h"
#include "cu_ctrl_port_i.h"
#include "capi_intf_extn_imcl.h"

ar_result_t cu_init_ctrl_port_buf_q(cu_base_t *me_ptr, gu_ctrl_port_t *gu_ctrl_port_ptr)
{
   // We create intent buffer queue for outgoing control messages.
   ar_result_t result = AR_EOK;
   char_t      outgoing_buf_q_name[POSAL_DEFAULT_NAME_LEN]; // data queue name
   snprintf(outgoing_buf_q_name, POSAL_DEFAULT_NAME_LEN, "%s%s%8lX", "OUT_BUF", "CNTR", me_ptr->gu_ptr->log_id);
   uint32_t num_elements = CU_MAX_INTENT_Q_ELEMENTS;

   cu_int_ctrl_port_t *ctrl_port_ptr =
      (cu_int_ctrl_port_t *)((uint8_t *)(gu_ctrl_port_ptr) + me_ptr->int_ctrl_port_cu_offset);

   POSAL_HEAP_ID operating_heap_id = gu_ctrl_port_ptr->operating_heap_id;

   // Initialize the channel for outgoing buffer queue. This is dummy channel, its not used for polling.
   if (AR_DID_FAIL(result = posal_channel_create(&ctrl_port_ptr->buf_q_channel_ptr, operating_heap_id)))
   {
      CU_MSG(me_ptr->gu_ptr->log_id, DBG_ERROR_PRIO, "Unable to create buf queue channel");
      return result;
   }

   void *queue_dest_ptr = posal_memory_malloc(posal_queue_get_size(), operating_heap_id);
   // this queue is never polled
   result = cu_init_queue(me_ptr,
                          outgoing_buf_q_name,
                          num_elements,
                          0x1,                              // bitmask (irrelevant)
                          NULL,                             /*queue handler*/
                          ctrl_port_ptr->buf_q_channel_ptr, /*channel ptr - not used*/
                          &ctrl_port_ptr->buffer_q_ptr,
                          queue_dest_ptr,
                          operating_heap_id);

   if (AR_FAILED(result))
   {
      posal_memory_free(queue_dest_ptr);
   }

   CU_MSG(me_ptr->gu_ptr->log_id,
          DBG_HIGH_PRIO,
          "Created Ctrl port's (ID 0x%lx) out-buf Q with result %lu",
          gu_ctrl_port_ptr->id,
          result);

   return result;
}

void cu_deinit_ctrl_port_buf_q(cu_base_t *me_ptr, gu_ctrl_port_t *gu_ctrl_port_ptr)
{
   cu_int_ctrl_port_t *ctrl_port_ptr =
      (cu_int_ctrl_port_t *)((uint8_t *)(gu_ctrl_port_ptr) + me_ptr->int_ctrl_port_cu_offset);

   if (!ctrl_port_ptr->buffer_q_ptr)
   {
      return;
   }

   // Destroy the buffers in the control port queue.
   // This is a blocking call. It waits until all the buffers are returned to the
   // queue from the peer.
   spf_svc_free_ctrl_msgs_in_buf_queue_blocking(ctrl_port_ptr->buffer_q_ptr, &ctrl_port_ptr->num_bufs);

   // Destroy the outgoing buf queue.
   if (ctrl_port_ptr->buffer_q_ptr)
   {
      posal_queue_destroy(ctrl_port_ptr->buffer_q_ptr);
      posal_channel_destroy(&ctrl_port_ptr->buf_q_channel_ptr);
      ctrl_port_ptr->buffer_q_ptr      = NULL;
      ctrl_port_ptr->buf_q_channel_ptr = NULL;
   }
}

ar_result_t cu_check_and_recreate_ctrl_port_buffers(cu_base_t *me_ptr, gu_ctrl_port_t *gu_ctrl_port_ptr)
{
   ar_result_t result           = AR_EOK;
   bool_t      need_to_recreate = FALSE;

   cu_int_ctrl_port_t *ctrl_port_ptr =
      (cu_int_ctrl_port_t *)((uint8_t *)(gu_ctrl_port_ptr) + me_ptr->int_ctrl_port_cu_offset);

   // Get new num_bufs and new buf size and validate.
   uint32_t new_buf_size = ctrl_port_ptr->new_event_buf_size;
   uint32_t new_num_bufs = ctrl_port_ptr->new_event_num_bufs;

   if (!ctrl_port_ptr->buffer_q_ptr && (GU_STATUS_DEFAULT != gu_ctrl_port_ptr->gu_status))
   {
      CU_MSG(me_ptr->gu_ptr->log_id, DBG_LOW_PRIO, " Cannot create ctrl buffer, buffer queue is not created yet.");
      return result;
   }

   if (!new_buf_size || !new_num_bufs)
   {
      return result;
   }

   //create the buf q if recurring buffers are required.
   if (new_num_bufs && !ctrl_port_ptr->buffer_q_ptr)
   {
      result = cu_init_ctrl_port_buf_q(me_ptr, gu_ctrl_port_ptr);
      if (AR_FAILED(result))
      {
         return result;
      }
   }

   if ((new_buf_size != ctrl_port_ptr->buf_size))
   {
      // Destroy the existing buffers in the queue using non blocking gkc svc util.
      // if we had allocated buffers, need to destroy now
      if (ctrl_port_ptr->num_bufs)
      {
         spf_svc_free_ctrl_msgs_in_buf_queue_nonblocking(ctrl_port_ptr->buffer_q_ptr, &ctrl_port_ptr->num_bufs);

         CU_MSG(me_ptr->gu_ptr->log_id, DBG_LOW_PRIO, " Destroyed old outgoing control buffers..");
      }

      ctrl_port_ptr->buf_size = new_buf_size;
      need_to_recreate        = TRUE;
   }

   if (new_num_bufs != ctrl_port_ptr->num_bufs)
   {
      need_to_recreate = TRUE;
   }

   // Create new buffers with the new size.
   if (need_to_recreate)
   {
      result = cu_create_ctrl_bufs_util(me_ptr, gu_ctrl_port_ptr, new_num_bufs);
   }

   return result;
}

ar_result_t cu_handle_imcl_event_non_island(cu_base_t *        cu_ptr,
                                            gu_module_t *      module_ptr,
                                            capi_event_info_t *event_info_ptr)
{
   ar_result_t                             result  = AR_EOK;
   capi_buf_t *                            payload = &event_info_ptr->payload;
   capi_event_get_data_from_dsp_service_t *dsp_event_ptr =
      (capi_event_get_data_from_dsp_service_t *)(payload->data_ptr);

   switch (dsp_event_ptr->param_id)
   {
      case INTF_EXTN_EVENT_ID_IMCL_RECURRING_BUF_INFO:
      {
         if (dsp_event_ptr->payload.actual_data_len < sizeof(event_id_imcl_recurring_buf_info_t))
         {
            CU_MSG(cu_ptr->gu_ptr->log_id,
                   DBG_ERROR_PRIO,
                   "Module 0x%lX: Error in callback function. The actual size %lu is less than the required "
                   "size "
                   "%lu for id 0x%lX.",
                   module_ptr->module_instance_id,
                   payload->actual_data_len,
                   sizeof(event_id_imcl_recurring_buf_info_t),
                   (uint32_t)(dsp_event_ptr->param_id));
            return AR_ENEEDMORE;
         }

         event_id_imcl_recurring_buf_info_t *data_ptr =
            (event_id_imcl_recurring_buf_info_t *)(dsp_event_ptr->payload.data_ptr);

         CU_MSG(cu_ptr->gu_ptr->log_id,
                DBG_LOW_PRIO,
                "Module 0x%lX: Recurring buffer info event received: port_id=0x%lX, num_bufs=%lu, buf_size=%lu",
                module_ptr->module_instance_id,
                data_ptr->port_id,
                data_ptr->num_bufs,
                data_ptr->buf_size);

         // Get the ctrl port handle from control port ID.
         gu_ctrl_port_t *ctrl_port_ptr = (gu_ctrl_port_t *)gu_find_ctrl_port_by_id(module_ptr, data_ptr->port_id);
         if (!ctrl_port_ptr)
         {
            CU_MSG(cu_ptr->gu_ptr->log_id,
                   DBG_ERROR_PRIO,
                   "Module 0x%lX: Invalid control port id",
                   module_ptr->module_instance_id,
                   data_ptr->port_id);
            return AR_EFAILED;
         }

         uint32_t new_event_buf_size = data_ptr->buf_size + sizeof(intf_extn_param_id_imcl_incoming_data_t);
         uint32_t new_event_num_bufs = data_ptr->num_bufs;

         cu_int_ctrl_port_t *int_ctrl_port_ptr =
            (cu_int_ctrl_port_t *)((uint8_t *)(ctrl_port_ptr) + cu_ptr->int_ctrl_port_cu_offset);

         /* Update the new buf size and number of bufs */
         int_ctrl_port_ptr->new_event_buf_size = new_event_buf_size;
         int_ctrl_port_ptr->new_event_num_bufs = new_event_num_bufs;

         cu_check_and_recreate_ctrl_port_buffers(cu_ptr, ctrl_port_ptr);

         break;
      }
      default:
      {
         CU_MSG(cu_ptr->gu_ptr->log_id,
                DBG_ERROR_PRIO,
                "Module 0x%lX: Error in callback function. ID 0x%lx not supported.",
                module_ptr->module_instance_id,
                (dsp_event_ptr->param_id));
         return AR_EUNSUPPORTED;
      }
   }

   return result;
}

ar_result_t cu_handle_ctrl_port_trigger_cmd(cu_base_t *me_ptr)
{
   ar_result_t result = AR_EOK;

#ifdef CTRL_LINK_DEBUG
   CU_MSG(me_ptr->gu_ptr->log_id,
          DBG_HIGH_PRIO,
          "CMD:CTRL_PORT_TRIGGER: Executing control port trigger msg. current channel mask=0x%x",
          me_ptr->curr_chan_mask);
#endif

   // Get the ctrl port handle from the message header.
   spf_msg_header_t *       header_ptr    = (spf_msg_header_t *)me_ptr->cmd_msg.payload_ptr;
   spf_msg_ctrl_port_msg_t *ctrl_msg_ptr  = (spf_msg_ctrl_port_msg_t *)&header_ptr->payload_start;
   gu_ctrl_port_t *         ctrl_port_ptr = NULL;

   if (ctrl_msg_ptr->is_intra_cntr)
   {
      ctrl_port_ptr = (gu_ctrl_port_t *)ctrl_msg_ptr->dst_intra_cntr_port_hdl;
   }
   else
   {
      gu_ext_ctrl_port_t *dst_port_ptr = (gu_ext_ctrl_port_t *)header_ptr->dst_handle_ptr;
      ctrl_port_ptr                    = dst_port_ptr->int_ctrl_port_ptr;
   }

#if 0
   // Check if the port state is started and then only do the set param. Else return.
   topo_port_state_t ctrl_port_state;
   me_ptr->topo_vtbl_ptr->get_port_property(me_ptr->topo_ptr,
                                            TOPO_CONTROL_PORT_TYPE,
                                            PORT_PROPERTY_TOPO_STATE,
                                            (void *)ctrl_port_ptr,
                                            (uint32_t *)&ctrl_port_state);
#endif

   // Send the incoming intent to the capi module.
   result = me_ptr->topo_island_vtbl_ptr->handle_incoming_ctrl_intent(ctrl_port_ptr,
                                                                     &ctrl_msg_ptr->data_buf[0],
                                                                     ctrl_msg_ptr->max_size,
                                                                     ctrl_msg_ptr->actual_size);

   if (AR_DID_FAIL(result))
   {
      CU_MSG(me_ptr->gu_ptr->log_id, DBG_HIGH_PRIO, "CMD:CTRL_PORT_TRIGGER: Handling incoming control message failed.");
   }
   else
   {
      CU_MSG(me_ptr->gu_ptr->log_id,
             DBG_LOW_PRIO,
             "CMD:CTRL_PORT_TRIGGER: mid: 0x%x ctrl port id 0x%lx Set incoming ctrl message on done.",
             ctrl_port_ptr->module_ptr->module_instance_id,
             ctrl_port_ptr->id);
   }

   return spf_msg_return_msg(&me_ptr->cmd_msg);
}
