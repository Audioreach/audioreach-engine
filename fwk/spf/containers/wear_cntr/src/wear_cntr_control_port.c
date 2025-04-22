/**
 * \file wear_cntr_ctrl_port.c
 * \brief
 *     This file contains container utility functions for external control port handling (input and output).
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "wear_cntr_i.h"
#include "capi_intf_extn_imcl.h"
#include "posal_queue.h"
/* =======================================================================
Static Function Definitions
========================================================================== */

#define INTENT_BUFFER_OFFSET                                                                                           \
   ((sizeof(spf_msg_header_t) - sizeof(uint64_t)) + (sizeof(spf_msg_ctrl_port_msg_t) - sizeof(uint64_t)) +             \
    sizeof(intf_extn_param_id_imcl_incoming_data_t))


static ar_result_t wcntr_send_ctrl_trigger_spf_msg(spf_msg_t *msg_ptr, spf_handle_t *dst_handle_ptr)
{
   return spf_msg_send_cmd(msg_ptr, dst_handle_ptr);
}

static ar_result_t wcntr_send_ctrl_polling_spf_msg(spf_msg_t *msg_ptr, spf_handle_t *dst_handle_ptr)
{
   ar_result_t    result     = AR_EOK;
   posal_queue_t *data_q_ptr = dst_handle_ptr->q_ptr;
   if (NULL == data_q_ptr)
   {
      AR_MSG_ISLAND(DBG_ERROR_PRIO,
                    "spf_msg_send() failed: dst handle's queue is NULL. Message "
                    "opcode: 0x%lx",
                    msg_ptr->msg_opcode);
      return AR_EFAILED;
   }

   result = posal_queue_push_back(data_q_ptr, (posal_queue_element_t *)msg_ptr);
   if (AR_DID_FAIL(result))
   {
      AR_MSG_ISLAND(DBG_ERROR_PRIO,
                    "spf_msg_send() failed: failed to push to the dst handle's "
                    "queue. Message opcode: 0x%lx",
                    msg_ptr->msg_opcode);
      return AR_EFAILED;
   }
   return AR_EOK;
}


ar_result_t wcntr_handle_imcl_ext_event(wcntr_base_t *        cu_ptr,
                                           wcntr_gu_module_t *      module_ptr,
                                           capi_event_info_t *event_info_ptr)
{
   ar_result_t                             result  = AR_EOK;
   capi_buf_t *                            payload = &event_info_ptr->payload;
   capi_event_get_data_from_dsp_service_t *dsp_event_ptr =
      (capi_event_get_data_from_dsp_service_t *)(payload->data_ptr);

   WCNTR_MSG(cu_ptr->gu_ptr->log_id,
             DBG_HIGH_PRIO,
             "Module 0x%lX: handle_imcl_ext_event pid 0x%X ",
             module_ptr->module_instance_id,
             dsp_event_ptr->param_id);

   switch (dsp_event_ptr->param_id)
   {
      case INTF_EXTN_EVENT_ID_IMCL_GET_RECURRING_BUF:
      {
         if (dsp_event_ptr->payload.actual_data_len < sizeof(event_id_imcl_get_recurring_buf_t))
         {
            WCNTR_MSG(cu_ptr->gu_ptr->log_id,
                      DBG_ERROR_PRIO,
                      "Module 0x%lX: Error in callback function. The actual size %lu is less than the required size "
                      "%lu for id %lu.",
                      module_ptr->module_instance_id,
                      payload->actual_data_len,
                      sizeof(event_id_imcl_get_recurring_buf_t),
                      (uint32_t)(dsp_event_ptr->param_id));
            return AR_ENEEDMORE;
         }

         // Get the ctrl port handle from control port ID.
           event_id_imcl_get_recurring_buf_t *data_ptr =
            (event_id_imcl_get_recurring_buf_t *)(dsp_event_ptr->payload.data_ptr);

		 wcntr_gu_ctrl_port_t *ctrl_port_ptr =
			  (wcntr_gu_ctrl_port_t *)wcntr_gu_find_ctrl_port_by_id(module_ptr, data_ptr->port_id);

		  
         if (!ctrl_port_ptr)
         {
            WCNTR_MSG(cu_ptr->gu_ptr->log_id,
                      DBG_ERROR_PRIO,
                      "Module 0x%lX: Invalid control port id",
                      module_ptr->module_instance_id,
                      data_ptr->port_id);
            return AR_EFAILED;
         }
		 
         WCNTR_MSG(cu_ptr->gu_ptr->log_id,
             DBG_HIGH_PRIO,
             "Module 0x%lX: control port id 0x%lX  called INTF_EXTN_EVENT_ID_IMCL_GET_RECURRING_BUF",
             module_ptr->module_instance_id,
             ctrl_port_ptr->id);	  

         {
            wcntr_int_ctrl_port_get_recurring_buf(cu_ptr, ctrl_port_ptr, &data_ptr->buf);         
         }

         break;
      }
      case INTF_EXTN_EVENT_ID_IMCL_GET_ONE_TIME_BUF:
      {
         if (dsp_event_ptr->payload.actual_data_len < sizeof(event_id_imcl_get_one_time_buf_t))
         {
            WCNTR_MSG(cu_ptr->gu_ptr->log_id,
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
         wcntr_gu_ctrl_port_t *ctrl_port_ptr =
            (wcntr_gu_ctrl_port_t *)wcntr_gu_find_ctrl_port_by_id(module_ptr, data_ptr->port_id);
         if (ctrl_port_ptr)
         {
                  WCNTR_MSG(cu_ptr->gu_ptr->log_id,
             DBG_HIGH_PRIO,
             "Module 0x%lX: control port id 0x%lX called IMCL_GET_ONE_TIME_BUF",
             module_ptr->module_instance_id,
             ctrl_port_ptr->id);	
            wcntr_int_ctrl_port_get_one_time_buf(cu_ptr, ctrl_port_ptr, &data_ptr->buf);
         }
         else
         {
            WCNTR_MSG(cu_ptr->gu_ptr->log_id,
                      DBG_ERROR_PRIO,
                      "Module 0x%lX: no control port id %lu",
                      module_ptr->module_instance_id,
                      data_ptr->port_id);
            return CAPI_EFAILED;
         }

         break;
      }
      case INTF_EXTN_EVENT_ID_IMCL_RECURRING_BUF_INFO:
      {
         if (dsp_event_ptr->payload.actual_data_len < sizeof(event_id_imcl_recurring_buf_info_t))
         {
            WCNTR_MSG(cu_ptr->gu_ptr->log_id,
                      DBG_ERROR_PRIO,
                      "Module 0x%lX: Error in callback function. The actual size %lu is less than the required size "
                      "%lu for id %lu.",
                      module_ptr->module_instance_id,
                      payload->actual_data_len,
                      sizeof(event_id_imcl_recurring_buf_info_t),
                      (uint32_t)(dsp_event_ptr->param_id));
            return AR_ENEEDMORE;
         }

         event_id_imcl_recurring_buf_info_t *data_ptr =
            (event_id_imcl_recurring_buf_info_t *)(dsp_event_ptr->payload.data_ptr);

         // Get the ctrl port handle from control port ID.
         wcntr_gu_ctrl_port_t *ctrl_port_ptr =
            (wcntr_gu_ctrl_port_t *)wcntr_gu_find_ctrl_port_by_id(module_ptr, data_ptr->port_id);
         if (!ctrl_port_ptr)
         {
            WCNTR_MSG(cu_ptr->gu_ptr->log_id,
                      DBG_ERROR_PRIO,
                      "Module 0x%lX: Invalid control port id",
                      module_ptr->module_instance_id,
                      data_ptr->port_id);
            return AR_EFAILED;
         }

         {

	

         uint32_t new_event_buf_size = data_ptr->buf_size + sizeof(intf_extn_param_id_imcl_incoming_data_t);
         uint32_t new_event_num_bufs = data_ptr->num_bufs;
            wcu_int_ctrl_port_t *int_ctrl_port_ptr =
               (wcu_int_ctrl_port_t *)((uint8_t *)(ctrl_port_ptr) + cu_ptr->int_ctrl_port_cu_offset);

            /* Update the new buf size and number of bufs */
            int_ctrl_port_ptr->new_event_buf_size = new_event_buf_size;
            int_ctrl_port_ptr->new_event_num_bufs = new_event_num_bufs;

					 WCNTR_MSG(cu_ptr->gu_ptr->log_id,
             DBG_HIGH_PRIO,
             "Module 0x%lX: control port id 0x%lX  called IMCL_RECURRING_BUF_INFO with new_event_buf_size %u,new_event_num_bufs %u",
             module_ptr->module_instance_id,
             ctrl_port_ptr->id,new_event_buf_size,new_event_num_bufs);

            wcntr_check_and_recreate_int_ctrl_port_buffers(cu_ptr, ctrl_port_ptr);

         }

         break;
      }
      case INTF_EXTN_EVENT_ID_IMCL_OUTGOING_DATA:
      {
         if (dsp_event_ptr->payload.actual_data_len < sizeof(event_id_imcl_outgoing_data_t))
         {
            WCNTR_MSG(cu_ptr->gu_ptr->log_id,
                      DBG_ERROR_PRIO,
                      "Module 0x%lX: Error in callback function. The actual size %lu is less than the required size "
                      "%lu for id %lu.",
                      module_ptr->module_instance_id,
                      payload->actual_data_len,
                      sizeof(event_id_imcl_outgoing_data_t),
                      (uint32_t)(dsp_event_ptr->param_id));
            return AR_ENEEDMORE;
         }

         event_id_imcl_outgoing_data_t *data_ptr = (event_id_imcl_outgoing_data_t *)(dsp_event_ptr->payload.data_ptr);

         // Get the ctrl port handle from control port ID.
         wcntr_gu_ctrl_port_t *ctrl_port_ptr =
            (wcntr_gu_ctrl_port_t *)wcntr_gu_find_ctrl_port_by_id(module_ptr, data_ptr->port_id);
         if (!ctrl_port_ptr)
         {
            WCNTR_MSG(cu_ptr->gu_ptr->log_id,
                      DBG_ERROR_PRIO,
                      "Module 0x%lX: Invalid control port id",
                      module_ptr->module_instance_id,
                      data_ptr->port_id);
            return AR_EFAILED;
         }
         {
           		 WCNTR_MSG(cu_ptr->gu_ptr->log_id,
             DBG_HIGH_PRIO,
             "Module 0x%lX: control port id 0x%lX IMCL_OUTGOING_DATA",
             module_ptr->module_instance_id,
             ctrl_port_ptr->id);
				 
            result = wcntr_int_ctrl_port_process_outgoing_buf(cu_ptr, ctrl_port_ptr, &data_ptr->buf, data_ptr->flags);
         }
         break;
      }
      default:
      {
         WCNTR_MSG(cu_ptr->gu_ptr->log_id,
                   DBG_ERROR_PRIO,
                   "Module 0x%lX: Error in callback function. ID 0x%lx not supported.",
                   module_ptr->module_instance_id,
                   (dsp_event_ptr->param_id));
         return AR_EUNSUPPORTED;
      }
   }

   return result;
}

/*
 * Create a control port control queue common for all the intra-container control links.
 */
ar_result_t wcntr_init_int_ctrl_port_queue(wcntr_base_t *cu_ptr, void* q_dest_ptr)
{
   ar_result_t result   = AR_EOK;
   uint32_t    bit_mask = 0;

   char incoming_intent_q_name[POSAL_DEFAULT_NAME_LEN]; // data queue name
   snprintf(incoming_intent_q_name, POSAL_DEFAULT_NAME_LEN, "%s%s%8lX", "INT_IMCL_Q", "WCNTR", cu_ptr->gu_ptr->log_id);

   uint32_t num_elements = WCNTR_MAX_INTENT_Q_ELEMENTS;

   bit_mask = wcntr_request_bit_in_bit_mask(&cu_ptr->available_ctrl_chan_mask);
   if (0 == bit_mask)
   {
      WCNTR_MSG(cu_ptr->gu_ptr->log_id,
             DBG_ERROR_PRIO,
             "Bit mask has no bits available 0x%lx",
             cu_ptr->available_bit_mask);
      return result;
   }

   cu_ptr->cntr_cmn_imcl_handle.cmd_handle_ptr = &cu_ptr->cmd_handle;

   // Intialize the channel for the polling the incoming buffer queue.

   // init the queue
   result = wcntr_init_queue(cu_ptr,
                            incoming_intent_q_name,
                            num_elements,
                            bit_mask,
                            NULL, /*queue handler*/
                            cu_ptr->ctrl_channel_ptr,
                            &cu_ptr->cntr_cmn_imcl_handle.q_ptr,
                            q_dest_ptr,
                            cu_ptr->heap_id);

   WCNTR_MSG(cu_ptr->gu_ptr->log_id,
          DBG_HIGH_PRIO,
          "IMCL control Q 0x%lX inited with max incoming intent elements = %lu, bit_mask: "
          "0x%lX available_ctrl_chan_mask 0x%lX ",
          cu_ptr->cntr_cmn_imcl_handle.q_ptr,
          num_elements,
          bit_mask,cu_ptr->available_ctrl_chan_mask);							

   return result;
}

ar_result_t wcntr_deinit_int_ctrl_port_queue(wcntr_base_t *cu_ptr)
{
   ar_result_t result = AR_EOK;
   if (cu_ptr->cntr_cmn_imcl_handle.q_ptr)
   {
      uint32_t bitmask = posal_queue_get_channel_bit(cu_ptr->cntr_cmn_imcl_handle.q_ptr);
      WCNTR_MSG(cu_ptr->gu_ptr->log_id,
             DBG_HIGH_PRIO,
             "IMCL control Q 0x%lX will be deinited bit_mask: "
             "0x%lx",cu_ptr->cntr_cmn_imcl_handle.q_ptr,
             bitmask);
      /*Release the available control channel mask mask */
      cu_ptr->available_ctrl_chan_mask |= bitmask;

      /*deinit the queue */
      posal_queue_deinit(cu_ptr->cntr_cmn_imcl_handle.q_ptr);
      cu_ptr->cntr_cmn_imcl_handle.q_ptr = NULL;
   }
   return result;
}

/*Load buffers to the port's buffer queue, with dst handle as the cntr cmn imcl q*/
static ar_result_t wcntr_create_int_ctrl_bufs_util(wcntr_base_t *     me_ptr,
                                                wcntr_gu_ctrl_port_t *gu_ctrl_port_ptr,
                                                uint32_t        new_num_bufs)
{
   INIT_EXCEPTION_HANDLING
   ar_result_t result                   = AR_EOK;
   bool_t      IS_INTER_PROC_LINK_FALSE = FALSE;

   wcu_int_ctrl_port_t *ctrl_port_ptr =
      (wcu_int_ctrl_port_t *)((uint8_t *)(gu_ctrl_port_ptr) + me_ptr->int_ctrl_port_cu_offset);

   if (new_num_bufs > WCNTR_MAX_INTENT_Q_ELEMENTS)
   {
      WCNTR_MSG(me_ptr->gu_ptr->log_id, DBG_ERROR_PRIO, "Not enough intent bufQ elements");
      return AR_ENORESOURCE;
   }

   if (ctrl_port_ptr->buf_size <= 1)
   {
      return AR_EOK;
   }

   WCNTR_MSG(me_ptr->gu_ptr->log_id,
          DBG_LOW_PRIO,
          " Creating %lu buffers (new_num_bufs %u,old num_bufs %u) outgoing intent buffers of size %lu; Self Qptr = 0x%lX",
          (new_num_bufs - ctrl_port_ptr->num_bufs),new_num_bufs,ctrl_port_ptr->num_bufs,
          ctrl_port_ptr->buf_size,
          ctrl_port_ptr->buffer_q_ptr);

   TRY(result,
       spf_svc_create_and_push_ctrl_msg_bufs_to_buf_queue(ctrl_port_ptr->buffer_q_ptr,
                                                          ctrl_port_ptr->buf_size,
                                                          new_num_bufs,
                                                          &me_ptr->cntr_cmn_imcl_handle,
                                                          IS_INTER_PROC_LINK_FALSE,
                                                          me_ptr->heap_id,
                                                          &ctrl_port_ptr->num_bufs));

   CATCH(result, WCNTR_MSG_PREFIX, me_ptr->gu_ptr->log_id)
   {
   }

   return result;
}

ar_result_t wcntr_check_and_recreate_int_ctrl_port_buffers(wcntr_base_t *me_ptr, wcntr_gu_ctrl_port_t *gu_ctrl_port_ptr)
{
   ar_result_t result           = AR_EOK;
   bool_t      need_to_recreate = FALSE;

   wcu_int_ctrl_port_t *ctrl_port_ptr =
      (wcu_int_ctrl_port_t *)((uint8_t *)(gu_ctrl_port_ptr) + me_ptr->int_ctrl_port_cu_offset);

   // Get new num_bufs and new buf size and validate.
   uint32_t new_buf_size = ctrl_port_ptr->new_event_buf_size;
   uint32_t new_num_bufs = ctrl_port_ptr->new_event_num_bufs;

   if (!ctrl_port_ptr->buffer_q_ptr)
   {
      WCNTR_MSG(me_ptr->gu_ptr->log_id, DBG_LOW_PRIO, " Cannot create ctrl buffer, buffer queue is not created yet.");
      return result;
   }

   if (!new_buf_size || !new_num_bufs)
   {
      return result;
   }

   if ((new_buf_size != ctrl_port_ptr->buf_size))
   {
      // Destroy the existing buffers in the queue using non blocking gkc svc util.
      // if we had allocated buffers, need to destroy now
      if (ctrl_port_ptr->num_bufs)
      {
         spf_svc_free_ctrl_msgs_in_buf_queue_nonblocking(ctrl_port_ptr->buffer_q_ptr, &ctrl_port_ptr->num_bufs);

         WCNTR_MSG(me_ptr->gu_ptr->log_id, DBG_LOW_PRIO, " Destroyed old outgoing control buffers..");
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
      result = wcntr_create_int_ctrl_bufs_util(me_ptr, gu_ctrl_port_ptr, new_num_bufs);
   }

   return result;
}

ar_result_t wcntr_int_ctrl_port_get_recurring_buf(wcntr_base_t *me_ptr, wcntr_gu_ctrl_port_t *gu_ctrl_port_ptr, capi_buf_t *buf_ptr)
{
   ar_result_t              result           = AR_EOK;
   spf_msg_header_t *       header_ptr       = NULL;
   spf_msg_ctrl_port_msg_t *ctrl_msg_hdr_ptr = NULL;

   wcu_int_ctrl_port_t *ctrl_port_ptr =
      (wcu_int_ctrl_port_t *)((uint8_t *)(gu_ctrl_port_ptr) + me_ptr->int_ctrl_port_cu_offset);

   if (!ctrl_port_ptr->num_bufs || !ctrl_port_ptr->buf_size)
   {
      return AR_EFAILED;
   }

   // poll until we get a buf that doesn't get recreated
   uint32_t loop_count = 0;
   while (NULL == header_ptr)
   {
      // Take next buffer off the q
      if (AR_DID_FAIL(result = posal_queue_pop_front(ctrl_port_ptr->buffer_q_ptr, (posal_queue_element_t*)&(ctrl_port_ptr->recurring_msg))))
      {
         WCNTR_MSG(me_ptr->gu_ptr->log_id, DBG_ERROR_PRIO, "Failure on Outgoing intent Buffer Pop ");
         return result;
      }

#ifdef VERBOSE_LOGGING
      WCNTR_MSG(me_ptr->gu_ptr->log_id,
             DBG_LOW_PRIO,
             "outgoing intent buffer  0x%p",
             ctrl_port_ptr->recurring_msg.q_payload_ptr);
#endif // VERBOSE_LOGGING

      header_ptr                   = (spf_msg_header_t *)ctrl_port_ptr->recurring_msg.payload_ptr;
      header_ptr->return_q_ptr     = ctrl_port_ptr->buffer_q_ptr;
      header_ptr->rsp_handle_ptr   = NULL;
      header_ptr->token.token_data = ctrl_port_ptr->outgoing_intent_cnt++;
      header_ptr->rsp_result       = 0;
      header_ptr->dst_handle_ptr   = &me_ptr->cntr_cmn_imcl_handle;

      ctrl_msg_hdr_ptr = (spf_msg_ctrl_port_msg_t *)&header_ptr->payload_start[0];

      // if buf size doesn't match then recreate.
      if (ctrl_msg_hdr_ptr->max_size != ctrl_port_ptr->buf_size)
      {
         // AKR: check and exit island after moving (phase 2)
         // Free the popped buffer of old size.
         posal_memory_free(header_ptr);
         ctrl_port_ptr->num_bufs--;
         ctrl_port_ptr->recurring_msg.payload_ptr = NULL;

         result =
            wcntr_create_int_ctrl_bufs_util(me_ptr,
                                         gu_ctrl_port_ptr,
                                         ctrl_port_ptr->num_bufs + 1); // one buf was destroyed, so create one more.
         if (AR_DID_FAIL(result))
         {
            WCNTR_MSG(me_ptr->gu_ptr->log_id, DBG_ERROR_PRIO, "Intent Buffer recreate failed %d", result);
         }
      }
      // if buf got recreated, go back to loop.

      loop_count++;
      if (loop_count > 1000)
      {
         WCNTR_MSG(me_ptr->gu_ptr->log_id, DBG_ERROR_PRIO, "Warning: possible infinite loop.", result);
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

//#ifdef VERBOSE_LOGGING
   WCNTR_MSG(me_ptr->gu_ptr->log_id,
          DBG_HIGH_PRIO,
          "module intent buffer  0x%p, max_size:: %lu ",
          buf_ptr->data_ptr,
          ctrl_msg_hdr_ptr->max_size);
//#endif // VERBOSE_LOGGING

   return result;
}

ar_result_t wcntr_init_internal_ctrl_port(wcntr_base_t *me_ptr, wcntr_gu_ctrl_port_t *gu_ctrl_port_ptr)
{
   // We create intent buffer queue for outgoing control messages.
   ar_result_t result = AR_EOK;
   char_t      outgoing_buf_q_name[POSAL_DEFAULT_NAME_LEN]; // data queue name
   snprintf(outgoing_buf_q_name, POSAL_DEFAULT_NAME_LEN, "%s%s%8lX", "OUT_BUF", "CNTR", me_ptr->gu_ptr->log_id);
   uint32_t num_elements = WCNTR_MAX_INTENT_Q_ELEMENTS;

   wcu_int_ctrl_port_t *ctrl_port_ptr =
      (wcu_int_ctrl_port_t *)((uint8_t *)(gu_ctrl_port_ptr) + me_ptr->int_ctrl_port_cu_offset);


	if(ctrl_port_ptr->buffer_q_ptr) 
	{
	WCNTR_MSG(me_ptr->gu_ptr->log_id, DBG_HIGH_PRIO, "Module id 0x%lX control port 0x%lX buffer_q_ptr already created",gu_ctrl_port_ptr->module_ptr->module_instance_id,gu_ctrl_port_ptr->id);
	return result;
	}

   // Intialize the channel for outgoing buffer queue. This is dummy channel, its not used for polling.
   if (AR_DID_FAIL(result = posal_channel_create(&ctrl_port_ptr->buf_q_channel_ptr, me_ptr->heap_id)))
   {
      WCNTR_MSG(me_ptr->gu_ptr->log_id, DBG_ERROR_PRIO, "Unable to create buf queue channel");
      return result;
   }

   // this queue is never polled
   result = wcntr_init_queue(me_ptr,
                            outgoing_buf_q_name,
                            num_elements,
                            0x1,                              // bitmask (irrelevant)
                            NULL,                             /*queue handler*/
                            ctrl_port_ptr->buf_q_channel_ptr, /*channel ptr - not used*/
                            &ctrl_port_ptr->buffer_q_ptr,
							WCNTR_GET_INT_CTRL_PORT_Q_OFFSET_ADDR(gu_ctrl_port_ptr),
                            me_ptr->heap_id);

   WCNTR_MSG(me_ptr->gu_ptr->log_id,
          DBG_HIGH_PRIO,
          " For (miid,ctrl port_id) (0x%lX,0x%lX) created out-buf Q 0x%lX  with result %lu",
         gu_ctrl_port_ptr->module_ptr->module_instance_id, gu_ctrl_port_ptr->id,ctrl_port_ptr->buffer_q_ptr,
          result);

   return result;
}

/*
 *Go over the messages in the queue, check the source,
 *    if it belongs to the ctrl port that wants to retrieve,
 *    return to Q
 */
ar_result_t wcntr_check_and_retrieve_bufq_msgs_from_cmn_ctrlq(wcntr_base_t *me_ptr, wcntr_gu_ctrl_port_t *gu_ctrl_port_ptr)
{
   ar_result_t result = AR_EOK;
   // Poll for set channel_bits
   uint32_t set_ch_bitmask =
      posal_channel_poll(me_ptr->ctrl_channel_ptr, posal_queue_get_channel_bit(me_ptr->cntr_cmn_imcl_handle.q_ptr));
   void *port_hdl       = (void*)gu_ctrl_port_ptr;
   void *   first_msg_ptr = NULL;
   if (!set_ch_bitmask)
   {
      // queue is empty, can return safely
      return AR_EOK;
   }

   spf_msg_t temp_msg;
   while (1)
   {
      // we need to do this till we encounter the first message again

      // pop the first message
      result = posal_queue_pop_front(me_ptr->cntr_cmn_imcl_handle.q_ptr, (posal_queue_element_t *)&temp_msg);

      if (AR_ENEEDMORE == result)
      {
         return AR_EOK;
      }

      if (AR_EOK != result)
      {
         WCNTR_MSG(me_ptr->gu_ptr->log_id, DBG_ERROR_PRIO, "Couldn't pop incoming ctrl message from the queue.");
         return result;
      }
      // check if it came from the ctrl port in question
      // Extract the data buffer from the ctrl msg.
      spf_msg_header_t *header_ptr = (spf_msg_header_t *)temp_msg.payload_ptr;
      if (NULL == first_msg_ptr)
      {
         first_msg_ptr = temp_msg.payload_ptr; // cache
      }
      spf_msg_ctrl_port_msg_t *ctrl_msg_ptr = (spf_msg_ctrl_port_msg_t *)&header_ptr->payload_start[0];
      // if anything is addressed to this port, or anything is coming from this port
      if ((port_hdl == ctrl_msg_ptr->src_intra_cntr_port_hdl) || (port_hdl == ctrl_msg_ptr->dst_intra_cntr_port_hdl))
      {
         if (first_msg_ptr == temp_msg.payload_ptr)
         {
            // going to return the first message, so set it to NULL again.
            first_msg_ptr = NULL;
         }
         spf_msg_return_msg(&temp_msg);
#ifdef CTRL_LINK_DEBUG
         WCNTR_MSG(me_ptr->gu_ptr->log_id,
                DBG_MED_PRIO,
                "Check and retrieve: Returned buffer from ctrl q to bufq of port id 0x%lx",
                gu_ctrl_port_ptr->id);
#endif         
      }
      else
      {
         // push back to the queue
         wcntr_send_ctrl_polling_spf_msg(&temp_msg, &me_ptr->cntr_cmn_imcl_handle);
      }

      if (first_msg_ptr)
      {
         spf_msg_t *front_ptr = NULL;
         posal_queue_peek_front(me_ptr->cntr_cmn_imcl_handle.q_ptr, (posal_queue_element_t**)&front_ptr);
         if (front_ptr->payload_ptr == first_msg_ptr)
         {
            // wrapped around the queue, we are done
            break;
         }
      }
   }
   return result;
}

/*
 * De-init the internal control port. The following
 *    1. Check the Container common control Q for any messages from this port's bufQ.
 *      1a) return to Q
 *    2. Destroy outgoing intent buffers.
 *    3. Destroy outgoing queue.
 */
ar_result_t wcntr_deinit_internal_ctrl_port(wcntr_base_t *me_ptr, wcntr_gu_ctrl_port_t *gu_ctrl_port_ptr)
{
   ar_result_t result = AR_EOK;

   wcu_int_ctrl_port_t *ctrl_port_ptr =
      (wcu_int_ctrl_port_t *)((uint8_t *)(gu_ctrl_port_ptr) + me_ptr->int_ctrl_port_cu_offset);
   
   if(!ctrl_port_ptr->buffer_q_ptr)
   {
      return result;
   }
         
   // 1.
   result = wcntr_check_and_retrieve_bufq_msgs_from_cmn_ctrlq(me_ptr, gu_ctrl_port_ptr);
   // Destroy the buffers in the control port queue.
   // This is a blocking call. It waits until all the buffers are returned to the
   // queue from the peer.
   uint32_t num_out_bufs = ctrl_port_ptr->num_bufs;

   // Destroy buffers in the queue.
   spf_svc_free_ctrl_msgs_in_buf_queue_blocking(ctrl_port_ptr->buffer_q_ptr, &num_out_bufs);

   WCNTR_MSG(me_ptr->gu_ptr->log_id,
          DBG_HIGH_PRIO,
          "Internal Control port id 0x%lx intent buffers are destroyed, remaining %lu ",
          gu_ctrl_port_ptr->id,
          num_out_bufs);

   // Destroy the outgoing buf queue.
   if (ctrl_port_ptr->buffer_q_ptr)
   {
      posal_queue_deinit(ctrl_port_ptr->buffer_q_ptr);
      posal_channel_destroy(&ctrl_port_ptr->buf_q_channel_ptr);
      ctrl_port_ptr->buffer_q_ptr = NULL;
   }

   return result;
}

ar_result_t wcntr_int_ctrl_port_process_outgoing_buf(wcntr_base_t *               me_ptr,
                                                  wcntr_gu_ctrl_port_t *          gu_ctrl_port_ptr,
                                                  capi_buf_t *              buf_ptr,
                                                  imcl_outgoing_data_flag_t flags)
{
   ar_result_t result = AR_EOK;

   wcu_int_ctrl_port_t *ctrl_port_ptr =
      (wcu_int_ctrl_port_t *)((uint8_t *)(gu_ctrl_port_ptr) + me_ptr->int_ctrl_port_cu_offset);

   if (!buf_ptr->data_ptr)
   {
      return AR_EFAILED;
   }

   if ((NULL == gu_ctrl_port_ptr->peer_ctrl_port_ptr) || (NULL == gu_ctrl_port_ptr->peer_ctrl_port_ptr->module_ptr))
   {
      WCNTR_MSG(me_ptr->gu_ptr->log_id,
             DBG_ERROR_PRIO,
             " No Peer Ctrl port for internal ctrl port ID 0x%lx",
             gu_ctrl_port_ptr->id);
      return AR_EFAILED;
   }
   uint32_t peer_port_id = gu_ctrl_port_ptr->peer_ctrl_port_ptr->id;   
   // add the intf extn header to the actual size to get the actual size of outgoing intent param size.
   buf_ptr->actual_data_len += sizeof(intf_extn_param_id_imcl_incoming_data_t);

   // Get spf_msg_header_t* from the intent buf pointer.
   spf_msg_header_t *header_ptr = (spf_msg_header_t *)((int8_t *)buf_ptr->data_ptr - INTENT_BUFFER_OFFSET);

   bool_t is_recurring = FALSE, IS_INTRA_CNTR_TRUE = TRUE;
   if (header_ptr->return_q_ptr == ctrl_port_ptr->buffer_q_ptr)
   {
      // If the return Q of the buffer is same as the control ports buffer q then its recurring,
      // else its one time buffer
      // return q is populated when module request for buffer.
      is_recurring = TRUE;
   }

   result = wcntr_prep_and_send_outgoing_ctrl_msg(me_ptr,
                                               header_ptr,
                                               &me_ptr->cntr_cmn_imcl_handle,
                                               buf_ptr,
                                               flags,
                                               IS_INTRA_CNTR_TRUE,
                                               peer_port_id,
                                               (void *)gu_ctrl_port_ptr,
                                               (void *)(gu_ctrl_port_ptr->peer_ctrl_port_ptr),
                                               is_recurring);

   return result;
}

ar_result_t wcntr_int_ctrl_port_get_one_time_buf(wcntr_base_t *me_ptr, wcntr_gu_ctrl_port_t *gu_ctrl_port_ptr, capi_buf_t *buf_ptr)
{
   ar_result_t result = AR_EOK;
   if (!buf_ptr->actual_data_len)
   {

      WCNTR_MSG(me_ptr->gu_ptr->log_id, DBG_ERROR_PRIO, " Requesting for zero size buffer. ");
      return AR_EFAILED;
   }

   wcu_int_ctrl_port_t *ctrl_port_ptr =
      (wcu_int_ctrl_port_t *)((uint8_t *)(gu_ctrl_port_ptr) + me_ptr->int_ctrl_port_cu_offset);

   bool_t IS_INTRA_CNTR_TRUE = TRUE;
   result = wcntr_alloc_populate_one_time_ctrl_msg(me_ptr,
                                                buf_ptr,
                                                IS_INTRA_CNTR_TRUE,
                                                (void *)gu_ctrl_port_ptr, //src handle
                                                (void *)(gu_ctrl_port_ptr->peer_ctrl_port_ptr), //peer handle
                                                me_ptr->heap_id,
                                                &(ctrl_port_ptr->one_time_msg),
                                                ctrl_port_ptr->outgoing_intent_cnt++,
                                                &me_ptr->cntr_cmn_imcl_handle);

   return result;
}

ar_result_t wcntr_init_internal_ctrl_ports(wcntr_base_t *base_ptr)
{
   INIT_EXCEPTION_HANDLING
   ar_result_t result = AR_EOK;
   // Iterate through all sgs and modules to get to the internal control ports
   for (wcntr_gu_sg_list_t *sg_list_ptr = base_ptr->gu_ptr->sg_list_ptr; (NULL != sg_list_ptr); LIST_ADVANCE(sg_list_ptr))
   {   
      for (wcntr_gu_module_list_t *module_list_ptr = sg_list_ptr->sg_ptr->module_list_ptr; (NULL != module_list_ptr);
           LIST_ADVANCE(module_list_ptr))
      {
         wcntr_gu_module_t *module_ptr = module_list_ptr->module_ptr;

         /** Iterate through module's input ports and apply port states. */
         for (wcntr_gu_ctrl_port_list_t *ctrl_port_list_ptr = module_ptr->ctrl_port_list_ptr; (NULL != ctrl_port_list_ptr);
              LIST_ADVANCE(ctrl_port_list_ptr))
         {
            wcntr_gu_ctrl_port_t *ctrl_port_ptr = ctrl_port_list_ptr->ctrl_port_ptr;

               // internal control port
               TRY(result, wcntr_init_internal_ctrl_port(base_ptr, ctrl_port_ptr));
         } // ctrl port loop
      }    // module list loop
   }
   CATCH(result, WCNTR_MSG_PREFIX, base_ptr->gu_ptr->log_id)
   {
   }
   return result;       // sg list loop
}

/**
  Handles the destruction of the queues and their buffers associated with
  internal control ports.

  If the graph management command is NULL then consider it as forced destroy and destroy
 * all the internal ports.
*/
void wcntr_deinit_internal_ctrl_ports(wcntr_base_t *base_ptr, spf_msg_cmd_graph_mgmt_t *cmd_gmgmt_ptr)
{
   INIT_EXCEPTION_HANDLING
   ar_result_t result = AR_EOK;
   spf_cntr_sub_graph_list_t *spf_sg_list_ptr = (NULL != cmd_gmgmt_ptr) ? &cmd_gmgmt_ptr->sg_id_list : NULL;

   // Iterate through all sgs and modules to get to the internal control ports
   for (wcntr_gu_sg_list_t *sg_list_ptr = base_ptr->gu_ptr->sg_list_ptr; (NULL != sg_list_ptr); LIST_ADVANCE(sg_list_ptr))
   {
      // If the command pointer is NULL, then consider it as forced destroy and mark found = TRUE.
      bool_t   found  = (NULL == cmd_gmgmt_ptr) ? TRUE : FALSE;
      wcntr_gu_sg_t *sg_ptr = sg_list_ptr->sg_ptr;
      // if it's not for this subgraph
      if (wcntr_gu_is_sg_id_found_in_spf_array(spf_sg_list_ptr, sg_ptr->id))
      {
         found = TRUE;
      }

      if (!found)
      {
         continue;
      }

      for (wcntr_gu_module_list_t *module_list_ptr = sg_list_ptr->sg_ptr->module_list_ptr; (NULL != module_list_ptr);
           LIST_ADVANCE(module_list_ptr))
      {
         wcntr_gu_module_t *module_ptr = module_list_ptr->module_ptr;
         
         for (wcntr_gu_ctrl_port_list_t *ctrl_port_list_ptr = module_ptr->ctrl_port_list_ptr; (NULL != ctrl_port_list_ptr);
              LIST_ADVANCE(ctrl_port_list_ptr))
         {
            wcntr_gu_ctrl_port_t *ctrl_port_ptr = ctrl_port_list_ptr->ctrl_port_ptr;
               // internal control port
               TRY(result, wcntr_deinit_internal_ctrl_port(base_ptr, ctrl_port_ptr));
               if(ctrl_port_ptr->peer_ctrl_port_ptr)
               {
                  TRY(result, wcntr_deinit_internal_ctrl_port(base_ptr, ctrl_port_ptr->peer_ctrl_port_ptr)); 
               }               
         } // ctrl port loop
      }    // module list loop
   }// sg list loop
   CATCH(result, WCNTR_MSG_PREFIX, base_ptr->gu_ptr->log_id)
   {
   }
}
ar_result_t wcntr_poll_and_process_int_ctrl_msgs(wcntr_base_t *me_ptr)
{
   ar_result_t result = AR_EOK;

   // Poll for set channel_bits
   uint32_t set_ch_bitmask =
      posal_channel_poll(me_ptr->ctrl_channel_ptr, posal_queue_get_channel_bit(me_ptr->cntr_cmn_imcl_handle.q_ptr));

   if (!set_ch_bitmask)
   {
      return AR_EOK;
   }

   spf_msg_t incoming_ctrl_msg; // Current incoming ctrl q msg, being handled
#ifdef CTRL_LINK_DEBUG
   WCNTR_MSG(me_ptr->gu_ptr->log_id, DBG_HIGH_PRIO, "Internal Control message channel bit_mask: 0x%lx ", set_ch_bitmask);
#endif // CTRL_LINK_DEBUG
   uint32_t MAX_LOOP_COUNT = 50;
   for (uint32_t i = 0;
        posal_channel_poll(me_ptr->ctrl_channel_ptr, posal_queue_get_channel_bit(me_ptr->cntr_cmn_imcl_handle.q_ptr)) &&
        (i < MAX_LOOP_COUNT);
        i++)
   {
      // Pop the messages from the queue.
      result = posal_queue_pop_front(me_ptr->cntr_cmn_imcl_handle.q_ptr, (posal_queue_element_t *)&(incoming_ctrl_msg));
      if (AR_ENEEDMORE == result)
      {
         incoming_ctrl_msg.payload_ptr = NULL;
         // queue is empty
         break;
      }
      if (AR_EOK != result)
      {
         incoming_ctrl_msg.payload_ptr = NULL;
         WCNTR_MSG(me_ptr->gu_ptr->log_id, DBG_ERROR_PRIO, "Couldn't pop incoming ctrl message from the queue.");
         return result;
      }

      // Check if we are getting data buffer.
      if (SPF_MSG_CTRL_PORT_POLLING_MSG != incoming_ctrl_msg.msg_opcode)
      {
         WCNTR_MSG(me_ptr->gu_ptr->log_id, DBG_ERROR_PRIO, "Unexpected msg op code received.");
         return AR_EUNEXPECTED;
      }
#ifdef CTRL_LINK_DEBUG
      WCNTR_MSG(me_ptr->gu_ptr->log_id, DBG_LOW_PRIO, "Polled and got same container IMCL");
#endif // CTRL_LINK_DEBUG

      // Extract the data buffer from the ctrl msg.
      spf_msg_header_t *       header_ptr   = (spf_msg_header_t *)incoming_ctrl_msg.payload_ptr;
      spf_msg_ctrl_port_msg_t *ctrl_msg_ptr = (spf_msg_ctrl_port_msg_t *)&header_ptr->payload_start[0];

      // now, we need to route this message correctly to the destination peer
      wcntr_gu_ctrl_port_t *ctrl_port_ptr = (wcntr_gu_ctrl_port_t *)ctrl_msg_ptr->dst_intra_cntr_port_hdl;
      if (ctrl_port_ptr)
      {
//#ifdef CTRL_LINK_DEBUG
         WCNTR_MSG(me_ptr->gu_ptr->log_id,
                DBG_HIGH_PRIO,
                "wcntr_poll_and_process_int_ctrl_msgs(): incoming intent msg  0x%p, miid 0x%lx port_id: 0x%lx, msg_token: "
                "%lu ",
                incoming_ctrl_msg.payload_ptr,
                ctrl_port_ptr->module_ptr->module_instance_id,
                ctrl_port_ptr->id,
                header_ptr->token.token_data);

         WCNTR_MSG(me_ptr->gu_ptr->log_id,
                DBG_HIGH_PRIO,
                "cu_poll_and_process_int_ctrl_msgs(): module intent buffer  0x%p, max_size:: %lu ",
                &ctrl_msg_ptr->data_buf[0],
                ctrl_msg_ptr->max_size);
//#endif // CTRL_LINK_DEBUG

         // Send the incoming intent to the capi module.
      result =  wcntr_topo_handle_incoming_ctrl_intent(ctrl_port_ptr,
                                                                     &ctrl_msg_ptr->data_buf[0],
                                                                     ctrl_msg_ptr->max_size,
                                                                     ctrl_msg_ptr->actual_size); 
      }

      // return the buffer
      result |= spf_msg_return_msg(&incoming_ctrl_msg);
   }
   return result;
}

ar_result_t wcntr_prep_and_send_outgoing_ctrl_msg(wcntr_base_t *               me_ptr,
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
      WCNTR_MSG(me_ptr->gu_ptr->log_id,
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

#if defined (VERBOSE_LOGGING) || defined(CTRL_LINK_DEBUG)
   WCNTR_MSG(me_ptr->gu_ptr->log_id,
          DBG_MED_PRIO,
          " Sending ctrl msg is_recurring= %lu, max_size= %lu, actual_size= %lu",
          is_recurring,
          ctrl_buf_ptr->max_size,
          buf_ptr->actual_data_len);
#endif

   if (flags.is_trigger)
   {
      result = wcntr_send_ctrl_trigger_spf_msg(&msg, dest_handle_ptr);
   }
   else
   {
      result = wcntr_send_ctrl_polling_spf_msg(&msg, dest_handle_ptr);
   }

   if (AR_DID_FAIL(result))
   {
      WCNTR_MSG(me_ptr->gu_ptr->log_id,
             DBG_ERROR_PRIO,
             " Failed to push the buffer to peer control port queue %d",
             result);
      spf_msg_return_msg(&msg);
   }
   return result;
}

ar_result_t wcntr_alloc_populate_one_time_ctrl_msg(wcntr_base_t *   me_ptr,
                                                capi_buf_t *  buf_ptr,
                                                bool_t        is_intra_cntr_msg,
                                                void *        self_port_hdl,
                                                void *        peer_port_hdl,
                                                POSAL_HEAP_ID peer_heap_id,
                                                spf_msg_t *   one_time_msg_ptr,
                                                uint32_t      token,
                                                spf_handle_t *dst_handle_ptr)
{
   uint32_t out_buf_size = buf_ptr->actual_data_len + sizeof(intf_extn_param_id_imcl_incoming_data_t);

   // Update the required for the data ptr and msg header.
   uint32_t req_size = GET_SPF_INLINE_CTRLBUF_REQ_SIZE(out_buf_size);

   {
      // Get onetime msg from the buffer manager.
      if (AR_DID_FAIL(spf_msg_create_msg(one_time_msg_ptr,
                                         &req_size,
                                         SPF_MSG_CMD_CTRL_PORT_TRIGGER_MSG, // opcode is updated based on trigger mode
                                                                            // when sending.
                                         NULL,
                                         NULL,
                                         dst_handle_ptr,
                                         peer_heap_id)))
      {
         WCNTR_MSG(me_ptr->gu_ptr->log_id, DBG_ERROR_PRIO, "Fail to create one time intent buffer message");
         return AR_EFAILED;
      }
   }

   // Reinterpret node's buffer as a header.
   spf_msg_header_t *header_ptr = (spf_msg_header_t *)one_time_msg_ptr->payload_ptr;
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




										   
