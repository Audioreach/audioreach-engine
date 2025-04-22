/**
 * \file cu_int_ctrl_port_util.c
 * \brief
 *     This file contains container utility functions for external control port handling (input and output).
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "cu_i.h"
#include "cu_ctrl_port_i.h"
#include "capi_intf_extn_imcl.h"
/* =======================================================================
Static Function Definitions
========================================================================== */

/* ==========================================================================  */

/*
 * Create a control port control queue common for all the intra-container control links.
 */
ar_result_t cu_create_cmn_int_ctrl_port_queue(cu_base_t *cu_ptr, uint32_t ctrl_port_queue_offset)
{
   ar_result_t result   = AR_EOK;
   uint32_t    bit_mask = 0;

   char incoming_intent_q_name[POSAL_DEFAULT_NAME_LEN]; // data queue name
   snprintf(incoming_intent_q_name, POSAL_DEFAULT_NAME_LEN, "%s%s%8lX", "INT_IMCL_Q", "CNTR", cu_ptr->gu_ptr->log_id);

   uint32_t num_elements = CU_MAX_INTENT_Q_ELEMENTS;

   bit_mask = cu_request_bit_in_bit_mask(&cu_ptr->available_ctrl_chan_mask);
   if (0 == bit_mask)
   {
      CU_MSG(cu_ptr->gu_ptr->log_id,
             DBG_ERROR_PRIO,
             "Bit mask has no bits available 0x%lx",
             cu_ptr->available_bit_mask);
      return result;
   }

   CU_MSG(cu_ptr->gu_ptr->log_id,
          DBG_HIGH_PRIO,
          "Container's Internal IMCL control Q will be created with max incoming intent elements = %lu, bit_mask: "
          "0x%lx heap_id: 0x%lx",
          num_elements,
          bit_mask,
          cu_ptr->heap_id);

   cu_ptr->cntr_cmn_imcl_handle.cmd_handle_ptr = &cu_ptr->cmd_handle;

   // Create the queue
   result = cu_init_queue(cu_ptr,
                            incoming_intent_q_name,
                            num_elements,
                            bit_mask,
                            NULL, /*queue handler*/
                            cu_ptr->ctrl_channel_ptr,
                            &cu_ptr->cntr_cmn_imcl_handle.q_ptr,
                            CU_PTR_PUT_OFFSET(cu_ptr, ctrl_port_queue_offset),
                            cu_ptr->heap_id);

   // if this queue is created in island heap then update the island channel mask.
   if (POSAL_IS_ISLAND_HEAP_ID(cu_ptr->heap_id))
   {
      cu_ptr->island_ctrl_chan_mask |= bit_mask;
   }

   return result;
}

ar_result_t cu_destroy_cmn_int_port_queue(cu_base_t *cu_ptr)
{
   ar_result_t result = AR_EOK;
   if (cu_ptr->cntr_cmn_imcl_handle.q_ptr)
   {
      uint32_t bitmask = posal_queue_get_channel_bit(cu_ptr->cntr_cmn_imcl_handle.q_ptr);
      CU_MSG(cu_ptr->gu_ptr->log_id,
             DBG_HIGH_PRIO,
             "Container's Internal IMCL control Q will be deinited bit_mask: "
             "0x%lx",
             bitmask);
      /*Release the available control channel mask mask */
      cu_ptr->available_ctrl_chan_mask |= bitmask;

      //clear the bit from island channel mask
      cu_ptr->island_ctrl_chan_mask &= (~cu_ptr->available_ctrl_chan_mask);

      /*deinit the queue */
      posal_queue_deinit(cu_ptr->cntr_cmn_imcl_handle.q_ptr);
      cu_ptr->cntr_cmn_imcl_handle.q_ptr = NULL;
   }
   return result;
}

/*
 *Go over the messages in the queue, check the source,
 *    if it belongs to the ctrl port that wants to retrieve,
 *    return to Q
 */
ar_result_t cu_check_and_retrieve_bufq_msgs_from_cmn_ctrlq(cu_base_t *me_ptr, gu_ctrl_port_t *gu_ctrl_port_ptr)
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
         CU_MSG(me_ptr->gu_ptr->log_id, DBG_ERROR_PRIO, "Couldn't pop incoming ctrl message from the queue.");
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
         CU_MSG(me_ptr->gu_ptr->log_id,
                DBG_MED_PRIO,
                "Check and retrieve: Returned buffer from ctrl q to bufq of port id 0x%lx",
                gu_ctrl_port_ptr->id);
#endif
      }
      else
      {
         // push back to the queue
         cu_send_ctrl_polling_spf_msg(&temp_msg, &me_ptr->cntr_cmn_imcl_handle);
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
ar_result_t cu_deinit_internal_ctrl_port(cu_base_t *me_ptr, gu_ctrl_port_t *gu_ctrl_port_ptr, bool_t b_skip_q_flush)
{
   ar_result_t result = AR_EOK;

   if (!b_skip_q_flush)
   {
      result = cu_check_and_retrieve_bufq_msgs_from_cmn_ctrlq(me_ptr, gu_ctrl_port_ptr);
   }
   // Destroy the output buffer Q
   cu_deinit_ctrl_port_buf_q(me_ptr, gu_ctrl_port_ptr);

   gu_ctrl_port_ptr->peer_ctrl_port_ptr = NULL;

   return result;
}

ar_result_t cu_operate_on_int_ctrl_port(cu_base_t *      me_ptr,
                                        uint32_t         sg_ops,
                                        gu_ctrl_port_t **gu_ctrl_port_ptr_ptr,
                                        bool_t           is_self_sg)
{
   gu_ctrl_port_t *    gu_ctrl_port_ptr = *gu_ctrl_port_ptr_ptr;
   ar_result_t result = AR_EOK;
   if (is_self_sg)
   {
      if (((TOPO_SG_OP_STOP | TOPO_SG_OP_CLOSE) & sg_ops))
      {
         // FLush incoming ctrl msg queue.
         CU_MSG(me_ptr->gu_ptr->log_id,
                DBG_MED_PRIO,
                "Operate on in ctrl port 0x%lx, retrieving bufq msgs from ctrl q",
                gu_ctrl_port_ptr->id);

         result = cu_check_and_retrieve_bufq_msgs_from_cmn_ctrlq(me_ptr, gu_ctrl_port_ptr);
      }
      if (TOPO_SG_OP_PREPARE & sg_ops)
      {
         result = cu_check_and_recreate_ctrl_port_buffers(me_ptr, gu_ctrl_port_ptr);

         /*If it is an internal control port part of different SG then check if buf-q needs to be created on that as
          * well.*/
         if (AR_SUCCEEDED(result) && gu_ctrl_port_ptr->peer_ctrl_port_ptr)
         {
            result = cu_check_and_recreate_ctrl_port_buffers(me_ptr, gu_ctrl_port_ptr->peer_ctrl_port_ptr);
         }
      }
   }
   return result;
}

/**
  Handles the destruction of the queues and their buffers associated with
  internal control ports.
*/
void cu_deinit_internal_ctrl_ports(cu_base_t *base_ptr, bool_t b_destroy_all_ports)
{
   INIT_EXCEPTION_HANDLING
   ar_result_t result = AR_EOK;

   gu_t *gu_ptr = get_gu_ptr_for_current_command_context(base_ptr->gu_ptr);

   // Iterate through all sgs and modules to get to the internal control ports
   for (gu_sg_list_t *sg_list_ptr = gu_ptr->sg_list_ptr; (NULL != sg_list_ptr); LIST_ADVANCE(sg_list_ptr))
   {
      if (!(b_destroy_all_ports || (GU_STATUS_CLOSING == sg_list_ptr->sg_ptr->gu_status)))
      {
         continue;
      }

      for (gu_module_list_t *module_list_ptr = sg_list_ptr->sg_ptr->module_list_ptr; (NULL != module_list_ptr);
           LIST_ADVANCE(module_list_ptr))
      {
         gu_module_t *module_ptr = module_list_ptr->module_ptr;

         for (gu_ctrl_port_list_t *ctrl_port_list_ptr = module_ptr->ctrl_port_list_ptr; (NULL != ctrl_port_list_ptr);
              LIST_ADVANCE(ctrl_port_list_ptr))
         {
            gu_ctrl_port_t *ctrl_port_ptr = ctrl_port_list_ptr->ctrl_port_ptr;
            if (NULL != ctrl_port_ptr->peer_ctrl_port_ptr)
            {
               gu_ctrl_port_t *peer_ctrl_port_ptr = ctrl_port_ptr->peer_ctrl_port_ptr;

               // internal control port
               TRY(result, cu_deinit_internal_ctrl_port(base_ptr, ctrl_port_ptr, FALSE));

               // don't need to flush the intent Q, it is already flushed at this point.
               TRY(result, cu_deinit_internal_ctrl_port(base_ptr, peer_ctrl_port_ptr, TRUE));
            }
         } // ctrl port loop
      }    // module list loop
   }       // sg list loop
   CATCH(result, CU_MSG_PREFIX, base_ptr->gu_ptr->log_id)
   {
   }
}
