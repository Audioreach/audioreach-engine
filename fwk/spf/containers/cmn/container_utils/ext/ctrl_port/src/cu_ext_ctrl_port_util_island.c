/**
 * \file cu_ext_ctrl_port_util_island.c
 * \brief
 *     This file contains container utility functions for external control port handling in island.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "cu_i.h"
#include "cu_ctrl_port_i.h"
#include "capi_intf_extn_imcl.h"

/* Poll and process the ctrl port messages in the queue.*/
ar_result_t cu_ext_ctrl_port_poll_and_process_ctrl_msgs(cu_base_t *me_ptr,  gu_ext_ctrl_port_t *gu_ext_ctrl_port_ptr)
{
   ar_result_t result = AR_EOK;

   uint32_t MAX_LOOP_COUNT = 50;

   for (uint32_t i = 0; posal_channel_poll(me_ptr->ctrl_channel_ptr,
                                           posal_queue_get_channel_bit(gu_ext_ctrl_port_ptr->this_handle.q_ptr)) &&
                        (i < MAX_LOOP_COUNT);
        i++)
   {
      // Pop the messages from the queue.
      spf_msg_t incoming_ctrl_msg = { 0 }; // Current incoming ctrl q msg, being handled.
      result =
         posal_queue_pop_front(gu_ext_ctrl_port_ptr->this_handle.q_ptr, (posal_queue_element_t *)&(incoming_ctrl_msg));
      if (AR_EOK != result)
      {
         incoming_ctrl_msg.payload_ptr = NULL;
         CU_MSG_ISLAND(me_ptr->gu_ptr->log_id, DBG_ERROR_PRIO, "Couldn't pop incoming ctrl message from the queue.");
         return result;
      }
      // Check if it is inter-proc IMCL
      if (SPF_MSG_CMD_GPR == incoming_ctrl_msg.msg_opcode)
      {
         CU_MSG_ISLAND(me_ptr->gu_ptr->log_id, DBG_HIGH_PRIO, "MDF: IMCL Polled and got inter proc IMCL message");
         /** Get the pointer to GPR command */
         gpr_packet_t *pkt_ptr      = (gpr_packet_t *)incoming_ctrl_msg.payload_ptr;
         void *        payload_ptr  = GPR_PKT_GET_PAYLOAD(void, pkt_ptr);
         uint32_t      payload_size = GPR_PKT_GET_PAYLOAD_BYTE_SIZE(pkt_ptr->header);

         if (IMCL_INTER_PROC_POLLING_MSG_GPR != pkt_ptr->opcode)
         {
            CU_MSG_ISLAND(me_ptr->gu_ptr->log_id,
                          DBG_ERROR_PRIO,
                          "IMCL: MDF: Unexpected packet opcode 0x%lx received.",
                          pkt_ptr->opcode);
            return AR_EUNEXPECTED;
         }
         // this loop iteration has an interproc imcl message

         // Send the incoming intent to the capi module.
         result = me_ptr->topo_island_vtbl_ptr->handle_incoming_ctrl_intent(gu_ext_ctrl_port_ptr->int_ctrl_port_ptr,
                                                                           payload_ptr,
                                                                           payload_size,
                                                                           payload_size);

         __gpr_cmd_end_command(pkt_ptr, AR_EOK);
      }
      else
      {
#ifdef CTRL_LINK_DEBUG
         CU_MSG_ISLAND(me_ptr->gu_ptr->log_id, DBG_LOW_PRIO, "Polled and got same proc IMCL");
#endif // CTRL_LINK_DEBUG

         // Check if we are getting data buffer.
         if (SPF_MSG_CTRL_PORT_POLLING_MSG != incoming_ctrl_msg.msg_opcode)
         {
            CU_MSG_ISLAND(me_ptr->gu_ptr->log_id, DBG_ERROR_PRIO, "Unexpected msg op code received.");
            return AR_EUNEXPECTED;
         }

         // Extract the data buffer from the ctrl msg.
         spf_msg_header_t *       header_ptr   = (spf_msg_header_t *)incoming_ctrl_msg.payload_ptr;
         spf_msg_ctrl_port_msg_t *ctrl_msg_ptr = (spf_msg_ctrl_port_msg_t *)&header_ptr->payload_start[0];

#ifdef CTRL_LINK_DEBUG
         CU_MSG_ISLAND(me_ptr->gu_ptr->log_id,
                       DBG_HIGH_PRIO,
                       "cu_poll_and_process_ctrl_msgs(): incoming intent msg  0x%p, port_id: 0x%lx, msg_token: %lu ",
                       incoming_ctrl_msg.payload_ptr,
                       gu_ext_ctrl_port_ptr->int_ctrl_port_ptr->id,
                       header_ptr->token.token_data);

         CU_MSG_ISLAND(me_ptr->gu_ptr->log_id,
                       DBG_HIGH_PRIO,
                       "cu_poll_and_process_ctrl_msgs(): module intent buffer  0x%p, max_size:: %lu ",
                       &ctrl_msg_ptr->data_buf[0],
                       ctrl_msg_ptr->max_size);
#endif // CTRL_LINK_DEBUG

        // Send the incoming intent to the capi module.
        result = me_ptr->topo_island_vtbl_ptr->handle_incoming_ctrl_intent(gu_ext_ctrl_port_ptr->int_ctrl_port_ptr,
                                                                          &ctrl_msg_ptr->data_buf[0],
                                                                          ctrl_msg_ptr->max_size,
                                                                          ctrl_msg_ptr->actual_size);

         // return the buffer
         result |= spf_msg_return_msg(&incoming_ctrl_msg);
      }
   }
   return result;
}

// never call this func directly. call cu_poll_and_process_ctrl_msgs first.
ar_result_t cu_poll_and_process_ext_ctrl_msgs_util_(cu_base_t *me_ptr, uint32_t set_ch_bitmask)
{
   ar_result_t result = AR_EOK;

   if (set_ch_bitmask & (~me_ptr->island_ctrl_chan_mask))
   {
      // channel bit is set for the control ports operating in non-island. exit island
      if (me_ptr->cntr_vtbl_ptr->exit_island_temporarily)
      {
         me_ptr->cntr_vtbl_ptr->exit_island_temporarily((void *)me_ptr);
      }
   }

#ifdef CTRL_LINK_DEBUG
   CU_MSG_ISLAND(me_ptr->gu_ptr->log_id, DBG_HIGH_PRIO, "Control message channel bit_mask: 0x%lx ", set_ch_bitmask);
#endif // CTRL_LINK_DEBUG

   for (gu_ext_ctrl_port_list_t *ext_ctrl_port_list_ptr = me_ptr->gu_ptr->ext_ctrl_port_list_ptr;
        (NULL != ext_ctrl_port_list_ptr);
        LIST_ADVANCE(ext_ctrl_port_list_ptr))
   {
      gu_ext_ctrl_port_t *gu_ext_ctrl_port_ptr = ext_ctrl_port_list_ptr->ext_ctrl_port_ptr;

#ifdef CTRL_LINK_DEBUG
      CU_MSG_ISLAND(me_ptr->gu_ptr->log_id,
                    DBG_HIGH_PRIO,
                    "Control Q bit_mask: 0x%lx ",
                    posal_queue_get_channel_bit(gu_ext_ctrl_port_ptr->this_handle.q_ptr));
#endif // CTRL_LINK_DEBUG

      // Check if signal is set on the current ch bit.
      if (set_ch_bitmask & (posal_queue_get_channel_bit(gu_ext_ctrl_port_ptr->this_handle.q_ptr)))
      {
         // Clear the bit for this queue.
         set_ch_bitmask = set_ch_bitmask & (~(posal_queue_get_channel_bit(gu_ext_ctrl_port_ptr->this_handle.q_ptr)));

         result |= cu_ext_ctrl_port_poll_and_process_ctrl_msgs(me_ptr, gu_ext_ctrl_port_ptr);
      }
   }

   return result;
}
