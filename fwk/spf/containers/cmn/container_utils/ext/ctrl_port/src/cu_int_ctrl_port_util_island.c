/**
 * \file cu_ext_ctrl_port_util_island.c
 * \brief
 *     This file contains container utility functions for internal control port message handling
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "cu_i.h"
#include "cu_ctrl_port_i.h"
#include "capi_intf_extn_imcl.h"

ar_result_t cu_poll_and_process_int_ctrl_msgs_util_(cu_base_t *me_ptr, uint32_t set_ch_bitmask)
{
   ar_result_t result = AR_EOK;

   spf_msg_t incoming_ctrl_msg = {0}; // Current incoming ctrl q msg, being handled
#ifdef CTRL_LINK_DEBUG
   CU_MSG_ISLAND(me_ptr->gu_ptr->log_id, DBG_HIGH_PRIO, "Internal Control message channel bit_mask: 0x%lx ", set_ch_bitmask);
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
         CU_MSG_ISLAND(me_ptr->gu_ptr->log_id, DBG_ERROR_PRIO, "Couldn't pop incoming ctrl message from the queue.");
         return result;
      }

      // Check if we are getting data buffer.
      if (SPF_MSG_CTRL_PORT_POLLING_MSG != incoming_ctrl_msg.msg_opcode)
      {
         CU_MSG_ISLAND(me_ptr->gu_ptr->log_id, DBG_ERROR_PRIO, "Unexpected msg op code received.");
         return AR_EUNEXPECTED;
      }
#ifdef CTRL_LINK_DEBUG
      CU_MSG_ISLAND(me_ptr->gu_ptr->log_id, DBG_LOW_PRIO, "Polled and got same container IMCL");
#endif // CTRL_LINK_DEBUG

      //exit island if message is in non-island
      if (posal_island_get_island_status() &&
          !posal_check_addr_from_tcm_island_heap_mgr(incoming_ctrl_msg.payload_ptr) &&
          me_ptr->cntr_vtbl_ptr->exit_island_temporarily)
      {
         me_ptr->cntr_vtbl_ptr->exit_island_temporarily((void*)me_ptr);
      }

      // Extract the data buffer from the ctrl msg.
      spf_msg_header_t *       header_ptr   = (spf_msg_header_t *)incoming_ctrl_msg.payload_ptr;
      spf_msg_ctrl_port_msg_t *ctrl_msg_ptr = (spf_msg_ctrl_port_msg_t *)&header_ptr->payload_start[0];

      // now, we need to route this message correctly to the destination peer
      gu_ctrl_port_t *ctrl_port_ptr = (gu_ctrl_port_t *)ctrl_msg_ptr->dst_intra_cntr_port_hdl;
      if (ctrl_port_ptr)
      {
#ifdef CTRL_LINK_DEBUG
         CU_MSG_ISLAND(me_ptr->gu_ptr->log_id,
                DBG_HIGH_PRIO,
                "cu_poll_and_process_int_ctrl_msgs(): incoming intent msg  0x%p, miid 0x%lx port_id: 0x%lx, msg_token: "
                "%lu ",
                incoming_ctrl_msg.payload_ptr,
                ctrl_port_ptr->module_ptr->module_instance_id,
                ctrl_port_ptr->id,
                header_ptr->token.token_data);

         CU_MSG_ISLAND(me_ptr->gu_ptr->log_id,
                DBG_HIGH_PRIO,
                "cu_poll_and_process_int_ctrl_msgs(): module intent buffer  0x%p, max_size:: %lu ",
                &ctrl_msg_ptr->data_buf[0],
                ctrl_msg_ptr->max_size);
#endif // CTRL_LINK_DEBUG

         // Send the incoming intent to the capi module.
         result = me_ptr->topo_island_vtbl_ptr->handle_incoming_ctrl_intent(ctrl_port_ptr,
                                                                           &ctrl_msg_ptr->data_buf[0],
                                                                           ctrl_msg_ptr->max_size,
                                                                           ctrl_msg_ptr->actual_size);
      }

      // return the buffer
      result |= spf_msg_return_msg(&incoming_ctrl_msg);
   }
   return result;
}
