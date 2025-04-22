/**
 * \file spf_msg_utils_island.c
 * \brief
 *     This file contains the implementation for spf
 *  message utility functions.
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/* =======================================================================
INCLUDE FILES FOR MODULE
========================================================================== */
#include "spf_utils.h"

#include "ar_msg.h"

/* =======================================================================
**                          Function Definitions
** ======================================================================= */

ar_result_t spf_msg_return_msg(spf_msg_t *msg_ptr)
{
  if(spf_lpi_pool_is_addr_from_md_pool(msg_ptr->payload_ptr))
  {
      spf_lpi_pool_return_node(msg_ptr->payload_ptr);
      return AR_EOK;
  }
   // Get the return queue from the message header.
   spf_msg_header_t *header_ptr   = (spf_msg_header_t *)msg_ptr->payload_ptr;
   posal_queue_t *   return_q_ptr = NULL;

   if ((NULL == header_ptr) || (NULL == header_ptr->return_q_ptr))
   {
      AR_MSG_ISLAND(DBG_ERROR_PRIO,
             "spf_msg_return_msg() failed: the message's payload or return queue is "
             "NULL. Message opcode: 0x%lx",
             msg_ptr->msg_opcode);
      return AR_EFAILED;
   }

   return_q_ptr         = header_ptr->return_q_ptr;
   msg_ptr->payload_ptr = NULL;
   msg_ptr->msg_opcode  = 0;

   // Check if the buffer is from buffer manager.
   if (spf_is_bufmgr_node(header_ptr))
   {
      // Return to buffer manager.
      return spf_bufmgr_return_buf(header_ptr);
   }
   else
   {
      // Manually push to the back of the return handle.
      posal_bufmgr_node_t buf_mgr_node;
      buf_mgr_node.return_q_ptr = return_q_ptr;
      buf_mgr_node.buf_ptr      = header_ptr;

      return posal_queue_push_back(return_q_ptr, (posal_queue_element_t *)(&buf_mgr_node));
   }
}

ar_result_t spf_msg_ack_msg(spf_msg_t *msg_ptr, uint32_t rsp_result)
{
   // Send response if response handle is available. Otherwise, return the
   // payload to return queue.
   spf_msg_header_t *header_ptr = (spf_msg_header_t *)msg_ptr->payload_ptr;

   if (header_ptr->rsp_handle_ptr)
   {
      // Populate the response result.
      header_ptr->rsp_result = rsp_result;

      // Return the message buffer to its response handle.
      return spf_msg_send_response(msg_ptr);
   }

   // Return the message buffer to its return queue.
   return spf_msg_return_msg(msg_ptr);
}

spf_msg_t *spf_msg_convt_buf_node_to_msg(posal_bufmgr_node_t *node_ptr,
                                         uint32_t             opcode,
                                         spf_handle_t *       resp_handle_ptr,
                                         spf_msg_token_t *    msg_token_ptr,
                                         uint32_t             rsp_result,
                                         spf_handle_t *       dst_handle_ptr)
{
   if (!node_ptr)
   {
      AR_MSG_ISLAND(DBG_ERROR_PRIO, "spf_msg_convt_buf_node_to_msg() failed: node_ptr is NULL.");
      return NULL;
   }

   // Reinterpret node's buffer as a header.
   spf_msg_header_t *header_ptr = (spf_msg_header_t *)node_ptr->buf_ptr;
   header_ptr->rsp_handle_ptr   = resp_handle_ptr;
   header_ptr->token            = (NULL != msg_token_ptr) ? *msg_token_ptr : (spf_msg_token_t){ 0 };
   header_ptr->return_q_ptr     = node_ptr->return_q_ptr;
   header_ptr->rsp_result       = rsp_result;
   header_ptr->dst_handle_ptr   = dst_handle_ptr;

   // Reinterpret the node itself as a spf_msg_t.
   spf_msg_t *msg_ptr   = (spf_msg_t *)node_ptr;
   msg_ptr->msg_opcode  = opcode;
   msg_ptr->payload_ptr = header_ptr;

   return msg_ptr;
}
ar_result_t spf_msg_send_cmd(spf_msg_t *msg_ptr, spf_handle_t *dst_handle_ptr)
{
   ar_result_t result = AR_EOK;
   if (NULL == dst_handle_ptr->cmd_handle_ptr)
   {
      AR_MSG_ISLAND(DBG_ERROR_PRIO,
             "spf_msg_send_cmd() failed: command handle is NULL. Message "
             "opcode: 0x%lx",
             msg_ptr->msg_opcode);
      return AR_EFAILED;
   }

   posal_queue_t *cmd_q_ptr = dst_handle_ptr->cmd_handle_ptr->cmd_q_ptr;
   if (NULL == cmd_q_ptr)
   {
      AR_MSG_ISLAND(DBG_ERROR_PRIO,
             "spf_msg_send_cmd() failed: command queue is NULL. Message opcode: "
             " 0x%lx",
             msg_ptr->msg_opcode);
      return AR_EFAILED;
   }

   result = posal_queue_push_back(cmd_q_ptr, (posal_queue_element_t *)msg_ptr);

   if (AR_DID_FAIL(result))
   {
      AR_MSG_ISLAND(DBG_ERROR_PRIO,
             "spf_msg_send_cmd() failed: failed to push to command queue. "
             "Message opcode: 0x%lx",
             msg_ptr->msg_opcode);
      return AR_EFAILED;
   }
   return AR_EOK;
}

ar_result_t spf_msg_create_msg(spf_msg_t *      msg_ptr,
                               uint32_t *       buf_size_ptr,
                               uint32_t         msg_opcode,
                               spf_handle_t *   rsp_handle_ptr,
                               spf_msg_token_t *msg_token_ptr,
                               spf_handle_t *   dst_handle_ptr,
                               POSAL_HEAP_ID    heap_id)
{
   ar_result_t         result;
   uint32_t            actual_size;
   posal_bufmgr_node_t buf_node;

   // Get memory for the message header and payload from the buffer manager.
   if (AR_DID_FAIL(result = spf_bufmgr_poll_for_buffer(*buf_size_ptr, &buf_node, &actual_size, heap_id)))
   {
      AR_MSG_ISLAND(DBG_ERROR_PRIO, "spf_msg_create_msg() failed: Failed to get message buffer.");
      return result;
   }

   // Fill the header.
   spf_msg_header_t *header_ptr = (spf_msg_header_t *)buf_node.buf_ptr;

   msg_ptr->msg_opcode        = msg_opcode;
   header_ptr->return_q_ptr   = buf_node.return_q_ptr;
   header_ptr->rsp_handle_ptr = rsp_handle_ptr;
   header_ptr->token          = (NULL != msg_token_ptr) ? *msg_token_ptr : (spf_msg_token_t){ 0 };
   header_ptr->dst_handle_ptr = dst_handle_ptr;
   header_ptr->payload_size   = *buf_size_ptr;

   // Zero-fill the unset fields.
   header_ptr->rsp_result       = 0;
   header_ptr->payload_start[0] = 0;

   // Fill the message.
   msg_ptr->payload_ptr = header_ptr;

   return AR_EOK;
}
