/**
 * \file spf_msg_utils.c
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

ar_result_t spf_msg_init_handle(spf_handle_t *handle_ptr, spf_cmd_handle_t *cmd_handle_ptr, posal_queue_t *q_ptr)
{
   handle_ptr->cmd_handle_ptr = cmd_handle_ptr;
   handle_ptr->q_ptr          = q_ptr;
   return AR_EOK;
}

ar_result_t spf_msg_init_cmd_handle(spf_cmd_handle_t *handle_ptr,
                                    posal_thread_t    thread_id,
                                    posal_queue_t *   cmd_q_ptr,
                                    posal_queue_t *   sys_cmd_q_ptr)
{
   handle_ptr->thread_id     = thread_id;
   handle_ptr->cmd_q_ptr     = cmd_q_ptr;
   handle_ptr->sys_cmd_q_ptr = sys_cmd_q_ptr;
   return AR_EOK;
}

ar_result_t spf_msg_send_sys_cmd(spf_msg_t *msg_ptr, spf_handle_t *dst_handle_ptr)
{
   ar_result_t result = AR_EOK;
   if (NULL == dst_handle_ptr->cmd_handle_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "spf_msg_send_sys_cmd() failed: system command handle is NULL. Message opcode: 0x%lx",
             msg_ptr->msg_opcode);
      return AR_EFAILED;
   }

   posal_queue_t *sys_cmd_q_ptr = dst_handle_ptr->cmd_handle_ptr->sys_cmd_q_ptr;
   if (NULL == sys_cmd_q_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "spf_msg_send_sys_cmd() failed: system command queue is NULL. Message opcode: 0x%lx",
             msg_ptr->msg_opcode);
      return AR_EFAILED;
   }

   result = posal_queue_push_back(sys_cmd_q_ptr, (posal_queue_element_t *)msg_ptr);

   if (AR_DID_FAIL(result))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "spf_msg_send_sys_cmd() failed: failed to push to system command queue. Message opcode: 0x%lx",
             msg_ptr->msg_opcode);
      return AR_EFAILED;
   }
   return AR_EOK;
}


ar_result_t spf_msg_send_response(spf_msg_t *msg_ptr)
{
   // Get the response handle from the message header.
   spf_msg_header_t *header_ptr          = (spf_msg_header_t *)msg_ptr->payload_ptr;
   spf_handle_t *    response_handle_ptr = header_ptr->rsp_handle_ptr;
   if (NULL == response_handle_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "spf_msg_send_response() failed: the message's response handle is "
             "NULL.");
      return AR_EFAILED;
   }

   posal_queue_t *rsp_q_ptr = response_handle_ptr->q_ptr;
   if (NULL == rsp_q_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "spf_msg_send_response() failed: the response handle's queue is "
             "NULL. Message opcode: 0x%lx",
             msg_ptr->msg_opcode);
      return AR_EFAILED;
   }

   ar_result_t result = AR_EOK;
   result             = posal_queue_push_back(rsp_q_ptr, (posal_queue_element_t *)msg_ptr);
   if (AR_DID_FAIL(result))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "spf_msg_send_response() failed: failed to push to the response "
             "queue. Message opcode: 0x%lx",
             msg_ptr->msg_opcode);
      return AR_EFAILED;
   }

   msg_ptr->payload_ptr = NULL;
   msg_ptr->msg_opcode  = 0;

   return AR_EOK;
}

ar_result_t spf_msg_wait_for_ack(spf_msg_t *msg_ptr, spf_msg_t *rsp_ptr)
{
   ar_result_t       result;
   spf_msg_header_t *rsp_header_ptr;
   spf_msg_t         rsp;

   // Store the response on the stack if a NULL rsp_ptr is passed in.
   if (NULL == rsp_ptr)
   {
      rsp_ptr = &rsp;
   }

   spf_msg_header_t *header_ptr     = (spf_msg_header_t *)msg_ptr->payload_ptr;
   spf_handle_t *    rsp_handle_ptr = header_ptr->rsp_handle_ptr;

   if (NULL == rsp_handle_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "spf_msg_wait_for_ack() failed: waiting for ACK on NULL Queue. "
             "message opcode: 0x%lx",
             msg_ptr->msg_opcode);

      return AR_EBADPARAM;
   }

   // Listen to the response queue.
   posal_queue_t *rsp_q_ptr = rsp_handle_ptr->q_ptr;
   posal_channel_wait(posal_queue_get_channel(rsp_q_ptr), posal_queue_get_channel_bit(rsp_q_ptr));

   result = posal_queue_pop_front(rsp_q_ptr, (posal_queue_element_t *)rsp_ptr);
   if (AR_DID_FAIL(result))
   {
      AR_MSG(DBG_ERROR_PRIO, "Message 0x%lx failed to get ACK!", msg_ptr->msg_opcode);
      return result;
   }

   rsp_header_ptr = (spf_msg_header_t *)rsp_ptr->payload_ptr;

   // Compare tokens and opcodes.
   if ((memcmp(&header_ptr->token, &rsp_header_ptr->token, sizeof(spf_msg_token_t))) ||
       (msg_ptr->msg_opcode != rsp_ptr->msg_opcode))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "Message got mismatched ACK, sender token: 0x%lx, "
             "receiver token: 0x%1x, sender opcode: 0x%1x, receiver opcode: "
             "0x%1x",
             header_ptr->token.token_ptr,
             rsp_header_ptr->token.token_ptr,
             msg_ptr->msg_opcode,
             rsp_ptr->msg_opcode);

      return AR_EBADPARAM;
   }

   return result;
}
