#ifndef _SPF_MSG_UTIL_H_
#define _SPF_MSG_UTIL_H_

/**
 * \file spf_msg_util.h
 * \brief
 *     This file defines common utility function for spf messages.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/*===========================================================================
NOTE: The @brief description above does not appear in the PDF.
      The description that displays in the PDF is located in the
      Elite_mainpage.dox file. Contact Tech Pubs for assistance.
===========================================================================*/

/*-------------------------------------------------------------------------
Include Files
-------------------------------------------------------------------------*/
#include "spf_utils.h"
#include "posal.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

/*-------------------------------------------------------------------------
Preprocessor Definitions and Constants
-------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------
Type Declarations
-------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
Class Definitions
----------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
Function Declarations and Documentation
----------------------------------------------------------------------------*/

/** @ingroup spf_msg_create_header
  Creates a message payload buffer based on the size requested, and fills the
  buffer opcode, return queue, response queue, and token fields in the payload
  buffer. The response result, payload_size, and payload_start fields are zerod
  out. Then, fills the payload into a spf message.

  The caller is responsible for filling the payload_size and payload_start
  fields.

  @datatypes
  spf_msg_t \n
  spf_handle_t

  @param[in,out] msg_ptr         Pointer to the message to be filled in.
  @param[in]     buf_size_ptr    Pointer to the payload buffer size requested.
  @param[in]     msg_opcode      Operation code to fill in the message.
  @param[in]     resp_handle_ptr Pointer to the response of this message.
  @param[in]     msg_token       Client token of this message.
  @param[in]     dst_handle_ptr  Handle of the destination the message will be
                                 sent to. NULL if unused.
  @param[in]     heap_id         heap id used for mallocs.
  @return
  Indication of success or failure.

  @dependencies
  None.
 */
ar_result_t spf_msg_create_msg(spf_msg_t *      msg_ptr,
                               uint32_t *       buf_size_ptr,
                               uint32_t         msg_opcode,
                               spf_handle_t *   resp_handle_ptr,
                               spf_msg_token_t *msg_token_ptr,
                               spf_handle_t *   dst_handle_ptr,
                               POSAL_HEAP_ID    heap_id);

/** @ingroup spf_msg_init_handle
  Initializes the fields of a spf_handle.

  @datatypes
  spf_handle_t \n
  spf_cmd_handle_t \n
  posal_queue_t

  @param[in,out] handle_ptr       Pointer to the handle to be initialized.
  @param[in]     spf_cmd_handle_t  Pointer to the command handle. NULL if there
                                  is no associated command handle.
  @param[in]     posal_queue_t    Pointer to the queue.

  @return
  Indication of success or failure.

  @dependencies
  None.
 */
ar_result_t spf_msg_init_handle(spf_handle_t *handle_ptr, spf_cmd_handle_t *cmd_handle_ptr, posal_queue_t *q_ptr);

/** @ingroup spf_msg_init_cmd_handle
  Initializes the fields of a spf_cmd_handle.

  @datatypes
  spf_cmd_handle_t \n
  posal_queue_t

  @param[in,out] spf_cmd_handle_t Pointer to the command handle to be
                                 initialized.
  @param[in]     thread_id       The thread id.
  @param[in]     posal_queue_t   Pointer to the command queue.
  @param[in]     posal_queue_t   Pointer to the system command queue.
  @return
  Indication of success or failure.

  @dependencies
  None.
 */
ar_result_t spf_msg_init_cmd_handle(spf_cmd_handle_t *handle_ptr,
                                    posal_thread_t    thread_id,
                                    posal_queue_t *   cmd_q_ptr,
                                    posal_queue_t *   sys_cmd_q_ptr);

/** @ingroup spf_msg_send_cmd
  Sends a message to the destination handle's control path by pushing the
  message to the handle's command queue.

  @datatypes
  spf_msg_t \n
  spf_handle_t

  @param[in]     msg_ptr         The message to send.
  @param[in]     dst_handle_ptr  The handle to send the message to.

  @return
  Indication of success or failure.

  @dependencies
  None.
 */
ar_result_t spf_msg_send_cmd(spf_msg_t *msg_ptr, spf_handle_t *dst_handle_ptr);

/** @ingroup spf_msg_send_sys_cmd
  Sends a message to the destination handle's "priority" control path by pushing the
  message to the handle's "system command queue".

  @datatypes
  spf_msg_t \n
  spf_handle_t

  @param[in]     msg_ptr         The message to send.
  @param[in]     dst_handle_ptr  The handle to send the message to.

  @return
  Indication of success or failure.

  @dependencies
  None.
 */
ar_result_t spf_msg_send_sys_cmd(spf_msg_t *msg_ptr, spf_handle_t *dst_handle_ptr);

/** @ingroup spf_msg_send_data
  Sends a message to the destination handle's data path by pushing the
  message to the handle's queue.

  @datatypes
  spf_msg_t \n
  spf_handle_t

  @param[in]     msg_ptr         The message to send.
  @param[in]     dst_handle_ptr  The handle to send the message to.

  @return
  Indication of success or failure.

  @dependencies
  None.
 */
ar_result_t spf_msg_send_data(spf_msg_t *msg_ptr, spf_handle_t *dst_handle_ptr);

/** @ingroup spf_msg_send_data
  Sends a message to its response handle by pushing the message to its response
  handle's queue.

  @datatypes
  spf_msg_t \n
  spf_handle_t

  @param[in]     msg_ptr         The message to send.

  @return
  Indication of success or failure.

  @dependencies
  None.
 */
ar_result_t spf_msg_send_response(spf_msg_t *msg_ptr);

/** @ingroup spf_msg_return_msg
  Checks if the message's return queue was created by the buffer manager,
  and if so, returns the spf message header to the buffer manager. Otherwise,
  pushes the spf message header to the return queue.

  @datatypes
  spf_msg_t

  @param[in]     msg_ptr    Pointer to the spf message. Returns to this
                            message's return queue.
  @return
  Indication of success or failure.

  @dependencies
  None.
 */
ar_result_t spf_msg_return_msg(spf_msg_t *msg_ptr);

/** @ingroup spf_msg_ack_msg
  If the message contains a response handle, then sends the message to the
  response handle, modifying the response result using the passed-in result.
  If the message does not contain a response handle, then pushes the message
  to its return queue.

  @datatypes
  spf_msg_t

  @param[in]     msg_ptr       Pointer to the message to be returned.
  @param[in]     rsp_result    The response result.

  @return
  Indication of success or failure.

  @dependencies
  None.
 */
ar_result_t spf_msg_ack_msg(spf_msg_t *msg_ptr, uint32_t rsp_result);

/** @ingroup spf_msg_wait_for_ack_and_get_rsp
  Blocking call that waits for a response to be pushed to the response handle
  of the message. Then, pops the message from the response handle's queue and
  validates that the message and its response have equal opcodes and tokens.
  Returns the response message.

  A NULL rsp_ptr indicates a response message is not needed.

  @datatypes
  spf_msg_t

  @param[in]     msg_ptr       Pointer to the message to be returned.
  @param[in]     rsp_ptr       Pointer to the response message. NULL if not
                               needed.
  @return
  Success indicates the message was successfully popped and the sent/received
  messages have matching tokens.

  @dependencies
  None.
 */
ar_result_t spf_msg_wait_for_ack(spf_msg_t *msg_ptr, spf_msg_t *rsp_ptr);

/** @ingroup spf_msg_convt_buf_node_to_msg
  Converts a buffer node to a spf message. Conversion is in-place, and the
  memory management is transferred from the keeper of the node to the keeper
  of the spf message.

  @datatypes
  fwk_bufmgr_node_t \n
  spf_handle_t

  @param[in]     node_ptr        The node to convert to a message.
  @param[in]     msg_opcode      Operation code to fill in the message.
  @param[in]     resp_handle_ptr Pointer to the response of this message.
  @param[in]     msg_token       Token of this message.
  @param[in]     rsp_result      Response result of this message.
  @param[in]     dst_handle_ptr  Handle of the port the message will be sent to.
                                 NULL indicates the message is not directed to a
                                 specific port.
  @return
  A pointer to the new message.

  @dependencies
  None.
 */
spf_msg_t *spf_msg_convt_buf_node_to_msg(posal_bufmgr_node_t *node_ptr,
                                         uint32_t             msg_opcode,
                                         spf_handle_t *       resp_handle_ptr,
                                         spf_msg_token_t *    msg_token_ptr,
                                         uint32_t             rsp_result,
                                         spf_handle_t *       dst_handle_ptr);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef _SPF_MSG_UTIL_H_
