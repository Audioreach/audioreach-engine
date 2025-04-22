/**
 * \file spf_svc_utils.h
 * \brief
 *     This file contains structures and message ID's for communication between
 *  spf services.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef _SPF_SVC_UTIL_H_
#define _SPF_SVC_UTIL_H_

/* =======================================================================
INCLUDE FILES FOR MODULE
========================================================================== */
#include "spf_utils.h"
#include "posal_intrinsics.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

/* =======================================================================
Static Inline Functions
========================================================================== */

static inline uint32_t spf_get_bit_index_from_mask(uint32_t mask)
{
   return (31 - s32_cl0_s32(mask)); // Count leading zeros starting from MSB.
   // (Subtracting from 31 gives index of the 1 from right, (conventional bit index))
}

/* -----------------------------------------------------------------------
** Type definitions
** ----------------------------------------------------------------------- */

/** @ingroup spf_svc_datatypes
  Defines a function pointer signature for a message handler function.

  @datatypes
  spf_msg_t

  @param[in] me_ptr      Pointer to the instance structure of the calling
                         service.
  @param[in] msg_ptr     Pointer to the message to process.

  @return
  Depends on the message, which is implicitly agreed upon between the caller
  and callee.

  @dependencies
  None.
 */
typedef ar_result_t (*spf_svc_msg_handler_func)(void * /*me_ptr*/, spf_msg_t * /*msg_ptr*/);

/****************************************************************************
** Service utility functions.
*****************************************************************************/

/** @ingroup spf_svc_func_process_cmd_q
  Loops through the command queue and calls the handler for each function. This
  function finds the handler function via the table lookup from the message ID.

  @datatypes
  spf_handle_t \n
  #spf_svc_msg_handler_func

  @param[in] me_ptr      Pointer to the instance of the calling service, to
                         be passed to the handler.
  @param[in] handle_ptr  Pointer to the service handle of the calling service.
  @param[in] handles     Handler function lookup table of the calling service.
  @param[in] handler_table_size  Number of elements in the lookup table.

  @return
  None.

  @dependencies
  None.
*/
ar_result_t spf_svc_process_cmd_queue(void *                   me_ptr,
                                      spf_handle_t *           handle_ptr,
                                      spf_svc_msg_handler_func handles[],
                                      uint32_t                 handler_table_size);

/** @ingroup spf_svc_unsupported
  Returns the AR_UNSUPPORTED error code. The handler table for a service can
  reference this function for unsupported messages.

  @datatypes
  spf_msg_t

  @param[in] me_ptr      Pointer to the instance structure of the calling
                         service.
  @param[in] msg_ptr     Pointer to the message that requires handling.

  @return
  AR_UNSUPPORTED.

  @dependencies
  None.
*/
ar_result_t spf_svc_unsupported(void *me_ptr, spf_msg_t *msg_ptr);

/** @ingroup spf_svc_func_return_success
  Referenced by the handler table for a service to return an AR_EOK error
  code for a message.

  @datatypes
  spf_msg_t

  @param[in] me_ptr      Pointer to the instance structure of the calling
                         service.
  @param[in] msg_ptr     Pointer to the message that requires handling.

  @return
  AR_EOK.

  @dependencies
  None.
*/
ar_result_t spf_svc_return_success(void *me_ptr, spf_msg_t *msg_ptr);

/** @ingroup spf_svc_func_deinit_data_q
  Flushes and deinits a data queue. This function is called by typical service
  destructors.

  @datatypes

  @param[in] data_q_ptr    Pointer to the data queue of the service.

  @return
  None.

  @dependencies
  None.
*/
void spf_svc_deinit_data_queue(posal_queue_t *data_q_ptr);

/** @ingroup spf_svc_func_destroy_cmd_q
  Flushes and destroys a command queue. This function is called by typical
  service destructors.

  @datatypes


  @param[in] cmdQ    Pointer to the command queue of the service.

  @return
  None.

  @dependencies
  None.
*/
void spf_svc_destroy_cmd_queue(posal_queue_t *cmdQ);

/** @ingroup spf_svc_func_deinit_cmd_q
  Flushes and deinits a command queue. This function is called by typical
  service destructors.

  @datatypes


  @param[in] cmdQ    Pointer to the command queue of the service.

  @return
  None.

  @dependencies
  None.
*/
void spf_svc_deinit_cmd_queue(posal_queue_t *cmd_q_ptr);

/** @ingroup spf_svc_func_deinit_cmd_q
  Only Flushes command queue. This function is called by during FLush
  command or destroy commands.

  @datatypes

  @param[in] cmdQ    Pointer to the queue descriptor.

  @return
  None.

  @dependencies
*/
void spf_svc_drain_cmd_queue(posal_queue_t *cmd_q_ptr);

/** @ingroup spf_svc_func_destroy_buf_q
  Awaits the return of outstanding buffers to their queue, and then destroys
  the buffer queue, thus freeing the buffers.

  This function is to be called by typical service destructors.

  @datatypes


  @param[in]     buf_q_ptr                 Pointer to the buffer queue descriptor.
  @param[in/out] num_bufs_allocated_ptr    Pointer to the number of buffers allocated
                                           and stored in the buffer queue. This function
                                           will decrement this number each time it pops and
                                           frees a buffer. By the end of the function call,
                                           all buffers will be freed and this value will reach
                                           zero.

  @return
  None.

  @dependencies
  None.
*/
void spf_svc_destroy_buf_queue(posal_queue_t *buf_q_ptr, uint32_t *num_bufs_allocated_ptr);

/** @ingroup spf_svc_func_deinit_buf_q
  Awaits the return of outstanding buffers to their queue, and then deinits
  the buffer queue, thus freeing the buffers.

  This function is to be called by typical service destructors.

  @datatypes


  @param[in]     buf_q_ptr                 Pointer to the buffer queue descriptor.
  @param[in/out] num_bufs_allocated_ptr    Pointer to the number of buffers allocated
                                           and stored in the buffer queue. This function
                                           will decrement this number each time it pops and
                                           frees a buffer. By the end of the function call,
                                           all buffers will be freed and this value will reach
                                           zero.

  @return
  None.

  @dependencies
  None.
*/
void spf_svc_deinit_buf_queue(posal_queue_t *buf_q_ptr, uint32_t *num_bufs_allocated_ptr);

/** @ingroup spf_svc_func_free_buf_queue_blocking
  Awaits the return of outstanding buffers to their queue, and then frees the
  buffers but does not destroy the buffer queue.

  This function is called when it is required to free buffers but not destroy
  the queue.

  @datatypes


  @param[in]     buf_q_ptr                 Pointer to the buffer queue descriptor.
  @param[in/out] num_bufs_allocated_ptr    Pointer to the number of buffers allocated
                                           and stored in the buffer queue. This function
                                           will decrement this number each time it pops and
                                           frees a buffer. This function will continue to wait
                                           on the queue until this number reaches zero.
  @return
  None.

  @dependencies
  None.
*/
void spf_svc_free_buffers_in_buf_queue_blocking(posal_queue_t *buf_q_ptr, uint32_t *num_bufs_allocated_ptr);

/** @ingroup spf_svc_func_free_buf_queue_nonblocking
  Frees the buffers that currently sit in the buffer queue.

  @datatypes


  @param[in]     buf_q_ptr                 Pointer to the buffer queue descriptor.
  @param[in/out] num_bufs_allocated_ptr    Pointer to the number of buffers allocated
                                           and stored in the buffer queue. This function
                                           will decrement this number each time it pops and
                                           frees a buffer.

  @return
  None.

  @dependencies
  None.
*/
void spf_svc_free_buffers_in_buf_queue_nonblocking(posal_queue_t *buf_q_ptr, uint32_t *num_bufs_allocated_ptr);

/** @ingroup spf_svc_create_and_push_buffers_to_buf_queue
  Allocate the buffers and store in the buffer queue.

  @datatypes


  @param[in]     buf_q_ptr               Pointer to the buffer queue descriptor.
  @param[in]     buf_size                Size in bytes of buffer to create. Does not include size of
                                         the gk message header.
  @param[in]     dst_handle_ptr          Destination handle to store in the buffer gk message header.
  @param[in]     heap_id                 Heap id to use for allocation.
  @param[in/out] num_bufs_allocated_ptr  Pointer to the number of buffers allocated
                                         and stored in the buffer queue. This function
                                         will increment the number if successful
  @param[in/out] msg_ptr                 Pointer to spf_msg_t. The allocated memory will be filling inside msg_ptr.

  @return
  None.

  @dependencies
  None.
*/
ar_result_t spf_svc_create_buffer_for_buf_queue(posal_queue_t *buf_q_ptr,
                                                uint32_t       buf_size,
                                                spf_handle_t * dst_handle_ptr,
                                                POSAL_HEAP_ID  heap_id,
                                                spf_msg_t *    msg_ptr);
/** @ingroup spf_svc_create_buffer_v2_for_buf_queue
  Allocate the v2 buffers and store in the buffer queue.

  @datatypes


  @param[in]     buf_q_ptr               Pointer to the buffer queue descriptor.
  @param[in]     bufs_num                Number of data buffers in this data message
  @param[in]     buf_size                Size in bytes of buffer to create. Does not include size of
                                         the gk message header.
  @param[in]     dst_handle_ptr          Destination handle to store in the buffer gk message header.
  @param[in]     heap_id                 Heap id to use for allocation.
  @param[in/out] num_bufs_allocated_ptr  Pointer to the number of buffers allocated
                                         and stored in the buffer queue. This function
                                         will increment the number if successful
  @param[in/out] msg_ptr                 Pointer to spf_msg_t. The allocated memory will be filling inside msg_ptr.

  @return
  None.

  @dependencies
  None.
*/
ar_result_t spf_svc_create_buffer_v2_for_buf_queue(posal_queue_t *  buf_q_ptr,
                                                   uint32_t         bufs_num,
                                                   uint32_t         buf_size_per_data_buf,
                                                   spf_handle_t *   dst_handle_ptr,
                                                   POSAL_HEAP_ID    heap_id,
                                                   spf_msg_token_t *token_ptr,
                                                   spf_msg_t *      msg_ptr);
/** @ingroup spf_svc_create_and_push_buffers_to_buf_queue
  Allocate the buffers and store in the buffer queue.

  @datatypes


  @param[in]     buf_q_ptr               Pointer to the buffer queue descriptor.
  @param[in]     buf_size                Size in bytes of buffer to create. Does not include size of
                                         the gk message header.
  @param[in]     num_out_bufs            Allocate buffers until this many exist in the queue. Will allocate
                                         less than this number if num_bufs_allocated_ptr is > 0.
  @param[in]     dst_handle_ptr          Destination handle to store in the buffer gk message header.
  @param[in]     heap_id                 Heap id to use for allocation.
  @param[in]     msg_ptr                 Payload ptr will be copied to this.

  @return
  None.

  @dependencies
  None.
*/
ar_result_t spf_svc_create_and_push_buffers_to_buf_queue(posal_queue_t *buf_q_ptr,
                                                         uint32_t       buf_size,
                                                         uint32_t       num_out_bufs,
                                                         spf_handle_t * dst_handle_ptr,
                                                         POSAL_HEAP_ID  heap_id,
                                                         uint32_t *     num_bufs_allocated_ptr);
/** @ingroup spf_svc_create_and_push_buffers_v2_to_buf_queue
  Allocate the v2 buffers and store in the buffer queue.

  @datatypes


  @param[in]     buf_q_ptr               Pointer to the buffer queue descriptor.
  @param[in]     bufs_num                Number of data buffers in this data message
  @param[in]     buf_size                Size in bytes of buffer to create. Does not include size of
                                         the gk message header.
  @param[in]     num_out_bufs            Allocate buffers until this many exist in the queue. Will allocate
                                         less than this number if num_bufs_allocated_ptr is > 0.
  @param[in]     dst_handle_ptr          Destination handle to store in the buffer gk message header.
  @param[in]     heap_id                 Heap id to use for allocation.
  @param[in]     heap_id                 Token set for the created message.
  @param[in]     msg_ptr                 Payload ptr will be copied to this.

  @return
  None.

  @dependencies
  None.
*/
ar_result_t spf_svc_create_and_push_buffers_v2_to_buf_queue(posal_queue_t *  buf_q_ptr,
                                                            uint32_t         bufs_num,
                                                            uint32_t         buf_size_per_data_buf,
                                                            uint32_t         num_out_bufs,
                                                            spf_handle_t *   dst_handle_ptr,
                                                            POSAL_HEAP_ID    heap_id,
                                                            spf_msg_token_t *token_ptr,
                                                            uint32_t *       num_bufs_allocated_ptr);
/** @ingroup spf_svc_create_and_push_ctrl_msg_bufs_to_buf_queue
  Allocate the control buffers and store in the control port handle's buffer queue.

  @datatypes


  @param[in]     buf_q_ptr               Pointer to the control buffer queue descriptor.
  @param[in]     buf_size                Size in bytes of control buffer to create. Does not include size of
                                         the gk msg header.
  @param[in]     num_out_bufs            Allocate buffers until this many exist in the queue. Will allocate
                                         less than this number if num_bufs_allocated_ptr is > 0.
  @param[in]     dst_handle_ptr          Destination control port handle to store in the buffer gk message header.
  @param[in]     heap_id                 Heap id to use for allocation.
  @param[in]     msg_ptr                 Payload ptr will be copied to this.

  @return
  None.

  @dependencies
  None.
*/
ar_result_t spf_svc_create_and_push_ctrl_msg_bufs_to_buf_queue(posal_queue_t *buf_q_ptr,
                                                               uint32_t       buf_size,
                                                               uint32_t       num_out_bufs,
                                                               spf_handle_t * dst_handle_ptr,
                                                               bool_t         is_inter_proc_link,
                                                               POSAL_HEAP_ID  heap_id,
                                                               uint32_t *     num_bufs_allocated_ptr);

/** @ingroup spf_svc_drain_ctrl_msg_queue
    Flushes the incoming control messages in the control port queue.

  @datatypes

  @param[in] incoming ctrl msg queue ptr.

  @return
  None.

  @dependencies
*/
void spf_svc_drain_ctrl_msg_queue(posal_queue_t *ctrl_msg_q_ptr);

/** @ingroup spf_svc_drain_ctrl_msg_queue
    Blocking call to wait on outgoing ctrl_port msg buffers q_ptr and free all the allocated buffers.
    Generally done during control port destroy.

  @datatypes

  @param[in] outgoing ctrl msg queue ptr.
  @param[in] pointer to number of allocated buffers. updated on return.

  @return
  None.

  @dependencies
*/
void spf_svc_free_ctrl_msgs_in_buf_queue_blocking(posal_queue_t *ctrl_msg_q_ptr, uint32_t *num_bufs_allocated_ptr);

/** @ingroup spf_svc_drain_ctrl_msg_queue
    Non-Blocking call to wait on outgoing ctrl_port msg buffers q_ptr and free all the allocated buffers.
    Generally called during buffers recreate.

  @datatypes

  @param[in] outgoing ctrl msg queue ptr.
  @param[in] pointer to number of allocated buffers. updated on return.

  @return
  None.

  @dependencies
*/
void spf_svc_free_ctrl_msgs_in_buf_queue_nonblocking(posal_queue_t *ctrl_msg_q_ptr, uint32_t *num_bufs_allocated_ptr);

/** @ingroup spf_svc_get_frame_size
  Called by the services to determine the operative frame size in samples based
  on the input sampling rate. The service must validate the sample rate before
  calling this function.

  @param[in] sample_rate      Sampling rate for which the frame size in samples
                              must be calculated.
  @param[in] num_samples_ptr  Pointer to the frame size in samples to report
                              in an acknowledgement.

  @return
  Frame size is returned in an acknowledgement.

  @dependencies
  None.
*/
static inline void spf_svc_get_frame_size(uint32_t sample_rate, uint32_t *num_samples_ptr)
{
   // Returns 1 sample as a minimum value
   if (sample_rate < 1000)
   {
      *num_samples_ptr = 1;
      return;
   }
   *num_samples_ptr = (sample_rate / 1000);
}

static inline void spf_svc_crash(void)
{
#if defined(SIM)
   *((volatile uint32_t *)0) = 0;
#else
   return;
#endif
}

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef _SPF_SVC_UTIL_H_