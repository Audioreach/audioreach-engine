/**
 * \file cu_prebuffer_q.c
 *
 * \brief
 *     Implementation of pre-buffer q utility to hold prebuffers during first frame processing
 *
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear */

/* =======================================================================
Includes
========================================================================== */
#include "cu_i.h"

/* =======================================================================
Function definitions
========================================================================== */

// function to push back a data message into prebuffer Q.
static ar_result_t cu_push_msg_to_prebuffer_q(cu_base_t *me_ptr, cu_ext_in_port_t *cu_ext_in_port_ptr, spf_msg_t msg)
{
   ar_result_t result = AR_EOK;

   void *obj_ptr = posal_memory_malloc(sizeof(spf_msg_t), me_ptr->heap_id);

   if (NULL == obj_ptr)
   {
      CU_MSG(me_ptr->gu_ptr->log_id, DBG_ERROR_PRIO, "memory allocation failed!");
      return AR_ENOMEMORY;
   }

   memscpy(obj_ptr, sizeof(spf_msg_t), &msg, sizeof(msg));

   result = spf_list_insert_tail(&cu_ext_in_port_ptr->prebuffer_queue_ptr, obj_ptr, me_ptr->heap_id, FALSE);

   return result;
}

// function to pop front a data message from prebuffer Q.
static ar_result_t cu_pop_msg_from_prebuffer_q(cu_base_t        *me_ptr,
                                               cu_ext_in_port_t *cu_ext_in_port_ptr,
                                               spf_msg_t        *msg_ptr)
{
   ar_result_t result = AR_EOK;

   void *obj_ptr = spf_list_pop_head(&cu_ext_in_port_ptr->prebuffer_queue_ptr, FALSE);

   if (obj_ptr)
   {
      memscpy(msg_ptr, sizeof(spf_msg_t), obj_ptr, sizeof(spf_msg_t));
      posal_memory_free(obj_ptr);
   }
   else
   {
      return AR_EFAILED;
   }

   return result;
}

/**
 * Function to
 *  1. check and push a data message into the prebuffer Q.
 *  2. check and pop a data message from the prebuffer Q.
 */
ar_result_t cu_ext_in_handle_prebuffer(cu_base_t        *me_ptr,
                                       gu_ext_in_port_t *gu_ext_in_port_ptr,
                                       uint32_t          min_num_buffer_to_hold)
{
   // todo: only allow PCM media format

   cu_ext_in_port_t *cu_ext_in_port_ptr =
      (cu_ext_in_port_t *)(((uint8_t *)gu_ext_in_port_ptr) + me_ptr->ext_in_port_cu_offset);

   /* We need to avoid the prebuffers to drain due to disabled threshold (by sync modules).
    * For each input data trigger, data message is first pushed to the prebuffer Q and a data
    * message is popped from the prebuffer Q.
    * If multiple prebuffers are pushed by upstream then we first stores all those prebuffers into the Q before popping
    * a buffer from it. If prebuffers are not pushed by upstream then we at least need to maintain one prebuffer in the
    * Q if smart sync is present. (this is needed when threshold is disabled due to VFR resync, upstream doesn't know
    * about the resync so it is not going to send the prebuffers.)
    */
   if (cu_ext_in_port_ptr->input_data_q_msg.payload_ptr)
   {
      spf_msg_header_t      *header_ptr    = (spf_msg_header_t *)(cu_ext_in_port_ptr->input_data_q_msg.payload_ptr);
      spf_msg_data_buffer_t *input_buf_ptr = (spf_msg_data_buffer_t *)&header_ptr->payload_start;
      module_cmn_md_list_t  *md_list_ptr   = input_buf_ptr->metadata_list_ptr;
      bool_t                 is_flushing_eos_dfg_found = FALSE;

      while (md_list_ptr)
      {
         if (MODULE_CMN_MD_ID_EOS == md_list_ptr->obj_ptr->metadata_id)
         {
            module_cmn_md_eos_t *eos_metadata_ptr = NULL;

            if (md_list_ptr->obj_ptr->metadata_flag.is_out_of_band)
            {
               eos_metadata_ptr = (module_cmn_md_eos_t *)md_list_ptr->obj_ptr->metadata_ptr;
            }
            else
            {
               eos_metadata_ptr = (module_cmn_md_eos_t *)&(md_list_ptr->obj_ptr->metadata_buf);
            }

            is_flushing_eos_dfg_found |= eos_metadata_ptr->flags.is_flushing_eos;
         }
         else if (MODULE_CMN_MD_ID_DFG == md_list_ptr->obj_ptr->metadata_id)
         {
            is_flushing_eos_dfg_found = TRUE;
         }
         LIST_ADVANCE(md_list_ptr);
      }

      bool_t is_prebuffer =
         cu_get_bits(input_buf_ptr->flags, DATA_BUFFER_FLAG_PREBUFFER_MARK_MASK, DATA_BUFFER_FLAG_PREBUFFER_MARK_SHIFT);

      ar_result_t result = cu_push_msg_to_prebuffer_q(me_ptr, cu_ext_in_port_ptr, cu_ext_in_port_ptr->input_data_q_msg);
      if (AR_EOK == result)
      {
         CU_MSG(me_ptr->gu_ptr->log_id,
                DBG_HIGH_PRIO,
                "Pushed a data buffer to the prebuffer queue at ext input idx = %ld, miid = 0x%lx, buffer ptr "
                "0x%x, buffer "
                "size %lu",
                gu_ext_in_port_ptr->int_in_port_ptr->cmn.index,
                gu_ext_in_port_ptr->int_in_port_ptr->cmn.module_ptr->module_instance_id,
                cu_ext_in_port_ptr->input_data_q_msg.payload_ptr,
                input_buf_ptr->actual_size);

         cu_ext_in_port_ptr->input_data_q_msg.payload_ptr = NULL;

         uint32_t prebuffers_count = spf_list_count_elements(cu_ext_in_port_ptr->prebuffer_queue_ptr);

         /* Pop the buffer from prebuffer Q if the recently pushed buffer is not a prebuffer, we also need to maintain
          * at least one prebuffer when there is a smart sync module, because it can disable the threshold any time.
          * if EOS or DFG is found then must continue draining prebuffer queue
          */
         if (is_flushing_eos_dfg_found || (!is_prebuffer && prebuffers_count > min_num_buffer_to_hold))
         {
            cu_pop_msg_from_prebuffer_q(me_ptr, cu_ext_in_port_ptr, &cu_ext_in_port_ptr->input_data_q_msg);

            CU_MSG(me_ptr->gu_ptr->log_id,
                   DBG_HIGH_PRIO,
                   "Popped a data buffer to the prebuffer queue at ext input idx = %ld, miid = 0x%lx, buffer_ptr "
                   "0x%x",
                   gu_ext_in_port_ptr->int_in_port_ptr->cmn.index,
                   gu_ext_in_port_ptr->int_in_port_ptr->cmn.module_ptr->module_instance_id,
                   cu_ext_in_port_ptr->input_data_q_msg.payload_ptr);

            // EOS,DFG may be stuck in the prebuffer queue, move it back to the main queue so that container can wake
            // up.
            if (is_flushing_eos_dfg_found)
            {
               cu_ext_in_requeue_prebuffers(me_ptr, gu_ext_in_port_ptr);
            }
         }
      }
   }

   return AR_EOK;
}

/**
 * requeue all messages from the external input port's prebuffer queue to the data queue.
 */
ar_result_t cu_ext_in_requeue_prebuffers(cu_base_t *me_ptr, gu_ext_in_port_t *gu_ext_in_port_ptr)
{
   cu_ext_in_port_t *cu_ext_in_port_ptr =
      (cu_ext_in_port_t *)(((uint8_t *)gu_ext_in_port_ptr) + me_ptr->ext_in_port_cu_offset);

   if (NULL != cu_ext_in_port_ptr->prebuffer_queue_ptr)
   {
      spf_msg_t q_msg = { 0 };
      // first push all the messages from the main DATA Q to the Prebuffer Q and then from Prebuffer Q to the main Data
      // to maintain FIFO order
      CU_MSG(me_ptr->gu_ptr->log_id,
             DBG_HIGH_PRIO,
             "requeuing prebuffers at ext input idx = %ld, miid = 0x%lx",
             gu_ext_in_port_ptr->int_in_port_ptr->cmn.index,
             gu_ext_in_port_ptr->int_in_port_ptr->cmn.module_ptr->module_instance_id);

      posal_queue_lock_mutex(gu_ext_in_port_ptr->this_handle.q_ptr);
      while (
         AR_SUCCEEDED(posal_queue_pop_front(gu_ext_in_port_ptr->this_handle.q_ptr, (posal_queue_element_t *)&q_msg)))
      {
         cu_push_msg_to_prebuffer_q(me_ptr, cu_ext_in_port_ptr, q_msg);
      }

      while (AR_SUCCEEDED(cu_pop_msg_from_prebuffer_q(me_ptr, cu_ext_in_port_ptr, &q_msg)))
      {
         CU_MSG(me_ptr->gu_ptr->log_id,
                DBG_HIGH_PRIO,
                "Pushing a data buffer from the prebuffer queue to input Q at ext input idx = %ld, miid = 0x%lx, "
                "buffer ptr 0x%x",
                gu_ext_in_port_ptr->int_in_port_ptr->cmn.index,
                gu_ext_in_port_ptr->int_in_port_ptr->cmn.module_ptr->module_instance_id,
                q_msg.payload_ptr);

         posal_queue_push_back(gu_ext_in_port_ptr->this_handle.q_ptr, (posal_queue_element_t *)&q_msg);
      }
      posal_queue_unlock_mutex(gu_ext_in_port_ptr->this_handle.q_ptr);
   }

   return AR_EOK;
}

/**
 * release all messages from the external input port's prebuffer data queue.
 */
ar_result_t cu_ext_in_release_prebuffers(cu_base_t *me_ptr, gu_ext_in_port_t *gu_ext_in_port_ptr)
{
   cu_ext_in_port_t *cu_ext_in_port_ptr =
      (cu_ext_in_port_t *)(((uint8_t *)gu_ext_in_port_ptr) + me_ptr->ext_in_port_cu_offset);

   if (NULL != cu_ext_in_port_ptr->prebuffer_queue_ptr)
   {
      spf_msg_t cache_msg                              = cu_ext_in_port_ptr->input_data_q_msg;
      cu_ext_in_port_ptr->input_data_q_msg.payload_ptr = NULL;
      CU_MSG(me_ptr->gu_ptr->log_id,
             DBG_HIGH_PRIO,
             "freeing prebuffer queue at ext input idx = %ld, miid = 0x%lx",
             gu_ext_in_port_ptr->int_in_port_ptr->cmn.index,
             gu_ext_in_port_ptr->int_in_port_ptr->cmn.module_ptr->module_instance_id);

      while (
         AR_SUCCEEDED(cu_pop_msg_from_prebuffer_q(me_ptr, cu_ext_in_port_ptr, &cu_ext_in_port_ptr->input_data_q_msg)))
      {
         cu_free_input_data_cmd(me_ptr, gu_ext_in_port_ptr, AR_EOK);
      }
      cu_ext_in_port_ptr->input_data_q_msg = cache_msg;
   }

   /* if data buffers are dropped then prebuffering will be lost, so set the flag so that prebuffering can be preserved
    * for next frame processing*/
   cu_ext_in_port_ptr->preserve_prebuffer = TRUE;

   return AR_EOK;
}

