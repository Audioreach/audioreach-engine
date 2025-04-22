/**
 * \file cu_island.c
 *
 * \brief
 *
 *     Container utilities.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/* =======================================================================
INCLUDE FILES FOR MODULE
========================================================================== */
/*==============================================================================
   Global Defines
==============================================================================*/
#include "cu_i.h"

ar_result_t cu_handle_cmd_queue(cu_base_t *me_ptr, uint32_t channel_bit_index)
{
   ar_result_t result = AR_EOK;

   // Here we only exit island temporarily instead of voting against island.
   // For eg. if the command results in event handling we can vote against island in event handler context
   if (me_ptr->cntr_vtbl_ptr->exit_island_temporarily)
   {
      me_ptr->cntr_vtbl_ptr->exit_island_temporarily(me_ptr);
   }

   // Take next msg from the queue.
   result = posal_queue_pop_front(me_ptr->cmd_handle.cmd_q_ptr, (posal_queue_element_t *)&(me_ptr->cmd_msg));

   // Process the message.
   if (AR_EOK == result)
   {
#ifdef CONTAINER_ASYNC_CMD_HANDLING
      if (FALSE == cu_async_cmd_handle_check_and_push_cmd(me_ptr))
#endif
      {
         result = cu_process_cmd_queue(me_ptr);
      }
   }

   return result;
}

// Pre-workloop tasks.
ar_result_t cu_workloop_entry(void *instance_ptr)
{
   ar_result_t result = AR_EOK;
   cu_base_t  *me_ptr = (cu_base_t *)instance_ptr;

   // If there was a previous thread, join it. me_ptr->cntr_handle.thread_id
   // is assigned only in workloop.
   if (me_ptr->thread_id_to_exit)
   {
      CU_MSG(me_ptr->gu_ptr->log_id,
             DBG_HIGH_PRIO,
             "Joining thread ID 0x%lX",
             posal_thread_get_tid_v2(me_ptr->thread_id_to_exit));

      posal_thread_join(me_ptr->thread_id_to_exit, &result);
      me_ptr->thread_id_to_exit = 0;
   }
#ifdef CONTAINER_ASYNC_CMD_HANDLING
   // If container thread is re-launched with updated stack size then update the thread pool also
   cu_async_cmd_handle_update(me_ptr);
#endif
   me_ptr->gu_ptr->data_path_thread_id = posal_thread_get_curr_tid();

   // If any command handling was done partially, complete the rest now.
   if (cu_is_any_handle_rest_pending(me_ptr))
   {
      me_ptr->handle_rest_fn(me_ptr, me_ptr->handle_rest_ctx_ptr);
   }

   // if handle rest is again set, don't enter workloop, just exit
   if (!cu_is_any_handle_rest_pending(me_ptr))
   {
      // Call the workloop.
      result = cu_workloop(me_ptr);
   }

   return result;
}

ar_result_t cu_workloop(cu_base_t *me_ptr)
{
   ar_result_t result = AR_EFAILED;
   uint32_t    channel_status;
   SPF_MANAGE_CRITICAL_SECTION

   // Loop until termination.
   for (;;)
   {
      // Block on any selected queues to get a msg.
      (void)posal_channel_wait_inline(me_ptr->channel_ptr, me_ptr->curr_chan_mask);

      SPF_CRITICAL_SECTION_START(me_ptr->gu_ptr);
      for (;;)
      {
         // Check for signals.
         channel_status = posal_channel_poll_inline(me_ptr->channel_ptr, me_ptr->curr_chan_mask);

         if (channel_status == 0)
         {
            break;
         }

         int32_t bit_index = cu_get_bit_index_from_mask(channel_status);

         if (NULL == me_ptr->qftable[bit_index])
         {
            CU_MSG(me_ptr->gu_ptr->log_id,
                   DBG_ERROR_PRIO,
                   "No handler at bit position %lu, mask 0x%lx. Not listening "
                   "to this bit anymore",
                   bit_index,
                   channel_status);

            // Clear the bit in the me_ptr->curr_chan_mask.
            me_ptr->curr_chan_mask &= (~(1 << bit_index));
            continue;
         }

         result = me_ptr->qftable[bit_index](me_ptr, bit_index);

         if (result == AR_ETERMINATED)
         {
            return AR_EOK;
         }

         // In case new thread got created.
         if (posal_thread_get_tid_v2(me_ptr->thread_id_to_exit) == posal_thread_get_curr_tid())
         {
            CU_MSG(me_ptr->gu_ptr->log_id,
                   DBG_MED_PRIO,
                   "Thread ID 0x%lX exited",
                   posal_thread_get_tid_v2(me_ptr->thread_id_to_exit));
            SPF_CRITICAL_SECTION_END(me_ptr->gu_ptr);
            return AR_EOK;
         }
      }
      SPF_CRITICAL_SECTION_END(me_ptr->gu_ptr);
   }

   return result;
}
