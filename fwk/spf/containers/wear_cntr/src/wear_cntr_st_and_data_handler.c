/**
 * \file wear_cntr_st_and_data_handler.c
 * \brief
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "wear_cntr_i.h"

static void wcntr_st_track_signal_miss(wcntr_t *me_ptr);



ar_result_t wcntr_signal_trigger(wcntr_base_t *cu_ptr, uint32_t channel_bit_index)
{
   INIT_EXCEPTION_HANDLING
   ar_result_t result = AR_EOK;
   wcntr_t *me_ptr = (wcntr_t *)cu_ptr;

#ifdef VERBOSE_DEBUGGING
   WCNTR_MSG(me_ptr->topo.gu.log_id, DBG_LOW_PRIO, "wcntr_trigger: Received trigger signal");
#endif
   me_ptr->signal_trigger_count++;
   /*clear the trigger signal */
   posal_signal_clear(me_ptr->trigger_signal_ptr);

   me_ptr->topo.proc_context.curr_trigger = WCNTR_TOPO_SIGNAL_TRIGGER;

   /* Trigger process once all the ports are prepared with inputs/outputs */
   TRY(result, wcntr_data_process_frames(me_ptr));

   CATCH(result, WCNTR_MSG_PREFIX, me_ptr->topo.gu.log_id)
   {
   }

   me_ptr->topo.proc_context.curr_trigger = WCNTR_TOPO_INVALID_TRIGGER;
   wcntr_st_track_signal_miss(me_ptr);

   return result;
}

/**
 * may run in interrupt thread
 */
static void wcntr_st_track_signal_miss(wcntr_t *me_ptr)
{
   // this method cannot detect multiple signal misses.
   if (posal_signal_is_set(me_ptr->trigger_signal_ptr))
   {
      WCNTR_MSG(me_ptr->topo.gu.log_id, DBG_HIGH_PRIO, "WCNTR interrupt callback: signal miss");
      me_ptr->signal_miss_count++;
   }
}

wcntr_topo_module_t *wcntr_get_stm_module(wcntr_t *me_ptr)
{
   wcntr_topo_module_t *stm_ptr         = NULL;
   uint32_t           num_stm_modules = 0;
   for (wcntr_gu_sg_list_t *sg_list_ptr = me_ptr->topo.gu.sg_list_ptr; (NULL != sg_list_ptr); LIST_ADVANCE(sg_list_ptr))
   {
      for (wcntr_gu_module_list_t *module_list_ptr = sg_list_ptr->sg_ptr->module_list_ptr; (NULL != module_list_ptr);
           LIST_ADVANCE(module_list_ptr))
      {
         wcntr_topo_module_t *module_ptr = (wcntr_topo_module_t *)module_list_ptr->module_ptr;

         if (module_ptr->flags.need_stm_extn)
         {
            stm_ptr = module_ptr;
            num_stm_modules++;
         }
      }
   }

   // Return error if there are more than one stm module in the topology.
   if (num_stm_modules > 1)
   {
      WCNTR_MSG(me_ptr->topo.gu.log_id, DBG_ERROR_PRIO, "%lu > 1 STM modules found.", num_stm_modules);
      return NULL;
   }

   return stm_ptr;
}


ar_result_t wcntr_data_process_frames(wcntr_t *me_ptr)
{
   ar_result_t result = AR_EOK;
   wcntr_gu_module_list_t *start_module_list_ptr = me_ptr->topo.gu.sorted_module_list_ptr;
   wcntr_poll_and_process_int_ctrl_msgs(&me_ptr->cu);
   result =  wcntr_topo_topo_process(&me_ptr->topo, &start_module_list_ptr);
   //Runtime change is not supported currently,Calling the following function
   //would propagate mf and threshold. But glich may be seen
   wcntr_handle_fwk_events(me_ptr);
   return result;
}
