/**
 * \file gen_cntr_fwk_extn_utils_isalnd.c
 * \brief
 *     This file contaouts utility functions for GEN_CNTR capi fwk extensions in island.
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "gen_cntr_i.h"

/** This function is invoked by the workloop on receiving the async signal.*/
ar_result_t gen_cntr_workloop_async_signal_trigger_handler(cu_base_t *base_ptr, uint32_t bit_index)
{
   ar_result_t              result           = AR_EOK;
   gen_cntr_t              *me_ptr           = (gen_cntr_t *)base_ptr;
   gen_cntr_async_signal_t *async_st_hdl_ptr = NULL;

   /** get async signal handle corresponding to the module */
   for (spf_list_node_t *list_ptr = me_ptr->async_signal_list_ptr; NULL != list_ptr; LIST_ADVANCE(list_ptr))
   {
      gen_cntr_async_signal_t *temp_ptr = (gen_cntr_async_signal_t *)list_ptr->obj_ptr;
      if (bit_index == temp_ptr->bit_index)
      {
         async_st_hdl_ptr = temp_ptr;
         break;
      }
   }

   if (NULL == async_st_hdl_ptr)
   {
      GEN_CNTR_MSG_ISLAND(me_ptr->topo.gu.log_id,
                          DBG_ERROR_PRIO,
                          "Async signal not found corresponding. stop listening to bit_index:0x%lu",
                          bit_index);
      /** This is unexpected racecondition, that can happen if the signal is set by an HW interrupt while async signal
       * is being destroyed. */
      uint32_t bit_mask = 1 << bit_index;
      cu_stop_listen_to_mask(&me_ptr->cu, bit_mask);
      return AR_EFAILED;
   }

   /*clear the trigger signal */
   posal_signal_clear(async_st_hdl_ptr->signal_ptr);

   gen_topo_module_t *module_ptr = async_st_hdl_ptr->module_ptr;
   GEN_CNTR_MSG_ISLAND(me_ptr->topo.gu.log_id,
                       DBG_HIGH_PRIO,
                       "Module 0x%lX: Recevied an Async signal trigger, bit_index:0x%lu",
                       module_ptr->gu.module_instance_id,
                       bit_index);

   result = async_st_hdl_ptr->cb_fn_ptr(async_st_hdl_ptr->cb_context_ptr);
   if (AR_FAILED(result))
   {
      GEN_CNTR_MSG_ISLAND(me_ptr->topo.gu.log_id,
                          DBG_ERROR_PRIO,
                          "Module 0x%lX: callback return failure on Async signal trigger, bit_index:0x%lu",
                          module_ptr->gu.module_instance_id,
                          bit_index);
   }

   /** Async signals are treated as data triggers, because this signal itself could be the start of module's process
    * trigger. */
   me_ptr->topo.proc_context.curr_trigger = GEN_TOPO_DATA_TRIGGER;

   GEN_CNTR_MSG_ISLAND(me_ptr->topo.gu.log_id, DBG_LOW_PRIO, "Processing frames in the ASYNC signal context");

   /** handle any events raised in the async signal context callback before processing frames. */
   gen_cntr_handle_fwk_events_in_data_path(me_ptr);

   gen_cntr_data_process_frames(me_ptr);

   /** handle if there are any pending ctrl msgs after processing data. data process frames can return early and
       not handle ctrl msgs, hence make sure to handle pending ctrl msgs if data is not processed. */
   cu_poll_and_process_ctrl_msgs(&me_ptr->cu);

   return result;
}