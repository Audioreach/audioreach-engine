/**
 * \file gen_cntr_pure_st_topo_handler_island.c
 * \brief
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "gen_cntr_i.h"

/* Checks if pure signal triggered data process frames can be used for signal triggered containers.
   If there is any module with active trigger policy then it uses generic topology, else it uses
   pure signal triggered topology. */
ar_result_t gen_cntr_check_and_assign_st_data_process_fn(gen_cntr_t *me_ptr)
{
   // if the container is already downgraded to generic topology then no need to check further.
   if (!gen_cntr_is_pure_signal_triggered(me_ptr))
   {
      return AR_EOK;
   }

   uint32_t num_data_tpm   = 0;
   uint32_t num_signal_tpm = 0;
   for (gu_sg_list_t *sg_list_ptr = me_ptr->topo.gu.sg_list_ptr; (NULL != sg_list_ptr); LIST_ADVANCE(sg_list_ptr))
   {
      bool_t is_sg_state_started = (TOPO_SG_STATE_STARTED == gen_topo_get_sg_state(sg_list_ptr->sg_ptr)) ? TRUE : FALSE;

      // checking SG state as well. Stopped SG may be getting handled in the parallel thread simultaneously.
      if (!is_sg_state_started)
      {
         continue;
      }
      for (gu_module_list_t *module_list_ptr = sg_list_ptr->sg_ptr->module_list_ptr; (NULL != module_list_ptr);
           LIST_ADVANCE(module_list_ptr))
      {
         gen_topo_module_t *module_ptr = (gen_topo_module_t *)module_list_ptr->module_ptr;
         if (gen_topo_is_module_data_trigger_policy_active(module_ptr))
         {
            num_data_tpm++;
         }

         if (gen_topo_is_module_signal_trigger_policy_active(module_ptr))
         {
            // todo: if signal trigger policy is same as default module trigger policy as container then dont increment
            // num_signal_tpm in the case module can still be pure signal triggered
            num_signal_tpm++;
         }
      }
   }

   // container can be a pure signal triggered, if it never had a module with Data TP or Signal TP.
   if (num_signal_tpm || !(me_ptr->topo.flags.is_signal_triggered && me_ptr->topo.flags.is_signal_triggered_active))
   {
      me_ptr->topo.flags.cannot_be_pure_signal_triggered = TRUE;
   }

   // if the container frame size is more than 5ms then it cannot be Pure signal triggered,
   // we hold buffers in Pure ST, so its not recommended to use for higher frame lengths since it can
   // lead to high Island memory consumption.
   uint32_t frame_duration_ms = capi_cmn_divide(me_ptr->cu.cntr_frame_len.frame_len_us , 1000);
   if (frame_duration_ms > PERF_MODE_LOW_POWER_FRAME_DURATION_MS)
   {
      me_ptr->topo.flags.cannot_be_pure_signal_triggered = TRUE;
   }

   GEN_CNTR_MSG(me_ptr->topo.gu.log_id,
                DBG_LOW_PRIO,
                "This is pure signal trigger container=%lu, signal_triggered:%lu num_data_tpm:%lu num_signal_tpm:%lu ",
                gen_cntr_is_pure_signal_triggered(me_ptr),
                me_ptr->topo.flags.is_signal_triggered,
                num_data_tpm,
                num_signal_tpm);
   return AR_EOK;
}
