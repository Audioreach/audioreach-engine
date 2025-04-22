/**
 * \file gen_topo_data_flow_state.c
 * \brief
 *     This file implements the data flow state
 *
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "gen_topo.h"


/**
 * Check if all sources, including external inputs and source modules, are FTRT and at gap and return TRUE if so.
 *
 * If so, no need to vote power resources until data flow starts.
 *
 * When requesting vote, check if all ext-in are FTRT (both US and DS) and don't vote (release) if none of them are
 * data-flow-started.
 * This helps in container not voting for FTRT containers (NT mode or after DAM) unless data flow starts
 *
 * return value can be used for releasing votes. TRUE => all ext-in and src are FTRT and at-gap, hence vote can be
 * released.
 */
bool_t gen_topo_check_if_all_src_are_ftrt_n_at_gap(gen_topo_t *topo_ptr, bool_t *is_ftrt_ptr)
{
   *is_ftrt_ptr = FALSE;

   if ((/* topo_ptr->flags.is_signal_triggered && */ topo_ptr->flags.is_signal_triggered_active))
   {
      return FALSE;
   }

   // Check if any input or output is RT, if so, return FALSE as voting needs to be done independent of data flow state.
   for (gu_ext_in_port_list_t *ext_in_port_list_ptr = topo_ptr->gu.ext_in_port_list_ptr; (NULL != ext_in_port_list_ptr);
        LIST_ADVANCE(ext_in_port_list_ptr))
   {
      gu_ext_in_port_t *     gu_ext_in_port_ptr     = ext_in_port_list_ptr->ext_in_port_ptr;
      gen_topo_input_port_t *in_port_ptr            = (gen_topo_input_port_t *)gu_ext_in_port_ptr->int_in_port_ptr;
      uint32_t               is_downstream_realtime = FALSE;
      uint32_t               is_upstream_realtime   = FALSE;
      gen_topo_get_port_property(topo_ptr,
                                 TOPO_DATA_INPUT_PORT_TYPE,
                                 PORT_PROPERTY_IS_UPSTREAM_RT,
                                 in_port_ptr,
                                 &is_upstream_realtime);
      gen_topo_get_port_property(topo_ptr,
                                 TOPO_DATA_INPUT_PORT_TYPE,
                                 PORT_PROPERTY_IS_DOWNSTREAM_RT,
                                 in_port_ptr,
                                 &is_downstream_realtime);

      if (is_downstream_realtime || is_upstream_realtime)
      {
         // both US and DS must not be RT for port to be FTRT
         return FALSE;
      }
   }

   // Since no US / DS of an input port is RT, and no STM, container is FTRT
   *is_ftrt_ptr = TRUE;

   // only for FTRT, data flow state change is relevant.

   for (gu_ext_in_port_list_t *ext_in_port_list_ptr = topo_ptr->gu.ext_in_port_list_ptr; (NULL != ext_in_port_list_ptr);
        LIST_ADVANCE(ext_in_port_list_ptr))
   {
      gu_ext_in_port_t      *gu_ext_in_port_ptr = ext_in_port_list_ptr->ext_in_port_ptr;
      gen_topo_input_port_t *in_port_ptr        = (gen_topo_input_port_t *)gu_ext_in_port_ptr->int_in_port_ptr;
      bool_t                 is_sg_state_started =
         (TOPO_SG_STATE_STARTED == gen_topo_get_sg_state(gu_ext_in_port_ptr->sg_ptr)) ? TRUE : FALSE;

      // checking SG state as well. because for self SG stop, the data flow state will be updated asynchronously while
      // SG state is updated synchronously.
      if (is_sg_state_started && TOPO_DATA_FLOW_STATE_FLOWING == in_port_ptr->common.data_flow_state)
      {
         // if any port has data flow started
         return FALSE;
      }
   }

   // check all sources to see if any one is enabled or not FTRT (signal triggered - already checked above)
   for (gu_sg_list_t *sg_list_ptr = topo_ptr->gu.sg_list_ptr; (NULL != sg_list_ptr); LIST_ADVANCE(sg_list_ptr))
   {
      bool_t is_sg_state_started = (TOPO_SG_STATE_STARTED == gen_topo_get_sg_state(sg_list_ptr->sg_ptr)) ? TRUE : FALSE;
      if (!is_sg_state_started)
      {
         continue;
      }

      for (gu_module_list_t *module_list_ptr = sg_list_ptr->sg_ptr->module_list_ptr; (NULL != module_list_ptr);
           LIST_ADVANCE(module_list_ptr))
      {
         gen_topo_module_t *module_ptr = (gen_topo_module_t *)module_list_ptr->module_ptr;
         if (module_ptr->gu.flags.is_source)
         {
            // if src module is enabled, then we need voting, cannot release.
            if (!module_ptr->flags.disabled)
            {
               return FALSE;
            }
            // otherwise, we can release only after dfs becomes stop (this allows us wait till EOS propagates out of src
            // module)
         }
      }
   }

   // Disabling of source modules is not sufficient. Source might have gotten disabled just now, but internal-EOS is yet
   // to reach ext-out port. The same can happen when EOS comes from ext-in as well.
   for (gu_ext_out_port_list_t *ext_out_port_list_ptr = topo_ptr->gu.ext_out_port_list_ptr;
        (NULL != ext_out_port_list_ptr);
        LIST_ADVANCE(ext_out_port_list_ptr))
   {
      gu_ext_out_port_t *     gu_ext_out_port_ptr = ext_out_port_list_ptr->ext_out_port_ptr;
      gen_topo_output_port_t *out_port_ptr        = (gen_topo_output_port_t *)gu_ext_out_port_ptr->int_out_port_ptr;
      bool_t                  is_sg_state_started =
         (TOPO_SG_STATE_STARTED == gen_topo_get_sg_state(gu_ext_out_port_ptr->sg_ptr)) ? TRUE : FALSE;

      // checking SG state as well. because for self SG stop, the data flow state will be updated asynchronously while
      // SG state is updated synchronously.
      if (is_sg_state_started && TOPO_DATA_FLOW_STATE_FLOWING == out_port_ptr->common.data_flow_state)
      {
         return FALSE;
      }
   }

   // All ext-in and sources are FTRT and data flow state stopped. There cannot be any container with no ext-in as well
   // as source modules.
   return TRUE;
}

//fwk extn related handling at data-flow start
void gen_topo_handle_fwk_ext_at_dfs_util_(gen_topo_t *topo_ptr, gu_cmn_port_t *cmn_port_ptr)
{
   if (topo_ptr->flags.is_sync_module_present)
   {
      gen_topo_module_t *module_ptr = NULL;

      if (gu_is_output_port_id(cmn_port_ptr->id))
      {
         gu_output_port_t *out_port_ptr = (gu_output_port_t *)cmn_port_ptr;

         if (out_port_ptr->conn_in_port_ptr)
         {
            module_ptr = (gen_topo_module_t *)out_port_ptr->conn_in_port_ptr->cmn.module_ptr;
         }
      }
      else
      {
         gu_input_port_t *in_port_ptr = (gu_input_port_t *)cmn_port_ptr;
         module_ptr                   = (gen_topo_module_t *)in_port_ptr->cmn.module_ptr;
      }

      // notify sync module that data flow is started.
      gen_topo_fwk_extn_sync_will_start(topo_ptr, module_ptr);
   }
}

/**
 * currently not called when modules raise MF. to avoid those changes, using only 2 states: flowing, at-gap.
 * MF or data buf sets to flowing state, whichever is first
 */
ar_result_t gen_topo_handle_data_flow_preflow(gen_topo_t *            topo_ptr,
                                              gen_topo_common_port_t *cmn_port_ptr,
                                              gu_cmn_port_t *         gu_cmn_port_ptr)
{
   ar_result_t            result  = AR_EOK;
   topo_data_flow_state_t ref_val = TOPO_DATA_FLOW_STATE_FLOWING; // TOPO_DATA_FLOW_STATE_PREFLOW
   if (ref_val == cmn_port_ptr->data_flow_state)
   {
      return result;
   }

   cmn_port_ptr->data_flow_state = ref_val;

   if(topo_ptr->gen_topo_vtable_ptr->update_module_info)
   {
       topo_ptr->gen_topo_vtable_ptr->update_module_info(gu_cmn_port_ptr->module_ptr);
   }

#ifdef DATA_FLOW_STATE_DEBUG
   gen_topo_module_t *module_ptr = (gen_topo_module_t *)gu_cmn_port_ptr->module_ptr;
   TOPO_MSG_ISLAND(topo_ptr->gu.log_id,
                  DBG_LOW_PRIO,
                  SPF_LOG_PREFIX "data_flow_state: Module 0x%lX, port 0x%lx, state data-flowing",
                  module_ptr->gu.module_instance_id,
                  gu_cmn_port_ptr->id);
#endif
   return result;
}