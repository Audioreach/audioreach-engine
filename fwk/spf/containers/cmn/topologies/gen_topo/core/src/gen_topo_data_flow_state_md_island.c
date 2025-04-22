/**
 * \file gen_topo_data_flow_state_md_island.c
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

// Always use gen_topo_handle_data_flow_begin first.
ar_result_t gen_topo_handle_data_flow_begin_util_(gen_topo_t *            topo_ptr,
                                                  gen_topo_common_port_t *cmn_port_ptr,
                                                  gu_cmn_port_t *         gu_cmn_port_ptr)
{
   ar_result_t result = AR_EOK;

   // As soon as data buffer comes in, change the data flow.
   cmn_port_ptr->data_flow_state = TOPO_DATA_FLOW_STATE_FLOWING;

   // once data flow starts, port will move out of reset state.
   cmn_port_ptr->flags.port_is_not_reset = TRUE;

   if (topo_ptr->gen_topo_vtable_ptr->update_module_info)
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

   /** To notify SYNC module about the data flow from this port,
    * so that it can disables the threshold beforehand.
    */
   gen_topo_handle_fwk_ext_at_dfs(topo_ptr, gu_cmn_port_ptr);

   return result;
}

ar_result_t gen_topo_handle_data_flow_end(gen_topo_t *            topo_ptr,
                                          gen_topo_common_port_t *cmn_port_ptr,
                                          gu_cmn_port_t *         gu_cmn_port_ptr)
{
   ar_result_t result = AR_EOK;
   if (TOPO_DATA_FLOW_STATE_AT_GAP == cmn_port_ptr->data_flow_state)
   {
      return result;
   }
   cmn_port_ptr->data_flow_state = TOPO_DATA_FLOW_STATE_AT_GAP;

   if(topo_ptr->gen_topo_vtable_ptr->update_module_info)
   {
       topo_ptr->gen_topo_vtable_ptr->update_module_info(gu_cmn_port_ptr->module_ptr);
   }

#ifdef DATA_FLOW_STATE_DEBUG
   gen_topo_module_t *module_ptr = (gen_topo_module_t *)gu_cmn_port_ptr->module_ptr;
   TOPO_MSG_ISLAND(topo_ptr->gu.log_id,
                  DBG_LOW_PRIO,
                  SPF_LOG_PREFIX "data_flow_state:Module 0x%lX, port 0x%lx, state data-at-gap",
                  module_ptr->gu.module_instance_id,
                  gu_cmn_port_ptr->id);
#endif
   return result;
}
