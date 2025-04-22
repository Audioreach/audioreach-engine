/**
 * \file apm_graph_utils.c
 *
 * \brief
 *     This file contains Stubbed Implementation of APM Graph
 *     Management Utilities
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/**==============================================================================
   Global Defines
==============================================================================*/

#include "apm_internal.h"

ar_result_t apm_update_cont_graph(apm_graph_info_t *graph_info_ptr,
                                  apm_container_t * src_cont_node_ptr,
                                  apm_container_t * dst_cont_node_ptr,
                                  bool_t            retain_sorted_cont_graph)
{
   return AR_EOK;
}

ar_result_t apm_update_cont_graph_list(apm_graph_info_t *graph_info_ptr)
{
   return AR_EOK;
}

ar_result_t apm_remove_cont_from_graph(apm_graph_info_t *graph_info_ptr, apm_container_t *container_node_ptr)
{
   /** Remove this container from the standalone container list */
   if (FALSE ==
       spf_list_find_delete_node(&graph_info_ptr->standalone_cont_list_ptr, container_node_ptr, TRUE /*pool_used*/))
   {
      AR_MSG(DBG_MED_PRIO, "APM Remove Node: Target node not present in the list");
   }

   return AR_EOK;
}

ar_result_t apm_remove_sg_from_cont_graph(apm_graph_info_t *graph_info_ptr, apm_sub_graph_t *target_sg_node_ptr)
{
   return AR_EOK;
}

ar_result_t apm_gm_cmd_get_cont_graph_node(apm_cmd_ctrl_t *apm_cmd_ctrl_ptr, spf_list_node_t **graph_node_pptr)
{
   return AR_EOK;
}

ar_result_t apm_set_up_graph_list_traversal(apm_graph_info_t *graph_info_ptr, apm_cmd_ctrl_t *apm_cmd_ctrl_ptr)
{
   ar_result_t result = AR_EOK;

   /** Initialize the graph mgmt command control state */
   apm_cmd_ctrl_ptr->graph_mgmt_cmd_ctrl.cont_proc_info.curr_cont_node_ptr = graph_info_ptr->standalone_cont_list_ptr;

   return result;
}

ar_result_t apm_gm_cmd_get_next_cont_in_sorted_list(apm_cmd_ctrl_t *  apm_cmd_ctrl_ptr,
                                                    apm_container_t **container_node_pptr)
{
   ar_result_t      result = AR_EOK;
   spf_list_node_t *curr_cont_node_ptr;

   /** Init the return container pointer */
   *container_node_pptr = NULL;

   /** Get current container in the graph being processed */
   curr_cont_node_ptr = apm_cmd_ctrl_ptr->graph_mgmt_cmd_ctrl.cont_proc_info.curr_cont_node_ptr;

   if (!curr_cont_node_ptr)
   {
      return AR_EOK;
   }

   /** Save the current container being processed */
   apm_cmd_ctrl_ptr->graph_mgmt_cmd_ctrl.cont_proc_info.curr_cont_node_ptr = curr_cont_node_ptr->next_ptr;

   /** Save the container node return pointer */
   *container_node_pptr = (apm_container_t *)curr_cont_node_ptr->obj_ptr;

   return result;
}
