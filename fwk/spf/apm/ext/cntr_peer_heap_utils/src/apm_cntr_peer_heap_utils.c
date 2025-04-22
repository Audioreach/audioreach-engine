/**
 * \file apm_cntr_peer_heap_utils.c
 *
 * \brief
 *
 *     This file contains implementation of
        APM Peer Heap Propagation utilities.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/****************************************************************************
 * INCLUDE HEADER FILES                                                    *
 ****************************************************************************/

#include "apm_internal.h"

/**==============================================================================
   Function Declaration
==============================================================================*/

ar_result_t apm_cache_mxd_heap_id_cntr_link_in_db(apm_cont_cmd_ctrl_t *cont_cmd_ctrl_ptr,
                                                  void *               curr_conn_cfg_ptr,
                                                  uint32_t             peer_heap_id,
                                                  bool_t               is_data_link);

void apm_check_and_handle_mixed_heap_id_cntr_links(apm_module_t **module_node_ptr_list,
                                                   uint32_t *     peer_heap_id_ptr_arr,
                                                   bool_t *       is_mixed_heap_data_link_ptr,
                                                   bool_t         is_data_link);

/**==============================================================================
   Global Defines
==============================================================================*/

apm_cntr_peer_heap_utils_vtable_t cntr_peer_heap_utils_funcs = { .apm_cache_mxd_heap_id_cntr_link_in_db_fptr =
                                                                    apm_cache_mxd_heap_id_cntr_link_in_db,

                                                                 .apm_check_and_handle_mixed_heap_id_cntr_links_fptr =
                                                                    apm_check_and_handle_mixed_heap_id_cntr_links };

ar_result_t apm_cache_mxd_heap_id_cntr_link_in_db(apm_cont_cmd_ctrl_t *cont_cmd_ctrl_ptr,
                                                  void *               curr_conn_cfg_ptr,
                                                  uint32_t             peer_heap_id,
                                                  bool_t               is_data_link)
{
   apm_mxd_heap_id_link_cfg_t *obj_ptr =
      (void *)posal_memory_malloc(sizeof(apm_mxd_heap_id_link_cfg_t), APM_INTERNAL_STATIC_HEAP_ID);

   if (NULL == obj_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "MXD_HEAP_LINK_PARSE: Failed to malloc mxd heap link structure");
      return AR_ENOMEMORY;
   }

   obj_ptr->conn_ptr        = curr_conn_cfg_ptr;
   obj_ptr->heap_id.heap_id = peer_heap_id;

   spf_list_node_t **cont_cached_mxd_heap_id_list_pptr = NULL;
   uint32_t *        num_mxd_heap_id_links_ptr         = NULL;
   uint32_t          arr_idx                           = is_data_link ? LINK_TYPE_DATA : LINK_TYPE_CTRL;

   /** Pointer to this container's running list of lp-default connections to be
   sent to containers as part of GRAPH OPEN command */
   cont_cached_mxd_heap_id_list_pptr =
      &cont_cmd_ctrl_ptr->cached_cfg_params.graph_open_params.mxd_heap_id_links_cfg[arr_idx].list_ptr;

   /** Total number of mixed heap connections configured for the host container */
   num_mxd_heap_id_links_ptr =
      &cont_cmd_ctrl_ptr->cached_cfg_params.graph_open_params.mxd_heap_id_links_cfg[arr_idx].num_nodes;

   /** Add the config structure to the running list of module
    *  connections to be sent to containers */
   return apm_db_add_node_to_list(cont_cached_mxd_heap_id_list_pptr, obj_ptr, num_mxd_heap_id_links_ptr);
}

void apm_check_and_handle_mixed_heap_id_cntr_links(apm_module_t **module_node_ptr_list,
                                                   uint32_t *     peer_heap_id_ptr_arr,
                                                   bool_t *       is_mixed_heap_data_link_ptr,
                                                   bool_t         is_data_link)
{
   enum
   {
      PEER_1_MODULE = 0,
      PEER_2_MODULE = 1
   };
   *is_mixed_heap_data_link_ptr = FALSE;

   // cache peer 2 heap in idx=0
   peer_heap_id_ptr_arr[PEER_1_MODULE] = module_node_ptr_list[PEER_2_MODULE]->host_cont_ptr->prop.heap_id;

   // cache peer 1 heap in idx=1
   peer_heap_id_ptr_arr[PEER_2_MODULE] = module_node_ptr_list[PEER_1_MODULE]->host_cont_ptr->prop.heap_id;

   /* Containers with default heap id need to be informed that they are connected to a module in a non-default heap
    * container*/
   if (peer_heap_id_ptr_arr[PEER_1_MODULE] != peer_heap_id_ptr_arr[PEER_2_MODULE])
   {
      *is_mixed_heap_data_link_ptr = TRUE;

      AR_MSG(DBG_HIGH_PRIO,
             "apm_gpr_cmd_parser: Found mixed-heap inter-container [1:data/0:control] links %lu between miid 0x%lx and "
             "miid "
             "and 0x%lx",
             is_data_link,
             module_node_ptr_list[PEER_1_MODULE]->instance_id,
             module_node_ptr_list[PEER_2_MODULE]->instance_id);
   }
}

ar_result_t apm_cntr_peer_heap_utils_init(apm_t *apm_info_ptr)
{
   ar_result_t result = AR_EOK;

   apm_info_ptr->ext_utils.cntr_peer_heap_utils_vtbl_ptr = &cntr_peer_heap_utils_funcs;

   return result;
}
