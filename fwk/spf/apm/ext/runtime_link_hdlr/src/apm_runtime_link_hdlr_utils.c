/**
 * \file apm_runtime_link_hdlr_utils.c
 *
 * \brief
 *     This file contains APM Link Open Handling across started subgraphs
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/****************************************************************************
 * INCLUDE HEADER FILES                                                     *
 ****************************************************************************/

#include "apm_internal.h"
#include "apm_cmd_utils.h"

/**==============================================================================
   Function Declaration
==============================================================================*/

ar_result_t apm_check_and_cache_link_to_start(apm_t *                apm_info_ptr,
                                              apm_module_t **        module_node_ptr_list,
                                              void *                 link_cfg_ptr,
                                              spf_module_link_type_t link_type,
                                              bool_t *               link_start_reqd_ptr);

ar_result_t apm_graph_open_handle_link_start(apm_t *apm_info_ptr);

/**==============================================================================
   Global Defines
==============================================================================*/

apm_runtime_link_hdlr_utils_vtable_t runtime_link_hdlr_util_funcs = { .apm_check_and_cache_link_to_start_fptr =
                                                                         apm_check_and_cache_link_to_start,

                                                                      .apm_graph_open_handle_link_start_fptr =
                                                                         apm_graph_open_handle_link_start };

ar_result_t apm_check_and_cache_link_to_start(apm_t *                apm_info_ptr,
                                              apm_module_t **        module_node_ptr_list,
                                              void *                 link_cfg_ptr,
                                              spf_module_link_type_t link_type,
                                              bool_t *               link_start_reqd_ptr)
{
   ar_result_t result = AR_EOK;
   apm_list_t *data_ctrl_link_list_ptr;

   enum
   {
      PEER_1_MODULE = 0,
      PEER_2_MODULE = 1
   };

   /** Init the return pointer */
   *link_start_reqd_ptr = FALSE;

   /** For GRAPH OPEN command, if the host sub-graphs for source
    *  and destination modules in the data/ctrl link have already
    *  been started, then cache this link for starting once the
    *  graph open is finished */
   if ((APM_SG_STATE_STARTED == module_node_ptr_list[PEER_1_MODULE]->host_sub_graph_ptr->state) &&
       (APM_SG_STATE_STARTED == module_node_ptr_list[PEER_2_MODULE]->host_sub_graph_ptr->state))
   {
      /** If the link is opened within a running sub-graph, that is
       *  an error case. */
      if (module_node_ptr_list[PEER_1_MODULE]->host_sub_graph_ptr->sub_graph_id ==
          module_node_ptr_list[PEER_2_MODULE]->host_sub_graph_ptr->sub_graph_id)
      {
         AR_MSG(DBG_ERROR_PRIO,
                "Link type[%lu 0:data/1:ctrl] open un-supported within in a single sub-graph in STARTED state, "
                "peer1[MIID, SGID]: "
                "[0x%lX, 0x%lX], peer2[MIID, "
                "SGID]: [0x%lX, 0x%lX] ",
                link_type,
                module_node_ptr_list[PEER_1_MODULE]->instance_id,
                module_node_ptr_list[PEER_1_MODULE]->host_sub_graph_ptr->sub_graph_id,
                module_node_ptr_list[PEER_2_MODULE]->instance_id,
                module_node_ptr_list[PEER_2_MODULE]->host_sub_graph_ptr->sub_graph_id);

         return AR_EFAILED;
      }

      /** Execution falls through if the link open is across 2
       *  different sub-graphs in STARTED state */

      /** Now check if the link is opened in a single container.
       *  link open within container is currently not supported,
       *  hence erroring out for this case. This check can be
       *  removed once containers have been updated to handle link
       *  open across started sub-graphs. */
      if (module_node_ptr_list[PEER_1_MODULE]->host_cont_ptr->container_id ==
          module_node_ptr_list[PEER_2_MODULE]->host_cont_ptr->container_id)
      {
         AR_MSG(DBG_ERROR_PRIO,
                "Link type[%lu 0:data/1:ctrl] open un-supported within a single container, peer1[MIID, CONT_ID]: "
                "[0x%lX, 0x%lX], peer2[MIID, "
                "CONT_ID]: [0x%lX, 0x%lX] ",
                link_type,
                module_node_ptr_list[PEER_1_MODULE]->instance_id,
                module_node_ptr_list[PEER_1_MODULE]->host_cont_ptr->container_id,
                module_node_ptr_list[PEER_2_MODULE]->instance_id,
                module_node_ptr_list[PEER_2_MODULE]->host_cont_ptr->container_id);

         return AR_EFAILED;
      }

      /** Execution falls through if the link open is across
       *  different started sub-graphs across different containers */

      AR_MSG(DBG_HIGH_PRIO,
             "Link type[%lu 0:data/1:ctrl] open between started sub-graphs, peer1[MIID, SGID]: "
             "[0x%lX, 0x%lX], peer2[MIID, "
             "SGID]: [0x%lX, 0x%lX] ",
             link_type,
             module_node_ptr_list[PEER_1_MODULE]->instance_id,
             module_node_ptr_list[PEER_1_MODULE]->host_sub_graph_ptr->sub_graph_id,
             module_node_ptr_list[PEER_2_MODULE]->instance_id,
             module_node_ptr_list[PEER_2_MODULE]->host_sub_graph_ptr->sub_graph_id);

      /** Get the pointer to data/ctrl link list pointer   */
      data_ctrl_link_list_ptr = &apm_info_ptr->curr_cmd_ctrl_ptr->graph_open_cmd_ctrl.data_ctrl_link_list[link_type];

      /** Cache this link as it needs to be started after open
       *  processing is done */
      if (AR_EOK != (result = apm_db_add_node_to_list(&data_ctrl_link_list_ptr->list_ptr,
                                                      link_cfg_ptr,
                                                      &data_ctrl_link_list_ptr->num_nodes)))
      {
         AR_MSG(DBG_ERROR_PRIO, "apm_check_and_cache_link_to_start: Failed to add data/ctrl link to the list");
      }
      else
      {
         /** Set the flag to indicate this opened link needs start
          *  processing as well */
         *link_start_reqd_ptr = TRUE;
      }
   }

   return result;
}

static ar_result_t apm_graph_mgmt_cache_links_for_start_at_open(apm_t *apm_info_ptr)
{
   ar_result_t             result = AR_EOK;
   apm_graph_info_t *      graph_info_ptr;
   apm_cmd_ctrl_t *        apm_cmd_ctrl_ptr;
   apm_cont_cmd_ctrl_t *   cont_cmd_ctrl_ptr;
   uint32_t                num_containers;
   spf_list_node_t **      list_pptr;
   uint32_t *              node_cntr_ptr;
   apm_container_t *       curr_cont_obj_ptr;
   apm_module_t *          module_node_ptr_list[2];
   uint32_t                link_port_id_list[2];
   spf_list_node_t *       curr_node_ptr;
   uint32_t                peer1_miid, peer2_miid;
   bool_t                  DANGLING_LINK_NOT_ALLOWED = FALSE;

   apm_module_conn_cfg_t *     data_link_cfg_ptr;
   apm_module_ctrl_link_cfg_t *ctrl_link_cfg_ptr;
   spf_module_port_type_t      port_type;

   enum
   {
      PEER_1 = 0,
      PEER_2 = 1,
   };

   /** Get the pointer to graph db   */
   graph_info_ptr = &apm_info_ptr->graph_info;

   /** Get the pointer to current command control */
   apm_cmd_ctrl_ptr = apm_info_ptr->curr_cmd_ctrl_ptr;

   for (uint32_t link_idx = 0; link_idx < LINK_TYPE_MAX; link_idx++)
   {
      /** Get the pointer to list of data/ctrl links   */
      if (NULL == (curr_node_ptr = apm_cmd_ctrl_ptr->graph_open_cmd_ctrl.data_ctrl_link_list[link_idx].list_ptr))
      {
         continue;
      }

      /** Iterate over the list of cached data/ctrl link  */
      while (curr_node_ptr)
      {
         if (LINK_TYPE_DATA == link_idx)
         {
            data_link_cfg_ptr         = (apm_module_conn_cfg_t *)curr_node_ptr->obj_ptr;
            peer1_miid                = data_link_cfg_ptr->src_mod_inst_id;
            link_port_id_list[PEER_1] = data_link_cfg_ptr->src_mod_op_port_id;

            peer2_miid                = data_link_cfg_ptr->dst_mod_inst_id;
            link_port_id_list[PEER_2] = data_link_cfg_ptr->dst_mod_ip_port_id;
         }
         else /** Control link */
         {
            ctrl_link_cfg_ptr = (apm_module_ctrl_link_cfg_t *)curr_node_ptr->obj_ptr;

            peer1_miid                = ctrl_link_cfg_ptr->peer_1_mod_iid;
            link_port_id_list[PEER_1] = ctrl_link_cfg_ptr->peer_1_mod_ctrl_port_id;

            peer2_miid                = ctrl_link_cfg_ptr->peer_2_mod_iid;
            link_port_id_list[PEER_2] = ctrl_link_cfg_ptr->peer_2_mod_ctrl_port_id;
         }

         /** Validate the module instance pair if they exist */
         if (AR_EOK != (result = apm_validate_module_instance_pair(graph_info_ptr,
                                                                   peer1_miid,
                                                                   peer2_miid,
                                                                   module_node_ptr_list,
                                                                   DANGLING_LINK_NOT_ALLOWED)))
         {
            AR_MSG(DBG_ERROR_PRIO, "apm_graph_mgmt_cache_links_for_start_at_open: Module IID validation failed");

            return result;
         }

         num_containers = 1;

         /** If host container ID's are different then need to send
          *  message to both the containers */
         if ((module_node_ptr_list[PEER_1]->host_cont_ptr->container_id) !=
             (module_node_ptr_list[PEER_2]->host_cont_ptr->container_id))
         {
            num_containers = 2;
         }

         if (AR_EOK != (result = apm_validate_and_cache_link_for_state_mgmt(apm_info_ptr,
                                                                            &apm_cmd_ctrl_ptr->graph_mgmt_cmd_ctrl,
                                                                            module_node_ptr_list)))
         {
            return result;
         }

         for (uint32_t cntr_idx = 0; cntr_idx < num_containers; cntr_idx++)
         {
            curr_cont_obj_ptr = module_node_ptr_list[cntr_idx]->host_cont_ptr;

            /** Get container's cmd ctrl obj for current APM command in
             *  process */
            if (AR_EOK != (result = apm_get_cont_cmd_ctrl_obj(curr_cont_obj_ptr, apm_cmd_ctrl_ptr, &cont_cmd_ctrl_ptr)))
            {
               AR_MSG(DBG_ERROR_PRIO, " Failed to get port handle, skipping");

               return result;
            }

            /** If the data link is present within the container, there
             *  will be no port handles. Need to send the link as is to the
             *  container for closing. The link information is not stored
             *  in the container cached config but rather as command
             *  params because as per the sub-graph state for the host
             *  module, they might not be processed in the first
             *  iteration of the close command. */
            if ((module_node_ptr_list[PEER_1]->host_cont_ptr->container_id) ==
                (module_node_ptr_list[PEER_2]->host_cont_ptr->container_id))
            {
               if (LINK_TYPE_DATA == link_idx)
               {
                  list_pptr = &cont_cmd_ctrl_ptr->cached_cfg_params.graph_mgmt_params.mod_data_link_cfg_list_ptr;

                  node_cntr_ptr = &cont_cmd_ctrl_ptr->cached_cfg_params.graph_mgmt_params.num_data_links;
               }
               else /** Control link */
               {
                  list_pptr = &cont_cmd_ctrl_ptr->cached_cfg_params.graph_mgmt_params.mod_ctrl_link_cfg_list_ptr;

                  node_cntr_ptr = &cont_cmd_ctrl_ptr->cached_cfg_params.graph_mgmt_params.num_ctrl_links;
               }

               /** Cache this data/ctrl link for state management */
               apm_db_add_node_to_list(list_pptr, curr_node_ptr->obj_ptr, node_cntr_ptr);
            }
            else /** Link is across 2 containers */
            {
               if (LINK_TYPE_DATA == link_idx)
               {
                  if (PEER_1 == cntr_idx)
                  {
                     port_type = PORT_TYPE_DATA_OP;
                  }
                  else if (PEER_2 == cntr_idx)
                  {
                     port_type = PORT_TYPE_DATA_IP;
                  }
               }
               else /** Control link */
               {
                  port_type = PORT_TYPE_CTRL_IO;
               }

                if (AR_EOK != (result = apm_search_and_cache_cont_port_hdl(curr_cont_obj_ptr,
                                                                          cont_cmd_ctrl_ptr,
                                                                          module_node_ptr_list[cntr_idx]->instance_id,
                                                                          link_port_id_list[cntr_idx],
                                                                          port_type,
                                                                          apm_cmd_ctrl_ptr->cmd_opcode)))
               {
                  AR_MSG(DBG_ERROR_PRIO,
                         "apm_graph_mgmt_cache_links_for_start_at_open(): Failed to cache port handle for CONT_ID[0x%lX]"
                         "M_IID[0x%lX], port_id[0x%lx]",
                         curr_cont_obj_ptr->container_id,
                         module_node_ptr_list[cntr_idx]->instance_id,
                         link_port_id_list[cntr_idx]);

                  return result;
               }

            } /** End of if-else (link across 2 containers) */

         } /** End for (host container list ) */

         /** Advance to next node in the list   */
         curr_node_ptr = curr_node_ptr->next_ptr;

      } /** End of while( data/ctrl link list) */

   } /** End of for (link index) */

   return result;
}

ar_result_t apm_graph_open_handle_link_start(apm_t *apm_info_ptr)
{
   ar_result_t     result = AR_EOK;
   apm_cmd_ctrl_t *cmd_ctrl_ptr;

   cmd_ctrl_ptr = apm_info_ptr->curr_cmd_ctrl_ptr;

   switch (cmd_ctrl_ptr->cmd_seq.graph_open_seq.op_idx)
   {
      case APM_OPEN_CMD_OP_HDL_LINK_OPEN_INFO:
      {
         result = apm_graph_mgmt_cache_links_for_start_at_open(apm_info_ptr);

         break;
      }
      case APM_OPEN_CMD_OP_HDL_LINK_START:
      {
         cmd_ctrl_ptr->cmd_seq.graph_mgmt_seq.op_idx = APM_GM_CMD_OP_PROCESS_REG_GRAPH;
         result = apm_cmd_graph_mgmt_cmn_sequencer(apm_info_ptr);

         /** If graph management sequence is completed, clear the
          *  pending flag for current close all operation */
         cmd_ctrl_ptr->cmd_seq.graph_open_seq.curr_cmd_op_pending =
            cmd_ctrl_ptr->cmd_seq.graph_mgmt_seq.curr_cmd_op_pending;

         break;
      }
      default:
      {
         result = AR_EUNSUPPORTED;

         AR_MSG(DBG_ERROR_PRIO,
                "apm_graph_open_handle_link_start: Unsupported op_idx[%lu], "
                "cmd_opcode[0x%08lx]",
                cmd_ctrl_ptr->cmd_seq.graph_open_seq.op_idx,
                cmd_ctrl_ptr->cmd_opcode);

         break;
      }
   }

   return result;
}

ar_result_t apm_runtime_link_hdlr_utils_init(apm_t *apm_info_ptr)
{
   ar_result_t result = AR_EOK;

   apm_info_ptr->ext_utils.runtime_link_hdlr_vtbl_ptr = &runtime_link_hdlr_util_funcs;

   return result;
}
