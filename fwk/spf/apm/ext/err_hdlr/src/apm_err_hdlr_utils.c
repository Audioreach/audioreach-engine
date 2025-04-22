/**
 * \file apm_err_hdlr_utils.c
 *
 * \brief
 *     This file contains utility functions for error handling
 *     during command processing
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
#include "apm_proxy_vcpm_utils.h"

/**==============================================================================
   Function Declaration
==============================================================================*/

ar_result_t apm_cmd_graph_open_err_hdlr_sequencer(apm_t *apm_info_ptr);

ar_result_t apm_populate_cont_open_cmd_err_hdlr_seq(apm_t *apm_info_ptr);

ar_result_t apm_err_hdlr_cache_container_rsp(apm_t *apm_info_ptr);

ar_result_t apm_err_hdlr_clear_cont_cached_graph_open_cfg(apm_cont_cached_cfg_t *cached_cfg_ptr,
                                                          apm_cmd_ctrl_t        *apm_cmd_ctrl_ptr);

/**==============================================================================
   Global Defines
==============================================================================*/

apm_err_hdlr_utils_vtable_t err_hdlr_util_funcs = {
   .apm_cmd_graph_open_err_hdlr_sequencer_fptr = apm_cmd_graph_open_err_hdlr_sequencer,

   .apm_populate_cont_open_cmd_err_hdlr_seq_fptr = apm_populate_cont_open_cmd_err_hdlr_seq,

   .apm_err_hdlr_cache_container_rsp_fptr = apm_err_hdlr_cache_container_rsp,

   .apm_err_hdlr_clear_cont_cached_graph_open_cfg_fptr = apm_err_hdlr_clear_cont_cached_graph_open_cfg,
};

/* Create list of subgraph ids to be closed */
static ar_result_t apm_prepare_sg_close_payload(apm_t *apm_info_ptr)
{

   ar_result_t      result = AR_EOK;
   apm_cmd_ctrl_t  *cmd_ctrl_ptr;
   apm_sub_graph_t *sg_obj_ptr;

   /** Get the pointer to current cmd control object  */
   cmd_ctrl_ptr = apm_info_ptr->curr_cmd_ctrl_ptr;

   /** If sub-graph and/or container list is empty, or no
    *  sub-graphs were opened as part of open command, only data
    *  and/or control links */
   if (!apm_info_ptr->graph_info.num_sub_graphs || !apm_info_ptr->graph_info.num_containers ||
       !cmd_ctrl_ptr->graph_open_cmd_ctrl.num_sub_graphs)
   {
      AR_MSG(DBG_HIGH_PRIO, "apm_prepare_sg_close_payload(): Sub-graph and/or container list is empty");

      return AR_EOK;
   }

   /* Add each subgraph id to the payload */
   spf_list_node_t *curr_ptr = cmd_ctrl_ptr->graph_open_cmd_ctrl.sg_id_list_ptr;

   while (curr_ptr)
   {
      sg_obj_ptr = (apm_sub_graph_t *)curr_ptr->obj_ptr;

      /** Force the sub-graph state to be stopped. Due to the error
       *  it might not have been opened or partially opened in some
       *  containers. For the close routine to be called, the
       *  assumed state is STOPPED for clean up.  */
      sg_obj_ptr->state = APM_SG_STATE_STOPPED;

      apm_db_add_node_to_list(&cmd_ctrl_ptr->graph_mgmt_cmd_ctrl.sg_list.reg_sg_list_ptr,
                              sg_obj_ptr,
                              &cmd_ctrl_ptr->graph_mgmt_cmd_ctrl.sg_list.num_reg_sub_graphs);

      /** Advance to next node in the list */
      curr_ptr = curr_ptr->next_ptr;
   }

   /** If there are any sub-graphs present managed via proxy
    *  manager, separate them out from the regular sub-graph
    *  list.
    *  This is during graph open failure. So, graph info is not sent to VCPM.
    *  Hence while closing the graphs, there is no need to take permission from VCPM. */

   // if (AR_EOK != (result = apm_proxy_util_sort_graph_mgmt_sg_lists(apm_info_ptr)))
   //{
   // AR_MSG(DBG_ERROR_PRIO,
   //"apm_prepare_sg_close_payload(): Failed to update grph_mgmt lists, cmd_opcode[0x%08lx]",
   // cmd_ctrl_ptr->cmd_opcode);
   // return result;
   //}

   return result;
}

static ar_result_t apm_clear_open_cmd_sg_list(apm_t *apm_info_ptr)
{
   ar_result_t      result = AR_EOK;
   apm_cmd_ctrl_t  *cmd_ctrl_ptr;
   spf_list_node_t *curr_node_ptr;
   spf_list_node_t *next_node_ptr;
   apm_sub_graph_t *graph_open_sg_obj_ptr;
   bool_t           node_found = FALSE;

   /** Get the pointer to current command control object   */
   cmd_ctrl_ptr = apm_info_ptr->curr_cmd_ctrl_ptr;

   /** Get the pointer to list of sub-graphs in current
    * graph open command */
   curr_node_ptr = cmd_ctrl_ptr->graph_open_cmd_ctrl.sg_id_list_ptr;

   while (curr_node_ptr)
   {
      /** Get the current sub-graph obj pointer   */
      graph_open_sg_obj_ptr = (apm_sub_graph_t *)curr_node_ptr->obj_ptr;

      /** Get the next node pointer   */
      next_node_ptr = curr_node_ptr->next_ptr;

      /** Delete this sub-graph object from the open command list   */
      spf_list_find_delete_node(&cmd_ctrl_ptr->graph_open_cmd_ctrl.sg_id_list_ptr,
                                graph_open_sg_obj_ptr,
                                TRUE /** POOL Used*/);

      /** Remove this sub-graph from the APM global sub-graph list */
      if (TRUE == (node_found = spf_list_find_delete_node(&apm_info_ptr->graph_info.sub_graph_list_ptr,
                                                          graph_open_sg_obj_ptr,
                                                          TRUE /** Pool used*/)))
      {
         /** Decrement sub-graph counter   */
         apm_info_ptr->graph_info.num_sub_graphs--;

         AR_MSG(DBG_HIGH_PRIO,
                "apm_clear_open_cmd_sg_list(): Removed SG_ID[0x%lX]",
                graph_open_sg_obj_ptr->sub_graph_id);

         /** Free up the memory of this sub-graph node object */
         posal_memory_free(graph_open_sg_obj_ptr);
      }

      /** Advance to next node in the list   */
      curr_node_ptr = next_node_ptr;

   } /** End of while (sub_graph list) */

   return result;
}

ar_result_t apm_graph_open_fail_hdlr_pre_process(apm_t *apm_info_ptr)
{
   ar_result_t     result = AR_EOK;
   apm_cmd_ctrl_t *cmd_ctrl_ptr;

   /** Get the pointer to current cmd ctrl obj  */
   cmd_ctrl_ptr = apm_info_ptr->curr_cmd_ctrl_ptr;

   switch (cmd_ctrl_ptr->graph_open_cmd_ctrl.cmd_status)
   {
      case APM_OPEN_CMD_STATE_CONT_CREATE_FAIL:
      {
         /** Clear the pending container list, if non-empty.
          *  This will also release the cached container resposne
          *  messages */
         if (cmd_ctrl_ptr->rsp_ctrl.pending_cont_list_ptr)
         {
            /** Clear the list of pending containers */
            result = apm_clear_container_list(cmd_ctrl_ptr,
                                              &cmd_ctrl_ptr->rsp_ctrl.pending_cont_list_ptr,
                                              &cmd_ctrl_ptr->rsp_ctrl.num_pending_container,
                                              APM_CONT_CACHED_CFG_RELEASE,
                                              APM_CONT_CMD_PARAMS_RELEASE,
                                              APM_PENDING_CONT_LIST);
         }
         break;
      }
      case APM_OPEN_CMD_STATE_OPEN_FAIL:
      case APM_OPEN_CMD_STATE_NORMAL:
      case APM_OPEN_CMD_STATE_CONNECT_FAIL:
      default:
      {
         /** Do nothing */
         break;
      }
   }

   if (AR_EOK != (result = apm_prepare_sg_close_payload(apm_info_ptr)))
   {
      AR_MSG(DBG_ERROR_PRIO, "apm_cmd_graph_open_err_hdlr_sequencer(): Failed to populate sub-graph list");
   }

   return result;
}

ar_result_t apm_prepare_cont_list_to_destroy(apm_cmd_ctrl_t *cmd_ctrl_ptr)
{
   ar_result_t      result = AR_EOK;
   spf_list_node_t *curr_node_ptr, *next_node_ptr, *temp_list_ptr = NULL;
   apm_container_t *cont_obj_ptr;
   uint32_t         num_cont = 1;

   /** Get the pointer to the list of pending containers   */
   curr_node_ptr = cmd_ctrl_ptr->rsp_ctrl.pending_cont_list_ptr;

   /** Iterate over the list of containers pending cmd
    *  processing */
   while (curr_node_ptr)
   {
      /** Get the pointer to current container onject   */
      cont_obj_ptr = (apm_container_t *)curr_node_ptr->obj_ptr;

      /** Get pointer to next node in the list   */
      next_node_ptr = curr_node_ptr->next_ptr;

      /** If the container is not newly created, remove it from the
       *  list of pending containers */
      if (!cont_obj_ptr->newly_created)
      {
         /** Insert this node to a temporary list  */
         if (AR_EOK !=
             (result = spf_list_insert_tail(&temp_list_ptr, cont_obj_ptr, APM_INTERNAL_STATIC_HEAP_ID, TRUE /** Pool Used*/)))
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "apm_prepare_cont_list_to_destroy(): Failed to add node to temp list, CONT_ID [0x%lX], result: "
                   "0x%lX",
                   cont_obj_ptr->container_id,
                   result);

            /** Continue   */
         }

         /** Clear cached configuration for this container   */
         if (AR_EOK != (result = apm_clear_container_list(cmd_ctrl_ptr,
                                                          &temp_list_ptr,
                                                          &num_cont,
                                                          APM_CONT_CACHED_CFG_RELEASE,
                                                          APM_CONT_CMD_PARAMS_RELEASE,
                                                          APM_PENDING_CONT_LIST)))
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "apm_prepare_cont_list_to_destroy(): Failed to clear container cached cfg, CONT_ID [0x%lX], result: "
                   "0x%lX",
                   cont_obj_ptr->container_id,
                   result);

            /** Continue   */
         }

         /** Remove this node from list of pending containers   */
         spf_list_delete_node_update_head(&curr_node_ptr,
                                          &cmd_ctrl_ptr->rsp_ctrl.pending_cont_list_ptr,
                                          TRUE /** Pool used*/);

         /** Decrement the number of pernding containers   */
         cmd_ctrl_ptr->rsp_ctrl.num_pending_container--;
      }

      /** Advance to next node in the list  */
      curr_node_ptr = next_node_ptr;
   }

   return result;
}

ar_result_t apm_cmd_graph_open_err_hdlr_sequencer(apm_t *apm_info_ptr)
{
   ar_result_t     result = AR_EOK;
   apm_cmd_ctrl_t *cmd_ctrl_ptr;
   apm_op_seq_t   *curr_op_seq_ptr;

   /** Get the current command control obj pointer */
   cmd_ctrl_ptr = apm_info_ptr->curr_cmd_ctrl_ptr;

   /** Set the current op seq for error handling  */
   cmd_ctrl_ptr->cmd_seq.curr_op_seq_ptr = &cmd_ctrl_ptr->cmd_seq.err_hdlr_seq;

   curr_op_seq_ptr = cmd_ctrl_ptr->cmd_seq.curr_op_seq_ptr;

   AR_MSG(DBG_HIGH_PRIO,
          "apm_cmd_graph_open_err_hdlr_sequencer(): Executing curr_op_idx[0x%lX], curr_seq_idx[%lu], cmd_opcode[0x%lX]",
          curr_op_seq_ptr->op_idx,
          curr_op_seq_ptr->curr_seq_idx,
          cmd_ctrl_ptr->cmd_opcode);

   switch (cmd_ctrl_ptr->cmd_seq.err_hdlr_seq.op_idx)
   {
      case APM_OPEN_CMD_OP_ERR_HDLR:
      {
         if (APM_OPEN_CMD_STATE_CONT_CREATE_FAIL == cmd_ctrl_ptr->graph_open_cmd_ctrl.cmd_status)
         {
            result = apm_prepare_cont_list_to_destroy(cmd_ctrl_ptr);
         }

         break;
      }
      case APM_OPEN_CMD_OP_HDL_CREATE_FAIL:
      {
         /** If container create failed, and some containers got
          *  created, destroy those containers */
         if (APM_OPEN_CMD_STATE_CONT_CREATE_FAIL == cmd_ctrl_ptr->graph_open_cmd_ctrl.cmd_status)
         {
            result = apm_cmd_cmn_sequencer(apm_info_ptr);

            /** Clear the list of sub-graphs opened   */
            if (!curr_op_seq_ptr->curr_cmd_op_pending)
            {
               apm_clear_open_cmd_sg_list(apm_info_ptr);
               apm_proxy_util_clear_vcpm_active_or_inactive_proxy(apm_info_ptr, cmd_ctrl_ptr);
            }
         }

         break;
      }
      case APM_OPEN_CMD_OP_PREPPROC_OPEN_CONNECT_FAIL:
      {
         result = apm_graph_open_fail_hdlr_pre_process(apm_info_ptr);

         break;
      }
      case APM_OPEN_CMD_HDLR_CLOSE_SG_LIST:
      {
         result = apm_cmd_graph_mgmt_sequencer(apm_info_ptr);

         /** If the graph close sequence is done, end this sequencer.
          *  Or if any further errors occured  */
         if ((APM_CMN_CMD_OP_COMPLETED == cmd_ctrl_ptr->cmd_seq.graph_mgmt_seq.op_idx) || (AR_EOK != result))
         {
            /** Set the op index to end this error handler sequencer   */
            curr_op_seq_ptr->op_idx = APM_OPEN_CMD_OP_ERR_HDLR_COMPLETED;
         }

         /** If the graph management sequencer is on going, op status
          *  will be pending. If the gm seq is done, then need to next
          *  op in the error handler. Pending flag need to be set to
          *  true so that sequencer can return back to the set op
          *  index */
         curr_op_seq_ptr->curr_cmd_op_pending = TRUE;

         break;
      }
      case APM_OPEN_CMD_OP_ERR_HDLR_COMPLETED:
      {
         /** Clearn graph open command sub-graph list, if present   */
         apm_clear_open_cmd_sg_list(apm_info_ptr);

         apm_proxy_util_clear_vcpm_active_or_inactive_proxy(apm_info_ptr, cmd_ctrl_ptr);

         /** Clear the curr op pending status   */
         apm_clear_curr_cmd_op_pending_status(cmd_ctrl_ptr->cmd_seq.curr_op_seq_ptr);

         /** End the error handler sequencer. Restore the curr seq obj to
          *  primary */
         apm_end_cmd_op_sequencer(cmd_ctrl_ptr, cmd_ctrl_ptr->cmd_seq.curr_op_seq_ptr);

         break;
      }
      default:
      {
         AR_MSG(DBG_ERROR_PRIO,
                "apm_cmd_graph_open_err_hdlr_sequencer(): Unexpected op_idx[0x%lx]",
                curr_op_seq_ptr->op_idx);

         result = AR_EFAILED;

         break;
      }
   }

   return result;
}

ar_result_t apm_populate_cont_open_cmd_err_hdlr_seq(apm_t *apm_info_ptr)
{
   ar_result_t            result = AR_EOK;
   apm_cmd_ctrl_t        *apm_cmd_ctrl_ptr;
   apm_cont_msg_opcode_t *cont_msg_opcode_ptr;

   /** Get the pointer to current command control object   */
   apm_cmd_ctrl_ptr = apm_info_ptr->curr_cmd_ctrl_ptr;

   /** Get the pointer to APM command control */
   cont_msg_opcode_ptr = &apm_cmd_ctrl_ptr->cont_msg_opcode;

   switch (apm_cmd_ctrl_ptr->cmd_seq.err_hdlr_seq.op_idx)
   {
      case APM_OPEN_CMD_OP_HDL_CREATE_FAIL:
      {
         /** If container creation failed and pending container list is
          *  non-empty, then need to send destroy command to created
          *  containers */
         if ((APM_OPEN_CMD_STATE_CONT_CREATE_FAIL == apm_cmd_ctrl_ptr->graph_open_cmd_ctrl.cmd_status) &&
             apm_cmd_ctrl_ptr->rsp_ctrl.pending_cont_list_ptr)
         {
            /** Need to send destroy command to created containers   */
            cont_msg_opcode_ptr->opcode_list[0] = SPF_MSG_CMD_DESTROY_CONTAINER;
         }

         break;
      }
      case APM_OPEN_CMD_OP_PREPPROC_OPEN_CONNECT_FAIL:
      case APM_OPEN_CMD_HDLR_CLOSE_SG_LIST:
      {
         result = apm_populate_cont_graph_mgmt_cmd_seq(apm_info_ptr);

         break;
      }
      default:
      {
         AR_MSG(DBG_ERROR_PRIO,
                "apm_populate_cont_open_cmd_err_hdlr_seq(): Unexpected op_idx[0x%lX], cmd_opcode[0x%08lX]",
                apm_cmd_ctrl_ptr->cmd_seq.err_hdlr_seq.op_idx,
                apm_cmd_ctrl_ptr->cmd_opcode);

         result = AR_EFAILED;
         break;
      }

   } /** End of switch (apm_cmd_ctrl_ptr->cmd_seq.curr_open_cmd_op_idx) */

   return result;
}

ar_result_t apm_err_hdlr_add_sg_to_port_hdl_sg_list(spf_module_port_conn_t *cont_rsp_ptr,
                                                    apm_cont_cmd_ctrl_t    *cont_cmd_ctrl_ptr)
{

   if (!cont_rsp_ptr || !cont_cmd_ctrl_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "apm_err_hdlr_add_sg_to_port_hdl_sg_list(): Invalid i/p args, cont_rsp_ptr[0x%lX], "
             "cont_cmd_ctrl_ptr[0x%lX]",
             cont_rsp_ptr,
             cont_cmd_ctrl_ptr);

      return AR_EFAILED;
   }

   uint32_t         host_sg_id   = cont_rsp_ptr->self_mod_port_hdl.sub_graph_id;
   apm_container_t *cont_obj_ptr = (apm_container_t *)cont_cmd_ctrl_ptr->host_container_ptr;
   apm_cmd_ctrl_t  *cmd_ctrl_ptr = (apm_cmd_ctrl_t *)cont_cmd_ctrl_ptr->apm_cmd_ctrl_ptr;
   apm_sub_graph_t *sg_obj_ptr;
   ar_result_t      result = AR_EOK;

   spf_list_node_t *curr_ptr = cont_obj_ptr->sub_graph_list_ptr;

   while (curr_ptr)
   {
      sg_obj_ptr = (apm_sub_graph_t *)curr_ptr->obj_ptr;

      if (sg_obj_ptr->sub_graph_id == host_sg_id)
      {

         /*SG state could be Invalid if failure occured before Open Command was completed.*/
         if (APM_SG_STATE_INVALID == sg_obj_ptr->state)
         {
            sg_obj_ptr->state = APM_SG_STATE_STOPPED;
         }

         if (AR_EOK !=
             (result =
                 apm_db_search_and_add_node_to_list(&cmd_ctrl_ptr->graph_mgmt_cmd_ctrl.sg_list
                                                        .cont_port_hdl_sg_list_ptr,
                                                    sg_obj_ptr,
                                                    &cmd_ctrl_ptr->graph_mgmt_cmd_ctrl.sg_list.num_cont_port_hdl_sg)))
         {
            return result;
         }

         AR_MSG(DBG_HIGH_PRIO,
                "apm_err_hdlr_add_sg_to_port_hdl_sg_list(): Searched and Added SG ID [0x%lX] to graph managment cmd "
                "ctrl for "
                "link failure clean up.",
                sg_obj_ptr->sub_graph_id);
      }

      curr_ptr = curr_ptr->next_ptr;
   }

   return result;
}

ar_result_t apm_err_hdlr_cache_module_port_conn(spf_module_port_conn_t *cont_rsp_ptr,
                                                apm_cont_cmd_ctrl_t    *cont_cmd_ctrl_ptr,
                                                spf_module_port_type_t  port_type,
                                                apm_port_cycle_type_t   port_cycle_type)
{
   ar_result_t             result = AR_EOK;
   spf_module_port_conn_t *module_port_info_ptr;

   module_port_info_ptr =
      (spf_module_port_conn_t *)posal_memory_malloc(sizeof(spf_module_port_conn_t), APM_INTERNAL_STATIC_HEAP_ID);

   if (!module_port_info_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "apm_err_hdlr_allocate_module_port_conn(): Failed to allocate module port info memory");

      return AR_ENOMEMORY;
   }

   /** Copy the connection contents */
   memscpy(module_port_info_ptr, sizeof(spf_module_port_conn_t), cont_rsp_ptr, sizeof(spf_module_port_conn_t));

   result = apm_db_add_node_to_list(&cont_cmd_ctrl_ptr->cached_cfg_params.graph_mgmt_params
                                        .cont_ports[port_cycle_type][port_type]
                                        .list_ptr,
                                    module_port_info_ptr,
                                    &cont_cmd_ctrl_ptr->cached_cfg_params.graph_mgmt_params
                                        .cont_ports[port_cycle_type][port_type]
                                        .num_nodes);

   /* Cache the host sub-graphs for the corresponding port handles */
   result = apm_err_hdlr_add_sg_to_port_hdl_sg_list(cont_rsp_ptr, cont_cmd_ctrl_ptr);

   return result;
}

ar_result_t apm_err_hdlr_cache_cont_graph_open_rsp(apm_cmd_ctrl_t      *cmd_ctrl_ptr,
                                                   apm_container_t     *cont_obj_ptr,
                                                   apm_cont_cmd_ctrl_t *cont_cmd_ctrl_ptr)
{
   ar_result_t                   result = AR_EOK, cont_rsp;
   spf_msg_header_t             *msg_header_ptr;
   spf_cntr_port_connect_info_t *port_conn_msg_ptr;
   spf_list_node_t              *temp_list_ptr = NULL;
   uint32_t                      num_cont      = 1;
   spf_msg_t                     rsp_msg;
   spf_module_port_conn_t       *rsp_port_conn_list_ptr;

   /** Cache container response message  */
   rsp_msg = cont_cmd_ctrl_ptr->rsp_ctrl.rsp_msg;

   /** Clear the response message book keeping in container cmd
    *  ctrl obj, to avoid double free */
   memset(&cont_cmd_ctrl_ptr->rsp_ctrl.rsp_msg, 0, sizeof(spf_msg_t));

   /** Insert this node to a temporary list  */
   if (AR_EOK !=
       (result = spf_list_insert_tail(&temp_list_ptr, cont_obj_ptr, APM_INTERNAL_STATIC_HEAP_ID, TRUE /** Pool Used*/)))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "apm_err_hdlr_cache_cont_graph_open_rsp(): Failed to add node to temp list, CONT_ID [0x%lX], result: "
             "0x%lX",
             cont_obj_ptr->container_id,
             result);

      goto __bailout_cache_graph_open_rsp_params;
   }

   /** Cache container response  */
   cont_rsp = cont_cmd_ctrl_ptr->rsp_ctrl.rsp_result;

   /** Clear this container's cached configurations */
   if (AR_EOK != (result = apm_clear_container_list(cmd_ctrl_ptr,
                                                    &temp_list_ptr,
                                                    &num_cont,
                                                    APM_CONT_CACHED_CFG_RELEASE,
                                                    APM_CONT_CMD_PARAMS_RELEASE,
                                                    APM_PENDING_CONT_LIST)))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "apm_err_hdlr_cache_cont_graph_open_rsp(): Failed to clear container cached cfg, CONT_ID [0x%lX], result: "
             "0x%lX",
             cont_obj_ptr->container_id,
             result);

      goto __bailout_cache_graph_open_rsp_params;
   }

   /** Check if this container returned  failure response, then
    *  skip. Container should have taken care of clean up  */
   if (AR_EOK != cont_rsp)
   {
      /** This container returned error code, clean up is handled
       *  by the container */
      goto __bailout_cache_graph_open_rsp_params;
   }

   msg_header_ptr = (spf_msg_header_t *)rsp_msg.payload_ptr;

   /** Validate the message pointer */
   if (!msg_header_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "apm_err_hdlr_cache_cont_graph_open_rsp(): Cont rsp msg ptr is NULL, CONT_ID [0x%lX]",
             cont_obj_ptr->container_id);

      goto __bailout_cache_graph_open_rsp_params;
   }

   /** Get the pointer to start of the GRAPH OPEN response
    *  message payload for this container */
   port_conn_msg_ptr = (spf_cntr_port_connect_info_t *)&msg_header_ptr->payload_start;

   AR_MSG(DBG_HIGH_PRIO,
          "apm_err_hdlr_cache_cont_graph_open_rsp(): CONT_ID [0x%lX], NUM_IP_HDL "
          "[%lu], NUM_OP_HDL [%lu], NUM_CTRL_HDL [%lu]",
          cont_obj_ptr->container_id,
          port_conn_msg_ptr->num_ip_data_port_conn,
          port_conn_msg_ptr->num_op_data_port_conn,
          port_conn_msg_ptr->num_ctrl_port_conn);

   rsp_port_conn_list_ptr = port_conn_msg_ptr->ip_data_port_conn_list_ptr;

   /** Cache the input port handles, if present   */
   for (uint32_t conn_list_idx = 0; conn_list_idx < port_conn_msg_ptr->num_ip_data_port_conn; conn_list_idx++)
   {
      result |= apm_err_hdlr_cache_module_port_conn(&rsp_port_conn_list_ptr[conn_list_idx],
                                                    cont_cmd_ctrl_ptr,
                                                    PORT_TYPE_DATA_IP,
                                                    APM_PORT_TYPE_ACYCLIC);
   }

   rsp_port_conn_list_ptr = port_conn_msg_ptr->op_data_port_conn_list_ptr;

   /** Cache the output port handles, if present   */
   for (uint32_t conn_list_idx = 0; conn_list_idx < port_conn_msg_ptr->num_op_data_port_conn; conn_list_idx++)
   {
      result |= apm_err_hdlr_cache_module_port_conn(&rsp_port_conn_list_ptr[conn_list_idx],
                                                    cont_cmd_ctrl_ptr,
                                                    PORT_TYPE_DATA_OP,
                                                    APM_PORT_TYPE_ACYCLIC);
   }

   rsp_port_conn_list_ptr = port_conn_msg_ptr->ctrl_port_conn_list_ptr;

   /** Cache the control port handles, if present   */
   for (uint32_t conn_list_idx = 0; conn_list_idx < port_conn_msg_ptr->num_ctrl_port_conn; conn_list_idx++)
   {
      result |= apm_err_hdlr_cache_module_port_conn(&rsp_port_conn_list_ptr[conn_list_idx],
                                                    cont_cmd_ctrl_ptr,
                                                    PORT_TYPE_CTRL_IO,
                                                    APM_PORT_TYPE_ACYCLIC);
   }

__bailout_cache_graph_open_rsp_params:

   /** Return the response message packet  */
   result |= spf_msg_return_msg(&rsp_msg);

   return result;
}

ar_result_t apm_err_hdlr_cache_cont_per_cmd_rsp(apm_cmd_ctrl_t *cmd_ctrl_ptr, apm_container_t *cont_obj_ptr)
{
   ar_result_t          result            = AR_EOK;
   apm_cont_cmd_ctrl_t *cont_cmd_ctrl_ptr = NULL;

   /** Get the allocated cmd ctrl object for this container   */
   if (AR_EOK != (result = apm_get_allocated_cont_cmd_ctrl_obj(cont_obj_ptr, cmd_ctrl_ptr, &cont_cmd_ctrl_ptr)))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "apm_err_hdlr_cache_cont_per_cmd_rsp(): Failed to get allocated cmd ctrl obj for CONT_ID [0x%lX]",
             cont_obj_ptr->container_id);

      return result;
   }

   switch (cmd_ctrl_ptr->cmd_opcode)
   {
      case APM_CMD_GRAPH_OPEN:
      {
         result = apm_err_hdlr_cache_cont_graph_open_rsp(cmd_ctrl_ptr, cont_obj_ptr, cont_cmd_ctrl_ptr);

         break;
      }
      default:
      {
         AR_MSG(DBG_ERROR_PRIO,
                "apm_err_hdlr_cache_cont_per_cmd_rsp(): Unexpected cmd_opcode[0x%lX],  CONT_ID [0x%lX]",
                cmd_ctrl_ptr->cmd_opcode,
                cont_obj_ptr->container_id);

         result = AR_EUNEXPECTED;

         break;
      }
   }

   return result;
}

ar_result_t apm_err_hdlr_cache_container_rsp(apm_t *apm_info_ptr)
{
   ar_result_t      result = AR_EOK;
   spf_list_node_t *curr_node_ptr, *next_node_ptr;
   apm_container_t *curr_cont_obj_ptr;
   apm_cmd_ctrl_t  *cmd_ctrl_ptr;

   /** Get current cmd ctrl pointer  */
   cmd_ctrl_ptr = apm_info_ptr->curr_cmd_ctrl_ptr;

   /** Get the pointer to list of pending containers */
   curr_node_ptr = cmd_ctrl_ptr->rsp_ctrl.pending_cont_list_ptr;

   /** Itearte over the list of pending containers   */
   while (curr_node_ptr)
   {
      /** Get pointer to current container object   */
      curr_cont_obj_ptr = (apm_container_t *)curr_node_ptr->obj_ptr;

      /** Get the next list node pointer   */
      next_node_ptr = curr_node_ptr->next_ptr;

      /** Cache port handles opened by containers returning
       *  success for graph open */
      if (AR_EOK != (result |= apm_err_hdlr_cache_cont_per_cmd_rsp(cmd_ctrl_ptr, curr_cont_obj_ptr)))
      {
         AR_MSG(DBG_ERROR_PRIO,
                "apm_err_hdlr_cache_container_rsp(): Failed to cache rsp params for CONT_ID [0x%lX]",
                curr_cont_obj_ptr->container_id);

         /** Aggregate the error code and continue to clean up
          *  whatever possible */
      }

      /** Remove this container from the list of pending containers */
      spf_list_delete_node_update_head(&curr_node_ptr,
                                       &cmd_ctrl_ptr->rsp_ctrl.pending_cont_list_ptr,
                                       TRUE /** Pool used */);

      /** Update the pending container counter  */
      cmd_ctrl_ptr->rsp_ctrl.num_pending_container--;

      /** Advance to next node in the list   */
      curr_node_ptr = next_node_ptr;
   }

   return result;
}

ar_result_t apm_err_hdlr_clear_cont_cached_graph_open_cfg(apm_cont_cached_cfg_t *cached_cfg_ptr,
                                                          apm_cmd_ctrl_t        *apm_cmd_ctrl_ptr)
{
   ar_result_t result = AR_EOK;

   switch (apm_cmd_ctrl_ptr->cmd_seq.err_hdlr_seq.op_idx)
   {
      case APM_OPEN_CMD_OP_ERR_HDLR:
      case APM_OPEN_CMD_OP_HDL_CREATE_FAIL:
      case APM_OPEN_CMD_OP_PREPPROC_OPEN_CONNECT_FAIL:
      {
         result = apm_release_cont_cached_graph_open_cfg(cached_cfg_ptr);
         break;
      }
      case APM_OPEN_CMD_HDLR_CLOSE_SG_LIST:
      default:
      {
         result = apm_release_cont_cached_graph_mgmt_cfg(apm_cmd_ctrl_ptr, cached_cfg_ptr);
         break;
      }
   }

   return result;
}

ar_result_t apm_err_hdlr_utils_init(apm_t *apm_info_ptr)
{
   ar_result_t result = AR_EOK;

   apm_info_ptr->ext_utils.err_hdlr_vtbl_ptr = &err_hdlr_util_funcs;

   return result;
}
