/**
 * \file apm_proxy_vcpm_cmd_rsp_handler.c
 * \brief
 *
 *     This file contains APM's vcpm proxy manager command response handling utility functions
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/****************************************************************************
 * INCLUDE HEADER FILES                                                     *
 ****************************************************************************/

#include "apm_msg_utils.h"
#include "apm_cmd_sequencer.h"
#include "apm_cmd_utils.h"
#include "apm_proxy_def.h"
#include "apm_msg_rsp_handler.h"
#include "apm_proxy_utils.h"
#include "apm_proxy_vcpm_utils_i.h"
#include "apm_proxy_vcpm_utils.h"

/****************************************************************************
 * GLOBALS                                                                  *
 ****************************************************************************/

/****************************************************************************
 * Function Declarations
 ****************************************************************************/

ar_result_t apm_proxy_graph_mgmt_cmd_hdlr(apm_t *apm_info_ptr, spf_msg_t *msg_ptr, bool_t *cmd_def_check_reqd_ptr)
{
   ar_result_t     result            = AR_EOK;
   apm_cmd_ctrl_t *apm_curr_cmd_ctrl = NULL;

   *cmd_def_check_reqd_ptr = TRUE;

   apm_graph_info_t *graph_info_ptr         = &apm_info_ptr->graph_info;
   spf_list_node_t * proxy_manager_list_ptr = graph_info_ptr->proxy_manager_list_ptr;

   /** Get the start pointer to graph mgmt msg header */
   spf_msg_header_t *              msg_header_ptr  = (spf_msg_header_t *)msg_ptr->payload_ptr;
   spf_msg_cmd_proxy_graph_mgmt_t *msg_payload_ptr = (spf_msg_cmd_proxy_graph_mgmt_t *)&msg_header_ptr->payload_start;

   apm_curr_cmd_ctrl = apm_info_ptr->curr_cmd_ctrl_ptr;

   /** Validate if the proxy manager corresponding to sub-graph
    *  ID list is present in APM book keeping */
   if (AR_EOK != (result = apm_proxy_util_validate_input_sg_list(proxy_manager_list_ptr,
                                                                 msg_payload_ptr->sg_array_ptr,
                                                                 msg_payload_ptr->num_sgs)))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "apm_proxy_graph_mgmt_cmd_hdlr: Input validation Failed"
             " for CMD ID 0x%08X, result %d ",
             msg_ptr->msg_opcode,
             result);

      return result;
   }

   /** Validate if the sub-graph ID list is present in APM book
    *  keeping structure, if present, add them to the list of
    *  regular sub-graphs */
   if (AR_EOK != (result = apm_gm_cmd_validate_sg_list(apm_info_ptr,
                                                       msg_payload_ptr->num_sgs,
                                                       msg_payload_ptr->sg_array_ptr,
                                                       apm_curr_cmd_ctrl->cmd_opcode)))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "apm_proxy_graph_mgmt_cmd_hdlr: Input validation Failed"
             " for CMD ID 0x%08X, result %d ",
             msg_ptr->msg_opcode,
             result);

      return result;
   }

   /** Cache sub-graph list info  */
   apm_curr_cmd_ctrl->graph_mgmt_cmd_ctrl.sg_list.num_cmd_sg_id      = msg_payload_ptr->num_sgs;
   apm_curr_cmd_ctrl->graph_mgmt_cmd_ctrl.sg_list.cmd_sg_id_list_ptr = msg_payload_ptr->sg_array_ptr;

    /** Command deferral check required for direct commands */
   *cmd_def_check_reqd_ptr = msg_payload_ptr->is_direct_cmd;

   return result;
}

ar_result_t apm_graph_proxy_mgmt_cmd_rsp_hdlr(apm_t *apm_info_ptr, spf_msg_t *rsp_msg_ptr)
{
   ar_result_t     result = AR_EOK;
   apm_cmd_ctrl_t *apm_cmd_ctrl_ptr;

   /** Aggregate container response */
   apm_aggregate_proxy_manager_cmd_response(apm_info_ptr, rsp_msg_ptr);

   /** Get apm cmd ctrl pointer.*/
   apm_cmd_ctrl_ptr = apm_info_ptr->curr_cmd_ctrl_ptr;

   /** If containers response pending, just return */
   if (apm_cmd_ctrl_ptr->rsp_ctrl.cmd_rsp_pending)
   {
      return AR_EOK;
   }

   /** Clear the active proxy manager list, if non-empty */
   if (apm_cmd_ctrl_ptr->rsp_ctrl.active_proxy_mgr_list_ptr)
   {
      apm_clear_active_proxy_list(apm_info_ptr, apm_cmd_ctrl_ptr);
   }

   /** If any of the Proxy Manager returned error, update the
    *  overall command status to EFAILED */
   if (AR_EOK != apm_cmd_ctrl_ptr->rsp_ctrl.rsp_status)
   {
      apm_cmd_ctrl_ptr->cmd_status = AR_EFAILED;

      //      apm_cmd_ctrl_ptr->cmd_pending = FALSE;

      AR_MSG(DBG_HIGH_PRIO,
             "GRAPH MGMT RSP: Completed the container msg seq for "
             "cmd_opcode[0x%08lx], cmd_status[0x%lX]",
             apm_cmd_ctrl_ptr->cmd_opcode,
             apm_cmd_ctrl_ptr->cmd_status);
   }

   return result;
}

ar_result_t apm_graph_proxy_permission_cmd_rsp(spf_msg_rsp_proxy_permission_t *proxy_response,
                                               apm_cmd_ctrl_t *                apm_curr_cmd_ctrl,
                                               apm_proxy_cmd_ctrl_t *          proxy_cmd_ctrl)
{
   /** Receiving this response message indicates that the Proxy manager processed the command successfully.
    Hence, update the command and response control flags accordingly.*/

   uint32_t            num_permitted_sgs = proxy_response->num_permitted_subgraphs;
   apm_sub_graph_id_t *rsp_sg_array      = proxy_response->permitted_sg_array_ptr;

   uint32_t mem_size = sizeof(apm_sub_graph_id_t) * num_permitted_sgs;

   if (num_permitted_sgs)
   {
      apm_sub_graph_id_t *cache_sg_array_ptr = NULL;
      if (NULL == (cache_sg_array_ptr = (apm_sub_graph_id_t *)posal_memory_malloc(mem_size, APM_INTERNAL_STATIC_HEAP_ID)))
      {
         AR_MSG(DBG_ERROR_PRIO, "APM Proxy RSP HDLR:: Failed to allocate memory, for Proxy permission rsp");

         return AR_ENOMEMORY;
      }

      for (uint32_t i = 0; i < num_permitted_sgs; i++)
      {
         AR_MSG(DBG_HIGH_PRIO,
                "APM Proxy RSP HDLR:: Received permission for SG ID 0x%08X, cmd_id 0x%08X",
                rsp_sg_array[i].sub_graph_id,
                apm_curr_cmd_ctrl->cmd_opcode);

         cache_sg_array_ptr[i].sub_graph_id = rsp_sg_array[i].sub_graph_id;
      }

      /** Save the Subgraph array for processing the graph Mgmt command.*/
      proxy_cmd_ctrl->rsp_ctrl.num_permitted_subgraphs = num_permitted_sgs;
      proxy_cmd_ctrl->rsp_ctrl.permitted_sg_array_ptr  = cache_sg_array_ptr;

      apm_db_add_node_to_list(&apm_curr_cmd_ctrl->rsp_ctrl.permitted_proxy_mgr_list_ptr,
                              proxy_cmd_ctrl->host_proxy_mgr_ptr,
                              &apm_curr_cmd_ctrl->rsp_ctrl.num_permitted_proxy_mgrs);

      apm_curr_cmd_ctrl->rsp_ctrl.permitted_proxy_sgs_pending = TRUE;

   } /** End of if (num_permitted_sgs) */
   return AR_EOK;
}

ar_result_t apm_graph_proxy_permission_rsp_hndlr(apm_t *apm_info_ptr, spf_msg_t *rsp_msg_ptr)
{
   ar_result_t       result = AR_EOK;
   //spf_msg_header_t *msg_payload_ptr;
   apm_cmd_ctrl_t *  apm_curr_cmd_ctrl = NULL;

   /** Get the message payload pointer */
   //msg_payload_ptr = (spf_msg_header_t *)rsp_msg_ptr->payload_ptr;

   /** Aggregate container response */
   apm_aggregate_proxy_manager_cmd_response(apm_info_ptr, rsp_msg_ptr);

   apm_curr_cmd_ctrl = apm_info_ptr->curr_cmd_ctrl_ptr;

   /** If the response is pending from proxy manager, return and
    *  wait for response */
   if (apm_curr_cmd_ctrl->rsp_ctrl.proxy_resp_pending)
   {
      return AR_EOK;
   }

   /** Increment the proxy msg opcode index */
   // apm_curr_cmd_ctrl->proxy_msg_opcode.proxy_opcode_idx++;

   /** Execution falls through if responses have been received
    *  from all the proxy managers. Now check if permission
    *  received for at least one sub-graph. If not,  clear the
    *  list of pending proxy mgr list */
   if (!apm_curr_cmd_ctrl->rsp_ctrl.permitted_proxy_sgs_pending)
   {
      AR_MSG(DBG_MED_PRIO,
             "PROXY GRAPH MGMT RSP: Permitted proxy sg list is empty "
             "cmd_opcode[0x%08lx], cmd_status[0x%lX]",
             apm_curr_cmd_ctrl->cmd_opcode,
             apm_curr_cmd_ctrl->cmd_status);

      goto end_proxy_mgmt_cmd;
   }

   /** Process pending sub-graphs  */
   if (apm_curr_cmd_ctrl->rsp_ctrl.permitted_proxy_sgs_pending)
   {
      /** Process Containers related to Proxy subgraphs. Move them
       *  to proxy to regular list for further processing */
      if (AR_EOK != (result = apm_proxy_util_process_pending_subgraphs(apm_info_ptr, apm_curr_cmd_ctrl)))
      {
         apm_curr_cmd_ctrl->cmd_status  = result;
         apm_curr_cmd_ctrl->cmd_pending = FALSE;

         AR_MSG(DBG_ERROR_PRIO,
                "PROXY GRAPH MGMT RSP: Failed to send message to Containers for "
                "cmd_opcode[0x%08lx], cmd_status[0x%lX]",
                apm_curr_cmd_ctrl->cmd_opcode,
                apm_curr_cmd_ctrl->cmd_status);
      }
   }

   apm_gm_cmd_clear_proxy_mgr_sg_info(apm_info_ptr);

   return result;

end_proxy_mgmt_cmd:
   /** Done with sending all proxy commands.
      Clear the pending proxy manager list, if non-empty */
   if (apm_curr_cmd_ctrl->rsp_ctrl.active_proxy_mgr_list_ptr)
   {
      /** Clear the list of pending managers */
      apm_clear_active_proxy_list(apm_info_ptr, apm_curr_cmd_ctrl);
   }

   return result;
}

ar_result_t apm_graph_proxy_graph_info_rsp_hdlr(apm_t *apm_info_ptr, spf_msg_t *rsp_msg_ptr)
{
   ar_result_t     result = AR_EOK;
   apm_cmd_ctrl_t *apm_cmd_ctrl_ptr;

   /** Aggregate container response */
   apm_aggregate_proxy_manager_cmd_response(apm_info_ptr, rsp_msg_ptr);

   /** Get apm cmd ctrl pointer.*/
   apm_cmd_ctrl_ptr = apm_info_ptr->curr_cmd_ctrl_ptr;

   /** If containers response pending, just return */
   if (apm_cmd_ctrl_ptr->rsp_ctrl.cmd_rsp_pending)
   {
      return AR_EOK;
   }

   if (AR_EOK == apm_cmd_ctrl_ptr->rsp_ctrl.rsp_status)
   {
      apm_proxy_util_update_proxy_manager(apm_cmd_ctrl_ptr);

      /** Clear the list of active managers */
      apm_clear_active_proxy_list(apm_info_ptr, apm_cmd_ctrl_ptr);
   }
   else
   {
      /** If any of the Proxy Manager returned error, update the
       *  overall command status to EFAILED */

      /** Clear the list of active managers */
      apm_clear_active_proxy_list(apm_info_ptr, apm_cmd_ctrl_ptr);

      apm_cmd_ctrl_ptr->cmd_status = AR_EFAILED;

      /** Change the overall command status */
      apm_cmd_ctrl_ptr->graph_open_cmd_ctrl.cmd_status = APM_OPEN_CMD_STATE_OPEN_FAIL;
   }

   return result;
}
