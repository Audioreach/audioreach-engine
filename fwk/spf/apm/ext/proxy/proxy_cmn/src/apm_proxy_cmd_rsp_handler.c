/**
 * \file apm_proxy_cmd_rsp_handler.c
 * \brief
 *
 *     This file contains APM proxy manager command response handling utility functions
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/****************************************************************************
 * INCLUDE HEADER FILES                                                     *
 ****************************************************************************/

#include "apm_proxy_utils.h"
#include "apm_msg_utils.h"
#include "apm_cmd_sequencer.h"
#include "apm_cmd_utils.h"
#include "apm_proxy_def.h"
#include "apm_msg_rsp_handler.h"
#include "irm_api.h"
#include "amdb_api.h"
#include "apm_proxy_vcpm_utils.h"
#include "apm_internal_if.h"

/*****************************************************************
 **********************RSP Handlers*******************************
 *****************************************************************/

ar_result_t apm_aggregate_proxy_manager_cmd_response(apm_t *apm_info_ptr, spf_msg_t *rsp_msg_ptr)
{
   ar_result_t result = AR_EOK;

   spf_msg_header_t *msg_payload_ptr;

   apm_cmd_ctrl_t *      apm_curr_cmd_ctrl = NULL;
   apm_proxy_cmd_ctrl_t *proxy_cmd_ctrl    = NULL;

   /** Get the message payload pointer */
   msg_payload_ptr = (spf_msg_header_t *)rsp_msg_ptr->payload_ptr;

   proxy_cmd_ctrl = (apm_proxy_cmd_ctrl_t *)msg_payload_ptr->token.token_ptr;

   apm_curr_cmd_ctrl = (apm_cmd_ctrl_t *)proxy_cmd_ctrl->apm_cmd_ctrl_ptr;

   /** Update the current APM cmd corresponding to current
    *  response in process */
   apm_info_ptr->curr_cmd_ctrl_ptr = apm_curr_cmd_ctrl;

   /** Cache the response result */
   proxy_cmd_ctrl->rsp_ctrl.rsp_result = msg_payload_ptr->rsp_result;

   /** If the response message buffer is not expected to be
    *  used, releaes it now */

   /** Increment the response received counter */
   apm_curr_cmd_ctrl->rsp_ctrl.num_proxy_resp_rcvd++;

   /** Check if the return opcode is ETERMINATED, it should not
    *  be treated as failed response */
   if (AR_ETERMINATED != msg_payload_ptr->rsp_result)
   {
      /** Aggreate response result */
      apm_curr_cmd_ctrl->rsp_ctrl.rsp_status |= msg_payload_ptr->rsp_result;

      /** Increment the failed response counter for any error code */
      if (AR_EOK != msg_payload_ptr->rsp_result)
      {
         /** Increment the failed response counter */
         apm_curr_cmd_ctrl->rsp_ctrl.num_proxy_rsp_failed++;
      }
   }

   if (SPF_MSG_RSP_PROXY_MGMT_PERMISSION == rsp_msg_ptr->msg_opcode)
   {
      result = apm_graph_proxy_permission_cmd_rsp((spf_msg_rsp_proxy_permission_t *)&msg_payload_ptr->payload_start,
                                                  apm_curr_cmd_ctrl,
                                                  proxy_cmd_ctrl);
      if (result != AR_EOK)
      {
         return result;
      }

   } /** End of if (SPF_MSG_RSP_PROXY_MGMT_PERMISSION == rsp_msg_ptr->msg_opcode) */

   AR_MSG(DBG_MED_PRIO,
          "APM Proxy RSP HDLR: proxy result [0x%lX], num_proxy_cmds_issued[%lu], num_proxy_resp_rcvd[%lu]",
          msg_payload_ptr->rsp_result,
          apm_curr_cmd_ctrl->rsp_ctrl.num_proxy_cmd_issued,
          apm_curr_cmd_ctrl->rsp_ctrl.num_proxy_resp_rcvd);

   if (apm_curr_cmd_ctrl->rsp_ctrl.num_proxy_resp_rcvd == apm_curr_cmd_ctrl->rsp_ctrl.num_proxy_cmd_issued)
   {
      apm_curr_cmd_ctrl->rsp_ctrl.proxy_resp_pending = FALSE;

      if (APM_CMD_GET_CFG == apm_curr_cmd_ctrl->cmd_opcode)
      {
         /** Cache aggregate rsp_status instead of
          *  moving to cmd_status and clearing */
         apm_curr_cmd_ctrl->agg_rsp_status = apm_curr_cmd_ctrl->rsp_ctrl.rsp_status;
      }

      /** If any of the proxy managers returned error, update the
       *  overall command status.
       *  NOTE: We don't want to do this for GET_CFG, hence the elif */
      else if (0 != apm_curr_cmd_ctrl->rsp_ctrl.num_proxy_rsp_failed)
      {
         /** If no error occured previously, update the command status
          *  with first error received */
         if (AR_EOK == apm_curr_cmd_ctrl->cmd_status)
         {
            apm_curr_cmd_ctrl->cmd_status = apm_curr_cmd_ctrl->rsp_ctrl.rsp_status;
         }
         else if (apm_curr_cmd_ctrl->cmd_status != apm_curr_cmd_ctrl->rsp_ctrl.rsp_status)
         {
            /** Cmd is failed already and next container response is also
             *  failed but with a different error code, then overall
             *  failure code is EFAILED. Client can parse the error code
             *  in each PID to identify individual failures */
            apm_curr_cmd_ctrl->cmd_status = AR_EFAILED;
         }

         /** Set the current seq up status   */
         apm_curr_cmd_ctrl->cmd_seq.curr_op_seq_ptr->status = apm_curr_cmd_ctrl->cmd_status;
      }

      /** Clear the response pending flag */
      apm_curr_cmd_ctrl->rsp_ctrl.cmd_rsp_pending = FALSE;
   }

   spf_msg_return_msg(rsp_msg_ptr);

   return result;
}

ar_result_t apm_proxy_free_broadcast_payload(apm_t *apm_info_ptr, apm_proxy_cmd_ctrl_t *proxy_cmd_ctrl)
{
   ar_result_t          result             = AR_EOK;
   apm_proxy_manager_t *host_proxy_mgr_ptr = (apm_proxy_manager_t *)proxy_cmd_ctrl->host_proxy_mgr_ptr;

   /** Only applicable for broadcast module, AMDB and IRM */
   if ((IRM_MODULE_INSTANCE_ID == host_proxy_mgr_ptr->proxy_instance_id) ||
       (AMDB_MODULE_INSTANCE_ID == host_proxy_mgr_ptr->proxy_instance_id))
   {
      spf_list_node_t *node_ptr = proxy_cmd_ctrl->cached_cfg_params.proxy_cfg_params.param_data_list_ptr;
      for (; NULL != node_ptr; LIST_ADVANCE(node_ptr))
      {
         if (node_ptr->obj_ptr)
         {
            // NOTE: pbm TODO: free based on opcode instead of param id (like open close etc)
            apm_module_param_data_t *mod_data_ptr = (apm_module_param_data_t *)node_ptr->obj_ptr;
            /** Free the payload only if the module id matches with the proxy and for
             * APM_PARAM_ID_SATELLITE_PD_INFO param id*/
            if ((mod_data_ptr->module_instance_id == host_proxy_mgr_ptr->proxy_instance_id) &&
                ((mod_data_ptr->param_id == APM_PARAM_ID_SATELLITE_PD_INFO) ||
                 ((mod_data_ptr->param_id == APM_PARAM_ID_SET_CNTR_HANDLES) ||
                  (mod_data_ptr->param_id == APM_PARAM_ID_RESET_CNTR_HANDLES))))
            {
               apm_proxy_cfg_params_t *proxy_cfg_params_ptr = &proxy_cmd_ctrl->cached_cfg_params.proxy_cfg_params;

               /** Remove Proxy Manager node from apm graph info list. */
               apm_db_remove_node_from_list(&proxy_cfg_params_ptr->param_data_list_ptr,
                                            node_ptr->obj_ptr,
                                            &proxy_cfg_params_ptr->num_mod_param_cfg);
               posal_memory_free(node_ptr->obj_ptr);
            }
         }
      }
   }
   return result;
}

ar_result_t apm_proxy_cmn_cmd_rsp_hdlr(apm_t *apm_info_ptr, spf_msg_t *rsp_msg_ptr)
{
   ar_result_t           result = AR_EOK;
   apm_cmd_ctrl_t *      apm_cmd_ctrl_ptr;
   spf_msg_header_t *    msg_payload_ptr    = (spf_msg_header_t *)rsp_msg_ptr->payload_ptr;
   apm_proxy_cmd_ctrl_t *proxy_cmd_ctrl_ptr = (apm_proxy_cmd_ctrl_t *)msg_payload_ptr->token.token_ptr;

   /** Aggregate container response */
   apm_aggregate_proxy_manager_cmd_response(apm_info_ptr, rsp_msg_ptr);

   /** Get the pointer to current APM command control object */
   apm_cmd_ctrl_ptr = apm_info_ptr->curr_cmd_ctrl_ptr;

   /** Check and free the payload in the rsp msg, if they we allocated as part of broadcast param */
   result = apm_proxy_free_broadcast_payload(apm_info_ptr, proxy_cmd_ctrl_ptr);

   /** If no response active corresponding from Proxy mangers */
   if (!apm_cmd_ctrl_ptr->rsp_ctrl.proxy_resp_pending)
   {
      /** Clear the list of active managers */
      apm_clear_active_proxy_list(apm_info_ptr, apm_cmd_ctrl_ptr);
   }

   return result;
}

ar_result_t apm_proxy_manager_response_handler(apm_t *apm_info_ptr, spf_msg_t *rsp_msg_ptr)
{
   ar_result_t result = AR_EOK;
   // spf_msg_header_t *msg_payload_ptr;

   // apm_cmd_ctrl_t *      apm_curr_cmd_ctrl = NULL;
   // apm_proxy_cmd_ctrl_t *proxy_cmd_ctrl    = NULL;

   /** Get the message payload pointer */
   // msg_payload_ptr = (spf_msg_header_t *)rsp_msg_ptr->payload_ptr;

   // proxy_cmd_ctrl = (apm_proxy_cmd_ctrl_t *)msg_payload_ptr->token.token_ptr;

   // apm_curr_cmd_ctrl = (apm_cmd_ctrl_t *)proxy_cmd_ctrl->apm_cmd_ctrl_ptr;

   AR_MSG(DBG_MED_PRIO, "APM RSP HDLR: Received response/event Opcode[0x%lX]", rsp_msg_ptr->msg_opcode);

   // move to VCPM response handler
   switch (rsp_msg_ptr->msg_opcode)
   {
      /** For Proxy Managers, these are usually failure responses.*/
      case SPF_MSG_CMD_GRAPH_PREPARE:
      case SPF_MSG_CMD_GRAPH_START:
      case SPF_MSG_CMD_GRAPH_STOP:
      case SPF_MSG_CMD_GRAPH_CLOSE:
      {
         result = apm_graph_proxy_mgmt_cmd_rsp_hdlr(apm_info_ptr, rsp_msg_ptr);
         break;
      }

      /** Graph Mgmt command response from Proxy maangers.*/
      case SPF_MSG_RSP_PROXY_MGMT_PERMISSION:
      {
         result = apm_graph_proxy_permission_rsp_hndlr(apm_info_ptr, rsp_msg_ptr);
         break;
      }

      case SPF_MSG_CMD_PROXY_GRAPH_INFO:
      {
         result = apm_graph_proxy_graph_info_rsp_hdlr(apm_info_ptr, rsp_msg_ptr);
         break;
      }

      // This is the only common branch
      case SPF_MSG_CMD_SET_CFG:
      case SPF_MSG_CMD_GET_CFG:
      case SPF_MSG_CMD_REGISTER_CFG:
      case SPF_MSG_CMD_DEREGISTER_CFG:
      {
         result = apm_proxy_cmn_cmd_rsp_hdlr(apm_info_ptr, rsp_msg_ptr);
         break;
      }

      default:
      {
         AR_MSG(DBG_ERROR_PRIO,
                "APM RSP HDLR: Received unknown msg response/event Opcode[0x%lX]",
                rsp_msg_ptr->msg_opcode);
         result = AR_EUNSUPPORTED;
         break;
      }
   }

   return result;
}
