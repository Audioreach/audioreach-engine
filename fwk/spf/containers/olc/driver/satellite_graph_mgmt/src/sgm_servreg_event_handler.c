/**
 * \file sgm_cmd_handler.c
 * \brief
 *     This file contains Satellite Graph Management functions
 *     for the service registry down-notification handler.
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "sgm_i.h"
#include "apm.h"
/* =======================================================================
Static Function Definitions
========================================================================== */

ar_result_t sgm_set_servreg_error_notify_cmd_rsp_fn_handler(spgm_info_t *spgm_ptr,
                                                            void *       servreg_error_notify_rsp_vtbl_ptr)
{
   ar_result_t result = AR_EOK;
   uint32_t    log_id = 0;
   INIT_EXCEPTION_HANDLING

   VERIFY(result, (NULL != spgm_ptr));
   VERIFY(result, (NULL != servreg_error_notify_rsp_vtbl_ptr));
   // update the OLC response table function
   spgm_ptr->servreg_error_notify_cmd_rsp_vtbl = (sgmc_rsp_h_vtable_t *)servreg_error_notify_rsp_vtbl_ptr;

   CATCH(result, OLC_MSG_PREFIX, log_id)
   {
   }

   return result;
}

/* Function to handle the pending command responses for the down-notification from the service registry  */
static ar_result_t spgm_servreg_notify_cmd_rsp_handler(cu_base_t *           cu_ptr,
                                                       spgm_info_t *         spgm_ptr,
                                                       spgm_cmd_hndl_node_t *cmd_hndl_node_ptr)
{

   ar_result_t result = AR_EOK;
   uint32_t    log_id = 0;
   INIT_EXCEPTION_HANDLING

   spgm_cmd_rsp_node_t *rsp_info = &spgm_ptr->rsp_info;

   VERIFY(result, (NULL != spgm_ptr));
   log_id = spgm_ptr->sgm_id.log_id;
   VERIFY(result, (NULL != cu_ptr));
   VERIFY(result, (NULL != spgm_ptr->cmd_rsp_vtbl));

   OLC_SGM_MSG(OLC_SGM_ID,
               DBG_HIGH_PRIO,
               "ssr_pdr_cmd_rsp: handling for opcode(%lX) token(%lx) ",
               cmd_hndl_node_ptr->opcode,
               cmd_hndl_node_ptr->token);

   rsp_info->rsp_result = AR_EUNEXPECTED;
   rsp_info->opcode     = cmd_hndl_node_ptr->opcode;
   rsp_info->token      = cmd_hndl_node_ptr->token;
   rsp_info->cmd_msg    = &cmd_hndl_node_ptr->cmd_msg;

   switch (cmd_hndl_node_ptr->opcode)
   {
      case APM_CMD_GRAPH_OPEN:
      {
         TRY(result, spgm_ptr->servreg_error_notify_cmd_rsp_vtbl->graph_open_rsp_h(cu_ptr, rsp_info));
         break;
      }
      case APM_CMD_GRAPH_CLOSE:
      {
         // AKR: if you TRY, AR_ETERMINATED will raise an exception.
         result = spgm_ptr->servreg_error_notify_cmd_rsp_vtbl->graph_close_rsp_h(cu_ptr, rsp_info);
         break;
      }
      case APM_CMD_GRAPH_PREPARE:
      {
         TRY(result, spgm_ptr->servreg_error_notify_cmd_rsp_vtbl->graph_prepare_rsp_h(cu_ptr, rsp_info));
         break;
      }
      case APM_CMD_GRAPH_START:
      {
         TRY(result, spgm_ptr->servreg_error_notify_cmd_rsp_vtbl->graph_start_rsp_h(cu_ptr, rsp_info));
         break;
      }
      case APM_CMD_GRAPH_FLUSH:
      {
         TRY(result, spgm_ptr->servreg_error_notify_cmd_rsp_vtbl->graph_flush_rsp_h(cu_ptr, rsp_info));
         break;
      }
      case APM_CMD_GRAPH_STOP:
      {
         TRY(result, spgm_ptr->servreg_error_notify_cmd_rsp_vtbl->graph_stop_rsp_h(cu_ptr, rsp_info));
         break;
      }
      case APM_CMD_GET_CFG:
      {
         if (FALSE == cmd_hndl_node_ptr->is_apm_cmd_rsp)
         {
            // Handling the response to the case where get configuration is sent to the module instance.
            TRY(result, spgm_ptr->servreg_error_notify_cmd_rsp_vtbl->graph_set_get_cfg_packed_rsp_h(cu_ptr, rsp_info, NULL));
         }
         else
         {
            rsp_info->sec_opcode    = cmd_hndl_node_ptr->sec_opcode;
            rsp_info->cmd_extn_info = cmd_hndl_node_ptr->cmd_extn_info;
            TRY(result, spgm_ptr->servreg_error_notify_cmd_rsp_vtbl->graph_set_get_cfg_rsp_h(cu_ptr, rsp_info));
         }
         break;
      }

      case APM_CMD_SET_CFG:
      {
         if (FALSE == cmd_hndl_node_ptr->is_apm_cmd_rsp)
         {
            // Handling the response to the case where Set configuration is sent to the module instance.
            TRY(result, spgm_ptr->servreg_error_notify_cmd_rsp_vtbl->graph_set_get_cfg_packed_rsp_h(cu_ptr, rsp_info, NULL));
         }
         else
         {
            // Handling the response to the case where Set configuration is sent to the APM module instance.
            rsp_info->sec_opcode    = cmd_hndl_node_ptr->sec_opcode;
            rsp_info->cmd_extn_info = cmd_hndl_node_ptr->cmd_extn_info;
            TRY(result, spgm_ptr->servreg_error_notify_cmd_rsp_vtbl->graph_set_get_cfg_rsp_h(cu_ptr, rsp_info));
         }
         break;
      }

      case APM_CMD_REGISTER_CFG:
      case APM_CMD_REGISTER_SHARED_CFG:
      case APM_CMD_DEREGISTER_CFG:
      case APM_CMD_DEREGISTER_SHARED_CFG:
      {
         if (FALSE == cmd_hndl_node_ptr->is_apm_cmd_rsp)
         {
            // Handling the response to the case where register/de-register
            // configuration is sent to the module instance.
            TRY(result, spgm_ptr->servreg_error_notify_cmd_rsp_vtbl->graph_set_persistent_rsp_h(cu_ptr, rsp_info));
         }
         else
         {
            // Handling the response to the case where register/de-register
            // configuration is sent to the APM module instance.
            TRY(result,
                spgm_ptr->servreg_error_notify_cmd_rsp_vtbl->graph_set_persistent_packed_rsp_h(cu_ptr, rsp_info));
         }
         break;
      }

      case APM_CMD_REGISTER_MODULE_EVENTS:
      {
         // The token is zero for event sent from OLC.
         // There is no better way to identify if the event is sent by HLOS or by the OLC.
         // If the command is sent by HLOS, the token is always non-zero.
         if (0 != cmd_hndl_node_ptr->token)
         {
            TRY(result, spgm_ptr->servreg_error_notify_cmd_rsp_vtbl->graph_event_reg_rsp_h(cu_ptr, rsp_info));
         }
         break;
      }

      default:
      {
         OLC_SGM_MSG(OLC_SGM_ID,
                     DBG_ERROR_PRIO,
                     "ssr_pdr_cmd_rsp: unsupported response with pkt opcode(%lX) token(%lx) ",
                     cmd_hndl_node_ptr->opcode,
                     cmd_hndl_node_ptr->token);
      }
   }

   CATCH(result, OLC_MSG_PREFIX, log_id)
   {
   }

   OLC_SGM_MSG(OLC_SGM_ID,
               DBG_HIGH_PRIO,
               "ssr_pdr_cmd_rsp: pending command handling for opcode(%lX) token(%lx) completed",
               cmd_hndl_node_ptr->opcode,
               cmd_hndl_node_ptr->token);

   return result;
}

ar_result_t sgm_servreg_notify_event_handler(cu_base_t *cu_ptr, spgm_info_t *spgm_ptr)
{
   ar_result_t result = AR_EOK;
   uint32_t    log_id = 0;
   INIT_EXCEPTION_HANDLING
   spf_list_node_t *     curr_node_ptr     = NULL;
   spgm_cmd_hndl_node_t *cmd_hndl_node_ptr = NULL;

   VERIFY(result, (NULL != spgm_ptr));
   log_id = spgm_ptr->sgm_id.log_id;

   OLC_SGM_MSG(OLC_SGM_ID,
               DBG_HIGH_PRIO,
               "ssr_pdr_cmd_rsp: satellite crash notified, process pending command response,"
               " num pending command response %lu",
               spgm_ptr->cmd_hndl_list.num_active_cmd_hndls);

   /** Get the pointer to start of the list of module list nodes */
   curr_node_ptr = spgm_ptr->cmd_hndl_list.cmd_hndl_list_ptr;

   while (spgm_ptr->cmd_hndl_list.num_active_cmd_hndls)
   {
      if (NULL != curr_node_ptr)
      {
         cmd_hndl_node_ptr = (spgm_cmd_hndl_node_t *)curr_node_ptr->obj_ptr;
         if (cmd_hndl_node_ptr->wait_for_rsp)
         {
            spgm_servreg_notify_cmd_rsp_handler(cu_ptr, spgm_ptr, cmd_hndl_node_ptr);
         }
         sgm_destroy_cmd_handle(spgm_ptr, cmd_hndl_node_ptr->opcode, cmd_hndl_node_ptr->token);
         curr_node_ptr = curr_node_ptr->next_ptr;
      }
      else
      {
         OLC_SGM_MSG(OLC_SGM_ID,
                     DBG_ERROR_PRIO,
                     "ssr_pdr_cmd_rsp: invalid object node pointer, num pending command response %lu",
                     spgm_ptr->cmd_hndl_list.num_active_cmd_hndls);
         break;
      }
   }

   CATCH(result, OLC_MSG_PREFIX, log_id)
   {
   }

   return result;
}
