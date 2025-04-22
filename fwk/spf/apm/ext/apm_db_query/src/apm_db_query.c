/**
 * \file apm_db_query.c
 *
 * \brief
 *
 *     This file handles apm db query requests.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/**==============================================================================
 * INCLUDE HEADER FILES
 ==============================================================================*/
#include "apm_db_query.h"
#include "apm_internal_if.h"
#include "apm_graph_db.h"
#include "apm_internal.h"
#include "irm_api.h"
#include "apm_proxy_utils.h"
#include "apm_cmd_sequencer.h"
#include "irm.h"

ar_result_t apm_db_query_preprocess_get_param(apm_t *apm_info_ptr, apm_module_param_data_t *param_data_ptr);
ar_result_t apm_db_query_add_cntr_to_list(apm_t *apm_info_ptr, void *node_ptr, bool_t is_open);
ar_result_t apm_db_query_handle_graph_open(apm_t *apm_info_ptr);
ar_result_t apm_db_query_handle_graph_close(apm_t *apm_info_ptr);
void        apm_db_free_cntr_and_sg_list();
/**==============================================================================
   Global Defines
 ==============================================================================*/
apm_db_query_info_t g_apm_db_query_info;

apm_db_query_utils_vtable_t g_db_query_util_funcs = {
   .apm_db_query_preprocess_get_param_fptr = apm_db_query_preprocess_get_param,
   .apm_db_query_add_cntr_to_list_fptr     = apm_db_query_add_cntr_to_list,
   .apm_db_query_handle_graph_open_fptr    = apm_db_query_handle_graph_open,
   .apm_db_query_handle_graph_close_fptr   = apm_db_query_handle_graph_close,
   .apm_db_free_cntr_and_sg_list_fptr      = apm_db_free_cntr_and_sg_list,
};
/**==============================================================================
   Function Defines
 =============================================================================*/

/**********************************************************************************************************************/
static bool_t apm_db_query_is_sg_id_present(spf_list_node_t *subgraph_list_ptr, uint32_t sg_id)
{
   if (NULL == subgraph_list_ptr)
   {
      return FALSE;
   }
   spf_list_node_t *local_subgraph_list_ptr = subgraph_list_ptr;
   for (; NULL != local_subgraph_list_ptr; LIST_ADVANCE(local_subgraph_list_ptr))
   {
      apm_sub_graph_t *sg_node_ptr = (apm_sub_graph_t *)local_subgraph_list_ptr->obj_ptr;
      if (NULL == sg_node_ptr)
      {
         continue;
      }
      if (sg_node_ptr->sub_graph_id == sg_id)
      {
         return TRUE;
      }
   }
   return FALSE;
}

/**********************************************************************************************************************/
static uint32_t apm_db_query_find_num_instances(bool_t is_open)
{
   uint32_t         num_instances = 0;
   spf_list_node_t *cntr_list_ptr = NULL;

   if (FALSE == irm_is_cntr_or_mod_prof_enabled())
   {
      return num_instances;
   }

   if (is_open)
   {
      cntr_list_ptr = g_apm_db_query_info.open_cntr_list_ptr;
   }
   else
   {
      if (NULL == g_apm_db_query_info.close_sg_list_ptr)
      {
         return num_instances;
      }
      cntr_list_ptr = g_apm_db_query_info.close_cntr_list_ptr;
   }

   // Find number of containers and modules in the all the subgraphs being opened
   for (; NULL != cntr_list_ptr; LIST_ADVANCE(cntr_list_ptr))
   {
      apm_db_query_cntr_node_t *db_query_cntr_ptr = (apm_db_query_cntr_node_t *)cntr_list_ptr->obj_ptr;
      if ((NULL == db_query_cntr_ptr) || (NULL == db_query_cntr_ptr->cntr_node_ptr))
      {
         continue;
      }
      apm_container_t *cntr_node_ptr = (apm_container_t *)db_query_cntr_ptr->cntr_node_ptr;

      if (is_open)
      {
         num_instances++;
      }
      else
      {
         AR_MSG(DBG_HIGH_PRIO,
                "APM DB QUERY: sg_count = %lu, num_sub_graphs = %lu",
                db_query_cntr_ptr->sg_count,
                cntr_node_ptr->num_sub_graphs);
         if (db_query_cntr_ptr->sg_count == cntr_node_ptr->num_sub_graphs)
         {
            num_instances++;
         }
      }

      spf_list_node_t *pspc_module_list_ptr = cntr_node_ptr->pspc_module_list_node_ptr;
      for (; NULL != pspc_module_list_ptr; LIST_ADVANCE(pspc_module_list_ptr))
      {
         apm_pspc_module_list_t *pspc_module_node_ptr = (apm_pspc_module_list_t *)pspc_module_list_ptr->obj_ptr;
         if (NULL == pspc_module_node_ptr)
         {
            continue;
         }

         if (cntr_node_ptr->container_id == pspc_module_node_ptr->container_id)
         {
            if (is_open)
            {
               num_instances += pspc_module_node_ptr->num_modules;
            }
            else
            {
               if (TRUE == apm_db_query_is_sg_id_present(g_apm_db_query_info.close_sg_list_ptr,
                                                         pspc_module_node_ptr->sub_graph_id))
               {
                  num_instances += pspc_module_node_ptr->num_modules;
               }
            }
         }
      }
   }

   return num_instances;
}

/**********************************************************************************************************************/
static void apm_db_query_populate_graph_set_param_payload(apm_module_param_data_t *mod_data_ptr,
                                                          uint32_t                 num_instances_ids,
                                                          bool_t                   is_open)
{
   spf_list_node_t                  *cntr_list_ptr             = NULL;
   param_id_cntr_instance_handles_t *cntr_instance_handles_ptr = (param_id_cntr_instance_handles_t *)(mod_data_ptr + 1);
   cntr_instance_handles_ptr->num_instance_ids                 = num_instances_ids;
   param_id_cntr_instance_handles_payload_t *cntr_instance_handles_payload_ptr =
      (param_id_cntr_instance_handles_payload_t *)(cntr_instance_handles_ptr + 1);

   if (is_open)
   {
      cntr_list_ptr = g_apm_db_query_info.open_cntr_list_ptr;
   }
   else
   {
      cntr_list_ptr = g_apm_db_query_info.close_cntr_list_ptr;
   }

   // Populate the param payload with container and module handles
   for (; NULL != cntr_list_ptr; LIST_ADVANCE(cntr_list_ptr))
   {
      apm_db_query_cntr_node_t *db_query_cntr_ptr = (apm_db_query_cntr_node_t *)cntr_list_ptr->obj_ptr;
      if ((NULL == db_query_cntr_ptr) || (NULL == db_query_cntr_ptr->cntr_node_ptr))
      {
         continue;
      }
      apm_container_t *cntr_node_ptr = (apm_container_t *)db_query_cntr_ptr->cntr_node_ptr;

      if (is_open)
      {
         cntr_instance_handles_payload_ptr->container_instance_id = cntr_node_ptr->container_id;
         cntr_instance_handles_payload_ptr->handle_ptr            = cntr_node_ptr->cont_hdl_ptr;
         cntr_instance_handles_payload_ptr++;
      }
      else
      {
         if (db_query_cntr_ptr->sg_count == cntr_node_ptr->num_sub_graphs)
         {
            cntr_instance_handles_payload_ptr->container_instance_id = cntr_node_ptr->container_id;
            cntr_instance_handles_payload_ptr++;
         }
      }

      spf_list_node_t *pspc_module_list_ptr = cntr_node_ptr->pspc_module_list_node_ptr;
      for (; NULL != pspc_module_list_ptr; LIST_ADVANCE(pspc_module_list_ptr))
      {
         apm_pspc_module_list_t *pspc_module_node_ptr = (apm_pspc_module_list_t *)pspc_module_list_ptr->obj_ptr;
         if (NULL == pspc_module_node_ptr)
         {
            continue;
         }

         if ((FALSE == is_open) && (FALSE == apm_db_query_is_sg_id_present(g_apm_db_query_info.close_sg_list_ptr,
                                                                           pspc_module_node_ptr->sub_graph_id)))
         {
            continue;
         }

         if (cntr_node_ptr->container_id == pspc_module_node_ptr->container_id)
         {
            spf_list_node_t *module_list_ptr = pspc_module_node_ptr->module_list_ptr;
            for (; NULL != module_list_ptr; LIST_ADVANCE(module_list_ptr))
            {
               apm_module_t *module_node_ptr = (apm_module_t *)module_list_ptr->obj_ptr;
               if (NULL == module_node_ptr)
               {
                  continue;
               }

               cntr_instance_handles_payload_ptr->module_instance_id = module_node_ptr->instance_id;
               if (is_open)
               {
                  cntr_instance_handles_payload_ptr->handle_ptr            = cntr_node_ptr->cont_hdl_ptr;
                  cntr_instance_handles_payload_ptr->container_instance_id = cntr_node_ptr->container_id;
               }

               cntr_instance_handles_payload_ptr++;
            }
         }
      }
   }
}

/**********************************************************************************************************************/
static void apm_db_query_find_all_graph_close_sg_n_update_cntrs(apm_t *apm_info_ptr)
{
   ar_result_t         result               = AR_EOK;
   apm_cmd_ctrl_t     *cmd_ctrl_ptr         = apm_info_ptr->curr_cmd_ctrl_ptr;
   spf_list_node_t    *reg_sg_list_ptr      = cmd_ctrl_ptr->graph_mgmt_cmd_ctrl.sg_list.reg_sg_list_ptr;
   apm_sub_graph_id_t *proxy_sg_id_list_ptr = cmd_ctrl_ptr->graph_mgmt_cmd_ctrl.sg_list.proxy_sg_id_list_ptr;
   uint32_t            num_close_sg         = 0;
   bool_t              IS_OPEN_FALSE        = FALSE;

   // TODO:pbm rebase to sumeet's changes
   for (; NULL != reg_sg_list_ptr; LIST_ADVANCE(reg_sg_list_ptr))
   {
      if (AR_EOK !=
          (result =
              apm_db_add_node_to_list(&g_apm_db_query_info.close_sg_list_ptr, reg_sg_list_ptr->obj_ptr, &num_close_sg)))
      {
         AR_MSG(DBG_ERROR_PRIO, "APM DB QUERY: FAILED to add node to the list, result: 0x%lx", result);
         continue;
      }
   }

   for (uint32_t num_sg = 0; num_sg < cmd_ctrl_ptr->graph_mgmt_cmd_ctrl.sg_list.num_proxy_sub_graphs; num_sg++)
   {
      apm_sub_graph_t *sub_graph_node_ptr = NULL;

      result = apm_db_get_sub_graph_node(&apm_info_ptr->graph_info,
                                         proxy_sg_id_list_ptr[num_sg].sub_graph_id,
                                         &sub_graph_node_ptr,
                                         APM_DB_OBJ_QUERY);
      if ((AR_EOK != result) || (NULL == sub_graph_node_ptr))
      {
         continue;
      }
      if (AR_EOK != (result = apm_db_add_node_to_list(&g_apm_db_query_info.close_sg_list_ptr,
                                                      (void *)sub_graph_node_ptr,
                                                      &num_close_sg)))
      {
         AR_MSG(DBG_ERROR_PRIO, "APM DB QUERY: FAILED to add node to the list, result: 0x%lx", result);
         continue;
      }
   }

   spf_list_node_t *sg_list_ptr = g_apm_db_query_info.close_sg_list_ptr;
   for (; NULL != sg_list_ptr; LIST_ADVANCE(sg_list_ptr))
   {
      apm_sub_graph_t *sub_graph_node_ptr = (apm_sub_graph_t *)sg_list_ptr->obj_ptr;
      if (NULL == sub_graph_node_ptr)
      {
         continue;
      }
      spf_list_node_t *cntr_list_ptr = sub_graph_node_ptr->container_list_ptr;
      for (; NULL != cntr_list_ptr; LIST_ADVANCE(cntr_list_ptr))
      {
         apm_container_t *cntr_node_ptr = (apm_container_t *)cntr_list_ptr->obj_ptr;
         if (NULL == cntr_node_ptr)
         {
            continue;
         }
         apm_db_query_add_cntr_to_list(apm_info_ptr, (void *)cntr_node_ptr, IS_OPEN_FALSE);
      }
   }
   return;
}

/**********************************************************************************************************************/
void apm_db_free_cntr_and_sg_list()
{
   // Free open and close cntr list since we have created the payload using the info
   // Need to clear both since in the case of a failed graph open they will both have valid container lists
   if ((NULL != g_apm_db_query_info.open_cntr_list_ptr))
   {
      spf_list_delete_list_and_free_objs(&g_apm_db_query_info.open_cntr_list_ptr, TRUE);
      g_apm_db_query_info.open_cntr_list_ptr = NULL;
   }
   if (NULL != g_apm_db_query_info.close_cntr_list_ptr)
   {
      spf_list_delete_list_and_free_objs(&g_apm_db_query_info.close_cntr_list_ptr, TRUE);
      g_apm_db_query_info.close_cntr_list_ptr = NULL;
   }
   if ((NULL != g_apm_db_query_info.close_sg_list_ptr))
   {
      spf_list_delete_list(&g_apm_db_query_info.close_sg_list_ptr, TRUE);
      g_apm_db_query_info.close_sg_list_ptr = NULL;
   }
}

/**********************************************************************************************************************/
static ar_result_t apm_db_query_graph_close_preprocess(apm_t *apm_info_ptr)
{
   ar_result_t              result              = AR_EOK;
   uint32_t                 num_instance_ids    = 0;
   uint32_t                 total_param_size    = 0;
   uint32_t                 param_payload_size  = 0;
   apm_module_param_data_t *mod_data_ptr        = NULL;
   bool_t                   USE_SYS_Q_FALSE     = FALSE;
   bool_t                   IS_GRAPH_OPEN_FASLE = FALSE;

   apm_db_query_find_all_graph_close_sg_n_update_cntrs(apm_info_ptr);

   num_instance_ids = apm_db_query_find_num_instances(IS_GRAPH_OPEN_FASLE);

   if (0 == num_instance_ids)
   {
      AR_MSG(DBG_ERROR_PRIO, "APM DB QUERY: No instance IDs found in the Query.. Ignoring");
      goto __bailout_apm_db_query_graph_close_preprocess;
   }
   /** Create the set param payload based on num instances */
   param_payload_size = ALIGN_8_BYTES(sizeof(param_id_cntr_instance_handles_t) +
                                      (num_instance_ids * sizeof(param_id_cntr_instance_handles_payload_t)));
   total_param_size   = sizeof(apm_module_param_data_t) + param_payload_size;

   /** Allocate payload for graph open db query param to be sent to IRM */
   mod_data_ptr = (apm_module_param_data_t *)posal_memory_malloc(total_param_size, APM_INTERNAL_STATIC_HEAP_ID);
   if (NULL == mod_data_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "APM DB QUERY: Failed to allocated memory for IRM graph open db query");
      return AR_ENOMEMORY;
   }
   memset((void *)mod_data_ptr, 0, total_param_size);
   mod_data_ptr->module_instance_id = IRM_MODULE_INSTANCE_ID;
   mod_data_ptr->param_size         = param_payload_size;
   mod_data_ptr->param_id           = APM_PARAM_ID_RESET_CNTR_HANDLES;

   /** Populate the set param payload with instance ids and handles for container and modules */
   apm_db_query_populate_graph_set_param_payload(mod_data_ptr, num_instance_ids, IS_GRAPH_OPEN_FASLE);

   /** Allocate proxy manager for IRM, and use new payload */
   result = apm_handle_proxy_mgr_cfg_params(apm_info_ptr, mod_data_ptr, USE_SYS_Q_FALSE);
   if (AR_EOK != result)
   {
      AR_MSG(DBG_ERROR_PRIO, "APM DB QUERY: Failed to allocate IRM proxy manager");
      posal_memory_free(mod_data_ptr);
   }
   // since this message is unique to the irm, we activate ONLY the IRM proxy here
   result = apm_move_proxy_to_active_list_by_id(apm_info_ptr, IRM_MODULE_INSTANCE_ID);

// TODO:pbm test with basic, voice, data path related, run time link handling
__bailout_apm_db_query_graph_close_preprocess:
   apm_db_free_cntr_and_sg_list();
   return result;
}
/**********************************************************************************************************************/
ar_result_t apm_db_query_handle_graph_close(apm_t *apm_info_ptr)
{
   ar_result_t     result = AR_EOK;
   apm_cmd_ctrl_t *cmd_ctrl_ptr;
   cmd_ctrl_ptr = apm_info_ptr->curr_cmd_ctrl_ptr;

   if (((APM_CMD_GRAPH_OPEN != cmd_ctrl_ptr->cmd_opcode) && (APM_CMD_GRAPH_CLOSE != cmd_ctrl_ptr->cmd_opcode) &&
        (APM_CMD_CLOSE_ALL != cmd_ctrl_ptr->cmd_opcode)) ||
       (FALSE == irm_is_cntr_or_mod_prof_enabled()))
   {
      return result;
   }

   switch (cmd_ctrl_ptr->cmd_seq.graph_mgmt_seq.op_idx)
   {
      case APM_GM_CMD_OP_PRE_PROCESS:
      {
         result = apm_db_query_graph_close_preprocess(apm_info_ptr);
         break;
      }
      case APM_GM_CMD_OP_DB_QUERY_SEND_INFO:
      {
         result = apm_cmd_set_get_cfg_sequencer(apm_info_ptr);
         break;
      }
      default:
      {
         result = AR_EUNSUPPORTED;

         AR_MSG(DBG_ERROR_PRIO,
                "APM DB QUERY: Unsupported op_idx[%lu], cmd_opcode[0x%08lx]",
                cmd_ctrl_ptr->cmd_seq.graph_open_seq.op_idx,
                cmd_ctrl_ptr->cmd_opcode);

         break;
      }
   }
   return result;
}

/**********************************************************************************************************************/
static ar_result_t apm_db_query_graph_open_preprocess(apm_t *apm_info_ptr)
{
   ar_result_t result = AR_EOK;

   uint32_t                 num_instance_ids   = 0;
   uint32_t                 total_param_size   = 0;
   uint32_t                 param_payload_size = 0;
   apm_module_param_data_t *mod_data_ptr       = NULL;
   bool_t                   USE_SYS_Q_FALSE    = FALSE;
   bool_t                   IS_GRAPH_OPEN_TRUE = TRUE;

   num_instance_ids = apm_db_query_find_num_instances(IS_GRAPH_OPEN_TRUE);
   if (0 == num_instance_ids)
   {
      AR_MSG(DBG_ERROR_PRIO, "APM DB QUERY: No instance IDs found in the Query.. Ignoring");
      goto __bailout_apm_db_query_graph_open_preprocess;
   }

   /** Create the set param payload based on num instances */
   param_payload_size = ALIGN_8_BYTES(sizeof(param_id_cntr_instance_handles_t) +
                                      (num_instance_ids * sizeof(param_id_cntr_instance_handles_payload_t)));
   total_param_size   = sizeof(apm_module_param_data_t) + param_payload_size;

   /** Allocate payload for graph open db query param to be sent to IRM */
   mod_data_ptr = (apm_module_param_data_t *)posal_memory_malloc(total_param_size, APM_INTERNAL_STATIC_HEAP_ID);
   if (NULL == mod_data_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "APM DB QUERY: Failed to allocated memory for IRM graph open db query");
      return AR_ENOMEMORY;
   }
   memset((void *)mod_data_ptr, 0, total_param_size);
   mod_data_ptr->module_instance_id = IRM_MODULE_INSTANCE_ID;
   mod_data_ptr->param_size         = param_payload_size;
   mod_data_ptr->param_id           = APM_PARAM_ID_SET_CNTR_HANDLES;

   /** Populate the set param payload with instance ids and handles for container and modules */
   apm_db_query_populate_graph_set_param_payload(mod_data_ptr, num_instance_ids, IS_GRAPH_OPEN_TRUE);

   /** Allocate proxy manager for IRM, and use new payload */
   result = apm_handle_proxy_mgr_cfg_params(apm_info_ptr, mod_data_ptr, USE_SYS_Q_FALSE);
   if (AR_EOK != result)
   {
      AR_MSG(DBG_ERROR_PRIO, "APM DB QUERY: Failed to allocate IRM proxy manager");
      posal_memory_free(mod_data_ptr);
   }
   // since this message is unique to the irm, we activate ONLY the IRM proxy here
   result = apm_move_proxy_to_active_list_by_id(apm_info_ptr, IRM_MODULE_INSTANCE_ID);

__bailout_apm_db_query_graph_open_preprocess:
   apm_db_free_cntr_and_sg_list();

   return result;
}

/**********************************************************************************************************************/
ar_result_t apm_db_query_handle_graph_open(apm_t *apm_info_ptr)
{
   ar_result_t     result = AR_EOK;
   apm_cmd_ctrl_t *cmd_ctrl_ptr;

   cmd_ctrl_ptr = apm_info_ptr->curr_cmd_ctrl_ptr;

   if ((APM_CMD_GRAPH_OPEN != cmd_ctrl_ptr->cmd_opcode) || (FALSE == irm_is_cntr_or_mod_prof_enabled()))
   {
      return result;
   }
   switch (cmd_ctrl_ptr->cmd_seq.graph_open_seq.op_idx)
   {
      case APM_OPEN_CMD_OP_HDL_DB_QUERY_PREPROCESS:
      {
         result = apm_db_query_graph_open_preprocess(apm_info_ptr);
         break;
      }
      case APM_OPEN_CMD_OP_HDL_DB_QUERY_SEND_INFO:
      {
         result = apm_cmd_set_get_cfg_sequencer(apm_info_ptr);
         break;
      }
      default:
      {
         result = AR_EUNSUPPORTED;

         AR_MSG(DBG_ERROR_PRIO,
                "APM DB QUERY: Unsupported op_idx[%lu], cmd_opcode[0x%08lx]",
                cmd_ctrl_ptr->cmd_seq.graph_open_seq.op_idx,
                cmd_ctrl_ptr->cmd_opcode);

         break;
      }
   }
   return result;
}
/**********************************************************************************************************************/
ar_result_t apm_db_query_add_cntr_to_list(apm_t *apm_info_ptr, void *node_ptr, bool_t is_open)
{
   ar_result_t               result = AR_EOK;
   uint32_t                  num_cntr;
   apm_db_query_cntr_node_t *cntr_node_ptr = NULL;
   if (NULL == node_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "APM DB QUERY: Invalid node ptr");
      return AR_EFAILED;
   }

   if (FALSE == irm_is_cntr_or_mod_prof_enabled())
   {
      return result;
   }

   spf_list_node_t **cntr_list_pptr = NULL;
   if (is_open)
   {
      cntr_list_pptr = &g_apm_db_query_info.open_cntr_list_ptr;
   }
   else
   {
      cntr_list_pptr = &g_apm_db_query_info.close_cntr_list_ptr;
   }
   spf_list_node_t *cntr_list_ptr = (*cntr_list_pptr);
   for (; (NULL != cntr_list_ptr); LIST_ADVANCE(cntr_list_ptr))
   {
      cntr_node_ptr = (apm_db_query_cntr_node_t *)cntr_list_ptr->obj_ptr;
      if ((NULL != cntr_node_ptr) && (cntr_node_ptr->cntr_node_ptr == node_ptr))
      {
         cntr_node_ptr->sg_count++;
         break;
      }
   }

   if (NULL == cntr_list_ptr)
   {
      cntr_node_ptr =
         (apm_db_query_cntr_node_t *)posal_memory_malloc(sizeof(apm_db_query_cntr_node_t), APM_INTERNAL_STATIC_HEAP_ID);
      if (NULL == cntr_node_ptr)
      {
         AR_MSG(DBG_ERROR_PRIO, "APM DB QUERY: Failed to allocate node");
         return AR_ENOMEMORY;
      }
      memset(cntr_node_ptr, 0, sizeof(apm_db_query_cntr_node_t));
      cntr_node_ptr->cntr_node_ptr = node_ptr;
      if (AR_EOK != (result = apm_db_add_node_to_list(cntr_list_pptr, (void *)cntr_node_ptr, &num_cntr)))
      {
         AR_MSG(DBG_ERROR_PRIO, "APM DB QUERY: FAILED to add node to the list, result: 0x%lx", result);
         return AR_EFAILED;
      }
      cntr_node_ptr->sg_count++;
   }

   apm_container_t *cntr_ptr = (apm_container_t *)cntr_node_ptr->cntr_node_ptr;
   AR_MSG(DBG_HIGH_PRIO,
          "APM DB QUERY: While adding sg_count = %lu, num_sub_graphs = %lu",
          cntr_node_ptr->sg_count,
          cntr_ptr->num_sub_graphs);
   return result;
}

/**********************************************************************************************************************/
ar_result_t apm_db_query_handle_get_cntr_handles(apm_t *apm_info_ptr, apm_module_param_data_t *param_data_ptr)
{
   ar_result_t result = AR_EOK;
   if ((sizeof(param_id_cntr_instance_handles_t) > param_data_ptr->param_size))
   {
      AR_MSG(DBG_ERROR_PRIO, "APM DB QUERY: Not enough param size");
      param_data_ptr->error_code = AR_ENEEDMORE;
      return AR_ENEEDMORE;
   }

   param_id_cntr_instance_handles_t *cntr_instance_handles_ptr =
      (param_id_cntr_instance_handles_t *)(param_data_ptr + 1);

   uint32_t total_param_size =
      (sizeof(param_id_cntr_instance_handles_t)) +
      (cntr_instance_handles_ptr->num_instance_ids * sizeof(param_id_cntr_instance_handles_payload_t));

   if (total_param_size > param_data_ptr->param_size)
   {
      AR_MSG(DBG_ERROR_PRIO, "APM DB QUERY: Not enough param size");
      param_data_ptr->error_code = AR_ENEEDMORE;
      return AR_ENEEDMORE;
   }

   param_id_cntr_instance_handles_payload_t *cntr_instance_handles_payload_arr_ptr =
      (param_id_cntr_instance_handles_payload_t *)(cntr_instance_handles_ptr + 1);
   for (uint32_t iid_idx = 0; iid_idx < cntr_instance_handles_ptr->num_instance_ids; iid_idx++)
   {
      param_id_cntr_instance_handles_payload_t *cntr_instance_handles_payload_ptr =
         &cntr_instance_handles_payload_arr_ptr[iid_idx];

      bool_t is_container_iid = (0 == cntr_instance_handles_payload_ptr->module_instance_id);
      if (TRUE == is_container_iid)
      {
         apm_container_t *cntr_ptr = NULL;

         apm_db_get_container_node(&apm_info_ptr->graph_info,
                                   cntr_instance_handles_payload_ptr->container_instance_id,
                                   &cntr_ptr,
                                   APM_DB_OBJ_QUERY);
         if (NULL != cntr_ptr)
         {
            cntr_instance_handles_payload_ptr->handle_ptr = cntr_ptr->cont_hdl_ptr;
         }
         AR_MSG(DBG_ERROR_PRIO,
                "APM DB QUERY: Container IID = 0x%X, handle = 0x%X",
                cntr_instance_handles_payload_ptr->container_instance_id,
                cntr_instance_handles_payload_ptr->handle_ptr);
      }
      else
      {
         apm_module_t *module_ptr = NULL;
         apm_db_get_module_node(&apm_info_ptr->graph_info,
                                cntr_instance_handles_payload_ptr->module_instance_id,
                                &module_ptr,
                                APM_DB_OBJ_QUERY);
         if ((NULL != module_ptr) && (NULL != module_ptr->host_cont_ptr))
         {
            apm_container_t *cntr_ptr                                = module_ptr->host_cont_ptr;
            cntr_instance_handles_payload_ptr->handle_ptr            = cntr_ptr->cont_hdl_ptr;
            cntr_instance_handles_payload_ptr->container_instance_id = cntr_ptr->container_id;
         }

         AR_MSG(DBG_ERROR_PRIO,
                "APM DB QUERY: Module IID = 0x%X, handle = 0x%X",
                cntr_instance_handles_payload_ptr->module_instance_id,
                cntr_instance_handles_payload_ptr->handle_ptr);
      }
   }
   return result;
}

/**********************************************************************************************************************/
ar_result_t apm_db_query_handle_get_all_cntr_handles(apm_t *apm_info_ptr, apm_module_param_data_t *param_data_ptr)
{

   // TODO: allocate oob payload
   ar_result_t       result = AR_EOK;
   spf_list_node_t  *curr_ptr;
   apm_graph_info_t *graph_info_ptr = &apm_info_ptr->graph_info;

   if ((sizeof(param_id_cntr_instance_handles_t) > param_data_ptr->param_size))
   {
      AR_MSG(DBG_ERROR_PRIO, "APM DB QUERY: Not enough param size");
      param_data_ptr->error_code = AR_ENEEDMORE;
      return AR_ENEEDMORE;
   }

   param_id_cntr_instance_handles_t *cntr_instance_handles_ptr =
      (param_id_cntr_instance_handles_t *)(param_data_ptr + 1);

   uint32_t total_param_size =
      (sizeof(param_id_cntr_instance_handles_t)) +

      // TODO Rename apm_db_get_num_instances
      (apm_db_get_num_instances(graph_info_ptr) * sizeof(param_id_cntr_instance_handles_payload_t));

   if (total_param_size > param_data_ptr->param_size)
   {
      AR_MSG(DBG_ERROR_PRIO, "APM DB QUERY: Not enough param size");
      param_data_ptr->error_code = AR_ENEEDMORE;
      return AR_ENEEDMORE;
   }

   param_id_cntr_instance_handles_payload_t *cntr_instance_handles_payload_arr_ptr =
      (param_id_cntr_instance_handles_payload_t *)(cntr_instance_handles_ptr + 1);

   // Iterate over all containers and modules, and add them to the payload
   uint32_t iid_idx = 0;
   for (uint8_t i = 0; i < APM_CONT_HASH_TBL_SIZE; ++i)
   {
      curr_ptr                  = graph_info_ptr->container_list_ptr[i];
      apm_container_t *cntr_ptr = NULL;
      while (NULL != curr_ptr)
      {
         cntr_ptr = (apm_container_t *)curr_ptr->obj_ptr;
         if (NULL != cntr_ptr)
         {
            param_id_cntr_instance_handles_payload_t *cntr_instance_handles_payload_ptr =
               &cntr_instance_handles_payload_arr_ptr[iid_idx];
            cntr_instance_handles_payload_ptr->handle_ptr            = cntr_ptr->cont_hdl_ptr;
            cntr_instance_handles_payload_ptr->container_instance_id = cntr_ptr->container_id;
            cntr_instance_handles_payload_ptr->module_instance_id    = 0;
            AR_MSG(DBG_ERROR_PRIO,
                   "APM DB QUERY: Container IID = 0x%X, handle = 0x%X",
                   cntr_instance_handles_payload_ptr->container_instance_id,
                   cntr_instance_handles_payload_ptr->handle_ptr);
            ++iid_idx;
         }

         curr_ptr = curr_ptr->next_ptr;
      }
   }

   for (uint8_t i = 0; i < APM_MODULE_HASH_TBL_SIZE; ++i)
   {
      curr_ptr                 = graph_info_ptr->module_list_ptr[i];
      apm_module_t *module_ptr = NULL;
      while (NULL != curr_ptr)
      {
         module_ptr = (apm_module_t *)curr_ptr->obj_ptr;
         if ((NULL != module_ptr) && (NULL != module_ptr->host_cont_ptr))
         {
            param_id_cntr_instance_handles_payload_t *cntr_instance_handles_payload_ptr =
               &cntr_instance_handles_payload_arr_ptr[iid_idx];
            apm_container_t *cntr_ptr                                = module_ptr->host_cont_ptr;
            cntr_instance_handles_payload_ptr->handle_ptr            = cntr_ptr->cont_hdl_ptr;
            cntr_instance_handles_payload_ptr->container_instance_id = cntr_ptr->container_id;
            cntr_instance_handles_payload_ptr->module_instance_id    = module_ptr->instance_id;

            AR_MSG(DBG_ERROR_PRIO,
                   "APM DB QUERY: Module IID = 0x%X, handle = 0x%X",
                   cntr_instance_handles_payload_ptr->module_instance_id,
                   cntr_instance_handles_payload_ptr->handle_ptr);
            ++iid_idx;
         }

         curr_ptr = curr_ptr->next_ptr;
      }
   }

   cntr_instance_handles_ptr->num_instance_ids = iid_idx;

   return result;
}

/**********************************************************************************************************************/
ar_result_t apm_db_query_preprocess_get_param(apm_t *apm_info_ptr, apm_module_param_data_t *param_data_ptr)
{
   ar_result_t result = AR_EOK;

   if (NULL == param_data_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "APM DB QUERY: NULL param data ptr");
      return AR_EFAILED;
   }

   switch (param_data_ptr->param_id)
   {
      case APM_PARAM_ID_GET_CNTR_HANDLES:
      {
         result = apm_db_query_handle_get_cntr_handles(apm_info_ptr, param_data_ptr);
         break;
      }
      case APM_PARAM_ID_GET_ALL_CNTR_HANDLES:
      {
         result = apm_db_query_handle_get_all_cntr_handles(apm_info_ptr, param_data_ptr);
         break;
      }
      default:
      {
         AR_MSG(DBG_ERROR_PRIO, "APM DB QUERY: Unsupported param id 0x%X", param_data_ptr->param_id);
      }
   }
   return result;
}

/**********************************************************************************************************************/
ar_result_t apm_db_query_init(apm_t *apm_info_ptr)
{
   ar_result_t result                        = AR_EOK;
   apm_info_ptr->ext_utils.db_query_vtbl_ptr = &g_db_query_util_funcs;
   return result;
}
/**********************************************************************************************************************/
