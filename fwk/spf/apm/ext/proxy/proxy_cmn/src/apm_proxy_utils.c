/**
 * \file apm_proxy_utils.c
 *
 * \brief
 *
 *     This file contains APM proxy manager utlity function defintions
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/****************************************************************************
 * INCLUDE HEADER FILES                                                     *
 ****************************************************************************/

#include "apm_graph_db.h"
#include "apm_msg_utils.h"
#include "apm_cmd_sequencer.h"
#include "apm_proxy_def.h"
#include "apm_cmd_utils.h"
#include "apm_proxy_utils.h"
#include "apm_proxy_vcpm_utils.h"
#include "vcpm_api.h"
#include "posal_intrinsics.h"
#include "irm_api.h"
#include "amdb_api.h"
#include "irm.h"
#include "amdb_thread.h"

/****************************************************************************
 * GLOBALS                                                                  *
 ****************************************************************************/

/****************************************************************************
 * Function Declarations
 ****************************************************************************/

ar_result_t apm_proxy_util_get_allocated_cmd_ctrl_obj(apm_proxy_manager_t   *proxy_mgr_ptr,
                                                      apm_cmd_ctrl_t        *apm_cmd_ctrl_ptr,
                                                      apm_proxy_cmd_ctrl_t **proxy_cmd_ctrl_pptr)
{
   ar_result_t           result = AR_EOK;
   apm_proxy_cmd_ctrl_t *proxy_cmd_ctrl_ptr;

   /** Validate input params. */
   if (NULL == apm_cmd_ctrl_ptr || NULL == proxy_mgr_ptr || NULL == proxy_cmd_ctrl_pptr)
   {
      return AR_EBADPARAM;
   }

   /** Get the pointer to this container's commond control
    *  object list */
   proxy_cmd_ctrl_ptr = proxy_mgr_ptr->cmd_list.cmd_ctrl_list;

   /** Init the return pointer */
   *proxy_cmd_ctrl_pptr = NULL;

   /** Check if any of the cmd obj is already allocated for current
    *  APM command */
   for (uint32_t idx = 0; idx < APM_NUM_MAX_PARALLEL_CMD; idx++)
   {
      /** If the APM cmd ctrl pointer matches with the in container
       *  cmd obj */
      if ((void *)apm_cmd_ctrl_ptr == proxy_cmd_ctrl_ptr[idx].apm_cmd_ctrl_ptr)
      {
         /** Match found, return */
         *proxy_cmd_ctrl_pptr = &proxy_cmd_ctrl_ptr[idx];

         AR_MSG(DBG_HIGH_PRIO,
                "apm_proxy_util_get_allocated_cmd_ctrl_obj(): found allocated proxy cmd ctrl obj with list idx %d for "
                "proxy manager with vsid 0x%08X",
                idx,
                proxy_mgr_ptr->vcpm_properties.vsid);

         return AR_EOK;
      }

   } /** End of for (cont cmd obj list) */

   return result;
}

/**
 * Helper function for apm_proxy_util_release_cmd_ctrl_obj
 * */
ar_result_t apm_proxy_clear_cached_proxy_params(apm_proxy_cmd_ctrl_t *proxy_cmd_ctrl_ptr, uint32_t cmd_opcode)
{
   ar_result_t     result = AR_EOK;
   apm_cmd_ctrl_t *cmd_ctrl_ptr;

   /** Get the command control object for current command in
    *  progrres */
   cmd_ctrl_ptr = (apm_cmd_ctrl_t *)proxy_cmd_ctrl_ptr->apm_cmd_ctrl_ptr;

   switch (cmd_opcode)
   {
      case APM_CMD_GRAPH_OPEN:
      {
         switch (cmd_ctrl_ptr->cmd_seq.graph_open_seq.op_idx)
         {
            case APM_OPEN_CMD_OP_HDL_DB_QUERY_SEND_INFO:
            {
               break;
            }
            default:
            {
               /** Clear cached proxy manager params,
                * In case of graph open failure also this is needed, to avoid leaks*/
               if (proxy_cmd_ctrl_ptr->cached_cfg_params.graph_open_params.param_data_list_ptr)
               {
                  spf_list_delete_list(&proxy_cmd_ctrl_ptr->cached_cfg_params.graph_open_params.param_data_list_ptr,
                                       TRUE /** POOL USED */);

                  proxy_cmd_ctrl_ptr->cached_cfg_params.graph_open_params.num_mod_param_cfg = 0;
               }

               if (proxy_cmd_ctrl_ptr->cached_cfg_params.graph_open_params.sg_list_ptr)
               {
                  spf_list_delete_list(&proxy_cmd_ctrl_ptr->cached_cfg_params.graph_open_params.sg_list_ptr,
                                       TRUE /** POOL USED */);

                  proxy_cmd_ctrl_ptr->cached_cfg_params.graph_open_params.num_proxy_sub_graphs = 0;
               }
               break;
            }
         }

         break;
      }
      case APM_CMD_SET_CFG:
      case APM_CMD_GET_CFG:
      case APM_CMD_REGISTER_CFG:
      case APM_CMD_DEREGISTER_CFG:
      {
         if (proxy_cmd_ctrl_ptr->cached_cfg_params.proxy_cfg_params.param_data_list_ptr)
         {
            spf_list_delete_list(&proxy_cmd_ctrl_ptr->cached_cfg_params.proxy_cfg_params.param_data_list_ptr,
                                 TRUE /** POOL USED */);

            proxy_cmd_ctrl_ptr->cached_cfg_params.proxy_cfg_params.num_mod_param_cfg = 0;
         }

         break;
      }
      default:
      {
         break;
      }
   }

   return result;
}

ar_result_t apm_proxy_util_release_cmd_ctrl_obj(apm_proxy_manager_t  *proxy_mgr_ptr,
                                                apm_proxy_cmd_ctrl_t *proxy_cmd_ctrl_ptr)
{
   ar_result_t     result = AR_EOK;
   apm_cmd_ctrl_t *cmd_ctrl_ptr;
   uint32_t        cmd_opcode;

   if (!proxy_mgr_ptr || !proxy_cmd_ctrl_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "apm_proxy_util_release_cmd_ctrl_obj(): Proxy Manager ptr(0x%lX) and/or Proxy cmd ctrl ptr(0x%lX) is NULL",
             proxy_mgr_ptr,
             proxy_cmd_ctrl_ptr);

      return AR_EFAILED;
   }

   /** Get the current command control pointer */
   cmd_ctrl_ptr = proxy_cmd_ctrl_ptr->apm_cmd_ctrl_ptr;

   /** Get the currnt cmd opcode   */
   cmd_opcode = cmd_ctrl_ptr->cmd_opcode;

   /** Clear any cached configuration for current command */
   apm_proxy_clear_cached_proxy_params(proxy_cmd_ctrl_ptr, cmd_opcode);

   /** Clear this bit in the active command mask */
   APM_CLR_BIT(&proxy_mgr_ptr->cmd_list.active_cmd_mask, proxy_cmd_ctrl_ptr->list_idx);

   /** Release the graph open cached configuration (due to
    *  linked list usage) */

   /** Clear the command control */
   memset(proxy_cmd_ctrl_ptr, 0, sizeof(apm_proxy_cmd_ctrl_t));

   AR_MSG(DBG_MED_PRIO,
          "apm_proxy_util_release_cmd_ctrl_obj(): Release cmd ctrl obj, cmd_opcode[0x%lX] proxy manager with vsid "
          "0x%08X",
          cmd_opcode,
          proxy_mgr_ptr->vcpm_properties.vsid);

   return result;
}

ar_result_t apm_proxy_util_get_cmd_ctrl_obj(apm_proxy_manager_t   *proxy_mgr_ptr,
                                            apm_cmd_ctrl_t        *apm_cmd_ctrl_ptr,
                                            apm_proxy_cmd_ctrl_t **proxy_cmd_ctrl_pptr)
{
   ar_result_t           result = AR_EOK;
   uint32_t              cmd_slot_idx;
   apm_proxy_cmd_ctrl_t *proxy_cmd_ctrl_ptr = NULL;

   /** Validate input params. */
   if (NULL == apm_cmd_ctrl_ptr || NULL == proxy_mgr_ptr || NULL == proxy_cmd_ctrl_pptr)
   {
      return AR_EBADPARAM;
   }

   /** First check if any command object is already allocated
    *  corresponding to current command */
   apm_proxy_util_get_allocated_cmd_ctrl_obj(proxy_mgr_ptr, apm_cmd_ctrl_ptr, &proxy_cmd_ctrl_ptr);

   /** If allocated object found, return */
   if (proxy_cmd_ctrl_ptr)
   {
      *proxy_cmd_ctrl_pptr = proxy_cmd_ctrl_ptr;

      return result;
   }

   /** Execeution falls through if a new object needs to be
    *  allocated now */

   /** Check if all the slots in the command obj list are
    *  occupied.
    *  This condition should not hit as the APM cmd Q is removed
    *  from the wait mask once all the cmd obj slots are
    *  occupied. */
   if (APM_CMD_LIST_FULL_MASK == proxy_mgr_ptr->cmd_list.active_cmd_mask)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "apm_proxy_util_get_cmd_ctrl_obj(), cmd obj list is full, Proxy Instance_ID[0x%lX]",
             proxy_mgr_ptr->proxy_instance_id);

      return AR_EFAILED;
   }

   /** Find the next available slot in the command list */
   cmd_slot_idx = s32_ct1_s32(proxy_mgr_ptr->cmd_list.active_cmd_mask);

   /** Set this bit in the active command mask */
   APM_SET_BIT(&proxy_mgr_ptr->cmd_list.active_cmd_mask, cmd_slot_idx);

   /** Get the pointer to command control object corrsponding to
    *  available slot */
   proxy_cmd_ctrl_ptr = &proxy_mgr_ptr->cmd_list.cmd_ctrl_list[cmd_slot_idx];

   proxy_cmd_ctrl_ptr->msg_token = APM_CMD_TOKEN_PROXY_CTRL_TYPE;

   /** Save the list index in cmd obj */
   proxy_cmd_ctrl_ptr->list_idx = cmd_slot_idx;

   /** Save the APM command control pointer in this object */
   proxy_cmd_ctrl_ptr->apm_cmd_ctrl_ptr = (void *)apm_cmd_ctrl_ptr;

   /** Save the host container pointer in this object */
   proxy_cmd_ctrl_ptr->host_proxy_mgr_ptr = (void *)proxy_mgr_ptr;

   /** Populate the return pointer */
   *proxy_cmd_ctrl_pptr = proxy_cmd_ctrl_ptr;

   AR_MSG(DBG_MED_PRIO,
          "apm_proxy_util_get_cmd_ctrl_obj(): Assigned cmd ctrl obj idx[%lu], cmd_opcode[0x%lX] proxy manager with "
          "vsid 0x%08X",
          cmd_slot_idx,
          apm_cmd_ctrl_ptr->cmd_opcode,
          proxy_mgr_ptr->vcpm_properties.vsid);

   return result;
}

ar_result_t apm_clear_active_proxy_list(apm_t *apm_info_ptr, apm_cmd_ctrl_t *apm_cmd_ctrl_ptr)
{
   ar_result_t           result = AR_EOK;
   spf_list_node_t      *curr_node_ptr, *next_node_ptr;
   apm_proxy_manager_t  *proxy_mgr_ptr;
   apm_proxy_cmd_ctrl_t *proxy_cmd_ctrl_ptr;

   /** Validate input params. */
   if (NULL == apm_cmd_ctrl_ptr)
   {
      return AR_EBADPARAM;
   }

   /** Get the pointer to list of containers pending send
    *  message */
   curr_node_ptr = apm_cmd_ctrl_ptr->rsp_ctrl.active_proxy_mgr_list_ptr;

   /** Iterate over the container list */
   while (curr_node_ptr)
   {
      proxy_mgr_ptr = (apm_proxy_manager_t *)curr_node_ptr->obj_ptr;

      next_node_ptr = curr_node_ptr->next_ptr;

      /** Get the container's command control object corresponding
       *  to current APM command control object */
      apm_proxy_util_get_allocated_cmd_ctrl_obj(proxy_mgr_ptr, apm_cmd_ctrl_ptr, &proxy_cmd_ctrl_ptr);

      if (NULL != proxy_cmd_ctrl_ptr)
      {
         if (NULL != proxy_cmd_ctrl_ptr->rsp_ctrl.permitted_sg_array_ptr)
         {
            posal_memory_free(proxy_cmd_ctrl_ptr->rsp_ctrl.permitted_sg_array_ptr);
            proxy_cmd_ctrl_ptr->rsp_ctrl.permitted_sg_array_ptr = NULL;
         }

         /** Clear the container command control */
         memset(&proxy_cmd_ctrl_ptr->rsp_ctrl, 0, sizeof(apm_proxy_cmd_rsp_ctrl_t));

         /** If last message in the sequence */
         if ((apm_cmd_ctrl_ptr->proxy_msg_opcode.proxy_opcode_idx + 1) ==
             apm_cmd_ctrl_ptr->proxy_msg_opcode.num_proxy_msg_opcode)
         {
            /** Release the container command control object */
            apm_proxy_util_release_cmd_ctrl_obj(proxy_mgr_ptr, proxy_cmd_ctrl_ptr);

            /** Free up the list node memory and
             *  advance to next node in the list */
            spf_list_find_delete_node(&apm_cmd_ctrl_ptr->rsp_ctrl.active_proxy_mgr_list_ptr,
                                      proxy_mgr_ptr,
                                      TRUE /*pool_used */);

            /** Decrement number of proxy manager */
            apm_cmd_ctrl_ptr->rsp_ctrl.num_active_proxy_mgrs--;

            /** Check and free the host proxy manager if not needed any more
             *  Only delete the proxy manager when there are no other commands
             *  using it. This needs to be done here because the proxy manager
             *  list in the rsp control will be lost after this. Also,
             *  active_cmd_mask will be cleard before this for current command
             */
            if ((0 == proxy_mgr_ptr->cmd_list.active_cmd_mask) &&
                ((IRM_MODULE_INSTANCE_ID == proxy_mgr_ptr->proxy_instance_id) ||
                 (AMDB_MODULE_INSTANCE_ID == proxy_mgr_ptr->proxy_instance_id)))
            {
               /** Remove Proxy Manager node from apm graph info list. */
               apm_db_remove_node_from_list(&apm_info_ptr->graph_info.proxy_manager_list_ptr,
                                            proxy_mgr_ptr,
                                            &apm_info_ptr->graph_info.num_proxy_managers);
               posal_memory_free(proxy_mgr_ptr);
            }
         }
      }

      /** Advance to next node */
      curr_node_ptr = next_node_ptr;

   } /** End of while() */

   return result;
}

static ar_result_t apm_proxy_util_find_proxy_mgr_from_id(apm_graph_info_t     *apm_graph_info_ptr,
                                                         apm_proxy_manager_t **proxy_mgr_pptr,
                                                         uint32_t              instance_id,
                                                         spf_handle_t         *proxy_handle_ptr)
{
   bool_t           match_found        = FALSE;
   ar_result_t      result             = AR_EOK;
   spf_list_node_t *proxy_mgr_list_ptr = NULL;
   /** Validate input params. */
   if (NULL == apm_graph_info_ptr || NULL == proxy_mgr_pptr)
   {
      return FALSE;
   }

   proxy_mgr_list_ptr = apm_graph_info_ptr->proxy_manager_list_ptr;

   /** Loop through all the proxy managers in the list and find the
       proxy manager with the given instance ID.*/
   while (proxy_mgr_list_ptr)
   {
      apm_proxy_manager_t *proxy_mgr = (apm_proxy_manager_t *)proxy_mgr_list_ptr->obj_ptr;

      if (instance_id == proxy_mgr->proxy_instance_id)
      {
         AR_MSG(DBG_HIGH_PRIO, "apm_proxy_util_find_proxy_mgr_from_id, proxy manager found for id %lu", instance_id);
         match_found     = TRUE;
         *proxy_mgr_pptr = proxy_mgr;
         break;
      }

      proxy_mgr_list_ptr = proxy_mgr_list_ptr->next_ptr;
   }

   /** If the proxy manager is not found, allocate and initialize */
   if (!match_found)
   {
      /** Allocate memory for Proxy manager node for APM graph DB */
      apm_proxy_manager_t *proxy_mgr_ptr =
         (apm_proxy_manager_t *)posal_memory_malloc(sizeof(apm_proxy_manager_t), APM_INTERNAL_STATIC_HEAP_ID);
      if (NULL == proxy_mgr_ptr)
      {
         AR_MSG(DBG_ERROR_PRIO, "apm_proxy_util_find_proxy_mgr_from_id: Failed to allocate Proxy Manager node mem");
         return AR_ENOMEMORY;
      }

      AR_MSG(DBG_HIGH_PRIO,
             "apm_proxy_util_find_proxy_mgr_from_id: Allocated Proxy Manager node mems, 0x%x",
             proxy_mgr_ptr);

      /** Clear the allocated struct */
      memset(proxy_mgr_ptr, 0, sizeof(apm_proxy_manager_t));

      proxy_mgr_ptr->proxy_instance_id = instance_id;
      proxy_mgr_ptr->proxy_handle_ptr  = proxy_handle_ptr;

      /** Add module node to the proxy manager list*/
      if (AR_EOK != (result = apm_db_add_node_to_list(&apm_graph_info_ptr->proxy_manager_list_ptr,
                                                      proxy_mgr_ptr,
                                                      &apm_graph_info_ptr->num_proxy_managers)))
      {
         AR_MSG(DBG_ERROR_PRIO,
                "apm_proxy_util_find_proxy_mgr_from_id: Failed to add proxy manager to the list of APM graph info");

         return AR_EFAILED;
      }
      *proxy_mgr_pptr = proxy_mgr_ptr;
   }

   return result;
}

ar_result_t apm_free_proxy_mgr_from_id(apm_t *apm_info_ptr, uint32_t instance_id)
{
   ar_result_t      result             = AR_EOK;
   spf_list_node_t *proxy_mgr_list_ptr = apm_info_ptr->graph_info.proxy_manager_list_ptr;

   /** Loop through all the proxy managers in the list and find the
       proxy manager with the given instance ID.*/
   while (proxy_mgr_list_ptr)
   {
      apm_proxy_manager_t *proxy_mgr = (apm_proxy_manager_t *)proxy_mgr_list_ptr->obj_ptr;

      if (instance_id == proxy_mgr->proxy_instance_id)
      {
         apm_db_remove_node_from_list(&apm_info_ptr->graph_info.proxy_manager_list_ptr,
                                      proxy_mgr,
                                      &apm_info_ptr->graph_info.num_proxy_managers);
         posal_memory_free(proxy_mgr);
         break;
      }
      proxy_mgr_list_ptr = proxy_mgr_list_ptr->next_ptr;
   }

   return result;
}

static ar_result_t apm_add_param_to_proxy_param_list(apm_cmd_ctrl_t          *apm_cmd_ctrl_ptr,
                                                     apm_proxy_cmd_ctrl_t    *curr_proxy_cmd_ctrl,
                                                     apm_module_param_data_t *mod_data_ptr,
                                                     bool_t                   use_sys_q)
{

   ar_result_t             result                     = AR_EOK;
   spf_list_node_t       **proxy_param_data_list_pptr = NULL;
   uint32_t               *proxy_num_cfg_ptr          = NULL;
   apm_proxy_cached_cfg_t *cached_cfg_params_ptr      = &curr_proxy_cmd_ctrl->cached_cfg_params;
   switch (apm_cmd_ctrl_ptr->cmd_opcode)
   {
      case (APM_CMD_GRAPH_OPEN):
      {
         switch (apm_cmd_ctrl_ptr->cmd_seq.graph_open_seq.op_idx)
         {
            case APM_OPEN_CMD_OP_HDL_DB_QUERY_PREPROCESS:
            {
               proxy_param_data_list_pptr = &cached_cfg_params_ptr->proxy_cfg_params.param_data_list_ptr;
               proxy_num_cfg_ptr          = &cached_cfg_params_ptr->proxy_cfg_params.num_mod_param_cfg;
               break;
            } // NOTE: add switch case for open as well, we shouldn't default for open
            default:
            {
               proxy_param_data_list_pptr = &cached_cfg_params_ptr->graph_open_params.param_data_list_ptr;
               proxy_num_cfg_ptr          = &cached_cfg_params_ptr->graph_open_params.num_mod_param_cfg;
               break;
            }
         }
         break;
      }
      case (APM_CMD_GRAPH_CLOSE):
      {

         switch (apm_cmd_ctrl_ptr->cmd_seq.graph_mgmt_seq.op_idx)
         {
            case APM_GM_CMD_OP_PRE_PROCESS:
            {
               proxy_param_data_list_pptr = &cached_cfg_params_ptr->proxy_cfg_params.param_data_list_ptr;
               proxy_num_cfg_ptr          = &cached_cfg_params_ptr->proxy_cfg_params.num_mod_param_cfg;
               break;
            }
            default:
            {
               return result;
            }
         }
         break;
      }
      case APM_CMD_SET_CFG:
      {
         // Get the sys q flag and add it to cached_cfg_params here
         // sanity check to see of only sys q flag is used or drop when flag changes
         if ((TRUE == cached_cfg_params_ptr->proxy_cfg_params.use_sys_q) && (FALSE == use_sys_q))
         {
            AR_MSG(DBG_ERROR_PRIO, "apm_proxy_util_cache_config: Mismatched enable of using sys queue ");
            return AR_EFAILED;
         }
         cached_cfg_params_ptr->proxy_cfg_params.use_sys_q = use_sys_q;
         proxy_param_data_list_pptr = &cached_cfg_params_ptr->proxy_cfg_params.param_data_list_ptr;
         proxy_num_cfg_ptr          = &cached_cfg_params_ptr->proxy_cfg_params.num_mod_param_cfg;
         break;
      }
      default: // NOTE: add switch case here as well, we should default to this
      {
         proxy_param_data_list_pptr = &cached_cfg_params_ptr->proxy_cfg_params.param_data_list_ptr;
         proxy_num_cfg_ptr          = &cached_cfg_params_ptr->proxy_cfg_params.num_mod_param_cfg;
         break;
      }
   }

   /** Add this param ID to the cached configuration list of this module */
   result = apm_db_add_node_to_list(proxy_param_data_list_pptr, mod_data_ptr, proxy_num_cfg_ptr);
   return result;
}

static ar_result_t apm_proxy_util_find_proxy_mgr(apm_graph_info_t        *graph_info_ptr,
                                                 apm_proxy_manager_t    **proxy_mgr_pptr,
                                                 apm_module_param_data_t *mod_data_ptr)
{
   ar_result_t   result           = AR_EOK;
   spf_handle_t *proxy_handle_ptr = NULL;

   switch (mod_data_ptr->module_instance_id)
   {
      case VCPM_MODULE_INSTANCE_ID:
      {
         if (TRUE != apm_proxy_util_find_vcpm_proxy_mgr(graph_info_ptr, proxy_mgr_pptr, mod_data_ptr))
         {
            AR_MSG(DBG_ERROR_PRIO, "apm_proxy_util_cache_config: Failed to allocate proxy_cmd_ctrl. result %d", result);
            mod_data_ptr->error_code = AR_ENOTEXIST;
            return AR_EOK;
         }
         break;
      }
      case IRM_MODULE_INSTANCE_ID:
      {
         result = irm_get_spf_handle((void **)&proxy_handle_ptr);
         if (AR_EOK != result)
         {
            mod_data_ptr->error_code = result;
            AR_MSG(DBG_ERROR_PRIO, "apm_proxy_util_cache_config: Failed to get IRM spf handle. result %d", result);
            return AR_EOK;
         }

         if (AR_EOK != (result = apm_proxy_util_find_proxy_mgr_from_id(graph_info_ptr,
                                                                       proxy_mgr_pptr,
                                                                       mod_data_ptr->module_instance_id,
                                                                       proxy_handle_ptr)))
         {
            mod_data_ptr->error_code = result;
            AR_MSG(DBG_ERROR_PRIO, "apm_proxy_util_cache_config: Failed to allocate proxy_cmd_ctrl. result %d", result);
            return AR_EOK;
         }
         break;
      }
      case AMDB_MODULE_INSTANCE_ID:
      {
         result = amdb_get_spf_handle((void **)&proxy_handle_ptr);
         if (AR_EOK != result)
         {
            mod_data_ptr->error_code = result;
            AR_MSG(DBG_ERROR_PRIO, "apm_proxy_util_cache_config: Failed to get AMDB spf handle. result %d", result);
            return AR_EOK;
         }

         if (AR_EOK != (result = apm_proxy_util_find_proxy_mgr_from_id(graph_info_ptr,
                                                                       proxy_mgr_pptr,
                                                                       mod_data_ptr->module_instance_id,
                                                                       proxy_handle_ptr)))
         {
            mod_data_ptr->error_code = result;
            AR_MSG(DBG_ERROR_PRIO, "apm_proxy_util_cache_config: Failed to allocate proxy_cmd_ctrl. result %d", result);
            return AR_EOK;
         }
         break;
      }
      default:
      {
         AR_MSG(DBG_ERROR_PRIO,
                "apm_proxy_util_cache_config: Un-supported static instance id: 0x%lX",
                mod_data_ptr->module_instance_id);

         mod_data_ptr->error_code = AR_ENOTEXIST;

         return AR_EUNSUPPORTED;
      }
   }
   return result;
}

ar_result_t apm_handle_proxy_mgr_cfg_params(apm_t                   *apm_info_ptr,
                                            apm_module_param_data_t *mod_data_ptr,
                                            bool_t                   use_sys_q)
{
   ar_result_t           result              = AR_EOK;
   apm_proxy_manager_t  *proxy_mgr_ptr       = NULL;
   apm_cmd_ctrl_t       *apm_cmd_ctrl_ptr    = NULL;
   apm_proxy_cmd_ctrl_t *curr_proxy_cmd_ctrl = NULL;
   // apm_graph_info_t *    graph_info_ptr      = NULL;

   /** Validate input params. */
   if (NULL == apm_info_ptr || NULL == mod_data_ptr)
   {
      return AR_EBADPARAM;
   }

   apm_cmd_ctrl_ptr = apm_info_ptr->curr_cmd_ctrl_ptr;
   // graph_info_ptr   = &apm_info_ptr->graph_info;

   /** Find the proxy manager node corresponding to the module instance ID.*/
   if (AR_EOK != (result = apm_proxy_util_find_proxy_mgr(&apm_info_ptr->graph_info, &proxy_mgr_ptr, mod_data_ptr)))
   {
      AR_MSG(DBG_MED_PRIO,
             "apm_proxy_util_cache_config:  Failed to find proxy manager, module IID = 0x%X, result = 0x%X",
             mod_data_ptr->module_instance_id,
             result);
   }

   AR_MSG(DBG_MED_PRIO, "apm_proxy_util_cache_config:  received ParamID. 0x%08X", mod_data_ptr->param_id);

   result = apm_proxy_util_get_cmd_ctrl_obj(proxy_mgr_ptr, apm_cmd_ctrl_ptr, &curr_proxy_cmd_ctrl);

   if (AR_EOK != result || NULL == curr_proxy_cmd_ctrl)
   {
      mod_data_ptr->error_code = AR_EBUSY;
      AR_MSG(DBG_ERROR_PRIO, "apm_proxy_util_cache_config: Failed to allocate proxy_cmd_ctrl. result %d", result);
      return result;
   }

   /** Get the pointer to proxy manager cached configuration params
    *  corresponding to current APM command and add module param to that list */
   if (AR_EOK !=
       (result = apm_add_param_to_proxy_param_list(apm_cmd_ctrl_ptr, curr_proxy_cmd_ctrl, mod_data_ptr, use_sys_q)))
   {
      AR_MSG(DBG_ERROR_PRIO, "apm_proxy_util_cache_config: Failed to add param to proxy param list. result %d", result);
      return result;
   }

   /** Add Proxy manger to apm cmd-Rsp ctrl pending list*/
   if (!curr_proxy_cmd_ctrl->rsp_ctrl.pending_msg_proc)
   {
      switch (apm_cmd_ctrl_ptr->cmd_opcode)
      {
         case APM_CMD_GRAPH_OPEN:
         case APM_CMD_GRAPH_CLOSE:
         case APM_CMD_CLOSE_ALL:
            apm_db_add_node_to_list(&apm_cmd_ctrl_ptr->rsp_ctrl.inactive_proxy_mgr_list_ptr,
                                    (void *)proxy_mgr_ptr,
                                    &apm_cmd_ctrl_ptr->rsp_ctrl.num_inactive_proxy_mgrs);
            break;
         default:
            // Set/get cfg, reg/dereg cfg
            apm_db_add_node_to_list(&apm_cmd_ctrl_ptr->rsp_ctrl.active_proxy_mgr_list_ptr,
                                    (void *)proxy_mgr_ptr,
                                    &apm_cmd_ctrl_ptr->rsp_ctrl.num_active_proxy_mgrs);
      }

      /** Set the pending flag  */
      curr_proxy_cmd_ctrl->rsp_ctrl.pending_msg_proc = TRUE;
   }

   return result;
}

ar_result_t apm_move_proxies_to_active_list(apm_t *apm_info_ptr)
{
   ar_result_t          result                  = AR_EOK;
   apm_proxy_manager_t *proxy_mgr_ptr           = NULL;
   spf_list_node_t     *inactive_proxy_list_ptr = NULL;
   spf_list_node_t     *proxy_list_next_ptr     = NULL;
   apm_cmd_ctrl_t      *apm_cmd_ctrl_ptr        = NULL;
   uint32_t             proxy_count             = 0; // for dbg, counts how many proxies got activated

   apm_cmd_ctrl_ptr        = apm_info_ptr->curr_cmd_ctrl_ptr;
   inactive_proxy_list_ptr = apm_cmd_ctrl_ptr->rsp_ctrl.inactive_proxy_mgr_list_ptr;

   while (inactive_proxy_list_ptr)
   {
      proxy_mgr_ptr       = inactive_proxy_list_ptr->obj_ptr;
      proxy_list_next_ptr = inactive_proxy_list_ptr->next_ptr;

      apm_db_add_node_to_list(&apm_cmd_ctrl_ptr->rsp_ctrl.active_proxy_mgr_list_ptr,
                              (void *)proxy_mgr_ptr,
                              &apm_cmd_ctrl_ptr->rsp_ctrl.num_active_proxy_mgrs);

      apm_db_remove_node_from_list(&apm_cmd_ctrl_ptr->rsp_ctrl.inactive_proxy_mgr_list_ptr,
                                   (void *)proxy_mgr_ptr,
                                   &apm_cmd_ctrl_ptr->rsp_ctrl.num_inactive_proxy_mgrs);

      inactive_proxy_list_ptr = proxy_list_next_ptr;
      proxy_count++;
   }

   AR_MSG(DBG_MED_PRIO, "apm_move_proxies_to_active: activated all proxies, %d found", proxy_count);

   return result;
}

ar_result_t apm_move_proxy_to_active_list_by_id(apm_t *apm_info_ptr, uint32_t instance_id)
{
   ar_result_t      result                  = AR_EOK;
   spf_list_node_t *inactive_proxy_list_ptr = NULL;
   apm_cmd_ctrl_t  *apm_cmd_ctrl_ptr        = NULL;
   bool_t           found_proxy             = FALSE;

   apm_cmd_ctrl_ptr        = apm_info_ptr->curr_cmd_ctrl_ptr;
   inactive_proxy_list_ptr = apm_cmd_ctrl_ptr->rsp_ctrl.inactive_proxy_mgr_list_ptr;

   /** Loop through all the proxy managers in the list and find the
       proxy manager with the given instance ID.*/
   while (inactive_proxy_list_ptr)
   {
      apm_proxy_manager_t *proxy_mgr_ptr = (apm_proxy_manager_t *)inactive_proxy_list_ptr->obj_ptr;

      if (instance_id == proxy_mgr_ptr->proxy_instance_id)
      {
         AR_MSG(DBG_HIGH_PRIO,
                "apm_move_proxy_to_active_list_by_id, adding proxy manager found for id 0x%lx to active list",
                instance_id);

         apm_db_add_node_to_list(&apm_cmd_ctrl_ptr->rsp_ctrl.active_proxy_mgr_list_ptr,
                                 (void *)proxy_mgr_ptr,
                                 &apm_cmd_ctrl_ptr->rsp_ctrl.num_active_proxy_mgrs);

         apm_db_remove_node_from_list(&apm_cmd_ctrl_ptr->rsp_ctrl.inactive_proxy_mgr_list_ptr,
                                      (void *)proxy_mgr_ptr,
                                      &apm_cmd_ctrl_ptr->rsp_ctrl.num_inactive_proxy_mgrs);

         found_proxy = TRUE;

         break;
      }

      inactive_proxy_list_ptr = inactive_proxy_list_ptr->next_ptr;
   }

   if (!found_proxy)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "apm_move_proxy_to_active_list_by_id: failed to find proxy based on iid: 0x%lx",
             instance_id);
   }

   return result;
}