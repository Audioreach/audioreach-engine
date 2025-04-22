
/**
@file irm_apm_if_utils.cpp

@brief IRM to APM interface utils.

================================================================================
Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
SPDX-License-Identifier: BSD-3-Clause-Clear
==============================================================================*/
// clang-format off
// clang-format on

#include "irm_api.h"
#include "irm_i.h"
#include "apm_internal_if.h"
#include "irm_cntr_if.h"

/*----------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------*/
static void irm_count_or_fill_instance_info(irm_t                                    *irm_ptr,
                                            bool                                      fill_payload,
                                            uint32_t                                 *num_instances_ptr,
                                            param_id_cntr_instance_handles_payload_t *cntr_instances_payload_ptr)
{
   spf_list_node_t *block_node_ptr = irm_ptr->core.block_head_node_ptr;

   for (; NULL != block_node_ptr; LIST_ADVANCE(block_node_ptr))
   {
      irm_node_obj_t *block_obj_ptr = (irm_node_obj_t *)block_node_ptr->obj_ptr;

      if ((NULL != block_obj_ptr) &&
          ((IRM_BLOCK_ID_CONTAINER == block_obj_ptr->id) || (IRM_BLOCK_ID_MODULE == block_obj_ptr->id)))
      {
         spf_list_node_t *instance_node_ptr = block_obj_ptr->head_node_ptr;
         for (; NULL != instance_node_ptr; LIST_ADVANCE(instance_node_ptr))
         {
            irm_node_obj_t *instance_obj_ptr = (irm_node_obj_t *)instance_node_ptr->obj_ptr;
            if ((NULL == instance_obj_ptr) || (NULL != instance_obj_ptr->handle_ptr))
            {
               continue;
            }

            if (TRUE == fill_payload)
            {
               cntr_instances_payload_ptr->handle_ptr = NULL;
               if (IRM_BLOCK_ID_MODULE == block_obj_ptr->id)
               {
                  cntr_instances_payload_ptr->module_instance_id    = instance_obj_ptr->id;
                  cntr_instances_payload_ptr->container_instance_id = 0;
               }
               else
               {
                  cntr_instances_payload_ptr->module_instance_id    = 0;
                  cntr_instances_payload_ptr->container_instance_id = instance_obj_ptr->id;
               }
               AR_MSG(DBG_HIGH_PRIO,
                      "IRM: irm_count_or_fill_instance_info: CNTR IID = 0x%X, MOD IID = 0x%X",
                      cntr_instances_payload_ptr->container_instance_id,
                      cntr_instances_payload_ptr->module_instance_id);
               cntr_instances_payload_ptr += 1;
            }
            else
            {
               (*num_instances_ptr)++;
            }
         }
      }
   }
   return;
}

static ar_result_t check_insert_instance_node(irm_t            *irm_ptr,
                                              spf_list_node_t **instance_list_ptr,
                                              uint32_t          block_id,
                                              uint32_t          cntr_id,
                                              uint32_t          module_id)
{
   ar_result_t     result       = AR_EOK;
   irm_node_obj_t *new_node_obj = NULL;
   if (IRM_BLOCK_ID_CONTAINER == block_id)
   {
      new_node_obj = irm_check_insert_node(irm_ptr, instance_list_ptr, cntr_id);
      if (NULL == new_node_obj)
      {
         AR_MSG(DBG_ERROR_PRIO, "IRM: Failed to add node object");
         return AR_EFAILED;
      }
   }
   else
   {
      new_node_obj = irm_check_insert_node(irm_ptr, instance_list_ptr, module_id);
      if (NULL == new_node_obj)
      {
         AR_MSG(DBG_ERROR_PRIO, "IRM: Failed to add node object");
         return AR_EFAILED;
      }
      new_node_obj->cntr_iid = cntr_id;
   }

   return result;
}

/*----------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------*/
void irm_update_cntr_mod_prof_enable_flag(irm_t  *irm_ptr,
                                          bool_t *is_mem_prof_enabled_ptr,
                                          bool_t *is_mod_process_prof_enabled_ptr)
{
   bool_t is_cntr_or_mod_prof_enabled = FALSE;

   /* Check if any container or module profiling is enabled
    *    If true - Enable is_cntr_or_mod_prof_enabled in IRM structure - Used by APM to decide whether to send info
    *              during open/close
    * Check if any container or module level heap profiling is enabled
    *    If True - mark is_mem_prof_enabled_ptr as true - used to inform posal that mem prof is needed
    * Check if any module level pcycles or packet_cnt profiling is enabled
    *    If True - mark is_mod_process_prof_enabled_ptr as true - used to inform pm manager to release island vote
    */

   spf_list_node_t *block_node_ptr = irm_ptr->core.block_head_node_ptr;
   for (; NULL != block_node_ptr; LIST_ADVANCE(block_node_ptr))
   {
      irm_node_obj_t *block_obj_ptr = (irm_node_obj_t *)block_node_ptr->obj_ptr;

      if ((NULL != block_obj_ptr) &&
          ((IRM_BLOCK_ID_CONTAINER == block_obj_ptr->id) || (IRM_BLOCK_ID_MODULE == block_obj_ptr->id)))
      {
         spf_list_node_t *instance_node_ptr = block_obj_ptr->head_node_ptr;
         for (; NULL != instance_node_ptr; LIST_ADVANCE(instance_node_ptr))
         {
            irm_node_obj_t *instance_obj_ptr = (irm_node_obj_t *)instance_node_ptr->obj_ptr;
            if (NULL == instance_obj_ptr)
            {
               continue;
            }
            is_cntr_or_mod_prof_enabled = TRUE;

            spf_list_node_t *metric_node_ptr = instance_obj_ptr->head_node_ptr;
            for (; NULL != metric_node_ptr; LIST_ADVANCE(metric_node_ptr))
            {
               irm_node_obj_t *metric_obj_ptr = (irm_node_obj_t *)metric_node_ptr->obj_ptr;
               if (NULL == metric_obj_ptr)
               {
                  continue;
               }

               if (IRM_METRIC_ID_HEAP_INFO == metric_obj_ptr->id)
               {
                  (*is_mem_prof_enabled_ptr) = TRUE;
               }

               if (IRM_BLOCK_ID_MODULE == block_obj_ptr->id)
               {
                  if ((IRM_METRIC_ID_PROCESSOR_CYCLES == metric_obj_ptr->id) ||
                      (IRM_METRIC_ID_PACKET_COUNT == metric_obj_ptr->id))
                  {
                     (*is_mod_process_prof_enabled_ptr) = TRUE;
                  }
               }
            }
         }
      }
   }
   irm_ptr->core.is_cntr_or_mod_prof_enabled = is_cntr_or_mod_prof_enabled;
}

/*----------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------*/
ar_result_t irm_request_instance_handles_payload(irm_t *irm_ptr)
{
   ar_result_t                               result                     = AR_EOK;
   uint32_t                                  param_payload_size         = 0;
   uint32_t                                  buf_size                   = 0;
   param_id_cntr_instance_handles_t         *cntr_instances_handle_ptr  = NULL;
   param_id_cntr_instance_handles_payload_t *cntr_instances_payload_ptr = NULL;
   bool_t                                    IS_FILL_FALSE              = FALSE;
   bool_t                                    IS_FILL_TRUE               = TRUE;
   uint32_t                                  num_instances_ids          = 0;
   spf_handle_t                             *apm_handle_ptr             = apm_get_apm_handle();

   if (NULL == apm_handle_ptr)
   {
      AR_MSG(DBG_HIGH_PRIO, "IRM: NULL APM handle");
      return AR_EFAILED;
   }

   irm_count_or_fill_instance_info(irm_ptr, IS_FILL_FALSE, &num_instances_ids, NULL);
   AR_MSG(DBG_HIGH_PRIO, "IRM: Number of instance IDs = %lu", num_instances_ids);

   if (0 == num_instances_ids)
   {
      return result;
   }

   param_payload_size =
      sizeof(param_id_cntr_instance_handles_t) + (num_instances_ids * sizeof(param_id_cntr_instance_handles_payload_t));
   buf_size = sizeof(spf_msg_cmd_param_data_cfg_t) + (sizeof(void *)) + sizeof(apm_module_param_data_t) +
              ALIGN_8_BYTES(param_payload_size);
   buf_size = GET_SPF_MSG_REQ_SIZE(buf_size);

   spf_msg_t       msg;
   spf_msg_token_t token;
   token.token_data = 0;

   result = spf_msg_create_msg(&msg,
                               &buf_size,
                               SPF_MSG_CMD_GET_CFG,
                               &irm_ptr->irm_handle,
                               &token,
                               apm_handle_ptr,
                               irm_ptr->heap_id);
   if (AR_EOK != result)
   {
      AR_MSG(DBG_ERROR_PRIO, "IRM: Failed to allocate the msg");
      return result;
   }

   spf_msg_header_t             *msg_header_ptr     = (spf_msg_header_t *)msg.payload_ptr;
   spf_msg_cmd_param_data_cfg_t *param_data_cfg_ptr = (spf_msg_cmd_param_data_cfg_t *)&msg_header_ptr->payload_start;
   param_data_cfg_ptr->num_param_id_cfg             = 1;
   param_data_cfg_ptr->param_data_pptr              = (void **)(param_data_cfg_ptr + 1);
   apm_module_param_data_t *param_data_ptr =
      (apm_module_param_data_t *)((uint8_t *)param_data_cfg_ptr->param_data_pptr + (sizeof(void *)));
   *(param_data_cfg_ptr->param_data_pptr) = param_data_ptr;
   param_data_ptr->module_instance_id     = APM_MODULE_INSTANCE_ID;
   param_data_ptr->param_id               = APM_PARAM_ID_GET_CNTR_HANDLES;
   param_data_ptr->param_size             = param_payload_size;
   param_data_ptr->error_code             = AR_EOK;

   cntr_instances_handle_ptr                   = (param_id_cntr_instance_handles_t *)(param_data_ptr + 1);
   cntr_instances_handle_ptr->num_instance_ids = num_instances_ids;

   cntr_instances_payload_ptr = (param_id_cntr_instance_handles_payload_t *)(cntr_instances_handle_ptr + 1);

   irm_count_or_fill_instance_info(irm_ptr, IS_FILL_TRUE, NULL, cntr_instances_payload_ptr);

   if (AR_DID_FAIL(result = spf_msg_send_cmd(&msg, apm_handle_ptr)))
   {
      AR_MSG(DBG_ERROR_PRIO, "IRM: Failed to send spf msg to APM");
      spf_msg_return_msg(&msg);
   }

   return result;
}

ar_result_t irm_request_all_instance_handles(irm_t *irm_ptr)
{
   ar_result_t   result             = AR_EOK;
   uint32_t      param_payload_size = 0;
   uint32_t      buf_size           = 0;
   uint32_t      num_instances_ids  = 0;
   spf_handle_t *apm_handle_ptr     = apm_get_apm_handle();

   if (NULL == apm_handle_ptr)
   {
      AR_MSG(DBG_HIGH_PRIO, "IRM: NULL APM handle");
      return AR_EFAILED;
   }

   // Request max - current # of instances so we always get all possible instances when the enable-all
   num_instances_ids = g_irm_cmn_capabilities.max_num_containers_supported +
                       g_irm_cmn_capabilities.max_module_supported - irm_ptr->num_usecase_handles;
   AR_MSG(DBG_HIGH_PRIO, "IRM: Number of instance IDs requested = %lu", num_instances_ids);

   param_payload_size =
      sizeof(param_id_cntr_instance_handles_t) + (num_instances_ids * sizeof(param_id_cntr_instance_handles_payload_t));
   buf_size = sizeof(spf_msg_cmd_param_data_cfg_t) + (sizeof(void *)) + sizeof(apm_module_param_data_t) +
              ALIGN_8_BYTES(param_payload_size);
   buf_size = GET_SPF_MSG_REQ_SIZE(buf_size);

   spf_msg_t       msg;
   spf_msg_token_t token;
   token.token_data = 0;

   result = spf_msg_create_msg(&msg,
                               &buf_size,
                               SPF_MSG_CMD_GET_CFG,
                               &irm_ptr->irm_handle,
                               &token,
                               apm_handle_ptr,
                               irm_ptr->heap_id);
   if (AR_EOK != result)
   {
      AR_MSG(DBG_ERROR_PRIO, "IRM: Failed to allocate the msg");
      return result;
   }

   spf_msg_header_t             *msg_header_ptr     = (spf_msg_header_t *)msg.payload_ptr;
   spf_msg_cmd_param_data_cfg_t *param_data_cfg_ptr = (spf_msg_cmd_param_data_cfg_t *)&msg_header_ptr->payload_start;
   param_data_cfg_ptr->num_param_id_cfg             = 1;
   param_data_cfg_ptr->param_data_pptr              = (void **)(param_data_cfg_ptr + 1);
   apm_module_param_data_t *param_data_ptr =
      (apm_module_param_data_t *)((uint8_t *)param_data_cfg_ptr->param_data_pptr + (sizeof(void *)));
   *(param_data_cfg_ptr->param_data_pptr) = param_data_ptr;
   param_data_ptr->module_instance_id     = APM_MODULE_INSTANCE_ID;
   param_data_ptr->param_id               = APM_PARAM_ID_GET_ALL_CNTR_HANDLES;
   param_data_ptr->param_size             = param_payload_size;
   param_data_ptr->error_code             = AR_EOK;

   if (AR_DID_FAIL(result = spf_msg_send_cmd(&msg, apm_handle_ptr)))
   {
      AR_MSG(DBG_ERROR_PRIO, "IRM: Failed to send spf msg to APM");
      spf_msg_return_msg(&msg);
   }

   // After requesting for the instance handles, reset the count because otherwise all of the handles will be double
   // counted
   irm_ptr->num_usecase_handles = 0;

   return result;
}

/*----------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------*/
ar_result_t irm_parse_instance_handle_rsp(irm_t *irm_ptr, apm_module_param_data_t *param_data_ptr)
{
   ar_result_t    result                 = AR_EOK;
   spf_handle_t **mod_cntr_handles_pptr  = NULL;
   uint32_t       num_cntrs_with_modules = 0;

   if ((sizeof(param_id_cntr_instance_handles_t) > param_data_ptr->param_size))
   {
      AR_MSG(DBG_ERROR_PRIO, "IRM: Not enough param size");
      return AR_EFAILED;
   }

   bool_t is_handle_reset = (APM_PARAM_ID_RESET_CNTR_HANDLES == param_data_ptr->param_id);

   param_id_cntr_instance_handles_t *cntr_instance_handles_ptr =
      (param_id_cntr_instance_handles_t *)(param_data_ptr + 1);

   uint32_t total_param_size =
      (sizeof(param_id_cntr_instance_handles_t)) +
      (cntr_instance_handles_ptr->num_instance_ids * sizeof(param_id_cntr_instance_handles_payload_t));

   if (total_param_size > param_data_ptr->param_size)
   {
      AR_MSG(DBG_ERROR_PRIO, "IRM: Not enough param size");
      return AR_EFAILED;
   }

   if (AR_ENEEDMORE == param_data_ptr->error_code)
   {
      AR_MSG(DBG_ERROR_PRIO, "IRM: instance handle response returned ENEEDMORE (probably too many instances to query)");
      return AR_EFAILED;
   }
   else if (AR_EOK != param_data_ptr->error_code)
   {
      AR_MSG(DBG_ERROR_PRIO, "IRM: instance handle response failed with 0x%x", param_data_ptr->error_code);
      return AR_EFAILED;
   }

   param_id_cntr_instance_handles_payload_t *cntr_instance_handles_payload_arr_ptr =
      (param_id_cntr_instance_handles_payload_t *)(cntr_instance_handles_ptr + 1);

   AR_MSG(DBG_HIGH_PRIO, "IRM: num_instance_ids = %lu", cntr_instance_handles_ptr->num_instance_ids);

   if (cntr_instance_handles_ptr->num_instance_ids == 0)
   {
      // no handles from APM, nothing to do
      return AR_EOK;
   }

   if (FALSE == is_handle_reset)
   {
      uint32_t module_cntr_handle_set_size = (sizeof(spf_handle_t *) * cntr_instance_handles_ptr->num_instance_ids);
      mod_cntr_handles_pptr = (spf_handle_t **)posal_memory_malloc(module_cntr_handle_set_size, irm_ptr->heap_id);
      if (NULL == mod_cntr_handles_pptr)
      {
         AR_MSG(DBG_HIGH_PRIO, "IRM: Failed to allocate module cntr hdl set");
         return AR_EFAILED;
      }
      memset((void *)mod_cntr_handles_pptr, 0, module_cntr_handle_set_size);
      irm_ptr->num_usecase_handles += cntr_instance_handles_ptr->num_instance_ids;
   }
   else
   {
      irm_ptr->num_usecase_handles -= cntr_instance_handles_ptr->num_instance_ids;
   }

   for (uint32_t iid_idx = 0; iid_idx < cntr_instance_handles_ptr->num_instance_ids; iid_idx++)
   {
      param_id_cntr_instance_handles_payload_t *cntr_instance_handles_payload_ptr =
         &cntr_instance_handles_payload_arr_ptr[iid_idx];
      uint32_t block_id =
         (0 == cntr_instance_handles_payload_ptr->module_instance_id ? IRM_BLOCK_ID_CONTAINER : IRM_BLOCK_ID_MODULE);

      spf_list_node_t *block_node_ptr = irm_ptr->core.block_head_node_ptr;

      AR_MSG(DBG_HIGH_PRIO,
             "IRM: iid_idx = %lu, container iid = 0x%X, module iid 0x%X, handle = 0x%X",
             iid_idx,
             cntr_instance_handles_payload_ptr->container_instance_id,
             cntr_instance_handles_payload_ptr->module_instance_id,
             cntr_instance_handles_payload_ptr->handle_ptr);

      bool_t insert_block_flag = TRUE;
      for (; NULL != block_node_ptr; LIST_ADVANCE(block_node_ptr))
      {
         irm_node_obj_t *block_obj_ptr = (irm_node_obj_t *)block_node_ptr->obj_ptr;
         if ((NULL == block_obj_ptr) || (block_id != block_obj_ptr->id))
         {
            continue;
         }
         insert_block_flag = FALSE;
      }

      if (TRUE == insert_block_flag && irm_ptr->enable_all)
      {
         irm_node_obj_t *new_block_obj = irm_check_insert_node(irm_ptr, &irm_ptr->core.block_head_node_ptr, block_id);
         if (NULL == new_block_obj)
         {
            AR_MSG(DBG_ERROR_PRIO, "IRM: Failed to insert new block object. Block id: %d", block_id);
            return AR_EFAILED;
         }
         AR_MSG(DBG_MED_PRIO, "IRM: Inserted new block ID: %d from apm handle details", block_id);
      }
      block_node_ptr = irm_ptr->core.block_head_node_ptr;

      for (; NULL != block_node_ptr; LIST_ADVANCE(block_node_ptr))
      {
         irm_node_obj_t *block_obj_ptr = (irm_node_obj_t *)block_node_ptr->obj_ptr;
         if ((NULL == block_obj_ptr) || (block_id != block_obj_ptr->id))
         {
            continue;
         }

         bool_t insert_node_obj_flag = TRUE;

         spf_list_node_t *instance_node_ptr = block_obj_ptr->head_node_ptr;
         for (; NULL != instance_node_ptr; LIST_ADVANCE(instance_node_ptr))
         {
            irm_node_obj_t *instance_obj_ptr = (irm_node_obj_t *)instance_node_ptr->obj_ptr;
            if (NULL != instance_obj_ptr)
            {
               if (((IRM_BLOCK_ID_CONTAINER == block_id) &&
                    (instance_obj_ptr->id == cntr_instance_handles_payload_ptr->container_instance_id)) ||
                   ((IRM_BLOCK_ID_MODULE == block_id) &&
                    (instance_obj_ptr->id == cntr_instance_handles_payload_ptr->module_instance_id)))
               {
                  insert_node_obj_flag = FALSE;
               }
            }
         }

         if (TRUE == insert_node_obj_flag && irm_ptr->enable_all)
         {
            check_insert_instance_node(irm_ptr,
                                       &block_obj_ptr->head_node_ptr,
                                       block_id,
                                       cntr_instance_handles_payload_ptr->container_instance_id,
                                       cntr_instance_handles_payload_ptr->module_instance_id);
            AR_MSG(DBG_MED_PRIO,
                   "IRM: Inserted new Instance node MIID: 0x%x, CNTR ID: 0x%x into block ID: %d from apm handle "
                   "details",
                   cntr_instance_handles_payload_ptr->module_instance_id,
                   cntr_instance_handles_payload_ptr->container_instance_id,
                   block_obj_ptr->id);
         }

         instance_node_ptr = block_obj_ptr->head_node_ptr;

         for (; NULL != instance_node_ptr; LIST_ADVANCE(instance_node_ptr))
         {
            irm_node_obj_t *instance_obj_ptr = (irm_node_obj_t *)instance_node_ptr->obj_ptr;
            if (NULL != instance_obj_ptr)
            {
               if (((IRM_BLOCK_ID_CONTAINER == block_id) &&
                    (instance_obj_ptr->id == cntr_instance_handles_payload_ptr->container_instance_id)) ||
                   ((IRM_BLOCK_ID_MODULE == block_id) &&
                    (instance_obj_ptr->id == cntr_instance_handles_payload_ptr->module_instance_id)))
               {

                  if (TRUE == is_handle_reset)
                  {
                     instance_obj_ptr->handle_ptr    = NULL;
                     instance_obj_ptr->mod_mutex_ptr = NULL;
                     // If the handles are reset then the first rtm packet is not valid
                     spf_list_node_t *metric_node_ptr = instance_obj_ptr->head_node_ptr;
                     for (; NULL != metric_node_ptr; LIST_ADVANCE(metric_node_ptr))
                     {
                        irm_node_obj_t *metric_obj_ptr = (irm_node_obj_t *)metric_node_ptr->obj_ptr;
                        if (NULL == metric_obj_ptr)
                        {
                           continue;
                        }
                        metric_obj_ptr->is_first_time = TRUE;
                     }
                  }
                  else
                  {
                     if (NULL != cntr_instance_handles_payload_ptr->handle_ptr)
                     {
                        instance_obj_ptr->handle_ptr = cntr_instance_handles_payload_ptr->handle_ptr;
                        instance_obj_ptr->cntr_iid   = cntr_instance_handles_payload_ptr->container_instance_id;

                        // Update the list of unique container handles present in the incoming set param
                        // This info is later used to handshake with the corresponding containers
                        irm_update_cntr_handle_list(mod_cntr_handles_pptr,
                                                    &num_cntrs_with_modules,
                                                    instance_obj_ptr->handle_ptr);
                     }
                  }

                  AR_MSG(DBG_HIGH_PRIO,
                         "IRM: ID = 0x%X, handle = 0x%X, is_handle_reset = %lu",
                         instance_obj_ptr->id,
                         instance_obj_ptr->handle_ptr,
                         is_handle_reset);
               }
            }
         }
      }
   }

   if (FALSE == is_handle_reset)
   {
      bool_t IS_ENABLE_TRUE = TRUE;
      irm_handle_send_cntr_module_enable_info(irm_ptr, mod_cntr_handles_pptr, num_cntrs_with_modules, IS_ENABLE_TRUE);
      if (NULL != mod_cntr_handles_pptr)
      {
         posal_memory_free(mod_cntr_handles_pptr);
      }
   }

   return result;
}
