
/**
@file irm_cntr_if_utils.cpp

@brief IRM to APM interface utils.

================================================================================
Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
SPDX-License-Identifier: BSD-3-Clause-Clear
==============================================================================*/
// clang-format off
// clang-format on

#include "ar_msg.h"
#include "irm_i.h"
#include "apm_internal_if.h"
#include "irm_cntr_if.h"
#include "spf_list_utils.h"

typedef struct irm_handle_instance_info_t
{
   spf_handle_t    *handle_ptr;
   uint32_t         num_instance;
   uint32_t         num_metrics;
   uint32_t         cntr_iid;
   spf_list_node_t *instance_head_node_ptr;
} irm_handle_instance_info_t;

/*----------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------*/
static void irm_update_cntr_module_info_payload(irm_t           *irm_ptr,
                                                spf_handle_t    *handle_ptr,
                                                uint8_t         *payload_ptr,
                                                spf_list_node_t *instance_node_ptr)
{
   for (; NULL != instance_node_ptr; LIST_ADVANCE(instance_node_ptr))
   {
      irm_node_obj_t *instance_obj_ptr = (irm_node_obj_t *)instance_node_ptr->obj_ptr;
      if ((NULL == instance_obj_ptr) || (handle_ptr != instance_obj_ptr->handle_ptr))
      {
         continue;
      }

      cntr_param_id_mod_metric_info_t *mod_metric_info_ptr = (cntr_param_id_mod_metric_info_t *)payload_ptr;
      payload_ptr += sizeof(cntr_param_id_mod_metric_info_t);

      mod_metric_info_ptr->instance_id = instance_obj_ptr->id;
      mod_metric_info_ptr->num_metrics = 0;
      spf_list_node_t *metric_node_ptr = NULL;
      if (irm_ptr->enable_all)
      {
         spf_list_node_t *irm_cached_metric_head_ptr = NULL;
         irm_cached_metric_head_ptr = irm_find_node_from_id(irm_ptr->tmp_metric_payload_list, IRM_BLOCK_ID_MODULE);
         if (NULL == irm_cached_metric_head_ptr)
         {
            AR_MSG(DBG_MED_PRIO, "IRM: enable-all module metrics are disabled");
            continue;
         }
         else if (NULL == irm_cached_metric_head_ptr->obj_ptr)
         {
            AR_MSG(DBG_ERROR_PRIO, "IRM: warning: cached IRM module metrics node is null");
			break;
         }

         metric_node_ptr = ((irm_node_obj_t *)irm_cached_metric_head_ptr->obj_ptr)->head_node_ptr;
      }
      else
      {
         metric_node_ptr = instance_obj_ptr->head_node_ptr;
      }
      for (; NULL != metric_node_ptr; LIST_ADVANCE(metric_node_ptr))
      {
         irm_node_obj_t *metric_obj_ptr = (irm_node_obj_t *)metric_node_ptr->obj_ptr;
         if (NULL == metric_obj_ptr)
         {
            continue;
         }
         cntr_param_id_mod_metric_payload_t *mod_metric_payload_ptr = (cntr_param_id_mod_metric_payload_t *)payload_ptr;
         payload_ptr += sizeof(cntr_param_id_mod_metric_payload_t);

         mod_metric_info_ptr->num_metrics++;
         mod_metric_payload_ptr->metric_id   = metric_obj_ptr->id;
         mod_metric_payload_ptr->payload_ptr = NULL;
         AR_MSG(DBG_HIGH_PRIO,
                "IRM: num_metrics = %lu, metric_id = 0x%X",
                mod_metric_info_ptr->num_metrics,
                mod_metric_payload_ptr->metric_id);
      }
   }
}

/*----------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------*/
static void irm_create_temp_enable_disable_list(irm_t                      *irm_ptr,
                                                irm_enable_disable_block_t *set_enable_disable_ptr,
                                                irm_handle_instance_info_t *mod_enable_disable_info_ptr,
                                                irm_node_obj_t             *instance_obj_ptr,
                                                bool_t                      is_enable)
{
   irm_node_obj_t *local_instance_obj_ptr = irm_check_insert_node(irm_ptr,
                                                                  &mod_enable_disable_info_ptr->instance_head_node_ptr,
                                                                  set_enable_disable_ptr->instance_id);

   if (NULL == local_instance_obj_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "IRM: Failed to insert instance node");
      return;
   }

   local_instance_obj_ptr->cntr_iid   = instance_obj_ptr->cntr_iid;
   local_instance_obj_ptr->handle_ptr = instance_obj_ptr->handle_ptr;
   mod_enable_disable_info_ptr->num_instance++;

   if ((FALSE == is_enable) && (0 == set_enable_disable_ptr->num_metric_ids))
   {
      spf_list_node_t *metric_node_ptr = instance_obj_ptr->head_node_ptr;
      for (; NULL != metric_node_ptr; LIST_ADVANCE(metric_node_ptr))
      {
         irm_node_obj_t *metric_obj_ptr = (irm_node_obj_t *)metric_node_ptr->obj_ptr;
         if (NULL == metric_obj_ptr)
         {
            continue;
         }
         irm_node_obj_t *local_metric_obj_ptr =
            irm_check_insert_node(irm_ptr, &local_instance_obj_ptr->head_node_ptr, metric_obj_ptr->id);
         if (NULL == local_metric_obj_ptr)
         {
            AR_MSG(DBG_ERROR_PRIO, "IRM: Failed to insert node");
         }
         else
         {
            mod_enable_disable_info_ptr->num_metrics++;
         }
      }
   }
   else
   {
      uint32_t *metric_ids_ptr = (uint32_t *)(set_enable_disable_ptr + 1);
      for (uint32_t metric_count = 0; metric_count < set_enable_disable_ptr->num_metric_ids; metric_count++)
      {
         irm_node_obj_t *local_metric_obj_ptr =
            irm_check_insert_node(irm_ptr, &local_instance_obj_ptr->head_node_ptr, metric_ids_ptr[metric_count]);
         if (NULL == local_metric_obj_ptr)
         {
            AR_MSG(DBG_ERROR_PRIO, "IRM: Failed to insert node");
         }
         else
         {
            mod_enable_disable_info_ptr->num_metrics++;
         }
      }
   }
}

/*----------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------*/
static void irm_delete_temp_enable_disable_list(irm_t                      *irm_ptr,
                                                irm_handle_instance_info_t *mod_enable_disable_info_ptr,
                                                uint32_t                    num_handles)
{
   for (uint32_t handle_idx = 0; handle_idx < num_handles; handle_idx++)
   {
      irm_handle_instance_info_t *per_cntr_mod_enable_disable_info_ptr = &mod_enable_disable_info_ptr[handle_idx];
      spf_list_node_t            *instance_node_ptr = per_cntr_mod_enable_disable_info_ptr->instance_head_node_ptr;
      for (; NULL != instance_node_ptr; LIST_ADVANCE(instance_node_ptr))
      {
         irm_node_obj_t *instance_obj_ptr = (irm_node_obj_t *)instance_node_ptr->obj_ptr;
         if (NULL == instance_obj_ptr)
         {
            continue;
         }
         if (NULL != instance_obj_ptr->head_node_ptr)
         {
            irm_delete_list(&instance_obj_ptr->head_node_ptr);
         }
      }
      if (NULL != per_cntr_mod_enable_disable_info_ptr->instance_head_node_ptr)
      {
         irm_delete_list(&per_cntr_mod_enable_disable_info_ptr->instance_head_node_ptr);
      }
   }
}

/*----------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------*/
static irm_node_obj_t *irm_get_cntr_enable_disable_instance_obj(irm_t                      *irm_ptr,
                                                                irm_enable_disable_block_t *set_enable_disable_ptr)
{
   spf_list_node_t *block_node_ptr    = NULL;
   irm_node_obj_t  *block_obj_ptr     = NULL;
   spf_list_node_t *instance_node_ptr = NULL;
   irm_node_obj_t  *instance_obj_ptr  = NULL;
   block_node_ptr = irm_find_node_from_id(irm_ptr->core.block_head_node_ptr, set_enable_disable_ptr->block_id);

   if ((NULL == block_node_ptr) || (NULL == block_node_ptr->obj_ptr))
   {
      return NULL;
   }

   block_obj_ptr     = block_node_ptr->obj_ptr;
   instance_node_ptr = irm_find_node_from_id(block_obj_ptr->head_node_ptr, set_enable_disable_ptr->instance_id);

   if ((NULL == instance_node_ptr) || (NULL == instance_node_ptr->obj_ptr))
   {
      return NULL;
   }

   instance_obj_ptr = instance_node_ptr->obj_ptr;
   if (NULL == instance_obj_ptr->handle_ptr)
   {
      return NULL;
   }

   if (IRM_BLOCK_ID_MODULE == set_enable_disable_ptr->block_id)
   {
      return instance_obj_ptr;
   }
   else if (IRM_BLOCK_ID_CONTAINER == set_enable_disable_ptr->block_id)
   {
      uint32_t *metrid_ids_ptr = (uint32_t *)(set_enable_disable_ptr + 1);
      for (uint32_t metric_idx = 0; metric_idx < set_enable_disable_ptr->num_metric_ids; metric_idx++)
      {
         if (IRM_METRIC_ID_HEAP_INFO == metrid_ids_ptr[metric_idx])
         {
            return instance_obj_ptr;
         }
      }
   }
   else
   {
      return NULL;
   }

   return NULL;
}

/*----------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------*/
static void irm_create_cntr_enable_disable_info_from_payload(irm_t                      *irm_ptr,
                                                             uint8_t                    *payload_ptr,
                                                             irm_handle_instance_info_t *mod_enable_disable_info_ptr,
                                                             uint32_t                   *num_handles_ptr,
                                                             bool_t                      is_enable)
{
   uint32_t num_handles = 0;

   param_id_enable_disable_metrics_t *param_ptr = (param_id_enable_disable_metrics_t *)payload_ptr;
   payload_ptr += sizeof(param_id_enable_disable_metrics_t);

   for (uint32_t block_count = 0; block_count < param_ptr->num_blocks; block_count++)
   {
      irm_enable_disable_block_t *set_enable_disable_ptr = (irm_enable_disable_block_t *)(payload_ptr);
      payload_ptr += sizeof(irm_enable_disable_block_t) + (sizeof(uint32_t) * set_enable_disable_ptr->num_metric_ids);

      irm_node_obj_t *instance_obj_ptr = irm_get_cntr_enable_disable_instance_obj(irm_ptr, set_enable_disable_ptr);
      if (NULL == instance_obj_ptr)
      {
         continue;
      }

      uint32_t handle_idx = 0;
      for (; handle_idx < num_handles; handle_idx++)
      {
         if (mod_enable_disable_info_ptr[handle_idx].handle_ptr == instance_obj_ptr->handle_ptr)
         {
            if ((0 != mod_enable_disable_info_ptr[handle_idx].cntr_iid) &&
                (instance_obj_ptr->cntr_iid != mod_enable_disable_info_ptr[handle_idx].cntr_iid))
            {
               AR_MSG(DBG_ERROR_PRIO, "IRM: WARNING, multiple container iid detected with same handles");
            }
            mod_enable_disable_info_ptr[handle_idx].cntr_iid = instance_obj_ptr->cntr_iid;
            break;
         }
      }
      if (handle_idx == num_handles)
      {
         mod_enable_disable_info_ptr[handle_idx].handle_ptr = instance_obj_ptr->handle_ptr;
         mod_enable_disable_info_ptr[handle_idx].cntr_iid   = instance_obj_ptr->cntr_iid;
         num_handles++;
      }

      if (IRM_BLOCK_ID_MODULE == set_enable_disable_ptr->block_id)
      {
         irm_create_temp_enable_disable_list(irm_ptr,
                                             set_enable_disable_ptr,
                                             &mod_enable_disable_info_ptr[handle_idx],
                                             instance_obj_ptr,
                                             is_enable);
      }
   }
   (*num_handles_ptr) = num_handles;
}

/*----------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------*/
static ar_result_t irm_create_and_fill_cntr_msg_header(irm_t                    *irm_ptr,
                                                       spf_msg_t                *msg_ptr,
                                                       spf_handle_t             *handle_ptr,
                                                       apm_module_param_data_t **param_data_pptr,
                                                       uint32_t                  num_modules,
                                                       uint32_t                  num_metrics,
                                                       uint32_t                  cntr_iid,
                                                       bool_t                    is_enable)
{
   ar_result_t     result             = AR_EOK;
   uint32_t        buf_size           = 0;
   uint32_t        param_payload_size = 0;
   spf_msg_token_t token;
   token.token_data = 0;

   param_payload_size = sizeof(cntr_param_id_get_prof_info_t) +
                        (num_modules * sizeof(cntr_param_id_mod_metric_info_t)) +
                        (num_metrics * sizeof(cntr_param_id_mod_metric_payload_t));

   buf_size = sizeof(spf_msg_cmd_param_data_cfg_t) + (sizeof(void *)) + sizeof(apm_module_param_data_t) +
              ALIGN_8_BYTES(param_payload_size);
   buf_size = GET_SPF_MSG_REQ_SIZE(buf_size);

   result = spf_msg_create_msg(msg_ptr,
                               &buf_size,
                               SPF_MSG_CMD_GET_CFG,
                               &irm_ptr->irm_handle,
                               &token,
                               handle_ptr,
                               irm_ptr->heap_id);
   if (AR_EOK != result)
   {
      AR_MSG(DBG_ERROR_PRIO, "IRM: Failed to allocate the msg");
      return AR_EFAILED;
   }

   spf_msg_header_t             *msg_header_ptr     = (spf_msg_header_t *)msg_ptr->payload_ptr;
   spf_msg_cmd_param_data_cfg_t *param_data_cfg_ptr = (spf_msg_cmd_param_data_cfg_t *)&msg_header_ptr->payload_start;
   param_data_cfg_ptr->num_param_id_cfg             = 1;
   param_data_cfg_ptr->param_data_pptr              = (void **)(param_data_cfg_ptr + 1);

   apm_module_param_data_t *param_data_ptr =
      (apm_module_param_data_t *)((uint8_t *)param_data_cfg_ptr->param_data_pptr + (sizeof(void *)));
   *(param_data_cfg_ptr->param_data_pptr) = param_data_ptr;

   (*param_data_pptr)                 = param_data_ptr;
   param_data_ptr->module_instance_id = cntr_iid;
   param_data_ptr->param_id           = CNTR_PARAM_ID_GET_PROF_INFO;
   param_data_ptr->param_size         = param_payload_size;
   param_data_ptr->error_code         = AR_EOK;

   cntr_param_id_get_prof_info_t *cntr_param_id_get_prof_info = (cntr_param_id_get_prof_info_t *)(param_data_ptr + 1);

   cntr_param_id_get_prof_info->cntr_mutex_ptr = NULL;
   cntr_param_id_get_prof_info->heap_id        = irm_ptr->heap_id;
   cntr_param_id_get_prof_info->is_enable      = is_enable;
   cntr_param_id_get_prof_info->num_modules    = num_modules;
   return result;
}

static ar_result_t irm_send_handles_to_cntrs_internal(irm_t                      *irm_ptr,
                                                      uint32_t                    num_handles,
                                                      irm_handle_instance_info_t *mod_enable_disable_info_ptr,
                                                      bool_t                      is_enable)
{
   ar_result_t result = AR_EOK;
   for (uint32_t handle_idx = 0; handle_idx < num_handles; handle_idx++)
   {
      irm_handle_instance_info_t *per_cntr_mod_enable_disable_info_ptr = &mod_enable_disable_info_ptr[handle_idx];
      spf_handle_t               *handle_ptr  = per_cntr_mod_enable_disable_info_ptr->handle_ptr;
      uint32_t                    num_modules = per_cntr_mod_enable_disable_info_ptr->num_instance;
      uint32_t                    num_metrics = per_cntr_mod_enable_disable_info_ptr->num_metrics;
      uint32_t                    cntr_iid    = per_cntr_mod_enable_disable_info_ptr->cntr_iid;
      AR_MSG(DBG_HIGH_PRIO,
             "IRM: Enable/disable = %lu, num_modules = %lu, num_metrics = %lu, cntr_iid 0x%X",
             is_enable,
             num_modules,
             num_metrics,
             cntr_iid);

      if (0 == cntr_iid)
      {
         continue;
      }
      apm_module_param_data_t *param_data_ptr = NULL;
      spf_msg_t                msg;

      ar_result_t local_result = irm_create_and_fill_cntr_msg_header(irm_ptr,
                                                                     &msg,
                                                                     handle_ptr,
                                                                     &param_data_ptr,
                                                                     num_modules,
                                                                     num_metrics,
                                                                     cntr_iid,
                                                                     is_enable);
      if ((AR_EOK != local_result) || (NULL == param_data_ptr))
      {
         result |= local_result;
         AR_MSG(DBG_ERROR_PRIO,
                "IRM:Failed to fill cntr msg header result = %lu, param data ptr = 0x%X",
                local_result,
                param_data_ptr);
         continue;
      }

      if ((0 != num_modules) && (0 != num_metrics))
      {
         cntr_param_id_get_prof_info_t *cntr_param_id_get_prof_info =
            (cntr_param_id_get_prof_info_t *)(param_data_ptr + 1);
         uint8_t *payload_ptr = (uint8_t *)(cntr_param_id_get_prof_info + 1);

         irm_update_cntr_module_info_payload(irm_ptr,
                                             handle_ptr,
                                             payload_ptr,
                                             per_cntr_mod_enable_disable_info_ptr->instance_head_node_ptr);
      }

      if (AR_DID_FAIL(result = spf_msg_send_cmd(&msg, handle_ptr)))
      {
         AR_MSG(DBG_ERROR_PRIO, "IRM: Failed to send spf msg to Container");
         spf_msg_return_msg(&msg);
      }
   }
   return result;
}

ar_result_t irm_handle_send_cntr_module_enable_disable_info(irm_t *irm_ptr, uint8_t *payload_ptr, bool_t is_enable)
{
   ar_result_t result      = AR_EOK;
   uint32_t    num_handles = 0;
   uint32_t    num_blocks  = 0;

   if (NULL == payload_ptr)
   {
      return AR_EFAILED;
   }
   else
   {
      param_id_enable_disable_metrics_t *param_ptr = (param_id_enable_disable_metrics_t *)payload_ptr;
      num_blocks                                   = param_ptr->num_blocks;
   }

   if (0 == num_blocks)
   {
      return AR_EOK;
   }

   irm_handle_instance_info_t *mod_enable_disable_info_ptr =
      posal_memory_malloc(sizeof(irm_handle_instance_info_t) * num_blocks, irm_ptr->heap_id);
   if (NULL == mod_enable_disable_info_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "IRM: Failed to allocate per_cntr_mod_disable_info");
      return AR_ENOMEMORY;
   }
   memset((void *)mod_enable_disable_info_ptr, 0, sizeof(irm_handle_instance_info_t) * num_blocks);
   irm_create_cntr_enable_disable_info_from_payload(irm_ptr,
                                                    payload_ptr,
                                                    mod_enable_disable_info_ptr,
                                                    &num_handles,
                                                    is_enable);

   result = irm_send_handles_to_cntrs_internal(irm_ptr, num_handles, mod_enable_disable_info_ptr, is_enable);

   irm_delete_temp_enable_disable_list(irm_ptr, mod_enable_disable_info_ptr, num_handles);
   if (NULL != mod_enable_disable_info_ptr)
   {
      posal_memory_free(mod_enable_disable_info_ptr);
   }
   return result;
}

/*----------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------*/
static void irm_cal_module_metric_info_with_handle(irm_t        *irm_ptr,
                                                   spf_handle_t *handle_ptr,
                                                   uint32_t     *num_modules_ptr,
                                                   uint32_t     *num_metrics_ptr,
                                                   bool_t       *send_info_ptr,
                                                   uint32_t     *cntr_iid_ptr)
{
   (*num_modules_ptr) = 0;
   (*num_metrics_ptr) = 0;
   (*send_info_ptr)   = FALSE;
   (*cntr_iid_ptr)    = 0;

   spf_list_node_t *block_node_ptr = irm_ptr->core.block_head_node_ptr;
   for (; NULL != block_node_ptr; LIST_ADVANCE(block_node_ptr))
   {
      irm_node_obj_t *block_obj_ptr = (irm_node_obj_t *)block_node_ptr->obj_ptr;

      if ((NULL == block_obj_ptr) ||
          (!((IRM_BLOCK_ID_MODULE == block_obj_ptr->id) || (IRM_BLOCK_ID_CONTAINER == block_obj_ptr->id))))
      {
         continue;
      }

      AR_MSG(DBG_MED_PRIO, "IRM: Block id = 0x%X", block_obj_ptr->id);

      spf_list_node_t *instance_node_ptr = block_obj_ptr->head_node_ptr;
      for (; NULL != instance_node_ptr; LIST_ADVANCE(instance_node_ptr))
      {
         irm_node_obj_t *instance_obj_ptr = (irm_node_obj_t *)instance_node_ptr->obj_ptr;
         if ((NULL == instance_obj_ptr) || (handle_ptr != instance_obj_ptr->handle_ptr))
         {
            continue;
         }

         AR_MSG(DBG_MED_PRIO,
                "IRM: Instance id = 0x%X, cntr_iid = 0x%X",
                instance_obj_ptr->id,
                instance_obj_ptr->cntr_iid);

         if ((0 != (*cntr_iid_ptr)) && (instance_obj_ptr->cntr_iid != (*cntr_iid_ptr)))
         {
            AR_MSG(DBG_ERROR_PRIO, "IRM: WARNING, multiple container iid detected with same handles");
         }

         (*cntr_iid_ptr) = instance_obj_ptr->cntr_iid;

         if (IRM_BLOCK_ID_MODULE == block_obj_ptr->id)
         {
            (*num_modules_ptr)++;
            AR_MSG(DBG_MED_PRIO, "IRM: num_modules = %lu", (*num_modules_ptr));
            if (irm_ptr->enable_all)
            {
               // in the enable all case the number of metrics = # availible metrics. Index = BID-1 since BIDs start at
               // 1
               (*num_metrics_ptr) += g_capability_list_ptr[IRM_BLOCK_ID_MODULE - 1].num_metrics;
               AR_MSG(DBG_MED_PRIO, "IRM: num_metrics = %lu", (*num_metrics_ptr));
            }
            else
            {
               // otherwise, walk the list to see the # of metrics
               spf_list_node_t *metric_node_ptr = instance_obj_ptr->head_node_ptr;
               for (; NULL != metric_node_ptr; LIST_ADVANCE(metric_node_ptr))
               {
                  irm_node_obj_t *metric_obj_ptr = (irm_node_obj_t *)metric_node_ptr->obj_ptr;
                  if (NULL != metric_obj_ptr)
                  {
                     (*num_metrics_ptr)++;
                     AR_MSG(DBG_MED_PRIO, "IRM: num_metrics = %lu", (*num_metrics_ptr));
                  }
               }
            }
         }
         else
         {
            if (irm_ptr->enable_all)
            {
               (*send_info_ptr) = TRUE;
            }
            else
            {
               spf_list_node_t *metric_node_ptr = instance_obj_ptr->head_node_ptr;
               for (; NULL != metric_node_ptr; LIST_ADVANCE(metric_node_ptr))
               {
                  irm_node_obj_t *metric_obj_ptr = (irm_node_obj_t *)metric_node_ptr->obj_ptr;
                  if (NULL != metric_obj_ptr)
                  {
                     if (IRM_METRIC_ID_HEAP_INFO == metric_obj_ptr->id)
                     {
                        (*send_info_ptr) = TRUE;
                     }
                  }
               }
            }
         }
      }
   }

   if ((0 != (*num_modules_ptr)) && (0 != (*num_metrics_ptr)))
   {
      (*send_info_ptr) = TRUE;
   }
}

/*----------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------*/
void irm_handle_send_cntr_module_enable_info(irm_t         *irm_ptr,
                                             spf_handle_t **module_cntr_handle_set_pptr,
                                             uint32_t       num_cntrs_with_modules,
                                             uint32_t       is_enable)
{
   ar_result_t result = AR_EOK;
   if (0 == num_cntrs_with_modules || NULL == module_cntr_handle_set_pptr)
   {
      return;
   }

   for (uint32_t idx = 0; idx < num_cntrs_with_modules; idx++)
   {
      uint32_t      num_modules = 0;
      uint32_t      num_metrics = 0;
      spf_handle_t *handle_ptr  = module_cntr_handle_set_pptr[idx];
      bool_t        send_info   = FALSE;
      uint32_t      cntr_iid    = 0;
      irm_cal_module_metric_info_with_handle(irm_ptr, handle_ptr, &num_modules, &num_metrics, &send_info, &cntr_iid);

      AR_MSG(DBG_HIGH_PRIO,
             "IRM: handle_ptr = 0x%X, send_info = %lu, cntr_iid = 0x%X, num_modules = %lu, num_metrics = %lu",
             handle_ptr,
             send_info,
             cntr_iid,
             num_modules,
             num_metrics);

      if ((FALSE == send_info) || (0 == cntr_iid))
      {
         continue;
      }

      apm_module_param_data_t *param_data_ptr = NULL;
      spf_msg_t                msg;

      ar_result_t local_result = irm_create_and_fill_cntr_msg_header(irm_ptr,
                                                                     &msg,
                                                                     handle_ptr,
                                                                     &param_data_ptr,
                                                                     num_modules,
                                                                     num_metrics,
                                                                     cntr_iid,
                                                                     is_enable);
      if ((AR_EOK != local_result) || (NULL == param_data_ptr))
      {
         AR_MSG(DBG_ERROR_PRIO, "IRM: Failed to create and fill cntr msg header, error = 0x%X", local_result);
         result |= local_result;
         continue;
      }

      if ((0 != num_modules) && (0 != num_metrics))
      {
         cntr_param_id_get_prof_info_t *cntr_param_id_get_prof_info =
            (cntr_param_id_get_prof_info_t *)(param_data_ptr + 1);
         uint8_t *payload_ptr = (uint8_t *)(cntr_param_id_get_prof_info + 1);

         spf_list_node_t *block_node_ptr = irm_ptr->core.block_head_node_ptr;
         for (; NULL != block_node_ptr; LIST_ADVANCE(block_node_ptr))
         {
            irm_node_obj_t *block_obj_ptr = (irm_node_obj_t *)block_node_ptr->obj_ptr;

            if ((NULL != block_obj_ptr) && (IRM_BLOCK_ID_MODULE == block_obj_ptr->id))
            {
               spf_list_node_t *instance_node_ptr = block_obj_ptr->head_node_ptr;
               irm_update_cntr_module_info_payload(irm_ptr, handle_ptr, payload_ptr, instance_node_ptr);
               break;
            }
         }
      }
      AR_MSG(DBG_LOW_PRIO, "IRM: sending spf msg to Container 0x%x", cntr_iid);

      if (AR_DID_FAIL(result = spf_msg_send_cmd(&msg, handle_ptr)))
      {
         AR_MSG(DBG_ERROR_PRIO, "IRM: Failed to send spf msg to Container");
         spf_msg_return_msg(&msg);
      }
   }
}

/*----------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------*/
static void irm_cntr_rsp_update_metric_payload_info(irm_t                              *irm_ptr,
                                                    cntr_param_id_get_prof_info_t      *cntr_param_id_get_prof_info,
                                                    cntr_param_id_mod_metric_info_t    *mod_metric_info_ptr,
                                                    cntr_param_id_mod_metric_payload_t *mod_metric_payload_ptr)
{
   spf_list_node_t *block_node_ptr = irm_ptr->core.block_head_node_ptr;
   for (; NULL != block_node_ptr; LIST_ADVANCE(block_node_ptr))
   {
      irm_node_obj_t *block_obj_ptr = (irm_node_obj_t *)block_node_ptr->obj_ptr;

      if ((NULL == block_obj_ptr) || (IRM_BLOCK_ID_MODULE != block_obj_ptr->id))
      {
         continue;
      }

      spf_list_node_t *instance_node_ptr = block_obj_ptr->head_node_ptr;
      for (; NULL != instance_node_ptr; LIST_ADVANCE(instance_node_ptr))
      {
         irm_node_obj_t *instance_obj_ptr = (irm_node_obj_t *)instance_node_ptr->obj_ptr;
         if ((NULL == instance_obj_ptr) || (instance_obj_ptr->id != mod_metric_info_ptr->instance_id))
         {
            continue;
         }
         instance_obj_ptr->mod_mutex_ptr = cntr_param_id_get_prof_info->cntr_mutex_ptr;
         instance_obj_ptr->heap_id       = mod_metric_info_ptr->module_heap_id & IRM_HEAP_ID_MASK;
         for (uint32_t metric_idx = 0; metric_idx < mod_metric_info_ptr->num_metrics; metric_idx++)
         {
            spf_list_node_t *metric_node_ptr = instance_obj_ptr->head_node_ptr;
            for (; NULL != metric_node_ptr; LIST_ADVANCE(metric_node_ptr))
            {
               irm_node_obj_t *metric_obj_ptr = (irm_node_obj_t *)metric_node_ptr->obj_ptr;
               if ((NULL == metric_obj_ptr) || (metric_obj_ptr->id != mod_metric_payload_ptr[metric_idx].metric_id))
               {
                  continue;
               }

               metric_obj_ptr->metric_info.current_mod_statistics_ptr =
                  (void *)mod_metric_payload_ptr[metric_idx].payload_ptr;

               AR_MSG(DBG_HIGH_PRIO,
                      "IRM: module iid = 0x%X, metric id = 0x%X, metric id payload = 0x%X",
                      instance_obj_ptr->id,
                      metric_obj_ptr->id,
                      metric_obj_ptr->metric_info.current_mod_statistics_ptr);
            }
         }
      }
   }
}

/*----------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------*/
static void irm_parse_cntr_rsp_update_cntr_heap_id(irm_t *irm_ptr, uint32_t cntr_instance_id, uint32_t cntr_heap_id)
{
   spf_list_node_t *block_node_ptr = irm_find_node_from_id(irm_ptr->core.block_head_node_ptr, IRM_BLOCK_ID_CONTAINER);
   if ((NULL != block_node_ptr) && (NULL != block_node_ptr->obj_ptr))
   {
      irm_node_obj_t  *block_obj_ptr     = block_node_ptr->obj_ptr;
      spf_list_node_t *instance_node_ptr = irm_find_node_from_id(block_obj_ptr->head_node_ptr, cntr_instance_id);
      if (NULL != instance_node_ptr && NULL != instance_node_ptr->obj_ptr)
      {
         irm_node_obj_t *instance_obj_ptr = (irm_node_obj_t *)instance_node_ptr->obj_ptr;
         instance_obj_ptr->heap_id        = cntr_heap_id & IRM_HEAP_ID_MASK;
      }
   }
}

/*----------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------*/
ar_result_t irm_parse_cntr_rsp(irm_t *irm_ptr, apm_module_param_data_t *param_data_ptr)
{
   ar_result_t result           = AR_EOK;
   uint32_t    total_param_size = param_data_ptr->param_size;
   uint32_t    required_size    = sizeof(cntr_param_id_get_prof_info_t);
   uint8_t    *payload_ptr      = (uint8_t *)(param_data_ptr + 1);

   if (required_size > total_param_size)
   {
      AR_MSG(DBG_ERROR_PRIO, "IRM: insufficient size, given = %lu, required = %lu ", total_param_size, required_size);
      return AR_EBADPARAM;
   }
   cntr_param_id_get_prof_info_t *cntr_param_id_get_prof_info_ptr = (cntr_param_id_get_prof_info_t *)payload_ptr;
   payload_ptr += sizeof(cntr_param_id_get_prof_info_t);

   irm_parse_cntr_rsp_update_cntr_heap_id(irm_ptr,
                                          param_data_ptr->module_instance_id /** cntr instance id*/,
                                          cntr_param_id_get_prof_info_ptr->cntr_heap_id);

   for (uint32_t mod_idx = 0; mod_idx < cntr_param_id_get_prof_info_ptr->num_modules; mod_idx++)
   {
      required_size += sizeof(cntr_param_id_mod_metric_info_t);
      if (required_size > total_param_size)
      {
         AR_MSG(DBG_ERROR_PRIO,
                "IRM: insufficient size, given = %lu, required = %lu ",
                total_param_size,
                required_size);
         return AR_EBADPARAM;
      }
      cntr_param_id_mod_metric_info_t *mod_metric_info_ptr = (cntr_param_id_mod_metric_info_t *)payload_ptr;
      payload_ptr += sizeof(cntr_param_id_mod_metric_info_t);

      required_size += (mod_metric_info_ptr->num_metrics * sizeof(cntr_param_id_mod_metric_payload_t));
      if (required_size > total_param_size)
      {
         AR_MSG(DBG_ERROR_PRIO,
                "IRM: insufficient size, given = %lu, required = %lu ",
                total_param_size,
                required_size);
         return AR_EBADPARAM;
      }

      cntr_param_id_mod_metric_payload_t *mod_metric_payload_ptr = (cntr_param_id_mod_metric_payload_t *)payload_ptr;
      payload_ptr += (mod_metric_info_ptr->num_metrics * sizeof(cntr_param_id_mod_metric_payload_t));

      AR_MSG(DBG_HIGH_PRIO,
             "IRM: is_enable = %lu mod_idx = %lu, module iid = 0x%X, num metrics = %lu, mutex ptr = 0x%X",
             cntr_param_id_get_prof_info_ptr->is_enable,
             mod_idx,
             mod_metric_info_ptr->instance_id,
             mod_metric_info_ptr->num_metrics,
             cntr_param_id_get_prof_info_ptr->cntr_mutex_ptr);

      irm_cntr_rsp_update_metric_payload_info(irm_ptr,
                                              cntr_param_id_get_prof_info_ptr,
                                              mod_metric_info_ptr,
                                              mod_metric_payload_ptr);
   }

   return result;
}

/*----------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------*/
void irm_update_cntr_handle_list(spf_handle_t **mod_cntr_handles_pptr,
                                 uint32_t      *num_cntrs_with_modules_ptr,
                                 spf_handle_t  *handle_ptr)
{
   for (uint32_t idx = 0; idx < (*num_cntrs_with_modules_ptr); idx++)
   {
      if (handle_ptr == mod_cntr_handles_pptr[idx])
      {
         return;
      }
   }
   mod_cntr_handles_pptr[(*num_cntrs_with_modules_ptr)] = handle_ptr;
   (*num_cntrs_with_modules_ptr)++;
}
