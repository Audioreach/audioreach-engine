/**
 * \file cu_soft_timer_fwk_ext.c
 *
 * \brief
 *     Implementation of soft_timer fwk extn in container utils
 *
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear */

/* =======================================================================
Includes
========================================================================== */
#include "cu_i.h"
#include "cu_soft_timer_fwk_ext.h"

/* =======================================================================
Static function declarations
========================================================================== */
static ar_result_t cu_fwk_extn_soft_timer_delete(cu_base_t *base_ptr, spf_list_node_t **cur_timer_node_pptr);

/* =======================================================================
Function definitions
========================================================================== */
/* Delete specific node in timer list */
static ar_result_t cu_fwk_extn_soft_timer_delete(cu_base_t *base_ptr, spf_list_node_t **cur_timer_node_pptr)
{
   ar_result_t result = AR_EOK;

   spf_list_node_t *cur_timer_node_ptr = *cur_timer_node_pptr;
   cu_timer_node_t *   timer_node_ptr     = (cu_timer_node_t *)cur_timer_node_ptr->obj_ptr;
   CU_MSG(base_ptr->gu_ptr->log_id, DBG_HIGH_PRIO, "Deleting timer node with id %d", timer_node_ptr->timer_id);

   result = posal_timer_destroy(&timer_node_ptr->one_shot_timer);
   if (AR_EOK != result)
   {
      CU_MSG(base_ptr->gu_ptr->log_id, DBG_ERROR_PRIO, "timer delete failed");
      return result;
   }

   cu_deinit_signal(base_ptr, &timer_node_ptr->timer_signal_ptr);

   // Add it back to available bit mask
   cu_release_bit_in_bit_mask(base_ptr, timer_node_ptr->channel_bit_mask);

   result = spf_list_delete_node_and_free_obj(cur_timer_node_pptr, &base_ptr->timer_list_ptr, TRUE /* pool_used */);
   if (AR_EOK != result)
   {
      CU_MSG(base_ptr->gu_ptr->log_id, DBG_ERROR_PRIO, "gk list delete timer node failed");
      return result;
   }
   return result;
}

/* Enable timer and add node to the list*/
ar_result_t cu_fwk_extn_soft_timer_start(cu_base_t *  base_ptr,
                                         gu_module_t *module_ptr,
                                         uint32_t     timer_id,
                                         int64_t      duration_us)
{
   INIT_EXCEPTION_HANDLING
   ar_result_t result   = AR_EOK;
   uint32_t    bit_mask = 0;

   // Create timer obj and save info
   cu_timer_node_t *timer_node_ptr = (cu_timer_node_t *)posal_memory_malloc(sizeof(cu_timer_node_t), base_ptr->heap_id);
   if (NULL == timer_node_ptr)
   {
      CU_MSG(base_ptr->gu_ptr->log_id, DBG_ERROR_PRIO, "Failed to malloc timer memory");
      return AR_ENOMEMORY;
   }

   timer_node_ptr->module_iid       = module_ptr->module_instance_id;
   timer_node_ptr->timer_id         = timer_id;
   timer_node_ptr->timer_signal_ptr = NULL;

   bit_mask = cu_request_bit_in_bit_mask(&base_ptr->available_bit_mask);

   TRY(result, cu_init_signal(base_ptr, bit_mask, cu_fwk_extn_soft_timer_expired, &timer_node_ptr->timer_signal_ptr));

   // save bit mask
   timer_node_ptr->channel_bit_mask = bit_mask;

   TRY(result,
       posal_timer_create(&timer_node_ptr->one_shot_timer,
                          POSAL_TIMER_ONESHOT_DURATION,
                          POSAL_TIMER_USER,
                          timer_node_ptr->timer_signal_ptr,
                          base_ptr->heap_id));

   TRY(result, posal_timer_oneshot_start_duration(timer_node_ptr->one_shot_timer, duration_us));

   TRY(result, spf_list_insert_tail(&base_ptr->timer_list_ptr, timer_node_ptr, base_ptr->heap_id, TRUE /* use_pool*/));

   CATCH(result, CU_MSG_PREFIX, base_ptr->gu_ptr->log_id)
   {
      cu_deinit_signal(base_ptr, &timer_node_ptr->timer_signal_ptr);
   }

   CU_MSG(base_ptr->gu_ptr->log_id, DBG_MED_PRIO, "Created soft timer id: %lu, bit_mask: 0x%lx", timer_id, bit_mask);

   return result;
}

/* Disable timer and remove corresponding node from the list*/
ar_result_t cu_fwk_extn_soft_timer_disable(cu_base_t *base_ptr, gu_module_t *module_ptr, uint32_t timer_id)
{
   ar_result_t result = AR_EOK;

   bool_t           found_node         = FALSE;
   spf_list_node_t *cur_timer_node_ptr = (spf_list_node_t *)base_ptr->timer_list_ptr;

   while (NULL != cur_timer_node_ptr)
   {
      spf_list_node_t *next_timer_node_ptr = cur_timer_node_ptr->next_ptr;
      cu_timer_node_t *timer_node_ptr      = (cu_timer_node_t *)cur_timer_node_ptr->obj_ptr;

      // Check if module instance id and timer id match
      if ((module_ptr->module_instance_id == timer_node_ptr->module_iid) && (timer_id == timer_node_ptr->timer_id))
      {
         result     = cu_fwk_extn_soft_timer_delete(base_ptr, &cur_timer_node_ptr);
         found_node = TRUE;
         break;
      }
      cur_timer_node_ptr = next_timer_node_ptr;
   }
   if (!found_node)
   {
      CU_MSG(base_ptr->gu_ptr->log_id, DBG_ERROR_PRIO, "Cannot find node in list");
      result = AR_EFAILED;
   }
   return result;
}

/* Callback when timer expires */
ar_result_t cu_fwk_extn_soft_timer_expired(cu_base_t *base_ptr, uint32_t ch_bit_index)
{
   ar_result_t result     = AR_EOK;
   bool_t      found_node = FALSE;

   spf_list_node_t *cur_timer_node_ptr = (spf_list_node_t *)base_ptr->timer_list_ptr;

   while (NULL != cur_timer_node_ptr)
   {
      spf_list_node_t *next_timer_node_ptr = cur_timer_node_ptr->next_ptr;

      cu_timer_node_t *timer_node_ptr = (cu_timer_node_t *)cur_timer_node_ptr->obj_ptr;
      uint32_t         bit_index      = cu_get_bit_index_from_mask(timer_node_ptr->channel_bit_mask);

      CU_MSG(base_ptr->gu_ptr->log_id,
             DBG_HIGH_PRIO,
             "Soft timer expired. bit_index :0x%lx, timer id: %ld ",
             ch_bit_index,
             timer_node_ptr->timer_id);

      // check if bit index matches
      if (ch_bit_index == bit_index)
      {
         // set param payload
         struct
         {
            apm_module_param_data_t                module_data;
            fwk_extn_param_id_soft_timer_expired_t payload;
         } set_param_payload;

         set_param_payload.payload.timer_id               = timer_node_ptr->timer_id;
         set_param_payload.module_data.module_instance_id = timer_node_ptr->module_iid;
         set_param_payload.module_data.param_id           = FWK_EXTN_PARAM_ID_SOFT_TIMER_EXPIRED;
         set_param_payload.module_data.param_size =
            (sizeof(fwk_extn_param_id_soft_timer_expired_t) + sizeof(apm_module_param_data_t));
         set_param_payload.module_data.error_code = 0;

         result = base_ptr->topo_vtbl_ptr->set_param(base_ptr->topo_ptr, &set_param_payload.module_data);
         if (AR_EOK != result)
         {
            CU_MSG(base_ptr->gu_ptr->log_id, DBG_ERROR_PRIO, "Timer expiry handling failed");
            return result;
         }

         // delete the timer
         result = cu_fwk_extn_soft_timer_delete(base_ptr, &cur_timer_node_ptr);
         if (AR_EOK != result)
         {
            return result;
         }

         found_node = TRUE;
         break;
      }
      cur_timer_node_ptr = next_timer_node_ptr;
   }
   if (!found_node)
   {
      CU_MSG(base_ptr->gu_ptr->log_id, DBG_ERROR_PRIO, "Cannot find node in list");
      result = AR_EFAILED;
   }
   return result;
}

void cu_fwk_extn_soft_timer_destroy_at_close(cu_base_t *base_ptr, gu_module_t *module_ptr)
{

   spf_list_node_t *cur_timer_node_ptr = (spf_list_node_t *)base_ptr->timer_list_ptr;

   while (NULL != cur_timer_node_ptr)
   {
      spf_list_node_t *next_timer_node_ptr = cur_timer_node_ptr->next_ptr;

      cu_timer_node_t *timer_node_ptr = (cu_timer_node_t *)cur_timer_node_ptr->obj_ptr;
      if (module_ptr->module_instance_id == timer_node_ptr->module_iid)
      {
         // delete the timer
         cu_fwk_extn_soft_timer_delete(base_ptr, &cur_timer_node_ptr);
      }

      cur_timer_node_ptr = next_timer_node_ptr;
   }
}
