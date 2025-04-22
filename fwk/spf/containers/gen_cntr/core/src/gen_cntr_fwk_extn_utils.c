/**
 * \file gen_cntr_fwk_extn_utils.c
 * \brief
 *     This file contaouts utility functions for GEN_CNTR capi fwk extensions.
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "gen_cntr_i.h"
#include "pt_cntr.h"

static ar_result_t gen_cntr_fwk_extn_async_st_get_cb_info(gen_cntr_t *             me_ptr,
                                                          gen_topo_module_t *      module_ptr,
                                                          gen_cntr_async_signal_t *async_st_hdl_ptr)
{
   capi_err_t err_code = CAPI_EOK;

   /* Property structure for async signal trigger */
   typedef struct
   {
      capi_custom_property_t                     cust_prop;
      capi_prop_async_signal_callback_info_t    async_signal;
   } callback_info_t;

   callback_info_t callback_info;
   memset(&callback_info, 0, sizeof(callback_info_t));

   /* Populate the async signal trigger */
   callback_info.cust_prop.secondary_prop_id = FWK_EXTN_PROPERTY_ID_ASYNC_SIGNAL_CALLBACK_INFO;

   capi_prop_t get_prop[] = { { CAPI_CUSTOM_PROPERTY,
                                { (int8_t *)(&callback_info), 0 /*actual_len*/, sizeof(callback_info) /*max_len*/ },
                                { FALSE, FALSE, 0 } } };

   capi_proplist_t get_proplist = { SIZE_OF_ARRAY(get_prop), get_prop };

   err_code = module_ptr->capi_ptr->vtbl_ptr->get_properties(module_ptr->capi_ptr, &get_proplist);
   if (CAPI_FAILED(err_code))
   {
      TOPO_MSG(me_ptr->topo.gu.log_id,
               DBG_ERROR_PRIO,
               "Module 0x%lX: Get async signal cb info returned error 0x%lx",
               module_ptr->gu.module_instance_id,
               capi_err_to_ar_result(err_code));
      if(CAPI_EUNSUPPORTED == err_code)
      {
         return AR_EOK;
      }
      return capi_err_to_ar_result(err_code);
   }

   if (sizeof(callback_info) > get_prop[0].payload.actual_data_len)
   {
      TOPO_MSG(me_ptr->topo.gu.log_id,
               DBG_ERROR_PRIO,
               "Module 0x%lX: unexpected payload actual_data_len %lu",
               module_ptr->gu.module_instance_id,
               get_prop[0].payload.actual_data_len);
      return AR_EFAILED;
   }

   // cache the callback info in the async signal's object
   async_st_hdl_ptr->cb_fn_ptr      = callback_info.async_signal.module_callback_fptr;
   async_st_hdl_ptr->cb_context_ptr = callback_info.async_signal.module_context_ptr;

   return AR_EOK;
}

static ar_result_t gen_cntr_fwk_extn_async_signal_ctrl(gen_cntr_t              *me_ptr,
                                                       gen_topo_module_t       *module_ptr,
                                                       gen_cntr_async_signal_t *async_st_hdl_ptr,
                                                       bool_t                   is_enable)
{
   capi_err_t err_code = CAPI_EOK;

   /* Property structure for async signal trigger */
   typedef struct
   {
      capi_custom_property_t        cust_prop;
      capi_prop_async_signal_ctrl_t async_signal;
   } signal_ctrl_t;

   signal_ctrl_t signal_ctrl;
   memset(&signal_ctrl, 0, sizeof(signal_ctrl));

   /* Populate the async signal trigger */
   signal_ctrl.cust_prop.secondary_prop_id   = FWK_EXTN_PROPERTY_ID_ASYNC_SIGNAL_CTRL;
   signal_ctrl.async_signal.enable           = is_enable ? TRUE : FALSE;
   signal_ctrl.async_signal.async_signal_ptr = is_enable ? async_st_hdl_ptr->signal_ptr : NULL;

   capi_prop_t set_prop[] = { { CAPI_CUSTOM_PROPERTY,
                                { (int8_t *)(&signal_ctrl), sizeof(signal_ctrl), sizeof(signal_ctrl) },
                                { FALSE, FALSE, 0 } } };

   capi_proplist_t set_proplist = { SIZE_OF_ARRAY(set_prop), set_prop };

   err_code = module_ptr->capi_ptr->vtbl_ptr->set_properties(module_ptr->capi_ptr, &set_proplist);
   if (CAPI_FAILED(err_code))
   {
      TOPO_MSG(me_ptr->topo.gu.log_id,
               DBG_ERROR_PRIO,
               "Module 0x%lX: Set property async signal ctrl returned error 0x%lx",
               module_ptr->gu.module_instance_id,
               capi_err_to_ar_result(err_code));
      if(CAPI_EUNSUPPORTED == err_code)
      {
         return AR_EOK;
      }

      return capi_err_to_ar_result(err_code);
   }

   return AR_EOK;
}

ar_result_t gen_cntr_fwk_extn_async_signal_enable(gen_cntr_t *me_ptr, gen_topo_module_t *module_ptr)
{
   INIT_EXCEPTION_HANDLING
   ar_result_t              result           = AR_EOK;
   cu_base_t *              base_ptr         = (cu_base_t *)&me_ptr->cu;
   uint32_t                 bit_mask         = 0;
   gen_cntr_async_signal_t *async_st_hdl_ptr = NULL;

   // create a handle for async signal
   MALLOC_MEMSET(async_st_hdl_ptr,
                 gen_cntr_async_signal_t,
                 sizeof(gen_cntr_async_signal_t),
                 me_ptr->cu.heap_id,
                 result);

   async_st_hdl_ptr->module_ptr = module_ptr;

   /*Initialize async signal */
   TRY(result, posal_signal_create(&async_st_hdl_ptr->signal_ptr, me_ptr->cu.heap_id));

   bit_mask = cu_request_bit_in_bit_mask(&base_ptr->available_bit_mask);

   /* Add trigger signal to the base channel with the above mask */
   TRY(result, posal_channel_add_signal(me_ptr->cu.channel_ptr, async_st_hdl_ptr->signal_ptr, bit_mask));
   cu_set_bit_in_bit_mask(base_ptr, bit_mask);
   cu_set_handler_for_bit_mask(base_ptr, bit_mask, gen_cntr_workloop_async_signal_trigger_handler);

   /** cache bit mask to identify the module when the signal is set.*/
   async_st_hdl_ptr->bit_index  = cu_get_bit_index_from_mask(bit_mask);

   /** Query async signal callback info from the module*/
   TRY(result, gen_cntr_fwk_extn_async_st_get_cb_info(me_ptr, module_ptr, async_st_hdl_ptr));

   /** set prop to enable async signal*/
   TRY(result, gen_cntr_fwk_extn_async_signal_ctrl(me_ptr, module_ptr, async_st_hdl_ptr, TRUE /* is_enable*/));

   /* Start listening to the mask : always*/
   cu_start_listen_to_mask(base_ptr, bit_mask);

   /** add async signal to the containers list*/
   TRY(result,
       spf_list_insert_tail(&me_ptr->async_signal_list_ptr, async_st_hdl_ptr, me_ptr->cu.heap_id, TRUE /*Use pool */));

   GEN_CNTR_MSG(me_ptr->topo.gu.log_id,
                DBG_LOW_PRIO,
                "Module 0x%lX: Enabled async signal trigger with bit_mask:0x%lx ",
                module_ptr->gu.module_instance_id,
                bit_mask);

   CATCH(result, GEN_CNTR_MSG_PREFIX, me_ptr->topo.gu.log_id)
   {
      if (bit_mask)
      {
         cu_stop_listen_to_mask(&me_ptr->cu, bit_mask);
         cu_release_bit_in_bit_mask(&me_ptr->cu, bit_mask);
      }

      if (async_st_hdl_ptr && async_st_hdl_ptr->signal_ptr)
      {
         posal_signal_destroy(&async_st_hdl_ptr->signal_ptr);
         async_st_hdl_ptr->signal_ptr = NULL;
      }
      MFREE_NULLIFY(async_st_hdl_ptr);

      /**if module returns unsupported, its not a ctricial failure, hence fwk just need to free the resources for the
         async signal and return EOK*/
      if (AR_EUNSUPPORTED == result)
      {
         return AR_EOK;
      }
   }

   return result;
}

ar_result_t gen_cntr_fwk_extn_async_signal_disable(gen_cntr_t *me_ptr, gen_topo_module_t *module_ptr)
{
   ar_result_t              result           = AR_EOK;
   cu_base_t *              base_ptr         = &me_ptr->cu;
   gen_cntr_async_signal_t *async_st_hdl_ptr = NULL;
   spf_list_node_t         *async_list_ptr   = NULL;
   uint32_t                 bit_mask         = 0;

   GEN_CNTR_MSG(me_ptr->topo.gu.log_id,
               DBG_LOW_PRIO,
               "Module 0x%lX: Disabling async signal trigger",
               module_ptr->gu.module_instance_id);

   /** get async signal handle corresponding to the module */
   for (spf_list_node_t *list_ptr = me_ptr->async_signal_list_ptr; NULL != list_ptr; LIST_ADVANCE(list_ptr))
   {
      gen_cntr_async_signal_t *temp_ptr = (gen_cntr_async_signal_t *)list_ptr->obj_ptr;
      if (module_ptr == temp_ptr->module_ptr)
      {
         async_list_ptr    = list_ptr;
         async_st_hdl_ptr  = temp_ptr;
         break;
      }
   }

   /** async signal may not be created by the fwk if module returned unsupported at the time of enable(), hence not
      found is not a critical error */
   if (NULL == async_st_hdl_ptr)
   {
      GEN_CNTR_MSG(me_ptr->topo.gu.log_id,
                   DBG_ERROR_PRIO,
                   "Module 0x%lX: Failed async signal disable, couldnt find async signal info for the module.",
                   module_ptr->gu.module_instance_id);
      return AR_EOK;
   }

   bit_mask = 1 << async_st_hdl_ptr->bit_index;
   cu_stop_listen_to_mask(base_ptr, bit_mask);

   /** set prop to disable async signal*/
   result = gen_cntr_fwk_extn_async_signal_ctrl(me_ptr, module_ptr, async_st_hdl_ptr, FALSE /* is_enable*/);

   if (async_st_hdl_ptr && async_st_hdl_ptr->signal_ptr)
   {
      posal_signal_clear(async_st_hdl_ptr->signal_ptr);
      posal_signal_destroy(&async_st_hdl_ptr->signal_ptr);
      async_st_hdl_ptr->signal_ptr = NULL;
   }

   cu_release_bit_in_bit_mask(base_ptr, bit_mask);
   MFREE_NULLIFY(async_st_hdl_ptr);

   async_list_ptr->obj_ptr= NULL;
   spf_list_delete_node_and_free_obj((spf_list_node_t **)&async_list_ptr,
                                    (spf_list_node_t **)&me_ptr->async_signal_list_ptr,
                                    TRUE /* pool_used */);

   GEN_CNTR_MSG(me_ptr->topo.gu.log_id,
                DBG_LOW_PRIO,
                "Module 0x%lX: Disabled async signal trigger with bit_mask:0x%lx ",
                module_ptr->gu.module_instance_id,
                bit_mask);

   return result;
}
