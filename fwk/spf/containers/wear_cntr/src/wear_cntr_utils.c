/**
 * \file wear_cntr_utils.c
 * \brief
 *     This file contaouts utility functions for WCNTR
 *
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "wear_cntr_i.h"
#include "wear_cntr_events.h"
#include "capi_intf_extn_imcl.h"

// TODO: to profile these and tune
static const uint32_t WCNTR_PROCESS_STACK_SIZE     = 1024;
static const uint32_t WCNTR_BASE_STACK_SIZE        = 1024;


/**
 * me_ptr - WCNTR instance pointer
 * stack_size - input: stack size decided by CAPIs
 *              output: stack size decided based on comparing client given size, capi given size etc
 */
ar_result_t wcntr_get_thread_stack_size(wcntr_t *me_ptr, uint32_t *stack_size)
{
   *stack_size = MAX(me_ptr->cu.configured_stack_size, *stack_size);

   {
      *stack_size = MAX(WCNTR_BASE_STACK_SIZE, *stack_size);
      *stack_size += WCNTR_PROCESS_STACK_SIZE;
   }

   // Check this after adding the WCNTR_PROCESS_STACK_SIZE to the stack_size
   // to prevent multiple addition during relaunch
   *stack_size = MAX(me_ptr->cu.actual_stack_size, *stack_size);
   WCNTR_MSG(me_ptr->topo.gu.log_id,
                DBG_HIGH_PRIO,
                SPF_LOG_PREFIX "Stack sizes: Configured %lu, actual %lu, final %lu",
                me_ptr->cu.configured_stack_size,
                me_ptr->cu.actual_stack_size,
                *stack_size);
   return AR_EOK;
}

ar_result_t wcntr_get_set_thread_priority(wcntr_t *me_ptr, int32_t *priority_ptr, bool_t should_set)
{
   ar_result_t         result    = AR_EOK;
   bool_t              has_stm   = TRUE; //wcntr is always Signal triggered from endpoint module
   posal_thread_prio_t curr_prio = posal_thread_prio_get();
   posal_thread_prio_t new_prio  = curr_prio;

   if (!me_ptr->cu.flags.is_cntr_started)
   {
      new_prio = posal_thread_get_floor_prio(SPF_THREAD_STAT_CNTR_ID);
   }
   else
   {

      if ((WCNTR_FRAME_SIZE_DONT_CARE == me_ptr->cu.cntr_proc_duration) || (0 == me_ptr->cu.cntr_proc_duration))
      {
         new_prio = posal_thread_prio_get();
      }
      else
      {

         prio_query_t query_tbl;
         query_tbl.frame_duration_us = me_ptr->cu.cntr_proc_duration;
         query_tbl.static_req_id     = SPF_THREAD_DYN_ID;
         query_tbl.is_interrupt_trig = has_stm;
         result                      = posal_thread_calc_prio(&query_tbl, &new_prio);
         if (AR_DID_FAIL(result))
         {
            WCNTR_MSG(me_ptr->topo.gu.log_id, DBG_HIGH_PRIO, "Failed to get WCNTR thread priority");
            return result;
         }
      }
      new_prio = MAX(new_prio, me_ptr->cu.configured_thread_prio);
   }

   if (curr_prio != new_prio)
   {
      WCNTR_MSG(me_ptr->topo.gu.log_id,
                   DBG_HIGH_PRIO,
                   SPF_LOG_PREFIX "wcntr thread priority new_prio %d curr_prio %d ,cntr_proc_duration %lu   ",
                   new_prio,curr_prio,
                   me_ptr->cu.cntr_proc_duration);

      if (should_set)
      {
         posal_thread_prio_t prio = (posal_thread_prio_t)new_prio;
         posal_thread_set_prio(prio);
      }
   }

   SET_IF_NOT_NULL(priority_ptr, new_prio);

   return AR_EOK;
}

/**
 * Assign if instance variable has zero (not configured even once);
 * otherwise, verify that incoming variable is same as instance var.
 */
#define ASSIGN_IF_ZERO_ELSE_VERIFY(result, instance_var, incoming_var)                                                 \
   do                                                                                                                  \
   {                                                                                                                   \
      if (0 == instance_var)                                                                                           \
      {                                                                                                                \
         instance_var = incoming_var;                                                                                  \
      }                                                                                                                \
      else                                                                                                             \
      {                                                                                                                \
         VERIFY(result, instance_var == incoming_var);                                                                 \
      }                                                                                                                \
   } while (0)

ar_result_t wcntr_parse_container_cfg(wcntr_t *me_ptr, apm_container_cfg_t *container_cfg_ptr)
{
   ar_result_t result = AR_EOK;
   INIT_EXCEPTION_HANDLING
   apm_prop_data_t *cntr_prop_ptr;
   uint32_t         host_domain;
   __gpr_cmd_get_host_domain_id(&host_domain);

   if (!container_cfg_ptr)
   {
      WCNTR_MSG(me_ptr->topo.gu.log_id, DBG_LOW_PRIO, "Container cfg not given (Can be ignored)");
      return AR_EOK;
   }

   ASSIGN_IF_ZERO_ELSE_VERIFY(result, me_ptr->cu.cntr_instance_id, container_cfg_ptr->container_id);

   cntr_prop_ptr = (apm_prop_data_t *)(container_cfg_ptr + 1);

   for (uint32_t i = 0; i < container_cfg_ptr->num_prop; i++)
   {
      switch (cntr_prop_ptr->prop_id)
      {
         case APM_CONTAINER_PROP_ID_CONTAINER_TYPE:
         {
            VERIFY(result, cntr_prop_ptr->prop_size >= sizeof(apm_cont_prop_id_type_t));

            apm_cont_prop_id_type_t *type_ptr = (apm_cont_prop_id_type_t *)(cntr_prop_ptr + 1);
            VERIFY(result, (1 == type_ptr->version));

            VERIFY(result, (APM_CONTAINER_TYPE_ID_WC == type_ptr->type_id.type));

            ASSIGN_IF_ZERO_ELSE_VERIFY(result, me_ptr->cu.cntr_type, type_ptr->type_id.type);
            break;
         }
         case APM_CONTAINER_PROP_ID_GRAPH_POS:
         {
            VERIFY(result, cntr_prop_ptr->prop_size >= sizeof(apm_cont_prop_id_graph_pos_t));

            apm_cont_prop_id_graph_pos_t *pos_ptr = (apm_cont_prop_id_graph_pos_t *)(cntr_prop_ptr + 1);

            ASSIGN_IF_ZERO_ELSE_VERIFY(result, me_ptr->cu.position, pos_ptr->graph_pos);

            break;
         }
         case APM_CONTAINER_PROP_ID_STACK_SIZE:
         {
            VERIFY(result, cntr_prop_ptr->prop_size >= sizeof(apm_cont_prop_id_stack_size_t));

            apm_cont_prop_id_stack_size_t *stack_size_ptr = (apm_cont_prop_id_stack_size_t *)(cntr_prop_ptr + 1);

            if (APM_PROP_ID_DONT_CARE == stack_size_ptr->stack_size)
            {
               // max stack size will be used by container in the aggregation logic
               // so we can't directly copy
               me_ptr->cu.configured_stack_size = 0;
            }
            else
            {
               me_ptr->cu.configured_stack_size = stack_size_ptr->stack_size;
            }
            break;
         }
         case APM_CONTAINER_PROP_ID_PROC_DOMAIN:
         {
            VERIFY(result, cntr_prop_ptr->prop_size >= sizeof(apm_cont_prop_id_proc_domain_t));

            apm_cont_prop_id_proc_domain_t *proc_domain_ptr = (apm_cont_prop_id_proc_domain_t *)(cntr_prop_ptr + 1);

            VERIFY(result,
                   ((host_domain == proc_domain_ptr->proc_domain) ||
                    (APM_PROP_ID_DONT_CARE == proc_domain_ptr->proc_domain)));

            // can always assign the host domain from GPR to cu.
            me_ptr->cu.proc_domain = host_domain;

            break;
         }
         case APM_CONTAINER_PROP_ID_HEAP_ID:
         {
            VERIFY(result, cntr_prop_ptr->prop_size >= sizeof(apm_cont_prop_id_heap_id_t));

            apm_cont_prop_id_heap_id_t *heap_cfg_ptr = (apm_cont_prop_id_heap_id_t *)(cntr_prop_ptr + 1);

            me_ptr->cu.pm_info.register_info.mode =
               (APM_CONT_HEAP_LOW_POWER == heap_cfg_ptr->heap_id) ? PM_MODE_ISLAND : PM_MODE_DEFAULT;

            WCNTR_MSG(me_ptr->topo.gu.log_id,
                         DBG_MED_PRIO,
                         "is an island container (1 - True , 0 - False) %lu. ",
                         (APM_CONT_HEAP_LOW_POWER == heap_cfg_ptr->heap_id));

            break;
         }
         case APM_CONTAINER_PROP_ID_PARENT_CONTAINER_ID:
         {
            WCNTR_MSG(me_ptr->topo.gu.log_id,
                         DBG_HIGH_PRIO,
                         "Not parsing container property, 0x%X, ignoring",
                         cntr_prop_ptr->prop_id);

            break;
         }
         default:
         {
            WCNTR_MSG(me_ptr->topo.gu.log_id,
                         DBG_HIGH_PRIO,
                         "WARNING: Unsupported Container property, 0x%X, ignoring",
                         cntr_prop_ptr->prop_id);
         }
      }

      cntr_prop_ptr =
         (apm_prop_data_t *)((uint8_t *)cntr_prop_ptr + cntr_prop_ptr->prop_size + sizeof(apm_prop_data_t));
   }

   CATCH(result, WCNTR_MSG_PREFIX, me_ptr->topo.gu.log_id)
   {
   }

   return result;
}

bool_t wcntr_is_signal_triggered(wcntr_t *me_ptr)
{
   if (me_ptr->topo.flags.is_signal_triggered)
   {
      return TRUE;
   }
   return FALSE;
}


static ar_result_t wcntr_stm_fwk_extn_handle_disable(wcntr_t *me_ptr, wcntr_topo_module_t *module_ptr)
{
   ar_result_t result = AR_EOK;

   if (NULL == me_ptr->trigger_signal_ptr)
   {
      WCNTR_MSG(me_ptr->topo.gu.log_id, DBG_HIGH_PRIO, "STM module is already disabled.");
      return AR_EOK;
   }

   WCNTR_MSG(me_ptr->topo.gu.log_id, DBG_HIGH_PRIO, "Disabling CAPI signal triggered module");

   /* Disable the end point*/
   typedef struct
   {
      capi_custom_property_t cust_prop;
      capi_prop_stm_ctrl_t   ctrl;
   } stm_ctrl_t;

   stm_ctrl_t stm_ctrl;
   stm_ctrl.cust_prop.secondary_prop_id = FWK_EXTN_PROPERTY_ID_STM_CTRL;
   stm_ctrl.ctrl.enable                 = FALSE;

   capi_prop_t set_props[] = {
      { CAPI_CUSTOM_PROPERTY, { (int8_t *)(&stm_ctrl), sizeof(stm_ctrl), sizeof(stm_ctrl) }, { FALSE, FALSE, 0 } }
   };

   capi_proplist_t set_proplist = { SIZE_OF_ARRAY(set_props), set_props };

   if (CAPI_EOK != (result = module_ptr->capi_ptr->vtbl_ptr->set_properties(module_ptr->capi_ptr, &set_proplist)))
   {
      if (CAPI_EUNSUPPORTED == result)
      {
         WCNTR_MSG(me_ptr->topo.gu.log_id, DBG_HIGH_PRIO, "Unsupported stm cfg for input port 0x%x", result);
      }
      else
      {
         WCNTR_MSG(me_ptr->topo.gu.log_id,
                      DBG_ERROR_PRIO,
                      "Failed to apply trigger and stm cfg for input port 0x%x",
                      result);
         result = AR_EFAILED;
      }
   }

   if (me_ptr->trigger_signal_ptr)
   {
      // Get channel bit for the signal
      uint32_t bitmask = posal_signal_get_channel_bit(me_ptr->trigger_signal_ptr);

      // Stop listening to the mask
      wcntr_stop_listen_to_mask(&me_ptr->cu, bitmask);

      // Destroy the trigger signal
      posal_signal_clear(me_ptr->trigger_signal_ptr);
      posal_signal_destroy(&me_ptr->trigger_signal_ptr);
      me_ptr->trigger_signal_ptr = NULL;
   }

   return result;
}

static ar_result_t wcntr_stm_fwk_extn_handle_enable(wcntr_t *me_ptr, wcntr_topo_module_t *module_ptr)
{
   ar_result_t result   = AR_EOK;
   uint32_t    bit_mask = 0;
   INIT_EXCEPTION_HANDLING

   if (me_ptr->trigger_signal_ptr)
   {
      WCNTR_MSG(me_ptr->topo.gu.log_id, DBG_HIGH_PRIO, "STM module is already enabled.");
      return AR_EOK;
   }

   // Enable the stm control
   WCNTR_MSG(me_ptr->topo.gu.log_id, DBG_HIGH_PRIO, "Enabling CAPI signal triggered module");

   {
      /* Get the bit mask for stm signal, this signal trigger data processing
            on the container on every DMA interrupt or every time the timer expires */
      bit_mask = WCNTR_TIMER_BIT_MASK;

      /*Initialize dma signal */
      me_ptr->trigger_signal_ptr = NULL;
      TRY(result, posal_signal_create(&me_ptr->trigger_signal_ptr, me_ptr->cu.heap_id));

      /* Add trigger signal to the base channel with the above mask */
      TRY(result, posal_channel_add_signal(me_ptr->cu.channel_ptr, me_ptr->trigger_signal_ptr, bit_mask));

      wcntr_set_handler_for_bit_mask(&me_ptr->cu, bit_mask, wcntr_signal_trigger);

      /* Start listening to the mask : always*/
      wcntr_start_listen_to_mask(&me_ptr->cu, bit_mask);

      /* Property structure for stm trigger */
      typedef struct
      {
         capi_custom_property_t  cust_prop;
         capi_prop_stm_trigger_t trigger;
      } stm_trigger_t;

      stm_trigger_t stm_trigger;

      /* Populate the stm trigger */
      stm_trigger.cust_prop.secondary_prop_id = FWK_EXTN_PROPERTY_ID_STM_TRIGGER;
      stm_trigger.trigger.signal_ptr          = (void *)me_ptr->trigger_signal_ptr;

      /* Enable the stm*/
      typedef struct
      {
         capi_custom_property_t cust_prop;
         capi_prop_stm_ctrl_t   ctrl;
      } stm_ctrl_t;

      stm_ctrl_t stm_ctrl;
      stm_ctrl.cust_prop.secondary_prop_id = FWK_EXTN_PROPERTY_ID_STM_CTRL;
      stm_ctrl.ctrl.enable                 = TRUE;

      capi_prop_t set_props[] = { { CAPI_CUSTOM_PROPERTY,
                                    { (int8_t *)(&stm_trigger), sizeof(stm_trigger), sizeof(stm_trigger) },
                                    { FALSE, FALSE, 0 } },
                                  { CAPI_CUSTOM_PROPERTY,
                                    { (int8_t *)(&stm_ctrl), sizeof(stm_ctrl), sizeof(stm_ctrl) },
                                    { FALSE, FALSE, 0 } } };

      capi_proplist_t set_proplist = { SIZE_OF_ARRAY(set_props), set_props };

      if (CAPI_EOK != (result = module_ptr->capi_ptr->vtbl_ptr->set_properties(module_ptr->capi_ptr, &set_proplist)))
      {
         if (CAPI_EUNSUPPORTED == result)
         {
            WCNTR_MSG(me_ptr->topo.gu.log_id, DBG_HIGH_PRIO, "Unsupported stm cfg for input port 0x%x", result);
         }
         else
         {
            WCNTR_MSG(me_ptr->topo.gu.log_id,
                         DBG_ERROR_PRIO,
                         "Failed to apply trigger and stm cfg for input port 0x%x",
                         result);
            if (me_ptr->trigger_signal_ptr)
            {
               WCNTR_MSG(me_ptr->topo.gu.log_id,
                            DBG_ERROR_PRIO,
                            "Failed to set the trigger signal on the stm 0x%lx. Destroying trigger signal",
                            result);

               // Stop listening to the mask
               wcntr_stop_listen_to_mask(&me_ptr->cu, bit_mask);
               posal_signal_destroy(&me_ptr->trigger_signal_ptr);
               me_ptr->trigger_signal_ptr = NULL;
            }
            THROW(result, AR_EFAILED);
         }
      }
      else
      {
         capi_param_id_stm_latest_trigger_ts_ptr_t cfg = { 0 };

         uint32_t param_payload_size = (uint32_t)sizeof(capi_param_id_stm_latest_trigger_ts_ptr_t);
         result                      = wcntr_topo_capi_get_param(me_ptr->topo.gu.log_id,
                                          module_ptr->capi_ptr,
                                          FWK_EXTN_PARAM_ID_LATEST_TRIGGER_TIMESTAMP_PTR,
                                          (int8_t *)&cfg,
                                          &param_payload_size);
         if (AR_DID_FAIL(result))
         {
            WCNTR_MSG(me_ptr->topo.gu.log_id,
                         DBG_ERROR_PRIO,
                         "Failed to get timestamp pointer 0x%x. Ignoring.",
                         result);
         }
      }
   }

   CATCH(result, WCNTR_MSG_PREFIX, me_ptr->topo.gu.log_id)
   {
   }

   return result;
}

ar_result_t wcntr_fwk_extn_handle_at_start(wcntr_t *me_ptr, wcntr_gu_module_list_t *module_list_ptr,uint32_t sg_id)
{
   ar_result_t result = AR_EOK;

   bool_t stm_extn_found = FALSE;
   for (; (NULL != module_list_ptr); LIST_ADVANCE(module_list_ptr))
   {
      wcntr_topo_module_t *module_ptr = (wcntr_topo_module_t *)module_list_ptr->module_ptr;

      if (module_ptr->flags.need_stm_extn)
      {

         if (module_ptr->gu.sg_ptr->id != sg_id)
         {
            continue;
         }

         if (!stm_extn_found)
         {
            result |= wcntr_stm_fwk_extn_handle_enable(me_ptr, module_ptr);
            stm_extn_found = TRUE;
         }
         else
         {
            WCNTR_MSG(me_ptr->topo.gu.log_id, DBG_ERROR_PRIO, "more than 1 STM modules found.");
         }
      }
   }

   return result;
}

ar_result_t wcntr_fwk_extn_handle_at_stop(wcntr_t *me_ptr, wcntr_gu_module_list_t *module_list_ptr,uint32_t sg_id)
{
   ar_result_t result         = AR_EOK;
   bool_t      stm_extn_found = FALSE;

   for (; (NULL != module_list_ptr); LIST_ADVANCE(module_list_ptr))
   {
      wcntr_topo_module_t *module_ptr = (wcntr_topo_module_t *)module_list_ptr->module_ptr;
      if (module_ptr->flags.need_stm_extn)
      {

         if (module_ptr->gu.sg_ptr->id != sg_id)
         {
            continue;
         }

         if (!stm_extn_found)
         {
            result |= wcntr_stm_fwk_extn_handle_disable(me_ptr, module_ptr);
            stm_extn_found = TRUE;
         }
         else
         {
            WCNTR_MSG(me_ptr->topo.gu.log_id, DBG_ERROR_PRIO, "more than 1 STM modules found.");
         }
      }
   }

   return result;
}


ar_result_t wcntr_handle_event_to_dsp_service_topo_cb(wcntr_topo_module_t *module_ptr,
                                                         capi_event_info_t *event_info_ptr)
{
   wcntr_t *                      me_ptr        = (wcntr_t *)WCNTR_GET_BASE_PTR(wcntr_t, topo, module_ptr->topo_ptr);
   ar_result_t                       result        = AR_EOK;
   capi_buf_t *                      payload       = &event_info_ptr->payload;
   capi_event_data_to_dsp_service_t *dsp_event_ptr = (capi_event_data_to_dsp_service_t *)(payload->data_ptr);

   switch (dsp_event_ptr->param_id)
   {
      case INTF_EXTN_EVENT_ID_IMCL_RECURRING_BUF_INFO:
      case INTF_EXTN_EVENT_ID_IMCL_OUTGOING_DATA:
      {
         result = wcntr_handle_imcl_ext_event(&me_ptr->cu, &module_ptr->gu, event_info_ptr);
         break;
      }

      default:
      {

         WCNTR_MSG(me_ptr->topo.gu.log_id,
                      DBG_HIGH_PRIO,
                      "param_id ID %u is not supported",
                      dsp_event_ptr->param_id);
         result = AR_EUNSUPPORTED;
      }
   }

   return result;
}

ar_result_t wcntr_handle_event_from_dsp_service_topo_cb(wcntr_topo_module_t *module_ptr,
                                                           capi_event_info_t *event_info_ptr)
{
   wcntr_t *                            me_ptr  = (wcntr_t *)WCNTR_GET_BASE_PTR(wcntr_t, topo, module_ptr->topo_ptr);
   ar_result_t                             result  = AR_EOK;
   capi_buf_t *                            payload = &event_info_ptr->payload;
   capi_event_get_data_from_dsp_service_t *dsp_event_ptr =
      (capi_event_get_data_from_dsp_service_t *)(payload->data_ptr);

   WCNTR_MSG(me_ptr->topo.gu.log_id,
                DBG_HIGH_PRIO,
                "wcntr_handle_event_from_dsp_service_topo_cb param_id ID 0x%X ",
                dsp_event_ptr->param_id);

   switch (dsp_event_ptr->param_id)
   {
      case INTF_EXTN_EVENT_ID_IMCL_GET_RECURRING_BUF:	  	 
      case INTF_EXTN_EVENT_ID_IMCL_GET_ONE_TIME_BUF:
      {
         result = wcntr_handle_imcl_ext_event(&me_ptr->cu, &module_ptr->gu, event_info_ptr);
         break;
      }

      default:
      {

         WCNTR_MSG(me_ptr->topo.gu.log_id,
                      DBG_HIGH_PRIO,
                      "param_id ID %u is not supported",
                      dsp_event_ptr->param_id);
         result = AR_EUNSUPPORTED;
      }
   }
   return result;
}

ar_result_t wcntr_handle_capi_event(wcntr_topo_module_t *topo_module_ptr,
                                       capi_event_id_t    event_id,
                                       capi_event_info_t *event_info_ptr)
{
   wcntr_t *me_ptr = (wcntr_t *)WCNTR_GET_BASE_PTR(wcntr_t, topo, topo_module_ptr->topo_ptr);
   ar_result_t result = AR_EOK;

   wcntr_base_t *  cu_ptr     = &me_ptr->cu;
   wcntr_gu_module_t *module_ptr = &topo_module_ptr->gu;

   wcu_module_t *cu_module_ptr = (wcu_module_t *)((uint8_t *)module_ptr + cu_ptr->module_cu_offset);
   switch (event_id)
   {
      case CAPI_EVENT_DATA_TO_DSP_CLIENT:
      {
         if (event_info_ptr->payload.actual_data_len < sizeof(capi_event_data_to_dsp_client_t))
         {
            result |= AR_ENEEDMORE;
            break;
         }

         capi_event_data_to_dsp_client_t *payload_ptr =
            (capi_event_data_to_dsp_client_t *)(event_info_ptr->payload.data_ptr);

         spf_list_node_t *client_list_ptr;
         if (AR_EOK != (result = wcntr_find_client_info(cu_ptr->gu_ptr->log_id,
                                                           payload_ptr->param_id,
                                                           cu_module_ptr->event_list_ptr,
                                                           &client_list_ptr)))
         {
            WCNTR_MSG(cu_ptr->gu_ptr->log_id,
                   DBG_ERROR_PRIO,
                   "Failed to get client list for event id: 0x%lx, result: %d",
                   payload_ptr->param_id,
                   result);
            result |= AR_EFAILED;
            break;
         }

         gpr_packet_t *      event_packet_ptr = NULL;
         apm_module_event_t *event_payload;

         for (wcntr_client_info_t *client_info_ptr = (wcntr_client_info_t *)(client_list_ptr->obj_ptr);
              (NULL != client_list_ptr);
              LIST_ADVANCE(client_list_ptr))
         {
            event_packet_ptr = NULL;
            gpr_cmd_alloc_ext_t args;
            args.src_domain_id = client_info_ptr->dest_domain_id;
            args.dst_domain_id = client_info_ptr->src_domain_id;
            args.src_port      = module_ptr->module_instance_id;
            args.dst_port      = client_info_ptr->src_port;
            args.token         = client_info_ptr->token;
            args.opcode        = APM_EVENT_MODULE_TO_CLIENT;
            args.payload_size  = sizeof(apm_module_event_t) + payload_ptr->payload.actual_data_len;
            args.client_data   = 0;
            args.ret_packet    = &event_packet_ptr;
            result             = __gpr_cmd_alloc_ext(&args);
            if (NULL == event_packet_ptr)
            {
               WCNTR_MSG(cu_ptr->gu_ptr->log_id,
                      DBG_ERROR_PRIO,
                      "Module 0x%lX: event 0x%lX NOT sent to client with error code %lu",
                      module_ptr->module_instance_id,
                      payload_ptr->param_id,
                      result);
               result |= AR_EFAILED;
               break;
            }

            event_payload = GPR_PKT_GET_PAYLOAD(apm_module_event_t, event_packet_ptr);

            event_payload->event_id           = payload_ptr->param_id;
            event_payload->event_payload_size = payload_ptr->payload.actual_data_len;

            memscpy(event_payload + 1,
                    event_payload->event_payload_size,
                    payload_ptr->payload.data_ptr,
                    payload_ptr->payload.actual_data_len);

            result = __gpr_cmd_async_send(event_packet_ptr);

            if (AR_EOK != result)
            {
               WCNTR_MSG(cu_ptr->gu_ptr->log_id,
                         DBG_ERROR_PRIO,
                         "Module 0x%lX: Unable to send event 0x%lX to client with result %lu "
                         "destn port: 0x%lX, destn domain ID: 0x%lX, src domain ID: 0x%lX ",
                         module_ptr->module_instance_id,
                         payload_ptr->param_id,
                         result,
                         client_info_ptr->src_port,
                         client_info_ptr->src_domain_id,
                         client_info_ptr->dest_domain_id);
               result = __gpr_cmd_free(event_packet_ptr);
            }
            else
            {

               WCNTR_MSG(cu_ptr->gu_ptr->log_id,
                         DBG_HIGH_PRIO,
                         "Module 0x%lX: event 0x%lX sent to client with result %lu "
                         "destn port: 0x%lX, destn domain ID: 0x%lX, src domain ID: 0x%lX ",
                         module_ptr->module_instance_id,
                         payload_ptr->param_id,
                         result,
                         client_info_ptr->src_port,
                         client_info_ptr->src_domain_id,
                         client_info_ptr->dest_domain_id);
            }
         }

         break;
      }
      case CAPI_EVENT_DATA_TO_DSP_CLIENT_V2:
      {
         if (event_info_ptr->payload.actual_data_len < sizeof(capi_event_data_to_dsp_client_t))
         {
            result |= AR_ENEEDMORE;
            break;
         }

         capi_event_data_to_dsp_client_v2_t *payload_ptr =
            (capi_event_data_to_dsp_client_v2_t *)(event_info_ptr->payload.data_ptr);
         gpr_packet_t *      event_packet_ptr = NULL;
         apm_module_event_t *event_payload;

         wcntr_topo_evt_dest_addr_t dest_address;
         dest_address.address = payload_ptr->dest_address;
         /* Allocate the event packet
          * 64 bit destination address is populated as follows:
          * bits 0-31 : src port
          * bits 32-39: src domain id
          * bits 40-47: dest domain id
          * bits 48-63: 0 */

         gpr_cmd_alloc_ext_t args;
         args.src_domain_id = dest_address.a.dest_domain_id;
         args.dst_domain_id = dest_address.a.src_domain_id;
         args.src_port      = module_ptr->module_instance_id;
         args.dst_port      = dest_address.a.src_port;
         args.token         = payload_ptr->token;
         args.opcode        = APM_EVENT_MODULE_TO_CLIENT;
         args.payload_size  = sizeof(apm_module_event_t) + payload_ptr->payload.actual_data_len;
         args.client_data   = 0;
         args.ret_packet    = &event_packet_ptr;
         result             = __gpr_cmd_alloc_ext(&args);
         if (NULL == event_packet_ptr)
         {
            WCNTR_MSG(cu_ptr->gu_ptr->log_id,
                   DBG_ERROR_PRIO,
                   "Module 0x%lX: event 0x%lX NOT sent to client with error code %lu",
                   module_ptr->module_instance_id,
                   payload_ptr->event_id,
                   result);
            result |= AR_EFAILED;
            break;
         }

         event_payload = GPR_PKT_GET_PAYLOAD(apm_module_event_t, event_packet_ptr);

         event_payload->event_id           = payload_ptr->event_id;
         event_payload->event_payload_size = payload_ptr->payload.actual_data_len;

         memscpy(event_payload + 1,
                 event_payload->event_payload_size,
                 payload_ptr->payload.data_ptr,
                 event_payload->event_payload_size);

         result = __gpr_cmd_async_send(event_packet_ptr);

         if (AR_EOK != result)
         {
            WCNTR_MSG(cu_ptr->gu_ptr->log_id,
                      DBG_ERROR_PRIO,
                      "Module 0x%lX: Unable to send event 0x%lX to client with result %lu "
                      "destn port: 0x%lX, destn domain ID: 0x%lX, src domain ID: 0x%lX ",
                      module_ptr->module_instance_id,
                      payload_ptr->event_id,
                      result,
                      dest_address.a.src_port,
                      dest_address.a.src_domain_id,
                      dest_address.a.dest_domain_id);
            result = __gpr_cmd_free(event_packet_ptr);
         }
         else
         {
            WCNTR_MSG(cu_ptr->gu_ptr->log_id,
                      DBG_HIGH_PRIO,
                      "Module 0x%lX: sent event 0x%lX to client with result %lu "
                      "destn port: 0x%lX, destn domain ID: 0x%lX, src domain ID: 0x%lX ",
                      module_ptr->module_instance_id,
                      payload_ptr->event_id,
                      result,
                      dest_address.a.src_port,
                      dest_address.a.src_domain_id,
                      dest_address.a.dest_domain_id);
         }
         break;
      }

      default:
      {
         WCNTR_MSG(cu_ptr->gu_ptr->log_id, DBG_ERROR_PRIO, "No support  for event id: 0x%lx", event_id);
         break;
      }
   }

   return result;
}


ar_result_t wcntr_destroy_module(wcntr_topo_t *       topo_ptr,
                                    wcntr_topo_module_t *module_ptr,
                                    bool_t             reset_capi_dependent_dont_destroy)
{

   wcntr_module_t *wcntr_module_ptr = (wcntr_module_t *)module_ptr;
   wcntr_delete_all_event_nodes(&wcntr_module_ptr->cu.event_list_ptr);

   return AR_EOK;
}

ar_result_t wcntr_handle_fwk_events(wcntr_t *me_ptr)
{
   if ((0 == me_ptr->cu.fwk_evt_flags.word) && (0 == me_ptr->topo.capi_event_flag.word))
   {
      return AR_EOK;
   }

   WCNTR_MSG(me_ptr->topo.gu.log_id,
                DBG_LOW_PRIO,
                "Handling fwk events: fwk events 0x%lX, capi events 0x%lX",
                me_ptr->cu.fwk_evt_flags.word,
                me_ptr->topo.capi_event_flag.word);


if(me_ptr->cu.fwk_evt_flags.word)
{
	WCNTR_MSG(me_ptr->topo.gu.log_id,
					DBG_LOW_PRIO,
					"fwk_evt_flags sg_state_change %u  cntr_run_state_change %u kpps_bw_scale_factor_change %u  ",
					me_ptr->cu.fwk_evt_flags.sg_state_change,
					me_ptr->cu.fwk_evt_flags.cntr_run_state_change,
					me_ptr->cu.fwk_evt_flags.kpps_bw_scale_factor_change);

}
if(me_ptr->topo.capi_event_flag.word)
{
	WCNTR_MSG(me_ptr->topo.gu.log_id,
					DBG_LOW_PRIO,
					"capi_event_flag kpps %u,bw %u,port_thresh %u,media_fmt_event %u,process_state %u ",
					me_ptr->topo.capi_event_flag.kpps,
					me_ptr->topo.capi_event_flag.bw,
					me_ptr->topo.capi_event_flag.port_thresh,
					me_ptr->topo.capi_event_flag.media_fmt_event,
					me_ptr->topo.capi_event_flag.process_state);

}


bool_t port_thresh=me_ptr->topo.capi_event_flag.port_thresh;
bool_t media_fmt_event=me_ptr->topo.capi_event_flag.media_fmt_event;
				
   if (port_thresh || media_fmt_event)
   {
      wcntr_topo_propagate_media_fmt(&me_ptr->topo, FALSE /*is_data_path*/);
      wcntr_update_cntr_kpps_bw(me_ptr);
	  
	  if(me_ptr->topo.mf_propagation_done)
	  {
      wcntr_handle_port_data_thresh_change_event(&me_ptr->cu);
	  }
   }

   wcntr_perf_vote(me_ptr);

   me_ptr->topo.capi_event_flag.word = 0;
   me_ptr->cu.fwk_evt_flags.word     = 0;

   //Add back the flags. This would trigger propagatign media format again
   if(!me_ptr->topo.mf_propagation_done)
   {
     me_ptr->topo.capi_event_flag.port_thresh= port_thresh;
	 me_ptr->topo.capi_event_flag.media_fmt_event= media_fmt_event;
   }

   return AR_EOK;
}

uint32_t wcntr_gpr_callback(gpr_packet_t *packet, void *callback_data)
{
   ar_result_t result = AR_EOK;
   INIT_EXCEPTION_HANDLING
   spf_handle_t *handle_ptr = NULL;

   AR_MSG(DBG_LOW_PRIO,
          SPF_LOG_PREFIX "GPR callback from src 0x%lX to dst 0x%lX, opcode 0x%lX, token 0x%lX",
          packet->src_port,
          packet->dst_port,
          packet->opcode,
          packet->token);

   spf_msg_t msg;
   uint32_t  thread_id = 0;
   msg.payload_ptr     = packet;
   msg.msg_opcode      = SPF_MSG_CMD_GPR;

   // No need of mutex here. If a container destroys a module, it will first
   // deregister from GPR/ Until this function finishes, GPR must block
   // deregister.
   handle_ptr = (spf_handle_t *)callback_data;

   /*Validate handles and queue pointers */
   VERIFY(result, (handle_ptr && handle_ptr->cmd_handle_ptr && handle_ptr->cmd_handle_ptr->cmd_q_ptr));

   thread_id = (uint32_t)posal_thread_get_tid(handle_ptr->cmd_handle_ptr->thread_id);

   switch (wcntr_get_bits(packet->opcode, AR_GUID_TYPE_MASK, AR_GUID_TYPE_SHIFT))
   {
      case AR_GUID_TYPE_CONTROL_CMD:
      {
         /** control commands */
         TRY(result,
             (ar_result_t)posal_queue_push_back(handle_ptr->cmd_handle_ptr->cmd_q_ptr, (posal_queue_element_t *)&msg));
         break;
      }
      case AR_GUID_TYPE_DATA_CMD:
      {
         VERIFY(result, (NULL != handle_ptr->q_ptr));
         /** Data commands */
         TRY(result, (ar_result_t)posal_queue_push_back(handle_ptr->q_ptr, (posal_queue_element_t *)&msg));
         break;
      }
      default:
      {
         AR_MSG(DBG_ERROR_PRIO, "CNTR TID: 0x%lx Improper GUID 0x%lX", thread_id, packet->opcode);

         THROW(result, AR_EFAILED);
      }
   }

   CATCH(result, "CNTR TID ID: 0x%lx ", thread_id)
   {
      __gpr_cmd_end_command(packet, result);
   }

   return result;
}
