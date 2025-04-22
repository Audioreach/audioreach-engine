/**
 * \file cu_utils.c
 * \brief
 *
 *     CU's utility
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/* =======================================================================
INCLUDE FILES FOR MODULE
========================================================================== */
#include "container_utils.h"
#include "capi_intf_extn_imcl.h"
#include "capi_intf_extn_path_delay.h"

/*==============================================================================
   Global Defines
==============================================================================*/

ar_result_t cu_raise_frame_done_event(cu_base_t *base_ptr, uint32_t log_id)
{
   return cu_raise_container_events_to_clients(base_ptr, CNTR_EVENT_ID_CONTAINER_FRAME_DELIVERY_DONE, (int8_t *)NULL, 0);
}

ar_result_t cu_handle_event_to_dsp_service_topo_cb(cu_base_t *        cu_ptr,
                                                   gu_module_t *      module_ptr,
                                                   capi_event_info_t *event_info_ptr)
{
   ar_result_t                       result        = AR_EOK;
   capi_buf_t *                      payload       = &event_info_ptr->payload;
   capi_event_data_to_dsp_service_t *dsp_event_ptr = (capi_event_data_to_dsp_service_t *)(payload->data_ptr);

   switch (dsp_event_ptr->param_id)
   {
      case INTF_EXTN_EVENT_ID_IMCL_RECURRING_BUF_INFO:
      case INTF_EXTN_EVENT_ID_IMCL_OUTGOING_DATA:
      {
         result = cu_handle_imcl_event(cu_ptr, module_ptr, event_info_ptr);
         break;
      }
      case INTF_EXTN_EVENT_ID_ALLOW_DUTY_CYCLING:
      {
         //TODO aggregate for all modules
         if(dsp_event_ptr->payload.actual_data_len < sizeof(intf_extn_event_id_allow_duty_cycling_t) || (!dsp_event_ptr->payload.data_ptr))
         {
            CU_MSG(cu_ptr->gu_ptr->log_id,
                    DBG_ERROR_PRIO,
                    "Module 0x%lX: INTF_EXTN_EVENT_ID_ALLOW_DUTY_CYCLING. : bad param length ",
                    module_ptr->module_instance_id);
            result |= AR_ENEEDMORE;
            break;
         }

         intf_extn_event_id_allow_duty_cycling_t *event_payload = (intf_extn_event_id_allow_duty_cycling_t *)dsp_event_ptr->payload.data_ptr;

         cu_ptr->pm_info.flags.module_disallows_duty_cycling = !(event_payload->is_buffer_full_req_dcm_to_unblock_island_entry);
         CU_MSG(cu_ptr->gu_ptr->log_id,
                    DBG_HIGH_PRIO,
                    "Module 0x%lX: INTF_EXTN_EVENT_ID_ALLOW_DUTY_CYCLING. :module_disallows_duty_cycling:%d",
                    module_ptr->module_instance_id, cu_ptr->pm_info.flags.module_disallows_duty_cycling);
         break;
      }
      case INTF_EXTN_EVENT_ID_ALLOW_DUTY_CYCLING_V2:
      {
         //TODO aggregate for all modules
         if (dsp_event_ptr->payload.actual_data_len < sizeof(intf_extn_event_id_allow_duty_cycling_v2_t) || (!dsp_event_ptr->payload.data_ptr))
         {
            CU_MSG(cu_ptr->gu_ptr->log_id,
                    DBG_ERROR_PRIO,
                    "Module 0x%lX: INTF_EXTN_EVENT_ID_ALLOW_DUTY_CYCLING. : bad param length ",
                    module_ptr->module_instance_id);
            result |= AR_ENEEDMORE;
            break;
         }

         intf_extn_event_id_allow_duty_cycling_v2_t *event_payload = (intf_extn_event_id_allow_duty_cycling_v2_t *)dsp_event_ptr->payload.data_ptr;

         cu_ptr->pm_info.flags.module_disallows_duty_cycling = !(event_payload->allow_duty_cycling);
         CU_MSG(cu_ptr->gu_ptr->log_id,
                    DBG_HIGH_PRIO,
                    "Module 0x%lX: INTF_EXTN_EVENT_ID_ALLOW_DUTY_CYCLING. :module_disallows_duty_cycling:%d",
                    module_ptr->module_instance_id, cu_ptr->pm_info.flags.module_disallows_duty_cycling);
         break;
      }
      default:
      {
         result = cu_handle_event_to_dsp_service_topo_cb_for_path_delay(cu_ptr, module_ptr, event_info_ptr);
         if (AR_EUNSUPPORTED == result)
         {
            CU_MSG(cu_ptr->gu_ptr->log_id,
                   DBG_ERROR_PRIO,
                   "Module 0x%lX: Error in callback function. ID 0x%lx not supported.",
                   module_ptr->module_instance_id,
                   (dsp_event_ptr->param_id));
            return result;
         }
      }
   }

   return result;
}

ar_result_t cu_handle_capi_event(cu_base_t *        cu_ptr,
                                 gu_module_t *      module_ptr,
                                 capi_event_id_t    event_id,
                                 capi_event_info_t *event_info_ptr)
{
   ar_result_t  result        = AR_EOK;
   cu_module_t *cu_module_ptr = (cu_module_t *)((uint8_t *)module_ptr + cu_ptr->module_cu_offset);
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
         if (AR_EOK != (result = cu_find_client_info(cu_ptr->gu_ptr->log_id,
                                                     payload_ptr->param_id,
                                                     cu_module_ptr->event_list_ptr,
                                                     &client_list_ptr)))
         {
            CU_MSG(cu_ptr->gu_ptr->log_id,
                   DBG_ERROR_PRIO,
                   "Failed to get client list for event id: 0x%lx, result: %d",
                   payload_ptr->param_id,
                   result);
            result |= AR_EFAILED;
            break;
         }

         gpr_packet_t *      event_packet_ptr = NULL;
         apm_module_event_t *event_payload;

         for (cu_client_info_t *client_info_ptr = (cu_client_info_t *)(client_list_ptr->obj_ptr);
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
               CU_MSG(cu_ptr->gu_ptr->log_id,
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
               CU_MSG(cu_ptr->gu_ptr->log_id,
                      DBG_ERROR_PRIO,
                      "Module 0x%lX:  Unable to send event 0x%lX to client with result %lu "
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
               CU_MSG(cu_ptr->gu_ptr->log_id,
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
         result |= cu_handle_event_data_to_dsp_client_v2_topo_cb(cu_ptr, module_ptr, event_info_ptr);
         break;
      }
      default:
      {
         // others must be handled in topo layer
         break;
      }
   }

   return result;
}

bool_t cu_check_all_subgraphs_duty_cycling_allowed(cu_base_t *cu_ptr)
{
   for (gu_sg_list_t *sg_list_ptr = cu_ptr->gu_ptr->sg_list_ptr; (NULL != sg_list_ptr);
        LIST_ADVANCE(sg_list_ptr))
   {
      if (sg_list_ptr->sg_ptr->duty_cycling_mode_enabled)
      {
          cu_ptr->pm_info.flags.cntr_duty_cycling_allowed_subgraphs = TRUE;
      }
      else
      {
          cu_ptr->pm_info.flags.cntr_duty_cycling_allowed_subgraphs = FALSE;
         break;
      }
   }

   CU_MSG_ISLAND(cu_ptr->gu_ptr->log_id,
                DBG_HIGH_PRIO,
                "cntr_duty_cycling_enabled_subgraphs to %d (0-False, 1-True)",
                cu_ptr->pm_info.flags.cntr_duty_cycling_allowed_subgraphs);

   return cu_ptr->pm_info.flags.cntr_duty_cycling_allowed_subgraphs;
}
