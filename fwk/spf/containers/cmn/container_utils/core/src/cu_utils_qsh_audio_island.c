/**
 * \file cu_utils_qsh_audio_island.c
 *
 * \brief
 *
 *     CU's utility
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/* =======================================================================
INCLUDE FILES FOR MODULE
========================================================================== */
#include "container_utils.h"
#include "cu_i.h"
#include "capi_intf_extn_imcl.h"

/* Topo callback to handle module event to DSP client v2 */
ar_result_t cu_handle_event_data_to_dsp_client_v2_topo_cb(cu_base_t *        cu_ptr,
                                                          gu_module_t *      module_ptr,
                                                          capi_event_info_t *event_info_ptr)
{
   ar_result_t result = AR_EOK;
   if (event_info_ptr->payload.actual_data_len < sizeof(capi_event_data_to_dsp_client_v2_t))
   {
      result |= AR_ENEEDMORE;
      return result;
   }

   capi_event_data_to_dsp_client_v2_t *payload_ptr =
      (capi_event_data_to_dsp_client_v2_t *)(event_info_ptr->payload.data_ptr);
   gpr_packet_t *      event_packet_ptr = NULL;
   apm_module_event_t *event_payload;

   topo_evt_dest_addr_t dest_address;
   dest_address.address = payload_ptr->dest_address;
   /* Allocate the event packet
    * 64 bit destination address is populated as follows:
    * bits 0-31 : src port
    * bits 32-39: src domain id
    * bits 40-47: dest domain id
    * bits 48-63: 0 */

   gpr_cmd_alloc_ext_v2_t args;
   args.heap_index    = dest_address.a.gpr_heap_index;
   args.src_domain_id = dest_address.a.dest_domain_id;
   args.dst_domain_id = dest_address.a.src_domain_id;
   args.src_port      = module_ptr->module_instance_id;
   args.dst_port      = dest_address.a.src_port;
   args.token         = payload_ptr->token;
   args.opcode        = APM_EVENT_MODULE_TO_CLIENT;
   args.payload_size  = sizeof(apm_module_event_t) + payload_ptr->payload.actual_data_len;
   args.client_data   = 0;
   args.ret_packet    = &event_packet_ptr;
   result             = __gpr_cmd_alloc_ext_v2(&args);
   if (NULL == event_packet_ptr)
   {
      CU_MSG_ISLAND(cu_ptr->gu_ptr->log_id,
                    DBG_ERROR_PRIO,
                    "Module 0x%lX: event 0x%lX NOT sent to client with error code %lu",
                    module_ptr->module_instance_id,
                    payload_ptr->event_id,
                    result);
      return AR_EFAILED;
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
       CU_MSG_ISLAND(cu_ptr->gu_ptr->log_id,
                          DBG_ERROR_PRIO,
                          "Module 0x%lX: Unable to send event 0x%lX to client, error code %lu "
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
       CU_MSG_ISLAND(cu_ptr->gu_ptr->log_id,
                          DBG_HIGH_PRIO,
                          "Module 0x%lX: event 0x%lX sent to client with error code %lu "
                          "destn port: 0x%lX, destn domain ID: 0x%lX, src domain ID: 0x%lX ",
                          module_ptr->module_instance_id,
                          payload_ptr->event_id,
                          result,
                          dest_address.a.src_port,
                          dest_address.a.src_domain_id,
                          dest_address.a.dest_domain_id);
   }

   return result;
}
