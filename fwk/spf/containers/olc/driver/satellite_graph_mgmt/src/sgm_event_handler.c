/**
 * \file sgm_event_handler.c
 * \brief
 *     This file contains the functions to handle the event registration and responses from the satellite Graph
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "sgm_i.h"
#include "offload_sp_api.h"

/* =======================================================================
Static Function Definitions
========================================================================== */

static ar_result_t spgm_offload_cfg_event_handler(spgm_info_t *spgm_ptr, gpr_packet_t *packet_ptr)
{
   ar_result_t result = AR_EOK;
   uint32_t    log_id = 0;
   INIT_EXCEPTION_HANDLING
   uint32_t            event_id    = 0;
   apm_module_event_t *payload_ptr = NULL;

   VERIFY(result, (NULL != spgm_ptr));
   log_id = spgm_ptr->sgm_id.log_id;

   ////payload will have apm_module_event_t followed by the payload
   payload_ptr = GPR_PKT_GET_PAYLOAD(apm_module_event_t, packet_ptr);
   VERIFY(result, (NULL != payload_ptr));

   event_id = payload_ptr->event_id;

   switch (event_id)
   {
      case OFFLOAD_EVENT_ID_GET_CONTAINER_DELAY:
      {
         TRY(result, spgm_handle_event_get_container_delay(spgm_ptr, packet_ptr));
         break;
      }
      default:
      {
         break;
      }
   }

   CATCH(result, OLC_MSG_PREFIX, log_id)
   {
   }

   return result;
}

bool_t sgm_get_event_cmd_hndl_node(spf_list_node_t *   evnt_reg_list_ptr,
                                   spgm_event_info_t **spgm_event_info_ptr,
                                   uint32_t            olc_event_reg_token)
{

   spf_list_node_t *  curr_node_ptr;
   spgm_event_info_t *evnt_reg_node_ptr = NULL;
   /** Get the pointer to start of the list of module list nodes */
   curr_node_ptr = evnt_reg_list_ptr;

   /** Check if the module instance  exists */
   while (curr_node_ptr)
   {
      evnt_reg_node_ptr = (spgm_event_info_t *)curr_node_ptr->obj_ptr;

      /** validate the instance pointer */
      if (NULL == evnt_reg_node_ptr)
      {
         return FALSE;
      }

      if (olc_event_reg_token == evnt_reg_node_ptr->olc_event_reg_token)
      {
         *spgm_event_info_ptr = evnt_reg_node_ptr;
         return TRUE;
      }

      /** Else, keep traversing the list */
      curr_node_ptr = curr_node_ptr->next_ptr;
   }
   return FALSE;
}

static ar_result_t spgm_module_event_handler(spgm_info_t *spgm_ptr, gpr_packet_t *packet_ptr)
{
   ar_result_t result = AR_EOK;
   uint32_t    log_id = 0;
   INIT_EXCEPTION_HANDLING

   gpr_packet_t *      event_packet_ptr         = NULL;
   apm_module_event_t *client_event_payload_ptr = NULL;
   apm_module_event_t *payload_ptr              = NULL;
   spgm_event_info_t * spgm_event_info_ptr      = NULL;
   uint32_t            event_id                 = 0;
   uint32_t            event_payload_size       = 0;
   bool_t              is_event_reg             = FALSE;

   VERIFY(result, (NULL != spgm_ptr));
   log_id = spgm_ptr->sgm_id.log_id;

   ////payload will have apm_module_event_t followed by the payload
   payload_ptr = GPR_PKT_GET_PAYLOAD(apm_module_event_t, packet_ptr);
   VERIFY(result, (NULL != payload_ptr));

   event_id           = payload_ptr->event_id;
   event_payload_size = payload_ptr->event_payload_size;

   OLC_SGM_MSG(OLC_SGM_ID,
               DBG_HIGH_PRIO,
               "event_rsp: rcvd event id 0x%lx of size %lu miid 0x%lx token(%lx)",
               event_id,
               event_payload_size,
               packet_ptr->src_port,
               packet_ptr->token);

   is_event_reg = sgm_get_event_cmd_hndl_node(spgm_ptr->event_reg_list_ptr, &spgm_event_info_ptr, packet_ptr->token);
   if (FALSE == is_event_reg)
   {
      return AR_EBADPARAM;
   }

   VERIFY(result, (NULL != spgm_event_info_ptr));

   if (spgm_event_info_ptr->module_iid != packet_ptr->src_port)
   {
      OLC_SGM_MSG(OLC_SGM_ID,
                  DBG_ERROR_PRIO,
                  "event_rsp: wrong payload, miid stored in node = 0x%lx, "
                  "event received from port 0x%lx, event_id is = 0x%lx",
                  spgm_event_info_ptr->module_iid,
                  packet_ptr->src_port,
                  event_id);
      return AR_EBADPARAM;
   }

   gpr_cmd_alloc_ext_t args;
   args.src_domain_id = packet_ptr->dst_domain_id;
   args.dst_domain_id = spgm_event_info_ptr->client_domain_id;
   args.src_port      = packet_ptr->src_port;
   args.dst_port      = spgm_event_info_ptr->client_port_id;
   args.token         = spgm_event_info_ptr->client_token;
   args.opcode        = APM_EVENT_MODULE_TO_CLIENT;
   args.payload_size  = sizeof(apm_module_event_t) + event_payload_size;
   args.client_data   = 0;
   args.ret_packet    = &event_packet_ptr;
   result             = __gpr_cmd_alloc_ext(&args);
   if (NULL == event_packet_ptr)
   {
      OLC_SGM_MSG(OLC_SGM_ID,
                  DBG_ERROR_PRIO,
                  "event_rsp: event not sent to client."
                  " failed to allocate memory event id is = 0x%lx",
                  event_id);
      return AR_EFAILED;
   }

   client_event_payload_ptr = GPR_PKT_GET_PAYLOAD(apm_module_event_t, event_packet_ptr);
   VERIFY(result, (NULL != client_event_payload_ptr));

   client_event_payload_ptr->event_id           = event_id;
   client_event_payload_ptr->event_payload_size = event_payload_size;

   memscpy(client_event_payload_ptr + 1, event_payload_size, payload_ptr + 1, event_payload_size);

   result = __gpr_cmd_async_send(event_packet_ptr);
   if (AR_DID_FAIL(result))
   {
      OLC_SGM_MSG(OLC_SGM_ID,
                  DBG_ERROR_PRIO,
                  "event_rsp: sending event to client failed, event ID is = 0x%lx",
                  event_id);
      __gpr_cmd_free(event_packet_ptr);
      return AR_EFAILED;
   }

   OLC_SGM_MSG(OLC_SGM_ID,
               DBG_MED_PRIO,
               "event_rsp: successfully sent event payload 0x%lx of size %lu to "
               "client port id 0x%lx, at domain id %lu",
               event_id,
               event_payload_size,
               spgm_event_info_ptr->client_port_id,
               spgm_event_info_ptr->client_domain_id);

   CATCH(result, OLC_MSG_PREFIX, log_id)
   {
   }

   return result;
}


/* Function to handle the events from the to Satellite PD*/
ar_result_t spgm_cmd_queue_handler(cu_base_t *cu_ptr, spgm_info_t *spgm_ptr,gpr_packet_t *packet_ptr)
{
   ar_result_t result          = AR_EOK;
   ar_result_t cmd_resp_result = AR_EOK;
   uint32_t    log_id          = 0;
   INIT_EXCEPTION_HANDLING

   VERIFY(result, (NULL != spgm_ptr));
   log_id = spgm_ptr->sgm_id.log_id;
   VERIFY(result, (NULL != cu_ptr));
   VERIFY(result, (NULL != packet_ptr));

   OLC_SGM_MSG(OLC_SGM_ID,
               DBG_HIGH_PRIO,
               "GPR cmd rcvd from satellite with "
               "cmd opcode(%lX) token(%lx) src_port (%lX) dst port (%lX)",
               packet_ptr->opcode,
               packet_ptr->token,
               packet_ptr->src_port,
               packet_ptr->dst_port);

   switch (packet_ptr->opcode)
   {
      case OFFLOAD_EVENT_ID_UPSTREAM_STATE:
      {
         TRY(result, spgm_handle_event_upstream_state(spgm_ptr, packet_ptr));
         __gpr_cmd_free(packet_ptr);
         packet_ptr = NULL;
         break;
      }

      case OFFLOAD_EVENT_ID_UPSTREAM_PEER_PORT_PROPERTY:
      {
         TRY(result, spgm_handle_event_upstream_peer_port_property(spgm_ptr, packet_ptr));
         __gpr_cmd_free(packet_ptr);
         packet_ptr = NULL;
         break;
      }

      case OFFLOAD_EVENT_ID_DOWNSTREAM_PEER_PORT_PROPERTY:
      {
         TRY(result, spgm_handle_event_downstream_peer_port_property(spgm_ptr, packet_ptr));
         __gpr_cmd_free(packet_ptr);
         packet_ptr = NULL;
         break;
      }

      default:
      {
         OLC_SGM_MSG(OLC_SGM_ID,
                     DBG_ERROR_PRIO,
                     "gpr cmd: rcvd with unsupported "
                     "pkt opcode(%lX) token(%lx)",
                     packet_ptr->opcode,
                     packet_ptr->token);

         cmd_resp_result = AR_EUNSUPPORTED;
         __gpr_cmd_end_command(packet_ptr, cmd_resp_result);
         packet_ptr      = NULL;
      }
   }

   CATCH(result, OLC_MSG_PREFIX, log_id)
   {
      if ((AR_EOK != result) && (NULL != packet_ptr)) // free the packet in failure case
      {
         __gpr_cmd_free(packet_ptr);
      }
   }
   return result;
}

/* Function to handle the events from the to Satellite PD*/
static ar_result_t spgm_event_rsp_handler(cu_base_t *cu_ptr, spgm_info_t *spgm_ptr)
{
   ar_result_t result          = AR_EOK;
   ar_result_t cmd_resp_result = AR_EOK;
   uint32_t    free_cmd_handle = TRUE;
   uint32_t    log_id          = 0;
   INIT_EXCEPTION_HANDLING

   spgm_cmd_rsp_node_t rsp_info   = { 0 };
   gpr_packet_t *      packet_ptr = NULL;

   VERIFY(result, (NULL != spgm_ptr));
   log_id = spgm_ptr->sgm_id.log_id;
   VERIFY(result, (NULL != cu_ptr));
   VERIFY(result, (NULL != spgm_ptr->cmd_rsp_vtbl));

   // Take next message from the queue.
   TRY(result, posal_queue_pop_front(spgm_ptr->evnt_q_ptr, (posal_queue_element_t *)&(spgm_ptr->event_msg)));

   packet_ptr = (gpr_packet_t *)spgm_ptr->event_msg.payload_ptr;
   VERIFY(result, (NULL != packet_ptr));

   OLC_SGM_MSG(OLC_SGM_ID,
               DBG_HIGH_PRIO,
               "event_rsp: rcvd from satellite with "
               "cmd opcode(%lX) token(%lx) src_port (%lX) dst port (%lX)",
               packet_ptr->opcode,
               packet_ptr->token,
			   packet_ptr->src_port,
			   packet_ptr->dst_port);

   switch (packet_ptr->opcode)
   {
      // Handling ibasic response
      case GPR_IBASIC_RSP_RESULT:
      {
         gpr_ibasic_rsp_result_t *rsp_ptr = (gpr_ibasic_rsp_result_t *)GPR_PKT_GET_PAYLOAD(void, packet_ptr);
         VERIFY(result, (NULL != rsp_ptr));

         rsp_info.rsp_result = rsp_ptr->status;
         rsp_info.opcode     = rsp_ptr->opcode;
         rsp_info.token      = packet_ptr->token;

         OLC_SGM_MSG(OLC_SGM_ID,
                     DBG_HIGH_PRIO,
                     "event_rsp: processing ibasic event response "
                     "for rsp opcode(%lX) token(%lx)",
                     rsp_ptr->opcode,
                     packet_ptr->token);

         switch (rsp_ptr->opcode)
         {
            default:
            {
               free_cmd_handle = FALSE;
               break;
            }
         }
         __gpr_cmd_free(packet_ptr);
         packet_ptr = NULL;
         break;
      }

      case APM_EVENT_MODULE_TO_CLIENT:
      {
         ////payload will have apm_module_event_t followed by the payload
         free_cmd_handle = FALSE;
         if (0 != packet_ptr->token)
         {
            TRY(result, spgm_module_event_handler(spgm_ptr, packet_ptr));
         }
         else
         {
            TRY(result, spgm_offload_cfg_event_handler(spgm_ptr, packet_ptr));
         }
         __gpr_cmd_free(packet_ptr);
         packet_ptr = NULL;
         break;
      }

      // Response media format
      case OFFLOAD_EVENT_ID_RD_SH_MEM_EP_MEDIA_FORMAT:
      {
         uint32_t rd_port_index = 0;
         free_cmd_handle = FALSE;
         result = sgm_get_data_port_index_given_rw_ep_miid(spgm_ptr,
                                                            IPC_READ_DATA,
                                                            packet_ptr->src_port,
                                                            &rd_port_index);
         TRY(result, spdm_process_media_format_event(spgm_ptr, packet_ptr, rd_port_index, FALSE));
         __gpr_cmd_free(packet_ptr);
         packet_ptr = NULL;
         break;
      }

      case OFFLOAD_EVENT_ID_SH_MEM_EP_OPERATING_FRAME_SIZE:
      {
         free_cmd_handle = FALSE;
         TRY(result, spgm_handle_event_opfs(spgm_ptr, packet_ptr));
         __gpr_cmd_free(packet_ptr);
         packet_ptr = NULL;
         break;
      }

      case EVENT_ID_MODULE_CMN_METADATA_CLONE_MD:
      {
    	  free_cmd_handle = FALSE;
    	  TRY(result, spgm_handle_event_clone_md(spgm_ptr, packet_ptr));
          __gpr_cmd_free(packet_ptr);
          packet_ptr = NULL;
    	  break;
      }

      case EVENT_ID_MODULE_CMN_METADATA_TRACKING_EVENT:
      {
    	  free_cmd_handle = FALSE;
    	  TRY(result, spgm_handle_tracking_md_event(spgm_ptr, packet_ptr));
          __gpr_cmd_free(packet_ptr);
          packet_ptr = NULL;
    	  break;
      }

      default:
      {
         OLC_SGM_MSG(OLC_SGM_ID,
                     DBG_ERROR_PRIO,
                     "event_rsp: rcvd response with unsupported "
                     "pkt opcode(%lX) token(%lx)",
                     packet_ptr->opcode,
                     packet_ptr->token);

         cmd_resp_result = AR_EUNSUPPORTED;
         __gpr_cmd_end_command(packet_ptr, cmd_resp_result);
         packet_ptr      = NULL;
         free_cmd_handle = FALSE;
      }
   }

   CATCH(result, OLC_MSG_PREFIX, log_id)
   {
      if ((AR_EOK != result) && (NULL != packet_ptr)) // free the packet in failure case
      {
         __gpr_cmd_free(packet_ptr);
      }
   }

   // Destroy the event handle for the specified token
   if (TRUE == free_cmd_handle)
   {
      sgm_destroy_cmd_handle(spgm_ptr, rsp_info.opcode, rsp_info.token);
   }
   return result;
}

/* Function to handle the event responses which are registered with the satellite Graph
 */
ar_result_t sgm_event_queue_handler(cu_base_t *cu_ptr, uint32_t channel_bit_index)
{
   ar_result_t result = AR_EOK;
   if (NULL == cu_ptr)
   {
      return AR_EBADPARAM;
   }
   uint8_t *    base_ptr = (uint8_t *)cu_ptr;
   spgm_info_t *spgm_ptr = (spgm_info_t *)(base_ptr + sizeof(cu_base_t));

   if (AR_EOK != (result = spgm_event_rsp_handler(cu_ptr, spgm_ptr)))
   {
      OLC_SGM_MSG(OLC_SGM_ID, DBG_ERROR_PRIO, "event_rsp: response handling failed");
   }

   return result;
}
