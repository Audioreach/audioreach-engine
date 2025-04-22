/**
 * \file spdm_state_prop_handler.c
 * \brief
 *     This file contains functions to handle the state propagation requirements of offload graph
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "spdm_i.h"
#include "media_fmt_extn_api.h"
#include "offload_sp_api.h"

/* =======================================================================
Static Function Definitions
========================================================================== */

static ar_result_t spdm_process_peer_port_property_change(spgm_info_t *spgm_ptr,
                                                   uint32_t     port_index,
                                                   uint32_t     data_conn_type,
                                                   int8_t *     peer_property_cfg_ptr,
                                                   uint32_t     property_size)
{
   ar_result_t result = AR_EOK;
   INIT_EXCEPTION_HANDLING
   uint32_t log_id                 = 0;
   uint32_t rw_ep_miid             = 0;
   uint32_t rw_client_miid         = 0;
   uint32_t payload_size           = 0;
   int8_t * port_state_payload_ptr = NULL;

   log_id = spgm_ptr->sgm_id.log_id;

   if (OLC_IPC_READ_CLIENT_CONN == data_conn_type)
   {
      VERIFY(result, (NULL != spgm_ptr->process_info.rdp_obj_ptr[port_index]));
#ifdef SGM_ENABLE_STATE_PROPAGATION_MSG
      OLC_SDM_MSG(OLC_SDM_ID, DBG_HIGH_PRIO, "processing downstream port property propagation");
#endif
      // satellite read EP module IID
      rw_ep_miid = spgm_ptr->process_info.rdp_obj_ptr[port_index]->port_info.ctrl_cfg.rw_ep_miid;
      // read client module IID
      rw_client_miid = spgm_ptr->process_info.rdp_obj_ptr[port_index]->port_info.ctrl_cfg.rw_client_miid;
   }
   else
   {
      VERIFY(result, (NULL != spgm_ptr->process_info.wdp_obj_ptr[port_index]));
#ifdef SGM_ENABLE_STATE_PROPAGATION_MSG
      OLC_SDM_MSG(OLC_SDM_ID, DBG_HIGH_PRIO, "processing up_stream port property propagation");
#endif
      // satellite write EP module IID
      rw_ep_miid = spgm_ptr->process_info.wdp_obj_ptr[port_index]->port_info.ctrl_cfg.rw_ep_miid;
      // write client module IID
      rw_client_miid = spgm_ptr->process_info.wdp_obj_ptr[port_index]->port_info.ctrl_cfg.rw_client_miid;
   }

   payload_size = sizeof(apm_cmd_header_t) + sizeof(apm_module_param_data_t) + sizeof(param_id_peer_port_property_t) +
                  property_size;
   port_state_payload_ptr = (int8_t *)posal_memory_malloc(payload_size, spgm_ptr->cu_ptr->heap_id);
   if (NULL == port_state_payload_ptr)
   {
      OLC_SDM_MSG(OLC_SDM_ID,
                  DBG_ERROR_PRIO,
                  "failed to allocate memory for gpr"
                  "payload for downstream port state propagation");
      THROW(result, AR_ENOMEMORY);
   }

   memset((void *)port_state_payload_ptr, 0, sizeof(payload_size));

   apm_cmd_header_t *cmd_header_ptr = (apm_cmd_header_t *)port_state_payload_ptr;

   // fill the header with the size information
   cmd_header_ptr->payload_size        = payload_size - sizeof(apm_cmd_header_t);
   cmd_header_ptr->mem_map_handle      = 0;
   cmd_header_ptr->payload_address_lsw = 0;
   cmd_header_ptr->payload_address_msw = 0;

   apm_module_param_data_t *calib_header_ptr = (apm_module_param_data_t *)(cmd_header_ptr + 1);

   // fill the calibration payload
   calib_header_ptr->module_instance_id = rw_ep_miid;
   calib_header_ptr->param_id           = PARAM_ID_PEER_PORT_PROPERTY_UPDATE;
   calib_header_ptr->param_size         = sizeof(param_id_peer_port_property_t) + property_size;
   calib_header_ptr->error_code         = 0;

   // fill the port state information
   param_id_peer_port_property_t *ppp_ptr = (param_id_peer_port_property_t *)(calib_header_ptr + 1);
   ppp_ptr->num_properties                = 1;
   memscpy(ppp_ptr->peer_port_property_payload, property_size, peer_property_cfg_ptr, property_size);

   // fill the details to be filled in the GPR packet
   spgm_ptr->process_info.active_data_hndl.payload_size = payload_size;
   spgm_ptr->process_info.active_data_hndl.payload_ptr  = (uint8_t *)port_state_payload_ptr;
   spgm_ptr->process_info.active_data_hndl.src_port     = rw_client_miid;
   spgm_ptr->process_info.active_data_hndl.dst_port     = rw_ep_miid;
   spgm_ptr->process_info.active_data_hndl.opcode       = APM_CMD_SET_CFG;
   spgm_ptr->process_info.active_data_hndl.token        = 0;

   // Send the state configuration command to RD/WR EP module on the satellite graph
   TRY(result, sgm_ipc_send_data_pkt(spgm_ptr));

   // reset the active handle
   memset(&spgm_ptr->process_info.active_data_hndl, 0, sizeof(spgm_ipc_data_obj_t));

   CATCH(result, OLC_MSG_PREFIX, log_id)
   {
   }

   if (NULL != port_state_payload_ptr)
   {
      posal_memory_free(port_state_payload_ptr);
   }

   return result;
}

ar_result_t spdm_process_us_port_state_change(spgm_info_t *spgm_ptr, uint32_t port_index)
{
   ar_result_t result = AR_EOK;
   INIT_EXCEPTION_HANDLING
   uint32_t log_id            = 0;
   uint32_t wr_ep_port_id     = 0;
   uint32_t wr_client_port_id = 0;
   uint32_t payload_size      = 0;

   data_cmd_wr_sh_mem_ep_peer_port_property_t *port_state_cfg_ptr = NULL;

   log_id = spgm_ptr->sgm_id.log_id;
   VERIFY(result, (NULL != spgm_ptr->process_info.wdp_obj_ptr[port_index]));

#ifdef SGM_ENABLE_STATE_PROPAGATION_MSG
   OLC_SDM_MSG(OLC_SDM_ID, DBG_HIGH_PRIO, "processing upstream state propagation");
#endif

   // satellite write EP module IID
   wr_ep_port_id = spgm_ptr->process_info.wdp_obj_ptr[port_index]->port_info.ctrl_cfg.rw_ep_miid;
   // write client module IID
   wr_client_port_id = spgm_ptr->process_info.wdp_obj_ptr[port_index]->port_info.ctrl_cfg.rw_client_miid;

   payload_size = sizeof(data_cmd_wr_sh_mem_ep_peer_port_property_t) + sizeof(spf_msg_peer_port_property_info_t);

   port_state_cfg_ptr =
      (data_cmd_wr_sh_mem_ep_peer_port_property_t *)posal_memory_malloc(payload_size, spgm_ptr->cu_ptr->heap_id);

   if (NULL == port_state_cfg_ptr)
   {
      OLC_SDM_MSG(OLC_SDM_ID,
                  DBG_ERROR_PRIO,
                  "md_dbg: failed to allocate memory for gpr"
                  "payload for port state propagation while handling internal EOS");
      return AR_ENOMEMORY;
   }

   memset((void *)port_state_cfg_ptr, 0, payload_size);

   port_state_cfg_ptr->num_properties = 1;
   spf_msg_peer_port_property_info_t *port_property_info_ptr =
      (spf_msg_peer_port_property_info_t *)(port_state_cfg_ptr + 1);
   // fill the port state information
   port_property_info_ptr->property_type  = PORT_PROPERTY_DATA_FLOW_STATE;
   port_property_info_ptr->property_value = TOPO_DATA_FLOW_STATE_AT_GAP;

   // fill the details to be filled in the GPR packet
   spgm_ptr->process_info.active_data_hndl.payload_size = payload_size;
   spgm_ptr->process_info.active_data_hndl.payload_ptr  = (uint8_t *)port_state_cfg_ptr;
   spgm_ptr->process_info.active_data_hndl.src_port     = wr_client_port_id;
   spgm_ptr->process_info.active_data_hndl.dst_port     = wr_ep_port_id;
   spgm_ptr->process_info.active_data_hndl.opcode       = DATA_CMD_WR_SH_MEM_EP_PEER_PORT_PROPERTY;
   spgm_ptr->process_info.active_data_hndl.token        = 0;

   // Send the meta data calibration command to WR EP module on the satellite graph
   TRY(result, sgm_ipc_send_data_pkt(spgm_ptr));

   // reset the active handle
   memset(&spgm_ptr->process_info.active_data_hndl, 0, sizeof(spgm_ipc_data_obj_t));

   CATCH(result, OLC_MSG_PREFIX, log_id)
   {
   }

   if (NULL != port_state_cfg_ptr)
   {
      posal_memory_free(port_state_cfg_ptr);
   }
   return result;
}

ar_result_t spdm_process_upstream_stopped(spgm_info_t *spgm_ptr, uint32_t port_index)
{
   ar_result_t result = AR_EOK;
   INIT_EXCEPTION_HANDLING
   uint32_t log_id            = 0;
   uint32_t rw_ep_port_id     = 0;
   uint32_t rw_client_port_id = 0;
   uint32_t payload_size      = 0;

   int8_t *payload_ptr = NULL;

   VERIFY(result, (NULL != spgm_ptr));
   log_id = spgm_ptr->sgm_id.log_id;

   VERIFY(result, (NULL != spgm_ptr->process_info.wdp_obj_ptr[port_index]));
#ifdef SGM_ENABLE_STATE_PROPAGATION_MSG
   OLC_SDM_MSG(OLC_SDM_ID, DBG_HIGH_PRIO, "processing up_stream stopped cmd, started");
#endif
   // satellite write EP module IID
   rw_ep_port_id = spgm_ptr->process_info.wdp_obj_ptr[port_index]->port_info.ctrl_cfg.rw_ep_miid;
   // write client module IID
   rw_client_port_id = spgm_ptr->process_info.wdp_obj_ptr[port_index]->port_info.ctrl_cfg.rw_client_miid;

   payload_size = sizeof(apm_cmd_header_t) + sizeof(apm_module_param_data_t);
   payload_ptr  = (int8_t *)posal_memory_malloc(payload_size, spgm_ptr->cu_ptr->heap_id);
   if (NULL == payload_ptr)
   {
      OLC_SDM_MSG(OLC_SDM_ID,
                  DBG_ERROR_PRIO,
                  "failed to allocate memory for gpr"
                  "payload for upstream stop cmd");
      THROW(result, AR_ENOMEMORY);
   }

   memset((void *)payload_ptr, 0, sizeof(payload_size));

   apm_cmd_header_t *cmd_header_ptr = (apm_cmd_header_t *)payload_ptr;

   // fill the header with the size information
   cmd_header_ptr->payload_size        = payload_size - sizeof(apm_cmd_header_t);
   cmd_header_ptr->mem_map_handle      = 0;
   cmd_header_ptr->payload_address_lsw = 0;
   cmd_header_ptr->payload_address_msw = 0;

   apm_module_param_data_t *calib_header_ptr = (apm_module_param_data_t *)(cmd_header_ptr + 1);

   // fill the calibration payload
   calib_header_ptr->module_instance_id = rw_ep_port_id;
   calib_header_ptr->param_id           = PARAM_ID_UPSTREAM_STOPPED;
   calib_header_ptr->param_size         = 0;
   calib_header_ptr->error_code         = 0;

   // fill the details to be filled in the GPR packet
   spgm_ptr->process_info.active_data_hndl.payload_size = payload_size;
   spgm_ptr->process_info.active_data_hndl.payload_ptr  = (uint8_t *)payload_ptr;
   spgm_ptr->process_info.active_data_hndl.src_port     = rw_client_port_id;
   spgm_ptr->process_info.active_data_hndl.dst_port     = rw_ep_port_id;
   spgm_ptr->process_info.active_data_hndl.opcode       = APM_CMD_SET_CFG;
   spgm_ptr->process_info.active_data_hndl.token        = 0;

   TRY(result, sgm_ipc_send_data_pkt(spgm_ptr));

   // reset the active handle
   memset(&spgm_ptr->process_info.active_data_hndl, 0, sizeof(spgm_ipc_data_obj_t));

#ifdef SGM_ENABLE_STATE_PROPAGATION_MSG
   OLC_SDM_MSG(OLC_SDM_ID, DBG_HIGH_PRIO, "processing up_stream stopped cmd done, result %lu", result);
#endif

   CATCH(result, OLC_MSG_PREFIX, log_id)
   {
   }

   if (NULL != payload_ptr)
   {
      posal_memory_free(payload_ptr);
   }

   return result;
}

ar_result_t sdm_handle_peer_port_property_update_cmd(spgm_info_t *spgm_ptr, uint32_t port_index, void *property_ptr)
{
   ar_result_t result = AR_EOK;
   INIT_EXCEPTION_HANDLING
   uint32_t log_id            = 0;
   uint32_t data_conn_type    = 0;
   bool_t   is_valid_property = FALSE;

   VERIFY(result, (NULL != spgm_ptr));
   log_id = spgm_ptr->sgm_id.log_id;
   VERIFY(result, (NULL != property_ptr));

   OLC_SDM_MSG(OLC_SDM_ID, DBG_HIGH_PRIO, "processing peer port property");

   spf_msg_peer_port_property_info_t *peer_port_property_ptr = (spf_msg_peer_port_property_info_t *)property_ptr;

   switch (peer_port_property_ptr->property_type)
   {
      case PORT_PROPERTY_IS_UPSTREAM_RT:
      {
         data_conn_type    = OLC_IPC_WRITE_CLIENT_CONN;
         is_valid_property = TRUE;
         break;
      }
      case PORT_PROPERTY_IS_DOWNSTREAM_RT:
      case PORT_PROPERTY_TOPO_STATE:
      {
         data_conn_type    = OLC_IPC_READ_CLIENT_CONN;
         is_valid_property = TRUE;
         break;
      }
      default:
      {
         OLC_SDM_MSG(OLC_SDM_ID,
                     DBG_HIGH_PRIO,
                     "CMD:PEER_PORT_PROPERTY_UPDATE: Invalid property_type=0x%lx",
                     peer_port_property_ptr->property_type);
         THROW(result, AR_EUNSUPPORTED);
         break;
      }
   }

   if (TRUE == is_valid_property)
   {
      OLC_SDM_MSG(OLC_SDM_ID,
                  DBG_HIGH_PRIO,
                  "processing peer port property id %lu",
                  peer_port_property_ptr->property_type);
      TRY(result,
          spdm_process_peer_port_property_change(spgm_ptr,
                                                 port_index,
                                                 data_conn_type,
                                                 property_ptr,
                                                 sizeof(spf_msg_peer_port_property_info_t)));
   }

   CATCH(result, OLC_MSG_PREFIX, log_id)
   {
   }
   return result;
}

ar_result_t spgm_handle_event_upstream_state(spgm_info_t *spgm_ptr, gpr_packet_t *packet_ptr)
{
   ar_result_t result = AR_EOK;
   INIT_EXCEPTION_HANDLING
   uint32_t log_id                     = 0;
   uint32_t payload_size               = 0;
   uint32_t port_index                 = 0;
   upstream_state_cfg_event_t *event_cfg_ptr = NULL;

   VERIFY(result, (NULL != spgm_ptr));
   log_id = spgm_ptr->sgm_id.log_id;
   OLC_SGM_MSG(OLC_SGM_ID, DBG_MED_PRIO, "processing upstream_state configuration");

   VERIFY(result, (NULL != packet_ptr));
   payload_size = GPR_PKT_GET_PAYLOAD_BYTE_SIZE(packet_ptr->header);
   VERIFY(result, (payload_size >= sizeof(upstream_state_cfg_event_t)));

   event_cfg_ptr = (upstream_state_cfg_event_t *)GPR_PKT_GET_PAYLOAD(void, packet_ptr);
   VERIFY(result, (NULL != event_cfg_ptr));

   TRY(result, sgm_get_data_port_index_given_rw_ep_miid(spgm_ptr, IPC_READ_DATA, event_cfg_ptr->ep_miid, &port_index));

   VERIFY(result, (1 == event_cfg_ptr->num_properties));

   if (PORT_PROPERTY_TOPO_STATE == event_cfg_ptr->peer_port_property_payload.property_type)
   {
      topo_port_state_t ds_state = (topo_port_state_t)event_cfg_ptr->peer_port_property_payload.property_value;
      switch (ds_state)
      {
         case TOPO_PORT_STATE_STOPPED:
         {
            OLC_SDM_MSG(OLC_SDM_ID, DBG_HIGH_PRIO, "processing upstream stopped state event");
            break;
            // todo: do we need to do anything ??
         }
         case TOPO_PORT_STATE_STARTED:
         {
            // need to send the read buffers again from OLC to RD EP
            OLC_SDM_MSG(OLC_SDM_ID, DBG_HIGH_PRIO, "processing upstream started state event");
            break;
         }
         default:
         {
            OLC_SDM_MSG(OLC_SDM_ID, DBG_ERROR_PRIO, "processing upstream state, invalid state");
            break;
         }
      }
   }

   CATCH(result, OLC_MSG_PREFIX, log_id)
   {
   }

   return result;
}

ar_result_t spgm_handle_event_upstream_peer_port_property(spgm_info_t *spgm_ptr, gpr_packet_t *packet_ptr)
{
   ar_result_t result = AR_EOK;
   INIT_EXCEPTION_HANDLING
   uint32_t log_id                     = 0;
   uint32_t payload_size               = 0;
   uint32_t port_index                 = 0;
   uint32_t cnt_ext_port_bit_mask      = 0;
   uint32_t cnt_ext_out_port_bit_index = 0;
   uint32_t sce_payload_size           = 0;

   //read_data_port_obj_t *             rdp_obj_ptr            = NULL;
   state_cfg_event_t *                sce_rsp_ptr            = NULL;
   spf_msg_peer_port_property_info_t *peer_port_property_ptr = NULL;

   VERIFY(result, (NULL != spgm_ptr));
   log_id = spgm_ptr->sgm_id.log_id;

   OLC_SGM_MSG(OLC_SGM_ID, DBG_MED_PRIO, "processing upstream_peer_port_property propagation");

   VERIFY(result, (NULL != packet_ptr));
   payload_size = GPR_PKT_GET_PAYLOAD_BYTE_SIZE(packet_ptr->header);
   VERIFY(result, (payload_size >= sizeof(state_cfg_event_t)));

   sce_rsp_ptr = (state_cfg_event_t *)GPR_PKT_GET_PAYLOAD(void, packet_ptr);
   VERIFY(result, (NULL != sce_rsp_ptr));

   TRY(result, sgm_get_data_port_index_given_rw_ep_miid(spgm_ptr, IPC_READ_DATA, sce_rsp_ptr->ep_miid, &port_index));
   //rdp_obj_ptr           = spgm_ptr->process_info.rdp_obj_ptr[port_index];
   cnt_ext_port_bit_mask = spgm_ptr->process_info.rdp_obj_ptr[port_index]->port_info.ctrl_cfg.cnt_ext_port_bit_mask;
   cnt_ext_out_port_bit_index = cu_get_bit_index_from_mask(cnt_ext_port_bit_mask);

   sce_payload_size = sizeof(state_cfg_event_t);
   sce_payload_size += (sce_rsp_ptr->num_properties) * sizeof(spf_msg_peer_port_property_info_t);

   if (payload_size >= sce_payload_size)
   {
      OLC_SDM_MSG(OLC_SDM_ID,
                  DBG_HIGH_PRIO,
                  "US_SP, processing payload size %lu num_properties %lu",
                  payload_size,
                  sce_rsp_ptr->num_properties);

      peer_port_property_ptr = (spf_msg_peer_port_property_info_t *)(sce_rsp_ptr + 1);
      for (uint32_t i = 0; i < sce_rsp_ptr->num_properties; i++)
      {
         switch (peer_port_property_ptr->property_type)
         {
            case PORT_PROPERTY_IS_UPSTREAM_RT:
            {
               result |= olc_handle_peer_port_property_from_satellite_upstream(spgm_ptr->cu_ptr,
                                                                               cnt_ext_out_port_bit_index,
                                                                               peer_port_property_ptr);
               break;
            }
            case PORT_PROPERTY_DATA_FLOW_STATE:
            {
               result |= olc_reset_downstream_and_send_internal_eos(spgm_ptr->cu_ptr, cnt_ext_out_port_bit_index);
               break;
            }
            default:
            {
               OLC_SDM_MSG(OLC_SDM_ID,
                           DBG_ERROR_PRIO,
                           "US_SP,unsupported property type %lu",
                           peer_port_property_ptr->property_type);
               result |= AR_EBADPARAM;
            }
         }
         peer_port_property_ptr++;
      }
   }
   else
   {
      OLC_SDM_MSG(OLC_SDM_ID,
                  DBG_ERROR_PRIO,
                  "US_SP, payload size %lu required size %lu",
                  payload_size,
                  sce_payload_size);
      THROW(result, AR_EBADPARAM);
   }

   CATCH(result, OLC_MSG_PREFIX, log_id)
   {
   }

   return result;
}

ar_result_t spgm_handle_event_downstream_peer_port_property(spgm_info_t *spgm_ptr, gpr_packet_t *packet_ptr)
{
   ar_result_t result = AR_EOK;
   INIT_EXCEPTION_HANDLING
   uint32_t log_id                    = 0;
   uint32_t payload_size              = 0;
   uint32_t port_index                = 0;
   uint32_t cnt_ext_port_bit_mask     = 0;
   uint32_t cnt_ext_in_port_bit_index = 0;
   uint32_t sce_payload_size          = 0;
   state_cfg_event_t *                sce_rsp_ptr            = NULL;
   spf_msg_peer_port_property_info_t *peer_port_property_ptr = NULL;

   VERIFY(result, (NULL != spgm_ptr));
   log_id = spgm_ptr->sgm_id.log_id;

   OLC_SGM_MSG(OLC_SGM_ID, DBG_MED_PRIO, "processing downstream_peer_port_property propagation");

   VERIFY(result, (NULL != packet_ptr));
   payload_size = GPR_PKT_GET_PAYLOAD_BYTE_SIZE(packet_ptr->header);
   VERIFY(result, (payload_size >= sizeof(state_cfg_event_t)));

   sce_rsp_ptr = (state_cfg_event_t *)GPR_PKT_GET_PAYLOAD(void, packet_ptr);
   VERIFY(result, (NULL != sce_rsp_ptr));

   TRY(result, sgm_get_data_port_index_given_rw_ep_miid(spgm_ptr, IPC_WRITE_DATA, sce_rsp_ptr->ep_miid, &port_index));
   cnt_ext_port_bit_mask     = spgm_ptr->process_info.wdp_obj_ptr[port_index]->port_info.ctrl_cfg.cnt_ext_port_bit_mask;
   cnt_ext_in_port_bit_index = cu_get_bit_index_from_mask(cnt_ext_port_bit_mask);

   sce_payload_size = sizeof(state_cfg_event_t);
   sce_payload_size += (sce_rsp_ptr->num_properties) * sizeof(spf_msg_peer_port_property_info_t);

   if (payload_size >= sce_payload_size)
   {
      OLC_SDM_MSG(OLC_SDM_ID,
                  DBG_HIGH_PRIO,
                  "DS_SP processing, payload size %lu num_properties %lu",
                  payload_size,
                  sce_rsp_ptr->num_properties);

      peer_port_property_ptr = (spf_msg_peer_port_property_info_t *)(sce_rsp_ptr + 1);

      for (uint32_t i = 0; i < sce_rsp_ptr->num_properties; i++)
      {
         switch (peer_port_property_ptr->property_type)
         {
            case PORT_PROPERTY_IS_DOWNSTREAM_RT:
            case PORT_PROPERTY_TOPO_STATE:
            {
               result |= olc_handle_peer_port_property_from_satellite_downstream(spgm_ptr->cu_ptr,
                                                                                 cnt_ext_in_port_bit_index,
                                                                                 peer_port_property_ptr);
               break;
            }
            default:
            {
               OLC_SDM_MSG(OLC_SDM_ID,
                           DBG_ERROR_PRIO,
                           "DS_SP,unsupported property type %lu",
                           peer_port_property_ptr->property_type);
               result |= AR_EBADPARAM;
            }
         }
         peer_port_property_ptr++;
      }
   }
   else
   {
      OLC_SDM_MSG(OLC_SDM_ID,
                  DBG_ERROR_PRIO,
                  "DS_SP, payload size %lu required size %lu",
                  payload_size,
                  sce_payload_size);
      THROW(result, AR_EBADPARAM);
   }

   CATCH(result, OLC_MSG_PREFIX, log_id)
   {
   }

   return result;
}
