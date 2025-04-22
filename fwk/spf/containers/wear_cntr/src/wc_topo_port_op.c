/**
 * \file wc_topo_port_op.c
 *
 * \brief
 *
 *      This file contains functions for topo common data processing
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "wc_topo.h"
#include "wc_topo_capi.h"

ar_result_t wcntr_topo_set_ctrl_port_properties(wcntr_topo_module_t *module_ptr,
                                              wcntr_topo_t *       topo_ptr,
                                              bool_t             is_placeholder_replaced) /*TBD: Not used atm*/
{
   ar_result_t result = AR_EOK;

   INIT_EXCEPTION_HANDLING

   // New ports may be added. need to set it to module.
   for (wcntr_gu_ctrl_port_list_t *ctrl_port_list_ptr = module_ptr->gu.ctrl_port_list_ptr; (NULL != ctrl_port_list_ptr);
        LIST_ADVANCE(ctrl_port_list_ptr))
   {
      wcntr_topo_ctrl_port_t *ctrl_port_ptr = (wcntr_topo_ctrl_port_t *)ctrl_port_list_ptr->ctrl_port_ptr;

      if (WCNTR_GU_STATUS_NEW == ctrl_port_ptr->gu.gu_status) // || is_placeholder_replaced)
      {
         if (module_ptr->capi_ptr)
         {
            TRY(result,
                wcntr_topo_set_ctrl_port_operation(&ctrl_port_ptr->gu, INTF_EXTN_IMCL_PORT_OPEN, topo_ptr->heap_id));
         }
      }
      ctrl_port_ptr->gu.gu_status = WCNTR_GU_STATUS_DEFAULT;
   }
   CATCH(result, WCNTR_TOPO_MSG_PREFIX, topo_ptr->gu.log_id)
   {
   }
   return AR_EOK;
}

/*Used to set ctrl port operations on modules. Currently, like data ports,
  we do this operation per port even though the API supports multiple ports.
  The caller is expected to iterate over all the ports and call this on the
  intended ports. */
ar_result_t wcntr_topo_set_ctrl_port_operation(wcntr_gu_ctrl_port_t *             gu_ctrl_port_ptr,
                                             intf_extn_imcl_port_opcode_t opcode,
                                             POSAL_HEAP_ID                heap_id)
{
   ar_result_t result       = AR_EOK;
   capi_err_t  capi_result  = CAPI_EOK;
   uint32_t    ctrl_port_id = 0;
   uint32_t    log_id       = 0;
   INIT_EXCEPTION_HANDLING

   // Check for NULL pointers.
   if (!gu_ctrl_port_ptr || !gu_ctrl_port_ptr->module_ptr)
   {
      WCNTR_TOPO_MSG(log_id, DBG_ERROR_PRIO, "Received NULL pointers. ");
      return AR_EFAILED;
   }

   // Check if the module and capi ptrs are valid.
   wcntr_topo_ctrl_port_t *ctrl_port_ptr = (wcntr_topo_ctrl_port_t *)gu_ctrl_port_ptr;
   wcntr_topo_module_t *   module_ptr    = (wcntr_topo_module_t *)ctrl_port_ptr->gu.module_ptr;
   log_id                              = module_ptr->topo_ptr->gu.log_id;

   if (!module_ptr->capi_ptr)
   {
      WCNTR_TOPO_MSG(log_id,
               DBG_HIGH_PRIO,
               "Capi not created for module 0x%lX, cannot set on ctrl port state.",
               module_ptr->gu.module_instance_id);
      return AR_EOK;
   }

   ctrl_port_id = ctrl_port_ptr->gu.id;
   // Modules may not handle if the same command is issued twice. So issue command only if opcode is different.
   if (ctrl_port_ptr->last_issued_opcode == opcode)
   {
      WCNTR_TOPO_MSG(log_id,
               DBG_HIGH_PRIO,
               "Ctrl port state already set to opcode=0x%lx on module iid= 0x%lX , port id= 0x%lx",
               opcode,
               module_ptr->gu.module_instance_id,
               ctrl_port_id);
      return AR_EOK;
   }

 

   capi_port_info_t port_info;
   port_info.is_valid = FALSE;

   capi_buf_t buf;

   intf_extn_param_id_imcl_port_operation_t port_op;
   memset(&port_op, 0, sizeof(intf_extn_param_id_imcl_port_operation_t));

   port_op.opcode = opcode;

   struct
   {
      intf_extn_imcl_port_close_t close_payload;
      uint32_t                    port_id;
   } ctrl_port_close;

   struct
   {
      intf_extn_imcl_port_start_t start_payload;
      uint32_t                    port_id;
   } ctrl_port_start;

   struct
   {
      intf_extn_imcl_port_stop_t stop_payload;
      uint32_t                   port_id;
   } ctrl_port_stop;

   switch (opcode)
   {
      case INTF_EXTN_IMCL_PORT_OPEN:
      {
         uint32_t temp_size = 0;
         // First we need to check how many intents are associated with this ctrl port id.
         // Get the ctrl port handle from control port ID.
         // TBD: See if we can avoid search and pass the port handle
         wcntr_gu_ctrl_port_t *ctrl_port_ptr = (wcntr_gu_ctrl_port_t *)wcntr_gu_find_ctrl_port_by_id(&module_ptr->gu, ctrl_port_id);
         VERIFY(result, ((NULL != ctrl_port_ptr) && (NULL != ctrl_port_ptr->intent_info_ptr)));

         uint32_t size = sizeof(intf_extn_imcl_port_open_t) + sizeof(intf_extn_imcl_id_intent_map_t) +
                         (ctrl_port_ptr->intent_info_ptr->num_intents * sizeof(uint32_t));

         /*Filling the set param buf fields*/
         buf.actual_data_len = sizeof(port_op);
         buf.max_data_len    = buf.actual_data_len;
         buf.data_ptr        = (int8_t *)&port_op;

         /*Filling the port_op payload buf fields*/
         port_op.op_payload.actual_data_len = size;
         port_op.op_payload.max_data_len    = size;

         /*Mallocing at the op_payload*/
         intf_extn_imcl_port_open_t *open_payload_ptr =
            (intf_extn_imcl_port_open_t *)posal_memory_malloc(size, heap_id);

         VERIFY(result, NULL != open_payload_ptr);

         port_op.op_payload.data_ptr = (int8_t *)open_payload_ptr;

         uint32_t peer_module_instance_id = 0;
         uint32_t peer_port_id            = 0;

         {
            peer_module_instance_id = ctrl_port_ptr->peer_ctrl_port_ptr->module_ptr->module_instance_id;
            peer_port_id            = ctrl_port_ptr->peer_ctrl_port_ptr->id;
         }

         WCNTR_TOPO_MSG(log_id,
                  DBG_LOW_PRIO,
                  "Control port open between Module, Port (0x%08lX, 0x%lX) <-> (0x%08lX, 0x%lX)",
                  module_ptr->gu.module_instance_id,
                  ctrl_port_id,
                  peer_module_instance_id,
                  peer_port_id);

         open_payload_ptr->num_ports                             = 1; // per port
         open_payload_ptr->intent_map[0].port_id                 = ctrl_port_id;
         open_payload_ptr->intent_map[0].peer_module_instance_id = peer_module_instance_id;
         open_payload_ptr->intent_map[0].peer_port_id            = peer_port_id;
         open_payload_ptr->intent_map[0].num_intents             = ctrl_port_ptr->intent_info_ptr->num_intents;

         temp_size = memscpy(open_payload_ptr->intent_map[0].intent_arr,
                             open_payload_ptr->intent_map[0].num_intents * sizeof(uint32_t),
                             &ctrl_port_ptr->intent_info_ptr->intent_id_list[0],
                             ctrl_port_ptr->intent_info_ptr->num_intents * sizeof(uint32_t));

         VERIFY(result, open_payload_ptr->intent_map[0].num_intents * sizeof(uint32_t) == temp_size);

         break;
      }
      case INTF_EXTN_IMCL_PORT_CLOSE:
      {

         ctrl_port_close.close_payload.num_ports = 1;
         ctrl_port_close.port_id                 = ctrl_port_id;

         /*Filling the set param buf fields*/
         buf.actual_data_len = sizeof(port_op);
         buf.max_data_len    = buf.actual_data_len;
         buf.data_ptr        = (int8_t *)&port_op;

         /*Filling the port_op payload buf fields*/
         port_op.op_payload.actual_data_len = sizeof(ctrl_port_close);
         port_op.op_payload.max_data_len    = sizeof(ctrl_port_close);
         port_op.op_payload.data_ptr        = (int8_t *)(&ctrl_port_close);


         WCNTR_TOPO_MSG(log_id,
                  DBG_LOW_PRIO,
                  "INTF_EXTN_IMCL_PORT_CLOSE miid,port (0x%08lX, 0x%lX)",
                  module_ptr->gu.module_instance_id,
                  ctrl_port_id);		 

         break;
      }
      case INTF_EXTN_IMCL_PORT_PEER_CONNECTED:
      {
         ctrl_port_start.start_payload.num_ports = 1;
         ctrl_port_start.port_id                 = ctrl_port_id;

         /*Filling the set param buf fields*/
         buf.actual_data_len = sizeof(port_op);
         buf.max_data_len    = buf.actual_data_len;
         buf.data_ptr        = (int8_t *)&port_op;

         /*Filling the port_op payload buf fields*/
         port_op.op_payload.actual_data_len = sizeof(ctrl_port_start);
         port_op.op_payload.max_data_len    = sizeof(ctrl_port_start);
         port_op.op_payload.data_ptr        = (int8_t *)(&ctrl_port_start);

		          WCNTR_TOPO_MSG(log_id,
                  DBG_LOW_PRIO,
                  "INTF_EXTN_IMCL_PORT_PEER_CONNECTED miid,port (0x%08lX, 0x%lX)",
                  module_ptr->gu.module_instance_id,
                  ctrl_port_id);
				  
         break;
      }
      case INTF_EXTN_IMCL_PORT_PEER_DISCONNECTED:
      {
         ctrl_port_stop.stop_payload.num_ports = 1;
         ctrl_port_stop.port_id                = ctrl_port_id;

         /*Filling the set param buf fields*/
         buf.actual_data_len = sizeof(port_op);
         buf.max_data_len    = buf.actual_data_len;
         buf.data_ptr        = (int8_t *)&port_op;

         /*Filling the port_op payload buf fields*/
         port_op.op_payload.actual_data_len = sizeof(ctrl_port_stop);
         port_op.op_payload.max_data_len    = sizeof(ctrl_port_stop);
         port_op.op_payload.data_ptr        = (int8_t *)(&ctrl_port_stop);

		 		          WCNTR_TOPO_MSG(log_id,
                  DBG_LOW_PRIO,
                  "INTF_EXTN_IMCL_PORT_PEER_DISCONNECTED miid,port (0x%08lX, 0x%lX)",
                  module_ptr->gu.module_instance_id,
                  ctrl_port_id);

         break;
      }
      default:
      {
         WCNTR_TOPO_MSG(log_id,
                  DBG_ERROR_PRIO,
                  "set param for INTF_EXTN_PARAM_ID_IMCL_PORT_OPERATION unsupported opcode %lu",
                  opcode);
         return AR_EUNSUPPORTED;
      }
   }

   capi_result = module_ptr->capi_ptr->vtbl_ptr->set_param(module_ptr->capi_ptr,
                                                           INTF_EXTN_PARAM_ID_IMCL_PORT_OPERATION,
                                                           &port_info,
                                                           &buf);

   if (CAPI_EUNSUPPORTED == capi_result)
   {
      // All modules may note support peer active/inactive state handling.
      // Only modules sending trigger messages may implement these states.
      if ((opcode == INTF_EXTN_IMCL_PORT_PEER_CONNECTED) || (opcode == INTF_EXTN_IMCL_PORT_PEER_DISCONNECTED))
      {
         WCNTR_TOPO_MSG(log_id,
                  DBG_HIGH_PRIO,
                  "Warning: Module doesn't support contrl port peer active/inactive states opcode %lu, result %d",
                  opcode,
                  capi_result);

         capi_result = CAPI_EOK;
      }
   }

   if (CAPI_FAILED(capi_result))
   {
      WCNTR_TOPO_MSG(log_id,
               DBG_ERROR_PRIO,
               "Failed to set control port state on the module iid= 0x%lx , port id= 0x%lx, opcode= 0x%lx, result = %d",
               module_ptr->gu.module_instance_id,
               ctrl_port_id,
               opcode,
               capi_result);
   }
   else
   {
      WCNTR_TOPO_MSG(log_id,
               DBG_LOW_PRIO,
               "Succesfully set Contrl port state set on the miid,port (0x%08lX, 0x%lX) , opcode= 0x%lx",
               module_ptr->gu.module_instance_id,
               ctrl_port_id,
               opcode);

      // Update the last successful opcode issued on a given control port.
      ctrl_port_ptr->last_issued_opcode = opcode;
   }

   result = capi_err_to_ar_result(capi_result);

   CATCH(result, WCNTR_TOPO_MSG_PREFIX, log_id)
   {
   }

   /* Free memory allocated for the intent id list. */
   if ((INTF_EXTN_IMCL_PORT_OPEN == opcode) && port_op.op_payload.data_ptr)
   {
      MFREE_NULLIFY(port_op.op_payload.data_ptr);
   }

   return result;
}

ar_result_t wcntr_topo_check_set_connected_ctrl_port_operation(uint32_t                   log_id,
                                                             wcntr_topo_module_t *        this_module_ptr,
                                                             wcntr_topo_ctrl_port_t *     connected_port_ptr,
                                                             spf_cntr_sub_graph_list_t *spf_sg_list_ptr,
                                                             uint32_t                   sg_ops)
{
   ar_result_t                  result = AR_EOK;
   intf_extn_imcl_port_opcode_t opcode = INTF_EXTN_IMCL_PORT_CLOSE;

   // Inform the connected module in the connected intra-container SG only if it's not in the spf_sg_list_ptr coming
   // from the cmd. The module itself need not be informed as it's getting closed anyway
   if ((connected_port_ptr) &&
       ((connected_port_ptr->gu.module_ptr->sg_ptr->id != this_module_ptr->gu.sg_ptr->id) ||
        (!wcntr_gu_is_sg_id_found_in_spf_array(spf_sg_list_ptr, connected_port_ptr->gu.module_ptr->sg_ptr->id))))
   {
      if (!wcntr_gu_is_sg_id_found_in_spf_array(spf_sg_list_ptr, connected_port_ptr->gu.module_ptr->sg_ptr->id))
      {
         if ((WCNTR_TOPO_SG_OP_CLOSE | WCNTR_TOPO_SG_OP_DISCONNECT) & sg_ops)
         {
            wcntr_topo_module_t *connected_module_ptr = (wcntr_topo_module_t *)connected_port_ptr->gu.module_ptr;
            if (connected_module_ptr->capi_ptr)
            {
               result =
                  wcntr_topo_set_ctrl_port_operation(&connected_port_ptr->gu, opcode, this_module_ptr->topo_ptr->heap_id);
            }
            else
            {
               // framework module
            }
         }
      }
   }

   return AR_EOK;
}

ar_result_t wcntr_topo_check_set_self_ctrl_port_operation(uint32_t              log_id,
                                                        wcntr_topo_ctrl_port_t *topo_ctrl_port_ptr,
                                                        uint32_t              sg_ops)
{
   ar_result_t        result     = AR_EOK;
   wcntr_topo_module_t *module_ptr = (wcntr_topo_module_t *)topo_ctrl_port_ptr->gu.module_ptr;

   if ((WCNTR_TOPO_SG_OP_STOP | WCNTR_TOPO_SG_OP_CLOSE | WCNTR_TOPO_SG_OP_SUSPEND) & sg_ops)
   {
      intf_extn_imcl_port_opcode_t opcode = INTF_EXTN_IMCL_PORT_STATE_INVALID;

      if ((WCNTR_TOPO_SG_OP_STOP | WCNTR_TOPO_SG_OP_SUSPEND) & sg_ops)
      {
         // For control port, port stop uses INTF_EXTN_IMCL_PORT_PEER_DISCONNECTED opcode
         opcode = INTF_EXTN_IMCL_PORT_PEER_DISCONNECTED;
      }
      else if (WCNTR_TOPO_SG_OP_CLOSE & sg_ops)
      {
         opcode = INTF_EXTN_IMCL_PORT_CLOSE;
      }

      if ((module_ptr->capi_ptr) && (INTF_EXTN_IMCL_PORT_STATE_INVALID != opcode))
      {
         result = wcntr_topo_set_ctrl_port_operation(&topo_ctrl_port_ptr->gu, opcode, module_ptr->topo_ptr->heap_id);
      }
      else
      {
         // framework module
      }
   }

   return AR_EOK;
}

ar_result_t wcntr_topo_handle_incoming_ctrl_intent(void *   ctx_ptr,
                                                 void *   intent_buf,
                                                 uint32_t max_size,
                                                 uint32_t actual_size)
{
   ar_result_t result = AR_EOK;

   wcntr_topo_ctrl_port_t *                   topo_ctrl_port_ptr = (wcntr_topo_ctrl_port_t *)ctx_ptr;
   wcntr_topo_module_t *                      module_ptr         = (wcntr_topo_module_t *)topo_ctrl_port_ptr->gu.module_ptr;
   intf_extn_param_id_imcl_incoming_data_t *intent_hdr_ptr     = (intf_extn_param_id_imcl_incoming_data_t *)intent_buf;

   if (intent_hdr_ptr->port_id != topo_ctrl_port_ptr->gu.id)
   {
      WCNTR_TOPO_MSG(module_ptr->topo_ptr->gu.log_id,
               DBG_ERROR_PRIO,
               "Module: 0x%lx received a control message with ctrl port id: 0x%lx; actual port_id from gu is 0x%lx",
               module_ptr->gu.module_instance_id,
               intent_hdr_ptr->port_id,
               topo_ctrl_port_ptr->gu.id);
      return AR_EFAILED;
   }

   capi_buf_t buf;
   buf.data_ptr        = (int8_t *)intent_buf;
   buf.actual_data_len = actual_size;
   buf.max_data_len    = max_size;

   capi_port_info_t port_info;
   port_info.is_valid = FALSE;

   WCNTR_TOPO_MSG(module_ptr->topo_ptr->gu.log_id,
            DBG_MED_PRIO,
            "Module: 0x%lx received a control message on the ctrl port id: 0x%lx ",
            module_ptr->gu.module_instance_id,
            topo_ctrl_port_ptr->gu.id);

   result |= module_ptr->capi_ptr->vtbl_ptr->set_param(module_ptr->capi_ptr,
                                                       INTF_EXTN_PARAM_ID_IMCL_INCOMING_DATA,
                                                       &port_info,
                                                       &buf);

   // return  to buf mgr and make it NULL.
   if (CAPI_FAILED(result))
   {
      WCNTR_TOPO_MSG(module_ptr->topo_ptr->gu.log_id,
               DBG_ERROR_PRIO,
               "Sending incoming intent set param to module 0x%x failed result %d",
               module_ptr->gu.module_instance_id,
               result);
   }
   else
   {
      WCNTR_TOPO_MSG(module_ptr->topo_ptr->gu.log_id,
               DBG_LOW_PRIO,
               "Sending incoming intent set param to module 0x%x is done.",
               module_ptr->gu.module_instance_id);
   }

   return AR_EOK;
}

intf_extn_imcl_port_opcode_t wcntr_topo_port_state_to_ctrl_port_opcode(wcntr_topo_port_state_t topo_state)
{
   switch (topo_state)
   {
      case WCNTR_TOPO_PORT_STATE_STARTED:
      {
         return INTF_EXTN_IMCL_PORT_PEER_CONNECTED;
      }
      case WCNTR_TOPO_PORT_STATE_STOPPED:
      {
         return INTF_EXTN_IMCL_PORT_PEER_DISCONNECTED;
      }
      default:
         return INTF_EXTN_IMCL_PORT_STATE_INVALID;
   }
}

static intf_extn_imcl_port_opcode_t wcntr_topo_sg_state_to_ctrl_port_opcode(wcntr_topo_sg_state_t topo_sg_state)
{
   switch (topo_sg_state)
   {
      case WCNTR_TOPO_SG_STATE_STARTED:
      {
         return INTF_EXTN_IMCL_PORT_PEER_CONNECTED;
      }
      case WCNTR_TOPO_SG_STATE_STOPPED:
      case WCNTR_TOPO_SG_STATE_SUSPENDED:
      {
         return INTF_EXTN_IMCL_PORT_PEER_DISCONNECTED;
      }
      default:
         return INTF_EXTN_IMCL_PORT_STATE_INVALID;
   }
}

ar_result_t wcntr_topo_set_ctrl_port_state(void *ctx_ptr, wcntr_topo_port_state_t new_state)
{
   ar_result_t           result             = AR_EOK;
   wcntr_topo_ctrl_port_t *topo_ctrl_port_ptr = (wcntr_topo_ctrl_port_t *)ctx_ptr;
   wcntr_topo_module_t *   module_ptr         = (wcntr_topo_module_t *)topo_ctrl_port_ptr->gu.module_ptr;

   // Update the port state.
   topo_ctrl_port_ptr->state = new_state;

   intf_extn_imcl_port_opcode_t op_code = wcntr_topo_port_state_to_ctrl_port_opcode(new_state);

   // Only START/STOP states are updated here.
   if (op_code != INTF_EXTN_IMCL_PORT_STATE_INVALID)
   {
      result = wcntr_topo_set_ctrl_port_operation(&topo_ctrl_port_ptr->gu, op_code, module_ptr->topo_ptr->heap_id);
   }

   return result;
}

ar_result_t wcntr_topo_from_sg_state_set_ctrl_port_state(void *ctx_ptr, wcntr_topo_sg_state_t new_state)
{
   ar_result_t           result             = AR_EOK;
   wcntr_topo_ctrl_port_t *topo_ctrl_port_ptr = (wcntr_topo_ctrl_port_t *)ctx_ptr;
   wcntr_topo_module_t *   module_ptr         = (wcntr_topo_module_t *)topo_ctrl_port_ptr->gu.module_ptr;

   // Update the port state.
   //  topo_ctrl_port_ptr->state = new_state;

   intf_extn_imcl_port_opcode_t op_code = wcntr_topo_sg_state_to_ctrl_port_opcode(new_state);

   // Only START/STOP states are updated here.
   if (op_code != INTF_EXTN_IMCL_PORT_STATE_INVALID)
   {
      result = wcntr_topo_set_ctrl_port_operation(&topo_ctrl_port_ptr->gu, op_code, module_ptr->topo_ptr->heap_id);
   }

   return result;
}

ar_result_t wcntr_topo_intf_extn_data_ports_hdl_at_init(wcntr_topo_t *topo_ptr, wcntr_topo_module_t *module_ptr)
{
   INIT_EXCEPTION_HANDLING
   capi_err_t                       result          = CAPI_EOK;
   intf_extn_data_port_operation_t *in_port_op_ptr  = NULL;
   intf_extn_data_port_operation_t *out_port_op_ptr = NULL;

   if (!topo_ptr || !module_ptr)
   {
      return AR_EFAILED;
   }

   VERIFY(result, module_ptr->capi_ptr);

   // Open all the input ports of the module in one shot.
   if (module_ptr->gu.num_input_ports)
   {
      uint32_t size = sizeof(intf_extn_data_port_operation_t) +
                      sizeof(intf_extn_data_port_id_idx_map_t) * module_ptr->gu.num_input_ports;

      MALLOC_MEMSET(in_port_op_ptr, intf_extn_data_port_operation_t, size, topo_ptr->heap_id, result);

      in_port_op_ptr->is_input_port                      = TRUE;
      in_port_op_ptr->opcode                             = INTF_EXTN_DATA_PORT_OPEN;
      in_port_op_ptr->num_ports                          = module_ptr->gu.num_input_ports;
      in_port_op_ptr->opcode_payload_buf.data_ptr        = NULL;
      in_port_op_ptr->opcode_payload_buf.actual_data_len = 0;
      in_port_op_ptr->opcode_payload_buf.max_data_len    = 0;
      intf_extn_data_port_id_idx_map_t *id_idx_map_ptr   = in_port_op_ptr->id_idx;
      uint32_t                          j                = 0;
      for (wcntr_gu_input_port_list_t *in_port_list_ptr = module_ptr->gu.input_port_list_ptr; (NULL != in_port_list_ptr);
           LIST_ADVANCE(in_port_list_ptr))
      {
         wcntr_gu_input_port_t *ip_port_ptr = in_port_list_ptr->ip_port_ptr;
         id_idx_map_ptr[j].port_id    = ip_port_ptr->cmn.id;
         id_idx_map_ptr[j].port_index = ip_port_ptr->cmn.index;
         j++;
      }

      capi_buf_t buf;
      buf.actual_data_len = size;
      buf.max_data_len    = size;
      buf.data_ptr        = (int8_t *)(in_port_op_ptr);

      capi_port_info_t port_info;
      port_info.is_valid = FALSE;

      result |= module_ptr->capi_ptr->vtbl_ptr->set_param(module_ptr->capi_ptr,
                                                          INTF_EXTN_PARAM_ID_DATA_PORT_OPERATION,
                                                          &port_info,
                                                          &buf);
   }

   // Open all the output ports of the module in one shot.
   if (module_ptr->gu.num_output_ports)
   {
      uint32_t size = sizeof(intf_extn_data_port_operation_t) +
                      sizeof(intf_extn_data_port_id_idx_map_t) * module_ptr->gu.num_output_ports;

      MALLOC_MEMSET(out_port_op_ptr, intf_extn_data_port_operation_t, size, topo_ptr->heap_id, result);

      out_port_op_ptr->is_input_port                      = FALSE;
      out_port_op_ptr->opcode                             = INTF_EXTN_DATA_PORT_OPEN;
      out_port_op_ptr->num_ports                          = module_ptr->gu.num_output_ports;
      out_port_op_ptr->opcode_payload_buf.data_ptr        = NULL;
      out_port_op_ptr->opcode_payload_buf.actual_data_len = 0;
      out_port_op_ptr->opcode_payload_buf.max_data_len    = 0;
      intf_extn_data_port_id_idx_map_t *id_idx_map_ptr    = out_port_op_ptr->id_idx;
      uint32_t                          j                 = 0;
      for (wcntr_gu_output_port_list_t *out_port_list_ptr = module_ptr->gu.output_port_list_ptr; (NULL != out_port_list_ptr);
           LIST_ADVANCE(out_port_list_ptr))
      {
         wcntr_gu_output_port_t *out_port_ptr = out_port_list_ptr->op_port_ptr;
         id_idx_map_ptr[j].port_id      = out_port_ptr->cmn.id;
         id_idx_map_ptr[j].port_index   = out_port_ptr->cmn.index;
         j++;
      }

      capi_buf_t buf;
      buf.actual_data_len = size;
      buf.max_data_len    = size;
      buf.data_ptr        = (int8_t *)(out_port_op_ptr);

      capi_port_info_t port_info;
      port_info.is_valid = FALSE;

      result |= module_ptr->capi_ptr->vtbl_ptr->set_param(module_ptr->capi_ptr,
                                                          INTF_EXTN_PARAM_ID_DATA_PORT_OPERATION,
                                                          &port_info,
                                                          &buf);
   }

   CATCH(result, WCNTR_TOPO_MSG_PREFIX, topo_ptr->gu.log_id)
   {
   }

   // free allocated memory
   MFREE_NULLIFY(in_port_op_ptr);
   MFREE_NULLIFY(out_port_op_ptr);

   return result;
}

// correspnding func for control ports: wcntr_topo_port_state_to_ctrl_port_opcode
intf_extn_data_port_opcode_t wcntr_topo_port_state_to_data_port_opcode(wcntr_topo_port_state_t topo_state)
{
   switch (topo_state)
   {
      case WCNTR_TOPO_PORT_STATE_STARTED:
         return INTF_EXTN_DATA_PORT_START;

      case WCNTR_TOPO_PORT_STATE_STOPPED:
         return INTF_EXTN_DATA_PORT_STOP;
      default:
         return INTF_EXTN_DATA_PORT_OP_INVALID;
   }
}

// correspnding func for control ports: wcntr_topo_port_state_to_ctrl_port_opcode
intf_extn_data_port_opcode_t wcntr_topo_sg_state_to_data_port_opcode(wcntr_topo_sg_state_t topo_sg_state)
{
   switch (topo_sg_state)
   {
      case WCNTR_TOPO_SG_STATE_STARTED:
         return INTF_EXTN_DATA_PORT_START;

      case WCNTR_TOPO_SG_STATE_STOPPED:
         return INTF_EXTN_DATA_PORT_STOP;

      default:
         return INTF_EXTN_DATA_PORT_OP_INVALID;
   }
}

/**
 * port operation on one port. returns back for single port module.
 *
 * 1. in open:
 *    a) set port-open to the module one by one if only port is created (ext port or intra-container SG port)
 *    b) set port open for all ports if module itself id new.
   2. in close:
      a) if ext port close, close only port.
      b) if intra container SG  close is done, then close ports.
         if whole module is getting closed, no need to close ports.
 */
ar_result_t wcntr_topo_capi_set_data_port_op(wcntr_topo_module_t *           module_ptr,
                                           intf_extn_data_port_opcode_t  opcode,
                                           intf_extn_data_port_opcode_t *last_issued_opcode_ptr,
                                           bool_t                        is_input,
                                           uint32_t                      port_index,
                                           uint32_t                      port_id)
{
   capi_err_t  err_code = CAPI_EOK;
   ar_result_t result   = AR_EOK;
   wcntr_topo_t *topo_ptr = module_ptr->topo_ptr;
   uint32_t    log_id   = topo_ptr->gu.log_id;

   // function is only for CAPI modules
   if (!module_ptr->capi_ptr)
   {
      return result;
   }
   // if data port intf extension is not supported return
   if (FALSE == module_ptr->flags.supports_data_port_ops)
   {
      return result;
   }

   // For last port (first port), num_port must be decremented before (incremented after) calling this function

   // Modules may not handle if the same command is issued twice. So issue command only if opcode is different.
   // And Do not apply any operations on Closed ports. Closed ports will be eventually destroyed, they cannot move to
   // other states.
   if (*last_issued_opcode_ptr == opcode || (*last_issued_opcode_ptr == INTF_EXTN_DATA_PORT_CLOSE))
   {
      WCNTR_TOPO_MSG(log_id,
               DBG_LOW_PRIO,
               "No need to apply port operation on module iid= 0x%lx , port id= 0x%lx, last_issued_opcode=0x%lx, cur "
               "opcode=0x%lx",
               module_ptr->gu.module_instance_id,
               port_id,
               *last_issued_opcode_ptr,
               opcode);

      return AR_EOK;
   }

   struct
   {
      intf_extn_data_port_operation_t  op;
      intf_extn_data_port_id_idx_map_t id_index;
   } ops;

   ops.id_index.port_id                      = port_id;
   ops.id_index.port_index                   = port_index;
   ops.op.is_input_port                      = is_input;
   ops.op.num_ports                          = 1;
   ops.op.opcode                             = opcode;
   ops.op.opcode_payload_buf.actual_data_len = 0;
   ops.op.opcode_payload_buf.data_ptr        = NULL;
   ops.op.opcode_payload_buf.max_data_len    = 0;

   capi_buf_t buf;
   buf.data_ptr        = (int8_t *)(&ops);
   buf.actual_data_len = sizeof(ops);
   buf.max_data_len    = sizeof(ops);

   capi_port_info_t port_info;
   port_info.is_valid      = TRUE;
   port_info.is_input_port = is_input;
   port_info.port_index    = port_index;

   err_code = module_ptr->capi_ptr->vtbl_ptr->set_param(module_ptr->capi_ptr,
                                                        INTF_EXTN_PARAM_ID_DATA_PORT_OPERATION,
                                                        &port_info,
                                                        &buf);

   if ((err_code != CAPI_EOK) && (err_code != CAPI_EUNSUPPORTED))
   {
      WCNTR_TOPO_MSG(log_id, DBG_ERROR_PRIO, "setting port operation failed");
      return capi_err_to_ar_result(err_code);
   }
   else
   {
#ifdef DEBUG_TOPO_BOUNDARY_STATE_PROP
      WCNTR_TOPO_MSG(log_id,
               DBG_LOW_PRIO,
               "Data port op done with opcode=0x%lx on module iid= 0x%lx , port id= 0x%lx, result = 0x%lx",
               opcode,
               module_ptr->gu.module_instance_id,
               port_id,
               err_code);
#endif
   }

   // Update last issued opcode.
   *last_issued_opcode_ptr = opcode;

   return AR_EOK;
}

ar_result_t  wcntr_topo_capi_set_data_port_op_from_sg_ops(wcntr_topo_module_t *           module_ptr,
                                                       uint32_t                      sg_ops,
                                                       intf_extn_data_port_opcode_t *last_issued_opcode_ptr,
                                                       bool_t                        is_input,
                                                       uint32_t                      port_index,
                                                       uint32_t                      port_id)
{
   ar_result_t                  result = AR_EOK;
   intf_extn_data_port_opcode_t opcode = INTF_EXTN_DATA_PORT_OP_INVALID;

   if (WCNTR_TOPO_SG_OP_STOP & sg_ops)
   {
      opcode = INTF_EXTN_DATA_PORT_STOP;
   }
   else if (WCNTR_TOPO_SG_OP_SUSPEND & sg_ops)
   {
      opcode = INTF_EXTN_DATA_PORT_SUSPEND;
   }
   else if ((WCNTR_TOPO_SG_OP_CLOSE | WCNTR_TOPO_SG_OP_DISCONNECT) & sg_ops)
   {
      opcode = INTF_EXTN_DATA_PORT_CLOSE;
   }

   if (opcode == INTF_EXTN_DATA_PORT_OP_INVALID)
   {
      return result;
   }

   
         WCNTR_TOPO_MSG(module_ptr->topo_ptr->gu.log_id,
               DBG_LOW_PRIO,
               "wcntr_topo_capi_set_data_port_op_from_sg_ops module iid= 0x%lx , is_input %u,port id= 0x%lx, last_issued_opcode=0x%lx, current =0x%lx ",
               module_ptr->gu.module_instance_id,is_input,
               port_id,
               *last_issued_opcode_ptr,
               opcode);


   return wcntr_topo_capi_set_data_port_op(module_ptr, opcode, last_issued_opcode_ptr, is_input, port_index, port_id);
}

ar_result_t wcntr_topo_capi_set_data_port_op_from_state(wcntr_topo_module_t *           module_ptr,
                                                      wcntr_topo_port_state_t             downgraded_state,
                                                      intf_extn_data_port_opcode_t *last_issued_opcode_ptr,
                                                      bool_t                        is_input,
                                                      uint32_t                      port_index,
                                                      uint32_t                      port_id)
{
   ar_result_t result = AR_EOK;
   // convert the port state to Capi data port operation.
   intf_extn_data_port_opcode_t opcode = wcntr_topo_port_state_to_data_port_opcode(downgraded_state);

   // Only START/STOP states are updated here.
   if (opcode == INTF_EXTN_DATA_PORT_OP_INVALID)
   {
      return result;
   }

   return wcntr_topo_capi_set_data_port_op(module_ptr, opcode, last_issued_opcode_ptr, is_input, port_index, port_id);
}


ar_result_t wcntr_topo_capi_set_data_port_op_from_data_port_state(wcntr_topo_module_t *           module_ptr,
                                                         wcntr_topo_port_state_t               data_port_state,
                                                         intf_extn_data_port_opcode_t *last_issued_opcode_ptr,
                                                         bool_t                        is_input,
                                                         uint32_t                      port_index,
                                                         uint32_t                      port_id)
{

   ar_result_t result = AR_EOK;

  intf_extn_data_port_opcode_t opcode = wcntr_topo_port_state_to_data_port_opcode(data_port_state);

   // Only START/STOP states are updated here.
   if (opcode == INTF_EXTN_DATA_PORT_OP_INVALID)
   {
      return result;
   }

   return wcntr_topo_capi_set_data_port_op(module_ptr, opcode, last_issued_opcode_ptr, is_input, port_index, port_id);
}
													  

ar_result_t wcntr_topo_capi_set_data_port_op_from_sg_state(wcntr_topo_module_t *           module_ptr,
                                                         wcntr_topo_sg_state_t               sg_state,
                                                         intf_extn_data_port_opcode_t *last_issued_opcode_ptr,
                                                         bool_t                        is_input,
                                                         uint32_t                      port_index,
                                                         uint32_t                      port_id)
{

   ar_result_t result = AR_EOK;

   intf_extn_data_port_opcode_t opcode = wcntr_topo_sg_state_to_data_port_opcode(sg_state);

   // Only START/STOP states are updated here.
   if (opcode == INTF_EXTN_DATA_PORT_OP_INVALID)
   {
      return result;
   }

   return wcntr_topo_capi_set_data_port_op(module_ptr, opcode, last_issued_opcode_ptr, is_input, port_index, port_id);
}