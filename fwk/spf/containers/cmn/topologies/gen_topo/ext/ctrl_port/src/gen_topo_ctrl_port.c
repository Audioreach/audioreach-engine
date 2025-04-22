/**
 * \file gen_topo_ctrl_port.c
 *
 * \brief
 *
 *     Implementation of path delay aspects of topology interface functions.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "gen_topo.h"
#include "gen_topo_capi.h"

ar_result_t gen_topo_set_ctrl_port_properties(gen_topo_module_t *module_ptr,
                                              gen_topo_t *       topo_ptr,
                                              bool_t             is_placeholder_replaced) /*TBD: Not used atm*/
{
   ar_result_t result = AR_EOK;

   INIT_EXCEPTION_HANDLING

   // New ports may be added. need to set it to module.
   for (gu_ctrl_port_list_t *ctrl_port_list_ptr = module_ptr->gu.ctrl_port_list_ptr; (NULL != ctrl_port_list_ptr);
        LIST_ADVANCE(ctrl_port_list_ptr))
   {
      gen_topo_ctrl_port_t *ctrl_port_ptr = (gen_topo_ctrl_port_t *)ctrl_port_list_ptr->ctrl_port_ptr;

      if (GU_STATUS_NEW == ctrl_port_ptr->gu.gu_status) // || is_placeholder_replaced)
      {
         if (module_ptr->capi_ptr)
         {
            TRY(result,
                gen_topo_set_ctrl_port_operation(&ctrl_port_ptr->gu, INTF_EXTN_IMCL_PORT_OPEN, topo_ptr->heap_id));
         }
      }
      ctrl_port_ptr->gu.gu_status = GU_STATUS_DEFAULT;
   }
   CATCH(result, TOPO_MSG_PREFIX, topo_ptr->gu.log_id)
   {
   }
   return AR_EOK;
}

/*Used to set ctrl port operations on modules. Currently, like data ports,
  we do this operation per port even though the API supports multiple ports.
  The caller is expected to iterate over all the ports and call this on the
  intended ports. */
ar_result_t gen_topo_set_ctrl_port_operation(gu_ctrl_port_t *             gu_ctrl_port_ptr,
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
      TOPO_MSG(log_id, DBG_ERROR_PRIO, "Received NULL pointers. ");
      return AR_EFAILED;
   }

   // Check if the module and capi ptrs are valid.
   gen_topo_ctrl_port_t *ctrl_port_ptr = (gen_topo_ctrl_port_t *)gu_ctrl_port_ptr;
   gen_topo_module_t *   module_ptr    = (gen_topo_module_t *)ctrl_port_ptr->gu.module_ptr;
   log_id                              = module_ptr->topo_ptr->gu.log_id;

   if (!module_ptr->capi_ptr)
   {
      TOPO_MSG(log_id,
               DBG_HIGH_PRIO,
               "Capi not created for module 0x%x, cannot set on ctrl port state.",
               module_ptr->gu.module_instance_id);
      return AR_EOK;
   }

   // Modules may not handle if the same command is issued twice. So issue command only if opcode is different.
   if (ctrl_port_ptr->last_issued_opcode == opcode)
   {
      return AR_EOK;
   }

   ctrl_port_id = ctrl_port_ptr->gu.id;

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
         gu_ctrl_port_t *ctrl_port_ptr = (gu_ctrl_port_t *)gu_find_ctrl_port_by_id(&module_ptr->gu, ctrl_port_id);
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
         if (ctrl_port_ptr->ext_ctrl_port_ptr)
         {
            peer_module_instance_id = ctrl_port_ptr->ext_ctrl_port_ptr->peer_handle.module_instance_id;
            peer_port_id            = ctrl_port_ptr->ext_ctrl_port_ptr->peer_handle.port_id;
         }
         else
         {
            peer_module_instance_id = ctrl_port_ptr->peer_ctrl_port_ptr->module_ptr->module_instance_id;
            peer_port_id            = ctrl_port_ptr->peer_ctrl_port_ptr->id;
         }

         TOPO_MSG(log_id,
                  DBG_LOW_PRIO,
                  "Control port open between (0x%08lX, 0x%lX) <-> (0x%08lX, 0x%lX), num_intents:%lu id1:0x%lx id2:0x%lx id3:0x%lx",
                  module_ptr->gu.module_instance_id,
                  ctrl_port_id,
                  peer_module_instance_id,
                  peer_port_id,
                  ctrl_port_ptr->intent_info_ptr->num_intents,
                  ctrl_port_ptr->intent_info_ptr->intent_id_list[0],
                  ctrl_port_ptr->intent_info_ptr->intent_id_list[1],
                  ctrl_port_ptr->intent_info_ptr->intent_id_list[2]);

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

         break;
      }
      default:
      {
         TOPO_MSG(log_id,
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
         TOPO_MSG(log_id,
                  DBG_HIGH_PRIO,
                  "Warning: Module doesn't support contrl port peer active/inactive states opcode %lu, result %d",
                  opcode,
                  capi_result);

         capi_result = CAPI_EOK;
      }
   }

   if (CAPI_FAILED(capi_result))
   {
      TOPO_MSG(log_id,
               DBG_ERROR_PRIO,
               "Failed to set control port state on the module iid= 0x%lx , port id= 0x%lx, opcode= 0x%lx, result = %d",
               module_ptr->gu.module_instance_id,
               ctrl_port_id,
               opcode,
               capi_result);
   }
   else
   {
      TOPO_MSG(log_id,
               DBG_LOW_PRIO,
               "Succesfully set Contrl port state set on the module iid= 0x%lx , port id= 0x%lx, opcode= 0x%lx",
               module_ptr->gu.module_instance_id,
               ctrl_port_id,
               opcode);

      // Update the last successful opcode issued on a given control port.
      ctrl_port_ptr->last_issued_opcode = opcode;
   }

   result = capi_err_to_ar_result(capi_result);

   CATCH(result, TOPO_MSG_PREFIX, log_id)
   {
   }

   /* Free memory allocated for the intent id list. */
   if ((INTF_EXTN_IMCL_PORT_OPEN == opcode) && port_op.op_payload.data_ptr)
   {
      MFREE_NULLIFY(port_op.op_payload.data_ptr);
   }

   return result;
}

ar_result_t gen_topo_check_set_connected_ctrl_port_operation(uint32_t                   log_id,
                                                             gen_topo_module_t *        this_module_ptr,
                                                             gen_topo_ctrl_port_t *     connected_port_ptr,
                                                             spf_cntr_sub_graph_list_t *spf_sg_list_ptr,
                                                             uint32_t                   sg_ops)
{
   ar_result_t                  result = AR_EOK;
   intf_extn_imcl_port_opcode_t opcode = INTF_EXTN_IMCL_PORT_CLOSE;

   gen_topo_module_t *connected_module_ptr = (gen_topo_module_t *)connected_port_ptr->gu.module_ptr;
   if (connected_module_ptr->capi_ptr)
   {
      result =
         gen_topo_set_ctrl_port_operation(&connected_port_ptr->gu, opcode, this_module_ptr->topo_ptr->heap_id);
   }
   else
   {
      // framework module
   }

   return result;
}

ar_result_t gen_topo_check_set_self_ctrl_port_operation(uint32_t              log_id,
                                                        gen_topo_ctrl_port_t *topo_ctrl_port_ptr,
                                                        uint32_t              sg_ops)
{
   ar_result_t        result     = AR_EOK;
   gen_topo_module_t *module_ptr = (gen_topo_module_t *)topo_ctrl_port_ptr->gu.module_ptr;

   if ((TOPO_SG_OP_STOP | TOPO_SG_OP_CLOSE | TOPO_SG_OP_SUSPEND) & sg_ops)
   {
      intf_extn_imcl_port_opcode_t opcode = INTF_EXTN_IMCL_PORT_STATE_INVALID;

      if ((TOPO_SG_OP_STOP | TOPO_SG_OP_SUSPEND) & sg_ops)
      {
         // For control port, port stop uses INTF_EXTN_IMCL_PORT_PEER_DISCONNECTED opcode
         opcode = INTF_EXTN_IMCL_PORT_PEER_DISCONNECTED;
      }
      else if (TOPO_SG_OP_CLOSE & sg_ops)
      {
         opcode = INTF_EXTN_IMCL_PORT_CLOSE;
      }

      if ((module_ptr->capi_ptr) && (INTF_EXTN_IMCL_PORT_STATE_INVALID != opcode))
      {
         result = gen_topo_set_ctrl_port_operation(&topo_ctrl_port_ptr->gu, opcode, module_ptr->topo_ptr->heap_id);
      }
      else
      {
         // framework module
      }
   }

   return result;
}



static intf_extn_imcl_port_opcode_t topo_port_state_to_ctrl_port_opcode(topo_port_state_t topo_state)
{
   switch (topo_state)
   {
      case TOPO_PORT_STATE_STARTED:
      {
         return INTF_EXTN_IMCL_PORT_PEER_CONNECTED;
      }
      case TOPO_PORT_STATE_STOPPED:
      case TOPO_PORT_STATE_SUSPENDED:
      {
         return INTF_EXTN_IMCL_PORT_PEER_DISCONNECTED;
      }
      default:
         return INTF_EXTN_IMCL_PORT_STATE_INVALID;
   }
}

ar_result_t gen_topo_set_ctrl_port_state(void *ctx_ptr, topo_port_state_t new_state)
{
   ar_result_t           result             = AR_EOK;
   gen_topo_ctrl_port_t *topo_ctrl_port_ptr = (gen_topo_ctrl_port_t *)ctx_ptr;
   gen_topo_module_t *   module_ptr         = (gen_topo_module_t *)topo_ctrl_port_ptr->gu.module_ptr;

   // Update the port state.
   topo_ctrl_port_ptr->state = new_state;

   intf_extn_imcl_port_opcode_t op_code = topo_port_state_to_ctrl_port_opcode(new_state);

   // Only START/STOP states are updated here.
   if (op_code != INTF_EXTN_IMCL_PORT_STATE_INVALID)
   {
      result = gen_topo_set_ctrl_port_operation(&topo_ctrl_port_ptr->gu, op_code, module_ptr->topo_ptr->heap_id);
   }

   return result;
}
