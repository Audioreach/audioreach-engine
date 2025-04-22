/**
 * \file capi_cmn_ctrl_port_list.c
 *  
 * \brief
 *        utility to maintain list of control port.
 *  any module can use this utility to maintain its control port data base.
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "spf_list_utils.h"
#include "capi_cmn.h"
#include "capi_cmn_ctrl_port_list.h"

#define CHECK_THROW_ERROR(result, error_msg, ...)                                                                      \
   {                                                                                                                   \
      if (CAPI_FAILED(result))                                                                                         \
      {                                                                                                                \
         AR_MSG(DBG_ERROR_PRIO, error_msg, ##__VA_ARGS__);                                                             \
         return result;                                                                                                \
      }                                                                                                                \
   }

#if 0
static void print_port_data(ctrl_port_list_handle_t *me_ptr)
{
   spf_list_node_t *  head          = (spf_list_node_t *)me_ptr->list_head;
   ctrl_port_data_t *port_data_ptr = NULL;

   while (head)
   {
      port_data_ptr = (ctrl_port_data_t *)head->obj_ptr;
      head          = head->next_ptr;

      for (uint32_t i = 0; i < port_data_ptr->port_info.num_intents; i++)
      {
         AR_MSG(DBG_HIGH_PRIO,
                "control port 0x%x, status %lu, intent id 0x%x",
                port_data_ptr->port_info.port_id,
                port_data_ptr->state,
                port_data_ptr->port_info.intent_arr[i]);
      }
   }
}
#endif

static capi_err_t get_port_data(ctrl_port_list_handle_t *me_ptr, uint32_t port_id, ctrl_port_data_t **port_data_pptr)
{
   capi_err_t        result        = CAPI_EFAILED;
   spf_list_node_t * head          = (spf_list_node_t *)me_ptr->list_head;
   ctrl_port_data_t *port_data_ptr = NULL;

   if (port_data_pptr)
   {
      *port_data_pptr = NULL;
   }

   // move to the previously searched port_id/intent_id pair.
   while (head)
   {
      port_data_ptr = (ctrl_port_data_t *)head->obj_ptr;
      head          = head->next_ptr;

      if (port_data_ptr->port_info.port_id == port_id)
      {
         if (port_data_pptr)
         {
            *port_data_pptr = port_data_ptr;
         }

         result = CAPI_EOK;
         break;
      }
   }

   return result;
}

static capi_err_t set_port_state(ctrl_port_list_handle_t *me_ptr, uint32_t port_id, imcl_port_state_t state)
{
   capi_err_t        result        = CAPI_EOK;
   ctrl_port_data_t *port_data_ptr = NULL;

   result = get_port_data(me_ptr, port_id, &port_data_ptr);

   if (port_data_ptr)
   {
      port_data_ptr->state = state;
   }

   CHECK_THROW_ERROR(result, "Control port 0x%x not opened.", port_id);
   return result;
}

static capi_err_t close_port(ctrl_port_list_handle_t *me_ptr, uint32_t port_id)
{
   capi_err_t        result        = CAPI_EOK;
   ctrl_port_data_t *port_data_ptr = NULL;

   result = get_port_data(me_ptr, port_id, &port_data_ptr);

   if (port_data_ptr)
   {
      spf_list_find_delete_node((spf_list_node_t **)&me_ptr->list_head, port_data_ptr, FALSE);
      posal_memory_free(port_data_ptr);
   }

   CHECK_THROW_ERROR(result, "Control port 0x%x not opened.", port_id);
   return result;
}

void capi_cmn_ctrl_port_list_init(ctrl_port_list_handle_t *me_ptr)
{
   me_ptr->list_head = NULL;
}

capi_err_t capi_cmn_ctrl_port_list_open_port(ctrl_port_list_handle_t *       me_ptr,
                                             POSAL_HEAP_ID                   heap_id,
                                             uint32_t                        client_payload_size,
                                             intf_extn_imcl_id_intent_map_t *port_info_ptr,
                                             ctrl_port_data_t **             port_data_pptr)
{
   capi_err_t        result        = CAPI_EOK;
   ctrl_port_data_t *port_data_ptr = NULL;

   result |= (NULL == port_info_ptr) ? CAPI_EBADPARAM : result;

   CHECK_THROW_ERROR(result, "Bad pointer received.");

   // check if port-id is already in the list.
   if (CAPI_SUCCEEDED(get_port_data(me_ptr, port_info_ptr->port_id, port_data_pptr)))
   {
      AR_MSG(DBG_ERROR_PRIO, "Control port 0x%x already opened.", port_info_ptr->port_id);
      return CAPI_EFAILED;
   }

   uint32_t port_info_size = sizeof(intf_extn_imcl_id_intent_map_t) + port_info_ptr->num_intents * sizeof(uint32_t);
   uint32_t port_data_size = ALIGN_8_BYTES(sizeof(ctrl_port_data_t) + port_info_ptr->num_intents * sizeof(uint32_t)) +
                             ALIGN_8_BYTES(client_payload_size);

   // if no entry in the list then add a new one.
   port_data_ptr = (ctrl_port_data_t *)posal_memory_malloc(port_data_size, heap_id);

   if (port_data_ptr == NULL)
   {
      CHECK_THROW_ERROR(CAPI_ENOMEMORY, "Failed to allocate memory.");
   }

   memscpy(&port_data_ptr->port_info, port_info_size, port_info_ptr, port_info_size);
   port_data_ptr->state = CTRL_PORT_OPEN;

   if (AR_ENOMEMORY == spf_list_insert_head((spf_list_node_t **)&me_ptr->list_head, port_data_ptr, heap_id, FALSE))
   {
      posal_memory_free(port_data_ptr);
      CHECK_THROW_ERROR(CAPI_ENOMEMORY, "Failed to insert element into the list.");
   }

   if (port_data_pptr)
   {
      *port_data_pptr = port_data_ptr;
   }

   return CAPI_EOK;
}

capi_err_t capi_cmn_ctrl_port_list_set_state(ctrl_port_list_handle_t *me_ptr, uint32_t port_id, imcl_port_state_t state)
{
   switch (state)
   {
      case CTRL_PORT_OPEN:
      case CTRL_PORT_PEER_CONNECTED:
      case CTRL_PORT_PEER_DISCONNECTED:
      {
         return set_port_state(me_ptr, port_id, state);
         break;
      }
      case CTRL_PORT_CLOSE:
         return close_port(me_ptr, port_id);
         break;
      default:
         CHECK_THROW_ERROR(CAPI_EBADPARAM, "Invalid control port state 0x%x", state);
   }

   return CAPI_EOK;
}

void capi_cmn_ctrl_port_list_get_next_port_data(ctrl_port_list_handle_t *me_ptr,
                                                uint32_t                 intent_id,
                                                uint32_t                 prev_port_id,
                                                ctrl_port_data_t **      port_data_pptr)
{
   spf_list_node_t * head          = (spf_list_node_t *)me_ptr->list_head;
   ctrl_port_data_t *port_data_ptr = NULL;

   if (port_data_pptr)
   {
      *port_data_pptr = NULL;
   }

   // move to the previously searched port_id/intent_id pair.
   while (head && prev_port_id)
   {
      port_data_ptr = (ctrl_port_data_t *)head->obj_ptr;
      head          = head->next_ptr;

      if (port_data_ptr->port_info.port_id == prev_port_id)
      {
         break;
      }
   }

   // search for the next port_id for the given intent_id.
   while (head)
   {
      port_data_ptr = (ctrl_port_data_t *)head->obj_ptr;
      head          = head->next_ptr;

      for (uint32_t i = 0; i < port_data_ptr->port_info.num_intents; i++)
      {
         if (port_data_ptr->port_info.intent_arr[i] == intent_id)
         {
            if (port_data_pptr)
            {
               *port_data_pptr = port_data_ptr;
            }
            return;
         }
      }
   }
   return;
}

void capi_cmn_ctrl_port_list_get_port_data(ctrl_port_list_handle_t *me_ptr,
                                           uint32_t                 port_id,
                                           ctrl_port_data_t **      port_data_pptr)
{
   get_port_data(me_ptr, port_id, port_data_pptr);
   return;
}

void capi_cmn_ctrl_port_list_deinit(ctrl_port_list_handle_t *me_ptr)
{
   spf_list_delete_list_and_free_objs((spf_list_node_t **)&me_ptr->list_head, FALSE);
}

capi_err_t capi_cmn_ctrl_port_operation_handler(ctrl_port_list_handle_t *me_ptr,
                                                capi_buf_t *             param_ptr,
                                                POSAL_HEAP_ID            heap_id,
                                                uint32_t                 client_payload_size,
                                                uint32_t                 num_intent,
                                                uint32_t *               supported_intent_id_arr)
{
   capi_err_t                                result = CAPI_EOK;
   intf_extn_param_id_imcl_port_operation_t *port_op_ptr =
      (intf_extn_param_id_imcl_port_operation_t *)(param_ptr->data_ptr);

   result |= (param_ptr->actual_data_len < sizeof(intf_extn_param_id_imcl_port_operation_t)) ? CAPI_ENEEDMORE : result;
   CHECK_THROW_ERROR(result,
                     "Insufficient payload size for ctrl port operation. Received %lu bytes",
                     param_ptr->actual_data_len);

   result |= (port_op_ptr->op_payload.data_ptr == NULL) ? CAPI_EFAILED : result;
   CHECK_THROW_ERROR(result, "null payload received for control port operation");

   switch (port_op_ptr->opcode)
   {
      case INTF_EXTN_IMCL_PORT_OPEN:
      {
         intf_extn_imcl_port_open_t *port_open_ptr = (intf_extn_imcl_port_open_t *)port_op_ptr->op_payload.data_ptr;

         // Size Validation
         uint32_t valid_size = sizeof(intf_extn_imcl_port_open_t);
         result |= (port_op_ptr->op_payload.actual_data_len < valid_size) ? CAPI_ENEEDMORE : result;
         CHECK_THROW_ERROR(result,
                           "Insufficient control port open payload size. Received %lu bytes",
                           port_op_ptr->op_payload.actual_data_len);

         // Size validation considering one intent per port.
         valid_size = sizeof(intf_extn_imcl_port_open_t) +
                      port_open_ptr->num_ports * (sizeof(intf_extn_imcl_id_intent_map_t) + sizeof(uint32_t));
         result |= (port_op_ptr->op_payload.actual_data_len < valid_size) ? CAPI_ENEEDMORE : result;
         CHECK_THROW_ERROR(result,
                           "Insufficient control port open payload size. Received %lu bytes",
                           port_op_ptr->op_payload.actual_data_len);

         for (uint32_t i = 0; i < port_open_ptr->num_ports; i++)
         {
            // Size validation if number of intents per port is more than one.
            valid_size += (port_open_ptr->intent_map[i].num_intents - 1) * sizeof(uint32_t);
            result |= (port_op_ptr->op_payload.actual_data_len < valid_size) ? CAPI_ENEEDMORE : result;
            CHECK_THROW_ERROR(result,
                              "Insufficient control port open payload size. Received %lu bytes",
                              port_op_ptr->op_payload.actual_data_len);

            for (uint32_t j = 0; j < port_open_ptr->intent_map[i].num_intents; j++)
            {
               result = CAPI_EUNSUPPORTED;
               for (uint32_t k = 0; k < num_intent; k++)
               {
                  if (port_open_ptr->intent_map[i].intent_arr[j] == supported_intent_id_arr[k])
                  {
                     result = CAPI_EOK;
                  }
               }
               CHECK_THROW_ERROR(result, "Intent ID: 0x%x, not supported.", port_open_ptr->intent_map[i].intent_arr[j]);
            }
            result = capi_cmn_ctrl_port_list_open_port(me_ptr,
                                                       heap_id,
                                                       client_payload_size,
                                                       &port_open_ptr->intent_map[i],
                                                       NULL);
            CHECK_THROW_ERROR(result, "Control port 0x%x open failed", port_open_ptr->intent_map[i].port_id);

            AR_MSG(DBG_MED_PRIO, "Opening Control port id: 0x%x", port_open_ptr->intent_map[i].port_id);
         }

         break;
      }
      case INTF_EXTN_IMCL_PORT_PEER_CONNECTED:
      {
         intf_extn_imcl_port_start_t *port_start_ptr = (intf_extn_imcl_port_start_t *)port_op_ptr->op_payload.data_ptr;

         // Size Validation: port close payload
         uint32_t valid_size = sizeof(intf_extn_imcl_port_start_t);
         result |= (port_op_ptr->op_payload.actual_data_len < valid_size) ? CAPI_ENEEDMORE : result;
         CHECK_THROW_ERROR(result,
                           "Insufficient control port start payload size. Received %lu bytes",
                           port_op_ptr->op_payload.actual_data_len);

         valid_size = sizeof(intf_extn_imcl_port_start_t) + port_start_ptr->num_ports * sizeof(uint32_t);
         result |= (port_op_ptr->op_payload.actual_data_len < valid_size) ? CAPI_ENEEDMORE : result;
         CHECK_THROW_ERROR(result,
                           "Insufficient control port start payload size. Received %lu bytes",
                           port_op_ptr->op_payload.actual_data_len);

         for (uint32_t i = 0; i < port_start_ptr->num_ports; i++)
         {
            result =
               capi_cmn_ctrl_port_list_set_state(me_ptr, port_start_ptr->port_id_arr[i], CTRL_PORT_PEER_CONNECTED);
            CHECK_THROW_ERROR(result,
                              "Control port start failed for control port id: 0x%x",
                              port_start_ptr->port_id_arr[i]);

            AR_MSG(DBG_MED_PRIO, "Starting Control port id: 0x%x", port_start_ptr->port_id_arr[i]);
         }
         break;
      }
      case INTF_EXTN_IMCL_PORT_PEER_DISCONNECTED:
      {
         intf_extn_imcl_port_stop_t *port_stop_ptr = (intf_extn_imcl_port_stop_t *)port_op_ptr->op_payload.data_ptr;

         // Size Validation: port close payload
         uint32_t valid_size = sizeof(intf_extn_imcl_port_stop_t);
         result |= (port_op_ptr->op_payload.actual_data_len < valid_size) ? CAPI_ENEEDMORE : result;
         CHECK_THROW_ERROR(result,
                           "Insufficient control port stop payload size. Received %lu bytes",
                           port_op_ptr->op_payload.actual_data_len);

         valid_size = sizeof(intf_extn_imcl_port_stop_t) + port_stop_ptr->num_ports * sizeof(uint32_t);
         result |= (port_op_ptr->op_payload.actual_data_len < valid_size) ? CAPI_ENEEDMORE : result;
         CHECK_THROW_ERROR(result,
                           "Insufficient control port stop payload size. Received %lu bytes",
                           port_op_ptr->op_payload.actual_data_len);

         for (uint32_t i = 0; i < port_stop_ptr->num_ports; i++)
         {
            result =
               capi_cmn_ctrl_port_list_set_state(me_ptr, port_stop_ptr->port_id_arr[i], CTRL_PORT_PEER_DISCONNECTED);
            CHECK_THROW_ERROR(result,
                              "Control port stop failed for control port id: 0x%x",
                              port_stop_ptr->port_id_arr[i]);

            AR_MSG(DBG_MED_PRIO, "stopping Control port id: 0x%x", port_stop_ptr->port_id_arr[i]);
         }
         break;
      }
      case INTF_EXTN_IMCL_PORT_CLOSE:
      {
         intf_extn_imcl_port_close_t *port_close_ptr = (intf_extn_imcl_port_close_t *)port_op_ptr->op_payload.data_ptr;

         // Size Validation: port close payload
         uint32_t valid_size = sizeof(intf_extn_imcl_port_close_t);
         result |= (port_op_ptr->op_payload.actual_data_len < valid_size) ? CAPI_ENEEDMORE : result;
         CHECK_THROW_ERROR(result,
                           "Insufficient control port close payload size. Received %lu bytes",
                           port_op_ptr->op_payload.actual_data_len);

         valid_size = sizeof(intf_extn_imcl_port_close_t) + port_close_ptr->num_ports * sizeof(uint32_t);
         result |= (port_op_ptr->op_payload.actual_data_len < valid_size) ? CAPI_ENEEDMORE : result;
         CHECK_THROW_ERROR(result,
                           "Insufficient control port close payload size. Received %lu bytes",
                           port_op_ptr->op_payload.actual_data_len);

         for (uint32_t i = 0; i < port_close_ptr->num_ports; i++)
         {
            result = capi_cmn_ctrl_port_list_set_state(me_ptr, port_close_ptr->port_id_arr[i], CTRL_PORT_CLOSE);
            CHECK_THROW_ERROR(result,
                              "Control port close failed for control port id: 0x%x",
                              port_close_ptr->port_id_arr[i]);

            AR_MSG(DBG_MED_PRIO, "closing Control port id: 0x%x", port_close_ptr->port_id_arr[i]);
         }

         break;
      }
      default:
      {
         AR_MSG(DBG_ERROR_PRIO, "Received unsupported ctrl port opcode %lu", port_op_ptr->opcode);
         return CAPI_EUNSUPPORTED;
      }
   }

   return result;
}
