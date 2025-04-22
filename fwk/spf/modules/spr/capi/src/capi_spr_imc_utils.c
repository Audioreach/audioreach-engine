/**
 *   \file capi_spr_imc_utils.c
 *   \brief
 *        This file contains CAPI V2 IMC utils implementation of SPR module
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "capi_spr_i.h"

/*==============================================================================
   Local Defines
==============================================================================*/

/*==============================================================================
   Local Function forward declaration
==============================================================================*/
static spr_ctrl_port_list_t *spr_get_ctrl_port_list_node(capi_spr_t *me_ptr, uint32_t ctrl_port_id);
static capi_err_t            spr_send_drift_handle_per_port(capi_spr_t *me_ptr, spr_ctrl_port_t *ctrl_port_ptr);
static bool_t                capi_spr_is_ctrl_port_mapped_to_output_port(capi_spr_t *me_ptr, uint32_t ctrl_port_id);
/*==============================================================================
      Public Function Implementation
==============================================================================*/

static bool_t capi_spr_is_ctrl_port_mapped_to_output_port(capi_spr_t *me_ptr, uint32_t ctrl_port_id)
{
   for (uint32_t arr_index = 0; arr_index < me_ptr->max_output_ports; arr_index++)
   {
      if (ctrl_port_id == me_ptr->out_port_info_arr[arr_index].ctrl_port_id)
      {
         SPR_MSG(me_ptr->miid,
                 DBG_HIGH_PRIO,
                 "capi_spr_is_ctrl_port_mapped_to_output_port : ctrl_port_id 0x%lX is mapped to output port id 0x%lX "
                 "with arr_index %d",
                 ctrl_port_id,
                 me_ptr->out_port_info_arr[arr_index].port_id,
                 arr_index);
         return TRUE;
      }
   }

   SPR_MSG(me_ptr->miid,
           DBG_HIGH_PRIO,
           "capi_spr_is_ctrl_port_mapped_to_output_port: ctrl_port_id 0x%lX is not mapped to any output port",
           ctrl_port_id);

   return FALSE;
}

/* Create mutex and setting drift function */
capi_err_t capi_spr_init_out_drift_info(spr_drift_info_t *drift_info_ptr, imcl_tdi_get_acc_drift_fn_t get_drift_fn_ptr)
{
   capi_err_t result = CAPI_EOK;

   if (!drift_info_ptr)
   {
      return CAPI_EBADPARAM;
   }

   /** Clear the drift info structure  */
   memset(drift_info_ptr, 0, sizeof(spr_drift_info_t));

   /* Create mutex for the drift info shared with rate matching modules */
   ar_result_t rc = posal_mutex_create(&drift_info_ptr->drift_info_mutex, POSAL_HEAP_DEFAULT);
   if (rc)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi_spr: Failed to create mutex");
      return CAPI_EFAILED;
   }

   /**Set the function pointer for querying the drift */
   drift_info_ptr->drift_info_hdl.get_drift_fn_ptr = get_drift_fn_ptr;

   return result;
}

/* Destroy mutex */
capi_err_t capi_spr_deinit_out_drift_info(spr_drift_info_t *drift_info_ptr)
{
   capi_err_t result = CAPI_EOK;

   if (!drift_info_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi_spr: Drift info pointer is NULL");
      return CAPI_EBADPARAM;
   }

   /** Destroy mutex */
   if (drift_info_ptr->drift_info_mutex)
   {
      posal_mutex_destroy(&drift_info_ptr->drift_info_mutex);
   }

   return result;
}

/* Copy from the spr modules (drift_info_hdl_ptr) to the drift pointer (acc_drift_out_ptr) */
ar_result_t spr_read_acc_out_drift(imcl_tdi_hdl_t *drift_info_hdl_ptr, imcl_tdi_acc_drift_t *acc_drift_out_ptr)
{
   ar_result_t result = AR_EOK;

   if (!drift_info_hdl_ptr || !acc_drift_out_ptr)
   {
      return AR_EFAILED;
   }

   spr_drift_info_t *shared_drift_ptr = (spr_drift_info_t *)drift_info_hdl_ptr;

   posal_mutex_lock(shared_drift_ptr->drift_info_mutex);

   /** Copy the accumulated drift info */
   memscpy(acc_drift_out_ptr,
           sizeof(imcl_tdi_acc_drift_t),
           &shared_drift_ptr->spr_acc_drift,
           sizeof(imcl_tdi_acc_drift_t));

   posal_mutex_unlock(shared_drift_ptr->drift_info_mutex);

   return result;
}

//get the updated drift value so that the instantaneous drift becomes zero.
static void capi_spr_resync_drift(capi_spr_t *me_ptr, spr_ctrl_port_t *ctrl_port_ptr)
{
   if ((ctrl_port_ptr) && (CTRL_PORT_PEER_CONNECTED == ctrl_port_ptr->state))
   {
      capi_spr_imcl_get_drift(me_ptr, ctrl_port_ptr);
   }
}

capi_err_t capi_spr_imcl_handle_incoming_data(capi_spr_t *me_ptr, capi_buf_t *params_ptr)
{
   capi_err_t result = CAPI_EOK;

   if (params_ptr->actual_data_len <
       sizeof(intf_extn_param_id_imcl_incoming_data_t) + sizeof(imcl_tdi_set_cfg_header_t))
   {
      SPR_MSG(me_ptr->miid,
              DBG_ERROR_PRIO,
              "Invalid payload size for incoming imcl data %d",
              params_ptr->actual_data_len);
      return CAPI_ENEEDMORE;
   }

   intf_extn_param_id_imcl_incoming_data_t *payload_ptr =
      (intf_extn_param_id_imcl_incoming_data_t *)params_ptr->data_ptr;

   uint32_t         ctrl_port_id  = payload_ptr->port_id;
   spr_ctrl_port_t *ctrl_port_ptr = spr_get_ctrl_port_instance(me_ptr, ctrl_port_id);

   if (!ctrl_port_ptr)
   {
      SPR_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Ctrl port 0x%lx non existent", ctrl_port_id);
      return CAPI_EBADPARAM;
   }

   if (0 == ctrl_port_ptr->is_mapped_to_output_port)
   {
      SPR_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Ctrl port 0x%lx mapping not found.", ctrl_port_id);
      return CAPI_EFAILED;
   }

   imcl_tdi_set_cfg_header_t *tdi_cfg_hdr_ptr = (imcl_tdi_set_cfg_header_t *)(payload_ptr + 1);

   switch (tdi_cfg_hdr_ptr->param_id)
   {
      case IMCL_PARAM_ID_TIMER_DRIFT_INFO:
      {
         if (sizeof(param_id_imcl_timer_drift_info) > tdi_cfg_hdr_ptr->param_size)
         {
            SPR_MSG(me_ptr->miid,
                    DBG_ERROR_PRIO,
                    "Invalid payload size %lu for timer drift. Required %lu",
                    tdi_cfg_hdr_ptr->param_size,
                    sizeof(param_id_imcl_timer_drift_info));
            return CAPI_EBADPARAM;
         }

         param_id_imcl_timer_drift_info *drift_info_ptr = (param_id_imcl_timer_drift_info *)(tdi_cfg_hdr_ptr + 1);
         ctrl_port_ptr->timer_drift_info_hdl_ptr        = (drift_info_ptr->handle_ptr);

#ifdef DEBUG_SPR_MODULE
         SPR_MSG(me_ptr->miid,
                 DBG_HIGH_PRIO,
                 "IMCL port 0x%x incoming data drift handle = 0x%x",
                 ctrl_port_id,
                 ctrl_port_ptr->timer_drift_info_hdl_ptr);
#endif

         capi_spr_send_output_drift_info(me_ptr);

         break;
      }
      case IMCL_PARAM_ID_TIMER_DRIFT_RESYNC:
      {
         capi_spr_resync_drift(me_ptr, ctrl_port_ptr);
#ifdef DEBUG_SPR_MODULE
         SPR_MSG(me_ptr->miid, DBG_HIGH_PRIO, "IMCL port 0x%x drift resynced..", ctrl_port_id);
#endif
         break;
      }
      default:
      {
         SPR_MSG(me_ptr->miid,
                 DBG_ERROR_PRIO,
                 "IMCL port incoming data, unsupported param_id 0x%X. port_id = 0x%X",
                 tdi_cfg_hdr_ptr->param_id,
                 ctrl_port_id);

         // TODO: send error or ignore?

         break;
      }
   }

   return result;
}

capi_err_t capi_spr_update_ctrl_data_port_map(capi_spr_t *me_ptr, capi_buf_t *params_ptr)
{
   capi_err_t result = CAPI_EOK;

   param_id_spr_ctrl_to_data_port_map_t *cfg_ptr = (param_id_spr_ctrl_to_data_port_map_t *)params_ptr->data_ptr;

   spr_port_binding_t *ctrl_to_outport_map = (spr_port_binding_t *)(cfg_ptr + 1);

   for (uint32_t i = 0; i < cfg_ptr->num_ctrl_ports; i++)
   {
      // Get the output ports arr_index.
      uint32_t op_arr_index = spr_get_arr_index_from_port_id(me_ptr, ctrl_to_outport_map[i].output_port_id);
      if (IS_INVALID_PORT_INDEX(op_arr_index))
      {
         SPR_MSG(me_ptr->miid,
                 DBG_ERROR_PRIO,
                 "Output port id 0x%lx is not found for mapping to control port 0x%x",
                 ctrl_to_outport_map[i].output_port_id,
                 ctrl_to_outport_map[i].control_port_id);

         result |= CAPI_EBADPARAM;
         continue;
      }

      if (me_ptr->out_port_info_arr[op_arr_index].ctrl_port_id &&
          (me_ptr->out_port_info_arr[op_arr_index].ctrl_port_id != ctrl_to_outport_map[i].control_port_id))
      {
         SPR_MSG(me_ptr->miid,
                 DBG_ERROR_PRIO,
                 "Ctrl port_id %lu to output port id %lu mapping is already set.",
                 me_ptr->out_port_info_arr[op_arr_index].ctrl_port_id,
                 ctrl_to_outport_map[i].output_port_id);
         result |= CAPI_EBADPARAM;
         continue;
      }

      /* In the case where output port is opened but control port is not yet opened, and we get the mapping then we
       * should still store the mapped control port id into the output port structure. once control port is connected
       * is_mapped_to_output_port will be modified */
      me_ptr->out_port_info_arr[op_arr_index].ctrl_port_id = ctrl_to_outport_map[i].control_port_id;

      spr_ctrl_port_t *ctrl_port_inst_ptr = spr_get_ctrl_port_instance(me_ptr, ctrl_to_outport_map[i].control_port_id);

      if (!ctrl_port_inst_ptr)
      {
         SPR_MSG(me_ptr->miid,
                 DBG_MED_PRIO,
                 "WARNING. Attempting to map dangling ctrl port_id %lu to output port id %lu",
                 ctrl_to_outport_map[i].control_port_id,
                 ctrl_to_outport_map[i].output_port_id);
         continue;
      }

      ctrl_port_inst_ptr->is_mapped_to_output_port = 1;

      SPR_MSG(me_ptr->miid,
              DBG_HIGH_PRIO,
              "Mapped ctrl port_id 0x%lx to output port id 0x%lx",
              ctrl_to_outport_map[i].control_port_id,
              ctrl_to_outport_map[i].output_port_id);
   }

   return result;
}

capi_err_t capi_spr_imcl_port_operation(capi_spr_t *me_ptr, capi_buf_t *params_ptr)
{
   capi_err_t result = CAPI_EOK;

   intf_extn_param_id_imcl_port_operation_t *port_op_ptr =
      (intf_extn_param_id_imcl_port_operation_t *)(params_ptr->data_ptr);

   if (!port_op_ptr->op_payload.data_ptr)
   {
      SPR_MSG(me_ptr->miid, DBG_ERROR_PRIO, "NULL payload for ctrl port operation opcode [0x%lX]", port_op_ptr->opcode);
      return CAPI_EBADPARAM;
   }

   switch (port_op_ptr->opcode)
   {
      case INTF_EXTN_IMCL_PORT_OPEN:
      {
         intf_extn_imcl_port_open_t *port_open_ptr = (intf_extn_imcl_port_open_t *)port_op_ptr->op_payload.data_ptr;
         uint32_t                    num_ports     = port_open_ptr->num_ports;

         // Check for max number of control ports
         if (num_ports > me_ptr->max_ctrl_ports)
         {
            SPR_MSG(me_ptr->miid,
                    DBG_ERROR_PRIO,
                    "SPR only supports a max of %lu control ports. "
                    "Trying to open %lu",
                    me_ptr->max_ctrl_ports,
                    num_ports);
            return CAPI_EBADPARAM;
         }

         // Validate the size
         uint32_t valid_size =
            sizeof(intf_extn_imcl_port_open_t) + (num_ports * sizeof(intf_extn_imcl_id_intent_map_t));

         if (port_op_ptr->op_payload.actual_data_len < valid_size)
         {
            SPR_MSG(me_ptr->miid,
                    DBG_ERROR_PRIO,
                    "Invalid payload size for ctrl port open. Received size %d Expected size %d",
                    params_ptr->actual_data_len,
                    valid_size);
            return CAPI_ENEEDMORE;
         }

         for (uint32_t iter = 0; iter < num_ports; iter++)
         {
            // Validate the size
            valid_size += (port_open_ptr->intent_map[iter].num_intents * sizeof(uint32_t));

            // SPR always expects just one intent per ctrl port
            if ((port_open_ptr->intent_map[iter].num_intents != SPR_MAX_INTENTS_PER_CTRL_PORT) ||
                (port_op_ptr->op_payload.actual_data_len < valid_size))
            {
               SPR_MSG(me_ptr->miid,
                       DBG_ERROR_PRIO,
                       "Number of supported intents per "
                       "ctrl port is only 1;"
                       "Invalid payload size for ctrl port OPEN %d",
                       params_ptr->actual_data_len);
               return CAPI_ENEEDMORE;
            }

            for (uint32_t i = 0; i < port_open_ptr->intent_map[iter].num_intents; i++)
            {
               if (INTENT_ID_TIMER_DRIFT_INFO != port_open_ptr->intent_map[iter].intent_arr[i])
               {
                  SPR_MSG(me_ptr->miid,
                          DBG_ERROR_PRIO,
                          "Ctrl port 0x%08lX, unsupported intent 0x%lX",
                          port_open_ptr->intent_map[iter].port_id,
                          port_open_ptr->intent_map[iter].intent_arr[i]);

                  return CAPI_EFAILED;
               }
            }

            uint32_t ctrl_port_id = port_open_ptr->intent_map[iter].port_id;
            bool_t   add_to_list  = FALSE;

            // Check if the associated control port instance already exists
            spr_ctrl_port_t *inst_ptr = spr_get_ctrl_port_instance(me_ptr, ctrl_port_id);

            // TODO: Do we need to destroy the control ports that we created as part of this payload?
            if (inst_ptr)
            {
               if (CTRL_PORT_OPEN == inst_ptr->state || CTRL_PORT_PEER_CONNECTED == inst_ptr->state)
               {
                  SPR_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Control port id: 0x%x is already opened. ", ctrl_port_id);
                  return CAPI_EFAILED;
               }
            }
            else
            {
               inst_ptr = (spr_ctrl_port_t *)posal_memory_malloc(sizeof(spr_ctrl_port_t), me_ptr->heap_id);

               if (!inst_ptr)
               {
                  SPR_MSG(me_ptr->miid,
                          DBG_ERROR_PRIO,
                          "Failed to create control port info instance for 0x%x",
                          ctrl_port_id);
                  return CAPI_ENOMEMORY;
               }

               memset(inst_ptr, 0, sizeof(spr_ctrl_port_t));
               add_to_list = TRUE;
            }

            inst_ptr->port_id                  = ctrl_port_id;
            inst_ptr->peer_module_instance_id  = port_open_ptr->intent_map[iter].peer_module_instance_id;
            inst_ptr->peer_port_id             = port_open_ptr->intent_map[iter].peer_port_id;
            inst_ptr->state                    = CTRL_PORT_OPEN;
            inst_ptr->num_intents              = port_open_ptr->intent_map[iter].num_intents;
            inst_ptr->is_mapped_to_output_port = capi_spr_is_ctrl_port_mapped_to_output_port(me_ptr, ctrl_port_id);

            for (uint32_t i = 0; i < port_open_ptr->intent_map[iter].num_intents; i++)
            {
               inst_ptr->intent_list_arr[i] = port_open_ptr->intent_map[iter].intent_arr[i];
#ifdef DEBUG_SPR_MODULE
               SPR_MSG(me_ptr->miid,
                       DBG_HIGH_PRIO,
                       "Ctrl port 0x%08lX, intent 0x%lX opened",
                       ctrl_port_id,
                       port_open_ptr->intent_map[iter].intent_arr[i]);
#endif
            }

            if (add_to_list)
            {
               result |= spf_list_insert_tail((spf_list_node_t **)&me_ptr->ctrl_port_list_ptr,
                                              (void *)inst_ptr,
                                              me_ptr->heap_id,
                                              TRUE /* use pool*/);
            }
         }

         break;
      }
      case INTF_EXTN_IMCL_PORT_CLOSE:
      {

         intf_extn_imcl_port_close_t *port_close_ptr = (intf_extn_imcl_port_close_t *)port_op_ptr->op_payload.data_ptr;
         uint32_t                     num_ports      = port_close_ptr->num_ports;

         if (num_ports > me_ptr->max_ctrl_ports)
         {
            SPR_MSG(me_ptr->miid,
                    DBG_ERROR_PRIO,
                    "module only supports a max of %lu control "
                    "ports. Trying to close %lu",
                    me_ptr->max_ctrl_ports,
                    num_ports);
            return CAPI_EBADPARAM;
         }

         uint32_t valid_size = sizeof(intf_extn_imcl_port_close_t) + (num_ports * sizeof(uint32_t));
         if (port_op_ptr->op_payload.actual_data_len < valid_size)
         {
            SPR_MSG(me_ptr->miid,
                    DBG_ERROR_PRIO,
                    "Invalid payload size for ctrl port CLOSE %d",
                    params_ptr->actual_data_len);
            return CAPI_ENEEDMORE;
         }

         // for each port id in the list that follows.
         for (uint32_t iter = 0; iter < num_ports; iter++)
         {
            uint32_t ctrl_port_id = port_close_ptr->port_id_arr[iter];

            spr_ctrl_port_list_t *imcl_port_ptr = spr_get_ctrl_port_list_node(me_ptr, ctrl_port_id);

            if (!imcl_port_ptr)
            {
               // TODO: check if error handling is applicable
               SPR_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Ctrl port 0x%lx not found.", ctrl_port_id);
               continue;
            }

            spf_list_delete_node_and_free_obj((spf_list_node_t **)&imcl_port_ptr,
                                              (spf_list_node_t **)&me_ptr->ctrl_port_list_ptr,
                                              TRUE /* use pool*/);

#ifdef DEBUG_SPR_MODULE
            AR_MSG(DBG_LOW_PRIO, "capi_spr: Ctrl port 0x%08lX closed", ctrl_port_id);
#endif
         }

         break;
      } // end of INTF_EXTN_IMCL_PORT_CLOSE
      case INTF_EXTN_IMCL_PORT_PEER_CONNECTED:
      {
         intf_extn_imcl_port_start_t *port_start_ptr = (intf_extn_imcl_port_start_t *)port_op_ptr->op_payload.data_ptr;

         // Size Validation
         uint32_t num_ports = port_start_ptr->num_ports;

         if (num_ports > me_ptr->max_ctrl_ports)
         {
            SPR_MSG(me_ptr->miid,
                    DBG_ERROR_PRIO,
                    "module only supports a max of 0x%lu control "
                    "ports. Trying to start %lu",
                    me_ptr->max_ctrl_ports,
                    num_ports);
            return CAPI_EBADPARAM;
         }

         uint32_t valid_size = sizeof(intf_extn_imcl_port_start_t) + (num_ports * sizeof(uint32_t));
         if (port_op_ptr->op_payload.actual_data_len < valid_size)
         {
            SPR_MSG(me_ptr->miid,
                    DBG_ERROR_PRIO,
                    "Invalid payload size for ctrl port Start %d",
                    params_ptr->actual_data_len);
            return CAPI_ENEEDMORE;
         }

         // for each port id in the list that follows...
         for (uint32_t iter = 0; iter < num_ports; iter++)
         {
            uint32_t         ctrl_port_id  = port_start_ptr->port_id_arr[iter];
            spr_ctrl_port_t *imcl_inst_ptr = spr_get_ctrl_port_instance(me_ptr, ctrl_port_id);

            if (!imcl_inst_ptr)
            {
               SPR_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Ctrl port 0x%lx not found.", ctrl_port_id);
               result |= CAPI_EBADPARAM;
               continue;
            }

            imcl_inst_ptr->state = CTRL_PORT_PEER_CONNECTED;

#ifdef DEBUG_SPR_MODULE
            SPR_MSG(me_ptr->miid, DBG_HIGH_PRIO, "ctrl port_id 0x%lx received start. ", ctrl_port_id);
#endif
         }

         capi_spr_send_output_drift_info(me_ptr);

         break;
      }
      case INTF_EXTN_IMCL_PORT_PEER_DISCONNECTED:
      {
         intf_extn_imcl_port_stop_t *port_stop_ptr = (intf_extn_imcl_port_stop_t *)port_op_ptr->op_payload.data_ptr;

         // Size Validation
         uint32_t num_ports = port_stop_ptr->num_ports;

         if (num_ports > me_ptr->max_ctrl_ports)
         {
            SPR_MSG(me_ptr->miid,
                    DBG_ERROR_PRIO,
                    "module only supports a max of 0x%lu control "
                    "ports. Trying to stop %lu",
                    me_ptr->max_ctrl_ports,
                    num_ports);
            return CAPI_EBADPARAM;
         }

         uint32_t valid_size = sizeof(intf_extn_imcl_port_stop_t) + (num_ports * sizeof(uint32_t));
         if (port_op_ptr->op_payload.actual_data_len < valid_size)
         {
            SPR_MSG(me_ptr->miid,
                    DBG_ERROR_PRIO,
                    "Invalid payload size for ctrl port Start %d",
                    params_ptr->actual_data_len);
            return CAPI_ENEEDMORE;
         }

         // for each port id in the list that follows...
         for (uint32_t iter = 0; iter < num_ports; iter++)
         {
            uint32_t         ctrl_port_id  = port_stop_ptr->port_id_arr[iter];
            spr_ctrl_port_t *imcl_inst_ptr = spr_get_ctrl_port_instance(me_ptr, ctrl_port_id);

            if (!imcl_inst_ptr)
            {
               SPR_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Ctrl port 0x%lx not found.", ctrl_port_id);
               result |= CAPI_EBADPARAM;
               continue;
            }

            imcl_inst_ptr->state = CTRL_PORT_PEER_DISCONNECTED;

#ifdef DEBUG_SPR_MODULE
            SPR_MSG(me_ptr->miid, DBG_HIGH_PRIO, "ctrl port_id 0x%lx received stop. ", ctrl_port_id);
#endif
         }

         break;
      }
      default:
      {
         SPR_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Received unsupported ctrl port opcode 0x%lx", port_op_ptr->opcode);
         return CAPI_EUNSUPPORTED;
      }
   }

   return result;
}

static spr_ctrl_port_list_t *spr_get_ctrl_port_list_node(capi_spr_t *me_ptr, uint32_t ctrl_port_id)
{
   spr_ctrl_port_list_t *ret_node_ptr = NULL;

   spr_ctrl_port_list_t *list_ptr = me_ptr->ctrl_port_list_ptr;

   while (list_ptr)
   {
      spr_ctrl_port_t *node_ptr = list_ptr->port_info_ptr;

      if (node_ptr->port_id == ctrl_port_id)
      {
         ret_node_ptr = list_ptr;
         break;
      }

      list_ptr = list_ptr->next_ptr;
   }

   return ret_node_ptr;
}

void capi_spr_send_output_drift_info(capi_spr_t *me_ptr)
{
   //capi_err_t capi_result = CAPI_EOK;

   spr_ctrl_port_list_t *ctrl_port_list_ptr  = me_ptr->ctrl_port_list_ptr;
   uint32_t              num_connected_ports = 0;

   while (ctrl_port_list_ptr)
   {
      spr_ctrl_port_t *ctrl_port_ptr = ctrl_port_list_ptr->port_info_ptr;

      SPR_MSG(me_ptr->miid,
              DBG_HIGH_PRIO,
              "ctrl port id = 0x%x, is_mapped_to_output_port(T/F) = %d, state = %d",
              ctrl_port_ptr->port_id,
              ctrl_port_ptr->is_mapped_to_output_port,
              ctrl_port_ptr->state);

      if (CTRL_PORT_PEER_CONNECTED == ctrl_port_ptr->state && 0 == ctrl_port_ptr->is_mapped_to_output_port)
      {
         num_connected_ports++;
         spr_send_drift_handle_per_port(me_ptr, ctrl_port_ptr);
      }

      ctrl_port_list_ptr = ctrl_port_list_ptr->next_ptr;
   }

   if (me_ptr->ctrl_port_list_ptr && !num_connected_ports)
   {
      SPR_MSG(me_ptr->miid, DBG_HIGH_PRIO, "Did not send drift to any port. Most likely not connected");
   }

}

static capi_err_t spr_send_drift_handle_per_port(capi_spr_t *me_ptr, spr_ctrl_port_t *ctrl_port_ptr)
{
   capi_err_t                      capi_result  = CAPI_EOK;
   uint32_t                        ctrl_port_id = ctrl_port_ptr->port_id;
   uint32_t                        req_buf_size;
   capi_buf_t                      one_time_buf;
   imcl_tdi_set_cfg_header_t *     imcl_hdr_ptr;
   param_id_imcl_timer_drift_info *drift_info_ptr;

   // Calculate the required payload size
   req_buf_size = sizeof(imcl_tdi_set_cfg_header_t) + sizeof(param_id_imcl_timer_drift_info);

   one_time_buf.actual_data_len = req_buf_size;

   // Get one time buffer from the framework
   if (CAPI_EOK !=
       (capi_result =
           capi_cmn_imcl_get_one_time_buf(&me_ptr->event_cb_info, ctrl_port_id, req_buf_size, &one_time_buf)))
   {
      SPR_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Failed to get IMCL buffer for port 0x%x", ctrl_port_id);
      return capi_result;
   }

   // Populate drift info in the buffer
   imcl_hdr_ptr = (imcl_tdi_set_cfg_header_t *)one_time_buf.data_ptr;

   imcl_hdr_ptr->param_id   = IMCL_PARAM_ID_TIMER_DRIFT_INFO;
   imcl_hdr_ptr->param_size = sizeof(imcl_tdi_set_cfg_header_t);

   drift_info_ptr = (param_id_imcl_timer_drift_info *)(one_time_buf.data_ptr + sizeof(imcl_tdi_set_cfg_header_t));

   // Update drift handle
   drift_info_ptr->handle_ptr = &me_ptr->spr_out_drift_info.drift_info_hdl;

   imcl_outgoing_data_flag_t flags;
   flags.should_send = TRUE;
   flags.is_trigger  = FALSE;

   capi_result = capi_cmn_imcl_send_to_peer(&me_ptr->event_cb_info, &one_time_buf, ctrl_port_id, flags);

#ifdef DEBUG_SPR_MODULE
   SPR_MSG(me_ptr->miid,
           DBG_LOW_PRIO,
           "Sent the spr drift pointer via ctrl_port 0x%lx with result 0x%x",
           ctrl_port_id,
           capi_result);
#endif

   return capi_result;
}
