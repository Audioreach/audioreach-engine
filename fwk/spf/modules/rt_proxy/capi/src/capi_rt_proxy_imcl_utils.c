/**
 *   \file capi_rt_proxy_imcl_utils.c
 *   \brief
 *        This file contains CAPI implementation of RT Proxy module.
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "capi_rt_proxy_i.h"

/*==============================================================================
   Local Function forward declaration
==============================================================================*/

/*==============================================================================
   Public Function Implementation
==============================================================================*/

/* Create mutex and setting drift function */
capi_err_t capi_rt_proxy_init_out_drift_info(rt_proxy_drift_info_t *     drift_info_ptr,
                                             imcl_tdi_get_acc_drift_fn_t get_drift_fn_ptr)
{
   capi_err_t result = CAPI_EOK;

   if (!drift_info_ptr)
   {
      return CAPI_EBADPARAM;
   }

   /** Clear the drift info structure  */
   memset(drift_info_ptr, 0, sizeof(rt_proxy_drift_info_t));

   /* Create mutex for the drift info shared with rate matching modules */
   posal_mutex_create(&drift_info_ptr->drift_info_mutex, POSAL_HEAP_DEFAULT);

   /**Set the function pointer for querying the drift */
   drift_info_ptr->drift_info_hdl.get_drift_fn_ptr = get_drift_fn_ptr;

   return result;
}

/* Destroy mutex */
capi_err_t capi_rt_proxy_deinit_out_drift_info(rt_proxy_drift_info_t *drift_info_ptr)
{
   capi_err_t result = CAPI_EOK;

   if (!drift_info_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "rt_proxy: Drift info pointer is NULL");
      return CAPI_EBADPARAM;
   }

   /** Destroy mutex */
   posal_mutex_destroy(&drift_info_ptr->drift_info_mutex);

   return result;
}

/* Client uses this to read drift from the BFBDE */
ar_result_t rt_proxy_imcl_read_acc_out_drift(imcl_tdi_hdl_t *      drift_info_hdl_ptr,
                                             imcl_tdi_acc_drift_t *acc_drift_out_ptr)
{
   ar_result_t result = AR_EOK;

#ifdef DEBUG_RT_PROXY_DRIVER
   AR_MSG(DBG_HIGH_PRIO, "rt_proxy: Reading drift from rt_proxy");
#endif

   if (!drift_info_hdl_ptr || !acc_drift_out_ptr)
   {
      return AR_EFAILED;
   }

   rt_proxy_drift_info_t *shared_drift_ptr = (rt_proxy_drift_info_t *)drift_info_hdl_ptr;

   posal_mutex_lock(shared_drift_ptr->drift_info_mutex);

   /** Copy the accumulated drift info */
   memscpy(acc_drift_out_ptr, sizeof(imcl_tdi_acc_drift_t), &shared_drift_ptr->acc_drift, sizeof(imcl_tdi_acc_drift_t));

   posal_mutex_unlock(shared_drift_ptr->drift_info_mutex);

   return result;
}

/* Client uses this to read drift from the BFBDE */
ar_result_t rt_proxy_update_accumulated_drift(rt_proxy_drift_info_t *shared_drift_ptr,
                                              int64_t                current_drift_adjustment,
                                              uint64_t               timestamp)
{
   ar_result_t result = AR_EOK;

#ifdef DEBUG_RT_PROXY_DRIVER
   AR_MSG(DBG_HIGH_PRIO, "rt_proxy: Reading drift from rt_proxy");
#endif

   if (!shared_drift_ptr)
   {
      return AR_EFAILED;
   }

   posal_mutex_lock(shared_drift_ptr->drift_info_mutex);

   shared_drift_ptr->acc_drift.acc_drift_us += current_drift_adjustment;

#ifdef DEBUG_RT_PROXY_DRIVER_DRIFT_ADJ
   int32_t new_acc_drift = shared_drift_ptr->acc_drift.acc_drift_us;
#endif

   posal_mutex_unlock(shared_drift_ptr->drift_info_mutex);

#ifdef DEBUG_RT_PROXY_DRIVER_DRIFT_ADJ
   AR_MSG(DBG_LOW_PRIO,
          "rt_proxy: inst_drift_adj= %ld, acc_drift_us= %ld",
          (int32_t)current_drift_adjustment,
          new_acc_drift);
#endif

   return result;
}

capi_err_t capi_rt_proxy_send_drift_info_hdl_to_rat(capi_rt_proxy_t *me_ptr)
{
   capi_err_t                      result = CAPI_EOK;
   capi_buf_t                      one_time_buf;
   imcl_tdi_set_cfg_header_t *     imcl_hdr_ptr;
   param_id_imcl_timer_drift_info *drift_info_ptr;
   uint32_t req_buf_size = sizeof(param_id_imcl_timer_drift_info) + sizeof(imcl_tdi_set_cfg_header_t);

   /** Get the one time buffer from the framework */
   if (CAPI_EOK != (result = capi_cmn_imcl_get_one_time_buf(&me_ptr->event_cb_info,
                                                            me_ptr->ctrl_port_info.port_id,
                                                            req_buf_size,
                                                            &one_time_buf)))
   {
      return result;
   }

   /** Populate the received buffer  */
   imcl_hdr_ptr = (imcl_tdi_set_cfg_header_t *)one_time_buf.data_ptr;

   imcl_hdr_ptr->param_id   = IMCL_PARAM_ID_TIMER_DRIFT_INFO;
   imcl_hdr_ptr->param_size = sizeof(param_id_imcl_timer_drift_info);

   drift_info_ptr = (param_id_imcl_timer_drift_info *)(one_time_buf.data_ptr + sizeof(imcl_tdi_set_cfg_header_t));

   /** Update the drift info handle to be shared */
   drift_info_ptr->handle_ptr = (imcl_tdi_hdl_t *)&me_ptr->drift_info;

   imcl_outgoing_data_flag_t flags;
   flags.should_send = TRUE;
   flags.is_trigger  = TRUE;

   /** Send to peer */
   result = capi_cmn_imcl_send_to_peer(&me_ptr->event_cb_info, &one_time_buf, me_ptr->ctrl_port_info.port_id, flags);

   return result;
}

capi_err_t capi_rt_proxy_handle_imcl_port_operation(capi_rt_proxy_t *me_ptr, capi_buf_t *params_ptr)
{
   capi_err_t result = CAPI_EOK;

   if (NULL == params_ptr->data_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "rt_proxy: Set param id 0x%lx, received null buffer",
             INTF_EXTN_PARAM_ID_IMCL_PORT_OPERATION);
      result = CAPI_EBADPARAM;
      return result;
   }
   if (params_ptr->actual_data_len < sizeof(intf_extn_param_id_imcl_port_operation_t))
   {
      AR_MSG(DBG_ERROR_PRIO, "rt_proxy: Invalid payload size for ctrl port operation %d", params_ptr->actual_data_len);
      return CAPI_ENEEDMORE;
   }
   intf_extn_param_id_imcl_port_operation_t *port_op_ptr =
      (intf_extn_param_id_imcl_port_operation_t *)(params_ptr->data_ptr);
   switch (port_op_ptr->opcode)
   {
      case INTF_EXTN_IMCL_PORT_OPEN:
      {
         if (port_op_ptr->op_payload.data_ptr)
         {
            intf_extn_imcl_port_open_t *port_open_ptr = (intf_extn_imcl_port_open_t *)port_op_ptr->op_payload.data_ptr;
            /*Size Validation*/
            uint32_t num_ports = port_open_ptr->num_ports;

            // Number of control ports can be more than the output ports
            if (num_ports > RT_PROXY_MAX_CTRL_PORTS)
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "rt_proxy: DAM only supports a max of %lu control ports. Trying to open %lu",
                      RT_PROXY_MAX_CTRL_PORTS,
                      num_ports);
               return CAPI_EBADPARAM;
            }

            uint32_t valid_size =
               sizeof(intf_extn_imcl_port_open_t) + (num_ports * sizeof(intf_extn_imcl_id_intent_map_t));
            if (port_op_ptr->op_payload.actual_data_len < valid_size)
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "rt_proxy: Invalid payload size for ctrl port OPEN %d",
                      params_ptr->actual_data_len);
               return CAPI_ENEEDMORE;
            }

            // Iterate through each port
            for (uint32_t iter = 0; iter < num_ports; iter++)
            {
               /*Size Validation*/
               valid_size += (port_open_ptr->intent_map[iter].num_intents * sizeof(uint32_t));

               // Dam always expects just one intent per ctrl port
               if ((port_open_ptr->intent_map[iter].num_intents > RT_PROXY_MAX_INTENTS_PER_CTRL_PORT) ||
                   (port_op_ptr->op_payload.actual_data_len < valid_size))
               {
                  AR_MSG(DBG_ERROR_PRIO,
                         "rt_proxy: Note: Always expects just one intent per ctrl port;"
                         "Invalid payload size for ctrl port OPEN %d",
                         params_ptr->actual_data_len);
                  return CAPI_ENEEDMORE;
               }

               uint32_t ctrl_port_id = port_open_ptr->intent_map[iter].port_id;

               // there is only one control port attached to rt_proxy module.
               // if the control port is already open return failure.
               if (me_ptr->ctrl_port_info.state == CTRL_PORT_OPEN)
               {
                  AR_MSG(DBG_ERROR_PRIO, "rt_proxy: Control port id: 0x%x is already opened. ", ctrl_port_id);
                  return CAPI_EFAILED;
               }

               // Check the intent list for each port.
               for (uint32_t intent_iter = 0; intent_iter < port_open_ptr->intent_map[iter].num_intents; intent_iter++)
               {
                  if (INTENT_ID_TIMER_DRIFT_INFO != port_open_ptr->intent_map[iter].intent_arr[intent_iter])
                  {
                     AR_MSG(DBG_MED_PRIO,
                            "rt_proxy: Opening Control port id: 0x%x with unsupported intent ID: 0x%x ",
                            ctrl_port_id,
                            port_open_ptr->intent_map[iter].intent_arr[intent_iter]);
                     return CAPI_EFAILED;
                  }
                  me_ptr->ctrl_port_info.intent_list_arr[intent_iter] = INTENT_ID_TIMER_DRIFT_INFO;
               }

               me_ptr->ctrl_port_info.num_intents = port_open_ptr->intent_map[iter].num_intents;
               me_ptr->ctrl_port_info.state       = CTRL_PORT_OPEN;
               me_ptr->ctrl_port_info.port_id     = ctrl_port_id;

               AR_MSG(DBG_MED_PRIO,
                      "rt_proxy: Opening Control port id: 0x%x with intent ID: 0x%x ",
                      ctrl_port_id,
                      port_open_ptr->intent_map[iter].intent_arr[0]);
            }
         }
         else
         {
            AR_MSG(DBG_ERROR_PRIO, "rt_proxy: Ctrl port open expects a payload. Failing.");
            return CAPI_EFAILED;
         }
         break;
      }
      case INTF_EXTN_IMCL_PORT_PEER_CONNECTED:
      {
         if (port_op_ptr->op_payload.data_ptr)
         {
            intf_extn_imcl_port_start_t *port_start_ptr =
               (intf_extn_imcl_port_start_t *)port_op_ptr->op_payload.data_ptr;
            /*Size Validation*/
            uint32_t num_ports = port_start_ptr->num_ports;

            if (num_ports > RT_PROXY_MAX_CTRL_PORTS)
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "rt_proxy: module only supports a max of %lu control ports. Trying to peer connect %lu",
                      RT_PROXY_MAX_CTRL_PORTS,
                      num_ports);
               return CAPI_EBADPARAM;
            }

            uint32_t valid_size = sizeof(intf_extn_imcl_port_start_t) + (num_ports * sizeof(uint32_t));
            if (port_op_ptr->op_payload.actual_data_len < valid_size)
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "rt_proxy: Invalid payload size for ctrl port peer connect %d",
                      params_ptr->actual_data_len);
               return CAPI_ENEEDMORE;
            }

            // for each port id in the list that follows...
            for (uint32_t iter = 0; iter < num_ports; iter++)
            {
               if ((port_start_ptr->port_id_arr[iter] == me_ptr->ctrl_port_info.port_id) &&
                   (CTRL_PORT_OPEN == me_ptr->ctrl_port_info.state))
               {
                  me_ptr->ctrl_port_info.state = CTRL_PORT_PEER_CONNECTED;

                  result |= capi_rt_proxy_send_drift_info_hdl_to_rat(me_ptr);
                  if (CAPI_EOK != result)
                  {
                     AR_MSG(DBG_ERROR_PRIO,
                            "rt_proxy: Failed to send drift info to RAT module. ctrl port id=0x%lx ",
                            port_start_ptr->port_id_arr[iter]);
                  }
               }
               else
               {
                  AR_MSG(DBG_ERROR_PRIO,
                         "rt_proxy: Failed to set peer connect ctrl port_id 0x%lx  ",
                         port_start_ptr->port_id_arr[iter]);
               }

               AR_MSG(DBG_HIGH_PRIO,
                      "rt_proxy: ctrl port_id 0x%lx received start. ",
                      port_start_ptr->port_id_arr[iter]);
            }
         }
         else
         {
            AR_MSG(DBG_ERROR_PRIO, "rt_proxy: Ctrl port peer connect expects a payload. Failing.");
            return CAPI_EFAILED;
         }
         break;
      }
      case INTF_EXTN_IMCL_PORT_PEER_DISCONNECTED:
      {
         if (port_op_ptr->op_payload.data_ptr)
         {
            intf_extn_imcl_port_stop_t *port_stop_ptr = (intf_extn_imcl_port_stop_t *)port_op_ptr->op_payload.data_ptr;
            /*Size Validation*/
            uint32_t num_ports = port_stop_ptr->num_ports;

            if (num_ports > RT_PROXY_MAX_CTRL_PORTS)
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "rt_proxy: module only supports a max of %lu control ports. Trying to STOP %lu",
                      RT_PROXY_MAX_CTRL_PORTS,
                      num_ports);
               return CAPI_EBADPARAM;
            }

            uint32_t valid_size = sizeof(intf_extn_imcl_port_stop_t) + (num_ports * sizeof(uint32_t));
            if (port_op_ptr->op_payload.actual_data_len < valid_size)
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "capi_dam: Invalid payload size for ctrl port peer disconnect %d",
                      params_ptr->actual_data_len);
               return CAPI_ENEEDMORE;
            }

            // for each port id in the list that follows...
            for (uint32_t iter = 0; iter < num_ports; iter++)
            {
               if ((port_stop_ptr->port_id_arr[iter] == me_ptr->ctrl_port_info.port_id) &&
                   (CTRL_PORT_PEER_CONNECTED == me_ptr->ctrl_port_info.state))
               {
                  me_ptr->ctrl_port_info.state = CTRL_PORT_PEER_DISCONNECTED;
               }
               else
               {
                  AR_MSG(DBG_ERROR_PRIO,
                         "rt_proxy: Failed to set peer disconnect ctrl port_id 0x%lx  ",
                         port_stop_ptr->port_id_arr[iter]);
               }

               AR_MSG(DBG_HIGH_PRIO,
                      "rt_proxy: ctrl port_id 0x%lx received peer disconnect. ",
                      port_stop_ptr->port_id_arr[iter]);
            }
         }
         else
         {
            AR_MSG(DBG_ERROR_PRIO, "rt_proxy: Ctrl port peer disconnect expects a payload. Failing.");
            return CAPI_EFAILED;
         }
         break;
      }
      case INTF_EXTN_IMCL_PORT_CLOSE:
      {
         if (port_op_ptr->op_payload.data_ptr)
         {
            intf_extn_imcl_port_close_t *port_close_ptr =
               (intf_extn_imcl_port_close_t *)port_op_ptr->op_payload.data_ptr;
            /*Size Validation*/
            uint32_t num_ports = port_close_ptr->num_ports;

            if (num_ports > RT_PROXY_MAX_CTRL_PORTS)
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "rt_proxy: Supports max of %lu control ports. Trying to close %lu ports",
                      RT_PROXY_MAX_CTRL_PORTS,
                      num_ports);
               return CAPI_EBADPARAM;
            }

            uint32_t valid_size = sizeof(intf_extn_imcl_port_close_t) + (num_ports * sizeof(uint32_t));
            if (port_op_ptr->op_payload.actual_data_len < valid_size)
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "rt_proxy: Invalid payload size for ctrl port CLOSE %d",
                      params_ptr->actual_data_len);
               return CAPI_ENEEDMORE;
            }

            // Iterate through each port and close the port.
            for (uint32_t iter = 0; iter < num_ports; iter++)
            {
               uint32_t ctrl_port_id = port_close_ptr->port_id_arr[iter];

               if (me_ptr->ctrl_port_info.state == CTRL_PORT_CLOSE)
               {
                  AR_MSG(DBG_ERROR_PRIO, "rt_proxy: Control port id: 0x%x is already closed. ", ctrl_port_id);
                  return CAPI_EFAILED;
               }
               else
               {
                  AR_MSG(DBG_HIGH_PRIO, "rt_proxy: Control port id: 0x%x is closed. ", ctrl_port_id);

                  memset(&me_ptr->ctrl_port_info, 0, sizeof(rt_proxy_ctrl_port_info_t));
               }
            }
         }
         else
         {
            AR_MSG(DBG_ERROR_PRIO, "rt_proxy: Ctrl port close expects a payload. Failing.");
            return CAPI_EFAILED;
         }
         break;
      }
      default:
      {
         AR_MSG(DBG_ERROR_PRIO, "rt_proxy: Received unsupported ctrl port opcode %lu", port_op_ptr->opcode);
         return CAPI_EUNSUPPORTED;
      }
   }

   return result;
}
