/**
 *   \file capi_duty_cycling_buf.c
 *   \brief
 *        This file contains CAPI implementation of duty cycling buffering module
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "duty_cycling_buf_module_api.h"
#include "capi_duty_cycling_buf.h"
#include "capi_duty_cycling_buf_utils.h"
#include "posal_memory.h"

/*==============================================================================
   Local Defines
==============================================================================*/
//#define BUF_DEBUG //

/*==============================================================================
   Static functions
==============================================================================*/

static void capi_duty_cycling_buf_init_events_config(capi_duty_cycling_buf_t *me_ptr);

static uint32_t capi_duty_cycling_buf_get_kpps(capi_duty_cycling_buf_t *me_ptr)
{
   return CAPI_DUTY_CYCLING_BUF_KPPS;
}

/**
 * Inits event configuration.
 */
void capi_duty_cycling_buf_init_events_config(capi_duty_cycling_buf_t *me_ptr)
{
   //Assume module is not enabled by default dont raise any event to container
   //From container perspective module is always enabled.Handle disable case internally in module
   me_ptr->events_config.enable      = FALSE;
   me_ptr->events_config.kpps        = CAPI_DUTY_CYCLING_BUF_KPPS;
   me_ptr->events_config.delay_in_us = 0;
   me_ptr->events_config.code_bw     = CAPI_DUTY_CYCLING_BUF_CODE_BW;
   me_ptr->events_config.data_bw     = CAPI_DUTY_CYCLING_BUF_DATA_BW;
}

/**
 * Function to raise various events.
 */
capi_err_t capi_duty_cycling_buf_raise_events(capi_duty_cycling_buf_t *me_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   if (NULL == me_ptr->event_cb_info.event_cb)
{
      AR_MSG(DBG_ERROR_PRIO, "Event callback is not set. Unable to raise events!");
      CAPI_SET_ERROR(capi_result, CAPI_EUNSUPPORTED);
      return capi_result;
}

   me_ptr->events_config.kpps = capi_duty_cycling_buf_get_kpps(me_ptr);

   capi_result |= capi_cmn_update_kpps_event(&me_ptr->event_cb_info, me_ptr->events_config.kpps);
   capi_result |= capi_cmn_update_bandwidth_event(&me_ptr->event_cb_info,
                                                  me_ptr->events_config.code_bw,
                                                  me_ptr->events_config.data_bw);
   capi_result |= capi_cmn_update_algo_delay_event(&me_ptr->event_cb_info, me_ptr->events_config.delay_in_us);
  
   return capi_result;
}

/*==============================================================================
   Local Function forward declaration
==============================================================================*/
capi_err_t capi_duty_cycling_buf_process_get_properties(capi_duty_cycling_buf_t *me_ptr, capi_proplist_t *proplist_ptr)
{
   capi_err_t capi_result = CAPI_EOK;

   if (NULL == proplist_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi duty_cycling_buf: Get property received null property array");
      return CAPI_EBADPARAM;
   }

   uint32_t fwk_extn_ids_arr[] = { FWK_EXTN_TRIGGER_POLICY, FWK_EXTN_CONTAINER_FRAME_DURATION };

   capi_basic_prop_t mod_prop;
   mod_prop.init_memory_req    = align_to_8_byte(sizeof(capi_duty_cycling_buf_t));
   mod_prop.stack_size         = DUTY_CYCLING_BUF_STACK_SIZE;
   mod_prop.num_fwk_extns      = sizeof(fwk_extn_ids_arr) / sizeof(fwk_extn_ids_arr[0]);
   mod_prop.fwk_extn_ids_arr   = fwk_extn_ids_arr;
   mod_prop.is_inplace         = FALSE;
   mod_prop.req_data_buffering = TRUE;
   mod_prop.max_metadata_size  = 0;

   capi_result = capi_cmn_get_basic_properties(proplist_ptr, &mod_prop);
   if (CAPI_EOK != capi_result)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "capi duty_cycling_buf: Get common basic properties failed with capi_result %lu",
             capi_result);
   }

   uint16_t     i;
   capi_prop_t *prop_ptr = proplist_ptr->prop_ptr;

   for (i = 0; i < proplist_ptr->props_num; i++)
   {
      capi_buf_t *payload_ptr = &prop_ptr[i].payload;

      switch (prop_ptr[i].id)
      {
         case CAPI_INIT_MEMORY_REQUIREMENT:
         case CAPI_STACK_SIZE:
         case CAPI_IS_INPLACE:
         case CAPI_REQUIRES_DATA_BUFFERING:
         case CAPI_MAX_METADATA_SIZE:
         case CAPI_NUM_NEEDED_FRAMEWORK_EXTENSIONS:
         case CAPI_NEEDED_FRAMEWORK_EXTENSIONS:
         case CAPI_OUTPUT_MEDIA_FORMAT_SIZE:
         {
            break;
         }
         case CAPI_OUTPUT_MEDIA_FORMAT_V2:
         {
            if (NULL == me_ptr)
            {
               AR_MSG(DBG_ERROR_PRIO, "pif is NULL for get OUTPUT MF");
               capi_result |= CAPI_EBADPARAM;
               break;
            }

            if (me_ptr->is_input_mf_received == FALSE)
            {

               AR_MSG(DBG_ERROR_PRIO, " cannot update OUTPUT MF without in MF");
               capi_result |= CAPI_EUNSUPPORTED;
               break;
            }

            // TOCheck if following can be skipped and directly raise based on input mf

            if (me_ptr->cir_buffer_raised_out_mf)
            {
               uint32_t ret_size =
                  sizeof(capi_set_get_media_format_t) + sizeof(capi_standard_data_format_v2_t) +
                  (me_ptr->out_media_fmt.format.num_channels * sizeof(capi_channel_type_t));
               /* Validate the MF payload */
               if (payload_ptr->max_data_len < sizeof(capi_media_fmt_v2_t))
               {
                  AR_MSG(DBG_ERROR_PRIO, "Invalid media format size %d", payload_ptr->max_data_len);
                  capi_result |= CAPI_ENEEDMORE;
                  break;
               }
               capi_media_fmt_v2_t *media_fmt_ptr = (capi_media_fmt_v2_t *)(payload_ptr->data_ptr);
               memscpy(media_fmt_ptr, ret_size, &me_ptr->out_media_fmt, ret_size);
               payload_ptr->actual_data_len = sizeof(capi_media_fmt_v2_t);
            }
            else
            {

               AR_MSG(DBG_ERROR_PRIO, " cannot update OUTPUT MF without circular buffer updating out MF");
               capi_result |= CAPI_EUNSUPPORTED;
            }
            break;
         }
         case CAPI_INTERFACE_EXTENSIONS:
         {
            if (payload_ptr->max_data_len >= sizeof(capi_interface_extns_list_t))
            {
               capi_interface_extns_list_t *intf_ext_list = (capi_interface_extns_list_t *)payload_ptr->data_ptr;
               if (payload_ptr->max_data_len < (sizeof(capi_interface_extns_list_t) +
                                                (intf_ext_list->num_extensions * sizeof(capi_interface_extn_desc_t))))
               {
                  AR_MSG(DBG_ERROR_PRIO, "CAPI_INTERFACE_EXTENSIONS invalid param size %lu", payload_ptr->max_data_len);
                  CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
               }
               else
               {
                  capi_interface_extn_desc_t *curr_intf_extn_desc_ptr =
                     (capi_interface_extn_desc_t *)(payload_ptr->data_ptr + sizeof(capi_interface_extns_list_t));

                  for (uint32_t i = 0; i < intf_ext_list->num_extensions; i++)
                  {
                     switch (curr_intf_extn_desc_ptr->id)
                     {
                        case INTF_EXTN_METADATA:
						case INTF_EXTN_DATA_PORT_OPERATION:
                        case INTF_EXTN_PROP_IS_RT_PORT_PROPERTY:
                        {
                           curr_intf_extn_desc_ptr->is_supported = TRUE;
                           break;
                        }
                        default:
                        {
                           curr_intf_extn_desc_ptr->is_supported = FALSE;
                           break;
                        }
                     }
                     AR_MSG(DBG_HIGH_PRIO,
                            "CAPI_INTERFACE_EXTENSIONS intf_ext = 0x%lx, is_supported = %d",
                            curr_intf_extn_desc_ptr->id,
                            (int)curr_intf_extn_desc_ptr->is_supported);
                     curr_intf_extn_desc_ptr++;
                  }
               }
            }
            else
            {
               AR_MSG(DBG_ERROR_PRIO, "CAPI_INTERFACE_EXTENSIONS Bad param size %lu", payload_ptr->max_data_len);
               CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
            }
            break;
         }
         case CAPI_PORT_DATA_THRESHOLD:
         {
            uint32_t threshold = 1;
            capi_result |= capi_cmn_handle_get_port_threshold(&prop_ptr[i], threshold);
            break;
         }
         default:
         {
            AR_MSG(DBG_HIGH_PRIO, "FAILED Get Property for 0x%x. Not supported.", prop_ptr[i].id);
            CAPI_SET_ERROR(capi_result, CAPI_EUNSUPPORTED);
         }
      }
   }

   return capi_result;
}

/*
  This function is used to query the static properties to create the CAPI.

  param[in] init_set_prop_ptr: Pointer to the initializing property list
  param[in, out] static_prop_ptr: Pointer to the static property list

  return: CAPI_EOK(0) on success else failure error code
 */
capi_err_t capi_duty_cycling_buf_get_static_properties(capi_proplist_t *init_set_prop_ptr,
                                                       capi_proplist_t *static_prop_ptr)
{
   capi_err_t result = CAPI_EOK;
   if (NULL == static_prop_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "received bad property pointer for get "
             "property, %p",
             static_prop_ptr);
      return (CAPI_EFAILED);
   }

   result = capi_duty_cycling_buf_process_get_properties((capi_duty_cycling_buf_t *)NULL, static_prop_ptr);

   return result;
}

/*
  This function is used init the CAPI lib.

  param[in] capi_ptr: Pointer to the CAPI lib.
  param[in] init_set_prop_ptr: Pointer to the property list that needs to be
  initialized

  return: CAPI_EOK(0) on success else failure error code
 */
capi_err_t capi_duty_cycling_buf_init(capi_t *capi_ptr, capi_proplist_t *init_set_prop_ptr)
{
   capi_err_t capi_result;

   if (NULL == capi_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "Init received bad pointer, %p", capi_ptr);
      return CAPI_EFAILED;
   }
   if (NULL == init_set_prop_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "Received bad property pointer for get property, %p", init_set_prop_ptr);
      return CAPI_EFAILED;
   }

   capi_duty_cycling_buf_t *me_ptr = (capi_duty_cycling_buf_t *)(capi_ptr);

   memset(me_ptr, 0, sizeof(capi_duty_cycling_buf_t));
   
  
   // After init, we should be ready to accept input data and store in circular buffer
   me_ptr->current_status = DUTY_CYCLING_WHILE_BUFFERING;
   me_ptr->vtbl           = capi_duty_cycling_buf_get_vtbl();

   capi_result = capi_duty_cycling_buf_set_properties(capi_ptr, init_set_prop_ptr);
   if (CAPI_EOK != capi_result)
   {
      AR_MSG(DBG_ERROR_PRIO, "Set basic properties failed with result %lu", capi_result);
   }

   capi_duty_cycling_buf_init_events_config(me_ptr);

   capi_result |= capi_cmn_init_media_fmt_v2(&(me_ptr->in_media_fmt));
   capi_result |= capi_cmn_init_media_fmt_v2(&(me_ptr->out_media_fmt));
   // Module should raise data trigger in signal trigger container as module has to fill
   // the buffer in faster than real time
   capi_result |= capi_duty_cycling_buf_raise_event_data_trigger_in_st_cntr(me_ptr);
   capi_result |= capi_duty_cycling_buf_raise_events(me_ptr);

   // Propagate during init that DS is NRT on input so that container will propagate further backwards
   // so SPR can work in non-timer mode. Assume module is enabled and raise NRT in init
   capi_result |=capi_duty_cycling_buf_raise_rt_port_prop_event(me_ptr, TRUE /*INPUT*/, FALSE /*NRT*/, DEFAULT_PORT_INDEX);
   
   // Irrespective of input being RT/NRT output is always RT
   capi_result |=capi_duty_cycling_buf_raise_rt_port_prop_event(me_ptr, FALSE /*INPUT*/, TRUE /*RT*/, DEFAULT_PORT_INDEX);

   AR_MSG(DBG_HIGH_PRIO, "Init Exit result %u", capi_result);
   return capi_result;
}

/*==============================================================================
   Local Function Implementation
==============================================================================*/
capi_err_t capi_duty_cycling_buf_set_properties(capi_t *capi_ptr, capi_proplist_t *proplist_ptr)
{
   capi_err_t capi_result = CAPI_EOK;

   if ((NULL == capi_ptr) || (NULL == proplist_ptr))
   {
      AR_MSG(DBG_ERROR_PRIO, "received bad pointer in set_property");
      return CAPI_EFAILED;
   }

   capi_duty_cycling_buf_t *me_ptr = (capi_duty_cycling_buf_t *)capi_ptr;

   capi_result = capi_cmn_set_basic_properties(proplist_ptr, &me_ptr->heap_info, &me_ptr->event_cb_info, FALSE);

   if (CAPI_EOK != capi_result)
   {
      AR_MSG(DBG_ERROR_PRIO, "Set basic properties failed with result %lu", capi_result);
      return capi_result;
   }

   capi_prop_t *prop_array = proplist_ptr->prop_ptr;
   uint32_t i = 0;

   for (i = 0; i < proplist_ptr->props_num; i++)
   {

      capi_buf_t *payload_ptr = &prop_array[i].payload;
      switch (prop_array[i].id)
      {

         case CAPI_HEAP_ID:
         case CAPI_EVENT_CALLBACK_INFO:
         {
        	 break;
         }
         case CAPI_PORT_NUM_INFO:
         {

            if (payload_ptr->actual_data_len < sizeof(capi_port_num_info_t))
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "Set Property Failed. id= 0x%x Bad param size %u",
                      (uint32_t)prop_array[i].id,
                      payload_ptr->actual_data_len);
               capi_result |= CAPI_ENEEDMORE;
               break;
            }
            if (NULL == payload_ptr->data_ptr)
            {
               AR_MSG(DBG_ERROR_PRIO, "Set property id 0x%lx, received null buffer", prop_array[i].id);
               capi_result |= CAPI_EBADPARAM;
               break;
            }
            capi_port_num_info_t *data_ptr = (capi_port_num_info_t *)payload_ptr->data_ptr;
            if (DUTY_CYCLING_BUF_MAX_INPUT_PORTS != data_ptr->num_input_ports ||
                DUTY_CYCLING_BUF_MAX_OUTPUT_PORTS != data_ptr->num_output_ports)
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "num port info- out of range, provided num input ports = %lu,num output ports = %lu ", 
                      data_ptr
                         ->num_input_ports,
                      data_ptr->num_output_ports);
               capi_result |= CAPI_EBADPARAM;
               break;
            }
            me_ptr->num_in_ports  = data_ptr->num_input_ports;
            me_ptr->num_out_ports = data_ptr->num_output_ports;

            break;
         }

         case CAPI_ALGORITHMIC_RESET:
         {

            if (!prop_array[i].port_info.is_valid)
            {
               AR_MSG(DBG_ERROR_PRIO, "Algo Reset port info is invalid");
               capi_result |= CAPI_EBADPARAM;
               break;
            }
            if (prop_array[i].port_info.is_input_port)
            {
               AR_MSG(DBG_LOW_PRIO, "Algo Reset received for input port, doing nothing");
               break;
            }

            // If we are here, then it is algo reset on output port
            AR_MSG(DBG_LOW_PRIO, "Algo Reset received for output port");
            if (capi_duty_cycling_buf_does_circular_buffer_exist(me_ptr))
            {
               uint32_t read_offset = 0; // set the unread_data to 0
               me_ptr->data_remaining_in_ms_in_circ_buf=0;
               (void)spf_circ_buf_read_adjust(me_ptr->reader_handle, read_offset, &read_offset);

			   //We should be in non-island when algo reset is received.
			   //Should not hit the following condition

               if (me_ptr->current_status != DUTY_CYCLING_WHILE_BUFFERING)
               {
                  capi_result |= capi_duty_cycling_buf_update_tgp_while_buffering(me_ptr);
                  me_ptr->current_status = DUTY_CYCLING_WHILE_BUFFERING;
               }
            }

            break;
         }
         case CAPI_INPUT_MEDIA_FORMAT_V2:
         {
            capi_result |= capi_duty_cycling_buf_set_prop_input_media_fmt(me_ptr, &(prop_array[i]));
            break;
         }
         default:
         {
            AR_MSG(DBG_ERROR_PRIO, "received bad prop_id 0x%x", (uint32_t)prop_array[i].id);
            capi_result |= CAPI_EUNSUPPORTED;
         }
      }
   }

   return capi_result;
}

capi_err_t capi_duty_cycling_buf_get_properties(capi_t *capi_ptr, capi_proplist_t *proplist_ptr)
{
   capi_err_t rc = CAPI_EOK;
   if (NULL == capi_ptr || NULL == proplist_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "Error: capi_ptr[%p], proplist_ptr[%p]", capi_ptr, proplist_ptr);
      return CAPI_EFAILED;
   }

   capi_duty_cycling_buf_t *me_ptr = (capi_duty_cycling_buf_t *)capi_ptr;

   rc = capi_duty_cycling_buf_process_get_properties(me_ptr, proplist_ptr);

   return rc;
}

capi_err_t capi_duty_cycling_buf_set_param_port_op(capi_duty_cycling_buf_t *capi_ptr, capi_buf_t *params_ptr)
{
   capi_err_t               capi_result = CAPI_EOK;
   capi_duty_cycling_buf_t *me_ptr      = (capi_duty_cycling_buf_t *)((capi_ptr));

   if (NULL == params_ptr->data_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "Set param id 0x%lx, received null buffer", INTF_EXTN_PARAM_ID_DATA_PORT_OPERATION);
      return CAPI_EBADPARAM;
   }

   if (params_ptr->actual_data_len < sizeof(intf_extn_data_port_operation_t))
   {
      AR_MSG(DBG_ERROR_PRIO, "Invalid payload size for port operation %d", params_ptr->actual_data_len);
      return CAPI_ENEEDMORE;
   }

   intf_extn_data_port_operation_t *data_ptr = (intf_extn_data_port_operation_t *)(params_ptr->data_ptr);
   if (params_ptr->actual_data_len <
       sizeof(intf_extn_data_port_operation_t) + (data_ptr->num_ports * sizeof(intf_extn_data_port_id_idx_map_t)))
   {
      AR_MSG(DBG_ERROR_PRIO, "Invalid payload size for port operation %d", params_ptr->actual_data_len);
      return CAPI_ENEEDMORE;
   }

   if (data_ptr->is_input_port && (data_ptr->num_ports > DUTY_CYCLING_BUF_MAX_INPUT_PORTS))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "Invalid input ports. num_ports =%d, max_input_ports = %d",
             data_ptr->num_ports,
             DUTY_CYCLING_BUF_MAX_INPUT_PORTS);
      return CAPI_EBADPARAM;
   }

   if (!data_ptr->is_input_port && (data_ptr->num_ports > DUTY_CYCLING_BUF_MAX_OUTPUT_PORTS))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "Invalid output ports. num_ports =%d, max_output_ports = %d",
             data_ptr->num_ports,
             DUTY_CYCLING_BUF_MAX_OUTPUT_PORTS);
      return CAPI_EBADPARAM;
   }

   for (uint32_t iter = 0; iter < data_ptr->num_ports; iter++)
   {
      AR_MSG(DBG_HIGH_PRIO,
             "Port operation 0x%x performed on port_index= %lu, port_id= %lu is_input_port= %d ",
             data_ptr->opcode,
             data_ptr->id_idx[iter].port_index,
             data_ptr->id_idx[iter].port_id,
             data_ptr->is_input_port);

      switch (data_ptr->opcode)
      {
         case INTF_EXTN_DATA_PORT_START:
         {
            if (data_ptr->is_input_port)
            {
               if (DUTY_CYCLING_WHILE_BUFFERING == me_ptr->current_status)
               {
                  capi_result = capi_duty_cycling_buf_update_tgp_while_buffering(me_ptr);
                  if (!CAPI_SUCCEEDED(capi_result))
                  {
                     AR_MSG(DBG_ERROR_PRIO,
                            "while buffering: signal trigger policy update failed with result : %d",
                            capi_result);
                  }
               }
               else if (DUTY_CYCLING_AFTER_BUFFERING == me_ptr->current_status)
               {
                  capi_result = capi_duty_cycling_buf_update_tgp_after_buffering(me_ptr);
                  if (!CAPI_SUCCEEDED(capi_result))
                  {
                     AR_MSG(DBG_ERROR_PRIO,
                            "after buffering: signal trigger policy update failed with result : %d",
                            capi_result);
                  }
               }
               else
               {
                  AR_MSG(DBG_ERROR_PRIO,
                         " current_status : %d is not DUTY_CYCLING_WHILE_BUFFERING or DUTY_CYCLING_AFTER_BUFFERING",
                         me_ptr->current_status);
               }
            }
            break;
         }
         default:
         {
            AR_MSG(DBG_LOW_PRIO, "Port operation - Unsupported opcode: %lu - Not an error", data_ptr->opcode); // TODO is this
                                                                                                // required?
            break;
         }
      }
   }
   return capi_result;
}

capi_err_t capi_duty_cycling_buf_set_param(capi_t *                capi_ptr,
                                           uint32_t                param_id,
                                           const capi_port_info_t *port_info_ptr,
                                           capi_buf_t *            params_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   if (NULL == capi_ptr || NULL == params_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "received bad property pointer for "
             "param_id property, 0x%x",
             param_id);
      return (CAPI_EFAILED);
   }

   capi_duty_cycling_buf_t *me_ptr = (capi_duty_cycling_buf_t *)((capi_ptr));

   switch (param_id)
   {
      case PARAM_ID_MODULE_ENABLE:
      {
         param_id_module_enable_t *enable_ptr = (param_id_module_enable_t *)params_ptr->data_ptr;
         if ((sizeof(param_id_module_enable_t) > params_ptr->actual_data_len))
         {
            AR_MSG(DBG_ERROR_PRIO,
                   " Param id 0x%lx Bad param size %lu",
                   (uint32_t)param_id,
                   params_ptr->actual_data_len);
            capi_result |= CAPI_ENEEDMORE;
            break;
         }

         bool_t mod_enable = (enable_ptr->enable) ? TRUE : FALSE;
         AR_MSG(DBG_HIGH_PRIO, "Received set_param enable - %u", mod_enable);

         // Assuming we don't receive enable or disable multiple times
         if (me_ptr->events_config.enable != mod_enable)
         {
            me_ptr->events_config.enable = mod_enable;
            bool is_rt                   = TRUE;
            // disable to enable case
            if (me_ptr->events_config.enable)
            {
               is_rt = FALSE;
            }
            capi_result |=
               capi_duty_cycling_buf_raise_rt_port_prop_event(me_ptr, TRUE /*INPUT*/, is_rt /*RT*/, DEFAULT_PORT_INDEX);
         }
         break;
      }
      case INTF_EXTN_PARAM_ID_DATA_PORT_OPERATION:
      {
         capi_result |= capi_duty_cycling_buf_set_param_port_op(me_ptr, params_ptr);
         break;
      }
      case INTF_EXTN_PARAM_ID_METADATA_HANDLER:
      {
         if (params_ptr->actual_data_len < sizeof(intf_extn_param_id_metadata_handler_t))
         {
            AR_MSG(DBG_ERROR_PRIO,
                   " Param id 0x%lx Bad param size %lu",
                   (uint32_t)param_id,
                   params_ptr->actual_data_len);
            capi_result |= CAPI_ENEEDMORE;
            break;
         }
         intf_extn_param_id_metadata_handler_t *payload_ptr =
            (intf_extn_param_id_metadata_handler_t *)params_ptr->data_ptr;
         me_ptr->metadata_handler = *payload_ptr;
         break;
      }
      case FWK_EXTN_PARAM_ID_TRIGGER_POLICY_CB_FN:
      {
         if (NULL == params_ptr->data_ptr)
         {
            AR_MSG(DBG_ERROR_PRIO, " Param id 0x%lx received null buffer", (uint32_t)param_id);
            capi_result |= CAPI_EBADPARAM;
            break;
         }

         if (params_ptr->actual_data_len < sizeof(fwk_extn_param_id_trigger_policy_cb_fn_t))
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "capi duty_cycling_buf: Param id 0x%lx Bad param size %lu",
                   (uint32_t)param_id,
                   params_ptr->actual_data_len);
            capi_result |= CAPI_ENEEDMORE;
            break;
         }

         fwk_extn_param_id_trigger_policy_cb_fn_t *payload_ptr =
            (fwk_extn_param_id_trigger_policy_cb_fn_t *)params_ptr->data_ptr;
         me_ptr->tgp.tg_policy_cb = *payload_ptr;

         // As soon as cb function is received update tgp to be in while_buffering mode
         capi_result |= capi_duty_cycling_buf_update_tgp_while_buffering(me_ptr);
         break;
      }
      case FWK_EXTN_PARAM_ID_CONTAINER_FRAME_DURATION:
      {
         if (params_ptr->actual_data_len < sizeof(fwk_extn_param_id_container_frame_duration_t))
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "Invalid payload size for param_id=0x%lx actual_data_len=%lu  ",
                   param_id,
                   params_ptr->actual_data_len);
            capi_result |= CAPI_ENEEDMORE;
            break;
         }

         fwk_extn_param_id_container_frame_duration_t *cfg_ptr =
            (fwk_extn_param_id_container_frame_duration_t *)params_ptr->data_ptr;

         if (0 == cfg_ptr->duration_us)
         {
            AR_MSG(DBG_ERROR_PRIO, "Frame duration sent as 0ms. Ignoring.");
            break;
         }

         if (me_ptr->cntr_frame_size_us != cfg_ptr->duration_us)
         {
            AR_MSG(DBG_HIGH_PRIO,
                   "Container frame size duration = %lu,prev frzme size = %lu",
                   cfg_ptr->duration_us,
                   me_ptr->cntr_frame_size_us);
            me_ptr->cntr_frame_size_us       = cfg_ptr->duration_us;
            me_ptr->cntr_frame_size_received = TRUE;
            capi_duty_cycling_buf_island_entry_exit_util(me_ptr);
            capi_result |= duty_cycling_buf_check_create_circular_buffer(me_ptr);

            // Set media format again to circular buffers if frame size changes.
            if (capi_duty_cycling_buf_does_circular_buffer_exist(me_ptr))
            {
               capi_duty_cycling_buf_set_circular_buffer_media_fmt(me_ptr,
                                                                   &(me_ptr->in_media_fmt),
                                                                   me_ptr->cntr_frame_size_us);
            }
         }

         break;
      }

      case PARAM_ID_DUTY_CYCLING_BUF_CONFIG:
      {
         if (params_ptr->actual_data_len < sizeof(param_id_duty_cycling_buf_config))
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "duty_cycling_buf: Invalid payload size for param_id=0x%lx actual_data_len=%lu  ",
                   param_id,
                   params_ptr->actual_data_len);
            capi_result |= CAPI_ENEEDMORE;
            break;
         }

         param_id_duty_cycling_buf_config *cfg_ptr = (param_id_duty_cycling_buf_config *)params_ptr->data_ptr;

         if (cfg_ptr->buffer_size_in_ms == 0 || cfg_ptr->lower_threshold_in_ms == 0)
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "duty_cycling_buf: buffer_size_in_ms %u lower_threshold_in_ms %u cannot be zero ",
                   cfg_ptr->buffer_size_in_ms,
                   cfg_ptr->lower_threshold_in_ms);
            capi_result |= CAPI_EBADPARAM;
            break;
         }

         me_ptr->buffer_size_in_ms                       = cfg_ptr->buffer_size_in_ms;
         me_ptr->lower_threshold_in_ms_rcvd_in_set_param = cfg_ptr->lower_threshold_in_ms;

         // set the calibration flag to TRUE.
         me_ptr->is_buf_cfg_received = TRUE;

         AR_MSG(DBG_HIGH_PRIO,
                "buffer_size_in_ms %u, lower_threshold_in_ms %u",
                me_ptr->buffer_size_in_ms,
                me_ptr->lower_threshold_in_ms_rcvd_in_set_param);

         capi_duty_cycling_buf_island_entry_exit_util(me_ptr);
         capi_result |= duty_cycling_buf_check_create_circular_buffer(me_ptr);

         break;
      }
      case INTF_EXTN_PARAM_ID_IS_RT_PORT_PROPERTY:
      {
         capi_result |= capi_duty_cycling_buf_set_data_port_property(me_ptr, params_ptr);
         break;
      }
      default:
      {
         AR_MSG(DBG_HIGH_PRIO, "capi duty_cycling_buf: Unsupported Param id ::0x%x \n", param_id);
         capi_result = CAPI_EUNSUPPORTED;
      }
      break;
   }

   return capi_result;
}

/*------------------------------------------------------------------------
  Function name: capi_duty_cycling_buf_get_param
  Gets either a parameter value or a parameter structure containing multiple
  parameters. In the event of a failure, the appropriate error code is
  returned.
 * -----------------------------------------------------------------------*/
capi_err_t capi_duty_cycling_buf_get_param(capi_t *                capi_ptr,
                                           uint32_t                param_id,
                                           const capi_port_info_t *port_info_ptr,
                                           capi_buf_t *            params_ptr)
{
   capi_err_t result = CAPI_EOK;
   if (NULL == capi_ptr || NULL == params_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "Received bad property pointer for "
             "param_id property, 0x%x",
             param_id);
      return (CAPI_EFAILED);
   }

   capi_duty_cycling_buf_t *me_ptr = (capi_duty_cycling_buf_t *)((capi_ptr));
   params_ptr->actual_data_len     = 0;

   switch (param_id)
   {

      case PARAM_ID_MODULE_ENABLE:
      {

         if (params_ptr->max_data_len < sizeof(param_id_module_enable_t))
         {
            AR_MSG(DBG_ERROR_PRIO,
                   " Param id 0x%lx Bad max_data_len %lu",
                   (uint32_t)param_id,
                   params_ptr->max_data_len);
            result |= CAPI_ENEEDMORE;
            break;
         }

         param_id_module_enable_t *enable_ptr = (param_id_module_enable_t *)params_ptr->data_ptr;
         enable_ptr->enable                   = me_ptr->events_config.enable;
         params_ptr->actual_data_len          = sizeof(param_id_module_enable_t);

         break;
      }

      case PARAM_ID_DUTY_CYCLING_BUF_CONFIG:
      {
         if (params_ptr->max_data_len < sizeof(param_id_duty_cycling_buf_config))
         {
            AR_MSG(DBG_ERROR_PRIO, " Badparam_id=0x%lx max_data_len=%lu  ", param_id, params_ptr->max_data_len);
            result |= CAPI_ENEEDMORE;
            break;
         }

         param_id_duty_cycling_buf_config *cfg_ptr = (param_id_duty_cycling_buf_config *)params_ptr->data_ptr;

         cfg_ptr->buffer_size_in_ms     = me_ptr->buffer_size_in_ms;
         cfg_ptr->lower_threshold_in_ms = me_ptr->lower_threshold_in_ms_rcvd_in_set_param;
         params_ptr->actual_data_len    = sizeof(param_id_duty_cycling_buf_config);

         break;
      }

      default:
      {
         AR_MSG(DBG_HIGH_PRIO, "Unsupported Param id ::0x%x \n", param_id);
         result = CAPI_EUNSUPPORTED;
      }
      break;
   }
   return result;
}

/*------------------------------------------------------------------------
  Function name: capi_duty_cycling_buf_end
  Returns the library to the uninitialized state and frees the memory that
  was allocated by Init(). This function also frees the virtual function
  table.
 * -----------------------------------------------------------------------*/
capi_err_t capi_duty_cycling_buf_end(capi_t *capi_ptr)
{
   capi_err_t result = CAPI_EOK;
   if (NULL == capi_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, " end received bad pointer, %p", capi_ptr);
      return (CAPI_EFAILED);
   }

   capi_duty_cycling_buf_t *        me_ptr      = (capi_duty_cycling_buf_t *)((capi_ptr));

   if (capi_duty_cycling_buf_does_circular_buffer_exist(me_ptr))
   {
      result |= capi_duty_cycling_buf_destroy_circular_buffer(me_ptr);
   }

   me_ptr->vtbl = NULL;

   AR_MSG(DBG_HIGH_PRIO, "end done ");
   return result;
}

capi_err_t capi_duty_cycling_buf_raise_rt_port_prop_event(capi_duty_cycling_buf_t *me_ptr,
                                                          bool_t                   is_input,
                                                          bool_t                   is_rt,
                                                          bool_t                   port_index)
{

   capi_err_t                               capi_result = CAPI_EOK;
   capi_buf_t                               payload;
   intf_extn_param_id_is_rt_port_property_t event;

   if (NULL == me_ptr->event_cb_info.event_cb)
   {
      AR_MSG(DBG_ERROR_PRIO, "Event callback is not set. Unable to raise events!");
      CAPI_SET_ERROR(capi_result, CAPI_EUNSUPPORTED);
      return capi_result;
   }

   event.is_input   = is_input;
   event.is_rt      = is_rt;
   event.port_index = port_index;

   payload.data_ptr        = (int8_t *)&event;
   payload.actual_data_len = payload.max_data_len = sizeof(event);

   capi_result =
      capi_cmn_raise_data_to_dsp_svc_event(&me_ptr->event_cb_info, INTF_EXTN_EVENT_ID_IS_RT_PORT_PROPERTY, &payload);

   if (CAPI_FAILED(capi_result))
   {
      AR_MSG(DBG_ERROR_PRIO, "Failed to raise event to enable data_trigger in ST container");
      return capi_result;
   }
   else
   {
      AR_MSG(DBG_HIGH_PRIO, "raised event to enable data_trigger in ST container");
   }

   return capi_result;
}

capi_err_t capi_duty_cycling_buf_destroy_circular_buffer(capi_duty_cycling_buf_t *        me_ptr)
{

   // Destroys circular buffers and reader/writer registers
   capi_err_t res = spf_circ_buf_deinit(&me_ptr->stream_drv_ptr);
   if (CAPI_EOK != res)
   {
      return CAPI_EFAILED;
   }

   me_ptr->writer_handle = NULL;
   me_ptr->reader_handle = NULL;

   return CAPI_EOK;
}


