/**
 *   \file capi_rt_proxy.c
 *   \brief
 *        This file contains CAPI implementation of RT Proxy module.
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "capi_rt_proxy_i.h"

static capi_err_t capi_rt_proxy_process(capi_t *_pif, capi_stream_data_t *input[], capi_stream_data_t *output[]);

static capi_err_t capi_rt_proxy_end(capi_t *_pif);

static capi_err_t capi_rt_proxy_set_param(capi_t *                _pif,
                                          uint32_t                param_id,
                                          const capi_port_info_t *port_info_ptr,
                                          capi_buf_t *            params_ptr);

static capi_err_t capi_rt_proxy_get_param(capi_t *                _pif,
                                          uint32_t                param_id,
                                          const capi_port_info_t *port_info_ptr,
                                          capi_buf_t *            params_ptr);

static capi_err_t capi_rt_proxy_set_properties(capi_t *_pif, capi_proplist_t *props_ptr);

static capi_err_t capi_rt_proxy_get_properties(capi_t *_pif, capi_proplist_t *props_ptr);

capi_err_t capi_rt_proxy_init(capi_t *capi_ptr, capi_proplist_t *init_set_prop_ptr, bool_t is_tx_module);

static const capi_vtbl_t vtbl = { capi_rt_proxy_process,        capi_rt_proxy_end,
                                  capi_rt_proxy_set_param,      capi_rt_proxy_get_param,
                                  capi_rt_proxy_set_properties, capi_rt_proxy_get_properties };

/*==============================================================================
   Local Function forward declaration
==============================================================================*/

/*==============================================================================
   Public Function Implementation
==============================================================================*/

/*
  This function is used to query the static properties to create the CAPI.

  param[in] init_set_prop_ptr: Pointer to the initializing property list
  param[in, out] static_prop_ptr: Pointer to the static property list

  return: CAPI_EOK(0) on success else failure error code
 */
capi_err_t capi_rt_proxy_rx_get_static_properties(capi_proplist_t *init_set_prop_ptr, capi_proplist_t *static_prop_ptr)
{
   return capi_rt_proxy_get_properties(NULL, static_prop_ptr);
}

capi_err_t capi_rt_proxy_rx_init(capi_t *capi_ptr, capi_proplist_t *init_set_prop_ptr)
{
   return capi_rt_proxy_init(capi_ptr, init_set_prop_ptr, FALSE);
}

capi_err_t capi_rt_proxy_tx_get_static_properties(capi_proplist_t *init_set_prop_ptr, capi_proplist_t *static_prop_ptr)
{
   return capi_rt_proxy_get_properties(NULL, static_prop_ptr);
}

capi_err_t capi_rt_proxy_tx_init(capi_t *capi_ptr, capi_proplist_t *init_set_prop_ptr)
{
   return capi_rt_proxy_init(capi_ptr, init_set_prop_ptr, TRUE);
}

/*
  This function is used init the CAPI lib.

  param[in] capi_ptr: Pointer to the CAPI lib.
  param[in] init_set_prop_ptr: Pointer to the property list that needs to be
            initialized

  return: CAPI_EOK(0) on success else failure error code
 */
capi_err_t capi_rt_proxy_init(capi_t *capi_ptr, capi_proplist_t *init_set_prop_ptr, bool_t is_tx_module)
{
   capi_err_t result = CAPI_EOK;
   if (NULL == capi_ptr || NULL == init_set_prop_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "NULL capi_ptr[%p], init_set_prop_ptr[%p]", capi_ptr, init_set_prop_ptr);
      return CAPI_EFAILED;
   }

   capi_rt_proxy_t *me_ptr = (capi_rt_proxy_t *)((capi_ptr));
   me_ptr->vtbl            = &vtbl;

   me_ptr->is_tx_module = is_tx_module;

   // Set the init properties.
   result = capi_rt_proxy_set_properties((capi_t *)me_ptr, init_set_prop_ptr);

   // Intialize drift accumulator handle. which will be shared with RAT module.
   result |= capi_rt_proxy_init_out_drift_info(&me_ptr->drift_info, rt_proxy_imcl_read_acc_out_drift);

   return result;
}

/*==============================================================================
   Local Function Implementation
==============================================================================*/

static capi_err_t capi_rt_proxy_set_properties(capi_t *capi_ptr, capi_proplist_t *proplist_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   if ((NULL == capi_ptr) || (NULL == proplist_ptr))
   {
      AR_MSG(DBG_ERROR_PRIO, "rt_proxy: Set property failed. received bad pointer in set_property");
      return CAPI_EFAILED;
   }

   capi_rt_proxy_t *me_ptr = (capi_rt_proxy_t *)capi_ptr;

   capi_prop_t *prop_ptr = proplist_ptr->prop_ptr;
   if (NULL == prop_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "rt_proxy: Set property failed. received bad pointer in prop_ptr");
      return CAPI_EFAILED;
   }

   uint32_t i;

   for (i = 0; i < proplist_ptr->props_num; i++)
   {
      capi_buf_t *payload_ptr = &prop_ptr[i].payload;

      switch (prop_ptr[i].id)
      {
         case CAPI_PORT_NUM_INFO:
         {
            if (payload_ptr->actual_data_len < sizeof(capi_port_num_info_t))
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "rt_proxy: CAPI V2 FAILED Set Property id 0x%x Bad param size %u",
                      (uint32_t)prop_ptr[i].id,
                      payload_ptr->actual_data_len);
               return CAPI_ENEEDMORE;
            }
            capi_port_num_info_t *data_ptr = (capi_port_num_info_t *)payload_ptr->data_ptr;

            if (data_ptr->num_input_ports != CAPI_RT_PROXY_MAX_INPUT_PORTS ||
                data_ptr->num_output_ports != CAPI_RT_PROXY_MAX_OUTPUT_PORTS)
            {
               AR_MSG(DBG_ERROR_PRIO, "rt_proxy: Set property failed. Num port info is not valid.");
               return CAPI_EFAILED;
            }

            break;
         }
         case CAPI_HEAP_ID:
         {
            if (payload_ptr->actual_data_len < sizeof(capi_heap_id_t))
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "rt_proxy:  Set Property failed. id= 0x%x Bad param size %u",
                      (uint32_t)prop_ptr[i].id,
                      payload_ptr->actual_data_len);
               return CAPI_ENEEDMORE;
            }

            capi_heap_id_t *data_ptr = (capi_heap_id_t *)payload_ptr->data_ptr;
            me_ptr->heap_id          = (POSAL_HEAP_ID)data_ptr->heap_id;

            break;
         }
         case CAPI_EVENT_CALLBACK_INFO:
         {
            if (payload_ptr->actual_data_len < sizeof(capi_event_callback_info_t))
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "rt_proxy: Set Property Failed. id= 0x%x Bad param size %u",
                      (uint32_t)prop_ptr[i].id,
                      payload_ptr->actual_data_len);
               return CAPI_ENEEDMORE;
            }

            capi_event_callback_info_t *data_ptr = (capi_event_callback_info_t *)payload_ptr->data_ptr;

            me_ptr->event_cb_info.event_cb      = data_ptr->event_cb;
            me_ptr->event_cb_info.event_context = data_ptr->event_context;

            break;
         }
         case CAPI_INPUT_MEDIA_FORMAT_V2:
         {
            if (NULL == payload_ptr->data_ptr)
            {
               AR_MSG(DBG_ERROR_PRIO, "rt_proxy: Set property id 0x%lx, received null buffer", prop_ptr[i].id);
               return CAPI_EFAILED;
            }

            if (!prop_ptr[i].port_info.is_valid)
            {
               AR_MSG(DBG_ERROR_PRIO, "rt_proxy: Media format port info is invalid");
               return CAPI_ENEEDMORE;
            }

            // validate the MF payload
            if (payload_ptr->actual_data_len <
                sizeof(capi_set_get_media_format_t) + sizeof(capi_standard_data_format_v2_t))
            {
               AR_MSG(DBG_ERROR_PRIO, "rt_proxy: Invalid media format size %d", payload_ptr->actual_data_len);
               return CAPI_ENEEDMORE;
            }

            // Handle media format if media format is not received yet or if it has changed.
            if (FALSE == me_ptr->is_input_mf_received ||
                !capi_cmn_media_fmt_equal(&me_ptr->operating_mf, (capi_media_fmt_v2_t *)payload_ptr->data_ptr))
            {
               // Validate the media format
               if (CAPI_EOK != (capi_result = rt_proxy_vaildate_and_cache_input_media_format(me_ptr, payload_ptr)))
               {
                  AR_MSG(DBG_ERROR_PRIO, "rt_proxy: Received invalid input media format.");
                  return CAPI_EFAILED;
               }

               spf_circ_buf_set_media_format(me_ptr->driver_hdl.writer_handle,
                                             (capi_media_fmt_v2_t *)payload_ptr->data_ptr,
                                             me_ptr->frame_duration_in_us);

               // Compute upped and lower drift thresholds. And water mark levels in bytes based on new mf.
               ar_result_t ar_result = AR_EOK;
               if (AR_EOK != (ar_result = rt_proxy_calibrate_driver(me_ptr)))
               {
                  AR_MSG(DBG_ERROR_PRIO, "rt_proxy: Failed calibrating the driver.");
               }

               // raise BW events.
               capi_result |= rt_proxy_raise_mpps_and_bw_events(me_ptr);
            }

            break;
         }
         default:
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "rt_proxy: Set property failed. received un-supported prop_id 0x%x",
                   (uint32_t)prop_ptr[i].id);
            return CAPI_EUNSUPPORTED;
         }
      } /* Outer switch - Generic CAPI Properties */
   }    /* Loop all properties */

   return capi_result;
}

static capi_err_t capi_rt_proxy_get_properties(capi_t *capi_ptr, capi_proplist_t *proplist_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   uint32_t   i;

   if (NULL == proplist_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "rt_proxy: Get properties received null arguments");
      return CAPI_EBADPARAM;
   }

   capi_prop_t *prop_ptr = proplist_ptr->prop_ptr;
   //   capi_rt_proxy_t *me_ptr   = (capi_rt_proxy_t *)capi_ptr;

   uint32_t fwk_extn_ids[RT_PROXY_NUM_FRAMEWORK_EXTENSIONS] = { 0 };
   fwk_extn_ids[0]                                          = FWK_EXTN_MULTI_PORT_BUFFERING;
   fwk_extn_ids[1]                                          = FWK_EXTN_TRIGGER_POLICY;
   fwk_extn_ids[2]                                          = FWK_EXTN_CONTAINER_FRAME_DURATION;

   capi_basic_prop_t mod_prop;
   mod_prop.init_memory_req    = sizeof(capi_rt_proxy_t);
   mod_prop.stack_size         = CAPI_RT_PROXY_MODULE_STACK_SIZE;
   mod_prop.num_fwk_extns      = RT_PROXY_NUM_FRAMEWORK_EXTENSIONS;
   mod_prop.fwk_extn_ids_arr   = fwk_extn_ids;
   mod_prop.is_inplace         = FALSE; // not capable of in-place processing of data
   mod_prop.req_data_buffering = FALSE; // Doesnt require data buffering.
   mod_prop.max_metadata_size  = 0;     // NA

   capi_result = capi_cmn_get_basic_properties(proplist_ptr, &mod_prop);
   if (CAPI_EOK != capi_result)
   {
      AR_MSG(DBG_ERROR_PRIO, "rt_proxy: Get properties failed with result %lu", capi_result);
      return capi_result;
   }

   // iterating over the properties
   for (i = 0; i < proplist_ptr->props_num; i++)
   {
      capi_buf_t *payload_ptr = &prop_ptr[i].payload;

      switch (prop_ptr[i].id)
      {
         case CAPI_INIT_MEMORY_REQUIREMENT:
         case CAPI_STACK_SIZE:
         case CAPI_IS_INPLACE:
         case CAPI_REQUIRES_DATA_BUFFERING:
         case CAPI_NUM_NEEDED_FRAMEWORK_EXTENSIONS:
         case CAPI_NEEDED_FRAMEWORK_EXTENSIONS:
         case CAPI_OUTPUT_MEDIA_FORMAT_SIZE:
         {
            // handled in capi common utils.
            break;
         }
         case CAPI_PORT_DATA_THRESHOLD:
         {
            break;
         }
         case CAPI_INTERFACE_EXTENSIONS:
         {
            capi_interface_extns_list_t *intf_ext_list = (capi_interface_extns_list_t *)payload_ptr->data_ptr;
            capi_result |=
               ((payload_ptr->max_data_len < sizeof(capi_interface_extns_list_t)) ||
                (payload_ptr->max_data_len < (sizeof(capi_interface_extns_list_t) +
                                              (intf_ext_list->num_extensions * sizeof(capi_interface_extn_desc_t)))))
                  ? CAPI_ENEEDMORE
                  : capi_result;

            if (CAPI_FAILED(capi_result))
            {
               payload_ptr->actual_data_len = 0;
               AR_MSG(DBG_ERROR_PRIO, "Insufficient get property size.");
               break;
            }

            capi_interface_extn_desc_t *curr_intf_extn_desc_ptr =
               (capi_interface_extn_desc_t *)(payload_ptr->data_ptr + sizeof(capi_interface_extns_list_t));

            for (uint32_t j = 0; j < intf_ext_list->num_extensions; j++)
            {
               switch (curr_intf_extn_desc_ptr->id)
               {
                  case INTF_EXTN_IMCL:
                     curr_intf_extn_desc_ptr->is_supported = TRUE;
                     break;
                  case INTF_EXTN_METADATA:
                     curr_intf_extn_desc_ptr->is_supported = TRUE;
                     break;
                  default:
                  {
                     curr_intf_extn_desc_ptr->is_supported = FALSE;
                     break;
                  }
               }
               curr_intf_extn_desc_ptr++;
            }

            break;
         } // CAPI_INTERFACE_EXTENSIONS
         default:
         {
            AR_MSG(DBG_ERROR_PRIO, "rt_proxy: Unknown Prop[%d]", prop_ptr[i].id);
            capi_result |= CAPI_EUNSUPPORTED;
            break;
         }
      } // switch
   }
   return capi_result;
}

/*------------------------------------------------------------------------
  Function name: capi_rt_proxy_set_param
  Sets either a parameter value or a parameter structure containing multiple
  parameters. In the event of a failure, the appropriate error code is
  returned.
 * -----------------------------------------------------------------------*/
static capi_err_t capi_rt_proxy_set_param(capi_t *                capi_ptr,
                                          uint32_t                param_id,
                                          const capi_port_info_t *port_info_ptr,
                                          capi_buf_t *            params_ptr)
{
   capi_err_t result = CAPI_EOK;
   if (NULL == capi_ptr || NULL == params_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "rt_proxy: Set param failed. received bad property pointer for param_id property, 0x%x",
             param_id);
      return CAPI_EFAILED;
   }

   capi_rt_proxy_t *me_ptr = (capi_rt_proxy_t *)((capi_ptr));

   switch (param_id)
   {
      case FWK_EXTN_PARAM_ID_CONTAINER_FRAME_DURATION:
      {
         if (params_ptr->actual_data_len < sizeof(fwk_extn_param_id_container_frame_duration_t))
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "rt_proxy: Invalid payload size for param_id=0x%lx actual_data_len=%lu  ",
                   param_id,
                   params_ptr->actual_data_len);
            result = CAPI_ENEEDMORE;
            break;
         }

         fwk_extn_param_id_container_frame_duration_t *cfg_ptr =
            (fwk_extn_param_id_container_frame_duration_t *)params_ptr->data_ptr;

         // handle change in frame duration
         if (me_ptr->frame_duration_in_us != cfg_ptr->duration_us)
         {
            AR_MSG(DBG_HIGH_PRIO,
                   "rt_proxy: threshold configuration changed from %lu to %lu",
                   me_ptr->frame_duration_in_us,
                   cfg_ptr->duration_us);

            me_ptr->frame_duration_in_us = cfg_ptr->duration_us;

			if(me_ptr->is_input_mf_received)
			{
				spf_circ_buf_set_media_format(me_ptr->driver_hdl.writer_handle,
											  &me_ptr->operating_mf,
											  me_ptr->frame_duration_in_us);
			}
         }

         AR_MSG(DBG_LOW_PRIO,
                "rt_proxy: threshold configuration = %lu us is set. result 0x%lx",
                cfg_ptr->duration_us,
                result);

         break;
      }
      case PARAM_ID_RT_PROXY_CONFIG:
      {
         if (params_ptr->actual_data_len < sizeof(param_id_rt_proxy_config_t))
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "rt_proxy: Invalid payload size for param_id=0x%lx actual_data_len=%lu  ",
                   param_id,
                   params_ptr->actual_data_len);
            result = CAPI_ENEEDMORE;
            break;
         }

         param_id_rt_proxy_config_t *cfg_ptr = (param_id_rt_proxy_config_t *)params_ptr->data_ptr;

         // copy configuration to capi memory.
         memscpy(&me_ptr->cfg, sizeof(param_id_rt_proxy_config_t), cfg_ptr, sizeof(param_id_rt_proxy_config_t));

         // set the calibration flag to TRUE.
         me_ptr->is_calib_set = TRUE;

         // If calibration and media format are set then initialize the driver.
         // we need calibration to create buffers.
         if (!me_ptr->driver_hdl.stream_buf)
         {
            ar_result_t result = AR_EOK;
            if (AR_EOK != (result = rt_proxy_driver_init(me_ptr)))
            {
               AR_MSG(DBG_ERROR_PRIO, "rt_proxy: Cannot intialize the driver. ");
               return CAPI_EFAILED;
            }
         }
         else
         {
            // if driver is already initialized just re calibrate the driver.
            rt_proxy_calibrate_driver(me_ptr);
         }

         break;
      }
      case INTF_EXTN_PARAM_ID_IMCL_PORT_OPERATION:
      {
         result = capi_rt_proxy_handle_imcl_port_operation(me_ptr, params_ptr);
         break;
      }
      case FWK_EXTN_PARAM_ID_TRIGGER_POLICY_CB_FN:
      {
         if (NULL == params_ptr->data_ptr)
         {
            AR_MSG(DBG_ERROR_PRIO, "rt_proxy: Set param id 0x%lx, received null buffer", param_id);
            result |= CAPI_EBADPARAM;
            break;
         }

         // Level 1 check
         if (params_ptr->actual_data_len < sizeof(fwk_extn_param_id_trigger_policy_cb_fn_t))
         {
            AR_MSG(DBG_ERROR_PRIO, "rt_proxy: Invalid payload size for trigger policy %d", params_ptr->actual_data_len);
            return CAPI_ENEEDMORE;
         }

         fwk_extn_param_id_trigger_policy_cb_fn_t *payload_ptr =
            (fwk_extn_param_id_trigger_policy_cb_fn_t *)params_ptr->data_ptr;
         me_ptr->policy_chg_cb = *payload_ptr;

         // By default set the mode to RT, when the write arrives then make it FTRT.
         rt_proxy_change_trigger_policy(me_ptr, FWK_EXTN_PORT_TRIGGER_POLICY_MANDATORY);

         break;
      }
      case INTF_EXTN_PARAM_ID_METADATA_HANDLER:
      {
         if (params_ptr->actual_data_len < sizeof(intf_extn_param_id_metadata_handler_t))
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "Param id 0x%lx Bad param size %lu",
                   (uint32_t)param_id,
                   params_ptr->actual_data_len);
            return CAPI_ENEEDMORE;
         }

         intf_extn_param_id_metadata_handler_t *payload_ptr =
            (intf_extn_param_id_metadata_handler_t *)params_ptr->data_ptr;
         me_ptr->metadata_handler = *payload_ptr;
         break;
      }
      default:
      {
         AR_MSG(DBG_ERROR_PRIO, "rt_proxy: Unsupported Param id= 0x%x \n", param_id);
         result = CAPI_EUNSUPPORTED;
      }
   }

   AR_MSG(DBG_HIGH_PRIO, "rt_proxy: Set param= 0x%x done with result= 0x%x", param_id, result);
   return result;
}

/*------------------------------------------------------------------------
  Function name: capi_rt_proxy_get_param
  Gets either a parameter value or a parameter structure containing multiple
  parameters. In the event of a failure, the appropriate error code is
  returned.
 * -----------------------------------------------------------------------*/
static capi_err_t capi_rt_proxy_get_param(capi_t *                capi_ptr,
                                          uint32_t                param_id,
                                          const capi_port_info_t *port_info_ptr,
                                          capi_buf_t *            params_ptr)
{
   capi_err_t result = CAPI_EOK;
   if (NULL == capi_ptr || NULL == params_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "rt_proxy: FAILED received bad property pointer for param_id property, 0x%x", param_id);
      return CAPI_EFAILED;
   }

   // capi_rt_proxy_t *me_ptr = (capi_rt_proxy_t *)((capi_ptr));

   switch (param_id)
   {
      default:
      {
         AR_MSG(DBG_HIGH_PRIO, "rt_proxy: Unsupported Param id= 0x%x \n", param_id);
         result = CAPI_EUNSUPPORTED;
      }
      break;
   }

   AR_MSG(DBG_HIGH_PRIO, "rt_proxy: Get param done for param id= 0x%x, result= 0x%x", param_id, result);
   return result;
}

/*------------------------------------------------------------------------
  Function name: capi_rt_proxy_process
  Processes an input buffer and generates an output buffer.
 * -----------------------------------------------------------------------*/
static capi_err_t capi_rt_proxy_process(capi_t *capi_ptr, capi_stream_data_t *input[], capi_stream_data_t *output[])
{
   capi_err_t result = CAPI_EOK;

   if (NULL == capi_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "rt_proxy: received bad property pointer");
      return CAPI_EFAILED;
   }

   if ((NULL == input) || (NULL == output))
   {
      return result;
   }

   capi_rt_proxy_t *me_ptr = (capi_rt_proxy_t *)capi_ptr;

   if (input)
   {
      // Check if the ports stream buffers are valid
      if (NULL == input[0])
      {
         AR_MSG(DBG_HIGH_PRIO, "Input buffers not available ");
      }
      else
      {
         // Write data into the circular buffer.
         result = rt_proxy_stream_write(me_ptr, input[0]);

         //         result |= capi_rt_proxy_propagate_metadata(me_ptr, input[0]);
      }
   }

   if (output)
   {
      // output buffers sanity check.
      if (NULL == output[0])
      {
         AR_MSG(DBG_HIGH_PRIO, "Output buffers not available ");
      }
      else
      {
         // Read data from the circular buffers.
         result = rt_proxy_stream_read(me_ptr, output[0]);
         if (result == AR_ENEEDMORE)
         {
            AR_MSG(DBG_ERROR_PRIO, "rt_proxy: Underrun detected.");

            // need to listen to both inputs and outputs now.
            rt_proxy_change_trigger_policy(me_ptr, FWK_EXTN_PORT_TRIGGER_POLICY_MANDATORY);

            return CAPI_ENEEDMORE;
         }
         else if (result == AR_EFAILED)
         {
            return CAPI_EFAILED;
         }
      }
   }

   return result;
}

/*------------------------------------------------------------------------
  Function name: capi_rt_proxy_end
  Returns the library to the uninitialized state and frees the memory that
  was allocated by Init(). This function also frees the virtual function
  table.
 * -----------------------------------------------------------------------*/
static capi_err_t capi_rt_proxy_end(capi_t *capi_ptr)
{
   capi_err_t result = CAPI_EOK;

   // uint32_t              port_idx      = 0;

   if (NULL == capi_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "rt_proxy: received null pointer, %p", capi_ptr);
      return CAPI_EFAILED;
   }
   capi_rt_proxy_t *me_ptr = (capi_rt_proxy_t *)((capi_ptr));

   // deinit the rt proxy driver, destroys the circular buffer.
   rt_proxy_driver_deinit(me_ptr);

   result |= capi_rt_proxy_deinit_out_drift_info(&me_ptr->drift_info);

   me_ptr->vtbl = NULL;
   return result;
}
