/**
 *   \file capi_jitter_buf.c
 *   \brief
 *        This file contains CAPI implementation of Jitter Buf module.
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "capi_jitter_buf_i.h"

static capi_err_t capi_jitter_buf_process(capi_t *_pif, capi_stream_data_t *input[], capi_stream_data_t *output[]);

static capi_err_t capi_jitter_buf_end(capi_t *_pif);

static capi_err_t capi_jitter_buf_set_param(capi_t *                _pif,
                                            uint32_t                param_id,
                                            const capi_port_info_t *port_info_ptr,
                                            capi_buf_t *            params_ptr);

static capi_err_t capi_jitter_buf_get_param(capi_t *                _pif,
                                            uint32_t                param_id,
                                            const capi_port_info_t *port_info_ptr,
                                            capi_buf_t *            params_ptr);

static capi_err_t capi_jitter_buf_set_properties(capi_t *_pif, capi_proplist_t *props_ptr);

static capi_err_t capi_jitter_buf_get_properties(capi_t *_pif, capi_proplist_t *props_ptr);

capi_err_t capi_jitter_buf_init(capi_t *capi_ptr, capi_proplist_t *init_set_prop_ptr);

static const capi_vtbl_t vtbl = { capi_jitter_buf_process,        capi_jitter_buf_end,
                                  capi_jitter_buf_set_param,      capi_jitter_buf_get_param,
                                  capi_jitter_buf_set_properties, capi_jitter_buf_get_properties };

/*==============================================================================
   Local Function forward declaration
==============================================================================*/

/*==============================================================================
   Public Function Implementation
==============================================================================*/

capi_err_t capi_jitter_buf_get_static_properties(capi_proplist_t *init_set_prop_ptr, capi_proplist_t *static_prop_ptr)
{
   return capi_jitter_buf_get_properties(NULL, static_prop_ptr);
}

/*
  This function is used to query the static properties to create the CAPI.

  param[in] init_set_prop_ptr: Pointer to the initializing property list
  param[in, out] static_prop_ptr: Pointer to the static property list

  return: CAPI_EOK(0) on success else failure error code
 */
/*
  This function is used init the CAPI lib.

  param[in] capi_ptr: Pointer to the CAPI lib.
  param[in] init_set_prop_ptr: Pointer to the property list that needs to be
            initialized

  return: CAPI_EOK(0) on success else failure error code
 */
capi_err_t capi_jitter_buf_init(capi_t *capi_ptr, capi_proplist_t *init_set_prop_ptr)
{
   capi_err_t result = CAPI_EOK;
   if (NULL == capi_ptr || NULL == init_set_prop_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "NULL capi_ptr[%p], init_set_prop_ptr[%p]", capi_ptr, init_set_prop_ptr);
      return CAPI_EFAILED;
   }

   capi_jitter_buf_t *me_ptr = (capi_jitter_buf_t *)((capi_ptr));

   memset(me_ptr, 0, sizeof(capi_jitter_buf_t));

   me_ptr->vtbl = &vtbl;

   /* The module is disabled only based on set param */
   me_ptr->event_config.process_state = TRUE;

   // default is to mark input non-triggerable optional.
   me_ptr->input_buffer_mode = JBM_BUFFER_INPUT_AT_OUTPUT_TRIGGER;

   /* Set the init properties. */
   result = capi_jitter_buf_set_properties((capi_t *)me_ptr, init_set_prop_ptr);

   /* Intialize drift accumulator handle. which will be shared with SS module. */
   result |= capi_jitter_buf_init_out_drift_info(&me_ptr->drift_info, jitter_buf_imcl_read_acc_out_drift);

   return result;
}

/*==============================================================================
   Local Function Implementation
==============================================================================*/

static capi_err_t capi_jitter_buf_set_properties(capi_t *capi_ptr, capi_proplist_t *proplist_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   if ((NULL == capi_ptr) || (NULL == proplist_ptr))
   {
      AR_MSG(DBG_ERROR_PRIO, "jitter_buf: Set property failed. received bad pointer in set_property");
      return CAPI_EFAILED;
   }

   capi_jitter_buf_t *me_ptr = (capi_jitter_buf_t *)capi_ptr;

   capi_prop_t *prop_ptr = proplist_ptr->prop_ptr;
   if (NULL == prop_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "jitter_buf: Set property failed. received bad pointer in prop_ptr");
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
                      "jitter_buf: CAPI V2 FAILED Set Property id 0x%x Bad param size %u",
                      (uint32_t)prop_ptr[i].id,
                      payload_ptr->actual_data_len);
               return CAPI_ENEEDMORE;
            }
            capi_port_num_info_t *data_ptr = (capi_port_num_info_t *)payload_ptr->data_ptr;

            if (data_ptr->num_input_ports != CAPI_JITTER_BUF_MAX_INPUT_PORTS ||
                data_ptr->num_output_ports != CAPI_JITTER_BUF_MAX_OUTPUT_PORTS)
            {
               AR_MSG(DBG_ERROR_PRIO, "jitter_buf: Set property failed. Num port info is not valid.");
               return CAPI_EFAILED;
            }

            break;
         }
         case CAPI_HEAP_ID:
         {
            if (payload_ptr->actual_data_len < sizeof(capi_heap_id_t))
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "jitter_buf:  Set Property failed. id= 0x%x Bad param size %u",
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
                      "jitter_buf: Set Property Failed. id= 0x%x Bad param size %u",
                      (uint32_t)prop_ptr[i].id,
                      payload_ptr->actual_data_len);
               return CAPI_ENEEDMORE;
            }

            capi_event_callback_info_t *data_ptr = (capi_event_callback_info_t *)payload_ptr->data_ptr;

            me_ptr->event_cb_info.event_cb      = data_ptr->event_cb;
            me_ptr->event_cb_info.event_context = data_ptr->event_context;

            break;
         }
         case CAPI_ALGORITHMIC_RESET:
         {
            /* If buffer is not configured and created nothing to reset*/
            if (NULL == me_ptr->driver_hdl.stream_buf || 0 == me_ptr->frame_duration_in_us)
            {
               break;
            }

            jitter_buf_driver_t *drv_ptr = &me_ptr->driver_hdl;

            uint32_t read_offset = 0; // set the unread_data to 0.

            spf_circ_buf_read_adjust(drv_ptr->reader_handle, read_offset, &read_offset);

            me_ptr->first_frame_written = FALSE;

            me_ptr->settlement_time_done = FALSE;

            AR_MSG(DBG_HIGH_PRIO, "jitter_buf: Reset!");

            break;
         }
         case CAPI_INPUT_MEDIA_FORMAT_V2:
         {
            if (NULL == payload_ptr->data_ptr)
            {
               AR_MSG(DBG_ERROR_PRIO, "jitter_buf: Set property id 0x%lx, received null buffer", prop_ptr[i].id);
               return CAPI_EFAILED;
            }

            if (!prop_ptr[i].port_info.is_valid)
            {
               AR_MSG(DBG_ERROR_PRIO, "jitter_buf: Media format port info is invalid");
               return CAPI_ENEEDMORE;
            }

            if (payload_ptr->actual_data_len <
                sizeof(capi_set_get_media_format_t) + sizeof(capi_standard_data_format_v2_t))
            {
               AR_MSG(DBG_ERROR_PRIO, "jitter_buf: Invalid media format size %d", payload_ptr->actual_data_len);
               return CAPI_ENEEDMORE;
            }

            /* Handle media format if media format is not received yet or if it has changed. */
            if (FALSE == me_ptr->is_input_mf_received ||
                !capi_cmn_media_fmt_equal(&me_ptr->operating_mf, (capi_media_fmt_v2_t *)payload_ptr->data_ptr))
            {
               /* Validate and cache the media format */
               if (CAPI_EOK != (capi_result = capi_jitter_buf_vaildate_and_cache_input_mf(me_ptr, payload_ptr)))
               {
                  AR_MSG(DBG_ERROR_PRIO, "jitter_buf: Received invalid input media format.");
                  return CAPI_EFAILED;
               }

               /* Set / modify the circular buffer media format */
               spf_circ_buf_set_media_format(me_ptr->driver_hdl.writer_handle,
                                             (capi_media_fmt_v2_t *)payload_ptr->data_ptr,
                                             me_ptr->frame_duration_in_us);

               ar_result_t ar_result = AR_EOK;

               /* Calibrate media format related entities for the driver */
               if (AR_EOK != (ar_result = jitter_buf_calibrate_driver(me_ptr)))
               {
                  AR_MSG(DBG_ERROR_PRIO, "jitter_buf: Failed calibrating the driver.");
               }
            }

            /* Raise output media format */
            capi_result |= capi_jitter_buf_raise_output_mf_event(me_ptr, (capi_media_fmt_v2_t *)payload_ptr->data_ptr);

            /* TODO: Pending - Check the values */
            /* Raise kpps and be event */
            capi_result |= capi_jitter_buf_raise_kpps_bw_event(me_ptr);

            break;
         }
         default:
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "jitter_buf: Set property failed. received un-supported prop_id 0x%x",
                   (uint32_t)prop_ptr[i].id);
            return CAPI_EUNSUPPORTED;
         }
      } /* Outer switch - Generic CAPI Properties */
   }    /* Loop all properties */

   return capi_result;
}

static capi_err_t capi_jitter_buf_get_properties(capi_t *capi_ptr, capi_proplist_t *proplist_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   uint32_t   i;

   if (NULL == proplist_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "jitter_buf: Get properties received null arguments");
      return CAPI_EBADPARAM;
   }

   capi_prop_t *prop_ptr = proplist_ptr->prop_ptr;

   uint32_t fwk_extn_ids[JITTER_BUF_NUM_FRAMEWORK_EXTENSIONS] = { 0 };
   fwk_extn_ids[0]                                            = FWK_EXTN_MULTI_PORT_BUFFERING;
   fwk_extn_ids[1]                                            = FWK_EXTN_TRIGGER_POLICY;

   capi_basic_prop_t mod_prop;
   mod_prop.init_memory_req    = sizeof(capi_jitter_buf_t);
   mod_prop.stack_size         = CAPI_JITTER_BUF_MODULE_STACK_SIZE;
   mod_prop.num_fwk_extns      = JITTER_BUF_NUM_FRAMEWORK_EXTENSIONS;
   mod_prop.fwk_extn_ids_arr   = fwk_extn_ids;
   mod_prop.is_inplace         = FALSE; /* not capable of in-place processing of data */
   mod_prop.req_data_buffering = TRUE;  /* Require data buffering. */
   mod_prop.max_metadata_size  = 0;     /* Not Applicable*/

   /* This module requires data buffering. Consider the case when decoder raises worst threshold 16k but sends
    * only 8k per call. The frame size set based on actual data len will be 8k. Which means jitter buffer consumes
    * only 8k at a time. If 16k is buffered up due to process not being called as data trigger condition is not met
    * even then circular buffer can still consume input buffer of only 8k at a time.*/

   capi_result = capi_cmn_get_basic_properties(proplist_ptr, &mod_prop);
   if (CAPI_EOK != capi_result)
   {
      AR_MSG(DBG_ERROR_PRIO, "jitter_buf: Get properties failed with result %lu", capi_result);
      return capi_result;
   }

   /* iterating over the properties */
   for (i = 0; i < proplist_ptr->props_num; i++)
   {
      capi_buf_t *payload_ptr = &prop_ptr[i].payload;

      switch (prop_ptr[i].id)
      {
         case CAPI_INIT_MEMORY_REQUIREMENT:
         case CAPI_STACK_SIZE:
         case CAPI_IS_INPLACE:
         case CAPI_NUM_NEEDED_FRAMEWORK_EXTENSIONS:
         case CAPI_NEEDED_FRAMEWORK_EXTENSIONS:
         case CAPI_OUTPUT_MEDIA_FORMAT_SIZE:
         case CAPI_REQUIRES_DATA_BUFFERING:
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
                  case INTF_EXTN_METADATA:
                  case INTF_EXTN_PROP_IS_RT_PORT_PROPERTY:
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
            AR_MSG(DBG_ERROR_PRIO, "jitter_buf: Unknown Prop[%d]", prop_ptr[i].id);
            capi_result |= CAPI_EUNSUPPORTED;
            break;
         }
      } // switch
   }
   return capi_result;
}

/*------------------------------------------------------------------------
  Function name: capi_jitter_buf_set_param
  Sets either a parameter value or a parameter structure containing multiple
  parameters. In the event of a failure, the appropriate error code is
  returned.
 * -----------------------------------------------------------------------*/
static capi_err_t capi_jitter_buf_set_param(capi_t *                capi_ptr,
                                            uint32_t                param_id,
                                            const capi_port_info_t *port_info_ptr,
                                            capi_buf_t *            params_ptr)
{
   capi_err_t result = CAPI_EOK;
   if (NULL == capi_ptr || NULL == params_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "jitter_buf: Set param failed. received bad property pointer for param_id property, 0x%x",
             param_id);
      return CAPI_EFAILED;
   }

   capi_jitter_buf_t *me_ptr = (capi_jitter_buf_t *)((capi_ptr));

   switch (param_id)
   {
      case PARAM_ID_JITTER_BUF_CONFIG:
      {
         if (params_ptr->actual_data_len < sizeof(param_id_jitter_buf_config_t))
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "jitter_buf: Invalid payload size for param_id=0x%lx actual_data_len=%lu  ",
                   param_id,
                   params_ptr->actual_data_len);
            result = CAPI_ENEEDMORE;
            break;
         }

         param_id_jitter_buf_config_t *cfg_ptr = (param_id_jitter_buf_config_t *)params_ptr->data_ptr;

         /* copy configuration */
         me_ptr->jitter_allowance_in_ms = cfg_ptr->jitter_allowance_in_ms;

         /* Set the param and calibrate the driver */
         result = capi_jitter_buf_set_size(me_ptr, FALSE /* NOT DEBUG PARAM */);

         break;
      }
      case PARAM_ID_JITTER_BUF_SIZE_CONFIG:
      {
         if (params_ptr->actual_data_len < sizeof(param_id_jitter_buf_size_config_t))
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "jitter_buf: Invalid payload size for param_id=0x%lx actual_data_len=%lu  ",
                   param_id,
                   params_ptr->actual_data_len);
            result = CAPI_ENEEDMORE;
            break;
         }

         param_id_jitter_buf_size_config_t *cfg_ptr = (param_id_jitter_buf_size_config_t *)params_ptr->data_ptr;

         /* copy configuration */
         me_ptr->debug_size_ms = cfg_ptr->jitter_allowance_in_ms;

         /* Set param and calibrate driver */
         result = capi_jitter_buf_set_size(me_ptr, FALSE);

         break;
      }
      case PARAM_ID_JITTER_BUF_SETTLING_TIME:
      {
         if (params_ptr->actual_data_len < sizeof(param_id_jitter_buf_settling_time_t))
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "jitter_buf: Invalid payload size for param_id=0x%lx actual_data_len=%lu  ",
                   param_id,
                   params_ptr->actual_data_len);
            result = CAPI_ENEEDMORE;
            break;
         }

         param_id_jitter_buf_settling_time_t *cfg_ptr = (param_id_jitter_buf_settling_time_t *)params_ptr->data_ptr;

         /* set  configuration */
         me_ptr->drift_settlement_in_ms = cfg_ptr->drift_settlement_in_ms;

         break;
      }

      case PARAM_ID_JITTER_BUF_INPUT_BUFFER_MODE:
      {

         if (params_ptr->actual_data_len < sizeof(param_id_jitter_buf_input_buffer_mode_t))
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "jitter_buf: Invalid payload size for param_id=0x%lx actual_data_len=%lu  ",
                   param_id,
                   params_ptr->actual_data_len);
            result = CAPI_ENEEDMORE;
            break;
         }

         param_id_jitter_buf_input_buffer_mode_t *cfg_ptr =
            (param_id_jitter_buf_input_buffer_mode_t *)params_ptr->data_ptr;

         /* set  configuration */
         me_ptr->input_buffer_mode = (JBM_BUFFER_INPUT_AT_INPUT_TRIGGER == cfg_ptr->input_buffer_mode)
                                        ? JBM_BUFFER_INPUT_AT_INPUT_TRIGGER
                                        : JBM_BUFFER_INPUT_AT_OUTPUT_TRIGGER;

         AR_MSG(DBG_HIGH_PRIO,
                "jitter_buf: input buffering mode %hu [1: BUFFER_AT_INPUT_TRIGGER, 0: BUFFER_AT_OUTPUT_TRIGGER] ",
                me_ptr->input_buffer_mode);

         result = capi_jitter_buf_change_trigger_policy(me_ptr);
         break;
      }
      case INTF_EXTN_PARAM_ID_IMCL_PORT_OPERATION:
      {
         result = capi_jitter_buf_handle_imcl_port_operation(me_ptr, params_ptr);
         break;
      }
      case FWK_EXTN_PARAM_ID_TRIGGER_POLICY_CB_FN:
      {
         if (NULL == params_ptr->data_ptr)
         {
            AR_MSG(DBG_ERROR_PRIO, "jitter_buf: Set param id 0x%lx, received null buffer", param_id);
            result |= CAPI_EBADPARAM;
            break;
         }

         if (params_ptr->actual_data_len < sizeof(fwk_extn_param_id_trigger_policy_cb_fn_t))
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "jitter_buf: Invalid payload size for trigger policy %d",
                   params_ptr->actual_data_len);
            return CAPI_ENEEDMORE;
         }

         fwk_extn_param_id_trigger_policy_cb_fn_t *payload_ptr =
            (fwk_extn_param_id_trigger_policy_cb_fn_t *)params_ptr->data_ptr;
         me_ptr->policy_chg_cb = *payload_ptr;

         /* This module is by default driven based on availability of output */
         result = capi_jitter_buf_change_trigger_policy(me_ptr);

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
      case INTF_EXTN_PARAM_ID_IS_RT_PORT_PROPERTY:
      {
         if (NULL == params_ptr->data_ptr ||
             sizeof(intf_extn_param_id_is_rt_port_property_t) > params_ptr->actual_data_len)
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "Param id 0x%lx Bad param size %lu",
                   (uint32_t)param_id,
                   params_ptr->actual_data_len);
            return CAPI_ENEEDMORE;
         }

         intf_extn_param_id_is_rt_port_property_t *data_ptr =
            (intf_extn_param_id_is_rt_port_property_t *)(params_ptr->data_ptr);

         capi_buf_t                               event_payload;
         intf_extn_param_id_is_rt_port_property_t event = *data_ptr;
         event_payload.data_ptr                         = (int8_t *)&event;
         event_payload.actual_data_len = event_payload.max_data_len = sizeof(event);

         if (JBM_BUFFER_INPUT_AT_INPUT_TRIGGER == me_ptr->input_buffer_mode)
         {
            // Reflect the property because input and output are independently triggered based on upstream and downstream
            // respectively.
            // US_RT as DS_RT and DS_RT as US_RT.
            capi_cmn_raise_data_to_dsp_svc_event(&me_ptr->event_cb_info,
                                                 INTF_EXTN_EVENT_ID_IS_RT_PORT_PROPERTY,
                                                 &event_payload);
         }
#if 0
         else // JBM_BUFFER_INPUT_AT_OUTPUT_TRIGGER
         {
            if (!data_ptr->is_input)
            {
               // Reflect the DS_RT as US_RT on output port.
               capi_cmn_raise_data_to_dsp_svc_event(&me_ptr->event_cb_info,
                                                    INTF_EXTN_EVENT_ID_IS_RT_PORT_PROPERTY,
                                                    &event_payload);

               // Propagate the DS_RT as DS_RT on the input port
               event.is_input = TRUE;
               capi_cmn_raise_data_to_dsp_svc_event(&me_ptr->event_cb_info,
                                                    INTF_EXTN_EVENT_ID_IS_RT_PORT_PROPERTY,
                                                    &event_payload);
            }
         }
#endif
         break;
      }
      default:
      {
         AR_MSG(DBG_ERROR_PRIO, "jitter_buf: Unsupported Param id= 0x%x \n", param_id);
         result = CAPI_EUNSUPPORTED;
      }
   }

   AR_MSG(DBG_HIGH_PRIO, "jitter_buf: Set param= 0x%x done with result= 0x%x", param_id, result);
   return result;
}

/*------------------------------------------------------------------------
  Function name: capi_jitter_buf_get_param
  Gets either a parameter value or a parameter structure containing multiple
  parameters. In the event of a failure, the appropriate error code is
  returned.
 * -----------------------------------------------------------------------*/
static capi_err_t capi_jitter_buf_get_param(capi_t *                capi_ptr,
                                            uint32_t                param_id,
                                            const capi_port_info_t *port_info_ptr,
                                            capi_buf_t *            params_ptr)
{
   capi_err_t result = CAPI_EOK;
   if (NULL == capi_ptr || NULL == params_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "jitter_buf: FAILED received bad property pointer for param_id property, 0x%x", param_id);
      return CAPI_EFAILED;
   }

   switch (param_id)
   {
      default:
      {
         AR_MSG(DBG_HIGH_PRIO, "jitter_buf: Unsupported Param id= 0x%x \n", param_id);
         result = CAPI_EUNSUPPORTED;
      }
      break;
   }

   AR_MSG(DBG_HIGH_PRIO, "jitter_buf: Get param done for param id= 0x%x, result= 0x%x", param_id, result);
   return result;
}

/*------------------------------------------------------------------------
  Function name: capi_jitter_buf_process
  Processes an input buffer and generates an output buffer.
 * -----------------------------------------------------------------------*/
static capi_err_t capi_jitter_buf_process(capi_t *capi_ptr, capi_stream_data_t *input[], capi_stream_data_t *output[])
{
   capi_err_t result = CAPI_EOK;

   if (NULL == capi_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "jitter_buf: received bad property pointer");
      return CAPI_EFAILED;
   }

   if ((NULL == input) || (NULL == output))
   {
      return result;
   }

   capi_jitter_buf_t *me_ptr = (capi_jitter_buf_t *)capi_ptr;

   if(!me_ptr->is_input_mf_received || !me_ptr->jitter_allowance_in_ms || me_ptr->is_disabled_by_failure)
   {
      AR_MSG(DBG_ERROR_PRIO, "jitter_buf: not buf size %d / mf %d, is_module_disabled_by_fatal_failure %d",me_ptr->jitter_allowance_in_ms, me_ptr->is_input_mf_received, me_ptr->is_disabled_by_failure);
      if(me_ptr->is_input_mf_received && input[0]->buf_ptr)
      {
         for (int i = 0; i < input[0]->bufs_num; i++)
         {
            // marking as input not consumed
            input[0]->buf_ptr[i].actual_data_len = 0;
         }
      }
      return CAPI_EOK;
   }

   if (0 == me_ptr->frame_duration_in_us)
   {
      if (0 == input[0]->buf_ptr->actual_data_len)
      {
         /* The first time when decoder sends data starts the process for jitter buffer.
          * Circ buf is set up and zeros are filled based on this. So for the first time
          * decoder produces data the downstream buffer will be available since we do not
          * fill it with anything till jitter buffer process starts. This also makes sure that
          * we capture the exact actual data len from the decoder. */

         AR_MSG(DBG_LOW_PRIO, "jitter_buf: frame duration not set yet.");
         return CAPI_EOK;
      }

      /* TODO: Pending - check calculation */
      me_ptr->frame_duration_in_bytes = input[0]->buf_ptr->actual_data_len;

       if (CAPI_EOK != (result = ar_result_to_capi_err(jitter_buf_calibrate_driver(me_ptr))))
       {
          AR_MSG(DBG_ERROR_PRIO, "jitter_buf: Failed calibrating the driver with error = %lx", result);

          for (int i = 0; i < input[0]->bufs_num; i++)
          {
             // marking as input not consumed
             input[0]->buf_ptr[i].actual_data_len = 0;
          }

          return result;
       }
   }

   bool_t is_output_written = FALSE;

   /* mode0: input buffer at output trigger
    * Module is driven based on the reader availability. So when output is available we
    * first read it and then only write input if available. This module does not tolerate dropping
    * input data at any cost.
    *
    * mode1: input buffer at input trigger
    * Module is driven based on either reader or writer availability. */

   // if input and output are independently triggered then always prefills zeros when buffer is empty.
   // This is to ensure that zeros are pushed in the beginning and not in between of valid input data.
   if (JBM_BUFFER_INPUT_AT_INPUT_TRIGGER == me_ptr->input_buffer_mode)
   {
      result |= jitter_buf_check_fill_zeros(me_ptr);
      if (result == AR_EFAILED)
      {
         return CAPI_EFAILED;
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

         if (!me_ptr->first_frame_written)
         {
            /* If first call we add delay amount of zeros here and then read it. If we reach here then
             * we have the first frame available since we would have to calibrate before reaching here  */
            result |= jitter_buf_check_fill_zeros(me_ptr);

            /* We read the filled zeros here */
            result |= jitter_buf_stream_read(me_ptr, output[0]);
            if (result == AR_EFAILED)
            {
               /* If we do not read into output we do not write into it either
                * This works as a queue to prevent overruns in case buffer is
                * almost full */
               return CAPI_EFAILED;
            }
         }
         else
         {
            bool_t is_buf_empty = FALSE;
            spf_circ_buf_driver_is_buffer_empty(me_ptr->driver_hdl.reader_handle, &is_buf_empty);

            if (!is_buf_empty)
            {
               /* At this point buffer is not empty - it may have pcm data / was filled with zeros so we
                * have data to read into output buffer.*/
               result |= jitter_buf_stream_read(me_ptr, output[0]);
               if (result == AR_EFAILED)
               {
                  /* If we do not read into output we do not write into it either
                   * This works as a queue to prevent overruns in case buffer is
                   * almost full */
                  return CAPI_EFAILED;
               }
               is_output_written = TRUE;
            }
            else // this will never happen if input is buffered at input triggered because we already prefilled zeros
                 // before reading.
            {
               /* At this point first frame was already written and first delay zeros already filled. If
                * we find empty buffer here and also nothing at input then we dont have data and fill delay
                * ms amount of zeros into the buffer since we have drained it. So we check if there is input
                * we can send to output - if not we have drained and send zeros.*/

               AR_MSG(DBG_HIGH_PRIO, "Buf empty at read so attempting to write before read ");
            }
         }
      }
   }

   if (input)
   {
      /* Check if the ports stream buffers are valid */
      if (NULL == input[0])
      {
         AR_MSG(DBG_HIGH_PRIO, "Input buffers not available ");
      }
      else
      {
         /* Write data into the jitter buffer one frame (assuming
          * one frame was successfully read into the output) */
         result = jitter_buf_stream_write(me_ptr, input[0]);
      }
   }

   if (!is_output_written)
   {
      /* If we have reached this point then jitter buffer was empty when trying to read.
       * We check if in the write cycle it was filled to send data out. If not the jitter
       * buffer is drained and we add zeros to the buffer and read it into the output. */
      result |= jitter_buf_check_fill_zeros(me_ptr);

      result |= jitter_buf_stream_read(me_ptr, output[0]);
      if (result == AR_EFAILED)
      {
         /* If we do not read into output we do not write into it either
          * This works as a queue to prevent overruns in case buffer is
          * almost full */
         return CAPI_EFAILED;
      }
   }

   return result;
}

/*------------------------------------------------------------------------
  Function name: capi_jitter_buf_end
  Returns the library to the uninitialized state and frees the memory that
  was allocated by Init(). This function also frees the virtual function
  table.
 * -----------------------------------------------------------------------*/
static capi_err_t capi_jitter_buf_end(capi_t *capi_ptr)
{
   capi_err_t result = CAPI_EOK;

   if (NULL == capi_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "jitter_buf: received null pointer, %p", capi_ptr);
      return CAPI_EFAILED;
   }
   capi_jitter_buf_t *me_ptr = (capi_jitter_buf_t *)((capi_ptr));

   /* deinit the jitter buf driver, destroys the circular buffer. */
   jitter_buf_driver_deinit(me_ptr);

   result |= capi_jitter_buf_deinit_out_drift_info(&me_ptr->drift_info);

   me_ptr->vtbl = NULL;
   return result;
}
