/**
 *   \file capi_trm.c
 *   \brief
 *        This file contains CAPI implementation of Timed Renderer Module
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "trm_api.h"
#include "capi_trm.h"
#include "capi_trm_utils.h"
#include "posal_memory.h"
#include "posal_timer.h"

/*==============================================================================
   Local Defines
==============================================================================*/
//#define TRM_DEBUG // Flag to enable TRM debug prints

/*==============================================================================
   Static functions
==============================================================================*/
static capi_err_t capi_trm_raise_events(capi_trm_t *me_ptr);

/**
 * Function to raise various events of the trm module. KPPS, bw, and process state
 * are static. Algo delay isn't sent until frame size is known.
 */
static capi_err_t capi_trm_raise_events(capi_trm_t *me_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   bool_t     ENABLED     = TRUE; // TRM is always enabled.
   if (NULL == me_ptr->event_cb_info.event_cb)
   {
      AR_MSG(DBG_ERROR_PRIO, "Event callback is not set. Unable to raise events!");
      CAPI_SET_ERROR(capi_result, CAPI_EUNSUPPORTED);
      return capi_result;
   }

   capi_result |= capi_cmn_update_kpps_event(&me_ptr->event_cb_info, CAPI_TRM_KPPS);
   capi_result |= capi_cmn_update_bandwidth_event(&me_ptr->event_cb_info, CAPI_TRM_CODE_BW, CAPI_TRM_DATA_BW);
   capi_result |= capi_cmn_update_process_check_event(&me_ptr->event_cb_info, ENABLED);
   return capi_result;
}

/*==============================================================================
   Local Function forward declaration
==============================================================================*/
capi_err_t capi_trm_process_get_properties(capi_trm_t *me_ptr, capi_proplist_t *proplist_ptr)
{
   capi_err_t capi_result = CAPI_EOK;

   if (NULL == proplist_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi timed renderer: Get property received null property array");
      return CAPI_EBADPARAM;
   }

   uint32_t fwk_extn_ids_arr[] = { FWK_EXTN_TRIGGER_POLICY, FWK_EXTN_CONTAINER_FRAME_DURATION, FWK_EXTN_DM };

   capi_basic_prop_t mod_prop;
   mod_prop.init_memory_req    = align_to_8_byte(sizeof(capi_trm_t));
   mod_prop.stack_size         = TRM_STACK_SIZE;
   mod_prop.num_fwk_extns      = sizeof(fwk_extn_ids_arr) / sizeof(fwk_extn_ids_arr[0]);
   mod_prop.fwk_extn_ids_arr   = fwk_extn_ids_arr;
   mod_prop.is_inplace         = FALSE;
   mod_prop.req_data_buffering = TRUE;
   mod_prop.max_metadata_size  = 0;

   capi_result = capi_cmn_get_basic_properties(proplist_ptr, &mod_prop);
   if (CAPI_EOK != capi_result)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "capi timed renderer: Get common basic properties failed with capi_result %lu",
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
               AR_MSG(DBG_ERROR_PRIO, "capi timed renderer: pif is NULL for get OUTPUT MF");
               capi_result |= CAPI_EBADPARAM;
               break;
            }
            uint32_t ret_size = sizeof(capi_set_get_media_format_t) + sizeof(capi_standard_data_format_v2_t) +
                                (me_ptr->media_format.format.num_channels * sizeof(capi_channel_type_t));
            /* Validate the MF payload */
            if (payload_ptr->max_data_len < sizeof(capi_media_fmt_v2_t))
            {
               AR_MSG(DBG_ERROR_PRIO, "capi timed renderer: Invalid media format size %d", payload_ptr->max_data_len);

               capi_result |= CAPI_ENEEDMORE;
               break;
            }
            capi_media_fmt_v2_t *media_fmt_ptr = (capi_media_fmt_v2_t *)(payload_ptr->data_ptr);
            memscpy(media_fmt_ptr, ret_size, &me_ptr->media_format, ret_size);
            payload_ptr->actual_data_len = sizeof(capi_media_fmt_v2_t);
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
                  AR_MSG(DBG_ERROR_PRIO,
                         "capi timed renderer: CAPI_INTERFACE_EXTENSIONS invalid param size %lu",
                         payload_ptr->max_data_len);
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
                        case INTF_EXTN_PROP_IS_RT_PORT_PROPERTY:
                        case INTF_EXTN_DATA_PORT_OPERATION:
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
                            "capi timed renderer: CAPI_INTERFACE_EXTENSIONS intf_ext = 0x%lx, is_supported = %d",
                            curr_intf_extn_desc_ptr->id,
                            (int)curr_intf_extn_desc_ptr->is_supported);
                     curr_intf_extn_desc_ptr++;
                  }
               }
            }
            else
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "capi timed renderer: CAPI_INTERFACE_EXTENSIONS Bad param size %lu",
                      payload_ptr->max_data_len);
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
#ifdef TRM_DEBUG
            AR_MSG(DBG_HIGH_PRIO, "capi timed renderer: FAILED Get Property for 0x%x. Not supported.", prop_ptr[i].id);
#endif
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
capi_err_t capi_trm_get_static_properties(capi_proplist_t *init_set_prop_ptr, capi_proplist_t *static_prop_ptr)
{
   capi_err_t result = CAPI_EOK;
   if (NULL == static_prop_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "capi timed renderer: FAILED received bad property pointer for get "
             "property, %p",
             static_prop_ptr);
      return (CAPI_EFAILED);
   }

   result = capi_trm_process_get_properties((capi_trm_t *)NULL, static_prop_ptr);

   return result;
}

/*
  This function is used init the CAPI lib.

  param[in] capi_ptr: Pointer to the CAPI lib.
  param[in] init_set_prop_ptr: Pointer to the property list that needs to be
  initialized

  return: CAPI_EOK(0) on success else failure error code
 */
capi_err_t capi_trm_init(capi_t *capi_ptr, capi_proplist_t *init_set_prop_ptr)
{
   capi_err_t capi_result;

   if (NULL == capi_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi timed renderer: Init received bad pointer, %p", capi_ptr);
      return CAPI_EFAILED;
   }
   if (NULL == init_set_prop_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "capi timed renderer: FAILED received bad property pointer for get property, %p",
             init_set_prop_ptr);
      return CAPI_EFAILED;
   }

   capi_trm_t *me_ptr = (capi_trm_t *)(capi_ptr);

   memset(me_ptr, 0, sizeof(capi_trm_t));
   me_ptr->tgp_state = CAPI_TRM_TGP_PENDING;

   me_ptr->is_input_mf_received = FALSE;

   me_ptr->vtbl = capi_trm_get_vtbl();

   capi_cmn_init_media_fmt_v2(&me_ptr->media_format);
   capi_result = capi_cmn_set_basic_properties(init_set_prop_ptr, &me_ptr->heap_mem, &me_ptr->event_cb_info, NULL);
   if (CAPI_EOK != capi_result)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi timed renderer: Set basic properties failed with result %lu", capi_result);
   }

   capi_result |= capi_trm_raise_event_data_trigger_in_st_cntr(me_ptr);
   capi_result |= capi_trm_raise_events(me_ptr);
   capi_trm_clear_ttr(me_ptr);

   return capi_result;
}

/*==============================================================================
   Local Function Implementation
==============================================================================*/
capi_err_t capi_trm_set_properties(capi_t *capi_ptr, capi_proplist_t *proplist_ptr)
{
   capi_err_t capi_result = CAPI_EOK;

   if ((NULL == capi_ptr) || (NULL == proplist_ptr))
   {
      AR_MSG(DBG_ERROR_PRIO, "capi timed renderer: FAILED received bad pointer in set_property");
      return CAPI_EFAILED;
   }

   capi_trm_t *me_ptr = (capi_trm_t *)capi_ptr;

   capi_prop_t *prop_ptr = proplist_ptr->prop_ptr;
   if (NULL == prop_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi timed renderer: FAILED received bad pointer in prop_ptr");
      return CAPI_EFAILED;
   }

   capi_result = capi_cmn_set_basic_properties(proplist_ptr, NULL, &me_ptr->event_cb_info, TRUE);
   if (CAPI_EOK != capi_result)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi timed renderer: Set basic properties failed with result %lu", capi_result);
   }

   uint32_t i;

   for (i = 0; i < proplist_ptr->props_num; i++)
   {
      capi_buf_t *payload_ptr = &prop_ptr[i].payload;

      switch (prop_ptr[i].id)
      {
         case CAPI_ALGORITHMIC_RESET:
         {
            break;
         }
         case CAPI_CUSTOM_PROPERTY:
         {
            capi_custom_property_t *cust_prop_ptr = (capi_custom_property_t *)payload_ptr->data_ptr;
            switch (cust_prop_ptr->secondary_prop_id)
            {
               default:
               {
                  AR_MSG(DBG_ERROR_PRIO, "capi timed renderer: Unknown Sec Prop[%d]", cust_prop_ptr->secondary_prop_id);
                  capi_result |= CAPI_EUNSUPPORTED;
                  break;
               }
            } /* inner switch - CUSTOM Properties */
            break;
         }
         case CAPI_INPUT_MEDIA_FORMAT_V2:
         {
            /* Validate the MF payload */
            if (payload_ptr->actual_data_len <
                sizeof(capi_set_get_media_format_t) + sizeof(capi_standard_data_format_v2_t))
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "capi timed renderer: Invalid media format size %d",
                      payload_ptr->actual_data_len);
               capi_result |= CAPI_ENEEDMORE;
               break;
            }
            if (NULL == payload_ptr->data_ptr)
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "capi timed renderer: Set property id 0x%lx, received null buffer",
                      prop_ptr[i].id);
               capi_result |= CAPI_EBADPARAM;
               break;
            }
            capi_media_fmt_v2_t *media_fmt_ptr = (capi_media_fmt_v2_t *)(payload_ptr->data_ptr);
            // /* Validate data format, interleaving and num channels */
            if ((CAPI_RAW_COMPRESSED == media_fmt_ptr->header.format_header.data_format) ||
                (CAPI_MAX_CHANNELS_V2 < media_fmt_ptr->format.num_channels))
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "capi timed renderer: Unsupported Data format %lu or num_channels %lu",
                      media_fmt_ptr->header.format_header.data_format,
                      media_fmt_ptr->format.num_channels);
               capi_result |= CAPI_EBADPARAM;
               break;
            }
            uint32_t size_to_copy = sizeof(capi_set_get_media_format_t) + sizeof(capi_standard_data_format_v2_t) +
                                    (media_fmt_ptr->format.num_channels * sizeof(capi_channel_type_t));
            // accept as the operating Media format
            memscpy(&me_ptr->media_format, size_to_copy, media_fmt_ptr, payload_ptr->actual_data_len);
            me_ptr->is_input_mf_received = TRUE;

            AR_MSG(DBG_HIGH_PRIO,
                   "capi timed renderer: Received num_channels %lu,"
                   "bits_per_sample %lu,"
                   "sampling rate %lu",
                   media_fmt_ptr->format.num_channels,
                   media_fmt_ptr->format.bits_per_sample,
                   media_fmt_ptr->format.sampling_rate);

            capi_result |= capi_trm_check_alloc_held_input_buffer(me_ptr);

            capi_result |= capi_cmn_output_media_fmt_event_v2(&me_ptr->event_cb_info, &me_ptr->media_format, FALSE, i);
            break;
         }
         default:
         {
#ifdef TRM_DEBUG
            AR_MSG(DBG_ERROR_PRIO, "capi timed renderer: received bad prop_id 0x%x", (uint32_t)prop_ptr[i].id);
#endif

            capi_result |= CAPI_EUNSUPPORTED;
         }
      }
   }

   return capi_result;
}

capi_err_t capi_trm_get_properties(capi_t *capi_ptr, capi_proplist_t *proplist_ptr)
{
   capi_err_t rc = CAPI_EOK;
   if (NULL == capi_ptr || NULL == proplist_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi timed renderer: NULL capi_ptr[%p], proplist_ptr[%p]", capi_ptr, proplist_ptr);
      return CAPI_EFAILED;
   }

   capi_trm_t *me_ptr = (capi_trm_t *)capi_ptr;

   rc = capi_trm_process_get_properties(me_ptr, proplist_ptr);

   return rc;
}

capi_err_t capi_trm_set_param(capi_t *                capi_ptr,
                              uint32_t                param_id,
                              const capi_port_info_t *port_info_ptr,
                              capi_buf_t *            params_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   if (NULL == capi_ptr || NULL == params_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "capi timed renderer: FAILED received bad property pointer for "
             "param_id property, 0x%x",
             param_id);
      return (CAPI_EFAILED);
   }

   capi_trm_t *me_ptr = (capi_trm_t *)((capi_ptr));

   switch (param_id)
   {
      case FWK_EXTN_DM_PARAM_ID_CHANGE_MODE: //default to fixed out due to HW-EP at downstream.
      case FWK_EXTN_DM_PARAM_ID_SET_SAMPLES: //can be ignored as TRM only supported in GC
      case FWK_EXTN_DM_PARAM_ID_SET_MAX_SAMPLES: //can be ignored as TRM only supported in GC
         break;
      case INTF_EXTN_PARAM_ID_METADATA_HANDLER:
      {
         if (params_ptr->actual_data_len < sizeof(intf_extn_param_id_metadata_handler_t))
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "capi timed renderer: Param id 0x%lx Bad param size %lu",
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
      case INTF_EXTN_PARAM_ID_DATA_PORT_OPERATION:
      {
         if (NULL == params_ptr->data_ptr)
         {
            AR_MSG(DBG_ERROR_PRIO, "capi timed renderer: Set property id 0x%lx, received null buffer", param_id);
            capi_result |= CAPI_EBADPARAM;
            break;
         }
         if (params_ptr->actual_data_len < sizeof(intf_extn_data_port_operation_t))
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "capi timed renderer: Invalid payload size for port operation %d",
                   params_ptr->actual_data_len);
            capi_result |= CAPI_ENEEDMORE;
            break;
         }
         intf_extn_data_port_operation_t *data_ptr = (intf_extn_data_port_operation_t *)(params_ptr->data_ptr);
         if (params_ptr->actual_data_len <
             sizeof(intf_extn_data_port_operation_t) + (data_ptr->num_ports * sizeof(intf_extn_data_port_id_idx_map_t)))
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "capi timed renderer: Invalid payload size for port operation %d",
                   params_ptr->actual_data_len);
            capi_result |= CAPI_ENEEDMORE;
            break;
         }

         switch (data_ptr->opcode)
         {
            case INTF_EXTN_DATA_PORT_OPEN:
            {
               capi_trm_raise_rt_port_prop_event(me_ptr, data_ptr->is_input_port);
               break;
            }
            case INTF_EXTN_DATA_PORT_START:
            {
               break;
            }
            case INTF_EXTN_DATA_PORT_STOP:
            {
               if (data_ptr->is_input_port)
               {
                  AR_MSG(DBG_HIGH_PRIO, "capi timed renderer: Received data port stop");
                  capi_trm_clear_ttr(me_ptr);
               }
               break;
            }
            case INTF_EXTN_DATA_PORT_CLOSE:
            {
               if (data_ptr->is_input_port)
               {
                  AR_MSG(DBG_HIGH_PRIO, "capi timed renderer: Received data port stop/close");
                  me_ptr->is_input_mf_received = FALSE;
                  capi_trm_clear_ttr(me_ptr);
               }
               break;
            }
            default:
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "capi timed renderer: Port operation - Unsupported opcode: %lu",
                      data_ptr->opcode);
               CAPI_SET_ERROR(capi_result, CAPI_EUNSUPPORTED);
               break;
            }
         }

         break;
      } // CAPI_PORT_OPERATION
      case FWK_EXTN_PARAM_ID_TRIGGER_POLICY_CB_FN:
      {
         if (NULL == params_ptr->data_ptr)
         {
            AR_MSG(DBG_ERROR_PRIO, "capi timed renderer: Param id 0x%lx received null buffer", (uint32_t)param_id);
            capi_result |= CAPI_EBADPARAM;
            break;
         }

         if (params_ptr->actual_data_len < sizeof(fwk_extn_param_id_trigger_policy_cb_fn_t))
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "capi timed renderer: Param id 0x%lx Bad param size %lu",
                   (uint32_t)param_id,
                   params_ptr->actual_data_len);
            capi_result |= CAPI_ENEEDMORE;
            break;
         }

         fwk_extn_param_id_trigger_policy_cb_fn_t *payload_ptr =
            (fwk_extn_param_id_trigger_policy_cb_fn_t *)params_ptr->data_ptr;
         me_ptr->tgp.tg_policy_cb = *payload_ptr;

         // Trigger policy is constant so update it as soon as we get the callback.
         capi_result |= capi_trm_update_tgp_before_sync(me_ptr);
         break;
      }
      case FWK_EXTN_PARAM_ID_CONTAINER_FRAME_DURATION:
      {
         if (params_ptr->actual_data_len < sizeof(fwk_extn_param_id_container_frame_duration_t))
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "capi timed renderer: Invalid payload size for param_id=0x%lx actual_data_len=%lu  ",
                   param_id,
                   params_ptr->actual_data_len);
            capi_result |= CAPI_ENEEDMORE;
            break;
         }

         fwk_extn_param_id_container_frame_duration_t *cfg_ptr = (fwk_extn_param_id_container_frame_duration_t *)params_ptr->data_ptr;

         // It is important not to handle redundant events, since check_alloc_held_input_buffer will reallocate the buffer and flush any
         // buffered data.
         if(me_ptr->frame_size_us != cfg_ptr->duration_us)
         {
            me_ptr->frame_size_us = cfg_ptr->duration_us;

            AR_MSG(DBG_HIGH_PRIO, "capi timed renderer: Nominal frame duration = %lu is set.", me_ptr->frame_size_us);

            capi_result |= capi_trm_check_alloc_held_input_buffer(me_ptr);

            // One frame of algo delay is for prebuffer.
            // Extra half frame is an estimation of partial hold duration. This minimizes worst case error to
            // a half frame. True partial hold duration is only known during process ctx rendering decision so don't
            // want to report algo delay change during runtime.
            capi_result |= capi_cmn_update_algo_delay_event(&me_ptr->event_cb_info, 1.5 * me_ptr->frame_size_us);
         }

         break;
      }
      default:
      {
         AR_MSG(DBG_HIGH_PRIO, "capi timed renderer: Unsupported Param id ::0x%x \n", param_id);
         capi_result = CAPI_EUNSUPPORTED;
      }
      break;
   }

   return capi_result;
}

/*------------------------------------------------------------------------
  Function name: capi_trm_get_param
  Gets either a parameter value or a parameter structure containing multiple
  parameters. In the event of a failure, the appropriate error code is
  returned.
 * -----------------------------------------------------------------------*/
capi_err_t capi_trm_get_param(capi_t *                capi_ptr,
                              uint32_t                param_id,
                              const capi_port_info_t *port_info_ptr,
                              capi_buf_t *            params_ptr)
{
   capi_err_t result = CAPI_EOK;
   if (NULL == capi_ptr || NULL == params_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "capi timed renderer: FAILED received bad property pointer for "
             "param_id property, 0x%x",
             param_id);
      return (CAPI_EFAILED);
   }

   // capi_trm_t *me_ptr = (capi_trm_t *)((capi_ptr));

   switch (param_id)
   {
      default:
      {
         AR_MSG(DBG_HIGH_PRIO, "capi timed renderer: Unsupported Param id ::0x%x \n", param_id);
         result = CAPI_EUNSUPPORTED;
      }
      break;
   }
   return result;
}

/*------------------------------------------------------------------------
  Function name: capi_trm_end
  Returns the library to the uninitialized state and frees the memory that
  was allocated by Init(). This function also frees the virtual function
  table.
 * -----------------------------------------------------------------------*/
capi_err_t capi_trm_end(capi_t *capi_ptr)
{
   capi_err_t result = CAPI_EOK;
   if (NULL == capi_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi timed renderer: end received bad pointer, %p", capi_ptr);
      return (CAPI_EFAILED);
   }

   AR_MSG(DBG_HIGH_PRIO, "capi timed renderer: end");

   capi_trm_t *me_ptr = (capi_trm_t *)((capi_ptr));

   if (me_ptr->held_input_buf.frame_ptr)
   {
      for (uint32_t i = 0; i < NUM_FRAMES_IN_LOCAL_BUFFER; i++)
      {
         result |= capi_trm_free_held_metadata(me_ptr, &(me_ptr->held_input_buf.frame_ptr[i]), NULL, TRUE);
      }
   }

   capi_trm_dealloc_held_input_buffer(me_ptr);

   me_ptr->vtbl = NULL;
   return result;
}
