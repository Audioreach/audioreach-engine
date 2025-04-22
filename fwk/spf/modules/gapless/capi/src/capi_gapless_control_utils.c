/**
 * \file capi_gapless_control_utils.c
 * \brief
 *     Implementation of utility functions for capi control handling (set params, set properties, etc).
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "capi_gapless_i.h"

static capi_err_t capi_gapless_set_param_port_op(capi_gapless_t *me_ptr, capi_buf_t *payload_ptr);
static capi_err_t capi_gapless_set_prop_port_num_info(capi_gapless_t *me_ptr, capi_prop_t *prop_ptr);
static capi_err_t capi_gapless_set_prop_input_media_fmt(capi_gapless_t *me_ptr, capi_prop_t *prop_ptr);
static capi_err_t capi_gapless_set_prop_register_event_to_dsp_client_v2(capi_gapless_t *me_ptr, capi_prop_t *prop_ptr);

static capi_err_t gapless_check_update_in_port_trigger_policy(capi_gapless_t *me_ptr,
                                                              uint32_t        in_index,
                                                              bool_t *        policy_needs_update_ptr);
static capi_err_t gapless_check_update_out_port_trigger_policy(capi_gapless_t *me_ptr, bool_t *policy_needs_update_ptr);

#ifdef CAPI_GAPLESS_DEBUG
static void gapless_print_trigger_policy(capi_gapless_t *                  me_ptr,
                                         fwk_extn_port_nontrigger_group_t *nontriggerable_ports_ptr,
                                         fwk_extn_port_trigger_policy_t    policy,
                                         uint32_t                          num_groups,
                                         fwk_extn_port_trigger_group_t *   triggerable_groups_ptr)
{
   AR_MSG(DBG_MED_PRIO, "Printing gapless trigger policy...");

   AR_MSG(DBG_MED_PRIO, "Internal port states...");

   capi_gapless_out_port_t *out_port_ptr = capi_gapless_get_out_port(me_ptr);

   if (!out_port_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "Error getting output port");
      return;
   }

   AR_MSG(DBG_MED_PRIO,
          "    Output port idx %ld id 0x%lx: 0x%lx",
          out_port_ptr->cmn.index,
          out_port_ptr->cmn.port_id,
          out_port_ptr->cmn.port_trigger_policy);

   for (uint32_t i = 0; i < me_ptr->num_in_ports; i++)
   {
      capi_gapless_in_port_t *in_port_ptr = &(me_ptr->in_ports[i]);

      AR_MSG(DBG_MED_PRIO,
             "    Input port idx %ld id 0x%lx: 0x%lx",
             in_port_ptr->cmn.index,
             in_port_ptr->cmn.port_id,
             in_port_ptr->cmn.port_trigger_policy);
   }
   AR_MSG(DBG_MED_PRIO, "Framework extension structure...");
   AR_MSG(DBG_MED_PRIO, "    Policy: %ld, num_triggerable_groups %ld", policy, num_groups);

   if (nontriggerable_ports_ptr->in_port_grp_policy_ptr)
   {
      for (uint32_t i = 0; i < me_ptr->num_in_ports; i++)
      {
         capi_gapless_in_port_t *in_port_ptr = &(me_ptr->in_ports[i]);

         AR_MSG(DBG_MED_PRIO,
                "    Nontrigger policy input port idx %ld id 0x%lx: 0x%lx",
                in_port_ptr->cmn.index,
                in_port_ptr->cmn.port_id,
                nontriggerable_ports_ptr->in_port_grp_policy_ptr[i]);
      }
   }
   if (nontriggerable_ports_ptr->out_port_grp_policy_ptr)
   {
      AR_MSG(DBG_MED_PRIO,
             "    Nontrigger policy output port idx %ld id 0x%lx: 0x%lx",
             out_port_ptr->cmn.index,
             out_port_ptr->cmn.port_id,
             nontriggerable_ports_ptr->out_port_grp_policy_ptr[0]);
   }

   for (uint32_t group_idx = 0; group_idx < num_groups; group_idx++)
   {
      AR_MSG(DBG_MED_PRIO, "    Group %ld", group_idx);

      for (uint32_t i = 0; i < me_ptr->num_in_ports; i++)
      {
         capi_gapless_in_port_t *in_port_ptr = &(me_ptr->in_ports[i]);

         AR_MSG(DBG_MED_PRIO,
                "        Trigger affinity for input port idx %ld id 0x%lx: 0x%lx",
                in_port_ptr->cmn.index,
                in_port_ptr->cmn.port_id,
                triggerable_groups_ptr[group_idx].in_port_grp_affinity_ptr[i]);
      }

      AR_MSG(DBG_MED_PRIO,
             "        Trigger affinity for output port idx %ld id 0x%lx: 0x%lx",
             out_port_ptr->cmn.index,
             out_port_ptr->cmn.port_id,
             triggerable_groups_ptr[group_idx].out_port_grp_affinity_ptr[0]);
   }
}
#endif

/**
 * Checks whether the media format is supported for the gapless module.
 */
bool_t capi_gapless_is_supported_media_type(capi_media_fmt_v2_t *format_ptr)
{
   if (CAPI_FIXED_POINT != format_ptr->header.format_header.data_format)
   {
      AR_MSG(DBG_ERROR_PRIO, "Unsupported data format %lu", (uint32_t)format_ptr->header.format_header.data_format);
      return FALSE;
   }

   if ((format_ptr->format.data_interleaving != CAPI_DEINTERLEAVED_UNPACKED) && (format_ptr->format.num_channels != 1))
   {
      AR_MSG(DBG_ERROR_PRIO, "Interleaved data is not supported.");
      return FALSE;
   }

   return TRUE;
}

/**
 * Returns TRUE if current port's media format should become the new operating media format. This is the case
 * if there's no current operating media format or the current port is the active port.
 */
bool_t capi_gapless_should_set_operating_media_format(capi_gapless_t *me_ptr, capi_gapless_in_port_t *in_port_ptr)
{
   bool_t should_set_omf = (!capi_gapless_is_supported_media_type(&(me_ptr->operating_media_fmt))) ||
                           (me_ptr->active_in_port_index == in_port_ptr->cmn.index);
   return should_set_omf;
}

/**
 * Implementation of get properties for CAPI interface.
 */
capi_err_t capi_gapless_get_properties(capi_t *_pif, capi_proplist_t *proplist_ptr)
{
   capi_err_t      capi_result = CAPI_EOK;
   capi_gapless_t *me_ptr      = (capi_gapless_t *)_pif;

   uint32_t fwk_extn_ids_arr[] = { FWK_EXTN_TRIGGER_POLICY, FWK_EXTN_CONTAINER_FRAME_DURATION };

   capi_basic_prop_t mod_prop;
   mod_prop.init_memory_req    = align_to_8_byte(sizeof(capi_gapless_t));
   mod_prop.stack_size         = GAPLESS_STACK_SIZE;
   mod_prop.num_fwk_extns      = sizeof(fwk_extn_ids_arr) / sizeof(fwk_extn_ids_arr[0]);
   mod_prop.fwk_extn_ids_arr   = fwk_extn_ids_arr;
   mod_prop.is_inplace         = FALSE;
   mod_prop.req_data_buffering = TRUE;
   mod_prop.max_metadata_size  = 0;

   capi_result = capi_cmn_get_basic_properties(proplist_ptr, &mod_prop);
   if (CAPI_EOK != capi_result)
   {
      AR_MSG(DBG_ERROR_PRIO, "Get common basic properties failed with result %lu", capi_result);
      return capi_result;
   }

   capi_prop_t *prop_array = proplist_ptr->prop_ptr;

   for (uint32_t i = 0; i < proplist_ptr->props_num; i++)
   {
      capi_buf_t *payload_ptr = &prop_array[i].payload;

      switch (prop_array[i].id)
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
               AR_MSG(DBG_ERROR_PRIO, "null ptr while querying output mf");
               return CAPI_EBADPARAM;
            }

            // Handle with the operating media format if it exists. Otherwise, send invalid.
            if (capi_gapless_is_supported_media_type(&(me_ptr->operating_media_fmt)))
            {
               capi_result = capi_cmn_handle_get_output_media_fmt_v2(&prop_array[i], &(me_ptr->operating_media_fmt));
            }
            else
            {
               capi_media_fmt_v2_t invalid_media_fmt;
               capi_cmn_init_media_fmt_v2(&invalid_media_fmt);
               capi_result = capi_cmn_handle_get_output_media_fmt_v2(&prop_array[i], &invalid_media_fmt);
            }

            break;
         }
         case CAPI_PORT_DATA_THRESHOLD:
         {
            if (NULL == me_ptr)
            {
               AR_MSG(DBG_ERROR_PRIO, "null ptr while querying threshold");
               return CAPI_EBADPARAM;
            }
            uint32_t threshold_in_bytes = 1; // default
            capi_result                 = capi_cmn_handle_get_port_threshold(&prop_array[i], threshold_in_bytes);
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
                         "capi_test_module: CAPI_INTERFACE_EXTENSIONS invalid param size %lu",
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
                        {
                           curr_intf_extn_desc_ptr->is_supported = TRUE;
                           break;
                        }
                        case INTF_EXTN_DATA_PORT_OPERATION:
                        {
                           curr_intf_extn_desc_ptr->is_supported = TRUE;
                           break;
                        }
                        case INTF_EXTN_DUTY_CYCLING_ISLAND_MODE:
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
               AR_MSG(DBG_ERROR_PRIO, "CAPI_INTERFACE_EXTENSIONS bad param size %lu", payload_ptr->max_data_len);
               CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
            }
            break;
         }
         default:
         {
            AR_MSG(DBG_HIGH_PRIO, "get property for ID %#x. Not supported.", prop_array[i].id);
            CAPI_SET_ERROR(capi_result, CAPI_EUNSUPPORTED);
            break;
         }
      }
      if (CAPI_FAILED(capi_result) && (CAPI_EUNSUPPORTED != capi_result))
      {
         AR_MSG(DBG_HIGH_PRIO, "get property for %#x failed with opcode %lx", prop_array[i].id, capi_result);
      }
   }
   return capi_result;
}

/**
 * Handles port number info set properties. Payload is validated against maximum num supported ports.
 */
static capi_err_t capi_gapless_set_prop_port_num_info(capi_gapless_t *me_ptr, capi_prop_t *prop_ptr)
{
   capi_err_t  capi_result = CAPI_EOK;
   capi_buf_t *payload_ptr = &(prop_ptr->payload);

   if (payload_ptr->actual_data_len < sizeof(capi_port_num_info_t))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "set param, Param id 0x%lx Bad param size %lu",
             (uint32_t)prop_ptr->id,
             payload_ptr->actual_data_len);
      CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
      return capi_result;
   }

   // If input & output ports are already allocated, error out.
   if ((GAPLESS_NUM_PORTS_INVALID != me_ptr->num_in_ports) || (GAPLESS_NUM_PORTS_INVALID != me_ptr->num_out_ports))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "Set property id 0x%lx number of input and output ports already configured "
             "to %lu and %lu respectively",
             (uint32_t)prop_ptr->id,
             me_ptr->num_in_ports,
             me_ptr->num_out_ports);
      CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      return capi_result;
   }

   capi_port_num_info_t *data_ptr = (capi_port_num_info_t *)payload_ptr->data_ptr;

   if ((GAPLESS_MAX_INPUT_PORTS < data_ptr->num_input_ports) || (0 == data_ptr->num_input_ports) ||
       (GAPLESS_MAX_OUTPUT_PORTS < data_ptr->num_output_ports) || (0 == data_ptr->num_output_ports))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "Set property id 0x%lx number of input and output ports cannot be more "
             "than %lu and %lu respectively (also neither can be 0). Received %lu and %lu respectively",
             (uint32_t)prop_ptr->id,
             GAPLESS_MAX_INPUT_PORTS,
             GAPLESS_MAX_OUTPUT_PORTS,
             data_ptr->num_input_ports,
             data_ptr->num_output_ports);
      CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      return capi_result;
   }

   // All sanity checks are complete
   me_ptr->num_in_ports  = data_ptr->num_input_ports;
   me_ptr->num_out_ports = data_ptr->num_output_ports;

#ifdef CAPI_GAPLESS_DEBUG
   AR_MSG(DBG_HIGH_PRIO, "Num in ports %d, Num output ports %d", me_ptr->num_in_ports, me_ptr->num_out_ports);
#endif

   capi_result |= capi_gapless_init_all_ports(me_ptr);

   return capi_result;
}

/**
 * Handling for set property of input media format.
 */
static capi_err_t capi_gapless_set_prop_input_media_fmt(capi_gapless_t *me_ptr, capi_prop_t *prop_ptr)
{
   capi_err_t capi_result = CAPI_EOK;

   capi_buf_t *payload_ptr = &(prop_ptr->payload);

   // Raise event for output media format.
#ifdef CAPI_GAPLESS_DEBUG
   AR_MSG(DBG_HIGH_PRIO,
          "Received input media format with payload port idx %ld, port info is_valid %ld",
          prop_ptr->port_info.port_index,
          prop_ptr->port_info.is_valid);
#endif

   if (sizeof(capi_media_fmt_v2_t) > payload_ptr->actual_data_len)
   {
      AR_MSG(DBG_ERROR_PRIO, "Input Media Format Bad param size %lu", payload_ptr->actual_data_len);
      CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
      return capi_result;
   }

   if (!prop_ptr->port_info.is_valid)
   {
      AR_MSG(DBG_ERROR_PRIO, "Media format port info is invalid");
      CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      return capi_result;
   }

   capi_media_fmt_v2_t *   data_ptr    = (capi_media_fmt_v2_t *)(payload_ptr->data_ptr);
   capi_gapless_in_port_t *in_port_ptr = NULL;

   if (!capi_gapless_is_supported_media_type(data_ptr))
   {
      AR_MSG(DBG_ERROR_PRIO, "Media format is unsupported");
      CAPI_SET_ERROR(capi_result, CAPI_EFAILED);
      return capi_result;
   }

   uint32_t port_index = prop_ptr->port_info.port_index;
   in_port_ptr         = capi_gapless_get_in_port_from_index(me_ptr, port_index);

   if (!in_port_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "Failed to find input port info for %d", port_index);
      CAPI_SET_ERROR(capi_result, CAPI_EFAILED);
      return capi_result;
   }

   // Can't stop handling early if the input media format didn't change since we may have to update the
   // operating media format.

   // Copy and save the input media fmt.
   in_port_ptr->media_fmt = *data_ptr; // Copy.

   AR_MSG(DBG_HIGH_PRIO,
          "Setting media format on port idx %ld, id %ld",
          in_port_ptr->cmn.index,
          in_port_ptr->cmn.port_id);

   capi_result |= gapless_check_create_delay_buffer(me_ptr);

   // Set the media format to the delay buffer if the media format changed.
   if (capi_gapless_does_delay_buffer_exist(me_ptr, in_port_ptr))
   {
      capi_media_fmt_v2_t cur_media_fmt;
      memset(&cur_media_fmt, 0, sizeof(capi_media_fmt_v2_t));

      capi_result |= capi_gapless_get_delay_buffer_media_fmt(me_ptr, in_port_ptr, &cur_media_fmt);

      if (CAPI_FAILED(capi_result))
      {
         AR_MSG(DBG_ERROR_PRIO, "Get delay buffer media fmt returned bad result %d", capi_result);
         return capi_result;
      }
      else
      {
         if (!capi_cmn_media_fmt_equal(&cur_media_fmt, &(in_port_ptr->media_fmt)))
         {
            capi_result |= capi_gapless_set_delay_buffer_media_fmt(me_ptr,
                                                                   in_port_ptr,
                                                                   &(in_port_ptr->media_fmt),
                                                                   me_ptr->cntr_frame_size_us);
         }
      }
   }
   else
   {
      if (capi_gapless_should_set_operating_media_format(me_ptr, in_port_ptr))
      {
         capi_gapless_set_operating_media_format(me_ptr, &(in_port_ptr->media_fmt));
      }
   }

   return capi_result;
}

/**
 * Handling for set property of input media format.
 */
static capi_err_t capi_gapless_set_prop_register_event_to_dsp_client_v2(capi_gapless_t *me_ptr, capi_prop_t *prop_ptr)
{
   capi_err_t  capi_result = CAPI_EOK;
   capi_buf_t *payload_ptr = &(prop_ptr->payload);

#ifdef CAPI_GAPLESS_DEBUG
   AR_MSG(DBG_HIGH_PRIO, "Set property received for registering event data to dsp client v2");
#endif

   /* Validate the payload */
   if (payload_ptr->actual_data_len < sizeof(capi_register_event_to_dsp_client_v2_t))
   {
      AR_MSG(DBG_ERROR_PRIO, "Invalid payload size %d", payload_ptr->actual_data_len);

      capi_result |= CAPI_ENEEDMORE;
      return capi_result;
   }

   capi_register_event_to_dsp_client_v2_t *reg_event_ptr =
      (capi_register_event_to_dsp_client_v2_t *)(payload_ptr->data_ptr);

   if (EVENT_ID_EARLY_EOS == reg_event_ptr->event_id)
   {
      AR_MSG(DBG_HIGH_PRIO,
             "Received for registering/deregistering for early eos event, is_register %d",
             reg_event_ptr->is_register);

      // Register case
      if (1 == reg_event_ptr->is_register)
      {
         if (!(me_ptr->client_registered))
         {
            me_ptr->event_dest_address = reg_event_ptr->dest_address;
            me_ptr->event_token        = reg_event_ptr->token;
            me_ptr->client_registered  = TRUE;
         }
         else
         {
            capi_result |= CAPI_EFAILED;
            AR_MSG(DBG_ERROR_PRIO, "Client already registered for early eos.");
         }
      }
      // Deregister case
      else if (0 == reg_event_ptr->is_register)
      {
         if (me_ptr->client_registered)
         {
            me_ptr->event_dest_address = 0;
            me_ptr->event_token        = 0;
            me_ptr->client_registered  = FALSE;
         }
         else
         {
            capi_result |= CAPI_EFAILED;
            AR_MSG(DBG_ERROR_PRIO, "Can't deregister: no client registered for early eos event");
         }
      }
   }
   else
   {
      AR_MSG(DBG_ERROR_PRIO, "Unsupported event ID[%x]", reg_event_ptr->event_id);

      capi_result |= CAPI_EUNSUPPORTED;
   }

   // No need to create delay buffers in deregister case.
   if (1 == reg_event_ptr->is_register)
   {
      capi_result |= gapless_check_create_delay_buffer(me_ptr);
   }
   return capi_result;
}

/**
 * Implementation of set properties for CAPI interface.
 */
capi_err_t capi_gapless_set_properties(capi_t *_pif, capi_proplist_t *proplist_ptr)

{
   capi_err_t      capi_result = CAPI_EOK;
   capi_gapless_t *me_ptr      = (capi_gapless_t *)_pif;

   if (NULL == me_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "Set common property received null ptr");
      return CAPI_EBADPARAM;
   }

   capi_result = capi_cmn_set_basic_properties(proplist_ptr, &me_ptr->heap_info, &me_ptr->cb_info, NULL);
   if (CAPI_EOK != capi_result)
   {
      AR_MSG(DBG_ERROR_PRIO, "Set basic properties failed with result %lu", capi_result);
      return capi_result;
   }

   capi_prop_t *prop_array = proplist_ptr->prop_ptr;
   uint32_t     i          = 0;

   for (i = 0; i < proplist_ptr->props_num; i++)
   {
      switch (prop_array[i].id)
      {
         case CAPI_EVENT_CALLBACK_INFO:
         case CAPI_HEAP_ID:
         case CAPI_CUSTOM_INIT_DATA:
         {
            break;
         }
         case CAPI_ALGORITHMIC_RESET:
         {
            capi_gapless_in_port_t *active_in_port_ptr = capi_gapless_get_active_in_port(me_ptr);

            if (active_in_port_ptr && capi_gapless_does_delay_buffer_exist(me_ptr, active_in_port_ptr))
            {
               uint32_t read_offset = 0; // set the unread_data to 0.

               (void)spf_circ_buf_read_adjust(active_in_port_ptr->reader_handle, read_offset, &read_offset);

               gapless_check_update_trigger_policy(me_ptr);
            }

            break;
         }
         case CAPI_PORT_NUM_INFO:
         {
            capi_result |= capi_gapless_set_prop_port_num_info(me_ptr, &(prop_array[i]));
            break;
         }
         case CAPI_INPUT_MEDIA_FORMAT_V2:
         {
            capi_result |= capi_gapless_set_prop_input_media_fmt(me_ptr, &(prop_array[i]));
            break;
         }
         case CAPI_REGISTER_EVENT_DATA_TO_DSP_CLIENT_V2:
         {
            capi_result |= capi_gapless_set_prop_register_event_to_dsp_client_v2(me_ptr, &(prop_array[i]));
            break;
         }
         default:
         {
#ifdef CAPI_GAPLESS_DEBUG
            AR_MSG(DBG_HIGH_PRIO, "Set property id %#x. Not supported.", prop_array[i].id);
#endif

            CAPI_SET_ERROR(capi_result, CAPI_EUNSUPPORTED);
            break;
         }
      }
      if (CAPI_FAILED(capi_result) && (CAPI_EUNSUPPORTED != capi_result))
      {
         AR_MSG(DBG_HIGH_PRIO, "Set property for %#x failed with opcode %lu", prop_array[i].id, capi_result);
      }
   }

   return capi_result;
}

/**
 * Gets either a parameter value or a parameter structure containing multiple parameters. In the event of a failure, the
 * appropriate error code is returned.
 */
capi_err_t capi_gapless_get_param(capi_t *                _pif,
                                  uint32_t                param_id,
                                  const capi_port_info_t *port_info_ptr,
                                  capi_buf_t *            params_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   if (NULL == _pif || NULL == params_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "get param received bad pointer, 0x%p, 0x%p", _pif, params_ptr);
      return CAPI_EBADPARAM;
   }

   capi_gapless_t *me_ptr = (capi_gapless_t *)_pif;

   switch (param_id)
   {
      case PARAM_ID_EARLY_EOS_DELAY:
      {
         /*check if available size is large enough*/
         if (params_ptr->max_data_len < sizeof(param_id_gapless_early_eos_delay_t))
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "Get PARAM_ID_EARLY_EOS_DELAY failed: Insufficient size: %ld",
                   params_ptr->max_data_len);
            return (CAPI_EBADPARAM);
         }

         param_id_gapless_early_eos_delay_t *param_payload_ptr =
            (param_id_gapless_early_eos_delay_t *)params_ptr->data_ptr;
         param_payload_ptr->early_eos_delay_ms = me_ptr->early_eos_delay_ms;

         params_ptr->actual_data_len = sizeof(param_id_gapless_early_eos_delay_t);
      }
      default:
      {
         AR_MSG(DBG_ERROR_PRIO, "get param, unsupported param ID 0x%x", (int)param_id);
         CAPI_SET_ERROR(capi_result, CAPI_EUNSUPPORTED);
         break;
      }
   }
   return capi_result;
}

/**
 * Handles port operation set param. Payload is validated and then each individual operation is delegated to
 * opcode-specific functions.
 */
static capi_err_t capi_gapless_set_param_port_op(capi_gapless_t *me_ptr, capi_buf_t *payload_ptr)
{
   capi_err_t capi_result = CAPI_EOK;

   if (NULL == payload_ptr->data_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "Set property port operation, received null buffer");
      CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      return capi_result;
   }

   // Validate length.
   if (sizeof(intf_extn_data_port_operation_t) > payload_ptr->actual_data_len)
   {
      AR_MSG(DBG_ERROR_PRIO, "Set property for port operation, Bad param size %lu", payload_ptr->actual_data_len);
      CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
      return capi_result;
   }

   intf_extn_data_port_operation_t *data_ptr = (intf_extn_data_port_operation_t *)(payload_ptr->data_ptr);

   if ((sizeof(intf_extn_data_port_operation_t) + (data_ptr->num_ports * sizeof(intf_extn_data_port_id_idx_map_t))) >
       payload_ptr->actual_data_len)
   {
      AR_MSG(DBG_ERROR_PRIO, "Set property for port operation, Bad param size %lu", payload_ptr->actual_data_len);
      CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
      return capi_result;
   }

   uint32_t max_ports = data_ptr->is_input_port ? GAPLESS_MAX_INPUT_PORTS : GAPLESS_MAX_OUTPUT_PORTS;

   if (max_ports < data_ptr->num_ports)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "Invalid num ports. is_input: %lu, num_ports = %lu, max_supported_ports = %lu",
             data_ptr->is_input_port,
             data_ptr->num_ports,
             max_ports);
      CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      return capi_result;
   }

   // For each port in the operation payload.
   for (uint32_t iter = 0; iter < data_ptr->num_ports; iter++)
   {

      uint32_t port_id    = data_ptr->id_idx[iter].port_id;
      uint32_t port_index = data_ptr->id_idx[iter].port_index;

#ifdef CAPI_GAPLESS_DEBUG
      AR_MSG(DBG_HIGH_PRIO, "Validating port id = %lu, idx = %lu ", port_id, port_index);
#endif

      bool_t is_valid_port = capi_gapless_port_id_is_valid(me_ptr, port_id, data_ptr->is_input_port);

      if (!is_valid_port)
      {
         AR_MSG(DBG_ERROR_PRIO,
                "Unsupported port idx = %lu, id= %lu is_input_port = %lu "
                "Mode = %d ",
                data_ptr->id_idx[iter].port_id,
                port_index,
                data_ptr->is_input_port);
         CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
         return capi_result;
      }

      // Validate port index doesn't go out of bounds.
      if (port_index >= max_ports)
      {
         AR_MSG(DBG_ERROR_PRIO,
                "Bad parameter in id-idx map on port %lu in payload, port_index = %lu, "
                "is_input = %lu, max ports = %d",
                iter,
                port_index,
                data_ptr->is_input_port,
                max_ports);

         CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
         return capi_result;
      }

      // Validate that id-index mapping matches what was previously sent, unless mapping doesn't exist yet.
      capi_gapless_cmn_port_t *port_cmn_ptr =
         capi_gapless_get_port_cmn_from_index(me_ptr, port_index, data_ptr->is_input_port);
      if (port_cmn_ptr && (DATA_PORT_STATE_CLOSED != port_cmn_ptr->state))
      {
         uint32_t prev_id = port_cmn_ptr->port_id;
         if (GAPLESS_PORT_ID_INVALID != prev_id)
         {
            if (prev_id != port_id)
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "Error: id-idx mapping changed on port %lu in payload, port_index = %lu, prev port_id %lu, new "
                      "port_id %lu is_input = %lu",
                      iter,
                      prev_id,
                      port_id,
                      data_ptr->is_input_port);
               CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
               return capi_result;
            }
         }
      }

      switch ((uint32_t)data_ptr->opcode)
      {
         case INTF_EXTN_DATA_PORT_OPEN:
         {
            capi_result |= capi_gapless_port_open(me_ptr, data_ptr->is_input_port, port_index, port_id);
            break;
         }
         case INTF_EXTN_DATA_PORT_CLOSE:
         {
            capi_result |= capi_gapless_port_close(me_ptr, port_index, data_ptr->is_input_port);
            break;
         }
         case INTF_EXTN_DATA_PORT_START:
         {
            capi_result |= capi_gapless_port_start(me_ptr, port_index, data_ptr->is_input_port);
            break;
         }
         case INTF_EXTN_DATA_PORT_STOP:
         {
            capi_result |= capi_gapless_port_stop(me_ptr, port_index, data_ptr->is_input_port);
            break;
         }
         default:
         {
            AR_MSG(DBG_HIGH_PRIO, "Port operation opcode %lu. Not supported.", data_ptr->opcode);
            CAPI_SET_ERROR(capi_result, CAPI_EUNSUPPORTED);
            break;
         }
      }
   }

   return capi_result;
}

/**
 * Sets either a parameter value or a parameter structure containing multiple parameters. In the event of a failure, the
 * appropriate error code is returned.
 */
capi_err_t capi_gapless_set_param(capi_t *                _pif,
                                  uint32_t                param_id,
                                  const capi_port_info_t *port_info_ptr,
                                  capi_buf_t *            params_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   if (NULL == _pif || NULL == params_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "set param received bad pointer, 0x%p, 0x%p", _pif, params_ptr);
      return CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
   }

   capi_gapless_t *me_ptr = (capi_gapless_t *)(_pif);

   switch (param_id)
   {
      case PARAM_ID_EARLY_EOS_DELAY:
      {
         if (params_ptr->actual_data_len < sizeof(param_id_gapless_early_eos_delay_t))
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "Param id 0x%lx Bad param size %lu",
                   (uint32_t)param_id,
                   params_ptr->actual_data_len);
            capi_result |= CAPI_ENEEDMORE;
            break;
         }

         capi_gapless_in_port_t *active_in_port_ptr = capi_gapless_get_active_in_port(me_ptr);
         if ((NULL != active_in_port_ptr))
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "Error: Cannot accept PARAM_ID_EARLY_EOS_DELAY during runtime. active input port idx %ld, id 0x%lx",
                   active_in_port_ptr->cmn.index,
                   active_in_port_ptr->cmn.port_id);
            return CAPI_EFAILED;
         }

         param_id_gapless_early_eos_delay_t *payload_ptr = (param_id_gapless_early_eos_delay_t *)params_ptr->data_ptr;

         if (payload_ptr->early_eos_delay_ms > GAPLESS_MAX_EARLY_EOS_DELAY_MS)
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "Bad param: early eos delay sent = %ld ms, max %ld ms",
                   payload_ptr->early_eos_delay_ms,
                   GAPLESS_MAX_EARLY_EOS_DELAY_MS);
            return CAPI_EBADPARAM;
         }

         me_ptr->early_eos_delay_ms = payload_ptr->early_eos_delay_ms;
         AR_MSG(DBG_HIGH_PRIO, "Received early eos delay ms %ld", me_ptr->early_eos_delay_ms);

         capi_result |= gapless_check_create_delay_buffer(me_ptr);
         break;
      }
      case INTF_EXTN_PARAM_ID_DATA_PORT_OPERATION:
      {
         capi_result |= capi_gapless_set_param_port_op(me_ptr, params_ptr);
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
            AR_MSG(DBG_ERROR_PRIO, "Set param id 0x%lx, received null buffer", param_id);
            capi_result |= CAPI_EBADPARAM;
            break;
         }

         if (params_ptr->actual_data_len < sizeof(fwk_extn_param_id_trigger_policy_cb_fn_t))
         {
            AR_MSG(DBG_ERROR_PRIO, "Invalid payload size for trigger policy %d", params_ptr->actual_data_len);
            return CAPI_ENEEDMORE;
         }

         fwk_extn_param_id_trigger_policy_cb_fn_t *payload_ptr =
            (fwk_extn_param_id_trigger_policy_cb_fn_t *)params_ptr->data_ptr;

         me_ptr->trigger_policy_cb_fn = *payload_ptr; // Copy payload.
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
            AR_MSG(DBG_ERROR_PRIO,
                   "Frame duration sent as 0ms. Ignoring.");
            break;
         }

         if (me_ptr->cntr_frame_size_us != cfg_ptr->duration_us)
         {
            me_ptr->cntr_frame_size_us = cfg_ptr->duration_us;
            capi_result |= gapless_check_create_delay_buffer(me_ptr);

            AR_MSG(DBG_HIGH_PRIO, "Container frame size duration = %lu is set.", me_ptr->cntr_frame_size_us);

            // Set media format again to circular buffers, since this needs to be done when the frame size changes.
            for (uint32_t i = 0; i < me_ptr->num_in_ports; i++)
            {
               capi_gapless_in_port_t *in_port_ptr = capi_gapless_get_in_port_from_index(me_ptr, i);
               if (in_port_ptr)
               {
                  if (capi_gapless_does_delay_buffer_exist(me_ptr, in_port_ptr))
                  {
                     capi_gapless_set_delay_buffer_media_fmt(me_ptr,
                                                             in_port_ptr,
                                                             &(in_port_ptr->media_fmt),
                                                             me_ptr->cntr_frame_size_us);
                  }
               }
            }
         }

         break;
      }
      case INTF_EXTN_PARAM_ID_CNTR_DUTY_CYCLING_ENABLED:
      {
         if (params_ptr->actual_data_len < sizeof(intf_extn_param_id_cntr_duty_cycling_enabled_t))
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "Param id 0x%lx Bad param size %lu",
                   (uint32_t)param_id,
                   params_ptr->actual_data_len);
            capi_result |= CAPI_ENEEDMORE;
            break;
         }
         intf_extn_param_id_cntr_duty_cycling_enabled_t *payload_ptr =
            (intf_extn_param_id_cntr_duty_cycling_enabled_t *)params_ptr->data_ptr;
         me_ptr->is_gapless_cntr_duty_cycling = payload_ptr->is_cntr_duty_cycling;
         break;
      }
      default:
      {
         AR_MSG(DBG_ERROR_PRIO, "set param, unsupported param ID 0x%x", (int)param_id);
         CAPI_SET_ERROR(capi_result, CAPI_EUNSUPPORTED);
         break;
      }
   }
   return capi_result;
}

/**
 * If configuration is complete, creates the delay buffer and moves module out of pass_through_mode.
 */
capi_err_t gapless_check_create_delay_buffer(capi_gapless_t *me_ptr)
{
   capi_err_t result = CAPI_EOK;

   if (0 == me_ptr->early_eos_delay_ms || (!(me_ptr->client_registered)) ||
       (GAPLESS_INVALID_CNTR_FRAME_SIZE == me_ptr->cntr_frame_size_us))
   {
      AR_MSG(DBG_MED_PRIO,
             "Delay buffers not yet created due to missing configuration. early_eos_delay_ms %ld, client_registered "
             "%ld, cntr_frame_size_us (internal cfg) %ld. Continuing to wait for configuration.",
             me_ptr->early_eos_delay_ms,
             me_ptr->client_registered,
             me_ptr->cntr_frame_size_us);

      me_ptr->pass_through_mode = TRUE;
      gapless_check_update_trigger_policy(me_ptr);
      return result;
   }

   me_ptr->pass_through_mode = FALSE;

   // Loop through ports with input media format set and check if the delay buffer is created. If not created or size is
   // changing, recreate the delay buffer.
   for (uint32_t i = 0; i < me_ptr->num_in_ports; i++)
   {
      capi_gapless_in_port_t *in_port_ptr = &(me_ptr->in_ports[i]);

      if (!in_port_ptr)
      {
         continue;
      }

      // Only consider opened ports.
      if (DATA_PORT_STATE_CLOSED != in_port_ptr->cmn.state)
      {
         if (!capi_gapless_is_supported_media_type(&(in_port_ptr->media_fmt)))
         {
            continue;
         }

         if (!capi_gapless_does_delay_buffer_exist(me_ptr, in_port_ptr))
         {
            result |= capi_gapless_create_delay_buffer(me_ptr,
                                                       in_port_ptr,
                                                       &(in_port_ptr->media_fmt),
                                                       me_ptr->early_eos_delay_ms);

            if (CAPI_FAILED(result))
            {
               AR_MSG(DBG_MED_PRIO,
                      "Failed to create delay buffer at input port idx %ld id 0x%lx with size %ld ms. result 0x%lx",
                      in_port_ptr->cmn.index,
                      in_port_ptr->cmn.port_id,
                      me_ptr->early_eos_delay_ms,
                      result);
               return result;
            }
            AR_MSG(DBG_MED_PRIO,
                   "Created delay buffer at input port idx %ld id 0x%lx with size %ld ms.",
                   in_port_ptr->cmn.index,
                   in_port_ptr->cmn.port_id,
                   me_ptr->early_eos_delay_ms);

            result |= capi_gapless_set_delay_buffer_media_fmt(me_ptr,
                                                              in_port_ptr,
                                                              &(in_port_ptr->media_fmt),
                                                              me_ptr->cntr_frame_size_us);
         }
      }
   }

   result |= gapless_check_update_trigger_policy(me_ptr);
   return result;
}

/**
 * Checks if there's a change in the trigger policy. If so, resends the trigger policy event.
 *
 * Input port trigger policy (same for active or non-active):
 *   - For pass through mode: (input 0 || input 1) && output.
 *
 *   - For non-pass through mode:
 *      - Optional:               Internal buffer is not full.
 *      - Nontriggerable-Blocked: Internal buffer is full.
 *      - Nonexistant:            Port is closed or stopped.
 */
static capi_err_t gapless_check_update_in_port_trigger_policy(capi_gapless_t *me_ptr,
                                                              uint32_t        i,
                                                              bool_t *        policy_needs_update_ptr)
{
   capi_gapless_in_port_t *           in_port_ptr           = &(me_ptr->in_ports[i]);
   capi_gapless_port_trigger_policy_t new_input_port_policy = PORT_TRIGGER_POLICY_DONT_LISTEN;

   if (!policy_needs_update_ptr)
   {
      return CAPI_EFAILED;
   }

   *policy_needs_update_ptr = FALSE;

   if (DATA_PORT_STATE_STARTED == in_port_ptr->cmn.state)
   {
      uint32_t in_index = in_port_ptr->cmn.index;

      // Sanity check that port index and input exist.
      if (GAPLESS_PORT_INDEX_INVALID == in_index || (in_index != i))
      {
         AR_MSG(DBG_ERROR_PRIO,
                "Bad input port state for started port. Stored in array at spot %ld port structure's index value %ld.",
                i,
                in_index);
         return CAPI_EFAILED;
      }

      if (me_ptr->pass_through_mode)
      {
         new_input_port_policy = PORT_TRIGGER_POLICY_PASS_THROUGH;
      }
      else
      {
         // This can happen if the use case is started without receiving media format. In this case we need to listen
         // for input so that we can handle data path media format messages. It can also happen in steady state if
         // second
         // stream didn't receive media format.
         if (!capi_gapless_does_delay_buffer_exist(me_ptr, in_port_ptr))
         {

#ifdef CAPI_GAPLESS_DEBUG
            AR_MSG(DBG_MED_PRIO,
                   "Not in pass_through mode but delay buffer not yet created for input port idx %ld id 0x%lx. "
                   "Listening to input for the data path media format message.",
                   in_index,
                   in_port_ptr->cmn.port_id);
#endif

            new_input_port_policy = PORT_TRIGGER_POLICY_LISTEN;
         }
         // Even if the gapless circular buffer is full, we can keep input as optional. Input will stay unconsumed if provided while
         // the internal buffer is full. This avoids steady-state trigger policy events.
         /*
         Listen to input if circular buffer is not full.
         else if (!capi_gapless_is_delay_buffer_full(me_ptr, in_port_ptr))
         {
            new_input_port_policy = PORT_TRIGGER_POLICY_LISTEN;
         }*/
         else
         {
            new_input_port_policy = PORT_TRIGGER_POLICY_LISTEN;
         }
      }
   }
   // Stopped and closed ports shouldn't belong in any trigger policy.
   else
   {
      new_input_port_policy = PORT_TRIGGER_POLICY_CLOSED;
   }

   if (new_input_port_policy != in_port_ptr->cmn.port_trigger_policy)
   {
      *policy_needs_update_ptr             = TRUE;
      in_port_ptr->cmn.port_trigger_policy = new_input_port_policy;

      fwk_extn_port_trigger_affinity_t  trigger_affinity  = FWK_EXTN_PORT_TRIGGER_AFFINITY_NONE;
      fwk_extn_port_nontrigger_policy_t nontrigger_policy = FWK_EXTN_PORT_NON_TRIGGER_INVALID;

      switch (in_port_ptr->cmn.port_trigger_policy)
      {
         case PORT_TRIGGER_POLICY_PASS_THROUGH:
         case PORT_TRIGGER_POLICY_LISTEN:
         {
            trigger_affinity  = FWK_EXTN_PORT_TRIGGER_AFFINITY_PRESENT;
            nontrigger_policy = FWK_EXTN_PORT_NON_TRIGGER_INVALID;
            break;
         }
         case PORT_TRIGGER_POLICY_DONT_LISTEN:
         {
            trigger_affinity  = FWK_EXTN_PORT_TRIGGER_AFFINITY_NONE;
            nontrigger_policy = FWK_EXTN_PORT_NON_TRIGGER_BLOCKED;
            break;
         }
         case PORT_TRIGGER_POLICY_CLOSED:
         {
            trigger_affinity  = FWK_EXTN_PORT_TRIGGER_AFFINITY_NONE;
            nontrigger_policy = FWK_EXTN_PORT_NON_TRIGGER_INVALID;
            break;
         }
         default:
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "Unexpected trigger policy 0x%lx for input port index %ld. Returning error.",
                   in_port_ptr->cmn.port_trigger_policy,
                   i);
            return CAPI_EFAILED;
         }
      }

      me_ptr->trigger_policy.in_port_trigger_policies[i]    = trigger_affinity;
      me_ptr->trigger_policy.in_port_nontrigger_policies[i] = nontrigger_policy;
   }

   return CAPI_EOK;
}

/**
 * Checks if there's a change in the output port trigger policy. If so, updates cached trigger policy state.
 *
 * Output port trigger policy:
 *   - For pass through mode: (input 0 || input 1) && output.
 *
 *   - For non-pass through mode:
 *   - Optional:                There is an active input port and there is internally buffered data on the active output
 *                              port.
 *   - Nontriggerable-Optional: Otherwise (no active input ports, or active input port has no internally buffered data).
 *   - Nonexistant:             Port is closed or stopped.
 */
static capi_err_t gapless_check_update_out_port_trigger_policy(capi_gapless_t *me_ptr, bool_t *policy_needs_update_ptr)
{
   capi_gapless_out_port_t *out_port_ptr = capi_gapless_get_out_port(me_ptr);

   if (!out_port_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "Error getting output port");
      return CAPI_EFAILED;
   }

   uint32_t OUT_PORT_INDEX_ZERO = 0;

   if (!policy_needs_update_ptr)
   {
      return CAPI_EFAILED;
   }

   *policy_needs_update_ptr = FALSE;

   // Check for the new trigger policy.
   capi_gapless_port_trigger_policy_t new_output_port_policy = PORT_TRIGGER_POLICY_DONT_LISTEN;

   if (DATA_PORT_STATE_STARTED == out_port_ptr->cmn.state)
   {
      if (me_ptr->pass_through_mode)
      {
         new_output_port_policy = PORT_TRIGGER_POLICY_PASS_THROUGH;
      }
      else
      {
         capi_gapless_in_port_t *active_in_port_ptr = capi_gapless_get_active_in_port(me_ptr);
         if (NULL != active_in_port_ptr)
         {
            if (!capi_gapless_is_delay_buffer_empty(me_ptr, active_in_port_ptr))
            {
               new_output_port_policy = PORT_TRIGGER_POLICY_LISTEN;
            }
#ifdef CAPI_GAPLESS_DEBUG
            else
            {
               AR_MSG(DBG_LOW_PRIO,
                      "The delay buffer for active input port idx %ld id 0x%lx is empty. Output is "
                      "optional-nontriggerable.",
                      active_in_port_ptr->cmn.index,
                      active_in_port_ptr->cmn.port_id);
            }
#endif
         }
      }
   }
   // Stopped and closed ports shouldn't belong in any trigger policy.
   else
   {
      new_output_port_policy = PORT_TRIGGER_POLICY_CLOSED;
   }

   // If the policy changed, update the cached trigger policy structures.
   if (new_output_port_policy != out_port_ptr->cmn.port_trigger_policy)
   {
      *policy_needs_update_ptr              = TRUE;
      out_port_ptr->cmn.port_trigger_policy = new_output_port_policy;

      fwk_extn_port_trigger_affinity_t  trigger_affinity  = FWK_EXTN_PORT_TRIGGER_AFFINITY_NONE;
      fwk_extn_port_nontrigger_policy_t nontrigger_policy = FWK_EXTN_PORT_NON_TRIGGER_INVALID;

      switch (out_port_ptr->cmn.port_trigger_policy)
      {
         case PORT_TRIGGER_POLICY_PASS_THROUGH:
         case PORT_TRIGGER_POLICY_LISTEN:
         {
            trigger_affinity  = FWK_EXTN_PORT_TRIGGER_AFFINITY_PRESENT;
            nontrigger_policy = FWK_EXTN_PORT_NON_TRIGGER_INVALID;
            break;
         }
         case PORT_TRIGGER_POLICY_DONT_LISTEN:
         {
            trigger_affinity  = FWK_EXTN_PORT_TRIGGER_AFFINITY_NONE;
            nontrigger_policy = FWK_EXTN_PORT_NON_TRIGGER_OPTIONAL;
            break;
         }
         case PORT_TRIGGER_POLICY_CLOSED:
         {
            trigger_affinity  = FWK_EXTN_PORT_TRIGGER_AFFINITY_NONE;
            nontrigger_policy = FWK_EXTN_PORT_NON_TRIGGER_INVALID;
            break;
         }
         default:
         {
            AR_MSG(DBG_MED_PRIO,
                   "Unexpected trigger policy 0x%lx for output port. Returning error.",
                   out_port_ptr->cmn.port_trigger_policy);
            return CAPI_EFAILED;
         }
      }

      me_ptr->trigger_policy.out_port_trigger_policies[OUT_PORT_INDEX_ZERO]    = trigger_affinity;
      me_ptr->trigger_policy.out_port_nontrigger_policies[OUT_PORT_INDEX_ZERO] = nontrigger_policy;
   }

   return CAPI_EOK;
}

/**
 * Checks if there's a change in the trigger policy. If so, resends the trigger policy event.
 *
 * Trigger policy:
 *   For pass through mode: (input 0 || input 1) && output.
 *
 *   Otherwise:
 *      Output port:
 *      - Optional:                There is an active input port and there is internally buffered data on the active
 * output
 *                                 port.
 *      - Nontriggerable-Optional: Otherwise (no active input ports, or active input port has no internally buffered
 * data).
 *      - Nonexistant:             Port is closed or stopped.
 *
 *      Input port (same for active or non-active):
 *      - Optional:               Internal buffer is not full.
 *      - Nontriggerable-Blocked: Internal buffer is full.
 *      - Nonexistant:            Port is closed or stopped.
 */
capi_err_t gapless_check_update_trigger_policy(capi_gapless_t *me_ptr)
{
   capi_err_t result = CAPI_EOK;

   bool_t policy_needs_update = FALSE;
   bool_t all_policies_closed = FALSE;

   // Go through input and output side, check what current policy should be. If different than previous,
   // update policy_needs_update and make changes to me_ptr->trigger policy structure.
   for (uint32_t i = 0; i < me_ptr->num_in_ports; i++)
   {
      bool_t in_policy_needs_update = FALSE;

      if (CAPI_EOK != (result |= gapless_check_update_in_port_trigger_policy(me_ptr, i, &in_policy_needs_update)))
      {
         AR_MSG(DBG_ERROR_PRIO, "Error 0x%lx returned from gapless_check_update_in_port_trigger_policy", result);
         return result;
      }

      policy_needs_update |= in_policy_needs_update;
      all_policies_closed &= (PORT_TRIGGER_POLICY_CLOSED == me_ptr->in_ports[i].cmn.port_trigger_policy);
   }

   // Output side
   capi_gapless_out_port_t *out_port_ptr = capi_gapless_get_out_port(me_ptr);

   if (!out_port_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "Error getting output port");
      return CAPI_EFAILED;
   }

   bool_t out_policy_needs_update = FALSE;
   if (CAPI_EOK != (result |= gapless_check_update_out_port_trigger_policy(me_ptr, &out_policy_needs_update)))
   {
      AR_MSG(DBG_ERROR_PRIO, "Error 0x%lx returned from gapless_check_update_out_port_trigger_policy", result);
      return result;
   }

   policy_needs_update |= out_policy_needs_update;
   all_policies_closed &= (PORT_TRIGGER_POLICY_CLOSED == out_port_ptr->cmn.port_trigger_policy);

   // Send trigger policy if anything changed, or we haven't sent a trigger policy yet.
   if ((policy_needs_update || (!me_ptr->sent_trigger_policy_at_least_once)))
   {
      // Populate fwk extn structs with data stored in me_ptr.
      result |= gapless_send_trigger_policy(me_ptr, all_policies_closed);

      me_ptr->sent_trigger_policy_at_least_once = TRUE;
   }

   return result;
}

/**
 * Sets up arguments to the trigger policy callback function and calls that function. Different cases for
 * pass through vs non-pass through since pass through has 2 groups (one for input, one for output).
 */
capi_err_t gapless_send_trigger_policy(capi_gapless_t *me_ptr, bool_t all_policies_closed)
{
   capi_err_t result = CAPI_EOK;

   if (!me_ptr->trigger_policy_cb_fn.change_data_trigger_policy_cb_fn)
   {
      AR_MSG(DBG_MED_PRIO,
             "Trigger policy function callback not yet set, can't send trigger policy (ok at start of use case)!");
      return result;
   }

   // Non pass-through mode policies, as well as if all ports are closed.
   if ((!me_ptr->pass_through_mode) || all_policies_closed)
   {
      // Policy is always optional. There is one group unless all ports are closed in which case there are no
      // groups.
      fwk_extn_port_trigger_policy_t   POLICY_OPTIONAL = FWK_EXTN_PORT_TRIGGER_POLICY_OPTIONAL;
      uint32_t                         NUM_GROUPS_ONE  = 1;
      uint32_t                         NUM_GROUPS_ZERO = 0;
      fwk_extn_port_nontrigger_group_t nontriggerable_ports;
      fwk_extn_port_trigger_group_t    triggerable_group;

      nontriggerable_ports.in_port_grp_policy_ptr  = me_ptr->trigger_policy.in_port_nontrigger_policies;
      nontriggerable_ports.out_port_grp_policy_ptr = me_ptr->trigger_policy.out_port_nontrigger_policies;
      triggerable_group.in_port_grp_affinity_ptr   = me_ptr->trigger_policy.in_port_trigger_policies;
      triggerable_group.out_port_grp_affinity_ptr  = me_ptr->trigger_policy.out_port_trigger_policies;

#ifdef CAPI_GAPLESS_DEBUG
      gapless_print_trigger_policy(me_ptr,
                                   &nontriggerable_ports,
                                   POLICY_OPTIONAL,
                                   (all_policies_closed ? NUM_GROUPS_ZERO : NUM_GROUPS_ONE),
                                   (all_policies_closed ? NULL : &triggerable_group));
#endif

      result |= me_ptr->trigger_policy_cb_fn
                   .change_data_trigger_policy_cb_fn(me_ptr->trigger_policy_cb_fn.context_ptr,
                                                     &nontriggerable_ports,
                                                     POLICY_OPTIONAL,
                                                     (all_policies_closed ? NUM_GROUPS_ZERO : NUM_GROUPS_ONE),
                                                     (all_policies_closed ? NULL : &triggerable_group));
      me_ptr->trigger_policy_sent_once = TRUE;
   }
   // Pass through mode policies.
   else
   {
      if (1 == me_ptr->num_in_ports)
      {
         // Normally, don't send trigger policy update - default will be used by fwk. But if we return to pass-through
         // after non-pass-through, we need to ensure trigger policy gets reset to default.
         if (me_ptr->trigger_policy_sent_once)
         {
            // Default policy is (input && output).
            fwk_extn_port_trigger_policy_t   POLICY_OPTIONAL = FWK_EXTN_PORT_TRIGGER_POLICY_MANDATORY;
            uint32_t                         NUM_GROUPS_ONE  = 1;
            fwk_extn_port_nontrigger_group_t nontriggerable_ports;
            fwk_extn_port_trigger_group_t    triggerable_groups[1];
            fwk_extn_port_trigger_group_t *  triggerable_groups_ptr = &(triggerable_groups[0]);

            // Default nontrigger policy is INVALID.
            nontriggerable_ports.in_port_grp_policy_ptr  = &(me_ptr->trigger_policy.default_nontrigger_policy);
            nontriggerable_ports.out_port_grp_policy_ptr = &(me_ptr->trigger_policy.default_nontrigger_policy);

            // Default trigger afinity is MANDATORY.
            triggerable_groups[0].in_port_grp_affinity_ptr  = &(me_ptr->trigger_policy.default_trigger_affinity);
            triggerable_groups[0].out_port_grp_affinity_ptr = &(me_ptr->trigger_policy.default_trigger_affinity);

#ifdef CAPI_GAPLESS_DEBUG
            gapless_print_trigger_policy(me_ptr,
                                         &nontriggerable_ports,
                                         POLICY_OPTIONAL,
                                         NUM_GROUPS_ONE,
                                         triggerable_groups_ptr);
#endif

            result |=
               me_ptr->trigger_policy_cb_fn.change_data_trigger_policy_cb_fn(me_ptr->trigger_policy_cb_fn.context_ptr,
                                                                             &nontriggerable_ports,
                                                                             POLICY_OPTIONAL,
                                                                             NUM_GROUPS_ONE,
                                                                             triggerable_groups_ptr);
            me_ptr->trigger_policy_sent_once = TRUE;
         }
      }
      else
      {
         // Policy is always optional.
         fwk_extn_port_trigger_policy_t   POLICY_OPTIONAL = FWK_EXTN_PORT_TRIGGER_POLICY_OPTIONAL;
         uint32_t                         NUM_GROUPS_TWO  = 2;
         fwk_extn_port_nontrigger_group_t nontriggerable_ports;
         fwk_extn_port_trigger_group_t    triggerable_groups[2];
         fwk_extn_port_trigger_group_t *  triggerable_groups_ptr = &(triggerable_groups[0]);

         nontriggerable_ports.in_port_grp_policy_ptr  = me_ptr->trigger_policy.in_port_nontrigger_policies;
         nontriggerable_ports.out_port_grp_policy_ptr = me_ptr->trigger_policy.out_port_nontrigger_policies;

         // Put inputs and outputs in different groups to get (input 0 || input 1) && (output).
         triggerable_groups[0].in_port_grp_affinity_ptr  = me_ptr->trigger_policy.in_port_trigger_policies;
         triggerable_groups[0].out_port_grp_affinity_ptr = me_ptr->trigger_policy.none_out_port_trigger_policies;
         triggerable_groups[1].in_port_grp_affinity_ptr  = me_ptr->trigger_policy.none_in_port_trigger_policies;
         triggerable_groups[1].out_port_grp_affinity_ptr = me_ptr->trigger_policy.out_port_trigger_policies;

#ifdef CAPI_GAPLESS_DEBUG
         gapless_print_trigger_policy(me_ptr,
                                      &nontriggerable_ports,
                                      POLICY_OPTIONAL,
                                      NUM_GROUPS_TWO,
                                      triggerable_groups_ptr);
#endif

         result |=
            me_ptr->trigger_policy_cb_fn.change_data_trigger_policy_cb_fn(me_ptr->trigger_policy_cb_fn.context_ptr,
                                                                          &nontriggerable_ports,
                                                                          POLICY_OPTIONAL,
                                                                          NUM_GROUPS_TWO,
                                                                          triggerable_groups_ptr);
         me_ptr->trigger_policy_sent_once = TRUE;
      }
   }

   return result;
}

/**
 * If the new media format is different than the operating media format, set it and raise an output media format
 * event.
 */
capi_err_t capi_gapless_set_operating_media_format(capi_gapless_t *me_ptr, capi_media_fmt_v2_t *media_fmt_ptr)
{
   capi_err_t result        = CAPI_EOK;
   bool_t     IS_INPUT_PORT = FALSE; // For output media format event.

   if (!capi_cmn_media_fmt_equal(&me_ptr->operating_media_fmt, media_fmt_ptr))
   {
      me_ptr->operating_media_fmt = *media_fmt_ptr; // Copy.

      capi_gapless_out_port_t *out_port_ptr = capi_gapless_get_out_port(me_ptr);
      bool_t is_output_opened = out_port_ptr ? (DATA_PORT_STATE_CLOSED != out_port_ptr->cmn.state) : FALSE;

      if (is_output_opened)
      {
         result |= capi_cmn_output_media_fmt_event_v2(&me_ptr->cb_info,
                                                      &(me_ptr->operating_media_fmt),
                                                      IS_INPUT_PORT,
                                                      out_port_ptr->cmn.index);

         AR_MSG(DBG_HIGH_PRIO, "delay_buf: circular buf raised output media format");
      }
   }
   else
   {
      AR_MSG(DBG_HIGH_PRIO, "delay_buf: not raising output media format since operating media format did not change.");
   }

   return result;
}
