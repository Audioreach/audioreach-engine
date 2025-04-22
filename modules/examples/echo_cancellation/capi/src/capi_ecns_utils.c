/**
 * \file capi_ecns_utils.c
 *  
 * \brief
 *  
 *     Example Echo Cancellation
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/*----------------------------------------------------------------------------
 * Include Files
 * -------------------------------------------------------------------------*/
#include "capi_ecns_i.h"
#include "capi_intf_extn_data_port_operation.h"

/*----------------------------------------------------------------------------
 * Local declerations
 * -------------------------------------------------------------------------*/
#define FWK_EXTNS_ARRAY                                                                                                \
   {                                                                                                                   \
      {                                                                                                                \
         FWK_EXTN_ECNS                                                                                                 \
      }                                                                                                                \
   }

#define LEN_OF_FWK_EXTNS_ARRAY                                                                                         \
   (sizeof((capi_framework_extension_id_t[])FWK_EXTNS_ARRAY) / sizeof(capi_framework_extension_id_t))

static capi_err_t capi_ecns_handle_input_media_format(capi_ecns_t *     me_ptr,
                                                      capi_port_info_t *port_info_ptr,
                                                      capi_buf_t *      payload_ptr);

static capi_err_t capi_ecns_validate_input_media_format(capi_ecns_t *             me_ptr,
                                                        uint32_t                  input_port_index,
                                                        capi_ecns_media_fmt_v2_t *data_format_ptr);

/* =========================================================================
 * FUNCTION : capi_ecns_validate_input_media_format
 *
 * DESCRIPTION:
 *    Validates input media format. Currently checks only data interleaving
 * type. Add checks depending on the libary specifications.
 * ========================================================================= */
static capi_err_t capi_ecns_validate_input_media_format(capi_ecns_t *             me_ptr,
                                                        uint32_t                  input_port_index,
                                                        capi_ecns_media_fmt_v2_t *data_format_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   if (CAPI_FIXED_POINT != data_format_ptr->header.format_header.data_format)
   {
      ECNS_DBG(me_ptr->miid,
               DBG_ERROR_PRIO,
               "Error! invalid data format %d",
               (int)data_format_ptr->header.format_header.data_format);
      return CAPI_EBADPARAM;
   }

   capi_standard_data_format_v2_t *pcm_format_ptr = &data_format_ptr->format;

   if (CAPI_DEINTERLEAVED_UNPACKED != pcm_format_ptr->data_interleaving)
   {
      ECNS_DBG(me_ptr->miid,
               DBG_ERROR_PRIO,
               "Error! Invalid data interleaving type %lu",
               pcm_format_ptr->data_interleaving);
      return CAPI_EBADPARAM;
   }

   /* Check for the validity port specific input media format . */
   if (ECNS_PRIMARY_INPUT_STATIC_PORT_ID == me_ptr->in_port_info[input_port_index].port_id)
   {

      /* NOTE: Do port specific media format validation */
   }
   else if(ECNS_REFERENCE_INPUT_STATIC_PORT_ID == me_ptr->in_port_info[input_port_index].port_id)
   {
      /* NOTE: Do port specific media format validation */
   }
   else // unknown input port index
   {
      CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
   }

   return capi_result;
}

/* =========================================================================
 * FUNCTION : capi_encs_media_fmt_equal
 *
 * DESCRIPTION:
 *    Checks if the there is any change in input media format.
 * ========================================================================= */
bool_t capi_encs_media_fmt_equal(capi_ecns_media_fmt_v2_t *media_fmt_1_ptr, capi_ecns_media_fmt_v2_t *media_fmt_2_ptr)
{
   bool_t are_dif = FALSE;

   // clang-format off
   are_dif |= (media_fmt_1_ptr->header.format_header.data_format != media_fmt_2_ptr->header.format_header.data_format);
   are_dif |= (media_fmt_1_ptr->format.bits_per_sample           != media_fmt_2_ptr->format.bits_per_sample);
   are_dif |= (media_fmt_1_ptr->format.bitstream_format          != media_fmt_2_ptr->format.bitstream_format);
   are_dif |= (media_fmt_1_ptr->format.data_interleaving         != media_fmt_2_ptr->format.data_interleaving);
   are_dif |= (media_fmt_1_ptr->format.data_is_signed            != media_fmt_2_ptr->format.data_is_signed);
   are_dif |= (media_fmt_1_ptr->format.num_channels              != media_fmt_2_ptr->format.num_channels);
   are_dif |= (media_fmt_1_ptr->format.q_factor                  != media_fmt_2_ptr->format.q_factor);
   are_dif |= (media_fmt_1_ptr->format.sampling_rate             != media_fmt_2_ptr->format.sampling_rate);
   // clang-format on

   return !are_dif;
}

/* =========================================================================
 * FUNCTION : capi_ecns_raise_port_threshold_event
 *
 * DESCRIPTION:
 *    Raises event and informs framework of a module threshold changes. The
 *  event is raised per port. So module needs to raise threshold for each of
 *  the ports.
 *
 *    Threshold changes with change in input media format and also due to change
 *  in module operation data frame size. Usually frame size is fixed, but it
 *  can change due to calibration. In this example, frame size is assumed to
 *  be fixed ECNS_PROCESS_FRAME_SIZE_IN_10MS [10 ms].
 * ========================================================================= */
static capi_err_t capi_ecns_raise_port_threshold_event(capi_ecns_t *me_ptr,
                                                       uint32_t     frame_size_in_samples,
                                                       uint32_t     port_index,
                                                       bool_t       is_input_port)
{
   capi_err_t capi_result       = CAPI_EOK;
   uint32_t   bytes_per_samples = 0;
   uint32_t   num_channels      = 0;

   if (is_input_port)
   {
      bytes_per_samples = me_ptr->in_port_info[port_index].media_fmt.format.bits_per_sample >> 3;
      num_channels      = me_ptr->in_port_info[port_index].media_fmt.format.num_channels;
   }
   else
   {
      bytes_per_samples = me_ptr->out_port_info[port_index].media_fmt.format.bits_per_sample >> 3;
      num_channels      = me_ptr->out_port_info[port_index].media_fmt.format.num_channels;
   }
   capi_port_data_threshold_change_t event = { frame_size_in_samples * bytes_per_samples * num_channels };

   capi_event_info_t event_info = { { TRUE, is_input_port, port_index },
                                    { (int8_t *)&event, sizeof(event), sizeof(event) } };

   capi_result |=
      me_ptr->cb_info.event_cb(me_ptr->cb_info.event_context, CAPI_EVENT_PORT_DATA_THRESHOLD_CHANGE, &event_info);
   if (CAPI_FAILED(capi_result))
   {
      ECNS_DBG(me_ptr->miid,
               DBG_ERROR_PRIO,
               "Failed to raise port_threshold event with result %lu for threshold %lu",
               capi_result,
               event.new_threshold_in_bytes);
   }

   ECNS_DBG(me_ptr->miid,
            DBG_HIGH_PRIO,
            "Raised port threshold. is_input_port(%ld) port_index(0x%lx) threshold (%lu)",
            is_input_port,
            port_index,
            event.new_threshold_in_bytes);

   return capi_result;
}

/* =========================================================================
 * FUNCTION : capi_ecns_raise_output_media_format_event
 *
 * DESCRIPTION:
 *    Handles raising output media format. Output media format can change if
 * the input media format changes like sampling rate, no of channels. It can
 * also change with the module calibration. For example, if beam forming is
 * enabled number of output channels can vary depending on calibration.
 * ========================================================================= */
capi_err_t capi_ecns_raise_output_media_format_event(capi_ecns_t *me_ptr)
{
   capi_err_t result = CAPI_EOK;

   if (NULL == me_ptr->cb_info.event_cb)
   {
      ECNS_DBG(me_ptr->miid,
               DBG_ERROR_PRIO,
               "Error event callback is not set.Unable to raise output "
               "media format event");
      return CAPI_EFAILED;
   }

   uint32_t prim_in_idx;
   result = ecns_get_input_port_arr_idx(me_ptr, ECNS_PRIMARY_INPUT_STATIC_PORT_ID, &prim_in_idx);
   if (CAPI_FAILED(result))
   {
      return CAPI_EFAILED;
   }

   if (FALSE == me_ptr->in_port_info[prim_in_idx].is_media_fmt_received)
   {
      ECNS_DBG(me_ptr->miid,
               DBG_ERROR_PRIO,
               "Primary input port hasn't received media fmt yet, cannot raise media format.");
      return result;
   }

   uint32_t prim_out_idx;
   result = ecns_get_output_port_arr_idx(me_ptr, ECNS_PRIMARY_OUTPUT_STATIC_PORT_ID, &prim_out_idx);
   if (CAPI_FAILED(result))
   {
      return CAPI_EFAILED;
   }

   capi_event_info_t event_info = { { TRUE, FALSE, prim_out_idx },
                                    { (int8_t *)&me_ptr->out_port_info[prim_out_idx].media_fmt,
                                      sizeof(capi_ecns_media_fmt_v2_t),
                                      sizeof(capi_ecns_media_fmt_v2_t) } };

   result =
      me_ptr->cb_info.event_cb(me_ptr->cb_info.event_context, CAPI_EVENT_OUTPUT_MEDIA_FORMAT_UPDATED_V2, &event_info);
   if (CAPI_FAILED(result))
   {
      ECNS_DBG(me_ptr->miid,
               DBG_ERROR_PRIO,
               "Error!! Failed to send output media format event with result %lu",
               result);
   }

   return result;
}

/* =========================================================================
 * FUNCTION : capi_ecns_update_raise_kpps_event
 *
 * DESCRIPTION:
 *    KPPS event indicates the framework the peak processing cycles that may
 * be required by the module. This is processor specific quantity, it varies
 * depending on host processor i.e hexagon, ARM etc.  This function updates module KPPS whenever there is a change. KPPS can change with calibration or media format.
 * ========================================================================= */
static void capi_ecns_update_raise_kpps_event(capi_ecns_t *me_ptr)
{
   if (NULL == me_ptr->cb_info.event_cb)
   {
      ECNS_DBG(me_ptr->miid, DBG_ERROR_PRIO, "Event callback is not set Unable to raise kpps event!");
      return;
   }

   capi_err_t        result = CAPI_EOK;
   capi_event_KPPS_t event;
   capi_event_info_t event_info;

   /* TODO: update appropriate kpps here. */
   uint32_t kpps = ECNS_KPPS_REQUIREMENT;

   // Raise the event only if there is a change.
   if (me_ptr->kpps != kpps)
   {
      me_ptr->kpps = kpps;

      event.KPPS                         = kpps;
      event_info.port_info.is_valid      = false;
      event_info.payload.actual_data_len = sizeof(event);
      event_info.payload.max_data_len    = sizeof(event);
      event_info.payload.data_ptr        = (int8_t *)&event;

      result = me_ptr->cb_info.event_cb(me_ptr->cb_info.event_context, CAPI_EVENT_KPPS, &event_info);
      if (CAPI_FAILED(result))
      {
         ECNS_DBG(me_ptr->miid,
                  DBG_ERROR_PRIO,
                  "Failed to send KPPS update event with %lu",
                  result);
      }
      else
      {
         ECNS_DBG(me_ptr->miid, DBG_HIGH_PRIO, "reported %lu KPPS", kpps);
      }
   }
}

/* =========================================================================
 * FUNCTION : capi_ecns_update_raise_bandwidth_event
 *
 * DESCRIPTION:
 *    Updates module data bandwidth/code bandwidth whenever there is a change.
 * This parameter can change with calibration or media format.
 * ========================================================================= */
static void capi_ecns_update_raise_bandwidth_event(capi_ecns_t *me_ptr)
{
   if (NULL == me_ptr->cb_info.event_cb)
   {
      ECNS_DBG(me_ptr->miid,
               DBG_ERROR_PRIO,
               "event callback is not set. Unable to raise bandwidth event!");
      return;
   }

   capi_err_t capi_result = CAPI_EOK;

   capi_event_bandwidth_t event = { 0 };
   capi_event_info_t      event_info;

   /* TODO: update appropriate bandwidth here. */
   event.code_bandwidth = ECNS_CODE_BW;
   event.data_bandwidth = ECNS_DATA_BW;

   ECNS_DBG(me_ptr->miid,
            DBG_MED_PRIO,
            "Raising bw event, code_bw = %lu, data_bw = %lu",
            event.code_bandwidth,
            event.data_bandwidth);

   event_info.port_info.is_valid   = false;
   event_info.payload.max_data_len = event_info.payload.actual_data_len = sizeof(event);

   event_info.payload.data_ptr = (int8_t *)&event;
   capi_result                 = me_ptr->cb_info.event_cb(me_ptr->cb_info.event_context, CAPI_EVENT_BANDWIDTH, &event_info);
   if (CAPI_FAILED(capi_result))
   {
      ECNS_DBG(me_ptr->miid,
               DBG_ERROR_PRIO,
               "Failed to send bandwidth update event with %lu",
               capi_result);
   }
}

/* =========================================================================
 * FUNCTION : capi_ecns_update_raise_algorithmic_delay_event
 *
 * DESCRIPTION:
 *    Updates module data path delay whenever there is a change. This parameter can
 * change with calibration or media format.
 * ========================================================================= */
static void capi_ecns_update_raise_algorithmic_delay_event(capi_ecns_t *me_ptr)
{
   if (NULL == me_ptr->cb_info.event_cb)
   {
      ECNS_DBG(me_ptr->miid,
               DBG_ERROR_PRIO,
               "Event callback is not set. Unable to raise delay event!");
      return;
   }

   capi_err_t                     result = CAPI_EOK;
   capi_event_algorithmic_delay_t event;
   capi_event_info_t              event_info;

   /* TODO: update appropriate algorithmic delay here. */
   uint32_t data_path_delay_in_us = ECNS_DEFAULT_DELAY_IN_US;

   if (me_ptr->delay_us != data_path_delay_in_us)
   {
      me_ptr->delay_us                   = data_path_delay_in_us;

      event.delay_in_us                  = data_path_delay_in_us;
      event_info.port_info.is_valid      = false;
      event_info.payload.actual_data_len = sizeof(event);
      event_info.payload.max_data_len    = sizeof(event);
      event_info.payload.data_ptr        = (int8_t *)&event;

      result = me_ptr->cb_info.event_cb(me_ptr->cb_info.event_context, CAPI_EVENT_ALGORITHMIC_DELAY, &event_info);
      if (CAPI_FAILED(result))
      {
         ECNS_DBG(me_ptr->miid, DBG_ERROR_PRIO, "Failed to send delay update event with %lu", result);
      }
      else
      {
         ECNS_DBG(me_ptr->miid, DBG_HIGH_PRIO, "reported algo delay = %ld", data_path_delay_in_us);
      }
   }
}

/* =========================================================================
 * FUNCTION : capi_ecns_handle_get_properties
 *
 * DESCRIPTION:
 *    Handle get properties. It is a helper function used to get the static
 * as well as dynamic properties of the module.
 *
 * Static properties include memory requirement, stack size, requires data
 * buffering info, list of framework extensions needed and interface
 * extensions supported by the module.
 * 
 * Dynamic propeteries supported for now is Output media format.
 *
 * ========================================================================= */
capi_err_t capi_ecns_handle_get_properties(capi_ecns_t *me_ptr, capi_proplist_t *proplist_ptr)
{
   uint32_t   i;
   capi_err_t result = CAPI_EOK;

   capi_prop_t *prop_array = proplist_ptr->prop_ptr;

   for (i = 0; i < proplist_ptr->props_num; i++)
   {
      capi_buf_t *   payload_ptr          = &(prop_array[i].payload);
      const uint32_t payload_max_data_len = payload_ptr->max_data_len;

      switch (prop_array[i].id)
      {
      case CAPI_INIT_MEMORY_REQUIREMENT: /* static property */
      {
         if (payload_max_data_len >= sizeof(capi_init_memory_requirement_t))
         {
            capi_init_memory_requirement_t *data_ptr = (capi_init_memory_requirement_t *)(payload_ptr->data_ptr);
            data_ptr->size_in_bytes                  = sizeof(capi_ecns_t);
            payload_ptr->actual_data_len             = sizeof(capi_init_memory_requirement_t);
         }
         else
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "Get property id 0x%lx failed! Bad param size %lu",
                   prop_array[i].id,
                   payload_max_data_len);
            CAPI_SET_ERROR(result, CAPI_ENEEDMORE);
            break;
         }
         break;
      }
      case CAPI_STACK_SIZE: /* static property */
      {
         if (payload_max_data_len >= sizeof(capi_stack_size_t))
         {
            capi_stack_size_t *data_ptr  = (capi_stack_size_t *)payload_ptr->data_ptr;
            data_ptr->size_in_bytes      = ECNS_STACK_SIZE_REQUIREMENT;
            payload_ptr->actual_data_len = sizeof(capi_stack_size_t);
         }
         else
         {
            AR_MSG(DBG_ERROR_PRIO,
                     "Get property id 0x%lx failed! Bad param size %lu",
                     prop_array[i].id,
                     payload_max_data_len);
            CAPI_SET_ERROR(result, CAPI_ENEEDMORE);
            break;
         }
         break;
      }
      case CAPI_IS_INPLACE: /* static property */
      {
         if (payload_max_data_len >= sizeof(capi_is_inplace_t))
         {
            capi_is_inplace_t *data_ptr = (capi_is_inplace_t *)payload_ptr->data_ptr;

            // ECNS is not an inplace module
            data_ptr->is_inplace = FALSE;

            payload_ptr->actual_data_len = sizeof(capi_is_inplace_t);
            break;
         }
         else
         {
            AR_MSG(  DBG_ERROR_PRIO,
                     "Get property id 0x%lx failed! Bad param size %lu",
                     prop_array[i].id,
                     payload_max_data_len);
            CAPI_SET_ERROR(result, CAPI_ENEEDMORE);
            break;
         }
      }
      case CAPI_REQUIRES_DATA_BUFFERING: /* static property */
      {
         if (payload_max_data_len >= sizeof(capi_requires_data_buffering_t))
         {
            capi_requires_data_buffering_t *data_ptr = (capi_requires_data_buffering_t *)payload_ptr->data_ptr;
            data_ptr->requires_data_buffering        = FALSE;
            payload_ptr->actual_data_len             = sizeof(capi_requires_data_buffering_t);
            break;
         }
         else
         {
            AR_MSG( DBG_ERROR_PRIO,
                     "Get property id 0x%lx failed! Bad param size %lu",
                     prop_array[i].id,
                     payload_max_data_len);
            CAPI_SET_ERROR(result, CAPI_ENEEDMORE);
            break;
         }
      }
      case CAPI_NUM_NEEDED_FRAMEWORK_EXTENSIONS: /* static property */
      {
         if (payload_max_data_len >= sizeof(capi_num_needed_framework_extensions_t))
         {
            capi_num_needed_framework_extensions_t *data_ptr =
               (capi_num_needed_framework_extensions_t *)payload_ptr->data_ptr;
            data_ptr->num_extensions     = LEN_OF_FWK_EXTNS_ARRAY;
            payload_ptr->actual_data_len = sizeof(capi_num_needed_framework_extensions_t);
            break;
         }
         else
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "Get property id 0x%lx failed! Bad param size %lu",
                   prop_array[i].id,
                   payload_max_data_len);
            CAPI_SET_ERROR(result, CAPI_ENEEDMORE);
            break;
         }
      }
      case CAPI_NEEDED_FRAMEWORK_EXTENSIONS: /* static property */
      {
         if (payload_max_data_len >= sizeof(capi_framework_extension_id_t))
         {
            capi_framework_extension_id_t *data_ptr = (capi_framework_extension_id_t *)payload_ptr->data_ptr;

            // copy list of framework extensions
            capi_framework_extension_id_t list[LEN_OF_FWK_EXTNS_ARRAY] = FWK_EXTNS_ARRAY;
            memscpy(data_ptr,
                    payload_max_data_len,
                    &list[0],
                    sizeof(capi_framework_extension_id_t) * LEN_OF_FWK_EXTNS_ARRAY);

            payload_ptr->actual_data_len = sizeof(capi_framework_extension_id_t) * LEN_OF_FWK_EXTNS_ARRAY;
            break;
         }
         else
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "Get property id 0x%lx failed! Bad param size %lu",
                   prop_array[i].id,
                   payload_max_data_len);
            CAPI_SET_ERROR(result, CAPI_ENEEDMORE);
            break;
         }
      }
      case CAPI_INTERFACE_EXTENSIONS: /* static property */
      {
         if (payload_ptr->max_data_len >= sizeof(capi_interface_extns_list_t))
         {
            capi_interface_extns_list_t *intf_ext_list = (capi_interface_extns_list_t *)payload_ptr->data_ptr;
            if (payload_ptr->max_data_len < (sizeof(capi_interface_extns_list_t) +
                                             (intf_ext_list->num_extensions * sizeof(capi_interface_extn_desc_t))))
            {
               AR_MSG(DBG_ERROR_PRIO, "Invalid interface extension list size %lu", payload_ptr->max_data_len);
               CAPI_SET_ERROR(result, CAPI_ENEEDMORE);
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
                  case INTF_EXTN_IMCL:
                  {
                     curr_intf_extn_desc_ptr->is_supported = FALSE;
                     break;
                  }
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
                         "Interface extn = 0x%lx, is_supported = %d",
                         curr_intf_extn_desc_ptr->id,
                         (int)curr_intf_extn_desc_ptr->is_supported);
                  curr_intf_extn_desc_ptr++;
               }
            }
         }
         else
         {
            AR_MSG(DBG_ERROR_PRIO, "Bad interface extenstion param size %lu", payload_ptr->max_data_len);
            CAPI_SET_ERROR(result, CAPI_ENEEDMORE);
         }
         break;
      }
      case CAPI_OUTPUT_MEDIA_FORMAT_V2: /* Dynamic property */
      {
         if (NULL == me_ptr)
         {
            AR_MSG(DBG_ERROR_PRIO, "Null ptr while querying output media format");
            return CAPI_EBADPARAM;
         }

         /* Check validity of the ouptut port index*/
         uint32_t output_port_index = prop_array[i].port_info.port_index;

         if (TRUE == prop_array[i].port_info.is_input_port ||
             (output_port_index >= me_ptr->num_port_info.num_output_ports))
         {
            ECNS_DBG(me_ptr->miid,
                     DBG_ERROR_PRIO,
                     "Get property id 0x%lx failed! Invalid output port index (%lu)",
                     prop_array[i].id,
                     output_port_index);
            return CAPI_EBADPARAM;
         }

         // Validate the MF payload
         if (payload_ptr->max_data_len < sizeof(capi_ecns_media_fmt_v2_t))
         {
            ECNS_DBG(me_ptr->miid, DBG_ERROR_PRIO, "Not valid media format size %d", payload_ptr->actual_data_len);
            return CAPI_EBADPARAM;
         }

         capi_ecns_media_fmt_v2_t *media_fmt_ptr = (capi_ecns_media_fmt_v2_t *)(payload_ptr->data_ptr);

         /* NOTE: 
         
            populate valid output media format info in the return payload ptr and update the 
            actual_data_len

            This should be the expected output media format for the given output port index i.e
            prop_array[i].port_info.port_index. 
            
            Check for the validity of the output media format before returning it to the framework. 
          */

         // Memset the media format payload
         memset(media_fmt_ptr, 0, sizeof(capi_ecns_media_fmt_v2_t));

         payload_ptr->actual_data_len = sizeof(capi_ecns_media_fmt_v2_t);
         break;
      }
      default:
      {
         CAPI_SET_ERROR(result, CAPI_EUNSUPPORTED);
      }
      }
   }
   return result;
}

/* =========================================================================
 * FUNCTION : capi_ecns_handle_set_properties
 *
 * DESCRIPTION:
 *    Handle set properties. It is a helper function called by the capi_*init()
 * and capi_*set_properties()
 * ========================================================================= */
capi_err_t capi_ecns_handle_set_properties(capi_ecns_t *me_ptr, capi_proplist_t *proplist_ptr)
{
   capi_err_t   capi_result = CAPI_EOK;
   capi_prop_t *prop_array  = proplist_ptr->prop_ptr;

   uint32_t i;

   for (i = 0; i < proplist_ptr->props_num; i++)
   {
      capi_buf_t *   payload_ptr             = &(prop_array[i].payload);
      const uint32_t payload_actual_data_len = payload_ptr->actual_data_len;

      switch (prop_array[i].id)
      {
      case CAPI_MODULE_INSTANCE_ID:
      {
         if (payload_actual_data_len < sizeof(capi_module_instance_id_t))
         {
            ECNS_DBG(me_ptr->miid,
                     DBG_ERROR_PRIO,
                     "Set property id 0x%lx, Bad param size %lu",
                     prop_array[i].id,
                     payload_actual_data_len);
            CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
            break;
         }

         capi_module_instance_id_t *data_ptr = (capi_module_instance_id_t *)payload_ptr->data_ptr;
         if (data_ptr == NULL)
         {
            ECNS_DBG(me_ptr->miid, DBG_ERROR_PRIO, "callback pointer is NULL");
            CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
         }
         else
         {
            /* Module instance ID info, used in debug messages. */
            me_ptr->miid = data_ptr->module_instance_id;
         }
         break;
      }
      case CAPI_EVENT_CALLBACK_INFO:
      {
         if (payload_actual_data_len < sizeof(capi_event_callback_info_t))
         {
            ECNS_DBG(me_ptr->miid,
                     DBG_ERROR_PRIO,
                     "Set property id 0x%lx, Bad param size %lu",
                     prop_array[i].id,
                     payload_actual_data_len);
            CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
            break;
         }

         capi_event_callback_info_t *data_ptr = (capi_event_callback_info_t *)payload_ptr->data_ptr;
         if (data_ptr == NULL)
         {
            ECNS_DBG(me_ptr->miid, DBG_ERROR_PRIO, "callback pointer is NULL");
            CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
         }
         else
         {
            /* Call back info, for raising any events to the framework*/
            me_ptr->cb_info = *data_ptr;
         }
         break;
      }
      case CAPI_PORT_NUM_INFO:
      {
         if (payload_actual_data_len < sizeof(capi_port_num_info_t))
         {
            ECNS_DBG(me_ptr->miid,
                     DBG_ERROR_PRIO,
                     "Set property id 0x%lx, Bad param size %lu",
                     prop_array[i].id,
                     payload_actual_data_len);
            CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
            break;
         }

         capi_port_num_info_t *num_port_info = (capi_port_num_info_t *)payload_ptr->data_ptr;

         /*Verify number of max input/output ports supported by the module
           
           Currently the example excepts number ports must be equal to the
           supported max number of the modules. In some case, number of ports
           is less than or equal to Max in/out ports. This check can be changed
           if module can support <= max.
          */
         if ((num_port_info->num_input_ports != ECNS_MAX_INPUT_PORTS) ||
             (num_port_info->num_output_ports != ECNS_MAX_OUTPUT_PORTS))
         {
            ECNS_DBG(me_ptr->miid,
                     DBG_ERROR_PRIO,
                     "incorrect number of input (%lu) or output (%lu) ports",
                     num_port_info->num_input_ports,
                     num_port_info->num_output_ports);
            CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
            break;
         }

         me_ptr->num_port_info = *num_port_info;

         ECNS_DBG(me_ptr->miid,
                  DBG_MED_PRIO,
                  "num_input_ports: %lu, num_output_ports: %lu",
                  me_ptr->num_port_info.num_input_ports,
                  me_ptr->num_port_info.num_output_ports);

         break;
      }
      case CAPI_HEAP_ID:
      {
         if (payload_actual_data_len < sizeof(capi_heap_id_t))
         {
            ECNS_DBG(me_ptr->miid, DBG_ERROR_PRIO, "heap id bad size");
            CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
            break;
         }

         capi_heap_id_t *heap_id_ptr = (capi_heap_id_t *)payload_ptr->data_ptr;

         /* This the heap ID used for any dynamic memory allocation by the module.*/
         me_ptr->heap_id = (POSAL_HEAP_ID)heap_id_ptr->heap_id;
         break;
      }
      case CAPI_ALGORITHMIC_RESET:
      {

         /* Reinitialize the alogirthm library if needed, reset is called upon received Subgrap stop on the module.
            And algorithmic reset is per port. Hence module developer must decide if the reset must be called if a
            given input/output port index requires it.
            
            Module must drop any algorithmic histroy buffering during reset and can reset the states corresponding to
            that input port.*/

         bool_t is_reset_requried = FALSE;
         if (0) // Check prop_array[i].port_info.port_index, prop_array[i].port_info.is_input_port requires reset
         {
            is_reset_requried = TRUE;
         }

         if (is_reset_requried)
         {
            // Handle algorithm reset
         }

         break;
      }
      case CAPI_INPUT_MEDIA_FORMAT_V2:
      {
         capi_result = capi_ecns_handle_input_media_format(me_ptr, &prop_array[i].port_info, payload_ptr);
         break;
      }
      default:
      {
         ECNS_DBG(me_ptr->miid,
                  DBG_MED_PRIO,
                  "Unsupported set property id 0x%lx",
                  prop_array[i].id);
      }
      }
   }
   return capi_result;
}

/* =========================================================================
 * FUNCTION : capi_ecns_handle_input_media_format
 *
 * DESCRIPTION:
 *    Helper function to handle Input media format set property.
 * ========================================================================= */
static capi_err_t capi_ecns_handle_input_media_format(capi_ecns_t *     me_ptr,
                                                      capi_port_info_t *port_info_ptr,
                                                      capi_buf_t *      payload_ptr)
{
   capi_err_t       capi_result     = CAPI_EOK;
   capi_port_info_t input_port_info = *port_info_ptr;

   if (!input_port_info.is_input_port || !input_port_info.is_valid)
   {
      ECNS_DBG(me_ptr->miid, DBG_ERROR_PRIO, "Error!! property is not for input media format port");
      CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      return capi_result;
   }

   // validate the MF payload
   if (payload_ptr->actual_data_len < sizeof(capi_set_get_media_format_t) + sizeof(capi_standard_data_format_v2_t))
   {
      ECNS_DBG(me_ptr->miid, DBG_ERROR_PRIO, "Invalid input smedia format size %d", payload_ptr->actual_data_len);
      return CAPI_ENEEDMORE;
   }

   capi_ecns_media_fmt_v2_t *ecns_data_format_ptr       = (capi_ecns_media_fmt_v2_t *)(payload_ptr->data_ptr);
   uint32_t                  input_port_index           = input_port_info.port_index;
   bool_t                    input_media_format_changed = FALSE;

   ECNS_DBG(me_ptr->miid, DBG_LOW_PRIO, "setting input media format on port index 0x%lx", input_port_index);

   /* Validate input media format based on the port index */
   capi_result = capi_ecns_validate_input_media_format(me_ptr, input_port_index, ecns_data_format_ptr);
   if (CAPI_FAILED(capi_result))
   {
      ECNS_DBG(me_ptr->miid, DBG_ERROR_PRIO, "Invalid media format received on input port index %lu", input_port_index);
      return capi_result;
   }

   /* Check and handle the input port media format for the port id associated to the "port_index" */
   if (ECNS_PRIMARY_INPUT_STATIC_PORT_ID == me_ptr->in_port_info[input_port_index].port_id)
   {
      /* If the media format doesn't change usually there is nothing to be handled by the module. */
      if (capi_encs_media_fmt_equal(&me_ptr->in_port_info[input_port_index].media_fmt, ecns_data_format_ptr))
      {
         ECNS_DBG(me_ptr->miid,
                  DBG_HIGH_PRIO,
                  "Input media format did not change, nothing to do. returning with result 0x%lx",
                  capi_result);

         CAPI_SET_ERROR(capi_result, CAPI_EOK);
         return capi_result;
      }

      /* Cache the new input media format and update the media format status */
      memscpy(&me_ptr->in_port_info[input_port_index].media_fmt,
              sizeof(capi_ecns_media_fmt_v2_t),
              ecns_data_format_ptr,
              sizeof(capi_ecns_media_fmt_v2_t));

      me_ptr->in_port_info[input_port_index].is_media_fmt_received = TRUE;
      input_media_format_changed                                   = TRUE;

      ECNS_DBG(me_ptr->miid,
               DBG_HIGH_PRIO,
               "Primary input media format: sample_rate = %lu, num_channels = %lu, bps = %lu",
               me_ptr->in_port_info[input_port_index].media_fmt.format.sampling_rate,
               me_ptr->in_port_info[input_port_index].media_fmt.format.num_channels,
               me_ptr->in_port_info[input_port_index].media_fmt.format.bits_per_sample);

      /* Raise threshold for the primary input port. In this example its hardcoded to 10ms i.e ECNS_PROCESS_FRAME_SIZE_IN_10MS
         Frame size must be changed depending upon the module behavior.
          */
      uint32_t frame_size_in_samples = (me_ptr->in_port_info[input_port_index].media_fmt.format.sampling_rate / 1000) *
                                       ECNS_PROCESS_FRAME_SIZE_IN_10MS;

      capi_ecns_raise_port_threshold_event(me_ptr, frame_size_in_samples, input_port_index, TRUE);

      /*  Update primary output media format based on the new primary input media format.
       *  i.e update me_ptr->out_port_info[pri_op_index].media_fmt info based on the current 
       *  media format and raise the event.
       *
       * The number of channels in the output media format can sometimes depend on module 
       * calibration, which will result in number of channels on output not same as primary 
       * input. [For example, beam forming can result in more channels than input].
       */

      // Get Primary output port index.
      uint32_t pri_op_index = 0;
      if (CAPI_FAILED(capi_result =
                         ecns_get_output_port_arr_idx(me_ptr, ECNS_PRIMARY_OUTPUT_STATIC_PORT_ID, &pri_op_index)))
      {
         return CAPI_EFAILED;
      }

      /* 
         # IMPORTANT NOTE # 
         In this example module case, primary output port media format is assumed to be same 
         as primary input. Module developer must assign appropriate output media format depending
         upon the module behavior.
       */
      memscpy(&me_ptr->out_port_info[pri_op_index].media_fmt,
              sizeof(capi_ecns_media_fmt_v2_t),
              &me_ptr->in_port_info[input_port_index].media_fmt,
              sizeof(capi_ecns_media_fmt_v2_t));

      /* Raise the new output media format event to the framework */
      capi_ecns_raise_output_media_format_event(me_ptr);

      /* Raise threshold for the primary output port, this is operational frame size of the module.
         In this example its hardcoded to 10ms i.e ECNS_PROCESS_FRAME_SIZE_IN_10MS  */
      frame_size_in_samples =
         (me_ptr->out_port_info[pri_op_index].media_fmt.format.sampling_rate / 1000) * ECNS_PROCESS_FRAME_SIZE_IN_10MS;

      /* Raise threshold on the primary output port */
      capi_ecns_raise_port_threshold_event(me_ptr, frame_size_in_samples, pri_op_index, FALSE);

   } // End of primary input handling.
   else if (ECNS_REFERENCE_INPUT_STATIC_PORT_ID == me_ptr->in_port_info[input_port_index].port_id)
   {
      /* Handle media format for reference port */
      if (capi_encs_media_fmt_equal(&me_ptr->in_port_info[input_port_index].media_fmt, ecns_data_format_ptr))
      {
         ECNS_DBG(me_ptr->miid,
                  DBG_HIGH_PRIO,
                  "Input media format did not change, nothing to do. returning with result 0x%lx",
                  capi_result);

         CAPI_SET_ERROR(capi_result, CAPI_EOK);
         return capi_result;
      }

      me_ptr->in_port_info[input_port_index].media_fmt             = *ecns_data_format_ptr;
      me_ptr->in_port_info[input_port_index].is_media_fmt_received = TRUE;
      input_media_format_changed                             = TRUE;

      /* Raise threshold for the primary input port. */
      uint32_t frame_size_in_samples =
         (me_ptr->in_port_info[input_port_index].media_fmt.format.sampling_rate / 1000) * ECNS_PROCESS_FRAME_SIZE_IN_10MS;

      capi_ecns_raise_port_threshold_event(me_ptr, frame_size_in_samples, input_port_index, TRUE);

      ECNS_DBG(me_ptr->miid,
               DBG_HIGH_PRIO,
               "Reference port media format: sample_rate = %lu, num_channels = %lu, bps = %lu",
               me_ptr->in_port_info[input_port_index].media_fmt.format.sampling_rate,
               me_ptr->in_port_info[input_port_index].media_fmt.format.num_channels,
               me_ptr->in_port_info[input_port_index].media_fmt.format.bits_per_sample);
   }
   else
   {
      ECNS_DBG(me_ptr->miid, DBG_ERROR_PRIO, "error incorrect port index");
      CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      return capi_result;
   }

   /* When ever this is change in input media format of Primar or reference input,
      algorithm may need to reintialize. And module needs to raise events to update
      KPPS, bandwidth and algorithmic delay 
      
      Note: This is optional if input media format doesn't affect change doesn't affect
      algorithm it doesnt need to handle this scenario. In this example, library is 
      getting intialized/reinitialized if there is any chnage in the media format.
      */
   if ( input_media_format_changed)
   {
      if (CAPI_FAILED(capi_result = capi_ecns_init_library(me_ptr)))
      {
         ECNS_DBG(me_ptr->miid, DBG_ERROR_PRIO, "Library initialization failed.");
         CAPI_SET_ERROR(capi_result, CAPI_EFAILED);
         return capi_result;
      }

      /* Raise KPPS, bandwidth and delay events */
      capi_ecns_update_raise_kpps_event(me_ptr);
      capi_ecns_update_raise_bandwidth_event(me_ptr);
      capi_ecns_update_raise_algorithmic_delay_event(me_ptr);
   }
   return capi_result;
}

/* =========================================================================
 * FUNCTION : capi_ecns_init_library
 *
 * DESCRIPTION:
 *    Initializes or Re-intializes library memory. Also applies
 * cached calibration data.
 * ========================================================================= */
capi_err_t capi_ecns_init_library(capi_ecns_t *me_ptr)
{
   capi_err_t capi_result = CAPI_EOK;

   /* If library is already initalized, de initialize the earlier instane and reinit.*/
   if (me_ptr->is_library_init_done)
   {
      ECNS_DBG(me_ptr->miid,
               DBG_HIGH_PRIO,
               "Library is already initalized, need to reinitalize.");
      capi_ecns_deinit_library(me_ptr);
   }

   /* Check the following conditions before initializing the library,
    *   1. Is all the necessary calibration received.
    *   2. Is media format received on the primary input port.
    * */
   if (0 /*me_ptr->is_calibration_received && me_ptr->in_port_info[port_index].is_media_fmt_received*/)
   {
      ECNS_DBG(me_ptr->miid, DBG_ERROR_PRIO, "Calibration is not received yet.");
      return CAPI_ENOTREADY;
   }

   /* If all the conditions to create library are met, intialize the library */
   me_ptr->is_library_init_done = TRUE;

   return capi_result;
}

/* =========================================================================
 * FUNCTION : capi_ecns_deinit_library
 * DESCRIPTION: Deintialize the library and free dynamic memory allocated
 * for the library.
 * ========================================================================= */
capi_err_t capi_ecns_deinit_library(capi_ecns_t *me_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   // If library is already initalized, de initialize
   if (TRUE == me_ptr->is_library_init_done)
   {
      ECNS_DBG(me_ptr->miid, DBG_MED_PRIO, "Deinit library");
      me_ptr->is_library_init_done = FALSE;
   }

   return capi_result;
}
