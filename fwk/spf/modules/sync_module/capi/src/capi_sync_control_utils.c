/**
 * \file capi_sync_control_utils.c
 * \brief
 *     Implementation of utility functions for capi control handling (set params, set properties, etc).
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "capi_sync_i.h"
#include "module_cmn_api.h"
#include "capi_intf_extn_data_port_operation.h"


/* =========================================================================
 * Static function declarations
 * =========================================================================*/

static bool_t sync_is_supported_media_type(const capi_media_fmt_v2_t *format_ptr);
static capi_err_t capi_sync_port_open(capi_sync_t *me_ptr, bool_t is_input, uint32_t port_index, uint32_t port_id);
static capi_err_t capi_sync_port_close(capi_sync_t *me_ptr, uint32_t port_index, bool_t is_input);
static capi_err_t capi_sync_set_properties_port_op(capi_sync_t *me_ptr, capi_buf_t *payload_ptr);
static capi_err_t capi_sync_populate_peer_port_info(capi_sync_t *         me_ptr,
                                                    capi_sync_cmn_port_t *port_cmn_ptr,
                                                    bool_t                is_input);
static capi_err_t capi_sync_validate_inputs_sr(capi_sync_t *me_ptr, capi_media_fmt_v2_t *data_ptr, uint32_t port_index);
static capi_err_t capi_sync_process_input_media_fmt(capi_sync_t *me_ptr, capi_prop_t *prop_ptr);
static capi_err_t capi_sync_process_port_num_info(capi_sync_t *me_ptr, capi_prop_t *prop_ptr);
static capi_err_t capi_sync_get_output_media_fmt(capi_sync_t *me_ptr, capi_prop_t *prop_ptr);

/* =========================================================================
 * Function definitions
 * =========================================================================*/

/**
 * Handles set properties for the Sync Module
 */
capi_err_t capi_sync_process_set_properties(capi_sync_t *me_ptr, capi_proplist_t *proplist_ptr)
{
   capi_err_t capi_result = CAPI_EOK;

   if (NULL == me_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi sync : Set common property received null ptr");
      return CAPI_EBADPARAM;
   }

   AR_MSG(DBG_HIGH_PRIO, "capi sync : Set properties ");

   // Set the common basic properties except port num info
   capi_result = capi_cmn_set_basic_properties(proplist_ptr, &me_ptr->heap_info, &me_ptr->cb_info, FALSE);
   if (CAPI_EOK != capi_result)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi sync: Set basic properties failed with result %lu", capi_result);
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
         case CAPI_ALGORITHMIC_RESET:
         case CAPI_CUSTOM_INIT_DATA:
         {
            break;
         }
         case CAPI_PORT_NUM_INFO:
         {
            capi_result |= capi_sync_process_port_num_info(me_ptr, &(prop_array[i]));
            break;
         }
         case CAPI_INPUT_MEDIA_FORMAT_V2:
         {
            capi_result |= capi_sync_process_input_media_fmt(me_ptr, &(prop_array[i]));
            break;
         }
         case CAPI_MODULE_INSTANCE_ID:
         {
            capi_buf_t *payload_ptr = &(prop_array[i].payload);
            if (payload_ptr->actual_data_len >= sizeof(capi_module_instance_id_t))
            {
               capi_module_instance_id_t *data_ptr = (capi_module_instance_id_t *)payload_ptr->data_ptr;
               me_ptr->miid                        = data_ptr->module_instance_id;
               AR_MSG(DBG_LOW_PRIO,
                      "capi sync: This module-id 0x%08lX, instance-id 0x%08lX",
                      data_ptr->module_id,
                      me_ptr->miid);
            }
            else
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "capi sync: Set property id %#x Bad param size %lu",
                      prop_array[i].id,
                      payload_ptr->actual_data_len);
               CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
            }
            break;
         }
         case CAPI_OUTPUT_MEDIA_FORMAT:
         default:
         {
            AR_MSG(DBG_HIGH_PRIO, "capi sync: Set property id %#x. Not supported.", prop_array[i].id);
            CAPI_SET_ERROR(capi_result, CAPI_EUNSUPPORTED);
            break;
         }
      }
      if (CAPI_FAILED(capi_result))
      {
         AR_MSG(DBG_HIGH_PRIO, "capi sync: Set property for %#x failed with result %lu", prop_array[i].id, capi_result);
      }
   }

   return capi_result;
}

/**
 * Handles get properties for the Sync Module
 */
capi_err_t capi_sync_process_get_properties(capi_sync_t *me_ptr, capi_proplist_t *proplist_ptr)
{
   capi_err_t capi_result = CAPI_EOK;

   uint32_t fwk_extn_ids[SYNC_NUM_FRAMEWORK_EXTENSIONS] = { FWK_EXTN_SYNC, FWK_EXTN_CONTAINER_FRAME_DURATION };

   capi_basic_prop_t mod_prop;
   mod_prop.init_memory_req    = align_to_8_byte(sizeof(capi_sync_t));
   mod_prop.stack_size         = SYNC_STACK_SIZE;
   mod_prop.num_fwk_extns      = SYNC_NUM_FRAMEWORK_EXTENSIONS;
   mod_prop.fwk_extn_ids_arr   = fwk_extn_ids;
   mod_prop.is_inplace         = FALSE;
   mod_prop.req_data_buffering = TRUE;
   mod_prop.max_metadata_size  = 0;

   capi_result = capi_cmn_get_basic_properties(proplist_ptr, &mod_prop);
   if (CAPI_EOK != capi_result)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi sync: Get common basic properties failed with result %lu", capi_result);
      return capi_result;
   }

   capi_prop_t *prop_array = proplist_ptr->prop_ptr;

   for (uint32_t i = 0; i < proplist_ptr->props_num; i++)
   {
      switch (prop_array[i].id)
      {
         case CAPI_INIT_MEMORY_REQUIREMENT:
         case CAPI_STACK_SIZE:
         case CAPI_IS_INPLACE:
         case CAPI_REQUIRES_DATA_BUFFERING:
         case CAPI_OUTPUT_MEDIA_FORMAT_SIZE:
         case CAPI_NUM_NEEDED_FRAMEWORK_EXTENSIONS:
         case CAPI_NEEDED_FRAMEWORK_EXTENSIONS:
         {
            // handled in capi common utils.
            break;
         }
         case CAPI_INTERFACE_EXTENSIONS:
         {
            capi_buf_t *payload_ptr = &prop_array[i].payload;
            if (payload_ptr->max_data_len >= sizeof(capi_interface_extns_list_t))
            {
               capi_interface_extns_list_t *intf_ext_list = (capi_interface_extns_list_t *)payload_ptr->data_ptr;
               if (payload_ptr->max_data_len < (sizeof(capi_interface_extns_list_t) +
                                                (intf_ext_list->num_extensions * sizeof(capi_interface_extn_desc_t))))
               {
                  AR_MSG(DBG_ERROR_PRIO,
                         "capi sync: CAPI_INTERFACE_EXTENSIONS invalid param size %lu",
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
                        case INTF_EXTN_DATA_PORT_OPERATION:
                        {

                           curr_intf_extn_desc_ptr->is_supported = TRUE;
                           break;
                        }
                        case INTF_EXTN_MIMO_MODULE_PROCESS_STATE:
                        {
                           curr_intf_extn_desc_ptr->is_supported = TRUE;
                           me_ptr->is_mimo_process_state_intf_ext_supported = TRUE;
                           break;
                        }

                        default:
                        {
                           curr_intf_extn_desc_ptr->is_supported = FALSE;
                           break;
                        }
                     }
                     AR_MSG(DBG_HIGH_PRIO,
                            "capi sync: CAPI_INTERFACE_EXTENSIONS intf_ext = 0x%lx, is_supported = %d",
                            curr_intf_extn_desc_ptr->id,
                            (int)curr_intf_extn_desc_ptr->is_supported);
                     curr_intf_extn_desc_ptr++;
                  }
               }
            }
            else
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "capi sync: CAPI_INTERFACE_EXTENSIONS Bad param size %lu",
                      payload_ptr->max_data_len);
               CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
            }
            break;
         }
         case CAPI_OUTPUT_MEDIA_FORMAT_V2:
         {
            if (NULL == me_ptr)
            {
               AR_MSG(DBG_ERROR_PRIO, "capi sync : null instance ptr while querying output mf");
               return CAPI_EBADPARAM;
            }

            capi_result |= capi_sync_get_output_media_fmt(me_ptr, &prop_array[i]);
            break;
         }
         case CAPI_PORT_DATA_THRESHOLD:
         {
            if (NULL == me_ptr)
            {
               AR_MSG(DBG_ERROR_PRIO, "capi sync : null instance ptr while querying threshold");
               return CAPI_EBADPARAM;
            }
            if (prop_array[i].payload.max_data_len >= sizeof(capi_port_data_threshold_t))
            {
               capi_port_data_threshold_t *data_ptr = (capi_port_data_threshold_t *)prop_array[i].payload.data_ptr;

               data_ptr->threshold_in_bytes          = 1;
               prop_array[i].payload.actual_data_len = sizeof(capi_port_data_threshold_t);
            }
            else
            {
               AR_MSG(DBG_ERROR_PRIO, "capi_sync : Get property_id CAPI_PORT_DATA_THRESHOLD, Bad param size");
               prop_array[i].payload.actual_data_len = 0;
               CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
            }
            break;
         }
         default:
         {
            AR_MSG(DBG_HIGH_PRIO, "capi sync: Get property for ID %#x. Not supported.", prop_array[i].id);
            CAPI_SET_ERROR(capi_result, CAPI_EUNSUPPORTED);
            break;
         }
      }
      if (CAPI_FAILED(capi_result))
      {
         AR_MSG(DBG_HIGH_PRIO, "capi sync: Get property for %#x failed with result %lu", prop_array[i].id, capi_result);
      }
   }
   return capi_result;
}

/**
 * TODO(claguna): If one port has fractional and one port has integral sample rate,
 *                we will run into problems due to non-constant block size.
 */
static bool_t sync_is_supported_media_type(const capi_media_fmt_v2_t *format_ptr)
{
   if (CAPI_FIXED_POINT != format_ptr->header.format_header.data_format)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "capi sync: unsupported data format %lu",
             (uint32_t)format_ptr->header.format_header.data_format);
      return FALSE;
   }

   if ((16 != format_ptr->format.bits_per_sample) && (32 != format_ptr->format.bits_per_sample))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "capi sync: only supports 16 and 32 bit data. Received %lu.",
             format_ptr->format.bits_per_sample);
      return FALSE;
   }

   if ((format_ptr->format.data_interleaving != CAPI_DEINTERLEAVED_UNPACKED) && (format_ptr->format.num_channels != 1))
   {
      AR_MSG(DBG_ERROR_PRIO, "capi sync : Interleaved data not supported.");
      return FALSE;
   }

   if (!format_ptr->format.data_is_signed)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi sync: Unsigned data not supported.");
      return FALSE;
   }

   if ((format_ptr->format.num_channels == 0) || (format_ptr->format.num_channels > CAPI_MAX_CHANNELS_V2))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "capi sync: unsupported number of channels. "
             "Received %lu. Max channels: %lu",
             format_ptr->format.num_channels,
			 CAPI_MAX_CHANNELS_V2);
      return FALSE;
   }

   return TRUE;
}

/**
 * Handling for port open. Fields have already been validated.
 * Store the port index, move port state to STOPPED. Note that we allocate
 * the port buffer for both ports when threshold is configured. Note that port info
 * exists regardless of whether ports have been opened since we expect both primary
 * and secondary to be opened at some point.
 */
static capi_err_t capi_sync_port_open(capi_sync_t *me_ptr, bool_t is_input, uint32_t port_index, uint32_t port_id)
{
   AR_MSG(DBG_MED_PRIO,
          "handling port open, is_input = %ld, port index = %ld, port_id = %ld",
          is_input,
          port_index,
          port_id);

   capi_err_t            capi_result  = CAPI_EOK;
   capi_sync_cmn_port_t *port_cmn_ptr = capi_sync_get_port_cmn_from_index(me_ptr, port_index, is_input);

   if (!port_cmn_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi sync: Couldnt find port info for index %ld, is_input %lu", port_index, is_input);
      return CAPI_EFAILED;
   }

   // Check if already opened. This shouldn't happen.
   if (CAPI_PORT_STATE_CLOSED != port_cmn_ptr->state)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi sync: Port already opened. port_index %ld, is_input %lu", port_index, is_input);
      return CAPI_EFAILED;
   }

   port_cmn_ptr->self_index   = port_index;
   port_cmn_ptr->self_port_id = port_id;
   port_cmn_ptr->state        = CAPI_PORT_STATE_STOPPED;

   capi_result = capi_sync_populate_peer_port_info(me_ptr, port_cmn_ptr, is_input);

   // check and raise inactive event
   capi_result |= capi_sync_check_raise_out_port_active_inactive_event(me_ptr);

   if (is_input)
   {
      me_ptr->num_opened_in_ports++;

      //enable module if it has multiple input. Syncing is needed
      if (me_ptr->num_opened_in_ports > 1)
      {
         capi_sync_raise_enable_disable_event(me_ptr, TRUE);
      }
   }

   return capi_result;
}

static capi_err_t capi_sync_port_stop(capi_sync_t *me_ptr, uint32_t port_index, bool_t is_input)
{
   AR_MSG(DBG_MED_PRIO, "handling port stop, port_idx = %ld, is_input = %ld.", port_index, is_input);

   capi_err_t            capi_result  = CAPI_EOK;
   capi_sync_cmn_port_t *port_cmn_ptr = capi_sync_get_port_cmn_from_index(me_ptr, port_index, is_input);

   if (!port_cmn_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi sync: Couldnt find port info for index %ld, is_input %lu", port_index, is_input);
      return CAPI_EFAILED;
   }

   port_cmn_ptr->state = CAPI_PORT_STATE_STOPPED;

   if (is_input)
   {
      capi_sync_in_port_t *input_port_ptr = (capi_sync_in_port_t *)port_cmn_ptr;
      capi_sync_clear_buffered_data(me_ptr, input_port_ptr);

      input_port_ptr->will_start_rcvd = FALSE;
   }
   else
   {
      capi_sync_out_port_t *output_port_ptr = (capi_sync_out_port_t *)port_cmn_ptr;
      output_port_ptr->is_ts_valid          = FALSE;
   }

   return capi_result;
}

/**
 * Handling for port close. Fields have already been validated.
 * Invalidate the port index, deallocate port buffer, move port state to CLOSED.
 */
static capi_err_t capi_sync_port_close(capi_sync_t *me_ptr, uint32_t port_index, bool_t is_input)
{
   AR_MSG(DBG_MED_PRIO, "handling port close, port_idx = %ld, is_input = %ld.", port_index, is_input);

   capi_err_t            capi_result  = CAPI_EOK;
   capi_sync_cmn_port_t *port_cmn_ptr = capi_sync_get_port_cmn_from_index(me_ptr, port_index, is_input);

   if (!port_cmn_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi sync: Couldnt find port info for index %ld, is_input %lu", port_index, is_input);
      return CAPI_EFAILED;
   }

   // Check if already closed. This shouldn't happen.
   if (CAPI_PORT_STATE_CLOSED == port_cmn_ptr->state)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "capi sync: Port already closed. port_index %lu, is_input %lu",
             port_cmn_ptr->self_index,
             is_input);
      return CAPI_EFAILED;
   }

   // Remove the connection information from the peer port
   uint32_t peer_index = port_cmn_ptr->conn_index;

   if (SYNC_PORT_INDEX_INVALID != peer_index)
   {
      capi_sync_cmn_port_t *peer_cmn_ptr = capi_sync_get_port_cmn_from_index(me_ptr, peer_index, !is_input);
      if (!peer_cmn_ptr)
      {
         AR_MSG(DBG_ERROR_PRIO,
                "capi sync: Couldnt find peer port info for index %ld, is_input %lu peer_index %ld",
                port_index,
                is_input,
                peer_index);
         return CAPI_EFAILED;
      }
      peer_cmn_ptr->conn_index   = SYNC_PORT_INDEX_INVALID;
      peer_cmn_ptr->conn_port_id = SYNC_PORT_ID_INVALID;

      // If the input port gets closed while it's still in start state, then we need to manually push EOS to the
      // output port on the next process call.
      if (is_input)
      {
         if (CAPI_PORT_STATE_STARTED == port_cmn_ptr->state)
         {
            capi_sync_out_port_t *out_peer_port_ptr = (capi_sync_out_port_t *)peer_cmn_ptr;
            out_peer_port_ptr->needs_eos_at_close   = TRUE;
         }
      }
   }

   // Deallocate port buffer if it exists (only for input ports). Also clear buffered data/metadata.
   if (is_input)
   {
      capi_sync_clear_buffered_data(me_ptr, (capi_sync_in_port_t *)port_cmn_ptr);
      capi_sync_deallocate_port_buffer(me_ptr, (capi_sync_in_port_t *)port_cmn_ptr);
      memset(port_cmn_ptr, 0, sizeof(capi_sync_in_port_t));

      me_ptr->num_opened_in_ports--;
   }
   else
   {
      memset(port_cmn_ptr, 0, sizeof(capi_sync_out_port_t));
   }

   // Update the state of the port being closed
   port_cmn_ptr->self_index   = SYNC_PORT_INDEX_INVALID;
   port_cmn_ptr->conn_index   = SYNC_PORT_INDEX_INVALID;
   port_cmn_ptr->self_port_id = SYNC_PORT_ID_INVALID;
   port_cmn_ptr->conn_port_id = SYNC_PORT_ID_INVALID;
   port_cmn_ptr->state        = CAPI_PORT_STATE_CLOSED;

   // akr: check and raise inactive event
   capi_result = capi_sync_check_raise_out_port_active_inactive_event(me_ptr);

   // If all ports are closed, fwk will push EOS for any ports not yet at gap. So remove any cached eos to push.
   if (0 == me_ptr->num_opened_in_ports)
   {
      for (uint32_t i = 0; i < me_ptr->num_out_ports; i++)
      {
         capi_sync_out_port_t *out_port_ptr = &(me_ptr->out_port_info_ptr[i]);

         if (out_port_ptr->needs_eos_at_close)
         {
            AR_MSG(DBG_HIGH_PRIO,
                   "capi sync: Clearing cached EOS to push on output port index %ld, id 0x%lx, due to all input ports "
                   "closed",
                   out_port_ptr->cmn.self_index,
                   out_port_ptr->cmn.self_port_id);

            out_port_ptr->needs_eos_at_close = FALSE;
         }
      }
   }

   return capi_result;
}

/**
 * Utility function to identify & populate the peer port information
 * given the port being opened & its direction (called when processing INTF_EXTN_DATA_PORT_OPEN property)
 */
static capi_err_t capi_sync_populate_peer_port_info(capi_sync_t *         me_ptr,
                                                    capi_sync_cmn_port_t *port_cmn_ptr,
                                                    bool_t                is_input)
{
   capi_err_t capi_result = CAPI_EOK;

   if (!port_cmn_ptr)
   {
      return CAPI_EBADPARAM;
   }

   uint32_t curr_port_id = port_cmn_ptr->self_port_id;
   uint32_t peer_port_id = SYNC_PORT_ID_INVALID;

   if (is_input)
   {
      // Output port is input_port_id - 1
      peer_port_id = curr_port_id - 1;
   }
   else
   {
      // Paired input port is output_port_id + 1
      peer_port_id = curr_port_id + 1;
   }

   if (SYNC_PORT_ID_INVALID == peer_port_id)
   {
      return CAPI_EFAILED;
   }

   port_cmn_ptr->conn_port_id = peer_port_id;

   AR_MSG(DBG_MED_PRIO, "capi sync: Connected port ids incoming %d peer %d ", curr_port_id, peer_port_id);

   // Since the pairing is for the port IDs, attempt to figure out the index information
   // for the peer port. If this function returns NULL, not to worry. This will be updated
   // when the peer port is opened.
   capi_sync_cmn_port_t *peer_cmn_ptr = capi_sync_get_port_cmn_from_port_id(me_ptr, peer_port_id, !is_input);

   if (!peer_cmn_ptr)
   {
      return CAPI_EOK;
   }

   // Update the connected indices for the incoming port cmn and its peer
   port_cmn_ptr->conn_index   = peer_cmn_ptr->self_index;
   peer_cmn_ptr->conn_index   = port_cmn_ptr->self_index;
   peer_cmn_ptr->conn_port_id = curr_port_id;

   AR_MSG(DBG_MED_PRIO,
          "capi sync: Connected port idxs incoming %d peer %d ",
          port_cmn_ptr->self_index,
          peer_cmn_ptr->self_index);

   return capi_result;
}

/**
 * Handling for port start will start. This is sent to the sync module when an upstream external input port
 * receives a data start indication (apm start or implicitly with first data data received on port).
 *
 * For EC sync module
 * For port start, clear out any buffered data in the secondary port (this is stale data that hasn't been sync'ed yet),
 * set internal state to SYNCING, and raise event to disable the container input data threshold.
 *
 */
capi_err_t capi_sync_process_port_will_start(capi_sync_t *me_ptr)
{
   capi_err_t capi_result       = CAPI_EOK;
   bool_t     DISABLE_THRESHOLD = FALSE;

   for (uint32_t i = 0; i < me_ptr->num_in_ports; i++)
   {
      capi_sync_in_port_t *in_port_ptr = capi_sync_get_in_port_from_index(me_ptr, i);
      if (!in_port_ptr)
      {
         AR_MSG(DBG_ERROR_PRIO, "capi v2 sync: Failed input port lookup for port_idx %d", i);
         return CAPI_EFAILED;
      }

      if (SYNC_PORT_INDEX_INVALID == in_port_ptr->cmn.conn_index)
      {
         AR_MSG(DBG_ERROR_PRIO, "capi v2 sync: Warning! Output port not found for port_idx %d", i);
         in_port_ptr->will_start_rcvd = TRUE;
         in_port_ptr->is_eos_rcvd     = FALSE;

         continue;
      }

      if (can_sync_in_port_start_be_ignored(me_ptr, in_port_ptr))
      {
         AR_MSG(DBG_MED_PRIO, "capi sync: ignoring will start, for port_idx %d", i);
         continue;
      }

      AR_MSG(DBG_MED_PRIO, "capi sync :handling will start for port_idx %d", i);

      in_port_ptr->will_start_rcvd       = TRUE;
      in_port_ptr->is_eos_rcvd           = FALSE;
      in_port_ptr->is_output_sent_once   = FALSE;
      in_port_ptr->is_threshold_disabled = TRUE;

      // Drop trailing end of stopped data if it exists on this input port
      capi_buf_t *bufs_ptr = in_port_ptr->int_bufs_ptr;

      if (bufs_ptr && bufs_ptr[0].actual_data_len)
      {
         capi_result |= capi_sync_clear_buffered_data(me_ptr, in_port_ptr);

         // TODO(claguna): if there was any data in the secondary buffer, we should have sent
         // data content discontinuity downstream. However this is not in process context so
         // that is not possible.
         AR_MSG(DBG_MED_PRIO,
                "Dropping data at port_will_start for index %d. Data content discontinuity marker not added.",
                i);
      }

      // Disable threshold if not already done
      if (!me_ptr->threshold_is_disabled)
      {
         me_ptr->synced_state = STATE_STARTING;
         capi_result |= capi_sync_raise_event_toggle_threshold(me_ptr, DISABLE_THRESHOLD);
      }
   }

   return capi_result;
}

/**
 * Handling for input port stop. Fields have already been validated.
 * Move port state to STOPPED.
 *
 * For EC Sync Case
 *   If this is primary port, then the TX path is stopping therefore
 *   there is no need to continue the reference path. Drop all data in that case. Propagate
 *   data flow gap flag on output.
 */
capi_err_t capi_sync_in_port_stop(capi_sync_t *me_ptr, capi_sync_in_port_t *in_port_ptr, capi_stream_data_t *output[])
{
#ifdef CAPI_SYNC_VERBOSE
   AR_MSG(DBG_MED_PRIO, "capi sync :handling input port stop, for port idx = %d.", in_port_ptr->cmn.self_index);
#endif

   capi_err_t            capi_result     = CAPI_EOK;
   capi_sync_cmn_port_t *in_port_cmn_ptr = &(in_port_ptr->cmn);

   // Check if already stopped. This may happen, no handling is needed.
   if (CAPI_PORT_STATE_STOPPED == in_port_cmn_ptr->state)
   {
      AR_MSG(DBG_MED_PRIO, "capi sync: Input port %lu already stopped, ignoring.", in_port_cmn_ptr->self_index);
      return capi_result;
   }

   in_port_ptr->will_start_rcvd     = FALSE;
   in_port_cmn_ptr->state           = CAPI_PORT_STATE_STOPPED;
   in_port_ptr->is_output_sent_once = FALSE;

   if (MODE_EC_PRIO_INPUT == me_ptr->mode)
   {
      // If secondary stops, don't drop any data because we still have to try to send partial data
      // through to the ec module.
      if (SYNC_EC_PRIMARY_IN_PORT_ID == in_port_cmn_ptr->self_port_id)
      {
         capi_sync_in_port_t *sec_in_port_ptr =
            capi_sync_get_in_port_from_port_id(me_ptr, SYNC_EC_SECONDARY_OUT_PORT_ID);
         if (!sec_in_port_ptr)
         {
            return CAPI_EFAILED;
         }

         // clear both primary & secondary port data and mark discontinuity
         capi_result |= capi_sync_clear_buffered_data(me_ptr, in_port_ptr);
         capi_result |= capi_sync_clear_buffered_data(me_ptr, sec_in_port_ptr);

         // If the primary port stops, we would need to resync when it starts again. This
         // also handles the case when primary stops while secondary is still running.
         me_ptr->synced_state = STATE_STARTING;
      }
   }
   else // generic sync module
   {
      capi_result |= capi_sync_clear_buffered_data(me_ptr, in_port_ptr);
      AR_MSG(DBG_MED_PRIO, "capi sync: Input port %lu stop handling complete", in_port_cmn_ptr->self_index);
   }

   return capi_result;
}

/**
 * Handles port operation set properties. Payload is validated and then each individual
 * operation is delegated to opcode-specific functions.
 */
static capi_err_t capi_sync_set_properties_port_op(capi_sync_t *me_ptr, capi_buf_t *payload_ptr)
{
   capi_err_t capi_result = CAPI_EOK;

   if (NULL == payload_ptr->data_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi sync: Set property port operation, received null buffer");
      CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      return capi_result;
   }

   // Validate length.
   if (sizeof(intf_extn_data_port_operation_t) > payload_ptr->actual_data_len)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "capi sync: Set property for port operation, Bad param size %lu",
             payload_ptr->actual_data_len);
      CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
      return capi_result;
   }

   intf_extn_data_port_operation_t *data_ptr = (intf_extn_data_port_operation_t *)(payload_ptr->data_ptr);

   if ((sizeof(intf_extn_data_port_operation_t) + (data_ptr->num_ports * sizeof(intf_extn_data_port_id_idx_map_t))) >
       payload_ptr->actual_data_len)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "capi sync: Set property for port operation, Bad param size %lu",
             payload_ptr->actual_data_len);
      CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
      return capi_result;
   }

   uint32_t max_ports = capi_sync_get_supported_num_ports(me_ptr, data_ptr->is_input_port);

   if (max_ports < data_ptr->num_ports)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "capi sync: Invalid num ports. is_input: %lu, num_ports = %lu, max_supported_ports = %lu, "
             "mode = %d",
             data_ptr->is_input_port,
             data_ptr->num_ports,
             max_ports,
             me_ptr->mode);
      CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      return capi_result;
   }

   // For each port in the operation payload.
   for (uint32_t iter = 0; iter < data_ptr->num_ports; iter++)
   {

      uint32_t port_id    = data_ptr->id_idx[iter].port_id;
      uint32_t port_index = data_ptr->id_idx[iter].port_index;

      AR_MSG(DBG_HIGH_PRIO, "capi sync: validating port idx = %lu, id= %lu ", port_index, port_id);

      bool_t is_valid_port = capi_sync_port_id_is_valid(me_ptr, port_id, data_ptr->is_input_port);

      if (!is_valid_port)
      {
         AR_MSG(DBG_ERROR_PRIO,
                "capi sync: unsupported port idx = %lu, id= %lu is_input_port = %lu "
                "Mode = %d ",
                data_ptr->id_idx[iter].port_id,
                port_index,
                data_ptr->is_input_port,
                me_ptr->mode);
         CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
         return capi_result;
      }

      // Validate port index doesn't go out of bounds.
      if (port_index >= max_ports)
      {
         AR_MSG(DBG_ERROR_PRIO,
                "capi sync: Bad parameter in id-idx map on port %lu in payload, port_index = %lu, "
                "is_input = %lu, max ports = %d, mode = %d",
                iter,
                port_index,
                data_ptr->is_input_port,
                max_ports,
                me_ptr->mode);

         CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
         return capi_result;
      }

      // Validate that id-index mapping matches what was previously sent, unless mapping doesn't exist yet.
      capi_sync_cmn_port_t *port_cmn_ptr =
         capi_sync_get_port_cmn_from_index(me_ptr, port_index, data_ptr->is_input_port);

      if (!port_cmn_ptr)
      {
         AR_MSG(DBG_ERROR_PRIO,
                "capi sync: Failed to lookup port info for port_index = %lu, is_input = %lu",
                port_index,
                data_ptr->is_input_port);

         CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
         return capi_result;
      }

      uint32_t prev_index = port_cmn_ptr->self_index;
      if (SYNC_PORT_INDEX_INVALID != prev_index)
      {
         if (prev_index != port_index)
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "capi sync: Error: id-idx mapping changed on port %lu in payload, port_index = %lu, "
                   "is_input = %lu",
                   iter,
                   port_index,
                   data_ptr->is_input_port);
            CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
            return capi_result;
         }
      }

      switch ((uint32_t)data_ptr->opcode)
      {
         case INTF_EXTN_DATA_PORT_OPEN:
         {
            capi_result |= capi_sync_port_open(me_ptr, data_ptr->is_input_port, port_index, port_id);
            break;
         }
         case INTF_EXTN_DATA_PORT_START:
         {
            // force raise threshold disabled (if disabled) event so that container can include this port in trigger
            // policy.
            if (me_ptr->threshold_is_disabled)
            {
               capi_sync_raise_event_toggle_threshold(me_ptr, FALSE);
            }

            break;
         }
         case INTF_EXTN_DATA_PORT_CLOSE:
         {
            capi_result |= capi_sync_port_close(me_ptr, port_index, data_ptr->is_input_port);
            break;
         }
         case INTF_EXTN_DATA_PORT_STOP:
         case INTF_EXTN_DATA_PORT_SUSPEND:
         {
            capi_result |= capi_sync_port_stop(me_ptr, port_index, data_ptr->is_input_port);
            break;
         }
         default:
         {
            AR_MSG(DBG_HIGH_PRIO, "capi sync: Port operation opcode %lu. Not supported.", data_ptr->opcode);
            CAPI_SET_ERROR(capi_result, CAPI_EUNSUPPORTED);
            break;
         }
      }
   }

   return capi_result;
}

/**
 * Sets either a parameter value or a parameter structure containing
 * multiple parameters. In the event of a failure, the appropriate error
 * code is returned.
 */
capi_err_t capi_sync_process_set_param(capi_sync_t *           me_ptr,
                                       uint32_t                param_id,
                                       const capi_port_info_t *port_info_ptr,
                                       capi_buf_t *            params_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   if (NULL == me_ptr || NULL == params_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi sync: set param received bad pointer, 0x%p, 0x%p", me_ptr, params_ptr);
      return CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
   }

   switch (param_id)
   {
      case INTF_EXTN_PARAM_ID_DATA_PORT_OPERATION:
      {
         capi_result |= capi_sync_set_properties_port_op(me_ptr, params_ptr);
         capi_sync_raise_event(me_ptr);
         break;
      }
      case INTF_EXTN_PARAM_ID_METADATA_HANDLER:
      {
         if (params_ptr->actual_data_len < sizeof(intf_extn_param_id_metadata_handler_t))
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "capi sync: Param id 0x%lx Bad param size %lu",
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
      case FWK_EXTN_PARAM_ID_CONTAINER_FRAME_DURATION:
      {
         if (params_ptr->actual_data_len < sizeof(fwk_extn_param_id_container_frame_duration_t))
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "capi sync: Param id 0x%lx Bad param size %lu",
                   (uint32_t)param_id,
                   params_ptr->actual_data_len);
            capi_result |= CAPI_ENEEDMORE;
            break;
         }

         fwk_extn_param_id_container_frame_duration_t *frame_len_param_ptr =
            (fwk_extn_param_id_container_frame_duration_t *)params_ptr->data_ptr;

         AR_MSG(DBG_HIGH_PRIO,
                "capi sync: Received Set param of fwk input threshold us = %ld",
                frame_len_param_ptr->duration_us);

         // Set threshold according to new amount. If media fmt already arrived,
         // allocate buffers at new size.
         if (me_ptr->module_config.frame_len_us != frame_len_param_ptr->duration_us)
         {
            me_ptr->module_config.frame_len_us = frame_len_param_ptr->duration_us;

            capi_result |= capi_sync_ext_input_threshold_change(me_ptr);
         }
         break;
      }
      case FWK_EXTN_SYNC_PARAM_ID_PORT_WILL_START:
      {
         capi_result |= capi_sync_process_port_will_start(me_ptr);
         break;
      }
      default:
      {
         AR_MSG(DBG_ERROR_PRIO, "capi sync Set, unsupported param ID 0x%x", (int)param_id);
         CAPI_SET_ERROR(capi_result, CAPI_EUNSUPPORTED);
         break;
      }
   }
   return capi_result;
}

/**
 * Gets either a parameter value or a parameter structure containing
 * multiple parameters. In the event of a failure, the appropriate error
 * code is returned.
 */
capi_err_t capi_sync_process_get_param(capi_sync_t *           me_ptr,
                                       uint32_t                param_id,
                                       const capi_port_info_t *port_info_ptr,
                                       capi_buf_t *            params_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   if (NULL == me_ptr || NULL == params_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi sync: Get param received bad pointer, 0x%p, 0x%p", me_ptr, params_ptr);
      return CAPI_EBADPARAM;
   }

   switch (param_id)
   {
      case FWK_EXTN_PARAM_ID_CONTAINER_FRAME_DURATION:
      {
         if (params_ptr->actual_data_len < sizeof(fwk_extn_param_id_container_frame_duration_t))
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "capi sync get: Param id 0x%lx Bad param size %lu",
                   (uint32_t)param_id,
                   params_ptr->actual_data_len);
            capi_result |= CAPI_ENEEDMORE;
            break;
         }

         fwk_extn_param_id_container_frame_duration_t *threshold_param_ptr =
            (fwk_extn_param_id_container_frame_duration_t *)params_ptr->data_ptr;

         threshold_param_ptr->duration_us = me_ptr->module_config.frame_len_us;
         params_ptr->actual_data_len      = sizeof(fwk_extn_param_id_container_frame_duration_t);
         break;
      }
      // Nothing to get for FWK_EXTN_SYNC_PARAM_ID_PORT_WILL_START.
      default:
      {
         AR_MSG(DBG_ERROR_PRIO, "capi sync get, unsupported param ID 0x%x", (int)param_id);
         CAPI_SET_ERROR(capi_result, CAPI_EUNSUPPORTED);
         break;
      }
   }
   return capi_result;
}

/**
 * Handles port number info set properties. Payload is validated against maximum num supported
 * ports
 */
static capi_err_t capi_sync_process_port_num_info(capi_sync_t *me_ptr, capi_prop_t *prop_ptr)
{
   capi_err_t  capi_result = CAPI_EOK;
   capi_buf_t *payload_ptr = &(prop_ptr->payload);

   if (payload_ptr->actual_data_len < sizeof(capi_port_num_info_t))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "capi sync: Set, Param id 0x%lx Bad param size %lu",
             (uint32_t)prop_ptr->id,
             payload_ptr->actual_data_len);
      CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
      return capi_result;
   }

   // If input & output ports are already allocated
   if ((me_ptr->in_port_info_ptr) || (me_ptr->out_port_info_ptr))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "capi sync: Set property id 0x%lx number of input and output ports already configured "
             "to %lu and %lu respectively",
             (uint32_t)prop_ptr->id,
             me_ptr->num_in_ports,
             me_ptr->num_out_ports);
      CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      return capi_result;
   }

   capi_port_num_info_t *data_ptr  = (capi_port_num_info_t *)payload_ptr->data_ptr;
   bool_t                IS_INPUT  = TRUE;
   bool_t                IS_OUTPUT = FALSE;

   uint32_t max_in_ports  = capi_sync_get_supported_num_ports(me_ptr, IS_INPUT);
   uint32_t max_out_ports = capi_sync_get_supported_num_ports(me_ptr, IS_OUTPUT);

   if ((data_ptr->num_input_ports > max_in_ports) || (data_ptr->num_output_ports > max_out_ports))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "capi sync: Set property id 0x%lx number of input and output ports cannot be more "
             "than %lu and %lu respectively. Received %lu and %lu respectively",
             (uint32_t)prop_ptr->id,
             max_in_ports,
             max_out_ports,
             data_ptr->num_input_ports,
             data_ptr->num_output_ports);
      CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      return capi_result;
   }

   // All sanity checks are complete
   me_ptr->num_in_ports  = data_ptr->num_input_ports;
   me_ptr->num_out_ports = data_ptr->num_output_ports;

   AR_MSG(DBG_HIGH_PRIO, "capi sync: Num in ports %d Num output ports %d", me_ptr->num_in_ports, me_ptr->num_out_ports);

   capi_result |= capi_sync_init_in_out_ports(me_ptr);

   return capi_result;
}

/**
 * Handles output media format get properties.
 *  The sync module derives it output media format from the connected input ports.
 *
 */
static capi_err_t capi_sync_get_output_media_fmt(capi_sync_t *me_ptr, capi_prop_t *prop_ptr)
{
   capi_err_t  capi_result = CAPI_EOK;
   capi_buf_t *payload_ptr = &(prop_ptr->payload);

   // For mulitport modules, output media format query must have a valid port info.
   if (!(prop_ptr->port_info.is_valid))
   {
      AR_MSG(DBG_ERROR_PRIO, "capi sync : getting output mf on multiport module without specifying output port!");
      return CAPI_EBADPARAM;
   }

   if (prop_ptr->port_info.is_input_port)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi sync : can't get output mf of input port!");
      return CAPI_EBADPARAM;
   }

   // Check a correct port index was sent. This is just for validation purposes.
   uint32_t querried_port_index = prop_ptr->port_info.port_index;

   capi_sync_out_port_t *out_port_ptr = capi_sync_get_out_port_from_index(me_ptr, querried_port_index);

   if (!out_port_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi sync get: index %d not assigned to an output port!", querried_port_index);
      return CAPI_EBADPARAM;
   }

   capi_sync_in_port_t *in_port_ptr = capi_sync_get_in_port_from_index(me_ptr, out_port_ptr->cmn.conn_index);
   if (!in_port_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi sync get: index %d not connected to any input port!", querried_port_index);
      return CAPI_EBADPARAM;
   }

   uint32_t req_size = capi_cmn_media_fmt_v2_required_size(in_port_ptr->media_fmt.format.num_channels);
   if (payload_ptr->max_data_len < req_size)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "capi sync : Not enough space for get output media format v2, size %d",
             payload_ptr->max_data_len);

      CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
      return capi_result;
   }

   // Copy proper media format to payload.
   capi_result |= capi_cmn_handle_get_output_media_fmt_v2(prop_ptr, &(in_port_ptr->media_fmt));

   return capi_result;
}

/**
 * Handles input media format set properties.
 */
static capi_err_t capi_sync_process_input_media_fmt(capi_sync_t *me_ptr, capi_prop_t *prop_ptr)
{
   capi_err_t capi_result = CAPI_EOK;

   capi_buf_t *payload_ptr = &(prop_ptr->payload);

   if (sizeof(capi_media_fmt_v2_t) > payload_ptr->actual_data_len)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi sync: Input Media Format Bad param size %lu", payload_ptr->actual_data_len);
      CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
      return capi_result;
   }

   if (!prop_ptr->port_info.is_valid)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi sync: Media format port info is invalid");
      CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      return capi_result;
   }

   // For output media format event.
   bool_t IS_INPUT_PORT = FALSE;

   capi_media_fmt_v2_t *data_ptr    = (capi_media_fmt_v2_t *)(payload_ptr->data_ptr);
   capi_sync_in_port_t *in_port_ptr = NULL;

   if (!sync_is_supported_media_type(data_ptr))
   {
      CAPI_SET_ERROR(capi_result, CAPI_EFAILED);
      return capi_result;
   }

   uint32_t port_index = prop_ptr->port_info.port_index;
   in_port_ptr         = capi_sync_get_in_port_from_index(me_ptr, port_index);

   if (!in_port_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi sync: Failed to find input port info for %d", port_index);
      CAPI_SET_ERROR(capi_result, CAPI_EFAILED);
      return capi_result;
   }

   if (SYNC_PORT_INDEX_INVALID == in_port_ptr->cmn.conn_index)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi sync: No corresponding out port identified for input %d", port_index);
      //CAPI_SET_ERROR(capi_result, CAPI_EFAILED);
      //return capi_result; returning error will result hang in other valid input port as well
   }

   // If new media format is same as the current one, then there's no necessary handling.
   if (capi_cmn_media_fmt_equal(data_ptr, &(in_port_ptr->media_fmt)))
   {
      return CAPI_EOK;
   }

   // RR: for now no error returned by this function. Revisit
   capi_result |= capi_sync_validate_inputs_sr(me_ptr, data_ptr, port_index);

   // Copy and save the input media fmt.
   in_port_ptr->media_fmt = *data_ptr; // Copy.

   // If threshold has been configured, recalculate bytes value since that depends on media format,
   // and reallocate buffers as size has changed.
   //
   // This will be configured even there is no threshold in the chain
   if (me_ptr->module_config.frame_len_us)
   {
      capi_result |= capi_sync_calc_threshold_bytes(me_ptr, in_port_ptr);
      capi_result |= capi_sync_allocate_port_buffer(me_ptr, in_port_ptr);
   }

   // Raise event for output media format.
   AR_MSG(DBG_HIGH_PRIO,
          "capi sync: Setting media format on port %ld, current thresh = %d",
          in_port_ptr->cmn.self_index,
          me_ptr->module_config.frame_len_us);

   // raise events only if connected to valid output port
   if (SYNC_PORT_INDEX_INVALID != in_port_ptr->cmn.conn_index)
   {
      capi_result |= capi_sync_raise_event(me_ptr);
      capi_result |= capi_cmn_output_media_fmt_event_v2(&me_ptr->cb_info,
                                                        &(in_port_ptr->media_fmt),
                                                        IS_INPUT_PORT,
                                                        in_port_ptr->cmn.conn_index);

      // force raise threshold disabled (if disabled) event so that container can include this port in trigger
      // policy.
      if (me_ptr->threshold_is_disabled)
      {
         capi_sync_raise_event_toggle_threshold(me_ptr, FALSE);
      }
   }
   return capi_result;
}

/**
 *  Utility function to validate the sampling rate of the incoming media format
 *  against input ports that already have valid media format.
 *
 *  For now, this function simply prints a warning if the sampling rates do not
 *   match.
 */
static capi_err_t capi_sync_validate_inputs_sr(capi_sync_t *me_ptr, capi_media_fmt_v2_t *data_ptr, uint32_t port_index)
{
   capi_err_t capi_result = AR_EOK;

   for (uint32_t i = 0; i < me_ptr->num_in_ports && i != port_index; i++)
   {
      capi_sync_in_port_t *in_port_ptr = &(me_ptr->in_port_info_ptr[i]);
      if (capi_sync_media_fmt_is_valid(in_port_ptr))
      {
         // RR: For now just print a warning. Update once concluded
         if (in_port_ptr->media_fmt.format.sampling_rate != data_ptr->format.sampling_rate)
         {
            AR_MSG(DBG_HIGH_PRIO,
                   "capi sync: Warning!! Incoming sampling rate %d for port idx %d doesnt match "
                   "existing configuration %d for port idx %d",
                   data_ptr->format.sampling_rate,
                   port_index,
                   in_port_ptr->media_fmt.format.sampling_rate);
         }
      }
   }

   return capi_result;
}
