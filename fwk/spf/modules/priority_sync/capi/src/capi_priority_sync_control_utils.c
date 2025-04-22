/**
 * \file capi_priority_sync_control_utils.c
 * \brief
 *      Implementation of utility functions for capi control handling (set params, set properties, etc).
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "capi_priority_sync_i.h"
#include "module_cmn_api.h"
#include "capi_intf_extn_data_port_operation.h"

/* =========================================================================
 * Static function declarations
 * =========================================================================*/

static capi_err_t capi_priority_sync_port_open(capi_priority_sync_t *me_ptr,
                                               bool_t                is_primary,
                                               bool_t                is_input,
                                               uint32_t              port_index);
static capi_err_t capi_priority_sync_port_close(capi_priority_sync_t *me_ptr, bool_t is_primary, bool_t is_input);
static capi_err_t capi_priority_sync_set_properties_port_op(capi_priority_sync_t *me_ptr, capi_buf_t *payload_ptr);
static capi_err_t capi_priority_sync_in_port_downgraded_stop(capi_priority_sync_t *me_ptr, bool_t is_primary);

/* =========================================================================
 * Function definitions
 * =========================================================================*/

/**
 * TODO(claguna): If one port has fractional and one port has integral sample rate,
 *                we will run into problems due to non-constant block size.
 */
static bool_t priority_sync_is_supported_media_type(const capi_media_fmt_v2_t *format_ptr)
{
   if (CAPI_FIXED_POINT != format_ptr->header.format_header.data_format)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "capi priority sync: unsupported data format %lu",
             (uint32_t)format_ptr->header.format_header.data_format);
      return FALSE;
   }

   if ((16 != format_ptr->format.bits_per_sample) && (24 != format_ptr->format.bits_per_sample) &&
       (32 != format_ptr->format.bits_per_sample))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "capi priority sync: only supports 16, 24 and 32 bit data. Received %lu.",
             format_ptr->format.bits_per_sample);
      return FALSE;
   }

   if ((format_ptr->format.data_interleaving != CAPI_DEINTERLEAVED_UNPACKED) && (format_ptr->format.num_channels != 1))
   {
      AR_MSG(DBG_ERROR_PRIO, "capi priority sync : Interleaved data not supported.");
      return FALSE;
   }

   if (!format_ptr->format.data_is_signed)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi priority sync: Unsigned data not supported.");
      return FALSE;
   }

   if ((format_ptr->format.num_channels == 0) || (format_ptr->format.num_channels > CAPI_MAX_CHANNELS_V2))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "capi priority sync: unsupported number of channels. "
             "Received %lu. Max channel: %lu",
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
static capi_err_t capi_priority_sync_port_open(capi_priority_sync_t *me_ptr,
                                               bool_t                is_primary,
                                               bool_t                is_input,
                                               uint32_t              port_index)
{
   AR_MSG(DBG_MED_PRIO,
          "handling port open, is_primary = %ld, is_input = %ld, port index = %ld",
          is_primary,
          is_input,
          port_index);

   capi_err_t                     capi_result  = CAPI_EOK;
   capi_priority_sync_cmn_port_t *port_cmn_ptr = capi_priority_sync_get_port_cmn(me_ptr, is_primary, is_input);

   // Check if already opened. This shouldn't happen.
   if (DATA_PORT_STATE_CLOSED != port_cmn_ptr->state)
   {
      PS_MSG(me_ptr->miid,
             DBG_ERROR_PRIO,
             "capi priority sync: Port already opened. is_primary %lu, is_input %lu",
             is_primary,
             is_input);
      return CAPI_EFAILED;
   }

   capi_priority_sync_init_cmn_port(me_ptr, port_cmn_ptr);

   port_cmn_ptr->index = port_index;
   port_cmn_ptr->state = DATA_PORT_STATE_OPENED;

   if (is_input)
   {
      // To handle open->close->open scenario
      port_cmn_ptr->flow_state = PRIORITY_SYNC_FLOW_STATE_AT_GAP;
      capi_cmn_init_media_fmt_v2(&((capi_priority_sync_in_port_t *)port_cmn_ptr)->media_fmt);
   }

   return capi_result;
}

/**
 * Handling for port close. Fields have already been validated.
 * Invalidate the port index, deallocate port buffer, move port state to CLOSED.
 */
static capi_err_t capi_priority_sync_port_close(capi_priority_sync_t *me_ptr, bool_t is_primary, bool_t is_input)
{
   PS_MSG(me_ptr->miid, DBG_MED_PRIO, "handling port close, is_primary = %ld, is_input = %ld.", is_primary, is_input);

   capi_err_t                     capi_result  = CAPI_EOK;
   capi_priority_sync_cmn_port_t *port_cmn_ptr = capi_priority_sync_get_port_cmn(me_ptr, is_primary, is_input);

   // Check if already closed. This shouldn't happen.
   if (DATA_PORT_STATE_CLOSED == port_cmn_ptr->state)
   {
      PS_MSG(me_ptr->miid,
             DBG_ERROR_PRIO,
             "capi priority sync: Port already closed. is_primary %lu, is_input %lu",
             is_primary,
             is_input);
      return CAPI_EFAILED;
   }

   port_cmn_ptr->index = PRIORITY_SYNC_PORT_INDEX_INVALID;
   port_cmn_ptr->state = DATA_PORT_STATE_CLOSED;

   // Deallocate port buffer if it exists (only for input ports).
   if (is_input)
   {
      capi_priority_sync_clear_buffered_data(me_ptr, is_primary);
      capi_priority_sync_deallocate_port_buffer(me_ptr, (capi_priority_sync_in_port_t *)port_cmn_ptr);
   }

   return capi_result;
}

/**
 * Handling for port start will start. This is sent to the priority sync module when an upstream external input port
 * receives a data start indication (first data data recieved on port).
 *
 * For port start, clear out any buffered data in the secondary port (this is stale data that hasn't been synced yet),
 * set internal state to SYNCING, and raise event to disable the container input data threshold.
 *
 * Note that will start is not port-specific because the only way to make that happen is to trace the connection
 * from priority sync input port backwards to (or forwards from) the container external input port.
 */
capi_err_t capi_priority_sync_port_will_start(capi_priority_sync_t *me_ptr)
{
   capi_err_t capi_result       = CAPI_EOK;
   bool_t     SECONDARY_PATH    = FALSE;
   bool_t     DISABLE_THRESHOLD = FALSE;

   // Ignore if threshold is disabled or both ports are in flowing state. If threshold is disabled, then we are
   // already starting. If both ports in flowing state, then this is a duplicate will_start.
   if (me_ptr->threshold_is_disabled ||
       ((PRIORITY_SYNC_FLOW_STATE_FLOWING == me_ptr->primary_in_port_info.cmn.flow_state) &&
        (PRIORITY_SYNC_FLOW_STATE_FLOWING == me_ptr->secondary_in_port_info.cmn.flow_state)))
   {
      PS_MSG(me_ptr->miid,
             DBG_MED_PRIO,
             "ignoring will start, threshold is disabled: %ld, primary is data flowing: %ld, secondary is data "
             "flowing: %ld",
             me_ptr->threshold_is_disabled,
             PRIORITY_SYNC_FLOW_STATE_FLOWING == me_ptr->primary_in_port_info.cmn.flow_state,
             PRIORITY_SYNC_FLOW_STATE_FLOWING == me_ptr->secondary_in_port_info.cmn.flow_state);
      return capi_result;
   }

   PS_MSG(me_ptr->miid, DBG_MED_PRIO, "handling will start.");

   /* Drop data on secondary path if primary data flow is starting because it may have old timestamp data.
    * Don't need to drop data if timestamp based synchronization is enabled.
    */
   if (!me_ptr->is_ts_based_sync)
   {
      capi_buf_t *sec_bufs_ptr = me_ptr->secondary_in_port_info.int_stream.buf_ptr;
      if (sec_bufs_ptr && sec_bufs_ptr[0].actual_data_len)
      {
         capi_result |= capi_priority_sync_clear_buffered_data(me_ptr, SECONDARY_PATH);
      }
   }

   // Begin synchronizing primary and secondary data.
   me_ptr->synced_state = PRIORITY_SYNC_STATE_STARTING;

   // Disable threshold.
   capi_result |= capi_priority_sync_raise_event_toggle_threshold(me_ptr, DISABLE_THRESHOLD);

   return capi_result;
}

/**
 * Move the input port to at_gap state. If this causes the downgraded state to be stopped, do stop handling.
 */
capi_err_t capi_priority_sync_in_port_flow_gap(capi_priority_sync_t *me_ptr, bool_t is_primary)
{
   bool_t                         IS_INPUT        = TRUE;
   capi_err_t                     capi_result     = CAPI_EOK;
   capi_priority_sync_cmn_port_t *in_port_cmn_ptr = capi_priority_sync_get_port_cmn(me_ptr, is_primary, IS_INPUT);

   if (PRIORITY_SYNC_FLOW_STATE_AT_GAP == in_port_cmn_ptr->flow_state)
   {
      return capi_result;
   }

   PS_MSG(me_ptr->miid, DBG_MED_PRIO, "moving input port to at gap, is_primary = %ld.", is_primary);

   intf_extn_data_port_state_t before_state = capi_priority_sync_get_downgraded_port_state(me_ptr, in_port_cmn_ptr);
   in_port_cmn_ptr->flow_state              = PRIORITY_SYNC_FLOW_STATE_AT_GAP;
   intf_extn_data_port_state_t after_state  = capi_priority_sync_get_downgraded_port_state(me_ptr, in_port_cmn_ptr);

   if (before_state != after_state)
   {
      capi_result |= capi_priority_sync_in_port_downgraded_stop(me_ptr, is_primary);
      capi_result |= capi_priority_sync_handle_is_rt_property(me_ptr);
      capi_priority_sync_handle_tg_policy(me_ptr);
   }

   return capi_result;
}

/**
 * Handling for port start. Caches port state.
 */
static capi_err_t capi_priority_sync_port_start(capi_priority_sync_t *me_ptr, bool_t is_primary, bool_t is_input_port)
{
   PS_MSG(me_ptr->miid,
          DBG_MED_PRIO,
          "handling port start, is_primary = %ld, is input %ld.",
          is_primary,
          is_input_port);

   capi_err_t                     capi_result  = CAPI_EOK;
   capi_priority_sync_cmn_port_t *port_cmn_ptr = capi_priority_sync_get_port_cmn(me_ptr, is_primary, is_input_port);
   port_cmn_ptr->state                         = DATA_PORT_STATE_STARTED;
   return capi_result;
}

/**
 * Handling for input port stop. Fields have already been validated.
 * Move port state to STOPPED. If this is primary port, then the TX path is stopping therefore
 * there is no need to continue the reference path. Drop all data in that case.
 */
static capi_err_t capi_priority_sync_port_stop(capi_priority_sync_t *me_ptr, bool_t is_primary, bool_t is_input_port)
{
   PS_MSG(me_ptr->miid, DBG_MED_PRIO, "handling port stop, is_primary = %ld, is_input %ld.", is_primary, is_input_port);

   capi_err_t                     capi_result  = CAPI_EOK;
   capi_priority_sync_cmn_port_t *port_cmn_ptr = capi_priority_sync_get_port_cmn(me_ptr, is_primary, is_input_port);

   // Check if already stopped. This may happen, no handling is needed.
   if (DATA_PORT_STATE_STOPPED == port_cmn_ptr->state)
   {
      return capi_result;
   }

   // Self stop needs to move back to at gap state.
   if (is_input_port)
   {
      capi_result |= capi_priority_sync_in_port_flow_gap(me_ptr, is_primary);
   }

   port_cmn_ptr->state = DATA_PORT_STATE_STOPPED;

   return capi_result;
}

/**
 * Common handling for port stop and flow state at gap. If this is primary port, then the TX path is stopping therefore
 * there is no need to continue the reference path. Drop all data in that case.
 */
static capi_err_t capi_priority_sync_in_port_downgraded_stop(capi_priority_sync_t *me_ptr, bool_t is_primary)
{
   PS_MSG(me_ptr->miid, DBG_MED_PRIO, "downgrading input port to stop state, is_primary = %ld.", is_primary);

   capi_err_t capi_result    = CAPI_EOK;

   // clear the internal buffer which is stopped.
   capi_priority_sync_clear_buffered_data(me_ptr, is_primary);

   if (is_primary)
   {
      me_ptr->synced_state = PRIORITY_SYNC_STATE_STARTING;
   }
   return capi_result;
}

/**
 * Handles port operation set properties. Payload is validated and then each individual
 * operation is delegated to opcode-specific functions.
 */
static capi_err_t capi_priority_sync_set_properties_port_op(capi_priority_sync_t *me_ptr, capi_buf_t *payload_ptr)
{
   capi_err_t capi_result = CAPI_EOK;

   if (NULL == payload_ptr->data_ptr)
   {
      PS_MSG(me_ptr->miid, DBG_ERROR_PRIO, "capi priority sync: Set property port operation, received null buffer");
      CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      return capi_result;
   }

   // Validate length.
   if (sizeof(intf_extn_data_port_operation_t) > payload_ptr->actual_data_len)
   {
      PS_MSG(me_ptr->miid,
             DBG_ERROR_PRIO,
             "capi priority sync: Set property for port operation, Bad param size %lu",
             payload_ptr->actual_data_len);
      CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
      return capi_result;
   }

   intf_extn_data_port_operation_t *data_ptr = (intf_extn_data_port_operation_t *)(payload_ptr->data_ptr);

   if ((sizeof(intf_extn_data_port_operation_t) + (data_ptr->num_ports * sizeof(intf_extn_data_port_id_idx_map_t))) >
       payload_ptr->actual_data_len)
   {
      PS_MSG(me_ptr->miid,
             DBG_ERROR_PRIO,
             "capi priority sync: Set property for port operation, Bad param size %lu",
             payload_ptr->actual_data_len);
      CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
      return capi_result;
   }

   uint32_t max_ports = data_ptr->is_input_port ? PRIORITY_SYNC_MAX_IN_PORTS : PRIORITY_SYNC_MAX_OUT_PORTS;
   if (max_ports < data_ptr->num_ports)
   {
      PS_MSG(me_ptr->miid,
             DBG_ERROR_PRIO,
             "capi priority sync: Invalid num ports. is_input: %lu, num_ports = %lu, max_input_ports = %lu, "
             "max_output_ports = %lu",
             data_ptr->is_input_port,
             data_ptr->num_ports,
             PRIORITY_SYNC_MAX_IN_PORTS,
             PRIORITY_SYNC_MAX_OUT_PORTS);
      CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      return capi_result;
   }

   // For each port in the operation payload.
   for (uint32_t iter = 0; iter < data_ptr->num_ports; iter++)
   {
      uint32_t primary_id =
         data_ptr->is_input_port ? PRIORITY_SYNC_PRIMARY_IN_PORT_ID : PRIORITY_SYNC_PRIMARY_OUT_PORT_ID;

      uint32_t secondary_id =
         data_ptr->is_input_port ? PRIORITY_SYNC_SECONDARY_IN_PORT_ID : PRIORITY_SYNC_SECONDARY_OUT_PORT_ID;

      uint32_t port_id    = data_ptr->id_idx[iter].port_id;
      uint32_t port_index = data_ptr->id_idx[iter].port_index;
      bool_t   is_primary = FALSE;

      // Validate port id and determine if primary or secondary operation.
      if (primary_id == port_id)
      {
         is_primary = TRUE;

         PS_MSG(me_ptr->miid,
                DBG_LOW_PRIO,
                "Port operation 0x%x performed on primary port idx = %lu, id= %lu is_input_port = %lu",
                data_ptr->opcode,
                port_index,
                data_ptr->id_idx[iter].port_id,
                data_ptr->is_input_port);
      }
      else if (secondary_id == port_id)
      {
         is_primary = FALSE;

         PS_MSG(me_ptr->miid,
                DBG_LOW_PRIO,
                "Port operation 0x%x performed on secondary port = idx %lu, id= %lu is_input_port = %lu",
                data_ptr->opcode,
                port_index,
                data_ptr->id_idx[iter].port_id,
                data_ptr->is_input_port);
      }
      else
      {
         PS_MSG(me_ptr->miid,
                DBG_ERROR_PRIO,
                "capi priority sync: unsupported port idx = %lu, id= %lu is_input_port = %lu. Only static ids for "
                "primary/secondary ports are "
                "supported.",
                port_index,
                data_ptr->id_idx[iter].port_id,
                data_ptr->is_input_port);
         CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
         return capi_result;
      }

      // Validate port index doesn't go out of bounds.
      if (port_index >= max_ports)
      {
         PS_MSG(me_ptr->miid,
                DBG_ERROR_PRIO,
                "capi priority sync: Bad parameter in id-idx map on port %lu in payload, port_index = %lu, "
                "is_input = %lu, max in ports = %d, max out ports = %d",
                iter,
                port_index,
                data_ptr->is_input_port,
                PRIORITY_SYNC_MAX_IN_PORTS,
                PRIORITY_SYNC_MAX_OUT_PORTS);

         CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
         return capi_result;
      }

      // Validate that id-index mapping matches what was previously sent, unless mapping doesn't exist yet.
      capi_priority_sync_cmn_port_t *port_cmn_ptr =
         capi_priority_sync_get_port_cmn(me_ptr, is_primary, data_ptr->is_input_port);
      uint32_t prev_index = port_cmn_ptr->index;
      if (PRIORITY_SYNC_PORT_INDEX_INVALID != prev_index)
      {
         if (prev_index != port_index)
         {
            PS_MSG(me_ptr->miid,
                   DBG_ERROR_PRIO,
                   "capi priority sync: Error: id-idx mapping changed on port %lu in payload, port_index = %lu, "
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
            capi_result |= capi_priority_sync_port_open(me_ptr, is_primary, data_ptr->is_input_port, port_index);
            break;
         }
         case INTF_EXTN_DATA_PORT_CLOSE:
         {
            capi_result |= capi_priority_sync_port_close(me_ptr, is_primary, data_ptr->is_input_port);
            break;
         }
         case INTF_EXTN_DATA_PORT_STOP:
         {
            capi_result |= capi_priority_sync_port_stop(me_ptr, is_primary, data_ptr->is_input_port);
            break;
         }
         case INTF_EXTN_DATA_PORT_START:
         {
            capi_result |= capi_priority_sync_port_start(me_ptr, is_primary, data_ptr->is_input_port);

            // force raise threshold disabled (if disabled) event so that container can include this port in trigger
            // policy.
            if (me_ptr->threshold_is_disabled)
            {
               capi_priority_sync_raise_event_toggle_threshold(me_ptr, FALSE);
            }

            break;
         }
         default:
         {
            PS_MSG(me_ptr->miid,
                   DBG_HIGH_PRIO,
                   "capi priority sync: Port operation opcode %lu. Not supported.",
                   data_ptr->opcode);
            CAPI_SET_ERROR(capi_result, CAPI_EUNSUPPORTED);
            break;
         }
      }
   }

   return capi_result;
}

capi_err_t capi_priority_sync_handle_is_rt_property(capi_priority_sync_t *me_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   /*
    * -upstream RT/FTRT state is same for both the output ports.
    * -It is RT if primary input port's upstream is RT.
    * -It is RT if primary input ports' upstream is FTRT and secondary input is in data_flow state and its upstream is
    * RT.
    * -It is FTRT in other cases.
    */
   capi_priority_sync_is_rt_state_t sec_in_us_rt_state =
      (DATA_PORT_STATE_STARTED ==
       capi_priority_sync_get_downgraded_port_state(me_ptr, &me_ptr->secondary_in_port_info.cmn))
         ? me_ptr->secondary_in_port_info.cmn.prop_state.us_rt
         : PRIORITY_SYNC_FTRT;

   bool_t out_us_rt_state = me_ptr->primary_in_port_info.cmn.prop_state.us_rt;

   if (PRIORITY_SYNC_FTRT == out_us_rt_state && PRIORITY_SYNC_RT == sec_in_us_rt_state)
   {
      out_us_rt_state = PRIORITY_SYNC_RT;
   }

   if (DATA_PORT_STATE_CLOSED != me_ptr->primary_out_port_info.cmn.state)
   {
      capi_buf_t                               payload;
      intf_extn_param_id_is_rt_port_property_t event;
      event.is_input = FALSE;
      event.is_rt    = (PRIORITY_SYNC_RT == out_us_rt_state) ? TRUE : FALSE;

      payload.data_ptr        = (int8_t *)&event;
      payload.actual_data_len = payload.max_data_len = sizeof(event);

      if (out_us_rt_state != me_ptr->primary_out_port_info.cmn.prop_state.us_rt)
      {
         me_ptr->primary_out_port_info.cmn.prop_state.us_rt = out_us_rt_state;
         event.port_index                                   = me_ptr->primary_out_port_info.cmn.index;
         capi_result |=
            capi_cmn_raise_data_to_dsp_svc_event(&me_ptr->cb_info, INTF_EXTN_EVENT_ID_IS_RT_PORT_PROPERTY, &payload);
      }

      if (DATA_PORT_STATE_CLOSED != me_ptr->secondary_out_port_info.cmn.state &&
          out_us_rt_state != me_ptr->secondary_out_port_info.cmn.prop_state.us_rt)
      {
         me_ptr->secondary_out_port_info.cmn.prop_state.us_rt = out_us_rt_state;
         event.port_index                                     = me_ptr->secondary_out_port_info.cmn.index;
         capi_result |=
            capi_cmn_raise_data_to_dsp_svc_event(&me_ptr->cb_info, INTF_EXTN_EVENT_ID_IS_RT_PORT_PROPERTY, &payload);
      }
   }

   /*
    * -downstream RT/FTRT state
    * -It is RT if primary output port's downstream is RT.
    * -It is RT if primary output port's downstream is FTRT and secondary input is in data_flow state and secondary
    * output's downstream is RT
    * -It is FTRT in other cases.
    */
   capi_priority_sync_is_rt_state_t sec_out_ds_rt_state =
      (DATA_PORT_STATE_STARTED ==
       capi_priority_sync_get_downgraded_port_state(me_ptr, &me_ptr->secondary_in_port_info.cmn))
         ? me_ptr->secondary_out_port_info.cmn.prop_state.ds_rt
         : PRIORITY_SYNC_FTRT;

   bool_t in_ds_rt_state = me_ptr->primary_out_port_info.cmn.prop_state.ds_rt;

   if (PRIORITY_SYNC_FTRT == in_ds_rt_state && PRIORITY_SYNC_RT == sec_out_ds_rt_state)
   {
      in_ds_rt_state = PRIORITY_SYNC_RT;
   }

   /* reflection of upstream RT property as downstream RT property.
    * if upstream is real time then downstream should also be real time. Consider a case where both the ports are
    * upstream real time and trigger policy is both port-mandatory. Now there can be a jitter between two external input
    * ports and we must have prebuffers to handle this jitter. if both inputs are us_rt and connected to the EC-SYNC
    * then there is a dependency between them due to mandatory trigger policy. This dependency causes similar jitter
    * issue as seen for upstream and downstream real time ports.
    *
    */
   bool_t pri_in_ds_rt_state = in_ds_rt_state | me_ptr->primary_in_port_info.cmn.prop_state.us_rt;
   bool_t sec_in_ds_rt_state = in_ds_rt_state | me_ptr->secondary_in_port_info.cmn.prop_state.us_rt;

   if (DATA_PORT_STATE_CLOSED != me_ptr->primary_in_port_info.cmn.state)
   {
      capi_buf_t                               payload;
      intf_extn_param_id_is_rt_port_property_t event;
      event.is_input = TRUE;

      payload.data_ptr        = (int8_t *)&event;
      payload.actual_data_len = payload.max_data_len = sizeof(event);

      if (pri_in_ds_rt_state != me_ptr->primary_in_port_info.cmn.prop_state.ds_rt)
      {
         me_ptr->primary_in_port_info.cmn.prop_state.ds_rt = pri_in_ds_rt_state;
         event.is_rt                                       = (PRIORITY_SYNC_RT == pri_in_ds_rt_state) ? TRUE : FALSE;
         event.port_index                                  = me_ptr->primary_in_port_info.cmn.index;
         capi_result |=
            capi_cmn_raise_data_to_dsp_svc_event(&me_ptr->cb_info, INTF_EXTN_EVENT_ID_IS_RT_PORT_PROPERTY, &payload);
      }

      if (DATA_PORT_STATE_CLOSED != me_ptr->secondary_in_port_info.cmn.state &&
          sec_in_ds_rt_state != me_ptr->secondary_in_port_info.cmn.prop_state.ds_rt)
      {
         me_ptr->secondary_in_port_info.cmn.prop_state.ds_rt = sec_in_ds_rt_state;
         event.is_rt                                         = (PRIORITY_SYNC_RT == sec_in_ds_rt_state) ? TRUE : FALSE;
         event.port_index                                    = me_ptr->secondary_in_port_info.cmn.index;
         capi_result |=
            capi_cmn_raise_data_to_dsp_svc_event(&me_ptr->cb_info, INTF_EXTN_EVENT_ID_IS_RT_PORT_PROPERTY, &payload);
      }
   }

   if (CAPI_FAILED(capi_result))
   {
      PS_MSG(me_ptr->miid, DBG_ERROR_PRIO, "capi priority sync: Port property event failed.");
   }
   return CAPI_EOK;
}
/**
 * Handles data port property propagation and raises event.
 */
static capi_err_t capi_priority_sync_set_data_port_property(capi_priority_sync_t *me_ptr, capi_buf_t *payload_ptr)
{
   capi_err_t capi_result = CAPI_EOK;

   if (NULL == payload_ptr->data_ptr)
   {
      PS_MSG(me_ptr->miid, DBG_ERROR_PRIO, "capi priority sync: Set port property, received null buffer");
      CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      return capi_result;
   }

   // Validate length.
   if (sizeof(intf_extn_param_id_is_rt_port_property_t) > payload_ptr->actual_data_len)
   {
      PS_MSG(me_ptr->miid,
             DBG_ERROR_PRIO,
             "capi priority sync: Set port property, Bad param size %lu",
             payload_ptr->actual_data_len);
      CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
      return capi_result;
   }

   intf_extn_param_id_is_rt_port_property_t *data_ptr =
      (intf_extn_param_id_is_rt_port_property_t *)(payload_ptr->data_ptr);

   PS_MSG(me_ptr->miid,
          DBG_HIGH_PRIO,
          "capi priority sync: data port property set param "
          "is_input %lu, port index %lu, is_rt %lu",
          data_ptr->is_input,
          data_ptr->port_index,
          data_ptr->is_rt);

   if (data_ptr->is_input)
   {
      if (data_ptr->port_index == me_ptr->secondary_in_port_info.cmn.index)
      {
         me_ptr->secondary_in_port_info.cmn.prop_state.us_rt = data_ptr->is_rt;
      }
      else if (data_ptr->port_index == me_ptr->primary_in_port_info.cmn.index)
      {
         me_ptr->primary_in_port_info.cmn.prop_state.us_rt = data_ptr->is_rt;
      }
   }
   else
   {
      if (data_ptr->port_index == me_ptr->secondary_out_port_info.cmn.index)
      {
         me_ptr->secondary_out_port_info.cmn.prop_state.ds_rt = data_ptr->is_rt;
      }
      else if (data_ptr->port_index == me_ptr->primary_out_port_info.cmn.index)
      {
         me_ptr->primary_out_port_info.cmn.prop_state.ds_rt = data_ptr->is_rt;
      }
   }

   capi_result |= capi_priority_sync_handle_is_rt_property(me_ptr);

   return capi_result;
}

capi_err_t capi_priority_sync_process_set_properties(capi_priority_sync_t *me_ptr, capi_proplist_t *proplist_ptr)
{
   capi_err_t capi_result = CAPI_EOK;

   if (NULL == me_ptr)
   {
      PS_MSG(MIID_UNKNOWN, DBG_ERROR_PRIO, "capi priority sync : Set common property received null ptr");
      return CAPI_EBADPARAM;
   }

   capi_result = capi_cmn_set_basic_properties(proplist_ptr, &me_ptr->heap_info, &me_ptr->cb_info, FALSE);
   if (CAPI_EOK != capi_result)
   {
#ifdef CAPI_PRIORITY_SYNC_DEBUG
      PS_MSG(me_ptr->miid,
             DBG_ERROR_PRIO,
             "capi priority sync: Set basic properties failed with result %lu",
             capi_result);
#endif

      return capi_result;
   }

   capi_prop_t *prop_array = proplist_ptr->prop_ptr;
   uint32_t     i          = 0;

   for (i = 0; i < proplist_ptr->props_num; i++)
   {
      capi_buf_t *payload_ptr = &(prop_array[i].payload);

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
            if (payload_ptr->actual_data_len >= sizeof(capi_port_num_info_t))
            {
               capi_port_num_info_t *data_ptr = (capi_port_num_info_t *)payload_ptr->data_ptr;

               if ((data_ptr->num_input_ports > PRIORITY_SYNC_MAX_IN_PORTS) ||
                   (data_ptr->num_output_ports > PRIORITY_SYNC_MAX_OUT_PORTS))
               {
                  PS_MSG(me_ptr->miid,
                         DBG_ERROR_PRIO,
                         "capi priority sync: Set property id 0x%lx number of input and output ports cannot be more "
                         "than 1",
                         (uint32_t)prop_array[i].id);
                  CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
                  break;
               }
            }
            else
            {
               PS_MSG(me_ptr->miid,
                      DBG_ERROR_PRIO,
                      "capi priority sync: Set, Param id 0x%lx Bad param size %lu",
                      (uint32_t)prop_array[i].id,
                      payload_ptr->actual_data_len);
               CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
               break;
            }
            break;
         }
         case CAPI_MODULE_INSTANCE_ID:
         {
            if (payload_ptr->actual_data_len >= sizeof(capi_module_instance_id_t))
            {
               capi_module_instance_id_t *data_ptr = (capi_module_instance_id_t *)payload_ptr->data_ptr;
               me_ptr->miid                        = data_ptr->module_instance_id;
               PS_MSG(me_ptr->miid,
                      DBG_LOW_PRIO,
                      "capi priority sync: This module-id 0x%08lX, instance-id 0x%08lX",
                      data_ptr->module_id,
                      me_ptr->miid);
            }
            else
            {
               PS_MSG(me_ptr->miid,
                      DBG_ERROR_PRIO,
                      "capi priority sync: Set, Prop id 0x%lx Bad param size %lu",
                      (uint32_t)prop_array[i].id,
                      payload_ptr->actual_data_len);
               CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
               break;
            }
            break;
         }
         case CAPI_INPUT_MEDIA_FORMAT_V2:
         {
            if (sizeof(capi_media_fmt_v2_t) <= payload_ptr->actual_data_len)
            {
               // For output media format event.
               bool_t IS_INPUT_PORT = FALSE;

               capi_media_fmt_v2_t *         data_ptr    = (capi_media_fmt_v2_t *)(payload_ptr->data_ptr);
               capi_priority_sync_in_port_t *in_port_ptr = NULL;

               if (!priority_sync_is_supported_media_type(data_ptr))
               {
                  CAPI_SET_ERROR(capi_result, CAPI_EFAILED);
                  break;
               }

               if (!prop_array[i].port_info.is_valid)
               {
                  PS_MSG(me_ptr->miid, DBG_ERROR_PRIO, "capi priority sync: Media format port info is invalid");
                  CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
                  break;
               }

               // Get pointer to media format for this port (get primary/secondary from id/idx mapping).
               bool_t is_primary = FALSE;

               if (me_ptr->primary_in_port_info.cmn.index == prop_array[i].port_info.port_index)
               {
                  PS_MSG(me_ptr->miid,
                         DBG_HIGH_PRIO,
                         "capi priority sync: Received Input media format on primary input");
                  is_primary  = TRUE;
                  in_port_ptr = &me_ptr->primary_in_port_info;
               }
               else if (me_ptr->secondary_in_port_info.cmn.index == prop_array[i].port_info.port_index)
               {
                  PS_MSG(me_ptr->miid,
                         DBG_HIGH_PRIO,
                         "capi priority sync: Received Input media format on secondary input");
                  is_primary  = FALSE;
                  in_port_ptr = &me_ptr->secondary_in_port_info;
               }
               else
               {
                  PS_MSG(me_ptr->miid,
                         DBG_ERROR_PRIO,
                         "capi priority sync: Media format port info for nonprimary and nonsecondary port idx %ld",
                         prop_array[i].port_info.port_index);
                  CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
                  return capi_result;
               }

               // If new media format is same as old, then there's no necessary handling.
               if (capi_cmn_media_fmt_equal(data_ptr, &(in_port_ptr->media_fmt)))
               {
                  break;
               }

               // Copy and save the input media fmt.
               in_port_ptr->media_fmt = *data_ptr; // Copy.

               // If threshold has been configured, recalculate bytes value since that depends on media format,
               // and reallocate buffers as size has changed.
               //
               // We only allocate/calculate threshold when the primary port received input media format, since
               // everywhere
               // we are using primary port media format as reference media format.
               if (me_ptr->module_config.threshold_us)
               {
                  capi_result |= capi_priority_sync_allocate_port_buffer(me_ptr, in_port_ptr);
               }

               // Raise event for output media format.
               PS_MSG(me_ptr->miid,
                      DBG_HIGH_PRIO,
                      "capi priority sync: Setting media format on port is_primary %ld",
                      is_primary);

               (void)capi_priority_sync_raise_event(me_ptr);

               uint32_t raise_index =
                  is_primary ? me_ptr->primary_out_port_info.cmn.index : me_ptr->secondary_out_port_info.cmn.index;
               capi_result |= capi_cmn_output_media_fmt_event_v2(&me_ptr->cb_info,
                                                                 &(in_port_ptr->media_fmt),
                                                                 IS_INPUT_PORT,
                                                                 raise_index);

               // force raise threshold disabled (if disabled) event so that container can include this output port in
               // trigger policy.
               if (me_ptr->threshold_is_disabled)
               {
                  capi_priority_sync_raise_event_toggle_threshold(me_ptr, FALSE);
               }
            }
            else
            {
               PS_MSG(me_ptr->miid,
                      DBG_ERROR_PRIO,
                      "capi priority sync: Set property id 0x%lx Bad param size %lu",
                      (uint32_t)prop_array[i].id,
                      payload_ptr->actual_data_len);
               CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
               break;
            }
            break;
         }
         case CAPI_OUTPUT_MEDIA_FORMAT:
         default:
         {
#ifdef CAPI_PRIORITY_SYNC_DEBUG
            PS_MSG(me_ptr->miid,
                   DBG_HIGH_PRIO,
                   "capi priority sync: Set property id %#x. Not supported.",
                   prop_array[i].id);
#endif

            CAPI_SET_ERROR(capi_result, CAPI_EUNSUPPORTED);
            break;
         }
      }
      if (CAPI_FAILED(capi_result))
      {
         PS_MSG(me_ptr->miid,
                DBG_HIGH_PRIO,
                "capi priority sync: Set property for %#x failed with opcode %lu",
                prop_array[i].id,
                capi_result);
      }
   }

   return capi_result;
}

capi_err_t capi_priority_sync_process_get_properties(capi_priority_sync_t *me_ptr, capi_proplist_t *proplist_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   uint32_t   miid        = (me_ptr) ? me_ptr->miid : MIID_UNKNOWN;

   uint32_t fwk_extn_ids[PRIORITY_SYNC_NUM_FRAMEWORK_EXTENSIONS] = { FWK_EXTN_SYNC,
                                                                     FWK_EXTN_TRIGGER_POLICY,
                                                                     FWK_EXTN_CONTAINER_FRAME_DURATION };

   capi_basic_prop_t mod_prop;
   mod_prop.init_memory_req    = align_to_8_byte(sizeof(capi_priority_sync_t));
   mod_prop.stack_size         = PRIORITY_SYNC_STACK_SIZE;
   mod_prop.num_fwk_extns      = PRIORITY_SYNC_NUM_FRAMEWORK_EXTENSIONS;
   mod_prop.fwk_extn_ids_arr   = fwk_extn_ids;
   mod_prop.is_inplace         = FALSE;
   mod_prop.req_data_buffering = TRUE;
   mod_prop.max_metadata_size  = 0;

   capi_result = capi_cmn_get_basic_properties(proplist_ptr, &mod_prop);
   if (CAPI_EOK != capi_result)
   {
      PS_MSG(miid,
             DBG_ERROR_PRIO,
             "capi priority sync: Get common basic properties failed with result %lu",
             capi_result);
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
         case CAPI_OUTPUT_MEDIA_FORMAT_SIZE:
         case CAPI_NUM_NEEDED_FRAMEWORK_EXTENSIONS:
         case CAPI_NEEDED_FRAMEWORK_EXTENSIONS:
         {
            // handled in capi common utils.
            break;
         }

         case CAPI_OUTPUT_MEDIA_FORMAT_V2:
         {
            if (NULL == me_ptr)
            {
               PS_MSG(miid, DBG_ERROR_PRIO, "capi priority sync : null ptr while querying output mf");
               return CAPI_EBADPARAM;
            }

            // For mulitport modules, output media format querry must have a valid port info.
            if (!(prop_array[i].port_info.is_valid))
            {
               PS_MSG(miid,
                      DBG_ERROR_PRIO,
                      "capi priority sync : getting output mf on multiport module without specifying output port!");
               return CAPI_EBADPARAM;
            }

            if (prop_array[i].port_info.is_input_port)
            {
               PS_MSG(miid, DBG_ERROR_PRIO, "capi priority sync : can't get output mf of input port!");
               return CAPI_EBADPARAM;
            }

            // Check a correct port index was sent. This is just for validation purposes.
            uint32_t querried_port_index = prop_array[i].port_info.port_index;
            bool_t   is_primary          = FALSE;
            if (me_ptr->primary_out_port_info.cmn.index == querried_port_index)
            {
               is_primary = TRUE;
            }
            else if (me_ptr->secondary_out_port_info.cmn.index == querried_port_index)
            {
               is_primary = FALSE;
            }
            else
            {
               PS_MSG(miid, DBG_ERROR_PRIO, "capi priority sync get: index not assigned to an output port!");
               return CAPI_EBADPARAM;
            }

            /* Validate the MF payload */
            capi_priority_sync_in_port_t *in_port_ptr =
               is_primary ? &me_ptr->primary_in_port_info : &me_ptr->secondary_in_port_info;
            uint32_t req_size = capi_cmn_media_fmt_v2_required_size(in_port_ptr->media_fmt.format.num_channels);
            if (payload_ptr->max_data_len < req_size)
            {
               PS_MSG(miid,
                      DBG_ERROR_PRIO,
                      "capi priority sync : Not enough space for get output media format v2, size %d",
                      payload_ptr->max_data_len);

               capi_result |= CAPI_ENEEDMORE;
               break;
            }

            // Copy proper media format to payload.
            capi_media_fmt_v2_t *media_fmt_ptr = &(in_port_ptr->media_fmt);
            memscpy(payload_ptr->data_ptr, payload_ptr->max_data_len, media_fmt_ptr, req_size);
            payload_ptr->actual_data_len = req_size;
            break;
         }
         case CAPI_PORT_DATA_THRESHOLD:
         {
            if (NULL == me_ptr)
            {
               PS_MSG(miid, DBG_ERROR_PRIO, "capi priority sync : null ptr while querying threshold");
               return CAPI_EBADPARAM;
            }
            uint32_t threshold_in_bytes = 1; // default
            capi_result                 = capi_cmn_handle_get_port_threshold(&prop_array[i], threshold_in_bytes);
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
               PS_MSG(miid, DBG_ERROR_PRIO, "Insufficient get property size.");
               break;
            }

            capi_interface_extn_desc_t *curr_intf_extn_desc_ptr =
               (capi_interface_extn_desc_t *)(payload_ptr->data_ptr + sizeof(capi_interface_extns_list_t));

            for (uint32_t j = 0; j < intf_ext_list->num_extensions; j++)
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
               curr_intf_extn_desc_ptr++;
            }

            break;
         } // CAPI_INTERFACE_EXTENSIONS
         default:
         {
#ifdef CAPI_PRIORITY_SYNC_DEBUG
            PS_MSG(miid,
                   DBG_HIGH_PRIO,
                   "capi priority sync: Get property for ID %#x. Not supported.",
                   prop_array[i].id);
#endif

            CAPI_SET_ERROR(capi_result, CAPI_EUNSUPPORTED);
            break;
         }
      }
      if (CAPI_FAILED(capi_result) && (CAPI_EUNSUPPORTED != capi_result))
      {
         PS_MSG(miid,
                DBG_HIGH_PRIO,
                "capi priority sync: Get property for %#x failed with opcode %lu",
                prop_array[i].id,
                capi_result);
      }
   }
   return capi_result;
}
/**
 * Sets either a parameter value or a parameter structure containing
 * multiple parameters. In the event of a failure, the appropriate error
 * code is returned.
 */
capi_err_t capi_priority_sync_set_param(capi_t *                _pif,
                                        uint32_t                param_id,
                                        const capi_port_info_t *port_info_ptr,
                                        capi_buf_t *            params_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   if (NULL == _pif || NULL == params_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi priority sync: set param received bad pointer, 0x%p, 0x%p", _pif, params_ptr);
      return CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
   }
   capi_priority_sync_t *me_ptr = (capi_priority_sync_t *)(_pif);

   switch (param_id)
   {
      case PARAM_ID_PRIORITY_SYNC_SECONDARY_BUFFERING_MODE:
      {
         if (params_ptr->actual_data_len < sizeof(param_id_priority_sync_secondary_buffering_mode_t))
         {
            PS_MSG(me_ptr->miid,
                   DBG_ERROR_PRIO,
                   "capi priority sync: Param id 0x%lx Bad param size %lu",
                   (uint32_t)param_id,
                   params_ptr->actual_data_len);
            capi_result |= CAPI_ENEEDMORE;
            break;
         }

         param_id_priority_sync_secondary_buffering_mode_t *param_ptr =
            (param_id_priority_sync_secondary_buffering_mode_t *)params_ptr->data_ptr;

         me_ptr->avoid_sec_buf_overflow =
            (PRIORITY_SYNC_SECONDARY_MAINTAIN_OLDEST_DATA == param_ptr->sec_buffering_mode) ? TRUE : FALSE;

         PS_MSG(me_ptr->miid,
                DBG_HIGH_PRIO,
                "avoid secondary buffer overflow %hu [0: allow_sec_buf_overflow, 1: avoid_sec_buf_overflow]",
                me_ptr->avoid_sec_buf_overflow);

         break;
      }
      case PARAM_ID_PRIORITY_SYNC_TIMESTAMP_SYNC:
      {
         if (params_ptr->actual_data_len < sizeof(param_id_priority_sync_timestamp_sync_t))
         {
            PS_MSG(me_ptr->miid,
                   DBG_ERROR_PRIO,
                   "capi priority sync: Param id 0x%lx Bad param size %lu",
                   (uint32_t)param_id,
                   params_ptr->actual_data_len);
            capi_result |= CAPI_ENEEDMORE;
            break;
         }

         param_id_priority_sync_timestamp_sync_t *param_ptr =
            (param_id_priority_sync_timestamp_sync_t *)params_ptr->data_ptr;

         const bool_t is_ts_sync_enabled = (param_ptr->enable) ? TRUE : FALSE;

         if (is_ts_sync_enabled != me_ptr->is_ts_based_sync)
         {
            // check the data flow state of both paths
            bool_t is_primary_buf_allocated = (me_ptr->primary_in_port_info.int_stream.buf_ptr != NULL) ? TRUE : FALSE;
            bool_t is_secondary_buf_allocated =
               (me_ptr->secondary_in_port_info.int_stream.buf_ptr != NULL) ? TRUE : FALSE;
            if (!is_primary_buf_allocated && !is_secondary_buf_allocated)
            {
               PS_MSG(me_ptr->miid,
                      DBG_HIGH_PRIO,
                      "timestamp based synchronization is %d [0: disabled, 1: enabled]",
                      is_ts_sync_enabled);
               me_ptr->is_ts_based_sync = is_ts_sync_enabled;
            }
            else
            {
               PS_MSG(me_ptr->miid,
                      DBG_ERROR_PRIO,
                      "Can't enabled/disable timestamp based synchronization, primary_active %d, secondary_active %d",
                      is_primary_buf_allocated,
                      is_secondary_buf_allocated);
            }
         }
         break;
      }
      case FWK_EXTN_PARAM_ID_CONTAINER_FRAME_DURATION:
      {
         if (params_ptr->actual_data_len < sizeof(fwk_extn_param_id_container_frame_duration_t))
         {
            PS_MSG(me_ptr->miid,
                   DBG_ERROR_PRIO,
                   "capi priority sync: Param id 0x%lx Bad param size %lu",
                   (uint32_t)param_id,
                   params_ptr->actual_data_len);
            capi_result |= CAPI_ENEEDMORE;
            break;
         }

         fwk_extn_param_id_container_frame_duration_t *threshold_param_ptr =
            (fwk_extn_param_id_container_frame_duration_t *)params_ptr->data_ptr;

         PS_MSG(me_ptr->miid,
                DBG_HIGH_PRIO,
                "capi priority sync: Received Set param of fwk container duration us = %ld",
                threshold_param_ptr->duration_us);

         // Set threshold according to new amount. If media fmt already arrived,
         // allocate buffers at new size.
         if (me_ptr->module_config.threshold_us != threshold_param_ptr->duration_us)
         {
            bool_t PRIMARY   = TRUE;
            bool_t SECONDARY = FALSE;

            me_ptr->module_config.threshold_us = threshold_param_ptr->duration_us;

            if (capi_priority_sync_media_fmt_is_valid(me_ptr, PRIMARY))
            {
               capi_priority_sync_clear_buffered_data(me_ptr, PRIMARY);
               capi_result |= capi_priority_sync_allocate_port_buffer(me_ptr, &me_ptr->primary_in_port_info);
            }

            if (capi_priority_sync_media_fmt_is_valid(me_ptr, SECONDARY))
            {
               capi_priority_sync_clear_buffered_data(me_ptr, SECONDARY);
               capi_result |= capi_priority_sync_allocate_port_buffer(me_ptr, &me_ptr->secondary_in_port_info);
            }
         }
         break;
      }
      case FWK_EXTN_SYNC_PARAM_ID_PORT_WILL_START:
      {
         capi_result |= capi_priority_sync_port_will_start(me_ptr);
         break;
      }
      case FWK_EXTN_PARAM_ID_TRIGGER_POLICY_CB_FN:
      {
         if (NULL == params_ptr->data_ptr)
         {
            PS_MSG(me_ptr->miid,
                   DBG_ERROR_PRIO,
                   "capi priority sync: Param id 0x%lx received null buffer",
                   (uint32_t)param_id);
            capi_result |= CAPI_EBADPARAM;
            break;
         }

         if (params_ptr->actual_data_len < sizeof(fwk_extn_param_id_trigger_policy_cb_fn_t))
         {
            PS_MSG(me_ptr->miid,
                   DBG_ERROR_PRIO,
                   "capi priority sync: Param id 0x%lx Bad param size %lu",
                   (uint32_t)param_id,
                   params_ptr->actual_data_len);
            capi_result |= CAPI_ENEEDMORE;
            break;
         }

         fwk_extn_param_id_trigger_policy_cb_fn_t *payload_ptr =
            (fwk_extn_param_id_trigger_policy_cb_fn_t *)params_ptr->data_ptr;
         me_ptr->tg_policy_cb = *payload_ptr;

         break;
      }
      case INTF_EXTN_PARAM_ID_DATA_PORT_OPERATION:
      {
         capi_result |= capi_priority_sync_set_properties_port_op(me_ptr, params_ptr);
         break;
      }
      case INTF_EXTN_PARAM_ID_METADATA_HANDLER:
      {
         if (params_ptr->actual_data_len < sizeof(intf_extn_param_id_metadata_handler_t))
         {
            PS_MSG(me_ptr->miid,
                   DBG_ERROR_PRIO,
                   "capi priority sync: Param id 0x%lx Bad param size %lu",
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
      case INTF_EXTN_PARAM_ID_IS_RT_PORT_PROPERTY:
      {
         capi_result |= capi_priority_sync_set_data_port_property(me_ptr, params_ptr);
         break;
      }
      default:
      {
         PS_MSG(me_ptr->miid, DBG_ERROR_PRIO, "capi priority sync Set, unsupported param ID 0x%x", (int)param_id);
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
capi_err_t capi_priority_sync_get_param(capi_t *                _pif,
                                        uint32_t                param_id,
                                        const capi_port_info_t *port_info_ptr,
                                        capi_buf_t *            params_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   if (NULL == _pif || NULL == params_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi priority sync: Get param received bad pointer, 0x%p, 0x%p", _pif, params_ptr);
      return CAPI_EBADPARAM;
   }

   capi_priority_sync_t *me_ptr = (capi_priority_sync_t *)_pif;

   switch (param_id)
   {
      case FWK_EXTN_PARAM_ID_CONTAINER_FRAME_DURATION:
      {
         if (params_ptr->actual_data_len < sizeof(fwk_extn_param_id_container_frame_duration_t))
         {
            PS_MSG(me_ptr->miid,
                   DBG_ERROR_PRIO,
                   "capi priority sync get: Param id 0x%lx Bad param size %lu",
                   (uint32_t)param_id,
                   params_ptr->actual_data_len);
            capi_result |= CAPI_ENEEDMORE;
            break;
         }

         fwk_extn_param_id_container_frame_duration_t *threshold_param_ptr =
            (fwk_extn_param_id_container_frame_duration_t *)params_ptr->data_ptr;

         threshold_param_ptr->duration_us = me_ptr->module_config.threshold_us;
         params_ptr->actual_data_len      = sizeof(fwk_extn_param_id_container_frame_duration_t);
         break;
      }
      // Nothing to get for FWK_EXTN_SYNC_PARAM_ID_PORT_WILL_START.
      default:
      {
         PS_MSG(me_ptr->miid, DBG_ERROR_PRIO, "capi priority sync get, unsupported param ID 0x%x", (int)param_id);
         CAPI_SET_ERROR(capi_result, CAPI_EUNSUPPORTED);
         break;
      }
   }
   return capi_result;
}

intf_extn_data_port_state_t capi_priority_sync_get_downgraded_port_state(capi_priority_sync_t *         me_ptr,
                                                                         capi_priority_sync_cmn_port_t *port_cmn_ptr)

{
   intf_extn_data_port_state_t downgraded_state = DATA_PORT_STATE_CLOSED;
   if (!me_ptr || !port_cmn_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "Bad function args me_ptr 0x%lx port_cmn_ptr 0x%lx", me_ptr, port_cmn_ptr);
      return downgraded_state;
   }

   switch (port_cmn_ptr->state)
   {
      case DATA_PORT_STATE_CLOSED:
      case DATA_PORT_STATE_OPENED:
      case DATA_PORT_STATE_STOPPED:
      {
         downgraded_state = port_cmn_ptr->state;
         break;
      }
      case DATA_PORT_STATE_STARTED:
      {
         if (PRIORITY_SYNC_FLOW_STATE_AT_GAP == port_cmn_ptr->flow_state)
         {
            downgraded_state = DATA_PORT_STATE_STOPPED;
         }
         // flow_state is PRIORITY_SYNC_FLOW_STATE_FLOWING
         else
         {
            downgraded_state = DATA_PORT_STATE_STARTED;
         }
         break;
      }
      default:
      {
         PS_MSG(me_ptr->miid, DBG_MED_PRIO, "unexpected port state 0x%lx");
         break;
      }
   }

   return downgraded_state;
}
