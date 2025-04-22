/**
 * \file capi_gapless_port_utils.c
 * \brief
 *     Implementation of utility functions for managing the CAPIV2 gapless module port structures.
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "capi_gapless_i.h"

static capi_err_t capi_gapless_init_cmn_port(capi_gapless_t *me_ptr, capi_gapless_cmn_port_t *cmn_port_ptr);
static capi_err_t capi_gapless_init_in_port(capi_gapless_t *me_ptr, capi_gapless_in_port_t *in_port_ptr);
static capi_err_t capi_gapless_init_out_port(capi_gapless_t *me_ptr, capi_gapless_out_port_t *out_port_ptr);

/**
 * Initialize gapless module ports. Ports are closed until opened explicitly.
 */
capi_err_t capi_gapless_init_all_ports(capi_gapless_t *me_ptr)
{
   capi_err_t capi_result = CAPI_EOK;

   if (NULL == me_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "Init ports received bad pointer, 0x%p", me_ptr);
      CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      return capi_result;
   }

   if ((GAPLESS_NUM_PORTS_INVALID == me_ptr->num_in_ports) || (GAPLESS_NUM_PORTS_INVALID == me_ptr->num_out_ports))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "Number of input and output ports not yet configured "
             "current values %lu and %lu respectively. Cannot init ",
             me_ptr->num_in_ports,
             me_ptr->num_out_ports);
      CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      return capi_result;
   }

   // Initialize the input ports.
   for (uint32_t i = 0; i < me_ptr->num_in_ports; i++)
   {
      capi_result |= capi_gapless_init_in_port(me_ptr, &(me_ptr->in_ports[i]));
   }

   // Initialize the output ports.
   for (uint32_t i = 0; i < me_ptr->num_out_ports; i++)
   {
      capi_result |= capi_gapless_init_out_port(me_ptr, &(me_ptr->out_ports[i]));
   }

   return capi_result;
}

/**
 * Initialize gapless common port. Ports are closed until opened.
 */
static capi_err_t capi_gapless_init_cmn_port(capi_gapless_t *me_ptr, capi_gapless_cmn_port_t *cmn_port_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   if (NULL == me_ptr || NULL == cmn_port_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "Init received bad pointer, 0x%p, 0x%p", me_ptr, cmn_port_ptr);
      CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      return capi_result;
   }

   cmn_port_ptr->index   = GAPLESS_PORT_INDEX_INVALID;
   cmn_port_ptr->port_id = GAPLESS_PORT_ID_INVALID;
   cmn_port_ptr->state   = DATA_PORT_STATE_CLOSED;

   return capi_result;
}

/**
 * Initialize gapless input port.
 */
static capi_err_t capi_gapless_init_in_port(capi_gapless_t *me_ptr, capi_gapless_in_port_t *in_port_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   if (NULL == me_ptr || NULL == in_port_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "Init received bad pointer, 0x%p, 0x%p", me_ptr, in_port_ptr);
      CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      return capi_result;
   }

   in_port_ptr->sdata_circ_buf_ptr = NULL; // TODO initialize this.

   capi_result |= capi_cmn_init_media_fmt_v2(&(in_port_ptr->media_fmt));
   capi_result |= capi_gapless_init_cmn_port(me_ptr, &(in_port_ptr->cmn));

   return capi_result;
}

/**
 * Initialize gapless output port.
 */
static capi_err_t capi_gapless_init_out_port(capi_gapless_t *me_ptr, capi_gapless_out_port_t *out_port_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   if (NULL == me_ptr || NULL == out_port_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "Init received bad pointer, 0x%p, 0x%p", me_ptr, out_port_ptr);
      CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      return capi_result;
   }

   capi_result = capi_gapless_init_cmn_port(me_ptr, &(out_port_ptr->cmn));

   return capi_result;
}

/**
 * Checks if the port id is valid.
 */
bool_t capi_gapless_port_id_is_valid(capi_gapless_t *me_ptr, uint32_t port_id, bool_t is_input)
{
   if (is_input)
   {
      return TRUE;
   }
   else
   {
      // Output ports.
      return TRUE;
   }
   return FALSE;
}

/**
 *  Given port index, fetch the input port structure. Returns NULL if not found.
 */
capi_gapless_in_port_t *capi_gapless_get_in_port_from_index(capi_gapless_t *me_ptr, uint32_t port_index)
{
   capi_gapless_in_port_t *ret_in_port_ptr = NULL;

   if (NULL == me_ptr || (port_index >= me_ptr->num_in_ports))
   {
      if (GAPLESS_PORT_INDEX_INVALID != port_index || NULL == me_ptr)
      {
         AR_MSG(DBG_ERROR_PRIO, "get input port received incorrect info, 0x%p, %d", me_ptr, port_index);
      }
      return ret_in_port_ptr;
   }

   // If ports are closed, then incoming index will not match self index
   if (DATA_PORT_STATE_CLOSED == me_ptr->in_ports[port_index].cmn.state)
   {
      ret_in_port_ptr = &(me_ptr->in_ports[port_index]);
   }

   // Check if the port indices match
   if (me_ptr->in_ports[port_index].cmn.index == port_index)
   {
      ret_in_port_ptr = &(me_ptr->in_ports[port_index]);
   }

   return ret_in_port_ptr;
}

/**
 * Gets the common port strucuture from the index. Returns NULL if not found.
 */
capi_gapless_cmn_port_t *capi_gapless_get_port_cmn_from_index(capi_gapless_t *me_ptr, uint32_t index, bool_t is_input)
{
   capi_gapless_cmn_port_t *ret_port_ptr = NULL;

   if (is_input)
   {
      capi_gapless_in_port_t *in_port_ptr = capi_gapless_get_in_port_from_index(me_ptr, index);
      if (in_port_ptr)
      {
         ret_port_ptr = &(in_port_ptr->cmn);
      }
   }
   else
   {
      capi_gapless_out_port_t *out_port_ptr = capi_gapless_get_out_port(me_ptr);

      // If ports are closed, then incoming index will not match self index
      if (out_port_ptr && ((DATA_PORT_STATE_CLOSED == out_port_ptr->cmn.state) || (index == out_port_ptr->cmn.index)))
      {
         ret_port_ptr = &(out_port_ptr->cmn);
      }
   }
   return ret_port_ptr;
}

/**
 * Returns NULL if the output port hasn't been created (created during num port info prop handling), otherwise returns
 * the output port.
 */
capi_gapless_out_port_t *capi_gapless_get_out_port(capi_gapless_t *me_ptr)
{
   capi_gapless_out_port_t *ret_out_port_ptr = NULL;

   if (GAPLESS_NUM_PORTS_INVALID != me_ptr->num_out_ports)
   {
      ret_out_port_ptr = &(me_ptr->out_ports[0]);
   }
   else
   {
      AR_MSG(DBG_MED_PRIO, "get output port returned NULL due to num_port_info not yet set.");
   }

   return ret_out_port_ptr;
}

/**
 * Gets the input port structure of the active input port. Returns NULL if there is currently no active input port.
 */
capi_gapless_in_port_t *capi_gapless_get_active_in_port(capi_gapless_t *me_ptr)
{
   return capi_gapless_get_in_port_from_index(me_ptr, me_ptr->active_in_port_index);
}

/**
 * Returns the inactive input port, if there is an active port. If there is no active port, returns NULL.
 */
capi_gapless_in_port_t *capi_gapless_get_other_in_port(capi_gapless_t *me_ptr)
{
   if (GAPLESS_PORT_INDEX_INVALID == me_ptr->active_in_port_index)
   {
      return NULL;
   }

   // The only valid port indices are 0 and 1.
   uint32_t other_port_index = me_ptr->active_in_port_index == 0 ? 1 : 0;

   return capi_gapless_get_in_port_from_index(me_ptr, other_port_index);
}

/**
 * Returns whether the input port is active or not.
 */
bool_t capi_gapless_is_in_port_active(capi_gapless_t *me_ptr, capi_gapless_in_port_t *in_port_ptr)
{
   // If no inputs are active, return FALSE.
   if (GAPLESS_PORT_INDEX_INVALID == me_ptr->active_in_port_index)
   {
      return FALSE;
   }

   return in_port_ptr->cmn.index == me_ptr->active_in_port_index;
}

/**
 * Checks if any data was provided on any started input port. If so, makes that port active.
 * This function should not be called if there's already an active port.
 */
capi_err_t capi_gapless_check_assign_new_active_in_port(capi_gapless_t *me_ptr, capi_stream_data_t *input[])
{
   capi_err_t result = CAPI_EOK;

   if (GAPLESS_PORT_INDEX_INVALID != me_ptr->active_in_port_index)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "Error: entered capi_gapless_check_assign_new_active_in_port but port %d is already active.",
             me_ptr->active_in_port_index);
      return CAPI_EFAILED;
   }

   // Check for data on any started port.
   for (uint32_t i = 0; i < me_ptr->num_in_ports; i++)
   {
      capi_gapless_in_port_t *in_port_ptr = capi_gapless_get_in_port_from_index(me_ptr, i);

      if (in_port_ptr && (DATA_PORT_STATE_STARTED == in_port_ptr->cmn.state))
      {
         capi_stream_data_v2_t *in_sdata_ptr = (capi_stream_data_v2_t *)input[i];

         bool_t has_data = gapless_sdata_has_data(me_ptr, in_sdata_ptr);

         if (has_data)
         {
#ifdef CAPI_GAPLESS_DEBUG
            AR_MSG(DBG_MED_PRIO, "Found data on port idx %ld - making port active.", i);
#endif

            if (!capi_gapless_is_supported_media_type(&(in_port_ptr->media_fmt)))
            {
               AR_MSG(DBG_ERROR_PRIO, "Error: data found on port %d but input media format is invalid.", i);
               CAPI_SET_ERROR(result, CAPI_EFAILED);
               return result;
            }

            me_ptr->active_in_port_index = i;
            result |= capi_gapless_set_operating_media_format(me_ptr, &(in_port_ptr->media_fmt));
            break;
         }
      }
   }

   return result;
}

/**
 * Handling for port open. Fields have already been validated. Store the port index, move port state to STOPPED. Note
 * that port info exists regardless of whether ports have been opened.
 */
capi_err_t capi_gapless_port_open(capi_gapless_t *me_ptr, bool_t is_input, uint32_t port_index, uint32_t port_id)
{

#ifdef CAPI_GAPLESS_DEBUG
   AR_MSG(DBG_MED_PRIO,
          "handling port open, is_input = %ld, port index = %ld, port_id = %ld",
          is_input,
          port_index,
          port_id);
#endif

   capi_err_t               capi_result  = CAPI_EOK;
   capi_gapless_cmn_port_t *port_cmn_ptr = capi_gapless_get_port_cmn_from_index(me_ptr, port_index, is_input);

   if (!port_cmn_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "Couldnt find port info for index %ld, is_input %lu", port_index, is_input);
      return CAPI_EFAILED;
   }

   // Check if already opened. This shouldn't happen.
   if (DATA_PORT_STATE_CLOSED != port_cmn_ptr->state)
   {
      AR_MSG(DBG_ERROR_PRIO, "Port already opened. port_index %ld, is_input %lu", port_index, is_input);
      return CAPI_EFAILED;
   }

   port_cmn_ptr->index   = port_index;
   port_cmn_ptr->port_id = port_id;
   port_cmn_ptr->state   = DATA_PORT_STATE_STOPPED;

   // If we are opening an output port and the operating media format already exists, raise output media format
   // immediately.
   if (!is_input && capi_gapless_is_supported_media_type(&(me_ptr->operating_media_fmt)))
   {
      capi_result |=
         capi_cmn_output_media_fmt_event_v2(&me_ptr->cb_info, &(me_ptr->operating_media_fmt), is_input, port_index);
   }

   return capi_result;
}

/**
 * Handling for port close. Fields have already been validated. Invalidate the port index, move port state to CLOSED.
 */
capi_err_t capi_gapless_port_close(capi_gapless_t *me_ptr, uint32_t port_index, bool_t is_input)
{

#ifdef CAPI_GAPLESS_DEBUG
   AR_MSG(DBG_MED_PRIO, "Handling port close, port_idx = %ld, is_input = %ld.", port_index, is_input);
#endif

   capi_err_t               capi_result  = CAPI_EOK;
   capi_gapless_cmn_port_t *port_cmn_ptr = capi_gapless_get_port_cmn_from_index(me_ptr, port_index, is_input);

   if (!port_cmn_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "Couldnt find port info for index %ld, is_input %lu", port_index, is_input);
      return CAPI_EFAILED;
   }

   // Check if already closed. This shouldn't happen.
   if (DATA_PORT_STATE_CLOSED == port_cmn_ptr->state)
   {
      AR_MSG(DBG_ERROR_PRIO, "Port already closed. port_index %lu, is_input %lu", port_cmn_ptr->index, is_input);
      return CAPI_EFAILED;
   }

   // Update the state of the port being closed
   port_cmn_ptr->index   = GAPLESS_PORT_INDEX_INVALID;
   port_cmn_ptr->port_id = GAPLESS_PORT_ID_INVALID;
   port_cmn_ptr->state   = DATA_PORT_STATE_CLOSED;

   // Deallocate port buffer if it exists (only for input ports). Also clear buffered data/metadata.
   if (is_input)
   {
      capi_gapless_in_port_t *in_port_ptr = (capi_gapless_in_port_t *)port_cmn_ptr;
      if (capi_gapless_does_delay_buffer_exist(me_ptr, in_port_ptr))
      {
         capi_gapless_destroy_delay_buffer(me_ptr, in_port_ptr);
      }
   }

   return capi_result;
}

/**
 * Handling for port start. Fields have already been validated. Move port state to STARTED.
 */
capi_err_t capi_gapless_port_start(capi_gapless_t *me_ptr, uint32_t port_index, bool_t is_input)
{

#ifdef CAPI_GAPLESS_DEBUG
   AR_MSG(DBG_MED_PRIO, "Handling port start, port_idx = %ld, is_input = %ld.", port_index, is_input);
#endif

   capi_err_t               capi_result  = CAPI_EOK;
   capi_gapless_cmn_port_t *port_cmn_ptr = capi_gapless_get_port_cmn_from_index(me_ptr, port_index, is_input);

   if (!port_cmn_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "Couldnt find port info for index %ld, is_input %lu", port_index, is_input);
      return CAPI_EFAILED;
   }

   // Check if already closed. This shouldn't happen.
   if (DATA_PORT_STATE_CLOSED == port_cmn_ptr->state)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "Port receiving start shouldn't be closed. port_index %lu, is_input %lu",
             port_cmn_ptr->index,
             is_input);
      return CAPI_EFAILED;
   }

   // Update the state of the port being started
   port_cmn_ptr->state = DATA_PORT_STATE_STARTED;

   if (is_input)
   {
      capi_gapless_in_port_t *in_port_ptr = (capi_gapless_in_port_t *)port_cmn_ptr;
      in_port_ptr->found_valid_timestamp  = FALSE;
   }

   gapless_check_update_trigger_policy(me_ptr);
   return capi_result;
}

/**
 * Handling for port stop. Fields have already been validated. Move port state to STOPPED.
 */
capi_err_t capi_gapless_port_stop(capi_gapless_t *me_ptr, uint32_t port_index, bool_t is_input)
{
#ifdef CAPI_GAPLESS_DEBUG
   AR_MSG(DBG_MED_PRIO, "Handling port stop, port_idx = %ld, is_input = %ld.", port_index, is_input);
#endif

   capi_err_t               capi_result  = CAPI_EOK;
   capi_gapless_cmn_port_t *port_cmn_ptr = capi_gapless_get_port_cmn_from_index(me_ptr, port_index, is_input);

   if (!port_cmn_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "Couldnt find port info for index %ld, is_input %lu", port_index, is_input);
      return CAPI_EFAILED;
   }

   // Check if already closed. This shouldn't happen.
   if (DATA_PORT_STATE_CLOSED == port_cmn_ptr->state)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "Port receiving stop shouldn't be closed. port_index %lu, is_input %lu",
             port_cmn_ptr->index,
             is_input);
      return CAPI_EFAILED;
   }

   // Update the state of the port being started
   port_cmn_ptr->state = DATA_PORT_STATE_STOPPED;

   gapless_check_update_trigger_policy(me_ptr);
   return capi_result;
}