/**
 * \file capi_sync_port_utils.c
 * \brief
 *       Implementation of utility functions for module port handling
 *  
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "capi_sync_i.h"
#include "module_cmn_api.h"

/* =========================================================================
 * Static function declarations
 * =========================================================================*/
static capi_err_t capi_sync_init_cmn_port(capi_sync_t *me_ptr, capi_sync_cmn_port_t *cmn_port_ptr);

static capi_err_t capi_sync_init_in_port(capi_sync_t *me_ptr, capi_sync_in_port_t *in_port_ptr);

static capi_err_t capi_sync_init_out_port(capi_sync_t *me_ptr, capi_sync_out_port_t *out_port_ptr);

/* =========================================================================
 * Function definitions
 * =========================================================================*/
/**
 * Initialize sync module ports. Ports are closed until opened explicitly
 */
capi_err_t capi_sync_init_in_out_ports(capi_sync_t *me_ptr)
{
   capi_err_t capi_result = CAPI_EOK;

   if (NULL == me_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi sync: Init ports received bad pointer, 0x%p", me_ptr);
      CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      return capi_result;
   }

   if ((0 == me_ptr->num_in_ports) || (0 == me_ptr->num_out_ports))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "capi sync: number of input and output ports not yet configured "
             "current values %lu and %lu respectively. Cannot init ",
             me_ptr->num_in_ports,
             me_ptr->num_out_ports);
      CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      return capi_result;
   }

   // Re-init not allowed
   if ((me_ptr->in_port_info_ptr) || (me_ptr->out_port_info_ptr))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "capi sync: number of input and output ports already configured "
             "to %lu and %lu respectively. Cannot re-init ",
             me_ptr->num_in_ports,
             me_ptr->num_out_ports);
      CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      return capi_result;
   }

   // Allocate memory for the input ports
   uint32_t alloc_size = (me_ptr->num_in_ports) * (sizeof(capi_sync_in_port_t));
   me_ptr->in_port_info_ptr =
      (capi_sync_in_port_t *)posal_memory_malloc(alloc_size, (POSAL_HEAP_ID)me_ptr->heap_info.heap_id);

   if (NULL == me_ptr->in_port_info_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi v2 sync: Couldn't allocate memory for input port structure.");
      return CAPI_ENOMEMORY;
   }

   memset(me_ptr->in_port_info_ptr, 0, alloc_size);

   // Initialize the input ports
   for (uint32_t i = 0; i < me_ptr->num_in_ports; i++)
   {
      capi_result |= capi_sync_init_in_port(me_ptr, &(me_ptr->in_port_info_ptr[i]));
   }

   // Allocate memory for the output ports
   alloc_size = (me_ptr->num_out_ports) * (sizeof(capi_sync_out_port_t));
   me_ptr->out_port_info_ptr =
      (capi_sync_out_port_t *)posal_memory_malloc(alloc_size, (POSAL_HEAP_ID)me_ptr->heap_info.heap_id);

   if (NULL == me_ptr->out_port_info_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi v2 sync: Couldn't allocate memory for output port structure.");
      return CAPI_ENOMEMORY;
   }

   memset(me_ptr->out_port_info_ptr, 0, alloc_size);

   // Initialize the output ports
   for (uint32_t i = 0; i < me_ptr->num_out_ports; i++)
   {
      capi_result |= capi_sync_init_out_port(me_ptr, &(me_ptr->out_port_info_ptr[i]));
   }

   return capi_result;
}

/**
 *  Given port index, fetch the input port structure
 */
capi_sync_in_port_t *capi_sync_get_in_port_from_index(capi_sync_t *me_ptr, uint32_t port_index)
{
   capi_sync_in_port_t *ret_in_port_ptr = NULL;

   if (NULL == me_ptr || (port_index >= me_ptr->num_in_ports))
   {
      AR_MSG(DBG_ERROR_PRIO, "capi sync: get input port received incorrect info, 0x%p, %d", me_ptr, port_index);
      return ret_in_port_ptr;
   }

   // If ports are closed, then incoming index will not match self index
   if (CAPI_PORT_STATE_CLOSED == me_ptr->in_port_info_ptr[port_index].cmn.state)
   {
      ret_in_port_ptr = &(me_ptr->in_port_info_ptr[port_index]);
   }

   // Check if the port indices match
   if (me_ptr->in_port_info_ptr[port_index].cmn.self_index == port_index)
   {
      ret_in_port_ptr = &(me_ptr->in_port_info_ptr[port_index]);
   }

   return ret_in_port_ptr;
}

/**
 *  Given port index, fetch the output port structure
 */
capi_sync_out_port_t *capi_sync_get_out_port_from_index(capi_sync_t *me_ptr, uint32_t port_index)
{
   capi_sync_out_port_t *ret_out_port_ptr = NULL;

   if (NULL == me_ptr || (port_index >= me_ptr->num_out_ports))
   {
      AR_MSG(DBG_ERROR_PRIO, "capi sync: get output port received incorrect info, 0x%p, %d", me_ptr, port_index);
      return ret_out_port_ptr;
   }

   // If ports are closed, then incoming index will not match self index
   if (CAPI_PORT_STATE_CLOSED == me_ptr->out_port_info_ptr[port_index].cmn.state)
   {
      ret_out_port_ptr = &(me_ptr->out_port_info_ptr[port_index]);
   }

   // Check if the port_indices match
   if (me_ptr->out_port_info_ptr[port_index].cmn.self_index == port_index)
   {
      ret_out_port_ptr = &(me_ptr->out_port_info_ptr[port_index]);
   }

   return ret_out_port_ptr;
}

/**
 *  Get the supported number of ports depending on the mode & type of ports
 */
uint32_t capi_sync_get_supported_num_ports(capi_sync_t *me_ptr, bool_t is_input)
{
   uint32_t max_ports = 0;

   if (NULL == me_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi sync: get supported ports received NULL pointer");
      return max_ports;
   }

   if (MODE_EC_PRIO_INPUT == me_ptr->mode)
   {
      max_ports = (is_input) ? EC_SYNC_MAX_IN_PORTS : EC_SYNC_MAX_OUT_PORTS;
   }
   else if (MODE_ALL_EQUAL_PRIO_INPUT == me_ptr->mode)
   {
      max_ports = (is_input) ? SYNC_MAX_IN_PORTS : SYNC_MAX_OUT_PORTS;
   }
   else
   {
      AR_MSG(DBG_ERROR_PRIO, "capi sync: unexpected error. configured mode [%d]", me_ptr->mode);
      return max_ports;
   }

   return max_ports;
}

/**
 * Given a port id & its type, verify if the values are expected
 */
bool_t capi_sync_port_id_is_valid(capi_sync_t *me_ptr, uint32_t port_id, bool_t is_input)
{
   bool_t is_valid = FALSE;

   if (NULL == me_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi sync: is port id valid received NULL pointer");
      return is_valid;
   }

   // For EC inputs, we support only static ids for in/out ports
   if (MODE_EC_PRIO_INPUT == me_ptr->mode)
   {
      if (is_input && ((SYNC_EC_PRIMARY_IN_PORT_ID == port_id) || (SYNC_EC_SECONDARY_IN_PORT_ID == port_id)))
      {
         is_valid = TRUE;
      }

      if (!is_input && ((SYNC_EC_PRIMARY_OUT_PORT_ID == port_id) || (SYNC_EC_SECONDARY_OUT_PORT_ID == port_id)))
      {
         is_valid = TRUE;
      }
   }
   else
   {
      // RR: Revisit this condition once IDs are concluded
      if (port_id >= SYNC_PORT_ID_RANGE_BEGIN)
      {
         is_valid = TRUE;
      }
   }

   return is_valid;
}

/**
 *  Given port id, fetch the common port structure
 */
capi_sync_cmn_port_t *capi_sync_get_port_cmn_from_port_id(capi_sync_t *me_ptr, uint32_t port_id, bool_t is_input)
{
   capi_sync_cmn_port_t *port_cmn_ptr = NULL;

   if (is_input)
   {
      capi_sync_in_port_t *in_port_ptr = capi_sync_get_in_port_from_port_id(me_ptr, port_id);

      if (in_port_ptr)
      {
         port_cmn_ptr = &(in_port_ptr->cmn);
      }
   }
   else
   {
      capi_sync_out_port_t *out_port_ptr = capi_sync_get_out_port_from_port_id(me_ptr, port_id);

      if (out_port_ptr)
      {
         port_cmn_ptr = &(out_port_ptr->cmn);
      }
   }

   return port_cmn_ptr;
}

/**
 *  Given port id, fetch the input port structure.
 */
capi_sync_in_port_t *capi_sync_get_in_port_from_port_id(capi_sync_t *me_ptr, uint32_t port_id)
{
   capi_sync_in_port_t *port_ptr = NULL;

   for (uint32_t i = 0; i < me_ptr->num_in_ports; i++)
   {
      if (port_id == me_ptr->in_port_info_ptr[i].cmn.self_port_id)
      {
         port_ptr = &me_ptr->in_port_info_ptr[i];
         break;
      }
   }

   return port_ptr;
}

/**
 *  Given port id, fetch the output port structure
 */
capi_sync_out_port_t *capi_sync_get_out_port_from_port_id(capi_sync_t *me_ptr, uint32_t port_id)
{
   capi_sync_out_port_t *port_ptr = NULL;

   for (uint32_t i = 0; i < me_ptr->num_out_ports; i++)
   {
      if (port_id == me_ptr->out_port_info_ptr[i].cmn.self_port_id)
      {
         port_ptr = &me_ptr->out_port_info_ptr[i];
         break;
      }
   }

   return port_ptr;
}

/**
 * Initialize sync common port. Ports are closed until opened.
 */
static capi_err_t capi_sync_init_cmn_port(capi_sync_t *me_ptr, capi_sync_cmn_port_t *cmn_port_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   if (NULL == me_ptr || NULL == cmn_port_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi sync: Init received bad pointer, 0x%p, 0x%p", me_ptr, cmn_port_ptr);
      CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      return capi_result;
   }

   cmn_port_ptr->self_index   = SYNC_PORT_INDEX_INVALID;
   cmn_port_ptr->conn_index   = SYNC_PORT_INDEX_INVALID;
   cmn_port_ptr->self_port_id = SYNC_PORT_ID_INVALID;
   cmn_port_ptr->conn_port_id = SYNC_PORT_ID_INVALID;
   cmn_port_ptr->state        = CAPI_PORT_STATE_CLOSED;

   return capi_result;
}

/**
 * Initialize sync input port.
 */
static capi_err_t capi_sync_init_in_port(capi_sync_t *me_ptr, capi_sync_in_port_t *in_port_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   if (NULL == me_ptr || NULL == in_port_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi sync: Init received bad pointer, 0x%p, 0x%p", me_ptr, in_port_ptr);
      CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      return capi_result;
   }

   in_port_ptr->int_bufs_ptr          = NULL;
   in_port_ptr->buffer_timestamp      = 0;
   in_port_ptr->is_threshold_disabled = FALSE; // Start with threshold enabled (default container mode)
   in_port_ptr->is_output_sent_once   = FALSE;
   in_port_ptr->pending_eos           = FALSE;

   capi_result |= capi_cmn_init_media_fmt_v2(&(in_port_ptr->media_fmt));
   capi_result |= capi_sync_init_cmn_port(me_ptr, &(in_port_ptr->cmn));

   return capi_result;
}

/**
 * Initialize sync output port.
 */
static capi_err_t capi_sync_init_out_port(capi_sync_t *me_ptr, capi_sync_out_port_t *out_port_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   if (NULL == me_ptr || NULL == out_port_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi sync: Init received bad pointer, 0x%p, 0x%p", me_ptr, out_port_ptr);
      CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      return capi_result;
   }

   capi_result = capi_sync_init_cmn_port(me_ptr, &(out_port_ptr->cmn));

   out_port_ptr->needs_eos_at_close = FALSE;

   return capi_result;
}
