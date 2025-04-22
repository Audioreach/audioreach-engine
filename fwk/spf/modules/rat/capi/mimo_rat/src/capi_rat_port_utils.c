/*==============================================================================
 @file capi_rat_port_utils.c
 This file contains port utility functions for Rate Adapted Timer Endpoint module

 ================================================================================
 Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 SPDX-License-Identifier: BSD-3-Clause-Clear
 ==============================================================================*/
// clang-format off
// clang-format on
/*=====================================================================
 Includes
 ======================================================================*/
#include "capi_rat_i.h"
/*=====================================================================
 Function declarations
 ======================================================================*/
static capi_err_t capi_rat_init_cmn_port(capi_rat_t *me_ptr, capi_rat_cmn_port_t *cmn_port_ptr);

static capi_err_t capi_rat_init_in_port(capi_rat_t *me_ptr, capi_rat_in_port_t *in_port_ptr);

static capi_err_t capi_rat_init_out_port(capi_rat_t *me_ptr, capi_rat_out_port_t *out_port_ptr);

static capi_err_t capi_rat_populate_peer_port_info(capi_rat_t *         me_ptr,
                                                   capi_rat_cmn_port_t *port_cmn_ptr,
                                                   bool_t               is_input);

static capi_err_t capi_rat_port_open(capi_rat_t *me_ptr, bool_t is_input, uint32_t port_index, uint32_t port_id);

static capi_err_t capi_rat_port_start(capi_rat_t *me_ptr, uint32_t port_index, bool_t is_input);

static capi_err_t capi_rat_port_stop(capi_rat_t *me_ptr, uint32_t port_index, bool_t is_input);

static capi_err_t capi_rat_port_close(capi_rat_t *me_ptr, uint32_t port_index, bool_t is_input);

/*============================ Multi Port Functions======================== */
/**
 * Initialize rat common port. Ports are closed until opened.
 */
static capi_err_t capi_rat_init_cmn_port(capi_rat_t *me_ptr, capi_rat_cmn_port_t *cmn_port_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   if (NULL == me_ptr || NULL == cmn_port_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "CAPI_RAT: Init received bad pointer, 0x%p, 0x%p", me_ptr, cmn_port_ptr);
      CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      return capi_result;
   }

   cmn_port_ptr->self_index   = RAT_PORT_INDEX_INVALID;
   cmn_port_ptr->conn_index   = RAT_PORT_INDEX_INVALID;
   cmn_port_ptr->self_port_id = RAT_PORT_ID_INVALID;
   cmn_port_ptr->conn_port_id = RAT_PORT_ID_INVALID;
   cmn_port_ptr->port_state   = DATA_PORT_STATE_CLOSED;

   return capi_result;
}

/** Initialize rat input port.*/
static capi_err_t capi_rat_init_in_port(capi_rat_t *me_ptr, capi_rat_in_port_t *in_port_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   if (NULL == me_ptr || NULL == in_port_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "CAPI_RAT: Init received bad pointer, 0x%p, 0x%p", me_ptr, in_port_ptr);
      CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      return capi_result;
   }

   capi_result |= capi_cmn_init_media_fmt_v2(&(in_port_ptr->media_fmt));
   capi_result |= capi_rat_init_cmn_port(me_ptr, &(in_port_ptr->cmn));

   return capi_result;
}

/* Initialize rat output port */
static capi_err_t capi_rat_init_out_port(capi_rat_t *me_ptr, capi_rat_out_port_t *out_port_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   if (NULL == me_ptr || NULL == out_port_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "CAPI_RAT: Init received bad pointer, 0x%p, 0x%p", me_ptr, out_port_ptr);
      CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      return capi_result;
   }

   out_port_ptr->begin_silence_insertion_md = FALSE;
   capi_result                              = capi_rat_init_cmn_port(me_ptr, &(out_port_ptr->cmn));

   return capi_result;
}

/** Handles port number info set properties. Allocates memory for inp and out ports */
capi_err_t capi_rat_process_port_num_info(capi_rat_t *me_ptr, capi_prop_t *prop_ptr)
{
   capi_err_t  capi_result = CAPI_EOK;
   capi_buf_t *payload_ptr = &(prop_ptr->payload);

   if (payload_ptr->actual_data_len < sizeof(capi_port_num_info_t))
   {
      RAT_MSG(me_ptr->iid,
              DBG_ERROR_PRIO,
              "CAPI_RAT: Set prop id 0x%lx Bad param size %lu",
              (uint32_t)prop_ptr->id,
              payload_ptr->actual_data_len);
      CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
      return capi_result;
   }

   // If input & output ports are already allocated
   if ((me_ptr->in_port_info_ptr) || (me_ptr->out_port_info_ptr))
   {
      RAT_MSG(me_ptr->iid,
              DBG_ERROR_PRIO,
              "CAPI_RAT: Set property id 0x%lx number of input and output ports already configured "
              "to %lu and %lu respectively",
              (uint32_t)prop_ptr->id,
              me_ptr->num_in_ports,
              me_ptr->num_out_ports);
      CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      return capi_result;
   }

   capi_port_num_info_t *data_ptr = (capi_port_num_info_t *)payload_ptr->data_ptr;

   // RAT can be opened without input/outputs when container needs an STM module, and RAT is placed to drive the
   // container with the timer interrupts.
   if ((0 == data_ptr->num_input_ports) && (0 == data_ptr->num_output_ports))
   {
      RAT_MSG(me_ptr->iid,
              DBG_HIGH_PRIO,
              "CAPI_RAT: Warning! RAT being opened with zero inputs=%lu or outputs=%lu ports.",
              data_ptr->num_input_ports,
              data_ptr->num_output_ports);
   }

   if ((CAPI_SISO_RAT == me_ptr->type) && ((data_ptr->num_input_ports > 1) || (data_ptr->num_output_ports > 1)))
   {
      RAT_MSG(me_ptr->iid,
              DBG_ERROR_PRIO,
              "CAPI_RAT: SISO RAT only supports one input and one output, ports configured in %d out %d ",
              me_ptr->num_in_ports,
              me_ptr->num_out_ports);
      CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      return capi_result;
   }

   // All sanity checks are complete
   me_ptr->num_in_ports  = data_ptr->num_input_ports;
   me_ptr->num_out_ports = data_ptr->num_output_ports;

   RAT_MSG(me_ptr->iid,
           DBG_HIGH_PRIO,
           "CAPI_RAT: Num in ports %d Num output ports %d",
           me_ptr->num_in_ports,
           me_ptr->num_out_ports);

   uint32_t alloc_size = 0;

   // Allocate memory for the input ports
   if (0 < me_ptr->num_in_ports)
   {
      alloc_size = (me_ptr->num_in_ports) * (sizeof(capi_rat_in_port_t));
      me_ptr->in_port_info_ptr =
         (capi_rat_in_port_t *)posal_memory_malloc(alloc_size, (POSAL_HEAP_ID)me_ptr->heap_mem.heap_id);

      if (NULL == me_ptr->in_port_info_ptr)
      {
         RAT_MSG(me_ptr->iid, DBG_ERROR_PRIO, "CAPI_RAT: Couldn't allocate memory for input port structure.");
         return CAPI_ENOMEMORY;
      }

      memset(me_ptr->in_port_info_ptr, 0, alloc_size);

      // Initialize the input ports
      for (uint32_t i = 0; i < me_ptr->num_in_ports; i++)
      {
         capi_result |= capi_rat_init_in_port(me_ptr, &(me_ptr->in_port_info_ptr[i]));
      }
   }

   if (0 < me_ptr->num_out_ports)
   {
      // Allocate memory for the output ports
      alloc_size = (me_ptr->num_out_ports) * (sizeof(capi_rat_out_port_t));
      me_ptr->out_port_info_ptr =
         (capi_rat_out_port_t *)posal_memory_malloc(alloc_size, (POSAL_HEAP_ID)me_ptr->heap_mem.heap_id);

      if (NULL == me_ptr->out_port_info_ptr)
      {
         RAT_MSG(me_ptr->iid, DBG_ERROR_PRIO, "CAPI_RAT: Couldn't allocate memory for output port structure.");
         return CAPI_ENOMEMORY;
      }

      memset(me_ptr->out_port_info_ptr, 0, alloc_size);

      // Initialize the output ports
      for (uint32_t i = 0; i < me_ptr->num_out_ports; i++)
      {
         capi_result |= capi_rat_init_out_port(me_ptr, &(me_ptr->out_port_info_ptr[i]));
      }
   }
   return capi_result;
}

/**
 * Utility function to identify & populate the peer port information
 * given the port being opened & its direction (called when processing INTF_EXTN_DATA_PORT_OPEN property)
 */
static capi_err_t capi_rat_populate_peer_port_info(capi_rat_t *         me_ptr,
                                                   capi_rat_cmn_port_t *port_cmn_ptr,
                                                   bool_t               is_input)
{
   capi_err_t capi_result = CAPI_EOK;

   if (!port_cmn_ptr)
   {
      return CAPI_EBADPARAM;
   }

   uint32_t curr_port_id = port_cmn_ptr->self_port_id;
   uint32_t peer_port_id = RAT_PORT_ID_INVALID;

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

   if (RAT_PORT_ID_INVALID == peer_port_id)
   {
      return CAPI_EFAILED;
   }

   port_cmn_ptr->conn_port_id = peer_port_id;

   RAT_MSG(me_ptr->iid, DBG_MED_PRIO, "CAPI_RAT: Connected port ids incoming %d peer %d ", curr_port_id, peer_port_id);

   // Since the pairing is for the port IDs, attempt to figure out the index information
   // for the peer port. If this function returns NULL, not to worry. This will be updated
   // when the peer port is opened.
   capi_rat_cmn_port_t *peer_cmn_ptr = capi_rat_get_port_cmn_from_port_id(me_ptr, peer_port_id, !is_input);

   if (!peer_cmn_ptr)
   {
      return CAPI_EOK;
   }

   // Update the connected indices for the incoming port cmn and its peer
   port_cmn_ptr->conn_index   = peer_cmn_ptr->self_index;
   peer_cmn_ptr->conn_index   = port_cmn_ptr->self_index;
   peer_cmn_ptr->conn_port_id = curr_port_id;

   RAT_MSG(me_ptr->iid,
           DBG_MED_PRIO,
           "CAPI_RAT: Connected port idxs incoming %d peer %d ",
           port_cmn_ptr->self_index,
           peer_cmn_ptr->self_index);

   return capi_result;
}

/**
 * Handling for port open. Fields have already been validated.
 * Store the port index, move port state to STOPPED.
 */
static capi_err_t capi_rat_port_open(capi_rat_t *me_ptr, bool_t is_input, uint32_t port_index, uint32_t port_id)
{

   RAT_MSG(me_ptr->iid,
           DBG_MED_PRIO,
           "CAPI_RAT: handling port open, is_input = %ld, port index = %ld, port_id = %ld",
           is_input,
           port_index,
           port_id);

   capi_err_t           capi_result  = CAPI_EOK;
   capi_rat_cmn_port_t *port_cmn_ptr = capi_rat_get_port_cmn_from_index(me_ptr, port_index, is_input);

   if (!port_cmn_ptr)
   {
      RAT_MSG(me_ptr->iid,
              DBG_ERROR_PRIO,
              "CAPI_RAT: Handling port open: Couldnt find port info for index %ld, is_input %lu",
              port_index,
              is_input);
      return CAPI_EFAILED;
   }

   // Check if already opened. This shouldn't happen.
   if (DATA_PORT_STATE_CLOSED != port_cmn_ptr->port_state)
   {
      RAT_MSG(me_ptr->iid,
              DBG_ERROR_PRIO,
              "CAPI_RAT: Port already opened. port_index %ld, is_input %lu",
              port_index,
              is_input);
      return CAPI_EFAILED;
   }

   port_cmn_ptr->self_index   = port_index;
   port_cmn_ptr->self_port_id = port_id;
   port_cmn_ptr->port_state   = DATA_PORT_STATE_STOPPED;

   capi_result = capi_rat_populate_peer_port_info(me_ptr, port_cmn_ptr, is_input);

   return capi_result;
}

static capi_err_t capi_rat_port_start(capi_rat_t *me_ptr, uint32_t port_index, bool_t is_input)
{
   capi_err_t           capi_result  = CAPI_EOK;
   capi_rat_cmn_port_t *port_cmn_ptr = capi_rat_get_port_cmn_from_index(me_ptr, port_index, is_input);

   if (!port_cmn_ptr)
   {
      RAT_MSG(me_ptr->iid,
              DBG_ERROR_PRIO,
              "CAPI_RAT: Handling port start, Couldnt find port info for index %ld, is_input %lu",
              port_index,
              is_input);
      return CAPI_EFAILED;
   }

   RAT_MSG(me_ptr->iid,
           DBG_LOW_PRIO,
           "CAPI_RAT: Handling port start, port_idx = %ld, is_input = %ld, port id %lx",
           port_index,
           is_input,
           port_cmn_ptr->self_port_id);

   if (FALSE == me_ptr->configured_media_fmt_received)
   {
      RAT_MSG(me_ptr->iid,
              DBG_ERROR_PRIO,
              "CAPI_RAT: Cannot start port idx %lx, since static output port mf not set",
              port_index);
      return CAPI_EFAILED;
   }

   // Check if dynamic port getting start has matching input media format SR family
   if ((is_input) && (me_ptr->in_port_info_ptr[port_index].inp_mf_received) &&
       (!capi_rat_is_sample_rate_accepted(me_ptr->configured_media_fmt.format.sampling_rate,
                                          me_ptr->in_port_info_ptr[port_index].media_fmt.format.sampling_rate)))
   {
      RAT_MSG(me_ptr->iid,
              DBG_ERROR_PRIO,
              "CAPI_RAT: Data port start failed: Unsupported sample rate %d received on port index %d, has to be in "
              "same family as static out mf sample rate %d",
              me_ptr->in_port_info_ptr[port_index].media_fmt.format.sampling_rate,
              port_index,
              me_ptr->configured_media_fmt.format.sampling_rate);
      return CAPI_EFAILED;
   }

   port_cmn_ptr->port_state = DATA_PORT_STATE_STARTED;

   return capi_result;
}

static capi_err_t capi_rat_port_stop(capi_rat_t *me_ptr, uint32_t port_index, bool_t is_input)
{
   capi_err_t           capi_result  = CAPI_EOK;
   capi_rat_cmn_port_t *port_cmn_ptr = capi_rat_get_port_cmn_from_index(me_ptr, port_index, is_input);

   if (!port_cmn_ptr)
   {
      RAT_MSG(me_ptr->iid,
              DBG_ERROR_PRIO,
              "CAPI_RAT: Handle port stop: Couldnt find port info for index %ld, is_input %lu",
              port_index,
              is_input);
      return CAPI_EFAILED;
   }

   RAT_MSG(me_ptr->iid,
           DBG_MED_PRIO,
           "CAPI_RAT: Handling port stop,  is_input = %ld, port_idx = %ld, port id = %lx",
           is_input,
           port_index,
           port_cmn_ptr->self_port_id);

   port_cmn_ptr->port_state = DATA_PORT_STATE_STOPPED;

   return capi_result;
}

/**
 * Handling for port close. Fields have already been validated.
 * Invalidate the port index, deallocate port buffer, move port state to CLOSED.
 */
static capi_err_t capi_rat_port_close(capi_rat_t *me_ptr, uint32_t port_index, bool_t is_input)
{
   capi_err_t           capi_result  = CAPI_EOK;
   capi_rat_cmn_port_t *port_cmn_ptr = capi_rat_get_port_cmn_from_index(me_ptr, port_index, is_input);

   if (!port_cmn_ptr)
   {
      RAT_MSG(me_ptr->iid,
              DBG_ERROR_PRIO,
              "CAPI_RAT: Couldnt find port info for index %ld, is_input %lu",
              port_index,
              is_input);
      return CAPI_EFAILED;
   }

   RAT_MSG(me_ptr->iid,
           DBG_MED_PRIO,
           "Handling port close, port_idx = %ld, is_input = %ld, port id %lx",
           port_index,
           is_input,
           port_cmn_ptr->self_port_id);

   // Check if already closed. This shouldn't happen.
   if (DATA_PORT_STATE_CLOSED == port_cmn_ptr->port_state)
   {
      RAT_MSG(me_ptr->iid,
              DBG_ERROR_PRIO,
              "CAPI_RAT: Port already closed. port_index %lu, is_input %lu",
              port_cmn_ptr->self_index,
              is_input);
      return CAPI_EFAILED;
   }

   // Remove the connection information from the peer port
   uint32_t peer_index = port_cmn_ptr->conn_index;

   if (RAT_PORT_INDEX_INVALID != peer_index)
   {
      capi_rat_cmn_port_t *peer_cmn_ptr = capi_rat_get_port_cmn_from_index(me_ptr, peer_index, !is_input);
      if (!peer_cmn_ptr)
      {
         RAT_MSG(me_ptr->iid,
                 DBG_ERROR_PRIO,
                 "CAPI_RAT: Couldnt find peer port info for index %ld, is_input %lu peer_index %ld",
                 port_index,
                 is_input,
                 peer_index);
         return CAPI_EFAILED;
      }
      peer_cmn_ptr->conn_index   = RAT_PORT_INDEX_INVALID;
      peer_cmn_ptr->conn_port_id = RAT_PORT_ID_INVALID;

      // If the input port gets closed while it's still in start state, then we need to manually push EOS to the
      // output port on the next process call.
      if (is_input)
      {
         if (DATA_PORT_STATE_STARTED == port_cmn_ptr->port_state)
         {
            capi_rat_out_port_t *out_peer_port_ptr = (capi_rat_out_port_t *)peer_cmn_ptr;

            RAT_MSG_ISLAND(me_ptr->iid, DBG_HIGH_PRIO, "CAPI_RAT: Input closed, setting flag to send BEGIN silence md");

            out_peer_port_ptr->begin_silence_insertion_md = TRUE;
         }
      }
   }

   if (is_input)
   {
      memset(port_cmn_ptr, 0, sizeof(capi_rat_in_port_t));
   }
   else
   {
      memset(port_cmn_ptr, 0, sizeof(capi_rat_out_port_t));
   }

   // Update the state of the port being closed
   port_cmn_ptr->self_index   = RAT_PORT_INDEX_INVALID;
   port_cmn_ptr->conn_index   = RAT_PORT_INDEX_INVALID;
   port_cmn_ptr->self_port_id = RAT_PORT_ID_INVALID;
   port_cmn_ptr->conn_port_id = RAT_PORT_ID_INVALID;
   port_cmn_ptr->port_state   = DATA_PORT_STATE_CLOSED;

   return capi_result;
}

capi_err_t capi_rat_handle_port_op(capi_rat_t *me_ptr, capi_buf_t *payload_ptr)
{
   capi_err_t capi_result = CAPI_EOK;

   if (NULL == payload_ptr->data_ptr)
   {
      RAT_MSG(me_ptr->iid, DBG_ERROR_PRIO, "CAPI_RAT: Set property port operation, received null buffer");
      CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      return capi_result;
   }

   // Validate length.
   if (sizeof(intf_extn_data_port_operation_t) > payload_ptr->actual_data_len)
   {
      RAT_MSG(me_ptr->iid,
              DBG_ERROR_PRIO,
              "CAPI_RAT: Set property for port operation, Bad param size %lu",
              payload_ptr->actual_data_len);
      CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
      return capi_result;
   }

   intf_extn_data_port_operation_t *data_ptr = (intf_extn_data_port_operation_t *)(payload_ptr->data_ptr);

   if ((sizeof(intf_extn_data_port_operation_t) + (data_ptr->num_ports * sizeof(intf_extn_data_port_id_idx_map_t))) >
       payload_ptr->actual_data_len)
   {
      RAT_MSG(me_ptr->iid,
              DBG_ERROR_PRIO,
              "CAPI_RAT: Set property for port operation, Bad param size %lu",
              payload_ptr->actual_data_len);
      CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
      return capi_result;
   }

   // For each port in the operation payload.
   for (uint32_t iter = 0; iter < data_ptr->num_ports; iter++)
   {
      uint32_t port_id    = data_ptr->id_idx[iter].port_id;
      uint32_t port_index = data_ptr->id_idx[iter].port_index;

#ifdef RAT_DEBUG
      RAT_MSG(me_ptr->iid,
              DBG_HIGH_PRIO,
              "CAPI_RAT: Validating port idx=%lu, port id=%lu is input %d ",
              port_index,
              port_id,
              data_ptr->is_input_port);
#endif

      // Validate that id-index mapping matches what was previously sent, unless mapping doesn't exist yet.
      capi_rat_cmn_port_t *port_cmn_ptr = capi_rat_get_port_cmn_from_index(me_ptr, port_index, data_ptr->is_input_port);

      if (!port_cmn_ptr)
      {
         RAT_MSG(me_ptr->iid,
                 DBG_ERROR_PRIO,
                 "CAPI_RAT: Failed to lookup port info for port_index = %lu, is_input = %lu",
                 port_index,
                 data_ptr->is_input_port);

         CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
         return capi_result;
      }

      uint32_t prev_index = port_cmn_ptr->self_index;
      if (RAT_PORT_INDEX_INVALID != prev_index)
      {
         if (prev_index != port_index)
         {
            RAT_MSG(me_ptr->iid,
                    DBG_ERROR_PRIO,
                    "CAPI_RAT: Error: id-idx mapping changed on port %lu in payload, port_index = %lu, "
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
            capi_result |= capi_rat_port_open(me_ptr, data_ptr->is_input_port, port_index, port_id);
            break;
         }
         case INTF_EXTN_DATA_PORT_START:
         {
            capi_result |= capi_rat_port_start(me_ptr, port_index, data_ptr->is_input_port);
            break;
         }
         case INTF_EXTN_DATA_PORT_CLOSE:
         {
            capi_result |= capi_rat_port_close(me_ptr, port_index, data_ptr->is_input_port);
            break;
         }
         case INTF_EXTN_DATA_PORT_STOP:
         case INTF_EXTN_DATA_PORT_SUSPEND:
         {
            capi_result |= capi_rat_port_stop(me_ptr, port_index, data_ptr->is_input_port);
            break;
         }
         default:
         {
            RAT_MSG(me_ptr->iid,
                    DBG_HIGH_PRIO,
                    "CAPI_RAT: Port operation opcode %lu. Not supported.",
                    data_ptr->opcode);
            CAPI_SET_ERROR(capi_result, CAPI_EUNSUPPORTED);
            break;
         }
      }
   }

   return capi_result;
}

/* Port Query functions */

/**
 *  Given port idx, fetch the port id.
 */
uint32_t capi_rat_get_port_id_from_port_idx(capi_rat_t *me_ptr, uint32_t port_idx, bool_t is_input)
{
   uint32_t port_id = RAT_PORT_ID_INVALID;

   if (is_input && (me_ptr->in_port_info_ptr))
   {
      for (uint32_t i = 0; i < me_ptr->num_in_ports; i++)
      {
         if (port_idx == me_ptr->in_port_info_ptr[i].cmn.self_index)
         {
            port_id = me_ptr->in_port_info_ptr[i].cmn.self_port_id;
            break;
         }
      }
   }
   else if (!is_input && (me_ptr->out_port_info_ptr))
   {
      for (uint32_t i = 0; i < me_ptr->num_out_ports; i++)
      {
         if (port_idx == me_ptr->out_port_info_ptr[i].cmn.self_index)
         {
            port_id = me_ptr->out_port_info_ptr[i].cmn.self_port_id;
            break;
         }
      }
   }
   return port_id;
}

/**
 *  Given port id, fetch the port idx.
 */
uint32_t capi_rat_get_port_idx_from_port_id(capi_rat_t *me_ptr, uint32_t port_id, bool_t is_input)
{
   uint32_t port_idx = RAT_PORT_INDEX_INVALID;

   if (is_input && (me_ptr->in_port_info_ptr))
   {
      for (uint32_t i = 0; i < me_ptr->num_in_ports; i++)
      {
         if (port_id == me_ptr->in_port_info_ptr[i].cmn.self_port_id)
         {
            port_idx = me_ptr->in_port_info_ptr[i].cmn.self_index;
            break;
         }
      }
   }
   else if (!is_input && (me_ptr->out_port_info_ptr))
   {
      for (uint32_t i = 0; i < me_ptr->num_out_ports; i++)
      {
         if (port_id == me_ptr->out_port_info_ptr[i].cmn.self_port_id)
         {
            port_idx = me_ptr->out_port_info_ptr[i].cmn.self_index;
            break;
         }
      }
   }
   return port_idx;
}

/**
 *  Given port id, fetch the input port structure.
 */
capi_rat_in_port_t *capi_rat_get_in_port_from_port_id(capi_rat_t *me_ptr, uint32_t port_id)
{
   capi_rat_in_port_t *port_ptr = NULL;

   if (!me_ptr->in_port_info_ptr)
   {
      return NULL;
   }

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
capi_rat_out_port_t *capi_rat_get_out_port_from_port_id(capi_rat_t *me_ptr, uint32_t port_id)
{
   capi_rat_out_port_t *port_ptr = NULL;
   if (!me_ptr->out_port_info_ptr)
   {
      return NULL;
   }

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

// Given port index, fetch the cmn port structure
capi_rat_cmn_port_t *capi_rat_get_port_cmn_from_index(capi_rat_t *me_ptr, uint32_t index, bool_t is_input)
{
   capi_rat_cmn_port_t *port_cmn_ptr = NULL;

   if (is_input)
   {
      if (me_ptr->in_port_info_ptr)
      {
         port_cmn_ptr = &me_ptr->in_port_info_ptr[index].cmn;
      }
   }
   else
   {
      if (me_ptr->out_port_info_ptr)
      {
         port_cmn_ptr = &me_ptr->out_port_info_ptr[index].cmn;
      }
   }
   return port_cmn_ptr;
}

// Given port id, fetch the cmn port structure
capi_rat_cmn_port_t *capi_rat_get_port_cmn_from_port_id(capi_rat_t *me_ptr, uint32_t port_id, bool_t is_input)
{
   capi_rat_cmn_port_t *port_cmn_ptr = NULL;
   if (is_input)
   {
      capi_rat_in_port_t *in_port_ptr = capi_rat_get_in_port_from_port_id(me_ptr, port_id);
      if (in_port_ptr)
      {
         port_cmn_ptr = &(in_port_ptr->cmn);
      }
   }
   else
   {
      capi_rat_out_port_t *out_port_ptr = capi_rat_get_out_port_from_port_id(me_ptr, port_id);
      if (out_port_ptr)
      {
         port_cmn_ptr = &(out_port_ptr->cmn);
      }
   }
   return port_cmn_ptr;
}
