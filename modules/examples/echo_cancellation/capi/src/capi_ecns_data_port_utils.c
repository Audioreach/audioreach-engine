/**
 * \file capi_ecns_data_port_utils.c
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

/**----------------------------------------------------------------------------
 ** Include Files
 ** -------------------------------------------------------------------------*/

#include "capi_ecns_i.h"

/* =========================================================================
 * FUNCTION : capi_ecns_handle_intf_extn_data_port_operation
 *
 * DESCRIPTION:
 * Performs data port operation like OPEN, CLOSE, START, STOP.
 *
 * OPEN -
 *  Framework gives the port_id to port_index mapping through this operation,
 *  for the newly opened ports. Module can allocate any dyanmic resource per
 *  port if needed. Note that for input port media format is not available at
 *  this point. So module can only allocate resources which are independent of
 *  the input media format. Note that module can receive data buffers only in
 *  the START state.
 *
 * CLOSE -
 *  Closes the given input/output port_index. Destroy dynamic resources for the
 *  port. If the Primary input itself is being closed destroy the library memory.
 *  Reinitialize when the port re-opens.
 *
 *  START -
 *  This indicates module can start to expect data buffers on this ports.
 *
 *  STOP -
 *  This indicates module must stop expecting data buffers on this ports.
 *
 *
 * ========================================================================= */
capi_err_t capi_ecns_handle_intf_extn_data_port_operation(capi_ecns_t *me_ptr, capi_buf_t *params_ptr)
{
   capi_err_t result = CAPI_EOK;
   if (NULL == params_ptr->data_ptr)
   {
      ECNS_DBG(me_ptr->miid, DBG_ERROR_PRIO, "Data port operation received null buffer");
      return CAPI_EBADPARAM;
   }
   if (params_ptr->actual_data_len < sizeof(intf_extn_data_port_operation_t))
   {
      ECNS_DBG(me_ptr->miid,
               DBG_ERROR_PRIO,
               "Invalid payload size for port operation %d",
               params_ptr->actual_data_len);
      return CAPI_ENEEDMORE;
   }

   intf_extn_data_port_operation_t *data_ptr = (intf_extn_data_port_operation_t *)(params_ptr->data_ptr);
   if (params_ptr->actual_data_len <
       sizeof(intf_extn_data_port_operation_t) + (data_ptr->num_ports * sizeof(intf_extn_data_port_id_idx_map_t)))
   {
      ECNS_DBG(me_ptr->miid,
               DBG_ERROR_PRIO,
               "Invalid payload size for port operation %d",
               params_ptr->actual_data_len);
      return CAPI_ENEEDMORE;
   }

   // Check if the number of input/output ports being operated are within the max number of ports
   if (data_ptr->is_input_port && (data_ptr->num_ports > me_ptr->num_port_info.num_input_ports))
   {
      ECNS_DBG(me_ptr->miid,
               DBG_ERROR_PRIO,
               "Invalid input ports. num_ports =%d, max_input_ports = %d",
               data_ptr->num_ports,
               me_ptr->num_port_info.num_input_ports);
      return CAPI_EBADPARAM;
   }

   if (!data_ptr->is_input_port && (data_ptr->num_ports > me_ptr->num_port_info.num_output_ports))
   {
      ECNS_DBG(me_ptr->miid,
               DBG_ERROR_PRIO,
               "Invalid output ports. num_ports =%d, max_output_ports = %d",
               data_ptr->num_ports,
               me_ptr->num_port_info.num_output_ports);
      return CAPI_EBADPARAM;
   }

   // Iterate and perform port operation on each input/output ports.
   for (uint32_t iter = 0; iter < data_ptr->num_ports; iter++)
   {
      uint32_t port_id    = data_ptr->id_idx[iter].port_id;
      uint32_t port_index = data_ptr->id_idx[iter].port_index;

      ECNS_DBG(me_ptr->miid,
               DBG_HIGH_PRIO,
               "Port operation 0x%x performed on port_index= %lu, port_id= 0x%lx is_input_port= %d ",
               iter,
               port_index,
               data_ptr->id_idx[iter].port_id,
               data_ptr->is_input_port);

      if ((data_ptr->is_input_port && (port_index >= me_ptr->num_port_info.num_input_ports)) ||
          (!data_ptr->is_input_port && (port_index >= me_ptr->num_port_info.num_output_ports)))
      {
         ECNS_DBG(me_ptr->miid,
                  DBG_ERROR_PRIO,
                  "Bad parameter in id-idx map on port_id 0x%lx, port_index = %lu, max in ports = %d, max "
                  "out ports = %d ",
                  iter,
                  port_index,
                  me_ptr->num_port_info.num_input_ports,
                  me_ptr->num_port_info.num_output_ports);
         return CAPI_EBADPARAM;
      }

      switch (data_ptr->opcode)
      {
      case INTF_EXTN_DATA_PORT_OPEN:
      {
         result = capi_encs_handle_port_open(me_ptr, port_id, port_index, data_ptr->is_input_port);
         break;
      }
      case INTF_EXTN_DATA_PORT_START:
      {
         result = capi_encs_handle_port_start(me_ptr, port_index, data_ptr->is_input_port);
         break;
      }
      case INTF_EXTN_DATA_PORT_STOP:
      {

         result = capi_encs_handle_port_stop(me_ptr, port_index, data_ptr->is_input_port);
         break;
      }
      case INTF_EXTN_DATA_PORT_CLOSE:
      {
         result = capi_encs_handle_port_close(me_ptr, port_index, data_ptr->is_input_port);
         break;
      }
      default:
      {
         ECNS_DBG(me_ptr->miid,
                  DBG_ERROR_PRIO,
                  "Port operation - Unsupported opcode: 0x%x",
                  data_ptr->opcode);
         CAPI_SET_ERROR(result, CAPI_EUNSUPPORTED);
         break;
      }
      }

      ECNS_DBG(me_ptr->miid,
               DBG_HIGH_PRIO,
               "Port operation 0x%lx done with result 0x%lx, for port id 0x%x is_input = 0x%x",
               data_ptr->opcode,
               result,
               port_id,
               data_ptr->is_input_port);
   }

   return result;
}

/* =========================================================================
 * FUNCTION : capi_encs_handle_port_open
 *
 * DESCRIPTION:
 *  Handle input/output port OPEN. Allocate any dynamic resources if required.
 * ========================================================================= */
capi_err_t capi_encs_handle_port_open(capi_ecns_t *me_ptr, uint32_t port_id, uint32_t port_index, bool_t is_input)
{
   capi_err_t result = CAPI_EOK;
   /* ECNS modules have static port IDs. The static port ID defines the nature of the data incoming
    * in the given port. Each port id is mapped to an port index, whose value ranges between (0,MAX_PORTS).
    *
    * During open operation, we received (port_id,port_index) which gives the mapping between port_id to
    * port_index*/
   if (is_input)
   {
      if (me_ptr->in_port_info[port_index].port_state != DATA_PORT_STATE_CLOSED)
      {
         ECNS_DBG(me_ptr->miid,
                  DBG_ERROR_PRIO,
                  "Input port_id 0x%x idx 0x%lx is already opened",
                  port_id,
                  port_index);
         return CAPI_EFAILED;
      }

      if (ECNS_PRIMARY_INPUT_STATIC_PORT_ID == port_id)
      {
         ECNS_DBG(me_ptr->miid,
                  DBG_HIGH_PRIO,
                  "Primary input port_id 0x%x, idx: 0x%x",
                  port_id,
                  port_index);
      }
      else if (ECNS_REFERENCE_INPUT_STATIC_PORT_ID == port_id)
      {
         ECNS_DBG(me_ptr->miid,
                  DBG_HIGH_PRIO,
                  "Reference input port_id 0x%x,  idx: 0x%x",
                  port_id,
                  port_index);
      }
      else
      {
         ECNS_DBG(me_ptr->miid, DBG_ERROR_PRIO, "Invalid input port_id 0x%x", port_id);
         return CAPI_EBADPARAM;
      }

      // Cache the input port ID,index and state.
      me_ptr->in_port_info[port_index].port_id    = port_id;
      me_ptr->in_port_info[port_index].port_index = port_index;
      me_ptr->in_port_info[port_index].port_state = DATA_PORT_STATE_OPENED;
   }
   else
   {
      if (me_ptr->out_port_info[port_index].port_state != DATA_PORT_STATE_CLOSED)
      {
         ECNS_DBG(me_ptr->miid,
                  DBG_ERROR_PRIO,
                  "Output port_id 0x%x idx 0x%lx is already opened",
                  port_id,
                  port_index);
         return CAPI_EFAILED;
      }

      if (ECNS_PRIMARY_OUTPUT_STATIC_PORT_ID == port_id)
      {
         ECNS_DBG(me_ptr->miid,
                  DBG_HIGH_PRIO,
                  "output port_id 0x%x, idx: 0x%x",
                  port_id,
                  port_index);
      }
      else
      {
         ECNS_DBG(me_ptr->miid, DBG_ERROR_PRIO, "Invalid output port_id 0x%x", port_id);
         return CAPI_EBADPARAM;
      }

      // Cache the output port ID ,index and state.
      me_ptr->out_port_info[port_index].port_id    = port_id;
      me_ptr->out_port_info[port_index].port_index = port_index;
      me_ptr->out_port_info[port_index].port_state = DATA_PORT_STATE_OPENED;
   }
   return result;
}

/* =========================================================================
 * FUNCTION : capi_encs_handle_port_close
 *
 * DESCRIPTION:
 *  Handle input/output port CLOSE. De-allocate if any dynamic resources are
 *  allocated earlier.
 * ========================================================================= */
capi_err_t capi_encs_handle_port_close(capi_ecns_t *me_ptr, uint32_t port_index, bool_t is_input)
{
   capi_err_t result = CAPI_EOK;
   if (is_input)
   {
      if (DATA_PORT_STATE_CLOSED != me_ptr->in_port_info[port_index].port_state)
      {
         // Destroy input port
         capi_encs_destroy_input_port(me_ptr, port_index);
      }
   }
   else
   {
      if (DATA_PORT_STATE_CLOSED != me_ptr->out_port_info[port_index].port_state)
      {
         // Destroy output port
         capi_encs_destroy_output_port(me_ptr, port_index);
      }
   }
   return result;
}

/* =========================================================================
 * FUNCTION : capi_encs_handle_port_start
 *
 * DESCRIPTION:
 *  Handle input/output port START. Indicates module can expect data buffers
 *  on this input/output port from next process cycle if trigger present.
 * ========================================================================= */
capi_err_t capi_encs_handle_port_start(capi_ecns_t *me_ptr, uint32_t port_index, bool_t is_input)
{
   capi_err_t result = CAPI_EOK;

   if (is_input)
   {
      // Mark port as started
      me_ptr->in_port_info[port_index].port_state = DATA_PORT_STATE_STARTED;
   }
   else
   {
      // Mark port as started
      me_ptr->out_port_info[port_index].port_state = DATA_PORT_STATE_STARTED;
   }

   return result;
}

/* =========================================================================
 * FUNCTION : capi_encs_handle_port_stop
 *
 * DESCRIPTION:
 *  Handle input/output port STOP. Indicates module must stop expecting data buffers
 *  on this input/output port.
 * ========================================================================= */
capi_err_t capi_encs_handle_port_stop(capi_ecns_t *me_ptr, uint32_t port_index, bool_t is_input)
{
   capi_err_t result = CAPI_EOK;
   if (is_input)
   {
      // Mark port as stopped
      me_ptr->in_port_info[port_index].port_state = DATA_PORT_STATE_STOPPED;
   }
   else
   {
      // Mark port as stopped
      me_ptr->out_port_info[port_index].port_state = DATA_PORT_STATE_STOPPED;
   }
   return result;
}

/* =========================================================================
 * FUNCTION : capi_encs_destroy_input_port
 *
 * DESCRIPTION:
 *  Destorys input port dynamic resources or reset the port state.
 * Function is called during port CLOSE or module END.
 * ========================================================================= */
void capi_encs_destroy_input_port(capi_ecns_t *me_ptr, uint32_t port_index)
{
   /* De-initialize the library at Primary input port stop, without primary input
    * ECNS cannot function */
   if (ECNS_PRIMARY_INPUT_STATIC_PORT_ID == me_ptr->in_port_info[port_index].port_id)
   {
      capi_ecns_deinit_library(me_ptr);
   }

   /* Destory metdata in the internal list*/
   capi_ecns_destroy_md_list(me_ptr, &me_ptr->in_port_info[port_index].md_list_ptr);

   /* Free any dyanmic memory if allocated for the given port */

   /* Memset the control port info structure */
   memset(&me_ptr->in_port_info[port_index], 0, sizeof(ecns_input_port_info_t));
}

/* =========================================================================
 * FUNCTION : capi_encs_destroy_output_port
 *
 * DESCRIPTION:
 *  Destorys output ports dynamic resources or reset the port state.
 * Function is called during port CLOSE or module END.
 * ========================================================================= */
void capi_encs_destroy_output_port(capi_ecns_t *me_ptr, uint32_t port_idx)
{
   /* Free any dyanmic memory if allocated for the given port */

   /* Memset the control port info structure */
   memset(&me_ptr->out_port_info[port_idx], 0, sizeof(ecns_output_port_info_t));
}

/* =========================================================================
 * FUNCTION : ecns_get_input_port_arr_idx
 *
 * DESCRIPTION:
 *
 * Each module maintains a input port info array.
 *    1. If a new input port ID is passed to this function, it will find an
 *       unused entry in the array and assigns that index to the given input
 *       port ID.
 *    2. If there is an existing entry with the given input control port ID.
 *       It returns that index.
 *    3. If a new input port ID is passed and there is no unused entry available,
 *       it return a UMAX_32 as error.
 * ========================================================================= */
capi_err_t ecns_get_input_port_arr_idx(capi_ecns_t *me_ptr, uint32_t input_port_id, uint32_t *input_port_idx_ptr)
{
   uint32_t available_ip_arr_idx = ECNS_MAX_INPUT_PORTS;
   for (uint32_t idx = 0; idx < ECNS_MAX_INPUT_PORTS; idx++)
   {
      if (input_port_id == me_ptr->in_port_info[idx].port_id)
      {
         *input_port_idx_ptr = idx;
         return CAPI_EOK;
      }
      else if (0 == me_ptr->in_port_info[idx].port_id)
      {
         available_ip_arr_idx = idx;
      }
   }

   if (available_ip_arr_idx != ECNS_MAX_INPUT_PORTS)
   {
      ECNS_DBG(me_ptr->miid,
               DBG_HIGH_PRIO,
               "capi mapping new input port id = 0x%x to index 0x%x",
               input_port_id,
               available_ip_arr_idx);

      me_ptr->in_port_info[available_ip_arr_idx].port_id = input_port_id;
      *input_port_idx_ptr                                = available_ip_arr_idx;
      return CAPI_EOK;
   }

   ECNS_DBG(me_ptr->miid,
            DBG_ERROR_PRIO,
            "Input Port ID = %lu to index mapping not found.",
            input_port_id);
   *input_port_idx_ptr = 0xFFFFFFFF;
   return CAPI_EFAILED;
}

/* =========================================================================
 * FUNCTION : ecns_get_output_port_arr_idx
 *
 * DESCRIPTION:
 * Each module maintains a output port info array.
 *    1. If a new output port ID is passed to this function, it will find an unused entry in the array and assigns
 *       that index to the given output port ID.
 *    2. If there is an existing entry with the given output control port ID. It returns that index.
 *    3. If a new output port ID is passed and there is no unused entry available, it return a UMAX_32 as error.
 * ========================================================================= */
capi_err_t ecns_get_output_port_arr_idx(capi_ecns_t *me_ptr, uint32_t output_port_id, uint32_t *output_port_idx_ptr)
{
   uint32_t available_op_arr_idx = ECNS_MAX_OUTPUT_PORTS;
   for (uint32_t idx = 0; idx < ECNS_MAX_OUTPUT_PORTS; idx++)
   {
      if (output_port_id == me_ptr->out_port_info[idx].port_id)
      {
         *output_port_idx_ptr = idx;
         return CAPI_EOK;
      }
      else if (0 == me_ptr->out_port_info[idx].port_id)
      {
         available_op_arr_idx = idx;
      }
   }

   if (available_op_arr_idx != ECNS_MAX_OUTPUT_PORTS)
   {
      ECNS_DBG(me_ptr->miid,
               DBG_HIGH_PRIO,
               "capi_ec: mapping new output port id = 0x%x to index 0x%x",
               output_port_id,
               available_op_arr_idx);

      me_ptr->out_port_info[available_op_arr_idx].port_id = output_port_id;
      *output_port_idx_ptr                                = available_op_arr_idx;
      return CAPI_EOK;
   }

   ECNS_DBG(me_ptr->miid,
            DBG_ERROR_PRIO,
            "capi_ec: output Port id = %lu to index mapping not found.",
            output_port_id);
   *output_port_idx_ptr = 0xFFFFFFFF;
   return CAPI_EFAILED;
}
