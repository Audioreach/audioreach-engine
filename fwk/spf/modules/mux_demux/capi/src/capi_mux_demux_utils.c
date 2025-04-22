/**
 * \file capi_mux_demux_utils.c
 *
 * \brief
 *        CAPI for mux demux module.
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "capi_mux_demux_utils.h"

static inline intf_extn_data_port_state_t portopcode_to_portstate(intf_extn_data_port_opcode_t opcode)
{
   switch (opcode)
   {
      case INTF_EXTN_DATA_PORT_OPEN:
         return DATA_PORT_STATE_OPENED;
      case INTF_EXTN_DATA_PORT_START:
         return DATA_PORT_STATE_STARTED;
      case INTF_EXTN_DATA_PORT_STOP:
         return DATA_PORT_STATE_STOPPED;
      case INTF_EXTN_DATA_PORT_SUSPEND:
         return DATA_PORT_STATE_STOPPED; // Handle Suspend similar to Stop
      default:
         return DATA_PORT_STATE_CLOSED;
   }
}

static void capi_mux_demux_update_raise_kpps_bw_events(capi_mux_demux_t* me_ptr)
{
   uint32_t total_kpps = 0;
   uint32_t total_bw   = 0;

   if (!me_ptr->output_port_info_ptr || !me_ptr->output_port_info_ptr[0].channel_connection_ptr)
   {
      return;
   }
   for (uint32_t out_arr_index = 0; out_arr_index < me_ptr->num_of_output_ports; out_arr_index++)
   {
      if (DATA_PORT_STATE_STARTED != me_ptr->output_port_info_ptr[out_arr_index].port_state)
      {
         continue;
      }

      uint32_t num_out_ch = MAX(me_ptr->output_port_info_ptr[out_arr_index].fmt.num_channels,
                                me_ptr->output_port_info_ptr[out_arr_index].fmt.num_max_channels);

      if (num_out_ch)
      {
         total_bw += (num_out_ch * me_ptr->operating_sample_rate *
                      (me_ptr->output_port_info_ptr[out_arr_index].fmt.bits_per_sample / 8));
      }

      for (uint32_t out_ch_index = 0; out_ch_index < me_ptr->output_port_info_ptr[out_arr_index].fmt.num_channels;
           out_ch_index++)
      {
         for (uint32_t k = 0; k < me_ptr->output_port_info_ptr[out_arr_index]
                                     .channel_connection_ptr[out_ch_index]
                                     .num_of_connected_input_channels;
              k++)
         {
            uint32_t input_port_index = me_ptr->output_port_info_ptr[out_arr_index]
                                           .channel_connection_ptr[out_ch_index]
                                           .input_connections_ptr[k]
                                           .input_port_index;

            if ((TRUE == me_ptr->input_port_info_ptr[input_port_index].fmt.is_valid) &&
                (DATA_PORT_STATE_STARTED == me_ptr->input_port_info_ptr[input_port_index].port_state))
            {
               total_bw += (me_ptr->operating_sample_rate *
                            (me_ptr->input_port_info_ptr[input_port_index].fmt.bits_per_sample / 8));
            }
         }
      }
   }

   // kpps for memset and memscpy, assusming 16 bytes are handled per packet.
   total_kpps = (total_bw >> 4) / 1000;

   capi_cmn_update_bandwidth_event(&me_ptr->cb_info, 0, total_bw);
   capi_cmn_update_kpps_event(&me_ptr->cb_info, total_kpps);
}

static void update_operating_media_format_candidate(capi_mux_demux_t *me_ptr)
{
   for (uint32_t i = 0; i < me_ptr->num_of_input_ports; i++)
   {
      me_ptr->input_port_info_ptr[i].b_operating_fmt_candidate = FALSE;

      if (DATA_PORT_STATE_STARTED != me_ptr->input_port_info_ptr[i].port_state || // port not yet started.
          0 == me_ptr->input_port_info_ptr[i].fmt.sample_rate)                    // valid media fmt not received.
      {
         continue;
      }
      for (uint32_t j = 0; j < me_ptr->num_of_output_ports; j++)
      {
         if (DATA_PORT_STATE_STARTED == me_ptr->output_port_info_ptr[j].port_state &&
             TRUE == me_ptr->input_port_info_ptr[i].is_output_connected[j]) // connected to an output port
         {
            me_ptr->input_port_info_ptr[i].b_operating_fmt_candidate = TRUE;
            break;
         }
      }

#ifdef MUX_DEMUX_TX_DEBUG_INFO
      AR_MSG(DBG_LOW_PRIO,
             "input port index,id [%lu, 0x%x] b_operating_fmt_candidate %d",
             i,
             me_ptr->input_port_info_ptr[i].port_id,
             me_ptr->input_port_info_ptr[i].b_operating_fmt_candidate);
#endif
   }
}

static void capi_mux_demux_raise_events(capi_mux_demux_t *me_ptr)
{
   if (0 == me_ptr->operating_sample_rate)
   {
      return; // not ready yet.
   }

   for (uint32_t i = 0; i < me_ptr->num_of_output_ports; i++)
   {
      capi_mux_demux_raise_out_port_media_format_event(me_ptr, i);
   }
   capi_mux_demux_update_raise_kpps_bw_events(me_ptr);
}

uint32_t capi_mux_demux_get_input_port_index_from_port_id(capi_mux_demux_t *me_ptr,
                                                          uint32_t         port_id)
{
   uint32_t num_ports = me_ptr->num_of_input_ports;
   mux_demux_input_port_info_t *port_info_ptr = me_ptr->input_port_info_ptr;
   for (uint32_t i = 0; i < num_ports; i++)
   {
      if (port_info_ptr[i].port_state != DATA_PORT_STATE_CLOSED && port_info_ptr[i].port_id == port_id)
      {
         return i;
      }
   }
   return num_ports;
}

uint32_t capi_mux_demux_get_output_arr_index_from_port_id(capi_mux_demux_t *me_ptr,
                                                           uint32_t         port_id)
{
   uint32_t available_arr_index = me_ptr->num_of_output_ports;
   mux_demux_output_port_info_t *port_info_ptr = me_ptr->output_port_info_ptr;

   for (uint32_t i = 0; i < me_ptr->num_of_output_ports; i++)
   {
      if (port_info_ptr[i].port_id == port_id)
      {
         return i;
      }
      else if (0 == port_info_ptr[i].port_id && i <= available_arr_index)
      {
          available_arr_index = i;

      }
   }

   if(me_ptr->num_of_output_ports > available_arr_index )
   {
       port_info_ptr[available_arr_index].port_id = port_id;
       return available_arr_index;
   }
   return me_ptr->num_of_output_ports;
}

uint32_t capi_mux_demux_get_output_arr_index_from_port_index(capi_mux_demux_t *me_ptr,
                                                           uint32_t                      port_index)
{
   for (uint32_t i = 0; i < me_ptr->num_of_output_ports; i++)
   {
      if (me_ptr->output_port_info_ptr[i].port_index == port_index)
      {
         return i;
      }
   }

#ifdef MUX_DEMUX_TX_DEBUG_INFO
   AR_MSG(DBG_ERROR_PRIO, "Port index not found, queried port index = %lu", port_index);
#endif

   return me_ptr->num_of_output_ports;
}

void capi_mux_demux_cleanup_port_config(capi_mux_demux_t *me_ptr)
{
   if (NULL != me_ptr->input_port_info_ptr)
   {
      posal_memory_free(me_ptr->input_port_info_ptr);
      me_ptr->input_port_info_ptr = NULL;
   }

   if (NULL != me_ptr->output_port_info_ptr)
   {
      if (NULL != me_ptr->output_port_info_ptr[0].channel_connection_ptr)
      {
         posal_memory_free(me_ptr->output_port_info_ptr[0].channel_connection_ptr);
      }

      posal_memory_free(me_ptr->output_port_info_ptr);
      me_ptr->output_port_info_ptr = NULL;
   }
}

capi_err_t capi_mux_demux_alloc_port_config(capi_mux_demux_t *me_ptr)
{
   uint32_t input_port_malloc_size  = 0;
   uint32_t output_port_malloc_size = 0;

   capi_mux_demux_cleanup_port_config(me_ptr);

   if (0 == me_ptr->num_of_input_ports || 0 == me_ptr->num_of_output_ports)
   {
      return CAPI_EOK; // no point of this module.
   }

   input_port_malloc_size =
      CAPI_ALIGN_8_BYTE(me_ptr->num_of_input_ports * sizeof(mux_demux_input_port_info_t)) +
      CAPI_ALIGN_8_BYTE(me_ptr->num_of_input_ports * me_ptr->num_of_output_ports * sizeof(bool_t));

   output_port_malloc_size = CAPI_ALIGN_8_BYTE(me_ptr->num_of_output_ports * sizeof(mux_demux_output_port_info_t));

   me_ptr->input_port_info_ptr =
      (mux_demux_input_port_info_t *)posal_memory_malloc(input_port_malloc_size, (POSAL_HEAP_ID)me_ptr->heap_id);
   me_ptr->output_port_info_ptr =
      (mux_demux_output_port_info_t *)posal_memory_malloc(output_port_malloc_size, (POSAL_HEAP_ID)me_ptr->heap_id);

   if (NULL == me_ptr->input_port_info_ptr || NULL == me_ptr->output_port_info_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "failed in malloc");
      capi_mux_demux_cleanup_port_config(me_ptr);
      return CAPI_ENOMEMORY;
   }

   memset(me_ptr->input_port_info_ptr, 0, input_port_malloc_size);
   memset(me_ptr->output_port_info_ptr, 0, output_port_malloc_size);

   me_ptr->input_port_info_ptr[0].is_output_connected =
      (bool_t *)CAPI_ALIGN_8_BYTE(me_ptr->input_port_info_ptr + me_ptr->num_of_input_ports);
   for (uint32_t i = 1; i < me_ptr->num_of_input_ports; i++)
   {
      me_ptr->input_port_info_ptr[i].is_output_connected =
         me_ptr->input_port_info_ptr[i - 1].is_output_connected + me_ptr->num_of_output_ports;
   }

   return CAPI_EOK;
}

/*function to handle data port operation */
capi_err_t capi_mux_demux_port_operation(capi_mux_demux_t *                     me_ptr,
                                         const intf_extn_data_port_operation_t *port_config_ptr)
{
   capi_err_t                   result              = CAPI_EOK;
   uint32_t                     number_of_port      = 0;
   void *                       port_info_ptr       = NULL;
   intf_extn_data_port_state_t *port_state_ptr      = NULL;
   uint32_t *                   port_id_ptr         = NULL;
   uint32_t *                   port_index_ptr      = NULL;
   bool_t                       b_update_connection = FALSE;

   if (port_config_ptr->is_input_port)
   {
      number_of_port = me_ptr->num_of_input_ports;
      port_info_ptr  = me_ptr->input_port_info_ptr;
   }
   else
   {
      number_of_port = me_ptr->num_of_output_ports;
      port_info_ptr  = me_ptr->output_port_info_ptr;
   }

   if (NULL == port_info_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "unexpected error.");
      return CAPI_EFAILED;
   }
#ifdef MUX_DEMUX_TX_DEBUG_INFO
   AR_MSG(DBG_LOW_PRIO, "received port operation, is_input_port %d", port_config_ptr->is_input_port);
#endif

   // update port info
   for (uint32_t i = 0; i < port_config_ptr->num_ports; i++)
   {
      uint32_t port_index = port_config_ptr->id_idx[i].port_index;
      uint32_t out_arr_index = me_ptr->num_of_output_ports;
      if (!port_config_ptr->is_input_port)
      {
         out_arr_index = capi_mux_demux_get_output_arr_index_from_port_id(me_ptr, port_config_ptr->id_idx[i].port_id); //for output port replace output port index with output array index
      }
      if (port_index >= number_of_port || (!port_config_ptr->is_input_port && out_arr_index >= number_of_port))
      {
         AR_MSG(DBG_ERROR_PRIO, "unexpected error");
         return CAPI_EFAILED;
      }
      else
      {
         if (port_config_ptr->is_input_port)
         {
            port_state_ptr = &(((mux_demux_input_port_info_t *)port_info_ptr)[port_index].port_state);
            port_id_ptr    = &(((mux_demux_input_port_info_t *)port_info_ptr)[port_index].port_id);
         }
         else
         {
            port_state_ptr = &(((mux_demux_output_port_info_t *)port_info_ptr)[out_arr_index].port_state);
            port_id_ptr    = &(((mux_demux_output_port_info_t *)port_info_ptr)[out_arr_index].port_id);
            port_index_ptr = &(((mux_demux_output_port_info_t *)port_info_ptr)[out_arr_index].port_index);
         }
         if (*port_state_ptr != DATA_PORT_STATE_CLOSED && *port_id_ptr != port_config_ptr->id_idx[i].port_id)
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "failing new port_id 0x%x operation because port_id 0x%x exists on the port_index 0x%x. output arr_index 0x%x.",
                   port_config_ptr->id_idx[i].port_id,
                   *port_id_ptr,
                   port_index,
                   out_arr_index);
            return CAPI_EFAILED;
         }

         if (DATA_PORT_STATE_CLOSED == *port_state_ptr &&
             DATA_PORT_STATE_CLOSED != portopcode_to_portstate(port_config_ptr->opcode))
         {
            b_update_connection = TRUE; // update connection if any port state is changed from close. This is the only
                                        // case where port_id and port_index mapping can change.

            // Invalidate media format.
            // Output port might already got the media format config, we should not clear here
            if (port_config_ptr->is_input_port)
            {
               memset(&(((mux_demux_input_port_info_t *)port_info_ptr)[port_index].fmt),
                      0,
                      sizeof(mux_demux_input_media_fmt_t));
            }
            else
            {
              //Resetting the output media format sum as port is closed, when it is reopend mf on output port is raised again.
              ((mux_demux_output_port_info_t *)port_info_ptr)[out_arr_index].fmt.media_fmt_sum = 0;
            }
         }
#ifdef MUX_DEMUX_TX_DEBUG_INFO
         AR_MSG(DBG_LOW_PRIO,
                "previous port info:\t is_input %d\t port_index 0x%x\t out_array_index 0x%x\t port_id 0x%x\t port_state 0x%x.",
                port_config_ptr->is_input_port,
                port_config_ptr->id_idx[i].port_index,
                out_arr_index,
                *port_id_ptr,
                *port_state_ptr);
#endif

         *port_id_ptr    = port_config_ptr->id_idx[i].port_id;
         if(NULL != port_index_ptr)
           {
             *port_index_ptr    = port_config_ptr->id_idx[i].port_index;
           }
         *port_state_ptr = portopcode_to_portstate(port_config_ptr->opcode);

         AR_MSG(DBG_LOW_PRIO,
                "updated port info:\t is_input %d\t port_index 0x%x\t out_array_index 0x%x\t port_id 0x%x\t port_state 0x%x.",
                port_config_ptr->is_input_port,
                port_config_ptr->id_idx[i].port_index,
                out_arr_index,
                *port_id_ptr,
                *port_state_ptr);
      }
   }

   if (b_update_connection)
   {
      result = capi_mux_demux_update_connection(me_ptr); // media format is updated inside this function.
   }
   else
   {
      // due to change in port state operating media format can change.
      capi_mux_demux_update_operating_fmt(me_ptr);
   }
   return result;
}

/*function to establish input to output connections */
capi_err_t capi_mux_demux_update_connection(capi_mux_demux_t *me_ptr)
{
   capi_err_t result = CAPI_EOK;
   if (NULL == me_ptr->input_port_info_ptr || NULL == me_ptr->output_port_info_ptr || NULL == me_ptr->cached_config_ptr)
   {
      return CAPI_EOK; // not enough info to update connections.
   }

#ifdef MUX_DEMUX_TX_DEBUG_INFO
   AR_MSG(DBG_LOW_PRIO, "updating connections.");
#endif

   mux_demux_connection_config_t *cached_connection_arr =
      (mux_demux_connection_config_t *)(me_ptr->cached_config_ptr + 1);
   uint32_t malloc_size =
      CAPI_ALIGN_8_BYTE(sizeof(mux_demux_input_connection_t) * me_ptr->cached_config_ptr->num_of_connections);
   mux_demux_input_connection_t *input_connection_ptr = NULL;

   // cleanup previous connection
   if (me_ptr->output_port_info_ptr[0].channel_connection_ptr)
   {
      posal_memory_free(me_ptr->output_port_info_ptr[0].channel_connection_ptr);
   }
   for (uint32_t i = 0; i < me_ptr->num_of_output_ports; i++)
   {
      me_ptr->output_port_info_ptr[i].channel_connection_ptr = NULL;

      // reset output stream's number of channels, it will be decided based on the connection config.
      me_ptr->output_port_info_ptr[i].fmt.num_channels = 0;
   }

   // clean the input-output connection matrix.
   memset(me_ptr->input_port_info_ptr[0].is_output_connected,
          0,
          sizeof(bool_t) * me_ptr->num_of_input_ports * me_ptr->num_of_output_ports);

   if (0 == me_ptr->cached_config_ptr->num_of_connections)
   {
      return CAPI_EOK;
   }

   // get the number of channels for each output stream
   for (uint32_t i = 0; i < me_ptr->cached_config_ptr->num_of_connections; i++)
   {

      uint32_t out_port_id = cached_connection_arr[i].output_port_id;
      uint32_t in_port_id  = cached_connection_arr[i].input_port_id;

      uint32_t out_port_arr_index = capi_mux_demux_get_output_arr_index_from_port_id(me_ptr,
                                                                                  out_port_id);
      uint32_t in_port_index  = capi_mux_demux_get_input_port_index_from_port_id(me_ptr,
                                                                                in_port_id);

      if (out_port_arr_index >= me_ptr->num_of_output_ports || in_port_index >= me_ptr->num_of_input_ports)
      {

         continue; // config received for the port which is not present
      }

      // find number of channels based on the maximum channel index.
      if (me_ptr->output_port_info_ptr[out_port_arr_index].fmt.num_channels <=
          cached_connection_arr[i].output_channel_index)
      {
         me_ptr->output_port_info_ptr[out_port_arr_index].fmt.num_channels =
            cached_connection_arr[i].output_channel_index + 1;
      }
   }

   // get the total size for output connection config
   for (uint32_t i = 0; i < me_ptr->num_of_output_ports; i++)
   {
#ifdef MUX_DEMUX_TX_DEBUG_INFO
      AR_MSG(DBG_LOW_PRIO,
             "output port index, id [%lu, %lu]\t number of channels based on connections %lu.",
             i,
             me_ptr->output_port_info_ptr[i].port_id,
             me_ptr->output_port_info_ptr[i].fmt.num_channels);
#endif
      malloc_size +=
         CAPI_ALIGN_8_BYTE(me_ptr->output_port_info_ptr[i].fmt.num_channels * sizeof(mux_demux_channel_connection_t));
   }

   // allocate new memory
   me_ptr->output_port_info_ptr[0].channel_connection_ptr =
      (mux_demux_channel_connection_t *)posal_memory_malloc(malloc_size, (POSAL_HEAP_ID)me_ptr->heap_id);
   if (NULL == me_ptr->output_port_info_ptr[0].channel_connection_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "failed in malloc");
      for (uint32_t i = 0; i < me_ptr->num_of_output_ports; i++)
      {
         me_ptr->output_port_info_ptr[i].fmt.num_channels = 0;
      }
      return CAPI_ENOMEMORY;
   }

   // clear the memory
   memset(me_ptr->output_port_info_ptr[0].channel_connection_ptr, 0, malloc_size);

   // assign the memory
   {
      uint32_t i = 1;
      for (i = 1; i < me_ptr->num_of_output_ports; i++)
      {
         me_ptr->output_port_info_ptr[i].channel_connection_ptr = (mux_demux_channel_connection_t *)CAPI_ALIGN_8_BYTE(
            me_ptr->output_port_info_ptr[i - 1].channel_connection_ptr +
            me_ptr->output_port_info_ptr[i - 1].fmt.num_channels);
      }

      input_connection_ptr =
         (mux_demux_input_connection_t *)CAPI_ALIGN_8_BYTE(me_ptr->output_port_info_ptr[i - 1].channel_connection_ptr +
                                                           me_ptr->output_port_info_ptr[i - 1].fmt.num_channels);
   }

   // assign the input connection memory and update connection
   for (uint32_t out_port_arr_index = 0; out_port_arr_index < me_ptr->num_of_output_ports; out_port_arr_index++)
   {
      for (uint32_t out_channel_position = 0;
           out_channel_position < me_ptr->output_port_info_ptr[out_port_arr_index].fmt.num_channels;
           out_channel_position++)
      {
         uint32_t j = 0;
         for (uint32_t i = 0; i < me_ptr->cached_config_ptr->num_of_connections; i++)
         {
            if (me_ptr->output_port_info_ptr[out_port_arr_index].port_id == cached_connection_arr[i].output_port_id &&
                out_channel_position == cached_connection_arr[i].output_channel_index)
            {

               uint32_t in_port_index =
                  capi_mux_demux_get_input_port_index_from_port_id(me_ptr,
                                                                   cached_connection_arr[i].input_port_id);

               if (in_port_index < me_ptr->num_of_input_ports)
               {
                  input_connection_ptr[j].input_port_index    = in_port_index;
                  input_connection_ptr[j].input_channel_index = cached_connection_arr[i].input_channel_index;
                  input_connection_ptr[j].coeff_q15           = ONE_Q15_32BIT;
                  me_ptr->input_port_info_ptr[in_port_index].is_output_connected[out_port_arr_index] = TRUE;

                  AR_MSG(DBG_LOW_PRIO,
                         "connection [%lu, %lu] ----0x%x---->[%lu, %lu]",
                         in_port_index,
                         input_connection_ptr[j].input_channel_index,
                         input_connection_ptr[j].coeff_q15,
                         me_ptr->output_port_info_ptr[out_port_arr_index].port_index,
                         out_channel_position);
                  j++;
               }
            }
         }
         me_ptr->output_port_info_ptr[out_port_arr_index]
            .channel_connection_ptr[out_channel_position]
            .num_of_connected_input_channels = j;
         me_ptr->output_port_info_ptr[out_port_arr_index]
            .channel_connection_ptr[out_channel_position]
            .input_connections_ptr = input_connection_ptr;
         input_connection_ptr += j;

#ifdef MUX_DEMUX_TX_DEBUG_INFO
         AR_MSG(DBG_LOW_PRIO,
                "output port index/channel [%lu, %lu] is connected with %lu number of input channels",
                me_ptr->output_port_info_ptr[out_port_arr_index].port_index,
                out_channel_position,
                j);
#endif
      }
   }

   // due to change in port connections operating media can change.
   capi_mux_demux_update_operating_fmt(me_ptr);

   return result;
}

void capi_mux_demux_update_operating_fmt(capi_mux_demux_t *me_ptr)
{
   uint32_t num_of_valid_input_ports = 0;

#ifdef MUX_DEMUX_TX_DEBUG_INFO
   AR_MSG(DBG_LOW_PRIO, "updating operating format. current operating sample rate %lu", me_ptr->operating_sample_rate);
#endif

   update_operating_media_format_candidate(me_ptr);

   // If operating format is not valid then find one.
   if (0 == me_ptr->operating_sample_rate)
   {
      for (uint32_t i = 0; i < me_ptr->num_of_input_ports; i++)
      {
         if (me_ptr->input_port_info_ptr[i].b_operating_fmt_candidate)
         {
            me_ptr->operating_sample_rate = me_ptr->input_port_info_ptr[i].fmt.sample_rate;
            break;
         }
      }
   }

   if (0 != me_ptr->operating_sample_rate)
   {
      // If operating format is valid then check it against all input port format and update them.
      for (uint32_t i = 0; i < me_ptr->num_of_input_ports; i++)
      {
         me_ptr->input_port_info_ptr[i].fmt.is_valid = FALSE;

         if (me_ptr->operating_sample_rate == me_ptr->input_port_info_ptr[i].fmt.sample_rate &&
             me_ptr->input_port_info_ptr[i].b_operating_fmt_candidate)
         {
            me_ptr->input_port_info_ptr[i].fmt.is_valid = TRUE;
            num_of_valid_input_ports++;
#ifdef MUX_DEMUX_TX_DEBUG_INFO
            AR_MSG(DBG_LOW_PRIO, "input port index %lu running on operating sample rate.", i);
#endif
         }
      }

      // If none of the input port format is valid then operating format is out-dated. update it again.
      if (0 == num_of_valid_input_ports)
      {
         me_ptr->operating_sample_rate = 0;
         capi_mux_demux_update_operating_fmt(me_ptr);
      }
      else
      {
         // update the events
         capi_mux_demux_raise_events(me_ptr);
      }
   }

#ifdef MUX_DEMUX_TX_DEBUG_INFO
   AR_MSG(DBG_LOW_PRIO, "updated operating format. current operating sample rate %lu", me_ptr->operating_sample_rate);
#endif
}

void capi_mux_demux_raise_out_port_media_format_event(capi_mux_demux_t *me_ptr, uint32_t out_port_arr_index)
{
   if (DATA_PORT_STATE_STARTED != me_ptr->output_port_info_ptr[out_port_arr_index].port_state ||
       0 == me_ptr->operating_sample_rate
       )
   {
      return; // not ready yet.
   }

   capi_media_fmt_v2_t media_fmt     = MUX_DEMUX_MEDIA_FMT_V2;
   uint32_t            media_fmt_sum = 0;
   media_fmt.format.sampling_rate    = me_ptr->operating_sample_rate;
   uint32_t out_port_index = me_ptr->output_port_info_ptr[out_port_arr_index].port_index;

   media_fmt.format.num_channels = me_ptr->output_port_info_ptr[out_port_arr_index].fmt.num_max_channels;

   // If output media configuration is not received from client then set it based on the first connected input.
   if (0 == me_ptr->output_port_info_ptr[out_port_arr_index].fmt.num_max_channels)
   {
      // number of channels based on the connections
      media_fmt.format.num_channels = me_ptr->output_port_info_ptr[out_port_arr_index].fmt.num_channels;

      // default channel map
      memscpy(me_ptr->output_port_info_ptr[out_port_arr_index].fmt.channel_type,
              sizeof(me_ptr->output_port_info_ptr[out_port_arr_index].fmt.channel_type),
              MUX_DEMUX_MEDIA_FMT_V2.channel_type,
              sizeof(MUX_DEMUX_MEDIA_FMT_V2.channel_type));

      // bits per sample and q factor based on the first valid and connected input stream
      for (uint32_t j = 0; j < me_ptr->num_of_input_ports; j++)
      {
         if (me_ptr->input_port_info_ptr[j].is_output_connected[out_port_arr_index] &&
             me_ptr->input_port_info_ptr[j].fmt.is_valid)
         {
            me_ptr->output_port_info_ptr[out_port_arr_index].fmt.bits_per_sample =
               me_ptr->input_port_info_ptr[j].fmt.bits_per_sample;
            me_ptr->output_port_info_ptr[out_port_arr_index].fmt.q_factor = me_ptr->input_port_info_ptr[j].fmt.q_factor;
            break;
         }
      }
   }

   media_fmt.format.bits_per_sample = me_ptr->output_port_info_ptr[out_port_arr_index].fmt.bits_per_sample;
   media_fmt.format.q_factor        = me_ptr->output_port_info_ptr[out_port_arr_index].fmt.q_factor;

   media_fmt_sum = media_fmt.format.sampling_rate * 10 + media_fmt.format.num_channels * 10 + media_fmt.format.q_factor;

   if (0 != media_fmt.format.num_channels       // Either out format is received or connected to input streams
       && 0 != media_fmt.format.bits_per_sample // Either out format is received or connected to valid input stream
       && media_fmt_sum != me_ptr->output_port_info_ptr[out_port_arr_index].fmt.media_fmt_sum) // media format is changed.
   {
      memscpy(media_fmt.channel_type,
              sizeof(media_fmt.channel_type),
              me_ptr->output_port_info_ptr[out_port_arr_index].fmt.channel_type,
              sizeof(me_ptr->output_port_info_ptr[out_port_arr_index].fmt.channel_type));
      capi_cmn_output_media_fmt_event_v2(&me_ptr->cb_info, &media_fmt, FALSE, out_port_index);

      me_ptr->output_port_info_ptr[out_port_arr_index].fmt.media_fmt_sum = media_fmt_sum;

#ifdef MUX_DEMUX_TX_DEBUG_INFO
      AR_MSG(DBG_LOW_PRIO,
             "output format event for port index,id [%lu, 0x%x] sample_rate %lu, num_channels %lu, qfactor %lu",
             me_ptr->output_port_info_ptr[out_port_arr_index].port_index,
             me_ptr->output_port_info_ptr[out_port_arr_index].port_id,
             media_fmt.format.sampling_rate,
             media_fmt.format.num_channels,
             media_fmt.format.q_factor);
#endif
   }
}
