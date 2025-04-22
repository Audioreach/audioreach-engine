/**
 * \file capi_demuxer.c
 * \brief
 *     Source file to implement the CAPI Interface for Demuxer Module.
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "capi_demuxer.h"
#include "capi_demuxer_utils.h"
#include "spf_list_utils.h"

static capi_vtbl_t vtbl = { capi_demuxer_process,        capi_demuxer_end,
                            capi_demuxer_set_param,      capi_demuxer_get_param,
                            capi_demuxer_set_properties, capi_demuxer_get_properties };

capi_vtbl_t *capi_demuxer_get_vtbl()
{
   return &vtbl;
}

// input_channel_index (num-1) and channel_type(num-2)  both of them unit16_t type
#define DEMUXER_NUM_OF_VARIABLES 2

/*------------------------------------------------------------------------
  Function name: capi_demuxer_get_static_properties
  DESCRIPTION: Function to get the static properties of ss module
  -----------------------------------------------------------------------*/
capi_err_t capi_demuxer_get_static_properties(capi_proplist_t *init_set_properties,
                                                  capi_proplist_t *static_properties)
{
   capi_err_t capi_result = CAPI_EOK;
   AR_MSG(DBG_HIGH_PRIO, "capi_demuxer: Enter Get static Properties");
   if (NULL != static_properties)
   {
      capi_result |= capi_demuxer_get_properties((capi_t *)NULL, static_properties);
   }
   return capi_result;
}

/*------------------------------------------------------------------------
  Function name: capi_demuxer_init
  DESCRIPTION: Initialize the CAPIv2 ss module and library.
  -----------------------------------------------------------------------*/
capi_err_t capi_demuxer_init(capi_t *_pif, capi_proplist_t *init_set_properties)
{
   if ((NULL == _pif) || (NULL == init_set_properties))
   {
      AR_MSG(DBG_ERROR_PRIO, "capi_demuxer: Init received NULL pointer");
      return CAPI_EBADPARAM;
   }
   AR_MSG(DBG_HIGH_PRIO, "capi_demuxer: Initializing");

   capi_demuxer_t *me_ptr = (capi_demuxer_t *)_pif;
   memset(me_ptr, 0, sizeof(capi_demuxer_t));
   me_ptr->vtbl.vtbl_ptr = capi_demuxer_get_vtbl(); // assigning the vtbl with all function pointers

   // init oprating mf
   capi_cmn_init_media_fmt_v2(&me_ptr->operating_mf);

   AR_MSG(DBG_HIGH_PRIO, "capi_demuxer: Setting Init Properties");
   // should contain EVENT_CALLBACK_INFO
   return capi_demuxer_set_properties((capi_t *)me_ptr, init_set_properties);
}

/*------------------------------------------------------------------------
  Function name: capi_demuxer_end
  DESCRIPTION: Returns the library to the uninitialized state and frees the
  memory that was allocated by module. This function also frees the virtual
  function table.
  -----------------------------------------------------------------------*/
capi_err_t capi_demuxer_end(capi_t *_pif)
{
   capi_err_t          capi_result = CAPI_EOK;
   capi_demuxer_t *me_ptr      = (capi_demuxer_t *)_pif;
   if (NULL == me_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi_demuxer: End received bad pointer, 0x%lx", (uint32_t)me_ptr);
      return CAPI_EBADPARAM;
   }
   if (NULL != me_ptr->out_port_state_arr)
   {
      posal_memory_free((void *)me_ptr->out_port_state_arr);
   }

   if (NULL != me_ptr->cached_out_cfg_arr)
   {
      posal_memory_free((void *)me_ptr->cached_out_cfg_arr);
   }

   me_ptr->vtbl.vtbl_ptr = NULL;

#ifdef DEMUXER_DBG_LOW
   AR_MSG(DBG_HIGH_PRIO, "capi_demuxer: Module ended");
#endif // DEMUXER_DBG_LOW
   return capi_result;
}
/*------------------------------------------------------------------------
  Function name: capi_demuxer_set_param
  DESCRIPTION: Function to set parameter value\structure.
  -----------------------------------------------------------------------*/
capi_err_t capi_demuxer_set_param(capi_t *                _pif,
                                      uint32_t                param_id,
                                      const capi_port_info_t *port_info_ptr,
                                      capi_buf_t *            params_ptr)
{
   capi_err_t          capi_result = CAPI_EOK;
   capi_demuxer_t *me_ptr      = (capi_demuxer_t *)(_pif);
   if (NULL == _pif || NULL == params_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi_demuxer: set param received bad pointer, 0x%p, 0x%p", _pif, params_ptr);
      return CAPI_EBADPARAM;
   }
   switch (param_id)
   {
      case PARAM_ID_DEMUXER_OUT_CONFIG:
      {
        AR_MSG(DBG_HIGH_PRIO, "SET params_ptr->actual_data_len  %u ",params_ptr->actual_data_len);
         const param_id_demuxer_out_config_t *config_ptr =
            (param_id_demuxer_out_config_t *)(params_ptr->data_ptr);
         uint32_t required_payload_size = sizeof(param_id_demuxer_out_config_t);
         // uint32_t                        local_payload_size=0;
         demuxer_out_config_t *out_config_t = NULL;

         if ((params_ptr->actual_data_len < required_payload_size) ||
             (params_ptr->actual_data_len <
              required_payload_size + (config_ptr->num_out_ports * sizeof(demuxer_out_config_t))))
         {
            AR_MSG(DBG_ERROR_PRIO, "Insufficient size for PARAM_ID_DEMUXER_OUT_CONFIG ");
            return CAPI_ENEEDMORE;
         }

         // memory allocation is based on me_ptr->num_out_ports
         if (config_ptr->num_out_ports > me_ptr->num_out_ports)
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "Mismatch in max number of output ports expected %u,received %u ",
                   me_ptr->num_out_ports,
                   config_ptr->num_out_ports);
            return CAPI_EBADPARAM;
         }

         for (uint32_t i = 0; i < config_ptr->num_out_ports; i++)
         {
            uint32_t out_port_index = 0;

            out_config_t = (demuxer_out_config_t *)(params_ptr->data_ptr + required_payload_size);
            required_payload_size += sizeof(demuxer_out_config_t) +
                                     (out_config_t->num_channels * sizeof(uint16_t) * DEMUXER_NUM_OF_VARIABLES);
            if (params_ptr->actual_data_len < required_payload_size)
            {
               AR_MSG(DBG_ERROR_PRIO, "Insufficient size for demuxer_out_config_t ");
               return CAPI_ENEEDMORE;
            }

            required_payload_size = CAPI_ALIGN_4_BYTE(required_payload_size); // in case if num_channels are odd

            // First cache the set param
            me_ptr->cached_out_cfg_arr[i].num_channels   = out_config_t->num_channels;
            me_ptr->cached_out_cfg_arr[i].output_port_id = out_config_t->output_port_id;
            uint8_t *chtype_address                      = (uint8_t *)(out_config_t + 1);
            chtype_address = chtype_address + out_config_t->num_channels * sizeof(uint16_t);
            memscpy(&me_ptr->cached_out_cfg_arr[i].input_channel_index[0],
                    sizeof(me_ptr->cached_out_cfg_arr[i].input_channel_index),
                    (out_config_t + 1),
                    out_config_t->num_channels * sizeof(uint16_t));

            for (uint32_t k = 0; k < out_config_t->num_channels; k++)
            {
               AR_MSG(DBG_HIGH_PRIO,
                      " output_port_id %u, input_channel_index %u - channel %u",
                      out_config_t->output_port_id,me_ptr->cached_out_cfg_arr[i].input_channel_index[k],
                      k);
            }

            memscpy(&me_ptr->cached_out_cfg_arr[i].output_channel_type[0],
                    sizeof(me_ptr->cached_out_cfg_arr[i].output_channel_type),
                    chtype_address,
                    out_config_t->num_channels * sizeof(uint16_t));

            for (uint32_t k = 0; k < out_config_t->num_channels; k++)
            {
               AR_MSG(DBG_HIGH_PRIO,
                      " output_port_id %u output_channel_type %u - channel %u",
                      out_config_t->output_port_id,me_ptr->cached_out_cfg_arr[i].output_channel_type[k],
                      k);
            }

            // Check if the port is already opened
            capi_demuxer_get_port_index(me_ptr, out_config_t->output_port_id, &out_port_index);
            if (out_port_index >= me_ptr->num_out_ports)
            {
               continue; // config received for the port which is not present
            }

            // If we are here, port is already opened.
            // Update out_port_state_arr with appropriate channels,channel indices etc
            me_ptr->out_port_state_arr[out_port_index].num_channels = out_config_t->num_channels;
            memscpy(&me_ptr->out_port_state_arr[out_port_index].input_channel_index[0],
                    sizeof(me_ptr->out_port_state_arr[out_port_index].input_channel_index),
                    (out_config_t + 1),
                    out_config_t->num_channels * sizeof(uint16_t));
            memscpy(&me_ptr->out_port_state_arr[out_port_index].output_channel_type[0],
                    sizeof(me_ptr->out_port_state_arr[out_port_index].output_channel_type),
                    chtype_address,
                    out_config_t->num_channels * sizeof(uint16_t));

            me_ptr->out_port_state_arr[out_port_index].is_cfg_received = TRUE;

            // If input media format is already received, raise ouput mf corresponding to this port
            if (me_ptr->is_in_media_fmt_set)
            {
               capi_demuxer_validate_out_cfg_and_raise_out_mf(me_ptr, out_port_index);
            }
         }

         break;
      }

      case INTF_EXTN_PARAM_ID_DATA_PORT_OPERATION:
      {
         if (NULL == me_ptr->out_port_state_arr)
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "Port structures memory should have been allocated for max ports during OPEN. Error");
            capi_result |= CAPI_EFAILED;
            break;
         }
         if (params_ptr->actual_data_len < sizeof(intf_extn_data_port_operation_t))
         {
            AR_MSG(DBG_ERROR_PRIO, "capi_demuxer: payload size for port operation %d", params_ptr->actual_data_len);
            capi_result |= CAPI_ENEEDMORE;
            break;
         }
         if (NULL == params_ptr->data_ptr)
         {
            AR_MSG(DBG_ERROR_PRIO, "capi_demuxer: Set property id 0x%lx, received null buffer", param_id);
            capi_result |= CAPI_EBADPARAM;
            break;
         }
         intf_extn_data_port_operation_t *data_ptr = (intf_extn_data_port_operation_t *)(params_ptr->data_ptr);
         if (params_ptr->actual_data_len <
             sizeof(intf_extn_data_port_operation_t) + (data_ptr->num_ports * sizeof(intf_extn_data_port_id_idx_map_t)))
         {
            AR_MSG(DBG_ERROR_PRIO, "capi_demuxer: payload size for port operation %d", params_ptr->actual_data_len);
            capi_result |= CAPI_ENEEDMORE;
            break;
         }
         /* we do bookkeeping only for the output ports for dmeuxer since there are multiple ports
          * on the output side. In general, we do bookkeeping for whichever side (can be both) that has multi-port
          * capability.*/
         if (FALSE == data_ptr->is_input_port)
         {
            // output port
            if (data_ptr->num_ports > me_ptr->num_out_ports)
            {
               AR_MSG(DBG_ERROR_PRIO, "capi_demuxer: Bad parameter, num_ports %d", data_ptr->num_ports);
               capi_result |= CAPI_EBADPARAM;
               break;
            }
            for (uint32_t i = 0; i < data_ptr->num_ports; i++)
            {
               if (data_ptr->id_idx[i].port_index >= me_ptr->num_out_ports)
               {
                  AR_MSG(DBG_ERROR_PRIO,
                         "capi_demuxer: Bad parameter in id-idx map on port %lu, port_index = %lu num ports = "
                         "%d",
                         i,
                         data_ptr->id_idx[i].port_index,
                         me_ptr->num_out_ports);
                  capi_result |= CAPI_EBADPARAM;
                  break; // breaks out of for
               }
               // save the port_id at the port_index (array of size max ports)
               switch (data_ptr->opcode)
               {
                  case INTF_EXTN_DATA_PORT_OPEN:
                  {
                     // don't need specific payload
                     me_ptr->out_port_state_arr[data_ptr->id_idx[i].port_index].port_id = data_ptr->id_idx[i].port_id;
                     me_ptr->out_port_state_arr[data_ptr->id_idx[i].port_index].state   = DATA_PORT_STATE_OPENED;
                     uint32_t port_index = data_ptr->id_idx[i].port_index;

                     AR_MSG(DBG_HIGH_PRIO,
                            "capi_demuxer: Opening Port ID %lu, Index %lu",
                            data_ptr->id_idx[i].port_id,
                            data_ptr->id_idx[i].port_index);

                     if (me_ptr->is_in_media_fmt_set)
                     {
                        if (me_ptr->out_port_state_arr[port_index].is_cfg_received)
                        {
                        }
                        else
                        {
                           capi_demuxer_search_cached_cfg_and_update_out_cfg(me_ptr,
                                                                                 port_index,
                                                                                 data_ptr->id_idx[i].port_id);
                        }
                        capi_demuxer_validate_out_cfg_and_raise_out_mf(me_ptr, port_index);
                     }
                     break;
                  }
                  case INTF_EXTN_DATA_PORT_CLOSE:
                  {

                     AR_MSG(DBG_HIGH_PRIO,
                            "capi_demuxer: Closing Port ID %lu, Index %lu",
                            data_ptr->id_idx[i].port_id,
                            data_ptr->id_idx[i].port_index);

                     // don't need specific payload
                     me_ptr->out_port_state_arr[data_ptr->id_idx[i].port_index].port_id = 0;
                     me_ptr->out_port_state_arr[data_ptr->id_idx[i].port_index].state   = DATA_PORT_STATE_CLOSED;
                     break;
                  }
                  case INTF_EXTN_DATA_PORT_START:
                  {
#ifdef DEMUXER_DBG_LOW
                     AR_MSG(DBG_HIGH_PRIO,
                            "capi_demuxer: Starting Port ID %lu, Index %lu",
                            data_ptr->id_idx[i].port_id,
                            data_ptr->id_idx[i].port_index);
#endif // DEMUXER_DBG_LOW
                     // don't need specific payload
                     me_ptr->out_port_state_arr[data_ptr->id_idx[i].port_index].state = DATA_PORT_STATE_STARTED;
                     break;
                  }
                  case INTF_EXTN_DATA_PORT_STOP:
                  {
#ifdef DEMUXER_DBG_LOW
                     AR_MSG(DBG_HIGH_PRIO,
                            "capi_demuxer: Stopping Port ID %lu, Index %lu",
                            data_ptr->id_idx[i].port_id,
                            data_ptr->id_idx[i].port_index);
#endif // DEMUXER_DBG_LOW
                     // don't need specific payload
                     me_ptr->out_port_state_arr[data_ptr->id_idx[i].port_index].state = DATA_PORT_STATE_STOPPED;
                     break;
                  }
                  default:
                  {
                     AR_MSG(DBG_ERROR_PRIO,
                            "capi_demuxer: Port operation - Unsupported opcode: %lu",
                            data_ptr->opcode);
                     CAPI_SET_ERROR(capi_result, CAPI_EUNSUPPORTED);
                     break;
                  }
               }
            }
         }
         break;
      } // CAPI_PORT_OPERATION
      default:
      {
         capi_result = CAPI_EUNSUPPORTED;
      }
   }
   return capi_result;
}
/*------------------------------------------------------------------------
  Function name: capi_demuxer_get_param
  DESCRIPTION: Function to get parameter value\structure.
  -----------------------------------------------------------------------*/
capi_err_t capi_demuxer_get_param(capi_t *                _pif,
                                      uint32_t                param_id,
                                      const capi_port_info_t *port_info_ptr,
                                      capi_buf_t *            params_ptr)
{

if (NULL == _pif || NULL == params_ptr || NULL == params_ptr->data_ptr)
{
   AR_MSG(DBG_ERROR_PRIO, "get param received bad pointers");
   return CAPI_EBADPARAM;
}	
 capi_err_t        result = CAPI_EOK;
 capi_demuxer_t *me_ptr      = (capi_demuxer_t *)(_pif);
 params_ptr->actual_data_len = 0;	
 
 switch (param_id)
   {
      case PARAM_ID_DEMUXER_OUT_CONFIG:
      {
		 uint32_t required_payload_size = sizeof(param_id_demuxer_out_config_t); 
		 
		 if ((params_ptr->max_data_len < required_payload_size) ||
             (params_ptr->max_data_len <
              required_payload_size + (me_ptr->num_out_ports * sizeof(demuxer_out_config_t))))
         {
            AR_MSG(DBG_ERROR_PRIO, "Insufficient size for PARAM_ID_DEMUXER_OUT_CONFIG ");
			params_ptr->actual_data_len=0;
            return CAPI_ENEEDMORE;
         }
		 
		 param_id_demuxer_out_config_t *config_ptr =
            (param_id_demuxer_out_config_t *)(params_ptr->data_ptr);
		 config_ptr->num_out_ports=	me_ptr->num_out_ports;
		 	 
		  demuxer_out_config_t *out_config_t = NULL;
		 for (uint32_t i = 0; i < me_ptr->num_out_ports; i++)
         {
			 
			 out_config_t = (demuxer_out_config_t *)(params_ptr->data_ptr + required_payload_size);
			 
			  required_payload_size += sizeof(demuxer_out_config_t) +
                                     (me_ptr->cached_out_cfg_arr[i].num_channels * sizeof(uint16_t) * DEMUXER_NUM_OF_VARIABLES);
			
			
            if (params_ptr->max_data_len < required_payload_size)
            {
               AR_MSG(DBG_ERROR_PRIO, "Insufficient size for demuxer_out_config_t for outport %u",i);
               return CAPI_ENEEDMORE;
            }			
		
		    out_config_t->num_channels=  me_ptr->cached_out_cfg_arr[i].num_channels; 
            out_config_t->output_port_id= me_ptr->cached_out_cfg_arr[i].output_port_id;
			
			uint8_t *chtype_address                      = (uint8_t *)(out_config_t + 1);
            chtype_address = chtype_address + out_config_t->num_channels * sizeof(uint16_t);
            memscpy((out_config_t + 1),out_config_t->num_channels * sizeof(uint16_t),
			&me_ptr->cached_out_cfg_arr[i].input_channel_index[0],
			sizeof(me_ptr->cached_out_cfg_arr[i].input_channel_index));
			
			 memscpy(chtype_address,out_config_t->num_channels * sizeof(uint16_t),
			 &me_ptr->cached_out_cfg_arr[i].output_channel_type[0],
             sizeof(me_ptr->cached_out_cfg_arr[i].output_channel_type));
		   params_ptr->actual_data_len = required_payload_size;
		  
         }
		 
		 AR_MSG(DBG_LOW_PRIO, "Filled PID actual_data_len %u",params_ptr->actual_data_len);
         break;
      }
      default:
         AR_MSG(DBG_ERROR_PRIO, "Invalid getparam received 0x%lx", param_id);
         result = CAPI_EUNSUPPORTED;
         break;
   }

   return result;
	
}
/*------------------------------------------------------------------------
  Function name: capi_demuxer_set_properties
  DESCRIPTION: Function to set properties to the Demuxer module
  -----------------------------------------------------------------------*/
capi_err_t capi_demuxer_set_properties(capi_t *_pif, capi_proplist_t *props_ptr)
{
   capi_err_t          capi_result = CAPI_EOK;
   capi_demuxer_t *me_ptr      = (capi_demuxer_t *)_pif;

   if ((NULL == props_ptr) || (NULL == me_ptr))
   {
      AR_MSG(DBG_ERROR_PRIO, "capi_demuxer: Get property received null arguments");
      return CAPI_EBADPARAM;
   }

   capi_prop_t *prop_ptr = props_ptr->prop_ptr;
   // iterate over the properties
   for (uint32_t i = 0; i < props_ptr->props_num; i++)
   {
      capi_buf_t *payload_ptr = &prop_ptr[i].payload;
      switch (prop_ptr[i].id)
      {
         case CAPI_HEAP_ID:
         {
            if (payload_ptr->max_data_len < sizeof(capi_heap_id_t))
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "capi_demuxer: Set property id 0x%lx, Bad param size %lu",
                      prop_ptr[i].id,
                      payload_ptr->max_data_len);
               capi_result |= CAPI_ENEEDMORE;
               break;
            }
            if (NULL == payload_ptr->data_ptr)
            {
               AR_MSG(DBG_ERROR_PRIO, "capi_demuxer: Set property id 0x%lx, received null buffer", prop_ptr[i].id);
               capi_result |= CAPI_EBADPARAM;
               break;
            }
            capi_heap_id_t *data_ptr = (capi_heap_id_t *)payload_ptr->data_ptr;
            me_ptr->heap_mem.heap_id = data_ptr->heap_id;
            break;
         } // CAPI_HEAP_ID
         case CAPI_EVENT_CALLBACK_INFO:
         {
            if (payload_ptr->max_data_len < sizeof(capi_event_callback_info_t))
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "capi_demuxer: Set property id 0x%lx, Bad param size %lu",
                      prop_ptr[i].id,
                      payload_ptr->max_data_len);
               capi_result |= CAPI_ENEEDMORE;
               break;
            }
            if (NULL == payload_ptr->data_ptr)
            {
               AR_MSG(DBG_ERROR_PRIO, "capi_demuxer: Set property id 0x%lx, received null buffer", prop_ptr[i].id);
               capi_result |= CAPI_EBADPARAM;
               break;
            }
            capi_event_callback_info_t *data_ptr = (capi_event_callback_info_t *)payload_ptr->data_ptr;
            me_ptr->cb_info.event_cb             = data_ptr->event_cb;
            me_ptr->cb_info.event_context        = data_ptr->event_context;
            payload_ptr->actual_data_len         = sizeof(capi_event_callback_info_t);
            break;
         } // CAPI_EVENT_CALLBACK_INFO
         case CAPI_MODULE_INSTANCE_ID:
         {
            if (payload_ptr->actual_data_len >= sizeof(capi_module_instance_id_t))
            {
               capi_module_instance_id_t *data_ptr = (capi_module_instance_id_t *)payload_ptr->data_ptr;
               me_ptr->miid                        = data_ptr->module_instance_id;
               AR_MSG(DBG_LOW_PRIO,
                      "capi_demuxer: This module-id 0x%08lX, instance-id 0x%08lX",
                      data_ptr->module_id,
                      me_ptr->miid);
            }
            else
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "capi_demuxer: Set property id 0x%lx, Bad param size %lu",
                      prop_ptr[i].id,
                      payload_ptr->max_data_len);
               capi_result |= CAPI_ENEEDMORE;
               break;
            }
            break;
         } // CAPI_MODULE_INSTANCE_ID
         case CAPI_PORT_NUM_INFO:
         {
            if (payload_ptr->actual_data_len < sizeof(capi_port_num_info_t))
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "capi_demuxer: Set property id 0x%lx, Bad param size %lu",
                      prop_ptr[i].id,
                      payload_ptr->actual_data_len);
               capi_result |= CAPI_ENEEDMORE;
               break;
            }
            if (NULL == payload_ptr->data_ptr)
            {
               AR_MSG(DBG_ERROR_PRIO, "capi_demuxer: Set property id 0x%lx, received null buffer", prop_ptr[i].id);
               capi_result |= CAPI_EBADPARAM;
               break;
            }
            capi_port_num_info_t *data_ptr = (capi_port_num_info_t *)payload_ptr->data_ptr;
            if (DEMUXER_MAX_INPUT_PORTS < data_ptr->num_input_ports)
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "capi_demuxer: Set property num port info - out of range, provided num input ports = "
                      "%lu, "
                      "num "
                      "output ports = %lu",
                      data_ptr->num_input_ports,
                      data_ptr->num_output_ports);
               capi_result |= CAPI_EBADPARAM;
               break;
            }
            // max number of output ports
            me_ptr->num_out_ports = data_ptr->num_output_ports;

            AR_MSG(DBG_HIGH_PRIO,
                   "capi_demuxer: Port num info set prop: num input ports = 1 and num output ports = %lu",
                   me_ptr->num_out_ports);

            // allocating memory for the bool array for max output ports
            me_ptr->out_port_state_arr = (capi_demuxer_out_port_state_t *)
               posal_memory_malloc(me_ptr->num_out_ports *
                                      (CAPI_ALIGN_8_BYTE(sizeof(capi_demuxer_out_port_state_t))),
                                   (POSAL_HEAP_ID)me_ptr->heap_mem.heap_id);
            if (NULL == me_ptr->out_port_state_arr)
            {
               AR_MSG(DBG_ERROR_PRIO, "Malloc Fail while allocating out_port_state_arr");
               capi_result |= CAPI_ENOMEMORY;
               break;
            }
            // false (inactive) intialize
            memset(me_ptr->out_port_state_arr, 0, me_ptr->num_out_ports * sizeof(capi_demuxer_out_port_state_t));

            // allocating memory for the bool array for max output ports
            me_ptr->cached_out_cfg_arr = (demuxer_cached_out_config_t *)
               posal_memory_malloc(me_ptr->num_out_ports * (CAPI_ALIGN_8_BYTE(sizeof(demuxer_cached_out_config_t))),
                                   (POSAL_HEAP_ID)me_ptr->heap_mem.heap_id);
            if (NULL == me_ptr->cached_out_cfg_arr)
            {
               AR_MSG(DBG_ERROR_PRIO, "Malloc Fail while allocating cached_out_cfg_arr");
               capi_result |= CAPI_ENOMEMORY;
               break;
            }
            // false (inactive) intialize
            memset(me_ptr->cached_out_cfg_arr, 0, me_ptr->num_out_ports * sizeof(demuxer_cached_out_config_t));

            // Set default indices for all ports
            for (uint32_t i = 0; i < me_ptr->num_out_ports; i++)
            {

               memscpy(&me_ptr->out_port_state_arr[i].input_channel_index[0],
                       sizeof(me_ptr->out_port_state_arr[i].input_channel_index),
                       &default_channel_index[0],
                       sizeof(default_channel_index));
            }

            break;
         } // CAPI_PORT_NUM_INFO
         case CAPI_ALGORITHMIC_RESET:
         {
            AR_MSG(DBG_HIGH_PRIO, "capi_demuxer: Reset, nothing to be done");
            break;
         } // CAPI_ALGORITHMIC_RESET
         case CAPI_INPUT_MEDIA_FORMAT_V2:
         {
            if (NULL == me_ptr->out_port_state_arr)
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "Port structures memory should have been allocated for max ports during OPEN. Error");
               capi_result |= CAPI_EFAILED;
               break;
            }
            /* Validate the MF payload */
            if (payload_ptr->actual_data_len <
                sizeof(capi_set_get_media_format_t) + sizeof(capi_standard_data_format_v2_t))
            {
               AR_MSG(DBG_ERROR_PRIO, "capi_demuxer: Invalid media format size %d", payload_ptr->actual_data_len);
               capi_result |= CAPI_ENEEDMORE;
               break;
            }
            if (NULL == payload_ptr->data_ptr)
            {
               AR_MSG(DBG_ERROR_PRIO, "capi_demuxer: Set property id 0x%lx, received null buffer", prop_ptr[i].id);
               capi_result |= CAPI_EBADPARAM;
               break;
            }
            capi_media_fmt_v2_t *media_fmt_ptr = (capi_media_fmt_v2_t *)(payload_ptr->data_ptr);
            // /* Validate data format, interleaving and num channels */
            if ((CAPI_RAW_COMPRESSED == media_fmt_ptr->header.format_header.data_format) ||
                (CAPI_MAX_CHANNELS_V2 < media_fmt_ptr->format.num_channels))
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "capi_demuxer: Unsupported Data format %lu or num_channels %lu. Max channels: %lu",
                      media_fmt_ptr->header.format_header.data_format,
                      media_fmt_ptr->format.num_channels,
					  CAPI_MAX_CHANNELS_V2);
               capi_result |= CAPI_EBADPARAM;
               break;
            }
            uint32_t size_to_copy = sizeof(capi_set_get_media_format_t) + sizeof(capi_standard_data_format_v2_t) +
                                    (media_fmt_ptr->format.num_channels * sizeof(capi_channel_type_t));

            AR_MSG(DBG_HIGH_PRIO,
                   "capi_demuxer: size_to_copy %u actual_data_len %u",
                   size_to_copy,
                   payload_ptr->actual_data_len);

            // accept as the operating Media format
            memscpy(&me_ptr->operating_mf, size_to_copy, media_fmt_ptr, payload_ptr->actual_data_len);
            me_ptr->is_in_media_fmt_set = TRUE;
            for (uint32_t i = 0; i < me_ptr->num_out_ports; i++)
            {

               // For all the outports - opened/not opened copy input media format to operating_out_mf
               // and copy default indices so that in case of any failure later, input will be
               // sent to output like simple_splitter
               memscpy(&me_ptr->out_port_state_arr[i].operating_out_mf,
                       size_to_copy,
                       media_fmt_ptr,
                       payload_ptr->actual_data_len);

               // If port is not opened dont raise output media format
               if (DATA_PORT_STATE_CLOSED == me_ptr->out_port_state_arr[i].state)
               {
                  continue;
               }

               if (me_ptr->out_port_state_arr[i].is_cfg_received)
               {
                  capi_result |= capi_demuxer_validate_out_cfg_and_raise_out_mf(me_ptr, i);
               }
               else
               {
                  AR_MSG(DBG_HIGH_PRIO,
                         "capi_demuxer: Output media format will be raised when "
                         "PARAM_ID_DEMUXER_OUT_CONFIG is received ");
               }
            }
            capi_result |= capi_demuxer_update_and_raise_kpps_event(me_ptr);
#ifdef DEMUXER_DBG_LOW
            AR_MSG(DBG_HIGH_PRIO,
                   "capi_demuxer: Input Media format set prop: bits per sample: %lu bytes, num in/out channels "
                   "%lu",
                   me_ptr->operating_mf.format.bits_per_sample,
                   me_ptr->operating_mf.format.num_channels);
#endif // DEMUXER_DBG_LOW
            break;
         } // CAPI_INPUT_MEDIA_FORMAT_V2
         default:
         {
            AR_MSG(DBG_ERROR_PRIO, "capi_demuxer: Unknown Prop[%d]", prop_ptr[i].id);
            capi_result |= CAPI_EUNSUPPORTED;
            break;
         }
      }
   } // for loop
   return capi_result;
}
/*------------------------------------------------------------------------
  Function name: capi_demuxer_get_properties
  DESCRIPTION: Function to get the properties from the Demuxer module
  -----------------------------------------------------------------------*/
capi_err_t capi_demuxer_get_properties(capi_t *_pif, capi_proplist_t *props_ptr)
{
   capi_err_t          capi_result = CAPI_EOK;
   capi_demuxer_t *me_ptr      = (capi_demuxer_t *)_pif;
   uint32_t            i;

   if (NULL == props_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi_demuxer: Get property received null property array");
      return CAPI_EBADPARAM;
   }
   capi_prop_t *prop_ptr = props_ptr->prop_ptr;

   capi_basic_prop_t mod_prop;
   mod_prop.init_memory_req    = sizeof(capi_demuxer_t);
   mod_prop.stack_size         = CAPI_DEMUXER_STACK_SIZE;
   mod_prop.num_fwk_extns      = 0;
   mod_prop.fwk_extn_ids_arr   = NULL;
   mod_prop.is_inplace         = FALSE; // not capable of in-place processing of data
   mod_prop.req_data_buffering = FALSE;
   mod_prop.max_metadata_size  = 0; // NA

   capi_result = capi_cmn_get_basic_properties(props_ptr, &mod_prop);
   if (CAPI_EOK != capi_result)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi_demuxer: Get common basic properties failed with result %lu", capi_result);
      return capi_result;
   }

   // iterating over the properties
   for (i = 0; i < props_ptr->props_num; i++)
   {
      capi_buf_t *payload_ptr = &prop_ptr[i].payload;

      switch (prop_ptr[i].id)
      {
         // begin static props
         case CAPI_INIT_MEMORY_REQUIREMENT:
         case CAPI_STACK_SIZE:
         case CAPI_IS_INPLACE:
         case CAPI_REQUIRES_DATA_BUFFERING:
         case CAPI_NUM_NEEDED_FRAMEWORK_EXTENSIONS:
         case CAPI_NEEDED_FRAMEWORK_EXTENSIONS:
         case CAPI_PORT_DATA_THRESHOLD: // ignore this.
         case CAPI_OUTPUT_MEDIA_FORMAT_SIZE:
         {
            break;
         }
         // end static props
         case CAPI_OUTPUT_MEDIA_FORMAT_V2:
         {
            if (NULL == me_ptr)
            {
               AR_MSG(DBG_ERROR_PRIO, "capi_demuxer: pif is NULL for get OUTPUT MF");
               capi_result |= CAPI_EBADPARAM;
               break;
            }

            AR_MSG(DBG_ERROR_PRIO, "capi_demuxer: %X get_prop CAPI_OUTPUT_MEDIA_FORMAT_V2", me_ptr->miid);

            if (prop_ptr[i].port_info.is_valid && !prop_ptr[i].port_info.is_input_port &&
                prop_ptr[i].port_info.port_index < me_ptr->num_out_ports)
            {
               uint32_t port_index = prop_ptr[i].port_info.port_index;

               if (me_ptr->out_port_state_arr == NULL)
               {
                  AR_MSG(DBG_ERROR_PRIO, "capi_demuxer: out_port_state_arr is NULL");
                  capi_result |= CAPI_EBADPARAM;
                  break;
               }

               if (me_ptr->is_in_media_fmt_set == FALSE)
               {
                  AR_MSG(DBG_ERROR_PRIO, "capi_demuxer: cannot update out mf before input mf is set");
                  capi_result |= CAPI_EBADPARAM;
                  break;
               }

               // Use channels in operating_out_mf not in out_port_state_arr
               uint32_t ret_size = sizeof(capi_set_get_media_format_t) + sizeof(capi_standard_data_format_v2_t) +
                                   (me_ptr->out_port_state_arr[port_index].operating_out_mf.format.num_channels *
                                    sizeof(capi_channel_type_t));

               /* Validate the MF payload */
               if (payload_ptr->max_data_len < sizeof(capi_media_fmt_v2_t))
               {
                  AR_MSG(DBG_ERROR_PRIO, "capi_demuxer: Invalid media format size %d", payload_ptr->max_data_len);

                  capi_result |= CAPI_ENEEDMORE;
                  break;
               }
               capi_media_fmt_v2_t *media_fmt_ptr = (capi_media_fmt_v2_t *)(payload_ptr->data_ptr);

               memscpy(media_fmt_ptr, ret_size, &me_ptr->out_port_state_arr[port_index].operating_out_mf, ret_size);
               payload_ptr->actual_data_len = sizeof(capi_media_fmt_v2_t);
            }
            break;
         } // CAPI_OUTPUT_MEDIA_FORMAT_V2
         case CAPI_INTERFACE_EXTENSIONS:
         {
            if (payload_ptr->max_data_len >= sizeof(capi_interface_extns_list_t))
            {
               capi_interface_extns_list_t *intf_ext_list = (capi_interface_extns_list_t *)payload_ptr->data_ptr;
               if (payload_ptr->max_data_len < (sizeof(capi_interface_extns_list_t) +
                                                (intf_ext_list->num_extensions * sizeof(capi_interface_extn_desc_t))))
               {
                  AR_MSG(DBG_ERROR_PRIO,
                         "capi_demuxer: CAPI_INTERFACE_EXTENSIONS invalid param size %lu",
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
                            "capi_demuxer: CAPI_INTERFACE_EXTENSIONS intf_ext = 0x%lx, is_supported = %d",
                            curr_intf_extn_desc_ptr->id,
                            (int)curr_intf_extn_desc_ptr->is_supported);
                     curr_intf_extn_desc_ptr++;
                  }
               }
            }
            else
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "capi_demuxer: CAPI_INTERFACE_EXTENSIONS Bad param size %lu",
                      payload_ptr->max_data_len);
               CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
            }
            break;
         }
         default:
         {
            AR_MSG(DBG_ERROR_PRIO, "capi_demuxer: Unknown Prop[0x%lX]", prop_ptr[i].id);
            capi_result |= CAPI_EUNSUPPORTED;
            break;
         }
      }
   }
   return capi_result;
}

/*------------------------------------------------------------------------
  Function name: capi_demuxer_process
  DESCRIPTION: Processes an input buffer and generates an output buffer.
  -----------------------------------------------------------------------*/
capi_err_t capi_demuxer_process(capi_t *_pif, capi_stream_data_t *input[], capi_stream_data_t *output[])
{

   capi_demuxer_t *me_ptr = (capi_demuxer_t *)_pif;
   uint32_t            data_len;

   POSAL_ASSERT(_pif);
   POSAL_ASSERT(input[0]);
   POSAL_ASSERT(output[0]);

   if (!me_ptr->is_in_media_fmt_set)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi_demuxer: Input Media format not set yet");
      return CAPI_EFAILED;
   }

   data_len = input[0]->buf_ptr[0].actual_data_len;

   for (uint32_t j = 0; j < me_ptr->num_out_ports; j++)
   {

      if (DATA_PORT_STATE_STARTED != me_ptr->out_port_state_arr[j].state || (NULL == output[j]) ||
          (NULL == output[j]->buf_ptr))
      {
         continue;
      }
      uint32_t out_num_channels = me_ptr->out_port_state_arr[j].operating_out_mf.format.num_channels;

      // For each channel in the output port
      for (uint32_t channel = 0; channel < out_num_channels; channel++)
      {

         uint16_t index = me_ptr->out_port_state_arr[j].input_channel_index[channel];

         // copy all words but EOS (EOS was already copied conditionally in handle_metadata)
         bool_t eos_flag = output[j]->flags.marker_eos;
         output[j]->flags.word |= input[0]->flags.word;
         output[j]->flags.marker_eos = eos_flag;

         output[j]->timestamp = input[0]->timestamp;

         // copy from corresponding index of input
         {
            output[j]->buf_ptr[channel].actual_data_len = memscpy(output[j]->buf_ptr[channel].data_ptr,
                                                                  output[j]->buf_ptr[channel].max_data_len,
                                                                  input[0]->buf_ptr[index].data_ptr,
                                                                  data_len);
         }

      }
   }

   // clear the flags once we propagate to the outputs.
   input[0]->flags.end_of_frame = FALSE;

   return CAPI_EOK;
}
