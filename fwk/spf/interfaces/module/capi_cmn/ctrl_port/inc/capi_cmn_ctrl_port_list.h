#ifndef CTRL_PORT_LIST_H
#define CTRL_PORT_LIST_H

/**
 * \file capi_cmn_ctrl_port_list.hh
 *  
 * \brief
 *        utility to maintain list of control port.
 *  any module can use this utility to maintain its control port data base.
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "capi_cmn_imcl_utils.h"

#ifdef __cplusplus
extern "C" {
#endif

// List element. port data structure.
typedef struct ctrl_port_data_t
{
   // port state
   imcl_port_state_t state;
   // port id-intent map
   intf_extn_imcl_id_intent_map_t port_info;
} ctrl_port_data_t;

// Control port list handle.
typedef struct ctrl_port_list_handle_t
{
   // head element of the list.
   void *list_head;
} ctrl_port_list_handle_t;

#define ALIGN_8_BYTES(a) ((a + 7) & (0xFFFFFFF8))

/**
 * This function return the client payload for the given port_data.
 *
 * @param[in] port_data_ptr: 	port data pointer.
 * @return    client payload
 */
inline void *capi_cmn_ctrl_port_list_get_client_payload(ctrl_port_data_t *port_data_ptr)
{
   if (port_data_ptr)
   {
      return ((int8_t *)port_data_ptr) +
             ALIGN_8_BYTES(sizeof(ctrl_port_data_t) + port_data_ptr->port_info.num_intents * sizeof(uint32_t));
   }
   return NULL;
}

/**
 * This function initializes the control port list.
 *
 * @param[in] me_ptr: 	pointer of handle.
 */
void capi_cmn_ctrl_port_list_init(ctrl_port_list_handle_t *me_ptr);

/**
 * This function adds one entry of port data into the list.
 *
 * @param[in] me_ptr: 	 pointer of handle.
 * @param[in] heap_id: 	 heap ID to allocate memory.
 * @param[in] client_payload_size: 	 Size of the client payload. Client can request a custom payload for each control
 * port.
 * @param[in] port_info_ptr:  control port info to be added in the list.
 * @param[out] port_data_pptr: reference of the port data info.
 *
 * @return    CAPI error code
 */
capi_err_t capi_cmn_ctrl_port_list_open_port(ctrl_port_list_handle_t *       me_ptr,
                                             POSAL_HEAP_ID                   heap_id,
                                             uint32_t                        client_payload_size,
                                             intf_extn_imcl_id_intent_map_t *port_info_ptr,
                                             ctrl_port_data_t **             port_data_pptr);
/**
 * This function changes the state of the control port. It removes the entry from the list at CLOSE.
 *
 * @param[in] me_ptr: 	 pointer of handle.
 * @param[in] port_id:   control port id.
 * @param[in] state:     control port state.
 *
 * @return    CAPI error code
 */
capi_err_t capi_cmn_ctrl_port_list_set_state(ctrl_port_list_handle_t *me_ptr,
                                             uint32_t                 port_id,
                                             imcl_port_state_t        state);

/**
 * This function looks for the control port info structure with the given intent_id
 * This function should be called in a loop to get all the port_data structures associated with the given intent ID.
 * -Initially prev_port_id should be set as zero, the function returns the first port_data structure for the given
 * intent ID. -next time the prev_port_id should be set as the control port id of the previously returned port_data,
 * function will return the next port_data. -if there is no more port_data for the given intent_id then function returns
 * NULL.
 *
 *
 * @param[in]  me_ptr: 		 	    pointer of handle.
 * @param[in]  intent_id: 	    	intent id.
 * @param[in]  prev_port_id:    	previously searched control port id for the given intent id.
 * @param[out] port_data_pptr:   reference of the searched port data info.
 *
 * @return    CAPI error code
 */
void capi_cmn_ctrl_port_list_get_next_port_data(ctrl_port_list_handle_t *me_ptr,
                                                uint32_t                 intent_id,
                                                uint32_t                 prev_port_id,
                                                ctrl_port_data_t **      port_data_pptr);

/**
 * This function looks for the control port info structure the given port_id
 *
 * @param[in]  me_ptr: 		 	    pointer of handle.
 * @param[in]  port_id: 	    	port id.
 * @param[out] port_data_pptr:   reference of the searched port data info.
 *
 * @return    CAPI error code
 */
void capi_cmn_ctrl_port_list_get_port_data(ctrl_port_list_handle_t *me_ptr,
                                           uint32_t                 port_id,
                                           ctrl_port_data_t **      port_data_pptr);

/**
 * This function handles the #INTF_EXTN_PARAM_ID_IMCL_PORT_OPERATION setparam
 * no need to call port_open/port_start/port_connect/port_close apis separately.
 *
 * Limitation: client payload size can not be different for each port id.
 *
 * @param[in]  me_ptr: 		 	    	pointer of handle.
 * @param[in]  param_ptr: 	    		set param_ptr.
 * @param[in]  heap_id: 	    		Heap ID.
 * @param[in]  client_payload_size: 	Size of the client payload. Client can request a custom payload for each control
 * port.
 * @param[in]  num_intent:      		number of intents supported by module.
 * @param[out] supported_intent_id_arr: array of supported intent ids by module
 *
 * @return    CAPI error code
 */
capi_err_t capi_cmn_ctrl_port_operation_handler(ctrl_port_list_handle_t *me_ptr,
                                                capi_buf_t *             param_ptr,
                                                POSAL_HEAP_ID            heap_id,
                                                uint32_t                 client_payload_size,
                                                uint32_t                 num_intent,
                                                uint32_t *               supported_intent_id_arr);

/**
 * This function frees all the elements of the list.
 *
 * @param[in] me_ptr: 		pointer of handle.
 */
void capi_cmn_ctrl_port_list_deinit(ctrl_port_list_handle_t *me_ptr);

#ifdef __cplusplus
}
#endif

#endif // CTRL_PORT_LIST_H
