/**
 * \file capi_cmn_imcl_utils.h
 *  
 * \brief
 *        Common utility function declaration for Inter Module Communication utils
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef _CAPI_CMN_IMCL_UTILS_H_
#define _CAPI_CMN_IMCL_UTILS_H_

/**------------------------------------------------------------------------
 * Include files
 * -----------------------------------------------------------------------*/

#include "capi_intf_extn_imcl.h"
#ifdef __cplusplus
extern "C" {
#endif
/**------------------------------------------------------------------------
 * Type Definitions
 * -----------------------------------------------------------------------*/

/**< Header used for param payloads in the incoming/outgoing intents.
 *
 *   Each intent msg payload, can have mutliple params packed in it.
 *
 *   size of intent buffer = sizeof(intf_extn_param_id_imcl_incoming_data_t) + // intent header
 *                           sizeof(imc_param_header_t) + // param 1 header
 *                           param1.actual_data_len     +
 *                           sizeof(imc_param_header_t) + // param 2 header
 *                           param2.actual_data_len
 */
typedef struct imc_param_header_t
{
   // specific purpose understandable to the IMCL peers only
   uint32_t opcode;

   // Size (in bytes) for the payload specific to the intent.
   uint32_t actual_data_len;
} imc_param_header_t;

/* clang-format off */

typedef enum imcl_port_state_t
{
   CTRL_PORT_CLOSE = 0,
   /**< Indicates that the control port is closed. And module can de allocate the resources for this port. */

   CTRL_PORT_OPEN = 1,
   /**< Indicates that the control port is open.
        Set to this when capi receives the port open.
        Doesn't necessarily mean that the connected port is opened. */

   CTRL_PORT_PEER_CONNECTED = 2,
   /**< Indicates that the peer is ready to receive the messages.
        Module can send messages only in this state.  */

   CTRL_PORT_PEER_DISCONNECTED = 3,
   /**< Indicates that peer module cannot handle any incoming messages on this control link.
        When the module get this state, it must stop sending the control messages further on this ctrl port. */

   PORT_STATE_INVALID = 0xFFFFFFFFL
   /**< Port state is not valid. */

}imcl_port_state_t;

/* clang-format on */

/**------------------------------------------------------------------------
 * Function Declarations
 * -----------------------------------------------------------------------*/

/**
 * This function is for registering for recurring buffers to be
 * used for sending control data by a module
 *
 * @param[in] event_cb_info_ptr: Module's CAPI Event CB info ptr
 * @param[in] ctrl_port_id:  Module control port ID
 * @param[in] buf_size:  Buffer size in bytes
 * @param[in] num_bufs:  Number of buffers requested
 *
 * @return    CAPI error code
 */
capi_err_t capi_cmn_imcl_register_for_recurring_bufs(capi_event_callback_info_t *event_cb_info_ptr,
                                                     uint32_t                    ctrl_port_id,
                                                     uint32_t                    buf_size,
                                                     uint32_t                    num_bufs);
/**
 * This function is for requesting a recurring buffer from the
 *  fwk. This util returns a pointer to the capi_buf_t whose
 *  data_ptr will point to the memory to be used by the module.
 *
 * @param[in] event_cb_info_ptr: Module's CAPI Event CB info ptr
 * @param[in] ctrl_port_id:  Module control port ID
 * @param[out] rec_buf_ptr:  Pointer to returned recurring
 *       buffer
 *
 * @return    CAPI error code
 */
capi_err_t capi_cmn_imcl_get_recurring_buf(capi_event_callback_info_t *event_cb_info_ptr,
                                           uint32_t                    ctrl_port_id,
                                           capi_buf_t *                rec_buf_ptr);

/**
 * This function is for requesting a one time buffer from the
 *  fwk. This util returns a pointer to the capi_buf_t whose
 *  data_ptr will point to the memory to be used by the module.
 *
 * @param[in] event_cb_info_ptr: Module's CAPI Event CB info ptr
 * @param[in] ctrl_port_id:  Module control port ID
 * @param[in] req_buf_size:  Required buffer size in bytes
 * @param[out] ot_buf_ptr:  Pointer to returned one-time
 *       buffer
 *
 * @return    CAPI error code
 */
capi_err_t capi_cmn_imcl_get_one_time_buf(capi_event_callback_info_t *event_cb_info_ptr,
                                          uint32_t                    ctrl_port_id,
                                          uint32_t                    req_buf_size,
                                          capi_buf_t *                ot_buf_ptr);

/**
 * This function is for sending the control data buffer to the
 * connected peer module.
 *
 * @param[in] event_cb_info_ptr: Module's CAPI Event CB info ptr
 * @param[in] ctrl_data_buf_ptr:  Pointer to control data CAPI
 *       buffer
 * @param[in] ctrl_port_id:  Module control port ID
 *
 * @return    CAPI error code
 */
capi_err_t capi_cmn_imcl_send_to_peer(capi_event_callback_info_t *event_cb_info_ptr,
                                      capi_buf_t *                ctrl_data_buf_ptr,
                                      uint32_t                    ctrl_port_id,
                                      imcl_outgoing_data_flag_t   flags);

/**
 * This function is for returning the control data buffer to the
 *  fwk because of some error leading to not sending to peer.
 *
 * @param[in] event_cb_info_ptr: Module's CAPI Event CB info ptr
 * @param[in] ctrl_data_buf_ptr:  Pointer to control data CAPI
 *       buffer
 * @param[in] ctrl_port_id:  Module control port ID
 *
 * @return    CAPI error code
 */
capi_err_t capi_cmn_imcl_return_to_fwk(capi_event_callback_info_t *event_cb_info_ptr,
                                       capi_buf_t *                ctrl_data_buf_ptr,
                                       uint32_t                    ctrl_port_id);

capi_err_t capi_cmn_imcl_data_send(capi_event_callback_info_t *event_cb_info_ptr,
                                          capi_buf_t *                ctrl_data_buf_ptr,
                                          uint32_t                    ctrl_port_id,
                                          imcl_outgoing_data_flag_t   flags);

#ifdef __cplusplus
}
#endif
#endif /** _CAPI_CMN_IMCL_UTILS_H_ */
