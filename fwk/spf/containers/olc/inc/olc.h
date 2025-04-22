/**
 * \file olc.h
 *  
 * \brief
 *  
 *     header file for the Offload Processing Container
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef OLC_H
#define OLC_H

#include "spf_utils.h"
#include "container_utils.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

/* =======================================================================
 INCLUDE FILES FOR MODULE
 ========================================================================== */

/** @addtogroup olc
 @{ */

/**
 Creates an instance of the offload processing container (OLC)

 @param [in]  init_params_ptr  Pointer to spf_cntr_init_params_t
 @param [out] handle           handle returned to the caller.

 @return
 Success or failure of the instance creation.

 @dependencies
 None.
 */
ar_result_t olc_create(cntr_cmn_init_params_t *init_params_ptr, spf_handle_t **handle, uint32_t cntr_type);

ar_result_t olc_write_done_handler(cu_base_t *base_ptr, uint32_t channel_bit_index);
ar_result_t olc_read_done_handler(cu_base_t *base_ptr,
                                  uint32_t   channel_bit_index,
                                  uint32_t   buf_size_to_deliver,
                                  uint32_t   required_data_buf_size);
ar_result_t olc_get_read_ext_output_buf(cu_base_t *base_ptr, uint32_t channel_bit_index, void *ext_out_data_ptr);
ar_result_t olc_get_ext_out_media_fmt(cu_base_t *base_ptr, uint32_t channel_bit_index, void *mf_out_data_ptr);
ar_result_t olc_media_fmt_event_handler(cu_base_t *base_ptr,
                                        uint32_t   channel_bit_index,
                                        uint8_t *  mf_payload_ptr,
                                        uint32_t   mf_payload_size,
                                        uint32_t   required_buffer_size,
                                        bool_t     is_data_path);

ar_result_t olc_update_path_delay(cu_base_t *cu_ptr, uint32_t master_path_id, void *delay_event_rsp_ptr);

ar_result_t olc_reset_downstream_and_send_internal_eos(cu_base_t *base_ptr, uint32_t channel_bit_index);
ar_result_t olc_handle_peer_port_property_from_satellite_upstream(cu_base_t *                        base_ptr,
                                                                  uint32_t                           channel_bit_index,
                                                                  spf_msg_peer_port_property_info_t *property_ptr);
ar_result_t olc_handle_peer_port_property_from_satellite_downstream(cu_base_t *base_ptr,
                                                                    uint32_t   channel_bit_index,
                                                                    spf_msg_peer_port_property_info_t *property_ptr);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef OLC_H
