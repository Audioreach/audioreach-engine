#ifndef CU_CTRL_PORT_I_H
#define CU_CTRL_PORT_I_H

/**
 * \file cu_ctrl_port_i.h
 *
 * \brief
 *
 *     Common container framework code.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "cu_ctrl_port.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

ar_result_t cu_init_ctrl_port_buf_q(cu_base_t *me_ptr, gu_ctrl_port_t *gu_ctrl_port_ptr);
void cu_deinit_ctrl_port_buf_q(cu_base_t *me_ptr, gu_ctrl_port_t *gu_ctrl_port_ptr);

ar_result_t cu_ctrl_port_process_inter_proc_outgoing_buf(cu_base_t *               me_ptr,
                                                           gu_ext_ctrl_port_t *      ext_port_ptr,
                                                           capi_buf_t *           buf_ptr,
                                                           imcl_outgoing_data_flag_t flags,
                                                           uint32_t                  src_miid,
                                                           uint32_t                  peer_port_id);

ar_result_t cu_handle_imcl_event_non_island(cu_base_t *cu_ptr, gu_module_t *module_ptr, capi_event_info_t *event_info_ptr);

ar_result_t cu_create_ctrl_bufs_util(cu_base_t *         cu_ptr,
					 gu_ctrl_port_t *gu_ctrl_port_ptr,
					 uint32_t            num_num_bufs);


static inline spf_handle_t* cu_ctrl_port_get_dst_handle(cu_base_t *me_ptr, gu_ctrl_port_t *gu_ctrl_port_ptr)
{
  return (gu_ctrl_port_ptr->ext_ctrl_port_ptr)
      ? gu_ctrl_port_ptr->ext_ctrl_port_ptr->peer_handle.spf_handle_ptr
	  : &me_ptr->cntr_cmn_imcl_handle;
}

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef CU_CTRL_PORT_I_H
