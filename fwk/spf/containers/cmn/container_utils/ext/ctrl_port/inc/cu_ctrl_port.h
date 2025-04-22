#ifndef CU_CTRL_PORT_H
#define CU_CTRL_PORT_H

/**
 * \file cu_ctrl_port.h
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

#include "posal.h"
#include "topo_utils.h"
#include "spf_list_utils.h"
#include "capi_intf_extn_imcl.h"
#include "graph_utils.h"
#include "shared_lib_api.h"
#include "cntr_cntr_offload_if.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

typedef struct cu_base_t         cu_base_t;
typedef struct cu_ext_out_port_t cu_ext_out_port_t;
typedef struct cu_ext_in_port_t  cu_ext_in_port_t;

//#define CTRL_LINK_DEBUG

#define INTENT_BUFFER_OFFSET                                                                                           \
   ((sizeof(spf_msg_header_t) - sizeof(uint64_t)) + (sizeof(spf_msg_ctrl_port_msg_t) - sizeof(uint64_t)) +             \
    sizeof(intf_extn_param_id_imcl_incoming_data_t))

// clang-format off
ar_result_t cu_init_ext_ctrl_port(cu_base_t *me_ptr, gu_ext_ctrl_port_t *gu_ext_ctrl_port_ptr, uint32_t ctrl_port_queue_offset);
ar_result_t cu_deinit_ext_ctrl_port(cu_base_t *me_ptr, gu_ext_ctrl_port_t *gu_ext_ctrl_port_ptr);
ar_result_t cu_connect_ext_ctrl_port(cu_base_t *me_ptr, gu_ext_ctrl_port_t *gu_ext_ctrl_port_ptr);
ar_result_t cu_operate_on_ext_ctrl_port(cu_base_t *          me_ptr,
                                        uint32_t             sg_ops,
                                        gu_ext_ctrl_port_t **ext_ctrl_port_pptr,
                                        bool_t               is_self_sg);


// never call this directly. call cu_poll_and_process_ctrl_msgs.
ar_result_t cu_poll_and_process_ext_ctrl_msgs_util_(cu_base_t *me_ptr, uint32_t set_ch_bitmask);
ar_result_t cu_poll_and_process_int_ctrl_msgs_util_(cu_base_t *me_ptr, uint32_t set_ch_bitmask);

ar_result_t cu_handle_ctrl_port_trigger_cmd(cu_base_t *me_ptr);

ar_result_t cu_set_downgraded_state_on_ctrl_port(cu_base_t *       me_ptr,
                                                 gu_ctrl_port_t *  ctrl_port_ptr,
                                                 topo_port_state_t downgraded_state);

ar_result_t cu_handle_inter_proc_triggered_imcl(cu_base_t *me_ptr, gpr_packet_t *packet_ptr);

ar_result_t cu_handle_imcl_event(cu_base_t *cu_ptr, gu_module_t *module_ptr, capi_event_info_t *event_info_ptr);

ar_result_t cu_send_ctrl_polling_spf_msg(spf_msg_t *msg_ptr, spf_handle_t *dst_handle_ptr);

static inline ar_result_t cu_send_ctrl_trigger_spf_msg(spf_msg_t *msg_ptr, spf_handle_t *dst_handle_ptr)
{
   return spf_msg_send_cmd(msg_ptr, dst_handle_ptr);
}

ar_result_t cu_create_cmn_int_ctrl_port_queue(cu_base_t *cu_ptr, uint32_t ctrl_port_queue_offset);
ar_result_t cu_destroy_cmn_int_port_queue(cu_base_t *cu_ptr);
ar_result_t cu_operate_on_int_ctrl_port(cu_base_t *      me_ptr,
                                        uint32_t         sg_ops,
                                        gu_ctrl_port_t **gu_ctrl_port_ptr_ptr,
                                        bool_t           is_self_sg);
ar_result_t cu_ext_ctrl_port_poll_and_process_ctrl_msgs(cu_base_t *me_ptr,  gu_ext_ctrl_port_t *gu_ext_ctrl_port_ptr);


ar_result_t cu_check_and_recreate_ctrl_port_buffers(cu_base_t *me_ptr, gu_ctrl_port_t *gu_ctrl_port_ptr);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef CU_CTRL_PORT_H
