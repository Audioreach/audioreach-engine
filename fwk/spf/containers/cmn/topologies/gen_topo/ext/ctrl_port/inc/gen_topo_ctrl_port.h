#ifndef GEN_TOPO_CTRL_PORT_H
#define GEN_TOPO_CTRL_PORT_H
/**
 * \file gen_topo_ctrl_port.h
 * \brief
 *     This file contains utility functions for control port including ones required to implement interface extension for
 *  IMCL
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "topo_utils.h"
#include "ar_defs.h"

// clang-format off

#if defined(__cplusplus)
extern "C" {
#endif // __cplusplus

typedef struct gen_topo_t gen_topo_t;
typedef struct gen_topo_module_t gen_topo_module_t;
typedef struct gen_topo_ctrl_port_t gen_topo_ctrl_port_t;

ar_result_t gen_topo_set_ctrl_port_properties(gen_topo_module_t *module_ptr,
                                              gen_topo_t *       topo_ptr,
                                              bool_t             is_placeholder_replaced);

ar_result_t gen_topo_handle_incoming_ctrl_intent(void *   topo_ctrl_port_ptr,
                                                 void *   intent_buf,
                                                 uint32_t max_size,
                                                 uint32_t actual_size);

ar_result_t gen_topo_set_ctrl_port_operation(gu_ctrl_port_t *             gu_ctrl_port_ptr,
                                             intf_extn_imcl_port_opcode_t opcode,
                                             POSAL_HEAP_ID                heap_id);

ar_result_t gen_topo_check_set_connected_ctrl_port_operation(uint32_t                  log_id,
                                                             gen_topo_module_t *       this_module_ptr,
                                                             gen_topo_ctrl_port_t *    connected_port_ptr,
                                                             spf_cntr_sub_graph_list_t *spf_sg_list_ptr,
                                                             uint32_t                  sg_ops);

ar_result_t gen_topo_check_set_self_ctrl_port_operation(uint32_t              log_id,
                                                        gen_topo_ctrl_port_t *topo_ctrl_port_ptr,
                                                        uint32_t              sg_ops);

ar_result_t gen_topo_set_ctrl_port_state(void *ctx_ptr, topo_port_state_t state);

#if defined(__cplusplus)
}
#endif // __cplusplus

#endif /* GEN_TOPO_CTRL_PORT_H */
