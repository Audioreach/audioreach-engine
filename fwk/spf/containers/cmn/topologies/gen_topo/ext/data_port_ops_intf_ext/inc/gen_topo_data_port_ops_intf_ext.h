#ifndef GEN_TOPO_DATA_PORT_OPS_H
#define GEN_TOPO_DATA_PORT_OPS_H
/**
 * \file gen_topo_data_port_ops_intf_ext.h
 * \brief
 *     This file contains utility functions FWK_EXTN_PARAM_ID_CONTAINER_PROC_DELAY
 *  
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "topo_utils.h"
#include "ar_defs.h"

#include "capi_intf_extn_data_port_operation.h"

// clang-format off

#if defined(__cplusplus)
extern "C" {
#endif // __cplusplus

typedef struct gen_topo_t gen_topo_t;
typedef struct gen_topo_module_t gen_topo_module_t;

ar_result_t gen_topo_intf_extn_data_ports_hdl_at_init(gen_topo_t *topo_ptr, gen_topo_module_t *module_ptr);


ar_result_t gen_topo_capi_set_data_port_op(gen_topo_module_t *           module_ptr,
                                             intf_extn_data_port_opcode_t  opcode,
                                             intf_extn_data_port_opcode_t *last_issued_opcode_ptr,
                                             bool_t                        is_input,
                                             uint32_t                      port_index,
                                             uint32_t                      port_id);

ar_result_t gen_topo_capi_set_data_port_op_from_state(gen_topo_module_t *           module_ptr,
                                                       topo_port_state_t             downgraded_state,
                                                       intf_extn_data_port_opcode_t *last_issued_opcode_ptr,
                                                       bool_t                        is_input,
                                                       uint32_t                      port_index,
                                                       uint32_t                      port_id);



ar_result_t gen_topo_capi_set_data_port_op_from_sg_ops(gen_topo_module_t *           module_ptr,
                                                       uint32_t                      sg_ops,
                                                       intf_extn_data_port_opcode_t *last_issued_opcode_ptr,
                                                       bool_t                        is_input,
                                                       uint32_t                      port_index,
                                                       uint32_t                      port_id);


#if defined(__cplusplus)
}
#endif // __cplusplus

#endif /* GEN_TOPO_DATA_PORT_OPS_H */
