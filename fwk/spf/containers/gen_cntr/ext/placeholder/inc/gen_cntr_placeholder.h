#ifndef GEN_CNTR_PLACEHOLDER_H
#define GEN_CNTR_PLACEHOLDER_H
/**
 * \file gen_cntr_placeholder.h
 * \brief
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "gen_cntr.h"
#include "gen_cntr_cmn_utils.h"
#include "gen_topo.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

typedef struct gen_cntr_t               gen_cntr_t;
typedef struct gen_cntr_timestamp_t     gen_cntr_timestamp_t;
typedef struct gen_cntr_ext_out_port_t  gen_cntr_ext_out_port_t;
typedef struct gen_cntr_ext_in_port_t   gen_cntr_ext_in_port_t;
typedef struct gen_cntr_ext_ctrl_port_t gen_cntr_ext_ctrl_port_t;
typedef struct gen_cntr_circ_buf_list_t gen_cntr_circ_buf_list_t;
typedef struct gen_cntr_module_t        gen_cntr_module_t;

ar_result_t gen_cntr_placeholder_check_if_real_id_rcvd_at_prepare(cu_base_t *               base_ptr,
                                                                  spf_msg_cmd_graph_mgmt_t *cmd_gmgmt_ptr);
ar_result_t gen_cntr_create_placeholder_module(gen_cntr_t *           me_ptr,
                                               gen_topo_module_t *    module_ptr,
                                               gen_topo_graph_init_t *graph_init_ptr);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef GEN_CNTR_PLACEHOLDER_H
