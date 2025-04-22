#ifndef GEN_CNTR_METADATA_H
#define GEN_CNTR_METADATA_H
/**
 * \file gen_cntr_metadata.h
 * \brief
 *     This file contains function definitions that handle metadata.
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

ar_result_t gen_cntr_clear_eos(gen_topo_t *         topo_ptr,
                               void *               ext_inp_ref,
                               uint32_t             ext_inp_id,
                               module_cmn_md_eos_t *eos_metadata_ptr);

ar_result_t gen_cntr_process_eos_md_from_peer_cntr_util_( gen_cntr_t *            me_ptr,
														gen_cntr_ext_in_port_t *ext_in_port_ptr,
														module_cmn_md_list_t ** md_list_head_pptr,
														module_cmn_md_list_t * eos_md_list_node_ptr);

ar_result_t gen_cntr_create_send_eos_md(gen_cntr_t *me_ptr, gen_cntr_ext_out_port_t *ext_out_port_ptr);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef GEN_CNTR_METADATA_H
