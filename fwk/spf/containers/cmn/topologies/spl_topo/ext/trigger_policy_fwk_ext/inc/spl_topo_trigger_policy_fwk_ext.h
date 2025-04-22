
#ifndef SPL_TOPO_TRIGGER_POLICY_FWK_H
#define SPL_TOPO_TRIGGER_POLICY_FWK_H
/**
 * \file spl_topo_trigger_policy_fwk_ext.h
 * \brief
 *     This file contains function definitions for FWK_EXTN_TRIGGER_POLICY

 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "topo_utils.h"
#include "ar_defs.h"

#if defined(__cplusplus)
extern "C" {
#endif // __cplusplus

typedef struct spl_topo_t             spl_topo_t;
typedef struct spl_topo_module_t      spl_topo_module_t;
typedef struct spl_topo_input_port_t  spl_topo_input_port_t;
typedef struct spl_topo_output_port_t spl_topo_output_port_t;

bool_t spl_topo_in_port_is_trigger_absent(void *ctx_topo_ptr, void *ctx_in_port_ptr);
bool_t      spl_topo_in_port_trigger_blocked(spl_topo_t *topo_ptr, spl_topo_input_port_t *in_port_ptr);

bool_t spl_topo_out_port_is_trigger_absent(void *ctx_topo_ptr, void *ctx_out_port_ptr);
bool_t spl_topo_out_port_trigger_blocked(spl_topo_t *topo_ptr, spl_topo_output_port_t *out_port_ptr);
bool_t spl_topo_int_in_port_is_trigger_present(void *  ctx_topo_ptr,
                                                 void *  ctx_in_port_ptr,
                                                 bool_t *is_ext_trigger_not_satisfied_ptr);
bool_t spl_topo_in_port_is_trigger_present(void *  ctx_topo_ptr,
                                             void *  ctx_in_port_ptr,
                                             bool_t *is_ext_trigger_not_satisfied_ptr,
                                             bool_t is_internal_check);
bool_t spl_topo_out_port_is_trigger_present(void *  ctx_topo_ptr,
                                              void *  ctx_out_port_ptr,
                                              bool_t *is_ext_trigger_not_satisfied_ptr);

uint32_t spl_topo_in_port_needs_data(spl_topo_t *topo_ptr,
		spl_topo_input_port_t *in_port_ptr);
uint32_t spl_topo_out_port_needs_trigger(spl_topo_t *topo_ptr,
		spl_topo_output_port_t *out_port_ptr);


#if defined(__cplusplus)
}
#endif // __cplusplus

#endif /* SPL_TOPO_TRIGGER_POLICY_FWK_H */
