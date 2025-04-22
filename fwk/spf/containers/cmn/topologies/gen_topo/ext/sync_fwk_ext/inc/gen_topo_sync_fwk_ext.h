#ifndef GEN_TOPO_SYNC_FWK_H
#define GEN_TOPO_SYNC_FWK_H
/**
 * // clang-format off
 * \file gen_topo_sync_fwk_ext.h
 * \brief
 *  This file contains utility functions for FWK_EXTN_SYNC
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */
// clang-format on

#include "ar_defs.h"
#include "topo_utils.h"

#if defined(__cplugenus)
extern "C" {
#endif // __cplugenus

typedef struct gen_topo_t gen_topo_t;
typedef struct gen_topo_module_t gen_topo_module_t;
typedef struct gen_topo_input_port_t gen_topo_input_port_t;

//to notify sync module that data-flow is started on one of the input port of module_ptr and sync may be required.
ar_result_t gen_topo_fwk_extn_sync_will_start(gen_topo_t *me_ptr, gen_topo_module_t *module_ptr);

void gen_topo_fwk_extn_sync_propagate_threshold_state(gen_topo_t *me_ptr);

bool_t gen_topo_fwk_extn_does_sync_module_exist_downstream_util_(gen_topo_t *me_ptr, gen_topo_input_port_t *in_port_ptr);

#if defined(__cplugenus)
}
#endif // __cplugenus

#endif /* GEN_TOPO_SYNC_FWK_H */
