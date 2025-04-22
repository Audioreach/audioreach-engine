#ifndef PURE_ST_TOPO_H
#define PURE_ST_TOPO_H
/**
 * \file pure_st_topo.h
 * \brief
 *     This file contains utility functions for Pure Signal Triggered topology.
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
typedef struct gu_module_list_t gu_module_list_t;
typedef struct gen_topo_output_port_t gen_topo_output_port_t;

ar_result_t st_topo_process(gen_topo_t *topo_ptr, gu_module_list_t **start_module_list_pptr);

#if defined(__cplusplus)
}
#endif // __cplusplus

#endif /* PURE_ST_TOPO_H */
