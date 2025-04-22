#ifndef PURE_ST_TOPO_I_H
#define PURE_ST_TOPO_I_H
/**
 * \file pure_st_topo_i.h
 * \brief
 *     This file contains utility functions for Pure Signal Triggered topology.
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "ar_defs.h"

// clang-format off

#if defined(__cplusplus)
extern "C" {
#endif // __cplusplus

typedef struct gen_topo_t gen_topo_t;
typedef struct gen_topo_output_port_t gen_topo_output_port_t;

void st_topo_drop_stale_data(gen_topo_t *topo_ptr, gen_topo_output_port_t *out_port_ptr);

#if defined(__cplusplus)
}
#endif // __cplusplus

#endif /* PURE_ST_TOPO_I_H */
