#ifndef GEN_TOPO_PATH_DELAY_H
#define GEN_TOPO_PATH_DELAY_H
/**
 * \file gen_topo_path_delay.h
 * \brief
 *     This file contains utility functions for handling path delay
 *  
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "topo_utils.h"
#include "ar_defs.h"
#include "apm_cntr_path_delay_if.h"

// clang-format off

#if defined(__cplusplus)
extern "C" {
#endif // __cplusplus

typedef struct gen_topo_t gen_topo_t;
typedef struct gen_topo_module_t gen_topo_module_t;
typedef struct gen_topo_common_port_t gen_topo_common_port_t;
typedef struct gu_ext_out_port_t gu_ext_out_port_t;


ar_result_t gen_topo_destroy_all_delay_path_per_port(gen_topo_t *            topo_ptr,
                                                     gen_topo_module_t *     module_ptr,
                                                     gen_topo_common_port_t *cmn_port_ptr);
ar_result_t gen_topo_add_path_delay_info(void *   v_topo_ptr,
                                         void *   v_module_ptr,
                                         uint32_t port_id,
                                         void *   v_gu_cmn_port_ptr,
                                         uint32_t path_id);
ar_result_t gen_topo_remove_path_delay_info(void *topo_ptr, uint32_t path_id);
ar_result_t gen_topo_update_path_delays(void *              v_topo_ptr,
                                        uint32_t            path_id,
                                        uint32_t *aggregated_algo_delay_ptr,
                                         uint32_t *aggregated_ext_in_delay_ptr,
                                         uint32_t *aggregated_ext_out_delay_ptr);
ar_result_t gen_topo_query_module_delay(void *    v_topo_ptr,
                                        void *    v_module_ptr,
                                        void *    v_cmn_in_port_ptr,
                                        void *    v_cmn_out_port_ptr,
                                        uint32_t *delay_us_ptr);


#if defined(__cplusplus)
}
#endif // __cplusplus

#endif /* GEN_TOPO_PATH_DELAY_H */
