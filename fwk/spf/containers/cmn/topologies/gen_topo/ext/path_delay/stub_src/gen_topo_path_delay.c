/**
 * \file gen_topo_path_delay.c
 *  
 * \brief
 *  
 *     Implementation of path delay aspects of topology interface functions.
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "gen_topo.h"
#include "gen_topo_capi.h"

ar_result_t gen_topo_add_path_delay_info(void *   v_topo_ptr,
                                         void *   v_module_ptr,
                                         uint32_t port_id,
                                         void *   v_gu_cmn_port_ptr,
                                         uint32_t path_id)
{
   ar_result_t result = AR_EOK;

   return result;
}

ar_result_t gen_topo_destroy_all_delay_path_per_port(gen_topo_t *            topo_ptr,
                                                     gen_topo_module_t *     module_ptr,
                                                     gen_topo_common_port_t *cmn_port_ptr)
{

   return AR_EOK;
}

/**
 * delay in the path is aggregated & stored in aggregated_delay_ptr
 */
ar_result_t gen_topo_update_path_delays(void *              v_topo_ptr,
                                        uint32_t            path_id,
                                        uint32_t *aggregated_algo_delay_ptr,
                                         uint32_t *aggregated_ext_in_delay_ptr,
                                         uint32_t *aggregated_ext_out_delay_ptr)
{
   ar_result_t result = AR_EOK;

   return result;
}

ar_result_t gen_topo_remove_path_delay_info(void *v_topo_ptr, uint32_t path_id)
{
   ar_result_t result = AR_EOK;

   return result;
}

/**
 * for source sink modules in or out can be NULL
 */
ar_result_t gen_topo_query_module_delay(void *    v_topo_ptr,
                                        void *    v_module_ptr,
                                        void *    v_cmn_in_port_ptr,
                                        void *    v_cmn_out_port_ptr,
                                        uint32_t *delay_us_ptr)
{
   ar_result_t result = AR_EOK;

   *delay_us_ptr = 0;

   return result;
}
