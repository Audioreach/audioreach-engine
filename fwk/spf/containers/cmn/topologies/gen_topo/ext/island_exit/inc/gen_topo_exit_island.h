#ifndef GEN_TOPO_EXIT_ISLAND_H
#define GEN_TOPO_EXIT_ISLAND_H
/**
 * \file gen_topo_exit_island.h
 * \brief
 *     This file contains utility function for exiting island before accessing certain libs which are marked as non-island
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

void gen_topo_vote_against_lpi_if_md_lib_in_nlpi(gen_topo_t *topo_ptr);

void gen_topo_exit_lpi_temporarily_if_md_lib_in_nlpi(gen_topo_t *topo_ptr);

capi_err_t gen_topo_capi_exit_island_metadata_create(void *                 context_ptr,
                                                     module_cmn_md_list_t **md_list_pptr,
                                                     uint32_t               size,
                                                     capi_heap_id_t         c_heap_id,
                                                     bool_t                 is_out_band,
                                                     module_cmn_md_t **     md_pptr);

capi_err_t gen_topo_capi_exit_island_metadata_create_with_tracking(void *                    context_ptr,
                                                                   module_cmn_md_list_t **   md_list_pptr,
                                                                   uint32_t                  size,
                                                                   capi_heap_id_t            heap_id,
                                                                   uint32_t                  metadata_id,
                                                                   module_cmn_md_flags_t     flags,
                                                                   module_cmn_md_tracking_t *tracking_info_ptr,
                                                                   module_cmn_md_t **        md_pptr);



capi_err_t gen_topo_capi_exit_island_metadata_clone(void *                 context_ptr,
                                                    module_cmn_md_t *      md_ptr,
                                                    module_cmn_md_list_t **out_md_list_pptr,
                                                    capi_heap_id_t         c_heap_id);


capi_err_t gen_topo_capi_exit_island_metadata_destroy(void *                 context_ptr,
                                                      module_cmn_md_list_t * md_list_ptr,
                                                      bool_t                 is_dropped,
                                                      module_cmn_md_list_t **head_pptr);

capi_err_t gen_topo_capi_exit_island_metadata_propagate(void *                      context_ptr,
                                                        capi_stream_data_v2_t *     input_stream_ptr,
                                                        capi_stream_data_v2_t *     output_stream_ptr,
                                                        module_cmn_md_list_t **     internal_md_list_pptr,
                                                        uint32_t                    algo_delay_us,
                                                        intf_extn_md_propagation_t *input_md_info_ptr,
                                                        intf_extn_md_propagation_t *output_md_info_ptr);

capi_err_t gen_topo_capi_exit_island_metadata_modify_at_data_flow_start(void *                 context_ptr,
                                                                        module_cmn_md_list_t * md_node_ptr,
                                                                        module_cmn_md_list_t **head_pptr);

#if defined(__cplusplus)
}
#endif // __cplusplus

#endif /* GEN_TOPO_EXIT_ISLAND_H */
