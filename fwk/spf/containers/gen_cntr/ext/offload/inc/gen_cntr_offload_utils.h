#ifndef GEN_CNTR_OFFLOAD_UTIL_H
#define GEN_CNTR_OFFLOAD_UTIL_H
/**
 * \file gen_cntr_offload_utils.h
 * \brief
 *     This file contains utility functions for offloading
 *  
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

ar_result_t gen_cntr_offload_send_opfs_event_to_rd_client(gen_cntr_t *             me_ptr,
                                                          gen_cntr_ext_out_port_t *ext_out_port_ptr);
ar_result_t gen_cntr_offload_send_opfs_event_to_wr_client(gen_cntr_t *me_ptr);
ar_result_t gen_cntr_offload_process_data_cmd_port_property_cfg(gen_cntr_t *            me_ptr,
                                                                gen_cntr_ext_in_port_t *ext_in_port_ptr);
ar_result_t gen_cntr_offload_propagate_internal_eos_port_property_cfg(gen_cntr_t *             me_ptr,
                                                                      gen_cntr_ext_out_port_t *ext_out_port_ptr);
ar_result_t gen_cntr_offload_process_peer_port_property_param(gen_cntr_t *  me_ptr,
                                                              spf_handle_t *handle_ptr,
                                                              int8_t *      param_data_ptr,
                                                              uint32_t      param_size);
ar_result_t gen_cntr_send_operating_framesize_event_to_wr_shmem_client(gen_cntr_t *            me_ptr,
                                                                       gen_cntr_ext_in_port_t *ext_in_port_ptr);
ar_result_t gen_cntr_offload_pack_write_data(gen_cntr_t *me_ptr, gen_cntr_ext_out_port_t *ext_out_port_ptr);

ar_result_t gen_cntr_offload_reg_evt_wr_sh_mem_ep(gen_cntr_t *       me_ptr,
                                                  gen_cntr_module_t *module_ptr,
                                                  topo_reg_event_t * event_cfg_payload_ptr,
                                                  bool_t             is_register);

ar_result_t gen_cntr_offload_handle_set_cfg_to_wr_sh_mem_ep(gen_cntr_t *        me_ptr,
                                                            gen_cntr_module_t * gen_cntr_module_ptr,
                                                            uint32_t            param_id,
                                                            int8_t *            param_data_ptr,
                                                            uint32_t            param_size,
                                                            spf_cfg_data_type_t cfg_type);

ar_result_t gen_cntr_offload_reg_evt_rd_sh_mem_ep(gen_cntr_t *       me_ptr,
                                                  gen_cntr_module_t *module_ptr,
                                                  topo_reg_event_t * event_cfg_payload_ptr,
                                                  bool_t             is_register);

ar_result_t gen_cntr_offload_handle_set_cfg_to_rd_sh_mem_ep(gen_cntr_t *        me_ptr,
                                                            gen_cntr_module_t * gen_cntr_module_ptr,
                                                            uint32_t            param_id,
                                                            int8_t *            param_data_ptr,
                                                            uint32_t            param_size,
                                                            spf_cfg_data_type_t cfg_type);

ar_result_t gen_cntr_offload_parse_inp_pcm_media_fmt_from_gpr_client(gen_cntr_t *            me_ptr,
                                                                     gen_cntr_ext_in_port_t *ext_in_port_ptr,
                                                                     media_format_t *        media_fmt_ptr,
                                                                     topo_media_fmt_t *      local_media_fmt_ptr);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef GEN_CNTR_OFFLOAD_UTIL_H
