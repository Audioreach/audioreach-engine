/**
 * \file sgm_cmd_parser_utils.h
 * \brief
 *     This file contains  definitions and declarations for the OLC command parser utilities
 *  Satellite Graph Management.
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef OLC_SGM_PARSER_UTILS_I_H
#define OLC_SGM_PARSER_UTILS_I_H

#include "olc_cmn_utils.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus//

#include "sgm_i.h"

/* =======================================================================
OLC SGM Macros
========================================================================== */

/* =======================================================================
OLC SGM Structure Definitions
========================================================================== */

/* =======================================================================
OLC SGM Function Declarations
========================================================================== */
/**--------------------------- olc_sgm_cmd_parser_utilities --------------------*/

ar_result_t sgm_client_command_fill_header_cfg(uint8_t *           payload_ptr,
                                               uint32_t *          header_payload_size_ptr,
                                               sgm_shmem_handle_t *shmem_node_ptr,
                                               uint32_t            cmd_payload_size,
                                               bool_t              is_payload_inband);

ar_result_t sgm_open_get_sub_graph_payload_size(apm_sub_graph_cfg_t **sg_cfg_list_pptr,
                                                uint32_t              num_sub_graphs,
                                                uint32_t *            sub_graph_payload_size_ptr,
                                                spgm_id_info_t *      sgm_id_ptr);

ar_result_t sgm_open_get_container_payload_size(apm_container_cfg_t **cont_cfg_list_pptr,
                                                uint32_t              num_container,
                                                uint32_t *            container_payload_size_ptr,
                                                spgm_id_info_t *      sgm_id_ptr,
                                                uint32_t *            get_sat_pd_ptr);

ar_result_t sgm_open_get_module_list_payload_size(apm_modules_list_t **module_cfg_list_pptr,
                                                  uint32_t             num_module_list,
                                                  uint32_t *           module_list_payload_size_ptr,
                                                  spgm_id_info_t *     sgm_id_ptr);

ar_result_t sgm_open_get_module_prop_payload_size(apm_module_prop_cfg_t **module_prop_list_pptr,
                                                  uint32_t                num_module_prop_list,
                                                  uint32_t *              module_prop_payload_size_ptr,
                                                  spgm_id_info_t *        sgm_id_ptr);

ar_result_t sgm_open_get_module_conn_payload_size(apm_module_conn_cfg_t **module_conn_list_pptr,
                                                  uint32_t                num_module_conn_list,
                                                  uint32_t *              module_conn_payload_size_ptr,
                                                  spgm_id_info_t *        sgm_id_ptr);

ar_result_t sgm_open_get_imcl_peer_info_payload_size(apm_imcl_peer_domain_info_t **imcl_peer_domain_info_pptr,
                                                     uint32_t                      num_offloaded_peers,
                                                     uint32_t *                    imcl_peer_cfg_payload_size_ptr,
                                                     spgm_id_info_t *              sgm_id_ptr);

ar_result_t sgm_open_get_ctrl_link_cfg_payload_size(apm_module_ctrl_link_cfg_t **ctrl_link_cfg_list_pptr,
                                                    uint32_t                     num_ctrl_link_cfg_list,
                                                    uint32_t *                   ctrl_link_cfg_payload_size_ptr,
                                                    spgm_id_info_t *             sgm_id_ptr);

ar_result_t sgm_open_get_param_data_payload_size(void **         param_data_cfg_list_pptr,
                                                 uint32_t        num_param_id_cfg,
                                                 uint32_t *      param_data_payload_size_ptr,
                                                 spgm_id_info_t *sgm_id_ptr);

ar_result_t sgm_open_fill_sub_graph_cfg(apm_sub_graph_cfg_t **sg_cfg_list_pptr,
                                        uint32_t              num_sub_graphs,
                                        uint8_t *             payload_ptr,
                                        uint32_t *            sub_graph_payload_size_ptr);

ar_result_t sgm_open_fill_container_cfg(apm_container_cfg_t **cont_cfg_list_pptr,
                                        uint32_t              num_container,
                                        uint8_t *             payload_ptr,
                                        uint32_t *            container_payload_size_ptr);

ar_result_t sgm_open_fill_module_list_cfg(apm_modules_list_t **module_cfg_list_pptr,
                                          uint32_t             num_module_list,
                                          uint8_t *            payload_ptr,
                                          uint32_t *           module_list_payload_size_ptr);

ar_result_t sgm_open_fill_module_prop_cfg(apm_module_prop_cfg_t **module_prop_list_pptr,
                                          uint32_t                num_module_prop_list,
                                          uint8_t *               payload_ptr,
                                          uint32_t *              module_prop_payload_size_ptr);

ar_result_t sgm_open_fill_module_conn_cfg(apm_module_conn_cfg_t **module_conn_list_pptr,
                                          uint32_t                num_module_conn_list,
                                          uint8_t *               payload_ptr,
                                          uint32_t *              module_conn_payload_size_ptr);

ar_result_t sgm_open_fill_imcl_peer_cfg(apm_imcl_peer_domain_info_t **imcl_peer_domain_info_pptr,
                                        uint32_t                      num_offloaded_peers,
                                        uint8_t *                     payload_ptr,
                                        uint32_t *                    imcl_peer_cfg_payload_size_ptr);

ar_result_t sgm_open_fill_ctrl_link_cfg(apm_module_ctrl_link_cfg_t **ctrl_link_cfg_list_pptr,
                                        uint32_t                     num_ctrl_link_cfg_list,
                                        uint8_t *                    payload_ptr,
                                        uint32_t *                   ctrl_link_cfg_payload_size_ptr);

ar_result_t sgm_open_fill_param_data_cfg(spgm_info_t *spgm_ptr,
                                         void **      param_data_cfg_list_pptr,
                                         uint32_t     num_param_id_cfg,
                                         uint8_t *    payload_ptr,
                                         uint32_t *   param_data_payload_size_ptr);

ar_result_t sgm_get_graph_mgmt_client_payload_size(spgm_info_t *             spgm_ptr,
                                                   spf_msg_cmd_graph_mgmt_t *gmc_apm_gmgmt_cmd_ptr,
                                                   uint32_t *                graph_mgmt_cmd_payload_size_ptr);

ar_result_t sgm_get_graph_close_client_payload_size(spgm_info_t *             spgm_ptr,
                                                    spf_msg_cmd_graph_mgmt_t *gmc_apm_close_cmd_ptr,
                                                    uint32_t *                graph_close_cmd_payload_size_ptr);

ar_result_t sgm_gmgmt_fill_sub_graph_list_info(uint32_t *sg_id_list_info,
                                               uint32_t  num_sub_graphs,
                                               uint8_t * payload_ptr,
                                               uint32_t *sub_graph_payload_size_ptr);

ar_result_t sgm_get_graph_set_get_cfg_client_payload_size(spgm_info_t *                 spgm_ptr,
                                                          spf_msg_cmd_param_data_cfg_t *gmc_apm_gmgmt_cmd_ptr,
                                                          uint32_t *gm_set_get_cfg_payload_size_ptr);

ar_result_t sgm_graph_set_get_cfg_fill_client_payload(spgm_info_t *                 spgm_ptr,
                                                      spf_msg_cmd_param_data_cfg_t *gmc_apm_gmgmt_set_get_cfg_ptr,
                                                      uint8_t *                     client_set_get_cfg_ptr,
                                                      uint32_t                      gm_set_get_cfg_payload_size);

ar_result_t sgm_create_graph_mgmt_command_client_payload(spgm_info_t *             spgm_ptr,
                                                         spf_msg_cmd_graph_mgmt_t *gmc_apm_gmgmt_cmd_ptr,
                                                         uint8_t *                 client_graph_mgmt_payload_ptr,
                                                         uint32_t                  graph_mgmt_cmd_payload_size);

ar_result_t sgm_create_graph_close_command_client_payload(spgm_info_t *             spgm_ptr,
                                                          spf_msg_cmd_graph_mgmt_t *gmc_apm_close_cmd_ptr,
                                                          uint8_t *                 client_graph_mgmt_payload_ptr,
                                                          uint32_t                  graph_close_cmd_payload_size);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef OLC_SGM_PARSER_UTILS_I_H
