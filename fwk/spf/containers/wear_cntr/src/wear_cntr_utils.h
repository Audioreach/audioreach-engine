#ifndef WCNTR_UTIL_H
#define WCNTR_UTIL_H
/**
 * \file wear_cntr_utils.h
 * \brief
 *     This file contains utility functions for WCNTR
 *
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "wear_cntr.h"
#include "wear_cntr_i.h"
#include "apm_container_api.h"
#include "wc_topo.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

typedef struct wcntr_base_t         wcntr_base_t;
typedef struct wcntr_t               wcntr_t;
typedef struct wcntr_timestamp_t     wcntr_timestamp_t;
typedef struct wcntr_module_t        wcntr_module_t;




/** ------------------------------------------- util --------------------------------------------------------------*/
ar_result_t wcntr_get_thread_stack_size(wcntr_t *me_ptr, uint32_t *stack_size);
ar_result_t wcntr_get_set_thread_priority(wcntr_t *me_ptr, int32_t *priority_ptr, bool_t should_set);
ar_result_t wcntr_handle_fwk_events(wcntr_t *me_ptr);
ar_result_t wcntr_handle_frame_done(wcntr_topo_t *topo_ptr);
ar_result_t wcntr_parse_container_cfg(wcntr_t *me_ptr, apm_container_cfg_t *container_cfg_ptr);

/** ------------------------------------------- (ADSP)PM- ---------------------------------------------------------*/
ar_result_t wcntr_update_cntr_kpps_bw(wcntr_t *me_ptr);
ar_result_t wcntr_aggregate_kpps_bw(void *cu_ptr, uint32_t *kpps_ptr, uint32_t *bw_ptr);
ar_result_t wcntr_perf_vote(wcntr_t *me_ptr);
ar_result_t wcntr_handle_island_vote(wcntr_topo_t *topo_ptr, bool_t island_vote);

/** ------------------------------------------- topo events ------------------------------------------------------*/
ar_result_t wcntr_handle_event_to_dsp_service_topo_cb(wcntr_topo_module_t *module_context_ptr,
                                                         capi_event_info_t *event_info_ptr);
ar_result_t wcntr_handle_event_from_dsp_service_topo_cb(wcntr_topo_module_t *module_ptr,
                                                           capi_event_info_t *event_info_ptr);

ar_result_t wcntr_handle_capi_event(wcntr_topo_module_t *module_ptr,
                                       capi_event_id_t    event_id,
                                       capi_event_info_t *event_info_ptr);
/** ------------------------------------------- data         -----------------------------------------------------*/
ar_result_t wcntr_data_process_frames(wcntr_t *me_ptr);
/** ------------------------------------------- Signal Trigger -----------------------------------------------------*/
ar_result_t wcntr_signal_trigger(wcntr_base_t *cu_ptr, uint32_t channel_bit_index);

/** ------------------------------------------- cmd          -----------------------------------------------------*/
ar_result_t wcntr_gpr_cmd(wcntr_base_t *me_ptr);
ar_result_t wcntr_graph_open(wcntr_base_t *me_ptr);
ar_result_t wcntr_set_get_cfg(wcntr_base_t *me_ptr);
ar_result_t wcntr_graph_prepare(wcntr_base_t *me_ptr);
ar_result_t wcntr_graph_start(wcntr_base_t *me_ptr);
ar_result_t wcntr_graph_suspend(wcntr_base_t *me_ptr);
ar_result_t wcntr_graph_stop(wcntr_base_t *me_ptr);
ar_result_t wcntr_graph_flush(wcntr_base_t *me_ptr);
ar_result_t wcntr_graph_close(wcntr_base_t *me_ptr);
ar_result_t wcntr_graph_connect(wcntr_base_t *me_ptr);
ar_result_t wcntr_graph_disconnect(wcntr_base_t *me_ptr);
ar_result_t wcntr_destroy(wcntr_t *me_ptr);
ar_result_t wcntr_destroy_container(wcntr_base_t *me_ptr);
ar_result_t wcntr_ctrl_path_media_fmt_cmd(wcntr_base_t *me_ptr);
ar_result_t wcntr_cmd_icb_info_from_downstream(wcntr_base_t *base_ptr);
ar_result_t wcntr_handle_ctrl_port_trigger_cmd(wcntr_base_t *base_ptr);
ar_result_t wcntr_handle_rest_of_graph_open(wcntr_base_t *base_ptr, void *ctx_ptr);
ar_result_t wcntr_handle_peer_port_property_update_cmd(wcntr_base_t *me_ptr);
ar_result_t wcntr_handle_upstream_stop_cmd(wcntr_base_t *base_ptr);
ar_result_t wcntr_prepare_to_launch_thread(wcntr_t *         me_ptr,
                                              uint32_t *           stack_size,
                                              posal_thread_prio_t *priority,
                                              char *               thread_name,
                                              uint32_t             name_length);

ar_result_t wcntr_set_get_cfg_util(wcntr_base_t *         base_ptr,
                                      void *              mod_ptr,
                                      uint32_t            pid,
                                      int8_t *            param_payload_ptr,
                                      uint32_t *          param_size_ptr,
                                      uint32_t *          error_code_ptr,
                                      bool_t              is_wcntr_set_cfg,
                                      bool_t              is_deregister,
                                      spf_cfg_data_type_t cfg_type);

ar_result_t wcntr_register_events_utils(wcntr_base_t *       base_ptr,
                                           wcntr_gu_module_t *     gu_module_ptr,
                                           wcntr_topo_reg_event_t *reg_event_payload_ptr,
                                           bool_t            is_register,
                                           bool_t *          capi_supports_v1_event_ptr);
ar_result_t wcntr_cache_set_event_prop(wcntr_t *       me_ptr,
                                          wcntr_module_t *module_ptr,
                                          wcntr_topo_reg_event_t * event_cfg_payload_ptr,
                                          bool_t             is_register);

bool_t wcntr_is_signal_triggered(wcntr_t *me_ptr);

/** ------------------------------------------- buffer utils    -----------------------------------------------------*/
ar_result_t wcntr_handle_port_data_thresh_change_event(void *me_ptr); // also handles create buf

/* ------------------------------------------ FWK Extns --------------------------------------------------------------*/
ar_result_t wcntr_fwk_extn_handle_at_start(wcntr_t *me_ptr, wcntr_gu_module_list_t *module_list_ptr,uint32_t sg_id);
ar_result_t wcntr_fwk_extn_handle_at_stop(wcntr_t *me_ptr, wcntr_gu_module_list_t *module_list_ptr,uint32_t sg_id);

wcntr_topo_module_t *wcntr_get_stm_module(wcntr_t *me_ptr);
ar_result_t wear_cntr_create_module(wcntr_topo_t *           topo_ptr,
                                   wcntr_topo_module_t *    module_ptr,
                                   wcntr_topo_graph_init_t *graph_init_data_ptr);
ar_result_t wcntr_destroy_module(wcntr_topo_t *       topo_ptr,
                                    wcntr_topo_module_t *module_ptr,
                                    bool_t             reset_capi_dependent_dont_destroy);

/** ------------------------------------------- data flow state -----------------------------------------------------*/

//
ar_result_t wcntr_handle_sg_mgmt_cmd(wcntr_base_t *me_ptr, uint32_t sg_ops, wcntr_topo_sg_state_t sg_state);

ar_result_t wcntr_register_module_events(wcntr_base_t *me_ptr, gpr_packet_t *packet_ptr);

ar_result_t wcntr_register_events_utils(wcntr_base_t *       base_ptr,
                                           wcntr_gu_module_t *     gu_module_ptr,
                                           wcntr_topo_reg_event_t *reg_event_payload_ptr,
                                           bool_t            is_register,
                                           bool_t *          capi_supports_v1_event_ptr);

ar_result_t wcntr_set_get_cfgs_packed(wcntr_base_t *me_ptr, gpr_packet_t *packet_ptr, spf_cfg_data_type_t cfg_type);



ar_result_t wcntr_set_get_cfgs_fragmented(wcntr_base_t *               me_ptr,
                                             apm_module_param_data_t **param_data_pptr,
                                             uint32_t                  num_param_id_cfg,
                                             bool_t                    is_wcntr_set_cfg,
                                             bool_t                    is_deregister,
                                             spf_cfg_data_type_t       cfg_type);
											 


uint32_t wcntr_gpr_callback(gpr_packet_t *packet, void *callback_data);

ar_result_t wcntr_deregister_with_pm(wcntr_base_t *me_ptr);
ar_result_t wcntr_register_with_pm(wcntr_base_t *me_ptr);

ar_result_t wcntr_handle_imcl_ext_event(wcntr_base_t *     cu_ptr,
                                        wcntr_gu_module_t *module_ptr,
                                        capi_event_info_t *event_info_ptr);

// control port
ar_result_t wcntr_init_int_ctrl_port_queue(wcntr_base_t *cu_ptr, void* q_dest_ptr);
ar_result_t wcntr_deinit_int_ctrl_port_queue(wcntr_base_t *cu_ptr);
ar_result_t wcntr_check_and_recreate_int_ctrl_port_buffers(wcntr_base_t *        me_ptr,
                                                           wcntr_gu_ctrl_port_t *gu_ctrl_port_ptr);
ar_result_t wcntr_int_ctrl_port_get_recurring_buf(wcntr_base_t *        me_ptr,
                                                  wcntr_gu_ctrl_port_t *gu_ctrl_port_ptr,
                                                  capi_buf_t *          buf_ptr);
ar_result_t wcntr_init_internal_ctrl_port(wcntr_base_t *me_ptr, wcntr_gu_ctrl_port_t *gu_ctrl_port_ptr);
ar_result_t wcntr_check_and_retrieve_bufq_msgs_from_cmn_ctrlq(wcntr_base_t *        me_ptr,
                                                              wcntr_gu_ctrl_port_t *gu_ctrl_port_ptr);
ar_result_t wcntr_deinit_internal_ctrl_port(wcntr_base_t *me_ptr, wcntr_gu_ctrl_port_t *gu_ctrl_port_ptr);
ar_result_t wcntr_operate_on_int_ctrl_port(wcntr_base_t *         me_ptr,
                                           uint32_t               sg_ops,
                                           wcntr_gu_ctrl_port_t **gu_ctrl_port_ptr_ptr,
                                           bool_t                 is_self_sg);
ar_result_t wcntr_int_ctrl_port_process_outgoing_buf(wcntr_base_t *            me_ptr,
                                                     wcntr_gu_ctrl_port_t *    gu_ctrl_port_ptr,
                                                     capi_buf_t *              buf_ptr,
                                                     imcl_outgoing_data_flag_t flags);
ar_result_t wcntr_int_ctrl_port_get_one_time_buf(wcntr_base_t *        me_ptr,
                                                 wcntr_gu_ctrl_port_t *gu_ctrl_port_ptr,
                                                 capi_buf_t *          buf_ptr);
ar_result_t wcntr_init_internal_ctrl_ports(wcntr_base_t *base_ptr);
void wcntr_deinit_internal_ctrl_ports(wcntr_base_t *base_ptr, spf_msg_cmd_graph_mgmt_t *cmd_gmgmt_ptr);
ar_result_t wcntr_poll_and_process_int_ctrl_msgs(wcntr_base_t *me_ptr);
ar_result_t wcntr_prep_and_send_outgoing_ctrl_msg(wcntr_base_t *            me_ptr,
                                                  spf_msg_header_t *        header_ptr,
                                                  spf_handle_t *            dest_handle_ptr,
                                                  capi_buf_t *              buf_ptr,
                                                  imcl_outgoing_data_flag_t flags,
                                                  bool_t                    is_intra_cntr_msg,
                                                  uint32_t                  peer_port_id,
                                                  void *                    self_port_hdl,
                                                  void *                    peer_port_hdl,
                                                  bool_t                    is_recurring);
ar_result_t wcntr_alloc_populate_one_time_ctrl_msg(wcntr_base_t *me_ptr,
                                                   capi_buf_t *  buf_ptr,
                                                   bool_t        is_intra_cntr_msg,
                                                   void *        self_port_hdl,
                                                   void *        peer_port_hdl,
                                                   POSAL_HEAP_ID peer_heap_id,
                                                   spf_msg_t *   one_time_msg_ptr,
                                                   uint32_t      token,
                                                   spf_handle_t *dst_handle_ptr);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef WCNTR_UTIL_H
