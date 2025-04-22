#ifndef _TOPO_INTERFACE_H
#define _TOPO_INTERFACE_H

/**
 * \file topo_interface.h
 *
 * \brief
 *
 *     This file contains the topology interface exposed publically. All topologies must implement this interface.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "apm_api.h"
#include "apm_sub_graph_api.h"
#include "apm_container_api.h"
#include "apm_module_api.h"
#include "apm_debug_info.h"
#include "spf_utils.h"
#include "capi.h"
#include "module_cmn_api.h"
#include "spf_macros.h"
#include "topo_utils.h"
#include "capi_intf_extn_imcl.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

/**
 * Public functions which must be implemented by all topologies.
 */
typedef struct topo_cu_vtable_t
{
   /**
    * Propagates media format through the topology. The media format will
    * end up in internal output ports. is_data_path is true if we are calling
    * from a data path media format, and false if we are calling from a control
    * path media format.
    */
   ar_result_t (*propagate_media_fmt)(void *topo_ptr, bool_t is_data_path);

   /**
    * Complete the passed-in operation on the modules in the module list.
    */
   ar_result_t (*operate_on_modules)(void *                     topo_ptr,
                                     uint32_t                   sg_ops,
                                     gu_module_list_t *         module_list_ptr,
                                     spf_cntr_sub_graph_list_t *spf_sg_list_ptr);

   /* Complete the passed-in operation on the internal input port*/
   ar_result_t (*operate_on_int_in_port)(void *                     topo_ptr,
                                         gu_input_port_t *          in_port_ptr,
                                         spf_cntr_sub_graph_list_t *spf_sg_list_ptr,
                                         uint32_t                   sg_ops,
                                         bool_t                     set_port_op);

   /* Complete the passed-in operation on the internal output port*/
   ar_result_t (*operate_on_int_out_port)(void *                     topo_ptr,
                                          gu_output_port_t *         out_port_ptr,
                                          spf_cntr_sub_graph_list_t *spf_sg_list_ptr,
                                          uint32_t                   sg_ops,
                                          bool_t                     set_port_op);

   /* Complete the passed-in operation on the internal control port*/
   ar_result_t (*operate_on_int_ctrl_port)(void *                     topo_ptr,
                                           gu_ctrl_port_t *           ctrl_link_ptr,
                                           spf_cntr_sub_graph_list_t *spf_sg_list_ptr,
                                           uint32_t                   sg_ops,
                                           bool_t                     set_port_op);

   /* Perform control port operations on module. */
   ar_result_t (*ctrl_port_operation)(gu_ctrl_port_t *             gu_ctrl_port_ptr,
                                      intf_extn_imcl_port_opcode_t op_code,
                                      POSAL_HEAP_ID                heap_id);

   /**
    * Subgraph related
    */
   topo_sg_state_t (*get_sg_state)(gu_sg_t *sg_ptr);

   /* Destroy all metadata in the metadata list belonging to the passed-in module. */
   ar_result_t (*destroy_all_metadata)(uint32_t               log_id,
                                       void *                 topo_module_ptr,
                                       module_cmn_md_list_t **md_list_pptr,
                                       bool_t                 is_dropped);

   /** Propagates boundary modules port states to internal modules and applies the port states on modules. */
   ar_result_t (*propagate_boundary_modules_port_state)(void *topo_ptr);

   /** Adds path delay info to given port */
   ar_result_t (*add_path_delay_info)(void *   v_topo_ptr,
                                      void *   v_module_ptr,
                                      uint32_t port_id,
                                      void *   v_cmn_port_ptr,
                                      uint32_t path_id);

   ar_result_t (*rtm_dump_data_port_media_fmt)(void *vtopo_ptr, uint32_t container_instance_id, uint32_t port_media_fmt_report_enable);

   ar_result_t (*update_path_delays)(void *              v_topo_ptr,
                                     uint32_t            path_id,
                                     uint32_t *          aggregated_algo_delay_ptr,
                                     uint32_t *          aggregated_ext_in_delay_ptr,
                                     uint32_t *          aggregated_ext_out_delay_ptr);

   /** Removes path delay info based on path-id */
   ar_result_t (*remove_path_delay_info)(void *topo_ptr, uint32_t path_id);

   ar_result_t (*query_module_delay)(void *    v_topo_ptr,
                                     void *    v_module_ptr,
                                     void *    v_cmn_in_port_ptr,
                                     void *    v_cmn_out_port_ptr,
                                     uint32_t *delay_us_ptr);

   /*Propagates given property type through the topology. usually called during prepare.**/
   ar_result_t (*propagate_port_property)(void *topo_ptr, topo_port_property_type_t prop_type);

   ar_result_t (*propagate_port_property_forwards)(void *                    vtopo_ptr,
                                                   void *                    vin_port_ptr,
                                                   topo_port_property_type_t prop_type,
                                                   uint32_t                  prop_payload,
                                                   uint32_t *                recurse_depth_ptr);
   ar_result_t (*propagate_port_property_backwards)(void *                    vtopo_ptr,
                                                    void *                    vout_port_ptr,
                                                    topo_port_property_type_t prop_type,
                                                    uint32_t                  prop_payload,
                                                    uint32_t *                recurse_depth_ptr);
   ar_result_t (*get_port_property)(void *                    vtopo_ptr,
                                    topo_port_type_t          port_type,
                                    topo_port_property_type_t prop_type,
                                    void *                    port_ptr,
                                    uint32_t *                val_ptr);
   ar_result_t (*set_port_property)(void *                    vtopo_ptr,
                                    topo_port_type_t          port_type,
                                    topo_port_property_type_t prop_type,
                                    void *                    port_ptr,
                                    uint32_t                  val);

   ar_result_t (*set_param)(void *vtopo_ptr, apm_module_param_data_t *param_ptr);

   ar_result_t (*get_prof_info)(void *vtopo_ptr, int8_t *param_payload_ptr, uint32_t *param_size_ptr);

   ar_result_t (*get_port_threshold)(void * port_ptr);

   ar_result_t (*check_update_started_sorted_module_list)(void *vtopo_ptr, bool_t b_force_update);

   ar_result_t (*set_global_sh_mem_msg)(void     *vtopo_ptr,
                                        uint32_t  miid,
                                        void     *virt_addr_ptr,
                                        void     *cmd_header_ptr);
} topo_cu_vtable_t;

typedef struct topo_cu_island_vtable_t
{
   /* Send incoming ctrl msg buffer to the module */
   ar_result_t (*handle_incoming_ctrl_intent)(void *   topo_ctrl_port_ptr,
                                              void *   intent_buf,
                                              uint32_t max_size,
                                              uint32_t actual_size);
} topo_cu_island_vtable_t;

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef _TOPO_INTERFACE_H
