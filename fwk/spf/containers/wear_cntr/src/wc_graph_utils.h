#ifndef WC_GRAPH_UTILS_H_
#define WC_GRAPH_UTILS_H_

/**
 * \file wc_graph_utils.h
 *
 * \brief
 *
 *     graph utilities
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "spf_utils.h"
#include "apm_sub_graph_api.h"
#include "ar_ids.h"
#include "gpr_api_inline.h"
#include "apm_cntr_if.h"

#define ENABLE_GU_TEST

// GRAPH_EASY flag enables prints which allow to create graphs from the logs.
//#define GRAPH_EASY

// Enable to add debug prints.
//#define GU_DEBUG

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

typedef struct wcntr_gu_t                    wcntr_gu_t;
typedef struct wcntr_gu_sg_t                 wcntr_gu_sg_t;
typedef struct wcntr_gu_module_t             wcntr_gu_module_t;
typedef struct wcntr_gu_input_port_t         wcntr_gu_input_port_t;
typedef struct wcntr_gu_output_port_t        wcntr_gu_output_port_t;
typedef struct wcntr_gu_ctrl_port_t          wcntr_gu_ctrl_port_t;
typedef struct wcntr_gu_module_list_t        wcntr_gu_module_list_t;
typedef struct wcntr_gu_input_port_list_t    wcntr_gu_input_port_list_t;
typedef struct wcntr_gu_output_port_list_t   wcntr_gu_output_port_list_t;
typedef struct wcntr_gu_sg_list_t            wcntr_gu_sg_list_t;
typedef struct wcntr_gu_ctrl_port_list_t     wcntr_gu_ctrl_port_list_t;


typedef struct wcntr_gu_ctrl_port_prop_intent_list_t wcntr_gu_ctrl_port_prop_intent_list_t;

/* List structures. These should all mirror spf_list_node_t to typecast. */

/* Modules */
typedef struct wcntr_gu_module_list_t
{
   wcntr_gu_module_t *     module_ptr;
   wcntr_gu_module_list_t *next_ptr;
   wcntr_gu_module_list_t *prev_ptr;
} wcntr_gu_module_list_t;

/* Module input ports */
typedef struct wcntr_gu_input_port_list_t
{
   wcntr_gu_input_port_t *     ip_port_ptr;
   wcntr_gu_input_port_list_t *next_ptr;
   wcntr_gu_input_port_list_t *prev_ptr;
} wcntr_gu_input_port_list_t;

/* Module output ports */
typedef struct wcntr_gu_output_port_list_t
{
   wcntr_gu_output_port_t *     op_port_ptr;
   wcntr_gu_output_port_list_t *next_ptr;
   wcntr_gu_output_port_list_t *prev_ptr;
} wcntr_gu_output_port_list_t;

/* Subgraphs */
typedef struct wcntr_gu_sg_list_t
{
   wcntr_gu_sg_t *     sg_ptr;
   wcntr_gu_sg_list_t *next_ptr;
   wcntr_gu_sg_list_t *prev_ptr;
} wcntr_gu_sg_list_t;

/**
 * Graph states
 */
typedef enum wcntr_gu_status_t
{
   WCNTR_GU_STATUS_DEFAULT = 0,
   WCNTR_GU_STATUS_NEW = 1,
   WCNTR_GU_STATUS_UPDATED = 2
} wcntr_gu_status_t;


/**
 * Common struct for input and output port.
 */
typedef struct wcntr_gu_cmn_port_t
{
   uint32_t     id;         /**< This is the ID of the port. Index of the port (CAPIv2 requirement) is different. Framework sets ID <->
                                   Index mapping to the module.*/
   uint32_t     index;      /**< index (0 to num_ports) is determined by number of active ports */
   wcntr_gu_module_t *module_ptr; /**< Ptr of the module to which this input port belongs */
   wcntr_gu_status_t  gu_status;
   bool_t       mark;       /**< for graph sort usage */
   bool_t   boundary_port;  /**<TRUE if connected port is in different subgraph*/
} wcntr_gu_cmn_port_t;

/**
 * input ports
 */
typedef struct wcntr_gu_input_port_t
{
   wcntr_gu_cmn_port_t     cmn;                /**< Must be first element */
   wcntr_gu_output_port_t  *conn_out_port_ptr; /**< Upstream or source port connecting to this input port */
} wcntr_gu_input_port_t;

/**
 * output ports
 */
typedef struct wcntr_gu_output_port_t
{
   wcntr_gu_cmn_port_t     cmn;                /**< Must be first element */
   wcntr_gu_input_port_t   *conn_in_port_ptr;  /**< Downstream or destination port connecting to this output port */
} wcntr_gu_output_port_t;


/**
 * Control port
 */
typedef struct wcntr_gu_ctrl_port_t
{
   uint32_t            id;                /**< Control port ID */
   wcntr_gu_module_t *       module_ptr;        /**< Module to which the current control port is connected to.*/
   wcntr_gu_status_t         gu_status;
   wcntr_gu_ctrl_port_t *    peer_ctrl_port_ptr; /**< Module to which the current control port is connected to.*/
   wcntr_gu_ctrl_port_prop_intent_list_t *intent_info_ptr; /**< Control link intent list info ptr. */
   bool_t                 is_peer_port_in_same_sg; /**<TRUE if connected control port is in different subgraph.Based on assumption that all subgraphs are in same container>*/
} wcntr_gu_ctrl_port_t;

/*
 * Control port intent list structure.
 * */
#include "spf_begin_pragma.h"
struct wcntr_gu_ctrl_port_prop_intent_list_t
{
   uint32_t num_intents;       /**< Number of intents associated with a control link */
   uint32_t intent_id_list[0]; /**< List of intents associated with a control link */
}
#include "spf_end_pragma.h"
;

/*
 * Control port list pointer
 * */
typedef struct wcntr_gu_ctrl_port_list_t
{
   wcntr_gu_ctrl_port_t *     ctrl_port_ptr;
   wcntr_gu_ctrl_port_list_t *next_ptr;
   wcntr_gu_ctrl_port_list_t *prev_ptr;
} wcntr_gu_ctrl_port_list_t;


typedef struct wcntr_gu_module_flags_t
{
   uint8_t is_connected : 1;  /**< Indicates if a module is fully connected. For modules that support input, should have at
                                   least one input port.
                                   For modules that support output, should have at least one output port. */
   uint8_t is_source : 1;     /**< is source module */
   uint8_t is_sink : 1;       /**< is sink module */
   uint8_t is_siso : 1;       /**< is single-input-single-output module */
} wcntr_gu_module_flags_t;

/**
 * Module
 */
typedef struct wcntr_gu_module_t
{
   uint32_t module_id;   /**< */
   uint32_t module_instance_id;
   void *   amdb_handle;
   uint8_t  module_type; /**< AMDB_MODULE_TYPE_DECODER, AMDB_MODULE_TYPE_ENCODER, AMDB_MODULE_TYPE_PACKETIZER,
                            AMDB_MODULE_TYPE_DEPACKETIZER,
                              AMDB_MODULE_TYPE_CONVERTER, AMDB_MODULE_TYPE_GENERIC, AMDB_MODULE_TYPE_FRAMEWORK */
   uint8_t  itype;       /**< interface type of the module: CAPIV2 or STUB */
   uint8_t  max_input_ports;
   uint8_t  max_output_ports;
   uint8_t  num_input_ports;
   uint8_t  num_output_ports;
   uint8_t  num_ctrl_ports;
   wcntr_gu_module_flags_t flags;
   wcntr_gu_input_port_list_t * input_port_list_ptr;  /**< wcntr_gu_input_port_t */
   wcntr_gu_output_port_list_t *output_port_list_ptr; /**< wcntr_gu_output_port_t */
   wcntr_gu_ctrl_port_list_t *ctrl_port_list_ptr; /**< list of control ports connected to the module. */
   wcntr_gu_sg_t *   sg_ptr; /**< every module belongs to a subgraph.  */
   wcntr_gu_status_t gu_status;
} wcntr_gu_module_t;

/**
 * subgraph
 */
typedef struct wcntr_gu_sg_t
{
   uint32_t          id;
   uint8_t           sid;       /**< Scenario ID */
   uint8_t           perf_mode; /**< Performance mode */
   uint8_t           direction;
   uint16_t          num_modules;
   wcntr_gu_module_list_t *module_list_ptr; /**< list of modules. Each node is a wcntr_gu_module_t. this is primary list of module tied to num_modules. */
   wcntr_gu_t *            gu_ptr;                /*< Graph pointer to which the sub graph belongs to.>*/
   wcntr_gu_status_t       gu_status;
   wcntr_gu_input_port_list_t * boundary_in_port_list_ptr; /**<linked list of boundary input ports>*/
   uint32_t num_boundary_in_ports;
   wcntr_gu_output_port_list_t *boundary_out_port_list_ptr; /**<linked list of boundary input ports>*/
   uint32_t num_boundary_out_ports;

} wcntr_gu_sg_t;


typedef struct wcntr_sg_debug_info_t
{
   uint32_t  id;
   uint32_t  direction;
   uint32_t  num_modules;  /* Number of modules present in subgraph */
   uint32_t  num_boundary_in_ports;
   uint32_t  num_boundary_out_ports;
   uint32_t  num_mod_debug_info_filled;  /* Debug information of number of modules filled */
} wcntr_sg_debug_info_t;




/**
 * graph utility
 */
typedef struct wcntr_gu_t
{
   uint32_t          log_id;        /**< logging id:  WCNTR_LOG_ID_CNTR_TYPE_SHIFT etc */
   bool_t            sort_status;   /**< indicates if sort was completed or not */
   uint8_t           num_subgraphs; /**< number of subgraphs */
   wcntr_gu_sg_list_t      *sg_list_ptr;   /**< list of subgraphs. Each node is a wcntr_gu_sg_t.*/
   wcntr_gu_module_list_t  *sorted_module_list_ptr; /**< sorted module list. Sorted in DAG order. Each node is wcntr_gu_module_t. Modules could
                                                   belong to different subgraphs. sg_ptr->module_list_ptr is the primary list. */
   wcntr_gu_module_list_t *floating_modules_list_ptr;
   wcntr_gu_module_list_t *src_module_list_ptr; /**< list of source modules. Each node is wcntr_gu_module_t. not a primary list for modules */
   wcntr_gu_module_list_t *snk_module_list_ptr; /**< list of sink modules. Each node is wcntr_gu_module_t. not a primary list for modules */
} wcntr_gu_t;

typedef struct wcntr_gu_sizes_t
{
   uint16_t ctrl_port_size;
   uint16_t in_port_size;
   uint16_t out_port_size;
   uint16_t sg_size;
   uint16_t module_size;
} wcntr_gu_sizes_t;

ar_result_t wcntr_gu_create_graph(wcntr_gu_t *                    gu_ptr,
                            spf_msg_cmd_graph_open_t *open_cmd_ptr,
                            wcntr_gu_sizes_t *              sizes_ptr,
                            POSAL_HEAP_ID             heap_id);

ar_result_t wcntr_gu_respond_to_graph_open(wcntr_gu_t *gu_ptr, spf_msg_t *cmd_msg_ptr, POSAL_HEAP_ID heap_id);

wcntr_gu_module_t *wcntr_gu_find_module(wcntr_gu_t *gu_ptr, uint32_t module_instance_id);

ar_result_t wcntr_gu_parse_data_link(wcntr_gu_t *                 gu_ptr,
                               apm_module_conn_cfg_t *data_link_ptr,
                               wcntr_gu_output_port_t **    src_mod_out_port_ptr,
                               wcntr_gu_input_port_t **     dst_mod_in_port_ptr);

ar_result_t wcntr_gu_parse_ctrl_link(wcntr_gu_t *                      gu_ptr,
                               apm_module_ctrl_link_cfg_t *ctrl_link_ptr,
                               wcntr_gu_ctrl_port_t **           peer_1_ctrl_port_pptr,
                               wcntr_gu_ctrl_port_t **           peer_2_ctrl_port_pptr);


ar_result_t wcntr_gu_destroy_graph(wcntr_gu_t *gu_ptr, spf_msg_cmd_graph_mgmt_t *cmd_gmgmt_ptr, POSAL_HEAP_ID heap_id);

bool_t wcntr_gu_is_sg_id_found_in_spf_array(spf_cntr_sub_graph_list_t *spf_sg_array_ptr, uint32_t sg_id);
bool_t wcntr_gu_is_port_handle_found_in_spf_array(uint32_t       num_handles,
                                            spf_handle_t **port_handles_array_pptr,
                                            spf_handle_t * given_handle);

wcntr_gu_cmn_port_t *wcntr_gu_find_cmn_port_by_id(wcntr_gu_module_t *module_ptr, uint32_t id);
// wcntr_gu_find_input_port_by_id
wcntr_gu_input_port_t *wcntr_gu_find_input_port(wcntr_gu_module_t *module_ptr, uint32_t id);
// wcntr_gu_find_output_port by id
wcntr_gu_output_port_t *wcntr_gu_find_output_port(wcntr_gu_module_t *module_ptr, uint32_t id);
wcntr_gu_output_port_t *wcntr_gu_find_output_port_by_index(wcntr_gu_module_t *module_ptr, uint32_t index);
wcntr_gu_input_port_t *wcntr_gu_find_input_port_by_index(wcntr_gu_module_t *module_ptr, uint32_t index);
wcntr_gu_ctrl_port_t *wcntr_gu_find_ctrl_port_by_id(wcntr_gu_module_t *module_ptr, uint32_t id);
wcntr_gu_sg_t *wcntr_gu_find_subgraph(wcntr_gu_t *gu_ptr, uint32_t id);

ar_result_t wcntr_gu_update_sorted_list(wcntr_gu_t *gu_ptr, POSAL_HEAP_ID heap_id);
ar_result_t wcntr_gu_prepare_for_module_loading(wcntr_gu_t *            gu_ptr,
                                          spf_list_node_t **amdb_h_list_pptr,
                                          wcntr_gu_module_t *     module_ptr,
                                          POSAL_HEAP_ID     heap_id);
ar_result_t wcntr_gu_handle_module_loading(spf_list_node_t **amdb_h_list_pptr, uint32_t log_id);
ar_result_t wcntr_gu_reset_graph_port_markers(wcntr_gu_t *gu_ptr);
void wcntr_gu_print_graph(wcntr_gu_t *gu_ptr);

/**
 * if return value is TRUE, then given port-id is that of output
 * if not it's input.
 */
static inline bool_t wcntr_gu_is_output_port_id(uint32_t port_id)
{
   bool_t is_output = (AR_PORT_DIR_TYPE_OUTPUT == spf_get_bits(port_id, AR_PORT_DIR_TYPE_MASK, AR_PORT_DIR_TYPE_SHIFT));
   return is_output;
}

/**
 * if return value is TRUE, then given domain-id is remote
 */
static inline bool_t wcntr_gu_is_domain_id_remote(uint32_t domain_id)
{
   uint32_t host_domain_id;
   __gpr_cmd_get_host_domain_id(&host_domain_id);
   bool_t is_remote = (host_domain_id != domain_id) ? TRUE : FALSE;
   return is_remote;
}

static inline POSAL_HEAP_ID gu_get_heap_id_from_heap_prop(uint32_t heap_prop)
{
   POSAL_HEAP_ID heap_id = (APM_CONT_HEAP_LOW_POWER == heap_prop) ? posal_get_island_heap_id() : POSAL_HEAP_DEFAULT;
   return heap_id;
}

#ifdef __cplusplus
}
#endif //__cplusplus

// clang-format on
#endif // #ifndef WC_GRAPH_UTILS_H_
