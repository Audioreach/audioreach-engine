#ifndef _APM_PROXY_DEF_H_
#define _APM_PROXY_DEF_H_

/**
 * \file apm_proxy_def.h
 * \brief
 *     This file contains declarations of the APM public API's
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "ar_error_codes.h"
#include "apm_graph_db.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/*****************************************
***   proxy mgr data structures   ***
******************************************/

/** Node type for the proxy subgraph list corresponding to a Proxy Manager.
    APM maintains sub graphs lists per Proxy Manager in this format.
*/

typedef struct apm_proxysg_list_node_t apm_proxysg_list_node_t;

struct apm_proxysg_list_node_t
{
   apm_sub_graph_t *apm_sg_node;

   apm_proxysg_list_node_t *next_ptr;

   apm_proxysg_list_node_t *prev_ptr;
};

/**
   Payload structure for SPF_MSG_CMD_PROXY_GRAPH_INFO command.
   APM sends this proxy sub graph list to corresponding Proxy Manager.
**/
typedef struct spf_msg_cmd_proxy_graph_info_t spf_msg_cmd_proxy_graph_info_t;

struct spf_msg_cmd_proxy_graph_info_t
{
   uint32_t num_proxy_sub_graphs;
   /**< Number of nodes in proxy_sg_list_ptr */

   void **sg_node_ptr_array;
   /**< Array of pointers to apm_sub_graph_t */

   uint32_t num_param_id_cfg;
   /**< Number of parameters being configured */

   void **param_data_pptr;
   /**< Array of pointers for module instance param data
        payload. Variable length array of pointers.
        Length of array is determined by number
        of paramter ID's being configured.
        Each index points to param data payload. */
};

/** Cached graph management parameters from client to be sent to ProxyManager */
typedef struct apm_proxy_graph_mgmt_params_t apm_proxy_graph_mgmt_params_t;

struct apm_proxy_graph_mgmt_params_t
{
   uint32_t input_start_index;

   uint32_t input_end_index;
   /** proxy_subgraph ids in graph mgmt cmd control are
       sorted in per-proxy manager order.
       above two are indices to pick the subgraphs from sorted list.*/
};

/** Cached graph management parameters from client to be sent to ProxyManager */
typedef struct apm_proxy_graph_open_params_t apm_proxy_graph_open_params_t;

struct apm_proxy_graph_open_params_t
{
   uint32_t num_proxy_sub_graphs;
   /**< Number of nodes in proxy_sg_list_ptr */

   spf_list_node_t *sg_list_ptr;
   /**< List of sub-graphs.
          Contains nodes of type apm_sub_graph_t */

   uint32_t num_mod_param_cfg;
   /**< Number of module param config */

   spf_list_node_t *param_data_list_ptr;
   /**< List of module parameter configuration payloads
        Node Type: apm_module_param_data_t */
};

struct apm_proxy_cfg_params_t
{
   bool_t use_sys_q;
   /**< Flag to indicate whether sys queue should be used
        to send the command */

   uint32_t num_mod_param_cfg;
   /**< Number of module param config */

   spf_list_node_t *param_data_list_ptr;
   /**< List of module parameter configuration payloads
        Node Type: apm_module_param_data_t */
};

/** Cached configuration parameters from client */
typedef struct apm_proxy_cfg_params_t apm_proxy_cfg_params_t;

/** Cached configuration parameters from client to be sent to ProxyManager */
typedef struct apm_proxy_cached_cfg_t apm_proxy_cached_cfg_t;

struct apm_proxy_cached_cfg_t
{
   union
   {
      apm_proxy_graph_open_params_t graph_open_params;
      /**< Cached parameter for container related to
        GRAPH OPEN command */

      apm_proxy_graph_mgmt_params_t graph_mgmt_params;
      /**< Cached parameter for container related to
        GRAPH MGMT commands */

      apm_proxy_cfg_params_t proxy_cfg_params;
      /**< Cached parameters for Proxy Manager related to
        SET_CFG command. */
   };
};

/** Response message received from a container */
typedef struct apm_proxy_cmd_rsp_ctrl_t apm_proxy_cmd_rsp_ctrl_t;

struct apm_proxy_cmd_rsp_ctrl_t
{
   bool_t pending_msg_proc;
   /**< If the message processing is pending
        for this proxyManager */

   bool_t rsp_pending;
   /**< If the response is pending from
        this container */

   spf_msg_t rsp_msg;
   /**< Response message from container */

   ar_result_t rsp_result;
   /**< Response result */

   bool_t reuse_rsp_msg_buf;
   /**< Reuse response msg buffer from container
        for subsequence message  */

   uint32_t num_permitted_subgraphs;
   /** Number of Subgraphs permitted by Proxy Manager
       for receiving Graph management command processing.*/

   apm_sub_graph_id_t *permitted_sg_array_ptr;
   /** Received array of Subgraphs permitted by proxy Manager
       for Graph management command processing.*/
};

typedef struct apm_proxy_cmd_ctrl_t apm_proxy_cmd_ctrl_t;

struct apm_proxy_cmd_ctrl_t
{
   apm_cmd_token_t msg_token;
   /**< Token to identify the context from a response message received. */

   uint32_t list_idx;
   /**< Flag to indicate if this cmd control object is in unse  */

   void *apm_cmd_ctrl_ptr;
   /**< Pointer to APM cmd control  */

   void *host_proxy_mgr_ptr;
   /**< Pointer to host container for this control object  */

   apm_proxy_cached_cfg_t cached_cfg_params;
   /**< Cached configuration parameters from client
        corresponding to this command */

   apm_proxy_cmd_rsp_ctrl_t rsp_ctrl;
   /**< Container response message info corresponding
        to this commands */
};

typedef struct apm_proxy_cmd_list_t apm_proxy_cmd_list_t;

struct apm_proxy_cmd_list_t
{
   uint32_t active_cmd_mask;
   /**< Bit mask for active commands under process */

   apm_proxy_cmd_ctrl_t *proxy_cmd_ctrl_ptr;
   /**< Current proxy command control object
        under process */

   apm_proxy_cmd_ctrl_t cmd_ctrl_list[APM_NUM_MAX_PARALLEL_CMD];
   /**< array of cmd control structures. */
};

typedef struct apm_vcpm_proxy_properties_t apm_vcpm_proxy_properties_t;

struct apm_vcpm_proxy_properties_t
{
   uint32_t vsid;
   /**< VSID corresponding to ProxyManager */
};

typedef struct apm_proxy_manager_t apm_proxy_manager_t;

struct apm_proxy_manager_t
{
   uint32_t scenario_id;
   /**< Scenario ID corresponding to ProxyManager */

   uint32_t proxy_instance_id;
   /**< static instance ID of the Proxy Manager. */

   spf_handle_t *proxy_handle_ptr;
   /**< Gk Handle to the Proxy Manager */

   uint32_t num_proxy_subgraphs;
   /**< Number of sub graphs in proxy_sg_list_ptr. */

   apm_proxysg_list_node_t *proxy_sg_list_ptr;
   /**< List of subgraphs handled by this ProxyManager
        Node Type: apm_proxysg_list_node_t */

   apm_proxy_cmd_list_t cmd_list;
   /**< List of commands under process for this ProxyManager */

   union
   {
      /** prpoerties to specific to the use case Proxy Manager.*/
      apm_vcpm_proxy_properties_t vcpm_properties;
   };
};

#ifdef __cplusplus
}
#endif /*__cplusplus*/
#endif /* #ifdef _APM_PROXY_DEF_H_ */
