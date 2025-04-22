#ifndef _APM_CONTAINER_PATH_DELAY_IF_H_
#define _APM_CONTAINER_PATH_DELAY_IF_H_

/**
 * \file apm_cntr_if.h
 *  
 * \brief
 *     This file defines APM to container functions.
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

#include "spf_utils.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

/********************************************************************************************************/
/*                                          Params                                                      */
/********************************************************************************************************/
typedef struct cntr_graph_vertex_t cntr_graph_vertex_t;

struct cntr_graph_vertex_t
{
   uint32_t module_instance_id;
   uint32_t port_id;
};

/**
 * list node definition
 *
 * APM and containers use spf_list_node_t by casting this.
 */
typedef struct cntr_list_node_t cntr_list_node_t;

struct cntr_list_node_t
{
   void *obj_ptr;
   /**< pointer to the instance this node refers to.
        This is used to implement generic type independent
        list utilities*/

   cntr_list_node_t *next_ptr;
   /**< pointer to the next node*/

   cntr_list_node_t *prev_ptr;
   /**< pointer to the next node*/
};

/**
   This param ID is used as part of #SPF_MSG_CMD_SET_CFG.
   It sets the delay path and also gets back the delay of the path.

   This parameter is used by container clients (e.g. APM) for
   the following:
     - Provide the data path definition to the container.
     - Provide the pointer to memory for container to store
       the overall path delay for a given path
     - Query the overall delay for the provided data path.

    APM sends this to containers to set path-delay cfg such as path-id, delay-ptr, path-definition.

     This is used in in 2 contexts
        a) where run time delay updates are needed (such as SPR).
        b) when HLOS queries for the path delay.

     Containers respond to this once they update delay once.
*/

#define CNTR_PARAM_ID_PATH_DELAY_CFG 0x0800111B

typedef struct cntr_param_id_path_delay_cfg_t cntr_param_id_path_delay_cfg_t;

/**
 * Payload for container param ID #CNTR_PARAM_ID_PATH_DELAY_CFG.
 */
struct cntr_param_id_path_delay_cfg_t
{
   bool_t is_one_time_query;        /**< is_one_time_query = TRUE: containers don't store path
                                                   FALSE: containers store path and keep delay updated */
   uint32_t           path_id;      /**< ID of the path. 0 is reserved. */
   volatile uint32_t *delay_us_ptr; /**< ptr to the delay variable created by APM */
   cntr_list_node_t * path_def_ptr; /**< List of vertices. Each object of type cntr_graph_vertex_t */
};

/**
 * This param ID is used as part of #SPF_MSG_CMD_SET_CFG.
 *
 * This parameter is used by container clients (E.g., APM) for
 * destroying the delay path configured using CNTR_PARAM_ID_PATH_DELAY_CFG.
 *
 * This is sent to all containers which are in the path.
 * This helps in making sure that the containers don't access the delay ptrs
 * when graph is destroyed
 *
 * During destroy CNTR_PARAM_ID_PATH_DESTROY is used first and
 * then CNTR_PARAM_ID_DESTROY_SRC_MOD_DELAY_LIST is used.
 *
 * payload : cntr_param_id_path_destroy_t
 *
 * This is used in the context where run time delay updates are needed (such as SPR)
 *
 */
#define CNTR_PARAM_ID_PATH_DESTROY 0x08001243

typedef struct cntr_param_id_path_destroy_t cntr_param_id_path_destroy_t;

/* Payload for CNTR_PARAM_ID_PATH_DESTROY */
struct cntr_param_id_path_destroy_t
{
   uint32_t path_id;
};

/**
 * This param ID is used as part of #SPF_MSG_CMD_SET_CFG.
 *
 * This parameter is used by container clients (E.g., APM) for
 * setting the delay list on modules that raised SPF_EVT_TO_APM_FOR_PATH_DELAY
 *
 * This is used in the context where run time delay updates are needed (such as SPR)
 *
 * Payload: cntr_param_id_cfg_src_mod_delay_list_t
 */
#define CNTR_PARAM_ID_CFG_SRC_MOD_DELAY_LIST 0x0800112A

typedef struct cntr_param_id_cfg_src_mod_delay_list_t cntr_param_id_cfg_src_mod_delay_list_t;

/**
 * Payload for CNTR_PARAM_ID_CFG_SRC_MOD_DELAY_LIST
 */
struct cntr_param_id_cfg_src_mod_delay_list_t
{
   uint32_t            path_id;                /**< ID assigned for the path by APM*/
   uint32_t            src_module_instance_id; /**< Module instance ID of the source of the path */
   uint32_t            src_port_id;            /**< Port ID of the source of the path */
   uint32_t            dst_module_instance_id; /**< Module instance ID of the destination of the path */
   uint32_t            dst_port_id;            /**< Port ID of the destination of the path */
   uint32_t            num_delay_ptrs;         /**< number of delay pointers */
   volatile uint32_t **delay_us_pptr;          /**< Array of ptrs to the delay variable created by APM */
};

/**
 * This param ID is used as part of #SPF_MSG_CMD_SET_CFG.
 *
 * This parameter is used by container clients (E.g., APM) for
 * destroying the delay list set on modules using CNTR_PARAM_ID_CFG_SRC_MOD_DELAY_LIST
 *
 * Payload: cntr_param_id_destroy_src_mod_delay_list_t
 */
#define CNTR_PARAM_ID_DESTROY_SRC_MOD_DELAY_LIST 0x0800112B

typedef struct cntr_param_id_destroy_src_mod_delay_list_t cntr_param_id_destroy_src_mod_delay_list_t;

/**
 * Payload for CNTR_PARAM_ID_DESTROY_SRC_MOD_DELAY_LIST
 */
struct cntr_param_id_destroy_src_mod_delay_list_t
{
   uint32_t path_id;                /**< ID assigned for the path by APM*/
   uint32_t src_module_instance_id; /**< Module instance ID of the source of the path */
   uint32_t src_port_id;            /**< Port ID of the source of the path */
};

/********************************************************************************************************/
/*                                          Messages                                                    */
/********************************************************************************************************/

typedef struct spf_data_path_dfn_t spf_data_path_dfn_t;
struct spf_data_path_dfn_t
{
   uint32_t src_module_instance_id; /**< Module instance ID of the source of the path */
   uint32_t src_port_id;            /**< Port ID of the source of the path. */
   uint32_t dst_module_instance_id; /**< Module instance ID of the destination of the path */
   uint32_t dst_port_id;            /**< Port ID of the destination of the path.  */
};

/**
 * Payload for SPF_EVT_TO_APM_FOR_PATH_DELAY
 */
typedef struct spf_evt_to_apm_for_path_delay_t spf_evt_to_apm_for_path_delay_t;
struct spf_evt_to_apm_for_path_delay_t
{
   spf_data_path_dfn_t path_def;
   /**< src_module_instance_id: Module instance ID of the source of the path */
   /**< src_port_id: Port ID of the source of the path. Can be zero if port-id is not known.
        If zero, first port that ends in dst_module_instance_id is picked.*/
   /**< dst_module_instance_id: Module instance ID of the destination of the path */
   /**< dst_port_id: Port ID of the destination of the path. Can be zero if port-id is not known.
        If zero, first port that ends in dst_module_instance_id is picked. */
};

/** Event from containers to APM for path delay.
 *
 * This is used in the context where run time delay updates are needed (such as SPR)
 *
 * payload : spf_evt_to_apm_for_path_delay_t*/
#define SPF_EVT_TO_APM_FOR_PATH_DELAY 0x0300100A


#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef _APM_CONTAINER_PATH_DELAY_IF_H_
