#ifndef _APM_DATA_PATH_UTILS_I_H__
#define _APM_DATA_PATH_UTILS_I_H__

/**
 * \file apm_data_path_utils_i.h
 * \brief
 *     This file contains private structure definition for data
 *     path utilities
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "apm_i.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/*------------------------------------------------------------------------------
 *  Enumerations
 *----------------------------------------------------------------------------*/

/* clang-format off */

/*------------------------------------------------------------------------------
 *  APM Data Path Related Structure Definition
 *----------------------------------------------------------------------------*/

typedef struct apm_data_path_flags_t apm_data_path_flags_t;

struct apm_data_path_flags_t
{
   uint32_t path_valid : 1;
   /**< Flag to indicate if the path is valid */

   uint32_t one_time_query : 1;
   /**< Flag to indicate if this data path is
        queried one time */

   uint32_t container_path_update: 1;
   /**< Flag to indicate if the update needs to
        be sent to containers */

   uint32_t src_module_closed: 1;
   /**< Flag to indicate if the source module
        in the data path has been closed */

   uint32_t dstn_module_closed: 1;
   /**< Flag to indicate if the dstn module
        in the data path has been closed */
};


typedef struct apm_path_delay_shmem_t apm_path_delay_shmem_t;

struct apm_path_delay_shmem_t
{
   uint32_t container_id;
   /**< Container ID to which the delay
    	memory is shared for a given
        path ID */

   uint32_t delay_us;
   /**< Container path delay in microseconds
        for a given path ID  */
};

/**
 * This structure is used for storing the data path
 * information calcuated as part of query from
 * client or containers
 */

typedef struct apm_data_path_info_t apm_data_path_info_t;

struct apm_data_path_info_t
{
   uint32_t                path_id;
   /**< Data path ID  */

   spf_data_path_dfn_t      path_dfn;
   /**< Data path definition received as part of the
        query command/event */

   uint32_t                num_vertices;
   /**< Num of vertices in the data path graph */

   spf_list_node_t          *vertices_list_ptr;
   /**< List of vertices in the data path graph
        Obj Type: apm_module_data_link_t */

   uint32_t                num_containers;
   /**< Number of containers in the data path */

   apm_list_t              container_list;
   /**< List of containers in the data path*/

   apm_data_path_flags_t   flags;
   /**< Data path flag bit field */
};


typedef struct apm_data_path_delay_info_t apm_data_path_delay_info_t;

struct apm_data_path_delay_info_t
{
   apm_data_path_info_t     data_path;
   /**< Data path info populated after
        DFS traversal from source to destination
        module provided as part of data path definition */

   apm_path_delay_shmem_t  *delay_shmem_list_ptr;
   /**< Pointer to list of shared memory location.
        This memory is shared with containers for
        updating the overall container delay for
        a given path ID */
};

/**
 * This structure is used for caching the data path information
 * to be set to the container as part of
 * 1. Event to APM for data path config
 * 2. Client command for data path query
 */

typedef struct apm_cont_path_delay_cfg_t apm_cont_path_delay_cfg_t;

struct apm_cont_path_delay_cfg_t
{
   apm_cont_set_get_cfg_hdr_t  header;
   /**< Header for caching the container param */

   apm_data_path_info_t        data_path;
   /**< Section of the overall data path for a
        given path IDsection falling under
        the container */

   volatile uint32_t           *delay_ptr;
   /**< Pointer to memory location allocated by APM and
        shared with containers for storing the overall
        delay for the data path section falling within
        the container boundary */
};


typedef struct apm_cont_path_delay_src_module_cfg_t apm_cont_path_delay_src_module_cfg_t;

/**
 * This structure is used for caching the data path delay
 * pointer list to be sent to the host container of the source
 * module in the data path definition
 */
struct apm_cont_path_delay_src_module_cfg_t
{
   apm_cont_set_get_cfg_hdr_t  header;
   /**< Header for caching the container param */

   apm_data_path_delay_info_t  *delay_path_info_ptr;
   /**< Pointer to global path info corresponding to a
        given path ID */
};

/*------------------------------------------------------------------------------
 *  Static Inline Function Definition
 *----------------------------------------------------------------------------*/

static inline void apm_update_cont_set_cfg_msg_hdr(uint32_t                 container_id,
                                                   uint32_t                 param_id,
                                                   uint32_t                 param_size,
                                                   apm_module_param_data_t *param_data_ptr)
{
   param_data_ptr->module_instance_id = container_id;
   param_data_ptr->param_id           = param_id;
   param_data_ptr->param_size         = param_size;
   param_data_ptr->error_code         = AR_EOK;

   return;
}


#ifdef __cplusplus
}
#endif /*__cplusplus*/
#endif /* _APM_DATA_PATH_UTILS_I_H__ */
