#ifndef _APM_DB_QUERY_H__
#define _APM_DB_QUERY_H__

/**
 * \file apm_db_query.h
 *
 * \brief
 *     This file handles apm db queries
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

/**------------------------------------------------------------------------------
 *  Structure Definition
 *----------------------------------------------------------------------------*/
typedef struct apm_db_query_cntr_node_t
{
   void *cntr_node_ptr;
   /**< Node ptr to the container */

   uint32_t sg_count;
   /**< Number of times containers occurs on open/close cmd*/
} apm_db_query_cntr_node_t;

typedef struct apm_db_query_info_t
{
   bool_t is_profiling_enabled;
   /**< This flag is enabled when IRM queries APM about handles */

   spf_list_node_t *open_cntr_list_ptr;
   /**< List of containers during open
        Node Type: apm_db_query_cntr_node_t */

   spf_list_node_t *close_cntr_list_ptr;
   /**< List of containers during close
        Node Type: apm_db_query_cntr_node_t */

   spf_list_node_t *close_sg_list_ptr;
   /**< List of sgs during close
        Node Type: apm_sub_graph_t */
   // TODO:pbm we wont need this once Sumeet makes changes to have sg id list
} apm_db_query_info_t;

typedef struct apm_db_query_utils_vtable_t
{
   ar_result_t (*apm_db_query_preprocess_get_param_fptr)(apm_t *apm_info_ptr, apm_module_param_data_t *param_data_ptr);
   ar_result_t (*apm_db_query_add_cntr_to_list_fptr)(apm_t *apm_info_ptr, void *obj_ptr, bool_t is_open);
   ar_result_t (*apm_db_query_handle_graph_open_fptr)(apm_t *apm_info_ptr);
   ar_result_t (*apm_db_query_handle_graph_close_fptr)(apm_t *apm_info_ptr);
   void        (*apm_db_free_cntr_and_sg_list_fptr)();
} apm_db_query_utils_vtable_t;
/**------------------------------------------------------------------------------
 *  Function Declaration
 *----------------------------------------------------------------------------*/
ar_result_t apm_db_query_init(apm_t *apm_info_ptr);
#ifdef __cplusplus
}
#endif /*__cplusplus*/
#endif /* _APM_DB_QUERY_H__ */
