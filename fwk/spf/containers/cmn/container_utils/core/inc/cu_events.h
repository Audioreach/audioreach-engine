#ifndef CU_EVENTS_H_
#define CU_EVENTS_H_
/**
 * \file cu_events.h
 * \brief
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "spf_utils.h"
#include "capi.h"

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

// clang-format off
/* Contains client info required to raise the event
 * and event cfg to cache event cfg for framework modules*/
typedef struct cu_client_info_t
{
   uint32_t      src_port;
   uint32_t      token;
   capi_buf_t    event_cfg;
   uint8_t       src_domain_id;
   uint8_t       dest_domain_id;
} cu_client_info_t;

/* Contains event ID and client list of the event */
typedef struct cu_event_info_t
{
   spf_list_node_t  *root_client_node;
   uint32_t        event_id;
} cu_event_info_t;
// clang-format on
ar_result_t cu_event_add_client(uint32_t          log_id,
                                uint32_t          event_id,
                                cu_client_info_t *client_info,
                                spf_list_node_t **rootNode,
                                POSAL_HEAP_ID     heap_id);

ar_result_t cu_event_delete_client(uint32_t          log_id,
                                   uint32_t          event_id,
                                   cu_client_info_t *client_info,
                                   spf_list_node_t **rootNode);

void cu_delete_all_event_nodes(spf_list_node_t **rootNode);

ar_result_t cu_find_client_info(uint32_t          log_id,
                                uint32_t          event_id,
                                spf_list_node_t * rootNode,
                                spf_list_node_t **client_list_ptr);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // CU_EVENTS_H_
