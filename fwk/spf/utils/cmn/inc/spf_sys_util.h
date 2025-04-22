/**
 * \file spf_sys_util.h
 * \brief
 *     This file contains structures and message ID's for service register utils.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef _SPF_SVC_REG_UTIL_H_
#define _SPF_SVC_REG_UTIL_H_

/* =======================================================================
INCLUDE FILES FOR MODULE
========================================================================== */
#include "posal_channel.h"
#include "posal_queue.h"
#include "spf_cmn_if.h"
#include "spf_sys_util_api.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

typedef struct spf_sys_util_ssr_info_t
{
   uint32_t proc_domain_id;     // proc domain id for which ssr registration is done
   void *   sev_reg_handle_ptr; // pointer to the handle returned by the ar osal ssr layer
} spf_sys_util_ssr_info_t;

typedef struct spf_sys_util_vtable
{
   // Function pointer for handling ssr up/down notification
   ar_result_t (*spf_sys_util_handle_svc_status)(void *status_ptr);

   // Function pointer for handling close all
   ar_result_t (*spf_sys_util_handle_close_all)(void *close_all_ptr);

   // Function pointer for handling close all
   ar_result_t (*spf_sys_util_handle_sat_pd_info)(uint32_t num_proc_domain_ids, uint32_t *proc_domain_id_list);

   // Function pointer for handling kill
   ar_result_t (*spf_sys_util_handle_kill)(void);
} spf_sys_util_vtable;

// Sturture definition for the handle returned by sys util layer
typedef struct spf_sys_util_handle_t
{
   posal_queue_t *          sys_queue_ptr;        // System queue pointer
   uint32_t                 sys_queue_mask;       // Channel mask to be used with system queue
   posal_channel_t          sys_cmd_done_channel; // Channel associated with sending sys cmd done
   posal_signal_t           sys_cmd_done_signal;  // Signal for sending sys cmd done
   uint32_t                 sys_cmd_done_mask;    // Mask for sending sys cmd done
   spf_sys_util_vtable      sys_util_vtable;      // Vtable of function pointers to handle sys cmds
   POSAL_HEAP_ID            heap_id;              // heap id - for the ssr callback and other allocations
   uint32_t                 num_proc_domain_ids;  // Number of proc domain ids registered for ssr
   spf_sys_util_ssr_info_t *reg_info_list;        // Proc id and handle info
} spf_sys_util_handle_t;

/*----------------------------------------------------------------------------------------------------------------------
  Returns the Sys util handle to b
 ----------------------------------------------------------------------------------------------------------------------*/
ar_result_t spf_sys_util_get_handle(spf_sys_util_handle_t **handle_pptr,
                                    spf_sys_util_vtable *   sys_util_vtable_ptr,
                                    posal_channel_t         channel,
                                    char_t *                q_name_ptr,
                                    uint32_t                q_bit_mask,
                                    POSAL_HEAP_ID           heap_id,
                                    uint32_t                num_max_sys_q_elements,
                                    uint32_t                num_max_prealloc_sys_q_elem);

/*----------------------------------------------------------------------------------------------------------------------
 Releases the sys util handle
 ----------------------------------------------------------------------------------------------------------------------*/
ar_result_t spf_sys_util_release_handle(spf_sys_util_handle_t **handle_pptr);

/*----------------------------------------------------------------------------------------------------------------------
 Used to register the given proc domain IDs with Service registry
 ----------------------------------------------------------------------------------------------------------------------*/
ar_result_t spf_sys_util_ssr_register(spf_sys_util_handle_t *handle_ptr,
                                      uint32_t               num_proc_domain,
                                      uint32_t *             proc_domain_id_list);

/*----------------------------------------------------------------------------------------------------------------------
  Used to deregister given proc domain IDs from the service registry
 ----------------------------------------------------------------------------------------------------------------------*/
ar_result_t spf_sys_util_ssr_deregister(spf_sys_util_handle_t *handle_ptr);

/*----------------------------------------------------------------------------------------------------------------------
  Parses sys queue commands and calls corresponding function pointers
 ----------------------------------------------------------------------------------------------------------------------*/
ar_result_t spf_sys_util_handle_cmd(spf_sys_util_handle_t *handle_ptr);

/*----------------------------------------------------------------------------------------------------------------------
 Generic util funciton to synchronosly push close all command to the sys queue. Waits until the thread handles close all.
 ----------------------------------------------------------------------------------------------------------------------*/
ar_result_t spf_sys_util_sync_push_close_all(spf_sys_util_handle_t *sys_util_handle_ptr,
                                             POSAL_HEAP_ID          heap_id,
                                             bool_t                 is_flush_needed,
                                             bool_t                 is_reset_needed);

/*----------------------------------------------------------------------------------------------------------------------
 Generic util funciton to push asynchronous close all command to the sys queue. Sends close all system cmd to the thread and returns.
 ----------------------------------------------------------------------------------------------------------------------*/
ar_result_t spf_sys_util_async_push_close_all(spf_sys_util_handle_t *sys_util_handle_ptr,
                                              POSAL_HEAP_ID          heap_id,
                                              bool_t                 is_flush_needed,
                                              bool_t                 is_reset_needed);

/*----------------------------------------------------------------------------------------------------------------------
 Generic util funciton to push kill command to the sys queue.
 ----------------------------------------------------------------------------------------------------------------------*/
ar_result_t spf_sys_util_push_kill_cmd(spf_sys_util_handle_t *sys_util_handle_ptr, POSAL_HEAP_ID heap_id);

#define APM_SSR_TEST_CODE 0

#if APM_SSR_TEST_CODE
void spf_svc_test_cb(spf_sys_util_handle_t *handle_ptr, bool_t status_up);
#endif

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef _SPF_SVC_REG_UTIL_H_
