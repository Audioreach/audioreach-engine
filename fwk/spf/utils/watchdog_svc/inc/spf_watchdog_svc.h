#ifndef __SPF_WATCHDOG_SVC_H
#define __SPF_WATCHDOG_SVC_H

/**
 * \file spf_watchdog_svc.h
 * \brief
 *    This file contains functions for watchdog service.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "ar_defs.h"
#include "posal.h"
#include "spf_list_utils.h"
#include "spf_thread_pool.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/*==============================================================================
   Structure declarations
==============================================================================*/

// todo: priority should be even less than AMDB (lowest in SPF)
#define SPF_DEFAULT_WATCHDOG_SVC_PRIO (posal_thread_get_floor_prio(SPF_THREAD_STAT_CNTR_ID))

// Default Thread pool stack size
#define SPF_DEFAULT_WATCHDOG_SVC_STACK_SIZE (512)

// callback function for the job which is called by the watchdog service after timeout.
// This callback function latency should be as low as possible.
typedef ar_result_t (*spf_watchdog_svc_job_func)(void *job_context_ptr);

/* Job structure for the watchdog service.
 * Structure memory is owned by the client, and should not be released/modified while job is with watchdog service.
 */
typedef struct spf_watchdog_svc_job_t
{
   spf_watchdog_svc_job_func job_func_ptr;    // callback function for the job.
   void                     *job_context_ptr; // callback context for the job
   uint64_t                  timeout_us;      // timeout time for the job
} spf_watchdog_svc_job_t;

/*==============================================================================
   Function declarations
==============================================================================*/

/*
 * Initializes the watchdog service
 */
ar_result_t spf_watchdog_svc_init();

/*
 * De-Initializes the watchdog service
 */
ar_result_t spf_watchdog_svc_deinit();

/*
 * Push a job to watchdog service.
 * Client is responsible to keeping system out of island.
 */
ar_result_t spf_watchdog_svc_add_job(spf_watchdog_svc_job_t *job_ptr);

/*
 * Remove a job to watchdog service.
 */
ar_result_t spf_watchdog_svc_remove_job(spf_watchdog_svc_job_t *job_ptr);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /*__SPF_WATCHDOG_SVC_H*/
