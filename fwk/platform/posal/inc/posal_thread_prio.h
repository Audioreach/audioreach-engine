/**
 * \file posal_thread_prio.h
 * \brief
 *  This file contains the structures and function declarations that will be exposed to the framework
 *  to be invoked in order to retrieve the thread priority.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef POSAL_THREAD_PRIO_H
#define POSAL_THREAD_PRIO_H

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

#include "ar_error_codes.h"
#include "posal_thread.h"
/* =======================================================================
 Type Declarations
========================================================================== */

typedef enum spf_thread_prio_id_t
{
   SPF_THREAD_DYN_ID = 0,
   SPF_THREAD_STAT_APM_ID,
   SPF_THREAD_STAT_CNTR_ID,
   SPF_THREAD_STAT_AMDB_ID,
   SPF_THREAD_STAT_IST_ID,
   SPF_THREAD_STAT_PRM_ID,
   SPF_THREAD_STAT_PM_SERVER_ID,
   SPF_THREAD_STAT_VOICE_TIMER_ID,
   SPF_THREAD_STAT_VCPM_ID,
   SPF_THREAD_STAT_ASPS_ID,
   SPF_THREAD_STAT_DLS_ID,
   SPF_THREAD_STAT_ID_MAX
} spf_thread_prio_id_t;

/**
 * prio_query_t struct holds a (static_req_id, thread priority, frame duration)
   indicating the correct thread priority for a given frame duration
 * (measured in microseconds) OR a req ID (indicates static/dynamic).
 */
typedef struct prio_query_t
{
   bool_t                 is_interrupt_trig;
   spf_thread_prio_id_t   static_req_id;
   uint32_t               frame_duration_us;
} prio_query_t;

/* =======================================================================
 Function Declarations
========================================================================== */
ar_result_t posal_thread_calc_prio(prio_query_t *prio_query_ptr, posal_thread_prio_t *thread_prio_ptr);

ar_result_t posal_thread_determine_attributes(prio_query_t *prio_query_ptr, posal_thread_prio_t *thread_prio_ptr, uint32_t *sched_policy_ptr, uint32_t *cpu_set_ptr);

/** Gets a default low priority for the passed in prio_id. Currently only implemented
    for SPF_THREAD_STAT_CNTR_ID. */
posal_thread_prio_t posal_thread_get_floor_prio(spf_thread_prio_id_t prio_id);

#ifdef __cplusplus
}
#endif //__cplusplus
#endif // POSAL_THREAD_PRIO_H
