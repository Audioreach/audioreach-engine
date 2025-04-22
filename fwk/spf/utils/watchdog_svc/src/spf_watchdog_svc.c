/**
 * \file spf_watchdog_svc.c
 * \brief
 *    This file contains functions for watchdog service.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "spf_watchdog_svc.h"
#include "posal_timer.h"

/*********************************************************************************/
// global structure for watchdog svc
struct spf_thread_pool_t
{
   posal_mutex_t lock;    // lock to protect the job list
   posal_channel_t     channel_ptr;            // channel ptr used by the WD to listen on list of jobs
   posal_signal_t      signal_ptr;             // signal wake up WD

   posal_condvar_t job_cond; // condition var to set when client pushes a job

   spf_list_node_t        *job_head_ptr;    // list of all jobs
   spf_thread_pool_inst_t *thread_pool_ptr; // thread pool for watchdog service
   spf_thread_pool_job_t   tp_job;          // watchdog service routine to run on thread pool

   uint32_t wd_alive_flag : 1; // flag to kill the watchdog service routine.
} g_wd_svc;

/*********************************************************************************/

static ar_result_t spf_watchdog_svc(void *ctx_ptr)
{
   posal_mutex_lock(g_wd_svc.lock);

   while (1)
   {
      uint64_t curr_time = posal_timer_get_time();

      if (!g_wd_svc.wd_alive_flag)
      {
         posal_mutex_unlock(g_wd_svc.lock);

         return AR_EOK;
      }

      spf_list_node_t *job_list_ptr = g_wd_svc.job_head_ptr;
      while (NULL != job_list_ptr)
      {
         spf_watchdog_svc_job_t *job_ptr           = (spf_watchdog_svc_job_t *)job_list_ptr->obj_ptr;
         spf_list_node_t        *next_job_list_ptr = job_list_ptr->next_ptr;

         if (job_ptr->timeout_us <= curr_time)
         {
            AR_MSG(DBG_HIGH_PRIO,
                   "Job timedout %p, at time %llu, timeout %llu",
                   job_ptr,
                   curr_time,
                   job_ptr->timeout_us);

            if (job_ptr->job_func_ptr)
            {
               job_ptr->job_func_ptr(job_ptr->job_context_ptr);
            }
            spf_list_find_delete_node(&g_wd_svc.job_head_ptr, (void *)job_ptr, FALSE);
         }
         job_list_ptr = next_job_list_ptr;
      }

      if (!g_wd_svc.job_head_ptr)
      {
         AR_MSG(DBG_LOW_PRIO, "Watchdog waiting for job");

         // clear the signal as list is empty
         posal_signal_clear(g_wd_svc.signal_ptr);

         posal_mutex_unlock(g_wd_svc.lock);

         // wait for new jobs
         posal_channel_wait(g_wd_svc.channel_ptr, 0x1);
      }
      else
      {
         AR_MSG(DBG_LOW_PRIO, "Watchdog sleeping for %lu us", SPF_WATCHDOG_SVC_PERIOD_US);

         posal_mutex_unlock(g_wd_svc.lock);

         posal_timer_sleep(SPF_WATCHDOG_SVC_PERIOD_US); //sleep period

         AR_MSG(DBG_LOW_PRIO, "Watchdog woke up");
      }
      posal_mutex_lock(g_wd_svc.lock);

   }
}

ar_result_t spf_watchdog_svc_init()
{
   ar_result_t result = AR_EOK;
   memset(&g_wd_svc, 0, sizeof(g_wd_svc));

   // initialize the mutex
   if (AR_FAILED(result = posal_mutex_create(&g_wd_svc.lock, POSAL_HEAP_DEFAULT)))
   {
      AR_MSG(DBG_ERROR_PRIO, "spf watchdog svc init failed, result 0x%x", result);
      return result;
   }

   // initialize the channel
   if (AR_FAILED(result = posal_channel_create(&g_wd_svc.channel_ptr, POSAL_HEAP_DEFAULT)))
   {
      AR_MSG(DBG_ERROR_PRIO, "spf watchdog svc init failed, result 0x%x", result);
      posal_mutex_destroy(&g_wd_svc.lock);
      return result;
   }

   // initialize the signal
   if (AR_FAILED(result = posal_signal_create(&g_wd_svc.signal_ptr, POSAL_HEAP_DEFAULT)))
   {
      AR_MSG(DBG_ERROR_PRIO, "spf watchdog svc init failed, result 0x%x", result);
      posal_mutex_destroy(&g_wd_svc.lock);
      posal_channel_destroy(&g_wd_svc.channel_ptr);
      return result;
   }

   //add signal to the channel
   posal_channel_add_signal(g_wd_svc.channel_ptr, g_wd_svc.signal_ptr, 0x1);

   // initialize the default watchdog svc which will be reserved and not updated
   // dynamically
   result = spf_thread_pool_get_instance(&g_wd_svc.thread_pool_ptr,
                                         POSAL_HEAP_DEFAULT,
                                         SPF_DEFAULT_WATCHDOG_SVC_PRIO,
                                         TRUE, /*is_dedicated_pool*/
                                         SPF_DEFAULT_WATCHDOG_SVC_STACK_SIZE,
                                         1,
                                         0);
   if (AR_FAILED(result))
   {
      posal_mutex_destroy(&g_wd_svc.lock);
      posal_signal_destroy(&g_wd_svc.signal_ptr);
      posal_channel_destroy(&g_wd_svc.channel_ptr);
      AR_MSG(DBG_ERROR_PRIO, "spf watchdog svc init failed, result 0x%x", result);
      return result;
   }

   g_wd_svc.tp_job.job_func_ptr = spf_watchdog_svc;
   g_wd_svc.wd_alive_flag       = TRUE;

   if (AR_FAILED(result = spf_thread_pool_push_job(g_wd_svc.thread_pool_ptr, &g_wd_svc.tp_job, 0)))
   {
      posal_mutex_destroy(&g_wd_svc.lock);
      posal_signal_destroy(&g_wd_svc.signal_ptr);
      posal_channel_destroy(&g_wd_svc.channel_ptr);
      g_wd_svc.wd_alive_flag = FALSE;
      spf_thread_pool_release_instance(&g_wd_svc.thread_pool_ptr, 0);
   }

   AR_MSG(DBG_HIGH_PRIO, "Watchdog Init Done.");

   return AR_EOK;
}

ar_result_t spf_watchdog_svc_deinit()
{
   ar_result_t result = AR_EOK;

   posal_mutex_lock(g_wd_svc.lock);

   // remove all the jobs
   spf_list_delete_list(&g_wd_svc.job_head_ptr, FALSE);

   g_wd_svc.wd_alive_flag = FALSE;

   // wake up the watchdog service
   posal_signal_send(g_wd_svc.signal_ptr);

   posal_mutex_unlock(g_wd_svc.lock);

   // release thread pool instance
   spf_thread_pool_release_instance(&g_wd_svc.thread_pool_ptr, 0);

   // destroy mutex
   posal_mutex_destroy(&g_wd_svc.lock);

   // destroy signal
   posal_signal_destroy(&g_wd_svc.signal_ptr);

   // destroy channel
   posal_channel_destroy(&g_wd_svc.channel_ptr);

   return result;
}

ar_result_t spf_watchdog_svc_add_job(spf_watchdog_svc_job_t *job_ptr)
{
   ar_result_t result = AR_EOK;

   if (!g_wd_svc.wd_alive_flag)
   {
      return result;
   }

   posal_mutex_lock(g_wd_svc.lock);

   // insert the job in WD job list
   spf_list_insert_head(&g_wd_svc.job_head_ptr, (void *)job_ptr, POSAL_HEAP_DEFAULT, FALSE);

   AR_MSG(DBG_LOW_PRIO, "Watchdog job pushed %p", job_ptr);

   // wake up the watchdog service
   posal_signal_send(g_wd_svc.signal_ptr);

   posal_mutex_unlock(g_wd_svc.lock);

   return result;
}

ar_result_t spf_watchdog_svc_remove_job(spf_watchdog_svc_job_t *job_ptr)
{
   ar_result_t result = AR_EOK;

   if (!g_wd_svc.wd_alive_flag)
   {
      return result;
   }

   posal_mutex_lock(g_wd_svc.lock);

   // remove the job from WD job list
   spf_list_find_delete_node(&g_wd_svc.job_head_ptr, (void *)job_ptr, FALSE);

   AR_MSG(DBG_LOW_PRIO, "Watchdog job removed %p", job_ptr);

   posal_mutex_unlock(g_wd_svc.lock);

   return result;
}
