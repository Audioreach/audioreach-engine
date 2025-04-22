/**
 * \file amdb_parallel_loader.c
 * \brief
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "amdb_parallel_loader.h"
#include "posal_thread.h"
#include "posal_atomic.h"
#include "amdb_queue.h"
#include "amdb_resource_voter.h"
#ifdef AMDB_MODULE_LOADING_TIMEOUT_US
#include "spf_watchdog_svc.h"
#include "posal_err_fatal.h"
#endif
static ar_result_t amdb_loader_thread_entry_function(void *context);

static const uint32_t AMDB_THREAD_STACK_SIZE = 8 * (1 << 10); // 4K stack is needed for dlopen according to the platform
                                                              // team. Keeping stack as 8k till ADSPPM team gives a
                                                              // proper number.
static const uint32_t AMDB_TASK_Q_LENGTH      = 64;           // Must be a power of 2.
#define AMDB_NUM_CLIENT_HANDLES 8            // Must be a power of 2.

typedef struct amdb_parallel_loader amdb_parallel_loader;
typedef struct amdb_loader_handle   amdb_loader_handle;

struct amdb_loader_handle
{
   amdb_get_modules_callback_f cb;
   void *                      context_ptr;
   amdb_loader_load_function   load_function;
   posal_atomic_word_t         refs;
   amdb_parallel_loader *      parallel_loader_ptr;
};

struct amdb_parallel_loader
{
   uint32_t           num_threads;
   posal_thread_t     threads[AMDB_MAX_THREADS];
   amdb_queue_t *     task_queue_ptr;
   amdb_queue_t *     handle_queue_ptr;
   amdb_loader_handle handles[AMDB_NUM_CLIENT_HANDLES];
   void *             voter_ptr;
};

static void amdb_loader_addref_handle(amdb_loader_handle *loader_handle_ptr);

typedef enum amdb_loader_task_type
{
   DLOPEN_TASK,
   THREAD_EXIT,
   VOTE,
   RELEASE_VOTE
} amdb_loader_task_type;

typedef struct amdb_loader_task
{
   amdb_loader_task_type type;
   union
   {
      struct dlopen_task_t
      {
         amdb_loader_handle *client_handle;
         uint64_t            task_info;
      } dlopen_task;
      // Other task structures can be added here.
   } payload;
} amdb_loader_task;

/*---------------------------------------------------------------------------------------------------------------------
  IN  : None
  OUT : None

  --------------------------------------------------------------------------------------------------------------------*/
void *amdb_loader_create(POSAL_HEAP_ID heap_id)
{
   AR_MSG(DBG_HIGH_PRIO, "AMDB: Creating parallel Loader");
   amdb_parallel_loader *obj_ptr = (amdb_parallel_loader *)posal_memory_malloc(sizeof(amdb_parallel_loader), heap_id);
   if (NULL == obj_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "amdb: Failed to allocate the amdb parallel loader.");
      return NULL;
   }

   obj_ptr->num_threads      = 0;
   obj_ptr->task_queue_ptr   = NULL;
   obj_ptr->handle_queue_ptr = NULL;
   obj_ptr->voter_ptr        = (void *)NULL;

   char task_q_name[]      = "AMDBTQ";
   obj_ptr->task_queue_ptr = amdb_queue_create(AMDB_TASK_Q_LENGTH, sizeof(amdb_loader_task), task_q_name, heap_id);
   if (NULL == obj_ptr->task_queue_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "amdb: Failed to allocate the task queue for the amdb parallel loader.");
      amdb_loader_destroy((void *)obj_ptr);
      return NULL;
   }

   char handle_q_name[] = "AMDBHQ";
   obj_ptr->handle_queue_ptr =
      amdb_queue_create(AMDB_NUM_CLIENT_HANDLES, sizeof(amdb_loader_handle *), handle_q_name, heap_id);
   if (NULL == obj_ptr->handle_queue_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "amdb: Failed to allocate the handle queue for the amdb parallel loader.");
      amdb_loader_destroy((void *)obj_ptr);
      return NULL;
   }

   for (uint32_t i = 0; i < sizeof(obj_ptr->handles) / sizeof(obj_ptr->handles[0]); i++)
   {
      obj_ptr->handles[i].cb          = NULL;
      obj_ptr->handles[i].context_ptr = NULL;

      posal_atomic_set(&obj_ptr->handles[i].refs, 0);

      amdb_loader_handle *h = &obj_ptr->handles[i];

      amdb_queue_push(obj_ptr->handle_queue_ptr, &h);
   }

   obj_ptr->voter_ptr = amdb_voter_create(heap_id);
   if (NULL == obj_ptr->voter_ptr)
   {
      amdb_loader_destroy(obj_ptr);
      return NULL;
   }

   ar_result_t         res              = AR_EOK;
   posal_thread_prio_t amdb_thread_prio = 0;
   uint32_t sched_policy = 0, affinity_mask = 0;

   prio_query_t query_tbl;
   query_tbl.frame_duration_us = 0;
   query_tbl.is_interrupt_trig = FALSE;
   query_tbl.static_req_id     = SPF_THREAD_STAT_AMDB_ID;

   res = posal_thread_determine_attributes(&query_tbl, &amdb_thread_prio, &sched_policy, &affinity_mask);
   if (AR_DID_FAIL(res))
   {
      AR_MSG(DBG_HIGH_PRIO, "amdb: Failed to get thread priority");
      return NULL;
   }

   // Note: Threads should be launched only after everything else is initialized, since they may access the object.
   char thread_name[] = "AMDB5";
   for (uint32_t i = 0; i < AMDB_MAX_THREADS; i++)
   {
      res = posal_thread_launch3(&obj_ptr->threads[i],
                                thread_name,
                                AMDB_THREAD_STACK_SIZE,
                                0,
                                amdb_thread_prio,
                                amdb_loader_thread_entry_function,
                                obj_ptr,
                                heap_id,
                                sched_policy,
                                affinity_mask);
      if (AR_DID_FAIL(res))
      {
         AR_MSG(DBG_HIGH_PRIO, "amdb: Failed to launch thread %lu.", i);
         break;
      }

      obj_ptr->num_threads++;
      thread_name[4]++;
   }

   if (0 == obj_ptr->num_threads)
   {
      // No thread could be launched.
      AR_MSG(DBG_ERROR_PRIO, "amdb: Failed to launch any background thread.");
      amdb_loader_destroy(obj_ptr);
      return NULL;
   }

   return (void *)obj_ptr;
}

/*---------------------------------------------------------------------------------------------------------------------
  IN  : None
  OUT : None

  --------------------------------------------------------------------------------------------------------------------*/
void *amdb_loader_get_handle(void *                      vobj_ptr,
                             amdb_get_modules_callback_f callback_f,
                             void *                      context_ptr,
                             amdb_loader_load_function   load_function)
{
   if (NULL == callback_f)
   {
      AR_MSG(DBG_ERROR_PRIO, "amdb: NULL callback function in the parallel loader.");
      return NULL;
   }

   amdb_parallel_loader *obj_ptr = (amdb_parallel_loader *)vobj_ptr;
   amdb_loader_handle *  h       = NULL;
   amdb_queue_pop(obj_ptr->handle_queue_ptr, &h);
   h->cb                  = callback_f;
   h->context_ptr         = context_ptr;
   h->load_function       = load_function;
   h->parallel_loader_ptr = obj_ptr;
   posal_atomic_set(&h->refs, 1);

   // Vote for resources for dlopen
   amdb_loader_task task;
   task.type = VOTE;
   amdb_queue_push(obj_ptr->task_queue_ptr, &task);

   AR_MSG(DBG_HIGH_PRIO, "amdb: Started parallel loading for handle 0x%p", h);
   return (void *)h;
}

/*---------------------------------------------------------------------------------------------------------------------
  IN  : None
  OUT : None

  --------------------------------------------------------------------------------------------------------------------*/
void amdb_loader_push_task(void *loader_vhandle_ptr, uint64_t task_info)
{
   amdb_loader_handle *loader_handle_ptr = (amdb_loader_handle *)loader_vhandle_ptr;
   amdb_loader_addref_handle(loader_handle_ptr);
   amdb_loader_task task;
   task.type                              = DLOPEN_TASK;
   task.payload.dlopen_task.task_info     = task_info;
   task.payload.dlopen_task.client_handle = loader_handle_ptr;

   amdb_queue_push(loader_handle_ptr->parallel_loader_ptr->task_queue_ptr, &task);
}

/*---------------------------------------------------------------------------------------------------------------------
  IN  : None
  OUT : None

  --------------------------------------------------------------------------------------------------------------------*/
static void amdb_loader_addref_handle(amdb_loader_handle *loader_handle_ptr)
{
   posal_atomic_increment(&loader_handle_ptr->refs);
}

/*---------------------------------------------------------------------------------------------------------------------
  IN  : None
  OUT : None

  --------------------------------------------------------------------------------------------------------------------*/
void amdb_loader_release_handle(void *loader_vhandle_ptr)
{
   amdb_loader_handle *loader_handle_ptr = (amdb_loader_handle *)loader_vhandle_ptr;
   int                 value             = posal_atomic_decrement(&loader_handle_ptr->refs);
   if (0 == value)
   {
      AR_MSG(DBG_HIGH_PRIO, "amdb: Finished parallel loading for client 0x%p", loader_handle_ptr);

      loader_handle_ptr->cb(loader_handle_ptr->context_ptr);

      amdb_queue_push(loader_handle_ptr->parallel_loader_ptr->handle_queue_ptr, &loader_handle_ptr);

      // Release vote for resources
      amdb_loader_task task;
      task.type = RELEASE_VOTE;
      amdb_queue_push(loader_handle_ptr->parallel_loader_ptr->task_queue_ptr, &task);
   }
}

/*---------------------------------------------------------------------------------------------------------------------
  IN  : None
  OUT : None

  --------------------------------------------------------------------------------------------------------------------*/
void amdb_loader_destroy(void *vobj_ptr)
{
   amdb_parallel_loader *obj_ptr = (amdb_parallel_loader *)vobj_ptr;
   if (NULL != obj_ptr)
   {

      for (uint32_t i = 0; i < obj_ptr->num_threads; i++)
      {
         amdb_loader_task task;
         task.type = THREAD_EXIT;
         amdb_queue_push(obj_ptr->task_queue_ptr, &task);
      }

      for (uint32_t i = 0; i < obj_ptr->num_threads; i++)
      {
         ar_result_t status = 0;
         posal_thread_join(obj_ptr->threads[i], &status);
      }

      if (NULL != obj_ptr->task_queue_ptr)
      {
         amdb_queue_destroy(obj_ptr->task_queue_ptr);
         obj_ptr->task_queue_ptr = NULL;
      }

      if (NULL != obj_ptr->handle_queue_ptr)
      {
         amdb_queue_destroy(obj_ptr->handle_queue_ptr);
         obj_ptr->handle_queue_ptr = NULL;
      }

      if (NULL != obj_ptr->voter_ptr)
      {
         amdb_voter_destroy(obj_ptr->voter_ptr);
         obj_ptr->voter_ptr = (void *)NULL;
      }

      posal_memory_free(obj_ptr);
   }
}

/*---------------------------------------------------------------------------------------------------------------------
  IN  : None
  OUT : None

  --------------------------------------------------------------------------------------------------------------------*/
#ifdef AMDB_MODULE_LOADING_TIMEOUT_US
static ar_result_t amdb_loader_timeout(void* ctx_ptr)
{
   amdb_module_handle_info_t *h_ptr = (amdb_module_handle_info_t*)ctx_ptr;

   AR_MSG(DBG_HIGH_PRIO, "module_loading 0x%x timed out", h_ptr->module_id);
   posal_err_fatal("Forcing Crash.");

   return AR_EOK;
}
#endif
static ar_result_t amdb_loader_thread_entry_function(void *context)
{
   amdb_parallel_loader *obj_ptr = (amdb_parallel_loader *)(context);
#ifdef AMDB_MODULE_LOADING_TIMEOUT_US
   spf_watchdog_svc_job_t wd_job = {.job_func_ptr = &amdb_loader_timeout};
#endif
   while (1)
   {
      // Pop from the queue.
      amdb_loader_task task;
      amdb_queue_pop(obj_ptr->task_queue_ptr, &task);

      //#ifdef AMDB_TEST
      //      extern volatile bool_t stall_background_threads;
      //      while (stall_background_threads)
      //         ;
      //#endif

      switch (task.type)
      {
         case DLOPEN_TASK:
         {
#ifdef AMDB_MODULE_LOADING_TIMEOUT_US
            wd_job.job_context_ptr = (void*)task.payload.dlopen_task.task_info;
            wd_job.timeout_us = posal_timer_get_time() + AMDB_MODULE_LOADING_TIMEOUT_US;
            spf_watchdog_svc_add_job(&wd_job);
#endif
            task.payload.dlopen_task.client_handle->load_function(task.payload.dlopen_task.task_info);
#ifdef AMDB_MODULE_LOADING_TIMEOUT_US
            spf_watchdog_svc_remove_job(&wd_job);
#endif
            amdb_loader_release_handle((void *)task.payload.dlopen_task.client_handle);
            break;
         }
         case THREAD_EXIT:
            return 0;
         case VOTE:
            amdb_voter_vote(obj_ptr->voter_ptr);
            break;
         case RELEASE_VOTE:
            amdb_voter_release(obj_ptr->voter_ptr);
            break;
         default:
            // Unknown opcode; can't do anything here.
            break;
      }
   }
   return 0;
}
