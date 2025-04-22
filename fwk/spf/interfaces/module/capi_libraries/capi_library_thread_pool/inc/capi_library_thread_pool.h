/**
 * \file capi_library_thread_pool.h
 * \brief
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef CAPI_LIBRARY_THREAD_POOL_H
#define CAPI_LIBRARY_THREAD_POOL_H

/*-----------------------------------------------------------------------
 Include files
 -----------------------------------------------------------------------*/
#include "ar_error_codes.h"

/*-----------------------------------------------------------------------
 interface structures
 -----------------------------------------------------------------------*/
#define TH_POOL_TH_NAME_SIZE (4) // algo name. THPL$ is appended to input
#define TH_POOL_EXIT_TASK_ID (0) // task id for thread exit

/** thread pool handle */
typedef struct th_pool_t th_pool_t;

/** task definition */
typedef struct th_pool_task_t th_pool_task_t;

/** function input arguments definition (final definition is by client) */
typedef struct th_pool_in_args_t th_pool_in_args_t;

/** function prototype and input arguments */
typedef void (*th_pool_fn)(th_pool_in_args_t *io_args);

/** callback function definition  */
typedef void (*th_pool_cb_fn)(void *cb_contex, th_pool_task_t *task_info);

/** default callback info */
typedef struct th_pool_default_cb_t th_pool_default_cb_t;

/** final task definition */
typedef struct th_pool_task_t
{
   int32_t task_id;            // non zero task id. 0 task id is for thread exit.
   th_pool_fn         fn;      // function to be called from the thread pool
   th_pool_in_args_t *io_args; // function in/out arguments pointer
   th_pool_cb_fn cb_fn;        // callback function to be called after function
   void *cb_context;           // context pointer to be passed to callback function
} th_pool_task_t;

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/*-----------------------------------------------------------------------
 thread pool function declaration
 -----------------------------------------------------------------------*/
th_pool_t *th_pool_create(uint32_t   th_prio,
                          uint32_t   num_threads,
                          uint32_t   th_stack,
                          const char th_name[TH_POOL_TH_NAME_SIZE]);
void th_pool_destroy(th_pool_t *th_pool_handle);
ar_result_t th_pool_push_task(th_pool_t *th_pool_handle, th_pool_task_t *task_ptr);

/*-----------------------------------------------------------------------
 default callback function declaration
 -----------------------------------------------------------------------*/
th_pool_default_cb_t *th_pool_default_cb_create(void);
void th_pool_set_num_tasks_to_wait(th_pool_default_cb_t *cb_info_ptr, uint32_t num_tasks);
void th_pool_default_cb_fn(void *cb_contex, th_pool_task_t *task_info);
void th_pool_wait_for_task_complete(th_pool_default_cb_t *cb_info_ptr);
void th_pool_default_cb_destroy(th_pool_default_cb_t *cb_info_ptr);
ar_result_t th_pool_work_loop(void *context);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* CAPI_LIBRARY_THREAD_POOL_H */
