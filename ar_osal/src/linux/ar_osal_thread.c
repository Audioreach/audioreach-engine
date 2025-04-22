/**
 * \file ar_osal_thread.c
 *
 * \brief
 *      This file has implementation of thread related helper functions.
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#define _GNU_SOURCE
#define AR_OSAL_THREAD_LOG_TAG  "COTL"
#include <stdlib.h>
#include <errno.h>
#include <pthread.h>
#include <string.h>
#include "ar_osal_thread.h"
#include "ar_osal_log.h"
#include "ar_osal_error.h"

typedef struct osal_int_thread {
    pthread_t pthread_handle;
    ar_osal_thread_start_routine fn;
    void* param;
} osal_int_thread_t;

static void* osal_thread_wrapper(void *param)
{
    osal_int_thread_t *the_thread = (osal_int_thread_t *)param;
    if (NULL != the_thread)
        the_thread->fn(the_thread->param);
    return NULL;
}


_IRQL_requires_max_(PASSIVE_LEVEL)
int32_t ar_osal_thread_attr_init(_In_ ar_osal_thread_attr_t *thread_attr)
{
    int32_t rc = 0;
    pthread_attr_t attr;
    struct sched_param param;
    size_t size;
    if (NULL == thread_attr)
    {
        rc = AR_EBADPARAM;
        goto done;
    }

    rc = pthread_attr_init(&attr);
    if (rc) {
        AR_LOG_ERR(AR_OSAL_THREAD_LOG_TAG,"%s: Failed to init attributes, rc = %d\n", __func__, rc);
        rc = AR_EFAILED;
        goto done;
    }

    rc = pthread_attr_getschedparam(&attr, &param);
    if (rc) {
        AR_LOG_ERR(AR_OSAL_THREAD_LOG_TAG,"%s: Failed to init attributes, rc = %d\n", __func__, rc);
        rc = AR_EFAILED;
        goto done;
    }

    rc = pthread_attr_getstacksize(&attr, &size);
    if (rc) {
        AR_LOG_ERR(AR_OSAL_THREAD_LOG_TAG,"%s: Failed to init attributes, rc = %d\n", __func__, rc);
        rc = AR_EFAILED;
        goto done;
    }
    thread_attr->thread_name = NULL;
    thread_attr->priority = param.sched_priority;
    thread_attr->stack_size = size;

done:
    return rc;
}

_IRQL_requires_max_(PASSIVE_LEVEL)
int32_t ar_osal_thread_create(_Out_  ar_osal_thread_t *ret_thread,
                                 _In_ ar_osal_thread_attr_t *attr_ptr,
                                 _In_ ar_osal_thread_start_routine osal_thread_start,
                                 _In_opt_ void *osal_thread_param)
{
    int32_t rc;
    osal_int_thread_t *the_thread;
    pthread_attr_t attr;
    void *ret_val;
    struct sched_param sch_param;

    if (NULL == ret_thread || NULL == attr_ptr) {
        AR_LOG_ERR(AR_OSAL_THREAD_LOG_TAG,"%s: ret_thread or attr_ptr is NULL\n", __func__);
        rc = AR_EBADPARAM;
        goto done;
    }

    the_thread = (osal_int_thread_t *) malloc(sizeof(osal_int_thread_t));
    if (NULL == the_thread) {
        AR_LOG_ERR(AR_OSAL_THREAD_LOG_TAG,"%s: Failed to allocate memory\n", __func__);
        rc = AR_ENOMEMORY;
        goto done;
    }

    rc = pthread_attr_init(&attr);
    if (rc) {
        AR_LOG_ERR(AR_OSAL_THREAD_LOG_TAG,"%s: Failed to init attributes, rc = %d\n", __func__, rc);
        rc = AR_EFAILED;
        goto err_attr;
    }
     rc = pthread_attr_setstacksize(&attr, attr_ptr->stack_size);
     if (rc) {
        AR_LOG_ERR(AR_OSAL_THREAD_LOG_TAG,"%s: Failed to set stack size, rc = %d\n", __func__, rc);
        rc = AR_EFAILED;
        goto err_attr;
    }

    memset(&sch_param, 0, sizeof(sch_param));
    sch_param.sched_priority = attr_ptr->priority;
    rc = pthread_attr_setschedparam (&attr, &sch_param);
    if (rc) {
        AR_LOG_ERR(AR_OSAL_THREAD_LOG_TAG,"%s: Failed to set thread priority , rc = %d\n", __func__, rc);
        rc = AR_EFAILED;
        goto err_set;
    }

    rc = pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
    if (rc) {
        AR_LOG_ERR(AR_OSAL_THREAD_LOG_TAG,"%s: Failed to set detach state, rc = %d\n", __func__, rc);
        rc = AR_EFAILED;
        goto err_set;
    }
    the_thread->fn = osal_thread_start;
    the_thread->param = osal_thread_param;
    rc = pthread_create(&the_thread->pthread_handle, &attr,
                      osal_thread_wrapper, the_thread);
    if (rc) {
        AR_LOG_ERR(AR_OSAL_THREAD_LOG_TAG,"%s: Failed to create thread, rc = %d\n", __func__, rc);
        rc = AR_EFAILED;
        goto err_set;
    }

    if (attr_ptr->thread_name){
        rc = pthread_setname_np(the_thread->pthread_handle, attr_ptr->thread_name);
        if (rc) {
            AR_LOG_ERR(AR_OSAL_THREAD_LOG_TAG,"%s: Failed to create thread, rc = %d\n", __func__, rc);
            rc = AR_EFAILED;
            goto err_destroy;
        }
    }

    rc = pthread_attr_destroy(&attr);
    if (rc) {
        AR_LOG_ERR(AR_OSAL_THREAD_LOG_TAG,"%s: Failed to destroy attributes, rc = %d\n", __func__, rc);
        rc = AR_EFAILED;
        goto err_destroy;
    }

    *ret_thread = (ar_osal_thread_t *)the_thread;
    goto done;

err_destroy:
    pthread_join(the_thread->pthread_handle, &ret_val);

err_set:
    pthread_attr_destroy(&attr);

err_attr:
    free(the_thread);

done:
    return rc;
}

_IRQL_requires_max_(PASSIVE_LEVEL)
int32_t ar_osal_thread_join_destroy(_In_ ar_osal_thread_t thread)
{
    int32_t rc = 0;
    osal_int_thread_t *the_thread;
    void *ret_val;

    if (NULL == thread) {
        AR_LOG_ERR(AR_OSAL_THREAD_LOG_TAG,"%s: Thread is NULL\n", __func__);
        rc = AR_EBADPARAM;
        goto done;
    }
    the_thread = (osal_int_thread_t *)thread;
    rc = pthread_join(the_thread->pthread_handle, &ret_val);
    if (rc) {
        AR_LOG_ERR(AR_OSAL_THREAD_LOG_TAG,"%s: Failed to join the thread, rc = %d\n", __func__, rc);
        rc = AR_EFAILED;
    }

    free(the_thread);

done:
    return rc;
}

static int32_t set_thread_prioirity(pthread_t *pthread_handle, int32_t new_thread_priority)
{
    int32_t rc = 0;
    int policy = 0;
    struct sched_param sch_param;

    memset(&sch_param, 0, sizeof(sch_param));
    rc = pthread_getschedparam(*pthread_handle, &policy, &sch_param);
    if (rc) {
        AR_LOG_ERR(AR_OSAL_THREAD_LOG_TAG,"%s: Failed to get thread scheduling params , rc = %d\n", __func__, rc);
        rc = AR_EFAILED;
        goto done;
    }
    sch_param.sched_priority = new_thread_priority;
    rc = pthread_setschedparam(*pthread_handle, policy, &sch_param);
    if (rc)
    {
        AR_LOG_ERR(AR_OSAL_THREAD_LOG_TAG,"%s: Failed to set new thread prioritypriority rc:%d\n", __func__, rc);
        rc = AR_EFAILED;
        goto done;
    }

done:
    return rc;
}

_IRQL_requires_min_(PASSIVE_LEVEL)
int32_t ar_osal_thread_set_priority(_In_ ar_osal_thread_t thread, _In_ int32_t new_thread_priority)
{
    int32_t rc = 0;
    osal_int_thread_t *the_thread;

    if (NULL == thread) {
        AR_LOG_ERR(AR_OSAL_THREAD_LOG_TAG,"%s: Thread is NULL\n", __func__);
        rc = AR_EBADPARAM;
        goto done;
    }

    the_thread = (osal_int_thread_t *)thread;
    rc = set_thread_prioirity(&the_thread->pthread_handle, new_thread_priority);

done:
    return rc;
}

_IRQL_requires_min_(PASSIVE_LEVEL)
int32_t ar_osal_thread_self_set_priority(_In_ int32_t new_thread_priority)
{
    int32_t rc = 0;
    pthread_t pthread_handle = pthread_self();

    rc = set_thread_prioirity(&pthread_handle, new_thread_priority);

    return rc;
}

_IRQL_requires_max_(PASSIVE_LEVEL)
int32_t ar_osal_thread_self_get_priority(_Inout_ int32_t *ret_thread_priority)
{
    int32_t rc = 0;
    pthread_t pthread_handle = pthread_self();
    struct sched_param sch_param;
    int policy;

    memset(&sch_param, 0, sizeof(sch_param));
    rc = pthread_getschedparam(pthread_handle, &policy, &sch_param);
    if (rc) {
        AR_LOG_ERR(AR_OSAL_THREAD_LOG_TAG,"%s: Failed to get thread priority , rc = %d\n", __func__, rc);
        rc = AR_EFAILED;
        goto done;
    }

    *ret_thread_priority = sch_param.sched_priority;

done:
    return rc;
}

_IRQL_requires_max_(PASSIVE_LEVEL)
int32_t ar_osal_thread_get_priority(_In_ ar_osal_thread_t thread, _Inout_ int32_t *ret_thread_priority)
{
    int32_t rc = 0;
    osal_int_thread_t *the_thread;
    struct sched_param sch_param;
    int policy;

    if (NULL == thread) {
        AR_LOG_ERR(AR_OSAL_THREAD_LOG_TAG,"%s: Thread is NULL\n", __func__);
        rc = AR_EBADPARAM;
        goto done;
    }

    the_thread = (osal_int_thread_t *)thread;

    memset(&sch_param, 0, sizeof(sch_param));
    rc = pthread_getschedparam (the_thread->pthread_handle, &policy, &sch_param);
    if (rc) {
        AR_LOG_ERR(AR_OSAL_THREAD_LOG_TAG,"%s: Failed to get thread priority , rc = %d\n", __func__, rc);
        rc = AR_EFAILED;
        goto done;
    }

    *ret_thread_priority = sch_param.sched_priority;

done:
    return rc;
}

_IRQL_requires_min_(PASSIVE_LEVEL)
int64_t ar_osal_thread_get_id()
{
    return (int64_t)pthread_self();
}

int32_t ar_osal_thread_self_terminate()
{
    AR_LOG_ERR(AR_OSAL_THREAD_LOG_TAG,"%s: Not Implemented\n", __func__);
    return AR_ENOTIMPL;
}
