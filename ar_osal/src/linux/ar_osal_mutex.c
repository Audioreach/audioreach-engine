/**
 * \file ar_osal_mutex.c
 *
 * \brief
 *      This file implements mutex apis. Recursive mutexes are always used
 *      for thread-safe programming.
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#define AR_OSAL_MUTEX_LOG_TAG     "COMU"
#include <stddef.h>
#include <stdlib.h>
#include <pthread.h>
#include <errno.h>
#include "ar_osal_mutex.h"
#include "ar_osal_log.h"
#include "ar_osal_error.h"

/* Internal lock definition */
typedef struct osal_int_mutex {
    pthread_mutex_t mutex;
} osal_int_mutex_t;

_IRQL_requires_max_(PASSIVE_LEVEL)
int32_t ar_osal_mutex_init(_Inout_ ar_osal_mutex_t mutex __unused)
{
    return AR_ENOTIMPL;
}

_IRQL_requires_max_(PASSIVE_LEVEL)
int32_t ar_osal_mutex_deinit(_In_ ar_osal_mutex_t mutex __unused)
{
    return AR_ENOTIMPL;
}

_IRQL_requires_max_(PASSIVE_LEVEL)
size_t ar_osal_mutex_get_size(void)
{
    return sizeof(osal_int_mutex_t);
}


_IRQL_requires_max_(PASSIVE_LEVEL)
int32_t ar_osal_mutex_create(_Inout_ ar_osal_mutex_t *ar_osal_mutex)
{
    int32_t rc;
    osal_int_mutex_t* the_mutex;

    if (NULL == ar_osal_mutex) {
        return AR_EBADPARAM;
    }

    the_mutex = ((osal_int_mutex_t *) malloc(sizeof(osal_int_mutex_t)));
    if (NULL == the_mutex) {
        AR_LOG_ERR(AR_OSAL_MUTEX_LOG_TAG,"%s: failed to allocate memory for mutex\n", __func__);
        rc = AR_ENOMEMORY;
        goto exit;
    }

    rc = pthread_mutex_init(&the_mutex->mutex, NULL);
    if (rc) {
        rc = AR_EFAILED;
        AR_LOG_ERR(AR_OSAL_MUTEX_LOG_TAG,"%s: failed to initialize mutex\n", __func__);
        goto fail;
    }

    *ar_osal_mutex = the_mutex;
    return 0;

fail:
    free(the_mutex);

exit:
    return rc;
}


_IRQL_requires_max_(PASSIVE_LEVEL)
int32_t ar_osal_mutex_destroy(_In_ ar_osal_mutex_t ar_osal_mutex)
{
    int32_t rc = 0;
    osal_int_mutex_t *the_mutex = ar_osal_mutex;

    if (NULL == the_mutex) {
        return AR_EBADPARAM;
    }

    rc = pthread_mutex_destroy(&the_mutex->mutex);
    if (rc) {
        AR_LOG_ERR(AR_OSAL_MUTEX_LOG_TAG,"%s: Failed to destroy mutex\n", __func__);
        rc = AR_EFAILED;
        goto exit;
    }
    free(the_mutex);

exit:
    return rc;
}

_IRQL_requires_min_(PASSIVE_LEVEL)
int32_t ar_osal_mutex_lock(_In_ ar_osal_mutex_t ar_osal_mutex)
{
    int32_t rc;
    osal_int_mutex_t *the_mutex = ar_osal_mutex;

    if (NULL == the_mutex) {
        AR_LOG_ERR(AR_OSAL_MUTEX_LOG_TAG,"%s: ar_osal_mutex is NULL\n", __func__);
        return AR_EBADPARAM;
    }

    rc = pthread_mutex_lock(&the_mutex->mutex);
    if (rc) {
        AR_LOG_ERR(AR_OSAL_MUTEX_LOG_TAG,"%s: Failed to lock ar_osal_mutex\n", __func__);
        rc = AR_EFAILED;
    }
    return rc;
}

_IRQL_requires_min_(PASSIVE_LEVEL)
int32_t ar_osal_mutex_try_lock(_In_ ar_osal_mutex_t ar_osal_mutex)
{
    int32_t rc;
    osal_int_mutex_t *the_mutex = ar_osal_mutex;

    if (NULL == the_mutex) {
        AR_LOG_ERR(AR_OSAL_MUTEX_LOG_TAG,"%s: ar_osal_mutex is NULL\n", __func__);
        return AR_EBADPARAM;
    }

    rc = pthread_mutex_trylock(&the_mutex->mutex);
    if (rc) {
        AR_LOG_ERR(AR_OSAL_MUTEX_LOG_TAG,"%s: Failed to lock ar_osal_mutex\n", __func__);
        rc = AR_EFAILED;
    }
    return rc;
}

_IRQL_requires_min_(PASSIVE_LEVEL)
int32_t ar_osal_mutex_unlock(_In_ ar_osal_mutex_t ar_osal_mutex)
{
    int32_t rc;
    osal_int_mutex_t *the_mutex = ar_osal_mutex;

    if (NULL == the_mutex) {
        AR_LOG_ERR(AR_OSAL_MUTEX_LOG_TAG,"%s: ar_osal_mutex is NULL\n", __func__);
        return AR_EBADPARAM;
    }

    rc = pthread_mutex_unlock(&the_mutex->mutex);
    if (rc) {
        AR_LOG_ERR(AR_OSAL_MUTEX_LOG_TAG,"%s: Failed to release ar_osal_mutex\n", __func__);
        rc = AR_EFAILED;
    }
    return rc;
}
