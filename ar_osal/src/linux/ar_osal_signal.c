/**
 * \file ar_osal_signal.c
 *
 * \brief
 *      This file defines signal variable implementaion.
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#define AR_OSAL_SIGNAL_LOG_TAG   "COSI"
#include <stdlib.h>
#include <time.h>
#include <errno.h>
#include <pthread.h>
#include <stdbool.h>
#include "ar_osal_signal.h"
#include "ar_osal_log.h"
#include "ar_osal_error.h"

/* Internal condition definition */
typedef struct osal_int_signal {
	pthread_mutex_t osal_mutex;
	pthread_cond_t osal_cond;
	bool signalled;
} osal_int_signal_t;

_IRQL_requires_max_(DISPATCH_LEVEL)
int32_t ar_osal_signal_init(_In_ ar_osal_signal_t signal __unused)
{
    return AR_ENOTIMPL;
}

_IRQL_requires_max_(DISPATCH_LEVEL)
int32_t ar_osal_signal_deinit(_In_ ar_osal_signal_t signal __unused)
{
    return AR_ENOTIMPL;
}

_IRQL_requires_max_(DISPATCH_LEVEL)
size_t ar_osal_signal_get_size()
{
    return sizeof(osal_int_signal_t);
}


_IRQL_requires_max_(DISPATCH_LEVEL)
int32_t ar_osal_signal_create(_Inout_ ar_osal_signal_t *ret_signal)
{
    int32_t rc;
    osal_int_signal_t *the_signal;
    pthread_condattr_t attr;
    if (NULL == ret_signal) {
        AR_LOG_ERR(AR_OSAL_SIGNAL_LOG_TAG,"%s: signal is NULL\n", __func__);
        rc = AR_EBADPARAM;
        goto done;
    }
    the_signal = (osal_int_signal_t *)malloc(sizeof(osal_int_signal_t));
    if (NULL == the_signal) {
        AR_LOG_ERR(AR_OSAL_SIGNAL_LOG_TAG,"%s: failed to allocate signal memory\n", __func__);
        rc = AR_ENOMEMORY;
        goto done;
    }

    rc = pthread_mutex_init(&the_signal->osal_mutex, NULL);
    if (rc) {
        AR_LOG_ERR(AR_OSAL_SIGNAL_LOG_TAG,"%s: Failed to init mutex, rc = %d\n", __func__, rc);
        rc = AR_EFAILED;
        goto err_mutex;
    }
    pthread_condattr_init(&attr);
    pthread_condattr_setclock(&attr, CLOCK_MONOTONIC);
    rc = pthread_cond_init(&the_signal->osal_cond, &attr);
    if (rc) {
        AR_LOG_ERR(AR_OSAL_SIGNAL_LOG_TAG,"%s: Failed to init cond, rc = %d\n", __func__, rc);
        rc = AR_EFAILED;
        goto err_cond;
    }
    the_signal->signalled = false;
    *ret_signal = the_signal;
    goto done;
err_cond:
    pthread_mutex_destroy(&the_signal->osal_mutex);
err_mutex:
    free(the_signal);
done:
   return rc;
}


_IRQL_requires_max_(PASSIVE_LEVEL)
int32_t ar_osal_signal_wait(_In_ ar_osal_signal_t signal)
{
    int32_t rc;
    osal_int_signal_t *the_signal;
    if (NULL == signal) {
        AR_LOG_ERR(AR_OSAL_SIGNAL_LOG_TAG,"%s: signal is NULL\n", __func__);
        rc = AR_EBADPARAM;
        goto done;
    }
    the_signal = (osal_int_signal_t *)signal;
    rc = pthread_mutex_lock(&the_signal->osal_mutex);
    if (rc) {
        AR_LOG_ERR(AR_OSAL_SIGNAL_LOG_TAG,"%s: Failed to acquire lock, rc = %d\n", __func__, rc);
        rc = AR_EFAILED;
        goto done;
    }
    if (true == the_signal->signalled)
    {
        rc = pthread_mutex_unlock(&the_signal->osal_mutex);
        if (rc) {
            AR_LOG_ERR(AR_OSAL_SIGNAL_LOG_TAG,"%s: Failed to unlock, rc = %d\n", __func__, rc);
            rc = AR_EFAILED;
        }
        goto done;
    }

    rc = pthread_cond_wait(&the_signal->osal_cond, &the_signal->osal_mutex);
    if (rc) {
        AR_LOG_ERR(AR_OSAL_SIGNAL_LOG_TAG,"%s: Failed to wait on signal, rc = %d\n", __func__, rc);
        rc = AR_EFAILED;
        goto err_cond;
    }

    rc = pthread_mutex_unlock(&the_signal->osal_mutex);
    if (rc) {
        AR_LOG_ERR(AR_OSAL_SIGNAL_LOG_TAG,"%s: Failed to unlock, rc = %d\n", __func__, rc);
        rc = AR_EFAILED;
        goto done;
    }
err_cond:
    pthread_mutex_unlock(&the_signal->osal_mutex);
done:
    return rc;
}

_IRQL_requires_max_(PASSIVE_LEVEL)
int32_t ar_osal_signal_timedwait(_In_ ar_osal_signal_t signal, _In_ int64_t timeout_in_nsec)
{
    int32_t rc;
    osal_int_signal_t *the_signal;
    struct timespec osal_ts;

    if (NULL == signal) {
        AR_LOG_ERR(AR_OSAL_SIGNAL_LOG_TAG,"%s: signal is NULL\n", __func__);
        rc = AR_EBADPARAM;
        goto done;
    }

    clock_gettime(CLOCK_MONOTONIC, &osal_ts);
    osal_ts.tv_sec += (timeout_in_nsec / 1000000000);
    osal_ts.tv_nsec += (timeout_in_nsec % 1000000000);

    if (osal_ts.tv_nsec >= 1000000000) {
        osal_ts.tv_sec += 1;
        osal_ts.tv_nsec -= 1000000000;
    }

    the_signal = (osal_int_signal_t *)signal;
    rc = pthread_mutex_lock(&the_signal->osal_mutex);
    if (rc) {
        AR_LOG_ERR(AR_OSAL_SIGNAL_LOG_TAG,"%s: Failed to acquire lock, rc = %d\n", __func__, rc);
        rc = AR_EFAILED;
        goto done;
    }

    if (true == the_signal->signalled) {
        rc = pthread_mutex_unlock(&the_signal->osal_mutex);
        if (rc) {
            AR_LOG_ERR(AR_OSAL_SIGNAL_LOG_TAG,"%s: Failed to unlock, rc = %d\n", __func__, rc);
            rc = AR_EFAILED;
        }
        goto done;
    }

    rc = pthread_cond_timedwait(&the_signal->osal_cond, &the_signal->osal_mutex, &osal_ts);
    if (rc) {

        AR_LOG_ERR(AR_OSAL_SIGNAL_LOG_TAG,"%s: Failed to wait on signal, rc = %d\n", __func__, rc);
        rc = AR_EFAILED;
        goto err_cond;
    }

    rc = pthread_mutex_unlock(&the_signal->osal_mutex);
    if (rc) {
        AR_LOG_ERR(AR_OSAL_SIGNAL_LOG_TAG,"%s: Failed to unlock, rc = %d\n", __func__, rc);
        rc = AR_EFAILED;
    }
err_cond:
    pthread_mutex_unlock(&the_signal->osal_mutex);
done:
    return rc;
}

_IRQL_requires_max_(DISPATCH_LEVEL)
int32_t ar_osal_signal_set(_In_ ar_osal_signal_t signal)
{
    int32_t rc;
    osal_int_signal_t *the_signal;

    if (NULL == signal) {
        AR_LOG_ERR(AR_OSAL_SIGNAL_LOG_TAG,"%s: signal is NULL\n", __func__);
        rc = AR_EBADPARAM;
        goto done;
    }

    the_signal = (osal_int_signal_t *)signal;
    rc = pthread_mutex_lock(&the_signal->osal_mutex);
    if (rc) {
        AR_LOG_ERR(AR_OSAL_SIGNAL_LOG_TAG,"%s: Failed to acquire lock, rc = %d\n", __func__, rc);
        rc = AR_EFAILED;
        goto done;
    }
    the_signal->signalled = true;

    rc = pthread_cond_broadcast(&the_signal->osal_cond);
    if (rc) {
        AR_LOG_ERR(AR_OSAL_SIGNAL_LOG_TAG,"%s: Failed to signal on signal, rc = %d\n", __func__, rc);
        rc = AR_EFAILED;
		goto err_cond;
    }

    rc = pthread_mutex_unlock(&the_signal->osal_mutex);
    if (rc) {
        AR_LOG_ERR(AR_OSAL_SIGNAL_LOG_TAG,"%s: Failed to unlock, rc = %d\n", __func__, rc);
        rc = AR_EFAILED;
    }
err_cond:
    pthread_mutex_unlock(&the_signal->osal_mutex);
done:
    return rc;
}

_IRQL_requires_max_(DISPATCH_LEVEL)
int32_t ar_osal_signal_destroy(_In_ ar_osal_signal_t signal)
{
    int32_t rc;
    osal_int_signal_t *the_signal;

    if (NULL == signal) {
        AR_LOG_ERR(AR_OSAL_SIGNAL_LOG_TAG,"%s: signal is NULL\n", __func__);
        rc = AR_EBADPARAM;
        goto done;
    }

    the_signal = (osal_int_signal_t *)signal;

    the_signal-> signalled = false;
    rc = pthread_mutex_lock(&the_signal->osal_mutex);

    if (rc) {
        AR_LOG_ERR(AR_OSAL_SIGNAL_LOG_TAG,"%s: Failed to acquire lock, rc = %d\n", __func__, rc);
        rc = AR_EFAILED;
        return rc;
    }
    rc = pthread_cond_destroy(&the_signal->osal_cond);

    if (rc) {
        AR_LOG_ERR(AR_OSAL_SIGNAL_LOG_TAG,"%s: Failed to destroy signal, rc = %d\n", __func__, rc);
        rc = AR_EFAILED;
        goto err_cond;
    }
    rc = pthread_mutex_unlock(&the_signal->osal_mutex);
    if (rc) {
        AR_LOG_ERR(AR_OSAL_SIGNAL_LOG_TAG,"%s: Failed to unlock, rc = %d\n", __func__, rc);
        rc = AR_EFAILED;
        goto err_destroy;
    }
    rc = pthread_mutex_destroy(&the_signal->osal_mutex);
    if (rc) {
        AR_LOG_ERR(AR_OSAL_SIGNAL_LOG_TAG,"%s: Failed to destroy mutex, rc = %d\n", __func__, rc);
        rc = AR_EFAILED;
    }
    goto done;

err_cond:
    pthread_mutex_unlock(&the_signal->osal_mutex);
err_destroy:
    pthread_mutex_destroy(&the_signal->osal_mutex);
done:
    free(signal);
    return rc;
}

_IRQL_requires_max_(DISPATCH_LEVEL)
int32_t ar_osal_signal_clear(_In_ ar_osal_signal_t signal)
{
    int32_t rc;
    osal_int_signal_t *the_signal;

    if (NULL == signal) {
        AR_LOG_ERR(AR_OSAL_SIGNAL_LOG_TAG,"%s: signal is NULL\n", __func__);
        rc = AR_EBADPARAM;
        goto done;
    }

    the_signal = (osal_int_signal_t *)signal;
    rc = pthread_mutex_lock(&the_signal->osal_mutex);
    if (rc) {
        AR_LOG_ERR(AR_OSAL_SIGNAL_LOG_TAG,"%s: Failed to acquire lock, rc = %d\n", __func__, rc);
        rc = AR_EFAILED;
        goto done;
    }

    the_signal->signalled = false;

    rc = pthread_mutex_unlock(&the_signal->osal_mutex);
    if (rc) {
        AR_LOG_ERR(AR_OSAL_SIGNAL_LOG_TAG,"%s: Failed to unlock, rc = %d\n", __func__, rc);
        rc = AR_EFAILED;
    }

done:
    return rc;
}
