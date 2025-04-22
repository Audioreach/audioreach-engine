/**
 * \file ar_osal_signal2.c
 *
 * \brief
 *       This file has implementation of signal2 APIs.
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include <stdlib.h>
#include <errno.h>
#include "ar_osal_signal2.h"

_IRQL_requires_max_(PASSIVE_LEVEL)
int32_t ar_osal_signal2_init(_Inout_ ar_osal_signal2_t osal_signal2)
{
    ALOGE("%s: not implemented\n", __func__);
    return -ENOSYS;
}

_IRQL_requires_max_(PASSIVE_LEVEL)
int32_t ar_osal_signal2_deinit(_In_ ar_osal_signal2_t osal_signal2)
{
    ALOGE("%s: not implemented\n", __func__);
    return -ENOSYS;
}

_IRQL_requires_max_(PASSIVE_LEVEL)
size_t ar_osal_signal2_get_size()
{
    ALOGE("%s: not implemented\n", __func__);
    return -ENOSYS;
}

_IRQL_requires_max_(PASSIVE_LEVEL)
int32_t ar_osal_signal2_create(_Out_ ar_osal_signal2_t *osal_signal2)
{
    ALOGE("%s: not implemented\n", __func__);
    return -ENOSYS;
}

_IRQL_requires_max_(PASSIVE_LEVEL)
int32_t ar_osal_signal2_destroy(_In_ ar_osal_signal2_t osal_signal2)
{
    ALOGE("%s: not implemented\n", __func__);
    return -ENOSYS;
}

_IRQL_requires_max_(PASSIVE_LEVEL)
uint32_t ar_osal_signal2_wait_any(_In_ ar_osal_signal2_t osal_signal2, _In_ uint32_t osal_signal2_mask)
{
    ALOGE("%s: not implemented\n", __func__);
    return -ENOSYS;
}

_IRQL_requires_max_(PASSIVE_LEVEL)
uint32_t ar_osal_signal2_wait_all( _In_ ar_osal_signal2_t osal_signal2, _In_ uint32_t osal_signal2_mask);
{
    ALOGE("%s: not implemented\n", __func__);
    return -ENOSYS;
}

_IRQL_requires_max_(PASSIVE_LEVEL)
int32_t ar_osal_signal2_set(_In_ ar_osal_signal2_t osal_signal2, _In_ uint32_t osal_signal2_mask)
{
    ALOGE("%s: not implemented\n", __func__);
    return -ENOSYS;
}

_IRQL_requires_max_(PASSIVE_LEVEL)
uint32_t ar_osal_signal2_get(_In_ ar_osal_signal2_t osal_signal2)
{
    ALOGE("%s: not implemented\n", __func__);
    return -ENOSYS;
}

_IRQL_requires_max_(PASSIVE_LEVEL)
int32_t ar_osal_signal2_clear(_In_ ar_osal_signal2_t osal_signal2, _In_ uint32_t osal_signal2_mask)
{
    ALOGE("%s: not implemented\n", __func__);
    return -ENOSYS;
}
