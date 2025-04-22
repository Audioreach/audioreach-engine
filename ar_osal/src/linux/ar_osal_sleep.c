/**
 * \file ar_osal_sleep.c
 *
 * \brief
 *      This file has implementation of suspend thread execution for the required
 *      duration.
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#define AR_OSAL_SLEEP_LOG_TAG   "COSL"
#include <unistd.h>
#include "ar_osal_signal.h"
#include "ar_osal_log.h"
#include "ar_osal_error.h"

_IRQL_requires_min_(PASSIVE_LEVEL)
int32_t ar_osal_micro_sleep(_In_ uint64_t micro_seconds)
{
    int32_t status = 0;
    while (micro_seconds != 0) {
        if (micro_seconds > 999999) {
            status = usleep(999999);
            if (0 != status) {
                AR_LOG_ERR(AR_OSAL_SLEEP_LOG_TAG,"%s: usleep error %d", __func__, status);
                goto exit;
            }
            micro_seconds -= 999999;
        }
        else {
            status = usleep(micro_seconds);
            if (0 != status) {
                AR_LOG_ERR(AR_OSAL_SLEEP_LOG_TAG,"%s: usleep error %d", __func__, status);
                goto exit;
            }
            break;
        }
    }
exit:
    return status;
}
