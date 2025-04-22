/**
* \file ar_osal_stub_log_pkt_op.c
* \brief
*    Defines stub APIs for ar_log_pkt.
* \copyright
*  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
*  SPDX-License-Identifier: BSD-3-Clause-Clear
*/

#include "ar_osal_log_pkt_op.h"
#include "ar_osal_error.h"

int32_t ar_log_pkt_op_init(ar_log_pkt_op_init_info_t *info)
{
    return AR_EOK;
}

int32_t ar_log_pkt_op_deinit()
{
    return AR_EOK;
}

uint32_t ar_log_pkt_get_max_size()
{
    return 0;
}

void *ar_log_pkt_alloc(uint16_t logcode, uint32_t length)
{
    return NULL;
}

int32_t ar_log_pkt_commit(void *ptr)
{
    return AR_EOK;
}

void ar_log_pkt_free(void *ptr)
{
}

bool_t ar_log_code_status(uint16_t logcode)
{
    return true;
}
