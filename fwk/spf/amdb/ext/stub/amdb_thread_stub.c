/**
 * \file amdb_thread_stub.c
 * \brief
 *     This file contains AMDB Module Implementation
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/**==============================================================================
   Global Defines
==============================================================================*/
#include "amdb_thread.h"

ar_result_t amdb_thread_deinit()
{
   return AR_EOK;
}

ar_result_t amdb_thread_reset(bool_t is_flush_needed, bool_t is_reset_needed)
{
   return AR_EOK;
}

ar_result_t amdb_get_spf_handle(void **spf_handle_pptr)
{
   return AR_EOK;
}

ar_result_t amdb_thread_init(POSAL_HEAP_ID heap_id)
{
   return AR_EOK;
}
