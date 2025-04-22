/**
@file irm.cpp

@brief Main file for Integrated Resource Monitor (IRM).

================================================================================
Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
SPDX-License-Identifier: BSD-3-Clause-Clear
==============================================================================*/
// clang-format off
// clang-format on

#include "posal.h"
#include "irm.h"

ar_result_t irm_init(POSAL_HEAP_ID heap_id)
{
    return AR_EOK;
}

bool_t irm_is_cntr_or_mod_prof_enabled()
{
   return FALSE;
}

ar_result_t irm_register_static_module(uint32_t mid, uint32_t heap_id, int64_t tid)
{
   return AR_EOK;
}

ar_result_t irm_reset(bool_t is_flush_needed, bool_t is_reset_needed)
{
   return AR_EOK;
}

ar_result_t irm_get_spf_handle(void **spf_handle_pptr)
{
    return AR_EUNSUPPORTED;
}

ar_result_t irm_deinit()
{
    return AR_EOK;
}

void irm_buf_pool_reset()
{
    return;
}
