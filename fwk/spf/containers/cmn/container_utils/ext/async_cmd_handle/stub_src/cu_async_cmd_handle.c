/**
 * \file cu_async_cmd_handle.c
 *
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/* =======================================================================
INCLUDE FILES FOR MODULE
========================================================================== */
#include "container_utils.h"

/***************************************************************************/

/***************************************************************************/

ar_result_t cu_async_cmd_handle_init(cu_base_t *cu_ptr, uint32_t sync_signal_bitmask)
{
   return AR_EUNSUPPORTED;
}

ar_result_t cu_async_cmd_handle_deinit(cu_base_t *cu_ptr)
{
   return AR_EUNSUPPORTED;
}

ar_result_t cu_async_cmd_handle_update(cu_base_t *cu_ptr)
{
   return AR_EUNSUPPORTED;
}

bool_t cu_async_cmd_handle_check_and_push_cmd(cu_base_t *cu_ptr)
{
   return FALSE;
}
