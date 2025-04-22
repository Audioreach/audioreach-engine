/**
@file irm.h

@brief Header for Integrated Resource Monitor (IRM).

================================================================================
Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
SPDX-License-Identifier: BSD-3-Clause-Clear
==============================================================================*/
// clang-format off
// clang-format on

#ifndef _IRM_H_
#define _IRM_H_

#include "ar_error_codes.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/
/*----------------------------------------------------------------------------------------------------------------------
  1. Initializes irm
  2. Creates the command handler thread for the IRM

  return:  on success, or error code otherwise.
----------------------------------------------------------------------------------------------------------------------*/
ar_result_t irm_init(POSAL_HEAP_ID heap_id);

/*----------------------------------------------------------------------------------------------------------------------
  resets the IRM to boot up state
----------------------------------------------------------------------------------------------------------------------*/
ar_result_t irm_reset(bool_t is_flush_needed, bool_t is_reset_needed);

/*----------------------------------------------------------------------------------------------------------------------
  1. Deinitializes irm
  2. Destroys the IRM command handler thread

  return:  None
----------------------------------------------------------------------------------------------------------------------*/
ar_result_t irm_deinit();

/*----------------------------------------------------------------------------------------------------------------------
 Fills the irm spf_handle

  return:  error code
----------------------------------------------------------------------------------------------------------------------*/
ar_result_t irm_get_spf_handle(void **spf_handle_pptr);

/*----------------------------------------------------------------------------------------------------------------------
 Resets the buf pool to the base state

  return:  error code
----------------------------------------------------------------------------------------------------------------------*/
void irm_buf_pool_reset();

/*----------------------------------------------------------------------------------------------------------------------
 Returns of any container or module profiling is enabled
----------------------------------------------------------------------------------------------------------------------*/
bool_t irm_is_cntr_or_mod_prof_enabled();

/*----------------------------------------------------------------------------------------------------------------------
 Registers the static module with the IRM for profiling. This should ONLY be called from the main thread
 (audio_process_r), not any static module thread. This is to avoid any race condition on the registration
 data structure before a mutex is setup.
----------------------------------------------------------------------------------------------------------------------*/
ar_result_t irm_register_static_module(uint32_t mid, uint32_t heap_id, int64_t tid);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* _IRM_H_ */
