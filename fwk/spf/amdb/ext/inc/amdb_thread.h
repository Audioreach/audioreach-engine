#ifndef _AMDB_THREAD_H_
#define _AMDB_THREAD_H_
/**
 * \file amdb_thread.h
 * \brief
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "ar_error_codes.h"
#include "posal.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus
/** Initializes amdb and creates the command handler thread for the AMDB */
ar_result_t amdb_thread_init(POSAL_HEAP_ID heap_id);

/** Handles close all by flushing the cmd queue */
ar_result_t amdb_thread_reset(bool_t is_flush_needed, bool_t is_reset_needed);

/** Returns the spf handle if thread has started */
ar_result_t amdb_get_spf_handle(void **spf_handle_pptr);

/** Deinitializes amdb and destroys the AMDB command handler thread */
ar_result_t amdb_thread_deinit();
#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* _AMDB_THREAD_H_ */
