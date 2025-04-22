#ifndef CU_ASYNC_CMD_HANDLE_H
#define CU_ASYNC_CMD_HANDLE_H

/**
 * \file cu_async_cmd_handle.h
 *
 * \brief
 *
 *     CU utility for async command handling in a seprate thread.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "posal.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

typedef struct cu_base_t cu_base_t;

typedef struct cu_async_cmd_handle_t cu_async_cmd_handle_t;

/* Initialize thread pool which will be used to offload async command processing.
 * sync_signal_bit_mask is the signal bit mask which is used by the thread pool to wakeup container in case if any
 * command is partially pending. This Bit-Mask should be higher priority than Container Command Queue bit mask.*/
ar_result_t cu_async_cmd_handle_init(cu_base_t *cu_ptr, uint32_t sync_signal_bit_mask);

/* De initialize the thread pool from this container.*/
ar_result_t cu_async_cmd_handle_deinit(cu_base_t *cu_ptr);

/* If Container thread is re launched with updated stack size then this function should be called to update the stack
 * size in the thread pool as well.*/
ar_result_t cu_async_cmd_handle_update(cu_base_t *cu_ptr);

/* Function to handle the command processing in thread pool.
 * If command is scheduled with thread pool then this functions returns TRUE.
 * If command is not scheduled with thread pool then this function returns FALSE */
bool_t cu_async_cmd_handle_check_and_push_cmd(cu_base_t *cu_ptr);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef CU_ASYNC_CMD_HANDLE_H
