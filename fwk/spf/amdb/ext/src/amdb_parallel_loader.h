/**
 * \file amdb_parallel_loader.h
 * \brief
 *    This file describes the interface for the parallel loader for AMDB. This utility calls dlopen for multiple modules
 *  concurrently.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef AMDB_PARALLEL_LOADER_H
#define AMDB_PARALLEL_LOADER_H

/*----------------------------------------------------------------------------
 * Include files
 * -------------------------------------------------------------------------*/
#include "amdb_static.h"
#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

/*----------------------------------------------------------------------------
 * Type Declarations
 * -------------------------------------------------------------------------*/
typedef void (*amdb_loader_load_function)(uint64_t task_info);

/*----------------------------------------------------------------------------
 * Function Declarations
 * -------------------------------------------------------------------------*/

void *amdb_loader_create(POSAL_HEAP_ID heap_id);
void *amdb_loader_get_handle(void *                      obj_ptr,
                             amdb_get_modules_callback_f callback_f,
                             void *                      context_ptr,
                             amdb_loader_load_function   load_function);
void  amdb_loader_push_task(void *loader_handle_ptr, uint64_t task_info);
void  amdb_loader_release_handle(void *loader_handle_ptr);
void  amdb_loader_destroy(void *obj_ptr);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef AMDB_PARALLEL_LOADER_H
