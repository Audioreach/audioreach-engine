/**
 * \file amdb_queue.h
 * \brief
 *    This file describes the interface for the queue used by the AMDB parallel loader.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef AMDB_QUEUE_H
#define AMDB_QUEUE_H

/*----------------------------------------------------------------------------
 * Include files
 * -------------------------------------------------------------------------*/
#include "ar_defs.h"
#include "ar_error_codes.h"
#include "posal.h"
#include "stringl.h"
#include "posal_nmutex.h"
#include "posal_condvar.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

/*----------------------------------------------------------------------------
 * Type Declarations
 * -------------------------------------------------------------------------*/

typedef struct amdb_queue_t
{
   uint8_t *data_ptr;
   uint32_t push_location;
   uint32_t num_elements;
   uint32_t max_elements;
   uint32_t size_of_element;
   char     name[16];

   posal_nmutex_t queue_nmutex; // Have to use qurt mutex instead of posal_mutex since posal uses
                                // pimutex, which is not compatible with condvars.
   posal_condvar_t queue_push_condition;
   posal_condvar_t queue_pop_condition;
} amdb_queue_t;

/*----------------------------------------------------------------------------
 * Function Declarations
 * -------------------------------------------------------------------------*/

amdb_queue_t *amdb_queue_create(uint32_t max_items, uint32_t size_of_element, char name[], POSAL_HEAP_ID heap_id);
void          amdb_queue_push(amdb_queue_t *queue_ptr, const void *in_data_ptr);
void          amdb_queue_pop(amdb_queue_t *queue_ptr, void *out_data_ptr);
void          amdb_queue_destroy(amdb_queue_t *queue_ptr);

#ifdef __cplusplus
}
#endif //__cplusplus
#endif // #ifndef AMDB_QUEUE_H
