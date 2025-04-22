/**
 * \file amdb_resource_voter.h
 * \brief
 *     This file describes the interface for the resource voter for AMDB. This utility allows threads to vote for dlopen
 *  resources
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef AMDB_RESOURCE_VOTER_H
#define AMDB_RESOURCE_VOTER_H
#include "posal.h"
#include "posal_memory.h"
#include "posal_mutex.h"
#include "posal_power_mgr.h"
#include "ar_error_codes.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus
/*----------------------------------------------------------------------------
 * Include files
 * -------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
 * Type Declarations
 * -------------------------------------------------------------------------*/
typedef struct amdb_voter_t
{
   posal_mutex_t       mutex;
   uint32_t            refs;
   posal_pm_handle_t   pm_handle_ptr; /**< Pointer to PM handle */
   posal_pm_register_t register_info; /**< PM register information */
} amdb_voter_t;

/*----------------------------------------------------------------------------
 * Function Declarations
 * -------------------------------------------------------------------------*/

amdb_voter_t *amdb_voter_create(POSAL_HEAP_ID heap_id);
void          amdb_voter_vote(amdb_voter_t *obj_ptr);
void          amdb_voter_release(amdb_voter_t *obj_ptr);
void          amdb_voter_destroy(amdb_voter_t *obj_ptr);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef AMDB_RESOURCE_VOTER_H
