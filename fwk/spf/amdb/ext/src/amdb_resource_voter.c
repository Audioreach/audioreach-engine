/**
 * \file amdb_resource_voter.c
 * \brief
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "amdb_resource_voter.h"

#define AMDB_LOG_ID (0xA3DB0000)
#define AMDB_RELEASE_DELAY_MS (0)

static void amdb_voter_register_pm(amdb_voter_t *obj_ptr);
static void amdb_voter_deregister_pm(amdb_voter_t *obj_ptr);
/*---------------------------------------------------------------------------------------------------------------------
  IN  : None
  OUT : None

  --------------------------------------------------------------------------------------------------------------------*/
amdb_voter_t *amdb_voter_create(POSAL_HEAP_ID heap_id)
{
   amdb_voter_t *obj_ptr = (amdb_voter_t *)posal_memory_malloc(sizeof(amdb_voter_t), heap_id);
   if (NULL == obj_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "amdb: Failed to allocate memory for the ADSPPM voter.");
      return NULL;
   }

   memset(obj_ptr, 0, sizeof(amdb_voter_t));

   posal_mutex_create(&obj_ptr->mutex, heap_id);

   amdb_voter_register_pm(obj_ptr);

   return obj_ptr;
}

/*---------------------------------------------------------------------------------------------------------------------
  IN  : None
  OUT : None

  --------------------------------------------------------------------------------------------------------------------*/
void amdb_voter_vote(amdb_voter_t *arg_ptr)
{
   amdb_voter_t *obj_ptr = (amdb_voter_t *)arg_ptr;
   posal_mutex_lock(obj_ptr->mutex);
   if (0 == obj_ptr->refs)
   {
      posal_power_mgr_request_max_out(obj_ptr->pm_handle_ptr, NULL, AMDB_LOG_ID);
   }

   obj_ptr->refs++;
   posal_mutex_unlock(obj_ptr->mutex);
}

/*---------------------------------------------------------------------------------------------------------------------
  IN  : None
  OUT : None

  --------------------------------------------------------------------------------------------------------------------*/
void amdb_voter_release(amdb_voter_t *arg_ptr)
{
   amdb_voter_t *obj_ptr = (amdb_voter_t *)arg_ptr;
   posal_mutex_lock(obj_ptr->mutex);
   obj_ptr->refs--;

   if (0 == obj_ptr->refs)
   {
      posal_power_mgr_release_max_out(obj_ptr->pm_handle_ptr, AMDB_LOG_ID, AMDB_RELEASE_DELAY_MS);
   }

   posal_mutex_unlock(obj_ptr->mutex);
}

/*---------------------------------------------------------------------------------------------------------------------
  IN  : None
  OUT : None

  --------------------------------------------------------------------------------------------------------------------*/
void amdb_voter_destroy(amdb_voter_t *arg_ptr)
{
   amdb_voter_t *obj_ptr = (amdb_voter_t *)arg_ptr;
   if (NULL != obj_ptr)
   {
      // Deregister
      amdb_voter_deregister_pm((void *)obj_ptr);
      posal_mutex_destroy(&obj_ptr->mutex);
      posal_memory_free((void *)obj_ptr);
   }
}

/*---------------------------------------------------------------------------------------------------------------------
  IN  : None
  OUT : None

  --------------------------------------------------------------------------------------------------------------------*/
static void amdb_voter_register_pm(amdb_voter_t *obj_ptr)
{
   ar_result_t result = posal_power_mgr_register(obj_ptr->register_info, /**< PM register information */
                                                 &obj_ptr->pm_handle_ptr,
                                                 NULL, /* Using NULL signal so that wrapper creates one locally */
                                                 AMDB_LOG_ID);

   AR_MSG(DBG_HIGH_PRIO, "amdb: PM register by AMDB. Result %d", result);
}

/*---------------------------------------------------------------------------------------------------------------------
  IN  : None
  OUT : None

  --------------------------------------------------------------------------------------------------------------------*/
static void amdb_voter_deregister_pm(amdb_voter_t *obj_ptr)
{
   if (!posal_power_mgr_is_registered(obj_ptr->pm_handle_ptr))
   {
      return;
   }
   ar_result_t result = posal_power_mgr_deregister(&obj_ptr->pm_handle_ptr, AMDB_LOG_ID);

   AR_MSG(DBG_HIGH_PRIO, "amdb: PM deregister by AMDB. Result %d", result);
}
