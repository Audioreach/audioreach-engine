/**
 * \file gpr_session.c
 * \brief
 *  	This file contains GPR session implementation
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/******************************************************************************
 * Includes                                                                    *
 *****************************************************************************/

#include "gpr_session.h"

/**
  @brief Searches for callback function based on src port

  @param[in] my_port      Address/port of module
  @param[out] ret_entry   Double pointer to the session found

  @detdesc
  Searches the linked list until node matching
  given src_port is found and returns pointer to the corresponding session

  @return
  #AR_EOK when successful.
*/
GPR_INTERNAL uint32_t gpr_get_session(uint32_t                my_port,
                                      gpr_module_entry_t    **ret_entry,
                                      gpr_module_node_list_t *list_handle)
{
   if (NULL == list_handle)
   {
      return AR_EFAILED;
   }
   gpr_module_node_t **chain = &list_handle->cb_list[0]; // Initialize current
   uint32_t            key   = SESSION_HASH(list_handle->max_cb_list_size, my_port);

   gpr_module_node_t *temp = chain[key];

   while (temp != NULL)
   {
      if (temp->my_module_port == my_port) // directly hashed slot finding
      {
         *ret_entry = &temp->session;
         return AR_EOK;
      }
      temp = temp->next; // if it is open hashed slots
   }

#ifdef GPR_DEBUG_MSG
   AR_MSG(DBG_ERROR_PRIO, "No such module port (%lu) found in linked list", my_port);
#endif
   return AR_ENOTEXIST;
}