/**
 * \file graph_utils.c
 *
 * \brief
 *
 *     Graph utilities
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "graph_utils.h"
#include "posal.h"
#include "posal_internal_inline.h"
#include "spf_list_utils.h"

gu_ctrl_port_t *gu_find_ctrl_port_by_id(gu_module_t *module_ptr, uint32_t id)
{
   if (!module_ptr)
   {
      return NULL;
   }

   // if currently is in island then need to be careful while accessing control port ptr.
   // some of the port's memory can be in non-island

   bool_t check_in_non_island = FALSE;

   gu_ctrl_port_list_t *list_ptr = module_ptr->ctrl_port_list_ptr;
   while (list_ptr)
   {
      if (list_ptr->ctrl_port_ptr)
      {
         if (posal_island_get_island_status_inline() &&
             !posal_check_addr_from_tcm_island_heap_mgr(list_ptr->ctrl_port_ptr))
         {
            check_in_non_island = TRUE;
            // will check this port after exiting island, first try to find the port while in island
         }
         else
         {
            gu_ctrl_port_t *port_ptr = (gu_ctrl_port_t *)list_ptr->ctrl_port_ptr;
            if (port_ptr->id == id)
            {
               return port_ptr;
            }
         }
      }
      LIST_ADVANCE(list_ptr);
   }

   if (check_in_non_island)
   {
      // exit island
      posal_island_trigger_island_exit();

      gu_ctrl_port_list_t *list_ptr = module_ptr->ctrl_port_list_ptr;
      while (list_ptr)
      {
         if (list_ptr->ctrl_port_ptr)
         {
            gu_ctrl_port_t *port_ptr = (gu_ctrl_port_t *)list_ptr->ctrl_port_ptr;
            if (port_ptr->id == id)
            {
               return port_ptr;
            }
         }
         LIST_ADVANCE(list_ptr);
      }
   }
   return NULL;
}
