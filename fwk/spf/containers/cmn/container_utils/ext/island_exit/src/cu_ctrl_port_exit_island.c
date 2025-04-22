/**
 * \file cu_ctrl_port_exit_island.c
 * \brief
 *     Function to exit island if ctrl port lib is not supported in island
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "cu_i.h"

#ifdef CTRL_PORT_LIB_IN_ISLAND

/* If ctrl port lib is compiled in island, we simply return without exiting island */
void cu_vote_against_lpi_if_ctrl_port_lib_in_nlpi(cu_base_t *me_ptr)
{
   return;
}

/* If ctrl port lib is compiled in island, we simply return without exiting island */
void cu_exit_lpi_temporarily_if_ctrl_port_lib_in_nlpi(cu_base_t *me_ptr)
{
   return;
}

#else // !CTRL_PORT_LIB_IN_ISLAND

/*
 * When ctrl port library is compiled in non-island, this function exists island and keeps the container in non-island
 * for next 2 frames.
 * When ctrl port library is compiled in island, stub function will be called and we wont exit island.
 */
void cu_vote_against_lpi_if_ctrl_port_lib_in_nlpi(cu_base_t *me_ptr)
{
   if (me_ptr->cntr_vtbl_ptr->vote_against_island)
   {
      me_ptr->cntr_vtbl_ptr->vote_against_island((void *)me_ptr);
   }
   return;
}

/*
 * When ctrl port lib is compiled in non island, to access this lib we exit island temporarily
 * When ctrl port lib is compiled in island, exit island stub function is called  */
void cu_exit_lpi_temporarily_if_ctrl_port_lib_in_nlpi(cu_base_t *me_ptr)
{
   if (me_ptr->cntr_vtbl_ptr->exit_island_temporarily)
   {
      me_ptr->cntr_vtbl_ptr->exit_island_temporarily((void *)me_ptr);
   }
   return;
}

#endif // CTRL_PORT_LIB_IN_ISLAND
