/**
 * \file gen_topo_metadata_exit_island.c
 * \brief
 *     Function to exit island if metadata is not supported in island
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "gen_topo.h"

#ifdef METADATA_LIB_IN_NLPI

/* This function is called in 1) propagate metadata, 2)when md arrives at container input
 *
 * When metadata library is compiled in non-island, this function exists island and keeps the container in non-island
 * for next 2 frames.
 * This is needed vs simple exit island temporarily call:
 * 1)to avoid multiple island exit/entry while the metadata propagates
 * 2)ensures md is handled in non-island throughout topo/module layers thereby preventing need for exit island functions
 * in those layers
 *
 * When metadata library is compiled in island, stub function will be called and we wont exit island.
 */
void gen_topo_vote_against_lpi_if_md_lib_in_nlpi(gen_topo_t *topo_ptr)
{
   if (topo_ptr->topo_to_cntr_vtable_ptr->vote_against_island)
   {
      topo_ptr->topo_to_cntr_vtable_ptr->vote_against_island(topo_ptr);
   }
   return;
}

/*For other metadata operations such as Metadata create, destroy and clone we need to call
 * gen_topo_exit_lpi_temporarily_if_md_lib_in_nlpi
 * We dont need to stay in non-island for 2 frames after, since it usually only involves one operation.
 *
 * When md lib is compiled in non island, to access this lib we exit island temporarily
 * When md lib is compiled in island, exit island stub function is called  */
void gen_topo_exit_lpi_temporarily_if_md_lib_in_nlpi(gen_topo_t *topo_ptr)
{
   gen_topo_exit_island_temporarily(topo_ptr);
   return;
}

#else // !METADATA_LIB_IN_NLPI

/* If metadata lib is compiled in island, we simply return without exiting island */
void gen_topo_vote_against_lpi_if_md_lib_in_nlpi(gen_topo_t *topo_ptr)
{
   return;
}

/* If metadata lib is compiled in island, we simply return without exiting island */
void gen_topo_exit_lpi_temporarily_if_md_lib_in_nlpi(gen_topo_t *topo_ptr)
{
   return;
}

#endif // METADATA_LIB_IN_NLPI
