#ifndef CU_EXIT_ISLAND_H
#define CU_EXIT_ISLAND_H
/**
 * \file cu_exit_island.h
 * \brief
 *     This file contains utility function for exiting island before accessing certain libs which are marked as non-island
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "topo_utils.h"
#include "ar_defs.h"

// clang-format off

#if defined(__cplusplus)
extern "C" {
#endif // __cplusplus

typedef struct cu_base_t         cu_base_t;


//Ctrl port related func declarations
void cu_vote_against_lpi_if_ctrl_port_lib_in_nlpi(cu_base_t *topo_ptr);

void cu_exit_lpi_temporarily_if_ctrl_port_lib_in_nlpi(cu_base_t *me_ptr);


#if defined(__cplusplus)
}
#endif // __cplusplus

#endif /* CU_EXIT_ISLAND_H */
