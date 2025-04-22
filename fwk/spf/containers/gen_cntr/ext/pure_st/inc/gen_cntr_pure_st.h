#ifndef GEN_CNTR_PURE_ST_H
#define GEN_CNTR_PURE_ST_H
/**
 * \file gen_cntr_pure_st.h
 * \brief
 *     This file contains functions for Pure ST contiainers.
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "gen_cntr.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

typedef struct gen_cntr_t               gen_cntr_t;

ar_result_t gen_cntr_pure_st_process_frames(gen_cntr_t *me_ptr);

ar_result_t gen_cntr_check_and_assign_st_data_process_fn(gen_cntr_t *me_ptr);

/* If st topo lib is compiled in island, we simply return without exiting island */
void gen_cntr_vote_against_lpi_if_pure_st_topo_lib_in_nlpi(gen_cntr_t *me_ptr);

/* If st topo lib is compiled in island, we simply return without exiting island */
void gen_cntr_exit_lpi_temporarily_if_pure_st_topo_lib_in_nlpi(gen_cntr_t *me_ptr);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef GEN_CNTR_PURE_ST_H
