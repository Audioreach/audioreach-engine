#ifndef GEN_CNTR_ERR_CHECK_H
#define GEN_CNTR_ERR_CHECK_H
/**
 * \file gen_cntr_err_check.h
 * \brief
 *     This file contains functions that do optional error checking. All products may not need such error checks
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
typedef struct gen_cntr_timestamp_t     gen_cntr_timestamp_t;
typedef struct gen_cntr_ext_out_port_t  gen_cntr_ext_out_port_t;
typedef struct gen_cntr_ext_in_port_t   gen_cntr_ext_in_port_t;
typedef struct gen_cntr_ext_ctrl_port_t gen_cntr_ext_ctrl_port_t;
typedef struct gen_cntr_circ_buf_list_t gen_cntr_circ_buf_list_t;
typedef struct gen_cntr_module_t        gen_cntr_module_t;

ar_result_t gen_cntr_check_for_multiple_thresh_modules(gen_cntr_t *me_ptr);

ar_result_t gen_cntr_check_handle_signal_miss(gen_cntr_t *me_ptr, bool_t is_after_process, bool_t *continue_processing);

bool_t gen_cntr_check_for_err_print(gen_topo_t *topo_ptr);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef GEN_CNTR_ERR_CHECK_H
