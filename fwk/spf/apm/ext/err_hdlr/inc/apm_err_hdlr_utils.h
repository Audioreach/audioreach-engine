#ifndef _APM_ERR_HDLR_UTILS_H_
#define _APM_ERR_HDLR_UTILS_H_

/**
 * \file apm_data_path_utils.h
 *
 * \brief
 *     This file contains declarations for utility functions for
 *     error handling during command processing
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "apm_i.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/**------------------------------------------------------------------------------
 *  Structure Definition
 *----------------------------------------------------------------------------*/


typedef struct apm_err_hdlr_utils_vtable
{
   ar_result_t (*apm_cmd_graph_open_err_hdlr_sequencer_fptr)(apm_t *apm_info_ptr);

   ar_result_t (*apm_populate_cont_open_cmd_err_hdlr_seq_fptr)(apm_t *apm_info_ptr);

   ar_result_t (*apm_err_hdlr_cache_container_rsp_fptr)(apm_t *apm_info_ptr);

   ar_result_t (*apm_err_hdlr_clear_cont_cached_graph_open_cfg_fptr)(apm_cont_cached_cfg_t *cached_cfg_ptr,
                                                                     apm_cmd_ctrl_t *       apm_cmd_ctrl_ptr);

} apm_err_hdlr_utils_vtable_t;


/**------------------------------------------------------------------------------
 *  Function Declaration
 *----------------------------------------------------------------------------*/

ar_result_t apm_err_hdlr_utils_init(apm_t *apm_info_ptr);

#ifdef __cplusplus
}
#endif /*__cplusplus*/
#endif /** #ifdef _APM_ERR_HDLR_UTILS_H_ */
