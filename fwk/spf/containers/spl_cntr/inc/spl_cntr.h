/**
 * \file spl_cntr.h
 * \brief
 *     Post Processing Container, handles audio and voice use cases
 *  
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef SPL_CNTR_H
#define SPL_CNTR_H

// clang-format off

#include "spf_utils.h"
#include "container_utils.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

/* =======================================================================
 INCLUDE FILES FOR MODULE
 ========================================================================== */

/** @addtogroup spl_cntr
 @{ */

/**
 Creates an instance of the audio processing container (SPL_CNTR)

 @param [in]  init_params_ptr  Pointer to spf_cntr_init_params_t
 @param [out] handle           handle returned to the caller.

 @return
 Success or failure of the instance creation.

 @dependencies
 None.
 */
ar_result_t spl_cntr_create(cntr_cmn_init_params_t *init_params_ptr, spf_handle_t **handle, uint32_t cntr_type);

#ifdef __cplusplus
}
#endif //__cplusplus

// clang-format on

#endif // #ifndef SPL_CNTR_H
