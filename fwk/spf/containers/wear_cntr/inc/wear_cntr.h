#ifndef WCNTR_H
#define WCNTR_H

/**
 * \file wear_cntr.h
 *
 * \brief
 *     Compression-Decompression Container
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "spf_utils.h"
#include "apm_cntr_if.h"


#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

/** @addtogroup wear_cntr
 @{ */

/**
 Creates an instance of the compression decompression container (WCNTR)

 @param [in]  init_params_ptr  Pointer to spf_cntr_init_params_t
 @param [out] handle           handle returned to the caller.

 @return
 Success or failure of the instance creation.

 @dependencies
 None.
 */
ar_result_t wear_cntr_create(cntr_cmn_init_params_t *init_params_ptr, spf_handle_t **handle);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef WCNTR_H
