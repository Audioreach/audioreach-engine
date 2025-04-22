#ifndef GEN_CNTR_H
#define GEN_CNTR_H

/**
 * \file gen_cntr.h
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
#include "container_utils.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

/** @addtogroup gen_cntr
 @{ */

/**
 Creates an instance of the compression decompression container (GEN_CNTR)

 @param [in]  init_params_ptr  Pointer to spf_cntr_init_params_t
 @param [out] handle           handle returned to the caller.

 @return
 Success or failure of the instance creation.

 @dependencies
 None.
 */
ar_result_t gen_cntr_create(cntr_cmn_init_params_t *init_params_ptr, spf_handle_t **handle, uint32_t cntr_type);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef AUDCMNUTIL_H
