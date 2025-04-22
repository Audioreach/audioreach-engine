/**
 * \file capi_duty_cycling_buf.h
 * \brief
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef CAPI_DUTY_CYCLING_BUF_H
#define CAPI_DUTY_CYCLING_BUF_H

/*------------------------------------------------------------------------
 * Include files
 * -----------------------------------------------------------------------*/
#include "capi.h"
#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/
/*------------------------------------------------------------------------
 * Function declarations
 * ----------------------------------------------------------------------*/
capi_err_t capi_duty_cycling_buf_get_static_properties(capi_proplist_t *init_set_properties, capi_proplist_t *static_properties);

capi_err_t capi_duty_cycling_buf_init(capi_t *_pif, capi_proplist_t *init_set_properties);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif // CAPI_DUTY_CYCLING_BUF_H