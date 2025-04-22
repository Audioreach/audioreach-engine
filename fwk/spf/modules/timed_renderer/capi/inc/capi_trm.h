/**
 * \file capi_trm.h
 * \brief
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef CAPI_TRM_H
#define CAPI_TRM_H

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
capi_err_t capi_trm_get_static_properties(capi_proplist_t *init_set_properties, capi_proplist_t *static_properties);

capi_err_t capi_trm_init(capi_t *_pif, capi_proplist_t *init_set_properties);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif // CAPI_TRM_H