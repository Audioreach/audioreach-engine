/**
 * \file capi_ecns.h
 *  
 * \brief
 *  
 *     Example Echo Cancellation
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/*------------------------------------------------------------------------
 * Include files and Macro definitions
 * -----------------------------------------------------------------------*/
#ifndef CAPI_EC_H
#define CAPI_EC_H

#include "capi.h"
#include "ar_defs.h"

/*------------------------------------------------------------------------
* Function declarations
* -----------------------------------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif

capi_err_t capi_ecns_get_static_properties(capi_proplist_t *init_set_properties, capi_proplist_t *static_properties);

capi_err_t capi_ecns_init(capi_t *_pif, capi_proplist_t *init_set_properties);

#ifdef __cplusplus
}
#endif

#endif //CAPI_EC_H

