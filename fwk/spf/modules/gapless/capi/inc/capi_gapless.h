/**
 * \file capi_gapless.h
 * \brief
 *      Header file to implement the Common Audio Post Processor Interface for the gapless module.
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef CAPI_GAPLESS_H
#define CAPI_GAPLESS_H

#include "capi.h"
#include "ar_defs.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/**
* Get static properties of Gain module such as
* memory, stack requirements etc.
* See Elite_CAPI.h for more details.
*/
capi_err_t capi_gapless_get_static_properties(capi_proplist_t *init_set_properties, capi_proplist_t *static_properties);

/**
* Instantiates(and allocates) the module memory.
* See Elite_CAPI.h for more details.
*/
capi_err_t capi_gapless_init(capi_t *_pif, capi_proplist_t *init_set_properties);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif // CAPI_GAPLESS_H
