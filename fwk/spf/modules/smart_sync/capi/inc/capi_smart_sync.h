/**
 * \file capi_smart_sync.h
 * \brief
 *     Header file to implement the Common Audio Post Processor Interface for the smart sync module.
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef _CAPI_SMART_SYNC_H_
#define _CAPI_SMART_SYNC_H_

#include "capi.h"
#include "ar_defs.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/**
 * Get static properties of smart sync module such as
 * memory, stack requirements etc.
 * See Elite_CAPI.h for more details.
 */
capi_err_t capi_smart_sync_get_static_properties(capi_proplist_t *init_set_properties,
                                                 capi_proplist_t *static_properties);

/**
 * Instantiates(and allocates) the module memory.
 * See Elite_CAPI.h for more details.
 */
capi_err_t capi_smart_sync_init(capi_t *_pif, capi_proplist_t *init_set_properties);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif // _CAPI_SMART_SYNC_H_
