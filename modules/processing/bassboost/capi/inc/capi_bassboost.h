/* =========================================================================
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 * =========================================================================*/

/**
 * @file capi_bassboost.h
 *
 * Header file to implement the Audio Post Processor Interface for
 * acoustic bass enhancement
 */

#ifndef CAPI_BASSBOOST_H
#define CAPI_BASSBOOST_H

#include "capi.h"
#include "ar_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Get static properties of bassboost module such as
 * memory, stack requirements etc.
 * See capi.h for more details.
 */
capi_err_t capi_bassboost_get_static_properties(capi_proplist_t *init_set_properties,
                                                      capi_proplist_t *static_properties);

/**
 * Instantiates(and allocates) the module memory.
 * See capi.h for more details.
 */
capi_err_t capi_bassboost_init(capi_t *_pif, capi_proplist_t *init_set_properties);

#ifdef __cplusplus
}
#endif

#endif /* CAPI_BASSBOOST_H */
