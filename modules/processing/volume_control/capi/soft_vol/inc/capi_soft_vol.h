/* =========================================================================
  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
  SPDX-License-Identifier: BSD-3-Clause-Clear
 * =========================================================================*/

/**
 * @file capi_soft_vol.h
 *
 * Common Audio Processor Interface for soft_vol.
 */

#ifndef CAPI_SOFT_VOL_H
#define CAPI_SOFT_VOL_H

#include "capi.h"
#include "ar_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Get static properties of soft_vol module such as
 * memory, stack requirements etc.
 * See Elite_CAPI.h for more details.
 */
capi_err_t capi_soft_vol_get_static_properties(capi_proplist_t *init_set_properties,
                                                     capi_proplist_t *static_properties);

/**
 * Instantiates(and allocates) the module memory.
 * See Elite_CAPI.h for more details.
 */
capi_err_t capi_soft_vol_init(capi_t *_pif, capi_proplist_t *init_set_properties);

#ifdef __cplusplus
}
#endif

#endif /* CAPI_SOFT_VOL_H */
