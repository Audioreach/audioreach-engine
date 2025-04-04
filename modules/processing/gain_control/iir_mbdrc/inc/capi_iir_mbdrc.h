/* =====================================================================
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause
 * =====================================================================*/

/**
 * @file capi_iir_mbdrc.h
 *
 */

#ifndef CAPI_IIR_MBDRC_H
#define CAPI_IIR_MBDRC_H

#include "capi.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Get static properties of mbdrc module such as
 * memory, stack requirements etc.
 * See Elite_CAPI.h for more details.
 */
capi_err_t capi_iir_mbdrc_get_static_properties(
		capi_proplist_t* init_set_properties,
		capi_proplist_t* static_properties);


/**
 * Instantiates(and allocates) the module memory.
 * See Elite_CAPI.h for more details.
 */
capi_err_t capi_iir_mbdrc_init(
		capi_t*          _pif,
		capi_proplist_t* init_set_properties);

#ifdef __cplusplus
}
#endif

#endif

