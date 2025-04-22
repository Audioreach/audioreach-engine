/* ======================================================================== */
/**
@file capi_latency.h

   Header file to implement the Common Audio Post Processor Interface
   for Tx/Rx Tuning latency block
*/

/* =========================================================================
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
  ========================================================================== */

/*------------------------------------------------------------------------
 * Include files
 * -----------------------------------------------------------------------*/
#ifndef CAPI_LATENCY_H
#define CAPI_LATENCY_H

#include "capi.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/**
* Get static properties of latency module such as
* memory, stack requirements etc.
* See capi.h for more details.
*/
capi_err_t capi_latency_get_static_properties(capi_proplist_t *init_set_properties, capi_proplist_t *static_properties);

/**
* Instantiates(and allocates) the module memory.
* See capi.h for more details.
*/
capi_err_t capi_latency_init(capi_t *_pif, capi_proplist_t *init_set_properties);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif // CAPI_LATENCY_H