/* ======================================================================== */
/**
@file capi_multistageiir.h

   Header file to implement the CAPI Interface for Multi-Stage IIR filter
   object (MSIIR).
*/

/* =========================================================================
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
  ========================================================================== */

/*------------------------------------------------------------------------
 * Include files
 * -----------------------------------------------------------------------*/
#ifndef __CAPI_MULTISTAGEIIR_H
#define __CAPI_MULTISTAGEIIR_H

#include "capi.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * Get static properties of multistage IIR module such as
 * memory, stack requirements etc.
 * See Elite_CAPI.h for more details.
 */
capi_err_t capi_multistageiir_get_static_properties(
      capi_proplist_t* init_set_properties,
      capi_proplist_t* static_properties);

/**
 * Instantiates(and allocates) the module memory.
 * See Elite_CAPI.h for more details.
 */
capi_err_t capi_multistageiir_init(
      capi_t*          _pif,
      capi_proplist_t* init_set_properties);

capi_err_t capi_multistageiir_left_init(
      capi_t*          _pif,
      capi_proplist_t* init_set_properties);

capi_err_t capi_multistageiir_right_init(
      capi_t*          _pif,
      capi_proplist_t* init_set_properties);

capi_err_t capi_multistageiir_left_get_static_properties(
      capi_proplist_t* init_set_properties,
      capi_proplist_t* static_properties);

capi_err_t capi_multistageiir_right_get_static_properties(
      capi_proplist_t* init_set_properties,
      capi_proplist_t* static_properties);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif //__CAPI_MULTISTAGEIIR_H

