/* =========================================================================
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 * =========================================================================*/

/**
 * @file capi_bassboost_stub.cpp
 *
 * Stubbed Interface for Acoustic Bass Enhancement
 */

#include "capi_bassboost.h"

/* -------------------------------------------------------------------------
 * Function name: capi_bassboost_get_static_properties
 * Capi_v2 BassBoost stub function to get the static properties
 * -------------------------------------------------------------------------*/
capi_err_t capi_bassboost_get_static_properties(capi_proplist_t *init_set_properties,
                                                      capi_proplist_t *static_properties)
{
   return CAPI_EUNSUPPORTED;
}

/* -------------------------------------------------------------------------
 * Function name: capi_bassboost_init
 * Stub function for the Bass-boost Initialization
 * -------------------------------------------------------------------------*/
capi_err_t capi_bassboost_init(capi_t *_pif, capi_proplist_t *init_set_properties)
{
   return CAPI_EUNSUPPORTED;
}
