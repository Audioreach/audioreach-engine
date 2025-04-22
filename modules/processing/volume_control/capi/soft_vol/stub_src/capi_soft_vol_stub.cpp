/* =========================================================================
  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
  SPDX-License-Identifier: BSD-3-Clause-Clear
 * =========================================================================*/

/**
 * @file capi_soft_vol_stub.cpp
 *
 * Stub Interface for soft_vol module.
 */

#include "capi_soft_vol.h"

capi_err_t capi_soft_vol_get_static_properties(
        capi_proplist_t *init_set_properties,
        capi_proplist_t *static_properties)
{
    return CAPI_EUNSUPPORTED;
}


capi_err_t capi_soft_vol_init(
        capi_t          *_pif,
        capi_proplist_t *init_set_properties)
{
    return CAPI_EUNSUPPORTED;
}

