#ifndef CAPI_SISO_RAT_H
#define CAPI_SISO_RAT_H

/* =============================================================================
  @file capi_siso_rat.h
  @brief This file contains CAPI API's published by Rate Adapted Timer Module

================================================================================
  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
  SPDX-License-Identifier: BSD-3-Clause-Clear
==============================================================================*/
// clang-format off

// clang-format on

/*------------------------------------------------------------------------
 * Include files
 * -----------------------------------------------------------------------*/
#include "capi.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/*------------------------------------------------------------------------
 * Function declarations
 * ----------------------------------------------------------------------*/
capi_err_t capi_siso_rat_get_static_properties(capi_proplist_t *init_set_properties,
                                               capi_proplist_t *static_properties);

capi_err_t capi_siso_rat_init(capi_t *_pif, capi_proplist_t *init_set_properties);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* CAPI_SISO_RAT_H */
