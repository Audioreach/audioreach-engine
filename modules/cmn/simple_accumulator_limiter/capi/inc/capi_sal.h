/* ======================================================================== */
/**
   @file capi_sal.h

   Header file containing the exposed CAPI APIs for the Simple Accumulator-Limiter module
*/

/* =========================================================================
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear
   ========================================================================== */

#ifndef CAPI_SAL_H
#define CAPI_SAL_H

/*------------------------------------------------------------------------
 * Include files
 * -----------------------------------------------------------------------*/
#include "capi.h"
#ifdef __cplusplus
extern "C"
{
#endif /*__cplusplus*/
/*------------------------------------------------------------------------
 * Function declarations
 * ----------------------------------------------------------------------*/
capi_err_t capi_sal_get_static_properties(capi_proplist_t *init_set_properties,
                                                          capi_proplist_t *static_properties);

capi_err_t capi_sal_init(capi_t *_pif, capi_proplist_t *init_set_properties);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif // CAPI_SAL_H