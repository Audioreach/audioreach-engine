/**
@file capi_drc.h

@brief CAPI V2 API wrapper for drc algorithm

*/

/*-----------------------------------------------------------------------
Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
SPDX-License-Identifier: BSD-3-Clause-Clear
-----------------------------------------------------------------------*/

#ifndef CAPI_DRC
#define CAPI_DRC

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

/*----------------------------------------------------------------------------
 * Include Files
 * -------------------------------------------------------------------------*/
#include "capi.h"

/*----------------------------------------------------------------------------
 * Function Declarations
 * -------------------------------------------------------------------------*/

capi_err_t capi_drc_get_static_properties(capi_proplist_t *init_set_properties,
                                                capi_proplist_t *static_properties);

capi_err_t capi_drc_init(capi_t *_pif, capi_proplist_t *init_set_properties);

#ifdef __cplusplus
}
#endif //__cplusplus
#endif // CAPI_DRC
