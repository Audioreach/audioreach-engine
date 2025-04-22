/* ======================================================================== */
/**
@file capi_mfc.h

   Header file to implement the media format convertor block*/

/* =========================================================================
  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
  SPDX-License-Identifier: BSD-3-Clause-Clear
  ========================================================================== */

/*------------------------------------------------------------------------
 * Include files
 * -----------------------------------------------------------------------*/
#ifndef CAPI_MFC_H
#define CAPI_MFC_H

#include "capi.h"
#include "ar_defs.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

capi_err_t capi_mfc_get_static_properties(capi_proplist_t *init_set_properties,
                                                capi_proplist_t *static_properties);
capi_err_t capi_mfc_init(capi_t *_pif, capi_proplist_t *init_set_properties);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif // CAPI_MFC_H
