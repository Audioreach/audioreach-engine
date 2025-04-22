/**
 * \file capi_mux_demux.h
 *
 * \brief
 *        CAPI API wrapper for MUX-DEMUX module.
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef CAPI_MUX_DEMUX_H_
#define CAPI_MUX_DEMUX_H_

#if defined(__cplusplus)
extern "C" {
#endif // __cplusplus

/*----------------------------------------------------------------------------
 * Include Files
 * -------------------------------------------------------------------------*/
#include "capi.h"
#include "posal_intrinsics.h"
/*----------------------------------------------------------------------------
 * Function Declarations
 * -------------------------------------------------------------------------*/

capi_err_t capi_mux_demux_get_static_properties(capi_proplist_t *init_set_properties,
                                                capi_proplist_t *static_properties);

capi_err_t capi_mux_demux_init(capi_t *_pif, capi_proplist_t *init_set_properties);

#if defined(__cplusplus)
}
#endif // __cplusplus

#endif /* CAPI_MUX_DEMUX_H_ */
