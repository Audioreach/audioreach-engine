/**
 * \file capi_splitter.h
 * \brief
 *        Header file containing the exposed CAPI APIs for the Simple Splitter (SPLITTER) module
 *  
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef CAPI_SPLITTER_H
#define CAPI_SPLITTER_H

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
capi_err_t capi_splitter_get_static_properties(capi_proplist_t *init_set_properties,
                                               capi_proplist_t *static_properties);

capi_err_t capi_splitter_init(capi_t *_pif, capi_proplist_t *init_set_properties);

#ifdef __cplusplus
}
#endif /*__cplusplus*/
#endif // CAPI_SPLITTER_H