#ifndef __CAPI_RT_PROXY_H
#define __CAPI_RT_PROXY_H

/**
 *   \file capi_rt_proxy.h
 *   \brief
 *        This file contains CAPI API's published by RT Proxy CAPI intialization.
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "capi.h"

/*==============================================================================
   Function declarations
==============================================================================*/

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/*
  This function is used to query the static properties to create the RT Proxy CAPI module.

  param[in] init_set_prop_ptr: Pointer to the initializing property list
  param[in, out] static_prop_ptr: Pointer to the static property list

  return: CAPI_EOK(0) on success else failure error code
 */
capi_err_t capi_rt_proxy_rx_get_static_properties(capi_proplist_t *init_set_prop_ptr, capi_proplist_t *static_prop_ptr);

/*
  This function is used init the RT Proxy CAPI lib.

  param[in] capi_ptr: Pointer to the CAPI lib.
  param[in] init_set_prop_ptr: Pointer to the property list that needs to be
            initialized

  return: CAPI_EOK(0) on success else failure error code
*/
capi_err_t capi_rt_proxy_rx_init(capi_t *capi_ptr, capi_proplist_t *init_set_prop_ptr);

/*
  This function is used to query the static properties to create the RT Proxy CAPI module.

  param[in] init_set_prop_ptr: Pointer to the initializing property list
  param[in, out] static_prop_ptr: Pointer to the static property list

  return: CAPI_EOK(0) on success else failure error code
 */
capi_err_t capi_rt_proxy_tx_get_static_properties(capi_proplist_t *init_set_prop_ptr, capi_proplist_t *static_prop_ptr);

/*
  This function is used init the RT Proxy CAPI lib.

  param[in] capi_ptr: Pointer to the CAPI lib.
  param[in] init_set_prop_ptr: Pointer to the property list that needs to be
            initialized

  return: CAPI_EOK(0) on success else failure error code
*/
capi_err_t capi_rt_proxy_tx_init(capi_t *capi_ptr, capi_proplist_t *init_set_prop_ptr);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /*__CAPI_RT_PROXY_H*/
