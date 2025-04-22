#ifndef __CAPI_JITTER_BUF_H
#define __CAPI_JITTER_BUF_H

/**
 *   \file capi_jitter_buf.h
 *   \brief
 *        This file contains CAPI API's published by Jitter Buffer CAPI intialization.
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
  This function is used to query the static properties to create the Jitter Buffer CAPI module.

  param[in] init_set_prop_ptr: Pointer to the initializing property list
  param[in, out] static_prop_ptr: Pointer to the static property list

  return: CAPI_EOK(0) on success else failure error code
 */
capi_err_t capi_jitter_buf_get_static_properties(capi_proplist_t *init_set_prop_ptr, capi_proplist_t *static_prop_ptr);

/*
  This function is used init the Jitter Buffer CAPI lib.

  param[in] capi_ptr: Pointer to the CAPI lib.
  param[in] init_set_prop_ptr: Pointer to the property list that needs to be
            initialized

  return: CAPI_EOK(0) on success else failure error code
*/
capi_err_t capi_jitter_buf_init(capi_t *capi_ptr, capi_proplist_t *init_set_prop_ptr);

/*
  This function is used to query the static properties to create the Jitter Buffer CAPI module.

  param[in] init_set_prop_ptr: Pointer to the initializing property list
  param[in, out] static_prop_ptr: Pointer to the static property list

  return: CAPI_EOK(0) on success else failure error code
 */

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /*__CAPI_JITTER_BUF_H*/
