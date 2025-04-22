/* ======================================================================== */
/**
  @file capi_alsa_device.h
  @brief This file contains CAPI API's published by alsa device module.

  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
\  SPDX-License-Identifier: BSD-3-Clause-Clear
==============================================================================*/

#ifndef CAPI_ALSA_DEVICE_H
#define CAPI_ALSA_DEVICE_H

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

capi_err_t capi_alsa_device_source_init(
   capi_t *_pif,
   capi_proplist_t *init_set_properties);

capi_err_t capi_alsa_device_source_get_static_properties(
   capi_proplist_t *init_set_properties,
   capi_proplist_t *static_properties);

capi_err_t capi_alsa_device_sink_init(
   capi_t *_pif,
   capi_proplist_t *init_set_properties);

capi_err_t capi_alsa_device_sink_get_static_properties(
   capi_proplist_t *init_set_properties,
   capi_proplist_t *static_properties);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif // CAPI_ALSA_DEVICE_H