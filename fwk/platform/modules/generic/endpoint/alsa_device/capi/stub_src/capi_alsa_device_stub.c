/* ========================================================================
  @file capi_alsa_device_stub.c
  @brief This file contains CAPI stub implementation of ALSA device module.

  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
  SPDX-License-Identifier: BSD-3-Clause-Clear
==============================================================================*/

/*--------------------------------------------------------------------------
 * Include files and Macro definitions
 * ------------------------------------------------------------------------ */
#include "capi.h"
#include "capi_alsa_device.h"

capi_err_t capi_alsa_device_source_init(
   capi_t *_pif,
   capi_proplist_t *init_set_properties)
{
   return CAPI_EUNSUPPORTED;
}

capi_err_t capi_alsa_device_source_get_static_properties(
   capi_proplist_t *init_set_properties,
   capi_proplist_t *static_properties)
{
   return CAPI_EUNSUPPORTED;
}

capi_err_t capi_alsa_device_sink_init(
   capi_t *_pif,
   capi_proplist_t *init_set_properties)
{
   return CAPI_EUNSUPPORTED;
}

capi_err_t capi_alsa_device_sink_get_static_properties(
   capi_proplist_t *init_set_properties,
   capi_proplist_t *static_properties)
{
   return CAPI_EUNSUPPORTED;
}