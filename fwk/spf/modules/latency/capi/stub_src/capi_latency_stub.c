/* ======================================================================== */
/**
   @file capi_latency.cpp

   Common Audio Post Processor Interface stub source file for compressed latency
*/

/* =========================================================================
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
   ========================================================================= */

/*------------------------------------------------------------------------
 * Include files and Macro definitions
 * -----------------------------------------------------------------------*/
#include "capi.h"
#include "capi_latency.h"

/*------------------------------------------------------------------------
  Function name: capi_latency_get_static_properties
  DESCRIPTION: Function to get the static properties for latency module
  -----------------------------------------------------------------------*/
capi_err_t capi_latency_get_static_properties(capi_proplist_t *init_set_properties, capi_proplist_t *static_properties)
{
   return CAPI_EUNSUPPORTED;
}

/*------------------------------------------------------------------------
  Function name: capi_latency_init
  DESCRIPTION: Initialize the latency module and library.
  -----------------------------------------------------------------------*/
capi_err_t capi_latency_init(capi_t *_pif, capi_proplist_t *init_set_properties)
{
   return CAPI_EUNSUPPORTED;
}
