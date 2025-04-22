/* ========================================================================
  @file capi_demuxer_stub.c
  @brief This file contains CAPI Stub implementation of Demuxer Module

  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
  SPDX-License-Identifier: BSD-3-Clause-Clear
==============================================================================*/

/*--------------------------------------------------------------------------
 * Include files and Macro definitions
 * ------------------------------------------------------------------------ */
#include "capi_demuxer.h"

capi_err_t capi_demuxer_get_static_properties(capi_proplist_t *init_set_properties,
                                                  capi_proplist_t *static_properties)
{
   return CAPI_EOK;
}

capi_err_t capi_demuxer_init(capi_t *_pif, capi_proplist_t *init_set_properties)
{
   return CAPI_EOK;
}
