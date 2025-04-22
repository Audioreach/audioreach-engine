/* ========================================================================
  @file capi_sh_mem_pull_push_mode_stub.c
  @brief This file contains CAPI Stub implementation of shared memory
         pull and push mode Module

  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
  SPDX-License-Identifier: BSD-3-Clause-Clear
==============================================================================*/

/*--------------------------------------------------------------------------
  * Include files and Macro definitions
  * ------------------------------------------------------------------------ */
#include "capi.h"

capi_err_t capi_pull_mode_get_static_properties(capi_proplist_t *init_set_properties,
                                                      capi_proplist_t *static_properties)
{
   return CAPI_EOK;
}

capi_err_t capi_pull_mode_init(capi_t *_pif, capi_proplist_t *init_set_properties)
{
   return CAPI_EOK;
}

capi_err_t capi_push_mode_get_static_properties(capi_proplist_t *init_set_properties,
                                                      capi_proplist_t *static_properties)
{
   return CAPI_EOK;
}

capi_err_t capi_push_mode_init(capi_t *_pif, capi_proplist_t *init_set_properties)
{
   return CAPI_EOK;
}
