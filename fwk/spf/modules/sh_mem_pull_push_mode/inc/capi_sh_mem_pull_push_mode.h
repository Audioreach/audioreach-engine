/* ======================================================================== */
/**
  @file capi_sh_mem_pull_push_mode.h
  @brief This file contains CAPI API's published by shared memory
         pull and push mode Module

  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
  SPDX-License-Identifier: BSD-3-Clause-Clear
==============================================================================*/

#ifndef CAPI_SH_MEM_PULL_PUSH_MODE_H
#define CAPI_SH_MEM_PULL_PUSH_MODE_H

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
capi_err_t capi_pull_mode_get_static_properties(capi_proplist_t *init_set_properties,
                                                      capi_proplist_t *static_properties);

capi_err_t capi_pull_mode_init(capi_t *_pif, capi_proplist_t *init_set_properties);

capi_err_t capi_push_mode_get_static_properties(capi_proplist_t *init_set_properties,
                                                      capi_proplist_t *static_properties);

capi_err_t capi_push_mode_init(capi_t *_pif, capi_proplist_t *init_set_properties);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* CAPI_SH_MEM_PULL_PUSH_MODE_H */
