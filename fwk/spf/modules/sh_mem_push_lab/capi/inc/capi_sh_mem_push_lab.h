/* ======================================================================== */
/**
  @file capi_sh_mem_push_lab.h
  @brief This file contains CAPI API's published by shared memory
         push lab Module

  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
  SPDX-License-Identifier: BSD-3-Clause-Clear
==============================================================================*/

#ifndef CAPI_SH_MEM_PUSH_LAB_H
#define CAPI_SH_MEM_PUSH_LAB_H

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

capi_err_t capi_sh_mem_push_lab_get_static_properties(capi_proplist_t *init_set_properties,
                                                      capi_proplist_t *static_properties);

capi_err_t capi_sh_mem_push_lab_init(capi_t *_pif, capi_proplist_t *init_set_properties);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* CAPI_SH_MEM_PUSH_LAB_H */
