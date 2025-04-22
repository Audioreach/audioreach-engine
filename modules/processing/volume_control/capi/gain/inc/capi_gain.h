/* ======================================================================== */
/**
@file capi_gain.h

   Header file to implement the Common Audio Post Processor Interface
   for Tx/Rx Tuning Gain block
*/

/* =========================================================================
  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
  SPDX-License-Identifier: BSD-3-Clause-Clear
  ========================================================================== */

/*------------------------------------------------------------------------
 * Include files
 * -----------------------------------------------------------------------*/
#ifndef CAPI_GAIN_H
#define CAPI_GAIN_H

#include "capi.h"
#include "ar_defs.h"

#ifdef __cplusplus
extern "C"{
#endif /*__cplusplus*/

/**
* Get static properties of Gain module such as
* memory, stack requirements etc.
*/
capi_err_t capi_gain_get_static_properties(
      capi_proplist_t *init_set_properties,
      capi_proplist_t *static_properties);


/**
* Instantiates(and allocates) the module memory.
*/
capi_err_t capi_gain_init(
      capi_t          *_pif,
      capi_proplist_t *init_set_properties);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif //CAPI_GAIN_H

