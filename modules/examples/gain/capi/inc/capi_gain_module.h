#ifndef CAPI_GAIN_MODULE_H
#define CAPI_GAIN_MODULE_H

/**
 * \file capi_gain_module.h
 *  
 * \brief
 *  
 *     Example Gain Module
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/*------------------------------------------------------------------------
 * Include files
 * -----------------------------------------------------------------------*/
#include "capi.h"
#include "ar_defs.h"

#ifdef __cplusplus
extern "C"{
#endif /*__cplusplus*/


/*------------------------------------------------------------------------
 * Capi entry point functions
 * -----------------------------------------------------------------------*/
/**
* Get static properties of Gain module such as
* memory, stack requirements etc.
*/
capi_err_t capi_gain_module_get_static_properties(
      capi_proplist_t *init_set_properties,
      capi_proplist_t *static_properties);


/**
* Instantiates(and allocates) the module memory.
*/
capi_err_t capi_gain_module_init(
      capi_t          *_pif,
      capi_proplist_t *init_set_properties);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif //CAPI_GAIN_MODULE_H

