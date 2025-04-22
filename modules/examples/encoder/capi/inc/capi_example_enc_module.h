#ifndef CAPI_EXAMPLE_ENC_MODULE_H
#define CAPI_EXAMPLE_ENC_MODULE_H
/**
 * \file example_encoder_module.h
 *  
 * \brief
 *  
 *     Example Encoder Module
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/*------------------------------------------------------------------------
 * Includes
 * -----------------------------------------------------------------------*/
#include "capi.h"
#include "ar_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

/*------------------------------------------------------------------------
 * Capi entry point functions
 * -----------------------------------------------------------------------*/
/** @ingroup
Used to query for static properties of the module that are independent of the
instance. This function is used to query the memory requirements of the module
in order to create an instance. The same properties that are sent to the module
in the call to init() are also sent to this function to enable the module to
calculate the memory requirement.

@param[in]  init_set_properties  The same properties that will be sent in the
                         call to the init() function.
@param[out] static_properties   Pointer to the structure that the module must
                         fill with the appropriate values based on the property
                         id.

@return
Indication of success or failure.

*/
capi_err_t capi_example_enc_module_get_static_properties(capi_proplist_t *init_set_properties,
                                                         capi_proplist_t *static_properties);

/** @ingroup
Instantiates the module to set up the virtual function table, and also
allocates any memory required by the module. States within the module must
be initialized at the same time.

@param[in,out] _pif  Pointer to the module object.
@param[in]    init_set_properties Properties set by the service to be used
                              while init().

@return
Indication of success or failure.

*/
capi_err_t capi_example_enc_module_init(capi_t *_pif, capi_proplist_t *init_set_properties);

#ifdef __cplusplus
} // extern "C"
#endif

#endif /* CAPI_EXAMPLE_ENC_MODULE_H */
