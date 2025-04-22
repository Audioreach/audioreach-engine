/* ======================================================================== */
/*
@file capi_multistageiir_stub.cpp.cpp

   Source file to implement the Audio Post Processor Interface for Multi-Stage
   IIR filters
*/

/* =========================================================================
  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
  SPDX-License-Identifier: BSD-3-Clause-Clear
  ========================================================================= */

/*------------------------------------------------------------------------
 * Include files
 * -----------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include "capi.h"

/*------------------------------------------------------------------------
 * Static declarations
 * -----------------------------------------------------------------------*/

/**
 * Get static properties of multistage IIR module such as
 * memory, stack requirements etc.
 * See Elite_CAPI.h for more details.
 */
capi_err_t capi_multistageiir_get_static_properties( capi_proplist_t* init_set_properties,
                                                           capi_proplist_t* static_properties)
{
   ;
   return CAPI_EUNSUPPORTED;
}

/**
 * Instantiates(and allocates) the module memory.
 * See Elite_CAPI.h for more details.
 */
capi_err_t capi_multistageiir_init( capi_t*          _pif,
                                          capi_proplist_t* init_set_properties)
{
   
   return CAPI_EUNSUPPORTED;
}


capi_err_t capi_multistageiir_left_init(
      capi_t*          _pif,
      capi_proplist_t* init_set_properties)
{
	return CAPI_EUNSUPPORTED;
}

capi_err_t capi_multistageiir_right_init(
      capi_t*          _pif,
      capi_proplist_t* init_set_properties)
{
	return CAPI_EUNSUPPORTED;
}

capi_err_t capi_multistageiir_left_get_static_properties(
      capi_proplist_t* init_set_properties,
      capi_proplist_t* static_properties)
{
	return CAPI_EUNSUPPORTED;
}

capi_err_t capi_multistageiir_right_get_static_properties(
      capi_proplist_t* init_set_properties,
      capi_proplist_t* static_properties)
{
	return CAPI_EUNSUPPORTED;
}
	

#ifdef __cplusplus
}
#endif /* __cplusplus */


