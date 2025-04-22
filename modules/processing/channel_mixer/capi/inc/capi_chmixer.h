/* ======================================================================== */
/**
    @file capi_chmixer.h

    Header file to implement the Audio Post Processor Interface for
    Channel Mixer.
*/

/* =========================================================================
  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
  SPDX-License-Identifier: BSD-3-Clause-Clear
   ========================================================================= */

/*------------------------------------------------------------------------
 * Include files and Macro definitions
 * -----------------------------------------------------------------------*/
#ifndef CAPI_CHMIXER_H
#define CAPI_CHMIXER_H

#include "capi.h"
#include "ar_defs.h"

/*------------------------------------------------------------------------
* Macros, Defines, Type declarations
* -----------------------------------------------------------------------*/
#define CAPI_CHMIXER_PARAM_ID_ENABLE (0)

/*------------------------------------------------------------------------
 * Structure definitions
 * -----------------------------------------------------------------------*/
/** Payload for CHANNELMIXER_PARAM_ID_ENABLE. */
typedef struct capi_chmixer_enable_payload
{
   uint32_t                          enable;               /**< Defined the state of the module. */
} capi_chmixer_enable_payload_t;

/*------------------------------------------------------------------------
* Function declarations
* -----------------------------------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif

capi_err_t capi_chmixer_get_static_properties (capi_proplist_t *init_set_properties, capi_proplist_t *static_properties);

capi_err_t capi_chmixer_init (capi_t *_pif, capi_proplist_t *init_set_properties);

#ifdef __cplusplus
}
#endif

#endif //CAPI_CHMIXER_H

