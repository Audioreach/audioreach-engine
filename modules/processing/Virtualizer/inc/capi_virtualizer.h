/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/**
 * @file capi_virtualizer.h
 *
 * Header file to implement the Audio Post Processor Interface for
 * virtualization
 */

#ifndef CAPI_VIRTUALIZER_H
#define CAPI_VIRTUALIZER_H

#include "capi.h"
#include "ar_defs.h"

#ifdef __cplusplus
extern "C" {
#endif


//#define PARAM_ID_VIRTUALIZER_DIPOLE_SPACING      0x0000DEF0
//#define PARAM_ID_VIRTUALIZER_VOLUME_RAMP         0x0000DEF2
//#define PARAM_ID_VIRTUALIZER_DELAY               0x0000DEF3


/**
 * Get static properties of virtualizer module such as
 * memory, stack requirements etc.
 * See Elite_CAPI.h for more details.
 */
capi_err_t capi_virtualizer_get_static_properties(
        capi_proplist_t *init_set_properties,
        capi_proplist_t *static_properties);


/**
 * Instantiates(and allocates) the module memory.
 * See Elite_CAPI.h for more details.
 */
capi_err_t capi_virtualizer_init(
        capi_t          *_pif,
        capi_proplist_t *init_set_properties);

#ifdef __cplusplus
}
#endif

#endif // CAPI_VIRTUALIZER_H
