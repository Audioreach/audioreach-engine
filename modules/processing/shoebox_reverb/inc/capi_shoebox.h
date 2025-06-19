/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/**
 * @file capi_shoebox.h
 *
 * Header file to implement the Audio Post Processor Interface for
 * Shoebox module
 */

#ifndef CAPI_SHOEBOX_H
#define CAPI_SHOEBOX_H

#include "capi.h"
#include "ar_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Get static properties of shoebox module such as
 * memory, stack requirements etc.
 * See Elite_CAPI.h for more details.
 */
capi_err_t capi_shoebox_get_static_properties(capi_proplist_t *init_set_properties,
                                                    capi_proplist_t *static_properties);

/**
 * Instantiates(and allocates) the module memory.
 * See Elite_CAPI.h for more details.
 */
capi_err_t capi_shoebox_init(capi_t *_pif, capi_proplist_t *init_set_properties);

/**
 * Get static properties of reverb module such as
 * memory, stack requirements etc.
 * See Elite_CAPI.h for more details.
 */
capi_err_t capi_reverb_get_static_properties(capi_proplist_t *init_set_properties,
                                                   capi_proplist_t *static_properties);

/**
 * Instantiates(and allocates) the module memory.
 * See Elite_CAPI.h for more details.
 */
capi_err_t capi_reverb_init(capi_t *_pif, capi_proplist_t *init_set_properties);

#ifdef __cplusplus
}
#endif

#endif /* CAPI_SHOEBOX_H */
