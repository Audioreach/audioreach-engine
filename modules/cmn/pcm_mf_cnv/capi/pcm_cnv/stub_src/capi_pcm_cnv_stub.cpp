/* =========================================================================
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 * =========================================================================*/

/**
 * @file capi_pcm_cnv_stub.cpp
 *
 * Stub Interface for pcm_cnv.
 */

#include "capi_pcm_cnv.h"

capi_err_t capi_pcm_cnv_get_static_properties(capi_proplist_t *init_set_properties,
                                                    capi_proplist_t *static_properties)
{
   return CAPI_EUNSUPPORTED;
}

capi_err_t capi_pcm_cnv_init(capi_t *_pif, capi_proplist_t *init_set_properties)
{
   return CAPI_EUNSUPPORTED;
}

capi_err_t capi_pcm_enc_get_static_properties(capi_proplist_t *init_set_properties,
                                                    capi_proplist_t *static_properties)
{
   return CAPI_EUNSUPPORTED;
}
capi_err_t capi_pcm_enc_init(capi_t *_pif, capi_proplist_t *init_set_properties)
{
   return CAPI_EUNSUPPORTED;
}

capi_err_t capi_pcm_dec_get_static_properties(capi_proplist_t *init_set_properties,
                                                    capi_proplist_t *static_properties)
{
   return CAPI_EUNSUPPORTED;
}

capi_err_t capi_pcm_dec_init(capi_t *_pif, capi_proplist_t *init_set_properties)
{
   return CAPI_EUNSUPPORTED;
}
