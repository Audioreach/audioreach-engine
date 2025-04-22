#ifndef CAPI_MM_ERROR_CODE_CONVERTER_H
#define CAPI_MM_ERROR_CODE_CONVERTER_H
/**
 * \file capi_mm_error_code_converter.h
 * \brief
 *      Utility header for converting between the CAPI error codes and the aDSP error codes.
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "capi_types.h"
#include "ar_error_codes.h"
#include "ar_osal_error.h"
static const struct ar_to_capi_err_t
{
   ar_result_t ar_err_code;
   capi_err_t  capi_err_code;
} ar_to_capi_err_lut[] = {
   { AR_EOK, CAPI_EOK },
   { AR_EFAILED, CAPI_EFAILED },
   { AR_EBADPARAM, CAPI_EBADPARAM },
   { AR_EUNSUPPORTED, CAPI_EUNSUPPORTED },
   { AR_ENOMEMORY, CAPI_ENOMEMORY },
   { AR_ENEEDMORE, CAPI_ENEEDMORE },
   { AR_ENOTREADY, CAPI_ENOTREADY },
   { AR_EALREADY, CAPI_EALREADY },
};

static inline ar_result_t capi_err_to_ar_result(capi_err_t capi_err)
{
   uint32_t i;

   if (CAPI_SUCCEEDED(capi_err))
   {
      return AR_EOK;
   }

   for (i = 1; i < sizeof(ar_to_capi_err_lut) / sizeof(ar_to_capi_err_lut[0]); ++i)
   {
      if (CAPI_IS_ERROR_CODE_SET(capi_err, ar_to_capi_err_lut[i].capi_err_code))
      {
         return ar_to_capi_err_lut[i].ar_err_code;
      }
   }

   return AR_EFAILED;
}

static inline capi_err_t ar_result_to_capi_err(ar_result_t ar_err)
{
   uint32_t i;
   for (i = 0; i < sizeof(ar_to_capi_err_lut) / sizeof(ar_to_capi_err_lut[0]); ++i)
   {
      if (ar_to_capi_err_lut[i].ar_err_code == ar_err)
      {
         return ar_to_capi_err_lut[i].capi_err_code;
      }
   }

   return CAPI_EFAILED;
}

#endif /* #ifndef CAPI_MM_ERROR_CODE_CONVERTER_H */
