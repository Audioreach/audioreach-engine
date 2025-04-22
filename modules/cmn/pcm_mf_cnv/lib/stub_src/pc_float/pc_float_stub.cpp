
/*========================================================================

 file pc_float_stub.cpp
This file contains stub functions for data format conversions

Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
SPDX-License-Identifier: BSD-3-Clause-Clear
======================================================================*/

/* =======================================================================
INCLUDE FILES FOR MODULE
========================================================================== */

#include "pc_converter.h"

ar_result_t pc_float_to_fixed_conv_process(void *          me_ptr,
                                           capi_buf_t *    input_buf_ptr,
                                           capi_buf_t *    output_buf_ptr,
                                           pc_media_fmt_t *input_media_fmt_ptr,
                                           pc_media_fmt_t *output_media_fmt_ptr)
{
   ar_result_t result = AR_EUNSUPPORTED;
   return result;
}

ar_result_t pc_fixed_to_float_conv_process(void *          me_ptr,
                                           capi_buf_t *    input_buf_ptr,
                                           capi_buf_t *    output_buf_ptr,
                                           pc_media_fmt_t *input_media_fmt_ptr,
                                           pc_media_fmt_t *output_media_fmt_ptr)

{
   ar_result_t result = AR_EUNSUPPORTED;
   return result;
}

bool_t pc_is_floating_point_data_format_supported()
{
   return FALSE;
}
