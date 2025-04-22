#ifndef ELITE_FWK_EXTNS_PCM_H
#define ELITE_FWK_EXTNS_PCM_H

/**
 *   \file capi_fwk_extns_pcm.h
 *   \brief
 *        fwk extension to take care of PCM use cases.
 *
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/*------------------------------------------------------------------------------
 * Include Files
 *----------------------------------------------------------------------------*/
#include "capi_types.h"

/** @addtogroup capi_fw_ext_pcm
The PCM framework extension (FWK_EXTN_PCM) is used for specific PCM use cases.
The PCM modules (such as converter, decoder, and encoder) require the framework
to support extended media formats and setting performance modes.
*/

/** @addtogroup capi_fw_ext_pcm
@{ */

/** Unique identifier of the framework extension for PCM modules. */
#define FWK_EXTN_PCM 0x0A001000

/*------------------------------------------------------------------------------
 * Parameter IDs
 *----------------------------------------------------------------------------*/

/** ID of the parameter that defines the extension to the media format.

    For an input media format:
     - This parameter is always set before #CAPI_INPUT_MEDIA_FORMAT or
       #CAPI_INPUT_MEDIA_FORMAT_V2.
     - Information from this format and the event must be handled in tandem.

    For an output media format:
     - This parameter is always queried after the
       #CAPI_EVENT_OUTPUT_MEDIA_FORMAT_UPDATED or
       #CAPI_EVENT_OUTPUT_MEDIA_FORMAT_UPDATED_V2 event, or after the
       #CAPI_OUTPUT_MEDIA_FORMAT property query.
     - Information from this format and the event or property query are handled
       in tandem.

     @msgpayload{fwk_extn_pcm_param_id_media_fmt_extn_t}
     @table{weak__fwk__extn__pcm__param__id__media__fmt__extn__t}
 */
#define FWK_EXTN_PCM_PARAM_ID_MEDIA_FORMAT_EXTN 0x0A001001

typedef struct fwk_extn_pcm_param_id_media_fmt_extn_t fwk_extn_pcm_param_id_media_fmt_extn_t;

/** @weakgroup weak_fwk_extn_pcm_param_id_media_fmt_extn_t
@{ */
struct fwk_extn_pcm_param_id_media_fmt_extn_t
{
   uint32_t bit_width;
   /**< Width of the sample word size.

        A CAPI media format has a bits_per_sample element
        (#capi_standard_data_format_v2_t) that stands for the sample word size.
        For example, if the bit_width is 24 bits, the sample word size is
        32 in Q27 format. @vtspstrbul
        - 24-bit bit_width data placed in 24 bits has a sample word size of
          24 (packed).
        - 24-bit bit_width data placed in 32 bits has a sample word size of
          32 (unpacked).

        Packing can be done in two ways: MSB aligned or LSB aligned.
        @vtspstrbul
        - For MSB-aligned, the Q factor is 31 in
          #capi_standard_data_format_v2_t, (set q_factor to
          #CAPI_DATA_FORMAT_INVALID_VAL).
        - For LSB aligned, the Q factor is 23.

        @contcell
        If the format is Q27, the actual bits_per_sample is 24.

        32-bit bit_width data can be in Q31 format. @vtspstrbul
        - The word size is always 32.
        - The alignment is #CAPI_DATA_FORMAT_INVALID_VAL.

        16-bit bit_width data can be in Q15 format. @vtspstrbul
        - The word size is 16 or 32.
        - If 16, alignment is #CAPI_DATA_FORMAT_INVALID_VAL.
        - If 32, alignment can be MSB or LSB aligned.

        An invalid value = #CAPI_DATA_FORMAT_INVALID_VAL. */

   uint32_t alignment;
   /**< Alignment of samples in a word.

        @valuesbul
        - PCM_LSB_ALIGNED
        - PCM_MSB_ALIGNED

        An invalid value = #CAPI_DATA_FORMAT_INVALID_VAL */

   uint32_t endianness;
   /**< Endianness of the data.

        @valuesbul
        - PCM_LITTLE_ENDIAN
        - PCM_BIG_ENDIAN

        An invalid value = #CAPI_DATA_FORMAT_INVALID_VAL */
};
/** @} */ /* end_weakgroup weak_fwk_extn_pcm_param_id_media_fmt_extn_t */

/** @} */ /* end_addtogroup capi_fw_ext_pcm */

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* #ifndef ELITE_FWK_EXTNS_PCM_H*/
