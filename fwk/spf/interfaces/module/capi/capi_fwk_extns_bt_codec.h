#ifndef ELITE_FWK_EXTNS_BT_CODEC_H
#define ELITE_FWK_EXTNS_BT_CODEC_H

/**
 *   \file capi_fwk_extns_bt_codec.h
 *   \brief
 *        Parameters required to be implemented by BT codec.
 *    This file defines a framework extension for BT codec.
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

/** @addtogroup capi_fw_ext_bt_codec
The Bluetooth framework extension (FWK_EXTN_BT_CODEC) provides special events
that are required to enable Bluetooth codecs.
*/

/** @addtogroup capi_fw_ext_bt_codec
@{ */

/** Unique identifier of the Bluetooth framework extension for a module.

    This extension supports the following events:
    - #CAPI_BT_CODEC_EXTN_EVENT_ID_DISABLE_PREBUFFER
    - #CAPI_BT_CODEC_EXTN_EVENT_ID_KPPS_SCALE_FACTOR
 */
#define FWK_EXTN_BT_CODEC 0x000132e4

/*------------------------------------------------------------------------------
 * Events
 *---------------------------------------------------------------------------*/

/** ID of the event the encoder module uses to disable pre-buffering.

    This event must be raised during CAPI initialization before data
    processing.

    @msgpayload{capi_bt_codec_extn_event_disable_prebuffer_t}
    @table{weak__capi__bt__codec__extn__event__disable__prebuffer__t}

    @sa #CAPI_EVENT_DATA_TO_DSP_SERVICE @newpage
 */
#define CAPI_BT_CODEC_EXTN_EVENT_ID_DISABLE_PREBUFFER 0x000132e5

/* Payload structure for the zero padding CAPI_BT_CODEC_EXTN_EVENT_ID_DISABLE_PREBUFFER
  */
typedef struct capi_bt_codec_extn_event_disable_prebuffer_t capi_bt_codec_extn_event_disable_prebuffer_t;

/** @weakgroup weak_capi_bt_codec_extn_event_disable_prebuffer_t
@{ */
struct capi_bt_codec_extn_event_disable_prebuffer_t
{
   uint32_t disable_prebuffering;
   /**< Specifies whether to disable pre-buffering.

        @valuesbul
        - @ge 1 -- Disable pre-buffering
        - 0 -- Enable pre-buffering @tablebulletend */
};
/** @} */ /* end_weakgroup weak_capi_bt_codec_extn_event_disable_prebuffer_t */

/** ID of the event the encoder module uses to set the KPPS scale factor.

    This scale factor increases the clock speed so the processing time of the
    encoder catches up with the real time. It is the factor by which the clock
    speed must be increased.

    This event can be raised by the module any time.

    @note1hang KPPS scaling does not scale the processing by the exact value.
               It will be lower than the factor due to thread pre-emptions and
               relative thread priorities in the system.

    @msgpayload{capi_bt_codec_etxn_event_kpps_scale_factor_t}
    @table{weak__capi__bt__codec__etxn__event__kpps__scale__factor__t}

    @sa #CAPI_EVENT_DATA_TO_DSP_SERVICE
 */
#define CAPI_BT_CODEC_EXTN_EVENT_ID_KPPS_SCALE_FACTOR 0x000132e7

/* Payload structure for the zero padding
   CAPI_BT_CODEC_EXTN_EVENT_ID_KPPS_SCALE_FACTOR
  */
typedef struct capi_bt_codec_etxn_event_kpps_scale_factor_t capi_bt_codec_etxn_event_kpps_scale_factor_t;

/** @weakgroup weak_capi_bt_codec_etxn_event_kpps_scale_factor_t
@{ */
struct capi_bt_codec_etxn_event_kpps_scale_factor_t
{
   uint32_t scale_factor;
   /**< Scale factor for KPPS voting (it can be a decimal number).

        @valuesbul
        - Bits 31 to 4 -- Integral part of the decimal number
        - Bits 0 to 3 -- Fractional part of the decimal number @tablebulletend */
};
/** @} */ /* end_weakgroup weak_capi_bt_codec_etxn_event_kpps_scale_factor_t */

/** @} */ /* end_addtogroup capi_fw_ext_bt_codec */

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* #ifndef ELITE_FWK_EXTNS_BT_CODEC_H */
