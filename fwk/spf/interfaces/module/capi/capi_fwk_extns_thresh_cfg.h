#ifndef CAPI_FWK_EXTNS_THRESH_CFG_H
#define CAPI_FWK_EXTNS_THRESH_CFG_H

/**
 *   \file capi_fwk_extns_thresh_cfg.h
 *   \brief
 *        framework extension for modules to receive threshold configuration.
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

/** @addtogroup capi_fw_ext_threshold
@{ */

/** Unique identifier of the framework extension that modules use to get the
    nominal frame duration (in microseconds) from the framework.

    Module using this extension should raise port threshold to avoid conflict between module's setting and actual
    container frame size which can also depends on other threshold modules or container property.
 */
#define FWK_EXTN_THRESHOLD_CONFIGURATION 0x0A00104D

/*------------------------------------------------------------------------------
 * Parameter IDs
 *----------------------------------------------------------------------------*/

/** ID of the parameter used configure the threshold based on the performance
    mode of the graph  or container frame size property.

    @msgpayload{fwk_extn_param_id_threshold_cfg_t}
    @table{weak__fwk__extn__param__id__threshold__cfg__t}
 */
#define FWK_EXTN_PARAM_ID_THRESHOLD_CFG 0x0A00104E

typedef struct fwk_extn_param_id_threshold_cfg_t fwk_extn_param_id_threshold_cfg_t;

/** @weakgroup weak_fwk_extn_param_id_threshold_cfg_t
@{ */
struct fwk_extn_param_id_threshold_cfg_t
{
   uint32_t duration_us;
   /**< Threshold configuration (in microseconds) based on either the performance mode
        of the graph or container frame size property.

        The actual container frame duration might be different depending on
        other threshold modules. See #FWK_EXTN_CONTAINER_FRAME_DURATION. */
};
/** @} */ /* end_weakgroup weak_fwk_extn_param_id_threshold_cfg_t */

/** @} */ /* end_addtogroup capi_fw_ext_threshold */

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* #ifndef CAPI_FWK_EXTNS_THRESH_CFG_H*/
