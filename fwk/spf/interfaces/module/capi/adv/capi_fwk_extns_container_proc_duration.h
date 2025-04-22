#ifndef CAPI_FWK_EXTNS_CONT_PROC_DELAY_H
#define CAPI_FWK_EXTNS_CONT_PROC_DELAY_H

/**
 * \file capi_fwk_extns_container_proc_duration.h
 * \brief
 *    Framework extension for the modules to receive container processing duration
 *
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "ar_defs.h"

/** @addtogroup capi_fw_ext_cont_proc_dur
@{ */

/** Unique identifier of the interface extension that modules use to receive
    container processing duration. The modules use this extension to get the
    container processing delay from the framework (see
    #FWK_EXTN_CONTAINER_FRAME_DURATION).

    Typically, the container processing duration and container frame duration
    are the same except when a floor clock is being voted for faster
    processing.
 */
#define FWK_EXTN_CONTAINER_PROC_DURATION 0x0A001043

/*------------------------------------------------------------------------------
 * Parameter IDs
 *----------------------------------------------------------------------------*/

/** ID of the parameter that sets the container processing delay to the
    modules.

    @msgpayload{fwk_extn_param_id_container_proc_duration_t}
    @table{weak__fwk__extn__param__id__container__proc__duration__t}
 */
#define FWK_EXTN_PARAM_ID_CONTAINER_PROC_DURATION 0x0A001044

typedef struct fwk_extn_param_id_container_proc_duration_t fwk_extn_param_id_container_proc_duration_t;
/** @weakgroup weak_fwk_extn_param_id_container_proc_duration_t
@{ */
struct fwk_extn_param_id_container_proc_duration_t
{
   uint32_t proc_duration_us;
   /**< Container processing delay in microseconds. */
};
/** @} */ /* end_weakgroup weak_fwk_extn_param_id_container_proc_duration_t */

/** @} */ /* end_addtogroup capi_fw_ext_cont_proc_dur */

#endif // CAPI_FWK_EXTNS_CONT_PROC_DELAY_H
