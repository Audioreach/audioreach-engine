#ifndef CAPI_INTF_EXTN_PERIOD_H
#define CAPI_INTF_EXTN_PERIOD_H

/**
 *  \file capi_intf_extn_period.h
 *  \brief
 *        Interface extensions related to period length (VFR cycle duration).
 *
 *        This file defines interface extensions that allow modules to
 *        get VFR cycle duration from the framework.
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

/** @addtogroup capi_if_ext_period
@{ */

/** Unique identifier to get the VFR cycle duration. */
#define INTF_EXTN_PERIOD 0x0A001060

/** ID of the parameter the framework uses to inform VFR cycle duration to
    the module.

    @msgpayload{intf_extn_period_t}
    @tablens{weak__intf__extn__period__t}
*/
#define INTF_EXTN_PARAM_ID_PERIOD 0x0A001061

typedef struct intf_extn_param_id_period_t intf_extn_param_id_period_t;

/** @weakgroup weak_intf_extn_period_t
@{ */

struct intf_extn_param_id_period_t
{
   uint32_t period_us;
   /** VFR cycle duration. */
};

/** @} */ /* end_weakgroup weak_intf_extn_period_t */

/** @} */ /* end_addtogroup capi_if_ext_period */

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* CAPI_INTF_EXTN_PERIOD_H */
