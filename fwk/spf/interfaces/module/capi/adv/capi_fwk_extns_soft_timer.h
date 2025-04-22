#ifndef _CAPI_FWK_EXTNS_SOFT_TIMER_H_
#define _CAPI_FWK_EXTNS_SOFT_TIMER_H_

/**
 *   \file capi_fwk_extns_soft_timer.h
 *   \brief
 *        This file contains CAPI Soft timer Extension Definitions
 *
 *    Framework extension for timers which can be used to start, disable timers and
 *    send a set param to corresponding module when timer expires. "soft" here means
 *    the timer is not expected to be precise since it will run in the same thread
 *    as the framework process() call and have lower priority
 *
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "capi_types.h"

/** @addtogroup capi_fw_ext_soft_timer
The CAPI soft timer framework extension (FWK_EXTN_SOFT_TIMER) defines the soft
timers that are used to start and disable timers and send a
capi_vtbl_t::set_param() to the corresponding module when the timer expires.

In this case, <i>soft</i> means the timer is not expected to be precise because
it runs in the same thread as the framework capi_vtbl_t::process() call and it
has a lower priority.
*/

/** @addtogroup capi_fw_ext_soft_timer
@{ */

/*==============================================================================
   Constants
==============================================================================*/

/** Unique identifier of the framework extension for soft timers.
 */
#define FWK_EXTN_SOFT_TIMER 0x0A001008

/*==============================================================================
   Constants
==============================================================================*/

/** ID of the custom event used to notify the framework to start a timer.

    @msgpayload{fwk_extn_event_id_soft_timer_start_t}
    @table{weak__fwk__extn__event__id__soft__timer__start__t}
 */
#define FWK_EXTN_EVENT_ID_SOFT_TIMER_START 0x0A001009

/*==============================================================================
   Type definitions
==============================================================================*/

/* Structure defined for above Property  */
typedef struct fwk_extn_event_id_soft_timer_start_t fwk_extn_event_id_soft_timer_start_t;

/** @weakgroup weak_fwk_extn_event_id_soft_timer_start_t
@{ */
struct fwk_extn_event_id_soft_timer_start_t
{
   uint32_t timer_id;
   /**< Identifies the specific timer to be started.

        @values 0 to 15000 */

   uint32_t duration_ms;
   /**< Indicates the duration when the timer is to finish (in milliseconds).

        @values 0 to 15000 */
};
/** @} */ /* end_weakgroup weak_fwk_extn_event_id_soft_timer_start_t */

/*==============================================================================
   Constants
==============================================================================*/

/** ID of the custom event used to notify the framework to disable the timer.

    @msgpayload{fwk_extn_event_id_soft_timer_disable_t}
    @table{weak__fwk__extn__event__id__soft__timer__disable__t}
 */
#define FWK_EXTN_EVENT_ID_SOFT_TIMER_DISABLE 0x0A00100A

/*==============================================================================
   Type definitions
==============================================================================*/

typedef struct fwk_extn_event_id_soft_timer_disable_t fwk_extn_event_id_soft_timer_disable_t;

/** @weakgroup weak_fwk_extn_event_id_soft_timer_disable_t
@{ */
struct fwk_extn_event_id_soft_timer_disable_t
{
   uint32_t timer_id; /**< Identifies the specific timer to be disabled. */
};
/** @} */ /* end_weakgroup weak_fwk_extn_event_id_soft_timer_disable_t */

/*==============================================================================
   Constants
==============================================================================*/
/** ID of the parameter used to notify a module that the timer expired.

    @msgpayload{fwk_extn_param_id_soft_timer_expired_t}
    @table{weak__fwk__extn__param__id__soft__timer__expired__t}
 */
#define FWK_EXTN_PARAM_ID_SOFT_TIMER_EXPIRED 0x0A00100B

/* Structure defined for above Property  */
typedef struct fwk_extn_param_id_soft_timer_expired_t fwk_extn_param_id_soft_timer_expired_t;

/** @weakgroup weak_fwk_extn_param_id_soft_timer_expired_t
@{ */
struct fwk_extn_param_id_soft_timer_expired_t
{
   uint32_t timer_id; /**< Identifies the specific timer that sends this
                           parameter to the module. */
};
/** @} */ /* end_weakgroup weak_fwk_extn_param_id_soft_timer_expired_t */

/** @} */ /* end_addtogroup capi_fw_ext_soft_timer */

#endif /* _CAPI_FWK_EXTNS_SOFT_TIMER_H_ */
