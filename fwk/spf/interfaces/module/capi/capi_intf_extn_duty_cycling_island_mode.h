#ifndef CAPI_INTF_EXTN_DUTY_CYCLING_ISLAND_MODE_H
#define CAPI_INTF_EXTN_DUTY_CYCLING_ISLAND_MODE_H

/**
 * @file  capi_intf_extn_duty_cycling_island_mode.h
 * @brief Interface extensions related to Duty Cycling LPI use cases (currently used for BT A2DP LPI) and
 * propagation of gapless module fill status for island entry to DCM.
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

/** @addtogroup capi_intf_extn_duty_cycling_island_mode
@{ */

#define INTF_EXTN_DUTY_CYCLING_ISLAND_MODE 0x0A001062
/** ID of the parameter a container uses to tell a module about the duty cycling mode of the container.

    Upon receiving the property, the gapless module raises a buffer fullness event.

    @msgpayload{intf_extn_param_id_cntr_duty_cycling_enabled_t}
    @table{weak__intf__extn__param__id__cntr__duty__cycling__enabled__t} @newpage
 */

#define INTF_EXTN_PARAM_ID_CNTR_DUTY_CYCLING_ENABLED 0x0A001063

/** ID of the event raised by a module via #CAPI_EVENT_DATA_TO_DSP_SERVICE in
    response to #INTF_EXTN_PARAM_ID_CNTR_DUTY_CYCLING_ENABLED.

    @msgpayload{intf_extn_param_id_cntr_duty_cycling_enabled_t}
    @table{weak__intf__extn__param__id__cntr__duty__cycling__enabled__t}
 */

typedef struct intf_extn_param_id_cntr_duty_cycling_enabled_t intf_extn_param_id_cntr_duty_cycling_enabled_t;

#include "spf_begin_pragma.h"
/** @weakgroup weak_intf_extn_param_id_cntr_duty_cycling_enabled_t
@{ */
struct intf_extn_param_id_cntr_duty_cycling_enabled_t
{
   bool_t is_cntr_duty_cycling;
   /**< Indicates whether the container is duty cycling enabled.

        @valuesbul
        - 0 -- FALSE
        - 1 -- TRUE  @tablebulletend */
}
#include "spf_end_pragma.h"
;
/** @} */ /* end_weakgroup weak_intf_extn_param_id_cntr_duty_cycling_enabled_t */

/** ID of the event raised by a module via #CAPI_EVENT_DATA_TO_DSP_SERVICE in
    response to #INTF_EXTN_EVENT_ID_ALLOW_DUTY_CYCLING.

    @msgpayload{intf_extn_event_id_allow_duty_cycling_t}
    @table{weak__intf__extn__event__id__allow__duty__cycling__t} @newpage
*/

#define INTF_EXTN_EVENT_ID_ALLOW_DUTY_CYCLING 0x0A001064

typedef struct intf_extn_event_id_allow_duty_cycling_t intf_extn_event_id_allow_duty_cycling_t;

#include "spf_begin_pragma.h"
/** @weakgroup weak_intf_extn_event_id_allow_duty_cycling_t
@{ */
struct intf_extn_event_id_allow_duty_cycling_t
{
   bool_t is_buffer_full_req_dcm_to_unblock_island_entry;
   /**< Indicates whether the module buffer is full, e.g., Gapless Delay Buffer.

        @valuesbul
        - 0 -- FALSE
        - 1 -- TRUE  @tablebulletend */
}
#include "spf_end_pragma.h"
;
/** @} */ /* end_weakgroup weak_intf_extn_event_id_allow_duty_cycling_t */

/** ID of the event raised by a module via #CAPI_EVENT_DATA_TO_DSP_SERVICE in
    response to #INTF_EXTN_EVENT_ID_ALLOW_DUTY_CYCLING_V2.

    @msgpayload{intf_extn_event_id_allow_duty_cycling_v2_t}
    @table{intf_extn_event_id_allow_duty_cycling_v2_t} @newpage
*/

#define INTF_EXTN_EVENT_ID_ALLOW_DUTY_CYCLING_V2 0x0A001069

typedef struct intf_extn_event_id_allow_duty_cycling_v2_t intf_extn_event_id_allow_duty_cycling_v2_t;

#include "spf_begin_pragma.h"
/** @weakgroup intf_extn_event_id_allow_duty_cycling_v2_t
@{ */
struct intf_extn_event_id_allow_duty_cycling_v2_t
{
   bool_t allow_duty_cycling;
   /**< Indicates whether the module allows duty cycling or not (e.g. Gapless module allows duty cycling when buffer is full)

        @valuesbul
        - 0 -- FALSE
        - 1 -- TRUE  @tablebulletend */
}
#include "spf_end_pragma.h"
;
/** @} */ /* end_weakgroup weak_intf_extn_event_id_allow_duty_cycling_v2_t */

/** @} */ /* end_addtogroup capi_intf_extn_duty_cycling_island_mode */

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* CAPI_INTF_EXTN_DUTY_CYCLING_ISLAND_MODE_H*/
