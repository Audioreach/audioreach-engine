#ifndef _CAPI_FWK_EXTNS_VOICE_TIMING_H_
#define _CAPI_FWK_EXTNS_VOICE_TIMING_H_

/**
 *   \file capi_fwk_extns_voice_delivery.h
 *   \brief
 *        This file contains CAPI Smart Sync Module Framework Extension Definitions
 *
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "capi_types.h"

/** @addtogroup capi_fw_ext_voice_del
The voice delivery framework extension (#FWK_EXTN_VOICE_DELIVERY) allows a
module to control the timing of its host container's topology invocations.

This extension is required for voice use cases when data processing must begin
at a precise time each VFR cycle; but, within that VFR cycle, multiple frames
might need to be processed as soon as possible. To achieve this, the container
must switch between timer-triggered and buffer-triggered topology invocation
policies. This customized behavior is placed within a module that implements
the FWK_EXTN_VOICE_DELIVERY extension.

A module that implements the FWK_EXTN_VOICE_DELIVERY extension is responsible
for the following:
- Subscribing to the voice timer to receive periodic triggers when receiving
  the #FWK_EXTN_PROPERTY_ID_VOICE_PROC_START_TRIGGER property.
- Raising the #FWK_EXTN_VOICE_DELIVERY_EVENT_ID_CHANGE_CONTAINER_TRIGGER_POLICY
  to tell the host container when to switch between timer-triggered and
  buffer-triggered policies.
- Receiving #FWK_EXTN_VOICE_DELIVERY_PARAM_ID_FIRST_PROC_TICK_NOTIF when the
  first timer trigger expires. The module can use this information as a signal
  to begin outputting data.

@note1hang This framework extension is specific to the Smart Synchronization
           module behavior, and thus it is not expected to be required for any
           other module.
*/

/** @addtogroup capi_fw_ext_voice_del
@{ */

/*==============================================================================
   Constants
==============================================================================*/

/** Unique identifier of the framework extension used for the Voice Delivery module.

    This extension supports the following property, event, and parameter IDs:
    - #FWK_EXTN_PROPERTY_ID_VOICE_PROC_START_TRIGGER
    - #FWK_EXTN_VOICE_DELIVERY_EVENT_ID_CHANGE_CONTAINER_TRIGGER_POLICY
    - #FWK_EXTN_VOICE_DELIVERY_PARAM_ID_FIRST_PROC_TICK_NOTIF @newpage
 */
#define FWK_EXTN_VOICE_DELIVERY 0x0A00103D

/** ID of the custom property used to set the trigger for voice processing to
    start.

    The framework sends a signal to the Voice Delivery module to register the
    voice timer for any VFR tick.

    @msgpayload{capi_prop_voice_proc_start_trigger_t}
    @table{weak__capi__prop__voice__proc__start__trigger__t}
*/
#define FWK_EXTN_PROPERTY_ID_VOICE_PROC_START_TRIGGER 0x0A00103E

/*==============================================================================
   Type definitions
==============================================================================*/

/* Structure defined for above Property  */
typedef struct capi_prop_voice_proc_start_trigger_t capi_prop_voice_proc_start_trigger_t;

/** @weakgroup weak_capi_prop_voice_proc_start_trigger_t
@{ */
struct capi_prop_voice_proc_start_trigger_t
{
   void *proc_start_signal_ptr;
   /**< Pointer to a posal_signal_t that is owned by the framework.

        This signal should be set when the timer trigger expires. The framework
        timer trigger handling will occur when this signal is set.

        The pointer is set via the
        #FWK_EXTN_PROPERTY_ID_VOICE_PROC_START_TRIGGER property. */

   void *resync_signal_ptr;
   /**< Resync signal to the framework posal_signal_t. */
};
/** @} */ /* end_weakgroup weak_capi_prop_voice_proc_start_trigger_t */

/*-----------------------------------------------------------------------------
 * Type definitions
 *---------------------------------------------------------------------------*/

/** Defines the trigger policy types.
 */
typedef enum container_trigger_policy_t {
   VOICE_TIMER_TRIGGER = 0,   /**< Container starts a topology process with
                                   the voice timer trigger. */
   OUTPUT_BUFFER_TRIGGER = 1, /**< Container starts a topology process with
                                   the output buffer trigger. */
   INVALID_TRIGGER = 2        /**< Invalid value. */
} /** @cond */ container_trigger_policy_t /** @endcond */;

/*==============================================================================
   Constants
==============================================================================*/

/** ID of the event the Voice Delivery module raises to set the container
    trigger policy.

    The container determines when to start processing based on the trigger
    policy.
    - The module raises this event with the VOICE_TIMER_TRIGGER policy to
      request that the host container triggers processing based on timer
      expiration.
    - The module raises this event with the OUTPUT_BUFFER_TRIGGER policy to
      request that the host container triggers processing based on the arrival
      of an output buffer.

      @msgpayload{capi_event_change_container_trigger_policy_t}
      @table{weak__capi__event__change__container__trigger__policy__t} @newpage
 */
#define FWK_EXTN_VOICE_DELIVERY_EVENT_ID_CHANGE_CONTAINER_TRIGGER_POLICY 0x0A00103F

/*==============================================================================
   Type definitions
==============================================================================*/

/* Structure defined for above event  */
typedef struct capi_event_change_container_trigger_policy_t capi_event_change_container_trigger_policy_t;

/** @weakgroup weak_capi_event_change_container_trigger_policy_t
@{ */
struct capi_event_change_container_trigger_policy_t
{
   container_trigger_policy_t container_trigger_policy;
   /**< Trigger policy the container uses to determine when to invoke the
        topology. */
};
/** @} */ /* end_weakgroup weak_capi_event_change_container_trigger_policy_t */

/*==============================================================================
   Constants
==============================================================================*/

/** ID of the custom parameter used to notify the Voice Delivery module that
    the container received the first processing tick.

    This parameter is set to the module when the first processing tick is
    received after any of the following occur:
    - The Voice Delivery module's subgraph switches from the Stop state to the
      Start state.
    - The Voice Delivery module's subgraph switches from the Suspend state to
      the Start state.
    - A resynchronization occurs.
 */
#define FWK_EXTN_VOICE_DELIVERY_PARAM_ID_FIRST_PROC_TICK_NOTIF 0x0A00104F
/* This parameter has no payload. */

/** Custom parameter ID to inform the voice delivery module of the container's reception
   of the VFR resync. This parameter has no payload. */
#define FWK_EXTN_VOICE_DELIVERY_PARAM_ID_RESYNC_NOTIF 0x0A001055

/** Custom parameter ID to inform the voice delivery module that there was upstream
   data drop which occurred while the voice delivery module is in the Syncing state.
 */
#define FWK_EXTN_VOICE_DELIVERY_PARAM_ID_DATA_DROP_DURING_SYNC 0x0A001007

/** Custom parameter ID to inform the voice delivery module that topo-process is being
 * invoked from the container. The module is only supposed to generate one cntr-frame-len
 * worth of data per topo-process.
 */
#define FWK_EXTN_VOICE_DELIVERY_PARAM_ID_TOPO_PROCESS_NOTIF 0x0A001017

/*==============================================================================
   Constants
==============================================================================*/

/** ID of the event the Voice Delivery module raises to inform VCPM if its
    inputs are synced or not.

   @msgpayload{fwk_extn_voice_delivery_event_update_sync_state_t}
   @table{weak__fwk__extn__voice__delivery__event__update__sync__state__t} @newpage
 */
#define FWK_EXTN_VOICE_DELIVERY_EVENT_ID_UPDATE_SYNC_STATE 0x0800137E

/*==============================================================================
   Type definitions
==============================================================================*/

/* Structure defined for above event  */
typedef struct fwk_extn_voice_delivery_event_update_sync_state_t fwk_extn_voice_delivery_event_update_sync_state_t;

/** @weakgroup weak_fwk_extn_voice_delivery_event_update_sync_state_t
@{ */
struct fwk_extn_voice_delivery_event_update_sync_state_t
{
   uint32_t is_synced;
   /**< Indicates if the inputs to the voice delivery module are synced or out of sync. */
};
/** @} */ /* end_weakgroup fwk_extn_voice_delivery_event_update_sync_state_t */

/** @} */ /* end_addtogroup capi_fw_ext_voice_del */

#endif /* _CAPI_FWK_EXTNS_VOICE_TIMING_H_ */
