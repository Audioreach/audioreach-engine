#ifndef _CAPI_FWK_EXTNS_SYNC_H_
#define _CAPI_FWK_EXTNS_SYNC_H_

/**
 *   \file capi_fwk_extns_sync.h
 *   \brief
 *        This file contains CAPI Sync Module Framework Extension Definitions
 *
 *    Framework extension for the  sync module which is used to synchronize primary
 *    and secondary paths.
 *
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "capi_types.h"

/* Following discussion is directly pulled into the PDF */
/** @addtogroup capi_fw_ext_sync
The Synchronization module framework extension is required for the following
purposes:
- During synchronization, the module must disable framework-layer threshold
  buffering. @vertspace{2}
   - The module has a synchronization error that is dependent on the amount of
     time between subsequent buffer arrivals. Therefore, it is imperative that
     the module receives input data immediately when that data arrives at its
     host container.
   - Once the ports are synchronized, the module no longer has this
     requirement and it can re-enable threshold buffering.
   - The module uses #FWK_EXTN_SYNC_EVENT_ID_ENABLE_THRESHOLD_BUFFERING for
     enabling and disabling the buffering.
- The framework must inform the module of the threshold by using the
     FWK_EXTN_PARAM_ID_CONTAINER_FRAME_DURATION set parameter in the
     #FWK_EXTN_CONTAINER_FRAME_DURATION extension.
- When an external port is started, the Synchronization module receives a
  #FWK_EXTN_SYNC_PARAM_ID_PORT_WILL_START set parameter that indicates when
  that port will start. @vertspace{2}
   - This set parameter is necessary because the mere presence of input data
     implies data flow is starting. However, input data is not sent to the
     Synchronization module until the entire threshold is buffered on all
     ports.
   - The Synchronization module must receive this set parameter so it can tell
     the container to disable the threshold. Then when the first data buffer is
     received, the framework can immediately send that buffer to the
     Synchronization module.
   - When the Synchronization module receives this set parameter, it is not
     required to know which port was started. This command is only handled when
     any of the module's ports are stopped; otherwise, the parameter is ignored.
   - This framework extension includes an FWK_EXTN_SYNC_EVENT_ID_DATA_PORT_ACTIVITY_STATE event.
     This event, if raised by the module, indicates that a particular port is active/inactive.
     The framework can use this information to optimally copy data between only the active
     ports and invoke capi_vtbl_t::process() on the module.
       - This event should only be raised on an output port of the Sync module if and
         only if the input port associated with it is not opened (closed).
       - By default, all ports are deemed active by the framework.
    - If a module has already disabled threshold buffering and a new input port
      starts or an output media format is propagated then the module must raise a duplicate
      threshold buffering disabled event.

@note1hang The Synchronization framework extension is specific to
           Synchronization module behavior. Thus, it is not expected to be
           required for any custom or non-synchronization modules.
*/

/** @addtogroup capi_fw_ext_sync
@{ */

/*==============================================================================
   Constants
==============================================================================*/

/** Unique identifier of the framework extension for the Synchronization module, which
    is used to synchronize data at its inputs.

    This extension supports the following event and parameter IDs:
    - #FWK_EXTN_SYNC_EVENT_ID_ENABLE_THRESHOLD_BUFFERING
    - #FWK_EXTN_SYNC_PARAM_ID_PORT_WILL_START
    - #FWK_EXTN_SYNC_EVENT_ID_DATA_PORT_ACTIVITY_STATE @newpage
*/
#define FWK_EXTN_SYNC 0x0A00101A

/** ID of the custom event raised when the Synchronization module enables or
    disables threshold buffering.

    With threshold buffering disabled, the framework invokes the topology
    whenever input data is received, regardless of whether the threshold amount
    of input data is met.

    @msgpayload{fwk_extn_sync_event_id_enable_threshold_buffering_t}
    @table{weak__fwk__extn__sync__event__id__enable__threshold__buffering__t}
 */
#define FWK_EXTN_SYNC_EVENT_ID_ENABLE_THRESHOLD_BUFFERING 0x0A00101B

/*==============================================================================
   Type definitions
==============================================================================*/

/* Structure defined for above Property  */
typedef struct fwk_extn_sync_event_id_enable_threshold_buffering_t fwk_extn_sync_event_id_enable_threshold_buffering_t;

/** @weakgroup weak_fwk_extn_sync_event_id_enable_threshold_buffering_t
@{ */
struct fwk_extn_sync_event_id_enable_threshold_buffering_t
{
   bool_t enable_threshold_buffering;
   /**< Indicates whether threshold buffering is to be enabled.

        @valuesbul
        - 0 -- Disabled
        - 1 -- Enabled @tablebulletend */
};
/** @} */ /* end_weakgroup weak_fwk_extn_sync_event_id_enable_threshold_buffering_t */

/*==============================================================================
   Constants
==============================================================================*/

/** ID of the custom parameter a container sends when an external input port
    connected to the Synchronization module moves to the Start state. The
    module then raises a disable threshold event to accept partial data to
    begin the synchronization process.
 */
#define FWK_EXTN_SYNC_PARAM_ID_PORT_WILL_START 0x0A00101D

/**
 @table{weak__fwk__extn__sync__event__id__data__port__activity__state__t}
 */
#define FWK_EXTN_SYNC_EVENT_ID_DATA_PORT_ACTIVITY_STATE 0x08001372

typedef struct fwk_extn_sync_event_id_data_port_activity_state_t fwk_extn_sync_event_id_data_port_activity_state_t;

#include "spf_begin_pragma.h"
/** @weakgroup weak_fwk_extn_sync_event_id_data_port_activity_state_t
@{ */
struct fwk_extn_sync_event_id_data_port_activity_state_t
{
   uint32_t is_inactive;
   /**< Indicates whether the port state is inactive.

        @valuesbul
        - 0 -- FALSE (active)
        - 1 -- TRUE (inactive) @tablebulletend */

   uint32_t out_port_index;
   /**< Output port index, depending on the value of is_input. */
}
#include "spf_end_pragma.h"
;
/** @} */ /* end_weakgroup fwk_extn_sync_event_id_data_port_activity_state_t */

/** @} */ /* end_addtogroup capi_fw_ext_sync */

#endif /* _CAPI_FWK_EXTNS_SYNC_H_ */
