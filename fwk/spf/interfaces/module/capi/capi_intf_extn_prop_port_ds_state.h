#ifndef CAPI_INTF_EXTN_PROP_PORT_DS_STATE_H
#define CAPI_INTF_EXTN_PROP_PORT_DS_STATE_H

/**
 *   \file capi_intf_extn_prop_port_ds_state.h
 *   \brief
 *        intf_extns related to propagation of port's downstream state such as DATA_PORT_STATE_STARTED and
 *  DATA_PORT_STATE_STOPPED.
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
#include "capi_intf_extn_data_port_operation.h"

/** @weakgroup weakf_capi_chapter_portpropprog_port_dsstate
The #INTF_EXTN_PROP_PORT_DS_STATE interface extension is used to propagate the
downstream state of a port across modules. The downstream state is different
from the port's own state. The framework first propagates the downstream state
and then applies the downgraded state on the port.

State propagation is only from downstream to upstream. A container sets the
state on the output port. A module can then propagate this state to the
connected input ports (connected from that output port only for which a set
parameter was done). When an event is raised from a module, it is raised on the
input port, and it can be raised only in the #INTF_EXTN_PARAM_ID_PORT_DS_STATE
context.

A port's downstream state can only be Prepare, Start, Suspend, or Stop. This
state is different from the port state itself. For example, you can propagate a
Stop state and the port itself might be stopped.

@heading3{Multi-port Modules}

All multi-port modules must implement the downstream state because a container
does not know the routing inside the module (unless the framework default works
for the module). Unlike the real-time flag, which depends on trigger policy
grouping or ports being marked as non-triggerable, the port state depends only
on the connection inside the module.

For example, consider a splitter that outputs data on two ports. If one of the
output paths is stopped somewhere, ideally, the other path should not be
affected. In this case, the stopped downstream state is propagated backwards,
which indicates to the splitter that it no longer needs to wait for buffers to
become available on the corresponding output port.

For modules that implement the #CAPI_MIN_PORT_NUM_INFO property and set
minimum_output_port to zero, refer to the #CAPI_MIN_PORT_NUM_INFO property
documentation.

@heading3{Framework Default Settings}

The framework default assumes that all the inputs are connected to all the
outputs.
- If all the output ports of a module are in the Stop state, propagate this
  state backwards on all the input ports.
- If an output port of a module is in the Start state, propagate this state on
  all the input ports.
- If an output port of a module is in the Prepare state and none of the output
  ports is in the Start state, propagate the Prepare state to all input ports.

The downstream state is handled through this #INTF_EXTN_PROP_PORT_DS_STATE
extension, but the modules are notified of the upstream state through an
internal EOS, which indicates that data flow is stopped. Availability of the
data indicates that data flow started. Data flow state propagation is discussed
in Section&nbsp;@xref{sec:dataFlowStates}.
*/

/** @addtogroup capi_if_ext_port_ds_states
@{ */

/** Unique identifier of the interface extension used to propagate the
    downstream state of a port.

    This extension supports the following parameter and event IDs:
    - #INTF_EXTN_PARAM_ID_PORT_DS_STATE
    - #INTF_EXTN_EVENT_ID_PORT_DS_STATE
    - #INTF_EXTN_EVENT_ID_BLOCK_PORT_DS_STATE_PROP
*/
#define INTF_EXTN_PROP_PORT_DS_STATE 0x0A001040

/** ID of the parameter that the container uses to send the downstream state of
    an output port to a module. Upon receiving the state, the module can either
    propagate the state to connected input ports or ignore the parameter.

    When a module implements this interface extension, the framework does not
    automatically propagate the state, even for SISO modules.

    If the module chooses to ignore the downstream state, it must ignore all
    Prepare, Start, and Stop states.

    @msgpayload{intf_extn_param_id_port_ds_state_t}
    @table{weak__intf__extn__param__id__port__ds__state__t} @newpage
 */
#define INTF_EXTN_PARAM_ID_PORT_DS_STATE 0x0A001041

/** ID of the event raised by module via #CAPI_EVENT_DATA_TO_DSP_SERVICE in
    response to #INTF_EXTN_PARAM_ID_PORT_DS_STATE.

    @msgpayload{intf_extn_event_id_port_ds_state_t}
    @table{weak__intf__extn__event__id__port__ds__state__t}
 */
#define INTF_EXTN_EVENT_ID_PORT_DS_STATE 0x0A001042

/** Valid values for propagated data port states.
 */
typedef enum intf_extn_prop_data_port_state_t {
   INTF_EXTN_PROP_DATA_PORT_STATE_STOPPED = 1,
   /**< Propagated state of the port is stopped. */

   INTF_EXTN_PROP_DATA_PORT_STATE_PREPARED = 2,
   /**< Propagated state of the port is prepared. */

   INTF_EXTN_PROP_DATA_PORT_STATE_STARTED = 3,
   /**< Propagated state of the port is started. */

   INTF_EXTN_PROP_DATA_PORT_STATE_SUSPENDED = 4,
   /**< Propagated state of data port is suspended. */

   INTF_EXTN_PROP_DATA_PORT_STATE_INVALID = 0xFFFFFFFF
   /**< Invalid propagated state. */
} /** @cond */ intf_extn_prop_data_port_state_t /** @endcond */;

/* Payload structure for the INTF_EXTN_PARAM_ID_PORT_DS_STATE. */
typedef struct intf_extn_param_id_port_ds_state_t intf_extn_param_id_port_ds_state_t;

#include "spf_begin_pragma.h"
/** @weakgroup weak_intf_extn_param_id_port_ds_state_t
@{ */
struct intf_extn_param_id_port_ds_state_t
{
   uint32_t output_port_index;
   /**< Index of the output port that receives the downstream state. */

   intf_extn_prop_data_port_state_t port_state;
   /**< Downstream state of the port being propagated.

        @valuesbul
        - #INTF_EXTN_PROP_DATA_PORT_STATE_STOPPED
        - #INTF_EXTN_PROP_DATA_PORT_STATE_PREPARED
        - #INTF_EXTN_PROP_DATA_PORT_STATE_STARTED @tablebulletend */
}
#include "spf_end_pragma.h"
;
/** @} */ /* end_weakgroup weak_intf_extn_param_id_port_ds_state_t */

/* Payload structure for the INTF_EXTN_EVENT_ID_PORT_DS_STATE. */
typedef struct intf_extn_event_id_port_ds_state_t intf_extn_event_id_port_ds_state_t;

#include "spf_begin_pragma.h"
/** @weakgroup weak_intf_extn_event_id_port_ds_state_t
@{ */
struct intf_extn_event_id_port_ds_state_t
{
   uint32_t input_port_index;
   /**< Index of the input port that is providing the downstream state. */

   intf_extn_prop_data_port_state_t port_state;
   /**< Downstream state of the port being propagated.

        @valuesbul
        - #INTF_EXTN_PROP_DATA_PORT_STATE_STOPPED
        - #INTF_EXTN_PROP_DATA_PORT_STATE_PREPARED
        - #INTF_EXTN_PROP_DATA_PORT_STATE_STARTED @tablebulletend */
}
#include "spf_end_pragma.h"
;
/** @} */ /* end_weakgroup weak_intf_extn_event_id_port_ds_state_t */

/** Raised on an output port to block the downstream state propagation from
    that port.

    A module raises this event when the state of its output port is not to
    change because of the downstream state. This event is raised only when the
    output port is open, so the module also must implement
    #INTF_EXTN_DATA_PORT_OPERATION.

    @msgpayload{intf_extn_event_id_block_port_ds_state_prop_t}
    @table{weak__intf__extn__event__id__block__port__ds__state__prop__t}
 */
#define INTF_EXTN_EVENT_ID_BLOCK_PORT_DS_STATE_PROP 0x0A001046

/* Payload structure for the INTF_EXTN_EVENT_ID_BLOCK_PORT_DS_STATE_PROP. */
typedef struct intf_extn_event_id_block_port_ds_state_prop_t intf_extn_event_id_block_port_ds_state_prop_t;

#include "spf_begin_pragma.h"
/** @weakgroup weak_intf_extn_event_id_block_port_ds_state_prop_t
@{ */
struct intf_extn_event_id_block_port_ds_state_prop_t
{
   uint32_t output_port_index;
   /**< Index of the output port that is not to change per the downstream
        state propagation. */
}
#include "spf_end_pragma.h"
;
/** @} */ /* end_weakgroup weak_intf_extn_event_id_block_port_ds_state_prop_t */

/** @} */ /* end_addtogroup capi_if_ext_port_ds_states */

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* CAPI_INTF_EXTN_PROP_PORT_DS_STATE_H*/
