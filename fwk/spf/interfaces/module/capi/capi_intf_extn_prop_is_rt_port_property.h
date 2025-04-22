#ifndef CAPI_INTF_EXTN_PROP_IS_RT_PORT_PROP_H
#define CAPI_INTF_EXTN_PROP_IS_RT_PORT_PROP_H

/**
 * @file  capi_intf_extn_prop_is_rt_port_property.h
 * @brief intf_extns related to propagation of port properties real time (RT) or non real time (NRT).
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

/** @weakgroup weakf_capi_chapter_portpropprog_portprops_rt
The #INTF_EXTN_PROP_IS_RT_PORT_PROPERTY interface extension allows propagation
of port properties across modules in real time or non-real time. An event from
a module indicates that the upstream port is in either real time or non-real
time.

When a module implements this interface extension, the framework does not
automatically propagate the port property, even for SISO modules.

@heading3{For Input Ports}

A capi_vtbl_t::set_param() call indicates that the upstream port is in either
real time or non-real time.

An event from a module indicates that the downstream port is in either real
time or non-real time.

The following figure shows upstream (US) and downstream (DS) real-time
(RT)/non-real-time (NRT) values. Practical graphs can have branches, which
means propagation might not be straightforward.

@inputfigcap{1,port_property_propagation,Upstream and downstream values in real time or non-real
time,upstreamDownstreamRtNrt}

@heading3{For Output Ports}

A capi_vtbl_t::set_param() call indicates that the downstream port is in either
real time or non-real time. An event from the module indicates that the
upstream port is either real time or non-real time.

@heading3{Usage Examples}

- Modules such as multi-port modules might need to propagate this flag because
  the container is not aware of routing from input to output. \n @vertspace{3}
  Also, the container is not aware of the trigger policy of the module
  (see Section&nbsp;@xref{sec:interactionPropTrigger}).
- A module that changes from real time to non-real time (such as a buffering
  module or a timer-triggered module) must also implement this flag. \n
  @vertspace{3}
  For example, introducing a buffering module in an otherwise real-time path
  changes the real-time flag to FALSE. Introducing a timer-driven module in a
  non-real-time path changes the flag to TRUE.

@heading3{Framework Default Settings}

- Initially, all ports are non-real time.
- If a started input port of a module is marked as real time upstream (through
  propagation), all the output ports should be marked as real time upstream.
  Otherwise, they are marked as non-real time.
- If a started output port of a module is marked as real time downstream
  (through propagation), all the input ports should be marked as real time
  downstream. Otherwise, they are marked as non-real time.
*/

/** @addtogroup capi_if_ext_port_props
@{ */

/** Unique identifier of the interface extension used to propagate port properties across
    modules in real time or non-real time.

    This extension supports the following parameter and event IDs:
    - #INTF_EXTN_PARAM_ID_IS_RT_PORT_PROPERTY
    - #INTF_EXTN_EVENT_ID_IS_RT_PORT_PROPERTY
*/
#define INTF_EXTN_PROP_IS_RT_PORT_PROPERTY 0x0A001048

/** ID of the parameter a container uses to tell a module about the port
    property of a specified port.

    Upon receiving the property, the module can either propagate the state
    to the connected input/output ports or ignore the parameter.

    @msgpayload{intf_extn_param_id_is_rt_port_property_t}
    @table{weak__intf__extn__param__id__is__rt__port__property__t} @newpage
 */
#define INTF_EXTN_PARAM_ID_IS_RT_PORT_PROPERTY 0x0A001049

/** ID of the event raised by a module via #CAPI_EVENT_DATA_TO_DSP_SERVICE in
    response to #INTF_EXTN_PARAM_ID_IS_RT_PORT_PROPERTY.

    @msgpayload{intf_extn_param_id_is_rt_port_property_t}
    @table{weak__intf__extn__param__id__is__rt__port__property__t}
 */
#define INTF_EXTN_EVENT_ID_IS_RT_PORT_PROPERTY 0x0A00104A

typedef struct intf_extn_param_id_is_rt_port_property_t intf_extn_param_id_is_rt_port_property_t;
typedef struct intf_extn_param_id_is_rt_port_property_t intf_extn_event_id_is_rt_port_property_t;

#include "spf_begin_pragma.h"
/** @weakgroup weak_intf_extn_param_id_is_rt_port_property_t
@{ */
struct intf_extn_param_id_is_rt_port_property_t
{
   bool_t is_rt;
   /**< Indicates whether the propagated property is real time.

        @valuesbul
        - 0 -- FALSE (non-real time)
        - 1 -- TRUE (real time) @tablebulletend */

   bool_t is_input;
   /**< Indicates whether the port is an input port.

        @valuesbul
        - 0 -- FALSE (output port)
        - 1 -- TRUE (input port) @tablebulletend */

   uint32_t port_index;
   /**< Input or output port index, depending on the value of is_input. */
}
#include "spf_end_pragma.h"
;
/** @} */ /* end_weakgroup weak_intf_extn_param_id_is_rt_port_property_t */

/** @} */ /* end_addtogroup capi_if_ext_port_props */

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* CAPI_INTF_EXTN_PROP_IS_RT_PORT_PROP_H*/
