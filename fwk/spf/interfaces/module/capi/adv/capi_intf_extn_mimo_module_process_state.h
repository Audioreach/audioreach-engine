#ifndef _CAPI_INTF_EXTN_MIMO_MODULE_PROCESS_STATE_H_
#define _CAPI_INTF_EXTN_MIMO_MODULE_PROCESS_STATE_H_

/**
 *   \file capi_intf_extn_mimo_module_process_state.h
 *   \brief
 *        This file contains CAPI Interface Extension Definitions for disabling MIMO modules.
 *
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "capi_types.h"

/* Following discussion is directly pulled into the PDF */
/** @addtogroup capi_intf_extn_mimo_module_process_state

MIMO modules that can sometimes work in SISO mode and don't need any processing from input to output can be disabled
using this extension.

If MIMO modules use the generic event (#CAPI_EVENT_PROCESS_STATE) to disable then data flow is blocked at their input
which
may not be desirable, so this extension can be used to inform the framework that they are disabled. The framework tries
to
honor the modules' request by evaluating the following conditions.
- Module must be in SISO mode. Only one active input and one active output port.
- Module must have zero algorithm/buffer delay. It should not be maintaining any delay buffer while in the Disabled
state.
- Module must have valid and same media format on input and output ports.

The framework can enable the module at any time without informing the module. Usually this occurs when:
- A new port opens and module is not operating in SISO mode anymore
- Framework detects non-zero algorithm delay for the module
- Framework detects a different media format on input and output ports

Modules can also be enabled temporarily to propagate certain metadata. This is why modules must not have any algorithm
delay
when disabled, because its processes can be called discontinuously and if there is any delay buffer then it
can get discontinuous data.

*/

/** @addtogroup capi_intf_extn_mimo_module_process_state
@{ */

/*==============================================================================
   Constants
==============================================================================*/

/** Unique identifier of the interface extension that the MIMO module uses to
   enable/disable itself.

   Only MIMO modules which can't use #CAPI_EVENT_PROCESS_STATE to update their enable/disable state should use this
   extension.

    This extension supports the following events:
    - #INTF_EXTN_EVENT_ID_MIMO_MODULE_PROCESS_STATE @newpage
*/
#define INTF_EXTN_MIMO_MODULE_PROCESS_STATE 0x0A00101C

/** ID of the custom event raised by the module to enable/disable itself.

    When the module disables itself the framework evaluates whether the module can be removed from the processing chain
   or not.
   - If it can be removed the the module's process will not be called .
   - If it can't be removed the module is considered enabled.

    Even if the module is considered disabled, the framework can still call the module's process, usually to propagate
   certain metadata.

    @msgpayload{intf_extn_event_id_mimo_module_process_state_t}
    @table{weak__intf__extn__event__id__mimo__module__process__state__t}
 */
#define INTF_EXTN_EVENT_ID_MIMO_MODULE_PROCESS_STATE 0x0A001059

/*==============================================================================
   Type definitions
==============================================================================*/

/* Structure defined for above Property  */
typedef struct intf_extn_event_id_mimo_module_process_state_t intf_extn_event_id_mimo_module_process_state_t;

/** @weakgroup weak_intf_extn_event_id_mimo_module_process_state_t
@{ */
struct intf_extn_event_id_mimo_module_process_state_t
{
   bool_t is_disabled;
   /**< Indicates whether the module is disabled or enabled.

        @valuesbul
        - 0 -- Enabled
        - 1 -- Disabled @tablebulletend */
};
/** @} */ /* end_weakgroup weak_intf_extn_event_id_mimo_module_process_state_t */

#endif /* _CAPI_INTF_EXTN_MIMO_MODULE_PROCESS_STATE_H_ */
