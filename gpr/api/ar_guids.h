#ifndef AR_GUIDS_H_
#define AR_GUIDS_H_
/**
 * \file gpr_list.h
 * \brief
 *  	This file contains GUID definitions
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */


/**
 * GUIDs are 32 bit numbers.
 *
 */

/**
 * An empty macro to mark non-GUIDs so that GUID script doesn't mistake the ID for a GUID
 *
 */
#define AR_NON_GUID(x)                    x

/**
 * GUID Owners
 */
/**
 * GUID owner is Qualcomm.
 */
#define AR_GUID_OWNER_QC                  0x0
/**
 * GUID owner is not-Qualcomm.
 * All ISVs, OEMs & Customers must use this range.
 */
#define AR_GUID_OWNER_NON_QC              0x1
   /** Rest are reserved */

/**
 * Mask for Owner
 */
#define AR_GUID_OWNER_MASK                AR_NON_GUID(0xF0000000)
/**
 * Shift for Owner
 */
#define AR_GUID_OWNER_SHIFT               28

/**
 * GUID types
 */
/**
 * Reserved GUID type. Used for legacy IDs.
 */
#define AR_GUID_TYPE_RESERVED             0x0

/**
 * GUID type for Control commands.
 */
#define AR_GUID_TYPE_CONTROL_CMD          0x1
/**
 * GUID type for command responses of control commands.
 */
#define AR_GUID_TYPE_CONTROL_CMD_RSP      0x2
/**
 * GUID type for control events.
 */
#define AR_GUID_TYPE_CONTROL_EVENT        0x3
/**
 * GUID type for data commands
 */
#define AR_GUID_TYPE_DATA_CMD             0x4
/**
 * GUID type for data command responses.
 */
#define AR_GUID_TYPE_DATA_CMD_RSP         0x5
/**
 * GUID type for data events
 */
#define AR_GUID_TYPE_DATA_EVENT           0x6
/**
 * GUID type for module-id
 */
#define AR_GUID_TYPE_MODULE_ID            0x7
/**
 * GUID type for module's param & event-id
 */
#define AR_GUID_TYPE_PARAM_EVENT_ID       0x8
/**
 * GUID type for media format-id.
 */
#define AR_GUID_TYPE_FORMAT_ID            0x9
/**
 * GUID type is CAPI.
 * All CAPI framework extensions, interface extensions
 * their events, params use this.
 *
 * All module params & events must use AR_GUID_TYPE_PARAM_EVENT_ID
 */
#define AR_GUID_TYPE_CAPI                 0xA

/** Rest are reserved */

/**
 * Mask for GUID type
 */
#define AR_GUID_TYPE_MASK                 AR_NON_GUID(0x0F000000)
/**
 * Shift for GUID type
 */
#define AR_GUID_TYPE_SHIFT                24

/**
 * Zero is an invalid ID.
 * To be used like NULL value for pointers.
 */
#define AR_GUID_INVALID                   0



#endif /* AR_GUIDS_H_ */

