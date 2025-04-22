#ifndef AR_GUIDS_H_
#define AR_GUIDS_H_
/**
  \file ar_guids.h
  \brief 
    GUID interpretation
  
  \copyright
     Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
     SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/*
  GUIDs are 32 bit numbers.
*/

/* Empty macro used to mark non-GUIDs so the GUID script does not mistake the
    ID for a GUID.
 */
#define AR_NON_GUID(x)                    x

/** @addtogroup spf_utils_guids
@{ */

/** GUID owner is Qualcomm.*/
#define AR_GUID_OWNER_QC                  0x0

/** GUID owner is not Qualcomm. All ISVs, OEMs, and customers
    must use this range.
 */
#define AR_GUID_OWNER_NON_QC              0x1
/* Rest are reserved */

/** Mask for the GUID owner. */
#define AR_GUID_OWNER_MASK                AR_NON_GUID(0xF0000000)

/** Bit shift for the GUID owner. */
#define AR_GUID_OWNER_SHIFT               28


/** Reserved GUID type; it is used for legacy IDs. */
#define AR_GUID_TYPE_RESERVED             0x0

/** GUID type for control commands. */
#define AR_GUID_TYPE_CONTROL_CMD          0x1

/** GUID type for command responses of control commands. */
#define AR_GUID_TYPE_CONTROL_CMD_RSP      0x2

/** GUID type for control events. */
#define AR_GUID_TYPE_CONTROL_EVENT        0x3

/** GUID type for data commands */
#define AR_GUID_TYPE_DATA_CMD             0x4

/** GUID type for data command responses. */
#define AR_GUID_TYPE_DATA_CMD_RSP         0x5

/** GUID type for data events. */
#define AR_GUID_TYPE_DATA_EVENT           0x6

/** GUID type for module IDs. */
#define AR_GUID_TYPE_MODULE_ID            0x7

/** GUID type for parameter and event IDs of a module. */
#define AR_GUID_TYPE_PARAM_EVENT_ID       0x8

/** GUID type for media format IDs. */
#define AR_GUID_TYPE_FORMAT_ID            0x9

/** GUID type for CAPI. All CAPI framework extensions, interface extensions,
    their events, and their parameters use this type.

    All module parameters and events must use #AR_GUID_TYPE_PARAM_EVENT_ID.
 */
#define AR_GUID_TYPE_CAPI                 0xA

/** GUID type for miscellaneous types, like container types. */
#define AR_GUID_TYPE_MISC                 0xB
/* Rest are reserved */

/** Mask for the GUID type. */
#define AR_GUID_TYPE_MASK                 AR_NON_GUID(0x0F000000)

/** Bit shift for the GUID type. */
#define AR_GUID_TYPE_SHIFT                24

/** Invalid or unknown GUID. This ID is to be used like a NULL value for
     pointers.
*/
#define AR_GUID_INVALID                   0

/** @} */ /* end_addtogroup spf_utils_guids */


#endif /* AR_GUIDS_H_ */

