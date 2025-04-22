/*========================================================================
Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
SPDX-License-Identifier: BSD-3-Clause-Clear
========================================================================= */
/**
@file platform_internal_api.h

@brief Interface for containers to vote for hw resources hidden from MMPM
 */
/*===========================================================================
NOTE: The @brief description above does not appear in the PDF.
      The description that displays in the PDF is located in the
      posal_mainpage.dox file.
===========================================================================*/

#ifndef PLATFORM_INTERNAL_API_H
#define PLATFORM_INTERNAL_API_H

/* -----------------------------------------------------------------------
 ** Global definitions/forward declarations
 ** ----------------------------------------------------------------------- */

/* Message opcodes */
#define PM_SERVER_CMD_REGISTER (0x0100104C)   /* Register new client opcode */
#define PM_SERVER_CMD_DEREGISTER (0x0100104D) /* Deregister new client opcode */
#define PM_SERVER_CMD_REQUEST (0x0100104E)    /* Request opcode */
#define PM_SERVER_CMD_RELEASE (0x0100104F)    /* Release opcode */

#define PM_SERVER_CMD_REGISTER_HW (0x01001050)            /* Register HW opcode */
#define PM_SERVER_CMD_DEREGISTER_HW (0x01001051)          /* DeRegister HW opcode */
#define PM_SERVER_CMD_REQUEST_HW (0x01001052)             /* Request for HW resources opcode */
#define PM_SERVER_CMD_RELEASE_HW (0x01001053)             /* Release for HW resources opcode */

/** message to subscribe to voice timer */
#define SPF_MSG_VTIMER_SUBSCRIBE      0x0100101C

/** message to unsubscribe to voice timer */
#define SPF_MSG_VTIMER_UNSUBSCRIBE      0x01001042

/** Message to re-sync voice timer with the updated timing reference */
#define SPF_MSG_VTIMER_RESYNC      0x01001058


#endif // PLATFORM_INTERNAL_API_H
