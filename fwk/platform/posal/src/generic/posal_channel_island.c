/**
 * \file posal_channel_island.c
 * \brief
 *  This file contains a utility to form a channel of a combination of up to
 *  32 signals/queues/timers. Client can wait on any combination thereof and
 *  be woken when any desired element is active.
 *
 * \copyright
 *    Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *    SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/* =======================================================================
INCLUDE FILES FOR MODULE
========================================================================== */

#include "posal.h"
#include "posal_internal.h"
/*--------------------------------------------------------------*/
/* Macro definitions                                            */
/* -------------------------------------------------------------*/

/* -----------------------------------------------------------------------
** Constant / Define Declarations
** ----------------------------------------------------------------------- */

/* =======================================================================
**                          Function Definitions
** ======================================================================= */

/****************************************************************************
** Channels
*****************************************************************************/
uint32_t posal_channel_wait(posal_channel_t pChannel, uint32_t unEnableBitfield)
{
    return posal_channel_wait_inline(pChannel, unEnableBitfield);
}

uint32_t posal_channel_poll(posal_channel_t pChannel, uint32_t unEnableBitfield)
{
   return posal_channel_poll_inline(pChannel, unEnableBitfield);
}
