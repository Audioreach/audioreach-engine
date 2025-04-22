/**
 * \file cu_duty_cycle.c
 * \brief
 *     This file contains container common Duty Cycling manager functions.
 *
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */


#include "cu_i.h"


/* =======================================================================
Public Functions
========================================================================== */

/* Duty cycling containers registering with Duty Cycling Manager
 * As this is stubbed src, registration to DCM should throw an
 * error to inform caller that this operation is not allowed */
ar_result_t cu_register_with_dcm(cu_base_t *me_ptr)
{
   CU_MSG(me_ptr->gu_ptr->log_id, DBG_ERROR_PRIO, "Registration attempt made with DCM but duty cycling is stubbed.");
   return AR_ENOTIMPL;
}

ar_result_t cu_deregister_with_dcm(cu_base_t *me_ptr)
{
   return AR_EOK;
}


// Duty cycling containers sending Island Entry command response to Duty Cycling Manager
ar_result_t cu_send_island_entry_ack_to_dcm(cu_base_t *me_ptr)
{
   return AR_EOK;
}
ar_result_t cu_dcm_island_entry_exit_handler(cu_base_t *base_ptr,
                                             int8_t *   param_payload_ptr,
                                             uint32_t * param_size_ptr,
                                             uint32_t   pid)
{
   return AR_EOK;
}
