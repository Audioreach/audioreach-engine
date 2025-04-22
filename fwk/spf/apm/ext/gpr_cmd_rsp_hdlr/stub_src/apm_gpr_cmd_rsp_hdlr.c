/**
 * \file apm_gpr_cmd_rsp_hdlr.c
 *
 * \brief
 *     This file contains APM GPR Command Response Handler utilities
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/****************************************************************************
 * Function Definitions
 ****************************************************************************/

#include "apm_internal.h"

ar_result_t apm_rspq_gpr_cmd_rsp_handler(apm_t *apm_info_ptr, spf_msg_t *msg_ptr)
{
   gpr_packet_t *gpr_pkt_ptr;

   gpr_pkt_ptr = (gpr_packet_t *)msg_ptr->payload_ptr;

   if (gpr_pkt_ptr != NULL)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "apm_rspq_gpr_cmd_rsp_handler(): Unexpected GPR opcode: 0x%lX in stub mode",
             gpr_pkt_ptr->opcode);

      /** End and Return the GPR command */
      __gpr_cmd_end_command(gpr_pkt_ptr, AR_EUNEXPECTED);
   }

   return AR_EOK;
}
