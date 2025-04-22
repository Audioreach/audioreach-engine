/**
 * \file apm_spf_cmd_hdlr.c
 *
 * \brief
 *     This file contains APM SPF Command Handler utilities
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/****************************************************************************
 * INCLUDE HEADER FILES                                                     *
 ****************************************************************************/

#include "apm_internal.h"

/****************************************************************************
 * Function Definitions
 ****************************************************************************/

ar_result_t apm_cmdq_spf_cmd_handler(apm_t *apm_info_ptr, spf_msg_t *msg_ptr)
{
   if (msg_ptr != NULL)
   {
      AR_MSG(DBG_ERROR_PRIO, "apm_cmdq_spf_cmd_handler(): Unexpected SPF message in stub mode");

      /** Return the SPF command */
      spf_msg_return_msg(msg_ptr);
   }

   return AR_EOK;
}
