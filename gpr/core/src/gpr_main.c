/**
 * \file gpr_main.c
 * \brief
 *  	This file contains GPR main implementation
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/******************************************************************************
 * Includes                                                                    *
 *****************************************************************************/

#include "ar_osal_error.h"
#include "ar_types.h"
#include "gpr_log.h"
#include "gpr_api_i.h"
#include "ar_msg.h"

/*****************************************************************************
 * Variables                                                                 *
 ****************************************************************************/
bool_t gpr_init_flag = FALSE;
bool_t gpr_init_domain_flags[GPR_PL_NUM_TOTAL_DOMAINS_V] = {FALSE};

/*****************************************************************************
 * Core Routine Implementations                                              *
 ****************************************************************************/
GPR_EXTERNAL uint32_t gpr_init(void)
{
   uint32_t rc;
   for (uint32_t i=0; i<GPR_PL_NUM_TOTAL_DOMAINS_V; i++)
   {
      if (gpr_init_domain_flags[i] || gpr_init_flag)
      {
         AR_MSG(DBG_HIGH_PRIO, "GPR is already initialized");
         return AR_EOK;
      }
   }
   rc = gpr_log_init();
#ifndef DISABLE_DEINIT
   if (rc)
   {
      (void)gpr_log_deinit();
      return AR_EFAILED;
   }
#endif
   rc = gpr_drv_init();
#ifndef DISABLE_DEINIT
   if (rc)
   {
      (void)gpr_deinit();
      return AR_EFAILED;
   }
#endif
   gpr_init_flag = TRUE;
   return AR_EOK;
}

/**
  @brief Enhanced gpr_init function to allow domain based initialization.

  @detdesc
  The enhanced gpr_init function allow gpr driver to be initialized by multiple clients
  with various domain ids. For instance, both GSL and SPF-on-ARM processes running on
  ARM cores need to initilize their respective gpr drivers. With domain id, IPC tables,
  downlink port and driver fd can be distinguished among these clients. Note: each client
  process is assumed to call either gpr_init() or gpr_init_domain() once.

  @return
  #AR_EOK when successful.
*/
GPR_EXTERNAL uint32_t gpr_init_domain(uint32_t domain_id)
{
   uint32_t rc;
   if (gpr_init_flag || (gpr_init_domain_flags[domain_id] == TRUE))
   {
      AR_MSG(DBG_HIGH_PRIO, "GPR is already initialized for domain id %d", domain_id);
      return AR_EOK;
   }
   rc = gpr_log_init();
#ifndef DISABLE_DEINIT
   if (rc)
   {
      (void)gpr_log_deinit();
      return AR_EFAILED;
   }
#endif
   rc = gpr_drv_init_domain(domain_id);
#ifndef DISABLE_DEINIT
   if (rc)
   {
      (void)gpr_deinit();
      return AR_EFAILED;
   }
#endif
   gpr_init_flag = TRUE;
   gpr_init_domain_flags[domain_id] = TRUE;
   return AR_EOK;
}

#ifndef DISABLE_DEINIT
GPR_EXTERNAL uint32_t gpr_deinit(void)
{
   AR_MSG(DBG_HIGH_PRIO, "De-initializing GPR driver");
   (void)gpr_drv_deinit();

   (void)gpr_log_deinit();

   gpr_init_flag = FALSE;
   return AR_EOK;
}
#endif
