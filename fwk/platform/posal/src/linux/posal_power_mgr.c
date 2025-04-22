/**
 * \file posal_power_mgr.c
 * \brief
 *  	This file contains profiling utilities.
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/* =======================================================================
INCLUDE FILES FOR MODULE
========================================================================== */
#include "posal_power_mgr.h"
#include "platform_internal_api.h"
#define POSAL_POWER_MGR_INVALID_CLIENT_ID 0

/** PM_WRAPPER max out information */
#define POSAL_POWER_MGR_MAX_OUT_BW (250 * 1024 * 1024)
#define POSAL_POWER_MGR_MAX_OUT_MPPS (500)
#define POSAL_POWER_MGR_MAX_OUT_FLOOR_CLK (500)

/* =======================================================================
 **                          Function Definitions
 ** ======================================================================= */

/** Sets up channel and response queue for local messaging */
static ar_result_t posal_power_mgr_setup_signal(posal_signal_t *signal_pptr, posal_channel_t *channel_pptr)
{
   ar_result_t result = AR_EOK;

   return result;
}

/** Destroys channel and response queue for local messaging */
static void posal_power_mgr_destroy_signal(posal_signal_t *pp_sig, posal_channel_t *pp_chan)
{

}

/** @ingroup posal_pm_wrapper
  Sends request to PM Server

  @return
  returns error code.

  @dependencies
  None.
 */
ar_result_t posal_power_mgr_request(posal_pm_request_info_t *request_info_ptr)
{
   ar_result_t cmd_result = AR_EOK;

   return cmd_result;
}

/** @ingroup posal_pm_wrapper
  Sends release to PM Server

  @return
  returns error code.

  @dependencies
  None.
 */
ar_result_t posal_power_mgr_release(posal_pm_release_info_t *release_info_ptr)
{
   ar_result_t cmd_result = AR_EOK;

   return cmd_result;
}

/** @ingroup posal_pm_wrapper
  Registers for kpps and bw

  @return
  returns error code.

  @dependencies
  None.
 */
ar_result_t posal_power_mgr_register(posal_pm_register_t register_info,
                                      posal_pm_handle_t*   pm_handle_pptr,
                                      posal_signal_t       wait_signal_ptr,
                                      uint32_t             log_id)
{
   return AR_EOK;
}

/** @ingroup posal_pm_wrapper
  Deregisters with PM Server

  @return
  returns error code.

  @dependencies
  None.
 */
ar_result_t posal_power_mgr_deregister(posal_pm_handle_t*  pm_handle_pptr, uint32_t log_id)
{
   return AR_EOK;
}

/**
 * bumps up the bus and Q6 clocks.
 */
ar_result_t posal_power_mgr_request_max_out(posal_pm_handle_t  pm_handle_ptr,
                                            posal_signal_t      wait_signal,
                                            uint32_t             log_id)
{
   ar_result_t cmd_result = AR_EOK;

   return cmd_result;
}

/**
 * releases the bus and Q6 clocks.
 */
ar_result_t posal_power_mgr_release_max_out(posal_pm_handle_t pm_handle_ptr, uint32_t log_id, uint32_t delay_ms)
{
   ar_result_t cmd_result  = AR_EOK;

   return cmd_result;
}

/**
 * Checks PM registration.
 */
bool_t posal_power_mgr_is_registered(posal_pm_handle_t pm_handle_ptr)
{
   return TRUE;
}

/**
* No platform specific initialization currently
*/
void posal_power_mgr_init()
{
   return;
}

/**
* No platform specific de-initialization currently
*/
void posal_power_mgr_deinit()
{
   return;
}
