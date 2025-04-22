/**
 * \file apm_pwr_mgr_utils.c
 *
 * \brief
 *     This file contains APM Power Management Utilities
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
#include "apm_memmap_api.h"

/**==============================================================================
   Function Declaration
==============================================================================*/

ar_result_t apm_pwr_mgr_vote(apm_t *apm_info_ptr);

ar_result_t apm_pwr_mgr_devote(apm_t *apm_info_ptr);

/**==============================================================================
   Global Defines
==============================================================================*/

apm_pwr_mgr_utils_vtable_t pwr_mgr_util_funcs = {.apm_pwr_mgr_vote_fptr = apm_pwr_mgr_vote,

                                                 .apm_pwr_mgr_devote_fptr = apm_pwr_mgr_devote };

ar_result_t apm_pwr_mgr_vote(apm_t *apm_info_ptr)
{
   ar_result_t result = AR_EOK;

   /** Insert Max vote synchronously */
   if (AR_EOK == (result = posal_power_mgr_request_max_out(apm_info_ptr->pm_info.pm_handle_ptr,
                                                           apm_info_ptr->gp_signal_ptr,
                                                           APM_LOG_ID)))
   {
      /** Increment the vote counter. */
      apm_info_ptr->pm_info.pm_vote_count++;
   }

   return result;
}

ar_result_t apm_pwr_mgr_devote(apm_t *apm_info_ptr)
{
   ar_result_t result = AR_EOK;
   /** Insert delayed release with a delay of 20 ms
    *  Only if the counter reaches 0, then make the actual vote */
   if (apm_info_ptr->pm_info.pm_vote_count > 0)
   {
      apm_info_ptr->pm_info.pm_vote_count--;

      AR_MSG(DBG_MED_PRIO,
             "apm_pwr_mgr_devote(): APM decrementing release ref count to %lu",
             apm_info_ptr->pm_info.pm_vote_count);

      /** Get the pointer to current commnad control object */
      /** If the ref count reaches zero, release the votes */
      if (0 == apm_info_ptr->pm_info.pm_vote_count)
      {
#ifdef SIM
         apm_cmd_ctrl_t *apm_cmd_ctrl_ptr;
         apm_cmd_ctrl_ptr = apm_info_ptr->curr_cmd_ctrl_ptr;
         /** Only do delayed release if not in SIM */
         if (APM_CMD_GRAPH_CLOSE == apm_cmd_ctrl_ptr->cmd_opcode || APM_CMD_CLOSE_ALL == apm_cmd_ctrl_ptr->cmd_opcode || APM_CMD_SET_CFG == apm_cmd_ctrl_ptr->cmd_opcode || 
             APM_CMD_SHARED_SATELLITE_MEM_UNMAP_REGIONS == apm_cmd_ctrl_ptr->cmd_opcode)
         {
            result = posal_power_mgr_release_max_out(apm_info_ptr->pm_info.pm_handle_ptr, APM_LOG_ID, 0);
         }
         else
         {
            result = posal_power_mgr_release_max_out(apm_info_ptr->pm_info.pm_handle_ptr, APM_LOG_ID, APM_PM_DELAY);
         }
#else
         result = posal_power_mgr_release_max_out(apm_info_ptr->pm_info.pm_handle_ptr, APM_LOG_ID, APM_PM_DELAY);
#endif
      } /** End of if (0 == apm_info_ptr->pm_info.pm_vote_count) */

   } /** End of if (apm_info_ptr->pm_info.pm_vote_count > 0) */

   return result;
}

ar_result_t apm_pwr_mgr_utils_init(apm_t *apm_info_ptr)
{
   ar_result_t result = AR_EOK;

   apm_info_ptr->ext_utils.pwr_mgr_vtbl_ptr = &pwr_mgr_util_funcs;

   return result;
}
