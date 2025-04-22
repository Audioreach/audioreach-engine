#ifndef _DUTY_CYCLE_CNTR_IF_H_
#define _DUTY_CYCLE_CNTR_IF_H_

/**
 * \file duty_cycle_cntr_if.h
 *
 * \brief
 *     This file defines Duty Cycle to container functions.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "apm_api.h"
#include "apm_sub_graph_api.h"
#include "apm_container_api.h"
#include "apm_module_api.h"

#include "spf_utils.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

/********************************************************************************************************/
/*                                          Params                                                      */
/********************************************************************************************************/

/**
 * This param ID is used as part of #SPF_MSG_CMD_SET_CFG.
 *
 * This parameter is used by container clients (E.g., DCM) to
 * unblock containers for island entry.
 *
 * Payload: cntr_param_id_unblock_island_entry_for_duty_cycling_t
 */
#define CNTR_PARAM_ID_UNBLOCK_ISLAND_ENTRY_FOR_DUTY_CYCLING 0x080013DB

typedef struct cntr_param_id_unblock_island_entry_for_duty_cycling_t cntr_param_id_unblock_island_entry_for_duty_cycling_t;

/**
 * Payload for CNTR_PARAM_ID_UNBLOCK_ISLAND_ENTRY_FOR_DUTY_CYCLING
 */
struct cntr_param_id_unblock_island_entry_for_duty_cycling_t
{
   uint32_t enable;
   /**< island entry enable/disable*/
};

/**
 * This param ID is used as part of #SPF_MSG_CMD_SET_CFG.
 *
 * This parameter is used by container clients (E.g., DCM) to
 * set the containers to exit from island.
 *
 * Payload: cntr_param_id_dcm_req_for_island_exit_t
 */
#define CNTR_PARAM_ID_DCM_REQ_FOR_ISLAND_EXIT 0x080013DC

typedef struct cntr_param_id_dcm_req_for_island_exit_t cntr_param_id_dcm_req_for_island_exit_t;

/**
 * Payload for CNTR_PARAM_ID_DCM_REQ_FOR_ISLAND_EXIT
 */
struct cntr_param_id_dcm_req_for_island_exit_t
{
   uint32_t enable;
   /**< island exit enable/disable*/
};

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef _DUTY_CYCLE_CNTR_IF_H_
