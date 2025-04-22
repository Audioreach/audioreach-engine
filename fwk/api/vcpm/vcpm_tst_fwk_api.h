#ifndef _VCPM_TST_FWK_API_H_
#define _VCPM_TST_FWK_API_H_

/**
* \file vcpm.h
* \brief
*     This file contains declarations of the testfwk API, used for island duty cycling
*  usecases. Currently used for island usecase,
*
*
* \copyright
*  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
*  SPDX-License-Identifier: BSD-3-Clause-Clear
*/
#include "ar_defs.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */

/* Event ID used to inform testfwk to slip into island. If a module raised an override event
EVENT_ID_TESTFWK_ISLAND_SLEEP_OVERRIDE, this will be ignored. Else testfwk will sleep for the specified
duration. Currently implemented by VCPM to inform testfwk to enter island once voice packet is processed. */
#define EVENT_ID_TESTFWK_ISLAND_SLEEP 0x0800124A

/* Event ID used by module to indicate testfwk to override island sleep [EVENT_ID_TESTFWK_ISLAND_SLEEP].
Currently this event is raised by DTMF generation module on receiving a tone config, to prevent
slipping into island in Volte Island usecases. */
#define EVENT_ID_TESTFWK_ISLAND_SLEEP_OVERRIDE 0x0800125D

/*==============================================================================
   Type definitions
==============================================================================*/

#include "spf_begin_pack.h"
struct event_id_island_sleep_override_t
{
   uint32_t do_not_allow_island_sleep;
   /* Indiciates testfwk if dsp can slip into island. This will  override any subsequent
      island sleep requests.*/
}
#include "spf_end_pack.h"
;
typedef struct event_id_island_sleep_override_t event_id_island_sleep_override_t;

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* #ifdef _VCPM_TST_FWK_API_H_ */
