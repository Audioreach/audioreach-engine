#ifndef IMCL_MUTE_API_H
#define IMCL_MUTE_API_H

/**
  @file imcl_mute_api.h

  @brief defines the Intent IDs for communication over Inter-Module Control
  Links (IMCL) between TTY modules(1x TTY/CTM/LTE_TTY) and Soft volume Modules

*/

/*==========================================================================
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 * =========================================================================*/

#define INTENT_ID_MUTE 0x08001195
#ifdef INTENT_ID_MUTE

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/


/*==============================================================================
  Intent ID - INTENT_ID_MUTE
==============================================================================*/
/**<
Intent ID for the control link between TTY module(1x/CTM/LTE_TTY in Rx path) and soft volume module(in Tx path).
TTY module communicates with the soft volume module to indicate whether to
mute or unmute the stream in the Tx path. The parameter - PARAM_ID_IMCL_MUTE
is used for this communication.

*/

/* ============================================================================
   Param ID
==============================================================================*/

#define PARAM_ID_IMCL_MUTE              0x08001196
/** @h2xmlp_parameter   {"PARAM_ID_IMCL_MUTE", PARAM_ID_IMCL_MUTE}
    @h2xmlp_description {Configures the mute flag}
    @h2xmlp_toolPolicy  {NO_SUPPORT}*/

#include "spf_begin_pack.h"

/* Payload of the PARAM_ID_IMCL_MUTE parameter used
 by the Volume Control module */
 /* Structure for the mute configuration parameter for a
 volume control module. */
struct param_id_imcl_mute_t
{
   uint32_t mute_flag;
/**< @h2xmle_description {Specifies whether mute is enabled}
     @h2xmle_rangeList   {"Disable"= 0;
                          "Enable"=1}
     @h2xmle_default     {0}  */
}
#include "spf_end_pack.h"
;

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif

#endif /* IMCL_MUTE_API_H */
