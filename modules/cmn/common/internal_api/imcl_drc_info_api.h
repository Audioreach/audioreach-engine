#ifndef IMCL_DRC_INFO_API_H
#define IMCL_DRC_INFO_API_H

/**
  @file imcl_drc_info_api.h

  @brief defines the Intent ID for communicating DRC parameters
  over Inter-Module Control Links (IMCL).

*/

/*==========================================================================
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 * =========================================================================*/

#define INTENT_ID_DRC_CONFIG 0x080010F3
#ifdef INTENT_ID_DRC_CONFIG

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/


/*==============================================================================
  Intent ID - INTENT_ID_DRC_CONFIG
==============================================================================*/
/**<
Intent ID to send DRC parameters over control link.

e.g.
Control link between Rx DRC and Tx AVC modules in voice path.
*/

/* ============================================================================
   Param ID
==============================================================================*/

#define PARAM_ID_IMCL_DRC_DOWN_COMP_THRESHOLD 	0x080010F4

/*==============================================================================
   Param structure defintions
==============================================================================*/

/* Structure definition for Parameter */
typedef struct param_id_imcl_drc_down_comp_threshold_t param_id_imcl_drc_down_comp_threshold_t;

/** @h2xmlp_parameter   {"PARAM_ID_IMCL_DRC_DOWN_COMP_THRESHOLD",
                         PARAM_ID_IMCL_DRC_DOWN_COMP_THRESHOLD}
    @h2xmlp_description {This parameter is used to send the drc down
                   	   	 compression threshold over the control link.}
    @h2xmlp_toolPolicy  {NO_SUPPORT} */

#include "spf_begin_pack.h"
struct param_id_imcl_drc_down_comp_threshold_t
{
   int16_t drc_rx_enable;
   /**< @h2xmle_description {Specifies whether DRC module is enabled.} */

   int16_t down_comp_threshold_L16Q7;
   /**< @h2xmle_description {DRC down-compression ratio in Q7 format.} */
}
#include "spf_end_pack.h"
;

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif // INTENT_ID_DRC_CONFIG

#endif /* IMCL_DRC_INFO_API_H */
