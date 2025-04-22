#ifndef IMCL_P_EQ_VOL_API_H
#define IMCL_P_EQ_VOL_API_H
/**
  @file imcl_p_eq_vol_api.h

  @brief defines the Intent IDs for communication over Inter-Module Control
  Links (IMCL) betweeen Soft Volume Module and Popless Equalizer module
*/

/*==========================================================================
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear
 * =========================================================================*/

#define INTENT_ID_P_EQ_VOL_HEADROOM 0x08001118
#ifdef INTENT_ID_P_EQ_VOL_HEADROOM

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/


/**< Header - Any IMCL message between soft volume module and Popless equalizer
 *  will have the following header followed by the actual payload.
 *  The peers have to parse the header accordingly*/

typedef struct vol_imcl_header_t
{
   // specific purpose understandable to the IMCL peers only
   uint32_t opcode;

   // Size (in bytes) for the payload specific to the intent.
   uint32_t actual_data_len;

} vol_imcl_header_t;


#define MIN_INCOMING_IMCL_PARAM_SIZE_P_EQ_VOL (sizeof(vol_imcl_header_t) + sizeof(intf_extn_param_id_imcl_incoming_data_t))

/*==============================================================================
  Intent ID - INTENT_ID_P_EQ_VOL_HEADROOM
==============================================================================*/
// Intent ID for the control link between Soft Vol and Popless Equalizer.


/* ============================================================================
   Param ID
==============================================================================*/

#define PARAM_ID_P_EQ_VOL_HEADROOM                    0x08001117

/*==============================================================================
   Param structure defintions
==============================================================================*/
/* Type definition for the above structure. */
typedef struct p_eq_vol_headroom_param_t p_eq_vol_headroom_param_t;
/** @h2xmlp_parameter    {"PARAM_ID_P_EQ_VOL_HEADROOM", PARAM_ID_P_EQ_VOL_HEADROOM}
    @h2xmlp_description  { Headroom in millibels communicated from popless equalizer to soft vol module\n}
    @h2xmlp_toolPolicy   {NO_SUPPORT} */

#include "spf_begin_pack.h"
struct p_eq_vol_headroom_param_t
{
   uint32_t headroom_in_millibels;
   /**< @h2xmle_description {Headroom requirement of the module.}
        @h2xmle_default     {0x0} */

}
#include "spf_end_pack.h"
;


#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif // INTENT_ID_P_EQ_VOL_HEADROOM

#endif /* #ifndef IMCL_P_EQ_VOL_API_H*/
