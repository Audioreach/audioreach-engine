#ifndef IMCL_MODULE_GAIN_API_H
#define IMCL_MODULE_GAIN_API_H

/**
  @file imcl_module_gain_api.h

  @brief defines the Intent ID for communicating module's gain
  over Inter-Module Control Links (IMCL).

*/

/*==========================================================================
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 * =========================================================================*/

#define INTENT_ID_GAIN_INFO 0x080010F5
#ifdef INTENT_ID_GAIN_INFO

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/


/*==============================================================================
  Intent ID - INTENT_ID_GAIN_INFO
==============================================================================*/
/**<
Intent ID for the control link to send gain information.
This intent can be used by a module to send its gain to another module.

e.g.
Soft Volume Control sends the target gain to AVC-TX module in voice path.
*/

/* ============================================================================
   Param ID
==============================================================================*/
/**<
 This parameter is used to send the target gain for each channel.
 It may be different than current gain.

 */
#define PARAM_ID_IMCL_TARGET_GAIN   0x080010F6

/*==============================================================================
   Param structure defintions
==============================================================================*/
typedef struct imcl_gain_config_t imcl_gain_config_t;

#include "spf_begin_pack.h"
struct imcl_gain_config_t
{
   uint32_t channel_mask_lsb;
   /**< @h2xmle_description  {Lower 32 bits of the mask that indicates the corresponding
                              channel whose maximum tap length is to be set.\n
                              - Set the bits corresponding to 1 to 31 channels of standard
                                channel mapping (channels are mapped per standard channel mapping)\n
                              - Position of the bit to set 1 (left shift)(channel_map) \n}
      @h2xmle_default      {0}
   */

   uint32_t channel_mask_msb;
   /**< @h2xmle_description  {Upper 32 bits of the mask that indicates the corresponding channel
                              whose maximum tap length is to be set. \n
                              - Set the bits corresponding to 32 to 63 channels of standard channel
                                mapping (channels are mapped per standard channel mapping) \n
                              - Position of the bit to set 1 (left shift)(channel_map - 32)}
        @h2xmle_default       {0} */

   uint32_t gain;
   /**< @h2xmle_description  {Gain value for the above channels in Q28 format}
        @h2xmle_dataFormat   {Q28}
        @h2xmle_default      {0} */
}
#include "spf_end_pack.h"
;

/* Structure definition for Parameter */
typedef struct param_id_imcl_gain_t param_id_imcl_gain_t;

/** @h2xmlp_parameter   {"PARAM_ID_IMCL_TARGET_GAIN",
                         PARAM_ID_IMCL_TARGET_GAIN}
    @h2xmlp_description {This parameter is used to send the target gain for each channel.}
    @h2xmlp_toolPolicy  {NO_SUPPORT} */

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
struct param_id_imcl_gain_t
{
   uint32_t num_config;
   /**< @h2xmle_description  {Number of channels-gain configurations provided}
        @h2xmle_range        {1..63}
        @h2xmle_default      {1}  */

   imcl_gain_config_t gain_data[0];
   /**< @h2xmle_description  {Payload consisting of all channels-gain pairs }
        @h2xmle_variableArraySize {num_config}	*/
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif // INTENT_ID_GAIN_INFO

#endif /* IMCL_MODULE_GAIN_API_H */
