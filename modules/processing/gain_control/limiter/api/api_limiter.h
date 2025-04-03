#ifndef API_LIMITER_H
#define API_LIMITER_H
/*========================================================================*/
/**
@file api_limiter.h

@brief api_limiter.h: This file contains the Module Id, Param IDs and configuration
    structures exposed by the Limiter Module.
*/
/* =========================================================================
  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
  SPDX-License-Identifier: BSD-3-Clause
   ========================================================================== */

/*------------------------------------------------------------------------
 * Include files
 * -----------------------------------------------------------------------*/
#include "ar_defs.h"


/** @ingroup ar_spf_mod_limiter_mod
    Identifier for the parameter used by the Limiter module to configure the
    tuning parameters of the limiter.

    @msgpayload
    param_id_limiter_cfg_t
 */
#define PARAM_ID_LIMITER_CFG                         0x08001050

/*# @h2xmlp_parameter   {"PARAM_ID_LIMITER_CFG", PARAM_ID_LIMITER_CFG}
    @h2xmlp_description {ID for the parameter used by the Limiter module to
                         configure the tuning parameters of the limiter.} */

/** @ingroup ar_spf_mod_limiter_mod
    Payload of #PARAM_ID_LIMITER_CFG, which configures the output bit width,
    number of channels, and sampling rate of the Limiter module. */
#include "spf_begin_pack.h"
struct param_id_limiter_cfg_t
{
   uint32_t threshold;
   /**< Threshold in decibels for the limiter output.

       @valuesbul
        - For a 16-bit use case, the limiter threshold is [-96 dB..0 dB].
        - For a 24-bit use case, the limiter threshold is [-162 dB..24 dB].
        - For a true 32-bit use case, the limiter threshold is [-162 dB..0 dB].

        If a value out of this range is configured, it is automatically limited
        to the upper bound or low bound in the DSP processing. */

   /*#< @h2xmle_description {Threshold in decibels for the limiter output.
                             For a 16-bit use case, the limiter threshold is 
                             [-96 dB..0 dB]. \n
                             For a 24-bit use case, the limiter threshold is
                             [-162 dB..24 dB]. \n
                             For a true 32-bit use case, the limiter threshold
                             is [-162 dB..0 dB]. \n
                             If a value out of this range is configured, it is
                             automatically limited to the upper bound or low
                             bound in the DSP processing.}
        @h2xmle_default     {133909036}
        @h2xmle_range       {0..2127207634}
        @h2xmle_dataFormat  {Q27} */

   uint32_t makeup_gain;
   /**< Makeup gain in decibels for the limiter output.

       @values 1 through 32228 decibels (Default = 256) */

   /*#< @h2xmle_description {Makeup gain in decibels for the limiter output.}
        @h2xmle_default     {256}
        @h2xmle_range       {1..32228}
        @h2xmle_dataFormat  {Q8} */

   uint32_t gc;
   /**< Coefficient for the limiter gain recovery.

       @values 0..32767 (Default = 32440) */

   /*#< @h2xmle_description {Coefficient for the limiter gain recovery.}
        @h2xmle_range       {0..32767}
        @h2xmle_default     {32440}
        @h2xmle_dataFormat  {Q15} */

   uint32_t max_wait;
   /**< Maximum limiter waiting time in seconds.

       @values 0 through 327 (Default = 33) */

   /*#< @h2xmle_description {Maximum limiter waiting time in seconds.}
        @h2xmle_range       {0..327}
        @h2xmle_default     {33}
        @h2xmle_dataFormat  {Q15} */

   uint32_t gain_attack;
   /**< Limiter gain attack time in seconds (Q31 format). */

   /*#< @h2xmle_description {Limiter gain attack time.}
        @h2xmle_default     {188099712}
        @h2xmle_range       {0..2147483648}
        @h2xmle_dataFormat  {Q31} */

   uint32_t gain_release;
   /**< Limiter gain release time in seconds (Q31 format) */

   /*#< @h2xmle_description {Limiter gain release time.}
        @h2xmle_default     {32559488}
        @h2xmle_range       {0..2147483648}
        @h2xmle_dataFormat  {Q31} */

   uint32_t attack_coef;
   /**< Speed coefficient for the limiter gain attack time. */

   /*#< @h2xmle_description {Speed coefficient for the limiter gain attack
                             time.}
        @h2xmle_default     {32768}
        @h2xmle_range       {32768..3276800}
        @h2xmle_dataFormat  {Q15} */

   uint32_t release_coef;
   /**< Speed coefficient for the limiter gain release time. */

   /*#< @h2xmle_description {Speed coefficient for the limiter gain release
                             time.}
        @h2xmle_default     {32768}
        @h2xmle_range       {32768..3276800}
        @h2xmle_dataFormat  {Q15} */

   uint32_t hard_threshold;
   /*#< Hard threshold in decibels for the limiter output.

        - For a 16-bit use case, the limiter hard threshold is [-96 dB..0 dB].
        - For a 24-bit use case, the limiter hard threshold is
          [-162 dB..24 dB].
        - For a true 32-bit use case, the limiter hard threshold is
          [-162 dB..0 dB].

        If a value out of this range is configured, it is automatically limited
        to the upper bound or low bound in the DSP processing. */

   /*#< @h2xmle_description {Hard threshold in decibels for the limiter output.
                             For a 16-bit use case, the limiter hard threshold
                             is [-96 dB..0 dB]. \n
                             For a 24-bit use case, the limiter hard threshold
                             is [-162 dB..24 dB]. \n
                             For a true 32-bit use case, the limiter hard
                             threshold is [-162 dB..0 dB]. \n
                             If a value out of this range is configured, it
                             is automatically limited to the upper bound or low
                             bound in the DSP processing.}
        @h2xmle_default     {134217728}
        @h2xmle_range       {0..2127207634}
        @h2xmle_dataFormat  {Q27} */
}
#include "spf_end_pack.h"
;
typedef struct param_id_limiter_cfg_t param_id_limiter_cfg_t;


#endif /* API_LIMITER_H */
