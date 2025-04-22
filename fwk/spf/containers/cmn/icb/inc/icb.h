#ifndef INTER_CONT_BUF_H_
#define INTER_CONT_BUF_H_

/**
 * \file icb.h
 *  
 * \brief
 *  
 *     inter-container-buffering
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "spf_utils.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

typedef struct icb_frame_length_t
{
   uint32_t sample_rate;
   /**< Sample rate in Hz. if zero, frame_len_us used, else frame_len_samples is used*/

   uint32_t frame_len_samples;
   /**< Frame length in samples. */

   uint32_t frame_len_us;
   /**< Frame length in microseconds. */
} icb_frame_length_t;

typedef union icb_flags_t
{
  struct
  {
     uint32_t variable_input : 1;
     /**< For downstream: indicates whether the input data consumed is fixed or variable. A data duration modifying module
      *can
      *   consume different sizes each time.
      **/

     uint32_t variable_output : 1;
     /**< For upstream : indicates whether the output frame length is fixed or variable.
      *   If there's a data duration modifying module (such as Rate matching module), it can output different sizes each
      *time.
      **/

     uint32_t is_real_time : 1;
     /**< Either the container itself is timer driven, or either upstream or downstream is real time.
      * For upstream : this flag indicates container itself or some upstream of the container is real time.
      * For downstream : this flag indicates container itself or  some downstream of the container is real time.
      **/

     uint32_t is_default_single_buffering_mode : 1;
     /** < This flag is set to FALSE by default to indicate dual buffer mode
      *    For The MDF use-cases, this flag would be set to TRUE in the satellite containers with RD/WR EP modules
      **/
  };
  uint32_t word;

} icb_flags_t;

typedef struct icb_upstream_info_t
{
   icb_frame_length_t len;
   /**< Frame length requirement */

   uint32_t period_us;
   /**< Period at which the upstream triggers processing. Number of regular buffers are created
    *   based on this value, as the input queue needs to hold the amount of data required for
    *   every processing cycle. */

   uint32_t log_id;
   /**< Logging ID */

   uint32_t sid;
   /**< Scenario ID */

   icb_flags_t flags;
   /**< flags related to upstream container */

   bool_t disable_otp;
   /**< indicates upstream doesn't need one time prebuf */
} icb_upstream_info_t;

typedef struct icb_downstream_info_t
{
   icb_frame_length_t len;
   /**< Frame length requirement */

   uint32_t period_us;
   /**< Period at which the downstream triggers processing. Number of regular buffers are created
    *   based on this value, as the input queue needs to hold the amount of data required for
    *   every processing cycle. */

   uint32_t sid;
   /**< Scenario ID */

   icb_flags_t flags;
   /**< flags related to downstream container */

} icb_downstream_info_t;

typedef struct icb_calc_output_t
{
   uint32_t num_reg_bufs;
   /**< number of regular buffers to be used by upstream */

   uint32_t num_reg_prebufs;
   /**< number of regular pre-buffers to be used by upstream */

   icb_frame_length_t otp;
   /**< there's only one one-time-prebuffer whose length is given by this. */
} icb_calc_output_t;

ar_result_t icb_determine_buffering(icb_upstream_info_t *  us_ptr,
                                    icb_downstream_info_t *ds_ptr,
                                    icb_calc_output_t *    result_ptr);

static inline uint64_t icb_ceil(uint64_t x, uint64_t y)
{
   return (uint64_t)((0 == x) ? 0 : ((((x - 1) / y) + 1)));
}

static inline uint32_t icb_samples_to_us(uint32_t samples, uint32_t sample_rate)
{
   return (uint32_t)(icb_ceil(((uint64_t)samples * 1000000ULL), (uint64_t)sample_rate));
}

static inline uint32_t icb_us_to_samples(uint32_t us, uint32_t sample_rate)
{
   return (uint32_t)(icb_ceil(((uint64_t)us * sample_rate), (uint64_t)1000000ULL));
}

static inline uint32_t icb_get_frame_len_samples(icb_frame_length_t *icb_frame_len_ptr, uint32_t target_buf_sample_rate)
{
   // Scaling by sample rates isn't useful here - because even if sample rates are multiple, we could hit rounding issues
   // if frame_len_samples does not have same multiplier. So if sample rates aren't same, scaling with frame duration will
   // suffice.
   if (icb_frame_len_ptr->sample_rate == target_buf_sample_rate)
   {
      return icb_frame_len_ptr->frame_len_samples;
   }
   else
   {
      return (uint32_t)icb_us_to_samples(icb_frame_len_ptr->frame_len_us, target_buf_sample_rate);
   }
}

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef INTER_CONT_BUF_H_
