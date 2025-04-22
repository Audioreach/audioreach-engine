/**
 *  \file imcl_prbdma_control.h
 *
 *  \brief
 *     This file contains API's for the control link between Codec DMA and AAD module.
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef _IMCL_PRBDMA_CONTROL_API_H_
#define _IMCL_PRBDMA_CONTROL_API_H_

/*------------------------------------------------------------------------------
 *  Header Includes
 *----------------------------------------------------------------------------*/
#include "ar_error_codes.h"
#include "imcl_fwk_intent_api.h"

#ifdef INTENT_ID_PRBDMA_CONTROL

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/** -----------------------------------------------------------------------
 ** Type Declarations
 ** ----------------------------------------------------------------------- */

/* clang-format off */

/**< Header*/
typedef struct imcl_prbdma_header_t
{
   // specific purpose understandable to the IMCL peers only
   uint32_t opcode;

   // Size (in bytes) for the payload specific to the intent.
   uint32_t actual_data_len;
} imcl_prbdma_header_t;

/*==============================================================================
   Constants
==============================================================================*/

/** This param is sent by AAD to Codec DMA to request enabling HW AAD block.
 *  The payload has prebuffer size, Codec DMA is expected to allocate the prebuffer
 *  enable HW AAD, and send an ACK i.e IMCL_PARAM_ID_HW_AAD_ENABLE_ACK
 *
 *  HW AAD needs to be enabled in certain sequence w.r.t to LPAIF and PRB DMA, hence
 *  AAD module cannot asynchronosly enable itself.
 *
 *  payload layout:
 *     < imcl_prbdma_header_t >
 *     < imcl_param_id_hw_aad_enable_req_t >
 *
 */
#define IMCL_PARAM_ID_HW_AAD_ENABLE_REQ                 0x08001A75

/*==============================================================================
   Type Definitions
==============================================================================*/

typedef struct imcl_param_id_hw_aad_enable_req_t imcl_param_id_hw_aad_enable_req_t;

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
struct imcl_param_id_hw_aad_enable_req_t
{
   uint32_t         preroll_buffer_size_in_ms;
   /**<  @h2xmle_description { preroll buffer size in milliseconds. }
         @h2xmle_range       {0..0xFFFFFFFF}
         @h2xmle_default     {0} */
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;

/*==============================================================================
   Constants
==============================================================================*/

/** This param is sent by Codec DMA as a response to the enable request i.e
 *  (IMCL_PARAM_ID_HW_AAD_ENABLE_REQ). Codec DMA allocates preroll buffer,
 *  enables PRB DMA and HW AAD, and then returns this a ack to AAD module.
 *
 *  The payload includes infor about the Preroll buffer addr and PRB DMA
 *  write position query function pointer. AAD module is expected to forward
 * this info to Dam through the ctrl link. Dam will use this info to initalize
 * the driver and handles draining the data to its outputs.
 *
 * The data buffer by PRB DMA is expected to be PCM and fixed point
 *
 *  payload layout:
 *     < imcl_prbdma_header_t >
 *     < imcl_param_id_hw_aad_enable_ack_t >
 *
 */
#define IMCL_PARAM_ID_HW_AAD_ENABLE_ACK                 0x08001A76

/*==============================================================================
   Type Definitions
==============================================================================*/

/** Client is expected to pass this structure to imcl_prbdma_get_writer_ptr_fn_ptr_t. And the function populates the
 * write position and returns to the client. */
typedef struct prb_wr_position_info_t
{
   uint32_t latest_write_addr;
   /**< @h2xmle_description {Address of the latest written sample.}
         @h2xmle_range       {0..0xFFFFFFFF}
         @h2xmle_default     {0} */

   int64_t latest_write_sample_ts;
   /**< @h2xmle_description { Timestamp associated with the latest written sample. }
        @h2xmle_range       {0..0xFFFFFFFF}
        @h2xmle_default     {0} */

   uint32_t   is_ts_valid;
   /**< @h2xmle_description {is above TS valid}
         @h2xmle_range       {0..0xFFFFFFFF}
         @h2xmle_default     {0} */

}prb_wr_position_info_t;

typedef ar_result_t (*imcl_prbdma_get_writer_ptr_fn_ptr_t) (uint32_t writer_handle /**in param*/, prb_wr_position_info_t *ret_ptr /**in/out param*/);

typedef struct imcl_param_id_hw_aad_enable_ack_t imcl_param_id_hw_aad_enable_ack_t;

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
struct imcl_param_id_hw_aad_enable_ack_t
{
   uint32_t circular_buffer_base_address;
   /**< @h2xmle_description { Buffer base address. }
         @h2xmle_range       {0..0xFFFFFFFF}
         @h2xmle_default     {0} */

   uint32_t circular_buffer_size_in_us;
   /**< @h2xmle_description { Size of the circular buffer in microseconds.}
        @h2xmle_range       {0..0xFFFFFFFF}
        @h2xmle_default     {0} */

   uint32_t circular_buffer_size_in_bytes;
   /**< @h2xmle_description { Size of the circular buffer in bytes.}
        @h2xmle_range       {0..0xFFFFFFFF}
        @h2xmle_default     {0} */

   imcl_prbdma_get_writer_ptr_fn_ptr_t get_writer_ptr_fn;
   /**< @h2xmle_description { callback function pointer to get the writer address.}
        @h2xmle_range       { 0..0xFFFFFFFF }
        @h2xmle_default     { 0 } */

   uint32_t writer_handle;
   /**< @h2xmle_description { callback function pointer to get the writer address.}
        @h2xmle_range       { 0..0xFFFFFFFF }
        @h2xmle_default     { 0 } */

   uint32_t num_channels;
   /**< @h2xmle_description { Number of channels written by the writer. }
         @h2xmle_range       {0..0xFFFFFFFF}
         @h2xmle_default     {0} */

   uint32_t bits_per_sample;
   /**< @h2xmle_description { word size of each sample }
        @h2xmle_range       {0..32}
        @h2xmle_default    {16} */

   uint32_t sampling_rate;
   /**< @h2xmle_description { sample rate of the mic data. }
         @h2xmle_range       {0..0xFFFFFFFF}
         @h2xmle_default     {0} */
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;

/*==============================================================================
   Constants
==============================================================================*/

/** This param is sent AAD module to Codec DMA module. Its used to switch the PRB
 *  DMA mode between SHORT and LONG buffer mode.
 *
 *    - Short buffer mode means PRBDMA moves data every LPAIF interrupt and
 *       raise PRBDMA done interrupt to ADSP.
 *
 *    - Long buffer mode means PRBMDA moves data every 'N' LPAIF interrupts and
 *       PRBDMA done interrupt will be masked until HW AAD requests to switch to
 *      short buffer mode.
 *
 *  payload layout:
 *     < imcl_prbdma_header_t >
 *     < imcl_param_id_switch_prbdma_mode_t >
 *
 */
#define IMCL_PARAM_ID_SWITCH_PRBDMA_MODE                  0x08001A77

/*==============================================================================
   Type Definitions
==============================================================================*/
typedef enum
{
   SHORT_BUFFER_MODE = 0,
   LONG_BUFFER_MODE  = 1,
   INVALID_MODE      = 0xFFFFFFFF
} imcl_prbmda_mode_t;

typedef struct imcl_param_id_switch_prbdma_mode_t imcl_param_id_switch_prbdma_mode_t;

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
struct imcl_param_id_switch_prbdma_mode_t
{
   imcl_prbmda_mode_t         mode;
   /**<  @h2xmle_description { PRBMDA operation mode}
         @h2xmle_rangeList   {"SHORT_BUFFER_MODE"= 0;
                              "LONG_BUFFER_MODE" = 1}
         @h2xmle_default     {0} */
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;

/* clang-format on */

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif // INTENT_ID_PRBDMA_CONTROL

#endif /* _IMCL_PRBDMA_CONTROL_API_H_ */
