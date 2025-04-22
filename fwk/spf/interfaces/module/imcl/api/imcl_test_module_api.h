#ifndef IMCL_TEST_MODULE_API_H
#define IMCL_TEST_MODULE_API_H

/**
  @file imcl_test_module_api.h

  @brief defines the Intent IDs for communication over Inter-Module Control
  Links (IMCL) betweeen Test Modules
*/
/*==========================================================================
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 * =========================================================================*/
#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

// not defining in the public file since its not necessary to expose to clients
#define INTENT_ID_TEST_MODULE_INTERACTION    0x08001361
/*==============================================================================
   Constants
==============================================================================*/
#define MIN_INCOMING_IMCL_PARAM_SIZE_TEST_MODULE (sizeof(test_module_imcl_header_t) + sizeof(intf_extn_param_id_imcl_incoming_data_t))

/**< Header - Any IMCL message going out of / coming in to the
      Test Module (Voice Activation) will have the
      following header followed by the actual payload.
      The peers have to parse the header accordingly*/
typedef struct test_module_imcl_header_t
{
   // specific purpose understandable to the IMCL peers only
   uint32_t opcode;

   // Size (in bytes) for the payload specific to the intent.
   uint32_t actual_data_len;
} test_module_imcl_header_t;

/*==============================================================================
  Intent ID -  INTENT_ID_TEST_MODULE_INTERACTION
==============================================================================*/
/**< Intent defines the payload structure of the IMCL message.
Test modules support the following functionalities –
1. IMCL frame counter: The transmitting test module sends messages
   with a frame counter in it (number of frames processed)
*/


#define PARAM_ID_TEST_MODULE_IMCL_FRAME_COUNTER          0x08001362

/*==============================================================================
   Type Definitions
==============================================================================*/

/* Structure definition for Parameter */
typedef struct param_id_test_module_imcl_frame_counter_t param_id_test_module_imcl_frame_counter_t;

/** @h2xmlp_parameter   {"PARAM_ID_TEST_MODULE_IMCL_FRAME_COUNTER",
                         PARAM_ID_TEST_MODULE_IMCL_FRAME_COUNTER}
    @h2xmlp_description {Sends an IMCL message with a frame counter}
    @h2xmlp_toolPolicy  {Calibration} */

#include "spf_begin_pack.h"
struct param_id_test_module_imcl_frame_counter_t
{
   uint32_t          frame_count;
   /**< @h2xmle_description {frame counter}
        @h2xmle_range       {0..4294967295}
        @h2xmle_default     {0} */
}
#include "spf_end_pack.h"
;
#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* #ifndef IMCL_TEST_MODULE_API_H*/
