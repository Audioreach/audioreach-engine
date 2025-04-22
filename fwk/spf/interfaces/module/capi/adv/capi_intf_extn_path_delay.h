#ifndef CAPI_INTF_EXTN_PATH_DELAY_H
#define CAPI_INTF_EXTN_PATH_DELAY_H

/**
 *   \file capi_intf_extn_path_delay.h
 *   \brief
 *        intf_extns related to the path delay
 *
 *    This file defines interface extensions that would help modules getting the path delays
 *
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/*------------------------------------------------------------------------------
 * Include Files
 *----------------------------------------------------------------------------*/
#include "capi_types.h"

/** @addtogroup capi_if_ext_path_delay
@{ */

/** Unique identifier of the Path Delay interface extension, which modules use
    to get defined path delays.

    This module supports the following event and parameter IDs:
    - #INTF_EXTN_EVENT_ID_REQUEST_PATH_DELAY
    - #INTF_EXTN_PARAM_ID_RESPONSE_PATH_DELAY
    - #INTF_EXTN_PARAM_ID_DESTROY_PATH_DELAY
*/
#define INTF_EXTN_PATH_DELAY 0x0A00102E

/** ID of the event to use with #CAPI_EVENT_DATA_TO_DSP_SERVICE to request a
    service to set up the path delay mechanism.

    @msgpayload{intf_extn_event_id_request_path_delay_t}
    @table{weak__intf__extn__event__id__request__path__delay__t} @newpage
 */
#define INTF_EXTN_EVENT_ID_REQUEST_PATH_DELAY 0x0A00102F

/* Payload for INTF_EXTN_EVENT_ID_REQUEST_PATH_DELAY */
typedef struct intf_extn_event_id_request_path_delay_t intf_extn_event_id_request_path_delay_t;

/** @weakgroup weak_intf_extn_event_id_request_path_delay_t
@{ */
struct intf_extn_event_id_request_path_delay_t
{
   uint32_t src_module_instance_id;
   /**< Identifies the module instance that is the source of the path. */

   uint32_t src_port_id;
   /**< Identifies the port that is the source of the path.

        If the ID is unknown, set this field to 0. */

   uint32_t dst_module_instance_id;
   /**< Identifies the module instance that is the destination of the path. */

   uint32_t dst_port_id;
   /**< Identifies the port that is the destination of the path.

        If the ID is unknown, set this field to 0. */
};
/** @} */ /* end_weakgroup weak_intf_extn_event_id_request_path_delay_t */

/** ID of the parameter used to set the path delay-related information.

    The containers sets this parameter on the module that raised
    #INTF_EXTN_EVENT_ID_REQUEST_PATH_DELAY.

    @msgpayload{intf_extn_path_delay_response_t}
    @table{weak__intf__extn__path__delay__response__t}
 */
#define INTF_EXTN_PARAM_ID_RESPONSE_PATH_DELAY 0x0A001030

typedef struct intf_extn_path_delay_response_t intf_extn_path_delay_response_t;

/** @weakgroup weak_intf_extn_path_delay_response_t
@{ */
struct intf_extn_path_delay_response_t
{
   uint32_t path_id;
   /**< Identifies the path assigned by the framework. */

   uint32_t src_module_instance_id;
   /**< Identifies the module instance that is the source of the path. */

   uint32_t src_port_id;
   /**< Identifies the port that is the source of the path. */

   uint32_t dst_module_instance_id;
   /**< Identifies the module instance that is the destination of the path. */

   uint32_t dst_port_id;
   /**< Identifies the port that is the destination of the path. */

   uint32_t num_delay_ptrs;
   /**< Number of delay pointers. */

   volatile uint32_t **delay_us_pptr;
   /**< Pointer to the array of pointers to the delay variable created
        by the Audio Processing Manager (APM). */
};
/** @} */ /* end_weakgroup weak_intf_extn_path_delay_response_t */

/** ID of the parameter used to clear the path delay-related information.

    The containers set this parameter on the module that received
    #INTF_EXTN_PARAM_ID_RESPONSE_PATH_DELAY.

    Even if the destroy function is not called, the modules are to clear
    everything in the end function.

    @msgpayload{intf_extn_path_delay_destroy_t}
    @table{weak__intf__extn__path__delay__destroy__t}
 */
#define INTF_EXTN_PARAM_ID_DESTROY_PATH_DELAY 0x0A001032

typedef struct intf_extn_path_delay_destroy_t intf_extn_path_delay_destroy_t;

/** @weakgroup weak_intf_extn_path_delay_destroy_t
@{ */
struct intf_extn_path_delay_destroy_t
{
   bool_t is_set;
   /**< Indicates whether the path delay is set.

        @valuesbul
        - 0 -- FALSE (unset the delay during close or destroy operations)
        - 1 -- TRUE (set the delay in response to
               #INTF_EXTN_EVENT_ID_REQUEST_PATH_DELAY) @tablebulletend */

   uint32_t path_id;
   /**< Identifies the path assigned by the framework. */

   uint32_t src_module_instance_id;
   /**< Identifies the module instance that is the source of the path. */

   uint32_t src_port_id;
   /**< Port ID of the source of the path. */
};
/** @} */ /* end_weakgroup weak_intf_extn_path_delay_destroy_t */

/** @} */ /* end_addtogroup capi_if_ext_path_delay */

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* CAPI_INTF_EXTN_PATH_DELAY_H*/
