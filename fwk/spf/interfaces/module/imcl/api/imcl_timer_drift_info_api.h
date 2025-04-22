/**
 *  \file imcl_timer_drift_info_api.h
 *  
 *  \brief
 *     This file contains API's for rate matching module to query the timer drift for a given real time drift source
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef _IMCL_TIMER_DRIFT_INFO_API_H_
#define _IMCL_TIMER_DRIFT_INFO_API_H_

/*------------------------------------------------------------------------------
 *  Header Includes
 *----------------------------------------------------------------------------*/
#include "ar_error_codes.h"
#include "imcl_fwk_intent_api.h"

#ifdef INTENT_ID_TIMER_DRIFT_INFO

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/** -----------------------------------------------------------------------
 ** Type Declarations
 ** ----------------------------------------------------------------------- */

/* clang-format off */

/** IMCL struct for sharing the real time drift source's
    accumulated drift relative to local timer with rate
    matching modules */

typedef struct imcl_tdi_hdl_t imcl_tdi_hdl_t;

typedef struct imcl_tdi_acc_drift_t
{
   int64_t acc_drift_us;
   /**< Accumulated drift in micro seconds
        relative to local timer. Drift is
        estimated relative to local timer.
        +tive drift indicates timer is faster
        than real time source.
        -tive drift indiated timer is slower
        than */

   uint64_t time_stamp_us;
   /**< Timestamp in microseconds in local timer 
        domain at which the drift is updated */

}imcl_tdi_acc_drift_t;


/** Function pointer for reading the accumulated drift */
typedef ar_result_t (*imcl_tdi_get_acc_drift_fn_t) (imcl_tdi_hdl_t*, imcl_tdi_acc_drift_t*);

/** Pointer to below structure is shared with rate matching
 *  modules for reading the accumulated drift */
typedef struct imcl_tdi_hdl_t
{
   imcl_tdi_get_acc_drift_fn_t    get_drift_fn_ptr;
   /**< Pointer to function for reading the drift  */
  
} imcl_tdi_hdl_t;


/** This param is used by real time drift source to share the
    pointer to memory location where the accumulated drift
    relative to local timer is shared. This param is used by
    the rate matching module to read the accumulated drift  */

#define IMCL_PARAM_ID_TIMER_DRIFT_INFO            0x080010C3


typedef struct param_id_imcl_timer_drift_info param_id_imcl_timer_drift_info;

struct param_id_imcl_timer_drift_info
{
   imcl_tdi_hdl_t          *handle_ptr;
   /**< Handle to the shared object containing 
        the accumulated drift info for a real time
        drift source */
};

/** This param is used by drift source module to inform the
 *  resync events to the rate matching modules.
 *  Upon receiving this, rate matching modules should resync
 *  to the current accumulated drift.
 *  This param doesn't have a payload.*/
#define IMCL_PARAM_ID_TIMER_DRIFT_RESYNC           0x08001245

/** This structure is used for sending or receiving the drift
 *  info related parameter ID. Immediately following this
 *  header is the actual parameter ID payload of size
 *  specified in the header. */

typedef struct imcl_tdi_set_cfg_header_t
{
   uint32_t  param_id;
   /**< Parameter ID to configure */

   uint32_t  param_size;
   /**< Parameter Size in bytes. */

}imcl_tdi_set_cfg_header_t;

/* clang-format on */

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif // INTENT_ID_TIMER_DRIFT_INFO

#endif /* _IMCL_TIMER_DRIFT_INFO_API_H_ */
