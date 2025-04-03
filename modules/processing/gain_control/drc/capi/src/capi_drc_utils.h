/**
@file capi_drc_utils.h

@brief CAPI V2 utility header for DRC module.

*/

/*-----------------------------------------------------------------------
Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
SPDX-License-Identifier: BSD-3-Clause
-----------------------------------------------------------------------*/

#include "api_drc.h"
#include "capi_cmn.h"
#include "capi_cmn_ctrl_port_list.h"
#include "capi_drc.h"
#include "drc_api.h" //lib interface
#include "imcl_drc_info_api.h"

/*------------------------------------------------------------------------------
 * Defines
 *-----------------------------------------------------------------------------*/

#define MIID_UNKNOWN 0

#define DRC_MSG_PREFIX "DRC:[%lX] "
#define DRC_MSG(ID, xx_ss_mask, xx_fmt, ...) AR_MSG(xx_ss_mask, DRC_MSG_PREFIX xx_fmt, ID, ##__VA_ARGS__)

#define CHECK_THROW_ERROR(ID, result, error_msg, ...)                                                                  \
   {                                                                                                                   \
      if (CAPI_FAILED(result))                                                                                         \
      {                                                                                                                \
         DRC_MSG(ID, DBG_ERROR_PRIO, error_msg, ##__VA_ARGS__);                                                        \
         return result;                                                                                                \
      }                                                                                                                \
   }

/* KPPS values */
#define DRC_NB_KPPS (500)   // 0.5 mpps in kpps for nb
#define DRC_WB_KPPS (800)   // 0.8 mpps in kpps for wb
#define DRC_SWB_KPPS (2000) // 2 mpps in kpps for swb - approx
#define DRC_FB_KPPS (2730)  // 2.71 mpps in kpps for fb - approx.

/*------------------------------------------------------------------------------
 * Type Definition
 *-----------------------------------------------------------------------------*/

typedef struct capi_drc_t
{
   capi_vtbl_t *              vtbl;
   capi_event_callback_info_t cb_info;
   POSAL_HEAP_ID              heap_id;

   drc_lib_t                  lib_handle;     // Library instance
   drc_static_struct_t        lib_static_cfg; // static parameters structure
   drc_config_t               lib_cfg;        // Configuration structure
   drc_feature_mode_t         mode;           // operation mode
   drc_lib_mem_requirements_t mem_req;        // memory requirements structure
   int16_t                    b_enable;       // enable/disable lib
   uint32_t                   delay_us;       // drc delay in microseconds.
   uint32_t                   miid;           // Module Instance ID

   ctrl_port_list_handle_t ctrl_port_info;

   capi_media_fmt_v2_t input_media_fmt;
} capi_drc_t;

/*------------------------------------------------------------------------------
 * Function Declarations
 *-----------------------------------------------------------------------------*/
void       drc_lib_set_default_config(capi_drc_t *me_ptr);
capi_err_t drc_lib_set_calib(capi_drc_t *me_ptr);
capi_err_t drc_lib_alloc_init(capi_drc_t *me_ptr);

capi_err_t drc_lib_send_config_imcl(capi_drc_t *me_ptr);

void raise_kpps_delay_process_events(capi_drc_t *me_ptr);
