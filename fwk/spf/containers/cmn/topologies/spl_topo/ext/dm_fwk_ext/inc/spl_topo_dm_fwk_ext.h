#ifndef SPL_TOPO_DM_FWK_H
#define SPL_TOPO_DM_FWK_H
/**
 * \file spl_topo_dm_fwk_ext.h
 * \brief
 *     This file contains function definitions for FWK_EXTN_DM

 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "topo_utils.h"
#include "ar_defs.h"

#if defined(__cplusplus)
extern "C" {
#endif // __cplusplus

typedef struct spl_topo_t        spl_topo_t;
typedef struct spl_topo_module_t spl_topo_module_t;

/**< Type of DM modules present in spl topo */
typedef enum spl_topo_present_dm_modes_t {
   SPL_TOPO_NO_DM_MODULE           = (0),
   SPL_TOPO_FIXED_INPUT_DM_MODULE  = (1),
   SPL_TOPO_FIXED_OUTPUT_DM_MODULE = (1 << 1)
} spl_topo_present_dm_modes_t;

typedef struct spl_topo_dm_info_t
{
   spl_topo_present_dm_modes_t         present_dm_modules;
   uint32_t                            num_ports_dm; // indicates how many ports req_samples_ptr is allocated for
   fwk_extn_dm_param_id_req_samples_t *dm_req_samples_ptr;
} spl_topo_dm_info_t;


ar_result_t spl_topo_fwk_ext_update_dm_modes(spl_topo_t *topo_ptr);
ar_result_t spl_topo_fwk_ext_handle_dm_report_samples_event(spl_topo_t *                      topo_ptr,
                                                    spl_topo_module_t *               module_ptr,
                                                    capi_event_data_to_dsp_service_t *event_data_ptr,
                                                    bool_t                            is_max);
ar_result_t spl_topo_fwk_ext_handle_dm_disable_event(spl_topo_t *                      topo_ptr,
                                             spl_topo_module_t *               module_ptr,
                                             capi_event_data_to_dsp_service_t *event_data_ptr);
void spl_topo_fwk_ext_free_dm_req_samples(spl_topo_t *topo_ptr);
ar_result_t spl_topo_fwk_ext_set_dm_samples_per_module(spl_topo_t *                        topo_ptr,
                                               spl_topo_module_t *                 module_ptr,
                                               fwk_extn_dm_param_id_req_samples_t *dm_req_samples_ptr,
                                               bool_t                              is_max);

/* Checks if a particular module has enabled/disabled dm mode */
bool_t spl_topo_fwk_ext_is_dm_enabled(spl_topo_module_t *module_ptr);

/* Checks if a particular module is in fixed out dm mode */
bool_t spl_topo_fwk_ext_is_module_dm_fixed_out(spl_topo_module_t *module_ptr);

/* Checks if a particular module is in fixed in dm mode */
bool_t spl_topo_fwk_ext_is_module_dm_fixed_in(spl_topo_module_t *module_ptr);

/* Checks if any module operating in dm mode is present in the topo */
static inline bool_t spl_topo_fwk_ext_any_dm_module_present(spl_topo_dm_info_t dm_info)
{
   return (dm_info.present_dm_modules != SPL_TOPO_NO_DM_MODULE);
}

/* Checks if any module operating in dm mode of fixed input is present in the topo */
static inline bool_t spl_topo_fwk_ext_is_fixed_in_dm_module_present(spl_topo_dm_info_t dm_info)
{
   return (dm_info.present_dm_modules & SPL_TOPO_FIXED_INPUT_DM_MODULE) ? TRUE : FALSE;
}

/* Checks if any module operating in dm mode of fixed output is present in the topo */
static inline bool_t spl_topo_fwk_ext_is_fixed_out_dm_module_present(spl_topo_dm_info_t dm_info)
{
   return (dm_info.present_dm_modules & SPL_TOPO_FIXED_OUTPUT_DM_MODULE) ? TRUE : FALSE;
}

#if defined(__cplusplus)
}
#endif // __cplusplus

#endif /* SPL_TOPO_DM_FWK_H */
