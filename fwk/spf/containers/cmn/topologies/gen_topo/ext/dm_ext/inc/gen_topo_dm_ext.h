#ifndef GEN_TOPO_DM_EXTN_H
#define GEN_TOPO_DM_EXTN_H
/**
 * \file gen_topo_dm_extn.h
 * \brief
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */


#include "topo_utils.h"
#include "ar_defs.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

typedef struct gen_topo_t               gen_topo_t;
typedef struct gen_topo_input_port_t    gen_topo_input_port_t;
typedef struct gen_topo_output_port_t   gen_topo_output_port_t;
typedef struct gen_topo_module_t        gen_topo_module_t;

/* Max extra buffer needed for DM modules variable input/output path*/
#define GEN_TOPO_DM_BUFFER_MAX_EXTRA_LEN_US 1000

// DM mode is only a 2 bit value, since its stored as module flag.
// refer: gen_topo_module_flags_t.dm_mode
// Each mode is directly mapped to a dm mode as defined in capi_fwk_extns_dm.h
typedef enum
{
   GEN_TOPO_DM_INVALID_MODE = 0x0,
   /**< Invalid value */
   GEN_TOPO_DM_FIXED_INPUT_MODE = 0x1,
   /**< Module runs in fixed input mode */
   GEN_TOPO_DM_FIXED_OUTPUT_MODE = 0x2,
   /**< Module runs in fixed output mode */
}gen_topo_dm_mode_t;

bool_t gen_topo_is_in_port_in_dm_variable_nblc(gen_topo_t *topo_ptr, gen_topo_input_port_t *in_port_ptr);

bool_t gen_topo_is_out_port_in_dm_variable_nblc(gen_topo_t *topo_ptr, gen_topo_output_port_t *out_port_ptr);

uint32_t gen_topo_compute_if_input_needs_addtional_bytes_for_dm(gen_topo_t *topo_ptr, gen_topo_input_port_t *in_port_ptr);

uint32_t gen_topo_compute_if_output_needs_addtional_bytes_for_dm(gen_topo_t *            topo_ptr,
                                                                 gen_topo_output_port_t *out_port_ptr);

ar_result_t gen_topo_update_dm_modes(gen_topo_t *topo_ptr);

ar_result_t gen_topo_updated_expected_samples_for_dm_modules(gen_topo_t *topo_ptr, gen_topo_module_t *module_ptr);

// handle DM mode change event raised by the module.
ar_result_t gen_topo_handle_dm_disable_event(gen_topo_t *                      topo_ptr,
                                             gen_topo_module_t *               module_ptr,
                                             capi_event_data_to_dsp_service_t *event_data_ptr);

ar_result_t gen_topo_determine_dm_mode_of_module(gen_topo_t *        topo_ptr,
                                                 gen_topo_module_t * module_ptr,
                                                 fwk_extn_dm_mode_t *dm_mode);

ar_result_t gen_topo_send_dm_consume_partial_input(gen_topo_t *topo_ptr, gen_topo_module_t* module_ptr, bool_t should_consume_partial_input);
                                   
#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef GEN_TOPO_DM_EXTN_H
