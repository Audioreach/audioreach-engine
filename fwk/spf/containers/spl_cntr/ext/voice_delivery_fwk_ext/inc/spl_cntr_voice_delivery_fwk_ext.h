#ifndef SPL_CNTR_VOICE_DELIVERY_FWK_H
#define SPL_CNTR_VOICE_DELIVERY_FWK_H
/**
 * \file spl_cntr_sync_fwk_ext.h
 * \brief
 *  This file contains utility functions for FWK_EXTN_VOICE_DELIVERY
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */
// clang-format off

#include "spl_cntr_i.h"
#include "spl_topo.h"
// clang-format on

#if defined(__cplusplus)
extern "C" {
#endif // __cplusplus

typedef struct spl_cntr_t spl_cntr_t;

ar_result_t spl_cntr_fwk_extn_voice_delivery_send_proc_start_signal_info(spl_cntr_t *       me_ptr,
                                                                         spl_topo_module_t *module_ptr);

ar_result_t spl_cntr_fwk_extn_voice_delivery_handle_topo_proc_notif(spl_cntr_t *me_ptr);

ar_result_t spl_cntr_fwk_extn_voice_delivery_close(spl_cntr_t *me_ptr);

bool_t spl_cntr_fwk_extn_voice_delivery_found(spl_cntr_t *me_ptr, spl_topo_module_t **found_module_pptr);

bool_t spl_cntr_fwk_extn_voice_delivery_need_drop_data_msg(spl_cntr_t *me_ptr, spl_cntr_ext_in_port_t *ext_in_port_ptr);

ar_result_t spl_cntr_fwk_extn_voice_delivery_handle_timestamp(spl_cntr_t *            me_ptr,
                                                              spl_cntr_ext_in_port_t *ext_in_port_ptr,
                                                              spf_msg_data_buffer_t * data_buf_ptr);

ar_result_t spl_cntr_fwk_extn_voice_delivery_push_ts_zeros(spl_cntr_t *            me_ptr,
                                                           spl_cntr_ext_in_port_t *ext_in_port_ptr,
                                                           uint32_t *              data_needed_bytes_per_ch_ptr);


#if defined(__cplusplus)
}
#endif // __cplusplus

#endif /* SPL_CNTR_VOICE_DELIVERY_FWK_H */
