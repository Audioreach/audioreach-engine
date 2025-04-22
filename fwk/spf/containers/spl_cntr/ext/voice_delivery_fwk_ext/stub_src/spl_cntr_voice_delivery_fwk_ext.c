/**
 * \file spl_cntr_voice_delivery_fwk_ext.c
 *
 * \brief
 * This file contains stub utility functions for FWK_EXTN_VOICE_DELIVERY
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "gen_topo.h"
#include "gen_topo_capi.h"

/**
 * Helper function to send vptx proc start signal info to smart sync module for voice timer registration
 */
ar_result_t spl_cntr_fwk_extn_voice_delivery_send_proc_start_signal_info(spl_cntr_t *       me_ptr,
                                                                         spl_topo_module_t *module_ptr)
{
   return AR_EOK;
}

/* Function to check if nay module in the container supports FWK_EXTN_VOICE_DELIVERY*/
bool_t spl_cntr_fwk_extn_voice_delivery_found(spl_cntr_t *me_ptr, spl_topo_module_t **found_module_pptr)
{
   return FALSE;
}

bool_t spl_cntr_fwk_extn_voice_delivery_need_drop_data_msg(spl_cntr_t *me_ptr, spl_cntr_ext_in_port_t *ext_in_port_ptr)
{
   return FALSE;
}


/**
 * Called during graph_close for voice delivery related cleanup.
 */
ar_result_t spl_cntr_fwk_extn_voice_delivery_close(spl_cntr_t *me_ptr)
{
   return AR_EOK;
}

ar_result_t spl_cntr_fwk_extn_voice_delivery_handle_timestamp(spl_cntr_t *            me_ptr,
                                                              spl_cntr_ext_in_port_t *ext_in_port_ptr,
                                                              spf_msg_data_buffer_t * data_buf_ptr)
{
   return AR_EOK;
}

ar_result_t spl_cntr_fwk_extn_voice_delivery_push_ts_zeros(spl_cntr_t *            me_ptr,
                                                           spl_cntr_ext_in_port_t *ext_in_port_ptr,
                                                           uint32_t *              data_needed_bytes_per_ch_ptr)
{
   return AR_EOK;
}

ar_result_t spl_cntr_fwk_extn_voice_delivery_handle_topo_proc_notif(spl_cntr_t *me_ptr)
{
  return AR_EOK;
}
