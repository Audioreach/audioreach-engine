/**
 * \file gen_cntr_offload_utils.c
 * \brief
 *     This file contains utility functions for required for off-loading in MDF
 *  
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "gen_cntr_i.h"
#include "media_fmt_extn_api.h"


/* function to send the operating frame-size to the RD client */
ar_result_t gen_cntr_offload_send_opfs_event_to_rd_client(gen_cntr_t *me_ptr, gen_cntr_ext_out_port_t *ext_out_port_ptr)
{

   return AR_EOK;
}

/* function to send the operating frame-size to the WR client */
ar_result_t gen_cntr_send_operating_framesize_event_to_wr_shmem_client(gen_cntr_t *            me_ptr,
                                                                       gen_cntr_ext_in_port_t *ext_in_port_ptr)
{
   ar_result_t result = AR_EOK;

   return result;
}

/* function to send the operating frame-size to all the WR clients */
ar_result_t gen_cntr_offload_send_opfs_event_to_wr_client(gen_cntr_t *me_ptr)
{
   ar_result_t result = AR_EOK;

   return result;
}

/* function to handle the peer port property cmd handling from master to satellite.
 * i.e., from the OLC WR/RD CLient module to the RD/WR SHMEM EP modules
 */
ar_result_t gen_cntr_offload_process_peer_port_property_param(gen_cntr_t *  me_ptr,
                                                              spf_handle_t *handle_ptr,
                                                              int8_t *      param_data_ptr,
                                                              uint32_t      param_size)
{
   ar_result_t result = AR_EOK;

   return result;
}

/* function to process the data cmd peer port configuration
 * Internal EOS is sent as a data message from the OLC to the satellite,
 * rather than EOS command. */
ar_result_t gen_cntr_offload_process_data_cmd_port_property_cfg(gen_cntr_t *            me_ptr,
                                                                gen_cntr_ext_in_port_t *ext_in_port_ptr)
{
   ar_result_t result = AR_EOK;

   return result;
}

/* function to process the internal EOS at RD EP and propagate the data flow gap through peer port property
 * configuration*/
ar_result_t gen_cntr_offload_propagate_internal_eos_port_property_cfg(gen_cntr_t *             me_ptr,
                                                                      gen_cntr_ext_out_port_t *ext_out_port_ptr)
{
   ar_result_t result = AR_EOK;

   return result;
}

ar_result_t gen_cntr_offload_parse_inp_pcm_media_fmt_from_gpr_client(gen_cntr_t *            me_ptr,
                                                                     gen_cntr_ext_in_port_t *ext_in_port_ptr,
                                                                     media_format_t *        media_fmt_ptr,
                                                                     topo_media_fmt_t *      local_media_fmt_ptr)
{
   return AR_EUNSUPPORTED;
}

ar_result_t gen_cntr_offload_handle_set_cfg_to_wr_sh_mem_ep(gen_cntr_t *        me_ptr,
                                                            gen_cntr_module_t * gen_cntr_module_ptr,
                                                            uint32_t            param_id,
                                                            int8_t *            param_data_ptr,
                                                            uint32_t            param_size,
                                                            spf_cfg_data_type_t cfg_type)
{
   return AR_EUNSUPPORTED;
}

ar_result_t gen_cntr_offload_pack_write_data(gen_cntr_t *me_ptr, gen_cntr_ext_out_port_t *ext_out_port_ptr)
{
   return AR_EOK;
}

ar_result_t gen_cntr_offload_reg_evt_rd_sh_mem_ep(gen_cntr_t *       me_ptr,
                                                  gen_cntr_module_t *module_ptr,
                                                  topo_reg_event_t * event_cfg_payload_ptr,
                                                  bool_t             is_register)
{
   return AR_EOK;
}

ar_result_t gen_cntr_offload_handle_set_cfg_to_rd_sh_mem_ep(gen_cntr_t *        me_ptr,
                                                            gen_cntr_module_t * gen_cntr_module_ptr,
                                                            uint32_t            param_id,
                                                            int8_t *            param_data_ptr,
                                                            uint32_t            param_size,
                                                            spf_cfg_data_type_t cfg_type)
{
   return AR_EUNSUPPORTED;
}

ar_result_t gen_cntr_offload_reg_evt_wr_sh_mem_ep(gen_cntr_t *       me_ptr,
                                                  gen_cntr_module_t *module_ptr,
                                                  topo_reg_event_t * event_cfg_payload_ptr,
                                                  bool_t             is_register)
{
   return AR_EOK;
}
