/*==============================================================================
 @file capi_rat.c
 This file contains capi functions for mimo and siso Rate Adapted Timer Endpoint module

 ================================================================================
 Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 SPDX-License-Identifier: BSD-3-Clause-Clear
 ==============================================================================*/
// clang-format off
// clang-format on
/*=====================================================================
 Includes
 ======================================================================*/

#include "capi_rat_i.h"

/*------------------------------------------------------------------------
 * Static declarations
 * -----------------------------------------------------------------------*/
static capi_err_t capi_rat_init(capi_t *_pif, capi_proplist_t *init_set_properties, capi_rat_type_t type);

static capi_err_t capi_rat_process_get_properties(capi_rat_t *     me_ptr,
                                                  capi_proplist_t *proplist_ptr,
                                                  capi_rat_type_t  type);

static capi_err_t capi_rat_process_set_properties(capi_rat_t *     me_ptr,
                                                  capi_proplist_t *proplist_ptr,
                                                  capi_rat_type_t  type);

/*------------------------------------------------------------------------
 Function name: capi_rat_get_static_properties
 DESCRIPTION: Function to get the static properties of mimo rat module
 -----------------------------------------------------------------------*/
capi_err_t capi_mimo_rat_get_static_properties(capi_proplist_t *init_set_properties, capi_proplist_t *static_properties)
{
   return capi_rat_process_get_properties((capi_rat_t *)NULL, static_properties, CAPI_MIMO_RAT);
}

/*------------------------------------------------------------------------
 Function name: capi_rat_init
 DESCRIPTION: Initialize the mimo RAT module and library.
 -----------------------------------------------------------------------*/
capi_err_t capi_mimo_rat_init(capi_t *_pif, capi_proplist_t *init_set_properties)
{
   return capi_rat_init(_pif, init_set_properties, CAPI_MIMO_RAT);
}

/*------------------------------------------------------------------------
 Function name: capi_rat_get_static_properties
 DESCRIPTION: Function to get the static properties of siso rat module
 -----------------------------------------------------------------------*/
capi_err_t capi_siso_rat_get_static_properties(capi_proplist_t *init_set_properties, capi_proplist_t *static_properties)
{
   return capi_rat_process_get_properties((capi_rat_t *)NULL, static_properties, CAPI_SISO_RAT);
}

/*------------------------------------------------------------------------
 Function name: capi_siso rat_init
 DESCRIPTION: Initialize the siso RAT module and library.
 -----------------------------------------------------------------------*/
capi_err_t capi_siso_rat_init(capi_t *_pif, capi_proplist_t *init_set_properties)
{
   return capi_rat_init(_pif, init_set_properties, CAPI_SISO_RAT);
}

/*------------------------------------------------------------------------
 Function name: capi_rat_init
 DESCRIPTION: Initialize the CAPI RAT module and library.
 This function can allocate memory.
 -----------------------------------------------------------------------*/
static capi_err_t capi_rat_init(capi_t *_pif, capi_proplist_t *init_set_properties, capi_rat_type_t type)
{
   capi_err_t capi_result = CAPI_EOK;

   if (NULL == _pif || NULL == init_set_properties)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "CAPI_RAT: Init received bad pointer, 0x%p, 0x%p",
             (uintptr_t)_pif,
             (uintptr_t)init_set_properties);
      return CAPI_EBADPARAM;
   }

   capi_rat_t *me_ptr = (capi_rat_t *)_pif;
   memset((void *)me_ptr, 0, sizeof(capi_rat_t));

   // Allocate vtbl and call driver init
   me_ptr->vtbl.vtbl_ptr = capi_rat_get_vtbl();

   me_ptr->type = type;

   // Set drift function for other modules to read drift
   capi_result = capi_rat_init_out_drift_info(&me_ptr->rat_out_drift_info, rat_read_acc_out_drift);
   if (CAPI_EOK != capi_result)
   {
      AR_MSG(DBG_ERROR_PRIO, "CAPI_RAT: Failed to initilize drift read function");
      return capi_result;
   }

   capi_result |= capi_cmn_init_media_fmt_v2(&(me_ptr->configured_media_fmt));
   me_ptr->configured_media_fmt.format.data_interleaving = CAPI_DEINTERLEAVED_UNPACKED_V2;

   capi_result = capi_rat_process_set_properties(me_ptr, init_set_properties, type);

   capi_result |= capi_cmn_raise_deinterleaved_unpacked_v2_supported_event(&me_ptr->cb_info);

   return capi_result;
}

/*------------------------------------------------------------------------
 Function name: capi_rat_end
 DESCRIPTION: Returns the library to the uninitialized state and frees the
 memory that was allocated by init(). This function also frees the virtual
 function table.
 -----------------------------------------------------------------------*/
capi_err_t capi_rat_end(capi_t *_pif)
{
   capi_err_t capi_result = CAPI_EOK;

   if (NULL == _pif)
   {
      AR_MSG(DBG_ERROR_PRIO, "CAPI_RAT: Capi end received bad pointer, 0x%p", (uintptr_t)_pif);
      return CAPI_EBADPARAM;
   }
   capi_rat_t *me_ptr = (capi_rat_t *)_pif;

   capi_result |= capi_rat_deinit_out_drift_info(&me_ptr->rat_out_drift_info);
   capi_result |= capi_rat_deinit_out_control_ports(&me_ptr->out_ctrl_port_info);

   if (me_ptr->out_port_info_ptr)
   {
      posal_memory_free(me_ptr->out_port_info_ptr);
      me_ptr->out_port_info_ptr = NULL;
   }

   if (me_ptr->in_port_info_ptr)
   {
      posal_memory_free(me_ptr->in_port_info_ptr);
      me_ptr->in_port_info_ptr = NULL;
   }

   // Set the signal ptr to NULL will be freed from fwk
   me_ptr->signal_ptr = NULL;

   me_ptr->vtbl.vtbl_ptr = NULL;
   return capi_result;
}

/*------------------------------------------------------------------------
 Function name: capi_rat_set_param
 DESCRIPTION: Sets either a parameter value or a parameter structure containing
 multiple parameters. In the event of a failure, the appropriate error code is
 returned.
 -----------------------------------------------------------------------*/
capi_err_t capi_rat_set_param(capi_t *                _pif,
                              uint32_t                param_id,
                              const capi_port_info_t *port_info_ptr,
                              capi_buf_t *            params_ptr)
{
   capi_err_t capi_result = CAPI_EOK;

   if ((NULL == _pif) || (NULL == params_ptr) || (NULL == params_ptr->data_ptr))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "CAPI_RAT: Set param received bad pointer, _pif 0x%p, params_ptr 0x%p",
             (uintptr_t)_pif,
             (uintptr_t)params_ptr);
      return CAPI_EBADPARAM;
   }

   capi_rat_t *me_ptr     = (capi_rat_t *)_pif;
   uint32_t    param_size = params_ptr->actual_data_len;

   RAT_MSG(me_ptr->iid, DBG_HIGH_PRIO, "CAPI_RAT: Set param received id 0x%lx", (uint32_t)param_id);

   switch (param_id)
   {
      case FWK_EXTN_PARAM_ID_THRESHOLD_CFG:
      {
         if (param_size < sizeof(fwk_extn_param_id_threshold_cfg_t))
         {
            RAT_MSG(me_ptr->iid,
                    DBG_ERROR_PRIO,
                    "CAPI_RAT: Param id 0x%lx Bad param size %lu",
                    (uint32_t)param_id,
                    param_size);
            capi_result |= CAPI_ENEEDMORE;
            break;
         }
         fwk_extn_param_id_threshold_cfg_t *fm_dur = (fwk_extn_param_id_threshold_cfg_t *)params_ptr->data_ptr;
         me_ptr->sg_frame_dur_us                   = fm_dur->duration_us;

         // update the frame duration based on the timer duration config
         if (CAPI_EOK != (capi_result = capi_rat_update_frame_duration(me_ptr)))
         {
            return capi_result;
         }

#ifdef RAT_DEBUG
         RAT_MSG(me_ptr->iid,
                 DBG_HIGH_PRIO,
                 "CAPI_RAT: Frame duration of RAT configured to %lu us",

                 fm_dur->duration_us);
#endif
         break;
      }
      case PARAM_ID_DATA_INTERLEAVING:
      {
         if (param_size < sizeof(param_id_module_data_interleaving_t))
         {
            AR_MSG(DBG_ERROR_PRIO,
                  "CAPI_RAT: SetParam 0x%lx, invalid param size %lx ",
                  param_id,
                  params_ptr->actual_data_len);
            capi_result = CAPI_ENEEDMORE;
            break;
         }
         param_id_module_data_interleaving_t *rat_data_int_ptr = (param_id_module_data_interleaving_t *)params_ptr->data_ptr;

         if ((PCM_DEINTERLEAVED_UNPACKED != rat_data_int_ptr->data_interleaving) &&
             (PCM_INTERLEAVED != rat_data_int_ptr->data_interleaving))
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "CAPI_RAT: Interleaving value [%lu] not supported. ",
                   rat_data_int_ptr->data_interleaving);
            capi_result = CAPI_EBADPARAM;
            break;
         }

         pcm_to_capi_interleaved_with_native_param(&me_ptr->configured_media_fmt.format.data_interleaving,
                                                   rat_data_int_ptr->data_interleaving,
                                                   CAPI_INVALID_INTERLEAVING);

         uint32_t static_out_port_index =
            capi_rat_get_port_idx_from_port_id(me_ptr, PORT_ID_RATE_ADAPTED_TIMER_OUTPUT, FALSE /*is_input*/);

         if(me_ptr->configured_media_fmt_received && (RAT_PORT_INDEX_INVALID != static_out_port_index))
         {
            // Raise out mf event
            RAT_MSG(me_ptr->iid,
                    DBG_HIGH_PRIO,
                    "CAPI_RAT: Raising out media fmt on static output port id 0x%lx",
                    PORT_ID_RATE_ADAPTED_TIMER_OUTPUT);

            static_out_port_index = RAT_PORT_INDEX_INVALID;
            static_out_port_index =
               capi_rat_get_port_idx_from_port_id(me_ptr, PORT_ID_RATE_ADAPTED_TIMER_OUTPUT, FALSE /*is_input*/);

            capi_result = capi_cmn_output_media_fmt_event_v2(&me_ptr->cb_info,
                                                            &me_ptr->configured_media_fmt,
                                                            FALSE /*is_input*/,
                                                            static_out_port_index);
         }
         AR_MSG(DBG_LOW_PRIO, "CAPI_RAT: Data interleaving set to %d.", me_ptr->configured_media_fmt.format.data_interleaving);
         break;
      }
      case PARAM_ID_RAT_MEDIA_FORMAT:
      {
         /* Can allow overriding of this media fmt as long as timer hasn't started.
          * We need to allow new MF for scenarios where PLACEHOLDER_MODULE_RESET is sent.
          * It will be sent at STOP at which time the timer will be disabled  */
         if (me_ptr->is_timer_enable)
         {
            RAT_MSG(me_ptr->iid,
                    DBG_ERROR_PRIO,
                    "CAPI_RAT: Received param id 0x%lx, timer already started, media format can only be changed at "
                    "STOP ",
                    (uint32_t)param_id);
            return CAPI_EFAILED;
         }

         // We don't error out if static output port is not opened because RAT can behave as sink module as well
         uint32_t static_out_port_index = RAT_PORT_INDEX_INVALID, static_in_port_index = RAT_PORT_INDEX_INVALID;
         bool_t   raise_out_mf = TRUE;

         static_out_port_index =
            capi_rat_get_port_idx_from_port_id(me_ptr, PORT_ID_RATE_ADAPTED_TIMER_OUTPUT, FALSE /*is_input*/);
         static_in_port_index =
            capi_rat_get_port_idx_from_port_id(me_ptr, PORT_ID_RATE_ADAPTED_TIMER_INPUT, TRUE /*is_input*/);

         if (RAT_PORT_INDEX_INVALID != static_out_port_index)
         {
            if (port_info_ptr->port_index != static_out_port_index)
            {
               RAT_MSG(me_ptr->iid,
                       DBG_ERROR_PRIO,
                       "CAPI_RAT: Received static mf cfg param on wrong port index %d, static port index %d",
                       port_info_ptr->port_index,
                       static_out_port_index);
               return CAPI_EBADPARAM;
            }
         }
         else
         {
            RAT_MSG(me_ptr->iid,
                    DBG_HIGH_PRIO,
                    "CAPI_RAT: Received static mf cfg param, but dont raise out mf, output port not found/connected");
            raise_out_mf = FALSE;
         }

         if (param_size < sizeof(param_id_rat_mf_t))
         {
            RAT_MSG(me_ptr->iid,
                    DBG_ERROR_PRIO,
                    "CAPI_RAT: Set param 0x%lx, invalid param size %lx ",
                    param_id,
                    param_size);
            return CAPI_ENEEDMORE;
         }

         param_id_rat_mf_t *rat_mf_cfg_ptr = (param_id_rat_mf_t *)params_ptr->data_ptr;

         if ((0 >= rat_mf_cfg_ptr->num_channels) || (CAPI_MAX_CHANNELS_V2 < rat_mf_cfg_ptr->num_channels))
         {
            RAT_MSG(me_ptr->iid, DBG_ERROR_PRIO, "CAPI_RAT: Unsupported num channels %d. Max supported channels: %d",
			        rat_mf_cfg_ptr->num_channels, CAPI_MAX_CHANNELS_V2);
            return CAPI_EBADPARAM;
         }

         uint32_t required_size = rat_mf_cfg_ptr->num_channels * sizeof(rat_mf_cfg_ptr->channel_map[0]);

         // Validate the size of payload
         if (param_size < required_size)
         {
            RAT_MSG(me_ptr->iid,
                    DBG_ERROR_PRIO,
                    "CAPI_RAT: Not valid media format size %d, required size %d",
                    param_size,
                    required_size);
            return CAPI_ENEEDMORE;
         }

         for (uint32_t i = 0; i < rat_mf_cfg_ptr->num_channels; i++)
         {
            if ((rat_mf_cfg_ptr->channel_map[i] < (uint16_t)PCM_CHANNEL_L) ||
                (rat_mf_cfg_ptr->channel_map[i] > (uint16_t)PCM_MAX_CHANNEL_MAP_V2))
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "Unsupported channel type channel idx %d, channel type %d received",
                      (int)i,
                      (int)rat_mf_cfg_ptr->channel_map[i]);
               return CAPI_EBADPARAM;
            }
         }

         if ((0 >= rat_mf_cfg_ptr->sample_rate) || (384000 < rat_mf_cfg_ptr->sample_rate))
         {
            RAT_MSG(me_ptr->iid,
                    DBG_ERROR_PRIO,
                    "CAPI_RAT: Unsupported sampling rate %d received",
                    rat_mf_cfg_ptr->sample_rate);
            return CAPI_EBADPARAM;
         }

         if ((16 != rat_mf_cfg_ptr->bits_per_sample) && (32 != rat_mf_cfg_ptr->bits_per_sample))
         {
            RAT_MSG(me_ptr->iid,
                    DBG_ERROR_PRIO,
                    "CAPI_RAT: Unsupported bits per sample %d received",
                    rat_mf_cfg_ptr->bits_per_sample);
            return CAPI_EBADPARAM;
         }

         if (((16 == rat_mf_cfg_ptr->bits_per_sample) && (15 != rat_mf_cfg_ptr->q_factor)) ||
             ((32 == rat_mf_cfg_ptr->bits_per_sample) &&
              ((27 != rat_mf_cfg_ptr->q_factor) && (31 != rat_mf_cfg_ptr->q_factor))))
         {
            RAT_MSG(me_ptr->iid,
                    DBG_ERROR_PRIO,
                    "CAPI_RAT: Unsupported bits per sample %d qfactor combo %d received",
                    rat_mf_cfg_ptr->bits_per_sample,
                    rat_mf_cfg_ptr->q_factor);
            return CAPI_EBADPARAM;
         }

         if (DATA_FORMAT_FIXED_POINT != rat_mf_cfg_ptr->data_format)
         {
            RAT_MSG(me_ptr->iid,
                    DBG_ERROR_PRIO,
                    "CAPI_RAT: Unsupported data format %d received",
                    rat_mf_cfg_ptr->data_format);
            return CAPI_EBADPARAM;
         }

         RAT_MSG(me_ptr->iid,
                 DBG_HIGH_PRIO,
                 "CAPI_RAT: Received static port rat media fmt with sr %d, num ch %d, bps %d, data fmt %d, qfac %d",
                 rat_mf_cfg_ptr->sample_rate,
                 rat_mf_cfg_ptr->num_channels,
                 rat_mf_cfg_ptr->bits_per_sample,
                 rat_mf_cfg_ptr->data_format,
                 rat_mf_cfg_ptr->q_factor);

         // cache the media fmt
         capi_result = capi_cmn_data_fmt_map((uint32_t *)&rat_mf_cfg_ptr->data_format, &me_ptr->configured_media_fmt);
         if (CAPI_EOK != capi_result)
         {
            return capi_result;
         }

         me_ptr->configured_media_fmt.format.sampling_rate   = rat_mf_cfg_ptr->sample_rate;
         me_ptr->configured_media_fmt.format.num_channels    = rat_mf_cfg_ptr->num_channels;
         me_ptr->configured_media_fmt.format.bits_per_sample = rat_mf_cfg_ptr->bits_per_sample;
         me_ptr->configured_media_fmt.format.q_factor        = rat_mf_cfg_ptr->q_factor;

         for (uint32_t i = 0; i < me_ptr->configured_media_fmt.format.num_channels; i++)
         {
            me_ptr->configured_media_fmt.format.channel_type[i] = rat_mf_cfg_ptr->channel_map[i];
         }

         // Set defaults for other fields
         me_ptr->configured_media_fmt.format.data_interleaving = CAPI_DEINTERLEAVED_UNPACKED_V2;
         me_ptr->configured_media_fmt.format.bitstream_format  = MEDIA_FMT_ID_PCM;
         me_ptr->configured_media_fmt.format.data_is_signed    = TRUE;
         me_ptr->configured_media_fmt.format.minor_version     = CAPI_MEDIA_FORMAT_MINOR_VERSION;

         // set flag
         me_ptr->configured_media_fmt_received = TRUE;

         // Update the frame duration based on the timer duration configuration
         // Update the frame duration based on the configured media format and
         // check the sanity of frame duration payload.
         // If the sanity fails, it would effectively reject the configured media format
         if (CAPI_EOK != (capi_result = capi_rat_update_frame_duration(me_ptr)))
         {
            return capi_result;
         }

         if (raise_out_mf)
         {
            // Raise out mf event
            RAT_MSG(me_ptr->iid,
                    DBG_HIGH_PRIO,
                    "CAPI_RAT: Raising out media fmt on static output port id 0x%lx",
                    PORT_ID_RATE_ADAPTED_TIMER_OUTPUT);

            capi_result = capi_cmn_output_media_fmt_event_v2(&me_ptr->cb_info,
                                                             &me_ptr->configured_media_fmt,
                                                             FALSE /*is_input*/,
                                                             static_out_port_index /*out index*/);
         }

         // Saving this variable to be used later in process to avoid recalc
         me_ptr->integ_sr_us = (uint64_t)me_ptr->frame_size_in_samples * NUM_MS_PER_SEC * NUM_MS_PER_SEC;

         // raise threshold per port, if any port index is invalid, function will handle
         capi_result |= capi_rat_raise_thresh_event(me_ptr,
                                                    &me_ptr->configured_media_fmt,
                                                    static_in_port_index /*in_index*/,
                                                    static_out_port_index /*out_index*/);

         // Raise these events based on op mf
         capi_result |= capi_rat_raise_kpps_bw_events(me_ptr);

         break;
      }
      case PARAM_ID_RAT_TIMER_DURATION_CONFIG:
      {

         // timer duration configuration can be updated only in STOPPED state
         if (me_ptr->is_timer_enable)
         {
            RAT_MSG(me_ptr->iid,
                    DBG_ERROR_PRIO,
                    "CAPI_RAT: Received param id 0x%lx, timer already started, timer configuration can only be changed "
                    "during STOP",
                    (uint32_t)param_id);
            return CAPI_EFAILED;
         }

         // check if the payload size is correct
         if (param_size < sizeof(param_id_rat_timer_duration_config_t))
         {
            RAT_MSG(me_ptr->iid,
                    DBG_ERROR_PRIO,
                    "CAPI_RAT: Set param 0x%lx, invalid param size %lx ",
                    param_id,
                    param_size);
            return CAPI_ENEEDMORE;
         }

         param_id_rat_timer_duration_config_t *rat_timer_duration_cfg_ptr =
            (param_id_rat_timer_duration_config_t *)params_ptr->data_ptr;

         // Validate the configuration details.
         if (CAPI_EOK != (capi_result = capi_rat_validate_timer_duration_cfg(me_ptr, rat_timer_duration_cfg_ptr)))
         {
            return capi_result;
         }

         // Cache the payload internally
         memscpy((void *)&me_ptr->rat_timer_duration_config,
                 sizeof(param_id_rat_timer_duration_config_t),
                 (void *)params_ptr->data_ptr,
                 params_ptr->actual_data_len);

         // Update the frame duration based on the time duration configuration
         if (CAPI_EOK != (capi_result = capi_rat_update_frame_duration(me_ptr)))
         {
            return capi_result;
         }

         // if Media format for output is configured, then update the module thresholds.
         if (me_ptr->configured_media_fmt_received)
         {

            // Saving this variable to be used later in process to avoid recalc
            me_ptr->integ_sr_us = (uint64_t)me_ptr->frame_size_in_samples * NUM_MS_PER_SEC * NUM_MS_PER_SEC;

            // Get the static input and output index.
            uint32_t             static_out_port_index = 0, static_in_port_index = 0;
            capi_rat_out_port_t *static_out_port_ptr =
               capi_rat_get_out_port_from_port_id(me_ptr, PORT_ID_RATE_ADAPTED_TIMER_OUTPUT);

            if ((me_ptr->num_out_ports > 0) && (NULL != static_out_port_ptr))
            {

               static_out_port_index = static_out_port_ptr->cmn.self_index;
               static_in_port_index  = static_out_port_ptr->cmn.conn_index;

               // raise threshold per port
               capi_result |= capi_rat_raise_thresh_event(me_ptr,
                                                          &me_ptr->configured_media_fmt,
                                                          static_in_port_index /*in_index*/,
                                                          static_out_port_index /*out_index*/);
            }
         }

         break;
      }
      case INTF_EXTN_PARAM_ID_METADATA_HANDLER:
      {
         if (params_ptr->actual_data_len < sizeof(intf_extn_param_id_metadata_handler_t))
         {
            RAT_MSG(me_ptr->iid,
                    DBG_ERROR_PRIO,
                    "CAPI_RAT: Set param 0x%lx, invalid param size %lx ",
                    param_id,
                    param_size);
            capi_result = CAPI_ENEEDMORE;
            break;
         }
         intf_extn_param_id_metadata_handler_t *payload_ptr =
            (intf_extn_param_id_metadata_handler_t *)params_ptr->data_ptr;
         me_ptr->metadata_handler = *payload_ptr;
         break;
      }
      case INTF_EXTN_PARAM_ID_IMCL_PORT_OPERATION:
      {
         /** Set the control port operation */
         if (CAPI_EOK != (capi_result = capi_rat_imcl_port_operation(me_ptr, port_info_ptr, params_ptr)))
         {
            RAT_MSG(me_ptr->iid,
                    DBG_ERROR_PRIO,
                    "CAPI_RAT :Failed to set control port operation with result %d",
                    capi_result);
         }
         break;
      }
      case INTF_EXTN_PARAM_ID_IMCL_INCOMING_DATA:
      {
         if (param_size < sizeof(intf_extn_param_id_imcl_incoming_data_t) + sizeof(imcl_tdi_set_cfg_header_t))
         {
            RAT_MSG(me_ptr->iid,
                    DBG_ERROR_PRIO,
                    "CAPI_RAT: Invalid payload size for param %x, size %d",
                    param_id,
                    params_ptr->actual_data_len);
            return CAPI_ENEEDMORE;
         }

         intf_extn_param_id_imcl_incoming_data_t *payload_ptr =
            (intf_extn_param_id_imcl_incoming_data_t *)params_ptr->data_ptr;

         /* Retrieve the control port based on port id*/
         rat_inp_ctrl_port_info_t *port_ptr = capi_rat_get_inp_ctrl_port_info_ptr(me_ptr, payload_ptr->port_id);
         if (NULL == port_ptr)
         {
            RAT_MSG(me_ptr->iid,
                    DBG_ERROR_PRIO,
                    "CAPI_RAT: Ctrl port %lu mapping not found for param %x",
                    payload_ptr->port_id,
                    param_id);
            return CAPI_EBADPARAM;
         }

         imc_param_header_t *imc_cfg_hdr_ptr = (imc_param_header_t *)(payload_ptr + 1);
#ifdef RAT_DEBUG
         RAT_MSG(me_ptr->iid,
                 DBG_HIGH_PRIO,
                 "CAPI_RAT: IMCL port incoming data called with param id 0x%x",

                 imc_cfg_hdr_ptr->opcode);
#endif

         switch (imc_cfg_hdr_ptr->opcode)
         {
            case IMCL_PARAM_ID_TIMER_DRIFT_INFO:
            {
               if (sizeof(param_id_imcl_timer_drift_info) > imc_cfg_hdr_ptr->actual_data_len)
               {
                  RAT_MSG(me_ptr->iid,
                          DBG_ERROR_PRIO,
                          "CAPI_RAT: Invalid payload size for timer drift %d, required = %d",
                          imc_cfg_hdr_ptr->actual_data_len,
                          sizeof(param_id_imcl_timer_drift_info));
                  return CAPI_ENEEDMORE;
               }

               param_id_imcl_timer_drift_info *timer_drift_info_ptr =
                  (param_id_imcl_timer_drift_info *)(imc_cfg_hdr_ptr + 1);

               /* Save the modules drift handle in rat modules port info to be used later on to query*/
               port_ptr->timer_drift_info_hdl_ptr = (timer_drift_info_ptr->handle_ptr);

               RAT_MSG(me_ptr->iid,
                       DBG_HIGH_PRIO,
                       "CAPI_RAT: IMCL port incoming data, timer_drift_info_ptr->handle_ptr = 0x%X. port_id = "
                       "0x%X",
                       timer_drift_info_ptr->handle_ptr,
                       payload_ptr->port_id);
               break;
            }
            case IMCL_PARAM_ID_TIMER_DRIFT_RESYNC:
            {
               if ((port_ptr) && (CTRL_PORT_PEER_CONNECTED == port_ptr->state))
               {
                  capi_rat_get_inp_drift(port_ptr);
               }

               RAT_MSG(me_ptr->iid,
                       DBG_HIGH_PRIO,
                       "CAPI_RAT: IMCL port incoming data, resync drift on port_id = 0x%X",
                       port_ptr->port_id);
               break;
            }
            default:
            {
               RAT_MSG(me_ptr->iid,
                       DBG_ERROR_PRIO,
                       "CAPI_RAT: IMCL port incoming data, Unsupported Param id ::0x%x",
                       imc_cfg_hdr_ptr->opcode);
               break;
            }
         }
         break;
      }
      case INTF_EXTN_PARAM_ID_DATA_PORT_OPERATION:
      {
         capi_result |= capi_rat_handle_port_op(me_ptr, params_ptr);
         break;
      }
      default:
      {
         RAT_MSG(me_ptr->iid, DBG_ERROR_PRIO, "CAPI_RAT: Set, unsupported param ID 0x%x", (int)param_id);
         capi_result |= CAPI_EUNSUPPORTED;
         break;
      }
   } /** End of switch (param ID) */

   return capi_result;
}

/*------------------------------------------------------------------------
 Function name: capi_rat_get_param
 DESCRIPTION: Gets either a parameter value or a parameter structure
 containing multiple parameters. In the event of a failure, the appropriate
 error code is returned.
 * -----------------------------------------------------------------------*/
capi_err_t capi_rat_get_param(capi_t *                _pif,
                              uint32_t                param_id,
                              const capi_port_info_t *port_info_ptr,
                              capi_buf_t *            params_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   if (NULL == _pif || NULL == params_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "CAPI_RAT: get: received bad input pointer(s) for param_id 0x%x property", param_id);
      return CAPI_EFAILED;
   }

   capi_rat_t *me_ptr = (capi_rat_t *)((_pif));

   switch (param_id)
   {
      case FWK_EXTN_PARAM_ID_LATEST_TRIGGER_TIMESTAMP_PTR:
      {
#if 0
         1. Long term: Splitter exposes configuration where per port graph designer can set whether TS is required or not. there will be a new extension to get this timestamp from HW EP. This work can be done along with voiceUI TS based prority sync work
         2. Short term: keep the "latest trigger TS" extension implemented only in HW EP. Dont implement in RAT case. With this, in RAT container, no ext output port carries TS unless input carries TS.
            This helps reduce container MPPS in BT use cases where TS is not required. TS calc involves divisions.
         a. Anyway we dont have TS based priority sync use case right now.
         b. RAT will still propagate TS (for Tx port) if input TS is valid.
#endif
         break;
      }

      default:
      {
         RAT_MSG(me_ptr->iid, DBG_ERROR_PRIO, "CAPI_RAT:  Unsupported Param id :0x%x \n", param_id);
         capi_result = CAPI_EUNSUPPORTED;
         break;
      }
   }

   return capi_result;
}

/*------------------------------------------------------------------------
 Function name: capi_rat_set_properties
 DESCRIPTION: Function to set the properties for the RAT module
 * -----------------------------------------------------------------------*/
capi_err_t capi_rat_set_properties(capi_t *_pif, capi_proplist_t *props_ptr)
{
   capi_rat_t *me_ptr = (capi_rat_t *)_pif;
   return capi_rat_process_set_properties(me_ptr, props_ptr, me_ptr->type);
}

/*------------------------------------------------------------------------
 Function name: capi_rat_get_properties
 DESCRIPTION: Function to get the properties for the RAT module
 * -----------------------------------------------------------------------*/
capi_err_t capi_rat_get_properties(capi_t *_pif, capi_proplist_t *props_ptr)
{
   capi_rat_t *me_ptr = (capi_rat_t *)_pif;
   return capi_rat_process_get_properties(me_ptr, props_ptr, me_ptr->type);
}

/*------------------------------------------------------------------------
 Function name: capi_rat_process_set_properties
 DESCRIPTION: Function to set the properties for the RAT module
 * -----------------------------------------------------------------------*/
static capi_err_t capi_rat_process_set_properties(capi_rat_t *     me_ptr,
                                                  capi_proplist_t *proplist_ptr,
                                                  capi_rat_type_t  type)
{
   capi_err_t capi_result = CAPI_EOK;

   if (NULL == me_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "CAPI_RAT: Set property received null property array");
      return CAPI_EBADPARAM;
   }

   capi_result = capi_cmn_set_basic_properties(proplist_ptr, &me_ptr->heap_mem, &me_ptr->cb_info, FALSE);
   if (CAPI_EOK != capi_result)
   {
      AR_MSG(DBG_ERROR_PRIO, "CAPI_RAT: Set basic properties failed with result %lu", capi_result);
      return capi_result;
   }

   capi_prop_t *prop_ptr = proplist_ptr->prop_ptr;
   uint32_t     i;

   for (i = 0; i < proplist_ptr->props_num; i++)
   {
      capi_buf_t *payload_ptr = &prop_ptr[i].payload;

      switch (prop_ptr[i].id)
      {
         case CAPI_HEAP_ID:
         {
            if (payload_ptr->actual_data_len < sizeof(capi_heap_id_t))
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "capi_spr:  Set Property failed. id= 0x%x Bad param size %u",
                      (uint32_t)prop_ptr[i].id,
                      payload_ptr->actual_data_len);
               return CAPI_ENEEDMORE;
            }

            capi_heap_id_t *data_ptr = (capi_heap_id_t *)payload_ptr->data_ptr;
            me_ptr->heap_mem.heap_id = (POSAL_HEAP_ID)data_ptr->heap_id;

            break;
         }
         case CAPI_EVENT_CALLBACK_INFO:
         case CAPI_ALGORITHMIC_RESET:
         {
            break;
         }
         case CAPI_PORT_NUM_INFO:
         {
            capi_result |= capi_rat_process_port_num_info(me_ptr, &(prop_ptr[i]));
            break;
         }
         case CAPI_INPUT_MEDIA_FORMAT_V2:
         {
            uint32_t port_index = prop_ptr[i].port_info.port_index;

            if ((FALSE == prop_ptr[i].port_info.is_input_port) || (FALSE == prop_ptr[i].port_info.is_valid) ||
                (port_index > me_ptr->num_in_ports))
            {
               AR_MSG(DBG_ERROR_PRIO, "CAPI_RAT: Invalid port info for input media format.");
               return CAPI_EBADPARAM;
            }

            if (!me_ptr->in_port_info_ptr)
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "CAPI_RAT: Error! No memory allocated for input ports, cannot recieve input media format");
               return CAPI_EBADPARAM;
            }
            else
            {
               AR_MSG(DBG_HIGH_PRIO,
                      "CAPI_RAT: Received input media fmt on inp port index 0x%d port ID 0x%lx",
                      port_index,
                      me_ptr->in_port_info_ptr[port_index].cmn.self_port_id);
            }

            // If mf cfg is not yet received yet return error, mandatory
            if (FALSE == me_ptr->configured_media_fmt_received)
            {
               RAT_MSG(me_ptr->iid, DBG_ERROR_PRIO, "CAPI_RAT: static mf cfg not received yet");
               return CAPI_EFAILED;
            }

            uint32_t required_size = sizeof(capi_standard_data_format_v2_t) + sizeof(capi_set_get_media_format_t);

            // Validate the size of payload
            if (payload_ptr->actual_data_len < required_size)
            {
               RAT_MSG(me_ptr->iid,
                       DBG_ERROR_PRIO,
                       "CAPI_RAT: Not valid media format size %d, needed %d",
                       payload_ptr->actual_data_len,
                       required_size);
               return CAPI_EBADPARAM;
            }

            capi_media_fmt_v2_t *inp_media_fmt_ptr = (capi_media_fmt_v2_t *)(payload_ptr->data_ptr);

            if ((0 >= inp_media_fmt_ptr->format.num_channels) ||
                (CAPI_MAX_CHANNELS_V2 < inp_media_fmt_ptr->format.num_channels))
            {
               RAT_MSG(me_ptr->iid,
                       DBG_ERROR_PRIO,
                       "CAPI_RAT: Unsupported num channels %d. Max channels supported: %lu",
                       inp_media_fmt_ptr->format.num_channels,
					   CAPI_MAX_CHANNELS_V2);
               return CAPI_EBADPARAM;
            }

            required_size += inp_media_fmt_ptr->format.num_channels * sizeof(inp_media_fmt_ptr->channel_type[0]);

            // Validate the size of payload
            if (payload_ptr->actual_data_len < required_size)
            {
               RAT_MSG(me_ptr->iid,
                       DBG_ERROR_PRIO,
                       "CAPI_RAT: Not valid media format size %d, required size %d",
                       payload_ptr->max_data_len,
                       required_size);
               return CAPI_ENEEDMORE;
            }

            for (uint32_t i = 0; i < inp_media_fmt_ptr->format.num_channels; i++)
            {
               if ((inp_media_fmt_ptr->format.channel_type[i] < (uint16_t)PCM_CHANNEL_L) ||
                   (inp_media_fmt_ptr->format.channel_type[i] > (uint16_t)PCM_MAX_CHANNEL_MAP_V2))
               {
                  AR_MSG(DBG_ERROR_PRIO,
                         "Unsupported channel type channel idx %d, channel type %d received",
                         (int)i,
                         (int)inp_media_fmt_ptr->format.channel_type[i]);
                  return CAPI_EBADPARAM;
               }
            }

            if (MEDIA_FMT_ID_PCM != inp_media_fmt_ptr->format.bitstream_format)
            {
               RAT_MSG(me_ptr->iid, DBG_ERROR_PRIO, "CAPI_RAT: Unsupported bitstream format");
               return CAPI_EBADPARAM;
            }

            if (CAPI_MEDIA_FORMAT_MINOR_VERSION > inp_media_fmt_ptr->format.minor_version)
            {
               RAT_MSG(me_ptr->iid,
                       DBG_ERROR_PRIO,
                       "CAPI_RAT: media format unsupported minor version %d",
                       inp_media_fmt_ptr->format.minor_version);
               return CAPI_EBADPARAM;
            }

            if ((CAPI_DEINTERLEAVED_UNPACKED_V2 != inp_media_fmt_ptr->format.data_interleaving) &&
                  (CAPI_INTERLEAVED != inp_media_fmt_ptr->format.data_interleaving))
            {
               RAT_MSG(me_ptr->iid,
                       DBG_ERROR_PRIO,
                       "CAPI_RAT: media format unsupported interleaving %d",
                       inp_media_fmt_ptr->format.data_interleaving);
               return CAPI_EBADPARAM;
            }

            if ((0 >= inp_media_fmt_ptr->format.sampling_rate) || (384000 < inp_media_fmt_ptr->format.sampling_rate))
            {
               RAT_MSG(me_ptr->iid,
                       DBG_ERROR_PRIO,
                       "CAPI_RAT: Unsupported sampling rate %d received",
                       inp_media_fmt_ptr->format.sampling_rate);
               return CAPI_EBADPARAM;
            }

            if ((16 != inp_media_fmt_ptr->format.bits_per_sample) && (32 != inp_media_fmt_ptr->format.bits_per_sample))
            {
               RAT_MSG(me_ptr->iid,
                       DBG_ERROR_PRIO,
                       "CAPI_RAT: Unsupported bits per sample %d received",
                       inp_media_fmt_ptr->format.bits_per_sample);
               return CAPI_EBADPARAM;
            }

            if (((16 == inp_media_fmt_ptr->format.bits_per_sample) && (15 != inp_media_fmt_ptr->format.q_factor)) ||
                ((32 == inp_media_fmt_ptr->format.bits_per_sample) &&
                 ((27 != inp_media_fmt_ptr->format.q_factor) && (31 != inp_media_fmt_ptr->format.q_factor))))
            {
               RAT_MSG(me_ptr->iid,
                       DBG_ERROR_PRIO,
                       "CAPI_RAT: Unsupported bits per sample %d qfactor combo %d received",
                       inp_media_fmt_ptr->format.bits_per_sample,
                       inp_media_fmt_ptr->format.q_factor);
               return CAPI_EBADPARAM;
            }

            /*If inp mf comes on static port do the mf validation against the configured mf
             *For other ports, when inp mf comes, we propagate to output
             */
            // Case 1: Input port is not opened
            if (RAT_PORT_INDEX_INVALID == me_ptr->in_port_info_ptr[port_index].cmn.self_port_id)
            {
               RAT_MSG(me_ptr->iid,
                       DBG_ERROR_PRIO,
                       "CAPI_RAT: port id is invalid 0x%lx, port index %d, port state %d",
                       me_ptr->in_port_info_ptr[port_index].cmn.self_port_id,
                       port_index,
                       me_ptr->in_port_info_ptr[port_index].cmn.port_state);
               return CAPI_EBADPARAM;
            }
            // Case 2: This is the static input port
            else if (PORT_ID_RATE_ADAPTED_TIMER_INPUT == me_ptr->in_port_info_ptr[port_index].cmn.self_port_id)
            {
               // For static port validate that media format matches, if not fail inp mf
               if (me_ptr->configured_media_fmt.header.format_header.data_format !=
                   inp_media_fmt_ptr->header.format_header.data_format)
               {
                  RAT_MSG(me_ptr->iid,
                          DBG_ERROR_PRIO,
                          "CAPI_RAT: data format received %d doesnt match set %d",
                          inp_media_fmt_ptr->header.format_header.data_format,
                          me_ptr->configured_media_fmt.header.format_header.data_format);
                  return CAPI_EBADPARAM;
               }

               // media fmt validation
               if ((inp_media_fmt_ptr->format.sampling_rate != me_ptr->configured_media_fmt.format.sampling_rate) ||
                   (inp_media_fmt_ptr->format.bits_per_sample != me_ptr->configured_media_fmt.format.bits_per_sample) ||
                   (inp_media_fmt_ptr->format.num_channels != me_ptr->configured_media_fmt.format.num_channels) ||
                   (inp_media_fmt_ptr->format.q_factor != me_ptr->configured_media_fmt.format.q_factor))
               {
                  RAT_MSG(me_ptr->iid,
                          DBG_ERROR_PRIO,
                          "CAPI_RAT: Media format validation failed: media format sampling rate: %d rat sampling rate: "
                          "%d, media format bits_per_sample %d, q factor "
                          "%d. rat bit width: %d q factor %d, media format num_channels: %d rat num_channels: %d",
                          inp_media_fmt_ptr->format.sampling_rate,
                          me_ptr->configured_media_fmt.format.sampling_rate,
                          inp_media_fmt_ptr->format.bits_per_sample,
                          inp_media_fmt_ptr->format.q_factor,
                          me_ptr->configured_media_fmt.format.bits_per_sample,
                          me_ptr->configured_media_fmt.format.q_factor,
                          inp_media_fmt_ptr->format.num_channels,
                          me_ptr->configured_media_fmt.format.num_channels);
                  return CAPI_EBADPARAM;
               }

               for (uint32_t i = 0; i < me_ptr->configured_media_fmt.format.num_channels; i++)
               {
                  if (me_ptr->configured_media_fmt.format.channel_type[i] != inp_media_fmt_ptr->format.channel_type[i])
                  {
                     RAT_MSG(me_ptr->iid,
                             DBG_ERROR_PRIO,
                             "media format channel type: %d rat channel type: %d for channel %d",
                             inp_media_fmt_ptr->format.channel_type[i],
                             me_ptr->configured_media_fmt.format.channel_type[i],
                             i);
                     return CAPI_EBADPARAM;
                  }
               }
            }
            // Case 3: If it is not the static port and mf cfg is received (checked above), ensure same family of rates
            else if (!capi_rat_is_sample_rate_accepted(me_ptr->configured_media_fmt.format.sampling_rate,
                                                       inp_media_fmt_ptr->format.sampling_rate))
            {
               RAT_MSG(me_ptr->iid,
                       DBG_ERROR_PRIO,
                       "CAPI_RAT: Unsupported sample rate %d received, has to be in same "
                       "family as operating sample rate %d",
                       inp_media_fmt_ptr->format.sampling_rate,
                       me_ptr->configured_media_fmt.format.sampling_rate);
               return CAPI_EBADPARAM;
            }

            // After all checks have passed, save the inp mf
            memscpy(&me_ptr->in_port_info_ptr[port_index].media_fmt,
                    sizeof(capi_media_fmt_v2_t),
                    inp_media_fmt_ptr,
                    sizeof(capi_media_fmt_v2_t));

            me_ptr->in_port_info_ptr[port_index].inp_mf_received = TRUE;

            // Raise media format on connected output for Case 2 and 3:
            if (RAT_PORT_INDEX_INVALID != me_ptr->in_port_info_ptr[port_index].cmn.conn_index)
            {
               capi_result = capi_cmn_output_media_fmt_event_v2(&me_ptr->cb_info,
                                                                inp_media_fmt_ptr,
                                                                FALSE,
                                                                me_ptr->in_port_info_ptr[port_index].cmn.conn_index);
            }

            capi_result |=
               capi_rat_raise_thresh_event(me_ptr,
                                           inp_media_fmt_ptr,
                                           port_index, /*inp index*/
                                           me_ptr->in_port_info_ptr[port_index].cmn.conn_index /*out index*/);

            break;
         }
         case CAPI_CUSTOM_PROPERTY:
         {
            capi_custom_property_t *cust_prop_ptr    = (capi_custom_property_t *)payload_ptr->data_ptr;
            void *                  cust_payload_ptr = (void *)(cust_prop_ptr + 1);

            switch (cust_prop_ptr->secondary_prop_id)
            {
               case FWK_EXTN_PROPERTY_ID_STM_TRIGGER:
               {
                  if (payload_ptr->actual_data_len < sizeof(capi_custom_property_t) + sizeof(capi_prop_stm_trigger_t))
                  {
                     RAT_MSG(me_ptr->iid,
                             DBG_ERROR_PRIO,
                             "CAPI_RAT: Property id 0x%lx Insufficient payload size %d",
                             (uint32_t)cust_prop_ptr->secondary_prop_id,
                             payload_ptr->actual_data_len);
                     return CAPI_EBADPARAM;
                  }

                  // Get stm info
                  capi_prop_stm_trigger_t *trig_ptr = (capi_prop_stm_trigger_t *)cust_payload_ptr;
                  me_ptr->signal_ptr                = trig_ptr->signal_ptr;

                  // Timer enable checked inside
                  capi_result |= capi_rat_timer_enable(me_ptr);
                  break;
               }
               case FWK_EXTN_PROPERTY_ID_STM_CTRL:
               {
                  if (payload_ptr->actual_data_len < sizeof(capi_prop_stm_ctrl_t) + sizeof(capi_custom_property_t))
                  {
                     RAT_MSG(me_ptr->iid,
                             DBG_ERROR_PRIO,
                             "CAPI_RAT: Param id 0x%lx Bad param size %lu",
                             (uint32_t)cust_prop_ptr->secondary_prop_id,
                             payload_ptr->actual_data_len);
                     capi_result |= CAPI_ENEEDMORE;
                     break;
                  }

                  capi_prop_stm_ctrl_t *timer_en = (capi_prop_stm_ctrl_t *)cust_payload_ptr;

                  // reset the timer when we get start/stop
                  me_ptr->counter = 1;

                  me_ptr->is_timer_enable = timer_en->enable;

                  capi_result |= capi_rat_timer_enable(me_ptr);
                  break;
               }
               default:
               {
                  RAT_MSG(me_ptr->iid,
                          DBG_HIGH_PRIO,
                          "CAPI_RAT: Unknown Custom Property[%d]",
                          cust_prop_ptr->secondary_prop_id);
                  capi_result |= CAPI_EUNSUPPORTED;
                  break;
               }
            }
            break;
         }
         case CAPI_MODULE_INSTANCE_ID:
         {
            if (payload_ptr->actual_data_len >= sizeof(capi_module_instance_id_t))
            {
               capi_module_instance_id_t *data_ptr = (capi_module_instance_id_t *)payload_ptr->data_ptr;
               me_ptr->iid                         = data_ptr->module_instance_id;
               AR_MSG(DBG_LOW_PRIO,
                      "CAPI RAT: This module-id 0x%08lX, instance-id 0x%08lX",
                      data_ptr->module_id,
                      me_ptr->iid);
            }
            break;
         }
         default:
         {
            capi_result |= CAPI_EUNSUPPORTED;
            continue;
         }
      } // Outer switch - Generic CAPI Properties
   }    // Loop all properties
   return capi_result;
}

/*------------------------------------------------------------------------
 Function name: capi_rat_process_get_properties
 DESCRIPTION: Function to get the properties for the RAT module
 * -----------------------------------------------------------------------*/
static capi_err_t capi_rat_process_get_properties(capi_rat_t *     me_ptr,
                                                  capi_proplist_t *proplist_ptr,
                                                  capi_rat_type_t  type)
{
   capi_err_t capi_result = CAPI_EOK;
   uint32_t   i;

   uint32_t fwk_extn_ids_arr[] = { FWK_EXTN_THRESHOLD_CONFIGURATION, FWK_EXTN_STM };

   capi_basic_prop_t mod_prop;
   mod_prop.init_memory_req    = sizeof(capi_rat_t);
   mod_prop.stack_size         = RATE_ADAPTED_TIMER_STACK_SIZE;
   mod_prop.num_fwk_extns      = RAT_NUM_FRAMEWORK_EXTENSIONS;
   mod_prop.fwk_extn_ids_arr   = fwk_extn_ids_arr;
   mod_prop.is_inplace         = FALSE;
   mod_prop.req_data_buffering = 0; // NA
   mod_prop.max_metadata_size  = 0; // NA

   capi_result = capi_cmn_get_basic_properties(proplist_ptr, &mod_prop);
   if (CAPI_EOK != capi_result)
   {
      AR_MSG(DBG_ERROR_PRIO, "CAPI_RAT: Get common basic properties failed with result %lu", capi_result);
      return capi_result;
   }

   capi_prop_t *prop_ptr = proplist_ptr->prop_ptr;

   for (i = 0; i < proplist_ptr->props_num; i++)
   {
      capi_buf_t *payload_ptr = &prop_ptr[i].payload;
      switch (prop_ptr[i].id)
      {
         case CAPI_INIT_MEMORY_REQUIREMENT:
         case CAPI_STACK_SIZE:
         case CAPI_NUM_NEEDED_FRAMEWORK_EXTENSIONS:
         case CAPI_NEEDED_FRAMEWORK_EXTENSIONS:
         case CAPI_IS_INPLACE:
         case CAPI_REQUIRES_DATA_BUFFERING:
         {
            break;
         }
         case CAPI_OUTPUT_MEDIA_FORMAT_SIZE:
         {
            if (NULL == me_ptr || NULL == me_ptr->out_port_info_ptr)
            {
               AR_MSG(DBG_ERROR_PRIO, "CAPI RAT: Get property id 0x%lx, module is not allocated", prop_ptr[i].id);
               return CAPI_EBADPARAM;
            }

            if (payload_ptr->max_data_len < sizeof(capi_output_media_format_size_t))
            {
               AR_MSG(DBG_ERROR_PRIO, "CAPI RAT: Insufficient get property size.");
               return CAPI_ENEEDMORE;
            }

            uint32_t out_port_index      = prop_ptr[i].port_info.port_index;
            uint32_t peer_inp_port_index = me_ptr->out_port_info_ptr[out_port_index].cmn.conn_index;

            capi_output_media_format_size_t *data_ptr = (capi_output_media_format_size_t *)payload_ptr->data_ptr;
            uint32_t                         channel_type_size = 0;

            if (prop_ptr[i].port_info.is_valid && !prop_ptr[i].port_info.is_input_port &&
                prop_ptr[i].port_info.port_index < me_ptr->num_out_ports && (me_ptr->in_port_info_ptr) &&
                (me_ptr->in_port_info_ptr[peer_inp_port_index].inp_mf_received))
            {
               channel_type_size = me_ptr->in_port_info_ptr[peer_inp_port_index].media_fmt.format.num_channels *
                                   sizeof(capi_channel_type_t);
               data_ptr->size_in_bytes      = sizeof(capi_standard_data_format_v2_t) + channel_type_size;
               payload_ptr->actual_data_len = sizeof(capi_output_media_format_size_t);
            }
            else
            {
               payload_ptr->actual_data_len = 0;
               AR_MSG(DBG_LOW_PRIO, "CAPI RAT: Cant return out mf yet, invalid port info or have not got inp mf");
               return CAPI_EBADPARAM;
            }
            break;
         }
         case CAPI_OUTPUT_MEDIA_FORMAT_V2:
         {
            if (NULL == me_ptr || NULL == me_ptr->out_port_info_ptr)
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "CAPI RAT: Get property id 0x%lx, module is not allocated or output ports are not opened",
                      prop_ptr[i].id);
               return CAPI_EBADPARAM;
            }

            if (!prop_ptr[i].port_info.is_valid || prop_ptr[i].port_info.is_input_port ||
                prop_ptr[i].port_info.port_index > me_ptr->num_out_ports)
            {
               AR_MSG(DBG_ERROR_PRIO, "CAPI RAT: Cant return out mf, it is not a valid output port");
               return CAPI_EBADPARAM;
            }

            uint32_t out_port_index      = prop_ptr[i].port_info.port_index;
            uint32_t peer_inp_port_index = me_ptr->out_port_info_ptr[out_port_index].cmn.conn_index;

            capi_media_fmt_v2_t *data_ptr = (capi_media_fmt_v2_t *)payload_ptr->data_ptr;
            capi_media_fmt_v2_t *mf_ptr   = NULL;

            // Case 1: for static output port
            if ((capi_rat_is_output_static_port(&me_ptr->out_port_info_ptr[out_port_index].cmn)) &&
                (me_ptr->configured_media_fmt_received))
            {
               mf_ptr = &me_ptr->configured_media_fmt;
            }
            // Case 2: for dynamic output port.
            // If corresponding inp port is not present, we will not raise out mf
            else if ((RAT_PORT_INDEX_INVALID != peer_inp_port_index) && (me_ptr->in_port_info_ptr) &&
                     (me_ptr->in_port_info_ptr[peer_inp_port_index].inp_mf_received))
            {
               mf_ptr = &me_ptr->in_port_info_ptr[peer_inp_port_index].media_fmt;
            }
            else
            {
               payload_ptr->actual_data_len = 0;
               AR_MSG(DBG_ERROR_PRIO, "CAPI RAT: Get out mf: Output port not opened or connected");
               return CAPI_EBADPARAM;
            }

            uint32_t required_channel_map_size = mf_ptr->format.num_channels * sizeof(capi_channel_type_t);
            uint32_t required_size =
               sizeof(capi_set_get_media_format_t) + sizeof(capi_standard_data_format_v2_t) + required_channel_map_size;

            if (payload_ptr->max_data_len < required_size)
            {
               AR_MSG(DBG_ERROR_PRIO, "CAPI RAT: Insufficient get property size.");
               return CAPI_ENEEDMORE;
            }

            payload_ptr->actual_data_len = memscpy(data_ptr, payload_ptr->max_data_len, mf_ptr, required_size);

            payload_ptr->actual_data_len += memscpy(data_ptr->channel_type,
                                                    required_channel_map_size,
                                                    mf_ptr->format.channel_type,
                                                    required_channel_map_size);

            break;
         }
         case CAPI_PORT_DATA_THRESHOLD:
         {
            if (NULL == me_ptr)
            {
               AR_MSG(DBG_ERROR_PRIO, "CAPI_RAT : null ptr while querying threshold");
               return CAPI_EBADPARAM;
            }

            uint32_t             threshold = 0, inp_port_index = 0;
            capi_media_fmt_v2_t *mf_ptr = NULL;

            // Case A: If threshold query is for output port
            if (prop_ptr[i].port_info.is_valid && !prop_ptr[i].port_info.is_input_port &&
                prop_ptr[i].port_info.port_index < me_ptr->num_out_ports && (me_ptr->out_port_info_ptr))
            {
               // Case 1: Static output port
               if ((PORT_ID_RATE_ADAPTED_TIMER_OUTPUT ==
                    me_ptr->out_port_info_ptr[prop_ptr[i].port_info.port_index].cmn.self_port_id) &&
                   (me_ptr->configured_media_fmt_received))
               {
                  mf_ptr = &me_ptr->configured_media_fmt;
               }
               else // dynamic out port
               {
                  inp_port_index = me_ptr->out_port_info_ptr[prop_ptr[i].port_info.port_index].cmn.conn_index;
                  // Case 2: Inp port not connected
                  if (RAT_PORT_INDEX_INVALID == inp_port_index)
                  {
                     AR_MSG(DBG_ERROR_PRIO,
                            "CAPI RAT: Get threshold: Input port not connected, no media format on this port");
                     return CAPI_EOK;
                  }
                  // Case 3: Inp port connected
                  else if (me_ptr->in_port_info_ptr[inp_port_index].inp_mf_received)
                  {
                     mf_ptr = &me_ptr->in_port_info_ptr[inp_port_index].media_fmt;
                  }
                  else
                  {
                     AR_MSG(DBG_ERROR_PRIO, "CAPI RAT: Get threshold: Output port not opened or connected");
                     return CAPI_EBADPARAM;
                  }
               }
            }
            // Case B: If threshold query is for inp port
            else if (prop_ptr[i].port_info.is_valid && prop_ptr[i].port_info.is_input_port &&
                     prop_ptr[i].port_info.port_index < me_ptr->num_in_ports && (me_ptr->in_port_info_ptr))
            {
               inp_port_index = prop_ptr[i].port_info.port_index;

               // Case 1: Static input port
               if ((PORT_ID_RATE_ADAPTED_TIMER_INPUT == me_ptr->in_port_info_ptr[inp_port_index].cmn.self_port_id) &&
                   (me_ptr->configured_media_fmt_received))
               {
                  mf_ptr = &me_ptr->configured_media_fmt;
               }
               // Case 2: Dynamic input port
               else if (me_ptr->in_port_info_ptr[inp_port_index].inp_mf_received)
               {
                  mf_ptr = &me_ptr->in_port_info_ptr[inp_port_index].media_fmt;
               }
               else
               {
                  AR_MSG(DBG_ERROR_PRIO, "CAPI RAT: Get threshold: Output port not opened or connected");
                  return CAPI_EBADPARAM;
               }
            }
            else
            {
               return CAPI_EBADPARAM;
            }

            uint32_t bytes_per_sample =
               (mf_ptr->format.bits_per_sample > BIT_WIDTH_16) ? BYTES_PER_SAMPLE_FOUR : BYTES_PER_SAMPLE_TWO;

            // Port threshold event
            // Mandatory : First get the samples and then scale up by channels , bytes_per_sample.
            threshold = (uint32_t)(
               ((uint64_t)((mf_ptr->format.sampling_rate / NUM_MS_PER_SEC) * me_ptr->frame_dur_us)) / NUM_MS_PER_SEC);

            threshold = threshold * mf_ptr->format.num_channels * bytes_per_sample;

            capi_result = capi_cmn_handle_get_port_threshold(&prop_ptr[i], threshold);
            break;
         }
         case CAPI_INTERFACE_EXTENSIONS:
         {
            if (payload_ptr->max_data_len >= sizeof(capi_interface_extns_list_t))
            {
               capi_interface_extns_list_t *intf_ext_list = (capi_interface_extns_list_t *)payload_ptr->data_ptr;
               if (payload_ptr->max_data_len < (sizeof(capi_interface_extns_list_t) +
                                                (intf_ext_list->num_extensions * sizeof(capi_interface_extn_desc_t))))
               {
                  AR_MSG(DBG_ERROR_PRIO,
                         "CAPI_RAT: CAPI_INTERFACE_EXTENSIONS invalid param size %lu",
                         payload_ptr->max_data_len);
                  CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
               }
               else
               {
                  capi_interface_extn_desc_t *curr_intf_extn_desc_ptr =
                     (capi_interface_extn_desc_t *)(payload_ptr->data_ptr + sizeof(capi_interface_extns_list_t));

                  for (uint32_t i = 0; i < intf_ext_list->num_extensions; i++)
                  {
                     switch (curr_intf_extn_desc_ptr->id)
                     {
                        case INTF_EXTN_IMCL:
                        case INTF_EXTN_METADATA:
                        case INTF_EXTN_DATA_PORT_OPERATION:
                        {
                           curr_intf_extn_desc_ptr->is_supported = TRUE;
                           break;
                        }
                        default:
                        {
                           curr_intf_extn_desc_ptr->is_supported = FALSE;
                           break;
                        }
                     }
                     AR_MSG(DBG_HIGH_PRIO,
                            "CAPI_RAT: CAPI_INTERFACE_EXTENSIONS intf_ext = 0x%lx, is_supported = %d",
                            curr_intf_extn_desc_ptr->id,
                            (int)curr_intf_extn_desc_ptr->is_supported);
                     curr_intf_extn_desc_ptr++;
                  }
               }
            }
            else
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "CAPI_RAT: CAPI_INTERFACE_EXTENSIONS Bad param size %lu",
                      payload_ptr->max_data_len);
               CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
            }
            break;
         }
         case CAPI_MIN_PORT_NUM_INFO:
         {
            if (payload_ptr->max_data_len >= sizeof(capi_min_port_num_info_t))
            {
               capi_min_port_num_info_t *data_ptr = (capi_min_port_num_info_t *)payload_ptr->data_ptr;
               data_ptr->num_min_input_ports      = 0; // rat can operate as src
               data_ptr->num_min_output_ports     = 0; // rat can operate as sink

               AR_MSG(DBG_HIGH_PRIO,
                      "CAPI_RAT: Get prop id 0x%lx Returning min inp ports %d, min out ports %d, "
                      "payload "
                      "size %d",
                      (uint32_t)prop_ptr[i].id,
                      data_ptr->num_min_input_ports,
                      data_ptr->num_min_output_ports,
                      payload_ptr->max_data_len);
               payload_ptr->actual_data_len = sizeof(capi_min_port_num_info_t);
            }
            else
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "CAPI_RAT: Get, Prop id 0x%lx Bad param size %lu",
                      (uint32_t)prop_ptr[i].id,
                      payload_ptr->max_data_len);
               payload_ptr->actual_data_len =
                  0; // according to CAPI V2, actual len should be num of bytes read(set)/written(get)
               return (capi_result | CAPI_ENEEDMORE);
            }
            break;
         }
         default:
         {
            continue;
         }
      } /** End of switch (prop_id) */
   }    /** End of for loop (num props)*/
   return capi_result;
}
