/* =========================================================================
  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
  SPDX-License-Identifier: BSD-3-Clause-Clear
 * =========================================================================*/

/**
 * @file capi_gain.cpp
 *
 * C source file to implement the Gain module
*/

#include "capi_gain_utils.h"

static capi_err_t capi_gain_process(capi_t *            _pif,
                                          capi_stream_data_t *input[],
                                          capi_stream_data_t *output[]);

static capi_err_t capi_gain_end(capi_t *_pif);

static capi_err_t capi_gain_set_param(capi_t *                _pif,
                                            uint32_t                   param_id,
                                            const capi_port_info_t *port_info_ptr,
                                            capi_buf_t *            params_ptr);

static capi_err_t capi_gain_get_param(capi_t *                _pif,
                                            uint32_t                   param_id,
                                            const capi_port_info_t *port_info_ptr,
                                            capi_buf_t *            params_ptr);

static capi_err_t capi_gain_set_properties(capi_t *_pif, capi_proplist_t *props_ptr);

static capi_err_t capi_gain_get_properties(capi_t *_pif, capi_proplist_t *props_ptr);

static capi_vtbl_t vtbl = { capi_gain_process,        capi_gain_end,
                               capi_gain_set_param,      capi_gain_get_param,
                               capi_gain_set_properties, capi_gain_get_properties };

/* -------------------------------------------------------------------------
 * Function name: capi_gain_get_static_properties
 * Capi_v2 Gain function to get the static properties
 * -------------------------------------------------------------------------*/
capi_err_t capi_gain_get_static_properties(capi_proplist_t *init_set_properties,
                                                 capi_proplist_t *static_properties)
{
   capi_err_t capi_result = CAPI_EOK;
   GAIN_MSG(MIID_UNKNOWN, DBG_HIGH_PRIO, "Enter get static properties");
   if (NULL != static_properties)
   {
      capi_result = capi_gain_process_get_properties((capi_gain_t *)NULL, static_properties);
      if (CAPI_FAILED(capi_result))
      {
         GAIN_MSG(MIID_UNKNOWN, DBG_ERROR_PRIO, "get static properties failed!");
         return capi_result;
      }
   }
   else
   {
      GAIN_MSG(MIID_UNKNOWN, DBG_ERROR_PRIO, "Get static properties received bad pointer, 0x%p", static_properties);
   }

   return capi_result;
}

/*------------------------------------------------------------------------
  Function name: capi_gain_init
  Initialize the CAPIv2 Gain Module. This function can allocate memory.
 * -----------------------------------------------------------------------*/

capi_err_t capi_gain_init(capi_t *_pif, capi_proplist_t *init_set_properties)
{
   capi_err_t capi_result = CAPI_EOK;
   GAIN_MSG(MIID_UNKNOWN, DBG_LOW_PRIO, "Enter gain init");
   if (NULL == _pif || NULL == init_set_properties)
   {
      GAIN_MSG(MIID_UNKNOWN, DBG_ERROR_PRIO, "Init received bad pointer, 0x%p, 0x%p", _pif, init_set_properties);

      CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      return capi_result;
   }

   capi_gain_t *me_ptr = (capi_gain_t *)_pif;

   memset(me_ptr, 0, sizeof(capi_gain_t));

   me_ptr->vtbl.vtbl_ptr = &vtbl;

   capi_cmn_init_media_fmt_v2(&me_ptr->inp_media_fmt);

   capi_gain_init_config(me_ptr);

   if (NULL != init_set_properties)
   {
      capi_result = capi_gain_process_set_properties(me_ptr, init_set_properties);
      // ignore unsupported error
      if (CAPI_FAILED(capi_result) && (CAPI_EUNSUPPORTED != capi_result))
      {
         GAIN_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Initialization Set Property Failed");
         return capi_result;
      }
   }

   capi_result |= capi_gain_raise_event(me_ptr);

   capi_result |= capi_cmn_raise_deinterleaved_unpacked_v2_supported_event(&me_ptr->cb_info);

   GAIN_MSG(me_ptr->miid, DBG_HIGH_PRIO, "Initialization completed !!");
   return capi_result;
}

/* -------------------------------------------------------------------------
 * Function name: capi_gain_process
 * Gain module Data Process function to process an input buffer
 * and generates an output buffer.
 * -------------------------------------------------------------------------*/
static capi_err_t capi_gain_process(capi_t *            _pif,
                                          capi_stream_data_t *input[],
                                          capi_stream_data_t *output[])
{
   capi_err_t   capi_result = CAPI_EOK;
   capi_gain_t *me_ptr         = (capi_gain_t *)_pif;
   POSAL_ASSERT(me_ptr);
   POSAL_ASSERT(input[0]);
   POSAL_ASSERT(output[0]);

   capi_result = gain_process(_pif, input, output);
   return capi_result;
}

/*------------------------------------------------------------------------
 * Function name: capi_gain_end
 * Gain End function, returns the library to the uninitialized
 * state and frees all the memory that was allocated. This function also
 * frees the virtual function table.
 * -----------------------------------------------------------------------*/
static capi_err_t capi_gain_end(capi_t *_pif)
{

   capi_err_t capi_result = CAPI_EOK;
   if (NULL == _pif)
   {
      GAIN_MSG(MIID_UNKNOWN, DBG_ERROR_PRIO, "End received bad pointer, 0x%p", _pif);
      CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      return capi_result;
   }

   capi_gain_t *me_ptr = (capi_gain_t *)_pif;
   uint32_t miid = me_ptr->miid;
   me_ptr->vtbl.vtbl_ptr  = NULL;

   GAIN_MSG(miid, DBG_HIGH_PRIO, "End done");
   return capi_result;
}

/* -------------------------------------------------------------------------
 * Function name: capi_gain_set_param
 * Sets either a parameter value or a parameter structure containing
 * multiple parameters. In the event of a failure, the appropriate error
 * code is returned.
 * -------------------------------------------------------------------------*/
static capi_err_t capi_gain_set_param(capi_t *                _pif,
                                            uint32_t                   param_id,
                                            const capi_port_info_t *port_info_ptr,
                                            capi_buf_t *            params_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   if (NULL == _pif || NULL == params_ptr)
   {
      GAIN_MSG(MIID_UNKNOWN, DBG_ERROR_PRIO, "set param received bad pointer, 0x%p, 0x%p", _pif, params_ptr);
      return CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
   }
   capi_gain_t *me_ptr = (capi_gain_t *)(_pif);

   switch (param_id)
   {
      case PARAM_ID_MODULE_ENABLE:
      {
         uint16_t param_size = params_ptr->actual_data_len;
         if (param_size >= sizeof(param_id_module_enable_t))
         {
            param_id_module_enable_t *codec_gain_enable_ptr = (param_id_module_enable_t *)(params_ptr->data_ptr);
            me_ptr->lib_config.enable                       = codec_gain_enable_ptr->enable;
            capi_result |= capi_gain_raise_process_event(me_ptr);
            GAIN_MSG(me_ptr->miid, DBG_HIGH_PRIO, "PARAM_ID_MODULE_ENABLE, %lu", me_ptr->lib_config.enable);
         }
         else
         {
            GAIN_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPIv2 Codec Gain Enable/Disable, Bad param size %hu", param_size);
            CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
         }
         break;
      }
      case PARAM_ID_GAIN:
      {
         if (params_ptr->actual_data_len >= sizeof(param_id_gain_cfg_t))
         {
            param_id_gain_cfg_t *rx_gain_packet_ptr = (param_id_gain_cfg_t *)(params_ptr->data_ptr);
            me_ptr->lib_config.gain_q13             = rx_gain_packet_ptr->gain;
            me_ptr->gain_q12                        = rx_gain_packet_ptr->gain >> 1;
            capi_result |= capi_gain_raise_process_event(me_ptr);
            GAIN_MSG(me_ptr->miid, DBG_HIGH_PRIO, "PARAM_ID_GAIN %d", me_ptr->lib_config.gain_q13);
         }
         else
         {
            GAIN_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Set, Bad param size %lu", params_ptr->actual_data_len);
            CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
         }
         break;
      }
      default:
      {
         GAIN_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Set, unsupported param ID 0x%x", (int)param_id);
         CAPI_SET_ERROR(capi_result, CAPI_EUNSUPPORTED);
         break;
      }
   }
   return capi_result;
}

/* -------------------------------------------------------------------------
 * Function name: capi_gain_get_param
 * Gets either a parameter value or a parameter structure containing
 * multiple parameters. In the event of a failure, the appropriate error
 * code is returned.
 * -------------------------------------------------------------------------*/
static capi_err_t capi_gain_get_param(capi_t *                _pif,
                                            uint32_t                   param_id,
                                            const capi_port_info_t *port_info_ptr,
                                            capi_buf_t *            params_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   if (NULL == _pif || NULL == params_ptr)
   {
      GAIN_MSG(MIID_UNKNOWN, DBG_ERROR_PRIO, "Get param received bad pointer, 0x%p, 0x%p", _pif, params_ptr);
      return CAPI_EBADPARAM;
   }

   capi_gain_t *me_ptr = (capi_gain_t *)_pif;

   switch (param_id)
   {
      case PARAM_ID_MODULE_ENABLE:
      {
         const uint16_t payload_size = (uint16_t)(params_ptr->max_data_len);
         if (payload_size >= sizeof(param_id_module_enable_t))
         {
            param_id_module_enable_t *codec_gain_enable_ptr = (param_id_module_enable_t *)(params_ptr->data_ptr);
            codec_gain_enable_ptr->enable                   = me_ptr->lib_config.enable;
            params_ptr->actual_data_len                     = (uint32_t)sizeof(param_id_module_enable_t);
         }
         else
         {
            GAIN_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPIv2 Codec Gain Get Enable Param, Bad payload size %d", payload_size);
            CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
         }
         break;
      }
      case PARAM_ID_GAIN:
      {
         if (params_ptr->max_data_len >= sizeof(param_id_gain_cfg_t))
         {
            param_id_gain_cfg_t *rx_gain_packet_ptr = (param_id_gain_cfg_t *)(params_ptr->data_ptr);
            params_ptr->actual_data_len             = sizeof(param_id_gain_cfg_t);
            rx_gain_packet_ptr->gain                = me_ptr->lib_config.gain_q13;
            rx_gain_packet_ptr->reserved            = 0;
         }
         else
         {
            GAIN_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Get, Bad param size %lu", params_ptr->max_data_len);
            CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
         }
         break;
      }
      default:
      {
         GAIN_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Get, unsupported param ID 0x%x", (int)param_id);
         CAPI_SET_ERROR(capi_result, CAPI_EUNSUPPORTED);
         break;
      }
   }
   return capi_result;
}

static capi_err_t capi_gain_set_properties(capi_t *_pif, capi_proplist_t *props_ptr)
{
   capi_gain_t *me_ptr = (capi_gain_t *)_pif;
   return capi_gain_process_set_properties(me_ptr, props_ptr);
}

/* -------------------------------------------------------------------------
 * Function name: capi_gain_get_properties
 * Function to get the properties of GAIN module
 * -------------------------------------------------------------------------*/
static capi_err_t capi_gain_get_properties(capi_t *_pif, capi_proplist_t *props_ptr)
{
   capi_gain_t *me_ptr = (capi_gain_t *)_pif;
   return capi_gain_process_get_properties(me_ptr, props_ptr);
}
