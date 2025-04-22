/**
 * \file capi_example_gain_module.c
 *  
 * \brief
 *  
 *     Example Gain Module
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/*------------------------------------------------------------------------
 * Include files
 * -----------------------------------------------------------------------*/
#include "capi_example_gain_module_structs.h"

/*------------------------------------------------------------------------
 * Static function declarations
 * -----------------------------------------------------------------------*/
static capi_err_t capi_example_gain_module_process(capi_t *_pif, capi_stream_data_t *input[], capi_stream_data_t *output[]);

static capi_err_t capi_example_gain_module_end(capi_t *_pif);

static capi_err_t capi_example_gain_module_set_param(capi_t *                _pif,
                                                     uint32_t                param_id,
                                                     const capi_port_info_t *port_info_ptr,
                                                     capi_buf_t *            params_ptr);

static capi_err_t capi_example_gain_module_get_param(capi_t *                _pif,
                                                     uint32_t                param_id,
                                                     const capi_port_info_t *port_info_ptr,
                                                     capi_buf_t *            params_ptr);

static capi_err_t capi_example_gain_module_set_properties(capi_t *_pif, capi_proplist_t *props_ptr);

static capi_err_t capi_example_gain_module_get_properties(capi_t *_pif, capi_proplist_t *props_ptr);

/* Function table for gain module*/
static capi_vtbl_t vtbl = { capi_example_gain_module_process,        capi_example_gain_module_end,
                            capi_example_gain_module_set_param,      capi_example_gain_module_get_param,
                            capi_example_gain_module_set_properties, capi_example_gain_module_get_properties };

/* -------------------------------------------------------------------------
 * Function name: capi_example_gain_module_get_static_properties
 * Used to query the static properties of the gain module that are independent of the
   instance. This function is used to query the memory requirements of the module
   in order to create an instance.
 * -------------------------------------------------------------------------*/
capi_err_t capi_example_gain_module_get_static_properties(capi_proplist_t *init_set_properties,
                                                          capi_proplist_t *static_properties)
{
   capi_err_t capi_result = CAPI_EOK;

   if (NULL != static_properties)
   {
      capi_result = capi_example_gain_module_process_get_properties((capi_example_gain_module_t *)NULL, static_properties);
      if (CAPI_FAILED(capi_result))
      {
         AR_MSG(DBG_ERROR_PRIO, "capi_example_gain_module: get static properties failed!");
         return capi_result;
      }
      else
      {
         AR_MSG(DBG_HIGH_PRIO, "capi_example_gain_module: get static properties successful");
      }
   }
   else
   {
      AR_MSG(DBG_ERROR_PRIO, "capi_example_gain_module: Get static properties received NULL bad pointer");
   }

   return capi_result;
}

/*------------------------------------------------------------------------
  Function name: capi_example_gain_module_init
  Instantiates the gain module to set up the virtual function table, and also
  allocates any memory required by the module. Default states within the module are also
  initialized here
 * -----------------------------------------------------------------------*/
capi_err_t capi_example_gain_module_init(capi_t *_pif, capi_proplist_t *init_set_properties)
{
   capi_err_t capi_result = CAPI_EOK;

   if (NULL == _pif || NULL == init_set_properties)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi_example_gain_module: Init received NULL pointers, failing");
      return CAPI_EBADPARAM;
   }

   /* Cast to gain module capi struct*/
   capi_example_gain_module_t *me_ptr = (capi_example_gain_module_t *)_pif;

   memset(me_ptr, 0, sizeof(capi_example_gain_module_t));

   /* Assign function table defined above*/
   me_ptr->vtbl.vtbl_ptr = &vtbl;

   /* Initialize the operating media format of the gain module*/
   capi_example_gain_module_module_init_media_fmt_v2(&me_ptr->operating_media_fmt);

   /* Initialize gain configuration to defaults*/
   uint16_t apply_gain          = 0x2000;
   me_ptr->gain_config.gain_q12 = apply_gain >> 1;
   me_ptr->gain_config.gain_q13 = apply_gain;

   if (NULL != init_set_properties)
   {
      capi_result = capi_example_gain_module_process_set_properties(me_ptr, init_set_properties);
      if (CAPI_FAILED(capi_result))
      {
         AR_MSG(DBG_ERROR_PRIO, "capi_example_gain_module:  Initialization Set Property Failed");
         return capi_result;
      }
   }

   AR_MSG(DBG_HIGH_PRIO, "capi_example_gain_module: Initialization completed !!");
   return capi_result;
}

/* -------------------------------------------------------------------------
 * Function name: capi_example_gain_module_process
 * Gain module Data Process function to process an input buffer
 * and fill an output buffer.
 * -------------------------------------------------------------------------*/
static capi_err_t capi_example_gain_module_process(capi_t *_pif, capi_stream_data_t *input[], capi_stream_data_t *output[])
{
   capi_err_t          capi_result = CAPI_EOK;
   capi_example_gain_module_t *me_ptr      = (capi_example_gain_module_t *)_pif;

   if ((!input[0]) && (!output[0]))
   {
      AR_MSG(DBG_ERROR_PRIO, "capi_example_gain_module: Process: No input or output buffer provided");
      return CAPI_EFAILED;
   }

   /* Call library function to process data*/
   uint32_t samples_to_process;
   int32_t  byte_sample_convert = (BIT_WIDTH_16 == me_ptr->operating_media_fmt.format.bits_per_sample) ? 1 : 2;

   int32_t inp_samples = input[0]->buf_ptr[0].actual_data_len >> byte_sample_convert;
   int32_t out_samples = output[0]->buf_ptr[0].max_data_len >> byte_sample_convert;

   /* Samples to process is the minimum of the input and output lengths available*/
   samples_to_process = (inp_samples < out_samples) ? inp_samples : out_samples;

   for (uint32_t ch = 0; ch < me_ptr->operating_media_fmt.format.num_channels; ch++)
   {
      if (BIT_WIDTH_16 == me_ptr->operating_media_fmt.format.bits_per_sample)
      {
         example_apply_gain_16((int16_t *)output[0]->buf_ptr[ch].data_ptr,
                               (int16_t *)input[0]->buf_ptr[ch].data_ptr,
                               me_ptr->gain_config.gain_q12,
                               samples_to_process);
         output[0]->buf_ptr[ch].actual_data_len = samples_to_process << 1;
         input[0]->buf_ptr[ch].actual_data_len  = samples_to_process << 1;
      }
      else
      {
         if ((uint16_t)(me_ptr->gain_config.gain_q12 >> 12) > 0)
         {
            // for gain greater than 1
            example_apply_gain_32_G1((int32_t *)output[0]->buf_ptr[ch].data_ptr,
                                     (int32_t *)input[0]->buf_ptr[ch].data_ptr,
                                     me_ptr->gain_config.gain_q12,
                                     samples_to_process);
         }
         else
         {
            // for gain less than 1
            example_apply_gain_32_L1((int32_t *)output[0]->buf_ptr[ch].data_ptr,
                                     (int32_t *)input[0]->buf_ptr[ch].data_ptr,
                                     me_ptr->gain_config.gain_q12,
                                     samples_to_process);
         }
         output[0]->buf_ptr[ch].actual_data_len = samples_to_process << 2;
         input[0]->buf_ptr[ch].actual_data_len  = samples_to_process << 2;
      }
   }

   return capi_result;
}

/*------------------------------------------------------------------------
 * Function name: capi_example_gain_module_end
 * Gain End function, returns the library to the uninitialized
 * state and frees all the memory that was allocated. This function also
 * frees the virtual function table.
 * -----------------------------------------------------------------------*/
static capi_err_t capi_example_gain_module_end(capi_t *_pif)
{

   capi_err_t capi_result = CAPI_EOK;
   if (NULL == _pif)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi_example_gain_module: End received bad pointer, 0x%p", _pif);
      CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      return capi_result;
   }

   capi_example_gain_module_t *me_ptr = (capi_example_gain_module_t *)_pif;
   me_ptr->vtbl.vtbl_ptr      = NULL;

   AR_MSG(DBG_HIGH_PRIO, "capi_example_gain_module: End done");
   return capi_result;
}

/* -------------------------------------------------------------------------
 * Function name: capi_example_gain_module_set_param
 * Sets either a parameter value or a parameter structure containing
 * multiple parameters. In the event of a failure, the appropriate error
 * code is returned.
 * The actual_data_len field of the parameter pointer is to be at least the size
  of the parameter structure. Every case statement should have this check
 * -------------------------------------------------------------------------*/
static capi_err_t capi_example_gain_module_set_param(capi_t *                _pif,
                                             uint32_t                param_id,
                                             const capi_port_info_t *port_info_ptr,
                                             capi_buf_t *            params_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   if (NULL == _pif || NULL == params_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi_example_gain_module: set param received NULL pointers");
      return CAPI_EBADPARAM;
   }

   capi_example_gain_module_t *me_ptr = (capi_example_gain_module_t *)(_pif);

   switch (param_id)
   {
      /* parameter to enable the gain module */
      case PARAM_ID_MODULE_ENABLE:
      {
         if (params_ptr->actual_data_len >= sizeof(param_id_module_enable_t))
         {
            param_id_module_enable_t *gain_module_enable_ptr = (param_id_module_enable_t *)(params_ptr->data_ptr);
            me_ptr->gain_config.enable                       = gain_module_enable_ptr->enable;

            /* Raise process check event based on enable value set here*/
            capi_result |= capi_example_gain_module_raise_process_event(me_ptr, me_ptr->gain_config.enable);

            AR_MSG(DBG_HIGH_PRIO, "capi_example_gain_module: PARAM_ID_MODULE_ENABLE, %lu", me_ptr->gain_config.enable);
         }
         else
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "capi_example_gain_module: PARAM_ID_MODULE_ENABLE, Bad param size %lu",
                   params_ptr->actual_data_len);
            capi_result |= CAPI_ENEEDMORE;
         }
         break;
      }
      case PARAM_ID_GAIN_MODULE_GAIN:
      {
         if (params_ptr->actual_data_len >= sizeof(param_id_module_gain_cfg_t))
         {
            param_id_module_gain_cfg_t *gain_cfg_ptr = (param_id_module_gain_cfg_t *)(params_ptr->data_ptr);
            me_ptr->gain_config.gain_q13             = gain_cfg_ptr->gain;
            me_ptr->gain_config.gain_q12             = gain_cfg_ptr->gain >> 1;

            /* Determine if gain module should be enabled
             * Module is enabled if PARAM_ID_MODULE_ENABLE was set to enable AND
             * if the gain value is not unity in Q13 format */
            uint32_t enable = TRUE;

            if (Q13_UNITY_GAIN == me_ptr->gain_config.gain_q12 << 1)
            {
               enable = FALSE;
            }
            enable = enable && ((me_ptr->gain_config.enable == 0) ? 0 : 1);

            /* Raise process check event*/
            capi_result |= capi_example_gain_module_raise_process_event(me_ptr, enable);

            AR_MSG(DBG_HIGH_PRIO, "capi_example_gain_module: PARAM_ID_GAIN %d", me_ptr->gain_config.gain_q13);
         }
         else
         {
            AR_MSG(DBG_ERROR_PRIO, "capi_example_gain_module: Set, Bad param size %lu", params_ptr->actual_data_len);
            CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
         }
         break;
      }
      default:
      {
         AR_MSG(DBG_ERROR_PRIO, "capi_example_gain_module: Set, unsupported param ID 0x%x", (int)param_id);
         CAPI_SET_ERROR(capi_result, CAPI_EUNSUPPORTED);
         break;
      }
   }
   return capi_result;
}

/* -------------------------------------------------------------------------
 * Function name: capi_example_gain_module_get_param
 * Gets either a parameter value or a parameter structure containing
 * multiple parameters. In the event of a failure, the appropriate error
 * code is returned.
 * The max_data_len field of the parameter pointer must be at least the size
 * of the parameter structure. Therefore, this check is made in every case statement
 * Before returning, the actual_data_len field must be filled with the number
  of bytes written into the buffer.
 * -------------------------------------------------------------------------*/
static capi_err_t capi_example_gain_module_get_param(capi_t *                _pif,
                                                     uint32_t                param_id,
                                                     const capi_port_info_t *port_info_ptr,
                                                     capi_buf_t *            params_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   if (NULL == _pif || NULL == params_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi_example_gain_module: Get param received bad pointer, 0x%p, 0x%p", _pif, params_ptr);
      return CAPI_EBADPARAM;
   }

   capi_example_gain_module_t *me_ptr = (capi_example_gain_module_t *)_pif;

   switch (param_id)
   {
      case PARAM_ID_MODULE_ENABLE:
      {
         if (params_ptr->max_data_len >= sizeof(param_id_module_enable_t))
         {
            param_id_module_enable_t *enable_ptr = (param_id_module_enable_t *)(params_ptr->data_ptr);
            enable_ptr->enable                   = me_ptr->gain_config.enable;

            /* Populate actual data length*/
            params_ptr->actual_data_len = (uint32_t)sizeof(param_id_module_enable_t);
         }
         else
         {
            AR_MSG(DBG_ERROR_PRIO, "capi_example_gain_module: Get Enable Param, Bad payload size %d", params_ptr->max_data_len);
            capi_result = CAPI_ENEEDMORE;
         }
         break;
      }
      case PARAM_ID_GAIN_MODULE_GAIN:
      {
         if (params_ptr->max_data_len >= sizeof(param_id_module_gain_cfg_t))
         {
            param_id_module_gain_cfg_t *gain_cfg_ptr = (param_id_module_gain_cfg_t *)(params_ptr->data_ptr);

            /* Fetch the gain values which were set to the lib*/
            gain_cfg_ptr->gain     = me_ptr->gain_config.gain_q13;
            gain_cfg_ptr->reserved = 0;

            /* Populate actual data length*/
            params_ptr->actual_data_len = sizeof(param_id_module_gain_cfg_t);
         }
         else
         {
            AR_MSG(DBG_ERROR_PRIO, "capi_example_gain_module: Get, Bad param size %lu", params_ptr->max_data_len);
            capi_result = CAPI_ENEEDMORE;
         }
         break;
      }
      default:
      {
         AR_MSG(DBG_ERROR_PRIO, "capi_example_gain_module: Get, unsupported param ID 0x%x", param_id);
         capi_result |= CAPI_EUNSUPPORTED;
         break;
      }
   }
   return capi_result;
}

/* -------------------------------------------------------------------------
 * Function name: capi_example_gain_module_set_properties
 * Function to set Sets a list of property values.
 * -------------------------------------------------------------------------*/
static capi_err_t capi_example_gain_module_set_properties(capi_t *_pif, capi_proplist_t *props_ptr)
{
   capi_example_gain_module_t *me_ptr = (capi_example_gain_module_t *)_pif;
   return capi_example_gain_module_process_set_properties(me_ptr, props_ptr);
}

/* -------------------------------------------------------------------------
 * Function name: capi_example_gain_module_get_properties
 * Function to get a list of property values.
 * -------------------------------------------------------------------------*/
static capi_err_t capi_example_gain_module_get_properties(capi_t *_pif, capi_proplist_t *props_ptr)
{
   capi_example_gain_module_t *me_ptr = (capi_example_gain_module_t *)_pif;
   return capi_example_gain_module_process_get_properties(me_ptr, props_ptr);
}
