/**
 * \file example_encoder_module.c
 *  
 * \brief
 *  
 *     Example Encoder Module
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/*------------------------------------------------------------------------
 * Include files
 * -----------------------------------------------------------------------*/
#include "capi_example_enc_module_structs.h"

/*------------------------------------------------------------------------
 Static function declaration
 * -----------------------------------------------------------------------*/
static capi_err_t capi_example_enc_module_process(capi_t *            _pif,
                                                        capi_stream_data_t *input[],
                                                        capi_stream_data_t *output[]);
static capi_err_t capi_example_enc_module_end(capi_t *_pif);
static capi_err_t capi_example_enc_module_set_param(capi_t *                _pif,
                                                          uint32_t                   param_id,
                                                          const capi_port_info_t *port_info_ptr,
                                                          capi_buf_t *            params_ptr);
static capi_err_t capi_example_enc_module_get_param(capi_t *                _pif,
                                                          uint32_t                   param_id,
                                                          const capi_port_info_t *port_info_ptr,
                                                          capi_buf_t *            params_ptr);
static capi_err_t capi_example_enc_module_set_properties(capi_t *_pif, capi_proplist_t *proplist_ptr);
static capi_err_t capi_example_enc_module_get_properties(capi_t *_pif, capi_proplist_t *proplist_ptr);

static capi_vtbl_t vtbl = { capi_example_enc_module_process,        capi_example_enc_module_end,
                               capi_example_enc_module_set_param,      capi_example_enc_module_get_param,
                               capi_example_enc_module_set_properties, capi_example_enc_module_get_properties };

/* -------------------------------------------------------------------------
 * Function name: capi_example_enc_module_get_static_properties
 * Used to query the static properties of the example enc module that are independent of the
   instance. This function is used to query the memory requirements of the module
   in order to create an instance.
 * -------------------------------------------------------------------------*/
capi_err_t capi_example_enc_module_get_static_properties(capi_proplist_t *init_set_properties,
                                                               capi_proplist_t *static_properties)
{
   capi_err_t result = CAPI_EOK;

   if (NULL != static_properties)
   {
      result =
         capi_example_enc_module_process_get_properties((capi_example_enc_module_t *)NULL, static_properties);
      if (result != CAPI_EOK)
      {
         return result;
      }
   }

   if (NULL != init_set_properties)
   {
      // ignore currently.
   }

   return result;
}

/*------------------------------------------------------------------------
  Function name: capi_example_enc_module_init
  Instantiates the example enc module to set up the virtual function table, and also
  allocates any memory required by the module. Default states within the module are also
  initialized here
 * -----------------------------------------------------------------------*/
capi_err_t capi_example_enc_module_init(capi_t *_pif, capi_proplist_t *init_set_properties)
{
   capi_err_t result = CAPI_EOK;
   AR_MSG(DBG_HIGH_PRIO, "capi_example_enc_module: Init done");
   if (!_pif)
   {
      return CAPI_EBADPARAM;
   }
   AR_MSG(DBG_HIGH_PRIO, "capi_example_enc_module: Init done");
   capi_example_enc_module_t *me_ptr = (capi_example_enc_module_t *)_pif;

   /* Assign function table defined above*/
   me_ptr->vtbl.vtbl_ptr = &vtbl;

   /* should contain EVENT_CALLBACK_INFO, PORT_INFO */
   if (NULL != init_set_properties)
   {
      result = capi_example_enc_module_process_set_properties(me_ptr, init_set_properties);
      if ((CAPI_EOK == result) || (CAPI_EUNSUPPORTED == result))
      {
         return CAPI_EOK;
      }
      else
      {
         return result;
      }
   }
   AR_MSG(DBG_HIGH_PRIO, "capi_example_enc_module: Init done");

   /* Call library init with defaults if any */
   example_enc_init(EXAMPLE_ENC_DEFAULT_ABC, EXAMPLE_ENC_DEFAULT_XYZ);

   /* Update kpps and bw*/
   result |= capi_example_enc_module_raise_kpps_event(me_ptr, CAPI_EXAMPLE_ENC_DEFAULT_KPPS);
   result |= capi_example_enc_module_raise_bandwidth_event(me_ptr,
                                                              CAPI_EXAMPLE_ENC_DEFAULT_CODE_BW,
                                                              CAPI_EXAMPLE_ENC_DEFAULT_DATA_BW);

   me_ptr->vtbl.vtbl_ptr = &vtbl;
   AR_MSG(DBG_HIGH_PRIO, "capi_example_enc_module: Init done");
   return result;
}

/* -------------------------------------------------------------------------
 * Function name: capi_example_enc_module_process
 * Processes input data and provides output for all input and output ports.
 * -------------------------------------------------------------------------*/
static capi_err_t capi_example_enc_module_process(capi_t *            _pif,
                                                        capi_stream_data_t *input[],
                                                        capi_stream_data_t *output[])
{
   if (!_pif)
   {
      return CAPI_EBADPARAM;
   }

   capi_err_t                 result = CAPI_EOK;
   capi_example_enc_module_t *me_ptr = (capi_example_enc_module_t *)_pif;

   /* Calculate how many byes will be consumed this process call depending on media format and size of input received*/
   uint32_t byte_sample_convert = (BIT_WIDTH_16 == me_ptr->input_media_format.format.bits_per_sample) ? 1 : 2;
   uint32_t inp_bytes           = input[0]->buf_ptr[0].actual_data_len >> byte_sample_convert;
   uint32_t out_bytes           = output[0]->buf_ptr[0].max_data_len >> byte_sample_convert;

   uint32_t nSampleCnt = (inp_bytes < out_bytes) ? inp_bytes : out_bytes;

   /* Call library process function with input and output buffers */
   example_enc_process(output[0]->buf_ptr[0].data_ptr, input[0]->buf_ptr[0].data_ptr, nSampleCnt);

   /* Update output actual data length based on what is produced */
   output[0]->buf_ptr[0].actual_data_len = nSampleCnt << byte_sample_convert;

   return result;
}

/*------------------------------------------------------------------------
 * Function name: capi_example_enc_module_end
 * Example encoder end function, returns the library to the uninitialized
 * state and frees all the memory that was allocated. This function also
 * frees the virtual function table.
 * -----------------------------------------------------------------------*/
static capi_err_t capi_example_enc_module_end(capi_t *_pif)
{
   if (!_pif)
      return CAPI_EOK;

   capi_example_enc_module_t *me_ptr = (capi_example_enc_module_t *)_pif;

   /* Call library deinit function */
   example_enc_deinit();

   me_ptr->capi_init_done = FALSE;
   me_ptr->vtbl.vtbl_ptr  = NULL;

   AR_MSG(DBG_HIGH_PRIO, "capi_example_enc_module: End done");

   return CAPI_EOK;
}

/* -------------------------------------------------------------------------
 * Function name: capi_example_enc_module_set_param
 * Sets either a parameter value or a parameter structure containing
 * multiple parameters. In the event of a failure, the appropriate error
 * code is returned.
 * The actual_data_len field of the parameter pointer is to be at least the size
  of the parameter structure. Every case statement should have this check
 * -------------------------------------------------------------------------*/
static capi_err_t capi_example_enc_module_set_param(capi_t *                _pif,
                                                          uint32_t                   param_id,
                                                          const capi_port_info_t *port_info_ptr,
                                                          capi_buf_t *            params_ptr)
{
   capi_example_enc_module_t *me_ptr = (capi_example_enc_module_t *)_pif;
   capi_err_t                 result = CAPI_EOK;
   AR_MSG(DBG_HIGH_PRIO, "capi_example_enc_module: Set param received param id 0x%lx ",param_id);

   switch (param_id)
   {
      case PARAM_ID_ENCODER_OUTPUT_CONFIG:
      {
         result = capi_example_enc_module_set_enc_cfg_blk(me_ptr, params_ptr);
         break;
      }
      case PARAM_ID_ENC_BITRATE:
      {
         result = capi_example_enc_module_set_bit_rate(me_ptr, params_ptr);
         break;
      }
      default:
      {
         AR_MSG(DBG_ERROR_PRIO, "capi_example_enc_module: Set param for 0x%lx not supported", param_id);
         result = CAPI_EUNSUPPORTED;
         break;
      }
   }

   AR_MSG(DBG_HIGH_PRIO, "capi_example_enc_module: Set param for 0x%lx done", param_id);

   return result;
}

/* -------------------------------------------------------------------------
 * Function name: capi_example_enc_module_get_param
 * Gets either a parameter value or a parameter structure containing
 * multiple parameters. In the event of a failure, the appropriate error
 * code is returned.
 * The max_data_len field of the parameter pointer must be at least the size
 * of the parameter structure. Therefore, this check is made in every case statement
 * Before returning, the actual_data_len field must be filled with the number
  of bytes written into the buffer.
 * -------------------------------------------------------------------------*/
static capi_err_t capi_example_enc_module_get_param(capi_t *                _pif,
                                                          uint32_t                   param_id,
                                                          const capi_port_info_t *port_info_ptr,
                                                          capi_buf_t *            params_ptr)
{
   capi_err_t result = CAPI_EOK;

   switch (param_id)
   {
      default:
      {
         AR_MSG(DBG_ERROR_PRIO, "capi_example_enc_module: Get param for 0x%lx not supported", param_id);
         result = CAPI_EUNSUPPORTED;
         // according to CAPI V2, actual len should be num of bytes read(set)/written(get)
         params_ptr->actual_data_len = 0;
         break;
      }
   }

   return result;
}

/* -------------------------------------------------------------------------
 * Function name: capi_example_enc_module_set_properties
 * This function is called only for setting runtime properties. Currently
 * only runtime property handled by this module is
 *     1. Media format on input ports.
 *     2. Perform algorithmic reset during module STOP/START sequence.
 * -------------------------------------------------------------------------*/
static capi_err_t capi_example_enc_module_set_properties(capi_t *_pif, capi_proplist_t *proplist_ptr)
{
   if (!_pif)
   {
      return CAPI_EBADPARAM;
   }

   capi_example_enc_module_t *me_ptr = (capi_example_enc_module_t *)_pif;

   return capi_example_enc_module_process_set_properties(me_ptr, proplist_ptr);
}

/* -------------------------------------------------------------------------
 * Function name: capi_example_enc_module_get_properties
 * Returns module properties during runtime. Currently the only runtime
 * property thats supported is get output media format. Framework can query media
 * format using this function during runtime [This will happen only if the
 * module doesn't raise output media format].
 * -------------------------------------------------------------------------*/
static capi_err_t capi_example_enc_module_get_properties(capi_t *_pif, capi_proplist_t *proplist_ptr)
{
   if (!_pif)
   {
      return CAPI_EBADPARAM;
   }

   capi_example_enc_module_t *me_ptr = (capi_example_enc_module_t *)_pif;

   return capi_example_enc_module_process_get_properties(me_ptr, proplist_ptr);
}
