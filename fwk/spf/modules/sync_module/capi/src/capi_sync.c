/**
 * \file capi_sync.c
 * \brief
 *       Implement capi API functions and capi structure setup/teardown.
 *  
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "capi_sync_i.h"
#include "module_cmn_api.h"
#include "sync_api.h"

/* =========================================================================
 * Static function declarations
 * =========================================================================*/
static capi_err_t capi_sync_cmn_init(capi_t *_pif, capi_proplist_t *init_set_properties, capi_sync_mode_t mode);

static capi_err_t capi_sync_process(capi_t *_pif, capi_stream_data_t *input[], capi_stream_data_t *output[]);

static capi_err_t capi_sync_end(capi_t *_pif);

static capi_err_t capi_sync_set_param(capi_t *                _pif,
                                      uint32_t                param_id,
                                      const capi_port_info_t *port_info_ptr,
                                      capi_buf_t *            params_ptr);

static capi_err_t capi_sync_get_param(capi_t *                _pif,
                                      uint32_t                param_id,
                                      const capi_port_info_t *port_info_ptr,
                                      capi_buf_t *            params_ptr);

static capi_err_t capi_sync_set_properties(capi_t *_pif, capi_proplist_t *props_ptr);

static capi_err_t capi_sync_get_properties(capi_t *_pif, capi_proplist_t *props_ptr);

static capi_vtbl_t vtbl = { capi_sync_process,        capi_sync_end,           capi_sync_set_param, capi_sync_get_param,
                            capi_sync_set_properties, capi_sync_get_properties };

/* =========================================================================
 * Function definitions
 * =========================================================================*/

/**
 * Initialize the CAPIv2 Sync Module.
 */
capi_err_t capi_sync_init(capi_t *_pif, capi_proplist_t *init_set_properties)
{
   return capi_sync_cmn_init(_pif, init_set_properties, MODE_ALL_EQUAL_PRIO_INPUT);
}

/**
 * capi sync function to get the static properties
 */
capi_err_t capi_sync_get_static_properties(capi_proplist_t *init_set_properties, capi_proplist_t *static_properties)
{
   capi_err_t capi_result = CAPI_EOK;

   AR_MSG(DBG_HIGH_PRIO, "Enter get static properties");
   if (NULL != static_properties)
   {
      capi_result = capi_sync_process_get_properties((capi_sync_t *)NULL, static_properties);
      if (CAPI_FAILED(capi_result))
      {
         AR_MSG(DBG_ERROR_PRIO, "CAPI sync: get static properties failed!");
         return capi_result;
      }
   }
   else
   {
      AR_MSG(DBG_ERROR_PRIO, "CAPI sync: Get static properties received bad pointer, 0x%p", static_properties);
   }

   return capi_result;
}

/**
 * Initialize the CAPIv2 EC Sync Module.
 */
capi_err_t capi_ec_sync1_init(capi_t *_pif, capi_proplist_t *init_set_properties)
{
   return capi_sync_cmn_init(_pif, init_set_properties, MODE_EC_PRIO_INPUT);
}

/**
 * capi sync function to get the static properties for EC sync module
 */
capi_err_t capi_ec_sync1_get_static_properties(capi_proplist_t *init_set_properties, capi_proplist_t *static_properties)
{
   return capi_sync_get_static_properties(init_set_properties, static_properties);
}

/**
 * Common Init Function used by the Sync Module for both Generic & EC Sync Modes
 */
static capi_err_t capi_sync_cmn_init(capi_t *_pif, capi_proplist_t *init_set_properties, capi_sync_mode_t mode)
{
   capi_err_t capi_result = CAPI_EOK;

   if (NULL == _pif || NULL == init_set_properties)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi sync: Init received bad pointer, 0x%p, 0x%p", _pif, init_set_properties);

      CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      return capi_result;
   }

   capi_sync_t *me_ptr = (capi_sync_t *)_pif;

   memset(me_ptr, 0, sizeof(capi_sync_t));

   me_ptr->vtbl.vtbl_ptr = &vtbl;

   capi_sync_init_config(me_ptr, mode);

   if (NULL != init_set_properties)
   {
      capi_result = capi_sync_process_set_properties(me_ptr, init_set_properties);
      if (CAPI_FAILED(capi_result))
      {
         AR_MSG(DBG_ERROR_PRIO, "capi sync:  Initialization Set Property Failed");
         return capi_result;
      }
   }

   capi_result |= capi_sync_raise_event(me_ptr);

   AR_MSG(DBG_HIGH_PRIO, "capi sync: Initialization completed !!");
   return capi_result;
}

/**
 * Set Param function for the Sync Module
 */
static capi_err_t capi_sync_set_param(capi_t *                _pif,
                                      uint32_t                param_id,
                                      const capi_port_info_t *port_info_ptr,
                                      capi_buf_t *            params_ptr)
{
   capi_sync_t *me_ptr = (capi_sync_t *)_pif;
   return capi_sync_process_set_param(me_ptr, param_id, port_info_ptr, params_ptr);
}

/**
 * Get Param function for the Sync Module
 */
static capi_err_t capi_sync_get_param(capi_t *                _pif,
                                      uint32_t                param_id,
                                      const capi_port_info_t *port_info_ptr,
                                      capi_buf_t *            params_ptr)
{
   capi_sync_t *me_ptr = (capi_sync_t *)_pif;
   return capi_sync_process_get_param(me_ptr, param_id, port_info_ptr, params_ptr);
}

/**
 * Function to set the properties of Sync module
 */
static capi_err_t capi_sync_set_properties(capi_t *_pif, capi_proplist_t *props_ptr)
{
   capi_sync_t *me_ptr = (capi_sync_t *)_pif;
   return capi_sync_process_set_properties(me_ptr, props_ptr);
}

/**
 * Function to get the properties of Sync module
 */
static capi_err_t capi_sync_get_properties(capi_t *_pif, capi_proplist_t *props_ptr)
{
   capi_sync_t *me_ptr = (capi_sync_t *)_pif;
   return capi_sync_process_get_properties(me_ptr, props_ptr);
}

/**
 * Sync module data process function to process an input buffer
 * and generates an output buffer.
 */
static capi_err_t capi_sync_process(capi_t *_pif, capi_stream_data_t *input[], capi_stream_data_t *output[])
{
   capi_err_t   capi_result = CAPI_EOK;
   capi_sync_t *me_ptr      = (capi_sync_t *)_pif;
   POSAL_ASSERT(me_ptr);

   capi_result = sync_module_process(me_ptr, input, output);

   return capi_result;
}

/**
 * End function of the Sync Module.
 * Frees all the memory that was allocated and the virtual function table.
 */
static capi_err_t capi_sync_end(capi_t *_pif)
{

   capi_err_t capi_result = CAPI_EOK;
   if (NULL == _pif)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi sync: End received bad pointer, 0x%p", _pif);
      CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      return capi_result;
   }

   capi_sync_t *me_ptr = (capi_sync_t *)_pif;

   if (me_ptr->in_port_info_ptr)
   {
      // In case an explicit close port operation was not sent (can happen for internal ports).
      for (uint32_t i = 0; i < me_ptr->num_in_ports; i++)
      {
         capi_sync_in_port_t *in_port_ptr = &(me_ptr->in_port_info_ptr[i]);

         capi_sync_clear_buffered_data(me_ptr, in_port_ptr);
         capi_sync_deallocate_port_buffer(me_ptr, in_port_ptr);
      }

      posal_memory_free(me_ptr->in_port_info_ptr);
   }

   if (me_ptr->out_port_info_ptr)
   {
      posal_memory_free(me_ptr->out_port_info_ptr);
   }

   me_ptr->vtbl.vtbl_ptr = NULL;

   AR_MSG(DBG_HIGH_PRIO, "capi sync: End done");
   return capi_result;
}
