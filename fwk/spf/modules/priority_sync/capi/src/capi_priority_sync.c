/**
 * \file capi_priority_sync.c
 * \brief
 *       Implement capi API functions and capi structure setup/teardown.
 *
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "capi_priority_sync_i.h"
#include "module_cmn_api.h"

/* =========================================================================
 * Static function declarations
 * =========================================================================*/

static capi_err_t capi_priority_sync_init_in_port(capi_priority_sync_t *        me_ptr,
                                                  capi_priority_sync_in_port_t *in_port_ptr);

static inline capi_err_t capi_priority_sync_init_out_port(capi_priority_sync_t *         me_ptr,
                                                          capi_priority_sync_out_port_t *out_port_ptr);

static capi_err_t capi_priority_sync_process(capi_t *_pif, capi_stream_data_t *input[], capi_stream_data_t *output[]);

static capi_err_t capi_priority_sync_end(capi_t *_pif);

capi_err_t capi_priority_sync_set_param(capi_t *                _pif,
                                        uint32_t                param_id,
                                        const capi_port_info_t *port_info_ptr,
                                        capi_buf_t *            params_ptr);

capi_err_t capi_priority_sync_get_param(capi_t *                _pif,
                                        uint32_t                param_id,
                                        const capi_port_info_t *port_info_ptr,
                                        capi_buf_t *            params_ptr);

static capi_err_t capi_priority_sync_set_properties(capi_t *_pif, capi_proplist_t *props_ptr);

static capi_err_t capi_priority_sync_get_properties(capi_t *_pif, capi_proplist_t *props_ptr);

static capi_vtbl_t vtbl = { capi_priority_sync_process,        capi_priority_sync_end,
                            capi_priority_sync_set_param,      capi_priority_sync_get_param,
                            capi_priority_sync_set_properties, capi_priority_sync_get_properties };

/* =========================================================================
 * Function definitions
 * =========================================================================*/

/**
 * capi priority sync function to get the static properties
 */
capi_err_t capi_priority_sync_get_static_properties(capi_proplist_t *init_set_properties,
                                                    capi_proplist_t *static_properties)
{
   capi_err_t capi_result = CAPI_EOK;

   if (NULL != static_properties)
   {
      capi_result = capi_priority_sync_process_get_properties((capi_priority_sync_t *)NULL, static_properties);
      if (CAPI_FAILED(capi_result))
      {
         PS_MSG(MIID_UNKNOWN, DBG_ERROR_PRIO, "CAPI priority sync: get static properties failed!");
         return capi_result;
      }
   }
   else
   {
      PS_MSG(MIID_UNKNOWN,
             DBG_ERROR_PRIO,
             "CAPI priority sync: Get static properties received bad pointer, 0x%p",
             static_properties);
   }

   return capi_result;
}

/**
 * Initialize priority sync common port. Ports are closed until opened.
 */
capi_err_t capi_priority_sync_init_cmn_port(capi_priority_sync_t *me_ptr, capi_priority_sync_cmn_port_t *cmn_port_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   if (NULL == me_ptr || NULL == cmn_port_ptr)
   {
      PS_MSG(MIID_UNKNOWN,
             DBG_ERROR_PRIO,
             "capi priority sync: Init received bad pointer, 0x%p, 0x%p",
             me_ptr,
             cmn_port_ptr);
      CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      return capi_result;
   }

   cmn_port_ptr->index = PRIORITY_SYNC_PORT_INDEX_INVALID;
   cmn_port_ptr->state = DATA_PORT_STATE_CLOSED;

   cmn_port_ptr->prop_state.ds_rt = PRIORITY_SYNC_FTRT;
   cmn_port_ptr->prop_state.us_rt = PRIORITY_SYNC_FTRT;

   return capi_result;
}

/**
 * Initialize priority sync input port.
 */
static capi_err_t capi_priority_sync_init_in_port(capi_priority_sync_t *        me_ptr,
                                                  capi_priority_sync_in_port_t *in_port_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   if (NULL == me_ptr || NULL == in_port_ptr)
   {
      PS_MSG(MIID_UNKNOWN,
             DBG_ERROR_PRIO,
             "capi priority sync: Init received bad pointer, 0x%p, 0x%p",
             me_ptr,
             in_port_ptr);
      CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      return capi_result;
   }

   memset(&in_port_ptr->int_stream, 0, sizeof(in_port_ptr->int_stream));
   capi_result |= capi_cmn_init_media_fmt_v2(&(in_port_ptr->media_fmt));
   capi_result |= capi_priority_sync_init_cmn_port(me_ptr, &(in_port_ptr->cmn));

   return capi_result;
}

/**
 * Initialize priority sync output port.
 */
static inline capi_err_t capi_priority_sync_init_out_port(capi_priority_sync_t *         me_ptr,
                                                          capi_priority_sync_out_port_t *out_port_ptr)
{
   return capi_priority_sync_init_cmn_port(me_ptr, &(out_port_ptr->cmn));
}

/**
 * Initialize the CAPIv2 PRIORITY Sync Module. This function can allocate memory.
 */
capi_err_t capi_priority_sync_init(capi_t *_pif, capi_proplist_t *init_set_properties)
{
   capi_err_t capi_result = CAPI_EOK;

   if (NULL == _pif || NULL == init_set_properties)
   {
      PS_MSG(MIID_UNKNOWN,
             DBG_ERROR_PRIO,
             "capi priority sync: Init received bad pointer, 0x%p, 0x%p",
             _pif,
             init_set_properties);

      CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      return capi_result;
   }

   capi_priority_sync_t *me_ptr = (capi_priority_sync_t *)_pif;

   memset(me_ptr, 0, sizeof(capi_priority_sync_t));

   me_ptr->vtbl.vtbl_ptr = &vtbl;

   capi_result |= capi_priority_sync_init_in_port(me_ptr, &me_ptr->primary_in_port_info);
   capi_result |= capi_priority_sync_init_out_port(me_ptr, &me_ptr->primary_out_port_info);
   capi_result |= capi_priority_sync_init_in_port(me_ptr, &me_ptr->secondary_in_port_info);
   capi_result |= capi_priority_sync_init_out_port(me_ptr, &me_ptr->secondary_out_port_info);

   if (CAPI_FAILED(capi_result))
   {
      PS_MSG(me_ptr->miid, DBG_ERROR_PRIO, "capi priority sync: Initialization Failed");
      return capi_result;
   }

   // Start out with ports closed. State is interpreted as synced until second port is opened.
   me_ptr->synced_state          = PRIORITY_SYNC_STATE_SYNCED;
   me_ptr->threshold_is_disabled = FALSE; // Begins with threshold enabled which is default container behavior.

   if (NULL != init_set_properties)
   {
      capi_result = capi_priority_sync_process_set_properties(me_ptr, init_set_properties);
      if (CAPI_FAILED(capi_result))
      {
         PS_MSG(me_ptr->miid, DBG_ERROR_PRIO, "capi priority sync:  Initialization Set Property Failed");
         return capi_result;
      }
   }

   return capi_result;
}

/**
 * PRIORITY Sync module data process function to process an input buffer
 * and generates an output buffer.
 */
static capi_err_t capi_priority_sync_process(capi_t *_pif, capi_stream_data_t *input[], capi_stream_data_t *output[])
{
   capi_err_t            capi_result = CAPI_EOK;
   capi_priority_sync_t *me_ptr      = (capi_priority_sync_t *)_pif;
   POSAL_ASSERT(me_ptr);

   if (me_ptr->is_ts_based_sync)
   {
      capi_result = priority_ts_sync_process(_pif, input, output);
   }
   else
   {
      capi_result = priority_sync_process(_pif, input, output);
   }
   return capi_result;
}

/**
 * PRIORITY Sync end function, returns the library to the uninitialized
 * state and frees all the memory that was allocated. This function also
 * frees the virtual function table.
 */
static capi_err_t capi_priority_sync_end(capi_t *_pif)
{
   bool_t     PRIMARY_PATH   = TRUE;
   bool_t     SECONDARY_PATH = FALSE;
   capi_err_t capi_result    = CAPI_EOK;
   if (NULL == _pif)
   {
      PS_MSG(MIID_UNKNOWN, DBG_ERROR_PRIO, "capi priority sync: End received bad pointer, 0x%p", _pif);
      CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      return capi_result;
   }

   capi_priority_sync_t *me_ptr = (capi_priority_sync_t *)_pif;

   // In case an explicit close port operation was not sent (can happen for internal ports).
   capi_priority_sync_clear_buffered_data(me_ptr, PRIMARY_PATH);
   capi_priority_sync_clear_buffered_data(me_ptr, SECONDARY_PATH);
   capi_priority_sync_deallocate_port_buffer(me_ptr, &(me_ptr->primary_in_port_info));
   capi_priority_sync_deallocate_port_buffer(me_ptr, &(me_ptr->secondary_in_port_info));

   me_ptr->vtbl.vtbl_ptr = NULL;

   PS_MSG(me_ptr->miid, DBG_HIGH_PRIO, "capi priority sync: End done");
   return capi_result;
}

static capi_err_t capi_priority_sync_set_properties(capi_t *_pif, capi_proplist_t *props_ptr)
{
   capi_priority_sync_t *me_ptr = (capi_priority_sync_t *)_pif;
   return capi_priority_sync_process_set_properties(me_ptr, props_ptr);
}

/**
 * Function to get the properties of PRIORITY Sync module
 */
static capi_err_t capi_priority_sync_get_properties(capi_t *_pif, capi_proplist_t *props_ptr)
{
   capi_priority_sync_t *me_ptr = (capi_priority_sync_t *)_pif;
   return capi_priority_sync_process_get_properties(me_ptr, props_ptr);
}
