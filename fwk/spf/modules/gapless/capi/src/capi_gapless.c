/**
 * \file capi_gapless.c
 * \brief
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "capi_gapless_i.h"

/*------------------------------------------------------------------------
 * Function declarations
 * -----------------------------------------------------------------------*/

static capi_err_t capi_gapless_process(capi_t *_pif, capi_stream_data_t *input[], capi_stream_data_t *output[]);

static capi_err_t capi_gapless_end(capi_t *_pif);

static capi_vtbl_t vtbl = { capi_gapless_process,        capi_gapless_end,
                            capi_gapless_set_param,      capi_gapless_get_param,
                            capi_gapless_set_properties, capi_gapless_get_properties };

/*------------------------------------------------------------------------
 * Function definitions
 * -----------------------------------------------------------------------*/

/**
 * Inits event configuration.
 */
void capi_gapless_init_config(capi_gapless_t *me_ptr)
{
   // Start disabled so that we will raise to enable once.
   me_ptr->events_config.enable      = FALSE;
   me_ptr->events_config.kpps        = GAPLESS_KPPS;
   me_ptr->events_config.delay_in_us = 0;
   me_ptr->events_config.code_bw     = GAPLESS_BW;
   me_ptr->events_config.data_bw     = 0;
}

/**
 * Get static properties implementation for CAPI interface.
 */
capi_err_t capi_gapless_get_static_properties(capi_proplist_t *init_set_properties, capi_proplist_t *static_properties)
{
   capi_err_t capi_result = CAPI_EOK;
   if (NULL != static_properties)
   {
      capi_result = capi_gapless_get_properties((capi_t *)NULL, static_properties);
      if (CAPI_FAILED(capi_result))
      {
         AR_MSG(DBG_ERROR_PRIO, "get static properties failed!");
         return capi_result;
      }
   }
   else
   {
      AR_MSG(DBG_ERROR_PRIO, "get static properties received bad pointer, 0x%p", static_properties);
   }

   return capi_result;
}

/**
 * Init implementation for CAPI interface - gets called when module is created.
 */
capi_err_t capi_gapless_init(capi_t *_pif, capi_proplist_t *init_set_properties)
{
   capi_err_t capi_result = CAPI_EOK;

   if (NULL == _pif || NULL == init_set_properties)
   {
      AR_MSG(DBG_ERROR_PRIO, "Init received bad pointer, 0x%p, 0x%p", _pif, init_set_properties);

      CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      return capi_result;
   }

   capi_gapless_t *me_ptr = (capi_gapless_t *)_pif;

   memset(me_ptr, 0, sizeof(capi_gapless_t));

   me_ptr->vtbl.vtbl_ptr = &vtbl;

   capi_gapless_init_config(me_ptr);

   me_ptr->num_in_ports                 = GAPLESS_NUM_PORTS_INVALID;
   me_ptr->num_out_ports                = GAPLESS_NUM_PORTS_INVALID;
   me_ptr->active_in_port_index         = GAPLESS_PORT_INDEX_INVALID;
   me_ptr->cntr_frame_size_us           = GAPLESS_INVALID_CNTR_FRAME_SIZE;
   me_ptr->is_gapless_cntr_duty_cycling = FALSE;

   me_ptr->trigger_policy.default_trigger_affinity  = FWK_EXTN_PORT_TRIGGER_AFFINITY_PRESENT;
   me_ptr->trigger_policy.default_nontrigger_policy = FWK_EXTN_PORT_NON_TRIGGER_INVALID;
   me_ptr->trigger_policy_sent_once                 = FALSE;

   capi_cmn_init_media_fmt_v2(&(me_ptr->operating_media_fmt));

   // Pass through is TRUE until downstream delay is sent and the early eos event is registered.
   me_ptr->pass_through_mode = TRUE;

   if (NULL != init_set_properties)
   {
      capi_result = capi_gapless_set_properties(_pif, init_set_properties);
      if ((CAPI_EOK != capi_result) && (CAPI_EUNSUPPORTED != capi_result))
      {
         AR_MSG(DBG_ERROR_PRIO, "Initialization Set Property Failed");
         return capi_result;
      }
   }

   capi_result |= gapless_raise_event(me_ptr);

   AR_MSG(DBG_HIGH_PRIO, "Initialization completed !!");
   return capi_result;
}

/**
 * Gain module Data Process function to process an input buffer and generates an output buffer.
 */
static capi_err_t capi_gapless_process(capi_t *_pif, capi_stream_data_t *input[], capi_stream_data_t *output[])
{
   capi_err_t      capi_result = CAPI_EOK;
   capi_gapless_t *me_ptr      = (capi_gapless_t *)_pif;

   POSAL_ASSERT(me_ptr);

   capi_result = gapless_process(me_ptr, input, output);

   return capi_result;
}

/**
 * End function, returns the library to the uninitialized state and frees all the memory that was allocated. This
 * function also frees the virtual function table.
 */
static capi_err_t capi_gapless_end(capi_t *_pif)
{

   capi_err_t capi_result = CAPI_EOK;
   if (NULL == _pif)
   {
      AR_MSG(DBG_ERROR_PRIO, "End received bad pointer, 0x%p", _pif);
      CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      return capi_result;
   }

   capi_gapless_t *me_ptr = (capi_gapless_t *)_pif;

   // Close all opened ports.
   for (uint32_t i = 0; i < me_ptr->num_in_ports; i++)
   {
      capi_gapless_in_port_t *in_port_ptr = &(me_ptr->in_ports[i]);

      if (DATA_PORT_STATE_CLOSED != in_port_ptr->cmn.state)
      {
         bool_t IS_INPUT = TRUE;
         capi_result |= capi_gapless_port_close(me_ptr, i, IS_INPUT);
      }
   }

   me_ptr->vtbl.vtbl_ptr = NULL;

   AR_MSG(DBG_HIGH_PRIO, "End done");
   return capi_result;
}
