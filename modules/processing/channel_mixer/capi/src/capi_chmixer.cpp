/* =========================================================================
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear
 * =========================================================================*/

/* ======================================================================== */
/**
 @file capi_chmixer.cpp

 C source file to implement the Audio Post Processor Interface for
 Channel Mixer.
 */

/*------------------------------------------------------------------------
 * Include files and Macro definitions
 * -----------------------------------------------------------------------*/
#include "posal.h"
#include "capi_chmixer.h"
#include "capi_chmixer_utils.h"

#include "chmixer_api.h"

#define ALIGN_4_BYTES(a) ((a + 3) & (0xFFFFFFFC))

#ifdef CAPI_CHMIXER_KPPS_PROFILING
#include <q6sim_timer.h>
#endif

/*------------------------------------------------------------------------
 * Static declarations
 * -----------------------------------------------------------------------*/

static capi_err_t capi_chmixer_process(capi_t *_pif, capi_stream_data_t *input[], capi_stream_data_t *output[]);

static capi_err_t capi_chmixer_end(capi_t *_pif);

static capi_err_t capi_chmixer_set_param(capi_t *                      _pif,
                                         const uint32_t                param_id,
                                         const capi_port_info_t *const port_info_ptr,
                                         capi_buf_t *const             params_ptr);

static capi_err_t capi_chmixer_get_param(capi_t *                      _pif,
                                         const uint32_t                param_id,
                                         const capi_port_info_t *const port_info_ptr,
                                         capi_buf_t *const             params_ptr);

static capi_err_t capi_chmixer_set_properties(capi_t *_pif, capi_proplist_t *props_ptr);

static capi_err_t capi_chmixer_get_properties(capi_t *_pif, capi_proplist_t *props_ptr);

static const capi_vtbl_t vtbl = { capi_chmixer_process,        capi_chmixer_end,
                                  capi_chmixer_set_param,      capi_chmixer_get_param,
                                  capi_chmixer_set_properties, capi_chmixer_get_properties };

/*------------------------------------------------------------------------
 Function name: capi_chmixer_get_static_properties
 DESCRIPTION: Function to get the static properties of CHMIXER module
 -----------------------------------------------------------------------*/
capi_err_t capi_chmixer_get_static_properties(capi_proplist_t *const init_set_properties,
                                              capi_proplist_t *const static_properties)
{
   capi_err_t capi_result = CAPI_EOK;

   if (NULL != static_properties)
   {
      capi_result |= capi_chmixer_process_get_properties((capi_chmixer_t *)NULL, static_properties);
      if (CAPI_FAILED(capi_result))
      {
         return capi_result;
      }
   }
   return capi_result;
}

/*------------------------------------------------------------------------
 Function name: capi_chmixer_init
 DESCRIPTION: Initialize the CAPI V2 CHMIXER module.
 -----------------------------------------------------------------------*/
capi_err_t capi_chmixer_init(capi_t *_pif, capi_proplist_t *init_set_properties)
{
   capi_err_t capi_result = CAPI_EOK;

   if (NULL == _pif)
   {
      CHMIXER_MSG(MIID_UNKNOWN, DBG_ERROR_PRIO, "Init received NULL pointer");
      return CAPI_EBADPARAM;
   }

   const uint32_t mem_size = capi_get_init_mem_req();
   memset(_pif, 0, mem_size);

   uintptr_t ptr = (uintptr_t)_pif;

   capi_chmixer_t *me_ptr = (capi_chmixer_t *)ptr;

   me_ptr->heap_mem.heap_id = POSAL_HEAP_DEFAULT;
   me_ptr->vtbl.vtbl_ptr    = &vtbl;
   me_ptr->client_enable    = CAPI_CHMIXER_ENABLE;

   // setting invalid media format
   capi_result = capi_cmn_init_media_fmt_v2(&me_ptr->input_media_fmt[CAPI_CHMIXER_DEFAULT_PORT]);
   me_ptr->output_media_fmt[CAPI_CHMIXER_DEFAULT_PORT] = me_ptr->input_media_fmt[CAPI_CHMIXER_DEFAULT_PORT];

   me_ptr->use_default_channel_info[CAPI_CHMIXER_DEFAULT_PORT] = TRUE;

   // No coefficients have been received yet
   me_ptr->config.lib_enable    = FALSE;
   me_ptr->config.num_coef_sets = 0;
   me_ptr->config.coef_sets_ptr = NULL;

   // Set flags
   me_ptr->inp_media_fmt_received = FALSE;
   me_ptr->is_native_mode         = TRUE;
   me_ptr->coef_payload_size      = 0;

   // sets invalid events
   capi_chmixer_init_events(me_ptr);

   if (NULL != init_set_properties)
   {
      // should contain  EVENT_CALLBACK_INFO, PORT_INFO
      capi_result |= capi_chmixer_process_set_properties(me_ptr, init_set_properties);

      // Ignoring non-fatal error code.
      capi_result ^= (capi_result & CAPI_EUNSUPPORTED);
   }
   uint32_t miid = me_ptr ? me_ptr->miid : MIID_UNKNOWN;

   if (CAPI_FAILED(capi_result))
   {
      CHMIXER_MSG(miid, DBG_ERROR_PRIO, "Init failed!");
   }
   else
   {
      capi_chmixer_raise_events(me_ptr, false);
#ifdef CAPI_CHMIXER_DEBUG_MSG
      CHMIXER_MSG(miid, DBG_ERROR_PRIO, "CAPI CHMIXER: Init Done!");
#endif
   }
   return capi_result;
}

/*------------------------------------------------------------------------
 Function name: capi_chmixer_set_properties
 DESCRIPTION: Function to set properties to the CHMIXER module
 -----------------------------------------------------------------------*/
static capi_err_t capi_chmixer_set_properties(capi_t *_pif, capi_proplist_t *props_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   if (NULL == _pif || NULL == props_ptr)
   {
      CHMIXER_MSG(MIID_UNKNOWN, DBG_ERROR_PRIO, "Set properties received NULL pointer");
      return CAPI_EBADPARAM;
   }
   capi_chmixer_t *me_ptr = (capi_chmixer_t *)_pif;
   capi_result |= capi_chmixer_process_set_properties(me_ptr, props_ptr);

   return capi_result;
}

/*------------------------------------------------------------------------
 Function name: capi_chmixer_get_properties
 DESCRIPTION: Function to get the properties from the CHMIXER module
 -----------------------------------------------------------------------*/
static capi_err_t capi_chmixer_get_properties(capi_t *_pif, capi_proplist_t *props_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   if (NULL == props_ptr)
   {
      CHMIXER_MSG(MIID_UNKNOWN, DBG_ERROR_PRIO, "Get properties received NULL pointer");
      return CAPI_EBADPARAM;
   }
   capi_chmixer_t *me_ptr = (capi_chmixer_t *)_pif;
   capi_result |= capi_chmixer_process_get_properties(me_ptr, props_ptr);

   return capi_result;
}

/*------------------------------------------------------------------------
 Function name: capi_chmixer_set_param
 DESCRIPTION: Function to clear coef sets value\ptr.
 -----------------------------------------------------------------------*/
static void capi_free_coef_sets(capi_chmixer_t *const me_ptr)
{
   // Freeing older memory
   if (NULL != me_ptr->config.coef_sets_ptr)
   {
      posal_memory_free(me_ptr->config.coef_sets_ptr);
      me_ptr->config.coef_sets_ptr = NULL;
      me_ptr->config.num_coef_sets = 0;
      me_ptr->coef_payload_size    = 0;
   }
}


/*------------------------------------------------------------------------
 Function name: capi_chmixer_set_param
 DESCRIPTION: Function to set parameter value\structure.
 -----------------------------------------------------------------------*/
static capi_err_t capi_chmixer_set_param(capi_t *                      _pif,
                                         const uint32_t                param_id,
                                         const capi_port_info_t *const port_info_ptr,
                                         capi_buf_t *const             params_ptr)

{
   if (NULL == _pif || NULL == params_ptr)
   {
      CHMIXER_MSG(MIID_UNKNOWN, DBG_ERROR_PRIO, "Set param received NULL pointer");
      return CAPI_EBADPARAM;
   }

   capi_err_t            capi_result = CAPI_EOK;
   capi_chmixer_t *const me_ptr      = (capi_chmixer_t *)(_pif);
   const void *const     param_ptr   = (void *)params_ptr->data_ptr;
   uint32_t              param_size  = params_ptr->actual_data_len;

   if (NULL == param_ptr)
   {
      CHMIXER_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Set param received NULL param pointer");
      return CAPI_EBADPARAM;
   }

   switch (param_id)
   {
      case CAPI_CHMIXER_PARAM_ID_ENABLE:
      {
         if (param_size >= sizeof(capi_chmixer_enable_payload_t))
         {
            const capi_chmixer_enable_payload_t *const enable_ptr = (const capi_chmixer_enable_payload_t *)(param_ptr);
            me_ptr->client_enable                                 = enable_ptr->enable;

            CHMIXER_MSG(me_ptr->miid,
                        DBG_HIGH_PRIO,
                        "CAPI_CHMIXER_PARAM_ID_ENABLE received with enable %d",
                        enable_ptr->enable);

            capi_chmixer_raise_events(me_ptr, false);
         }
         else
         {
            CHMIXER_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Set Enable/Disable, Bad param size %lu", param_size);
            CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
            break;
         }
         break;
      }
      case PARAM_ID_CHMIXER_COEFF:
      {
         CHMIXER_MSG(me_ptr->miid, DBG_HIGH_PRIO, "PARAM_ID_CHMIXER_COEFF received");
         if (param_size >= sizeof(param_id_chmixer_coeff_t))
         {
            size_t   req_param_size = 0, req_param_size_al = 0;
            uint32_t index = 0, total_payload_size = 0, total_payload_aligned_size = 0;

            const param_id_chmixer_coeff_t *const p_param = (param_id_chmixer_coeff_t *)param_ptr;

            param_size = param_size - sizeof(param_id_chmixer_coeff_t);

            // error check for coefficients index
            if (0 == p_param->num_coeff_tbls)
            {
               CHMIXER_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Set param received zero coefficients");

               // Freeing older memory
               capi_free_coef_sets(me_ptr);

               me_ptr->config.lib_enable = FALSE;
               break;
            }

            // pointer to the first array
            uint8_t *first_data_ptr   = (uint8_t *)(sizeof(param_id_chmixer_coeff_t) + params_ptr->data_ptr);
            uint8_t *new_coef_arr_ptr = first_data_ptr;

            if (param_size < sizeof(chmixer_coeff_t))
            {
               CHMIXER_MSG(me_ptr->miid,
                           DBG_ERROR_PRIO,
                           "Set coeff, Bad param size %lu, required param size %lu",
                           param_size,
                           sizeof(chmixer_coeff_t));
               return CAPI_ENEEDMORE;
            }
            chmixer_coeff_t *coeff_param_ptr = (chmixer_coeff_t *)first_data_ptr;

            // Size and error check loop
            for (index = 0; index < p_param->num_coeff_tbls; index++)
            {
               // Size of each tbl
               req_param_size =
                  sizeof(chmixer_coeff_t) + (sizeof(uint16_t) * coeff_param_ptr->num_output_channels) +
                  (sizeof(uint16_t) * coeff_param_ptr->num_input_channels) +
                  (sizeof(int16_t) * coeff_param_ptr->num_output_channels * coeff_param_ptr->num_input_channels);

               // if req param size is not 4byte align add padding len and store in req_param_size_aligned
               req_param_size_al = ALIGN_4_BYTES(req_param_size);
               CHMIXER_MSG(me_ptr->miid, DBG_HIGH_PRIO, "al size %d size %d", req_param_size_al, req_param_size);

               if (param_size < req_param_size_al)
               {
                  CHMIXER_MSG(me_ptr->miid,
                              DBG_ERROR_PRIO,
                              "Set coeff, Bad param size %lu, required param size %lu",
                              param_size,
                              req_param_size_al);
                  CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
                  return capi_result;
               }

               // error check for num channels
               if ((coeff_param_ptr->num_input_channels > CAPI_CHMIXER_MAX_CHANNELS) ||
                   (0 == coeff_param_ptr->num_input_channels) ||
                   (coeff_param_ptr->num_output_channels > CAPI_CHMIXER_MAX_CHANNELS) ||
                   (0 == coeff_param_ptr->num_output_channels))
               {
                  CHMIXER_MSG(me_ptr->miid,
                              DBG_ERROR_PRIO,
                              "Set coeff, invalid number of channels. inp ch %hu, out ch %hu",
                              coeff_param_ptr->num_input_channels,
                              coeff_param_ptr->num_output_channels);
                  return CAPI_EBADPARAM;
               }

               const uint16_t *out_ch_map = (uint16_t *)((uint8_t *)coeff_param_ptr + sizeof(chmixer_coeff_t));
               const uint16_t *in_ch_map  = out_ch_map + coeff_param_ptr->num_output_channels;

               // error check for channel type
               capi_result |= capi_chmixer_check_ch_type(in_ch_map, coeff_param_ptr->num_input_channels);
               capi_result |= capi_chmixer_check_ch_type(out_ch_map, coeff_param_ptr->num_output_channels);
               if (CAPI_FAILED(capi_result))
               {
                  CHMIXER_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Set coeff, invalid channel type");
                  return capi_result;
               }

               // Update size
               param_size = param_size - req_param_size_al;

               // Update param size and ptrs to point to the next coeff array provided its not last coefficient payload
               if (index != (p_param->num_coeff_tbls - 1))
               {
                  if (param_size < sizeof(chmixer_coeff_t))
                  {
                     CHMIXER_MSG(me_ptr->miid,
                                 DBG_ERROR_PRIO,
                                 "Set coeff, Bad param size %lu, required param size %lu",
                                 param_size,
                                 sizeof(chmixer_coeff_t));
                     return CAPI_EBADPARAM;
                  }

                  // update to move to the next array
                  new_coef_arr_ptr = (uint8_t *)(new_coef_arr_ptr + req_param_size_al);
                  coeff_param_ptr  = (chmixer_coeff_t *)(new_coef_arr_ptr);
               }

               // Update total payload size to be used when we store the params in chmixer
               total_payload_size += req_param_size;
               total_payload_aligned_size += req_param_size_al;
               CHMIXER_MSG(me_ptr->miid, DBG_HIGH_PRIO, "Total payload size %d", total_payload_size);
            }

            if (me_ptr->coef_payload_size != total_payload_size)
            {
               // Freeing older memory
               capi_free_coef_sets(me_ptr);

               uint32_t size_to_malloc =
                  total_payload_size +
                  (p_param->num_coeff_tbls * (sizeof(capi_chmixer_coef_set) - sizeof(chmixer_coeff_t)));

               me_ptr->config.coef_sets_ptr =
                  (capi_chmixer_coef_set *)posal_memory_malloc(size_to_malloc, (POSAL_HEAP_ID)me_ptr->heap_mem.heap_id);
               if (NULL == me_ptr->config.coef_sets_ptr)
               {
                  CHMIXER_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Set param failed to allocate memory");
                  return CAPI_ENOMEMORY;
               }
            }

            // For first iteration
            // Start of chmixer channel arrays
            uint8_t *new_coeff_arr = (uint8_t *)me_ptr->config.coef_sets_ptr;
            // start of chmixer channel maps
            uint8_t *new_ch_map_arr = new_coeff_arr + (p_param->num_coeff_tbls * sizeof(capi_chmixer_coef_set));

            // reset coeff param ptr to point to start of first array input cfg
            coeff_param_ptr = (chmixer_coeff_t *)first_data_ptr;

            // Processing loop
            for (index = 0; index < p_param->num_coeff_tbls; index++)
            {
               uint32_t padding = 0;
               // Size of each tbl
               uint32_t tbl_size =
                  (sizeof(uint16_t) * coeff_param_ptr->num_output_channels) +
                  (sizeof(uint16_t) * coeff_param_ptr->num_input_channels) +
                  (sizeof(int16_t) * coeff_param_ptr->num_output_channels * coeff_param_ptr->num_input_channels);

               if (0 != (tbl_size % 4))
               {
                  padding = 1; // it can only be 2bytes less since every entry is 2bytes
               }

               // input ptrs
               const uint16_t *out_ch_map = (uint16_t *)((uint8_t *)coeff_param_ptr + sizeof(chmixer_coeff_t));
               const uint16_t *in_ch_map  = out_ch_map + coeff_param_ptr->num_output_channels;
               const int16_t * coef_ptr   = (int16_t *)(in_ch_map + coeff_param_ptr->num_input_channels);

               // Assign pointers after malloc
               capi_chmixer_coef_set *coef_set = (capi_chmixer_coef_set *)new_coeff_arr;
               coef_set->num_in_ch             = coeff_param_ptr->num_input_channels;
               coef_set->num_out_ch            = coeff_param_ptr->num_output_channels;

               coef_set->out_ch_map = (uint16_t *)(new_ch_map_arr);
               coef_set->in_ch_map  = (uint16_t *)(coef_set->out_ch_map + coef_set->num_out_ch);
               coef_set->coef_ptr   = (int16_t *)(coef_set->in_ch_map + coef_set->num_in_ch);

#ifdef CAPI_CHMIXER_DEBUG_MSG

               uint8_t i = 0;
               uint8_t j = 0;

               CHMIXER_MSG(me_ptr->miid,
                           DBG_HIGH_PRIO,
                           "SET PARAM FOR COEFF: Inp Number of Channels %lu",
                           coeff_param_ptr->num_input_channels);

               for (i = 0; i < coef_set->num_in_ch; i++)
               {
                  CHMIXER_MSG(me_ptr->miid, DBG_HIGH_PRIO, "Inp Channel_Type[%hhu]: %d", i, in_ch_map[i]);
               }

               CHMIXER_MSG(me_ptr->miid,
                           DBG_HIGH_PRIO,
                           "Out Number of Channels %lu",
                           coeff_param_ptr->num_output_channels);
               for (j = 0; j < coef_set->num_out_ch; j++)
               {
                  CHMIXER_MSG(me_ptr->miid, DBG_HIGH_PRIO, "Out Channel_Type[%hhu]: %d", j, out_ch_map[j]);
               }

               uint8_t a = i * j;
               for (j = 0; j < a; j++)
               {
                  CHMIXER_MSG(me_ptr->miid, DBG_HIGH_PRIO, "coefficients: %x", *(coef_ptr + j));
               }
#endif
               // Copy values
               uint32_t size_to_cpy =
                  ((coef_set->num_in_ch * sizeof(uint16_t)) + (coef_set->num_out_ch * sizeof(uint16_t)) +
                   (coef_set->num_in_ch * coef_set->num_out_ch * sizeof(int16_t)));

               memscpy(coef_set->out_ch_map, size_to_cpy, out_ch_map, size_to_cpy);

               // Update in and out params to point to the next coeff array provided its not last coefficient payload
               if (index != (p_param->num_coeff_tbls - 1))
               {
                  // Increment with padding to read next sub-structure correctly
                  coeff_param_ptr = (chmixer_coeff_t *)(coef_ptr + (coeff_param_ptr->num_output_channels *
                                                                    coeff_param_ptr->num_input_channels) +
                                                        padding);
                  // point to next array
                  new_coeff_arr = new_coeff_arr + sizeof(capi_chmixer_coef_set);
                  // point to next map
                  new_ch_map_arr = (uint8_t *)(coef_set->coef_ptr + (coef_set->num_out_ch * coef_set->num_in_ch));
               }
            } // end of for loop

            // Update number of coeff sets
            me_ptr->config.num_coef_sets = p_param->num_coeff_tbls;

            // Update data length
            params_ptr->actual_data_len = total_payload_aligned_size;
            me_ptr->coef_payload_size   = total_payload_size;

            if ((me_ptr->input_media_fmt[CAPI_CHMIXER_DEFAULT_PORT].format.num_channels ==
                 CAPI_DATA_FORMAT_INVALID_VAL) ||
                (me_ptr->output_media_fmt[CAPI_CHMIXER_DEFAULT_PORT].format.num_channels ==
                 CAPI_DATA_FORMAT_INVALID_VAL))
            {
               // media format is not received.
               break;
            }

            capi_result |= capi_chmixer_check_init_lib_instance(me_ptr);
            if (CAPI_FAILED(capi_result))
            {
               CHMIXER_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Set coeff, Reinit failed");
               break;
            }
            capi_chmixer_raise_events(me_ptr, FALSE);
         }
         else
         {
            CHMIXER_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Set coeff, Bad param size %lu", param_size);
            CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
            break;
         }
         break;
      }
      case PARAM_ID_CHMIXER_OUT_CH_CFG:
      {
         // sanity check for valid and output port
         if ((port_info_ptr->is_valid) && (port_info_ptr->is_input_port))
         {
            CHMIXER_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Set param id 0x%lx, invalid port info.", param_id);
            CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
            break;
         }

         CHMIXER_MSG(me_ptr->miid, DBG_HIGH_PRIO, "Set param id PARAM_ID_CHMIXER_OUT_CH_CFG");

         // sanity check for valid output port index
         if ((port_info_ptr->is_valid) && (0 != port_info_ptr->port_index))
         {
            CHMIXER_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Set param id 0x%lx, invalid output port index.", param_id);
            CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
            break;
         }

         if (params_ptr->actual_data_len >= sizeof(param_id_chmixer_out_ch_cfg_t))
         {
            param_id_chmixer_out_ch_cfg_t *data_ptr      = (param_id_chmixer_out_ch_cfg_t *)params_ptr->data_ptr;
            capi_media_fmt_v2_t            out_media_fmt = me_ptr->input_media_fmt[CAPI_CHMIXER_DEFAULT_PORT];

            if (PARAM_VAL_UNSET == data_ptr->num_channels)
            {
#ifdef CAPI_CHMIXER_DEBUG_MSG
               CHMIXER_MSG(me_ptr->miid, DBG_HIGH_PRIO, "Set param received unset num channels");
#endif
               break;
            }
            else if (PARAM_VAL_NATIVE == data_ptr->num_channels)
            {
#ifdef CAPI_CHMIXER_DEBUG_MSG
               CHMIXER_MSG(me_ptr->miid, DBG_HIGH_PRIO, "Set param received native num channels");
#endif
               me_ptr->is_native_mode = TRUE;
            }
            else if ((CAPI_CHMIXER_MAX_CHANNELS < data_ptr->num_channels) ||
                     (data_ptr->num_channels == PARAM_VAL_INVALID))
            {
               CHMIXER_MSG(me_ptr->miid,
                           DBG_ERROR_PRIO,
                           "Invalid num channels %d received, Max supported = %d",
                           data_ptr->num_channels,
                           CAPI_CHMIXER_MAX_CHANNELS);
               CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
               break;
            }
            else // VALID
            {
               uint32_t required_size =
                  sizeof(param_id_chmixer_out_ch_cfg_t) + (data_ptr->num_channels * sizeof(uint16_t));
               if (params_ptr->actual_data_len < required_size)
               {
                  CHMIXER_MSG(me_ptr->miid,
                              DBG_ERROR_PRIO,
                              "Set param id 0x%lx, Bad param size %lu, required_size = %lu",
                              param_id,
                              params_ptr->actual_data_len,
                              required_size);
                  CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
                  break;
               }

               me_ptr->is_native_mode = FALSE;
               uint16_t *channel_map  = &data_ptr->channel_map[0];

               // free older configured chmap being used
               if (me_ptr->configured_ch_map)
               {
                  posal_memory_free(me_ptr->configured_ch_map);
                  me_ptr->configured_ch_map = NULL;
               }
               // save new config
               me_ptr->configured_num_channels = data_ptr->num_channels;

               me_ptr->configured_ch_map = (uint16_t *)posal_memory_malloc((sizeof(uint16_t) * data_ptr->num_channels),
                                                                           (POSAL_HEAP_ID)me_ptr->heap_mem.heap_id);
               if (NULL == me_ptr->configured_ch_map)
               {
                  CHMIXER_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Set param failed to allocate memory");
                  return CAPI_ENOMEMORY;
               }

               memscpy(&me_ptr->configured_ch_map[0],
                       sizeof(out_media_fmt.channel_type),
                       channel_map,
                       sizeof(uint16_t) * data_ptr->num_channels);

#ifdef CAPI_CHMIXER_DEBUG_MSG
               CHMIXER_MSG(me_ptr->miid,
                           DBG_HIGH_PRIO,
                           "Set param for out mf: Storing num channels %d because valid config",
                           data_ptr->num_channels);
               for (uint8_t j = 0; j < data_ptr->num_channels; j++)
               {
                  CHMIXER_MSG(me_ptr->miid,
                              DBG_HIGH_PRIO,
                              "Set param for out mf: chmap received[%d]: %x",
                              j,
                              data_ptr->channel_map[j]);
               }
#endif

               if (TRUE == me_ptr->inp_media_fmt_received) // copy to out media fmt and raise event
               {
                  out_media_fmt.format.num_channels = data_ptr->num_channels;
                  memscpy(out_media_fmt.channel_type,
                          sizeof(out_media_fmt.channel_type),
                          channel_map,
                          sizeof(uint16_t) * data_ptr->num_channels);
               }
            }

            if (TRUE == me_ptr->inp_media_fmt_received)
            {
               capi_result = capi_chmixer_set_output_media_fmt(me_ptr, &out_media_fmt);
            }
         }
         else
         {
            CHMIXER_MSG(me_ptr->miid,
                        DBG_ERROR_PRIO,
                        "Set param id 0x%lx, Bad param size %lu",
                        param_id,
                        params_ptr->actual_data_len);
            CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
         }
         break;
      }
      default:
      {
         CHMIXER_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Unsupported param id %lu", param_id);
         CAPI_SET_ERROR(capi_result, CAPI_EUNSUPPORTED);
         break;
      }
   }
   return capi_result;
}

/*------------------------------------------------------------------------
 Function name: capi_chmixer_get_param
 DESCRIPTION: Function to get parameter value\structure.
 -----------------------------------------------------------------------*/
static capi_err_t capi_chmixer_get_param(capi_t *                      _pif,
                                         const uint32_t                param_id,
                                         const capi_port_info_t *const port_info_ptr,
                                         capi_buf_t *const             params_ptr)
{
   if (NULL == _pif || NULL == params_ptr)
   {
      CHMIXER_MSG(MIID_UNKNOWN, DBG_ERROR_PRIO, "Get param received NULL pointer");
      return CAPI_EBADPARAM;
   }

   capi_err_t                  capi_result = CAPI_EOK;
   const capi_chmixer_t *const me_ptr      = (capi_chmixer_t *)(_pif);

   if (NULL == params_ptr->data_ptr)
   {
      CHMIXER_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Get param received NULL param pointer");
      return CAPI_EBADPARAM;
   }

   switch (param_id)
   {
      case PARAM_ID_CHMIXER_OUT_CH_CFG:
      {
         capi_media_fmt_v2_t out_media_fmt = me_ptr->output_media_fmt[CAPI_CHMIXER_DEFAULT_PORT];
         uint32_t            num_ch =
            ((CAPI_DATA_FORMAT_INVALID_VAL != out_media_fmt.format.num_channels) ? out_media_fmt.format.num_channels
                                                                                 : 0);
         uint32_t required_size = sizeof(param_id_chmixer_out_ch_cfg_t) + (num_ch * sizeof(uint16_t));

         if (params_ptr->max_data_len < required_size)
         {
            CHMIXER_MSG(me_ptr->miid,
                        DBG_ERROR_PRIO,
                        "Get param id 0x%lx, Bad param size %lu",
                        param_id,
                        params_ptr->max_data_len);
            params_ptr->actual_data_len = required_size;
            CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
            break;
         }

         param_id_chmixer_out_ch_cfg_t *data_ptr = (param_id_chmixer_out_ch_cfg_t *)params_ptr->data_ptr;
         data_ptr->num_channels                  = out_media_fmt.format.num_channels;
         uint16_t *channel_map                   = (uint16_t *)(data_ptr + 1);
         memscpy(channel_map,
                 params_ptr->max_data_len - sizeof(param_id_chmixer_out_ch_cfg_t),
                 out_media_fmt.channel_type,
                 (num_ch * sizeof(uint16_t)));

         params_ptr->actual_data_len = required_size;
         break;
      }
      default:
      {
         CHMIXER_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Unsupported param id %lu", param_id);
         CAPI_SET_ERROR(capi_result, CAPI_EUNSUPPORTED);
         break;
      }
   }
   return capi_result;
}

/*------------------------------------------------------------------------
 Function name: capi_chmixer_process
 DESCRIPTION: Processes an input buffer and generates an output buffer.
 -----------------------------------------------------------------------*/
static capi_err_t capi_chmixer_process(capi_t *_pif, capi_stream_data_t *input[], capi_stream_data_t *output[])
{
   capi_chmixer_t *const me_ptr = (capi_chmixer_t *)_pif;

   POSAL_ASSERT(me_ptr);
   POSAL_ASSERT(input[CAPI_CHMIXER_DEFAULT_PORT]);
   POSAL_ASSERT(output[CAPI_CHMIXER_DEFAULT_PORT]);

#ifdef CAPI_CHMIXER_KPPS_PROFILING
   me_ptr->profiler.start_cycles = q6sim_read_cycles();
#endif

   void *input_bufs[CAPI_CHMIXER_MAX_CHANNELS]  = { NULL };
   void *output_bufs[CAPI_CHMIXER_MAX_CHANNELS] = { NULL };

   uint32_t bytes_to_process = input[CAPI_CHMIXER_DEFAULT_PORT]->buf_ptr[0].actual_data_len;

   for (uint32_t i = 0; i < input[0]->bufs_num && i < CAPI_CHMIXER_MAX_CHANNELS; i++)
   {
      input_bufs[i] = (void *)input[CAPI_CHMIXER_DEFAULT_PORT]->buf_ptr[i].data_ptr;
      bytes_to_process =
         s32_min_s32_s32(bytes_to_process, input[CAPI_CHMIXER_DEFAULT_PORT]->buf_ptr[i].actual_data_len);
   }

   for (uint32_t i = 0; i < output[0]->bufs_num && i < CAPI_CHMIXER_MAX_CHANNELS; i++)
   {
      output_bufs[i]   = (void *)output[CAPI_CHMIXER_DEFAULT_PORT]->buf_ptr[i].data_ptr;
      bytes_to_process = s32_min_s32_s32(bytes_to_process, output[CAPI_CHMIXER_DEFAULT_PORT]->buf_ptr[i].max_data_len);
   }

   if (me_ptr->config.lib_enable)
   {
      uint16_t byte_sample_convert =
         (me_ptr->input_media_fmt[CAPI_CHMIXER_DEFAULT_PORT].format.bits_per_sample == 16) ? 1 : 2;

      uint32_t samples_to_process = bytes_to_process >> byte_sample_convert;

      ChMixerProcess(me_ptr->lib_ptr, output_bufs, input_bufs, samples_to_process);

      for (uint32_t i = 0; i < me_ptr->input_media_fmt[CAPI_CHMIXER_DEFAULT_PORT].format.num_channels; i++)
      {
         input[CAPI_CHMIXER_DEFAULT_PORT]->buf_ptr[i].actual_data_len = samples_to_process << byte_sample_convert;
      }

      for (uint32_t i = 0; i < me_ptr->output_media_fmt[CAPI_CHMIXER_DEFAULT_PORT].format.num_channels; i++)
      {
         output[CAPI_CHMIXER_DEFAULT_PORT]->buf_ptr[i].actual_data_len = samples_to_process << byte_sample_convert;
      }
   }
   else
   {
      // if process check is disabled then directly copy input to output.
      // This is needed because pspd-mtmx does not have process check event handling for chmixer.
      if (input[CAPI_CHMIXER_DEFAULT_PORT]->bufs_num == output[CAPI_CHMIXER_DEFAULT_PORT]->bufs_num)
      {
         for (uint16_t i = 0; i < input[CAPI_CHMIXER_DEFAULT_PORT]->bufs_num; i++)
         {
            memscpy(output[CAPI_CHMIXER_DEFAULT_PORT]->buf_ptr[i].data_ptr,
                    bytes_to_process,
                    input[CAPI_CHMIXER_DEFAULT_PORT]->buf_ptr[i].data_ptr,
                    bytes_to_process);

            input[CAPI_CHMIXER_DEFAULT_PORT]->buf_ptr[i].actual_data_len  = bytes_to_process;
            output[CAPI_CHMIXER_DEFAULT_PORT]->buf_ptr[i].actual_data_len = bytes_to_process;
         }
      }
      else
      {
         return CAPI_EFAILED;
      }
   }

   output[0]->flags = input[0]->flags;
   if (input[0]->flags.is_timestamp_valid)
   {
      output[0]->timestamp = input[0]->timestamp - me_ptr->event_config.chmixer_delay_in_us;
   }

#ifdef CAPI_CHMIXER_KPPS_PROFILING
   me_ptr->profiler.end_cycles         = q6sim_read_cycles();
   me_ptr->profiler.frame_sample_count = samples_to_process;
   capi_chmixer_kpps_profiler(me_ptr);
#endif

#ifdef CAPI_CHMIXER_DATA_LOG
   capi_chmixer_data_logger(me_ptr, input[CAPI_CHMIXER_DEFAULT_PORT], output[CAPI_CHMIXER_DEFAULT_PORT]);
#endif

   return CAPI_EOK;
}

/*------------------------------------------------------------------------
 Function name: capi_chmixer_end
 DESCRIPTION: Returns the library to the uninitialized state and frees the
 memory that was allocated by module. This function also frees the virtual
 function table.
 -----------------------------------------------------------------------*/
static capi_err_t capi_chmixer_end(capi_t *_pif)
{
   capi_chmixer_t *const me_ptr = (capi_chmixer_t * const)(_pif);

#ifdef CAPI_CHMIXER_KPPS_PROFILING
   capi_chmixer_kpps_print(me_ptr);
#endif

   capi_free_coef_sets(me_ptr);

   if (NULL != me_ptr->configured_ch_map)
   {
      posal_memory_free(me_ptr->configured_ch_map);
      me_ptr->configured_ch_map = NULL;
   }
   if (NULL != me_ptr->lib_ptr)
   {
	   posal_memory_aligned_free(me_ptr->lib_ptr);
      me_ptr->lib_ptr = NULL;
   }
   uint32_t miid = me_ptr->miid;
   memset((void *)me_ptr, 0, sizeof(me_ptr));

   me_ptr->vtbl.vtbl_ptr = NULL;

   CHMIXER_MSG(miid, DBG_HIGH_PRIO, "End done");
   return CAPI_EOK;
}
