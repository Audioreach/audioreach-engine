/**
 * \file gen_topo_capi_cb_handler_island.c
 * \brief
 *     This file contains functions for GEN_CNTR
 *
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "gen_topo.h"
#include "gen_topo_capi.h"

static capi_err_t gen_topo_capi_handle_event_within_island(void *             context_ptr,
                                                           capi_event_id_t    id,
                                                           capi_event_info_t *event_info_ptr,
                                                           bool_t *           handled_event_within_island);

capi_err_t gen_topo_capi_callback(void *context_ptr, capi_event_id_t id, capi_event_info_t *event_info_ptr)
{
   capi_err_t result = CAPI_EOK;
   SPF_MANAGE_CRITICAL_SECTION

   if (!context_ptr || !event_info_ptr)
   {
      return CAPI_EFAILED;
   }

   gen_topo_module_t *module_ptr = (gen_topo_module_t *)(context_ptr);
   gen_topo_t *       topo_ptr   = module_ptr->topo_ptr;

   SPF_CRITICAL_SECTION_START(&topo_ptr->gu);

   // try to handle the event in island, if not handle in non-island
   bool_t handled_event_within_island = FALSE;
   result = gen_topo_capi_handle_event_within_island(context_ptr, id, event_info_ptr, &handled_event_within_island);

   // Exit island and handle if it cannot be handled within island
   if (FALSE == handled_event_within_island)
   {
      // Exit island if in island before processing any event

      /*  if module raises an event that's handled outside island and then raises vote for island-entry, we
       *  shouldn't enter island immediately until frame count condition is met. E.g. gate open event followed by
       *  vote-for-island event: in this case after gate open we are supposed to be outside island, but a vote from
       *  module shouldn't push us back to island as rest of the fwk may be running outside island. Module doesn't
       *  know the whole picture. */
      if (topo_ptr->topo_to_cntr_vtable_ptr->vote_against_island)
      {
         topo_ptr->topo_to_cntr_vtable_ptr->vote_against_island(topo_ptr);
      }

      result = gen_topo_capi_callback_non_island(context_ptr, id, event_info_ptr);
   }
#ifdef VERBOSE_DEBUGGING
   else
   {
      // handled the event within island with result
      TOPO_MSG_ISLAND(topo_ptr->gu.log_id,
                      DBG_HIGH_PRIO,
                      "Handled the event callback within island result: 0x%lx",
                      result);
   }
#endif

   SPF_CRITICAL_SECTION_END(&topo_ptr->gu);

   return result;
}

/**
 * Handles capi events within island. If it cannot be handled within island return is_handled_within_island=FALSE.
 */
static capi_err_t gen_topo_capi_handle_event_within_island(void *             context_ptr,
                                                           capi_event_id_t    id,
                                                           capi_event_info_t *event_info_ptr,
                                                           bool_t *           handled_event_within_island)
{
   capi_err_t result = CAPI_EOK;

   gen_topo_module_t *module_ptr = (gen_topo_module_t *)(context_ptr);
   gen_topo_t *       topo_ptr   = module_ptr->topo_ptr;
   capi_buf_t *       payload    = &event_info_ptr->payload;

   // by deafult marking handled flag true,
   // make sure to return false if could not be handled within island
   *handled_event_within_island = TRUE;

   switch (id)
   {
#ifdef USES_QSH_AUDIO_IN_ISLAND
      case CAPI_EVENT_DATA_TO_DSP_CLIENT_V2:
      {
         if (event_info_ptr->payload.actual_data_len < sizeof(capi_event_data_to_dsp_client_v2_t))
         {
            result |= AR_ENEEDMORE;
            break;
         }

         capi_event_data_to_dsp_client_v2_t *payload_ptr =
            (capi_event_data_to_dsp_client_v2_t *)(event_info_ptr->payload.data_ptr);

         topo_evt_dest_addr_t dest_address;
         dest_address.address = payload_ptr->dest_address;

         if (GPR_HEAP_INDEX_DEFAULT == dest_address.a.gpr_heap_index)
         {
            // event cannot be handled in island
            *handled_event_within_island = FALSE;
            return AR_EUNSUPPORTED;
         }

         return ar_result_to_capi_err(
            topo_ptr->topo_to_cntr_vtable_ptr->raise_data_to_dsp_client_v2(module_ptr, event_info_ptr));
         break;
      }
#endif
      case CAPI_EVENT_GET_DATA_FROM_DSP_SERVICE:
      {
         if (payload->actual_data_len < sizeof(capi_event_get_data_from_dsp_service_t))
         {
            TOPO_MSG_ISLAND(topo_ptr->gu.log_id,
                            DBG_ERROR_PRIO,
                            "Module 0x%lX: Error in callback function. The actual size %lu is less than the required "
                            "size "
                            "%lu for id %lu.",
                            module_ptr->gu.module_instance_id,
                            payload->actual_data_len,
                            sizeof(capi_event_get_data_from_dsp_service_t),
                            (uint32_t)(id));
            return CAPI_ENEEDMORE;
         }

         // Some events can be handled within topo itself
         capi_buf_t *                            payload = &event_info_ptr->payload;
         capi_event_get_data_from_dsp_service_t *dsp_event_ptr =
            (capi_event_get_data_from_dsp_service_t *)(payload->data_ptr);

         switch (dsp_event_ptr->param_id)
         {
            case INTF_EXTN_EVENT_ID_IMCL_GET_ONE_TIME_BUF:
            case INTF_EXTN_EVENT_ID_IMCL_GET_RECURRING_BUF:
            {
               return ar_result_to_capi_err(
                  topo_ptr->topo_to_cntr_vtable_ptr->raise_data_from_dsp_service_event(module_ptr, event_info_ptr));
            }
            default:
            {
               TOPO_MSG_ISLAND(topo_ptr->gu.log_id,
                               DBG_HIGH_PRIO,
                               "Event ID 0x%X cannot be handled in isalnd",
                               dsp_event_ptr->param_id);
               *handled_event_within_island = FALSE;
               result                       = AR_EUNSUPPORTED;
            }
         }
         break;
      }
      case CAPI_EVENT_DATA_TO_DSP_SERVICE:
      {
         if (payload->actual_data_len < sizeof(capi_event_data_to_dsp_service_t))
         {
             TOPO_MSG_ISLAND(topo_ptr->gu.log_id,
                     DBG_ERROR_PRIO,
                     "Module 0x%lX: Error in callback function. The actual size %lu is less than the required size "
                     "%lu for id %lu.",
                     module_ptr->gu.module_instance_id,
                     payload->actual_data_len,
                     sizeof(capi_event_data_to_dsp_service_t),
                     (uint32_t)(id));
            return CAPI_ENEEDMORE;
         }

         // Some events can be handled within topo itself
         capi_buf_t *                      payload       = &event_info_ptr->payload;
         capi_event_data_to_dsp_service_t *dsp_event_ptr = (capi_event_data_to_dsp_service_t *)(payload->data_ptr);

         switch (dsp_event_ptr->param_id)
         {
            case INTF_EXTN_EVENT_ID_IMCL_RECURRING_BUF_INFO:
            case INTF_EXTN_EVENT_ID_IMCL_OUTGOING_DATA:
            {
                return ar_result_to_capi_err(
                   topo_ptr->topo_to_cntr_vtable_ptr->raise_data_to_dsp_service_event(module_ptr, event_info_ptr));
            }
            case INTF_EXTN_EVENT_ID_MIMO_MODULE_PROCESS_STATE:
            {
               //ignore this.
               break;
            }
            default:
            {
#ifdef VERBOSE_DEBUGGING
                TOPO_MSG_ISLAND(topo_ptr->gu.log_id,
                                  DBG_HIGH_PRIO,"Event ID 0x%X cannot be handled in island",dsp_event_ptr->param_id);
#endif
                 *handled_event_within_island = FALSE;
                 result                       = AR_EUNSUPPORTED;
            }
         }
         break;
      }
      case CAPI_EVENT_ISLAND_VOTE:
      {
         if (event_info_ptr->payload.actual_data_len < sizeof(capi_event_island_vote_t))
         {
            TOPO_MSG_ISLAND(topo_ptr->gu.log_id,
                            DBG_ERROR_PRIO,
                            "Module 0x%lX: Error in callback function. The actual size %lu is less than the required "
                            "size %zu for id %lu.",
                            module_ptr->gu.module_instance_id,
                            event_info_ptr->payload.actual_data_len,
                            sizeof(capi_event_island_vote_t),
                            (uint32_t)(id));
            return CAPI_ENEEDMORE;
         }

         if (!POSAL_IS_ISLAND_HEAP_ID((POSAL_HEAP_ID)topo_ptr->heap_id))
         {
            TOPO_MSG_ISLAND(topo_ptr->gu.log_id,
                            DBG_ERROR_PRIO,
                            "Module 0x%lX: ID %lu is not required for modules in Non Island "
                            "container. So, Ignoring.",
                            module_ptr->gu.module_instance_id,
                            (uint32_t)(id));
            return CAPI_EOK;
         }

         capi_event_island_vote_t *vote_ptr = (capi_event_island_vote_t *)(event_info_ptr->payload.data_ptr);

#ifndef SIM
         // don't check for the change on SIM, data logging module will be calling this
         if (module_ptr->flags.voted_island_exit != vote_ptr->island_vote)
#endif
         {
            module_ptr->flags.voted_island_exit = vote_ptr->island_vote;

#ifdef SIM
            //for sim, exit island without checking current island state.
            //even if topo level aggregate vote is already against island
            if (PM_ISLAND_VOTE_EXIT == module_ptr->flags.voted_island_exit)
            {
               posal_island_trigger_island_exit();
            }
#endif

            TOPO_MSG_ISLAND(topo_ptr->gu.log_id,
                            DBG_HIGH_PRIO,
                            "Module 0x%lX: Voted Island %ld (0-> For Island, 1-> Against Island)",
                            module_ptr->gu.module_instance_id,
                            (uint32_t)vote_ptr->island_vote);

            // exiting island irrespective of module votes.
            // frame count will be reset and if module allows then will vote for island after completing some frames.
            if (topo_ptr->topo_to_cntr_vtable_ptr->vote_against_island)
            {
               return topo_ptr->topo_to_cntr_vtable_ptr->vote_against_island(topo_ptr);
            }
         }

         return CAPI_EOK;

         break;
      }
      default: // event cannot be supported in island, return unsupported
      {
         *handled_event_within_island = FALSE;
         result                       = AR_EUNSUPPORTED;
         break;
      }
   }

   return result;
}
