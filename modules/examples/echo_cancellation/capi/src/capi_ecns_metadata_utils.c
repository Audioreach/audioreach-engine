/**
 * \file capi_ecns_metadata_utils.c
 *  
 * \brief
 *  
 *     Example Echo Cancellation
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/**----------------------------------------------------------------------------
 ** Include Files
 ** -------------------------------------------------------------------------*/

#include "capi_ecns_i.h"

/* =========================================================================
 * FUNCTION : capi_ecns_propagate_metadata
 * DESCRIPTION:
 *
 *  Metadata propagation -
 *   EC module must handle metadata propagation through primary and reference inputs.
 * Metadata is propagated every process call from inputs to outputs depending upon the
 * amount of data consumed and produced.
 *
 * Generally in EC modules, the primary input metadata is propagated to the output ports
 * and metadata received on reference input is dropped.
 *
 * If the module has non zero delay, some of the metadata can be held in the module
 * internal list. It will propagated out when the delayed data moves out of the module.
 *
 * Each process, EC module needs to do following,
 *  1. Check if metadata exists on Primary or Reference input ports.
 *  2. Handle any specific action related to metadata.
 *  3. If there is no specific action needed module can just propagate or Drop metadata.
 *  4. Propagate metadata from Primary input to output ports.
 *  5. Drop metadata on Reference input port.
 *
 * Currently {EOS, DFG} are two metadata that EC module can expect. Module doesnt have do any specific action
 * for these metadata. It just needs to Propagate these metadata from Primary input and Drop any metadata
 * received on Reference.
 * =========================================================================
 */                                          
capi_err_t capi_ecns_propagate_metadata(capi_ecns_t *       me_ptr,
                                        capi_stream_data_t *input[],
                                        capi_stream_data_t *output[],
                                        uint32_t            pri_in_bytes_before,
                                        uint32_t            pri_bytes_consumed_per_ch,
                                        uint32_t            ref_in_bytes_before,
                                        uint32_t            ref_in_bytes_consumed_per_ch)
{
   capi_err_t capi_result = CAPI_EOK;

   module_cmn_md_list_t **internal_list_pptr = NULL;
   capi_stream_data_v2_t *in_stream_ptr      = NULL;
   capi_stream_data_v2_t *out_stream_ptr     = NULL;

   uint32_t primary_ip_index = 0;
   if (CAPI_EOK != ecns_get_input_port_arr_idx(me_ptr, ECNS_PRIMARY_INPUT_STATIC_PORT_ID, &primary_ip_index))
   {
      return CAPI_EFAILED;
   }

   uint32_t output_port_index = 0;
   if (CAPI_EOK != ecns_get_output_port_arr_idx(me_ptr, ECNS_PRIMARY_OUTPUT_STATIC_PORT_ID, &output_port_index))
   {
      return CAPI_EFAILED;
   }

   for (uint32_t input_port_idx = 0; input_port_idx < me_ptr->num_port_info.num_input_ports; input_port_idx++)
   {
      // continue to next input if buffers or metadata not provided
      if ((NULL == input[input_port_idx]) || (NULL == input[input_port_idx]->buf_ptr))
      {
         continue;
      }

      // Metadata is supported only for stream version v2
      in_stream_ptr = (capi_stream_data_v2_t *)(input[input_port_idx]);
      if (CAPI_STREAM_V2 != in_stream_ptr->flags.stream_data_version)
      {
         // doesn't support metadata propagation, do nothing.
         continue;
      }

      /* Check if metadata needs to be propagated or not. Metadata handled for ECNS modules usually in the following
         way. 
           1. Propagate meta from primary input to primary output port.
           2. Reference input metdata can be dropped if there is no module downstream of ECNS which expects this metdata.
           3. If the output is buffers are not present, metdata can be dropped from Primary input and primary output.
           
           This is just a guideline, but its upto module/algorithm designer's decision on how to propagate metadata.
      */

      /* As an example, in this module we are dropping metdata on the reference input.*/
      bool_t need_to_drop_metadata = (input_port_idx == primary_ip_index) ? FALSE : TRUE;

      /* As an example, we are dropping metadata if output buffers are not present. */
      out_stream_ptr = (capi_stream_data_v2_t *)(output[output_port_index]);
      if (NULL == out_stream_ptr)
      {
         need_to_drop_metadata = TRUE;
      }

      // Drop metadata
      if (need_to_drop_metadata)
      {
         if (in_stream_ptr->metadata_list_ptr)
         {
            ECNS_DBG(me_ptr->miid, DBG_HIGH_PRIO, "destroying md, md_list_ptr 0x%lx.", in_stream_ptr->metadata_list_ptr);
            capi_result |= capi_ecns_destroy_md_list(me_ptr, &(in_stream_ptr->metadata_list_ptr));
         }

         if (in_stream_ptr->flags.end_of_frame)
         {
            ECNS_DBG(me_ptr->miid, DBG_ERROR_PRIO, "EOF is ignored/cleared on secondary input port.");
         }

         // Clear End of frame and marker eos flags
         in_stream_ptr->flags.end_of_frame = FALSE;
         in_stream_ptr->flags.marker_eos   = FALSE;
      }
      else // Propagate
      {
         // When ever EOF flag is set, entire input is expected to be consumed and EOF fame is either propagated or dropped.
         // In this example, we are just propagating from input to output. Module developer must modify this based on the
         // functional expectation.
         if (in_stream_ptr->flags.end_of_frame)
         {
            ECNS_DBG(me_ptr->miid, DBG_ERROR_PRIO, "EOF is propagated from primary input");
            out_stream_ptr->flags.end_of_frame = in_stream_ptr->flags.end_of_frame;
            in_stream_ptr->flags.end_of_frame  = FALSE;
         }

         /* Propagate metadata nodes from input list to output metadata list. There are two scenarios,
            
            1. Module without Algorithmic delay:
                 Modules without algorithmic delay propagte metadata associated with the amount of data consumed on the input
               to the capi outputs.

            2. Module with Algorithmic delay:
                  If the module has delay, the input metadata is propagate in two steps. 
                     
                     i). If the data associated with the metadata is delayed in algorithm, then hold the metadata in the internal
                         list pptr. 
                    
                     ii). When the delayed data flows out of the module, if there is any held metadata in internal list associated
                          with this delayed data it should be propagated to the capi output. 

            Metdata framework extension provides call back utilities to propagate metdata from input to output, even by considering
            the algorithmic delay. Please refer "capi_intf_extn_metadata.h" for detailed information on helper functions for metadata
            propagation.
         
         */

         internal_list_pptr = &(me_ptr->in_port_info[primary_ip_index].md_list_ptr);
         if (in_stream_ptr->metadata_list_ptr || (*internal_list_pptr))
         {
            // Itereate through each metadata node and perform any action if required for each of the metadata IDs.
            // if no action is required, just propagate metadata to the outut through metadata_propagate() function.
            for (module_cmn_md_list_t *node_ptr = in_stream_ptr->metadata_list_ptr; node_ptr != NULL;
                 node_ptr                       = node_ptr->next_ptr)
            {
               module_cmn_md_t *md_ptr = (module_cmn_md_t *)node_ptr->obj_ptr;
               capi_result             = capi_ecns_handle_metadata_node(me_ptr, md_ptr);
            }

            /* For propgation, we need the following information from the current process call.
                 1. Number of input bytes available before process
                 2. Number of input bytes consumed on the input
                 3. Modules' algo delay length and internal list pointer. If the module doesn't have any delay both are NULL.
                 4. Number of bytes produced on the output.
                 5. Need media format information of input and output ports.
            */
            intf_extn_md_propagation_t input_md_info;
            memset(&input_md_info, 0, sizeof(input_md_info));

            /* Populate input data available before and after process for this port_idx.
               
               Note: for this example we are propagating metadata only from primary input hence pri_in_bytes_before, 
               pri_bytes_consumed_per_ch is being populated. module developer must pass this info depending upon 
               the input port under consideration. 
            */
            input_md_info.initial_len_per_ch_in_bytes = pri_in_bytes_before; // before process
            input_md_info.len_per_ch_in_bytes         = pri_bytes_consumed_per_ch; // after process
            input_md_info.buf_delay_per_ch_in_bytes   = 0 ; // update this field if module has delay for this input port idx.
                                                            // this is usually zero.

            // Populate the media format of this input port port_idx.
            input_md_info.df = me_ptr->in_port_info[input_port_idx].media_fmt.header.format_header.data_format;
            input_md_info.bits_per_sample = me_ptr->in_port_info[input_port_idx].media_fmt.format.bits_per_sample;
            input_md_info.sample_rate     = me_ptr->in_port_info[input_port_idx].media_fmt.format.sampling_rate;

            intf_extn_md_propagation_t output_md_info;

            // Output lengths before and after process
            output_md_info.initial_len_per_ch_in_bytes = 0; // before process is usually zero
            output_md_info.len_per_ch_in_bytes = out_stream_ptr->buf_ptr[0].actual_data_len; // data produced on the output
            output_md_info.buf_delay_per_ch_in_bytes   = 0 ; // update this field if module has delay for this output port idx.
                                                             // this is usually zero.

            output_md_info.df = me_ptr->out_port_info[output_port_index].media_fmt.header.format_header.data_format;
            output_md_info.bits_per_sample = me_ptr->out_port_info[output_port_index].media_fmt.format.bits_per_sample;
            output_md_info.sample_rate     = me_ptr->out_port_info[output_port_index].media_fmt.format.sampling_rate;

            // This callback function will move the metadata nodes from input list to output list dependending
            // upon amount of data consumed, produced and module's delay.
            me_ptr->metadata_handler.metadata_propagate(me_ptr->metadata_handler.context_ptr,
                                                        in_stream_ptr,
                                                        out_stream_ptr,
                                                        internal_list_pptr,
                                                        me_ptr->delay_us,
                                                        &input_md_info,
                                                        &output_md_info);
         }
      }
   }

   return capi_result;
}

/* =========================================================================
 * FUNCTION : capi_ecns_handle_metadata_node
 *
 * DESCRIPTION:
 *   Placeholder function to handle each metadata node in the input stream data.
 * Currently just prints the metadata ids on the input port.
 * ========================================================================= */
capi_err_t capi_ecns_handle_metadata_node(capi_ecns_t *me_ptr, module_cmn_md_t *md_ptr)
{
   capi_err_t result = AR_EOK;

   if (MODULE_CMN_MD_ID_EOS == md_ptr->metadata_id)
   {
      // Nothing needs to be done for EOS.
      ECNS_DBG(me_ptr->miid, DBG_LOW_PRIO, "EOS metadata found on input");
   }
   if (MODULE_CMN_MD_ID_DFG == md_ptr->metadata_id)
   {
      // Nothing needs to be done for DFG.
      ECNS_DBG(me_ptr->miid, DBG_LOW_PRIO, "EOS metadata found on input");
   }
   else
   {
      // If there is any custom defined metdata, it can be handled here.
      ECNS_DBG(me_ptr->miid, DBG_LOW_PRIO, "found metadata 0x%lx on primary input", md_ptr->metadata_id);
   }

   return result;
}

/* =========================================================================
 * FUNCTION : capi_ecns_destroy_md_list
 * DESCRIPTION: Calls metadata_destroy on each node in the passed in metadata list.
 * ========================================================================= */
capi_err_t capi_ecns_destroy_md_list(capi_ecns_t *me_ptr, module_cmn_md_list_t **md_list_pptr)
{
   module_cmn_md_list_t *next_ptr = NULL;
   for (module_cmn_md_list_t *node_ptr = *md_list_pptr; node_ptr;)
   {
      bool_t IS_DROPPED_TRUE = TRUE;
      next_ptr               = node_ptr->next_ptr;
      if (me_ptr->metadata_handler.metadata_destroy)
      {
         me_ptr->metadata_handler.metadata_destroy(me_ptr->metadata_handler.context_ptr,
                                                   node_ptr,
                                                   IS_DROPPED_TRUE,
                                                   md_list_pptr);
      }
      else
      {
         ECNS_DBG(me_ptr->miid, DBG_ERROR_PRIO, "metadata handler not provided, can't drop metadata.");
         return CAPI_EFAILED;
      }
      node_ptr = next_ptr;
   }

   return CAPI_EOK;
}


/* =========================================================================
 * FUNCTION : capi_ecns_drop_input_md
 * ========================================================================= */
capi_err_t capi_ecns_drop_input_md(capi_ecns_t *me_ptr,
                                   capi_stream_data_t *input[])
{
   capi_err_t capi_result = CAPI_EOK;

   capi_stream_data_v2_t *in_stream_ptr = NULL;

   for (uint32_t input_port_idx = 0; input_port_idx < me_ptr->num_port_info.num_input_ports; input_port_idx++)
   {
      // continue to next input if buffers or metadata not provided
      if ((NULL == input[input_port_idx]) || (NULL == input[input_port_idx]->buf_ptr))
      {
         continue;
      }

      // Metadata is supported only for stream version v2
      in_stream_ptr = (capi_stream_data_v2_t *)(input[input_port_idx]);
      if (CAPI_STREAM_V2 != in_stream_ptr->flags.stream_data_version)
      {
         // doesn't support metadata propagation, do nothing.
         continue;
      }

      ECNS_DBG(me_ptr->miid,
               DBG_HIGH_PRIO,
               "destroying md for in idx %ld, md_list_ptr 0x%lx.",
               input_port_idx,
               in_stream_ptr->metadata_list_ptr);

      capi_result |= capi_ecns_destroy_md_list(me_ptr, &(in_stream_ptr->metadata_list_ptr));
   }

   return capi_result;
}

