/**
 * \file capi_ecns.c
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

/*----------------------------------------------------------------------------
 * Include Files
 * -------------------------------------------------------------------------*/
#include "capi_ecns_i.h"

/*----------------------------------------------------------------------------
 * Static function declarations
 * -------------------------------------------------------------------------*/
static capi_err_t capi_ecns_process(capi_t *_pif, capi_stream_data_t *input[], capi_stream_data_t *output[]);
static capi_err_t capi_ecns_end(capi_t *_pif);
static capi_err_t capi_ecns_set_param(capi_t *                _pif,
                                      uint32_t                param_id,
                                      const capi_port_info_t *port_info_ptr,
                                      capi_buf_t *            params_ptr);
static capi_err_t capi_ecns_get_param(capi_t *                _pif,
                                      uint32_t                param_id,
                                      const capi_port_info_t *port_info_ptr,
                                      capi_buf_t *            params_ptr);
static capi_err_t capi_ecns_set_properties(capi_t *_pif, capi_proplist_t *props_ptr);
static capi_err_t capi_ecns_get_properties(capi_t *_pif, capi_proplist_t *props_ptr);

static capi_vtbl_t capi_ecns_vtbl = { capi_ecns_process,        capi_ecns_end,
                                      capi_ecns_set_param,      capi_ecns_get_param,
                                      capi_ecns_set_properties, capi_ecns_get_properties };
/* =========================================================================
 * FUNCTION : capi_ecns_get_static_properties
 *
 * DESCRIPTION:
 *    Returns the following static properties of the module like,
 *       
 *       1. CAPI_INIT_MEMORY_REQUIREMENT - This is the size of the capi_ecns_t handle.
 *          Currently the default is set to ECNS_STACK_SIZE_REQUIREMENT i.e 4K.
 *       
 *       2. CAPI_IS_INPLACE - ECNS is not inplace so returned false for this property.
 *       
 *       3. CAPI_REQUIRES_DATA_BUFFERING - Its set to False.
 *       
 *       4. CAPI_NUM_NEEDED_FRAMEWORK_EXTENSIONS - Currently only INTEF_EXTN_ECNS is supported.
 *       
 *       5. CAPI_NEEDED_FRAMEWORK_EXTENSIONS  - Returns fwk extension Ids.
 *       
 *       6. CAPI_INTERFACE_EXTENSIONS -    
  *         ECNS supports following two extensions-
 *          
 *          1. INTF_EXTN_DATA_PORT_OPERATION - Data port operation -
 *             Since ECNS multi port module, has minimum 2 inputs. It needs to handle
 *             data port operations like OPEN, START, STOP, CLOSE.
 *
 *          2. INTF_EXTN_METADATA - Metdata propagation -
 *             This extension is needed for propagating metadata. Since ECNS is a
 *             multi-port, module must support metadata propagation.
 *
 *             In Single input single output case, this extension is optional - framework
 *             can handle metadata if module doesn't support this extension.
 *
 *          NOTE: 
               INTF_EXTN_IMCL - Inter module communication interface extension -
 *             In this example, IMCL extension is not enabled, if the module supports
 *             IMCL extension this can enabled.
 * ========================================================================= */
capi_err_t capi_ecns_get_static_properties(capi_proplist_t *init_set_properties, capi_proplist_t *static_properties)
{
   capi_err_t result = CAPI_EFAILED;

   if (NULL != static_properties)
   {
      result = capi_ecns_handle_get_properties(NULL, static_properties);
   }
   else
   {
      AR_MSG(DBG_ERROR_PRIO, "capi_ecns: get static prop bad ptrs");
   }

   return result;
}
/* =========================================================================
 * FUNCTION : capi_ecns_init
 *
 * DESCRIPTION:
 *    This is the first function called by the framework after creating the capi
 *  handle [*_pif]. Module must intialize the handle with init set properties
 *  payload passed by the frame work. Init set properties sets,
 * 
 *   1) CAPI_HEAP_ID - which must be used by the module for any dyanmic memory
 *   			  allocations.
 * 
 *   2) CAPI_PORT_NUM_INFO - maximum number of input/output ports that can be opened
 *      on the module.
 * 
 *   3) CAPI_EVENT_CALLBACK_INFO - sets callback handle for raising module events.
 * 
 *   4) CAPI_MODULE_INSTANCE_ID - Module instance ID as assigned in the ACDB graphs.
 *      This is helpul when printing the debug messages.
 * ========================================================================= */
capi_err_t capi_ecns_init(capi_t *_pif, capi_proplist_t *init_set_properties)
{
   capi_err_t result = CAPI_EOK;

   if (NULL == _pif || (NULL == init_set_properties))
   {
      AR_MSG(DBG_ERROR_PRIO, "capi_ecns: Error! null pointer.");
      return CAPI_EBADPARAM;
   }

   capi_ecns_t *me_ptr = (capi_ecns_t *)_pif;
   memset(me_ptr, 0, sizeof(capi_ecns_t));

   // apply init time set properties
   result = capi_ecns_handle_set_properties(me_ptr, init_set_properties);
   if (result == CAPI_EUNSUPPORTED)
   {
      // unsupported error must be ignored for init set properties
      result = CAPI_EOK;
   }
   else if (CAPI_FAILED(result)) // for any other failures return error.
   {
      // Free the allocated capi memory
      capi_ecns_end(_pif);
      ECNS_DBG(me_ptr->miid, DBG_ERROR_PRIO, "Init Failed!");
   }

   // intialize the capi handle.
   me_ptr->vtbl_ptr   = &capi_ecns_vtbl;
   me_ptr->is_enabled = TRUE;

   ECNS_DBG(me_ptr->miid, DBG_HIGH_PRIO, "init done with result %d", result);

   return result;
}

/* =========================================================================
 * FUNCTION : capi_ecns_set_properties
 *
 * DESCRIPTION:
 *   This function is called by framework to set runtime properties. Currently
 *   only runtime property handled by this module is
 * 
 *     1. CAPI_INPUT_MEDIA_FORMAT_V2 - 
 *          Media format on input ports. Media format is set only on the input ports.
 *        This is a per port operation and valid port info is passed by the framework.
 * 
 *     2. CAPI_ALGORITHMIC_RESET -
 *          Perform algorithmic reset during module STOP/START sequence. This is
 *        a per port operation and valid port info is passed by the framework.
 *          
 *          Algo reset indicates there is a data discontinuity, alogrithm may
 *        have to be reset depending upon the usecase. Please refer capi documentation
 *        for further information on algo reset.
 *           
 * ========================================================================= */
static capi_err_t capi_ecns_set_properties(capi_t *_pif, capi_proplist_t *props_ptr)
{
   if (NULL == _pif || (NULL == props_ptr))
   {
      AR_MSG( DBG_ERROR_PRIO, "capi_ecns: null pointer error");
      return CAPI_EBADPARAM;
   }

   return capi_ecns_handle_set_properties((capi_ecns_t *)_pif, props_ptr);
}

/* =========================================================================
 * FUNCTION : capi_ecns_get_properties
 *
 * DESCRIPTION:
 *   Returns module properties during runtime. Currently the following properties
 *  are expected to support at runtime.
 * 
 *  1. CAPI_OUTPUT_MEDIA_FORMAT_V2 -
 *        Framework may query media format during propagation.
 * ========================================================================= */
static capi_err_t capi_ecns_get_properties(capi_t *_pif, capi_proplist_t *props_ptr)
{
   capi_err_t result = CAPI_EOK;
   if (NULL == _pif)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi_ecns: Error!! NULL pointer error");
      return CAPI_EBADPARAM;
   }

   result = capi_ecns_handle_get_properties((capi_ecns_t *)_pif, props_ptr);
   
   return result;
}

/* =========================================================================
 * FUNCTION : capi_ecns_process

 * DESCRIPTION:

  Process input data & metadata from primary and reference path. And produces echo cancelled/noise supressed output.

  Data processing:
  Data is processed in following steps,

   STEP 1 - Check if the necessary conditions are met to process. For example,
      i. Library has been initialized
     ii. Module is enabled.
    iii. Mandatory calibration has been received to continue with the process 

   STEP 2 - Check input and output port states
      i. Check if mandatory input ports are started. For example, reference input is usually optional
         input for ECNS and Primary input is mandatory.
     ii. Check if mandatory outputs are opened and started.
    iii. Media format is received on the mandatory inputs.
     iv. Module can also check if sufficient data is present on the input ports, if data
         is not sufficient it can return CAPI_ENEEDMORE error indicating framework that
         it requires more data. 
         
         NOTE: If the module has raised input threshold event, the framework always 
         provides threshold amount of data [i.e frame size]. Exceptions are when EOS and 
         End of frame flag is set module can recevie partial frame.

   STEP 3 - LIbrary process.
      i. Process capi inputs and produce echo cancelled output, write it to capi output buffers.

   STEP 4 - 
      i. Update output acual data lengths.
     ii. For inputs, acutal_data_len after process indicates amount of data consumed by the module.
         For output, acutal_data_len retured to framework after process indicates amount of data
         produced.

   STEP 5 - Propagate metadata
      i. Based on the amount of data consumed and produced, we need to propagagte Metadata on each input port.
         Check the function capi_ecns_propagate_metadata() definition for details.

   STEP 6 - Update output timestamps.
      i. Update the timestamps on output buffers by accounting for the module's data path delay.
 * ========================================================================= */
static capi_err_t capi_ecns_process(capi_t *_pif, capi_stream_data_t *input[], capi_stream_data_t *output[])
{
   capi_err_t   result            = CAPI_EOK;
   capi_ecns_t *me_ptr            = (capi_ecns_t *)_pif;
   bool_t       is_ref_in_started = TRUE;

#ifdef CAPI_ENCS_DEBUG_VERBOSE
   ECNS_DBG(me_ptr->miid, DBG_HIGH_PRIO, "Entering ecns process");
#endif


   /* =========================================================================
    * STEP 1. Check if the necessary conditions are met to process.
    * ========================================================================= */

   // Return if the module is disabled
   if (!me_ptr->is_enabled)
   {
      ECNS_DBG(me_ptr->miid, DBG_MED_PRIO, "ECNS module is disabled, dropping input data");
      capi_ecns_drop_input_md(me_ptr, input);
      return CAPI_EOK;
   }

   /* Module can also check if,
   1. Library has been initialized
   2. Media format is received on the input ports.
   3. Mandatory calibration has been received to continue with the process */

   /* ====================== End of STEP 1 ====================== */


   /* =========================================================================
    * STEP 2 - Check if the Inputs and output ports are in the expected states.
    * ========================================================================= */

   /* Check the Primary input port states. The following three cases needs to be handled depending
      upon module/algorithm expectations,
         1. Primary input port is not OPENED.
         2. Primary input port is in STOPPED state.
         2. Primary input port is OPENED and in started state. */
   uint32_t pri_in_index = 0;
   bool_t   is_pri_in_started = FALSE;
   if (CAPI_EOK != (result = ecns_get_input_port_arr_idx(me_ptr, ECNS_PRIMARY_INPUT_STATIC_PORT_ID, &pri_in_index)))
   {
      ECNS_DBG(me_ptr->miid, DBG_ERROR_PRIO, "Primary input port is CLOSED ");
      
      /* Handle if Primary input is CLOSED. [Usually returns from process since Primary input is mandatory for process. ].
         If a port is not opened, capi input buffers will not be given by the framework.  */
   }
   else if (!is_ecns_input_port_state(me_ptr, pri_in_index, DATA_PORT_STATE_STARTED))
   {
      // Check if Primary input port is STARTED
      ECNS_DBG(me_ptr->miid, DBG_HIGH_PRIO, " Primary input port is STOPPED ");

      /* Handle if Primary port is in STOPPED state,
         If a port is in STOPPED state, capi input buffers will not be given by the framework.*/
   }
   else
   {
      /* Handle if Primary port is in OPENED and in STARTED state */
      is_pri_in_started = TRUE;
   }

   /* Check the Primary output port states. The following three cases needs to be handled,
         1. Primary output port is not OPENED.
         2. Primary output port is in STOPPED state.
         2. Primary output port is OPENED and in started state. */
   uint32_t pri_out_index      = 0;
   bool_t   is_pri_out_started = FALSE;
   if (CAPI_EOK != (result = ecns_get_output_port_arr_idx(me_ptr, ECNS_PRIMARY_OUTPUT_STATIC_PORT_ID, &pri_out_index)))
   {
      ECNS_DBG(me_ptr->miid, DBG_ERROR_PRIO, "Primary output port is CLOSED");

      /* Handle Primary output being CLOSED. [Usually returns from process since Primary output is mandatory for
         process. ]. Or module can process input and drop data at the algorithm's output. */
   }
   else if (!is_ecns_output_port_state(me_ptr, pri_out_index, DATA_PORT_STATE_STARTED))
   {
      // Check if Primary output port is STARTED
      ECNS_DBG(me_ptr->miid, DBG_HIGH_PRIO, "Primary output port is STOPPED.");

      /* Handle if Primary port is in STOPPED state,
         If a output port is in STOPPED state, capi buffers will not be given by the framework.*/
   }
   else
   {
      /* Handle if Primary port is in OPENED and in STARTED state */
      is_pri_out_started = TRUE;
   }

   /* Check the Reference input port states. The following three cases needs to be handled depending
      upon module/algorithm expectations. In general, reference input is not mandatory for EC modules,
      so module can internally push zeros if the port is stopped.
         1. Reference input port is not OPENED.
         2. Reference input port is in STOPPED state.
         2. Reference input port is OPENED and in started state. */
   uint32_t ref_in_index = 0;
   is_ref_in_started     = FALSE;
   if (CAPI_EOK != (result = ecns_get_input_port_arr_idx(me_ptr, ECNS_REFERENCE_INPUT_STATIC_PORT_ID, &ref_in_index)))
   {
      // Handle if reference input is CLOSED
   }
   else if (!is_ecns_input_port_state(me_ptr, ref_in_index, DATA_PORT_STATE_STARTED))
   {
      // Handle if reference input is STOPPED
      ECNS_DBG(me_ptr->miid, DBG_MED_PRIO, "Primary output port is STOPPED.");
   }
   else
   {
      /* Handle if reference port is in OPENED and in STARTED state */
      is_ref_in_started = TRUE;
   }
   /* ====================== End of STEP 2 ====================== */



   /* =========================================================================
    * STEP 3 - Prepare the buffer pointers and call library process
    * ========================================================================= */

   /* Prepare input buffers, 
        1. Check and validate the input buffer data lengths. If module raised threshold for this input, the same amount
           of data is expected for this input.
        2. input[pri_in_index]->bufs_num is based on the latest recent input media format set. */
   uint32_t pri_in_bytes_per_ch_before   = 0;
   if(is_pri_in_started && input[pri_in_index]->buf_ptr)
   {
       pri_in_bytes_per_ch_before = input[pri_in_index]->buf_ptr[0].actual_data_len;
   }

   uint32_t pri_in_bytes_consumed_per_ch = 0;

   /* Prepare Reference input buffers, 
        1. Check and validate the input buffer data lengths. If module raised threshold for this input, the same amount
           of data is expected for this input.
        2. input[ref_in_index]->bufs_num is based no of channels on the latest input media format set.*/
   uint32_t ref_in_bytes_per_ch_before   = 0;
   if(is_ref_in_started && input[ref_in_index]->buf_ptr)
   {
       ref_in_bytes_per_ch_before = input[ref_in_index]->buf_ptr[0].actual_data_len;
   }

   uint32_t ref_in_bytes_consumed_per_ch = 0;

   /* Prepare output buffers, 
        1. Check and validate the output buffer lengths.
        2. output[pri_in_index]->bufs_num is based no of channels raised in the output media format event.*/
   uint32_t pri_out_max_buf_len_per_ch   = 0;
   if(is_pri_out_started && output[pri_out_index]->buf_ptr)
   {
       pri_out_max_buf_len_per_ch = output[pri_out_index]->buf_ptr[0].max_data_len;
   }

   uint32_t prim_output_bytes_produced = 0;

   /* Most ECNS algorithms can only process at a fixed frame size. Despite raising an input threshold, modules
    * may still be passed partial or no data if it is an end_of_frame case. In that case the ECNS module should
    * drop the frame.
    */
   bool_t not_enough_ip_data = FALSE;

   if(is_pri_in_started)
   {
	  uint32_t pri_frame_samples = (me_ptr->in_port_info[pri_in_index].media_fmt.format.sampling_rate / 1000) *
                                ECNS_PROCESS_FRAME_SIZE_IN_10MS;
	  uint32_t pri_bytes_per_sample = me_ptr->in_port_info[pri_in_index].media_fmt.format.bits_per_sample >> 3;
      uint32_t pri_frame_bytes_per_ch = pri_bytes_per_sample * pri_frame_samples;

	  if(input[pri_in_index]->flags.end_of_frame && (pri_in_bytes_per_ch_before < pri_frame_bytes_per_ch))
	  {
		  ECNS_DBG(me_ptr->miid,
				   DBG_MED_PRIO,
				   "Partial data on pri in, %ld bpc given, %ld bpc threshold, dropping data.",
				   pri_in_bytes_per_ch_before,
				   pri_frame_bytes_per_ch);

		  not_enough_ip_data = TRUE;
	  }
   }

   if(is_ref_in_started)
   {
	  uint32_t ref_frame_samples = (me_ptr->in_port_info[ref_in_index].media_fmt.format.sampling_rate / 1000) *
									ECNS_PROCESS_FRAME_SIZE_IN_10MS;

	  uint32_t ref_bytes_per_sample = me_ptr->in_port_info[ref_in_index].media_fmt.format.bits_per_sample >> 3;
      uint32_t ref_frame_bytes_per_ch = ref_bytes_per_sample * ref_frame_samples;


	  if(input[ref_in_index]->flags.end_of_frame && (ref_in_bytes_per_ch_before < ref_frame_bytes_per_ch))
	  {
		  ECNS_DBG(me_ptr->miid,
				   DBG_MED_PRIO,
				   "Partial data on ref in, %ld bpc given, %ld bpc threshold, dropping data.",
				   ref_in_bytes_per_ch_before,
				   ref_frame_bytes_per_ch);

		  not_enough_ip_data = TRUE;
	  }
   }

   if(is_pri_in_started && not_enough_ip_data)
   {
	  //If EOF is encountered on any port, we are equired to either consume or drop all data and metadata.
	  //We also need to propagate EOF and clear it on input port to avoid hang states.

	  output[pri_out_index]->flags.end_of_frame = input[pri_in_index]->flags.end_of_frame; //Here we are propagating
      //EOF from primary input to output. Based on requirement it can be decided to propagate from primary or secondary

	  input[pri_in_index]->flags.end_of_frame = FALSE;
	  if(is_ref_in_started)
	  {
		input[ref_in_index]->flags.end_of_frame = FALSE;
	  }
      capi_ecns_drop_input_md(me_ptr, input);
      return CAPI_EOK;
   }


#ifdef CAPI_ENCS_DEBUG_VERBOSE
   ECNS_DBG(me_ptr->miid,
            DBG_HIGH_PRIO,
            "before ecns process: pri_in_bytes_per_ch_before = %lu pri_in_bytes_consumed_per_ch = %lu "
            "ref_in_bytes_per_ch_before = %lu ref_in_bytes_consumed_per_ch = %lu  pri_out_max_buf_len_per_ch = %lu  "
            "prim_output_bytes_produced = %lu",
            pri_in_bytes_per_ch_before,
            pri_in_bytes_consumed_per_ch,
            ref_in_bytes_per_ch_before,
            ref_in_bytes_consumed_per_ch,
            pri_out_max_buf_len_per_ch,
            prim_output_bytes_produced);
#endif

   /* ECNS alogrithm can process the input data at this point and produce ouptut.

      After process, Module must update the length of data consumed on the inputs and also update length of data
      produced on the output buffers. */

   /* For the sake of this example, the following code implements pass through mode by copying data from
      input to output buffers. If the reference input is started drop the data and mark it consumed.*/

   // If primary input and output are started copy from input ch buffer to output ch buffer.
   if (is_pri_in_started && is_pri_out_started)
   {
      // Iterate through each channel in input.
      for (uint32_t ch_idx = 0; (ch_idx < input[pri_out_index]->bufs_num) && (ch_idx < output[pri_out_index]->bufs_num);
           ch_idx++)
      {

         pri_in_bytes_consumed_per_ch = memscpy(output[pri_out_index]->buf_ptr[ch_idx].data_ptr,
                                                pri_out_max_buf_len_per_ch,
                                                input[pri_in_index]->buf_ptr[ch_idx].data_ptr,
                                                pri_in_bytes_per_ch_before);
         
         // cache data produced on the output.
         prim_output_bytes_produced = pri_in_bytes_consumed_per_ch;

         // if reference input is started 
         if(is_ref_in_started)
         {
            ref_in_bytes_consumed_per_ch = ref_in_bytes_per_ch_before;
         }
      }
   }

   // Return error if process if there is a fatal failure in the process.
   if (CAPI_EOK != result)
   {
      ECNS_DBG(me_ptr->miid, DBG_ERROR_PRIO, "ECNS library process failed");
      return CAPI_EFAILED;
   }

#ifdef CAPI_ENCS_DEBUG_VERBOSE
   ECNS_DBG(me_ptr->miid,
            DBG_HIGH_PRIO,
            "after ecns process: prim_in_bytes_before = %lu prim_input_bytes_consumed = %lu "
            "ref_input_bytes_before = %lu ref_input_bytes_consumed = %lu  prim_output_max_buf_size = %lu  "
            "prim_output_bytes_produced = %lu",
            pri_in_bytes_per_ch_before,
            pri_in_bytes_consumed_per_ch,
            ref_in_bytes_per_ch_before,
            ref_in_bytes_consumed_per_ch,
            pri_out_max_buf_len_per_ch,
            prim_output_bytes_produced);
#endif
   /* ====================== End of STEP 3 ====================== */

   /* =========================================================================
    * STEP 4 - Update output actual data length and input actual data length.
    * ========================================================================= */

   // update inputs actual data lengths indicating the amount of data consumed on Primary port.
   if (is_pri_in_started && input[pri_in_index] && input[pri_in_index]->buf_ptr)
   {
      for (uint32_t ch_idx = 0; ch_idx < me_ptr->in_port_info[pri_in_index].media_fmt.format.num_channels; ch_idx++)
      {
         input[pri_in_index]->buf_ptr[ch_idx].actual_data_len = pri_in_bytes_consumed_per_ch;
      }
   }

   // update inputs actual data lengths indicating the amount of data consumed on Reference port.
   if (is_ref_in_started  && input[ref_in_index] && input[ref_in_index]->buf_ptr)
   {
      for (uint32_t ch_idx = 0; ch_idx < me_ptr->in_port_info[ref_in_index].media_fmt.format.num_channels; ch_idx++)
      {
         input[ref_in_index]->buf_ptr[ch_idx].actual_data_len = ref_in_bytes_consumed_per_ch;
      }
   }

   // update inputs actual data lengths indicating the amount of data produced on the output port.
   if (is_pri_out_started && output[pri_out_index] && output[pri_out_index]->buf_ptr)
   {
      for (uint32_t ch_idx = 0; ch_idx < me_ptr->out_port_info[pri_out_index].media_fmt.format.num_channels; ch_idx++)
      {
         output[pri_out_index]->buf_ptr[ch_idx].actual_data_len = prim_output_bytes_produced;
      }
   }

   /* ====================== End of STEP 4 ====================== */

   /* =========================================================================
    * STEP 5 - Propagate metadata
    * ========================================================================= */

   /* Propagate metadata from inputs to output port.
   
      NOTE: In this example, we are propagating metadata from primary input to primary output.
      And droppping metadata received on the Reference ports since downstream of ECNS
      doesn't expect metadata from RX path. Module developer must reconsider functionl
      expectations and can modify this behavior if necessary.
   */
   result |= capi_ecns_propagate_metadata(me_ptr, 
                                          input, 
                                          output, 
                                          pri_in_bytes_per_ch_before, 
                                          pri_in_bytes_consumed_per_ch, 
                                          ref_in_bytes_per_ch_before,  
                                          ref_in_bytes_consumed_per_ch);

   /* ====================== End of STEP 5 ====================== */

   /* =========================================================================
    * STEP 6 - Update output ports timestamps
    * ========================================================================= */

   // Update the timestamp on the output buffer if any data has been produced.
   // if the module has a buffering delay, output will currently produced is delayed by the amount of data in the delay buffer.
   if (is_pri_in_started && prim_output_bytes_produced)
   {
      output[pri_out_index]->flags = input[pri_in_index]->flags;
      if (input[pri_in_index]->flags.is_timestamp_valid)
      {
         output[pri_out_index]->timestamp = input[pri_in_index]->timestamp - me_ptr->delay_us;
      }
   }
   /* ====================== End of STEP 6 ====================== */

   return result;
}

/* =========================================================================
 * FUNCTION : capi_ecns_end
 *
 * DESCRIPTION:
 *    Free all the dynamic resources allocated by the module. Do NOT free the
 *  capi handle it will be freed by the framework.
 * ========================================================================= */
static capi_err_t capi_ecns_end(capi_t *_pif)
{
   if (NULL == _pif)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi_encs: End received bad pointer");
      return CAPI_EBADPARAM;
   }
   capi_ecns_t *me_ptr = (capi_ecns_t *)(_pif);
   me_ptr->vtbl_ptr    = NULL;

   // destroy input ports
   for (uint32_t port_idx = 0; port_idx < me_ptr->num_port_info.num_input_ports; port_idx++)
   {
      capi_encs_handle_port_close(me_ptr, port_idx, TRUE);
   }

   // destroy output ports
   for (uint32_t port_idx = 0; port_idx < me_ptr->num_port_info.num_output_ports; port_idx++)
   {
      capi_encs_handle_port_close(me_ptr, port_idx, FALSE);
   }

   // Deinitalize ecns library resources
   capi_ecns_deinit_library(me_ptr);

   ECNS_DBG(me_ptr->miid, DBG_HIGH_PRIO, "end done");

   return CAPI_EOK;
}

/* =========================================================================
 * FUNCTION : capi_ecns_get_param
 *
 * DESCRIPTION:
 *  Module must return the param configuration based on current state of the module.
 *  Module developer must add case statements for each param ID supported by the module.
 * ========================================================================= */
static capi_err_t capi_ecns_get_param(capi_t *                _pif,
                                      uint32_t                param_id,
                                      const capi_port_info_t *port_info_ptr,
                                      capi_buf_t *            params_ptr)
{
   if (NULL == _pif || NULL == params_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi_encs: get param received bad pointers");
      return CAPI_EBADPARAM;
   }

   capi_err_t   result = CAPI_EOK;
   capi_ecns_t *me_ptr = (capi_ecns_t *)(_pif);

   switch (param_id)
   {
   case PARAM_ID_ECNS_DUMMY_CFG:
   {
      if (params_ptr->max_data_len < sizeof(param_id_ecns_dummy_cfg_t))
      {
         ECNS_DBG(me_ptr->miid, DBG_ERROR_PRIO, "get param (%#lx) failed. Size(%ld)", param_id, params_ptr->max_data_len);
         result = CAPI_ENEEDMORE;
         break;
      }

      // populate the get param return payload
      param_id_ecns_dummy_cfg_t *ecns_1 = (param_id_ecns_dummy_cfg_t *)params_ptr->data_ptr;
      ecns_1->feature_mask      = me_ptr->feature_mask;

      // update the actual length of returned payload.
      params_ptr->actual_data_len = sizeof(param_id_module_enable_t);

      ECNS_DBG(me_ptr->miid, DBG_HIGH_PRIO, "get param enable(%ld)", ecns_1->feature_mask);

      break;
   }
   default:
   {
      /*Check if the ECNS library supports the given get param ID*/
      if (0) // if supported
      {
         /* if the get param is supported copy the return payload to params_ptr->payload_ptr and update the 
            payload size params_ptr->actual_data_len.
            
            Make sure that params_ptr->actual_data_len < params_ptr->max_data_len */
         
         params_ptr->actual_data_len = 0; // populate appropriate value
         result = CAPI_EOK;
      }
      else
      {
         ECNS_DBG(me_ptr->miid, DBG_HIGH_PRIO, "unsupported get param 0x%lx", param_id);
         result = CAPI_EUNSUPPORTED;
         break;
      }
   }
   }

   ECNS_DBG(me_ptr->miid,
            DBG_HIGH_PRIO,
            "Get param id (%#lx) param size %lu completed with result(%#lx)",
            param_id,
            params_ptr->actual_data_len,
            result);

   return result;
}

/* =========================================================================
 * FUNCTION : capi_ecns_set_param
 *
 * DESCRIPTION:
 *  Calibrates the module with the given set param payload. All the param ID
 * calibrations supported by the module must be handled here. Currently these
 * are the calibrations supported by the module -
 *
 *   1. PARAM_ID_MODULE_ENABLE - This is a realtime calibration param, useful
 *      to enable/disable module runtime.
 * 
 *   2. PARAM_ID_ECNS_DUMMY_CFG - Dummy calibration param.
 *
 *  Apart from calibration set param handles the interface extensions as well,
 *   1. INTF_EXTN_PARAM_ID_METADATA_HANDLER -
 *         Gets the metadata handling obj from the framework.
 *
 *   2. INTF_EXTN_PARAM_ID_DATA_PORT_OPERATION -
 *        Handles dynamic input/output port OPEN, CLOSE, START and STOP cmds.
 *      This will affect the way module functions, like if the reference input
 *      closes dynamically, library cannot perform echo cancellation anymore.
 * 
 *      Refer capi_ecns_handle_intf_extn_data_port_operation() for details.
 * ========================================================================= */
static capi_err_t capi_ecns_set_param(capi_t *                _pif,
                                      uint32_t                param_id,
                                      const capi_port_info_t *port_info_ptr,
                                      capi_buf_t *            params_ptr)
{
   if (NULL == _pif || NULL == params_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi_ecns: set param received bad pointers");
      return CAPI_EBADPARAM;
   }

   capi_err_t   result = CAPI_EOK;
   capi_ecns_t *me_ptr = (capi_ecns_t *)(_pif);

   switch (param_id)
   {
   case INTF_EXTN_PARAM_ID_METADATA_HANDLER:
   {
      if (params_ptr->actual_data_len < sizeof(intf_extn_param_id_metadata_handler_t))
      {
         ECNS_DBG(me_ptr->miid, DBG_ERROR_PRIO,
                "Param id 0x%lx Bad param size %lu",
                (uint32_t)param_id,
                params_ptr->actual_data_len);
         result = CAPI_ENEEDMORE;
         break;
      }

      intf_extn_param_id_metadata_handler_t *payload_ptr =
         (intf_extn_param_id_metadata_handler_t *)params_ptr->data_ptr;

      /* Cache metadata handler in the module handle, this handler will be used propagate/create/destroy metadata. */
      me_ptr->metadata_handler = *payload_ptr;

      break;
   }
   case INTF_EXTN_PARAM_ID_DATA_PORT_OPERATION:
   {
      result = capi_ecns_handle_intf_extn_data_port_operation(me_ptr, params_ptr);
      break;
   }
   case PARAM_ID_MODULE_ENABLE:
   {
      if (params_ptr->actual_data_len < sizeof(param_id_module_enable_t))
      {
         ECNS_DBG(me_ptr->miid, DBG_ERROR_PRIO,
                "set param (%#lx) failed. expected param_size(%ld), actual_param_size:(%ld)",
                param_id,
                sizeof(param_id_module_enable_t),
                params_ptr->actual_data_len);
         result = CAPI_ENEEDMORE;
         break;
      }

      // Extract param config from set param payload
      param_id_module_enable_t *cfg_ptr = (param_id_module_enable_t *)params_ptr->data_ptr;
      
      // If the module is disable the process can be simply skipped.
      me_ptr->is_enabled                = cfg_ptr->enable;

      ECNS_DBG(me_ptr->miid, DBG_HIGH_PRIO, "Enabled = %lu ", cfg_ptr->enable);

      break;
   }
   case PARAM_ID_ECNS_DUMMY_CFG:
   {
      if (params_ptr->actual_data_len < sizeof(param_id_ecns_dummy_cfg_t))
      {
         ECNS_DBG(me_ptr->miid, DBG_ERROR_PRIO,
                "set param (%#lx) failed. expected param_size(%ld), actual_param_size:(%ld)",
                param_id,
                sizeof(param_id_ecns_dummy_cfg_t),
                params_ptr->actual_data_len);
         result = CAPI_ENEEDMORE;
         break;
      }

      // Extract param config from set param payload
      param_id_ecns_dummy_cfg_t *ecns_1 = (param_id_ecns_dummy_cfg_t *)params_ptr->data_ptr;
      
      /* TODO: Handle param specific action here*/
      me_ptr->feature_mask = ecns_1->feature_mask;

      ECNS_DBG(me_ptr->miid, DBG_HIGH_PRIO, "feature_mask 0x%lx is set", ecns_1->feature_mask);

      break;
   }
   default:
   {
      // If the param is supported handle the set param and return success.
      // if the param is not supported return UNSUPPORTED.
      if(0)
      {
         /* handle if the param is supported */
      }
      else
      {
         ECNS_DBG(me_ptr->miid, DBG_HIGH_PRIO, "unsupported set param 0x%lx", param_id);
         result = CAPI_EUNSUPPORTED;
         break;
      }
   }
   }

   ECNS_DBG(me_ptr->miid, DBG_HIGH_PRIO, "Set param for param id (0x%lx) completed with result(%#lx)", param_id, result);

   return result;
}
