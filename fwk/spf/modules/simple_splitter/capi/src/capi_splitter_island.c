/**
 * \file capi_splitter_island.c
 * \brief
 *     Source file to implement the CAPI Interface for Simple Splitter (SPLITTER) Module.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "capi_splitter.h"
#include "capi_splitter_utils.h"
#include "spf_list_utils.h"
#include "posal_timer.h"

static capi_vtbl_t vtbl = { capi_splitter_process,        capi_splitter_end,
                            capi_splitter_set_param,      capi_splitter_get_param,
                            capi_splitter_set_properties, capi_splitter_get_properties };

capi_vtbl_t *capi_splitter_get_vtbl()
{
   return &vtbl;
}

/*------------------------------------------------------------------------
  Function name: capi_splitter_process
  DESCRIPTION: Processes an input buffer and generates an output buffer.
  -----------------------------------------------------------------------*/
capi_err_t capi_splitter_process(capi_t *_pif, capi_stream_data_t *input[], capi_stream_data_t *output[])
{
   capi_err_t       result = CAPI_EOK;
   capi_splitter_t *me_ptr = (capi_splitter_t *)_pif;
   uint32_t         data_len;

   POSAL_ASSERT(_pif);
   POSAL_ASSERT(input[0]);
   POSAL_ASSERT(output[0]);

   if (!me_ptr->flags.is_in_media_fmt_set)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi_splitter: Input Media format not set yet");
      return CAPI_EFAILED;
   }

   capi_stream_data_v2_t *in_stream_ptr = (capi_stream_data_v2_t *)input[0];
   if (in_stream_ptr->metadata_list_ptr)
   {
      result = handle_metadata(me_ptr, input, output);
   }

   data_len = input[0]->buf_ptr[0].actual_data_len;
   // each ch buffer on the in port
   for (uint32_t i = 0; i < input[0]->bufs_num; i++)
   {
      for (uint32_t j = 0; j < me_ptr->num_out_ports; j++)
      {
         // Due to fwk optimization, proceed only when first channel data pointer is non NULL.
         if (DATA_PORT_STATE_STARTED != me_ptr->out_port_state_arr[j].state || (NULL == output[j]) ||
             (NULL == output[j]->buf_ptr) || (NULL == output[j]->buf_ptr[i].data_ptr) ||
             (NULL == output[j]->buf_ptr[0].data_ptr) )
         {
            continue;
         }
         // copy all words but EOS (EOS was already copied conditionally in handle_metadata)
         bool_t eos_flag = output[j]->flags.marker_eos;
         output[j]->flags.word |= input[0]->flags.word;
         output[j]->flags.marker_eos = eos_flag;

         if (SPLITTER_OUT_PORT_DEFAULT_TS_PROPAGATION == me_ptr->out_port_state_arr[j].flags.ts_cfg)
         {
            output[j]->timestamp = input[0]->timestamp;
         }
         else if (SPLITTER_OUT_PORT_STM_TS_PROPAGATION == me_ptr->out_port_state_arr[j].flags.ts_cfg)
         {
             if (me_ptr->ts_payload.ts_ptr)
             {
                 output[j]->flags.is_timestamp_valid = me_ptr->ts_payload.ts_ptr->is_valid;
                 output[j]->timestamp = me_ptr->ts_payload.ts_ptr->timestamp;
             }

             else
             {
                 output[j]->flags.is_timestamp_valid = FALSE;
             }
         }
         else
         {
            // TODO: In signal triggered container, fwk is asignign timestamp for the buffers where it is not valid.
            // this should be removed so that timestamp can remain invalid.
            output[j]->flags.is_timestamp_valid = FALSE;
         }
#ifdef SPLITTER_DBG_LOW
         AR_MSG_ISLAND(DBG_HIGH_PRIO,
                       "input_ts_lsb %lu, output_port_index %lu, output_ts_lsb %lu, is_valid %lu",
                       (uint32_t)input[0]->timestamp,
                       j,
                       (uint32_t)output[j]->timestamp,
                       output[j]->flags.is_timestamp_valid);
#endif

         // if input MF is unpacked v2 read/update lengths only for the first ch's buffer
         uint32_t actual_data_len;
         uint32_t max_data_len_per_buf = output[j]->buf_ptr[0].max_data_len;
         if (input[0]->buf_ptr[i].data_ptr == output[j]->buf_ptr[i].data_ptr)
         {
            actual_data_len = MIN(max_data_len_per_buf, data_len);
         }
         else
         {
            actual_data_len =
               memscpy(output[j]->buf_ptr[i].data_ptr, max_data_len_per_buf, input[0]->buf_ptr[i].data_ptr, data_len);
         }

         // for unpacked v2 read/update buf lengths only for the first ch
         if (0 == i)
         {
            output[j]->buf_ptr[0].actual_data_len = actual_data_len;
         }
      }
   }

   // clear the flags once we propagate to the outputs.
   input[0]->flags.end_of_frame = FALSE;

   return result;
}
