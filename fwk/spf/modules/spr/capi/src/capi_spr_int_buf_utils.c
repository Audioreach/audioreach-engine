/**
 *   \file capi_spr_int_buf_utils.c
 *   \brief
 *        This file contains CAPI implementation of Splitter Renderer Module internal buffer utilities
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "capi_spr_i.h"

/*==============================================================================
   Local Definitions
==============================================================================*/

/*==============================================================================
   Function Implementation
==============================================================================*/

/*------------------------------------------------------------------------------
  Function name: capi_spr_destroy_int_buf_list
  Destroys the internal buffer list along with metadata
 * ------------------------------------------------------------------------------*/
capi_err_t capi_spr_destroy_int_buf_list(capi_spr_t *         me_ptr,
                                         spr_int_buffer_t *   int_buf_ptr,
                                         capi_media_fmt_v2_t *media_fmt_ptr)
{
   capi_err_t result = CAPI_EOK;

   // Validate the incoming list pointers
   if (!(int_buf_ptr && int_buf_ptr->buf_list_ptr && media_fmt_ptr))
   {
      return CAPI_EOK;
   }

   capi_stream_data_t **  DUMMY_OUT_PPTR   = NULL;
   bool_t                 DROP_MD          = TRUE;
   bool_t                 POOL_USED        = TRUE;
   bool_t                 IGNORE_HEAD_NODE = FALSE;
   bool_t                 DROP_DATA        = TRUE;
   capi_stream_data_v2_t *int_buf_node_ptr =
      (capi_stream_data_v2_t *)spf_list_pop_head(&int_buf_ptr->buf_list_ptr, POOL_USED);

   while (int_buf_node_ptr)
   {
      // Drop metadata. Call spr_handle_metadata with dummy output stream argument
      result = spr_handle_metadata(me_ptr, (capi_stream_data_t **)&int_buf_node_ptr, DUMMY_OUT_PPTR, DROP_MD);

      if (CAPI_FAILED(result))
      {
         SPR_MSG(me_ptr->miid,
                 DBG_HIGH_PRIO,
                 "Warning: Failed to handle metadata in the int buf with error %x",
                 result);
      }

      capi_spr_destroy_int_buf_node(me_ptr, int_buf_ptr, int_buf_node_ptr, IGNORE_HEAD_NODE, media_fmt_ptr, DROP_DATA);

      // Pop the next element from the list.
      int_buf_node_ptr = (capi_stream_data_v2_t *)spf_list_pop_head(&int_buf_ptr->buf_list_ptr, POOL_USED);
   }

   return CAPI_EOK;
}
