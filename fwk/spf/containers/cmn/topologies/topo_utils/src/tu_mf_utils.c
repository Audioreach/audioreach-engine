/**
 * \file tu_mf_utils.c
 *
 * \brief
 *
 *     Implementation of topology media format utility functions.
 *     this utility maintains different media format blocks for a topo and assign them to the ports.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */


#include "topo_utils.h"

// node structure for the media format block list.
typedef struct mf_utils_node_t
{
   topo_media_fmt_t mf;        // media format
   uint32_t         ref_count; // number of ports with this media format.
} mf_utils_node_t;

/* =======================================================================
Public Function Definitions
========================================================================== */

static bool_t is_mf_changed(topo_media_fmt_t *a, topo_media_fmt_t *b)
{
   // memcmp is needed to check raw compressed format (assuming pcm part will be memset to 0).
   // memcmp alone is not good for PCM formats because we only need to check channel masks for valid number of channels.
   if (tu_has_media_format_changed(a, b) && (0 != memcmp(a, b, sizeof(topo_media_fmt_t))))
   {
      return TRUE;
   }
   return FALSE;
}

void tu_release_media_fmt(topo_mf_utils_t *me_ptr, topo_media_fmt_t **mf_pptr)
{
   if (!me_ptr || !mf_pptr)
   {
      return;
   }

   if (*mf_pptr)
   {
      mf_utils_node_t *node_obj_ptr = (mf_utils_node_t *)(*mf_pptr);
      node_obj_ptr->ref_count       = (node_obj_ptr->ref_count > 0) ? node_obj_ptr->ref_count - 1 : 0;

      if (0 == node_obj_ptr->ref_count)
      {
         if (SPF_RAW_COMPRESSED == node_obj_ptr->mf.data_format)
         {
            tu_capi_destroy_raw_compr_med_fmt(&node_obj_ptr->mf.raw);
         }

         spf_list_find_delete_node(&me_ptr->mf_node_ptr, node_obj_ptr, TRUE);

         posal_memory_free(node_obj_ptr);

         me_ptr->num_nodes--;
      }
      *mf_pptr = NULL;
   }
}

//function to set media from a port to another port.
ar_result_t tu_set_media_fmt_from_port(topo_mf_utils_t *  me_ptr,
                                       topo_media_fmt_t **dst_mf_pptr,
                                       topo_media_fmt_t * src_mf_ptr)
{
   if (!dst_mf_pptr || !src_mf_ptr || !me_ptr)
   {
      return AR_EFAILED;
   }

   // if src and dst media format are different then release the dst media format reference.
   // and assign the media fromat from the src
   if (*dst_mf_pptr != src_mf_ptr)
   {
      tu_release_media_fmt(me_ptr, dst_mf_pptr);

      mf_utils_node_t *node_obj_ptr = (mf_utils_node_t *)(src_mf_ptr);
      *dst_mf_pptr                  = src_mf_ptr;

      node_obj_ptr->ref_count++;
   }

   return AR_EOK;
}

//function to set the media format to a port
ar_result_t tu_set_media_fmt(topo_mf_utils_t *  me_ptr,
                             topo_media_fmt_t **dst_mf_pptr,
                             topo_media_fmt_t * src_mf_ptr,
                             POSAL_HEAP_ID      heap_id)
{
   if (!dst_mf_pptr || !me_ptr)
   {
      return AR_EFAILED;
   }

   topo_media_fmt_t default_media_fmt = { 0 };

   // if new media format pointer is null then assign default media format to the port
   if (NULL == src_mf_ptr)
   {
      src_mf_ptr = &default_media_fmt;
   }

   // new media fmt is same as old, nothing to do.
   if (*dst_mf_pptr && (0 == memcmp(*dst_mf_pptr, src_mf_ptr, sizeof(topo_media_fmt_t))))
   {
      return AR_EOK;
   }

   // release the existing media format.
   tu_release_media_fmt(me_ptr, dst_mf_pptr);

   // find and assign if there is an existing node with the new media format.
   spf_list_node_t *node_ptr = me_ptr->mf_node_ptr;
   while (node_ptr)
   {
      mf_utils_node_t *node_obj_ptr = (mf_utils_node_t *)node_ptr->obj_ptr;

      if (!is_mf_changed(&node_obj_ptr->mf, src_mf_ptr))
      {
         *dst_mf_pptr = &node_obj_ptr->mf;
         node_obj_ptr->ref_count++;

         return AR_EOK;
      }
      LIST_ADVANCE(node_ptr);
   }

   // if no node is not found with the new media format then allocate and assign one
   if (NULL == *dst_mf_pptr)
   {
      mf_utils_node_t *node_obj_ptr = (mf_utils_node_t *)posal_memory_malloc(sizeof(mf_utils_node_t), heap_id);
      if (!node_obj_ptr)
      {
         return AR_ENOMEMORY;
      }

      memscpy(&node_obj_ptr->mf, sizeof(node_obj_ptr->mf), src_mf_ptr, sizeof(topo_media_fmt_t));
      node_obj_ptr->ref_count = 1;
      *dst_mf_pptr            = &node_obj_ptr->mf;

      spf_list_insert_tail(&me_ptr->mf_node_ptr, node_obj_ptr, heap_id, TRUE);
      me_ptr->num_nodes++;
   }

   return AR_EOK;
}

uint32_t tu_get_max_num_channels(topo_mf_utils_t *me_ptr)
{
   // if MF is invalid, default is considered to be 1 channel
   uint32_t         max_num_channels = 1;
   spf_list_node_t *node_ptr         = me_ptr->mf_node_ptr;
   while (node_ptr)
   {
      mf_utils_node_t *node_obj_ptr = (mf_utils_node_t *)node_ptr->obj_ptr;

      uint32_t num_chs = 1;
      if (SPF_IS_PACKETIZED_OR_PCM(node_obj_ptr->mf.data_format))
      {
         num_chs = node_obj_ptr->mf.pcm.num_channels;
      }
      if (SPF_DEINTERLEAVED_RAW_COMPRESSED == node_obj_ptr->mf.data_format)
      {
         num_chs = node_obj_ptr->mf.deint_raw.bufs_num;
      }

      max_num_channels = MAX(max_num_channels, num_chs);

      LIST_ADVANCE(node_ptr);
   }

   return max_num_channels;
}

void tu_destroy_mf(topo_mf_utils_t *me_ptr)
{
   // raw compressed buf ptr should have been destroyed during port destroy.
   spf_list_delete_list_and_free_objs(&me_ptr->mf_node_ptr, TRUE);
}
