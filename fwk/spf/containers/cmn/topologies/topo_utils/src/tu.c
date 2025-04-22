/**
 * \file tu.c
 *
 * \brief
 *
 *     Implementation of topology utility functions.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */
// clang-format on=========================================================*/
// clang-format on

#include "topo_utils.h"

/* =======================================================================
Public Function Definitions
========================================================================== */
/**
 * Convert subgraph operation to port state. Assume only one subgraph operation
 * is set in the mask.
 *
 * sg_op = mask of topo_sg_operation_t
 */
topo_port_state_t tu_sg_op_to_port_state(uint32_t sg_op)
{
   if (sg_op & TOPO_SG_OP_START)
   {
      return TOPO_PORT_STATE_STARTED;
   }
   if (sg_op & TOPO_SG_OP_STOP)
   {
      return TOPO_PORT_STATE_STOPPED;
   }
   if (sg_op & TOPO_SG_OP_PREPARE)
   {
      return TOPO_PORT_STATE_PREPARED;
   }
   if (sg_op & TOPO_SG_OP_SUSPEND)
   {
      return TOPO_PORT_STATE_SUSPENDED;
   }
   return TOPO_PORT_STATE_INVALID;
}

/**
 * Checks if the two media formats are equivalent.
 */
bool_t tu_has_media_format_changed(topo_media_fmt_t *a1, topo_media_fmt_t *b1)
{
   uint32_t i = 0;

   if (a1 == b1)
   {
      return FALSE;
   }

   // if a & b data formats are different - then has changed.
   if ((a1->data_format != b1->data_format))
   {
      return TRUE;
   }

   // if a is not PCM/packetized, then generally marked as changed.
   if (!(SPF_IS_PACKETIZED_OR_PCM(a1->data_format)))
   {
      return TRUE;
   }

   topo_pcm_pack_med_fmt_t *a = &a1->pcm;
   topo_pcm_pack_med_fmt_t *b = &b1->pcm;

   if ((a->endianness != b->endianness) || (a->sample_rate != b->sample_rate) || (a->bit_width != b->bit_width) ||
       (a->bits_per_sample != b->bits_per_sample) || (a->interleaving != b->interleaving) ||
       (a->num_channels != b->num_channels) || (a1->fmt_id != b1->fmt_id) || (a->q_factor != b->q_factor))
   {
      return TRUE;
   }
   for (i = 0; i < a->num_channels; i++)
   {
      if (a->chan_map[i] != b->chan_map[i])
      {
         return TRUE;
      }
   }
   return FALSE;
}

ar_result_t tu_convert_media_fmt_spf_msg_to_topo(uint32_t                log_id,
                                                 spf_msg_media_format_t *spf_media_format_ptr,
                                                 topo_media_fmt_t *      topo_media_format_ptr,
                                                 POSAL_HEAP_ID           heap_id)
{
   ar_result_t result = AR_EOK;

   memset((void *)topo_media_format_ptr, 0, sizeof(topo_media_fmt_t));

   if (SPF_IS_PACKETIZED_OR_PCM(spf_media_format_ptr->df))
   {
      spf_std_media_format_t *std_ptr       = (spf_std_media_format_t *)&spf_media_format_ptr->payload_start;
      topo_media_format_ptr->pcm.endianness = TOPO_LITTLE_ENDIAN;
      topo_media_format_ptr->pcm.interleaving = (std_ptr->interleaving == SPF_INTERLEAVED)
                                                ? TOPO_INTERLEAVED : TOPO_DEINTERLEAVED_PACKED;

      if (SPF_FLOATING_POINT == spf_media_format_ptr->df)
      {
         topo_media_format_ptr->pcm.bit_width = std_ptr->bits_per_sample;
      }
      else
      {
         topo_media_format_ptr->pcm.bit_width = TOPO_QFORMAT_TO_BIT_WIDTH(std_ptr->q_format);
      }

      topo_media_format_ptr->pcm.num_channels = std_ptr->num_channels;
      topo_media_format_ptr->pcm.sample_rate  = std_ptr->sample_rate;

      // internally it's always 16 or 32.
      topo_media_format_ptr->pcm.bits_per_sample = std_ptr->bits_per_sample;
      topo_media_format_ptr->pcm.q_factor        = std_ptr->q_format;
      // topo_media_format_ptr->is_signed = pInPcmFmtBlk->is_signed; ignore is_signed
      memscpy(topo_media_format_ptr->pcm.chan_map,
              sizeof(topo_media_format_ptr->pcm.chan_map),
              std_ptr->channel_map,
              sizeof(std_ptr->channel_map));
   }
   else if (SPF_RAW_COMPRESSED == spf_media_format_ptr->df)
   {
      result |= tu_capi_create_raw_compr_med_fmt(log_id,
                                                 (uint8_t *)&spf_media_format_ptr->payload_start,
                                                 spf_media_format_ptr->actual_size,
                                                 spf_media_format_ptr->fmt_id,
                                                 topo_media_format_ptr,
                                                 TRUE /*with header*/,
                                                 heap_id);
   }
   // Buffer is not supported for deinterleaved raw compr formats
   else if (SPF_DEINTERLEAVED_RAW_COMPRESSED == spf_media_format_ptr->df)
   {
      topo_deint_raw_med_fmt_t *raw_dst_ptr     = (topo_deint_raw_med_fmt_t *)&spf_media_format_ptr->payload_start;
      topo_media_format_ptr->deint_raw.bufs_num = raw_dst_ptr->bufs_num;
      TOPO_MSG(log_id,
               DBG_HIGH_PRIO,
               "Received SPF_DEINTERLEAVED_RAW_COMPRESSED mf, num bufs %d",
               raw_dst_ptr->bufs_num);
      for (uint32_t i = 0; i < raw_dst_ptr->bufs_num; i++)
      {
         topo_media_format_ptr->deint_raw.ch_mask[i].channel_mask_lsw = raw_dst_ptr->ch_mask[i].channel_mask_lsw;
         topo_media_format_ptr->deint_raw.ch_mask[i].channel_mask_msw = raw_dst_ptr->ch_mask[i].channel_mask_msw;
      }
   }
   else
   {
      TOPO_MSG(log_id, DBG_ERROR_PRIO, "Error:data_format %lu currently unsupported", spf_media_format_ptr->df);
      result = AR_EUNSUPPORTED;
   }

   topo_media_format_ptr->data_format = spf_media_format_ptr->df;
   topo_media_format_ptr->fmt_id      = spf_media_format_ptr->fmt_id;

   return result;
}

void tu_copy_media_fmt(topo_media_fmt_t *dst_ptr, topo_media_fmt_t *src_ptr)
{
   // if dst already has raw compr, free it.
   // In case of bypass module, src raw buf ptr and dest can be same, should
   // not free.
   if (dst_ptr->raw.buf_ptr)
   {
      if (dst_ptr->raw.buf_ptr != src_ptr->raw.buf_ptr)
      {
         tu_capi_destroy_raw_compr_med_fmt(&dst_ptr->raw);
      }
      else
      {
#ifdef VERBOSE_DEBUGGING
         AR_MSG(DBG_LOW_PRIO, "Not freeing destination raw mf memory, since src has same reference.");
#endif
      }
   }

   // for Raw, simply copy pointers.
   *dst_ptr = *src_ptr;

   // after copying make ptr in src as NULL.
   if (SPF_RAW_COMPRESSED == src_ptr->data_format)
   {
      src_ptr->raw.buf_ptr  = NULL;
      src_ptr->raw.buf_size = 0;
   }
}

/**
 * Also destroys if already existing raw buf_ptr is non-NULL when media_fmt_ptr->data_format is raw_compressed.
 * Hence media_fmt_ptr->data_format must still have value corresponding to old MF .
 */
ar_result_t tu_capi_create_raw_compr_med_fmt(uint32_t          log_id,
                                             uint8_t *         raw_fmt_ptr,
                                             uint32_t          raw_fmt_size,
                                             uint32_t          fmt_id,
                                             topo_media_fmt_t *media_fmt_ptr,
                                             bool_t            with_header,
                                             POSAL_HEAP_ID     heap_id)
{
   capi_err_t result = AR_EOK;

   capi_set_get_media_format_t *      main_fmt;
   capi_raw_compressed_data_format_t *raw_fmt;
   uint32_t                           buf_size;
   uint32_t                           size_for_header = (with_header ? TOPO_MIN_SIZE_OF_RAW_MEDIA_FMT : 0);
   buf_size                                           = size_for_header + raw_fmt_size;
   topo_raw_med_fmt_t *raw_ptr                        = &media_fmt_ptr->raw;

   if (raw_ptr->buf_ptr && (raw_ptr->buf_size != buf_size))
   {
      // if data format was other than raw compressed then raw_ptr->buf_ptr is pointed to some other mem, as raw_ptr
      // is coming from union
      tu_capi_destroy_raw_compr_med_fmt(raw_ptr);
   }

   if (NULL == raw_ptr->buf_ptr)
   {
      raw_ptr->buf_ptr = (uint8_t *)posal_memory_malloc(buf_size, heap_id);
      if (NULL == raw_ptr->buf_ptr)
      {
         TOPO_MSG(log_id,
                  DBG_ERROR_PRIO,
                  "Insufficient memory to allocate a memory for raw media fmt ptr %lu bytes ",
                  buf_size);
         return AR_ENOMEMORY;
      }
   }

   if (with_header)
   {
      main_fmt = (capi_set_get_media_format_t *)raw_ptr->buf_ptr;
      raw_fmt  = (capi_raw_compressed_data_format_t *)(raw_ptr->buf_ptr + sizeof(capi_set_get_media_format_t));
      main_fmt->format_header.data_format = CAPI_RAW_COMPRESSED;
      raw_fmt->bitstream_format           = fmt_id;
   }

   if (raw_fmt_size)
   {
      memscpy(raw_ptr->buf_ptr + size_for_header, buf_size, raw_fmt_ptr, raw_fmt_size);
   }

   raw_ptr->buf_size = buf_size;

   return capi_err_to_ar_result(result);
}

ar_result_t tu_capi_destroy_raw_compr_med_fmt(topo_raw_med_fmt_t *raw_ptr)
{
   if (raw_ptr && (raw_ptr->buf_ptr))
   {
      posal_memory_free(raw_ptr->buf_ptr);
      raw_ptr->buf_ptr  = NULL;
      raw_ptr->buf_size = 0;
   }
   return AR_EOK;
}

/**
 * Function to check if a given media format is valid
 */
bool_t topo_is_valid_media_fmt(topo_media_fmt_t *med_fmt_ptr)
{
   if (NULL == med_fmt_ptr)
   {
      return FALSE;
   }

   if (SPF_UNKNOWN_DATA_FORMAT == med_fmt_ptr->data_format)
   {
      return FALSE;
   }

   if (SPF_RAW_COMPRESSED == med_fmt_ptr->data_format)
   {
      return TRUE;
   }

   if (SPF_DEINTERLEAVED_RAW_COMPRESSED == med_fmt_ptr->data_format)
   {
      if (0 == med_fmt_ptr->deint_raw.bufs_num)
      {
         return FALSE;
      }
      return TRUE;
   }

   if ((0 == med_fmt_ptr->pcm.num_channels) || (0 == med_fmt_ptr->pcm.sample_rate))
   {
      return FALSE;
   }

   if (SPF_FIXED_POINT == med_fmt_ptr->data_format)
   {
      if ((0 == med_fmt_ptr->pcm.bits_per_sample) || (0 == med_fmt_ptr->pcm.bit_width) ||
          (0 == med_fmt_ptr->pcm.q_factor))
      {
         return FALSE;
      }
   }
   if (SPF_FLOATING_POINT == med_fmt_ptr->data_format)
   {
      if ((0 == med_fmt_ptr->pcm.bits_per_sample) || (0 == med_fmt_ptr->pcm.bit_width))
      {
         return FALSE;
      }
   }

   return TRUE;
}

/**
 * The state of ports connected to other subgraphs is defined by a combination.
 * A port is running (STARTED) only if its connected port is also STARTED.
 * A port is PREPARED only if its connected side either PREPARED or STARTED.
 * Otherwise a port is in stop state.
 */
topo_port_state_t tu_get_downgraded_state_(topo_port_state_t self_port_state, topo_port_state_t connected_port_state)
{
   topo_port_state_t downgraded_state = TOPO_PORT_STATE_INVALID;

   switch (self_port_state)
   {
      case TOPO_PORT_STATE_STOPPED:
      {
         switch (connected_port_state)
         {
            case TOPO_PORT_STATE_STOPPED:
            case TOPO_PORT_STATE_SUSPENDED:
            case TOPO_PORT_STATE_PREPARED:
            case TOPO_PORT_STATE_STARTED:
            {
               downgraded_state = TOPO_PORT_STATE_STOPPED;
               break;
            }
            default:
            {
               downgraded_state = TOPO_PORT_STATE_INVALID;
               break;
            }
         }
         break;
      }
      case TOPO_PORT_STATE_SUSPENDED:
      {
         switch (connected_port_state)
         {
            case TOPO_PORT_STATE_STOPPED:
            {
               downgraded_state = TOPO_PORT_STATE_STOPPED;
               break;
            }
            case TOPO_PORT_STATE_SUSPENDED:
            case TOPO_PORT_STATE_PREPARED: // TODO: can prepare be retained as prepare ?
            case TOPO_PORT_STATE_STARTED:
            {
               downgraded_state = TOPO_PORT_STATE_SUSPENDED;
               break;
            }
            default:
            {
               downgraded_state = TOPO_PORT_STATE_INVALID;
               break;
            }
         }
         break;
      }
      case TOPO_PORT_STATE_PREPARED:
      {
         switch (connected_port_state)
         {
            case TOPO_PORT_STATE_STOPPED:
            {
               downgraded_state = TOPO_PORT_STATE_STOPPED;
               break;
            }
            case TOPO_PORT_STATE_SUSPENDED:
            {
               downgraded_state = TOPO_PORT_STATE_SUSPENDED; // TODO: can prepare be retained as prepare ?
               break;
            }
            case TOPO_PORT_STATE_PREPARED:
            case TOPO_PORT_STATE_STARTED:
            {
               downgraded_state = TOPO_PORT_STATE_PREPARED;
               break;
            }
            default:
            {
               downgraded_state = TOPO_PORT_STATE_INVALID;
               break;
            }
         }
         break;
      }
      case TOPO_PORT_STATE_STARTED:
      {
         switch (connected_port_state)
         {
            case TOPO_PORT_STATE_STOPPED:
            {
               downgraded_state = TOPO_PORT_STATE_STOPPED;
               break;
            }
            case TOPO_PORT_STATE_SUSPENDED:
            {
               downgraded_state = TOPO_PORT_STATE_SUSPENDED;
               break;
            }
            case TOPO_PORT_STATE_PREPARED:
            {
               downgraded_state = TOPO_PORT_STATE_PREPARED;
               break;
            }
            case TOPO_PORT_STATE_STARTED:
            {
               downgraded_state = TOPO_PORT_STATE_STARTED;
               break;
            }
            default:
            {
               downgraded_state = TOPO_PORT_STATE_INVALID;
               break;
            }
         }
         break;
      }
      default:
      {
         downgraded_state = TOPO_PORT_STATE_INVALID;
      }
   }

   return downgraded_state;
}
