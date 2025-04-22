/**
 * \file gpr_log_generic.c
 * \brief
 *    This file contains logger implementation
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/******************************************************************************
 * Includes                                                                    *
 *****************************************************************************/
#include "ar_osal_mem_op.h"
#include "ar_osal_mutex.h"
#include "ar_osal_heap.h"
#include "ar_osal_error.h"
#include "ar_osal_timer.h"
#include "ar_guids.h"
#include "ar_msg.h"
#include "gpr_api_i.h"
#include "gpr_log.h"
#include "gpr_msg_if.h"
#include "gpr_log_i.h"

/*****************************************************************************
 * Variables                                                                 *
 ****************************************************************************/

static ar_osal_mutex_t   gpr_log_history_mutex;
static gpr_log_history_t gpr_log_history_generic;

/*****************************************************************************
 * Function Definitions                                                      *
 ****************************************************************************/

GPR_INTERNAL uint32_t gpr_log_init(void)
{
   uint32_t result = AR_EOK;

   result = ar_osal_mutex_create(&gpr_log_history_mutex);
   if (AR_EOK != result)
   {
      if (NULL != gpr_log_history_mutex)
      {
         (void)ar_osal_mutex_destroy(gpr_log_history_mutex);
         gpr_log_history_mutex = NULL;
      }
      return AR_EFAILED;
   }

   (void)ar_mem_cpy(gpr_log_history_generic.start_marker, sizeof(gpr_log_history_generic.start_marker), "GPRSTART", 8);
   gpr_log_history_generic.size = GPR_LOG_HISTORY_BYTE_SIZE_V;
   (void)ar_mem_set(gpr_log_history_generic.log, 0xFF, sizeof(gpr_log_history_generic.log));
   (void)ar_mem_cpy(gpr_log_history_generic.end_marker, sizeof(gpr_log_history_generic.end_marker), "GPR STOP", 8);

   gpr_log_history_generic.log_payloads = 0;

#ifdef GPR_LOG_PACKET_PAYLOAD
   gpr_log_history_generic.log_payloads = 1;
#endif

   return result;
}

GPR_INTERNAL uint32_t gpr_log_deinit(void)
{
   AR_MSG(DBG_HIGH_PRIO, "GPR Log: Deinitializing gpr logger ");
   if (NULL != gpr_log_history_mutex)
   {
      (void)ar_osal_mutex_destroy(gpr_log_history_mutex);
   }
   return AR_EOK;
}

static void gpr_log_circ_write(uint8_t *const       buf,
                               uint32_t const       buf_size,
                               uint32_t            *offset,
                               const uint8_t *const data,
                               uint32_t const       data_size)
{
   uint32_t endpos;
   uint32_t segment_size;
   uint32_t remaining_size;

   remaining_size = ((data_size > buf_size) ? buf_size : data_size);
   endpos         = (*offset + remaining_size);
   segment_size   = (buf_size - *offset);

   /* Check whether buffer is full or not */
   if (endpos < buf_size)
   {
      (void)ar_mem_cpy(&buf[*offset], segment_size, data, remaining_size);
      *offset = endpos;
   }
   else
   {
      if (endpos == buf_size)
      {
         /* Reset offset to 0 if end position and buffer size are equals */
         (void)ar_mem_cpy(&buf[*offset], segment_size, data, remaining_size);
         *offset = 0;
      }
      else
      {
         /* Since current data cannot be fitted into the buffer, write only
          * data which cab be fitted at he end of buffer and write rest of the data
          * from begining of the buffer
          */
         (void)ar_mem_cpy(&buf[*offset], segment_size, data, segment_size);
         remaining_size -= segment_size;
         *offset = remaining_size;
         (void)ar_mem_cpy(buf, buf_size, &data[segment_size], remaining_size);
      }
   }
}

GPR_INTERNAL int32_t gpr_log_packet(gpr_packet_t *packet)
{
   uint32_t packet_size;
   uint32_t marker;

   uint64_t time = ar_timer_get_time_in_us();

   packet_size = GPR_PKT_GET_PACKET_BYTE_SIZE(packet->header);

   (void)ar_osal_mutex_lock(gpr_log_history_mutex);

   marker = GPR_LOG_SYNC_WORD;

   gpr_log_circ_write(gpr_log_history_generic.log,
                      gpr_log_history_generic.size,
                      &gpr_log_history_generic.offset,
                      ((uint8_t *)&marker),
                      sizeof(marker));
   /* Write the sync word. */
   gpr_log_circ_write(gpr_log_history_generic.log,
                      gpr_log_history_generic.size,
                      &gpr_log_history_generic.offset,
                      ((uint8_t *)&time),
                      sizeof(time));
   /* Write the timestamp. */
#ifdef GPR_LOG_PACKET_PAYLOAD
   gpr_log_circ_write(gpr_log_history_generic.log,
                      gpr_log_history_generic.size,
                      &gpr_log_history_generic.offset,
                      ((uint8_t *)packet),
                      packet_size);
#else
   gpr_log_circ_write(gpr_log_history_generic.log,
                      gpr_log_history_generic.size,
                      &gpr_log_history_generic.offset,
                      ((uint8_t *)packet),
                      sizeof(gpr_packet_t));
#endif
   /* Write the packet. */
   gpr_log_history_generic.offset = (((gpr_log_history_generic.offset + 3) >> 2) << 2);
   /* Align to 32-bits. */

   (void)ar_osal_mutex_unlock(gpr_log_history_mutex);

   gpr_log_diag_packet(packet, packet_size);

   return AR_EOK;
}
