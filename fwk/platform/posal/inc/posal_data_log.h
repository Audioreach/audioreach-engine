/**
 * \file posal_data_log.h
 * \brief Posal data log apis.
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef POSAL_DATA_LOG_H
#define POSAL_DATA_LOG_H

#include "posal_types.h"
#include "ar_error_codes.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

#define SPF_LOG_PREFIX "SPF_log_parser: "
/*==========================================================================
  Define constants
========================================================================== */
/** Log data formats. */
typedef enum posal_data_log_format_t
{
   LOG_DATA_FMT_PCM = 0,   /**< PCM data format. */
   LOG_DATA_FMT_BITSTREAM, /**< Bitstream data format. */
   LOG_DATA_FMT_RAW        /**< Raw data format. */
} posal_data_log_format_t;

typedef enum posal_data_log_mode_t
{
   /* logs the data only after the buffer is completely 7filled */
   LOG_DEFAULT,

   /* logs the data immediately */
   LOG_IMMEDIATE
} posal_data_log_mode_t;

/** PCM data information for the logging utility user. */
typedef struct posal_data_log_pcm_info_t
{
   uint32_t sampling_rate;
   /**< PCM sampling rate.
        @values 8000 Hz, 48000 Hz, etc. */

   uint16_t num_channels;
   /**< Number of channels in the PCM stream. */

   uint8_t bits_per_sample;
   /**< Bits per sample for the PCM data. */

   uint8_t interleaved;
   /**< Specifies whether the data is interleaved. */

   uint8_t q_factor;
   /**< q factor information for log packet. */

   uint8_t data_format;
   /**< data_format information for log packet. */

   uint16_t *channel_mapping;
   /**< Array of channel mappings. */
} posal_data_log_pcm_info_t;

/** Format of the data being logged: PCM or bitstream. */
typedef struct posal_data_log_fmt_info_t
{
   posal_data_log_pcm_info_t pcm_data_fmt; /**< Format of the PCM data. */
   uint32_t                  media_fmt_id; /**< Format of the bitstream data. */
} posal_data_log_fmt_info_t;

/** Log header and data payload information for the logging utility user. */
typedef struct posal_data_log_info_t
{
   uint32_t log_code;
   /**< log code for the log packet. */

   int8_t *buf_ptr;
   /**< Pointer to the buffer to be logged. */

   uint32_t buf_size;
   /**< Size of the payload to be logged, in bytes. */

   uint32_t session_id;
   /**< Session ID for the log packet. */

   uint32_t log_tap_id;
   /**< GUID for the tap point. */

   uint64_t log_time_stamp;
   /**< Timestamp in microseconds. */

   posal_data_log_format_t data_fmt;
   /**< Data format for the log packet. */

   posal_data_log_fmt_info_t data_info;
   /**< Pointer to the data packet information. */

   uint32_t *seq_number_ptr;
   /**< Reference to sequence number variable shared by client. */
} posal_data_log_info_t;

/*==========================================================================
  Function Declarations
  ========================================================================== */
/**
  This function checks if the cog code is enabled .

  @return
  TRUE if the log code is enabled and FALSE if it is disabled.
  @dependencies
  None
 */
bool_t posal_data_log_code_status(uint32_t log_code);

/**
  This function gives the maximum packet size allowed for logging.

  @dependencies
  None
 */
uint32_t posal_data_log_get_max_buf_size();

/**
  Allocates a log packet for PCM/bitstream data logging.

  @datatypes
  #log_data_format

  @param[in] buf_Size   Size of the data payload, excluding the log header.
  @param[in] log_code   Log code for this log packet.
  @param[in] data_fmt   PCM or bitstream data format.

  @return
  Pointer to payload of the allocated log packet. Returns NULL if buffer allocation fails
  or log code is disabled.

  @dependencies
  None.
 */
void *posal_data_log_alloc(uint32_t buf_Size, uint32_t log_code, posal_data_log_format_t data_fmt);

/**
  Populates the log header and data payload of an allocated log packet and
  commits the packet for logging.

  @param[in] log_pkt_payload_ptr   Pointer to payload of the allocated log packet.
  @param[in] log_tap_id            Tap point ID of the log packet.
  @param[in] session_id            Session ID of the log packet.
  @param[in] buf_size              Payload size of the log packet.


  @return
  0 -- Success
  @par
  Nonzero -- Failure

  @dependencies
  None
 */
ar_result_t posal_data_log_commit(void *log_pkt_payload_ptr, posal_data_log_info_t *log_info_ptr);

/**
 * Allocates the log packet, populates the log header and data payload, and
  commits the packet for logging.

  @datatypes
  #log_info

  @param[in] log_info_ptr  Pointer to the object containing the log header and
                           data payload information for the logging utility
                           client.
  @return
  0 -- Success
  @par
  Nonzero -- Failure

  @dependencies
  None
*/
ar_result_t posal_data_log_alloc_commit(posal_data_log_info_t *log_info_ptr);

/**
  This function frees the data log buffer in case of error scenerio.

  @param[in] log_ptr : payload of the data log buffer to be freed

  @return
  None.

  @dependencies
  None
 */
void posal_data_log_free(void *log_pkt_payload_ptr);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef POSAL_DATA_LOG_H