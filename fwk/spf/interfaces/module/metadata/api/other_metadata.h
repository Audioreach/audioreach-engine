#ifndef OTHER_METADATA_H
#define OTHER_METADATA_H

/**
 *   \file other_metadata.h
 *
 *   \brief
 *        defines custom metadata defined and used only by specific modules.
 *
 *    module_cmn_metadata.h defines the parent structure types and other metadata
 *    that are used commonly across modules.
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

#include "bt_metadata.h"


/**************************************** Delay Marker - Begin ***********************************/

/**
    Metadata ID for Delay Marker. module_cmn_md_t structure
    has to set the metadata_id field to this id when the metadata
    is Delay Marker. The module should check this ID before operating on
    Delay Marker Metadata structures.

    Delay Marker Metadata is a special marker inserted by a module in the chain
    that will eventually be intercepted by another one downstream. It is used to
    calculate the delay between insertion and interception in the path. The
    payload object is of type data_marker_md_id_delay_marker_t.
 */
#define DATA_MARKER_MD_ID_DELAY_MARKER 0x0A001035

typedef struct data_marker_md_id_delay_marker_t
{
   uint64_t insertion_time;
   /*Wall Clock timer value when inserted into the metadata list*/

   uint32_t token;
   /*Token to uniquely identify the metadata*/

   uint32_t frame_counter;
   /** frame counter which indicates frame number at which this marker was inserted. */
} data_marker_md_id_delay_marker_t;

/**************************************** Delay Marker - End *************************************/

/**************************************** TTR - Begin ***********************************/

/**
  Metadata ID for propagating TTR(Time to Render). module_cmn_md_t structure
  has to set the metadata_id field to this id when the metadata
  is TTR.

  In Rx path, TTR indicates the TS at which the Time renderer module should start Rendering this buffer.
  In Rx path, TTR indicates the TS at which the  Mailbox module should push this packet onto the mailbox shared memory.
*/
#define MD_ID_TTR 0x0A00103C

typedef enum ttr_packet_token_t {
   TTR_PACKET_TOKEN_P1, /* 20 ms Packet 1*/
   TTR_PACKET_TOKEN_P2  /* 20 ms Packet 2*/
} ttr_packet_token_t;

typedef struct md_ttr_t md_ttr_t;

/** Data structure for the TTR metadata */
struct md_ttr_t
{
   uint64_t ttr;
   /**< Time to Render represents the Qtimer TS at which the buffer needs to be rendered . */

   ttr_packet_token_t packet_token;
   /**< Token to identify the packet sequence in a VFR cycle. */

   uint32_t resync;
   /**< Resync flag to be set by the smart sync module post VFR resync.
        This can be used by the subsequent modules in the chain to identify
        the packets that originated post resync. */
};

/**************************************** TTR - End *************************************/

/**************************************** RESET_SESSION_TIME  - Begin ***********************************/

/**
    Metadata ID read by SPR indicating to reset session time. This is currently sent by the gapless module
    along with the first buffer of a new stream. SPR will re-sync session time.
 */
#define MODULE_CMN_MD_ID_RESET_SESSION_TIME 0x0A00104B

/* No payload. */

/**************************************** RESET_SESSION_TIME  - End *************************************/

/**************************************** SCALE_SESSION_TIME  - Begin ***********************************/

/**
    Metadata ID read by SPR indicating the appropriate scale factor for session time. This is currently sent by
    the TSM module along with the first output frame with a new scale factor. When received by SPR, the session
    time is scaled appropriately based on the payload information.
 */
#define MODULE_CMN_MD_ID_SCALE_SESSION_TIME 0x0A00100E

/* Data structure for scale session time metadata */
typedef struct md_session_time_scale_t
{
	uint32_t speed_factor;
   /* Speed factor in Q24 to be used for scaling the session time */

}md_session_time_scale_t;

/**************************************** SCALE_SESSION_TIME  - End *************************************/

/**************************************** Encoder Frame Length Info - Begin ***********************************/

/* Metadata contains the encoders output max frame size and the frame duration.

   Currently,this is being used only for G722 (MEDIA_FMT_ID_G722). G722 encoder generates this metadata and sends it
   downstream to inform Dam module about the worst case frame len. Using this info Dam calculates no of frames to buffer
   and size of the circular buffer for buffering g722 bitstream. */

#define MD_ID_ENCODER_FRAME_LENGTH_INFO 0x0A001BAE

/** Data structure for the stream's metadata */
typedef struct md_encoder_pcm_frame_length_t
{
   uint32_t bitstream_format;
   /* Bit stream format.*/

   uint32_t output_pcm_frame_duration_in_us;
   /**< Encoder's per output frame PCM frame duration in micro seconds*/

   uint32_t max_output_frame_len_in_bytes;
   /**< Encoder's worst case output frame length in bytes*/
} md_encoder_pcm_frame_length_t;

/**************************************** Encoder Frame Length Info - End ***********************************/
/**************************************** Silence data - Begin ***********************************/

/**
    A module such as RAT, indicates beginning and ending of inserted silence data to its downstream.
    Silence can either be due to absence of input data or due to input itself being indicated as silence_data.
    There is no payload associated with this MD.
 */
#define MD_ID_BEGIN_INSERTED_SILENCE_DATA 0x0A001067

#define MD_ID_END_INSERTED_SILENCE_DATA 0x0A001068

/**************************************** Silence data - End *************************************/

#ifdef __cplusplus
}
#endif /*__cplusplus */

// clang-format on

#endif // OTHER_METADATA_H
