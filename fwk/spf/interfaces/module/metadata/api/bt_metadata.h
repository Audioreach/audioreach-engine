#ifndef BT_METADATA_H
#define BT_METADATA_H

/**
 *   \file bt_metadata.h
 *
 *   \brief
 *        defines custom bt metadata defined and used only by bt modules.
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

#include "ar_defs.h"
/**************************************** Sideband info - Begin ***********************************/
/**
    Metadata ID for bt sideband info. module_cmn_md_t structure
    has to set the metadata_id field to this id when the metadata
    is for sending bt sideband info. The module also should check this ID before operating on
    bt sideband info structures.

    This metadata is created by the cop depacketizer and sent to the decoders which parse
    and then destroy it.
 */

#define MD_ID_BT_SIDEBAND_INFO 0x0A001036

#define BT_SIDEBAND_DATA_MAX_LENGTH 256

/** Data structure for the stream's metadata */
typedef struct md_bt_sideband_info_t
{
   uint8_t sideband_id;
   /**< Side band ID. Not necessarily aligned to 16 bits. */

   uint8_t sideband_length;
   /**< Side band data length. Not necessarily aligned to 16 bits. */

   uint8_t sideband_data[BT_SIDEBAND_DATA_MAX_LENGTH];
} md_bt_sideband_info_t;

#define MD_ID_BT_SIDEBAND_INFO_V2 0x0A001006

/** Data structure for the stream's metadata */
typedef struct md_bt_sideband_info_v2_t
{
   uint8_t sideband_id;
   /**< Side band ID. Not necessarily aligned to 16 bits. */

   bool_t channel_mask_valid;
   /**< is channel mask below valid */

   uint16_t sideband_length;
   /**< Length of the sideband payload. */

   uint32_t channel_mask_lsw;
   /**< channel mask indicating for which all channels sideband is applicable
    * Channel mask is obtained by shifting with channel map names
    * defined in media_fmt_api_basic.h::pcm_channel_map. E.g. channel map = PCM_CHANNEL_L, then mask = 1<<PCM_CHANNEL_L.
    * channel mask if zero implies all channels. For sidebands which don't rely on stream-id, channel mask is zero.
    * For sidebands which rely on stream-id, if channel mask is zero, then it sideband is applicable to all channels.*/

   uint32_t channel_mask_msw;
/**< MSW of channel mask.*/
#ifdef __H2XML__
   uint8_t sideband_data[0];
/**< variable length sideband payload. Note that for sideband ids which have stream-id, this payload also contains
 * stream-id,
 *   but receiver may ignore it and use channel_mask. */
#endif

} md_bt_sideband_info_v2_t;

/**************************************** Sideband info - End *************************************/

/**************************************** COP PCM Frame Length - Begin ***********************************/

/*
    Metadata ID for number of PCM samples (per channel) required to produce one encoded frame.
    It is sent by the encoder to the packetizer/conn proxy sink.
    A PCM sample is defined by a media format also set through metadata #MD_ID_COP_PCM_MEDIA_FORMAT.
    If this parameter is not set, the packetizer/conn proxy sink will not be able to process data. */

#define MD_ID_COP_PCM_FRAME_LENGTH 0x0A001037

/** Data structure for the stream's metadata */
typedef struct md_cop_pcm_frame_length_t
{
   uint32_t pcm_frame_length_per_ch;
   /**< Length of the frame in PCM samples. */
} md_cop_pcm_frame_length_t;

/**************************************** COP PCM Frame Length - End ***********************************/

/**************************************** COP PCM Media Fmt - Begin ***********************************/
/*
    Metadata ID for media format of the PCM data used for encoding
    It is sent by the encoder to the packetizer/conn proxy sink.
    If this parameter is not set, the packetizer/conn proxy sink will not be able to process data. */

#define MD_ID_COP_PCM_MEDIA_FORMAT 0x0A001038

/** Data structure for the stream's metadata */
typedef struct md_cop_pcm_media_format_t
{
   uint32_t sampling_rate;
   /**< Number of samples per second */

   uint16_t bits_per_sample;
   /**< Number of bits per sample per channel.*/

   uint16_t channels;
   /**< Number of PCM channels.*/

   uint16_t channel_type[CAPI_MAX_CHANNELS];
   /**< Array of channel types. */

} md_cop_pcm_media_format_t;

/**************************************** COP PCM Media Fmt - End ***********************************/

/**************************************** COP PCM Zero Padding - Begin ***********************************/

/*
 *  Metadata ID which provides setting and configuring of zero padding
 *  It is sent by the encoder to the packetizer/conn proxy sink modules */

#define MD_ID_COP_CONFIG_ZERO_PADDING 0x0A001039

/** Data structure for the stream's metadata */
typedef struct md_cop_config_zero_padding_t
{
   uint32_t zero_padding_config;
   /**< Configures zero padding.

    @values (for packetizer/conn proxy sink set)
    - 0 -- Disable zero padding
    - 1 -- Enable zero padding (Default) */

} md_cop_config_zero_padding_t;

/**************************************** COP PCM Zero Padding - End ***********************************/

/**************************************** Conn proxy Pre-buffering - Begin ***********************************/
/*
 *  Metadata ID which indicates whether pre-buffering must be enabled/disabled in the conn proxy sink.
 *  It is sent by the encoder to the conn proxy sink module
 *  If not sent, by default pre-buffering will be enabled*/

#define MD_ID_CONN_PROXY_CONFIG_PREBUFFER 0x0A001065

/** Data structure for the prebuffer metadata */
typedef struct md_conn_proxy_config_prebuffer_t
{
   uint32_t prebuffer_config;
   /**< Configures prebuffering.

    @values (for packetizer set)
    - 0 -- Disable prebuffering
    - 1 -- Enable prebuffering (Default) */

} md_conn_proxy_config_prebuffer_t;

/**************************************** Conn proxy  Pre-buffering - End ***********************************/

/**************************************** Conn proxy PLC - Begin ***********************************/
/*
 * Metadata ID which indicates whether packet loss concealment is needed or not
 * This is sent by Conn proxy source module to decoders
*/
#define MD_ID_CONN_PROXY_PACKET_LOSS_CONCEALMENT 0x0A001066

/** Data structure for metadata to indicate packet loss concealment */
typedef struct md_conn_proxy_config_plc_t
{
   uint8_t stream_id;
   /**< stream-id of the from-air stream */

   uint8_t status; 
   /**< status field. See enum conn_proxy_quality_mask_status_t */

} md_conn_proxy_config_plc_t;

/**************************************** Conn proxy PLC - End ***********************************/

#ifdef __cplusplus
}
#endif /*__cplusplus */

// clang-format on

#endif // BT_METADATA_H
