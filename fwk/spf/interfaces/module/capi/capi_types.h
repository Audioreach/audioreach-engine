#ifndef CAPI_TYPES_H
#define CAPI_TYPES_H

/**
 * \file capi_types.h
 * \brief
 *      This file defines the basic data types for the Common Audio Processing Interface
 *
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "ar_defs.h"
#include "ar_guids.h"
#include "module_cmn_metadata.h"

#define CAPI_INVALID_VAL AR_NON_GUID(0xFFFFFFFF)

/** @addtogroup capi_error_codes
    @xreflabel{hdr:errorCodes}
@{ */

/** Error code type for CAPI. */
typedef uint32_t capi_err_t;

/** Success. The operation completed with no errors. */
#define CAPI_EOK 0

/** General failure. */
#define CAPI_EFAILED ((uint32_t)1)

/** Invalid parameter value set. */
#define CAPI_EBADPARAM (((uint32_t)1) << 1)

/** Unsupported routine or operation. */
#define CAPI_EUNSUPPORTED (((uint32_t)1) << 2)

/** Operation does not have memory. */
#define CAPI_ENOMEMORY (((uint32_t)1) << 3)

/** Operation needs more data or buffer space. */
#define CAPI_ENEEDMORE (((uint32_t)1) << 4)

/** CAPI currently cannot perform this operation because necessary properties
    and parameters are not set or because of any internal state. */
#define CAPI_ENOTREADY (((uint32_t)1) << 5)

/** CAPI currently cannot perform this operation. There might be restrictions
    on overwriting calibration after a certain operation. For example,
    recalibrating the hardware interface after it is started. */
#define CAPI_EALREADY (((uint32_t)1) << 6)

/** Macro that checks whether a CAPI error code has any error bits set. */
#define CAPI_FAILED(x) (CAPI_EOK != (x))

/** Macro that checks whether a CAPI error code represents a success case. @newpage */
#define CAPI_SUCCEEDED(x) (CAPI_EOK == (x))

/** Macro that sets an error flag in a CAPI error code. */
#define CAPI_SET_ERROR(error_flags, return_code) ((error_flags) |= (return_code))

/** Macro that checks whether a specific error flag is set in a CAPI error
    code. */
#define CAPI_IS_ERROR_CODE_SET(error_flags, error_code) (((error_flags) & (error_code)) != CAPI_EOK)

/** @} */ /* end_addtogroup capi_error_codes */

typedef struct capi_buf_t capi_buf_t;

/** @addtogroup capi_data_types
@{ */
/** Contains input buffers, output buffers, property payloads, event payloads,
    and parameters that are passed into the CAPI functions.
*/
struct capi_buf_t
{
   int8_t *data_ptr;
   /**< Data pointer to the raw data. The alignment depends on the format
        of the raw data. */

   uint32_t actual_data_len;
   /**< Length of the valid data (in bytes).

        For input buffers: @vtspstrbul
        - The caller fills this field with the number of bytes of valid data in
          the buffer.
        - The callee fills this field with the number of bytes of data it read.

        For output buffers: @vtspstrbul
        - The caller leaves this field uninitialized.
        - The callee fills it with the number of bytes of data it filled.
        @tablebulletend */

   uint32_t max_data_len;
   /**< Total allocated size of the buffer (in bytes).

        The caller always fills this value, and the callee does not modify it.
        @newpagetable */
};

/** Version information for the #capi_stream_data_t structure specified in
    stream_data_version (in #capi_stream_flags_t).
*/
typedef enum capi_stream_version_t {
   CAPI_STREAM_V1 = 0,
   /**< Indicates the initial version of the structure
        (00 -- #capi_stream_data_t). */

   CAPI_STREAM_V2
   /**< Indicates version 2 of the structure (01 -- #capi_stream_data_v2_t). */
} /** @cond */ capi_stream_version_t /** @endcond */;

typedef union capi_stream_flags_t capi_stream_flags_t;

/** Flags that are passed with every input buffer and must be filled by the
    module for every output buffer. These flags apply only to the buffer with
    which they are associated.

    marker_eos and end_of_frame flags are closely associated with metadata.
    Modules that implement #INTF_EXTN_METADATA must take care of setting/clearing/propagating
    marker_eos and end_of_frame. For other modules, the framework takes care of end_of_frame and
    marker_eos. If the framework's method of handling marker_eos/end_of_frame
    does not address a module's requirements, the module must implement the #INTF_EXTN_METADATA extension
    and handle these flags.
*/
union capi_stream_flags_t
{
   /** Defines the flags.
   */
   struct
   {
      uint32_t is_timestamp_valid : 1;
      /**< Specifies whether the timestamp is valid.

           For SISO modules the framework propagates timestamp
         and related flags (timestamp valid, continue) from input to output.

           @valuesbul
           - 0 -- Not valid
           - 1 -- Valid @tablebulletend */

      uint32_t end_of_frame : 1;
      /**< Specifies whether the buffer has an end of frame.

           @valuesbul
           - 0 -- end_of_frame is not marked
           - 1 -- end_of_frame is marked

           When end_of_frame is set, the modules must try to process given data even
           if the threshold is not met. Otherwise, data might be dropped.

           For raw-compressed-data, end_of_frame usually indicates that the buffer has
           integral number of encoded frames.

           end_of_frame is also set for discontinuities (timestamp discontinuity, EOS).

           If a module does not implement the #INTF_EXTN_METADATA extension,
           then it must not write to this flag.

           Framework callbacks in #INTF_EXTN_METADATA, don't take care of end_of_frame.
           Therefore, modules that support the #INTF_EXTN_METADATA extension must also take care of
           setting/clearing/propagating of this flag.

           */

      uint32_t marker_eos : 1;
      /**< Indicates that this data is the last valid data from the upstream
           port.

           There are two types of EOS, flushing and non-flushing.
           This flag pertains to flushing EOS.

           Flushing EOS extracts all the data out of the modules. @vtspstrbul
            - For decoders, this is achieved by repeatedly calling the module
              without input.
            - For generic modules, this is achieved by pushing zeroes worth
              algorithm delay (zeroes worth = zero samples equal to the amount
              of algorithmic delay).
            - Multi-port modules must take care of flushing internally.


           Non-flushing EOS is indicated only through metadata (marker_eos is not set).

           marker_eos is accompanied by EOS metadata.

           Typical Flushing EOS propagation works as follows:
           1. EOS is given at the input of a module.
                 marker_eos flag is set on the input.
                 EOS metadata is present in the input port metadata list.
           2. EOS undergoes algorithmic or buffering delay (if applicable).
                 marker_eos flag is set on the input.
                 EOS metadata moves internal to the module and the input port metadata list is cleared.
           3. EOS goes to output, gets destroyed, or gets converted to non-flushing EOS.
                 The marker_eos flag moves to output.
                 Input marker_eos is cleared and the internal metadata list is cleared.
                 In case EOS moves to output, marker_eos on output is set and the output port metadata list is
         populated.

           On the input port, marker_eos is set whenever a new EOS arrives or
           when the previous EOS is stuck inside the module due to delays.
           Output marker_eos is set only when there is EOS metadata in the output list.

           marker_eos handling is taken care of by the framework, for modules that don't implement INTF_EXTN_METADATA.
           For others, EOS metdata propagation including marker_eos handling, is taken care of by
           #intf_extn_param_id_metadata_handler_t::metadata_propagate() in #INTF_EXTN_METADATA.
           However, for create/destroy/clone etc., the module must set/clear marker_eos.

            */

      uint32_t marker_1 : 1;
      /**< Data marker 1 the service uses to track data.

           The module must propagate this marker from the input port to any
           output port that gets input from this port. */

      uint32_t marker_2 : 1;
      /**< Data marker 2 the service uses to track data.

           The module must propagate this marker from the input port to any
           output port that gets input from this port. */

      uint32_t marker_3 : 1;
      /**< Data marker 3 the service uses to track data.

           The module must propagate this marker from the input port to any
           output port that gets input from this port. */

      uint32_t erasure : 1;
      /**< Explicitly signals erasure due to underflow.

           @valuesbul
           - 0 -- No erasure
           - 1 -- Erasure

           This flag triggers erasure handling in decoders. Some
           implementations push this flag to the modules while signaling
           erasure. */

      uint32_t stream_data_version : 2;
      /**< Version of the #capi_stream_data_t structure.

           Versions are defined in #capi_stream_version_t: @vtspstrbul
           - 00 -- #capi_stream_data_t
           - 01 -- #capi_stream_data_v2_t
           - 10 -- Reserved
           - 11 -- Reserved @tablebulletend */

      uint32_t ts_continue : 1;
      /**<  If the timestamp continue flag is set, then the timestamp field must not be read.
            Timestamp values based on previously set timestamp must continue.
            If the previous timestamp value is invalid, then this flag doesn't apply.
            If ts_continue is reset and ts_valid is set, sync to the input timestamp.
            This field should be used only for raw compressed data formats.

           @valuesbul
           - 0 -- ts_continue is set as FALSE
           - 1 -- ts_continue is set as TRUE @tablebulletend */

      uint32_t reserved : 20;
      /**< Reserved for future use. The module must ignore this value for input
           ports. @newpagetable */
   };

   uint32_t word;
   /**< Entire 32-bit word for easy access to read or write the entire word in
        one shot. */
};

typedef struct capi_stream_data_t capi_stream_data_t;

/** Data structure for one stream.
 */
struct capi_stream_data_t
{
   capi_stream_flags_t flags;
   /**< Flags that indicate the stream properties. For more information on
        the flags, see #capi_stream_flags_t. */

   int64_t timestamp;
   /**< Timestamp of the first data sample, in microseconds.

        The time origin is not fixed; it must be inferred from the timestamp of
        the first buffer. Negative values are allowed. */

   capi_buf_t *buf_ptr;
   /**< Pointer to the array of CAPI buffer elements.

        For deinterleaved unpacked uncompressed data, one buffer is to be
        used per channel. For CAPI_DEINTERLEAVED_RAW_COMPRESSED,
        as many buffers are used as specified in the media format.
        For all other cases, only one buffer is to be
        used. */

   uint32_t bufs_num;
   /**< Number of buffer elements in the buf_ptr array.

        For deinterleaved unpacked uncompressed data, this is equal to the
        number of channels in the stream. For CAPI_V2_DEINTERLEAVED_RAW_COMPRESSED,
        as many buffers are used as specified in the media format. For all other cases, all the data
        is put in one buffer, so this field is set to 1. */
};

typedef struct capi_stream_data_v2_t capi_stream_data_v2_t;

/** Version 2 of the data structure for one stream.
 */
struct capi_stream_data_v2_t
{
   capi_stream_flags_t flags;
   /**< Flags that indicate the stream properties.

        For more information on the flags, see #capi_stream_flags_t. */

   int64_t timestamp;
   /**< Timestamp of the first data sample, in microseconds.

        The time origin is not fixed; it must be inferred from the timestamp of
        the first buffer. Negative values are allowed. */

   capi_buf_t *buf_ptr;
   /**< Pointer to the array of CAPI buffer elements.

        For deinterleaved unpacked uncompressed data, one buffer is to be
        used per channel. For CAPI_V2_DEINTERLEAVED_RAW_COMPRESSED,
        as many buffers are used as specified in the media format.
        For all other cases, only one buffer is to be
        used. */

   uint32_t bufs_num;
   /**< Number of buffer elements in the buf_ptr array.

        For deinterleaved unpacked uncompressed data, this is equal to the
        number of channels in the stream. For all other cases, all the data
        is put in one buffer, so this field is set to 1. */

   module_cmn_md_list_t *metadata_list_ptr;
   /**< Pointer to the list of metadata. The object pointer in this list is of
        type #module_cmn_md_t. @newpagetable */
};

/** Maximum number of channels supported in a stream. */
#define CAPI_MAX_CHANNELS 16

/** Version 2 of the maximum number of channels macro. */
#ifdef PROD_SPECIFIC_MAX_CH
#define CAPI_MAX_CHANNELS_V2 PROD_SPECIFIC_MAX_CH
#else
#define CAPI_MAX_CHANNELS_V2 32
#endif
/** Minor version of media format version 2. */
#define CAPI_MEDIA_FORMAT_MINOR_VERSION (1)

/** Valid values for data formats of the inputs and outputs of a module.
*/
typedef enum data_format_t {
   CAPI_FIXED_POINT = 0,
   /**< Fixed point PCM data. @vertspace{4}

        Payload: #capi_standard_data_format_t @vertspace{6} */

   CAPI_FLOATING_POINT = 1,
   /**< Floating point PCM data. @vertspace{4}

        Payload: #capi_standard_data_format_t @vertspace{6} */

   CAPI_RAW_COMPRESSED = 2,
   /**< Compressed audio data bitstream in an unpacketized form. @vertspace{4}

        Payload: #capi_raw_compressed_data_format_t @vertspace{6} */

   CAPI_IEC61937_PACKETIZED = 3,
   /**< Compressed audio data bitstream packetized according to \n
        @xhyperref{S1,IEC~61937}.

        Payload: #capi_standard_data_format_t @vertspace{6} */

   CAPI_DSD_DOP_PACKETIZED = 4,
   /**< Compressed audio data bitstream packetized in Qualcomm DSD-over-PCM
        (DSD_DOP) format (16 bit, MSB first, word interleaved). @vertspace{4}

        Payload: #capi_standard_data_format_t @vertspace{6} */

   CAPI_COMPR_OVER_PCM_PACKETIZED = 5,
   /**< Compressed bitstreams packetized like PCM using a Qualcomm-designed
        packetizer. @vertspace{4}

        Payload: #capi_standard_data_format_t @vertspace{6} */

   CAPI_GENERIC_COMPRESSED = 6,
   /**< Compressed audio data bitstream that will be passed through as is,
        without knowing or modifying the underlying structure. @vertspace{4}

        Payload: #capi_standard_data_format_t @vertspace{6} */

   CAPI_IEC60958_PACKETIZED = 7,
   /**< PCM audio data bitstream packetized according to IEC 60958 \n format.
        @vertspace{4}

        Payload: #capi_standard_data_format_t @vertspace{6} */

   CAPI_IEC60958_PACKETIZED_NON_LINEAR = 8,
   /**< Compressed audio data bitstream packetized according to IEC 60958
        format. @vertspace{4}

        Payload: #capi_standard_data_format_t @vertspace{6} */

   CAPI_DEINTERLEAVED_RAW_COMPRESSED = 9,
   /**< Compressed audio data bitstream in an unpacketized form
    *   where each channel has its own buffers. @vertspace{4}
    *
    *   Payload: #capi_deinterleaved_raw_compressed_data_format_t @vertspace{6} */

   CAPI_MAX_FORMAT_TYPE = 0x7FFFFFFF
   /**< Maximum value that a data format can take. */
} /** @cond */ data_format_t /** @endcond */;

/** Macro used in any field of #capi_data_format_header_t to indicate that the
    value is unspecified.
 */
#define CAPI_DATA_FORMAT_INVALID_VAL CAPI_INVALID_VAL

/** Valid values for data interleaving or deinterleaving.
 */
typedef enum capi_interleaving_t {
   CAPI_INTERLEAVED,
   /**< Data for all channels is present in one buffer. The samples are
        interleaved per channel. @vertspace{6} */

   CAPI_DEINTERLEAVED_PACKED,
   /**< Data for all channels is present in one buffer. @vertspace{4}

        All of the samples of one channel are present, followed by all of the
        samples of the next channel, and so on. @vertspace{6} */

   CAPI_DEINTERLEAVED_UNPACKED,
   /**< Data for each channel is present in a different buffer. @vertspace{4}

        In this case, the #capi_stream_data_v2_t::buf_ptr field points to an
        array of buffer structures with the number of elements equal to the
        number of channels. The bufs_num field must be set to the number of
        channels. Actual data length and max data lengths is expected to be
        same for all the channel buffers. @vertspace{6} */

     CAPI_DEINTERLEAVED_UNPACKED_V2,
     /**< Data for each channel is present in a different buffer. @vertspace{4}

        In this case, the #capi_stream_data_v2_t::buf_ptr field points to an
        array of buffer structures with the number of elements equal to the
        number of channels. Actual data length and max data length of every
        channel is expected to be same, and lengths are set only for the first 
        channel. The bufs_num field must be set to the number of channels. 
        @vertspace{6} */
   CAPI_INVALID_INTERLEAVING = CAPI_DATA_FORMAT_INVALID_VAL
   /**< Interleaving is not valid. */
} /** @cond */ capi_interleaving_t /** @endcond */;

/** @} */ /* end_addtogroup capi_data_types */

typedef struct capi_data_format_header_t capi_data_format_header_t;

/** @addtogroup capi_payload_structs
@{ */
/** Header structure for a data format that is passed into the module.
    Following this header is the appropriate media format payload.
 */
struct capi_data_format_header_t
{
   data_format_t data_format;
   /**< Indicates the format in which the data is represented.
        The rest of the payload depends on the data format. */
};

typedef struct capi_set_get_media_format_t capi_set_get_media_format_t;

/** Header structure used to set and get a media format.
    Following this header is the appropriate media format payload.
 */
struct capi_set_get_media_format_t
{
   capi_data_format_header_t format_header;
   /**< Header of the media format. */
};

typedef struct capi_standard_data_format_t capi_standard_data_format_t;

/** Payload structure for the #CAPI_FIXED_POINT, #CAPI_FLOATING_POINT, and
    #CAPI_IEC61937_PACKETIZED data formats.
 */
struct capi_standard_data_format_t
{
   uint32_t bitstream_format;
   /**< Valid types are MEDIA_FMT_ID_* as defined in media_fmt_api.h. */

   uint32_t num_channels;
   /**< Number of channels. */

   uint32_t bits_per_sample;
   /**< Number of bits used to store each sample.

        This value should be interpreted as the sample word size in bits.
        For example, if the data is 24-bit audio packed in 24 bits, this value
        is 24. If the data is 24-bit audio packed in 32 bits, this value is
        32. */

   uint32_t q_factor;
   /**< Number of fractional bits in the fixed point representation of the
        data.

        If the data is floating point, this field must be set to
        #CAPI_DATA_FORMAT_INVALID_VAL. */

   uint32_t sampling_rate;
   /**< Sampling rate in samples per second. */

   uint32_t data_is_signed;
   /**< Specifies whether data is signed.

        @valuesbul
        - 1 -- Signed
        - 0 -- Unsigned @tablebulletend */

   capi_interleaving_t data_interleaving;
   /**< Indicates whether the data is interleaved. This value is not relevant
        for packetized data.

        @valuesbul
        - #CAPI_INTERLEAVED
        - #CAPI_DEINTERLEAVED_PACKED
        - #CAPI_DEINTERLEAVED_UNPACKED @tablebulletend */

   uint16_t channel_type[CAPI_MAX_CHANNELS];
   /**< Array of channel types for each num_channels.

        @values PCM_CHANNEL_* types as defined in media_fmt_api.h. */
};

/** Valid channel types as defined in media_fmt_api.h. */
typedef uint16_t capi_channel_type_t;

typedef struct capi_standard_data_format_v2_t capi_standard_data_format_v2_t;

#include "spf_begin_pragma.h"

/** Media format version 2 payload for the #CAPI_FIXED_POINT,
    #CAPI_FLOATING_POINT, and #CAPI_IEC61937_PACKETIZED data formats.
 */
struct capi_standard_data_format_v2_t
{
   uint32_t minor_version;
   /**< Minor version for this payload. */

   uint32_t bitstream_format;
   /**< Valid types are MEDIA_FMT_ID_* as defined in  media_fmt_api.h. */

   uint32_t num_channels;
   /**< Number of channels. */

   uint32_t bits_per_sample;
   /**< Number of bits used to store each sample.

        This value should be interpreted as the sample word size in bits. For
        example, if the data is 24-bit audio packed in 24 bits, this value is
        24. If the data is 24-bit audio packed in 32 bits, this value is 32. */

   uint32_t q_factor;
   /**< Number of fractional bits in the fixed point representation of the
        data.

        If the data is floating point, this field must be set to
        #CAPI_DATA_FORMAT_INVALID_VAL. */

   uint32_t sampling_rate;
   /**< Sampling rate in samples per second. */

   uint32_t data_is_signed;
   /**< Specifies whether data is signed.

        @valuesbul
        - 1 -- Signed
        - 0 -- Unsigned @tablebulletend */

   capi_interleaving_t data_interleaving;
   /**< Indicates whether the data is interleaved. This value is not relevant
        for packetized data.

        @valuesbul
        - #CAPI_INTERLEAVED
        - #CAPI_DEINTERLEAVED_PACKED
        - #CAPI_DEINTERLEAVED_UNPACKED @tablebulletend */

   capi_channel_type_t channel_type[0];
   /**< Channel type payload is of variable length and depends on the number
        of channels. This payload has channel types for each of the
        num_channels.

        @values PCM_CHANNEL_* types as defined in media_fmt_api.h @newpagetable */
}
#include "spf_end_pragma.h"
;

typedef struct capi_raw_compressed_data_format_t capi_raw_compressed_data_format_t;

/** Payload header for the RAW_COMPRESSED data format.

  Following this structure is the media format structure for the specific data
  format as defined in media_fmt_api.h or specific decoder API file.
 */
struct capi_raw_compressed_data_format_t
{
   uint32_t bitstream_format;
   /**< Valid types are MEDIA_FMT_ID_* as defined in media_fmt_api.h. */
};

/**
 * Channel mask.
 */
typedef struct capi_channel_mask_t capi_channel_mask_t;
struct capi_channel_mask_t
{
   uint32_t channel_mask_lsw;
   /**< LSW of the channel mask. */
   uint32_t channel_mask_msw;
   /**< MSW of the channel mask. */
};

typedef struct capi_deinterleaved_raw_compressed_data_format_t capi_deinterleaved_raw_compressed_data_format_t;

/** Payload header for the DEINTERLEAVED_RAW_COMPRESSED data format.
 *  Unlike raw compressed, there is no media format specific payload following
 *  this struct, only channel mask follows.
 */
struct capi_deinterleaved_raw_compressed_data_format_t
{
   uint32_t minor_version;
   /**< Minor version for this payload. Only version 1 supported currently. */

   uint32_t bitstream_format;
   /**< Valid types are MEDIA_FMT_ID_* as defined in media_fmt_api.h. */

   uint32_t bufs_num;
/**< Number of buffers. */

#if __H2XML__
   capi_channel_mask_t ch_mask[0];
/**< 64 bit channel mask array. Array length is same as bufs_num.
     Indicates the channels present in each buffer.
     The channel mask is derived by bit-shifting the channel map. Channel mask = 1 << channel type.
     Channel type values are defined in:

     @values PCM_CHANNEL_* types as defined in AudioReach Media format APIs.

   Example usage: one buffer has 2 channels, left & right:
      bufs_num = 1, channel_mask[1] = {(1<<PCM_CHANNEL_L)|(1<<PCM_CHANNEL_R)}
      here, the order of left and right inside the buffer is not defined by CAPI.

      left and right are in separate buffers.
      bufs_num = 2, channel_mask[2] = {(1<<PCM_CHANNEL_L), (1<<PCM_CHANNEL_R)}
      here, first buffer has left and second buffer has right.

      Mono:
      bufs_num = 1, channel_mask[1] = {(1<<PCM_CHANNEL_C)}
     */
#endif
};

/** @} */ /* end_addtogroup capi_payload_structs */

typedef struct capi_port_info_t capi_port_info_t;

/** @ingroup capi_data_types
    Payload structure header with data port information.

    Control ports do not use this structure. Control ports are handled through
    interface extensions.
 */
struct capi_port_info_t
{
   bool_t is_valid;
   /**< Indicates whether port_index is valid.

        @valuesbul
        - 0 -- Not valid
        - 1 -- Valid @tablebulletend */

   bool_t is_input_port;
   /**< Indicates the type of port.

        @valuesbul
        - TRUE -- Input port
        - FALSE -- Output port @tablebulletend */

   uint32_t port_index;
   /**< Identifies the port.

        Index values must be sequential numbers starting from zero. There are
        separate sequences for input and output ports. For example, if a
        module has three input ports and two output ports: @vtspstrbul
        - The input ports have index values of 0, 1, and 2.
        - The output ports have index values of 0 and 1. @vertspace{3}

        When #capi_vtbl_t::process() is called: @vtspstrbul
        - Data in input[0] is for input port 0.
        - Data in input[1] is for input port 1.
        - And so on.
        - Output port 0 must fill data into output[0].
        - Output port 1 must fill data into output[1].
        - And so on. @tablebulletend @newpagetable */
};

#endif /* #ifndef CAPI_TYPES_H */
