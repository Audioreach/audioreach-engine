#ifndef MEDIA_FMT_API_BASIC_H
#define MEDIA_FMT_API_BASIC_H
/**
 * \file media_fmt_api_basic.h
 * \brief
 *       This file contains media format IDs and definitions
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "ar_defs.h"
#include "media_fmt_api_ext.h"

#ifdef __cplusplus
extern "C"
{
#endif /*__cplusplus*/

/*# @h2xml_title1           {Media Format APIs}
    @h2xml_title_agile_rev  {Media Format APIs}
    @h2xml_title_date       {August 13, 2018} */

/*# @h2xmlx_xmlNumberFormat {int} */


/** @ingroup ar_spf_mod_media_fmt_ids
    Enumeration for infinity. */
#undef INFINITE
#define INFINITE                         -1

/** @ingroup ar_spf_mod_media_fmt_ids
    Enumeration for an invalid value. */
#define INVALID_VALUE                     0


/*********************************************** Channel Map Values ***************************************************/
/** @ingroup ar_spf_mod_media_fmt_ids
    Channel map values.
*/
enum pcm_channel_map
{
   PCM_CHANNEL_L = 1,                  /**< Front left channel. */
   PCM_CHANNEL_R = 2,                  /**< Front right channel. */
   PCM_CHANNEL_C = 3,                  /**< Front center channel. */
   PCM_CHANNEL_LS = 4,                 /**< Left surround channel. */
   PCM_CHANNEL_RS = 5,                 /**< Right surround channel. */
   PCM_CHANNEL_LFE = 6,                /**< Low frequency effect channel. */
   PCM_CHANNEL_CS = 7,                 /**< Center surround channel; rear
                                            center channel. */
   PCM_CHANNEL_CB = PCM_CHANNEL_CS,    /**< Center back channel. */
   PCM_CHANNEL_LB = 8,                 /**< Left back channel; rear left
                                           channel. */
   PCM_CHANNEL_RB = 9,                 /**< Right back channel; rear right
                                           channel. */
   PCM_CHANNEL_TS = 10,                /**< Top surround channel. */
   PCM_CHANNEL_CVH = 11,               /**< Center vertical height channel. */
   PCM_CHANNEL_TFC = PCM_CHANNEL_CVH,  /**< Top front center channel. */
   PCM_CHANNEL_MS = 12,                /**< Mono surround channel. */
   PCM_CHANNEL_FLC = 13,               /**< Front left of center channel. */
   PCM_CHANNEL_FRC = 14,               /**< Front right of center channel. */
   PCM_CHANNEL_RLC = 15,               /**< Rear left of center channel. */
   PCM_CHANNEL_RRC = 16,               /**< Rear right of center channel. */
   PCM_CHANNEL_LFE2 = 17,              /**< Secondary low frequency effect
                                            channel. */
   PCM_CHANNEL_SL = 18,                /**< Side left channel. */
   PCM_CHANNEL_SR = 19,                /**< Side right channel. */
   PCM_CHANNEL_TFL = 20,               /**< Top front left channel. */
   PCM_CHANNEL_LVH = PCM_CHANNEL_TFL,  /**< Left vertical height channel. */
   PCM_CHANNEL_TFR = 21,               /**< Top front right channel. */
   PCM_CHANNEL_RVH = PCM_CHANNEL_TFR,  /**< Right vertical height channel. */
   PCM_CHANNEL_TC = 22,                /**< Top center channel. */
   PCM_CHANNEL_TBL = 23,               /**< Top back left channel. */
   PCM_CHANNEL_TBR = 24,               /**< Top back right channel. */
   PCM_CHANNEL_TSL = 25,               /**< Top side left channel. */
   PCM_CHANNEL_TSR = 26,               /**< Top side right channel. */
   PCM_CHANNEL_TBC = 27,               /**< Top back center channel. */
   PCM_CHANNEL_BFC = 28,               /**< Bottom front center channel. */
   PCM_CHANNEL_BFL = 29,               /**< Bottom front left channel. */
   PCM_CHANNEL_BFR = 30,               /**< Bottom front right channel. */
   PCM_CHANNEL_LW = 31,                /**< Left wide channel. */
   PCM_CHANNEL_RW = 32,                /**< Right wide channel. */
   PCM_CHANNEL_LSD = 33,               /**< Left side direct channel. */
   PCM_CHANNEL_RSD = 34,               /**< Right side direct channel. */

  PCM_CUSTOM_CHANNEL_MAP_17 = 35, /**< Custom channel map 35. */
  PCM_CUSTOM_CHANNEL_MAP_18 = 36, /**< Custom channel map 36. */
  PCM_CUSTOM_CHANNEL_MAP_19 = 37, /**< Custom channel map 37. */
  PCM_CUSTOM_CHANNEL_MAP_20 = 38, /**< Custom channel map 38. */
  PCM_CUSTOM_CHANNEL_MAP_21 = 39, /**< Custom channel map 39. */
  PCM_CUSTOM_CHANNEL_MAP_22 = 40, /**< Custom channel map 40. */
  PCM_CUSTOM_CHANNEL_MAP_23 = 41, /**< Custom channel map 41. */
  PCM_CUSTOM_CHANNEL_MAP_24 = 42, /**< Custom channel map 42. */
  PCM_CUSTOM_CHANNEL_MAP_25 = 43, /**< Custom channel map 43. */
  PCM_CUSTOM_CHANNEL_MAP_26 = 44, /**< Custom channel map 44. */
  PCM_CUSTOM_CHANNEL_MAP_27 = 45, /**< Custom channel map 45. */
  PCM_CUSTOM_CHANNEL_MAP_28 = 46, /**< Custom channel map 46. */
  PCM_CUSTOM_CHANNEL_MAP_29 = 47, /**< Custom channel map 47. */

  PCM_CUSTOM_CHANNEL_MAP_1 = 48,  /**< Custom channel map 48. */
  PCM_CUSTOM_CHANNEL_MAP_2 = 49,  /**< Custom channel map 49. */
  PCM_CUSTOM_CHANNEL_MAP_3 = 50,  /**< Custom channel map 50. */
  PCM_CUSTOM_CHANNEL_MAP_4 = 51,  /**< Custom channel map 51. */
  PCM_CUSTOM_CHANNEL_MAP_5 = 52,  /**< Custom channel map 52. */
  PCM_CUSTOM_CHANNEL_MAP_6 = 53,  /**< Custom channel map 53. */
  PCM_CUSTOM_CHANNEL_MAP_7 = 54,  /**< Custom channel map 54. */
  PCM_CUSTOM_CHANNEL_MAP_8 = 55,  /**< Custom channel map 55. */
  PCM_CUSTOM_CHANNEL_MAP_9 = 56,  /**< Custom channel map 56. */
  PCM_CUSTOM_CHANNEL_MAP_10 = 57, /**< Custom channel map 57. */
  PCM_CUSTOM_CHANNEL_MAP_11 = 58, /**< Custom channel map 58. */
  PCM_CUSTOM_CHANNEL_MAP_12 = 59, /**< Custom channel map 59. */
  PCM_CUSTOM_CHANNEL_MAP_13 = 60, /**< Custom channel map 60. */
  PCM_CUSTOM_CHANNEL_MAP_14 = 61, /**< Custom channel map 61. */
  PCM_CUSTOM_CHANNEL_MAP_15 = 62, /**< Custom channel map 62. */
  PCM_CUSTOM_CHANNEL_MAP_16 = 63, /**< Custom channel map 63. */

  PCM_CUSTOM_CHANNEL_MAP_30 = 64,  /**< Custom channel map 64. */
  PCM_CUSTOM_CHANNEL_MAP_31 = 65,  /**< Custom channel map 65. */
  PCM_CUSTOM_CHANNEL_MAP_32 = 66,  /**< Custom channel map 66. */
  PCM_CUSTOM_CHANNEL_MAP_33 = 67,  /**< Custom channel map 67. */
  PCM_CUSTOM_CHANNEL_MAP_34 = 68,  /**< Custom channel map 68. */
  PCM_CUSTOM_CHANNEL_MAP_35 = 69,  /**< Custom channel map 69. */

  PCM_CUSTOM_CHANNEL_MAP_36 = 70,  /**< Custom channel map 70. */
  PCM_CUSTOM_CHANNEL_MAP_37 = 71,  /**< Custom channel map 71. */
  PCM_CUSTOM_CHANNEL_MAP_38 = 72,  /**< Custom channel map 72. */
  PCM_CUSTOM_CHANNEL_MAP_39 = 73,  /**< Custom channel map 73. */
  PCM_CUSTOM_CHANNEL_MAP_40 = 74,  /**< Custom channel map 74. */
  PCM_CUSTOM_CHANNEL_MAP_41 = 75,  /**< Custom channel map 75. */
  PCM_CUSTOM_CHANNEL_MAP_42 = 76,  /**< Custom channel map 76. */
  PCM_CUSTOM_CHANNEL_MAP_43 = 77,  /**< Custom channel map 77. */
  PCM_CUSTOM_CHANNEL_MAP_44 = 78,  /**< Custom channel map 78. */
  PCM_CUSTOM_CHANNEL_MAP_45 = 79,  /**< Custom channel map 79. */

  PCM_CUSTOM_CHANNEL_MAP_46 = 80,  /**< Custom channel map 80. */
  PCM_CUSTOM_CHANNEL_MAP_47 = 81,  /**< Custom channel map 81. */
  PCM_CUSTOM_CHANNEL_MAP_48 = 82,  /**< Custom channel map 82. */
  PCM_CUSTOM_CHANNEL_MAP_49 = 83,  /**< Custom channel map 83. */
  PCM_CUSTOM_CHANNEL_MAP_50 = 84,  /**< Custom channel map 84. */
  PCM_CUSTOM_CHANNEL_MAP_51 = 85,  /**< Custom channel map 85. */
  PCM_CUSTOM_CHANNEL_MAP_52 = 86,  /**< Custom channel map 86. */
  PCM_CUSTOM_CHANNEL_MAP_53 = 87,  /**< Custom channel map 87. */
  PCM_CUSTOM_CHANNEL_MAP_54 = 88,  /**< Custom channel map 88. */
  PCM_CUSTOM_CHANNEL_MAP_55 = 89,  /**< Custom channel map 89. */

  PCM_CUSTOM_CHANNEL_MAP_56 = 90,  /**< Custom channel map 90. */
  PCM_CUSTOM_CHANNEL_MAP_57 = 91,  /**< Custom channel map 91. */
  PCM_CUSTOM_CHANNEL_MAP_58 = 92,  /**< Custom channel map 92. */
  PCM_CUSTOM_CHANNEL_MAP_59 = 93,  /**< Custom channel map 93. */
  PCM_CUSTOM_CHANNEL_MAP_60 = 94,  /**< Custom channel map 94. */
  PCM_CUSTOM_CHANNEL_MAP_61 = 95,  /**< Custom channel map 95. */
  PCM_CUSTOM_CHANNEL_MAP_62 = 96,  /**< Custom channel map 96. */
  PCM_CUSTOM_CHANNEL_MAP_63 = 97,  /**< Custom channel map 97. */
  PCM_CUSTOM_CHANNEL_MAP_64 = 98,  /**< Custom channel map 98. */
  PCM_CUSTOM_CHANNEL_MAP_65 = 99,  /**< Custom channel map 99. */

  PCM_CUSTOM_CHANNEL_MAP_66 = 100,  /**< Custom channel map 100. */
  PCM_CUSTOM_CHANNEL_MAP_67 = 101,  /**< Custom channel map 101. */
  PCM_CUSTOM_CHANNEL_MAP_68 = 102,  /**< Custom channel map 102. */
  PCM_CUSTOM_CHANNEL_MAP_69 = 103,  /**< Custom channel map 103. */
  PCM_CUSTOM_CHANNEL_MAP_70 = 104,  /**< Custom channel map 104. */
  PCM_CUSTOM_CHANNEL_MAP_71 = 105,  /**< Custom channel map 105. */
  PCM_CUSTOM_CHANNEL_MAP_72 = 106,  /**< Custom channel map 106. */
  PCM_CUSTOM_CHANNEL_MAP_73 = 107,  /**< Custom channel map 107. */
  PCM_CUSTOM_CHANNEL_MAP_74 = 108,  /**< Custom channel map 108. */
  PCM_CUSTOM_CHANNEL_MAP_75 = 109,  /**< Custom channel map 109. */

  PCM_CUSTOM_CHANNEL_MAP_76 = 110,  /**< Custom channel map 110. */
  PCM_CUSTOM_CHANNEL_MAP_77 = 111,  /**< Custom channel map 111. */
  PCM_CUSTOM_CHANNEL_MAP_78 = 112,  /**< Custom channel map 112. */
  PCM_CUSTOM_CHANNEL_MAP_79 = 113,  /**< Custom channel map 113. */
  PCM_CUSTOM_CHANNEL_MAP_80 = 114,  /**< Custom channel map 114. */
  PCM_CUSTOM_CHANNEL_MAP_81 = 115,  /**< Custom channel map 115. */
  PCM_CUSTOM_CHANNEL_MAP_82 = 116,  /**< Custom channel map 116. */
  PCM_CUSTOM_CHANNEL_MAP_83 = 117,  /**< Custom channel map 117. */
  PCM_CUSTOM_CHANNEL_MAP_84 = 118,  /**< Custom channel map 118. */
  PCM_CUSTOM_CHANNEL_MAP_85 = 119,  /**< Custom channel map 119. */

  PCM_CUSTOM_CHANNEL_MAP_86 = 120,  /**< Custom channel map 120. */
  PCM_CUSTOM_CHANNEL_MAP_87 = 121,  /**< Custom channel map 121. */
  PCM_CUSTOM_CHANNEL_MAP_88 = 122,  /**< Custom channel map 122. */
  PCM_CUSTOM_CHANNEL_MAP_89 = 123,  /**< Custom channel map 123. */
  PCM_CUSTOM_CHANNEL_MAP_90 = 124,  /**< Custom channel map 124. */
  PCM_CUSTOM_CHANNEL_MAP_91 = 125,  /**< Custom channel map 125. */
  PCM_CUSTOM_CHANNEL_MAP_92 = 126,  /**< Custom channel map 126. */
  PCM_CUSTOM_CHANNEL_MAP_93 = 127,  /**< Custom channel map 127. */
  PCM_CUSTOM_CHANNEL_MAP_94 = 128,  /**< Custom channel map 127. */
  PCM_MAX_CHANNEL_MAP = 63, /**< (Deprecated)Last item in the list.*/
  PCM_MAX_CHANNEL_MAP_V2 = 128 /**< Last item in the list.*/

};


/*********************************************** Bits Per Sample Values *********************************************** */

/** @ingroup ar_spf_mod_media_fmt_ids
    Sixteen bits per sample (= sample word size). */
#define BITS_PER_SAMPLE_16        16

/** @ingroup ar_spf_mod_media_fmt_ids
     Twenty-four bits per sample (= sample word size). */
#define BITS_PER_SAMPLE_24        24

/** @ingroup ar_spf_mod_media_fmt_ids
    Thirty-two bits per sample (= sample word size). */
#define BITS_PER_SAMPLE_32        32

/** @ingroup ar_spf_mod_media_fmt_ids
    Sixty-four bits per sample (= sample word size). */
#define BITS_PER_SAMPLE_64        64

/********************************************** Bytes Per Sample Values ************************************************/

/** @ingroup ar_spf_mod_media_fmt_ids
    Two bytes per sample. */
#define BYTES_PER_SAMPLE_TWO       2

/** @ingroup ar_spf_mod_media_fmt_ids
    Three bytes per sample. */
#define BYTES_PER_SAMPLE_THREE     3

/** @ingroup ar_spf_mod_media_fmt_ids
    Four bytes per sample. */
#define BYTES_PER_SAMPLE_FOUR      4


/********************************************** Bit Width Values ****************************************************** */

/** @ingroup ar_spf_mod_media_fmt_ids
    Sample has a bit width of 16 (the actual width of the sample in a word). */
#define BIT_WIDTH_16              16

/** @ingroup ar_spf_mod_media_fmt_ids
    Sample has a bit width of 24 (the actual width of the sample in a word). */
#define BIT_WIDTH_24              24

/** @ingroup ar_spf_mod_media_fmt_ids
    Sample has a bit width of 32 (the actual width of the sample in a word). */
#define BIT_WIDTH_32              32

/** @ingroup ar_spf_mod_media_fmt_ids
    Sample has a bit width of 64 (the actual width of the sample in a word). */
#define BIT_WIDTH_64	          64

/********************************************** Alignment Values ******************************************************/

/** @ingroup ar_spf_mod_media_fmt_ids
    Enumeration for LSB alignment. */
#define PCM_LSB_ALIGNED            1

/** @ingroup ar_spf_mod_media_fmt_ids
    Enumeration for MSB alignment. */
#define PCM_MSB_ALIGNED            2


/********************************************** Q Factor Values *******************************************************/

/** @ingroup ar_spf_mod_media_fmt_ids
    Enumeration for the Q15 factor. */
#define PCM_Q_FACTOR_15            15

/** @ingroup ar_spf_mod_media_fmt_ids
    Enumeration for the Q23 factor. */
#define PCM_Q_FACTOR_23            23

/** @ingroup ar_spf_mod_media_fmt_ids
    Enumeration for the Q27 factor. */
#define PCM_Q_FACTOR_27            27

/** @ingroup ar_spf_mod_media_fmt_ids
    Enumeration for the Q31 factor. */
#define PCM_Q_FACTOR_31            31

/** @ingroup ar_spf_mod_media_fmt_ids
    Shift factor for Q31 <=> Q28 conversion for 32-bit P. */
#define PCM_QFORMAT_SHIFT_FACTOR         (PCM_Q_FACTOR_31 - PCM_Q_FACTOR_27)


/********************************************** Sampling Rates ********************************************************/

/** @ingroup ar_spf_mod_media_fmt_ids
    Sampling rate is 8 kHz. */
#define SAMPLE_RATE_8K             8000

/** @ingroup ar_spf_mod_media_fmt_ids
    Sampling rate is 11.025 kHz. */
#define SAMPLE_RATE_11_025K        11025

/** @ingroup ar_spf_mod_media_fmt_ids
    Sampling rate is 12 kHz. */
#define SAMPLE_RATE_12K            12000

/** @ingroup ar_spf_mod_media_fmt_ids
    Sampling rate is 16 kHz. */
#define SAMPLE_RATE_16K            16000

/** @ingroup ar_spf_mod_media_fmt_ids
    Sampling rate is 22.05 kHz. */
#define SAMPLE_RATE_22_05K         22050

/** @ingroup ar_spf_mod_media_fmt_ids
    Sampling rate is 24 kHz. */
#define SAMPLE_RATE_24K            24000

/** @ingroup ar_spf_mod_media_fmt_ids
    Sampling rate is 32 kHz. */
#define SAMPLE_RATE_32K            32000

/** @ingroup ar_spf_mod_media_fmt_ids
    Sampling rate is 44.1 kHz. */
#define SAMPLE_RATE_44_1K          44100

/** @ingroup ar_spf_mod_media_fmt_ids
    Sampling rate is 48 kHz. */
#define SAMPLE_RATE_48K            48000

/** @ingroup ar_spf_mod_media_fmt_ids
    Sampling rate is 88.2 kHz. */
#define SAMPLE_RATE_88_2K          88200

/** @ingroup ar_spf_mod_media_fmt_ids
    Sampling rate is 96 kHz. */
#define SAMPLE_RATE_96K            96000

/** @ingroup ar_spf_mod_media_fmt_ids
    Sampling rate is 176.4 kHz. */
#define SAMPLE_RATE_176_4K         176400

/** @ingroup ar_spf_mod_media_fmt_ids
    Sampling rate is 192 kHz. */
#define SAMPLE_RATE_192K           192000

/** @ingroup ar_spf_mod_media_fmt_ids
    Sampling rate is 352.8 kHz. */
#define SAMPLE_RATE_352_8K         352800

/** @ingroup ar_spf_mod_media_fmt_ids
    Sampling rate is 384 kHz. */
#define SAMPLE_RATE_384K           384000


/********************************************** Endianess Values*******************************************************/

/** @ingroup ar_spf_mod_media_fmt_ids
    Byte order is little endian. */
#define PCM_LITTLE_ENDIAN          1

/** @ingroup ar_spf_mod_media_fmt_ids
    Byte order is big endian. */
#define PCM_BIG_ENDIAN             2


/********************************************** Interleaving Values*****************************************************/

/** @ingroup ar_spf_mod_media_fmt_ids
    PCM data is interleaved. */
#define PCM_INTERLEAVED                   1

/** @ingroup ar_spf_mod_media_fmt_ids
    PCM data is de-interleaved packed.

    A buffer of maximum size M with C channels and N/C actual bytes per channel
    is de-interleaved packed if (M - N) is zero.
 */
#define PCM_DEINTERLEAVED_PACKED          2

/** @ingroup ar_spf_mod_media_fmt_ids
    PCM data is de-interleaved unpacked.

    A buffer of maximum size M with C channels and N/C actual bytes per channel
    is de-interleaved unpacked if (M - N) is nonzero.

    Or, each channel has its own buffers where the actual length is less than
    the maximum length.
 */
#define PCM_DEINTERLEAVED_UNPACKED        3


/********************************************** Data Formats **********************************************************/

/** @ingroup ar_spf_mod_media_fmt_ids
    Enumeration for the fixed-point data format. */
#define DATA_FORMAT_FIXED_POINT           1

/** @ingroup ar_spf_mod_media_fmt_ids
    Data format is an IEC 61937 packetized stream.

    Like PCM, the data has properties such as sample rate, number of channels,
    and bit width. */
#define DATA_FORMAT_IEC61937_PACKETIZED   2

/** @ingroup ar_spf_mod_media_fmt_ids
    Data format is IEC 60958 packetized stream for PCM only.

    Like PCM, the data has properties such as sample rate, number of channels,
    and bit width. */
#define DATA_FORMAT_IEC60958_PACKETIZED   3

/** @ingroup ar_spf_mod_media_fmt_ids
    Data format is a DSD-over-PCM stream.

    Like PCM, the data has properties such as sample rate, number of channels,
    and bit width. */
#define DATA_FORMAT_DSD_OVER_PCM          4

/** @ingroup ar_spf_mod_media_fmt_ids
    Data format is a generic compressed stream.

    Like PCM, the data has properties such as sample rate, number of channels,
    and bit width. */
#define DATA_FORMAT_GENERIC_COMPRESSED    5

/** @ingroup ar_spf_mod_media_fmt_ids
    Data format is a Raw compressed stream.

    Like PCM, the data does not have properties such as sample rate, number of
    channels, and bit width. */
#define DATA_FORMAT_RAW_COMPRESSED        6

/** @ingroup ar_spf_mod_media_fmt_ids
    Compressed bitstreams are packetized like PCM using a QTI-designed
    packetizer. */
#define DATA_FORMAT_COMPR_OVER_PCM_PACKETIZED     7

/** @ingroup ar_spf_mod_media_fmt_ids
    Data format is an IEC 60958 packetized stream for compressed streams.

    Like PCM, the data has properties such as sample rate, number of channels,
    and bit width. */
#define DATA_FORMAT_IEC60958_PACKETIZED_NON_LINEAR 8

/** @ingroup ar_spf_mod_media_fmt_ids
 * * Data format is of Floating Point type */
#define DATA_FORMAT_FLOATING_POINT      9

/** @ingroup ar_spf_mod_media_fmt_ids
 * Data format is deinterleaved raw compressed unlike other formats it has no channels or buffers */
#define DATA_FORMAT_DEINTERLEAVED_RAW_COMPRESSED 10

/********************************************** Configuration Modes****************************************************/

/** @ingroup ar_spf_mod_media_fmt_ids
    Value of a configured parameter (like bits_per_sample and num_channels) is
    <em>don't care</em> . The module should continue to use the previously set
    configuration (or module defaults).

    For example, if the client is to control num_channels from the host but
    control bits_per_sample from the ACDB, the ACDB will have the value of -2.
    After a graph is opened, the host can issue a command with bits_per_sample
    = -2 and num_channels with a proper value.
*/
#define PARAM_VAL_UNSET     (-2)

/** @ingroup ar_spf_mod_media_fmt_ids
    Value of a configured parameter (like bits_per_sample and num_channels)
    must follow the input media format that the module will receive or has
    received. */
#define PARAM_VAL_NATIVE    (-1)

/** @ingroup ar_spf_mod_media_fmt_ids
    Value of a configured parameter (like bits_per_sample and num_channels) is
    considered invalid and the Set parameter command should result in an error.

    This mode is used to detect explicit initialization vs. initialization by
    a zero-memory setting. @newpage */
#define PARAM_VAL_INVALID     0


#ifndef DOXYGEN_SHOULD_SKIP_THIS // for IEC61937 and IEC60958

/************************* IEC61937_PACKETIZED and IEC60958_PACKETIZED data format ************************************/

/* Payload of data_format = DATA_FORMAT_IEC61937_PACKETIZED or DATA_FORMAT_IEC60958_PACKETIZED
    Contains media format information independent of payload for packetized media_fmt_id
 */
#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
struct payload_data_fmt_iec_packetized_t
{
   uint32_t sample_rate;
   /*#< @h2xmle_description {Sampling rate of audio stream.}
        @h2xmle_default     {0}
        @h2xmle_rangeList   {"44.1 kHz"=44100;
                             "48 kHz"=48000;
                             "88.2 kHz"=88200;
                             "96 kHz"=96000;
                             "176.4 kHz"=176400;
                             "192 kHz"=192000}
        @h2xmle_policy      {Basic} */

   uint16_t num_channels;
   /*#< @h2xmle_description {Number of channels.}
        @h2xmle_default     {0}
        @h2xmle_rangeList   {"TWO"=2;
                             "EIGHT"=8}
        @h2xmle_policy      {Basic} */

   uint16_t reserved;
   /*#< @h2xmle_description {Used for alignment; must be set to 0.}
        @h2xmle_policy      {Basic} */

}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;
typedef struct payload_data_fmt_iec_packetized_t payload_data_fmt_iec_packetized_t;

#endif /* DOXYGEN_SHOULD_SKIP_THIS */


/************************* GENERIC_COMPRESSED data format ************************************/

/** @ingroup ar_spf_mod_media_fmt_ids
    Payload structure used when media_format_t::data_format =
    #DATA_FORMAT_GENERIC_COMPRESSED.

    This structure contains media format information that is independent of the
    payload for the generic compressed media_fmt_id.
 */
#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
struct payload_data_fmt_generic_compressed_t
{
   uint32_t sample_rate;
   /**< Number of samples per second in the audio stream.

        @values 0 to 384 kHz */

   /*#< @h2xmle_description {Number of samples per second in the audio stream.}
        @h2xmle_default     {0}
        @h2xmle_rangeList   {"8 kHz"=8000;
                             "11.025 kHz"=11025;
                             "12 kHz"=12000;
                             "16 kHz"=16000;
                             "22.05 kHz"=22050;
                             "24 kHz"=24000;
                             "32 kHz"=32000;
                             "44.1 kHz"=44100;
                             "48 kHz"=48000;
                             "88.2 kHz"=88200;
                             "96 kHz"=96000;
                             "176.4 kHz"=176400;
                             "192 kHz"=192000;
                             "352.8 kHz"=352800;
                             "384 kHz"=384000}
        @h2xmle_policy      {Basic} */

   uint16_t bits_per_sample;
   /**< Number of bits required to store one sample.

        @valuesbul
        - #INVALID_VALUE (Default)
        - #BITS_PER_SAMPLE_16
        - #BITS_PER_SAMPLE_32 @tablebulletend */

   /*#< @h2xmle_description {Number of bits required to store one sample.}
        @h2xmle_default     {0}
        @h2xmle_rangeList   {"INVALID_VALUE"=0;
                             "BITS_PER_SAMPLE_16"=16;
                             "BITS_PER_SAMPLE_32"=32}
        @h2xmle_policy      {Basic} */

   uint16_t num_channels;
   /**< Number of channels.

        @values 0 through 32 (Default = 0) */

   /*#< @h2xmle_description {Number of channels.}
        @h2xmle_default     {0}
        @h2xmle_range       {0..32}
        @h2xmle_policy      {Basic} @newpagetable */
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;
typedef struct payload_data_fmt_generic_compressed_t payload_data_fmt_generic_compressed_t;


/*************************************************** PCM Media Format *************************************************/

/** @ingroup ar_spf_mod_media_fmt_ids
    Identifier for the media format used to identify a PCM stream.

    Following is the overall payload structure:

    @code
    struct complete_media_fmt_pcm_t
    {
       payload_media_fmt_pcm_t pcm;
       uint8_t ch_map[num_channels];
    }
    @endcode

    @msgpayload
    payload_media_fmt_pcm_t
 */
#define MEDIA_FMT_ID_PCM                  0x09001000


/** @ingroup ar_spf_mod_media_fmt_ids
    Payload of #MEDIA_FMT_ID_PCM when media_format_t::fmt_id =
    #MEDIA_FMT_ID_PCM and data_format = #DATA_FORMAT_FIXED_POINT.
 */
#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
struct payload_media_fmt_pcm_t
{
   uint32_t sample_rate;
   /**< Number of samples per second in the audio stream.

        @values 0 to 384000 Hz */

   /*#< @h2xmle_description {Number of samples per second in the audio stream.}
        @h2xmle_default     {0}
        @h2xmle_rangeList   {"8 kHz"=8000;
                             "11.025 kHz"=11025;
                             "12 kHz"=12000;
                             "16 kHz"=16000;
                             "22.05 kHz"=22050;
                             "24 kHz"=24000;
                             "32 kHz"=32000;
                             "44.1 kHz"=44100;
                             "48 kHz"=48000;
                             "88.2 kHz"=88200;
                             "96 kHz"=96000;
                             "176.4 kHz"=176400;
                             "192 kHz"=192000;
                             "352.8 kHz"=352800;
                             "384 kHz"=384000}
        @h2xmle_policy      {Basic} */

   uint16_t bit_width;
   /**< Bit width of each sample.

        @valuesbul
        - #INVALID_VALUE (Default)
        - #BIT_WIDTH_16 Valid only for fixed point data format
        - #BIT_WIDTH_24 Valid only for fixed point data format
        - #BIT_WIDTH_32 Valid for fixed point and floating point data format
   	   	- #BIT_WIDTH_64 Valid only for floating point data format
   	   	 @tablebulletend */
   /*#< @h2xmle_description {Bit width of each sample.}
        @h2xmle_default     {0}
        @h2xmle_rangeList   {"INVALID_VALUE"=0;
                             "BIT_WIDTH_16"=16;
                             "BIT_WIDTH_24"=24;
                             "BIT_WIDTH_32"=32;
                             "BIT_WIDTH_64"=64}
        @h2xmle_policy      {Basic} */

   uint8_t alignment;
   /**< Alignment of bits_per_sample in sample_word_size.

        This field is relevant only when bits_per_sample = 24 and word_size
        = 32.

        @valuesbul
        - #INVALID_VALUE (Default)
        - #PCM_LSB_ALIGNED
        - #PCM_MSB_ALIGNED @tablebulletend */

   /*#< @h2xmle_description {Alignment of bits_per_sample in sample_word_size.
                             This field is relevant only when bits_per_sample =
                             24 and word_size = 32.}
        @h2xmle_default     {0}
        @h2xmle_rangeList   {"INVALID_VALUE"=0;
                             "PCM_LSB_ALIGNED"=1;
                             "PCM_MSB_ALIGNED"=2}
        @h2xmle_policy      {Basic} */

   uint8_t interleaved;
   /**< Indicates whether the data is interleaved.

        When the value is invalid, the data is assumed to be interleaved.

        @valuesbul
        - #INVALID_VALUE (Default)
        - #PCM_INTERLEAVED @tablebulletend */

   /*#< @h2xmle_description {Indicates whether the data is interleaved. When
                             the value is invalid, the data is assumed to be
                             interleaved.}
        @h2xmle_default     {0}
        @h2xmle_rangeList   {"INVALID_VALUE"=0;
                             "PCM_INTERLEAVED"=1}
        @h2xmle_policy      {Basic} */

   uint16_t bits_per_sample;
   /**< Number of bits required to store one sample.

        @valuesbul
        - #INVALID_VALUE (Default)
        - #BITS_PER_SAMPLE_16
        - #BITS_PER_SAMPLE_24
        - #BITS_PER_SAMPLE_32
        - #BITS_PER_SAMPLE_64

        @tblsubhdbul{For BITS\_PER\_SAMPLE\_32}
        - If bit_width = 24 and alignment = LSB aligned, the 24-bit samples are
          placed in the lower 24 bits of the 32-bit word. The upper bits might
          be sign-extended.
        - If bit_width = 24 and alignment = MSB aligned, the 24-bit samples are
          placed in the upper 24 bits of the 32-bit word. The lower bits might
          be set to 0.
        - If bit width = 32, 32-bit samples are placed in the 32-bit words.
        - BITS_PER_SAMPLE_64- Valid only for floating point data format.
        @tablebulletend */

   /*#< @h2xmle_description {Number of bits required to store one sample.}
        @h2xmle_default     {0}
        @h2xmle_rangeList   {"INVALID_VALUE"=0;
                             "BITS_PER_SAMPLE_16"=16;
                             "BITS_PER_SAMPLE_24"=24;
                             "BITS_PER_SAMPLE_32"=32;
                             "BITS_PER_SAMPLE_64"=64}
        @h2xmle_policy      {Basic} */

   uint16_t q_factor;
   /**< Indicates the Q factor of the PCM data:
        - Q15 for 16-bit_width signed data.
        - Q23 for 24-bit_width signed packed (24-bits_per_sample) data.
        - Q27 for LSB-aligned, 24-bit_width unpacked (32-bits_per_sample)
          signed data used internally to the SPF.
        - Q31 for MSB-aligned, 24-bit_width unpacked (32 bits_per_sample)
          signed data.
        - Q23 for LSB-aligned, 24-bit_width unpacked (32 bits_per_sample)
          signed data.
        - Q31 for 32-bit_width signed data@tablebulletend
        - Not applicable for floating point data format */

   /*#< @h2xmle_description {Indicates the Q factor of the PCM data:
                             - Q15 for 16-bit_width signed data
                             - Q23 for 24-bit_width signed packed
                               (24-bits_per_sample) data
                             - Q27 for LSB-aligned, 24-bit_width unpacked
                               (32-bits_per_sample) signed data used internally
                               to the SPF
                             - Q31 for MSB-aligned, 24-bit_width unpacked
                               (32 bits_per_sample) signed data
                             - Q23 for LSB-aligned, 24-bit_width unpacked
                               (32 bits_per_sample) signed data
                             - Q31 for 32-bit_width signed data}
        @h2xmle_default     {0}
        @h2xmle_rangeList   {"PARAM_VAL_INVALID"=0;
                             "Q15"=15;
                             "Q23"=23;
                             "Q27"=27;
                             "Q31"=31}
        @h2xmle_policy      {Basic} */

   uint16_t endianness;
   /**< Indicates whether PCM samples are stored in little endian or big endian
        format.

        @valuesbul
        - #INVALID_VALUE (Default)
        - #PCM_LITTLE_ENDIAN
        - #PCM_BIG_ENDIAN @tablebulletend */

   /*#< @h2xmle_description {Indicates whether PCM samples are stored in little
                             endian or big endian format.}
        @h2xmle_default     {0}
        @h2xmle_rangeList   {"INVALID_VALUE"=0;
                             "PCM_LITTLE_ENDIAN"=1;
                             "PCM_BIG_ENDIAN"=2}
        @h2xmle_policy      {Basic} */

   uint16_t num_channels;
   /**< Number of channels in the channel_mapping array.

        @values 0 through 128 (Default = 0) */

   /*#< @h2xmle_description {Number of channels.}
        @h2xmle_default     {0}
        @h2xmle_range       {0..128}
        @h2xmle_policy      {Basic} */

#if defined(__H2XML__)
   uint8_t channel_mapping[0];
   /**< Array of channel mappings of size num_channels.

        Channel[i] mapping describes channel i. Each element i of the array
        describes channel i inside the buffer where i is less than
        num_channels. An unused channel is set to 0. @newpagetable */

   /*#< @h2xmle_description      {Channel mapping array of variable size. The
                                  size of this array depends upon number of
                                  channels. Channel[i] mapping describes
                                  channel i. Each element i of the array
                                  describes channel i inside the buffer where i
                                  is less than num_channels. An unused channel
                                  is set to 0.}
        h2xmle_variableArraySize {(num_channels*4 + 3) / 4}
        @h2xmle_default          {0}
        @h2xmle_policy           {Basic} */

#endif
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;
typedef struct payload_media_fmt_pcm_t payload_media_fmt_pcm_t;


/********************************************** Media Format Parameter ************************************************/

/** @ingroup ar_spf_mod_media_fmt_ids
    Identifier for the parameter used to set the media format on any Shared
    Memory Endpoint module.

    This parameter ID is set via APM_CMD_SET_CFG (see AudioReach Signal Processing Framework (SPF) API Reference). It is accepted only when the subgraph is
    in the Stop or Prepare state.

    @msgpayload
    media_format_t
 */
#define PARAM_ID_MEDIA_FORMAT       0x0800100C

/*# @h2xmlp_parameter   {"PARAM_ID_MEDIA_FORMAT", PARAM_ID_MEDIA_FORMAT}
    @h2xmlp_toolPolicy  {Calibration}
    @h2xmlp_description {ID for the parameter used to set the media format on
                         any Shared Memory Endpoint module. This parameter ID
                         is set via APM_CMD_SET_CFG (see AudioReach Signal Processing Framework (SPF) API Reference). It is accepted only
                         when the subgraph is in the Stop or Prepare state.} */

/** @ingroup ar_spf_mod_media_fmt_ids
    Payload structure used by the following opcodes:
    - #PARAM_ID_MEDIA_FORMAT
    - #DATA_CMD_WR_SH_MEM_EP_MEDIA_FORMAT
    - #DATA_EVENT_ID_RD_SH_MEM_EP_MEDIA_FORMAT

    Immediately following this structure is a payload_size number of bytes that
    represent the actual media format block.

    Following is the overall payload structure:

    @code
    media_format_t mf;
    uint8_t payload[payload_size];
    @endcode @vertspace{6}
 */
#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
struct media_format_t
{
   uint32_t data_format;
   /**< Format of the data.

        @valuesbul
        - #INVALID_VALUE (Default)
        - #DATA_FORMAT_FIXED_POINT
        - #DATA_FORMAT_IEC61937_PACKETIZED
        - #DATA_FORMAT_IEC60958_PACKETIZED
        - #DATA_FORMAT_DSD_OVER_PCM
        - #DATA_FORMAT_GENERIC_COMPRESSED
        - #DATA_FORMAT_RAW_COMPRESSED @tablebulletend */

   /*#< @h2xmle_description {Format of the data.}
        @h2xmle_default     {0}
        @h2xmle_rangeList   {"INVALID_VALUE"=0,
                             "DATA_FORMAT_FIXED_POINT"=1,
                             "DATA_FORMAT_IEC61937_PACKETIZED"=2,
                             "DATA_FORMAT_IEC60958_PACKETIZED"=3,
                             "DATA_FORMAT_DSD_OVER_PCM"=4,
                             "DATA_FORMAT_GENERIC_COMPRESSED"=5,
                             "DATA_FORMAT_RAW_COMPRESSED"=6}
        @h2xmle_policy      {Basic} */

   uint32_t fmt_id;
   /**< Media format ID of the data stream.

        @valuesbul
        - #INVALID_VALUE (Default)
        - #MEDIA_FMT_ID_PCM @tablebulletend */

   /*#< @h2xmle_description {Media format ID of the data stream.}
        @h2xmle_default     {0}
        @h2xmle_rangeList   {"INVALID_VALUE"=0,
                             "Media format ID of PCM"=MEDIA_FMT_ID_PCM}
        @h2xmle_policy      {Basic} */

   uint32_t payload_size;
   /**< Size of the payload that immediately follows this structure.

        This size does not include bytes added for 32-bit alignment. */

   /*#< @h2xmle_description {Size of the payload that immediately follows this
                             structure. This size does not include bytes added
                             for 32-bit alignment.}
        @h2xmle_default     {0}
        @h2xmle_range       {0..0xFFFFFFFF}
        @h2xmle_policy      {Basic} */

#if defined(__H2XML__)
   uint8_t  payload[0];
   /**< Payload for media format configurations.

        The payload structure varies depending on the combination of the
        data_format and fmt_id fields and is of size payload_size. For example,
        a PCM fixed point (payload_media_fmt_pcm_t) and floating point might
        have different payloads. @newpagetable */

   /*#< @h2xmle_description       {Payload for media format configurations.
                                   The payload structure varies depending on
                                   the combination of the data_format and
                                   fmt_id fields and is of size payload_size.
                                   For example, if fmt_id=PARAM_ID_PCM_OUTPUT_FORMAT_CFG
                                   and data_format=DATA_FORMAT_FIXED_POINT, the
                                   payload is of type payload_media_fmt_pcm_t.
                                   The floating point might have a different
                                   payload.}
        @h2xmle_policy            {Basic} */

#endif
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;
typedef struct media_format_t media_format_t;


/*********************************** Encoder Configuration Block (Encoder Output Config) ******************************/

/** @ingroup ar_spf_mod_media_fmt_ids
    Identifier for the parameter that configures the encoder.

    Encoding occurs at the incoming sample rate, number of channels, and bit
    width. These parameters are controlled by the resamplers, MFC, channel
    mixer, or PCM converter. Not all encoders support the encoder output
    configuration.

    Following is the overall payload structure:

    @code
    {
       param_id_encoder_output_config_t cfg;
       uint8_t custom_enc_cfg[payload_size];
       uint8_t padding[if_any];
    }
    @endcode

    @msgpayload
    param_id_encoder_output_config_t @newpage
 */
#define PARAM_ID_ENCODER_OUTPUT_CONFIG                   0x08001009

/*# @h2xmlp_parameter   {"PARAM_ID_ENCODER_OUTPUT_CONFIG",
                          PARAM_ID_ENCODER_OUTPUT_CONFIG}
    @h2xmlp_description {ID for the parameter that configures the encoder.
                         Encoding occurs at the incoming sample rate, number
                         of channels, and bit width. These parameters are
                         controlled by the resamplers, MFC, channel mixer, or
                         PCM converter. Not all encoders support the encoder
                         output configuration.
    @h2xmlp_toolPolicy  {Calibration} */


/** @ingroup ar_spf_mod_media_fmt_ids
    Payload of the #PARAM_ID_ENCODER_OUTPUT_CONFIG parameter.
*/
#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
struct param_id_encoder_output_config_t
{
   uint32_t      data_format;
   /**< Format of the output data.

        @valuesbul
        - #INVALID_VALUE (Default)
        - #DATA_FORMAT_FIXED_POINT @tablebulletend */

   /*#< @h2xmle_description {Format of the output data.}
        @h2xmle_default     {0}
        @h2xmle_rangeList   {"PARAM_VAL_INVALID"=0,
                             "DATA_FORMAT_FIXED_POINT"=1}
        @h2xmle_policy      {Basic} */

   uint32_t       fmt_id;
   /**< Media format ID for the output data stream.

        For the PCM encoding use case, this value must match the fmt_id in
        #PARAM_ID_PCM_OUTPUT_FORMAT_CFG.

        @valuesbul
        - #INVALID_VALUE (Default)
        - #MEDIA_FMT_ID_PCM @tablebulletend */

   /*#< @h2xmle_description {Media format ID of the output data stream.
                             For the PCM encoding use case, this value must
                             match the fmt_id in
                             PARAM_ID_PCM_OUTPUT_FORMAT_CFG.}
        @h2xmle_default     {0}
        @h2xmle_rangeList   {"INVALID_VALUE"=0,
                             "Media format ID of PCM"=MEDIA_FMT_ID_PCM}
        @h2xmle_policy      {Basic} */

   uint32_t       payload_size;
   /**< Size of the custom payload that immediately follows this structure.

        This size does not include bytes added for 32-bit alignment. */

   /*#< @h2xmle_description {Size of the custom payload that immediately
                             follows this structure. This size does not
                             include bytes added for 32-bit alignment.}
        @h2xmle_default     {0}
        @h2xmle_range       {0..0xFFFFFFFF}
        @h2xmle_policy      {Basic} */

#if defined(__H2XML__)
   uint8_t  payload[0];
   /**< Custom payload for encoder output configuration of size payload_size.

        The payload structure varies depending on the combination of the
        data_format and fmt_id fields of size payload_size. For example, a PCM
        fixed point (payload_media_fmt_pcm_t) and floating point might have
        different payloads.

        The floating point data format might have a different payload. */

   /*#< @h2xmle_description       {Payload for encoder output configuration of
                                   size payload_size. The payload structure
                                   varies depending on the combination of the
                                   data_format and fmt_id fields. For example,
                                   if data_format=#DATA_FORMAT_FIXED_POINT and
                                   fmt_id = #MEDIA_FMT_ID_PCM, the payload is
                                   payload_pcm_output_format_cfg_t.
                                   The floating-point data format might have a
                                   different payload.}
        @h2xmle_policy            {Basic} */
#endif
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;
typedef struct param_id_encoder_output_config_t param_id_encoder_output_config_t;


#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* MEDIA_FMT_API_BASIC_H */
