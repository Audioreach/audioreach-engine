#ifndef TOPOLOGY_UTILS_H_
#define TOPOLOGY_UTILS_H_

/**
 * \file topo_utils.h
 *
 * \brief
 *
 *     Utility functions common across topologies Topology utils are utility functions that can be used by all topology
 *  implementations. The functions do not take any topology objects as arguments.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "apm_api.h"
#include "apm_private_api.h"
#include "apm_sub_graph_api.h"
#include "module_cmn_api.h"
#include "spf_utils.h"
#include "capi.h"
#include "graph_utils.h"
#include "spf_macros.h"
#include "capi_cmn.h"
#include "cntr_cntr_if.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

// clang-format off

//#define PROC_DELAY_DEBUG

/**
 * log-ID is used for
 * a) MSG logging: container-type-mask and sequence-mask are printed
 * b) QXDM data dump files: all fields are relevant
 */
#define LOG_ID_CNTR_TYPE_SHIFT              28
#define LOG_ID_SEQUENCE_SHIFT                12
#define LOG_ID_LOG_MODULE_INSTANCES_SHIFT    6
#define LOG_ID_LOG_DISCONTINUITY_SHIFT       0

/** 16 container container type */
#define LOG_ID_CNTR_TYPE_MASK                0xF0000000
/** sequence-id will wrap around after 65k */
#define LOG_ID_SEQUENCE_MASK                 0x0FFFF000
/** 64 log modules (only 32 supported now) in a container */
#define LOG_ID_LOG_MODULE_INSTANCES_MASK     0x00000FC0
/** 64 EOS / flush & other discontinuities (see Elite_fwk_extns_logging.h)*/
#define LOG_ID_LOG_DISCONTINUITY_MASK        0x0000003F


#define MAX_SAMPLING_RATE_HZ SAMPLE_RATE_384K
#ifdef PROD_SPECIFIC_MAX_CH
#define MAX_NUM_CHANNELS PROD_SPECIFIC_MAX_CH
#else
#define MAX_NUM_CHANNELS 32
#endif


#define TOPO_MSG_PREFIX "TU  :%08X: "
#define TOPO_MSG(ID, xx_ss_mask, xx_fmt, ...) AR_MSG(xx_ss_mask, TOPO_MSG_PREFIX xx_fmt, ID, ##__VA_ARGS__)
#define TOPO_MSG_ISLAND(ID, xx_ss_mask, xx_fmt, ...) AR_MSG_ISLAND(xx_ss_mask, TOPO_MSG_PREFIX xx_fmt, ID, ##__VA_ARGS__)

#ifndef ALIGN_8_BYTES
#define ALIGN_8_BYTES(a)   ((a + 7) & (0xFFFFFFF8))
#endif
// clang-format on
#define TOPO_IS_POW_OF_2(x) (!((x) & ((x)-1)))

#define BYTES_TO_BYTES_PER_CH(bytes, media_fmt_ptr)                                                                    \
   (SPF_IS_PACKETIZED_OR_PCM(media_fmt_ptr->data_format) ? (topo_div_num(bytes, media_fmt_ptr->pcm.num_channels))      \
                                                         : bytes)
/**
 * Must be called only for pcm-packetized
 */

#define TOPO_PRINT_PCM_MEDIA_FMT(log_id, media_fmt_ptr, IOTYPE)                \
  do {                                                                         \
    TOPO_MSG(log_id, DBG_HIGH_PRIO,                                            \
             SPF_LOG_PREFIX IOTYPE                                             \
             " media format: data format %lu, ch=%lu, SR=%lu, bit_width=%lu, " \
             "bits_per_sample=%lu, Q_fct=%lu, "                                \
             "endianness %lu, interleaving %lu (1->int,  2->deint pack, "      \
             "4->deint unpack 8->deint unpack v2)",                            \
             media_fmt_ptr->data_format, media_fmt_ptr->pcm.num_channels,      \
             media_fmt_ptr->pcm.sample_rate, media_fmt_ptr->pcm.bit_width,     \
             media_fmt_ptr->pcm.bits_per_sample, media_fmt_ptr->pcm.q_factor,  \
             media_fmt_ptr->pcm.endianness, media_fmt_ptr->pcm.interleaving);  \
                                                                               \
    uint8_t c[MAX_NUM_CHANNELS] = {0};                                         \
    uint32_t *c_ptr = (uint32_t *)c;                                           \
    for (uint32_t i = 0; i < media_fmt_ptr->pcm.num_channels; i += 4) {        \
      for (int32_t j = 3, k = 0; j >= 0; j--, k++) {                           \
        c[i + k] = media_fmt_ptr->pcm.chan_map[i + j];                         \
      }                                                                        \
    }                                                                          \
                                                                               \
    TOPO_MSG(log_id, DBG_HIGH_PRIO,                                            \
             SPF_LOG_PREFIX IOTYPE                                             \
             " media format:  channel mapping (hex bytes)  %08lx %08lx %08x "  \
             "%08x  %08lx %08lx %08x %08x",                                    \
             c_ptr[0], c_ptr[1], c_ptr[2], c_ptr[3], c_ptr[4], c_ptr[5],       \
             c_ptr[6], c_ptr[7]);                                              \
                                                                               \
  } while (0)

#define TOPO_PRINT_MEDIA_FMT(log_id, module_ptr, out_port_ptr, media_fmt_ptr, IOTYPE)                                  \
   do                                                                                                                  \
   {                                                                                                                   \
      TOPO_MSG(log_id,                                                                                                 \
               DBG_HIGH_PRIO,                                                                                          \
               SPF_LOG_PREFIX "Module 0x%lX (0x%lX): Port Id 0x%lx, propagating? %u, " IOTYPE                          \
                              " media format: data_format %lu, fmt_id 0x%lX",                                          \
               module_ptr->gu.module_instance_id,                                                                      \
               module_ptr->gu.module_id,                                                                               \
               out_port_ptr->gu.cmn.id,                                                                                \
               out_port_ptr->common.flags.media_fmt_event,                                                             \
               media_fmt_ptr->data_format,                                                                             \
               media_fmt_ptr->fmt_id);                                                                                 \
      if (SPF_IS_PACKETIZED_OR_PCM(media_fmt_ptr->data_format))                                                        \
      {                                                                                                                \
         TOPO_PRINT_PCM_MEDIA_FMT(log_id, media_fmt_ptr, IOTYPE);                                                      \
      }                                                                                                                \
      if (SPF_DEINTERLEAVED_RAW_COMPRESSED == media_fmt_ptr->data_format)                                              \
      {                                                                                                                \
         TOPO_MSG(log_id,                                                                                              \
                  DBG_HIGH_PRIO,                                                                                       \
                  SPF_LOG_PREFIX IOTYPE " media format: num_bufs=%lu",                                                 \
                  media_fmt_ptr->deint_raw.bufs_num);                                                                  \
         TOPO_MSG(log_id,                                                                                              \
                  DBG_HIGH_PRIO,                                                                                       \
                  SPF_LOG_PREFIX IOTYPE                                                                                \
                  " media format: ch_mask[0] 0x%lX%lX, ch_mask[1] 0x%lX%lX, ch_mask[2] 0x%lX%lX, ch_mask[3] 0x%lX%lX", \
                  media_fmt_ptr->deint_raw.ch_mask[0].channel_mask_msw,                                                \
                  media_fmt_ptr->deint_raw.ch_mask[0].channel_mask_lsw,                                                \
                  media_fmt_ptr->deint_raw.ch_mask[1].channel_mask_msw,                                                \
                  media_fmt_ptr->deint_raw.ch_mask[1].channel_mask_lsw,                                                \
                  media_fmt_ptr->deint_raw.ch_mask[2].channel_mask_msw,                                                \
                  media_fmt_ptr->deint_raw.ch_mask[2].channel_mask_lsw,                                                \
                  media_fmt_ptr->deint_raw.ch_mask[3].channel_mask_msw,                                                \
                  media_fmt_ptr->deint_raw.ch_mask[3].channel_mask_lsw);                                               \
      }                                                                                                                \
   } while (0)
// clang-format off
#define TOPO_BITS_TO_BYTES(x) CAPI_CMN_BITS_TO_BYTES(x)
#define TOPO_BYTES_TO_BITS(x) CAPI_CMN_BYTES_TO_BITS(x)

#define TOPO_CEIL(x, y) CAPI_CMN_CEIL(x, y)

/**
 *                                        |------------------------------------------------------------------------------------|
 *                                        | spf Client APIs       Topo               CAPIv2            spf
 * Inter-Container |
 *                                        |------------------------------------------------------------------------------------|
 * Q format (15,23,27,31)                 | Q factor                Q factor           Q factor          Q factor |
 * Each audio sample bit width (16,24,32) | bit width               bit width          NA                NA |
 * Each audio sample word size (16,24,32) | bits per sample         bits per sample    bits per sample   bits per sample
 * |
 *                                        |------------------------------------------------------------------------------------|
 */
#define TOPO_QFORMAT_TO_BIT_WIDTH(q) ((PCM_Q_FACTOR_15 == q) ? 16 : ((PCM_Q_FACTOR_23 == q) ? 24 : ((PCM_Q_FACTOR_27 == q) ? 24 : 32)))

#define TOPO_BYTES_PER_SEC(media_fmt)                                                                                  \
   ((media_fmt).pcm.sample_rate * (media_fmt).pcm.num_channels * TOPO_BITS_TO_BYTES((media_fmt).pcm.bits_per_sample))

#define TOPO_MIN_SIZE_OF_RAW_MEDIA_FMT                                                                                 \
   (sizeof(capi_set_get_media_format_t) + sizeof(capi_raw_compressed_data_format_t))

#define PERF_MODE_LOW_POWER_FRAME_DURATION_MS      (5)

#define PERF_MODE_LOW_LATENCY_FRAME_DURATION_MS    (1)

#define TOPO_PERF_MODE_TO_FRAME_DURATION_MS(perf_mode) ((APM_SG_PERF_MODE_LOW_POWER == perf_mode) ?                    \
                                                      PERF_MODE_LOW_POWER_FRAME_DURATION_MS :                          \
                                                      PERF_MODE_LOW_LATENCY_FRAME_DURATION_MS)

/**
 * to take up same memory as 32 channels.
 */
#define TOPO_MAX_NUM_BUFS_DEINT_RAW_COMPR (CAPI_MAX_CHANNELS_V2/8)

typedef enum topo_sg_state_t
{
   TOPO_SG_STATE_STOPPED   = 0,
   TOPO_SG_STATE_PREPARED  = 1,
   TOPO_SG_STATE_STARTED   = 2,
   TOPO_SG_STATE_SUSPENDED = 3,
   TOPO_SG_STATE_INVALID  = 0XFFFFFFFF,
} topo_sg_state_t;

/**
 * Codes for subgraph operations.
 */
typedef enum topo_sg_operation_t
{
   TOPO_SG_OP_FLUSH      = 0x0001,
   TOPO_SG_OP_START      = 0x0004,
   TOPO_SG_OP_PREPARE    = 0x0008,
   TOPO_SG_OP_STOP       = 0x0010,
   TOPO_SG_OP_CLOSE      = 0x0020,
   TOPO_SG_OP_DISCONNECT = 0x0040,
   TOPO_SG_OP_SUSPEND    = 0x0080,
   TOPO_SG_OP_INVALID    = 0xFFFF
} topo_sg_operation_t;

/**
 * Port states. Applicable to internal and external ports.
 * Note: the port state have a one to one mapping with topo_sg_state_t.
 * refer topo_sg_state_to_port_state()
 */
typedef enum topo_port_state_t
{
   TOPO_PORT_STATE_STOPPED = 0,
   TOPO_PORT_STATE_PREPARED,
   TOPO_PORT_STATE_STARTED,
   TOPO_PORT_STATE_SUSPENDED,
   TOPO_PORT_STATE_INVALID = 0xFFFFFFFF
} topo_port_state_t;

/**
 * data flow states
 */
typedef enum topo_data_flow_state_t
{
   TOPO_DATA_FLOW_STATE_AT_GAP = 0,   // at upstream stop or initial state.
                                      // buffer arrival changes to FLOWING. MF arrival changes to PREFLOW.
   //TOPO_DATA_FLOW_STATE_PREFLOW,      // After upstream sends media format, temporarily removed due to complexity
   TOPO_DATA_FLOW_STATE_FLOWING,      // After upstream sends first data buffers, EOS departure or reset due to self-STOP results in AT_GAP.
} topo_data_flow_state_t;

typedef enum topo_endianness_t { TOPO_UNKONWN_ENDIAN = 0, TOPO_LITTLE_ENDIAN, TOPO_BIG_ENDIAN } topo_endianness_t;

typedef enum topo_interleaving_t
{
   TOPO_INTERLEAVING_UNKNOWN = 0,
   TOPO_INTERLEAVED               = (1 << 0),
   TOPO_DEINTERLEAVED_PACKED      = (1 << 1),
   TOPO_DEINTERLEAVED_UNPACKED    = (1 << 2),
   TOPO_DEINTERLEAVED_UNPACKED_V2 = (1 << 3)
} topo_interleaving_t;

typedef enum topo_port_type_t
{
   TOPO_DATA_INVALID_PORT_TYPE=0,
   TOPO_DATA_INPUT_PORT_TYPE,
   TOPO_DATA_OUTPUT_PORT_TYPE,
   TOPO_CONTROL_PORT_TYPE,
} topo_port_type_t;
/* Port property propagation type.
 *
 * Used by generic port property propagations functions to indicate the
 * type of payload thats being propagated. */
typedef enum topo_port_property_type_t
{
   PORT_PROPERTY_ID_INVALID = 0,
   PORT_PROPERTY_IS_UPSTREAM_RT,
   PORT_PROPERTY_IS_DOWNSTREAM_RT,
   PORT_PROPERTY_TOPO_STATE,
   PORT_PROPERTY_DATA_FLOW_STATE, // note this prop is not propagated (it's assigned as data flows)
   PORT_PROPERTY_DOWNSTREAM_REQUIRES_DATA_BUFFERING,

} topo_port_property_type_t;

/** Returns capi_stream_data_v2_t.stream_data_version field BIT mask */
static inline uint32_t __topo_sdata_flag_stream_data_ver_bitmask()
{
	capi_stream_flags_t temp_flag;
	temp_flag.word = 0xFFFFFFFF;
	temp_flag.stream_data_version = 0;
	return (~temp_flag.word);
}

/** Returns capi_stream_data_v2_t.marker_eos field BIT mask */
static inline uint32_t __topo_sdata_flag_marker_eos_bitmask()
{
	capi_stream_flags_t temp_flag;
	temp_flag.word = 0xFFFFFFFF;
	temp_flag.marker_eos  = 0;
	return (~temp_flag.word);
}

/** Returns capi_stream_data_v2_t.end_of_frame field BIT mask */
static inline uint32_t __topo_sdata_flag_end_of_frame_bitmask()
{
	capi_stream_flags_t temp_flag;
	temp_flag.word = 0xFFFFFFFF;
	temp_flag.end_of_frame  = 0;
	return (~temp_flag.word);
}

/** Returns capi_stream_data_v2_t.end_of_frame field BIT mask */
static inline uint32_t __topo_sdata_flag_is_ts_valid_bitmask()
{
	capi_stream_flags_t temp_flag;
	temp_flag.word = 0xFFFFFFFF;
	temp_flag.is_timestamp_valid  = 0;
	return (~temp_flag.word);
}

/*CAPI SDATA stream flag bit masks*/
#define  SDATA_FLAG_END_OF_FRAME_BIT_MASK          (__topo_sdata_flag_end_of_frame_bitmask())     // end of frame
#define  SDATA_FLAG_MARKER_EOS_BIT_MASK            (__topo_sdata_flag_marker_eos_bitmask())       // marker eos
#define  SDATA_FLAG_STREAM_DATA_VERSION_BIT_MASK   (__topo_sdata_flag_stream_data_ver_bitmask())  // Stream data version
#define  SDATA_FLAG_IS_TS_VALID_BIT_MASK           (__topo_sdata_flag_is_ts_valid_bitmask())      // timestamp validity

// checks if EOS or EOF sdata flag is set.
static inline bool_t topo_is_sdata_flag_EOS_or_EOF_set(capi_stream_data_v2_t *sdata_ptr)
{
	// check if any bit other than stream data version flags are set [valid bit mask: 0x247]
	return (sdata_ptr->flags.word & (SDATA_FLAG_MARKER_EOS_BIT_MASK | SDATA_FLAG_END_OF_FRAME_BIT_MASK));
}

// returns mask of the flags other than stream data verion and ts validity
static inline uint32_t topo_sdata_get_flag_mask_other_than_ver_and_is_ts_valid(capi_stream_data_v2_t *sdata_ptr)
{
	// check if any bit other than stream data version flags are set [valid bit mask: 0x81]
	return sdata_ptr->flags.word & (~(SDATA_FLAG_STREAM_DATA_VERSION_BIT_MASK | SDATA_FLAG_IS_TS_VALID_BIT_MASK));
}

//typedef struct capi_buf_t topo_buf_t; /**< max len is nonzero only when data_ptr is set */

typedef struct topo_buf_t topo_buf_t;

struct topo_buf_t
{
   int8_t   *data_ptr;
   uint32_t actual_data_len;
   uint32_t max_data_len;           /**< max len is nonzero only when data_ptr is set */
};

/**
 * Media format for PCM and packetized data
 */
typedef struct topo_pcm_pack_med_fmt_t
{
   uint32_t                sample_rate;
   uint8_t                 bit_width;        /**< 16, 24, 32 (actual bit width independent of the word size)*/
   uint8_t                 bits_per_sample;  /**< bits per sample 16 or 32. This is actually word size in bits*/
   uint8_t                 q_factor;         /**< 15, 27, 31 */
   uint8_t                 num_channels;
   topo_interleaving_t     interleaving;
   topo_endianness_t       endianness;
   uint8_t                 chan_map[CAPI_MAX_CHANNELS_V2];
} topo_pcm_pack_med_fmt_t;

/**
 * Raw Media Format
 */
typedef struct topo_raw_med_fmt_t
{
   uint8_t  *buf_ptr; /**< struct of this mem: capi_set_get_media_format_t, capi_raw_compressed_data_format_t, payload*/
   /*
   - When sending mf between containers, we only send the payload(from above line).
         Note:
         * Encoders will raise this mf with no payload.
           So, buf_ptr will be NULL. In this case, We only populate the topo_media_fmt_t with data_format and format ID and send it.
   - When a container receives raw mf from peer, it'll call the tu_capi_create_raw_compr_med_fmt function with with_header = TRUE
     From then on, buf_ptr will have a value and it'll adhere to the structure: capi_set_get_media_format_t, capi_raw_compressed_data_format_t
   */
   uint32_t  buf_size;
} topo_raw_med_fmt_t;

typedef struct topo_deint_raw_med_fmt_t
{
   uint32_t             bufs_num;
   capi_channel_mask_t  ch_mask[TOPO_MAX_NUM_BUFS_DEINT_RAW_COMPR];
} topo_deint_raw_med_fmt_t;

/**
 * zero means not set with specific value
 *
 * even though pcm and raw will not be concurrent, when MF changes from raw to PCM,
 * using union can cause issues (interpreting pcm memory as raw.buf_ptr)
 */
typedef struct topo_media_fmt_t
{
   spf_data_format_t       data_format;
   uint32_t                fmt_id;
   union
   {
      topo_pcm_pack_med_fmt_t  pcm;          /**< when SPF_IS_PACKETIZED_OR_PCM(data_fmt is true)*/
      topo_deint_raw_med_fmt_t deint_raw;    /**< for SPF_DEINTERLEAVED_RAW_COMPRESSED */
   };
   topo_raw_med_fmt_t      raw; /**< when SPF_RAW_COMPRESSED == data_format*/
} topo_media_fmt_t;

/**
 *  Defined this for the 64 bit destination address used
 *  for registering events with modules
 */
typedef union topo_evt_dest_addr_t
{
   uint64_t address;
   struct
   {
      uint32_t src_port;
      uint8_t  src_domain_id;
      uint8_t  dest_domain_id;
      uint8_t  gpr_heap_index;
   } a;
} topo_evt_dest_addr_t;

/**
 *  Used to pass the register event data to containers
 */
typedef struct topo_reg_event_t
{
   uint32_t          token;
   uint32_t          event_id;
   capi_buf_t        event_cfg;
   uint32_t          src_port;
   uint8_t           src_domain_id;
   uint8_t           dest_domain_id;
   uint8_t           gpr_heap_index;
} topo_reg_event_t;


/**
 *  media format utility structure.
 *  this utility maintains the list of different media format used in a topo and assign them to the ports.
 */
typedef struct topo_mf_utils_t
{
  spf_list_node_t *mf_node_ptr; //list of different media format blocks.
  uint32_t num_nodes;       //number of different media format blocks in the list.
} topo_mf_utils_t;

/**
 * Convert subgraph state to port state.
 */
static inline topo_port_state_t topo_sg_state_to_port_state(topo_sg_state_t sg_state)
{
   topo_port_state_t port_state_array[] = {
      TOPO_PORT_STATE_STOPPED,  // TOPO_SG_STATE_STOPPED
      TOPO_PORT_STATE_PREPARED, // TOPO_SG_STATE_PREPARED
      TOPO_PORT_STATE_STARTED,   // TOPO_SG_STATE_STARTED
      TOPO_PORT_STATE_SUSPENDED  // TOPO_SG_STATE_SUSPENDED
   };

   if (sg_state > SIZE_OF_ARRAY(port_state_array))
   {
      return TOPO_PORT_STATE_INVALID;
   }

   return port_state_array[sg_state];
}

#define DEBUG_TOPO_PORT_PROP_TYPE

#ifdef DEBUG_TOPO_PORT_PROP_TYPE

#define TOPO_PORT_MAX_PROP_TYPE 3
#define TOPO_PORT_MAX_PROP_VALUES 4
#define TOPO_PORT_MAX_PROP_VALUE_STRING_LEN 10

extern char_t prop_type_to_string[TOPO_PORT_MAX_PROP_TYPE][TOPO_PORT_MAX_PROP_VALUE_STRING_LEN];
extern char_t prop_value_to_string[TOPO_PORT_MAX_PROP_TYPE][TOPO_PORT_MAX_PROP_VALUES][TOPO_PORT_MAX_PROP_VALUE_STRING_LEN];

static inline char_t* topo_prop_type_to_string(uint32_t prop_type)
{
   return prop_type_to_string[prop_type-1];
}

static inline char_t* topo_prop_value_to_string(uint32_t prop_type, uint32_t prop_value)
{
   return prop_value_to_string[prop_type-1][prop_value];
}

#endif // DEBUG_TOPO_PORT_PROP_TYPE

// clang-format on

#define TU_IS_ANY_DEINTERLEAVED_UNPACKED(interleaving)                                                                 \
   ((TOPO_DEINTERLEAVED_UNPACKED | TOPO_DEINTERLEAVED_UNPACKED_V2) & interleaving)

/** use this macro to check if a port's operating media format is PCM and deinterleaved unpacked V2.*/
#define TU_IS_PCM_DEINT_UNPACKED_V2(media_fmt_ptr)                                                                     \
   ((SPF_IS_PCM_DATA_FORMAT((media_fmt_ptr)->data_format) &&                                                           \
     (TOPO_DEINTERLEAVED_UNPACKED_V2 == (media_fmt_ptr)->pcm.interleaving)))

#define TU_IS_PCM_AND_ANY_DEINT_UNPACKED(media_fmt_ptr)                                                                \
   (SPF_IS_PCM_DATA_FORMAT((media_fmt_ptr)->data_format) &&                                                            \
    TU_IS_ANY_DEINTERLEAVED_UNPACKED((media_fmt_ptr)->pcm.interleaving))

#define TU_IS_PCM_DEINTERLEAVED_PACKED(media_fmt_ptr)                                                                  \
   ((SPF_IS_PCM_DATA_FORMAT((media_fmt_ptr)->data_format) &&                                                           \
     (TOPO_DEINTERLEAVED_PACKED == (media_fmt_ptr)->pcm.interleaving)))

static inline uint32_t topo_us_to_samples(uint64_t time_us, uint64_t sample_rate)
{
   return capi_cmn_us_to_samples(time_us, sample_rate);
}

static inline uint32_t topo_us_to_bytes_per_ch(uint64_t time_us, topo_media_fmt_t *med_fmt_ptr)
{
   return capi_cmn_us_to_bytes_per_ch(time_us, med_fmt_ptr->pcm.sample_rate, med_fmt_ptr->pcm.bits_per_sample);
}

static inline uint32_t topo_us_to_bytes(uint64_t time_us, topo_media_fmt_t *med_fmt_ptr)
{
   return capi_cmn_us_to_bytes(time_us,
                               med_fmt_ptr->pcm.sample_rate,
                               med_fmt_ptr->pcm.bits_per_sample,
                               med_fmt_ptr->pcm.num_channels);
}

static inline uint32_t topo_truncate_bytes_to_intgeral_samples_on_all_ch(uint32_t bytes, topo_media_fmt_t *med_fmt_ptr)
{
   uint32_t samples =
      capi_cmn_bytes_to_samples_per_ch(bytes, med_fmt_ptr->pcm.bits_per_sample, med_fmt_ptr->pcm.num_channels);
   uint32_t ret_bytes = samples * TOPO_BITS_TO_BYTES(med_fmt_ptr->pcm.bits_per_sample) * med_fmt_ptr->pcm.num_channels;

   return ret_bytes;
}

static inline uint32_t topo_bytes_to_samples(uint32_t bytes, topo_media_fmt_t *med_fmt_ptr)
{
   uint32_t samples = 0;
   if (med_fmt_ptr->pcm.bits_per_sample != 0)
   {
      samples = bytes / TOPO_BITS_TO_BYTES(med_fmt_ptr->pcm.bits_per_sample);
   }

   return samples;
}

static inline uint32_t topo_bytes_to_bytes_per_ch(uint32_t bytes_per_ch, topo_media_fmt_t *med_fmt_ptr)
{
   return (bytes_per_ch / med_fmt_ptr->pcm.num_channels);
}

static inline uint32_t topo_bytes_per_ch_to_bytes(uint32_t bytes_per_ch, topo_media_fmt_t *med_fmt_ptr)
{
   return (bytes_per_ch * med_fmt_ptr->pcm.num_channels);
}

static inline uint32_t topo_bytes_to_samples_per_ch(uint32_t bytes, topo_media_fmt_t *med_fmt_ptr)
{
   return capi_cmn_bytes_to_samples_per_ch(bytes, med_fmt_ptr->pcm.bits_per_sample, med_fmt_ptr->pcm.num_channels);
}

static inline uint32_t topo_samples_to_bytes(uint32_t samples, topo_media_fmt_t *med_fmt_ptr)
{
   return samples * med_fmt_ptr->pcm.num_channels * TOPO_BITS_TO_BYTES(med_fmt_ptr->pcm.bits_per_sample);
}

static inline uint32_t topo_samples_to_bytes_per_ch(uint32_t samples, topo_media_fmt_t *med_fmt_ptr)
{
   return samples * TOPO_BITS_TO_BYTES(med_fmt_ptr->pcm.bits_per_sample);
}

static inline uint32_t topo_bytes_per_ch_to_samples(uint32_t bytes_per_ch, topo_media_fmt_t *med_fmt_ptr)
{
   uint32_t samples;
   if (TOPO_IS_POW_OF_2(med_fmt_ptr->pcm.bits_per_sample))
   {
      // -3 for bits to bytes
      samples = s32_shr_s32_sat(bytes_per_ch, (s32_get_lsb_s32(med_fmt_ptr->pcm.bits_per_sample) - 3));
      return samples;
   }
   samples = bytes_per_ch / TOPO_BITS_TO_BYTES(med_fmt_ptr->pcm.bits_per_sample);
   return samples;
}

/**
 * Must be called only for PCM/packetized
 *
 * this function returns 0 if old OR new media fmt are zero.
 */
static inline uint32_t topo_rescale_byte_count_with_media_fmt(uint32_t          byte_count,
                                                              topo_media_fmt_t *new_med,
                                                              topo_media_fmt_t *old_med)
{
   if ((byte_count != 0) && (old_med->pcm.sample_rate != 0))
   {
      uint32_t old_samples = topo_bytes_to_samples_per_ch(byte_count, old_med);

      uint64_t p           = old_samples * (uint64_t)new_med->pcm.sample_rate;
      uint64_t q           = (uint64_t)old_med->pcm.sample_rate;
      uint32_t new_samples = (uint32_t)TOPO_CEIL(p, q);
      return (topo_samples_to_bytes_per_ch(new_samples, new_med) * new_med->pcm.num_channels);
   }

   return 0;
}

/**
   Gets kpps required to do a memscpy for data of the passed in media format.

   Regardless of frame size, in 1 sec we transfer SR*bytes*ch bytes using memcpy. memcpy usually copies 8-bytes at a
   time, where it does 1 8-byte read and 1 8-byte write = 2 instructions for every 8-byte copy.
   Therefore pps = 2*SR*bytes*ch/8 and kpps = pps/1000.
*/
static inline uint32_t topo_get_memscpy_kpps(uint32_t bits_per_sample, uint32_t num_channels, uint32_t sample_rate)
{
   return (TOPO_BITS_TO_BYTES(bits_per_sample) * num_channels * sample_rate) / 4000;
}

/**
  Determines the operating frame size in samples, based on the input sampling rate.
  The caller must validate the sample rate before calling this function.

  @param[in] sample_rate      Sampling rate, in Hz, for which the frame size in samples
                              must be calculated.

  @return
  Frame size in samples is returned in an acknowledgment.

  @dependencies
  None.
*/
static inline uint32_t tu_get_unit_frame_size(uint32_t sample_rate)
{
   // Returns 1 sample as a minimum value
   if (sample_rate < 1000)
   {
      return 1;
   }
   return (sample_rate / 1000);
}

static inline uint32_t tu_get_bits(uint32_t x, uint32_t mask, uint32_t shift)
{
   return (x & mask) >> shift;
}

static inline void tu_set_bits(uint32_t *x_ptr, uint32_t val, uint32_t mask, uint32_t shift)
{
   val    = (val << shift) & mask;
   *x_ptr = (*x_ptr & ~mask) | val;
}

/* Utility to convert Pnano seconds to Nano seconds

   pns = "P nano seconds" is fractional timestamp represented in terms of TIMESTAMP_NUM_FRACTIONAL_BITS bits.
   to covert to nano seconds ns = (timestamp_in_nps * 1000) >> TIMESTAMP_NUM_FRACTIONAL_BITS.
*/
static inline uint64_t tu_convert_pns_to_ns(uint64_t time_pns)
{
   return ((time_pns * 1000) >> TIMESTAMP_NUM_FRACTIONAL_BITS);
}

/* Utility to convert Pnano seconds to Micro seconds

   pns = "P nano seconds" is fractional timestamp represented in terms of TIMESTAMP_NUM_FRACTIONAL_BITS bits.
   to covert to nano seconds ns = (timestamp_in_nps * 1000) >> TIMESTAMP_NUM_FRACTIONAL_BITS.
*/
static inline uint64_t tu_convert_pns_to_us(uint64_t time_pns)
{
   return (time_pns >> TIMESTAMP_NUM_FRACTIONAL_BITS);
}

/* =======================================================================
Public Function Declarations
========================================================================== */

/**---------------------------------- tu.cpp -----------------------------------*/
topo_port_state_t tu_sg_op_to_port_state(uint32_t sg_op);
bool_t            tu_has_media_format_changed(topo_media_fmt_t *a1, topo_media_fmt_t *b1);
ar_result_t       tu_convert_media_fmt_spf_msg_to_topo(uint32_t                log_id,
                                                       spf_msg_media_format_t *spf_media_format_ptr,
                                                       topo_media_fmt_t *      topo_media_format_ptr,
                                                       POSAL_HEAP_ID           heap_id);
void              tu_copy_media_fmt(topo_media_fmt_t *dst_ptr, topo_media_fmt_t *src_ptr);
bool_t            topo_is_valid_media_fmt(topo_media_fmt_t *med_fmt_ptr);

ar_result_t tu_capi_create_raw_compr_med_fmt(uint32_t          log_id,
                                             uint8_t *         raw_fmt_ptr,
                                             uint32_t          raw_fmt_size,
                                             uint32_t          fmt_id,
                                             topo_media_fmt_t *media_fmt_ptr,
                                             bool_t            with_header,
                                             POSAL_HEAP_ID     heap_id);

ar_result_t tu_capi_destroy_raw_compr_med_fmt(topo_raw_med_fmt_t *raw_ptr);

void tu_release_media_fmt(topo_mf_utils_t *me_ptr, topo_media_fmt_t **mf_pptr);
ar_result_t tu_set_media_fmt_from_port(topo_mf_utils_t *  me_ptr,
                                       topo_media_fmt_t **dst_mf_pptr,
                                       topo_media_fmt_t * src_mf_ptr);
ar_result_t tu_set_media_fmt(topo_mf_utils_t *  me_ptr,
                             topo_media_fmt_t **dst_mf_pptr,
                             topo_media_fmt_t * src_mf_ptr,
                             POSAL_HEAP_ID      heap_id);

uint32_t tu_get_max_num_channels(topo_mf_utils_t *me_ptr);

void tu_destroy_mf(topo_mf_utils_t *me_ptr);

// don't call this function directly
topo_port_state_t tu_get_downgraded_state_(topo_port_state_t self_port_state, topo_port_state_t connected_port_state);

static inline topo_port_state_t tu_get_downgraded_state(topo_port_state_t self_port_state,
                                                        topo_port_state_t connected_port_state)
{
   if (self_port_state == connected_port_state)
   {
      return self_port_state;
   }
   return tu_get_downgraded_state_(self_port_state, connected_port_state);
}

static inline uint64_t topo_samples_to_us(uint32_t samples, uint32_t sample_rate, uint64_t *fract_time_pns_ptr)
{
   uint64_t time_us  = 0;
   uint64_t time_pns = 0;

   if (sample_rate != 0)
   {
      uint64_t total_byte_pns_per_s = (((uint64_t)samples * NUM_US_PER_SEC) << TIMESTAMP_NUM_FRACTIONAL_BITS);
      uint64_t sr                   = ((uint64_t)sample_rate);

      time_pns = (total_byte_pns_per_s / sr);

      if (fract_time_pns_ptr)
      {
         time_pns += *fract_time_pns_ptr;
         *fract_time_pns_ptr = time_pns & TIMESTAMP_FRACTIONAL_BIT_MASK;
      }
      time_us = tu_convert_pns_to_us(time_pns);
   }

   return time_us;
}

// this is a utility fn do not use it directly, use topo_bytes_to_us or topo_bytes_per_ch_to_us
uint64_t _topo_bytes_to_us_optimized_util(uint32_t          bytes,
                                          bool_t            bytes_passed_is_per_channel,
                                          topo_media_fmt_t *med_fmt_ptr,
                                          uint64_t *        fract_time_pns_ptr);

/**
 * includes previous fractional time, and calculates new fractional time in fract_time_pns_ptr
 * Unit of *fract_time_pns_ptr is "P nano seconds (pns)"
 *
 * Must be called only for PCM or packetized
 */
static inline uint64_t topo_samples_per_ch_to_us(uint32_t          samples_per_ch,
                                                 topo_media_fmt_t *med_fmt_ptr,
                                                 uint64_t *        fract_time_pns_ptr)
{
   if (!samples_per_ch)
   {
      return 0;
   }
   uint32_t bytes_per_ch = topo_samples_to_bytes_per_ch(samples_per_ch, med_fmt_ptr);
   return _topo_bytes_to_us_optimized_util(bytes_per_ch, TRUE, med_fmt_ptr, fract_time_pns_ptr);
}

/**
 * includes previous fractional time, and calculates new fractional time in fract_time_pns_ptr
 * Unit of *fract_time_pns_ptr is "P nano seconds (pns)"
 *
 * pns is fractional timestamp represented in terms of 10 bits.
 * to covert to nano seconds ns = (fract_timestamp_pns * 1000) >> 10.
 *
 * You must use this utility tu_convert_pns_to_ns() to convert to nano seconds
 * before doing any arthimetic op with it.
 *
 * Must be called only for PCM or packetized
 */
static inline uint64_t topo_bytes_to_us(uint32_t bytes, topo_media_fmt_t *med_fmt_ptr, uint64_t *fract_time_pns_ptr)
{
   if (!bytes)
   {
      return 0;
   }

   return _topo_bytes_to_us_optimized_util(bytes, FALSE, med_fmt_ptr, fract_time_pns_ptr);
}

/**
 * includes previous fractional time, and calculates new fractional time in fract_time_pns_ptr
 * Unit of *fract_time_pns_ptr is "P nano seconds (pns)"
 *
 * Must be called only for PCM or packetized
 */
static inline uint64_t topo_bytes_per_ch_to_us(uint32_t          bytes,
                                               topo_media_fmt_t *med_fmt_ptr,
                                               uint64_t *        fract_time_pns_ptr)
{
   if (!bytes)
   {
      return 0;
   }
   return _topo_bytes_to_us_optimized_util(bytes, TRUE, med_fmt_ptr, fract_time_pns_ptr);
}

/**
 * returns p/q where q has high chance of being power of 2
 */
static inline uint32_t topo_div_num(uint32_t num, uint32_t den)
{
   return capi_cmn_div_num(num, den);
}

/**
 * helps combine power of 2 check
 *
 * q stands for quotient
 */
static inline void topo_div_two_nums(uint32_t  num1,
                                     uint32_t *num1_q_ptr,
                                     uint32_t  num2,
                                     uint32_t *num2_q_ptr,
                                     uint32_t  den)
{
   if (TOPO_IS_POW_OF_2(den))
   {
      uint32_t shift = s32_get_lsb_s32(den);
      *num1_q_ptr    = s32_shr_s32_sat(num1, shift);
      *num2_q_ptr    = s32_shr_s32_sat(num2, shift);
      return;
   }
   *num1_q_ptr = num1 / den;
   *num2_q_ptr = num2 / den;
}

static inline void topo_div_three_nums(uint32_t  num1,
                                       uint32_t *num1_q_ptr,
                                       uint32_t  num2,
                                       uint32_t *num2_q_ptr,
                                       uint32_t  num3,
                                       uint32_t *num3_q_ptr,
                                       uint32_t  den)
{
   if (TOPO_IS_POW_OF_2(den))
   {
      uint32_t shift = s32_get_lsb_s32(den);
      *num1_q_ptr    = s32_shr_s32_sat(num1, shift);
      *num2_q_ptr    = s32_shr_s32_sat(num2, shift);
      *num3_q_ptr    = s32_shr_s32_sat(num3, shift);
      return;
   }
   *num1_q_ptr = num1 / den;
   *num2_q_ptr = num2 / den;
   *num3_q_ptr = num3 / den;
}

#if USES_DEBUG_DEV_ENV
//#define DEBUG_MEMCPY
#define MANDATORY_MEM_OP // when profiling without actual memcpy/mov, we might still need some memcpy/move -e.g. mp3 bit stream.
#endif

#ifdef DEBUG_MEMCPY
#define TOPO_MEMSCPY(ret, dst, dst_size, src, src_size, log_id, fmt, ...)                                              \
   TOPO_MSG(log_id,                                                                                                    \
            DBG_LOW_PRIO,                                                                                              \
            fmt ": MEMCPY (0x%p, %lu) -> (0x%p, %lu) ",                                                                \
            ##__VA_ARGS__,                                                                                             \
            src,                                                                                                       \
            src_size,                                                                                                  \
            dst,                                                                                                       \
            dst_size);                                                                                                 \
   ret = memscpy(dst, dst_size, src, src_size);

#define TOPO_MEMSMOV(ret, dst, dst_size, src, src_size, log_id, fmt, ...)                                              \
   TOPO_MSG(log_id,                                                                                                    \
            DBG_LOW_PRIO,                                                                                              \
            fmt ": MEMMOV (0x%p, %lu) -> (0x%p, %lu) ",                                                                \
            ##__VA_ARGS__,                                                                                             \
            src,                                                                                                       \
            src_size,                                                                                                  \
            dst,                                                                                                       \
            dst_size);                                                                                                 \
   ret = memsmove(dst, dst_size, src, src_size);

#else

#define TOPO_MEMSCPY(ret, dst, dst_size, src, src_size, fmt, ...) ret = memscpy(dst, dst_size, src, src_size);
#define TOPO_MEMSMOV(ret, dst, dst_size, src, src_size, fmt, ...) ret = memsmove(dst, dst_size, src, src_size);

//#define TOPO_MEMSCPY(ret, dst, dst_size, src, src_size, fmt, ...) ret = MIN(dst_size, src_size);
//#define TOPO_MEMSMOV(ret, dst, dst_size, src, src_size, fmt, ...) ret = MIN(dst_size, src_size);

#endif

// no return
#define TOPO_MEMSCPY_NO_RET(dst, dst_size, src, src_size, fmt, ...)                                                    \
   do                                                                                                                  \
   {                                                                                                                   \
      size_t temp;                                                                                                     \
      TOPO_MEMSCPY(temp, dst, dst_size, src, src_size, fmt, ##__VA_ARGS__);                                            \
      (void)temp;                                                                                                      \
   } while (0)

#define TOPO_MEMSMOV_NO_RET(dst, dst_size, src, src_size, fmt, ...)                                                    \
   do                                                                                                                  \
   {                                                                                                                   \
      size_t temp;                                                                                                     \
      TOPO_MEMSMOV(temp, dst, dst_size, src, src_size, fmt, ##__VA_ARGS__);                                            \
      (void)temp;                                                                                                      \
   } while (0)

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef TOPOLOGY_UTILS_
