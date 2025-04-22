/*=========================================================================
Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
SPDX-License-Identifier: BSD-3-Clause-Clear
========================================================================= */
#ifndef __IIR_RESAMPER_H__
#define __IIR_RESAMPER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "AudioComdef.h"

#if defined(__XTENSA__)
#include "NatureDSP_types_hifi5.h"
#include "NatureDSP_Signal_hifi5.h"
#endif

/* KPPS numbers for various modes. These are required for evaluating the offsets in various modules */
//DC: TBD KPI numbers to be added
#define IIR_RESAMPER_KPPS_8K_TO_16K  (329 )
#define IIR_RESAMPER_KPPS_8K_TO_24K  (329 )
#define IIR_RESAMPER_KPPS_8K_TO_32K  (748 )
#define IIR_RESAMPER_KPPS_8K_TO_48K  (945 )
#define IIR_RESAMPER_KPPS_8K_TO_96K  (945 )
#define IIR_RESAMPER_KPPS_8K_TO_192K (945 )
#define IIR_RESAMPER_KPPS_8K_TO_384K (945 )
#define IIR_RESAMPER_KPPS_8K_MAX     (945 )

#define IIR_RESAMPER_KPPS_16K_TO_8K   (325 )
#define IIR_RESAMPER_KPPS_16K_TO_24K  (325 )
#define IIR_RESAMPER_KPPS_16K_TO_32K  (756 )
#define IIR_RESAMPER_KPPS_16K_TO_48K  (1074)
#define IIR_RESAMPER_KPPS_16K_TO_96K  (1074)
#define IIR_RESAMPER_KPPS_16K_TO_192K (1074)
#define IIR_RESAMPER_KPPS_16K_TO_384K (1074)
#define IIR_RESAMPER_KPPS_16K_MAX     (1074)

#define IIR_RESAMPER_KPPS_24K_TO_8K   (325 )
#define IIR_RESAMPER_KPPS_24K_TO_16K  (325 )
#define IIR_RESAMPER_KPPS_24K_TO_32K  (756 )
#define IIR_RESAMPER_KPPS_24K_TO_48K  (1074)
#define IIR_RESAMPER_KPPS_24K_TO_96K  (1074)
#define IIR_RESAMPER_KPPS_24K_TO_192K (1074)
#define IIR_RESAMPER_KPPS_24K_TO_384K (1074)
#define IIR_RESAMPER_KPPS_24K_MAX     (1074)


#define IIR_RESAMPER_KPPS_32K_TO_8K   (736 )
#define IIR_RESAMPER_KPPS_32K_TO_16K  (748 )
#define IIR_RESAMPER_KPPS_32K_TO_24K  (748 )
#define IIR_RESAMPER_KPPS_32K_TO_48K  (2445)
#define IIR_RESAMPER_KPPS_32K_TO_96K  (2445)
#define IIR_RESAMPER_KPPS_32K_TO_192K (2445)
#define IIR_RESAMPER_KPPS_32K_TO_384K (2445)
#define IIR_RESAMPER_KPPS_32K_MAX     (2445)

#define IIR_RESAMPER_KPPS_48K_TO_8K   (927 )
#define IIR_RESAMPER_KPPS_48K_TO_16K  (1058)
#define IIR_RESAMPER_KPPS_48K_TO_24K  (1058)
#define IIR_RESAMPER_KPPS_48K_TO_32K  (2437)
#define IIR_RESAMPER_KPPS_48K_TO_96K  (2437)
#define IIR_RESAMPER_KPPS_48K_TO_192K (2437)
#define IIR_RESAMPER_KPPS_48K_TO_384K (2437)
#define IIR_RESAMPER_KPPS_48K_MAX     (2437)

#define IIR_RESAMPER_KPPS_96K_TO_8K   (927 )
#define IIR_RESAMPER_KPPS_96K_TO_16K  (1058)
#define IIR_RESAMPER_KPPS_96K_TO_24K  (1058)
#define IIR_RESAMPER_KPPS_96K_TO_32K  (2437)
#define IIR_RESAMPER_KPPS_96K_TO_48K  (2437)
#define IIR_RESAMPER_KPPS_96K_TO_192K (2437)
#define IIR_RESAMPER_KPPS_96K_TO_384K (2437)
#define IIR_RESAMPER_KPPS_96K_MAX     (2437)

#define IIR_RESAMPER_KPPS_192K_TO_8K   (927 )
#define IIR_RESAMPER_KPPS_192K_TO_16K  (1058)
#define IIR_RESAMPER_KPPS_192K_TO_24K  (1058)
#define IIR_RESAMPER_KPPS_192K_TO_32K  (2437)
#define IIR_RESAMPER_KPPS_192K_TO_48K  (2437)
#define IIR_RESAMPER_KPPS_192K_TO_96K  (2437)
#define IIR_RESAMPER_KPPS_192K_TO_384K (2437)
#define IIR_RESAMPER_KPPS_192K_MAX     (2437)

#define IIR_RESAMPER_KPPS_384K_TO_8K   (927 )
#define IIR_RESAMPER_KPPS_384K_TO_16K  (1058)
#define IIR_RESAMPER_KPPS_384K_TO_24K  (1058)
#define IIR_RESAMPER_KPPS_384K_TO_32K  (2437)
#define IIR_RESAMPER_KPPS_384K_TO_48K  (2437)
#define IIR_RESAMPER_KPPS_384K_TO_96K  (2437)
#define IIR_RESAMPER_KPPS_384K_TO_192K (2437)
#define IIR_RESAMPER_KPPS_384K_MAX     (2437)

#define RS_MAX_STAGES (10)

#ifdef PROD_SPECIFIC_MAX_CH
#define MAX_CHANNELS PROD_SPECIFIC_MAX_CH
#else
#define MAX_CHANNELS 32
#endif

#define RS_PRECISION_COEFF 16
#define IIR_RESAMPLER_MAX_STACK_SIZE 8192

#define IIR_RESAMPLER_RELEASE_VERSION_MSB  (0x02000101)
#define IIR_RESAMPLER_RELEASE_VERSION_LSB  (0x00000000)

#define IIR_RESAMPLER_IO_CONFIG              (1)      // refers to iir_resampler_io_config_t in iir_resampler_api.h
#define IIR_RESAMPLER_MEM_CONFIG             (2)      // refers to iir_resampler_memory_config_t in iir_resampler_api.h
#define IIR_RESAMPLER_BUF_PTR_ARRAY_PTR      (3)      // refers to pointer to input/output data buffer pointer array
#define PARAM_ID_IIR_RESAMPLER_DELAY         (4)      // refers to iir_resampler_delay_config_t


#define IIR_RESAMPLER_MAX_INPUT_CHANNELS   (1)      // maximum input TX channels
#define IIR_RESAMPLER_MAX_OUTPUT_CHANNELS  (1)      // maximum output channels
#define IIR_RESAMPLER_MIN_INPUT_CHANNELS   (1)      // minimum input TX channels
#define IIR_RESAMPLER_MIN_OUTPUT_CHANNELS  (1)      // minimum output channels

#ifdef PROD_SPECIFIC_MAX_CH
#define IIR_RESAMPLER_MAX_NUM_CHAN PROD_SPECIFIC_MAX_CH
#else
#define IIR_RESAMPLER_MAX_NUM_CHAN 32
#endif

#define IIR_RESAMPLER_SUCCESS              (0)     // successful result
#define IIR_RESAMPLER_FAIL                 (1)     // failed result

typedef uint32_t IIR_RESAMPLER_STRUCT_IDS;
typedef  uint32_t IIR_RESAMPLER_RESULT;


typedef struct iir_resampler_config_struct_t {
   uint32 input_sampling_rate;
   uint32 output_sampling_rate;
   uint32 input_dynamic_range;
   uint32 input_frame_samples;
   uint32 output_frame_samples;
   uint32 filter_sampling_rate;
   uint32 filter_cut_off_freq;
   int32 up_sample_flag;
   int32 dn_sample_flag;
   uint32 up_sample_factor;
   uint32 dn_sample_factor;
   uint32 stages;
#if (RS_PRECISION_COEFF==32)
   const int32 *f_x[RS_MAX_STAGES];
   const int32 *f_y[RS_MAX_STAGES];
#else
   const int16 *f_x[RS_MAX_STAGES];
   const int16 *f_y[RS_MAX_STAGES];
#endif
   int32 gain;
   int32 q_in;
   int32 q_out;
   int32 q_coeff;
   int32 q_g;
   int32 q_x;
   int32 q_y;
   int32 q_accu;
   uint32 single_channel_mem_size;    //Memoryfor single channel
   uint32 num_channels;
   uint32 filter_scratch_size;       // used for determining the scratch buffer size
   uint32 filter_frame_samples;      // number of filtering samples per frame
   uint32 group_delay_samples_x1000;
   int32 reserved;                  // used for alignment
} iir_resampler_config_struct_t;

typedef struct iir_resampler_channel_memory_struct_t {
   int32 *x_delay_ptr[RS_MAX_STAGES];
   int32 *y_delay_ptr[RS_MAX_STAGES];
   int32* filter_scratch_buf_ptr;   // used for intermediate filtering stage data
   int32 reserved;                  // used for alignment
   /* Memory for the above pointers*/
} iir_resampler_channel_memory_struct_t;

#if defined(__XTENSA__)
typedef struct biquad_t{
   bqriir32x16_df1_handle_t bq_handle;
   void *scratchmem;
   int32 *y_32_out;
   void *objmem;
}biquad_t;

typedef struct iir_resampler_lib_t {
   iir_resampler_config_struct_t iir_resampler_config_structure;
   iir_resampler_channel_memory_struct_t *iir_resampler_channel_memory_ptr[MAX_CHANNELS];
   biquad_t biquad_instance;
}iir_resampler_lib_t;
#else
typedef struct iir_resampler_lib_t {
   iir_resampler_config_struct_t iir_resampler_config_structure;
   iir_resampler_channel_memory_struct_t *iir_resampler_channel_memory_ptr[MAX_CHANNELS];
}iir_resampler_lib_t;
#endif

/* Supported sample rates
*/
typedef enum IIR_RESAMPLER_SAMPLE_RATE
{
    EIGHT_K = 8000,                // sample rate: 8k NB
    SIXTEEN_K = 16000,             // sample rate: 16k WB
    TWENTY_FOUR_K = 24000,         // sample rate: 24k
    THIRTY_TWO_K = 32000,          // sample rate: 32k SWB
    FOURTY_EIGHT_K = 48000,        // sample rate: 48k FB
    NINETY_SIX_K = 96000,          // sample rate: 96k
    ONE_NINETY_TWO_K = 192000,     // sample rate: 192k
    THREE_EIGHTY_FOUR_K = 384000   // sample rate: 384k
}IIR_RESAMPLER_SAMPLE_RATE;
//Make sure that variables are of size greater than or equal to uint_32


/************************************ Structures ****************************/
/*
iir_resampler_memory_config_t

iir_resampler_memory_config_t is associated with IIR_RESAMPLER_MEM_CONFIG id.
This is the library structure encapsulating memory details of the library
required for creating the instance. Caller allocates this structure for
callee to fill.

Supported for iir_resampler_get_req(...) function

lib_instance_mem_size:   [OUT]: callee fills size of library instance memory required in bytes
lib_stack_mem_size:      [OUT]: callee fills size of the stack memory required by library
*/
typedef struct iir_resampler_memory_config_t {
    uint32_t lib_instance_mem_size;
    uint32_t lib_stack_mem_size;
    uint32_t num_in_samples;
    uint32_t num_out_samples;
} iir_resampler_memory_config_t;



/*
iir_resampler_io_config_t

iir_resampler_io_config_t is the library structure encapsulating library I/O configuration

in_channels:        [IN]: input tx channels
out_channels:       [IN]: output channels
in_sample_rate:     [IN]: input sampling frequency
out_sample_rate:    [IN]: output sampling frequency
frame_length_ms:    [IN]: frame length (ms)

*/
typedef struct iir_resampler_io_config_t {
    uint32_t in_channels;
    uint32_t out_channels;
    uint32_t in_sample_rate;
    uint32_t out_sample_rate;
    uint32_t frame_length_ms;
    uint32_t bytes_per_sample;
} iir_resampler_io_config_t;


/*
iir_resampler_delay_config_t

iir_resampler_delay_config_t is the library structure encapsulating group_delay_samples

group_delay_samples:  [OUT]: number of samples by which the group is delayed by

*/
typedef struct iir_resampler_delay_config_t {
    uint32 group_delay_samples_x1000;
} iir_resampler_delay_config_t;


/*
iir_resampler_t

iir_resampler_t is the library instance definition.

IN: Memory for iir_resampler_t is used in iir_resampler_set(...),iir_resampler_get(...),iir_resampler_process(...)
OUT: Library provides pointer to iir_resampler_t in iir_resampler_init(...)
*/
typedef int8_t iir_resampler_t;

/************************************ Functions *****************************/

/*
iir_resampler_get_req

iir_resampler_get_req, signature of the structure is fixed as shown. This api provides
the memory and other requirements (if any) of the library based on the
configuration provided.

This function is expected to be called before init is invoked.

req_config_id:   [IN]: configuration structure ID for querying requirements, defines the payload pointed by iir_resampler_instance_memory_config_t
                   Supports: IIR_RESAMPLER_INSTANCE_MEM

req_config_ptr:  [IN/OUT]: requirements structure pointer in which the memory requirement that is calculated is updated and returned

req_config_size: [IN]: size of structure pointed by req_config_ptr in which the requirements are updated

config_id:       [IN]: requirements structure ID for querying requirements, defines the payload pointed by config_ptr
                           Supports: IIR_RESAMPLER_IO_CONFIG

config_ptr:      [IN]: req configuration structure pointer for iir_resampler to be used by library to calculate the requirement

config_size:     [IN]: size of structure pointed by config_ptr which has the configuration details for which the memory is calculated

Return:
result - IIR_RESAMPLER_RESULT
*/
IIR_RESAMPLER_RESULT iir_resampler_get_req(
    uint32_t req_config_id,
    int8_t* req_config_ptr,
    uint32_t req_config_size,
    uint32_t config_id,
    int8_t* config_ptr,
    uint32_t config_size
);


/*
iir_resampler_init

iir_resampler_init signature of the structure is fixed as shown.
This api
- Allocates static memory only to library instance
- Outputs iir_resampler's instance pointer iir_resampler_lib_ptr

This function is expected to be called before iir_resampler_process is invoked.

iir_resampler_lib_ptr :    [OUT]: pointer to the pointer pointing to the iir_resampler library instance
req_config_id  :            [IN]: structure ID that defines the payload pointed by req_config_ptr
req_config_ptr :            [IN]: pointer to static config for initializing the iir_resampler
req_config_size:            [IN]: size of the structure pointed by config_ptr
static_mem_ptr :            [IN]: pointer to the iir_resampler instance memory
static_mem_size:            [IN]: size of instance memory that has been allocated for the iir_resampler

Return:
result - IIR_RESAMPLER_RESULT
*/
IIR_RESAMPLER_RESULT iir_resampler_init(
    iir_resampler_t** iir_resampler_lib_ptr,
    uint32_t req_config_id,
    int8_t* req_config_ptr,
    uint32_t req_config_size,
    int8_t* static_mem_ptr,
    uint32_t static_mem_size
);

/*
iir_resampler_process

iir_resampler_lib_ptr:     [IN]: pointer to the iir_resampler public library structure
output_data_ptr:          [OUT]: caller provides the address of output_data that needs to be filled by the library(callee)
input_data_ptr:            [IN]: caller provides the address of input_data that contains the data of input channels
output_data_size:          [IN]: output data structure size
input_data_size:           [IN]: input data structure size
output_data_id:            [IN]: structure ID associated with the output data
                                 Supports: IIR_RESAMPLER_BUF_PTR
input_data_id:             [IN]: structure ID associated with the input data
                                 Supports: IIR_RESAMPLER_BUF_PTR
Return:
result - IIR_RESAMPLER_RESULT
*/
IIR_RESAMPLER_RESULT iir_resampler_process(
    iir_resampler_t* iir_resampler_lib_ptr,
    int8_t** output_data_ptr,
    int8_t** input_data_ptr,
    uint32_t output_data_size,
    uint32_t input_data_size,
    uint32_t output_data_id,
    uint32_t input_data_id
);

/*
iir_resampler_set

- This api sets data to the library

iir_resampler_lib_ptr:   [IN]: pointer to the iir_resampler public library structure
set_struct_id:           [IN]: this structure ID defines the structure in set_struct_ptr
set_struct_ptr:          [IN]: data pointer of the parameter buffer that is to be set to internal library
set_struct_size          [IN]: caller fills; size of the structure pointed by set_struct_ptr; It also indicates memory in bytes that will be filled by the caller
Return:
result - IIR_RESAMPLER_RESULT
*/
IIR_RESAMPLER_RESULT iir_resampler_set(
    iir_resampler_t* iir_resampler_lib_ptr,
    IIR_RESAMPLER_STRUCT_IDS set_struct_id,
    int8_t* set_struct_ptr,
    uint32_t set_struct_size
);

/*
iir_resampler_get

- This api gets data from library
- Returns data identified with get_struct_id which is filled by caller
- Callee fills data at the address pointed by get_struct_ptr
- Callee updates get_struct_filled_size

iir_resampler_lib_ptr:  [IN]: pointer to the iir_resampler public library structure
get_struct_id:          [IN]: this structre ID defines the structure in get_struct_ptr
                             Supports: PARAM_ID_IIR_RESAMPELR_VERSION
                                       PARAM_ID_IIR_RESAMPLER_DELAY
get_struct_ptr:         [IN/OUT]: data pointer of the parameter buffer in which the values read from internal library will be stored
get_struct_size:        [IN]: caller fills; get_struct_size i.e. memory in bytes available for callee to fill(note has to be properly type casted for the size)
*get_struct_filled_size:[IN/OUT]: Caller passes address; callee fills with memory in bytes read from internal library

Return:
result - IIR_RESAMPLER_RESULT
*/
IIR_RESAMPLER_RESULT iir_resampler_get(
    iir_resampler_t* iir_resampler_lib_ptr,
    IIR_RESAMPLER_STRUCT_IDS get_struct_id,
    int8_t* get_struct_ptr,
    uint32_t get_struct_size,
    uint32_t* get_struct_filled_size
);

/*
iir_resampler_get_upsample_factor
-this api is used to get up sample factor from lib which is used to calculate bandwidth.
iir_resampler_lib_ptr:  [IN]: pointer to the iir_resampler public library structure
up_sample_factor: [OUT]: uint32_t variable returns up_sample_factor
*/
uint32_t iir_resampler_get_upsample_factor(iir_resampler_t* iir_resampler_lib_ptr);
#ifdef __cplusplus
}
#endif

#endif//#ifndef __IIR_RESAMPER_H__
