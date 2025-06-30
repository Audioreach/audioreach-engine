/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef SHOEBOX_API_H
#define SHOEBOX_API_H

#include "ShoeBox_calibration_api.h"
#include "AudioComdef.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


/*----------------------------------------------------------------------------
   Error Code and Types
----------------------------------------------------------------------------*/
typedef enum SHOEBOX_RESULT {
   SHOEBOX_SUCCESS = 0,
   SHOEBOX_FAILURE,
   SHOEBOX_MEMERROR,
   SHOEBOX_UNSUPPORTED_PARAM,
   SHOEBOX_UNSUPPORTED_PARAM_VALUE,		//new return value added for the cases when input size/parameter value is out of algorithm supported boundary.  
										//Inside algorithm, the values will be bounded to the max that the algorithm supports
} SHOEBOX_RESULT;


/*
shoebox_struct_id_t 

shoebox_struct_id_t is Mandatory. Some of the struct_ids are mandatory as mentioned.

shoebox_struct_id_t enum typedef refers to structures that can be used to communicate with library functions.
Along with refering to structures in algo_api.h. This enum also refers to the calibration structures in algo_calibration_api.h

Note: enum numbers doesn't have to be sequential.
*/
typedef enum shoebox_struct_id_t {
	SHOEBOX_CONFIG = 0,				// mapped to struct: shoebox_config_t
	SHOEBOX_MEMORY,					// mapped to struct: shoebox_memory_config_t
	SHOEBOX_VERSION_PARAM,			// mapped to struct: shoebox_ver_param_t
	SHOEBOX_ENABLE_PARAM,			// mapped to struct: shoebox_enable_param_t
	SHOEBOX_WET_MIX_PARAM,			// mapped to struct: shoebox_wet_mix_param_t

	/* tuning parameters that are mapped to DayDream's */
	SHOEBOX_ROOM_SIZE_PARAM,		// mapped to struct: shoebox_room_size_param_t 
	SHOEBOX_ROOM_MATERIAL_PARAM,	// mapped to struct: shoebox_room_material_param_t 
	SHOEBOX_REVERB_GAIN_PARAM,		// mapped to struct: shoebox_reverb_gain_param_t
	SHOEBOX_DECAY_TIME_ADJUST_PARAM,// mapped to struct: shoebox_decay_time_adjust_param_t   
	SHOEBOX_BRIGHTNESS_ADJUST_PARAM,// mapped to struct: shoebox_brightness_adjust_param_t
		
	SHOEBOX_RESET_PARAM,			// mapped to struct: shoebox_reset_param_t
	SHOEBOX_ALG_DELAY_PARAM,		// mapped to struct: shoebox_alg_delay_param_t
	SHOEBOX_CROSSFADE_FLAG_PARAM	// mapped to struct: shoebox_crossflag_param_t
} shoebox_struct_id_t;


/*
shoebox_memory_t 

shoebox_memory_config_t is the library structure encapsulating memory details of the library. 
Memory for this structure has to be allocated by Caller before calling any of the library apis

Note: each structure definition should have a specific id in shoebox_struct_id_t (SHOEBOX_MEMORY in this case)

struct_size:              [IN]: Filled by the caller, shows the size of the structure
lib_static_mem_size:      [IN/OUT]: Size of library static memory required in bytes, this memory  
                          --[OUT]: Callee fills for api shoebox_get_req, --[IN]: Caller fills for api shoebox_init
lib_stack_size:           [OUT]: Size of the stack needed by library. [out]: for api shoebox_get_req, Don't care for api ashoebox_init

*/
typedef struct shoebox_memory_t {
	uint32_t	struct_size;
	uint32_t	lib_static_mem_size; 
	uint32_t	lib_scratch_mem_size;
	uint32_t	lib_stack_mem_size;
} shoebox_memory_t;


/*
shoebox_config_t 

shoebox__config_t is the library structure encapsulating library configurations 
which donot change with calibration or without memory reallocation

shoebox_config_t is a Mandatory Structure Definition. Members of this structure are not mandatory 
but recommended for every algorithm; however not limited by the following fields,i.e. algorithms can add new parameters as needed. 

Moreoever, more than one static config can be defined to differentate the evolution of parameters or fit the specific needs of the algorithm. 
Note: each structure definition should have a specific id in shoebox_struct_id_t

struct_size:[IN]: Mandatory : Filled by the caller, shows the size of the structure
sample_rate:[IN]: Input/outputs operating sampling rate. 
q_format:[IN]: Q-factor for input/output data:  15 for 16bit input and 27 for 32bit input
bytes_per_sample:[IN] : Number of bytes for input and output bytes. 16 (Q15 PCM) or 32 (Q27 PCM)
num_chs:[IN] : Number of input/output channels, 
max_frame_size:[IN]: : max frame size 
*/
typedef struct shoebox_config_t {		// ** static params
   uint32_t		struct_size;				
   uint32_t     sample_rate;			// supporting 8k/16k/32k/44.1k/48k/96k/192kHz 
   uint32_t     q_format;				// 15 (16bit path)  or 27 (32bit processing with 24bit path) supported currently 	
   uint32_t		bytes_per_sample;		// 2 or 4  (for 24bit path, shoebox always takes L32Q27 format input)
   uint32_t     num_chs;				// In first release v1, only stereo in/stereo out supported
   uint32_t     max_frame_size;		  		
} shoebox_config_t;


/*
shoebox_t 

shoebox_t is the library instance definition. 
shoebox_t is Mandatory Definition
*/
typedef int8_t    shoebox_t;


/*-----------------------------------------------------------------------------
   Function Declarations
----------------------------------------------------------------------------*/

/* shoebox_get_req
This api provides the memory and other requirements of the library based on the configuration provided

This function is expected to be called before shoebox_init is invoked.
req_config_id:  [IN] : Requirements Structure ID for querying memory requirements, defines the payload pointed by req_config_ptr
                Supports : SHOEBOX_MEMORY, (algorithms can add more shoebox_struct_id_t as the needs grow),
req_config_ptr: [OUT] : Requirements Structure pointer for engine to be filled by library     
config_id:		[IN] : Configuration Structure ID for querying engine requirements, defines the payload pointed by engine_config_ptr
                Currently only supports : SHOEBOX_CONFIG (static structure)
config_ptr:		[IN] : Configuration Structure pointer for engine to be used by library to calculate requirements
				For SHOEBOX_CONFIG, this pointer points to the structure containing the static structure for memory size estimate

Return:
result - SHOEBOX_RESULT 
*/
SHOEBOX_RESULT shoebox_get_req(
	uint32_t	req_config_id, 
	int8_t		*req_config_ptr, 
	uint32_t	config_id, 
	int8_t		*config_ptr
);


/* shoebox_init

shoebox_init signature of the structure is fixed as shown.
This api
- Allocates static memory only to library &
- partition the memory properly
- Initializes calibration to default values

This function is expected to be called before shoebox_process is invoked.
**shoebox_lib_ptr:		[OUT] : pointer to the algo public library structure
config_id:       [IN] : Configuration Structure ID for querying engine requirements, defines the payload pointed by engine_config_ptr
                        Supports : SHOEBOX_CONFIG
config_ptr:      [IN] : Configuration Structure pointer for engine to be used by library to calculate requirements
static_memory_ptr:  [IN] : Pointer to static memory for initializing the engine
static_memory_size: [IN] : size of the static memory pointer by pointer engine_static_memory_ptr

Return:
result - SHOEBOX_RESULT 
*/
SHOEBOX_RESULT shoebox_init(
	shoebox_t	**shoebox_lib_ptr,
	uint32_t    config_id,
	int8_t		*config_ptr,
	int8_t		*static_memory_ptr,
	uint32_t    static_memory_size
);


/*
shoebox_set
- Sets data to the library

shoebox_lib_ptr:	[IN] : pointer to the algo public library structure 
set_struct_id:		[IN] : this strucutre ID defines the structure in set_struct_ptr
set_struct_ptr:     [IN] : data pointer & struct_size of the memory or parameter buffer 

Return:
result - SHOEBOX_RESULT 
   */
SHOEBOX_RESULT shoebox_set(
	shoebox_t	*shoebox_lib_ptr, 
	uint32_t	set_struct_id, 
	int8_t		*set_struct_ptr
);


/*
shoebox_get
- This api gets data from library
- Returns data identified with get_data.struct_id which is filled by Callee
- Callee fills data at the address pointed by get_data.io_ptr
- Callee updates get_data.io_ptr.struct_size (note io_ptr has to be properly type casted for the struct_size)
example structures/data queried by Caller
  - Calibration data 
  - RTM
  - Algorithm Version
  
*shoebox_lib_ptr:	[IN] : pointer to the algo public library structure 
index:				[IN] : index value, ranges from 0 to max_inputs-1, defines the input from which the data has to be "get"
get_struct_id:      [IN] : this strucutre ID defines the structure in get_struct_ptr
get_struct_ptr:     [IN/OUT] : data pointer & struct_size of the parameter buffer 
					Caller fills:		[IN] : get_struct_ptr->struct_size i.e. memory in bytes available for callee to fill(note has to be properly type casted for the size)
					Callee fills:		[OUT] : get_struct_ptr->struct_size i.e. memory in bytes filled by callee and the rest of the fields of the structure as needed

Return:
result - SHOEBOX_RESULT 
*/
SHOEBOX_RESULT shoebox_get(
	shoebox_t	*shoebox_lib_ptr, 
	uint32_t	get_struct_id, 
	int8_t		*get_struct_ptr
);

/*
Reads input data and processes it according to the calibration data set using shoebox_set
This function can only be invoked after successful call to shoebox_init & after setting scratch memory as required

*shoebox_lib_ptr:		[IN] : pointer to the algo public library structure
**output_data_ptr:		[OUT] : This pointer points to the output data address that the library needs to fill. 
output_data_size:		[IN] : This parameter gives the per-channel output data frame's  size in terms of number of samples. Set by the caller.
**input_data_ptr:		[IN] : This pointer points to the actual input data address that the caller provides, assuming they are de-interleaved. 
input_data_size:		[IN] : This parameter gives the per-channel input data frame's size in terms of number of samples. Set by the caller.

Return:
result - SHOEBOX_RESULT 
*/
SHOEBOX_RESULT shoebox_process(
	shoebox_t	*shoebox_lib_ptr,
	int8_t		**output_data_ptr,	// always assuming de-interleaved output
	uint32_t	output_data_size,
	int8_t		**input_data_ptr,	// always assuming de-interleaved input
	uint32_t	input_data_size
);


int8_t* get_reverb_lib_ptr( shoebox_t* lib_ptr );


/*----------------------------------------------------------------------------
   Reverb Environment Enum
----------------------------------------------------------------------------
typedef enum environment_t {
   environment_Custom = 0,             // custom preset
   environment_Room = 1,               // 01
   environment_Bathroom,               // 02
   environment_ConcertHall,            // 03
   environment_Cave,                   // 04
   environment_Arena,                  // 05
   environment_Forest,                 // 06
   environment_City,                   // 07
   environment_Mountains,              // 08
   environment_Underwater,             // 09
   environment_Auditorium,             // 10
   environment_Alley,                  // 11
   environment_Hallway,                // 12
   environment_Hangar,                 // 13
   environment_Livingroom,             // 14
   environment_Smallroom,              // 15
   environment_Mediumroom,             // 16
   environment_Largeroom,              // 17
   environment_Mediumhall,             // 18
   environment_Largehall,              // 19
   environment_Plate,                  // 20
   environment_Generic,                // 21
   environment_PaddedCell,             // 22
   environment_Stoneroom,              // 23
   environment_CarpetedHallway,        // 24
   environment_StoneCorridor,          // 25
   environment_Quarry,                 // 26
   environment_Plain,                  // 27
   environment_ParkingLot,             // 28
   environment_SewerPipe,              // 29
   environment_Drugged,                // 30
   environment_Dizzy,                  // 31
   environment_Psychotic,              // 32
   environment_Count
} environment_t;
*/
/*----------------------------------------------------------------------------
   Parameters with IDs
----------------------------------------------------------------------------*/
#define REVERB_PARAM_LIB_VER           (0)   // ** param: library version
typedef int32 reverb_version_t;              //    access: get only

/*------------------ begin OpenSL ES compatible parameters ------------------*/
// Description: The master volume level of the environment reverb effect
#define REVERB_PARAM_ROOM_LEVEL        (1)   // ** param: room level
typedef int32 reverb_room_level_t;           //    access: get & set
                                             //    range: [-9600, 0] (mb)

// Description: The volume level at 5kHz relative to the volume level at low
// frequencies of the overall reverb effect
#define REVERB_PARAM_ROOM_HF_LEVEL     (2)   // ** param: room HF level
typedef int32 reverb_room_hf_level_t;        //    access: get & set
                                             //    range: [-9600, 0] (mb)

// Description: The time taken for the level of reverberation to decay by 60 dB
#define REVERB_PARAM_DECAY_TIME        (3)   // ** param: decay time RT60
typedef int32 reverb_decay_time_t;           //    access: get & set
                                             //    range: [100, 20000] (ms)

// Description: The ratio of high frequency decay time (at 5kHz) relative to
// the decay time at low frequencies
#define REVERB_PARAM_DECAY_HF_RATIO    (4)   // ** param: decay HF ratio
typedef int32 reverb_decay_hf_ratio_t;       //    access: get & set
                                             //    range: [100, 2000] (per millie)

// Description: The volume level of the early reflections
#define REVERB_PARAM_REFLECTIONS_LEVEL (5)   // ** param: reflections level
typedef int32 reverb_reflections_level_t;    //    access: get & set
                                             //    range: [-9600, 1000] (mb)

// Description: The delay time for the early reflection (relative to direct sound)
#define REVERB_PARAM_REFLECTIONS_DELAY (6)   // ** param: reflections delay
typedef int32 reverb_reflections_delay_t;    //    access: get & set
                                             //    range: [0, 300] (ms)

// Description: The volume level of the late reverberation
#define REVERB_PARAM_REVERB_LEVEL      (7)   // ** param: reverb level
typedef int32 reverb_reverb_level_t;         //    access: get & set
                                             //    range: [-9600, 2000] (mb)

// Description: The time between the first reflection and the late reverberation
#define REVERB_PARAM_REVERB_DELAY      (8)   // ** param: reverb delay
typedef int32 reverb_reverb_delay_t;         //    access: get & set
                                             //    range: [0, 100] (ms)

// Description: The echo density in the late reverberation decay
#define REVERB_PARAM_DIFFUSION         (9)   // ** param: diffusion
typedef int32 reverb_diffusion_t;            //    access: get & set
                                             //    range: [0, 1000] (per millie)

// Description: The modal density of the late reverberation decay
#define REVERB_PARAM_DENSITY           (10)  // ** param: density
typedef int32 reverb_density_t;              //    access: get & set
                                             //    range: [0, 1000] (per millie)

/*------------------- end OpenSL ES compatible parameters -------------------*/

#define REVERB_PARAM_ENABLE            (11)  // ** param: enable switch
typedef int32 reverb_enable_t;               //    access: get & set
                                             //    range: [1:on or 0:off]

#define REVERB_PARAM_MODE              (12)  // ** param: topology mode
// typedef enum reverb_mode_t;               //    access: get only

#define REVERB_PARAM_PRESET            (13)  // ** param: reverb preset
typedef int32 reverb_preset_t;               //    access: get & set
                                             //    range: [0, 32]

// Description: reverb dry/wet mix for insert mode only. no effect on aux mode
#define REVERB_PARAM_WET_MIX           (14)  // ** param: wet mix ratio
typedef int32 reverb_wet_mix_t;              //    access: get & set
                                             //    range: [0, 1000] (per millie)

#define REVERB_PARAM_GAIN_ADJUST       (15)  // ** param: overall gain adjustment
typedef int32 reverb_gain_adjust_t;          //    access: get & set
// (may cause saturation)                    //    range : [-600, 600] (mB)

#define REVERB_PARAM_RESET             (16)  // ** param: reset
                                             //    access: set only

#define REVERB_PARAM_ALG_DELAY         (17)  // ** param: algorithm delay
typedef int32 reverb_alg_delay_t;            //    access: get only

#define REVERB_PARAM_CROSSFADE_FLAG    (18)  // ** param: transition/crossfade flag
typedef int32 reverb_crossfade_t;            //    access: get only
                                             //    range: [0(no), 1(yes)]




#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* SHOEBOX_API_H */
