/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef SHOEBOX_CALIBRATION_API_H
#define SHOEBOX_CALIBRATION_API_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/*----------------------------------------------------------------------------
   Reverb Environment Enum
----------------------------------------------------------------------------*/
typedef enum room_material_type {  
	/* Q3D_AUDIO_MATERIAL list starts */
	TRANSPARENT = 0,
	ACOUSTIC_CEILING_TILES,
	BRICK_BARE,
	BRICK_PAINTED,
	CONCRETE_BLOCK_COARSE,
	CONCRETE_BLOCK_PAINTED,
	CURTAIN_HEAVY,
	FIBER_GLASS_INSULATION,
	GLASS_THICK,
	GLASS_THIN,
	GRASS,
	LINOLEUM_ON_CONCRETE,
	MARBLE,
	METAL,
	PARQUET_ON_CONCRETE,
	PLASTER_ROUGH,
	PLASTER_SMOOTH,
	PLYWOOD_PANEL,
	POLISHED_CONCRETE_OR_TILE,
	SHEET_ROCK,
	WATER_OR_ICE_SURFACE,
	WOOD_ON_CEILING,
	WOOD_PANEL,
	/* Q3D_AUDIO_MATERIAL list ends */

	MATERIAL_RESERVE1,
	MATERIAL_RESERVE2,	
} room_material_type;



/*----------------------------------------------------------------------------
   Parameters with IDs
----------------------------------------------------------------------------*/
/* shoebox_ver_param_t 

This structure defines the parameters associated with SHOEBOX_VER_PARAM in enum algo_struct_id_t, which is in algo_api.h

This is a Mandatory Param Structure 

Supported only for get_param(...) api

struct_size: [IN/OUT]: --IN : Caller fills size  for the size of the payload structure, expects size >= sizeof(shoebox_ver_param_t)
					   --OUT: Callee fills the appropriate size of the buffer filled which is sizeof(shoebox_ver_param_t)
ver:		 [OUT]  returned version number
*/
typedef struct shoebox_ver_param_t {
	uint32_t struct_size;
	uint32_t reserved;
	int64_t  ver;
} shoebox_ver_param_t;                         


/* shoebox_enable_param_t 

This structure defines the parameters associated with SHOEBOX_ENABLE_PARAM in enum algo_struct_id_t, which is in algo_api.h

This is a Mandatory Param Structure 

Supported only for get/set_param(...) api

struct_size: [IN/OUT]: --IN : Caller fills size  for the size of the payload structure, expects size >= sizeof(shoebox_enable_param_t)
					   --OUT: Callee fills the appropriate size of the buffer filled which is sizeof(shoebox_enable_param_t)
enable:		 [IN/OUT]  enable or disable status,  range: [1:on or 0:off]
			 range: [0(off),  1(on) ];
			 default value:  0 (off)
*/
typedef struct shoebox_enable_param_t {               
	uint32_t struct_size;
	uint32_t  enable;
} shoebox_enable_param_t;                                             


/* shoebox_wet_mix_param_t

This structure defines the mix ratio between dry and wet sound, which is associated with SHOEBOX_WET_MIX_PARAM defined in shoebox_api.h

Supported for set/get_param(...) api

struct_size: [IN/OUT]: --IN : Caller fills size for the size of the payload structure in apis set_param(...) & get_param(...). expects size >= sizeof(shoebox_wet_mix_param_t)
                       --OUT: Callee fills the appropriate size of the buffer filled in api get_param(...),sizeof(shoebox_wet_mix_param_t)
wet_mix:     [IN/OUT]: overall gain adjustment, range: [0, 1000] per millie
*/
typedef struct  shoebox_wet_mix_param_t			  
{
	uint32_t struct_size;
	int32_t wet_mix;		
} shoebox_wet_mix_param_t;


/* shoebox_room_size_param_t

This structure defines the room dimension parameters, which is associated with SHOEBOX_ROOM_SIZE_PARAM defined in shoebox_api.h

Supported for set/get_param(...) api

struct_size: [IN/OUT]: --IN : Caller fills size for the size of the payload structure in apis set_param(...) & get_param(...). expects size >= sizeof(shoebox_room_size_param_t)
                       --OUT: Callee fills the appropriate size of the buffer filled in api get_param(...),sizeof(shoebox_room_size_param_t)
length:		 [IN/OUT]: stores the length of the room, in the unit of meter in uint32Q16 format, 
			 range: [1 2^32-1]  (equivalent range in meter is [2^-16 m, 65535m])
			 default value:  524288 (equivalent value in meter is 8m)
width:		 [IN/OUT]: stores the width of the room, in the unit of meter in uint32Q16 format, 
			 range: [1 2^32-1]  (equivalent range in meter is [2^-16 m, 65535m])
			 default value:  327680 (equivalent value in meter is 5m)
height:		 [IN/OUT]: stores the height of the room, in the unit of meter in uint32Q16 format, 
			 range: [1 2^32-1]  (equivalent range in meter is [2^-16 m, 65535m])
			 default value:  196608 (equivalent value in meter is 3m)
*/
typedef struct shoebox_room_size_param_t
{
	uint32_t  struct_size;
	uint32_t length;						
	uint32_t width;					
	uint32_t height;						
  
} shoebox_room_size_param_t;					


/* shoebox_room_material_param_t

This structure defines the room walls material parameters, which is associated with SHOEBOX_ROOM_MATERIAL_PARAM defined in shoebox_api.h

Supported for set/get_param(...) api

struct_size: [IN/OUT]: --IN : Caller fills size for the size of the payload structure in apis set_param(...) & get_param(...). expects size >= sizeof(shoebox_room_material_param_t)
                       --OUT: Callee fills the appropriate size of the buffer filled in api get_param(...),sizeof(shoebox_room_material_param_t)
front_wall:		 [IN/OUT]: stores the material of front wall (defined in enum room_material_type)
				 range: [0 25]
				 default value: 16  (PLASTER_SMOOTH)
back_wall:		 [IN/OUT]: stores the material of back wall  (defined in enum room_material_type)
				 range: [0 25]
				 default value: 16  (PLASTER_SMOOTH)
left_wall:		 [IN/OUT]: stores the material of left wall  (defined in enum room_material_type)	
				 range: [0 25]
				 default value: 16  (PLASTER_SMOOTH)
right_wall:		 [IN/OUT]: stores the material of right wall (defined in enum room_material_type)
				 range: [0 25]
				 default value: 16  (PLASTER_SMOOTH)
ceiling:		 [IN/OUT]: stores the material of ceiling    (defined in enum room_material_type)
				 range: [0 25]
				 default value: 15  (PLASTER_ROUGH)
floor:		     [IN/OUT]: stores the material of floor wall (defined in enum room_material_type)	
				 range: [0 25]
				 default value: 14  (PARQUET_ON_CONCRETE)
*/
typedef struct shoebox_room_material_param_t
{
	uint32_t struct_size;
	int32_t	front_wall;			
	int32_t	back_wall;			
	int32_t	left_wall;			
	int32_t	right_wall;			
	int32_t	ceiling;			
	int32_t	floor;				
  
} shoebox_room_material_param_t;					


/* shoebox_reverb_gain_param_t

This structure defines the adjustment gain to environment reverb effect, which is associated with SHOEBOX_REVERB_GAIN_PARAM defined in shoebox_api.h
This is suppose to be mapped to DayDream's reverb gain parameter.

Supported for set/get_param(...) api

struct_size: [IN/OUT]: --IN : Caller fills size for the size of the payload structure in apis set_param(...) & get_param(...). expects size >= sizeof(shoebox_reverb_gain_param_t)
                       --OUT: Callee fills the appropriate size of the buffer filled in api get_param(...),sizeof(shoebox_reverb_gain_param_t)
room_level:  [IN/OUT]: gain adjustment to the environment reverb effect,   format in uint32Q16 
			 range: [0, 2^32-1], (equivalent floating point range is: 0.0 to (2^16-2^-16) )
			 default value:  20724 (equivaluent to -10dB)
*/
typedef struct shoebox_reverb_gain_param_t           
{                                              
	uint32_t struct_size;
	uint32_t reverb_gain;
} shoebox_reverb_gain_param_t;


/* shoebox_decay_time_adjust_param_t

This structure defines the adjust factor for decay time RT60 , which is associated with SHOEBOX_DECAY_TIME_ADJUST_PARAM defined in shoebox_api.h
This is supposed to be mapped to DayDream's time adjust

Supported for set/get_param(...) api

struct_size: [IN/OUT]: --IN : Caller fills size for the size of the payload structure in apis set_param(...) & get_param(...). expects size >= sizeof(shoebox_decay_time_adjust_param_t)
                       --OUT: Callee fills the appropriate size of the buffer filled in api get_param(...),sizeof(shoebox_decay_time_adjust_param_t)
decay_time_adjust:  [IN/OUT]: adjustment to RT60 time,  format in uint32Q16 
					range: [0, 2^32-1], (equivalent floating point range is: 0.0 to (2^16-2^-16) )
					default value:  65536  (equivalent to floating point value 1.0 )
*/
typedef struct shoebox_decay_time_adjust_param_t           
{													
	uint32_t struct_size;
	uint32_t decay_time_adjust;
} shoebox_decay_time_adjust_param_t;


/* shoebox_brightness_adjust_param_t

This structure defines the balance between the low and high frequency components of the reverb, which is associated 
with SHOEBOX_BRIGHTNESS_ADJUST_PARAM defined in shoebox_api.h.  
Internally, this is mapped to tuning of shoebox_decay_hf_ratio_param_t and shoebox_room_hf_level_param_t to match with DayDream's brightness_adjust

Supported for set/get_param(...) api

struct_size: [IN/OUT]: --IN : Caller fills size for the size of the payload structure in apis set_param(...) & get_param(...). expects size >= sizeof(shoebox_brightness_adjust_param_t)
                       --OUT: Callee fills the appropriate size of the buffer filled in api get_param(...),sizeof(shoebox_brightness_adjust_param_t)
brightness_adjust:  [IN/OUT]: the balance between the low and high frequency components of the reverb, format in uint32Q16 
					range: [0, 2^32-1], (equivalent floating point range is: 0.0 to (2^16-2^-16) )
					default value:  65536  (equivalent to floating point value 1.0 )
*/
typedef struct shoebox_brightness_param_t       
{                                              
	uint32_t struct_size;
	uint32_t brightness_adjust;
} shoebox_brightness_adjust_param_t;


/* shoebox_reset_param_t 

This structure defines the parameters associated with SHOEBOX_RESET_PARAM, in enum algo_struct_id_t, which is in algo_api.h

algo_set can be used to reinit the library while retaining the current set of calibration parameters
Supports set_param(...) only

struct_size: [IN]: --IN : Caller fills size for the size of the payload structure in apis set_param(...) . expects size >= sizeof(shoebox_reset_param_t) 

This parameter has no payload other than size
*/
typedef struct shoebox_reset_param_t {			  
	uint32_t struct_size;
} shoebox_reset_param_t;


/* shoebox_alg_delay_param_t 

This structure defines the algorithm delay associated with SHOEBOX_ALG_DELAY_PARAM, in enum algo_struct_id_t, which is in algo_api.h

algo_get can be used to return the algorithm delay in Shoebox
Supports get_param(...) only

struct_size: [IN]: IN : Caller fills size for the size of the payload structure in apis set_param(...) . expects size >= sizeof(shoebox_alg_delay_param_t) 
alg_delay:   [OUT]: the algorithm delay of shoebox.  For v1, it always return 0 for there's no algorithm delay in dry path
*/
typedef struct shoebox_alg_delay_param_t             
{
	uint32_t struct_size;
	uint32_t alg_delay;
} shoebox_alg_delay_param_t;


/* shoebox_crossfade_param_t 

This structure defines the crossfading flag associated with SHOEBOX_CROSSFADE_PARAM, in enum algo_struct_id_t, which is in algo_api.h
crossfading(gain panning) is executed when shoebox/reverb is enabled or disabled or cfg changed on the fly.

algo_get can be used to return the crossfading flag in Shoebox
Supports get_param(...) only

struct_size: [IN]: IN : Caller fills size for the size of the payload structure in apis set_param(...) . expects size >= sizeof(shoebox_crossfade_param_t) 
crossfade:   [OUT]: the crossfading flag returned from the library, 
			 range: [0(no), 1(yes)]
			 crossfading/transition time is fixed to 100ms inside Reverb.  No configurable through API.

*/
typedef struct shoebox_crossfade_param_t           
{                                               
	uint32_t struct_size;
	uint32_t crossfade;
} shoebox_crossfade_param_t;


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* SHOEBOX_CALIBRATION_API_H */
