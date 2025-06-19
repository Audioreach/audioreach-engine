/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef API_REVERB_H
#define API_REVERB_H
/*==============================================================================
  @file api_reverb.h
  @brief This file contains REVERB API
==============================================================================*/

/*------------------------------------------------------------------------
 * Include files
 * -----------------------------------------------------------------------*/
#include "module_cmn_api.h"
/*==============================================================================
   Constants
==============================================================================*/
#define REVERB_MAX_IN_PORTS 1
#define REVERB_MAX_OUT_PORTS 1
#define REVERB_STACK_SIZE 2000



#define MODULE_ID_REVERB                         0x07001059
/**
    @h2xmlm_module       {"MODULE_ID_REVERB",
                          MODULE_ID_REVERB}
    @h2xmlm_displayName  {"Reverb"}
    @h2xmlm_modSearchKeys{effects, Audio}
	@h2xmlm_description  {ID of the Reverberation module.\n

    This module supports the following parameter IDs:\n
    - #PARAM_ID_MODULE_ENABLE \n
	- #PARAM_ID_REVERB_MODE\n
    - #PARAM_ID_REVERB_PRESET\n
    - #PARAM_ID_REVERB_WET_MIX\n
    - #PARAM_ID_REVERB_GAIN_ADJUST\n
    - #PARAM_ID_REVERB_ROOM_LEVEL\n
    - #PARAM_ID_REVERB_ROOM_HF_LEVEL\n
    - #PARAM_ID_REVERB_DECAY_TIME\n
    - #PARAM_ID_REVERB_DECAY_HF_RATIO\n
    - #PARAM_ID_REVERB_REFLECTIONS_LEVEL\n
    - #PARAM_ID_REVERB_REFLECTIONS_DELAY\n
    - #PARAM_ID_REVERB_LEVEL\n
    - #PARAM_ID_REVERB_DELAY\n
    - #PARAM_ID_REVERB_DIFFUSION\n
    - #PARAM_ID_REVERB_DENSITY\n
    All parameter IDs are device independent.\n
	
	Supported Input Media Format:\n
*  - Data Format          : FIXED_POINT\n
*  - fmt_id               : Don't care\n
*  - Sample Rates         : {8000,11025,16000,22050,32000,44100,48000}\n
*  - Number of channels   : {1 ,2 ,6 ,8}\n
*  - Channel type         : 1 to 63\n
*  - Bits per sample      : 16, 32\n
*  - Q format             : 15, 27\n
*  - Interleaving         : de-interleaved unpacked\n
*  - Signed/unsigned      : Signed }
	
    
	@h2xmlm_toolPolicy   {Calibration}

    

    @h2xmlm_dataMaxInputPorts    {REVERB_MAX_IN_PORTS}
    @h2xmlm_dataInputPorts       {IN=2}
    @h2xmlm_dataMaxOutputPorts   {REVERB_MAX_OUT_PORTS}
    @h2xmlm_dataOutputPorts      {OUT=1}
    @h2xmlm_supportedContTypes  {APM_CONTAINER_TYPE_SC}
    @h2xmlm_isOffloadable        {true}
    @h2xmlm_stackSize            {REVERB_STACK_SIZE}
    @h2xmlm_ToolPolicy              {Calibration}
    @{                   <-- Start of the Module -->
	
	@h2xml_Select        {"param_id_module_enable_t"}
    @h2xmlm_InsertParameter
*/
/* ID of the Reverberation Mode parameter used by MODULE_ID_REVERB. */
#define PARAM_ID_REVERB_MODE                      0x080010FD

/* Structure for the mode parameter of Reverb module. */
typedef struct capi_reverb_mode_t capi_reverb_mode_t;
/** @h2xmlp_parameter   {"PARAM_ID_REVERB_MODE", PARAM_ID_REVERB_MODE}
    @h2xmlp_description { Used only for Get Parameter calls.\n
    This parameter must be initialized once when the library is created.\n
    The mode cannot be changed after initialization. Hence, this parameter
    ID is used only for Get Parameter calls.\n
	Insert effects\n
	One source is in and one source is out. The input/output channels match so
    the reverberation output is a mixture of original (dry) sound and
    reverberation (wet) sound. For example, applying reverberation to a music
    stream.\n
	Auxiliary effects\n
	Multiple input sources and one global reverberation engine. Each input source makes a copy of itself
	with its own (Q15) reverberation send gain applied. The copies are mixed internally inside the
	reverberation library.\n
	The sound sources can have different numbers of channels, and they do not need to match the
	reverberation output numbers of channels.\n
	After mixing all inputs, reverberation generates dense echoes (wet sound). The reverberation (wet)
	output must be mixed somewhere outside the reverberation library with the direct (dry) sound. For
	example, applying one global reverberation for gaming or multi-track MIDI.\n } */

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"

/* Payload of the PARAM_ID_REVERB_MODE parameter used by the
 Reverb module.
 */
struct capi_reverb_mode_t
{
   uint32_t reverb_mode;
   /**< @h2xmle_description  { Specifies the reverberation topology mode.\n
        The COPP_TOPOLOGY_ID_AUDIO_PLUS_HEADPHONE and
        COPP_TOPOLOGY_ID_AUDIO_PLUS_SPEAKER topologies support only
        Insert Effects mode.\n
        For Auxiliary Effect mode, a custom topology must be defined.\n }

        @h2xmle_rangeList    {"Insert Effects mode"=0;
                              "Auxiliary Effects mode"=1}
        @h2xmle_default      {0}   */

}

#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;

/* ID of the Reverberation Preset parameter used by MODULE_ID_REVERB. */
#define PARAM_ID_REVERB_PRESET                    0x080010FE

/** User-customized preset (with environmental reverberation parameters specified individually). */
#define  REVERB_PRESET_CUSTOM            0

/** Simulates an environment in a room. */
#define  REVERB_PRESET_ROOM              1

/** Simulates an environment in a bathroom. */
#define  REVERB_PRESET_BATHROOM          2

/** Simulates an environment in a concert hall. */
#define  REVERB_PRESET_CONCERTHALL       3

/** Simulates an environment in a cave. */
#define  REVERB_PRESET_CAVE              4

/** Simulates an environment in an arena. */
#define  REVERB_PRESET_ARENA             5

/** Simulates an environment in a forest. */
#define  REVERB_PRESET_FOREST            6

/** Simulates an environment in a city. */
#define  REVERB_PRESET_CITY              7

/** Simulates an environment in the mountains (open air). */
#define  REVERB_PRESET_MOUNTAINS         8

/** Simulates an environment under the water. */
#define  REVERB_PRESET_UNDERWATER        9

/** Simulates an environment in an auditorium. */
#define  REVERB_PRESET_AUDITORIUM        10

/** Simulates an environment in an alley. */
#define  REVERB_PRESET_ALLEY             11

/** Simulates an environment in a hallway. */
#define  REVERB_PRESET_HALLWAY           12

/** Simulates an environment in a hangar. */
#define  REVERB_PRESET_HANGAR            13

/** Simulates an environment in a living room. */
#define  REVERB_PRESET_LIVINGROOM        14

/** Simulates an environment in a small room. */
#define  REVERB_PRESET_SMALLROOM         15

/** Simulates an environment in a medium-sized room. */
#define  REVERB_PRESET_MEDIUMROOM        16

/** Simulates an environment in a large room. */
#define  REVERB_PRESET_LARGEROOM         17

/** Simulates an environment in a medium-sized hall. */
#define  REVERB_PRESET_MEDIUMHALL        18

/** Simulates an environment in a large hall. */
#define  REVERB_PRESET_LARGEHALL         19

/** Simulates sound being sent to a metal plate, which vibrates back and forth.
    These vibrations are transformed into an audio signal.
 */
 #define  REVERB_PRESET_PLATE             20

/** Simulates a generic reverberation effect. */
#define  REVERB_PRESET_GENERIC           21

/** Simulates an environment in a padded cell. */
#define  REVERB_PRESET_PADDEDCELL        22

/** Simulates an environment in a stone room. */
#define  REVERB_PRESET_STONEROOM         23

/** Simulates an environment in a carpeted hallway. */
#define  REVERB_PRESET_CARPETEDHALLWAY   24

/** Simulates an environment in a stone corridor. */
#define  REVERB_PRESET_STONECORRIDOR     25

/** Simulates an environment in a quarry. */
#define  REVERB_PRESET_QUARRY            26

/** Simulates an environment on an open plain. */
#define  REVERB_PRESET_PLAIN             27

/** Simulates an environment in a parking lot. */
#define  REVERB_PRESET_PARKINGLOT        28

/** Simulates an environment in a sewer pipe. */
#define  REVERB_PRESET_SEWERPIPE         29

/** Synthetic environment preset: drugged. */
#define  REVERB_PRESET_DRUGGED           30

/** Synthetic environment preset: dizzy. */
#define  REVERB_PRESET_DIZZY             31

/** Synthetic environment preset: psychotic. */
#define  REVERB_PRESET_PSYCHOTIC         32



/* Structure for the preset parameter of Reverb module. */
typedef struct capi_reverb_preset_t capi_reverb_preset_t;
/** @h2xmlp_parameter   {"PARAM_ID_REVERB_PRESET", PARAM_ID_REVERB_PRESET}
    @h2xmlp_description { Used to preset parameters for Reverb module.\n\n
User-customized preset (with environmental reverberation parameters specified individually) - REVERB_PRESET_CUSTOM\n\n
To simulates an environment in a room - REVERB_PRESET_ROOM\n\n
To simulates an environment in a bathroom - REVERB_PRESET_BATHROOM\n \n
To Simulates an environment in a concert hall -REVERB_PRESET_CONCERTHALL\n  \n
To Simulates an environment in a cave - REVERB_PRESET_CAVE\n \n
To Simulates an environment in an arena - REVERB_PRESET_ARENA\n \n
To Simulates an environment in a forest - REVERB_PRESET_FOREST\n \n
To  Simulates an environment in a city - REVERB_PRESET_CITY\n \n
To Simulates an environment in the mountains (open air) - REVERB_PRESET_MOUNTAINS\n \n
To Simulates an environment under the water - REVERB_PRESET_UNDERWATER\n \n
To Simulates an environment in an auditorium - REVERB_PRESET_AUDITORIUM\n\n
To Simulates an environment in an alley - REVERB_PRESET_ALLEY\n\n
To Simulates an environment in a hallway - REVERB_PRESET_HALLWAY\n\n
To Simulates an environment in a hangar - REVERB_PRESET_HANGAR\n\n
To Simulates an environment in a living room - REVERB_PRESET_LIVINGROOM\n \n
To Simulates an environment in a small room - REVERB_PRESET_SMALLROOM\n\n
To Simulates an environment in a medium-sized room - REVERB_PRESET_MEDIUMROOM\n\n
To Simulates an environment in a large room - REVERB_PRESET_LARGEROOM\n\n
To Simulates an environment in a medium-sized hall - REVERB_PRESET_MEDIUMHALL\n\n
To Simulates an environment in a large hall - REVERB_PRESET_LARGEHALL\n\n
To Simulates sound being sent to a metal plate, which vibrates back and forth(These vibrations are transformed into an audio signal) - REVERB_PRESET_PLATE\n\n
To Simulates a generic reverberation effect - REVERB_PRESET_GENERIC\n\n
To Simulates an environment in a padded cell - REVERB_PRESET_PADDEDCELL\n\n
ToSimulates an environment in a stone room - REVERB_PRESET_STONEROOM\n\n
To Simulates an environment in a carpeted hallway - REVERB_PRESET_CARPETEDHALLWAY\n\n
To Simulates an environment in a stone corridor - REVERB_PRESET_STONECORRIDOR\n\n
To Simulates an environment in a quarry - REVERB_PRESET_QUARRY\n\n
To Simulates an environment on an open plain - REVERB_PRESET_PLAIN\n\n
To Simulates an environment in a parking lot - REVERB_PRESET_PARKINGLOT\n\n
To Simulates an environment in a sewer pipe - REVERB_PRESET_SEWERPIPE \n\n
To Synthetic environment preset: drugged - REVERB_PRESET_DRUGGED \n\n
To Synthetic environment preset: dizzy - REVERB_PRESET_DIZZY\n\n
To Synthetic environment preset: psychotic - REVERB_PRESET_PSYCHOTIC\n } */

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"

/* Payload of the PARAM_ID_REVERB_PRESET parameter used by the
 Reverb module.
 */
struct capi_reverb_preset_t
{
   uint32_t reverb_preset;
   /**< @h2xmle_description  {Specifies one of the reverberation presets that create special
							  environmental audio effects.}
        @h2xmle_range        {0..32}
        @h2xmle_default     {REVERB_PRESET_CUSTOM}

        @h2xmle_rangeList    {"CAPI_REVERB_PRESET_CUSTOM"=0;
                              " CAPI_REVERB_PRESET_ROOM"=1;
                "CAPI_REVERB_PRESET_BATHROOM"=2;
                "CAPI_REVERB_PRESET_CONCERTHALL"=3;
                "CAPI_REVERB_PRESET_CAVE"=4;
                "CAPI_REVERB_PRESET_ARENA"=5;
                "CAPI_REVERB_PRESET_FOREST"=6;
                "CAPI_REVERB_PRESET_CITY"=7;
                "CAPI_REVERB_PRESET_MOUNTAINS "=8;
                "CAPI_REVERB_PRESET_UNDERWATER"=9;
                "CAPI_REVERB_PRESET_AUDITORIUM"=10;
                "CAPI_REVERB_PRESET_ALLEY"=11;
                "CAPI_REVERB_PRESET_HALLWAY"=12;
                " CAPI_REVERB_PRESET_HANGAR"=13;
                "CAPI_REVERB_PRESET_LIVINGROOM"=14;
                "CAPI_REVERB_PRESET_SMALLROOM "=15;
                "CAPI_REVERB_PRESET_MEDIUMROOM"=16;
                "CAPI_REVERB_PRESET_LARGEROOM "=17;
                "CAPI_REVERB_PRESET_MEDIUMHALL"=18;
                " CAPI_REVERB_PRESET_LARGEHALL "=19;
                "CAPI_REVERB_PRESET_PLATE"=20;
                " CAPI_REVERB_PRESET_GENERIC"=21;
                "CAPI_REVERB_PRESET_PADDEDCELL "=22;
                "CAPI_REVERB_PRESET_STONEROOM"=23;
                "CAPI_REVERB_PRESET_CARPETEDHALLWAY"=24;
                "CAPI_REVERB_PRESET_STONECORRIDOR"=25;
                "CAPI_REVERB_PRESET_QUARRY"=26;
                "CAPI_REVERB_PRESET_PLAIN"=27;
                "CAPI_REVERB_PRESET_PARKINGLOT"=28;
                "CAPI_REVERB_PRESET_SEWERPIPE"=29;
                "CAPI_REVERB_PRESET_DRUGGED"=30;
                "CAPI_REVERB_PRESET_DIZZY"=31;
                "CAPI_REVERB_PRESET_PSYCHOTIC"=32 }	*/

}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;


/* ID of the Reverberation Wet Mix parameter used by CAPI_MODULE_ID_REVERB. */
#define PARAM_ID_REVERB_WET_MIX                   0x080010FF

/* Structure for the wet mix parameter of Reverb module. */
typedef struct capi_reverb_wet_mix_t capi_reverb_wet_mix_t;
/** @h2xmlp_parameter   {"PARAM_ID_REVERB_WET_MIX", PARAM_ID_REVERB_WET_MIX}
    @h2xmlp_description { Specifies the reverberation wet/dry mix ratio for Insert Effects mode} */

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"

/* Payload of the PARAM_ID_REVERB_WET_MIX parameter used by the
 Reverb module.
 */
struct capi_reverb_wet_mix_t
{
   uint32_t reverb_wet_mix;
   /**< @h2xmle_description  {Specifies the reverberation wet/dry mix ratio for Insert Effects mode.}
        @h2xmle_range        {0..1000}
	@h2xmle_default	{618} */

}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;

/* ID of the Reverberation Gain Adjust parameter used by CAPI_MODULE_ID_REVERB. */
#define PARAM_ID_REVERB_GAIN_ADJUST               0x08001100

/* Structure for the gain adjust parameter of Reverb module. */
typedef struct capi_reverb_gain_adjust_t capi_reverb_gain_adjust_t;
/** @h2xmlp_parameter   {"PARAM_ID_REVERB_GAIN_ADJUST", PARAM_ID_REVERB_GAIN_ADJUST}
    @h2xmlp_description { Specifies the overall gain adjustment of reverberation outputs}  */

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"

/* Payload of the PARAM_ID_REVERB_GAIN_ADJUST parameter used by the
 Reverb module.
 */
struct capi_reverb_gain_adjust_t
{
   int32_t gain_adjust;
   /**< @h2xmle_description  {Specifies the overall gain adjustment of
                              reverberation outputs,in the units of 'millibels'.}
        @h2xmle_range        {-600..600}
        @h2xmle_default      {0}	*/

}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;

/* ID of the Reverberation Room Level parameter used by CAPI_MODULE_ID_REVERB. */
#define PARAM_ID_REVERB_ROOM_LEVEL                0x08001101

/* Structure for the room level parameter of Reverb module. */
typedef struct capi_reverb_room_level_t capi_reverb_room_level_t;
/** @h2xmlp_parameter   {"PARAM_ID_REVERB_ROOM_LEVEL", PARAM_ID_REVERB_ROOM_LEVEL}
    @h2xmlp_description { Specifies the master volume level of the environment reverberation effect} */

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"

/* Payload of the PARAM_ID_REVERB_ROOM_LEVEL parameter used by the
 Reverb module.
 */
struct capi_reverb_room_level_t
{
   int32_t room_level;
   /**< @h2xmle_description  {Specifies the master volume level of the environment reverberation
                              effect,in the units of 'millibels'.}
        @h2xmle_range        {-9600..0}
	@h2xmle_default       {-9600}       	*/

}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;

/* ID of the Reverberation Room High Frequency Level parameter used by CAPI_MODULE_ID_REVERB. */
#define PARAM_ID_REVERB_ROOM_HF_LEVEL             0x08001102

/* Structure for the room hf level parameter of Reverb module. */
typedef struct capi_reverb_room_hf_level_t capi_reverb_room_hf_level_t;
/** @h2xmlp_parameter   {"PARAM_ID_REVERB_ROOM_HF_LEVEL", PARAM_ID_REVERB_ROOM_HF_LEVEL}
    @h2xmlp_description { Specifies the relative volume level} */

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"

/* Payload of the PARAM_ID_REVERB_ROOM_HF_LEVEL parameter used by the
 Reverb module.
 */
struct capi_reverb_room_hf_level_t
{
   int32_t room_hf_level;
   /**< @h2xmle_description  { Specifies the volume level at 5 kHz relative to the volume level at
                               low frequencies of the overall reverberation effect,
							   in the units of 'millibels'.}
        @h2xmle_range        {-9600..0}
        @h2xmle_default       {0}		*/

}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;

/* ID of the Reverberation Decay Time parameter used by CAPI_MODULE_ID_REVERB. */
#define PARAM_ID_REVERB_DECAY_TIME                0x08001103

/* Structure for the decay time parameter of Reverb module. */
typedef struct capi_reverb_decay_time_t capi_reverb_decay_time_t;
/** @h2xmlp_parameter   {"PARAM_ID_REVERB_DECAY_TIME", PARAM_ID_REVERB_DECAY_TIME}
    @h2xmlp_description { Specifies the Specifies the time for the level of reverberation} */

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"

/* Payload of the PARAM_ID_REVERB_DECAY_TIME parameter used by the
 Reverb module.
 */
struct capi_reverb_decay_time_t
{
   uint32_t decay_time;
   /**< @h2xmle_description  {Specifies the time for the level of reverberation to decay
                              by 60 dB,in the units of 'milliseconds'.}
		@h2xmle_default      {1000}
        @h2xmle_range        {100..20000}  */

}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;

/* ID of the Reverberation Decay High Frequency Ratio parameter used by CAPI_MODULE_ID_REVERB. */
#define PARAM_ID_REVERB_DECAY_HF_RATIO            0x08001104

/* Structure for the decay hf ratio parameter of Reverb module. */
typedef struct capi_reverb_decay_hf_ratio_t capi_reverb_decay_hf_ratio_t;
/** @h2xmlp_parameter   {"PARAM_ID_REVERB_DECAY_HF_RATIO", PARAM_ID_REVERB_DECAY_HF_RATIO}
    @h2xmlp_description { Specifies the ratio of high frequency decay time} */

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"

/* Payload of the PARAM_ID_REVERB_DECAY_HF_RATIO parameter used by the
 Reverb module.
 */
struct capi_reverb_decay_hf_ratio_t
{
   uint32_t decay_hf_ratio;
   /**< @h2xmle_description  {Specifies the ratio of high frequency decay time (at 5 kHz) relative
                              to the decay time at low frequencies,in the units of 'milliseconds'.}
		@h2xmle_default      {500}
        @h2xmle_range        {150..2000}  */

}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;

/*ID of the Reverberation Reflections Level parameter used by CAPI_MODULE_ID_REVERB. */
#define PARAM_ID_REVERB_REFLECTIONS_LEVEL         0x08001105

/* Structure for the reverb reflections level parameter of Reverb module. */
typedef struct capi_reverb_reflections_level_t capi_reverb_reflections_level_t;
/** @h2xmlp_parameter   {"PARAM_ID_REVERB_REFLECTIONS_LEVEL",PARAM_ID_REVERB_REFLECTIONS_LEVEL}
    @h2xmlp_description { Specifies the volume level of the early reflections} */

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"

/* Payload of the PARAM_ID_REVERB_REFLECTIONS_LEVEL parameter used by the
 Reverb module.
 */
struct capi_reverb_reflections_level_t
{
   int32_t reflections_level;
   /**< @h2xmle_description  {Specifies the volume level of the early reflections,in the units of 'millibels'.}
        @h2xmle_range        {-9600..1000}
	@h2xmle_default       {-9600}  */

}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;


/*ID of the Reverberation Reflections Delay parameter used by CAPI_MODULE_ID_REVERB. */
#define PARAM_ID_REVERB_REFLECTIONS_DELAY         0x08001106

/* Structure for the reverb reflections delay parameter of Reverb module. */
typedef struct capi_reverb_reflections_delay_t capi_reverb_reflections_delay_t;
/** @h2xmlp_parameter   {"PARAM_ID_REVERB_REFLECTIONS_DELAY",PARAM_ID_REVERB_REFLECTIONS_DELAY}
    @h2xmlp_description { Specifies the time between the first reflection and the late reverberation} */

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"

/* Payload of the PARAM_ID_REVERB_REFLECTIONS_DELAY parameter used by the
 Reverb module.
 */
struct capi_reverb_reflections_delay_t
{
   uint32_t reflections_delay;
   /**< @h2xmle_description  { Specifies the time between the first reflection and
                               the late reverberation,in the units of 'milliseconds'.}
        @h2xmle_range        {0..300}
	@h2xmle_default       {20} */

}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;


/* ID of the Reverberation Level parameter used by CAPI_MODULE_ID_REVERB. */
#define PARAM_ID_REVERB_LEVEL                      0x08001107

/* Structure for the reverb level parameter of Reverb module. */
typedef struct capi_reverb_level_t capi_reverb_level_t;
/** @h2xmlp_parameter   {"PARAM_ID_REVERB_LEVEL",PARAM_ID_REVERB_LEVEL}
    @h2xmlp_description { Specifies the volume level of the late reverberation} */

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"

/* Payload of the PARAM_ID_REVERB_LEVEL parameter used by the
 Reverb module.
 */
struct capi_reverb_level_t
{
   int32_t reverb_level;
   /**< @h2xmle_description  {Specifies the volume level of the late reverberation.}
        @h2xmle_range        {-9600..2000}
        @h2xmle_default       {-9600} */

}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;


/*ID of the Reverberation Delay parameter used by CAPI_MODULE_ID_REVERB. */
#define PARAM_ID_REVERB_DELAY                      0x08001108

/* Structure for the reverb delay parameter of Reverb module. */
typedef struct capi_reverb_delay_t capi_reverb_delay_t;
/** @h2xmlp_parameter   {"PARAM_ID_REVERB_DELAY",PARAM_ID_REVERB_DELAY}
    @h2xmlp_description { Specifies the time between the first reflection and the late reverberation} */

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"

/* Payload of the PARAM_ID_REVERB_DELAY parameter used by the
 Reverb module.
 */
struct capi_reverb_delay_t
{
   uint32_t reverb_delay;
   /**< @h2xmle_description  {Specifies the time between the first reflection and
                              the late reverberation,in the units of ' milliseconds'.}
        @h2xmle_range        {0..100}
        @h2xmle_default      {40} */

}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;


/* ID of the Reverberation Diffusion parameter used by CAPI_MODULE_ID_REVERB. */
#define PARAM_ID_REVERB_DIFFUSION                  0x08001109

/* Structure for the reverb diffusion parameter of Reverb module. */
typedef struct capi_reverb_diffusion_t capi_reverb_diffusion_t;
/** @h2xmlp_parameter   {"PARAM_ID_REVERB_DIFFUSION",PARAM_ID_REVERB_DIFFUSION}
    @h2xmlp_description { Specifies the echo density in the late reverberation decay} */

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"

/* Payload of the PARAM_ID_REVERB_DIFFUSION parameter used by the
 Reverb module.
 */
struct capi_reverb_diffusion_t
{
   uint32_t diffusion;
   /**< @h2xmle_description  {Specifies the echo density in the late reverberation decay.}
        @h2xmle_range        {0..1000}
       @h2xmle_default       {1000}	 */

}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;

/** ID of the Reverberation Density parameter used by CAPI_MODULE_ID_REVERB. */
#define PARAM_ID_REVERB_DENSITY                    0x0800110A

/* Structure for the reverb density parameter of Reverb module. */
typedef struct capi_reverb_density_t capi_reverb_density_t;
/** @h2xmlp_parameter   {"PARAM_ID_REVERB_DENSITY",PARAM_ID_REVERB_DENSITY}
    @h2xmlp_description { Specifies the modal density of the late reverberation decay} */

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"

/* Payload of the PARAM_ID_REVERB_DENSITY parameter used by the
 Reverb module.
 */
struct capi_reverb_density_t
{
   uint32_t density;
   /**< @h2xmle_description  {Specifies the modal density of the late reverberation decay}
        @h2xmle_range    {0..1000}

        @h2xmle_default	{1000} 		*/

}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;
/** @}                   <-- End of the Module -->*/

#endif

