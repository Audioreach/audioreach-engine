/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef REVERB_CALIBRATION_API_H
#define REVERB_CALIBRATION_API_H

#include "AudioComdef.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/*----------------------------------------------------------------------------
   Reverb Environment Enum
----------------------------------------------------------------------------*/

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

#endif /* REVERB_CALIBRATION_API_H */
