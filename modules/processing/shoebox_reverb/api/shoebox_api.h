/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef API_SHOEBOX_H
#define API_SHOEBOX_H
/*==============================================================================
  @file api_shoebox.h
  @brief This file contains SHOEBOX API
==============================================================================*/

/*------------------------------------------------------------------------
 * Include files
 * -----------------------------------------------------------------------*/
#include "module_cmn_api.h"
/*==============================================================================
   Constants
==============================================================================*/

#define SHOEBOX_MAX_IN_PORTS 1
#define SHOEBOX_MAX_OUT_PORTS 1
#define SHOEBOX_STACK_SIZE 2000


#define MODULE_ID_SHOEBOX 0x07001058
/**
    @h2xmlm_module       {"MODULE_ID_SHOEBOX",
                          MODULE_ID_SHOEBOX}
    @h2xmlm_displayName  {"Shoebox"}
    @h2xmlm_modSearchKeys{effects, Audio}

    @h2xmlm_toolPolicy              {Calibration}
	@h2xmlm_description  {Shoebox Module which reduces reverberation effects in a cubical room.\n
	- This module supports the following parameter IDs:  \n
	   - #PARAM_ID_MODULE_ENABLE \n
*      - #PARAM_ID_SHOEBOX_BRIGHTNESS_ADJUST \n
*      - #PARAM_ID_SHOEBOX_DECAY_TIME_ADJUST \n
*      - #PARAM_ID_SHOEBOX_ROOM_MATERIAL_TIME_ADJUST \n
*      - #PARAM_ID_SHOEBOX_ROOM_SIZE_TIME_ADJUST \n
*      - #PARAM_ID_SHOEBOX_WET_MIX \n
*      - #PARAM_ID_SHOEBOX_GAIN \n
*
* Supported Input Media Format:\n
*  - Data Format          : FIXED_POINT\n
*  - fmt_id               : Don't care\n
*  - Sample Rates         : {8000,16000,32000,44100,48000,96000,192000}\n
*  - Number of channels   : {1 ,2}\n
*  - Channel type         : 1 to 63\n
*  - Bits per sample      : 16, 32\n
*  - Q format             : 15, 27\n
*  - Interleaving         : de-interleaved unpacked\n
*  - Signed/unsigned      : Signed }

    @h2xmlm_dataMaxInputPorts    {SHOEBOX_MAX_IN_PORTS}
    @h2xmlm_dataInputPorts       {IN=2}
    @h2xmlm_dataMaxOutputPorts   {SHOEBOX_MAX_OUT_PORTS}
    @h2xmlm_dataOutputPorts      {OUT=1}
    @h2xmlm_supportedContTypes  {APM_CONTAINER_TYPE_SC, APM_CONTAINER_TYPE_GC}
    @h2xmlm_isOffloadable        {true}
    @h2xmlm_stackSize            {SHOEBOX_STACK_SIZE}
    @h2xmlm_ToolPolicy              {Calibration}
    @{                   <-- Start of the Module -->  
	@h2xml_Select        {"param_id_module_enable_t"}
    @h2xmlm_InsertParameter
	*/

#define PARAM_ID_SHOEBOX_WET_MIX 0x0800111D

/* Structure for the wet mix parameter of Shoebox module. */
typedef struct shoebox_wet_mix_t shoebox_wet_mix_t;
/** @h2xmlp_parameter   {"PARAM_ID_SHOEBOX_WET_MIX", PARAM_ID_SHOEBOX_WET_MIX}
    @h2xmlp_description {Payload of the PARAM_ID_SHOEBOX_WET_MIX parameter used by the Shoebox module}
    */

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"

/* Payload of the PARAM_ID_SHOEBOX_WET_MIX parameter used by the
 Shoebox module.
 */
struct shoebox_wet_mix_t
{
   uint32_t wet_mix;
   /**< @h2xmle_description {Specifies the mix ratio between dry and wet sound}
      @h2xmle_default     {600}
      @h2xmle_range  {0..1000}  */
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;

#define PARAM_ID_SHOEBOX_ROOM_SIZE 0x0800111E

/* Structure for the room size parameter of Shoebox module. */
typedef struct shoebox_room_size_t shoebox_room_size_t;
/** @h2xmlp_parameter   {"PARAM_ID_SHOEBOX_ROOM_SIZE", PARAM_ID_SHOEBOX_ROOM_SIZE}
    @h2xmlp_description {Payload of the PARAM_ID_SHOEBOX_ROOM_SIZE parameter used by the Shoebox module}
   */

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"

/* Payload of the PARAM_ID_SHOEBOX_ROOM_SIZE parameter used by the
 Shoebox module.
 */
struct shoebox_room_size_t
{
   uint32_t length;
   /**< @h2xmle_description {Specifies length of the room, in the units of 'meter'}
      @h2xmle_default     {524288}
      @h2xmle_range  {0..13107200}
      @h2xmle_dataFormat  {Q16}     */

   uint32_t width;
   /**< @h2xmle_description { Specifies width of the room, in the units of 'meter'}
      @h2xmle_default     {327680}
      @h2xmle_range  {0..6553600}
      @h2xmle_dataFormat  {Q16}     */

   uint32_t height;
   /**< @h2xmle_description { Specifies height of the room, in the units of 'meter'}
      @h2xmle_default     {196608}
      @h2xmle_range  {0..655360}
      @h2xmle_dataFormat  {Q16}     */
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;

#define PARAM_ID_SHOEBOX_ROOM_MATERIAL 0x0800111F

/** supported material IDs forPARAM_ID_SHOEBOX_ROOM_MATERIAL
 */

/** To simulate the case when there is no reflection. */
#define SHOEBOX_MATERIAL_TRANSPARENT 0

/** To simulate the reflection from acoustic ceiling tiles. */
#define SHOEBOX_MATERIAL_ACOUSTIC_CEILING_TILES 1

/** To simulate the reflection from bare brick surface. */
#define SHOEBOX_MATERIAL_BRICK_BARE 2

/** To simulate the reflection from painted brick surface. */
#define SHOEBOX_MATERIAL_BRICK_PAINTED 3

/** To simulate the reflection from coarse concrete block. */
#define SHOEBOX_MATERIAL_CONCRETE_BLOCK_COARSE 4

/** To simulate the reflection from painted concrete block. */
#define SHOEBOX_MATERIAL_CONCRETE_BLOCK_PAINTED 5

/** To simulate the reflection from heavy curtain. */
#define SHOEBOX_MATERIAL_CURTAIN_HEAVY 6

/** To simulate the reflection from fiber glass insulation. */
#define SHOEBOX_MATERIAL_FIBER_GLASS_INSULATION 7

/** To simulate the reflection from thick glass */
#define SHOEBOX_MATERIAL_GLASS_THICK 8

/** To simulate the reflection from thin glass. */
#define SHOEBOX_MATERIAL_GLASS_THIN 9

/** To simulate the reflection from grass. */
#define SHOEBOX_MATERIAL_GRASS 10

/** To simulate the reflection from linoleum on concrete. */
#define SHOEBOX_MATERIAL_LINOLEUM_ON_CONCRETE 11

/** To simulate the reflection from marble. */
#define SHOEBOX_MATERIAL_MARBLE 12

/** To simulate the reflection from metal. */
#define SHOEBOX_MATERIAL_METAL 13

/** To simulate the reflection from parquet on concrete. */
#define SHOEBOX_MATERIAL_PARQUET_ON_CONCRETE 14

/** To simulate the reflection from rough plaster. */
#define SHOEBOX_MATERIAL_PLASTER_ROUGH 15

/** To simulate the reflection from smooth plaster. */
#define SHOEBOX_MATERIAL_PLASTER_SMOOTH 16

/** To simulate the reflection from plywood panel. */
#define SHOEBOX_MATERIAL_PLYWOOD_PANEL 17

/** To simulate the reflection from polished concrete or tile. */
#define SHOEBOX_MATERIAL_POLISHED_CONCRETE_OR_TILE 18

/** To simulate the reflection from sheet rock. */
#define SHOEBOX_MATERIAL_SHEET_ROCK 19

/** To simulate the reflection from water or ice surface. */
#define SHOEBOX_MATERIAL_WATER_OR_ICE_SURFACE 20

/** To simulate the reflection from wood on ceiling. */
#define SHOEBOX_MATERIAL_WOOD_ON_CEILING 21

/** To simulate the reflection from wood panel. */
#define SHOEBOX_MATERIAL_WOOD_PANEL 22

/* Structure for the room material parameter of Shoebox module. */
typedef struct shoebox_room_material_t shoebox_room_material_t;
/** @h2xmlp_parameter   {"PARAM_ID_SHOEBOX_ROOM_MATERIAL", PARAM_ID_SHOEBOX_ROOM_MATERIAL}
    @h2xmlp_description {Payload of the PARAM_ID_SHOEBOX_ROOM_MATERIAL parameter used by the
 Shoebox module.\n
To simulate the case when there is no reflection - SHOEBOX_MATERIAL_TRANSPARENT\n
To simulate the reflection from acoustic ceiling tiles - SHOEBOX_MATERIAL_ACOUSTIC_CEILING_TILES\n
To simulate the reflection from bare brick surface - SHOEBOX_MATERIAL_BRICK_BARE\n
To simulate the reflection from painted brick surface -
SHOEBOX_MATERIAL_BRICK_PAINTED\n
To simulate the reflection from coarse concrete block -
SHOEBOX_MATERIAL_CONCRETE_BLOCK_COARSE\n
To simulate the reflection from painted concrete block -
SHOEBOX_MATERIAL_CONCRETE_BLOCK_PAINTED\n
To simulate the reflection from heavy curtain -
SHOEBOX_MATERIAL_CURTAIN_HEAVY\n
To simulate the reflection from fiber glass insulation -
SHOEBOX_MATERIAL_FIBER_GLASS_INSULATION\n
To simulate the reflection from thick glass -
SHOEBOX_MATERIAL_GLASS_THICK\n
To simulate the reflection from thin glass -
SHOEBOX_MATERIAL_GLASS_THIN\n
To simulate the reflection from grass -
SHOEBOX_MATERIAL_GRASS\n
To simulate the reflection from linoleum on concrete -
SHOEBOX_MATERIAL_LINOLEUM_ON_CONCRETE\n
To simulate the reflection from marble -
SHOEBOX_MATERIAL_MARBLE\n
To simulate the reflection from metal -
SHOEBOX_MATERIAL_METAL\n
To simulate the reflection from parquet on concrete -
SHOEBOX_MATERIAL_PARQUET_ON_CONCRETE\n
To simulate the reflection from rough plaster -
SHOEBOX_MATERIAL_PLASTER_ROUGH\n
To simulate the reflection from smooth plaster -
SHOEBOX_MATERIAL_PLASTER_SMOOTH\n
To simulate the reflection from plywood panel -
SHOEBOX_MATERIAL_PLYWOOD_PANEL\n
To simulate the reflection from polished concrete or tile -
SHOEBOX_MATERIAL_POLISHED_CONCRETE_OR_TILE\n
To simulate the reflection from sheet rock -
SHOEBOX_MATERIAL_SHEET_ROCK\n
To simulate the reflection from water or ice surface -
SHOEBOX_MATERIAL_WATER_OR_ICE_SURFACE\n
To simulate the reflection from wood on ceiling -
SHOEBOX_MATERIAL_WOOD_ON_CEILING\n
To simulate the reflection from wood panel -
SHOEBOX_MATERIAL_WOOD_PANEL }

 */

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"

/* Payload of the PARAM_ID_SHOEBOX_ROOM_MATERIAL parameter used by the
 Shoebox module.

 This parameter supports the following material IDs:

 - #SHOEBOX_MATERIAL_TRANSPARENT
 - #SHOEBOX_MATERIAL_ACOUSTIC_CEILING_TILES
 - #SHOEBOX_MATERIAL_BRICK_BARE
 - #SHOEBOX_MATERIAL_BRICK_PAINTED
 - #SHOEBOX_MATERIAL_CONCRETE_BLOCK_COARSE
 - #SHOEBOX_MATERIAL_CONCRETE_BLOCK_PAINTED
 - #SHOEBOX_MATERIAL_CURTAIN_HEAVY
 - #SHOEBOX_MATERIAL_FIBER_GLASS_INSULATION
 - #SHOEBOX_MATERIAL_GLASS_THICK
 - #SHOEBOX_MATERIAL_GLASS_THIN
 - #SHOEBOX_MATERIAL_GRASS
 - #SHOEBOX_MATERIAL_LINOLEUM_ON_CONCRETE
 - #SHOEBOX_MATERIAL_MARBLE
 - #SHOEBOX_MATERIAL_METAL
 - #SHOEBOX_MATERIAL_PARQUET_ON_CONCRETE
 - #SHOEBOX_MATERIAL_PLASTER_ROUGH
 - #SHOEBOX_MATERIAL_PLASTER_SMOOTH
 - #SHOEBOX_MATERIAL_PLYWOOD_PANEL
 - #SHOEBOX_MATERIAL_POLISHED_CONCRETE_OR_TILE
 - #SHOEBOX_MATERIAL_SHEET_ROCK
 - #SHOEBOX_MATERIAL_WATER_OR_ICE_SURFACE
 - #SHOEBOX_MATERIAL_WOOD_ON_CEILING
 - #SHOEBOX_MATERIAL_WOOD_PANEL
 */
struct shoebox_room_material_t
{
   uint32_t front_wall;
   /**< @h2xmle_description {Specifies the material of front wall}
      @h2xmle_default     {SHOEBOX_MATERIAL_TRANSPARENT}
   @h2xmle_rangeList         {"SHOEBOX_MATERIAL_TRANSPARENT"=0;
                              "SHOEBOX_MATERIAL_ACOUSTIC_CEILING_TILES"=1;
                       "SHOEBOX_MATERIAL_BRICK_BARE"=2;
                       "SHOEBOX_MATERIAL_BRICK_PAINTED"=3;
                       "SHOEBOX_MATERIAL_CONCRETE_BLOCK_COARSE"=4;
                       "SHOEBOX_MATERIAL_CONCRETE_BLOCK_PAINTED"=5;
                       "SHOEBOX_MATERIAL_CURTAIN_HEAVY"=6;
                       "SHOEBOX_MATERIAL_FIBER_GLASS_INSULATION"=7;
                       "SHOEBOX_MATERIAL_GLASS_THICK"=8;
                       "SHOEBOX_MATERIAL_GLASS_THIN"=9;
                       "SHOEBOX_MATERIAL_GRASS"=10;
                       "SHOEBOX_MATERIAL_GRASS"=11;
                       "SHOEBOX_MATERIAL_LINOLEUM_ON_CONCRETE"=12;
                       "SHOEBOX_MATERIAL_MARBLE"=13;
                       "SHOEBOX_MATERIAL_METAL"=14;
                       "SHOEBOX_MATERIAL_PARQUET_ON_CONCRETE"=15;
                       "SHOEBOX_MATERIAL_PLASTER_ROUGH"=16;
                       "SHOEBOX_MATERIAL_PLASTER_SMOOTH"=17;
                       "SHOEBOX_MATERIAL_PLYWOOD_PANEL"=18;
                       "SHOEBOX_MATERIAL_POLISHED_CONCRETE_OR_TILE"=19;
                       "SHOEBOX_MATERIAL_SHEET_ROCK"=20;
                       "SHOEBOX_MATERIAL_WATER_OR_ICE_SURFACE"=21;
                       "SHOEBOX_MATERIAL_WOOD_ON_CEILING"=22;
                       "SHOEBOX_MATERIAL_WOOD_PANEL"=23 }				*/

   uint32_t back_wall;

   /**< @h2xmle_description {Specifies the material of back wall}
      @h2xmle_default     {SHOEBOX_MATERIAL_TRANSPARENT}
         @h2xmle_rangeList         {"SHOEBOX_MATERIAL_TRANSPARENT"=0;
                              "SHOEBOX_MATERIAL_ACOUSTIC_CEILING_TILES"=1;
                       "SHOEBOX_MATERIAL_BRICK_BARE"=2;
                       "SHOEBOX_MATERIAL_BRICK_PAINTED"=3;
                       "SHOEBOX_MATERIAL_CONCRETE_BLOCK_COARSE"=4;
                       "SHOEBOX_MATERIAL_CONCRETE_BLOCK_PAINTED"=5;
                       "SHOEBOX_MATERIAL_CURTAIN_HEAVY"=6;
                       "SHOEBOX_MATERIAL_FIBER_GLASS_INSULATION"=7;
                       "SHOEBOX_MATERIAL_GLASS_THICK"=8;
                       "SHOEBOX_MATERIAL_GLASS_THIN"=9;
                       "SHOEBOX_MATERIAL_GRASS"=10;
                       "SHOEBOX_MATERIAL_GRASS"=11;
                       "SHOEBOX_MATERIAL_LINOLEUM_ON_CONCRETE"=12;
                       "SHOEBOX_MATERIAL_MARBLE"=13;
                       "SHOEBOX_MATERIAL_METAL"=14;
                       "SHOEBOX_MATERIAL_PARQUET_ON_CONCRETE"=15;
                       "SHOEBOX_MATERIAL_PLASTER_ROUGH"=16;
                       "SHOEBOX_MATERIAL_PLASTER_SMOOTH"=17;
                       "SHOEBOX_MATERIAL_PLYWOOD_PANEL"=18;
                       "SHOEBOX_MATERIAL_POLISHED_CONCRETE_OR_TILE"=19;
                       "SHOEBOX_MATERIAL_SHEET_ROCK"=20;
                       "SHOEBOX_MATERIAL_WATER_OR_ICE_SURFACE"=21;
                       "SHOEBOX_MATERIAL_WOOD_ON_CEILING"=22;
                       "SHOEBOX_MATERIAL_WOOD_PANEL"=23 }*/

   uint32_t left_wall;

   /**< @h2xmle_description {Specifies the material of left wall}
      @h2xmle_default     {SHOEBOX_MATERIAL_TRANSPARENT}
      @h2xmle_rangeList         {"SHOEBOX_MATERIAL_TRANSPARENT"=0;
                              "SHOEBOX_MATERIAL_ACOUSTIC_CEILING_TILES"=1;
                       "SHOEBOX_MATERIAL_BRICK_BARE"=2;
                       "SHOEBOX_MATERIAL_BRICK_PAINTED"=3;
                       "SHOEBOX_MATERIAL_CONCRETE_BLOCK_COARSE"=4;
                       "SHOEBOX_MATERIAL_CONCRETE_BLOCK_PAINTED"=5;
                       "SHOEBOX_MATERIAL_CURTAIN_HEAVY"=6;
                       "SHOEBOX_MATERIAL_FIBER_GLASS_INSULATION"=7;
                       "SHOEBOX_MATERIAL_GLASS_THICK"=8;
                       "SHOEBOX_MATERIAL_GLASS_THIN"=9;
                       "SHOEBOX_MATERIAL_GRASS"=10;
                       "SHOEBOX_MATERIAL_GRASS"=11;
                       "SHOEBOX_MATERIAL_LINOLEUM_ON_CONCRETE"=12;
                       "SHOEBOX_MATERIAL_MARBLE"=13;
                       "SHOEBOX_MATERIAL_METAL"=14;
                       "SHOEBOX_MATERIAL_PARQUET_ON_CONCRETE"=15;
                       "SHOEBOX_MATERIAL_PLASTER_ROUGH"=16;
                       "SHOEBOX_MATERIAL_PLASTER_SMOOTH"=17;
                       "SHOEBOX_MATERIAL_PLYWOOD_PANEL"=18;
                       "SHOEBOX_MATERIAL_POLISHED_CONCRETE_OR_TILE"=19;
                       "SHOEBOX_MATERIAL_SHEET_ROCK"=20;
                       "SHOEBOX_MATERIAL_WATER_OR_ICE_SURFACE"=21;
                       "SHOEBOX_MATERIAL_WOOD_ON_CEILING"=22;
                       "SHOEBOX_MATERIAL_WOOD_PANEL"=23 }*/

   uint32_t right_wall;

   /**< @h2xmle_description {Specifies the material of right wall}
     @h2xmle_default     {SHOEBOX_MATERIAL_TRANSPARENT}
     @h2xmle_rangeList         {"SHOEBOX_MATERIAL_TRANSPARENT"=0;
                             "SHOEBOX_MATERIAL_ACOUSTIC_CEILING_TILES"=1;
                      "SHOEBOX_MATERIAL_BRICK_BARE"=2;
                      "SHOEBOX_MATERIAL_BRICK_PAINTED"=3;
                      "SHOEBOX_MATERIAL_CONCRETE_BLOCK_COARSE"=4;
                      "SHOEBOX_MATERIAL_CONCRETE_BLOCK_PAINTED"=5;
                      "SHOEBOX_MATERIAL_CURTAIN_HEAVY"=6;
                      "SHOEBOX_MATERIAL_FIBER_GLASS_INSULATION"=7;
                      "SHOEBOX_MATERIAL_GLASS_THICK"=8;
                      "SHOEBOX_MATERIAL_GLASS_THIN"=9;
                      "SHOEBOX_MATERIAL_GRASS"=10;
                      "SHOEBOX_MATERIAL_GRASS"=11;
                      "SHOEBOX_MATERIAL_LINOLEUM_ON_CONCRETE"=12;
                      "SHOEBOX_MATERIAL_MARBLE"=13;
                      "SHOEBOX_MATERIAL_METAL"=14;
                      "SHOEBOX_MATERIAL_PARQUET_ON_CONCRETE"=15;
                      "SHOEBOX_MATERIAL_PLASTER_ROUGH"=16;
                      "SHOEBOX_MATERIAL_PLASTER_SMOOTH"=17;
                      "SHOEBOX_MATERIAL_PLYWOOD_PANEL"=18;
                      "SHOEBOX_MATERIAL_POLISHED_CONCRETE_OR_TILE"=19;
                      "SHOEBOX_MATERIAL_SHEET_ROCK"=20;
                      "SHOEBOX_MATERIAL_WATER_OR_ICE_SURFACE"=21;
                      "SHOEBOX_MATERIAL_WOOD_ON_CEILING"=22;
                      "SHOEBOX_MATERIAL_WOOD_PANEL"=23 }*/

   uint32_t ceiling;

   /**< @h2xmle_description {Specifies the material of ceiling}
      @h2xmle_default     {SHOEBOX_MATERIAL_TRANSPARENT}
      @h2xmle_rangeList         {"SHOEBOX_MATERIAL_TRANSPARENT"=0;
                              "SHOEBOX_MATERIAL_ACOUSTIC_CEILING_TILES"=1;
                       "SHOEBOX_MATERIAL_BRICK_BARE"=2;
                       "SHOEBOX_MATERIAL_BRICK_PAINTED"=3;
                       "SHOEBOX_MATERIAL_CONCRETE_BLOCK_COARSE"=4;
                       "SHOEBOX_MATERIAL_CONCRETE_BLOCK_PAINTED"=5;
                       "SHOEBOX_MATERIAL_CURTAIN_HEAVY"=6;
                       "SHOEBOX_MATERIAL_FIBER_GLASS_INSULATION"=7;
                       "SHOEBOX_MATERIAL_GLASS_THICK"=8;
                       "SHOEBOX_MATERIAL_GLASS_THIN"=9;
                       "SHOEBOX_MATERIAL_GRASS"=10;
                       "SHOEBOX_MATERIAL_GRASS"=11;
                       "SHOEBOX_MATERIAL_LINOLEUM_ON_CONCRETE"=12;
                       "SHOEBOX_MATERIAL_MARBLE"=13;
                       "SHOEBOX_MATERIAL_METAL"=14;
                       "SHOEBOX_MATERIAL_PARQUET_ON_CONCRETE"=15;
                       "SHOEBOX_MATERIAL_PLASTER_ROUGH"=16;
                       "SHOEBOX_MATERIAL_PLASTER_SMOOTH"=17;
                       "SHOEBOX_MATERIAL_PLYWOOD_PANEL"=18;
                       "SHOEBOX_MATERIAL_POLISHED_CONCRETE_OR_TILE"=19;
                       "SHOEBOX_MATERIAL_SHEET_ROCK"=20;
                       "SHOEBOX_MATERIAL_WATER_OR_ICE_SURFACE"=21;
                       "SHOEBOX_MATERIAL_WOOD_ON_CEILING"=22;
                       "SHOEBOX_MATERIAL_WOOD_PANEL"=23 }*/

   uint32_t floor;

   /**< @h2xmle_description {Specifies the material of floor}
      @h2xmle_default     {SHOEBOX_MATERIAL_TRANSPARENT}
      @h2xmle_rangeList         {"SHOEBOX_MATERIAL_TRANSPARENT"=0;
                              "SHOEBOX_MATERIAL_ACOUSTIC_CEILING_TILES"=1;
                       "SHOEBOX_MATERIAL_BRICK_BARE"=2;
                       "SHOEBOX_MATERIAL_BRICK_PAINTED"=3;
                       "SHOEBOX_MATERIAL_CONCRETE_BLOCK_COARSE"=4;
                       "SHOEBOX_MATERIAL_CONCRETE_BLOCK_PAINTED"=5;
                       "SHOEBOX_MATERIAL_CURTAIN_HEAVY"=6;
                       "SHOEBOX_MATERIAL_FIBER_GLASS_INSULATION"=7;
                       "SHOEBOX_MATERIAL_GLASS_THICK"=8;
                       "SHOEBOX_MATERIAL_GLASS_THIN"=9;
                       "SHOEBOX_MATERIAL_GRASS"=10;
                       "SHOEBOX_MATERIAL_GRASS"=11;
                       "SHOEBOX_MATERIAL_LINOLEUM_ON_CONCRETE"=12;
                       "SHOEBOX_MATERIAL_MARBLE"=13;
                       "SHOEBOX_MATERIAL_METAL"=14;
                       "SHOEBOX_MATERIAL_PARQUET_ON_CONCRETE"=15;
                       "SHOEBOX_MATERIAL_PLASTER_ROUGH"=16;
                       "SHOEBOX_MATERIAL_PLASTER_SMOOTH"=17;
                       "SHOEBOX_MATERIAL_PLYWOOD_PANEL"=18;
                       "SHOEBOX_MATERIAL_POLISHED_CONCRETE_OR_TILE"=19;
                       "SHOEBOX_MATERIAL_SHEET_ROCK"=20;
                       "SHOEBOX_MATERIAL_WATER_OR_ICE_SURFACE"=21;
                       "SHOEBOX_MATERIAL_WOOD_ON_CEILING"=22;
                       "SHOEBOX_MATERIAL_WOOD_PANEL"=23 } */
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;

#define PARAM_ID_SHOEBOX_GAIN 0x08001120

/* Structure for the gain parameter of Shoebox module. */
typedef struct shoebox_gain_t shoebox_gain_t;
/** @h2xmlp_parameter   {"PARAM_ID_SHOEBOX_GAIN", PARAM_ID_SHOEBOX_GAIN}
    @h2xmlp_description {Payload of the PARAM_ID_SHOEBOX_GAIN parameter used by the
 Shoebox module.}
  */

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"

/* Payload of the PARAM_ID_SHOEBOX_GAIN parameter used by the
 Shoebox module.
 */
struct shoebox_gain_t
{
   uint32_t reverb_gain;
   /**< @h2xmle_description  {Specifies Gain adjustment to the environment reverb effect}
        @h2xmle_default      {65536}
        @h2xmle_range        {0..4294967295}
      @h2xmle_dataFormat   {Q16}  */
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;

#define PARAM_ID_SHOEBOX_DECAY_TIME_ADJUST 0x080010FB

/* Structure for the decay time adjust parameter of Shoebox module. */
typedef struct shoebox_decay_time_adjust_t shoebox_decay_time_adjust_t;
/** @h2xmlp_parameter   {"PARAM_ID_SHOEBOX_DECAY_TIME_ADJUST", PARAM_ID_SHOEBOX_DECAY_TIME_ADJUST}
    @h2xmlp_description {Payload of the PARAM_ID_SHOEBOX_DECAY_TIME_ADJUST parameter used by the Shoebox module.}
   */
#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"

/* Payload of the PARAM_ID_SHOEBOX_DECAY_TIME_ADJUST parameter used by the
 Shoebox module.
 */
struct shoebox_decay_time_adjust_t
{
   uint32_t decay_time_adjust;
   /**< @h2xmle_description  {Specifies Adjustment to RT60 time.}
        @h2xmle_default      {65536}
        @h2xmle_range        {0..4294967295}
      @h2xmle_dataFormat   {Q16}  */
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;

#define PARAM_ID_SHOEBOX_BRIGHTNESS_ADJUST 0x080010FC

/* Structure for the brightness adjust parameter of Shoebox module. */
typedef struct shoebox_brightness_adjust_t shoebox_brightness_adjust_t;
/** @h2xmlp_parameter   {"PARAM_ID_SHOEBOX_BRIGHTNESS_ADJUST", PARAM_ID_SHOEBOX_BRIGHTNESS_ADJUST}
    @h2xmlp_description {Payload of the PARAM_ID_SHOEBOX_BRIGHTNESS_ADJUST parameter used by the
 Shoebox module.}
  */
#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"

/* Payload of the PARAM_ID_SHOEBOX_BRIGHTNESS_ADJUST parameter used by the
 Shoebox module.
 */
struct shoebox_brightness_adjust_t
{
   uint32_t brightness_adjust;
   /**< @h2xmle_description  {Specifies The balance between the low and high frequency components of the Shoebox
      reverberation.}
        @h2xmle_default      {65536}
        @h2xmle_range        {0..4294967295}
      @h2xmle_dataFormat   {Q16}  */
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;

/**
   @h2xml_Select					{lib_version_t}
    @h2xmlm_InsertParameter
*/

/**  @}                   <-- End of the Module -->*/
#endif
