#ifndef P_EQ_API_H
#define P_EQ_API_H
/*==============================================================================
  @file popless_equalizer_api.h
  @brief This file contains POPLESS EQUALIZER API
==============================================================================*/

/*=======================================================================
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear
=========================================================================*/

#include "ar_defs.h"
#include "module_cmn_api.h"
#include "imcl_p_eq_vol_api.h"

/** @h2xml_title1           {POPLESS EQUALIZER Module API}
    @h2xml_title_agile_rev  {POPLESS EQUALIZER Module API}
    @h2xml_title_date       {May 22, 2019} */

/*==============================================================================
   Structures
==============================================================================*/
//TODO: rename it to PEQ
/** @ingroup ar_spf_mod_peq_macros
    User-customized equalizer preset (with audio effects specified
    individually). */
#define  PEQ_PRESET_USER_CUSTOM     (-1)

/** @ingroup ar_spf_mod_peq_macros
    Equalizer preset for no audio effects. */
#define  PEQ_PRESET_BLANK           0

/** @ingroup ar_spf_mod_peq_macros
    Equalizer preset for audio effects like a club. */
#define  PEQ_PRESET_CLUB            1

/** @ingroup ar_spf_mod_peq_macros
    Equalizer preset for audio effects like a dance. */
#define  PEQ_PRESET_DANCE           2

/** @ingroup ar_spf_mod_peq_macros
    Internal sound library equalizer preset for full bass audio effects. */
#define  PEQ_PRESET_FULLBASS        3

/** @ingroup ar_spf_mod_peq_macros
    Equalizer preset for bass and treble audio effects. */
#define  PEQ_PRESET_BASSTREBLE      4

/** @ingroup ar_spf_mod_peq_macros
    Equalizer preset for full treble audio effects. */
#define  PEQ_PRESET_FULLTREBLE      5

/** @ingroup ar_spf_mod_peq_macros
    Equalizer preset for audio effects that are suitable for playback through
    laptop/phone speakers. */
#define  PEQ_PRESET_LAPTOP          6

/** @ingroup ar_spf_mod_peq_macros
    Equalizer preset for audio effects that sound like a large hall. */
#define  PEQ_PRESET_LHALL           7

/** @ingroup ar_spf_mod_peq_macros
    Equalizer preset for audio effects that sound like a live performance. */
#define  PEQ_PRESET_LIVE            8

/** @ingroup ar_spf_mod_peq_macros
    Equalizer preset for audio effects that sound like a party. */
#define  PEQ_PRESET_PARTY           9

/** @ingroup ar_spf_mod_peq_macros
    Equalizer preset for pop audio effects. */
#define  PEQ_PRESET_POP             10

/** @ingroup ar_spf_mod_peq_macros
    Equalizer preset for reggae audio effects. */
#define  PEQ_PRESET_REGGAE          11

/** @ingroup ar_spf_mod_peq_macros
    Equalizer preset for rock audio effects. */
#define  PEQ_PRESET_ROCK            12

/** @ingroup ar_spf_mod_peq_macros
    Equalizer preset for Ska audio effects. */
#define  PEQ_PRESET_SKA             13

/** @ingroup ar_spf_mod_peq_macros
    Equalizer preset for soft pop audio effects. */
#define  PEQ_PRESET_SOFT            14

/** @ingroup ar_spf_mod_peq_macros
    Equalizer preset for soft rock audio effects. */
#define  PEQ_PRESET_SOFTROCK        15

/** @ingroup ar_spf_mod_peq_macros
    Equalizer preset for techno audio effects. */
#define  PEQ_PRESET_TECHNO          16

/** @ingroup ar_spf_mod_peq_macros
    User-customized equalizer preset (with audio effects specified
    individually) (External sound library). */
#define  PEQ_PRESET_USER_CUSTOM_AUDIO_FX    18

/** @ingroup ar_spf_mod_peq_macros
    Equalizer preset for normal (generic) audio effects (External sound library). */
#define  PEQ_PRESET_NORMAL_AUDIO_FX         19

/** @ingroup ar_spf_mod_peq_macros
    Equalizer preset for classical audio effects (External sound library). */
#define  PEQ_PRESET_CLASSICAL_AUDIO_FX      20

/** @ingroup ar_spf_mod_peq_macros
    Equalizer preset for dance audio effects (External sound library). */
#define  PEQ_PRESET_DANCE_AUDIO_FX          21

/** @ingroup ar_spf_mod_peq_macros
    Equalizer preset for no audio effects (External sound library). */
#define  PEQ_PRESET_FLAT_AUDIO_FX           22

/** @ingroup ar_spf_mod_peq_macros
    Equalizer preset for folk audio effects (External sound library). */
#define  PEQ_PRESET_FOLK_AUDIO_FX           23

/** @ingroup ar_spf_mod_peq_macros
    Equalizer preset for heavy metal audio effects (External sound library). */
#define  PEQ_PRESET_HEAVYMETAL_AUDIO_FX     24

/** @ingroup ar_spf_mod_peq_macros
    Equalizer preset for hip hop audio effects (External sound library). */
#define  PEQ_PRESET_HIPHOP_AUDIO_FX         25

/** @ingroup ar_spf_mod_peq_macros
    Equalizer preset for jazz audio effects (External sound library). */
#define  PEQ_PRESET_JAZZ_AUDIO_FX           26

/** @ingroup ar_spf_mod_peq_macros
    Equalizer preset for pop audio effects (External sound library). */
#define  PEQ_PRESET_POP_AUDIO_FX            27

/** @ingroup ar_spf_mod_peq_macros
    Equalizer preset for rock audio effects (External sound library). */
#define  PEQ_PRESET_ROCK_AUDIO_FX           28

/** @ingroup ar_spf_mod_peq_macros
    ID of the Popless Equalizer Configuration parameter used by MODULE_ID_POPLESS_EQUALIZER. */
#define PARAM_ID_EQ_CONFIG                             0x0800110C

//TODO: change all to peq instead of eq
typedef struct param_id_eq_per_band_config_t param_id_eq_per_band_config_t;

/** @ingroup ar_spf_mod_peq_macros
    Configures the module. */

/** @h2xmlp_parameter   {"PARAM_ID_EQ_CONFIG", PARAM_ID_EQ_CONFIG}
    @h2xmlp_description {Configures the module}  */

//TODO: check if info is available in lib for description
/** @h2xmlp_subStruct */
#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
struct param_id_eq_per_band_config_t
{
  uint32_t filter_type;
  /**< Type of equalizer filter in the sub-band. */

    /**< @h2xmle_description {Type of equalizer filter in the sub-band.\n }
         @h2xmle_default     {0x05}
         @h2xmle_policy       {Basic}
         @h2xmle_rangeList   {"EQ_TYPE_NONE"=0; "EQ_BASS_BOOST"=1; "EQ_BASS_CUT" = 2;
                              "EQ_TREBLE_BOOST" = 3; "EQ_TREBLE_CUT" =4; "EQ_BAND_BOOST" =5; "EQ_BAND_CUT"=6} */
  uint32_t freq_millihertz;
  /**< Center or cutoff frequency of the sub-band filter in millihertz.
       Supported values: 30 to fs/2 (Nyquist rate), depending on the sampling rate of the input audio signal. */

    /**< @h2xmle_description {Center or cutoff frequency of the sub-band filter in millihertz.
                              Supported values: 30 to fs/2 (Nyquist rate), depending on the sampling rate of the input audio signal\n }
         @h2xmle_default     {0xEA60}
         @h2xmle_range       {0x1E..0x5B8D800}
         @h2xmle_policy       {Basic} */

  int32_t gain_millibels;
  /**< Initial gain of the sub-band filter.
       Supported values: +1500 to -1500 mdB in 100 mdB increments. */

    /**< @h2xmle_description {Initial gain of the sub-band filter.
                              Supported values: +1500 to -1500 mdB in 100 mdB increments.\n }
         @h2xmle_default     {0x0}
         @h2xmle_rangeList   {" 1500 mdB " = 1500 ; 
                              " 1400 mdB " = 1400 ; 
                              " 1300 mdB " = 1300 ; 
                              " 1200 mdB " = 1200 ; 
                              " 1100 mdB " = 1100 ; 
                              " 1000 mdB " = 1000 ; 
                              " 900 mdB " = 900 ; 
                              " 800 mdB " = 800 ; 
                              " 700 mdB " = 700 ; 
                              " 600 mdB " = 600 ; 
                              " 500 mdB " = 500 ; 
                              " 400 mdB " = 400 ; 
                              " 300 mdB " = 300 ; 
                              " 200 mdB " = 200 ; 
                              " 100 mdB " = 100 ; 
                              " 0 mdB " = 0 ; 
                              " -100 mdB " = -100 ; 
                              " -200 mdB " = -200 ; 
                              " -300 mdB " = -300 ; 
                              " -400 mdB " = -400 ; 
                              " -500 mdB " = -500 ; 
                              " -600 mdB " = -600 ; 
                              " -700 mdB " = -700 ; 
                              " -800 mdB " = -800 ; 
                              " -900 mdB " = -900 ; 
                              " -1000 mdB " = -1000 ; 
                              " -1100 mdB " = -1100 ; 
                              " -1200 mdB " = -1200 ; 
                              " -1300 mdB " = -1300 ; 
                              " -1400 mdB " = -1400 ; 
                              " -1500 mdB " = -1500 }
         @h2xmle_policy       {Basic} */

  uint32_t quality_factor;
  /**< Sub-band filter quality factor expressed as a Q8 number (a
       fixed-point number with a quality factor of 8). For example,3000/(2^8). */

    /**< @h2xmle_description {Sub-band filter quality factor expressed as a Q8 number (a
                              fixed-point number with a quality factor of 8). For example,
                              3000/(2^8)\n }
         @h2xmle_default     {0x0100}
         @h2xmle_rangeList   {"100" =100 ;"105"=105; "110"=110; "115"=115; "120"=120; "135"=135; 
                              "155"=155; "210"=210; "256"=256}
         @h2xmle_policy       {Basic} */
//TODO: greater than 1 - check in the library
  uint32_t band_idx;
  /**< Index of the sub-band filter.
       Supported values: 0 to num_bands - 1 (num_bands is specified in param_id_eq_config_t). */

    /**< @h2xmle_description {Index of the sub-band filter.\n
                              Supported values: 0 to num_bands - 1 (num_bands is specified in
                              param_id_eq_config_t)}
         @h2xmle_default     {0x00}
         @h2xmle_range       {0...11}
         @h2xmle_policy       {Basic} */
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;


typedef struct param_id_eq_config_t  param_id_eq_config_t;

/** @h2xmlp_parameter   {"PARAM_ID_EQ_CONFIG", PARAM_ID_EQ_CONFIG}
    @h2xmlp_description {Configures the popless equalizer module.\n
                         This parameter is only used for Set Parameter calls.\n
                         Following this structure are num_bands of param_id_eq_per_band_config_t
                         (see Per-band equalizer parameters).\n
                         The length is dependent on the num_bands value.\n
                         For predefined internal sound library and predefined external sound library equalizers, the sequence of
                         per-band parameters is not required, and num_bands must be set to 0.\n }
    @h2xmlx_expandStructs  {false}
    @h2xmlp_toolPolicy              {Calibration}
    */

/** @ingroup ar_spf_mod_peq_macros
    Configures the popless equalizer module.
    This parameter is only used for Set Parameter calls.
    Following this structure are num_bands of param_id_eq_per_band_config_t
    (see Per-band equalizer parameters).
    The length is dependent on the num_bands value.
    For predefined internal sound library and predefined external sound library equalizers, the sequence of
    per-band parameters is not required, and num_bands must be set to 0.
*/

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"

struct param_id_eq_config_t
{
   int32_t eq_pregain;
   /**< Gain in Q27 before any equalization processing. */

   /**< @h2xmle_description {Gain in Q27 before any equalization processing.\n}
        @h2xmle_default     {0x08000000}
        @h2xmle_policy       {Basic} */

   int32_t preset_id;
   /**< Specifies either the user-customized equalizers or two sets of
        equalizers predefined by internal sound library and external sound library, respectively. \n
        @values
        -1 -- Custom equalizer that meets internal sound library requirements
        - 0 to 16 -- Equalizers that are predefined by internal sound library
        - 18 -- Custom equalizer that meets external sound library requirements
        - 19 to 28 -- Equalizers that are predefined by external sound library. */

   /**< @h2xmle_description  { Specifies either the user-customized equalizers or two sets of
                               equalizers predefined by internal sound library and external sound library, respectively. \n
                               @values\n
                               - -1 -- Custom equalizer that meets internal sound library requirements\n
                               - 0 to 16 -- Equalizers that are predefined by internal sound library\n
                               - 18 -- Custom equalizer that meets external sound library requirements\n
                               - 19 to 28 -- Equalizers that are predefined by external sound library \n }
        @h2xmle_range        {-1..28}
        @h2xmle_default      {18}
        @h2xmle_policy       {Basic}*/

   uint32_t num_bands;
   /**< Specifies number of subbands for equalization when a custom preset ID is
        selected in the preset_id field. \n
        @values
        - If preset_id = -1 -- 1 to 12
        - If preset_id = 18 -- 5
        - All other preset_id settings -- 0. */

   /**< @h2xmle_description  { Specifies number of subbands for equalization when a custom preset ID is
                               selected in the preset_id field.\n
                               @values\n
                               - If preset_id = -1 -- 1 to 12\n
                               - If preset_id = 18 -- 5\n
                               - All other preset_id settings -- 0\n }
        @h2xmle_range        {0..12}
        @h2xmle_default      {1}
        @h2xmle_policy       {Basic} */

   param_id_eq_per_band_config_t band_eq_params[0];
   /**< Per band configuration. */

   /**< @h2xmle_description  {per band config.}
        @h2xmle_variableArraySize {num_bands} */

}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;


#define PARAM_ID_EQ_NUM_BANDS                          0x0800110D

typedef struct param_id_eq_num_bands_t param_id_eq_num_bands_t;
/** @h2xmlp_parameter   {"PARAM_ID_EQ_NUM_BANDS", PARAM_ID_EQ_NUM_BANDS}
    @h2xmlp_description {Specifies number of subbands in the current equalizer filter.\n
                         This parameter is used only for Get Parameter calls.\n}
    @h2xmlp_toolPolicy              {RTC_READONLY} */

/** @ingroup ar_spf_mod_peq_macros
    Specifies number of subbands in the current equalizer filter.
    This parameter is used only for Get Parameter calls
*/

#include "spf_begin_pack.h"
struct param_id_eq_num_bands_t
{
   uint32_t num_bands;
   /**< Specifies Number of subbands in the current equalizer filter. \n
        @values
        - 1 to 12 -- When equalizers are compliant to or predefined by internal sound library
        - 5 -- When equalizers are compliant to or predefined by external sound library. */

   /**< @h2xmle_description  { Specifies Number of subbands in the current equalizer filter.\n
                               Supported values\n
                               - 1 to 12 -- When equalizers are compliant to or predefined by internal sound library\n
                               - 5 -- When equalizers are compliant to or predefined by external sound library\n }
        @h2xmle_range        {1..12}
        @h2xmle_default      {5}
        @h2xmle_readOnly  {true}  */

}
#include "spf_end_pack.h"
;


#define PARAM_ID_EQ_BAND_LEVELS                        0x0800110E

typedef struct param_id_eq_band_levels_t param_id_eq_band_levels_t;
/** @h2xmlp_parameter   {"PARAM_ID_EQ_BAND_LEVELS", PARAM_ID_EQ_BAND_LEVELS}
    @h2xmlp_description {Specifies number of subbands in the current equalizer filter.\n
                         This parameter is used only for Get Parameter calls.\n}
    @h2xmlp_toolPolicy              {RTC_READONLY} */

/** @ingroup ar_spf_mod_peq_macros
    Specifies number of subbands in the current equalizer filter.
    This parameter is used only for Get Parameter calls.
*/


#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"

struct param_id_eq_band_levels_t
{
   uint32_t num_bands;
   /**< Specifies Number of subbands in the current equalizer filter.
        For Get Parameter calls only, the library returns the value. \n
        @values
        - 1 to 12 -- When equalizers are compliant to or predefined by internal sound library
        - 5 -- When equalizers are compliant to or predefined by external sound library. */

   /**< @h2xmle_description  { Specifies Number of subbands in the current equalizer filter.\n
                               For Get Parameter calls only, the library returns the value.\n
                               Supported values\n
                               - 1 to 12 -- When equalizers are compliant to or predefined by internal sound library\n
                               - 5 -- When equalizers are compliant to or predefined by external sound library\n }
        @h2xmle_range        {1..12}
        @h2xmle_default      {5}
        @h2xmle_readOnly  {true}   */


   int32_t band_levels[12];
   /**< Array of gains (in millibels) of each sub-band filter.
        For Get Parameter calls only, the library returns the values.

        The meaningful contents in the array depend on the num_bands parameter.
        The following example contains valid values returned by the library:

        band_levels[0] - band_levels[num_bands-1]. */

   /**< @h2xmle_description { Array of gains (in millibels) of each sub-band filter.\n
                              For Get Parameter calls only, the library returns the values.\n

                              The meaningful contents in the array depend on the num_bands parameter.\n
                              The following example contains valid values returned by the library:\n

                              band_levels[0] - band_levels[num_bands-1] }
        @h2xmle_readOnly  {true}  */

}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;

#define PARAM_ID_EQ_BAND_LEVEL_RANGE                   0x0800110F

typedef struct param_id_eq_band_level_range_t   param_id_eq_band_level_range_t;
/** @h2xmlp_parameter   {"PARAM_ID_EQ_BAND_LEVEL_RANGE", PARAM_ID_EQ_BAND_LEVEL_RANGE}
    @h2xmlp_description {This parameter is used only for Get Parameter calls.\n}
    @h2xmlp_toolPolicy              {RTC_READONLY} */

/** @ingroup ar_spf_mod_peq_macros
    This parameter is used only for Get Parameter calls.
*/

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"

struct param_id_eq_band_level_range_t
{
   int32_t min_level_millibels;
   /**< Specifies minimum gain of sub-band equalizer filters in the current equalizer
        configuration.
        For Get Parameter calls only, the library returns the value.. */

    /**< @h2xmle_description  {Specifies minimum gain of sub-band equalizer filters in the current equalizer
                               configuration.\n For Get Parameter calls only, the library returns
                               the value.\n}

         @h2xmle_range {-1500..-1500}
         @h2xmle_default      {-1500}
         @h2xmle_readOnly  {true}  */

   int32_t max_level_millibels;
   /**< Specifies maximun gain of sub-band equalizer filters in the current equalizer
        configuration.
        For Get Parameter calls only, the library returns the value. */

   /**< @h2xmle_description  {Specifies maximun gain of sub-band equalizer filters in the current equalizer
                              configuration.\n For Get Parameter calls only, the library returns
                              the value.\n}

        @h2xmle_range  {1500..1500}
        @h2xmle_default      {1500}
        @h2xmle_readOnly  {true}  */

}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;


#define PARAM_ID_EQ_BAND_FREQS                         0x08001110

typedef struct param_id_eq_band_freqs_t param_id_eq_band_freqs_t;
/** @h2xmlp_parameter   {"PARAM_ID_EQ_BAND_FREQS", PARAM_ID_EQ_BAND_FREQS}
    @h2xmlp_description {This parameter is used only for Get Parameter calls.\n}
    @h2xmlp_toolPolicy              {RTC_READONLY}*/

/** @ingroup ar_spf_mod_peq_macros
    This parameter is used only for Get Parameter calls.
*/

#include "spf_begin_pack.h"

struct param_id_eq_band_freqs_t
{
   uint32_t num_bands;
   /**< Specifies number of sub-band equalizer filters in the current equalizer
        configuration.
        For Get Parameter calls only, the library returns the value. \n
        @values
        - 1 to 12 -- When equalizers are compliant to or predefined by internal sound library
        - 5 -- When equalizers are compliant to or predefined by external sound library. */

   /**< @h2xmle_description  { Specifies number of sub-band equalizer filters in the current equalizer
                               configuration.\n For Get Parameter calls only, the library returns
                               the value.\n
                               Supported values\n
                               - 1 to 12 -- When equalizers are compliant to or predefined by internal sound library\n
                               - 5 -- When equalizers are compliant to or predefined by external sound library\n }

        @h2xmle_range        {1..12}
        @h2xmle_default      {5}
        @h2xmle_readOnly  {true}   */


   uint32_t band_freqs[12];
   /**< Array of center or cutoff frequencies of each sub-band filter, in
        millihertz.
        For Get parameter calls only, the library returns the values.

        The meaningful contents in the array depend on the num_bands parameter.
        The following example contains valid values returned by the library:
        indent band_freqs[0] - band_freqs[num_bands-1]. */

    /**< @h2xmle_description { Array of center or cutoff frequencies of each sub-band filter, in
                               millihertz.\n
                               For Get parameter calls only, the library returns the values.\n

                               The meaningful contents in the array depend on the num_bands parameter.\n
                               The following example contains valid values returned by the library:
                               indent band_freqs[0] - band_freqs[num_bands-1] \n}
         @h2xmle_readOnly  {true}  */
}
#include "spf_end_pack.h"
;


#define PARAM_ID_EQ_SINGLE_BAND_FREQ_RANGE             0x08001111

typedef struct param_id_eq_single_band_freq_range_t param_id_eq_single_band_freq_range_t;
/** @h2xmlp_parameter   {"PARAM_ID_EQ_SINGLE_BAND_FREQ_RANGE", PARAM_ID_EQ_SINGLE_BAND_FREQ_RANGE}
    @h2xmlp_description {This parameter is used only for Get Parameter calls.\n}
    @h2xmlp_toolPolicy              {RTC_READONLY}*/

/** @ingroup ar_spf_mod_peq_macros
    This parameter is used only for Get Parameter calls.
*/


#include "spf_begin_pack.h"
struct param_id_eq_single_band_freq_range_t
{
   uint32_t min_freq_millihertz;
   /**< Specifies lower frequency boundary of the sub-band equalizer filter with the
        band_index provided by a previous Set Parameter call with
        PARAM_ID_EQ_BAND_INDEX.
        For Get Parameter calls only, the library returns the value.
        If the band index is not provided by a previous Set Parameter call,
        the parameters of the first sub-band with band_index = 0 are
        returned.\n Supported values  :Any value in the range of 0 to sample_rate/2. */

   /**< @h2xmle_description {Specifies lower frequency boundary of the sub-band equalizer filter with the
                             band_index provided by a previous Set Parameter call with
                             PARAM_ID_EQ_BAND_INDEX.\n
                             For Get Parameter calls only, the library returns the value.\n
                             If the band index is not provided by a previous Set Parameter call,
                             the parameters of the first sub-band with band_index = 0 are
                             returned.\n Supported values  :Any value in the range of 0 to sample_rate/2 \n}
        @h2xmle_readOnly  {true} */

   uint32_t max_freq_millihertz;
   /**< Specifies upper frequency boundary of the sub-band equalizer filter with the
        band_index provided by a previous Set Parameter call with
        PARAM_ID_EQ_BAND_INDEX.\n
        For Get Parameter calls only, the library returns the value.\n
        If the band index is not provided by a previous Set Parameter call,
        the parameters of the first sub-band with band_index = 0 are
        returned.\n
        @values
        Any value in the range of 0 to sample_rate/2. */

   /**< @h2xmle_description {Specifies upper frequency boundary of the sub-band equalizer filter with the
                             band_index provided by a previous Set Parameter call with
                             PARAM_ID_EQ_BAND_INDEX.\n
                             For Get Parameter calls only, the library returns the value.\n
                             If the band index is not provided by a previous Set Parameter call,
                             the parameters of the first sub-band with band_index = 0 are
                             returned.\n Supported values  :Any value in the range of 0 to sample_rate/2 \n}
        @h2xmle_readOnly  {true}  */

}
#include "spf_end_pack.h"
;

#define PARAM_ID_EQ_SINGLE_BAND_FREQ                   0x08001112

typedef struct param_id_eq_single_band_freq_t param_id_eq_single_band_freq_t;
/** @h2xmlp_parameter   {"PARAM_ID_EQ_SINGLE_BAND_FREQ",PARAM_ID_EQ_SINGLE_BAND_FREQ}
    @h2xmlp_description {Payload of the PARAM_ID_EQ_SINGLE_BAND_FREQ parameter used by the
                         Popless Equalizer module.\n}
    @h2xmlp_toolPolicy              {Calibration}*/

/** @ingroup ar_spf_mod_peq_macros
    Payload of the PARAM_ID_EQ_SINGLE_BAND_FREQ parameter used by the
    Popless Equalizer module.
*/


#include "spf_begin_pack.h"

struct param_id_eq_single_band_freq_t
{
   uint32_t freq_millihertz;
   /**< For Set Parameter calls only, center or cutoff frequency of the
        sub-band equalizer filter for which the band_index is requested in a
        subsequent Get Parameter call via PARAM_ID_EQ_BAND_INDEX.

        @values
        Any value in the range of 30 to sample_rate/2. */

    /**< @h2xmle_description {For Set Parameter calls only, center or cutoff frequency of the
                              sub-band equalizer filter for which the band_index is requested in a
                              subsequent Get Parameter call via PARAM_ID_EQ_BAND_INDEX.\n

                              Supported values : Any value in the range of 30 to sample_rate/2 \n }
         @h2xmle_default      {30}
         @h2xmle_range       {0x1E..0x5B8D800}
    */
}
#include "spf_end_pack.h"
;


#define PARAM_ID_EQ_BAND_INDEX                         0x08001113

typedef struct param_id_eq_band_index_t param_id_eq_band_index_t;
/** @h2xmlp_parameter   {"PARAM_ID_EQ_BAND_INDEX",PARAM_ID_EQ_BAND_INDEX}
    @h2xmlp_description {It is used for both Set and Get Parameter calls.\n} */

/** @ingroup ar_spf_mod_peq_macros
    It is used for both Set and Get Parameter calls.
*/

#include "spf_begin_pack.h"

struct param_id_eq_band_index_t
{
   uint32_t band_index;
   /**< Specifies index of each band. \n
        @values :
        - 0 to 11 -- When equalizers are compliant to or predefined by internal sound library
        - 0 to 4 -- When equalizers are compliant to or predefined by external sound library
        If PARAM_ID_EQ_BAND_INDEX is used in a Set Parameter call,
        this band_index is used to get the sub-band frequency range in the next
        Get Parameter call with PARAM_ID_EQ_SINGLE_BAND_FREQ_RANGE.

        If this parameter ID is used in a Get Parameter call, the band_index
        with the center or cutoff frequency is returned. The frequency is
        specified in the Set parameter of
        PARAM_ID_EQ_SINGLE_BAND_FREQ.

        If the center/cutoff frequency of the requested sub-band is not set
        before a Get Parameter call via PARAM_ID_EQ_BAND_INDEX, the
        default band_index of zero is returned. */

   /**< @h2xmle_description  {Specifies index of each band.\n
                              Supported values : \n
                              - 0 to 11 -- When equalizers are compliant to or predefined by internal sound library \n
                              - 0 to 4 -- When equalizers are compliant to or predefined by external sound library \n
                              If PARAM_ID_EQ_BAND_INDEX is used in a Set Parameter call,
                              this band_index is used to get the sub-band frequency range in the next
                              Get Parameter call with PARAM_ID_EQ_SINGLE_BAND_FREQ_RANGE.\n

                              If this parameter ID is used in a Get Parameter call, the band_index
                              with the center or cutoff frequency is returned. The frequency is
                              specified in the Set parameter of
                              PARAM_ID_EQ_SINGLE_BAND_FREQ.\n

                              If the center/cutoff frequency of the requested sub-band is not set
                              before a Get Parameter call via PARAM_ID_EQ_BAND_INDEX, the
                              default band_index of zero is returned.\n }


        @h2xmle_range {0..11}
        @h2xmle_default      {0x00}   */

}
#include "spf_end_pack.h"
;

#define PARAM_ID_EQ_PRESET_ID                          0x08001114

typedef struct param_id_eq_preset_id_t param_id_eq_preset_id_t;
/** @h2xmlp_parameter   {"PARAM_ID_EQ_PRESET_ID",PARAM_ID_EQ_PRESET_ID}
    @h2xmlp_description {This parameter is used only for Get Parameter calls.\n}
    @h2xmlp_toolPolicy  {RTC_READONLY}*/

/** @ingroup ar_spf_mod_peq_macros
    This parameter is used only for Get Parameter calls.
*/

#include "spf_begin_pack.h"

struct param_id_eq_preset_id_t
{
   int32_t preset_id;
   /**< Specifies preset ID of the current equalizer configuration.\n For Get Parameter calls only, the library returns
        the value. \n
        @values
        - -1 to 16 -- When equalizers are compliant to or predefined by internal sound library
        - 18 to 28 -- When equalizers are compliant to or predefined by external sound library. */

   /**< @h2xmle_description  { Specifies preset ID of the current equalizer configuration.\n For Get Parameter calls only, the library returns
                               the value.\n
                               Supported values\n
                               - -1 to 16 -- When equalizers are compliant to or predefined by internal sound library\n
                               - 18 to 28 -- When equalizers are compliant to or predefined by external sound library\n }

        @h2xmle_range        {-1..28}
        @h2xmle_readOnly  {true}   */

}
#include "spf_end_pack.h"
;

#define PARAM_ID_EQ_NUM_PRESETS                        0x08001115

typedef struct param_id_eq_num_presets_t param_id_eq_num_presets_t;
/** @h2xmlp_parameter   {"PARAM_ID_EQ_NUM_PRESETS", PARAM_ID_EQ_NUM_PRESETS}
    @h2xmlp_description {This parameter is used only for Get Parameter calls.\n}
    @h2xmlp_toolPolicy  {RTC_READONLY}*/

/** @ingroup ar_spf_mod_peq_macros
    This parameter is used only for Get Parameter calls.
*/

#include "spf_begin_pack.h"

struct param_id_eq_num_presets_t
{
   uint32_t num_presets;
   /**< Specifies Total number of supported presets in the current equalizer
        configuration.
        For Get Parameter calls only, the library returns the value. */

    /**< @h2xmle_description  { Specifies Total number of supported presets in the current equalizer
                                configuration.\n
                                For Get Parameter calls only, the library returns the value.\n }

         @h2xmle_rangeList        {"When equalizers are compliant to or predefined by internal sound library (17)"=17;"When equalizers are compliant to                                     or predefined by external sound library (10)"=10}
         @h2xmle_default      {10}
         @h2xmle_readOnly  {true}  */
}
#include "spf_end_pack.h"
;

#define PARAM_ID_EQ_PRESET_NAME                        0x08001116

typedef struct param_id_eq_preset_name_t param_id_eq_preset_name_t;
/** @h2xmlp_parameter   {"PARAM_ID_EQ_PRESET_NAME", PARAM_ID_EQ_PRESET_NAME}
    @h2xmlp_description {This parameter is used only for Get Parameter calls.\n}
    @h2xmlp_toolPolicy  {RTC_READONLY}*/

/** @ingroup ar_spf_mod_peq_macros
    This parameter is used only for Get Parameter calls.
*/

#include "spf_begin_pack.h"

struct param_id_eq_preset_name_t
{
   uint8_t preset_name[32];
   /**< Specifies Name of the current equalizer preset (in ASCII characters).
        For Get Parameter calls only, the library returns the value. */

   /**< @h2xmle_description  { Specifies Name of the current equalizer preset (in ASCII characters).\n
                               For Get Parameter calls only, the library returns the value\n }
        @h2xmle_readOnly  {true}  */

}
#include "spf_end_pack.h"
;

/*==============================================================================
   Constants
==============================================================================*/

/** @ingroup ar_spf_mod_peq_macros
    Input port ID of POPLESS_EQUALIZER module */
#define POPLESS_EQUALIZER_DATA_INPUT_PORT   0x2

/** @ingroup ar_spf_mod_peq_macros
    Output port ID of POPLESS_EQUALIZER module */
#define POPLESS_EQUALIZER_DATA_OUTPUT_PORT  0x1

/** @ingroup ar_spf_mod_peq_macros
    Max number of input ports of POPLESS_EQUALIZER module */
#define POPLESS_EQUALIZER_DATA_MAX_INPUT_PORTS 0x1

/** @ingroup ar_spf_mod_peq_macros
    Max number of output ports of POPLESS_EQUALIZER module */
#define POPLESS_EQUALIZER_DATA_MAX_OUTPUT_PORTS 0x1

/** @ingroup ar_spf_mod_peq_macros
    Stack size of POPLESS_EQUALIZERR module */
#define POPLESS_EQUALIZER_MODULE_STACK_SIZE 2000


/** @ingroup ar_spf_mod_peq_macros
    ID of the Popless Equalizer module.

    @subhead4{Supported parameter IDs}
    - #PARAM_ID_MODULE_ENABLE @lstsp1
    - #PARAM_ID_EQ_CONFIG @lstsp1
    - PARAM_ID_EQ_NUM_BANDS @lstsp1
    - PARAM_ID_EQ_BAND_LEVELS @lstsp1
    - PARAM_ID_EQ_BAND_LEVEL_RANGE @lstsp1
    - PARAM_ID_EQ_BAND_FREQS @lstsp1
    - PARAM_ID_EQ_SINGLE_BAND_FREQ_RANGE @lstsp1
    - PARAM_ID_EQ_SINGLE_BAND_FREQ @lstsp1
    - PARAM_ID_EQ_BAND_INDEX @lstsp1
    - PARAM_ID_EQ_PRESET_ID @lstsp1
    - PARAM_ID_EQ_NUM_PRESETS @lstsp1
    - PARAM_ID_EQ_PRESET_NAME

    @subhead4{Supported input media format ID}
    - Data Format          : FIXED_POINT @lstsp1
    - fmt_id               : MEDIA_FMT_ID_PCM @lstsp1
    - Sample Rates         : Standard sampling rates between 8000Hz and 192000Hz @lstsp1
    - Number of channels   : 1 to 8 @lstsp1
    - Channel type         : 1 to 63 @lstsp1
    - Bits per sample      : 16,32 @lstsp1
    - Q format             : 15,27 @lstsp1
    - Interleaving         : Deinterleaved Unpacked @lstsp1
    - Signed/unsigned      : Signed
*/

#define MODULE_ID_POPLESS_EQUALIZER                                    0x07001045

/**
    @h2xmlm_module       {"MODULE_ID_POPLESS_EQUALIZER",
                          MODULE_ID_POPLESS_EQUALIZER}
    @h2xmlm_displayName  {"Popless Equalizer"}
    @h2xmlm_modSearchKeys{effects, Audio}
    @h2xmlm_description  {ID of the Popless Equalizer module.\n
  \n
    . This module supports the following parameter IDs:\n
    .         PARAM_ID_MODULE_ENABLE\n
    .         PARAM_ID_EQ_CONFIG\n
    .         PARAM_ID_EQ_NUM_BANDS\n
    .         PARAM_ID_EQ_BAND_LEVELS\n
    .         PARAM_ID_EQ_BAND_LEVEL_RANGE\n
    .         PARAM_ID_EQ_BAND_FREQS\n
    .         PARAM_ID_EQ_SINGLE_BAND_FREQ_RANGE\n
    .         PARAM_ID_EQ_SINGLE_BAND_FREQ\n
    .         PARAM_ID_EQ_BAND_INDEX\n
    .         PARAM_ID_EQ_PRESET_ID\n
    .         PARAM_ID_EQ_NUM_PRESETS\n
    .         PARAM_ID_EQ_PRESET_NAME\n
    \n
    . All parameter IDs are device independent.\n\n
  (User-customized equalizer preset (with audio effects specified
    individually)) \n
.   PEQ_PRESET_USER_CUSTOM - (-1)\n\n

(Equalizer preset for no audio effects)\n
.   PEQ_PRESET_BLANK - 0\n\n

(Equalizer preset for audio effects like a club) \n
.   PEQ_PRESET_CLUB - 1\n\n

(Equalizer preset for audio effects like a dance) \n
.   PEQ_PRESET_DANCE - 2\n\n

(Internal sound library equalizer preset for full bass audio effects) \n
.   PEQ_PRESET_FULLBASS - 3\n\n

(Equalizer preset for bass and treble audio effects) \n
.   PEQ_PRESET_BASSTREBLE - 4\n\n

(Equalizer preset for full treble audio effects) \n
.   PEQ_PRESET_FULLTREBLE - 5\n\n

(Equalizer preset for audio effects that are suitable for playback through laptop/phone speakers) \n
.   PEQ_PRESET_LAPTOP - 6\n\n

(Equalizer preset for audio effects that sound like a large hall) \n
.   PEQ_PRESET_LHALL - 7\n\n

(Equalizer preset for audio effects that sound like a live performance) \n
.   PEQ_PRESET_LIVE - 8\n\n

(Equalizer preset for audio effects that sound like a party) \n
.   PEQ_PRESET_PARTY - 9\n\n

(Equalizer preset for pop audio effects) \n
.   PEQ_PRESET_POP - 10\n\n

(Equalizer preset for reggae audio effects) \n
.   PEQ_PRESET_REGGAE - 11\n\n

(Equalizer preset for rock audio effects) \n
.   PEQ_PRESET_ROCK - 12\n\n

(Equalizer preset for Ska audio effects) \n
.   PEQ_PRESET_SKA - 13\n\n

(Equalizer preset for soft pop audio effects) \n
.   PEQ_PRESET_SOFT - 14\n\n

(Equalizer preset for soft rock audio effects) \n
.   PEQ_PRESET_SOFTROCK - 15\n\n

(Equalizer preset for techno audio effects) \n
.   PEQ_PRESET_TECHNO - 16\n\n

(User-customized equalizer preset (with audio effects specified individually) (External sound library) ) \n
.   PEQ_PRESET_USER_CUSTOM_AUDIO_FX - 18\n\n

(Equalizer preset for normal (generic) audio effects (External sound library)) \n
.   PEQ_PRESET_NORMAL_AUDIO_FX - 19\n\n

(Equalizer preset for classical audio effects (External sound library)) \n
.   PEQ_PRESET_CLASSICAL_AUDIO_FX - 20\n\n

(Equalizer preset for dance audio effects (External sound library)) \n
.   PEQ_PRESET_DANCE_AUDIO_FX - 21\n\n

(Equalizer preset for no audio effects (External sound library)) \n
.   PEQ_PRESET_FLAT_AUDIO_FX - 22\n\n

(Equalizer preset for folk audio effects (External sound library)) \n
.   PEQ_PRESET_FOLK_AUDIO_FX - 23\n\n

(Equalizer preset for heavy metal audio effects (External sound library)) \n
.   PEQ_PRESET_HEAVYMETAL_AUDIO_FX - 24\n\n

(Equalizer preset for hip hop audio effects (External sound library)) \n
.   PEQ_PRESET_HIPHOP_AUDIO_FX - 25\n\n

(Equalizer preset for jazz audio effects (External sound library)) \n
.   PEQ_PRESET_JAZZ_AUDIO_FX - 26\n\n

(Equalizer preset for pop audio effects (External sound library)) \n
.   PEQ_PRESET_POP_AUDIO_FX - 27\n\n

(Equalizer preset for rock audio effects (External sound library)) \n
.   PEQ_PRESET_ROCK_AUDIO_FX - 28\n\n

    \n
   .    Supported Input Media Format:      \n
   .              Data Format          : FIXED_POINT \n
   .              fmt_id               : MEDIA_FMT_ID_PCM \n
   .              Sample Rates         : Standard sampling rates between 8000Hz and 192000Hz \n
   .              Number of channels   : 1 to 8  \n
   .              Channel type         : 1 to 63 \n
   .              Bits per sample      : 16,32 \n
   .              Q format             : 15,27 \n
   .              Interleaving         : Deinterleaved Unpacked \n
   .              Signed/unsigned      : Signed  \n
   }
    @h2xmlm_dataMaxInputPorts   {POPLESS_EQUALIZER_DATA_MAX_INPUT_PORTS}
    @h2xmlm_dataMaxOutputPorts  {POPLESS_EQUALIZER_DATA_MAX_OUTPUT_PORTS}
    @h2xmlm_dataInputPorts      {IN=POPLESS_EQUALIZER_DATA_INPUT_PORT}
    @h2xmlm_dataOutputPorts     {OUT=POPLESS_EQUALIZER_DATA_OUTPUT_PORT}
    @h2xmlm_supportedContTypes {APM_CONTAINER_TYPE_GC, APM_CONTAINER_TYPE_SC}
    @h2xmlm_isOffloadable       {true}
    @h2xmlm_stackSize           {POPLESS_EQUALIZER_MODULE_STACK_SIZE}
    @h2xmlm_ctrlDynamicPortIntent  { "Popless Equalizer to Soft Volume for headroom control" = INTENT_ID_P_EQ_VOL_HEADROOM, maxPorts= 1 }
    @h2xmlm_ToolPolicy          {Calibration}
    @{                   <-- Start of the Module -->
    @h2xml_Select        {"param_id_module_enable_t"}
      @h2xmlm_InsertParameter
    @h2xml_Select        {"param_id_eq_per_band_config_t"}
      @h2xmlm_InsertParameter
      @h2xmlp_toolPolicy   {Calibration}
    @h2xml_Select        {"param_id_eq_config_t"}
      @h2xmlm_InsertParameter
      @h2xmlp_toolPolicy   {Calibration}
    @h2xml_Select        {"param_id_eq_num_bands_t"}
      @h2xmlm_InsertParameter
      @h2xmlp_toolPolicy   {RTC_READONLY}
    @h2xml_Select        {"param_id_eq_band_levels_t"}
      @h2xmlm_InsertParameter
      @h2xmlp_toolPolicy   {RTC_READONLY}
    @h2xml_Select        {"param_id_eq_band_level_range_t"}
      @h2xmlm_InsertParameter
      @h2xmlp_toolPolicy   {RTC_READONLY}
    @h2xml_Select        {"param_id_eq_band_freqs_t"}
      @h2xmlm_InsertParameter
      @h2xmlp_toolPolicy   {RTC_READONLY}
    @h2xml_Select        {"param_id_eq_single_band_freq_range_t"}
      @h2xmlm_InsertParameter
      @h2xmlp_toolPolicy   {RTC_READONLY}
    @h2xml_Select        {"param_id_eq_single_band_freq_t"}
      @h2xmlm_InsertParameter
      @h2xmlp_toolPolicy   {Calibration}
    @h2xml_Select        {"param_id_eq_band_index_t"}
      @h2xmlm_InsertParameter
      @h2xmlp_toolPolicy   {Calibration}
    @h2xml_Select        {"param_id_eq_preset_id_t"}
      @h2xmlm_InsertParameter
      @h2xmlp_toolPolicy   {RTC_READONLY}
    @h2xml_Select        {"param_id_eq_num_presets_t"}
      @h2xmlm_InsertParameter
      @h2xmlp_toolPolicy   {RTC_READONLY}
    @h2xml_Select        {"param_id_eq_preset_name_t"}
      @h2xmlm_InsertParameter
      @h2xmlp_toolPolicy   {RTC_READONLY}
    @}                   <-- End of the Module -->*/

#endif // P_EQ_API_H
