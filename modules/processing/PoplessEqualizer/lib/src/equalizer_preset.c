/* Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
/****************************************************************************
 * FILE NAME: equalizer_preset.c
 * DESCRIPTION:
 *    Contains presets for equalizer. 
 * FUNCTION LIST :
 *    equalizer_load_preset: Read preset value into a struct.
*****************************************************************************/

#include "equalizer_preset.h"

/*=============================================================================
  Constants
  =============================================================================*/



const uint32 eq_audio_fx_band_freq_range[EQ_AUDIO_FX_BAND][2] = {
    {30000, 120000},
    {120001, 460000},
    {460001, 1800000},
    {1800001, 7000000},
    {7000001, 24000000}};




const char eq_settings_names[][EQ_MAX_PRESET_NAME_LEN]=
{

    "EQ_USER_CUSTOM",
    "EQ_PRESET_BLANK",
    "EQ_PRESET_CLUB",
    "EQ_PRESET_DANCE",
    "EQ_PRESET_FULLBASS",
    "EQ_PRESET_BASSTREBLE",
    "EQ_PRESET_FULLTREBLE",
    "EQ_PRESET_LAPTOP",
    "EQ_PRESET_LHALL",
    "EQ_PRESET_LIVE",
    "EQ_PRESET_PARTY",
    "EQ_PRESET_POP",
    "EQ_PRESET_REGGAE",
    "EQ_PRESET_ROCK",
    "EQ_PRESET_SKA",
    "EQ_PRESET_SOFT",
    "EQ_PRESET_SOFTROCK",
    "EQ_PRESET_TECHNO",

    "EQ_MAX_SETTINGS_QCOM",


    "EQ_USER_CUSTOM_AUDIO_FX",
    "EQ_PRESET_NORMAL_AUDIO_FX",
    "EQ_PRESET_CLASSICAL_AUDIO_FX",
    "EQ_PRESET_DANCE_AUDIO_FX",
    "EQ_PRESET_FLAT_AUDIO_FX",
    "EQ_PRESET_FOLK_AUDIO_FX",
    "EQ_PRESET_HEAVYMETAL_AUDIO_FX",
    "EQ_PRESET_HIPHOP_AUDIO_FX",
    "EQ_PRESET_JAZZ_AUDIO_FX",
    "EQ_PRESET_POP_AUDIO_FX",
    "EQ_PRESET_ROCK_AUDIO_FX",

    "EQ_MAX_SETTINGS_AUDIO_FX"
};


/*  Note: for band cases, Q is fixed at 3 now                                */
#define EQ_DEFAULT_QFACTOR  768 // Q8

/* qcom Presets list:                                                             */
/*  0:    "blank" (12 band blank data, for initialization)                   */
/*  1:    "club"                                                             */
/*  2:    "dance"                                                            */
/*  3:    "fullbass"                                                         */
/*  4:    "basstreble"                                                       */
/*  5:    "fulltreble"                                                       */
/*  6:    "laptop"                                                           */
/*  7:    "lhall"                                                            */
/*  8:    "live"                                                             */
/*  9:    "party"                                                            */
/*  10:   "pop"                                                              */
/*  11:   "reggae"                                                           */
/*  12:   "rock"                                                             */
/*  13:   "ska"                                                              */
/*  14:   "soft"                                                             */
/*  15:   "softrock"                                                         */
/*  16:   "techno"                                                           */

// audio fx preset list
// 19: {"Normal"}
// 20: {"Classical"}
// 21: {"Dance"}
// 22: {"Flat"}
// 23: {"Folk"}
// 24: {"Heavy Metal"}
// 25: {"Hip Hop"}
// 26: {"Jazz"}
// 27: {"Pop"}
// 28: {"Rock"}


/* settings for flat EQ (blank) */
const uint32 uiFreqBlank[12]={0,0,0,0,0,0,0,0,0,0,0,0};
const int32 iGainBlank[12]={0,0,0,0,0,0,0,0,0,0,0,0};
const eq_filter_type_t FilterTypeBlank[12]={ EQ_TYPE_NONE,EQ_TYPE_NONE,EQ_TYPE_NONE,EQ_TYPE_NONE, EQ_TYPE_NONE,EQ_TYPE_NONE,EQ_TYPE_NONE,EQ_TYPE_NONE,EQ_TYPE_NONE,EQ_TYPE_NONE,EQ_TYPE_NONE,EQ_TYPE_NONE};
const int32 QFactorBlank[12] = {0,0,0,0,0,0,0,0,0,0,0,0};

/* settings for club*/
const uint32 uiFreqClub[5]={310,600,1000,3000,6000};
const int32 iGainClub[5]={3,4,4,4,3};
const eq_filter_type_t FilterTypeClub[5]={EQ_BAND_BOOST,EQ_BAND_BOOST,EQ_BAND_BOOST,EQ_BAND_BOOST,EQ_BAND_BOOST};
const int32 QFactorClub[5] = {EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR};

/* settings for dance */
const uint32 uiFreqDance[7]={60,170,310,3000,6000,12000,14000};
const int32 iGainDance[7]={5,4,2,3,4,4,2};
const eq_filter_type_t FilterTypeDance[7]={EQ_BASS_BOOST,EQ_BAND_BOOST,EQ_BAND_BOOST,EQ_BAND_CUT,EQ_BAND_CUT,EQ_BAND_CUT,EQ_TREBLE_CUT};
const int32 QFactorDance[7] = {0,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,0};

/*settings for fullbass*/
const uint32 uiFreqFullbass[6]={310,600,1000,3000,6000,12000};
const int32 iGainFullbass[6]={6,3,1,2,3,9};
const eq_filter_type_t FilterTypeFullbass[6]={EQ_BASS_BOOST,EQ_BAND_BOOST,EQ_BAND_BOOST,EQ_BAND_CUT,EQ_BAND_CUT,EQ_TREBLE_CUT};
const int32 QFactorFullbass[6] = {0,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,0};

/*settings for basstreble*/
const uint32 uiFreqBasstreble[8]={60,170,310,600,1000,3000,6000,12000};
const int32 iGainBasstreble[8]={4,3,0,4,3,1,4,6};
const eq_filter_type_t FilterTypeBasstreble[8]={EQ_BASS_BOOST,EQ_BAND_BOOST,EQ_BAND_BOOST,EQ_BAND_CUT,EQ_BAND_CUT,EQ_BAND_BOOST,EQ_BAND_BOOST,EQ_TREBLE_BOOST};
const int32 QFactorBasstreble[8] = {0,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,0};

/* settings for fulltreble*/
const uint32 uiFreqFulltreble[5]={310,600,1000,3000,6000};
const int32 iGainFulltreble[5]={6,3,2,6,9};
const eq_filter_type_t FilterTypeFulltreble[5]={ EQ_BASS_CUT,EQ_BAND_CUT,EQ_BAND_BOOST,EQ_BAND_BOOST,EQ_TREBLE_BOOST};
const int32 QFactorFulltreble[5] = {0,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,0};

/* settings for laptop speakers/phones*/
const uint32 uiFreqLaptop[10]={60,170,310,600,1000,3000,6000,12000,14000,16000};
const int32 iGainLaptop [10]={3,6,3,2,1,1,3,6,8,9};
const eq_filter_type_t FilterTypeLaptop [10]={ EQ_BASS_BOOST,EQ_BAND_BOOST,EQ_BAND_BOOST,EQ_BAND_CUT,EQ_BAND_CUT,EQ_BAND_BOOST,EQ_BAND_BOOST,EQ_BAND_BOOST,EQ_BAND_BOOST,EQ_TREBLE_BOOST};
const int32 QFactorLaptop[10] = {0,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,0};

/* settings for large hall*/
const uint32 uiFreqLhall[7]={170,310,600,1000,3000,6000,12000};
const int32 iGainLhall[7]={6,3,3,0,3,3,3};
const eq_filter_type_t FilterTypeLhall[7]={ EQ_BASS_BOOST,EQ_BAND_BOOST,EQ_BAND_BOOST,EQ_BAND_BOOST,EQ_BAND_CUT,EQ_BAND_CUT,EQ_BAND_CUT};
const int32 QFactorLhall[7] = {0,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR};

/* settings for live*/
const uint32 uiFreqLive[10]={60,170,310,600,1000,3000,6000,12000,14000,16000};
const int32 iGainLive[10]={3,0,2,2,3,3,3,2,2,1};
const eq_filter_type_t FilterTypeLive[10]={ EQ_BASS_CUT,EQ_BAND_BOOST,EQ_BAND_BOOST,EQ_BAND_BOOST,EQ_BAND_BOOST,EQ_BAND_BOOST,EQ_BAND_BOOST,EQ_BAND_BOOST,EQ_BAND_BOOST,EQ_TREBLE_BOOST};
const int32 QFactorLive[10] = {0,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,0};

/* settings for party*/
const uint32 uiFreqParty[2]={170,14000};
const int32 iGainParty[2]={4,4};
const eq_filter_type_t FilterTypeParty[2]={EQ_BASS_BOOST,EQ_TREBLE_BOOST};
const int32 QFactorParty[2] = {0,0};

/* settings for pop*/
const uint32 uiFreqPop[7]={60,170,310,600,1000,3000,6000};
const int32 iGainPop[7]={1,3,4,5,3,0,1};
const eq_filter_type_t FilterTypePop[7]={ EQ_BASS_CUT,EQ_BAND_BOOST,EQ_BAND_BOOST,EQ_BAND_BOOST,EQ_BAND_BOOST,EQ_BAND_BOOST,EQ_TREBLE_CUT};
const int32 QFactorPop[7] = {0,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,0};

/* settings for reggae*/
const uint32 uiFreqReggae[5]={310,600,1000,3000,6000};
const int32 iGainReggae[5]={1,3,0,4,4};
const eq_filter_type_t FilterTypeReggae[5]={ EQ_BAND_CUT,EQ_BAND_CUT,EQ_BAND_BOOST,EQ_BAND_BOOST,EQ_BAND_BOOST};
const int32 QFactorReggae[5] = {EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR};

/* settings for rock*/
const uint32 uiFreqRock[7]={60,170,310,600,1000,3000,6000};
const int32 iGainRock[7]={5,3,3,5,2,2,6};
const eq_filter_type_t FilterTypeRock[7]={ EQ_BASS_BOOST,EQ_BAND_BOOST,EQ_BAND_CUT,EQ_BAND_CUT,EQ_BAND_CUT,EQ_BAND_BOOST,EQ_TREBLE_BOOST};
const int32 QFactorRock[7] = {0,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,0};

/* settings for ska*/
const uint32 uiFreqSka[8]={60,170,310,600,1000,3000,6000,12000};
const int32 iGainSka[8]={2,3,3,1,3,4,5,5};
const eq_filter_type_t FilterTypeSka[8]={ EQ_BASS_CUT,EQ_BAND_CUT,EQ_BAND_CUT,EQ_BAND_CUT,EQ_BAND_BOOST,EQ_BAND_BOOST,EQ_BAND_BOOST,EQ_TREBLE_BOOST};
const int32 QFactorSka[8] = {0,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,0};

/* settings for soft*/
const uint32 uiFreqSoft[9]={60,170,310,600,1000,3000,6000,12000,14000};
const int32 iGainSoft[9]={3,1,1,2,1,2,4,5,6};
const eq_filter_type_t FilterTypeSoft[9]={ EQ_BASS_BOOST,EQ_BAND_BOOST,EQ_BAND_CUT,EQ_BAND_CUT,EQ_BAND_CUT,EQ_BAND_BOOST,EQ_BAND_BOOST,EQ_BAND_BOOST,EQ_TREBLE_BOOST};
const int32 QFactorSoft[9] = {0,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,0};

/* settings for softrock*/
const uint32 uiFreqSoftrock[9]={170,310,600,1000,3000,6000,12000,14000,16000};
const int32 iGainSoftrock[9]={3,1,1,2,3,2,1,2,6};
const eq_filter_type_t FilterTypeSoftrock[9]={ EQ_BASS_BOOST,EQ_BAND_BOOST,EQ_BAND_CUT,EQ_BAND_CUT,EQ_BAND_CUT,EQ_BAND_CUT,EQ_BAND_CUT,EQ_BAND_CUT,EQ_BAND_BOOST};
const int32 QFactorSoftrock[9] = {0,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR};

/* settings for techno*/
const uint32 uiFreqTechno[8]={60,170,310,600,1000,3000,6000,12000};
const int32 iGainTechno[8]={5,3,0,-3,-2,0,5,6};
const eq_filter_type_t FilterTypeTechno[8]={ EQ_BASS_BOOST,EQ_BAND_BOOST,EQ_BAND_BOOST,EQ_BAND_CUT,EQ_BAND_CUT,EQ_BAND_BOOST,EQ_BAND_BOOST,EQ_TREBLE_BOOST};
const int32 QFactorTechno[8] = {0,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,EQ_DEFAULT_QFACTOR,0};


/*
// for audio fx 14k Hz only
// rest 4 bands use 256
-15 <= gain <= -12 or 12 <= gain <= 15, use q = 100
-11 <= gain <= -10 or 10 <= gain <= 11, use q = 105
-9 or 9, use 110
-8 or 8, use 115
-7 or 7, use 120
-6 or 6, use 135
-5 and 5, use 155
-4 and 4, use 210
-3 <= gain <= 3, use 256
*/

// AudioFX common params
const uint32 uiFreqAudioFx[5]={60,230,910,3600,14000};
const eq_filter_type_t FilterTypeAudioFx[5]={EQ_BAND_BOOST,EQ_BAND_BOOST,EQ_BAND_BOOST,EQ_BAND_BOOST,EQ_BAND_BOOST};


/* Normal Preset */
const int32 iGainNormalAudioFx[5]={3, 0, 0, 0, 3};
const int32 QFactorNormalAudioFx[5] = {256,256,256,256,256};

/* Classical Preset */
const int32 iGainClassicalAudioFx[5]={5, 3, -2, 4, 4};
const int32 QFactorClassicalAudioFx[5] = {256,256,256,256,210};

/* Dance Preset */
const int32 iGainDanceAudioFx[5]={6, 0, 2, 4, 1};
const int32 QFactorDanceAudioFx[5] = {256,256,256,256,256};

/* Flat Preset */
const int32 iGainFlatAudioFx[5]={0, 0, 0, 0, 0};
const int32 QFactorFlatAudioFx[5] = {256,256,256,256,256};

/* Folk Preset */
const int32 iGainFolkAudioFx[5]={3, 0, 0, 2, -1};
const int32 QFactorFolkAudioFx[5] = {256,256,256,256,256};

/* Heavy Metal Preset */
const int32 iGainHeavyMetalAudioFx[5]={4, 1, 9, 3, 0};
const int32 QFactorHeavyMetalAudioFx[5] = {256,256,256,256,256};

/* Hip Hop Preset */
const int32 iGainHipHopAudioFx[5]={5, 3, 0, 1, 3};
const int32 QFactorHipHopAudioFx[5] = {256,256,256,256,256};

/* Jazz Preset */
const int32 iGainJazzAudioFx[5]={4, 2, -2, 2, 5};
const int32 QFactorJazzAudioFx[5] = {256,256,256,256,155};

/* Pop Preset */
const int32 iGainPopAudioFx[5]={-1, 2, 5, 1, -2};
const int32 QFactorPopAudioFx[5] = {256,256,256,256,256};

/* Rock Preset */
const int32 iGainRockAudioFx[5]={5, 3, -1, 3, 5};
const int32 QFactorRockAudioFx[5] = {256,256,256,256,155};

/*-----------------------------------------------------------------------
 ** Constructor / Destructor definitions
 **-----------------------------------------------------------------------*/

/*===========================================================================*/
/* FUNCTION : equalizer_load_preset                                          */
/*                                                                           */
/* DESCRIPTION: load preset values into an array of struct for each eq band. */
/*                                                                           */
/* INPUTS: bands-> array of eq band structs                                  */
/*         nbands-> ptr to number of bands                                   */
/*         preset: preset index                                              */
/* OUTPUTS: bands-> array of eq band structs                                 */
/*          nbands-> ptr to number of bands                                  */
/*                                                                           */
/* IMPLEMENTATION NOTES:                                                     */
/* Decides the no of bands, cut offs, gains and filter types based on presets*/
/*===========================================================================*/
    EQ_RESULT   equalizer_load_preset
(
        eq_band_internal_specs_t    *bands,             // array of eq band struct
        uint32           *nbands,           // ptr to number of bands
        eq_settings_t         preset             // preset index
        )
{
    uint32 i;
    uint32 *Freqs;
    int32 *Gains;
    eq_filter_type_t *Types;
    int32 *QFactors;

    switch(preset)
    {
        case EQ_PRESET_BLANK: //blank (all flat)
            *nbands = sizeof(uiFreqBlank)/sizeof(int32);
            Freqs = (uint32*)uiFreqBlank;
            Gains = (int32*)iGainBlank;
            Types = (eq_filter_type_t*)FilterTypeBlank;
            QFactors = (int32*)QFactorBlank;
            break;

        case EQ_PRESET_CLUB: //club
            *nbands = sizeof(uiFreqClub)/sizeof(int32);
            Freqs = (uint32*)uiFreqClub;
            Gains = (int32*)iGainClub;
            Types = (eq_filter_type_t*)FilterTypeClub;
            QFactors = (int32*)QFactorClub;
            break;

        case EQ_PRESET_DANCE: //dance
            *nbands=sizeof(uiFreqDance)/sizeof(int32);
            Freqs = (uint32*)uiFreqDance;
            Gains = (int32*)iGainDance;
            Types = (eq_filter_type_t*)FilterTypeDance;
            QFactors = (int32*)QFactorDance;
            break;

        case EQ_PRESET_FULLBASS: //fullbass
            *nbands=sizeof(uiFreqFullbass)/sizeof(int32);
            Freqs = (uint32*)uiFreqFullbass;
            Gains = (int32*)iGainFullbass;
            Types = (eq_filter_type_t*)FilterTypeFullbass;
            QFactors = (int32*)QFactorFullbass;
            break;

        case EQ_PRESET_BASSTREBLE://basstreble
            *nbands=sizeof(uiFreqBasstreble)/sizeof(int32);
            Freqs = (uint32*)uiFreqBasstreble;
            Gains = (int32*)iGainBasstreble;
            Types = (eq_filter_type_t*)FilterTypeBasstreble;
            QFactors = (int32*)QFactorBasstreble;
            break;

        case EQ_PRESET_FULLTREBLE: //fulltreble
            *nbands=sizeof(uiFreqFulltreble)/sizeof(int32);
            Freqs = (uint32*)uiFreqFulltreble;
            Gains = (int32*)iGainFulltreble;
            Types = (eq_filter_type_t*)FilterTypeFulltreble;
            QFactors = (int32*)QFactorFulltreble;
            break;

        case EQ_PRESET_LAPTOP: //laptop
            *nbands=sizeof(uiFreqLaptop)/sizeof(int32);
            Freqs = (uint32*)uiFreqLaptop;
            Gains = (int32*)iGainLaptop;
            Types = (eq_filter_type_t*)FilterTypeLaptop;
            QFactors = (int32*)QFactorLaptop;
            break;

        case EQ_PRESET_LHALL: //lhall
            *nbands=sizeof(uiFreqLhall)/sizeof(int32);
            Freqs = (uint32*)uiFreqLhall;
            Gains = (int32*)iGainLhall;
            Types = (eq_filter_type_t*)FilterTypeLhall;
            QFactors = (int32*)QFactorLhall;
            break;

        case EQ_PRESET_LIVE: //live
            *nbands=sizeof(uiFreqLive)/sizeof(int32);
            Freqs = (uint32*)uiFreqLive;
            Gains = (int32*)iGainLive;
            Types = (eq_filter_type_t*)FilterTypeLive;
            QFactors = (int32*)QFactorLive;
            break;

        case EQ_PRESET_PARTY: //party
            *nbands=sizeof(uiFreqParty)/sizeof(int32);
            Freqs = (uint32*)uiFreqParty;
            Gains = (int32*)iGainParty;
            Types = (eq_filter_type_t*)FilterTypeParty;
            QFactors = (int32*)QFactorParty;
            break;

        case EQ_PRESET_POP: //pop
            *nbands=sizeof(uiFreqPop)/sizeof(int32);
            Freqs = (uint32*)uiFreqPop;
            Gains = (int32*)iGainPop;
            Types = (eq_filter_type_t*)FilterTypePop;
            QFactors = (int32*)QFactorPop;
            break;

        case EQ_PRESET_REGGAE: //reggae
            *nbands=sizeof(uiFreqReggae)/sizeof(int32);
            Freqs = (uint32*)uiFreqReggae;
            Gains = (int32*)iGainReggae;
            Types = (eq_filter_type_t*)FilterTypeReggae;
            QFactors = (int32*)QFactorReggae;
            break;

        case EQ_PRESET_ROCK:  //rock
            *nbands=sizeof(uiFreqRock)/sizeof(int32);
            Freqs = (uint32*)uiFreqRock;
            Gains = (int32*)iGainRock;
            Types = (eq_filter_type_t*)FilterTypeRock;
            QFactors = (int32*)QFactorRock;
            break;

        case EQ_PRESET_SKA: //ska
            *nbands=sizeof(uiFreqSka)/sizeof(int32);
            Freqs = (uint32*)uiFreqSka;
            Gains = (int32*)iGainSka;
            Types = (eq_filter_type_t*)FilterTypeSka;
            QFactors = (int32*)QFactorSka;
            break;

        case EQ_PRESET_SOFT: //soft
            *nbands=sizeof(uiFreqSoft)/sizeof(int32);
            Freqs = (uint32*)uiFreqSoft;
            Gains = (int32*)iGainSoft;
            Types = (eq_filter_type_t*)FilterTypeSoft;
            QFactors = (int32*)QFactorSoft;
            break;

        case EQ_PRESET_SOFTROCK: //softrock
            *nbands=sizeof(uiFreqSoftrock)/sizeof(int32);
            Freqs = (uint32*)uiFreqSoftrock;
            Gains = (int32*)iGainSoftrock;
            Types = (eq_filter_type_t*)FilterTypeSoftrock;
            QFactors = (int32*)QFactorSoftrock;
            break;

        case EQ_PRESET_TECHNO: //techno
            *nbands=sizeof(uiFreqTechno)/sizeof(int32);
            Freqs = (uint32*)uiFreqTechno;
            Gains = (int32*)iGainTechno;
            Types = (eq_filter_type_t*)FilterTypeTechno;
            QFactors = (int32*)QFactorTechno;
            break;


        case EQ_PRESET_NORMAL_AUDIO_FX:
            *nbands = sizeof(uiFreqAudioFx)/sizeof(int32);
            Freqs = (uint32*)uiFreqAudioFx;
            Gains = (int32*)iGainNormalAudioFx;
            Types = (eq_filter_type_t*)FilterTypeAudioFx;
            QFactors = (int32*)QFactorNormalAudioFx;
            break;

        case EQ_PRESET_CLASSICAL_AUDIO_FX:
            *nbands = sizeof(uiFreqAudioFx)/sizeof(int32);
            Freqs = (uint32*)uiFreqAudioFx;
            Gains = (int32*)iGainClassicalAudioFx;
            Types = (eq_filter_type_t*)FilterTypeAudioFx;
            QFactors = (int32*)QFactorClassicalAudioFx;
            break;

        case EQ_PRESET_DANCE_AUDIO_FX:
            *nbands = sizeof(uiFreqAudioFx)/sizeof(int32);
            Freqs = (uint32*)uiFreqAudioFx;
            Gains = (int32*)iGainDanceAudioFx;
            Types = (eq_filter_type_t*)FilterTypeAudioFx;
            QFactors = (int32*)QFactorDanceAudioFx;
            break;

        case EQ_PRESET_FLAT_AUDIO_FX:
            *nbands = sizeof(uiFreqAudioFx)/sizeof(int32);
            Freqs = (uint32*)uiFreqAudioFx;
            Gains = (int32*)iGainFlatAudioFx;
            Types = (eq_filter_type_t*)FilterTypeAudioFx;
            QFactors = (int32*)QFactorFlatAudioFx;
            break;

        case EQ_PRESET_FOLK_AUDIO_FX:
            *nbands = sizeof(uiFreqAudioFx)/sizeof(int32);
            Freqs = (uint32*)uiFreqAudioFx;
            Gains = (int32*)iGainFolkAudioFx;
            Types = (eq_filter_type_t*)FilterTypeAudioFx;
            QFactors = (int32*)QFactorFolkAudioFx;
            break;

        case EQ_PRESET_HEAVYMETAL_AUDIO_FX:
            *nbands = sizeof(uiFreqAudioFx)/sizeof(int32);
            Freqs = (uint32*)uiFreqAudioFx;
            Gains = (int32*)iGainHeavyMetalAudioFx;
            Types = (eq_filter_type_t*)FilterTypeAudioFx;
            QFactors = (int32*)QFactorHeavyMetalAudioFx;
            break;

        case EQ_PRESET_HIPHOP_AUDIO_FX:
            *nbands = sizeof(uiFreqAudioFx)/sizeof(int32);
            Freqs = (uint32*)uiFreqAudioFx;
            Gains = (int32*)iGainHipHopAudioFx;
            Types = (eq_filter_type_t*)FilterTypeAudioFx;
            QFactors = (int32*)QFactorHipHopAudioFx;
            break;

        case EQ_PRESET_JAZZ_AUDIO_FX:
            *nbands = sizeof(uiFreqAudioFx)/sizeof(int32);
            Freqs = (uint32*)uiFreqAudioFx;
            Gains = (int32*)iGainJazzAudioFx;
            Types = (eq_filter_type_t*)FilterTypeAudioFx;
            QFactors = (int32*)QFactorJazzAudioFx;
            break;

        case EQ_PRESET_POP_AUDIO_FX:
            *nbands = sizeof(uiFreqAudioFx)/sizeof(int32);
            Freqs = (uint32*)uiFreqAudioFx;
            Gains = (int32*)iGainPopAudioFx;
            Types = (eq_filter_type_t*)FilterTypeAudioFx;
            QFactors = (int32*)QFactorPopAudioFx;
            break;

        case EQ_PRESET_ROCK_AUDIO_FX:
            *nbands = sizeof(uiFreqAudioFx)/sizeof(int32);
            Freqs = (uint32*)uiFreqAudioFx;
            Gains = (int32*)iGainRockAudioFx;
            Types = (eq_filter_type_t*)FilterTypeAudioFx;
            QFactors = (int32*)QFactorRockAudioFx;
            break;

        default:
            return  EQ_FAILURE;

    } /* end of switch statement*/


    // init the preset EQ for each EQ band
    for(i = 0; i < *nbands; i++)
    {
        bands[i].uiBandIdx = i;
        bands[i].FreqHz = Freqs[i];
        bands[i].iFilterGain = Gains[i];
        bands[i].FiltType = Types[i];
        bands[i].iQFactor = QFactors[i];
    }


    return EQ_SUCCESS;
} /*---------------- end of function equalizer_load_preset ------------------*/

