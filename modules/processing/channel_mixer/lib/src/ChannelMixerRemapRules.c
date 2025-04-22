/*============================================================================
  @file ChannelMixerRemapRules.c

  Rules that will be used for down-mixing/up-mixing */

/*=========================================================================
Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
SPDX-License-Identifier: BSD-3-Clause-Clear
========================================================================= */

#include "ChannelMixerRemapRules.h"

/*  Paired re-mappings.
    ------------------
If the pair of channels (specified in the first two elements of a row) are
present in the input (and not present in the output), and if the pair of
channels (specified in the last two elements of a row in the array below) are
present in the output, then the corresponding remapping will be applied.
*/
const PairedRemapRules PairedRemapRulesList[NUM_PAIRED_REMAP_RULES] =
{
    // Remap LS,LB channels in input to LB,RLC in output
    {CH_MIXER_PCM_CH_LS, CH_MIXER_PCM_CH_LB, CH_MIXER_PCM_CH_LB, CH_MIXER_PCM_CH_RLC},
    // Remap RS,RB channels in input to RB,RRC in output
    {CH_MIXER_PCM_CH_RS, CH_MIXER_PCM_CH_RB, CH_MIXER_PCM_CH_RB, CH_MIXER_PCM_CH_RRC},
};

/* Setup array for remapping a single input channel. 
   -------------------------------------------------
These rules have the following characteristics:
1. For every input channel, a rule is defined to remap a channel to one 
channel or multiple channels.
2. There can be multiple such rules per input channel. Each of these rules are
arranged in the decreasing order (i.e) a rule can be applied only when any
rule above has not been satisfied.
3. If the input channel is present in the output as well, then none of these rules will
be applied.
*/
// Remap list for Front left channel
const InfoPerRemapToCh L_C[1] = { { CH_MIXER_PCM_CH_C, 11588 } };
const ChannelMap L_map[1] = {{1, L_C}};

// Remap list for Front Right channel
const InfoPerRemapToCh R_C[1] = { { CH_MIXER_PCM_CH_C, 11588 } };
const ChannelMap R_map[1] = {{1, R_C}};

// Remap list for front Center channel
const InfoPerRemapToCh C_L_R[2] = { { CH_MIXER_PCM_CH_L, 11588 }, { CH_MIXER_PCM_CH_R, 11588 } };
const ChannelMap C_map[1] = {{2, C_L_R}};

// Remap list for Left-surround channel
const InfoPerRemapToCh LS_LB[1] = { { CH_MIXER_PCM_CH_LB, 16384 } };
const InfoPerRemapToCh LS_L[1] = { { CH_MIXER_PCM_CH_L, 11588 } };
const InfoPerRemapToCh LS_C[1] = { { CH_MIXER_PCM_CH_C, 8192 } };
const ChannelMap LS_map[3] ={ {1, LS_LB}, {1, LS_L}, {1, LS_C} };

// Remap list for Right-surround channel
const InfoPerRemapToCh RS_RB[1] = { { CH_MIXER_PCM_CH_RB, 16384 } };
const InfoPerRemapToCh RS_R[1] = { { CH_MIXER_PCM_CH_R, 11588 } };
const InfoPerRemapToCh RS_C[1] = { { CH_MIXER_PCM_CH_C, 8192 } };
const ChannelMap RS_map[3] ={ {1, RS_RB}, {1, RS_R}, {1, RS_C} };

// Remap list for LFE-I
const InfoPerRemapToCh LFEI_L_R[2] = { {CH_MIXER_PCM_CH_L, 4096}, {CH_MIXER_PCM_CH_R, 4096} };
const InfoPerRemapToCh LFEI_C[1] = { {CH_MIXER_PCM_CH_C, 4096} };
const ChannelMap LFEI_map[2] = { {2, LFEI_L_R}, {1, LFEI_C} };

// Remap list for Left Back channel
const InfoPerRemapToCh LB_LS[1] = { { CH_MIXER_PCM_CH_LS, 16384 } };
const InfoPerRemapToCh LB_L[1] = { {CH_MIXER_PCM_CH_L, 11588} };
const InfoPerRemapToCh LB_C[1] = { {CH_MIXER_PCM_CH_C, 8192} };
const ChannelMap LB_map[3] = {{1, LB_LS}, {1, LB_L}, {1, LB_C}, };

// Remap list for Right Back channel
const InfoPerRemapToCh RB_RS[1] = { { CH_MIXER_PCM_CH_RS, 16384 } };
const InfoPerRemapToCh RB_R[1] = { {CH_MIXER_PCM_CH_R, 11588} };
const InfoPerRemapToCh RB_C[1] = { {CH_MIXER_PCM_CH_C, 8192} };
const ChannelMap RB_map[3] = { {1, RB_RS}, {1, RB_R}, {1, RB_C}, };

// Remap list for Center Surround
const InfoPerRemapToCh CS_LB_RB[2] = { {CH_MIXER_PCM_CH_LB, 11588}, {CH_MIXER_PCM_CH_RB, 11588} };
const InfoPerRemapToCh CS_C[1] = { {CH_MIXER_PCM_CH_C, 8192} };
const InfoPerRemapToCh CS_L_R[2] = { {CH_MIXER_PCM_CH_L, 8192}, {CH_MIXER_PCM_CH_R, 8192} };
const ChannelMap CS_map[3] = { {2, CS_LB_RB}, {1, CS_C}, {2, CS_L_R} } ;

// Remap list for Top Surround
const InfoPerRemapToCh TS_LB_RB[2] = { {CH_MIXER_PCM_CH_LB, 11588}, {CH_MIXER_PCM_CH_RB, 11588} };
const InfoPerRemapToCh TS_C[1] = { {CH_MIXER_PCM_CH_C, 8192} };
const InfoPerRemapToCh TS_L_R[2] = { {CH_MIXER_PCM_CH_L, 8192}, {CH_MIXER_PCM_CH_R, 8192} };
const ChannelMap TS_map[3] = { {2, TS_LB_RB}, {1, TS_C}, {2, TS_L_R} } ;

// Remap list for Top-Front-Center
const InfoPerRemapToCh TFC_C[1] = { {CH_MIXER_PCM_CH_C, 16384} };
const InfoPerRemapToCh TFC_L_R[2] = { {CH_MIXER_PCM_CH_L, 11588}, {CH_MIXER_PCM_CH_R, 11588} };
const ChannelMap TFC_map[2] = { {1, TFC_C}, {2, TFC_L_R} };

// Remap list for Front Left of Center
const InfoPerRemapToCh FLC_L_C[2] = { {CH_MIXER_PCM_CH_L, 8192}, {CH_MIXER_PCM_CH_C, 8192} };
const InfoPerRemapToCh FLC_L[1] = { {CH_MIXER_PCM_CH_L, 16384} };
const InfoPerRemapToCh FLC_C[1] = { {CH_MIXER_PCM_CH_C, 11588} };
const ChannelMap FLC_map[3] = { {2, FLC_L_C}, {1, FLC_L}, {1, FLC_C} };

// Remap list for Front Right of Center
const InfoPerRemapToCh FRC_R_C[2] = { {CH_MIXER_PCM_CH_R, 8192}, {CH_MIXER_PCM_CH_C, 8192} };
const InfoPerRemapToCh FRC_R[1] = { {CH_MIXER_PCM_CH_R, 16384} };
const InfoPerRemapToCh FRC_C[1] = { {CH_MIXER_PCM_CH_C, 11588} };
const ChannelMap FRC_map[3] = { {2, FRC_R_C}, {1, FRC_R}, {1, FRC_C} };

// Remap list for Rear Left of Center
const InfoPerRemapToCh RLC_LB[1] = { {CH_MIXER_PCM_CH_LB, 16384} };
const InfoPerRemapToCh RLC_CS[1] = { {CH_MIXER_PCM_CH_CS, 11588} };
const InfoPerRemapToCh RLC_L[1] = { {CH_MIXER_PCM_CH_L, 11588} };
const InfoPerRemapToCh RLC_C[1] = { {CH_MIXER_PCM_CH_C, 8192} };
const ChannelMap RLC_map[4] = { { 1, RLC_LB }, {1, RLC_CS}, {1, RLC_L}, {1, RLC_C} };

// Remap list for Rear Right of Center
const InfoPerRemapToCh RRC_RB[1] = { {CH_MIXER_PCM_CH_RB, 16384} };
const InfoPerRemapToCh RRC_CS[1] = { {CH_MIXER_PCM_CH_CS, 11588} };
const InfoPerRemapToCh RRC_R[1] = { {CH_MIXER_PCM_CH_R, 11588} };
const InfoPerRemapToCh RRC_C[1] = { {CH_MIXER_PCM_CH_C, 8192} };
const ChannelMap RRC_map[4] = { { 1, RRC_RB }, {1, RRC_CS}, {1, RRC_R}, {1, RRC_C} };

// Remap list for Top Center
const InfoPerRemapToCh TC_C_CS[2] = { {CH_MIXER_PCM_CH_C, 8192}, {CH_MIXER_PCM_CH_CS, 8192} };
const InfoPerRemapToCh TC_C[1] = { {CH_MIXER_PCM_CH_C, 8192} };
const InfoPerRemapToCh TC_L_R[2] = { {CH_MIXER_PCM_CH_L, 11588}, {CH_MIXER_PCM_CH_R, 11588} };
const ChannelMap TC_map[3] = { {2, TC_C_CS}, {1, TC_C}, {2, TC_L_R} };

// Some Common Remappings
const InfoPerRemapToCh CommonC_C[1] = { {CH_MIXER_PCM_CH_C, 8192} };
const InfoPerRemapToCh CommonC_L_R[2] = { {CH_MIXER_PCM_CH_L, 11588}, {CH_MIXER_PCM_CH_R, 11588} };
const ChannelMap CommonC_map[2] = { {1, CommonC_C}, {2, CommonC_L_R} };

const InfoPerRemapToCh CommonL_L[1] = { {CH_MIXER_PCM_CH_L, 11588} };
const InfoPerRemapToCh CommonL_C[1] = { {CH_MIXER_PCM_CH_C, 8192} };
const ChannelMap CommonL_map[2] = { {1, CommonL_L}, {1, CommonL_C} };

const InfoPerRemapToCh CommonR_R[1] = { {CH_MIXER_PCM_CH_R, 11588} };
const InfoPerRemapToCh CommonR_C[1] = { {CH_MIXER_PCM_CH_C, 8192} };
const ChannelMap CommonR_map[2] = { {1, CommonR_R}, {1, CommonR_C} };

// Form the rules
const SingleChRemapRules SingleChRemapRulesList[CH_MIXER_PCM_CH_MAX_TYPE + 1] =
{ 
    {0, NULL },
    // Remap list for Front left channel
    {1, L_map },
    // Remap list for Front Right channel
    {1, R_map },
    // Remap list for front Center channel
    {1, C_map },
    // Remap list for Left-surround channel
    {3, LS_map },
    // Remap list for Right-surround channel
    {3, RS_map },
    // Remap list for LFE-I
    {2, LFEI_map },
    // Remap list for Center Surround
    {3, CS_map },
    // Remap list of Left back/Rear Left
    {3, LB_map },
    // Remap list for Right back/Rear Right channel
    {3, RB_map },
    // Remap list for Top Surround
    {3, TS_map },
    // Remap list for Top-Front-Center
    {2, TFC_map },
    // Remap list for Mono Surround
    {3, CS_map },
    // Remap list for Front Left of Center
    {3, FLC_map },
    // Remap list for Front Right of Center
    {3, FRC_map },
    // Remap list for Rear Left of Center
    {4, RLC_map },
    // Remap list for Rear Right of Center
    {4, RRC_map },
    // Remap list for LFE II
    {2, CommonC_map },
    // Remap list for Side-Left
    {2, CommonL_map },
    // Remap list for Side-Right
    {2, CommonR_map },
    // Remap list for Top Front Left
    {2, CommonL_map },
    // Remap list for Top Front Right
    {2, CommonR_map },
    // Remap list for Top Center
    {3, TC_map },
    // Remap list for Top Back Left
    {2, CommonL_map },
    // Remap list for Top Back Right
    {2, CommonR_map },
    // Remap list for Top Side Left
    {2, CommonL_map },
    // Remap list for Top Side Right
    {2, CommonR_map },
    // Remap list for Top Back Center
    {2, CommonC_map },
    // Remap list for Bottom Front Center
    {2, CommonC_map },
    // Remap list for Bottom Front Left
    {2, CommonL_map },
    // Remap list for Bottom Front Right
    {2, CommonR_map },
    // Remap list for Left - Wide
    {2, CommonL_map },
    // Remap list for Right - Wide
    {2, CommonR_map },
    // Remap list for Left Side Direct
    {2, CommonL_map },
    // Remap list for Right Side Direct
    {2, CommonR_map },
};
