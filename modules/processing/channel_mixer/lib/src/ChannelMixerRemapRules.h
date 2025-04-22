#ifndef CHANNEL_MIXER_REMAP_RULES_H
#define CHANNEL_MIXER_REMAP_RULES_H

/*============================================================================
  @file ChannelMixerRemapRules.h

  Channel Mixer Remapping Rules header */

/*=========================================================================
Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
SPDX-License-Identifier: BSD-3-Clause-Clear
========================================================================= */

#include "AudioComdef.h"
#include "ar_msg.h"
#include "ChannelMixerLib.h"

#define NUM_PAIRED_REMAP_RULES 2
#define Q14_FACTOR                       14

// Define Remapping rules for paired input channels (i.e) a particular pair of
// input channels must be remapped to a particular pair of channels.
typedef struct _PairedRemapRules
{
    // The pair of input channels
    ChMixerChType fromCh1;
    ChMixerChType fromCh2;
    // The pair of output channels
    ChMixerChType toCh1;
    ChMixerChType toCh2;
} PairedRemapRules;

extern const PairedRemapRules PairedRemapRulesList[];

// Define remapping rules for a single input channel. If the input channel is
// not present in the output, then one of the following rules (for that input
// channel) will be applied. The rules for that input channel are arranged in a
// decreasing order of priority.
typedef struct _InfoPerRemapToCh
{
    ChMixerChType remapToCh;
    int16_t mixingParam; // in Q14
} InfoPerRemapToCh;

typedef struct _ChannelMap
{
    int32_t numToChPerRule;
    const InfoPerRemapToCh *outMap;
} ChannelMap;

typedef struct _SingleChRemapRules
{
    int32_t numRulesPerFromCh;
    const ChannelMap *map;
} SingleChRemapRules;

extern const SingleChRemapRules SingleChRemapRulesList[];

// The following steps must be taken to add a new rule for an input channel. Every rule can be divided into a set of
// "remap to" channels.
// 1. For each "remap to" channel define the *InfoPerRemapToCh* structure.
// 2. Use the *ChannelMap* structure to define the rule as a set of "remap to" channels.
// 3. Add the rule to the list of rules for the channel and update the *SingleChRemapRulesList*

#endif  // #ifndef CHANNEL_MIXER_REMAP_RULES_H
