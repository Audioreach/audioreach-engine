#ifndef _SYNC_API_H_
#define _SYNC_API_H_
/**
 * \file sync_api.h
 * \brief
 *    This file contains the Sync module APIs
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "ar_defs.h"
#include "apm_graph_properties.h"

/*# @h2xml_title1          {Synchronization Module API}
    @h2xml_title_agile_rev {Synchronization Module API}
    @h2xml_title_date      {July 12, 2018} */

/*==============================================================================
   Defines
==============================================================================*/

/*==============================================================================
   API definitions
==============================================================================*/

/** @ingroup ar_spf_mod_generic_sync
    Enumerates the stack size of the module. */
#define SYNC_STACK_SIZE 4096

/** @ingroup ar_spf_mod_generic_sync
    Enumerates the maximum number of input ports for the module. */
#define SYNC_MAX_IN_PORTS  32

/** @ingroup ar_spf_mod_generic_sync
    Enumerates the maximum number of output ports for the module. */
#define SYNC_MAX_OUT_PORTS 32


/** @ingroup ar_spf_mod_generic_sync
    Identifier for the generic Synchronization module.

    @subhead4{Supported input media format ID}
    - Data format       : #DATA_FORMAT_FIXED_POINT @lstsp1
    - fmt_id            : #MEDIA_FMT_ID_PCM @lstsp1
    - Sample rates      : 8, 11.025, 12, 16, 22.05, 24, 32, 44.1, 48, 88.2, 96,
                          176.4, 192, 352.8, 384 kHz @lstsp1
    - Number of channels: 1 to 128 (for certain products this module supports only 32 channels) @lstsp1
    - Channel type      : Don't care @lstsp1
    - Bit width: @lstsep
       - 16 (bits per sample 16 and Q15) @lstsp2
       - 32 (bits per sample 32 and Q31) @lstsp1
    - Q format: Q15, Q31 @lstsp1
    - Interleaving: @lstsep
       - De-interleaved unpacked @lstsp2
       - De-interleaved packed @lstsp1
    - Endianness: little, big

    Ports can have different media formats. The only restriction is that if one
    port runs at a fractional rate, the other ports must also run at a
    fractional rate. Use cases where ports have variable input frame sizes are
    not supported.
*/
#define MODULE_ID_SYNC      0x07001038

/*# @h2xmlm_module             {"MODULE_ID_SYNC", MODULE_ID_SYNC}
    @h2xmlm_displayName        {"Sync Module"}
    @h2xmlm_modSearchKeys	   {Audio, Voice}
    @h2xmlm_description        {ID for the Generic Synchronization module.
                                This module supports the MEDIA_FMT_ID_PCM
                                input media format. For more details, see AudioReach Signal Processing Framework (SPF) API Reference.}
    @h2xmlm_dataMaxInputPorts  {SYNC_MAX_IN_PORTS}
    @h2xmlm_dataInputPorts     {IN0 = 6;
                                IN1 = 8;
                                IN2 = 10;
                                IN3 = 12;
                                IN4 = 14;
                                IN5 = 16;
                                IN6 = 18;
                                IN7 = 20;
                                IN8 = 22;
                                IN9 = 24;
                                IN10 = 26;
                                IN11 = 28;
                                IN12 = 30;
                                IN13 = 32;
                                IN14 = 34;
                                IN15 = 36;
                                IN16 = 38;
                                IN17 = 40;
                                IN18 = 42;
                                IN19 = 44;
                                IN20 = 46;
                                IN21 = 48;
                                IN22 = 50;
                                IN23 = 52;
                                IN24 = 54;
                                IN25 = 56;
                                IN26 = 58;
                                IN27 = 60;
                                IN28 = 62;
                                IN29 = 64;
                                IN30 = 66;
                                IN31 = 68}
    @h2xmlm_dataMaxOutputPorts {SYNC_MAX_OUT_PORTS}
    @h2xmlm_dataOutputPorts    {OUT0 = 5;
                                OUT1 = 7;
                                OUT2 = 9;
                                OUT3 = 11;
                                OUT4 = 13;
                                OUT5 = 15;
                                OUT6 = 17;
                                OUT7 = 19;
                                OUT8 = 21;
                                OUT9 =  23;
                                OUT10 = 25;
                                OUT11 = 27;
                                OUT12 = 29;
                                OUT13 = 31;
                                OUT14 = 33;
                                OUT15 = 35;
                                OUT16 = 37;
                                OUT17 = 39;
                                OUT18 = 41;
                                OUT19 = 43;
                                OUT20 = 45;
                                OUT21 = 47;
                                OUT22 = 49;
                                OUT23 = 51;
                                OUT24 = 53;
                                OUT25 = 55;
                                OUT26 = 57;
                                OUT27 = 59;
                                OUT28 = 61;
                                OUT29 = 63;
                                OUT30 = 65;
                                OUT31 = 67}
    @h2xmlm_supportedContTypes {APM_CONTAINER_TYPE_SC, APM_CONTAINER_TYPE_GC}
    @h2xmlm_isOffloadable      {false}
    @h2xmlm_stackSize          {SYNC_STACK_SIZE}

    @{                      <-- Start of the Module -->
    @}                      <-- End of the Module   --> */


#endif //_SYNC_API_H_
