/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef _REVERBGLOBAL_H_      
#define _REVERBGLOBAL_H_


#include "audio_dsp.h"

#define BIT_EXACT_WITH_QSOUND 0  // to match QSound or not   
#define REVERB_HARDCODE_DELAYS 0 // load comb and all-pass delays from table
#define REVERB_LP_POST_BIQUAD 0  // low-power reverb post filter  

typedef struct _reverbConfig
{
    uint16          uinChannels;
    uint16          uiSampleRate;
    uint16          uOutChannels;
    int             enable;
    int             lowpowerFlag;
    int             envPreset;
    int             envReverbLevelMB;
    int             envReverbTimeMs;
    int             envDecayHFRatioQ16;
    int             envPreDelayMs;
    int             reverbSendGainMB;
} reverbConfig;

class CReverbLib {
    typedef struct _combFilter
    {   /* comb filter with a delay line and a damping(decayHFratio) filter      */
        /* first-order IIR damping filter:   */
        /*          c0+c1*z**-1              */
        /*          -----------              */
        /*          1-c2*z**-1               */
        int16  dampCoeffsL16Q14[3];         /* filter coeffs: c0, c1, and c2-1   */
        int16  dampStateXL16;               /* state: x(n-1) L16                 */
        int32  dampStateYL32;               /* state: y(n-1) L32                 */
        int16  delayIndex;                  /* delayline index                   */
        int16  delayLength;                 /* delayline length                  */
        int16 *delayBuf;                    /* pointer to delayline buffer       */
    } combFilter;

    typedef struct _allpassFilter
    {   /* all-pass filter with a delay line                                     */
        int16  gainL16Q15;                  /* all-pass gain, L16Q15             */
        int16  delayIndex;                  /* delayline index                   */
        int16  delayLength;                 /* delayline length                  */
        int16 *delayBuf;                    /* pointer to delayline buffer       */
    } allpassFilter;

    typedef struct _revWrap
    {   /* parameters wrapped to pass to the main processing block               */
        /*-------------------- local copy of main config params -----------------*/
        boolean         enable;             /* enable or bypass reverb process   */
        boolean         lowpowerFlag;       /* use or not the low-power version  */
        int16           envPreset;          /* environment preset number         */
        int16           envReverbLevelMB;   /* reverb level ("reverb" in preset) */
        int16           envReverbTimeMs;    /* reverb decay time                 */
        int32           envDecayHFRatioQ16; /* reverb decay HF ratio             */
        int16           envPreDelayMs;      /* reverb pre delay                  */
        int16           reverbSendGainMB;   /* reverb send amount from one source*/

        /*------------------------------- others --------------------------------*/
        int16           inChannels;         /* input channel number              */
        int16           outChannels;        /* output channel number             */
        uint16          sampleRate;         /* sampling rate                     */
        int16           reverbSendGainL16Q15;/* reverb send amount in L16Q15     */
        int16           reverbVolumeL16Q15; /* late reverb level + normalization */
        delaylineStruct *preDelayline;      /* reverb delay struct of wet signal */
        combFilter      *combs;             /* comb filter struct array          */
        allpassFilter   *allpass;           /* all-pass filter struct array      */
    } revWrap; 
#ifdef LP_REVERB
    typedef struct _revLpWrap
    {   /* parameters wrapped to pass to the low-power processing block          */
        int16           inChannels;         /* input channel number              */
        int16           outChannels;        /* output channel number             */
        uint16          sampleRate;         /* sampling Rate                     */
        int16           reverbSendGainMB;   /* reverb send amount in mb          */
        int16           reverbSendGainL16Q15; /* reverb send amount in L16Q15    */
        int16           reverbVolumeL16Q15; /* late reverb level + normalization */
        delaylineStruct *preDelayline;      /* reverb delay struct of wet signal */
        combFilter      *combs;             /* comb filter struct array          */
        allpassFilter   *allpass;           /* all-pass filter struct array      */
    } revLpWrap; 
#endif
    typedef enum 
    {   /* environment types */
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
        /*----- the last three are not fully implemented yet -----*/
        environment_Drugged,                // 30
        environment_Dizzy,                  // 31
        environment_Psychotic,              // 32
        environment_Count
    } environment;

    typedef struct _eaxListenerProperties
    {   /* selected elements following standard of EAX 2.0 listener properties   */
        int16   room;                   // room input attenuation (mB)         
        int16   decayTime;              // reverb time RT60 (ms)            
        int32   decayHFRatio;           // high/low freq decay time ratio, L32Q23
        int16   reflections;            // early echo level (mB),(not implemented)
        int16   reflectionsDelay;       // early echo delay (ms)            
        int16   reverb;                 // late reverb level (mB)           
        int16   reverbDelay;            // late reverb delay (ms)           
    } eaxListenerProperties;
#ifdef LP_REVERB
    typedef struct _reverbLpPreloadParams
    {
        int16   comb0CoeffsL16Q14[2];     /* damp coeffs c0 and c2-1 (c1==0)     */
        int16   comb1CoeffsL16Q14[2];     /* damp coeffs c0 and c2-1 (c1==0)     */
        int16   comb2CoeffsL16Q14[2];     /* damp coeffs c0 and c2-1 (c1==0)     */
        int16   combDelays[3];            /* comb delayline lengths              */
        int16   allpassDelays[2];         /* allpass delayline lengths           */
        int16   preDelay;                 /* initial reverb delay                */
    } reverbLpPreloadParams;
#endif
private:
    static const uint16 REVERB_BLOCKSIZE = 480;           // blocksize 
    static const uint16 COMBS = 6;                 /* number of comb filters per channel      */
    static const uint16 ALLPASS = 2;               /* number of all-pass filters per channel  */
    /* reverb time is defined as RT60 - the time that reverb sound drops 60 dB   */
    static const int16 DECAY_CENTS = -6000;
    /* cutoff frequencies for designing filters                                  */
    static const uint16 LowpassDesignFrequency = 5000;
    static const uint16 HighpassDesignFrequency = 100;
    static const uint16 COMBS_LP = 3;              /* 3 comb filters for low-power QReverb      */
    static const uint16 ALLPASS_LP = 2;            /* 2 all pass filters for low-power QReverb  */

    /* comb filter and allpass filter delay lines in milliseconds                */
    static const int16 combDelays[COMBS];  // ms
    static const int16 allpassDelays[ALLPASS];              // ms

    /* Following standard of EAX 2.0 listener property sets                      */
    /* I3DL2 and MIDI compatible reverb types, merged into one list              */
    static const eaxListenerProperties reverbPresets[environment_Count - 1];

    static const int32 sampleRates[7];
    static const int16 storedDelays[7][COMBS+ALLPASS];
#ifdef LP_REVERB
    static const reverbLpPreloadParams storedLpParams[7];
#endif
private:
    /* various delayline buffers */
    int16 revInputDlyBufL[2352];                 // 400ms, 48kHz
    int16 revInputDlyBufR[5424];                 // 464ms, 48kHz
    int16 revCombDlyBuf0[2*2401];                 // 50ms,  48kHz
    int16 revCombDlyBuf1[2*2689];                 // 56ms,  48kHz
    int16 revCombDlyBuf2[2*2929];                 // 61ms,  48kHz
    int16 revCombDlyBuf3[2*3265];                 // 68ms,  48kHz
    int16 revCombDlyBuf4[2*3457];                 // 72ms,  48kHz
    int16 revCombDlyBuf5[2*3747];                 // 78ms,  48kHz
    int16 revAllpassDlyBuf0[2*289];               // 6ms,   48kHz
    int16 revAllpassDlyBuf1[2*529];               // 11ms,  48kHz

    /* filter and delayline structs */
    combFilter revCombs[2*COMBS];         /* comb filter struct array     */
    allpassFilter revAllpass[2*ALLPASS];  /* all-pass filter struct array */
    delaylineStruct revPreDelayline[2];   /* reverb input delayline       */
    revWrap      revShared;          /* parameter wrapper struct              */
#ifdef LP_REVERB
    /* various delayline buffers */
    int16 revInputDlyBufLp[240];             /*  5 ms,          48kHz     */
    int16 revCombDlyBuf0Lp[1467];            /*  30.555 6ms,    48kHz     */
    int16 revCombDlyBuf1Lp[1595];            /*  33.1774 ms,    48kHz     */
    int16 revCombDlyBuf2Lp[1699];            /*  35.3810 ms,    48kHz     */
    int16 revAllpassDlyBuf0Lp[289];          /*  6 ms,          48kHz     */
    int16 revAllpassDlyBuf1Lp[529];          /*  11 ms,         48kHz     */

    /* others */
    combFilter revLpCombs[COMBS_LP];         /* comb filters              */
    allpassFilter revLpAllpass[ALLPASS_LP];  /* all-pass filters          */
    delaylineStruct revLpPreDelayline;       /* reverb input delayline    */
    revLpWrap revLpShared;
#endif
private:
    void reverb_init();

    void reverb_preload_delays
        (
        int16   *delays,                    /* comb and allpass filter delays    */
        uint16   sampleRate                 /* sampling rate                     */
        );

    void apply_allpass_filter
        (
        int16           *destBuf,       /* input(output) buffer                  */
        allpassFilter   *allpass,        /* all-pass filter struct                */
        int16            samples        /* number of sampels to process          */
        );

    void apply_comb_filter
        (
        int16       *destBuf,           /* output buffer to mix into             */
        int16       *srcBuf,            /* input buffer of comb filter           */
        combFilter  *comb,              /* comb filter struct                    */
        int16        n,                 /* number of samples                     */
        int16        reverbVolumeL16Q15 /* reverb volume                         */
#if BIT_EXACT_WITH_QSOUND
        ,boolean      firstCombFlag      /* The firstCombFlag is a flag to match  */
        /* QSound code. QSound does something different for the first comb filter*/
        /* in the parallel structure. This is only to match bit-exact. A working */
        /* algorithm does not need this.                                         */
#endif
        );

    void mix_parallel_combs
        (
        int16        channel,           /* channel index, 0:left or 1:right      */
        int16       *destBuf,           /* output buffer                         */
        int16       *srcBuf,            /* input buffer                          */
        combFilter  *combs,             /* comb filter struct array              */
        int16        n,                 /* number of samples to be processed     */
        int16        reverbVolumeL16Q15,/* reverb volume                         */
        int16        num_combs          /* number of comb filters in parallel    */
        );

    void reverb_delay_then_combs
        (
        int16        channel,           /* channel index, 0:left or 1:right      */
        int16       *combMixBuf,        /* output buffer of mixed comb outputs   */
        int16       *srcBuf,            /* reverb input buffer                   */
        delaylineStruct *preDelayline,  /* late reverb delayline struct          */
        combFilter  *combs,             /* comb filter struct array              */
        int16        reverbVolumeL16Q15,/* reverb volume                         */
        int16        num_combs,         /* number of comb filters                */
        int16        samples            /* number of sampels to process          */
        );

    void reverb_preproc_mix
        (
        int16       *revInBufL,         /* reverb input buffer (left )           */
        int16       *revInBufR,         /* reverb input buffer (right)           */
        int16       *srcBufL,           /* direct sound input buffer (left )     */
        int16       *srcBufR,           /* direct sound input buffer (right)     */
        revWrap      revShared,         /* parameter wrapper struct              */
        int16        samples            /* number of sampels to process          */
        );

    void reverb_process_mono_out
        (
        int16       *mixBuf,            /* direct sound and final mix            */
        int16       *revInBuf,          /* reverb input buffer                   */
        revWrap      revShared,         /* parameter wrapper struct              */
        int16        samples            /* number of sampels to process          */
        );

    void reverb_process_stereo_out
        (
        int16       *mixBufL,           /* direct sound and reverb mix           */
        int16       *mixBufR,           /* direct sound and reverb mix           */
        int16       *revInBufL,         /* reverb input buffer                   */
        int16       *revInBufR,         /* reverb input buffer                   */
        revWrap      revShared,         /* parameter wrapper struct              */
        int16        samples            /* number of sampels to process          */
        );

    void reverb_update_filter_delays
        ( 
        uint16      sampleRate              /* sampling rate                     */
        );
    void reverb_update_predelay
        ( 
        uint16      sampleRate              /* sampling rate                     */
        );
    void reverb_update_room_filters
        ( 
        uint16      sampleRate              /* sampling rate                     */
        );
#ifdef LP_REVERB
    boolean reverb_setup_lp
        (
        const reverbConfig  &cfg
        );

    boolean reverb_reset_lp(void);

    reverbLpPreloadParams reverb_lp_preload
        (
        uint16 sampleRate
        );

    void process_lp
        (
        int16         *srcBufL,         /* reverb input buffer                   */
        int16         *srcBufR,         /* reverb input buffer                   */
        int16         *mixBufL,         /* direct sound and reverb mix           */
        int16         *mixBufR,         /* direct sound and reverb mix           */
        uint32         samples          /* number of sampels to process          */
        );
#endif
public:
    CReverbLib();

    boolean initialize (const reverbConfig &cfg);

    void Reset();

   void process
   (
       int16           *outBufL,           /* output buffer L                   */
       int16           *outBufR,           /* output buffer R                   */    
       int16           *srcBufL,           /* input buffer L                    */
       int16           *srcBufR,           /* input buffer R                    */
       uint32           nSampleCnt=REVERB_BLOCKSIZE
   );
};
#endif  /* _REVERBGLOBAL_H_ */ 
