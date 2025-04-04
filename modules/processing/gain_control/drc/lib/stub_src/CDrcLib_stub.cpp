/*============================================================================
FILE:          CDrcLib_stub.cpp

OVERVIEW:      Implements the drciter algorithm.

DEPENDENCIES:  None

Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
SPDX-License-Identifier: BSD-3-Clause
============================================================================*/

/*----------------------------------------------------------------------------
 * Include Files
 * -------------------------------------------------------------------------*/

#include "CDrcLib.h"

/*----------------------------------------------------------------------------
* Function Definitions
* -------------------------------------------------------------------------*/
CDrcLib::CDrcLib()
{
    m_drcCfgInt.numChannel      = NUM_CHANNEL_DEFAULT;
    m_drcData.currState[0]      = 0;
    m_aGainL32Q15[0][0]         = 0;
    m_aRmsStateL32[0]           = 0;
    m_delayBufferLeftL16[0]     = 0;
    m_delayBufferRightL16[0]    = 0;
    fnpProcess                  = NULL;
}


PPStatus CDrcLib::Initialize (DrcConfig &cfg)
{
    return PPFAILURE;
}

PPStatus CDrcLib::ReInitialize (DrcConfig &cfg)
{
    return PPFAILURE;
}

void CDrcLib::Reset ()
{

}


void CDrcLib::Process ( int16 *pOutPtrL16,
                       int16 *pOutPtrR16,
                       int16 *pInPtrL16,
                       int16 *pInPtrR16,
                       uint32 nSampleCnt)
{
    return;
}



