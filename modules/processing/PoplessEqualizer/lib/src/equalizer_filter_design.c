/* Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "equalizer_filter_design.h"
#include "tangent.h"
#include "msiir_api.h"


//private function declarations

    int32 sqrtfpEQFD_v2(int32 a);
    PPStatus dsplib_approx_invertEQFD_v2(int32 input,int32 *result,int32 *shift_factor);

    int32 calculateBassBoost_v2(int32 V0, int32 fc, int32 fs, int32 *bassNum, int32 *bassDen);
    int32 calculateBoostDEN_v2(int32 cutoffQ0, int32 SamplingFreqQ0, int32* KQ28);
    int32 calculateNumeratorBassBoost_v2(int32 V0, int32 K, int32 *numerator, int32 DEN);
    int32 calculateDenominatorBassBoost_v2(int32 K, int32 *denominator, int32 DEN);

    int32 calculateBandBoost_v2(int32 , int32 , int32 , int32 , int32 *, int32 *);
    int32 calculateBandBoostDEN_v2(int32 cutoffQ0, int32 SamplingFreqQ0,int32 queQ0, int32* KQ28,int32 *KbyQ);
    int32 calculateNumBandBoost_v2(int32 V0, int32 K, int32 *numerator, int32 DEN,int32 KbyQ);
    int32 calculateDenBandBoost_v2(int32 K, int32 *denominator, int32 DEN, int32 KbyQ);

    void calculateTrebleBoost_v2(int32 , int32 , int32 , int32 *, int32 *,int32 *);
    int32 calculateTrebleBoostDEN_v2(int32 cutoffQ0, int32 SamplingFreqQ0,int32 queQ0, int32* KQ28);
    int32 calculateNumTrebleBassBoost_v2(int32 V0, int32 K, int32 *numerator, int32 DEN,int32 KbyQ);
    int32 calculateDenTrebleBassBoost_v2(int32 K, int32 *denominator, int32 DEN, int32 KbyQ);

    void calculateTrebleCut_v2(int32 V0, int32 fc, int32 fs, int32 *bassNum, int32 *bassDen);
    int32 calculateTrebleCutDEN_v2(int32 cutoffQ0, int32 SamplingFreqQ0,int32 V0, int32* KQ28,int32 *Ksqrt2V0);
    int32 calculateNumTrebleCut_v2(int32 V0, int32 K, int32 *numerator, int32 DEN);
    int32 calculateDenTrebleCut_v2(int32 K, int32 *denominator, int32 DEN,int32 Ksqrt2V0,int32 V0);


    void calculatebandCut_v2(int32 V0, int32 fc, int32 fs, int32 que, int32 *bandNum, int32 *bandDen);
    int32 calculatebandCutDEN_v2(int32 cutoffQ0, int32 SamplingFreqQ0,int32 V0, int32 queQ0,int32* KQ28,int32 *KbyQ28,int32 *denSR);
    int32 calculateNumbandCut_v2(int32 V0, int32 K, int32 *numerator, int32 DEN, int32 KbyQ28,int32 denSR);
    int32 calculateDenbandCut_v2(int32 K, int32 *denominator, int32 DEN,int32 V0,int32 KbyQ28,int32 denSR);

    void calculateBassCut_v2(int32 , int32 , int32 , int32 *, int32 *);
    int32 calculateBassCutDEN_v2(int32 cutoffQ0, int32 SamplingFreqQ0,int32 V0, int32* KQ28,int32 *Ksqrt2V0,int32 *denSR);
    int32 calculateNumBassCut_v2(int32 V0, int32 K, int32 *numerator, int32 DEN, int32 denSR);
    int32 calculateDenBassCut_v2(int32 K, int32 *denominator, int32 DEN,int32 Ksqrt2V0,int32 V0, int32 denSR);

    void calculateTrebleboost_v2(int32 V0, int32 fc, int32 fs, int32 *bassNum, int32 *bassDen, int32 *shift);
    int32 calculateNumTrebleboost_v2(int32 V0, int32 K, int32 *numerator, int32 DEN);
    int32 calculateDenTrebleboost_v2(int32 K, int32 *denominator, int32 DEN);

    void calccoefs_v2(int32 V0,int32 fc, int32 fs, eq_filter_type_t type, int32 Q,int32 *bassNum,int32 *bassDen, int16 *shiftQ);

    // utils.h added
    void qdsp_norm64EQFD_v2(int64 inp,int32 *norm_inp,int32 *shift);

    // saturate to signed 32-bit dynamic range
    int32 saturate_s32EQFD_v2(int64 var1);

/*-----------------------------------------------------------------------
** Constructor / Destructor definitions
**-----------------------------------------------------------------------*/



EQ_RESULT GetBandFreqRangeEQFD_v2 (eq_band_specs_t  *pEQFilterDesignData, uint32 uiSampleRate, uint16 uiNumBands, uint16 uiBandIdx, uint32 *piMinFreq, uint32 *piMaxFreq)
{
    uint32 iMinFreq;
    uint32 iMaxFreq;
    eq_filter_type_t filterType;
    iMinFreq = 0;
    iMaxFreq = 0;

    if ( uiNumBands <= uiBandIdx ) {
        return EQ_FAILURE;
    }

    // get the filter type
    filterType = pEQFilterDesignData[uiBandIdx].filter_type;

    // if the band is bass, the min and max band freq is as follows
    if ( EQ_TYPE_NONE == filterType ) {
        iMinFreq = 0; // min. freq is 0 Hz.
        iMaxFreq = uiSampleRate >> 1; // max freq is half of sampling rate
    }

    // if the band is bass, the min and max band freq is as follows
    if ( ( EQ_BASS_BOOST == filterType ) || ( EQ_BASS_CUT == filterType ) ) {
        iMinFreq = 0; // min. freq is 0 Hz.
        iMaxFreq = pEQFilterDesignData[uiBandIdx].freq_millihertz;
    }

    // if the band is treble, the min and max band freq is as follows
    if ( ( EQ_TREBLE_BOOST == filterType ) || ( EQ_TREBLE_CUT == filterType ) ) {
        iMinFreq = pEQFilterDesignData[uiBandIdx].freq_millihertz;
        iMaxFreq = uiSampleRate >> 1; // max freq is half of sampling rate
    }

    // if the band is band, the min and max band freq is as follows
    if ( ( EQ_BAND_BOOST == filterType ) || ( EQ_BAND_CUT == filterType ) ) {

        if ( 1 == uiNumBands ) {
            // if there is only one band, the min and max band freq is as follows
            iMinFreq = 0; // min. freq is 0 Hz.
            iMaxFreq = uiSampleRate >> 1; // max freq is half of sampling rate
        }
        else {
            // if there are more than 1 band, the min and max band freq is as follows

            // check whether it is the first band
            if ( 0 == uiBandIdx ) {
                iMinFreq = 0; // min. freq is 0 Hz.

                // check whether the next band is tremble
                if ( ( EQ_TREBLE_BOOST == pEQFilterDesignData[1].filter_type ) ||
                    ( EQ_TREBLE_CUT == pEQFilterDesignData[1].filter_type ) ) {
                        iMaxFreq = pEQFilterDesignData[1].freq_millihertz;
                        // max freq is the center of next band
                }
                else { // the next band is also not a tremble band
                    iMaxFreq = (pEQFilterDesignData[0].freq_millihertz +
                        pEQFilterDesignData[1].freq_millihertz) >> 1;
                    // max freq is half between the two adjacent bands
                }
            }
            else if ( uiNumBands-1 == uiBandIdx ) { // check whether it is the last band
                // check whether the previous band is bass
                if ( ( EQ_BASS_BOOST == pEQFilterDesignData[uiBandIdx-1].filter_type ) ||
                    ( EQ_BASS_CUT == pEQFilterDesignData[uiBandIdx-1].filter_type ) ) {
                        iMinFreq = pEQFilterDesignData[uiBandIdx-1].freq_millihertz;
                        // min freq is the center of previous band
                }
                else { // the previous band is not a bass band
                    iMinFreq = (pEQFilterDesignData[uiBandIdx-1].freq_millihertz +
                        pEQFilterDesignData[uiBandIdx].freq_millihertz) >> 1;
                    // min freq is half between the two adjacent bands
                }

                iMaxFreq = uiSampleRate >> 1; // max freq is half of sampling rate
            }
            else {
                // check whether the previous band is bass
                if ( ( EQ_BASS_BOOST == pEQFilterDesignData[uiBandIdx-1].filter_type ) ||
                    ( EQ_BASS_CUT == pEQFilterDesignData[uiBandIdx-1].filter_type ) ) {
                        iMinFreq = pEQFilterDesignData[uiBandIdx-1].freq_millihertz;
                        // min freq is the center of previous band
                }
                else { // the previous band is not a bass band
                    iMinFreq = (pEQFilterDesignData[uiBandIdx-1].freq_millihertz +
                        pEQFilterDesignData[uiBandIdx].freq_millihertz) >> 1;
                    // min freq is half between the two adjacent bands
                }

                // check whether the next band is tremble
                if ( ( EQ_TREBLE_BOOST == pEQFilterDesignData[uiBandIdx+1].filter_type ) ||
                    ( EQ_TREBLE_CUT == pEQFilterDesignData[uiBandIdx+1].filter_type ) ) {
                        iMaxFreq = pEQFilterDesignData[uiBandIdx+1].freq_millihertz;
                        // max freq is the center of next band
                }
                else { // the next band is also not a tremble band
                    iMaxFreq = (pEQFilterDesignData[uiBandIdx].freq_millihertz +
                        pEQFilterDesignData[uiBandIdx+1].freq_millihertz) >> 1;
                    // max freq is half between the two adjacent bands
                }
            }
        }
    }

    //  prepare the return values
    if ( NULL != piMinFreq ) {
        *piMinFreq = iMinFreq;
    }

    if ( NULL != piMaxFreq ) {
        *piMaxFreq = iMaxFreq;
    }

    return EQ_SUCCESS;
}





int16 ProcessEQFD_v2(EQFilterDesign_t * EQFilterDesign_ptr)
{
    int16 i;

    int16 uiNumValidBands = EQFilterDesign_ptr->uiNumBands;
    eq_band_internal_specs_t *pEQBandInp = &(EQFilterDesign_ptr->pEQFilterDesignData[0]);

    // check if num of band is out of range
    if ( EQFilterDesign_ptr->uiNumBands > EQ_MAX_BANDS )
    {
        return 0;
    }

    for (i = 0; i < EQFilterDesign_ptr->uiNumBands; i++)
    {

        if (pEQBandInp->FreqHz < EQFilterDesign_ptr->uiSampleRate>>1)

        {

            (void) calccoefs_v2(
                pEQBandInp->iFilterGain,
                pEQBandInp->FreqHz,
                EQFilterDesign_ptr->uiSampleRate,
                pEQBandInp->FiltType,
                pEQBandInp->iQFactor,
                &EQFilterDesign_ptr->piFilterCoeff[i*(MSIIR_NUM_COEFFS + MSIIR_DEN_COEFFS)],
                &EQFilterDesign_ptr->piFilterCoeff[i*(MSIIR_NUM_COEFFS + MSIIR_DEN_COEFFS)+MSIIR_NUM_COEFFS],
                &EQFilterDesign_ptr->piNumShiftFactor[i]);

        }
        else
        {
            uiNumValidBands = uiNumValidBands - 1;
        }
        pEQBandInp++;
    }

    return uiNumValidBands;
}


/*===========================================================================*/

/*****************************************************************/
/* Function: calccoefs                                           */
/*****************************************************************/
/* Description: Wrapper function that calls the required function*/
/*              to calculate the coefs and the shift compensation*/
/*              for the numerator coefs                          */
/*****************************************************************/

void calccoefs_v2(int32 V0,int32 fc, int32 fs, eq_filter_type_t type, int32 Q,int32 *bassNum,int32 *bassDen, int16 *shiftQ)
{
    int32 shift;
    int32 i;
    int32 iDenCoeff[3];
    *shiftQ=2;  // default shift compensation = 2

    //initialize with all-pass
    iDenCoeff[0] = 0x40000000;
    iDenCoeff[1] = 0;
    iDenCoeff[2] = 0;

    if (V0 < 0)
    {
        if (EQ_TYPE_NONE == type)
        {
            type = EQ_TYPE_NONE;
        } else if ( EQ_BASS_BOOST == type )
        {
            type = EQ_BASS_CUT;
        } else if ( EQ_BASS_CUT == type )
        {
            type = EQ_BASS_BOOST;
        } else if ( EQ_TREBLE_BOOST == type )
        {
            type = EQ_TREBLE_CUT;
        } else if ( EQ_TREBLE_CUT == type )
        {
            type = EQ_TREBLE_BOOST;
        } else if ( EQ_BAND_BOOST == type )
        {
            type = EQ_BAND_CUT;
        } else
        {
            type = EQ_BAND_BOOST;
        }
        V0 = -V0;
    }

    /* select the correct equalizer required*/
    switch(type)
    {
    case EQ_TYPE_NONE: /* all-pass filter */
        bassNum[0] = 0x40000000;
        bassNum[1] = 0;
        bassNum[2] = 0;
        iDenCoeff[0] = 0x40000000;
        iDenCoeff[1] = 0;
        iDenCoeff[2] = 0;
        break;
    case EQ_BASS_BOOST: /* bassboost*/
        *shiftQ = (int16) (calculateBassBoost_v2(V0, fc, fs, bassNum, iDenCoeff));
        break;
    case EQ_TREBLE_BOOST:  /* trebleboost*/
        calculateTrebleboost_v2(V0, fc, fs, bassNum, iDenCoeff,&shift);
        *shiftQ = (int16) (32 - shift); //Q factor compensation shift
        break;
    case EQ_BAND_BOOST:  /* bandboost*/
        *shiftQ= (int16) calculateBandBoost_v2(V0, Q, fc, fs, bassNum, iDenCoeff);
        break;
    case EQ_BASS_CUT:    /* bass cut*/
        calculateBassCut_v2(V0, fc, fs, bassNum, iDenCoeff);
        break;
    case EQ_TREBLE_CUT:
        calculateTrebleCut_v2(V0, fc, fs, bassNum, iDenCoeff);
        break;
    case EQ_BAND_CUT:
        calculatebandCut_v2(V0, fc, fs, Q, bassNum, iDenCoeff);
        break;
    default:
        //printf("\nwrong eq type. exiting...");
        //exit(0);
        break;
    } /* end of switch*/

    for(i = 0; i < 2; ++i)
    {
        *bassDen++   = iDenCoeff[i+1];
    }

} // end of wrapper function to calculate coefs.

/*****************************************************************/
/* Function: calculateBassBoost                                  */
/*****************************************************************/
/* Description: Wrapper function that calls the required function*/
/*              to calculate the coefs for bassboost case        */
/*****************************************************************/

int32 calculateBassBoost_v2(int32 V0, int32 fc, int32 fs, int32 *bassNum, int32 *bassDen)
{
    int32 mySR, DEN, K, index;
    mySR = 1;

    /* calculate V0, V0 in Q28*/
    index = (int32)V0+EQ_GAIN_INDEX_OFFSET;
    if(index > 15)
        index = 15;

    V0=m_V0LinearTable_v2[index];
    //V0=m_V0LinearTable_v2[(int32)V0+EQ_GAIN_INDEX_OFFSET];
    /* return DEN = Q24, K = Q28 */
    DEN = calculateBoostDEN_v2(fc, fs, &K);
#ifdef DEBUG
    printf("%DEN q24= 0x%x, K q28 =0x%x\n", DEN, K);
    printf("%DEN fp= %04f, K fp = %04f\n", (float)DEN/268435456.0*16, (float)K/268435456.0);
#endif
    /* calculate the numerator coeffs, mySR is the numerators Qfactor*/
    mySR = calculateNumeratorBassBoost_v2(V0, K, bassNum, DEN);
    /* calculate the denominator coeffs, Qfactor is fixed to 30*/
    calculateDenominatorBassBoost_v2(K, bassDen, DEN);
    mySR = 32-mySR;  // Q factors for the numerator.
    return mySR;
}

/* Equations:

Wc = 2*pi*(0.1/32); % cut off freq =  2*pi*fc/fs
K = Wc/2
V0 = 10^(6/20);
DEN = (1+(2^0.5)*K+K^2);
B1(1) = (1+(2*V0)^0.5*K+V0*(K^2))/DEN;
B1(2) = (2*(V0*(K^2)-1))/DEN;
B1(3) = (1-((2*V0)^0.5)*K+V0*(K^2))/DEN;
A1(1) = 1;
A1(2) = (2*(K^2-1))/DEN;
A1(3) = ( 1- (2^0.5)*K + (K^2) )/DEN;
*/
/*===============================================================*/
/* Description: calculates K and DEN for given filter            */
/*              uses newton's devide method.                     */
/* Inputs:      fc: cutoff or center freq.                       */
/*              fs: sampling freq.                               */
/* Outputs:     Kq28=pi*(fc/fs), Q28.                            */
/*              DEN = (1+(2^0.5)*K+K^2); Q24                     */
/*===============================================================*/
int32 calculateBoostDEN_v2(int32 cutoffQ0, int32 SamplingFreqQ0, int32* KQ28)
{
    int32 piQ29;
    int32 SR, K, oneQ24, sqrt2Q30, x, y;
    int64 temp64;

    piQ29 = 0x3243F6A8*2;
    oneQ24 = 1<<24;
    sqrt2Q30 = 0x5A827998;

    /***** Calculate K = tan(fc/Fs*pi) *****/
    // Compute fc/Fs with qfactor = SR
    dsplib_approx_divideEQFD_v2(cutoffQ0, SamplingFreqQ0, &K, &SR);
    // fc/Fs < 1 always, hence represent it in Q31
    if (SR > 31) {
        K = K >> (SR-31);
    } else {
        K = K << (31-SR);
    }

    // K = Q31 + Q29 - Q32 + Q0 = Q28
    K = mul32x32Shift2_v2(K,piQ29,0);

    // if using tan(20/44.1*pi), max K = 6.8 (in Q28)
    if(K >= K_MAX)
        K = K_MAX;
    // Compute K = tan(K) = tan(fc/Fs*pi), output is still Q28
    K = tangent_v2(K, 20);

#ifdef DEBUG
    printf("\nK = 0x%x = %d; inQ28 \n", K, K);
#endif


    /**** Calculate DEN = (1+(2^0.5)*K+K^2); ****/
    // sqrt2Q30 = 2^0.5 in Q30
    // Compute x = (2^0.5)*K in Q26
    // Q28 + Q30 - Q32 + Q0 = Q26
    x = mul32x32Shift2_v2(K,sqrt2Q30,0);

    // Compute y = K*K in Q24
    // Q28 + Q28 - Q32 + Q0 = Q24
    y = mul32x32Shift2_v2(K,K,0);

    // (2^0.5)*K is in Q26, needs 2-bit right shift before summation
    temp64 = (int64)oneQ24 + ((int64)x>>2) + (int64)y;
    // DEN = (1+(2^0.5)*K+K^2) in Q24
    x = saturate_s32EQFD_v2(temp64);

    *KQ28 = K;
    return x;
}

/*===============================================================*/
/* Description: calculates Numerator coeff. for BassBoost        */
/* Equation:                                                     */
/*          B1(1) = (1+(2*V0)^0.5*K+V0*(K^2))/DEN;               */
/*          B1(2) = (2*(V0*(K^2)-1))/DEN;                        */
/*          B1(3) = (1-((2*V0)^0.5)*K+V0*(K^2))/DEN;             */
/* inputs: V0 q28, K q28, DEN q24                                */
/* Output: Smax -- Qfactor of the numerators                     */
/*         numerator[], Qfactor is Smax                          */
/*===============================================================*/
int32 calculateNumeratorBassBoost_v2(int32 V0, int32 K, int32 *numerator, int32 DEN)
{
    int32 V0double, V0sqrt, V02sqrtQ28, KSqrQ24, temp1, temp2, normb;
    int32 oneQ24 ,     sqrt2Q30;
    int32 B1Q24, B2Q24, B3Q24, SR;
    int32 B1Q30, B2Q30, B3Q30;
    int32 SR1,SR2,SR3,Smax;
    int64 temp64;

    oneQ24 = 1<<24;
    sqrt2Q30=0x5A827998;

    /* calculate KsqrQ24 = Q24  */
    KSqrQ24 = mul32x32Shift2_v2(K, K, 0);

    /**** Calculate sqrt(V0*2) in Q28 ****/
    // v0DOUBLE is in Q28/
    V0double = V0;
    // Compute V0sqrt = Sqrt(V0) in Q14 */
    V0sqrt=sqrtfpEQFD_v2(V0double);
    // Shift V0sqrt to be Q28
    V0sqrt = V0sqrt << 14;
    // Compute sqrt(2*V0) = sqrt(2)*sqrt(v0) in Q28
    // sqrt(V0): Q28 + Q30 - Q32 + Q2 = Q28
    V02sqrtQ28 = mul32x32Shift2_v2(V0sqrt,sqrt2Q30,2);

    /* Calculate B(0) = 1+(2*V0)^0.5*K+V0*(K^2); */
    // temp1 = K*sqrt(2*V0): Q28 + Q28 - Q32 + Q0 = Q24
    temp1 = mul32x32Shift2_v2(K,V02sqrtQ28,0);

    // For large V0 and K, V0*K^2 might need up to 9 integer bits
    // temp2 = V0*(K^2): Q28 + Q24 - Q32 + Q2 = Q22
    temp2 = mul32x32Shift2_v2(V0,KSqrQ24,2);

    // Compute 64-bit sum: 1+(2*V0)^0.5*K+V0*(K^2);  in Q24
    temp64 = (int64)oneQ24+ (int64)temp1 + ((int64)temp2<<2);
    // extra function now.
    qdsp_norm64EQFD_v2(temp64,&normb,&SR1);
#ifdef DEBUG
    printf("\n Normalized value=%d,SR=%d\n",normb,SR1);
#endif
    // B1Q24's Qfactor is 24+SR1
    B1Q24 = normb; //Q fctr = sr1+24;

    /* Calculate B1 = 2*(V0*K^2 - 1); now */
    // Still in Q24
    temp64 = (((int64)temp2<<2) - (int64)oneQ24)<<1;
    qdsp_norm64EQFD_v2(temp64,&normb,&SR2);
    B2Q24 = normb;
#ifdef DEBUG
    printf("\n Normalized value=%d,SR=%d\n",normb,SR2);
#endif

    /* Calculate B2 = 1-sqrt(2*V0)*K +V0*K^2; */
    temp64 = (int64)oneQ24 - (int64)temp1 + ((int64)temp2<<2);
    qdsp_norm64EQFD_v2(temp64,&normb,&SR3);
#ifdef DEBUG
    printf("\n Normalized value=%d,SR=%d\n",normb,SR3);
#endif
    B3Q24 = (int32)normb;       // no overflows.

#ifdef DEBUG
    printf("\n Before divide: %x,\t%x,\t%x",B1Q24,B2Q24,B3Q24);
    printf("\n SRs=%d\t%d\t%d\n",SR1,SR2,SR3);
#endif

    // now divide by DEN
    dsplib_approx_divideEQFD_v2(B1Q24, DEN ,&B1Q30, &SR);
    SR1 = SR+SR1;
    dsplib_approx_divideEQFD_v2(B2Q24, DEN ,&B2Q30, &SR);
    SR2 = SR+SR2;
    dsplib_approx_divideEQFD_v2(B3Q24, DEN ,&B3Q30, &SR);
    SR3 = SR+SR3;
#ifdef DEBUG
    printf("\n After divide: %d,\t%d,\t%d",B1Q30,B2Q30,B3Q30);
    printf("\n SRs=%d\t%d\t%d\n",SR1,SR2,SR3);
#endif

    // find min of the SRs, the Q-factor for numerator fixed point representation
    Smax = (SR1>SR2)?SR2:SR1;
    Smax= (Smax>SR3)?SR3:Smax;

    numerator[0] = B1Q30>>(SR1-Smax);
    numerator[1] = B2Q30>>(SR2-Smax);
    numerator[2] = B3Q30>>(SR3-Smax);
    return Smax;
}

/*===============================================================*/
/* Description: calculates denominator coeff. for BassBoost      */
/* Equations:                                                    */
/*          A1(1) = 1;                                           */
/*          A1(2) = (2*(K^2-1))/DEN;                             */
/*          A1(3) = ( 1- (2^0.5)*K + (K^2) )/DEN;                */
/* inputs:  K Q28, DEN Q24                                       */
/* Output:  denominator[], fixed to Q30                          */
/*===============================================================*/
int32 calculateDenominatorBassBoost_v2(int32 K, int32 *denomenator, int32 DEN)
{
    int32 KSqrQ24, temp1;
    int32 oneQ24, sqrt2Q30 __attribute__ ((unused)), sqrt2Q28;
    int32 A1Q24, A2Q24, SR;
    int32 A0Q30, A1Q30, A2Q30;
    int64 temp64;

    oneQ24 = 1<<24;
    sqrt2Q30=0x5A827999;
    sqrt2Q28 = 379625062;

    /* Calculate A(1)=(2*(K^2-1));  in Q24 */
    // K is in Q28
    // KSqrQ24=K*K: Q28 + Q28 - Q32 + Q0 = Q24
    KSqrQ24 = mul32x32Shift2_v2(K, K, 0);
    temp64 = 2*((int64)KSqrQ24 - (int64)oneQ24);
    A1Q24 = saturate_s32EQFD_v2(temp64);

    /* Calculate A(2)=( 1- (2^0.5)*K + (K^2)); in Q24 */
    // temp1 = (2^0.5)*K: Q28 + Q28 -Q32 + Q0 = Q24
    temp1 = mul32x32Shift2_v2(K, sqrt2Q28, 0);
    temp64 = (int64)oneQ24 - (int64)temp1 + (int64)KSqrQ24;
    A2Q24 = saturate_s32EQFD_v2(temp64);

#ifdef DEBUG
    printf("A1q24=%d, A2q24=%d \n", A1Q24, A2Q24);
    printf("A1q24=%f, A2q24=%f \n\n", (float)A1Q24/268435456.0*16, (float)A2Q24/268435456.0*16);
#endif

    /* divide by DEN */
    // Assume all denominators are < 2, fixed Qfactor to 30
    A0Q30 = 1 << 30;

    // Compute A1Q30
    dsplib_approx_divideEQFD_v2(A1Q24, DEN ,&A1Q30, &SR);
#ifdef DEBUG
    printf("dsplib_divide: DEN= %d, %d, SR: %d\n", DEN, A1Q30, SR);
#endif
    if (SR>30)
        A1Q30 = A1Q30 >> (SR-30);
    else
        A1Q30 = A1Q30 << (30-SR);

    // Compute A2Q30
    dsplib_approx_divideEQFD_v2(A2Q24, DEN ,&A2Q30, &SR);
#ifdef DEBUG
    printf("dsplib_divide: DEN= %d, %d, SR: %d\n", DEN, A2Q30, SR);
#endif
    if (SR>30)
        A2Q30 = A2Q30 >> (SR-30);
    else
        A2Q30 = A2Q30 << (30-SR);

    denomenator[0]=A0Q30;
    denomenator[1]=A1Q30;
    denomenator[2]=A2Q30;
    return 1;
}



/******************************************************/
/* Function to calculate the coefficients of Bandboost*/
/* Inputs: V0 (q0), Q(q0) fc(q0), fs(q0),             */
/* Equations: K=Wc/2. Wc=28pi*fc/fs                   */
/*            DEN = 1+k/Q+k^2                         */
/*            V0 = 10^(gain/20)                       */
/*            B(0)= (1+V0*K/Q+K^2)/DEN                */
/*            B(1)= 2*(K^2-1)/DEN                     */
/*            B(2)= (1-V0K/Q +K^2)/DEN                */
/*            A(1)= 2*(K^2-1)/DEN                     */
/*            A(2)= (1-K/Q+k^2)/DEN                   */
/******************************************************/
int32 calculateBandBoost_v2(int32 V0, int32 que, int32 fc, int32 fs, int32 *bassNum, int32 *bassDen)
{
    int32 mySR, DEN, K,KbyQ, index;
    /* Calcualte K*/
    /* Themax value of K is pi/2 = 1.5708*/
    /* hence the Q factor can be Q2.30 (unsigned)*/

    /* calculate V0, V0 in Q28*/
    index = (int32)V0+EQ_GAIN_INDEX_OFFSET;
    if(index > 15)
        index = 15;

    V0=m_V0LinearTable_v2[index];

    /* calculate DEN, returns K,KbyQ & DEN in Q24.*/
    DEN=calculateBandBoostDEN_v2(fc, fs, que,&K, &KbyQ);
#ifdef DEBUG
    printf("%DEN q24= 0x%x, K q28 =0x%x\n", DEN, K, KbyQ);
    printf("%DEN fp= %04f, K fp = %04f\n", (float)DEN/268435456.0*16, (float)K/268435456.0);
#endif

    /* calculate the numerator coeffs, mySR is the numerators Qfactor*/
    mySR = calculateNumBandBoost_v2(V0, K, bassNum, DEN,KbyQ);
    /* calculate the denominator coeffs, Qfactor fixed to 30 */
    calculateDenBandBoost_v2(K, bassDen, DEN,KbyQ);
    return (32-mySR);
}

/*===============================================================*/
/* Description: calculates K and DEN for given filter            */
/*              uses dsp_approx_divide                           */
/* Inputs:      fc: cutoff or center freq.                       */
/*              fs: sampling freq.                               */
/* Outputs:     Kq28=pi*(fc/fs), q28.                            */
/*              DEN = 1+k/Q+k^2; q28                             */
/*===============================================================*/
int32 calculateBandBoostDEN_v2(int32 cutoffQ0, int32 SamplingFreqQ0, int32 queQ0,int32* KQ28,int32 *KbyQ27)
{
    int32 piQ29;
    int32 SR, K, oneQ24, x, y;
    int32 invque,queSR;
    int64 temp64;

    piQ29 = 0x3243F6A8*2;
    oneQ24 = 1<<24;

    /*calculate 1/Q. output: invque with qfactor=SR*/
    // if (queQ0 < 256);
    //  queQ0 = 256;
    dsplib_approx_divideEQFD_v2(1, queQ0, &invque, &queSR);
#ifdef DEBUG
    printf("\n invQ =%x SR=%d\n",invque,queSR);
#endif
    //queQ0 = Q*2^8: Q is in Q8 when passed in
    queSR = queSR -8;

    // Assume Q > 1/16, hence convert Q to Q27
    if (queSR > 27) {
        invque = invque >> (queSR-27);
    } else {
        invque = s32_shl_s32_sat(invque, (int16)(27-queSR));
    }


    /***** Calculate K = tan(fc/Fs*pi) *****/

    /* Compute fc/Fs with qfactor = SR */
    dsplib_approx_divideEQFD_v2(cutoffQ0, SamplingFreqQ0, &K, &SR);
    // fc/Fs < 1 always, hence represent it in Q31
    if (SR > 31) {
        K = K >> (SR-31);
    } else {
        K = K << (31-SR);
    }

    /* Compute K = K*pi = fc/Fs*pi */
    // K = Q31 + Q29 - Q32 + Q0 = Q28
    K = mul32x32Shift2_v2(K,piQ29,0);

    // if using tan(20/44.1*pi), max K = 6.8 (in Q28)
    if(K >= K_MAX)
        K = K_MAX;
    // Compute K = tan(K) = tan(fc/Fs*pi), output is still Q28
    K = tangent_v2(K, 20);
    *KQ28 = K;

#ifdef DEBUG
    printf("\nK = 0x%x = %d; inQ28 \n", K, K);
    printf("\n invQ =%x; in Q31",invque);
#endif

    /**** Calculate DEN = DEN = 1+k/Q+k^2; *****/

    /* x= K *invQ, */
    // Q28 + Q27 - Q32 + Q4 = Q27
    // if K is too large and Q is too small, K/Q will be saturated to 2^31-1.
    // This will affect the actual designed filter's bandwidth
    x = mul32x32Shift2_v2(K,invque,4);


    *KbyQ27 = x;
    //Convert K/Q to Q24
    x = x>>3;
#ifdef DEBUG
    printf("\n KbyQ=%x in Q24",x);
#endif

    /* y = K*K, q24 */
    // Q28 + Q28 - Q32 + Q0 = Q24
    y = mul32x32Shift2_v2(K,K,0);

    /* x = 1 + K/Q + K^2; */
    // output is in Q24
    temp64 = oneQ24+x+y;
    x = saturate_s32EQFD_v2(temp64);

    return x;
} /* End of function to calculate DEN for bandboost*/

/*===============================================================*/
/* Description: calculates Numerator coeff. for BandBoost        */
/*            B(0)= (1+V0*K/Q+K^2)/DEN                           */
/*            B(1)= 2*(K^2-1)/DEN                                */
/*            B(2)= (1-V0K/Q +K^2)/DEN                           */
/* inputs: V0 Q28, K Q28, DEN Q24, KbyQ Q27                      */
/* Output: numerator[], Q30                                      */
/*===============================================================*/
int32 calculateNumBandBoost_v2(int32 V0, int32 K, int32 *numerator, int32 DEN, int32 KbyQ)
{
    int32  V0KbyQ,  KSqrQ24;
    int32 oneQ24, normb;
    int32 B0Q24,B1Q24, B2Q24, SR,SR0,SR1,SR2,Smax;
    int32 B0Q30, B1Q30, B2Q30;
    int64 b0,b1,b2;

    oneQ24 = 1<<24;

    /**** calculate V0*K/Q ****/

    /* Calculate V0KbyQ = V0*K/Q, q26 */
    // V0 = 10^(G/20) in Q28 format, using table look-up to find its value
    // KbyQ = K/Q in Q27 format, got from DEN computation
    // V0KbyQ = K/Q: Q28 + Q27 - Q32 + Q1 = Q24
    V0KbyQ = mul32x32Shift2_v2(V0,KbyQ,1);
#ifdef DEBUG
    printf("\nVokbyq=%d",V0KbyQ);
#endif

    /* calculate KsqrQ24 = K*K in q24  */
    // We need Q25 at least if max(K) = tan(20/44.1*pi)
    // KSqrQ24 = K*K: Q28 + Q28 - Q32 + Q0 = Q24
    KSqrQ24 = mul32x32Shift2_v2(K, K, 0);

    /* Calculate b(0) = (1+V0*K/Q+(K^2)) */
    // b0 in Q24
    b0 = (int64)oneQ24 + (int64)V0KbyQ + (int64)KSqrQ24;
#ifdef DEBUG
    printf("\nb0=%d",b0);
#endif
    qdsp_norm64EQFD_v2(b0,&normb,&SR0);
    // although it's called B0Q24, it is not in Q24 now, but Q(24+SR1)
    B0Q24 = normb;

    /* Calculate b(1) = 2*(K^2-1) */
    // b1 in Q24, for if Kmax = tan(20/44.1*pi), (K^2-1)*2 needs Q24
    b1 = (KSqrQ24-oneQ24)<<1;
    qdsp_norm64EQFD_v2(b1,&normb,&SR1);
    // although it's called B1Q24, it is not in Q24 now, but Q(24+SR2)
    B1Q24 = normb;

    /* Calculate b(2)= (1-V0K/Q +K^2)*/
    // b2 in Q24, 64-bit,
    b2 = (int64)oneQ24 - (int64)V0KbyQ + (int64)KSqrQ24;
    qdsp_norm64EQFD_v2(b2,&normb,&SR2);
    // although it's called B2Q24, it is not in Q28 now, but Q(24+SR3)
    B2Q24 = normb;
#ifdef DEBUG
    printf("B0q24=%d, B1q24=%d, B2q24=%d \n", B0Q24, B1Q24, B2Q24);
    printf("B1q24=%04f, B2q24=%04f, B3q24=%04f \n\n",
        (float)B0Q24/268435456.0*16,(float)B1Q24/268435456.0*16, (float)B2Q24/268435456.0*16);
#endif


    /* b0,b1,b2 divided by DEN */
    //B0Q30 is in Q(SR1+SR) format at first, then converted to Q30
    dsplib_approx_divideEQFD_v2(B0Q24, DEN ,&B0Q30, &SR);
    SR0=SR+SR0;
#ifdef DEBUG
    printf("dsplib_divide: %d, SR: %d\n", B0Q30, SR0);
#endif

    //B1Q30 is in Q(SR2+SR) format at first, then converted to Q30
    dsplib_approx_divideEQFD_v2(B1Q24, DEN ,&B1Q30, &SR);
    SR1=SR+SR1;

#ifdef DEBUG
    printf("dsplib_divide: %d, SR: %d\n", B1Q30, SR1);
#endif

    //B2Q30 is in Q(SR3+SR) format at first, then converted to Q30
    dsplib_approx_divideEQFD_v2(B2Q24, DEN ,&B2Q30, &SR);
    SR2=SR2+SR;
#ifdef DEBUG
    printf("dsplib_divide: %d, SR: %d\n", B2Q30, SR2);
#endif

    Smax = (SR0>SR1)?SR1:SR0;
    Smax= (Smax>SR2)?SR2:Smax;

    numerator[0] = B0Q30>>(SR0-Smax);
    numerator[1] = B1Q30>>(SR1-Smax);
    numerator[2] = B2Q30>>(SR2-Smax);

#ifdef DEBUG
    printf("\nQ factor num =%d\n",Smax);
#endif
    return Smax;
} /* end of band boost num calculations.*/

/*===============================================================*/
/* Description: calculates denominator coeff. for BassBoost      */
/*            A(1)= 2*(K^2-1)/DEN                                */
/*            A(2)= (1-K/Q+k^2)/DEN                              */
/* inputs:  K q28, DEN q28, KbyQ q28                             */
/* Output: denominator[], q30                                    */
/*===============================================================*/
int32 calculateDenBandBoost_v2(int32 K, int32 *denomenator, int32 DEN, int32 KbyQ27)
{
    int32 oneQ24, KSqrQ24, normb;
    int32 A1Q24, A2Q24, SR, SR1, SR2;
    int32 A0Q30, A1Q30, A2Q30;
    int64 a1, a2;

    oneQ24 = 1<<24;

    /* Calculate KsqrQ24 = K*K */
    // KSqrQ24 = Q28 + Q28 - Q32 + Q0 = Q24
    KSqrQ24 = mul32x32Shift2_v2(K, K, 0);

    /* Calculate a(1) = (2*(K^2-1)), q24 */
    // a1 is in Q24 format;
    a1 = ((int64)KSqrQ24 - (int64)oneQ24) << 1;
    qdsp_norm64EQFD_v2(a1,&normb,&SR1);
    A1Q24 = normb;

    /* Calculate a2 = temp1 = ( 1- K/Q + (K^2) ) */
    // KbyQ27 = K/Q is in Q27, need to be right shifted 3 bits
    // a2 is in Q24 format;
    a2 = (int64)oneQ24 - ((int64)KbyQ27>>3) + (int64)KSqrQ24;
    qdsp_norm64EQFD_v2(a2,&normb,&SR2);
    A2Q24 = normb;

#ifdef DEBUG
    printf("A1q24=%d, A2q24=%d \n", a1, a2);
    printf("A1q24=%f, A2q24=%f \n\n", (float)a1/268435456.0*16, (float)a2/268435456.0*16);
#endif

    /* a0, a1, a2 divide by DEN */
    // Final A0,A1,A2 are always in Q30 format
    A0Q30 = 1 << 30;

    //A1Q30 is in Q(SR2+SR) format at first, then converted to Q30
    dsplib_approx_divideEQFD_v2(A1Q24, DEN ,&A1Q30, &SR);
    SR1=SR+SR1;
    if (SR1 > 30) {
        A1Q30 = A1Q30 >> (SR1-30);
        SR1 = 30;
    }
    else if (SR1 < 30) {
        A1Q30 = A1Q30 << (30-SR1);
        SR1 = 30;
    }
#ifdef DEBUG
    printf("dsplib_divide: DEN= %d, %d, SR: %d\n", DEN, A1Q30, SR1);
#endif

    dsplib_approx_divideEQFD_v2(A2Q24, DEN ,&A2Q30, &SR);
    SR2=SR+SR2;
    if (SR2 > 30) {
        A2Q30 = A2Q30 >> (SR2-30);
        SR2 = 30;
    }
    else if (SR2 < 30) {
        A2Q30 = A2Q30 << (30-SR2);
        SR2 = 30;
    }
#ifdef DEBUG
    printf("dsplib_divide: DEN= %d, %d, SR: %d\n", DEN, A2Q30, SR2);
#endif

    denomenator[0] = A0Q30;
    denomenator[1] = A1Q30;
    denomenator[2] = A2Q30;
    return 1;
}/*end of band boost den coefs calc*/

/********* End of function to calculate bandboost coefs.*****/

/******* Calculate coefficients for the treble cut     ***/

/********************************************************/
/* Function to calculate the coefficients of Treblecut  */
/* Inputs: V0 (q0), Q(q0) fc(q0), fs(q0),               */
/* Equations: K=Wc/2. Wc=28pi*fc/fs                     */
/*            V0 = 10^(gain/20)                         */
/*            DEN = V0+sqrt(2V0)*k+k^2                  */
/*            B(0)= (1+sqrt(2)K+K^2)/DEN                */
/*            B(1)= 2*(K^2-1)/DEN                       */
/*            B(2)= (1-sqrt(2)*K +K^2)/DEN              */
/*            A(1)= 2*(K^2-V0)/DEN                      */
/*            A(2)= (V0-sqrt(2*V0)K+k^2)/DEN            */
/********************************************************/
void calculateTrebleCut_v2(int32 V0, int32 fc, int32 fs, int32 *bassNum, int32 *bassDen)
{
    int32 DEN, K,Ksqrt2V0, index;
    /* Calcualte K*/
    /* Themax value of K is pi/2 = 1.5708*/
    /* hence the Q factor can be Q2.30 (unsigned)*/

    /* calculate V0, V0 in Q28*/
    index = (int32)V0+EQ_GAIN_INDEX_OFFSET;
    if(index > 15)
        index = 15;

    V0=m_V0LinearTable_v2[index];
    //V0=m_V0LinearTable_v2[(int32)V0+EQ_GAIN_INDEX_OFFSET];

    /* calculate DEN, returns K,KbyQ & DEN in Q27.*/
    DEN=calculateTrebleCutDEN_v2(fc, fs, V0,&K, &Ksqrt2V0);
#ifdef DEBUG
    printf("%DEN q27= 0x%x, K q28 =0x%x\n", DEN, K, Ksqrt2V0);
    printf("%DEN fp= %04f, K fp = %04f\n", (float)DEN/134217728.0, (float)K/268435456.0);
#endif

    /* calculate the numerator coeffs*/
    calculateNumTrebleCut_v2(V0, K, bassNum, DEN);
    /* calculate the denominator coeffs*/
    calculateDenTrebleCut_v2(K, bassDen, DEN,Ksqrt2V0,V0);
 }

/*===============================================================*/
/* Description: calculates K and DEN for given filter            */
/*              uses dsp_approx_divide                           */
/* Inputs:      fc: cutoff or center freq.                       */
/*              fs: sampling freq.                               */
/* Outputs:     Kq28 = tan(pi*(fc/fs));   Q28                    */
/*              DEN = V0+sqrt(2V0)*k+k^2; Q24                    */
/*              Ksqrt2V0 = sqrt(2V0)*k; Q24                  */
/*===============================================================*/
int32 calculateTrebleCutDEN_v2(int32 cutoffQ0, int32 SamplingFreqQ0,int32 V0, int32* KQ28,int32 *Ksqrt2V0)
{
    int32 piQ29 ,sqrt2Q30;
    int32 SR, K, x, y;
    int32 V0double, V0sqrt, V02sqrtQ26, den;
    int64 temp64;

    piQ29=0x3243F6A8*2;
    sqrt2Q30=0x5A827998;

    /* Compute K = fc/fs with Qfactor of SR */
    dsplib_approx_divideEQFD_v2(cutoffQ0, SamplingFreqQ0, &K, &SR);
    // fc/fs < 1, so convert K to Q31 */
    if (SR > 31) {
        K = K >> (SR-31);
    } else {
        K = K << (31-SR);
    }

    /* Multiply K by pi */
    // K = K*pi: Q31 + Q29 - Q32 + Q0 = Q28
    K = mul32x32Shift2_v2(K,piQ29,0);

    // if using tan(20/44.1*pi), max K = 6.8 (in Q28)
    if(K >= K_MAX)
        K = K_MAX;
    // Compute K = tan(K) = tan(fc/Fs*pi), output is still Q28
    K = tangent_v2(K, 20);

#ifdef DEBUG
    printf("\nK = 0x%x = %d; \n", K, K);
#endif

    /**** Compute (2*V0)^0.5 ****/
    // v0DOUBLE = Q28
    V0double = V0;
    // Take Sqrt of V0; V0sqrt = q14 */
    V0sqrt=sqrtfpEQFD_v2(V0double);
    // V0sqrt shifted to Q28
    V0sqrt = V0sqrt << 14;
    // sqrt(2*V0) = sqrt(2)*sqrt(v0),
    // V0sqrt = (2*V0)^0.5: Q28 + Q30 - Q32 + Q0 = Q26
    V02sqrtQ26 = mul32x32Shift2_v2(V0sqrt,sqrt2Q30,0);

    /**** Calculate DEN = V0+sqrt(2*V0)k+k^2;****/
    // x= K *sqrt(2*V0): Q28 + Q26 - Q32 + Q2 = Q24
    x = mul32x32Shift2_v2(K,V02sqrtQ26,2);
#ifdef DEBUG
    printf("\nX=%d\n",x);
#endif

    // y = K*K in Q24
    y = mul32x32Shift2_v2(K,K,0);

    // den = V0+sqrt(2*V0)k+k^2 in Q24
    temp64 = ((int64)V0>>4)+(int64)x+(int64)y;
    den = saturate_s32EQFD_v2(temp64);
#ifdef DEBUG
    printf("\nDen=%d",(int32)den);
#endif

    // store K*sqrt(2*V0) to Ksqrt2V0 in Q24
    *Ksqrt2V0=x;
    // store K=tan(fc/fs*pi) to K in Q28
    *KQ28 = K;
    return den;
} /* End of function to calculate DEN for Treble Cut*/


/*===============================================================*/
/* Description: calculates Numerator coeff. for TrebleCut        */
/*            B(0)= (1+sqrt(2)K+K^2)/DEN                         */
/*            B(1)= 2*(K^2-1)/DEN                                */
/*            B(2)= (1-sqrt(2)*K +K^2)/DEN                       */
/* inputs: V0 Q28, K Q28, DEN Q24,                               */
/* Output: numerator[], Q30, because numerators < 2 always       */
/*===============================================================*/
int32 calculateNumTrebleCut_v2(int32 V0, int32 K, int32 *numerator, int32 DEN)
{
    int32 KSqrQ24;
    int32 oneQ24, sqrt2Q30, KSqr2Q26;
    int32 B0Q24, B1Q24, B2Q24, SR;
    int32 B0Q30, B1Q30, B2Q30;
    int64 temp64;

    (void)(V0); // unreferenced parameter, this statement avoids level-4 warning of unreferenced parameter
    oneQ24 = 1<< 24;
    sqrt2Q30 = 0x5A827998;

    /* Dalculate KsqrQ28 = K*K in Q24  */
    // KSqrQ24=K*K: Q28 + Q28 - Q32 + Q0 = Q24
    KSqrQ24 = mul32x32Shift2_v2(K, K, 0);

    /* Calculate KSqr2Q28 = sqrt(2)*K in Q26 */
    // KSqr2Q28=sqrt(2)*K: Q30 + Q28 - Q32 + Q0 = Q26
    KSqr2Q26=mul32x32Shift2_v2(sqrt2Q30,K,0);

    /* Calculate B(0) = (1-sqrt(2)K+K^2) in Q24 */
    temp64 = (int64)oneQ24 + ((int64)KSqr2Q26>>2) + (int64)KSqrQ24;
    B0Q24 = saturate_s32EQFD_v2(temp64);

    /* Calculate B(1) = 2*(K^2-1) */
    B1Q24 = (KSqrQ24 - oneQ24)<<1;

    /* Calculate B(2)= (1+sqrt(2)*K +K^2))*/
    temp64 = (int64)oneQ24 - ((int64)KSqr2Q26>>2) + (int64)KSqrQ24;
    B2Q24 = saturate_s32EQFD_v2(temp64);

#ifdef DEBUG
    printf("B0q24=%d, B1q24=%d, B2q24=%d \n", B0Q24, B1Q24, B2Q24);
    printf("B1q24=%04f, B2q24=%04f, B3q24=%04f \n\n",
     (float)B0Q24/268435456.0*16,(float)B1Q24/268435456.0*16, (float)B2Q24/268435456.0*16);
#endif

    /* divide by DEN */
    // Assume all numerators for treble cut should be less than 2. Q factor is fixed to Q30
    dsplib_approx_divideEQFD_v2(B0Q24, DEN ,&B0Q30, &SR);
#ifdef DEBUG
    printf("dsplib_divide: %d, SR: %d\n", B0Q30, SR);
#endif
    // Convert B3Q30 to Q30
    if (SR > 30)
        B0Q30 = B0Q30 >> (SR-30);
    else
        B0Q30 = B0Q30 << (30-SR);

    // Compute B1Q30
    dsplib_approx_divideEQFD_v2(B1Q24, DEN ,&B1Q30, &SR);
#ifdef DEBUG
    printf("dsplib_divide: %d, SR: %d\n", B1Q30, SR);
#endif
    // Convert B1Q30 to Q30
    if (SR > 30)
        B1Q30 = B1Q30 >> (SR-30);
    else
        B1Q30 = B1Q30 << (30-SR);

    // Compute B2Q30
    dsplib_approx_divideEQFD_v2(B2Q24, DEN ,&B2Q30, &SR);
#ifdef DEBUG
    printf("dsplib_divide: %d, SR: %d\n", B2Q30, SR);
#endif
    // Convert B2Q30 to Q30
    if (SR > 30)
        B2Q30 = B2Q30 >> (SR-30);
    else
        B2Q30 = B2Q30 << (30-SR);

    numerator[0] = B0Q30; // to compensate for Q27 of den
    numerator[1] = B1Q30;
    numerator[2] = B2Q30;
    return 1;
} /* end of band boost num calculations.*/

/*===============================================================*/
/* Description: calculates denominator coeff. for TrebleCut      */
/*            A(1)= 2*(K^2-V0)/DEN                               */
/*            A(2)= (V0-sqrt(2*V0)K+k^2)/DEN                     */
/* inputs: V0 Q28, K Q28, DEN Q24,                               */
/*         Ksqrt2V0 = K*sqrt(2*V0) Q24                           */
/* Output: denominator[], Q30                                    */
/*===============================================================*/
int32 calculateDenTrebleCut_v2(int32 K, int32 *denomenator, int32 DEN, int32 Ksqrt2V0, int32 V0)
{
    int32 KSqrQ24;
    int32 A1Q24, A2Q24, SR;
    int32 A0Q30, A1Q30, A2Q30;
    int64 temp64;

    /* Calculate KSqrQ24 = K*K */
    // KSqrQ24 = K*K: Q28 + Q28 -Q32 +Q0 = Q24
    KSqrQ24 = mul32x32Shift2_v2(K, K, 0);

    /* Calculate A(2) = (2*(K^2-V0)) in Q24 */
    // V0 is in Q28, needs right shift by 4-bit
    A1Q24 = (KSqrQ24 - (V0>>4))<<1;

    /* Calculate A(2) = ( V0- sqrt(2*V0)K + K^2 ) */
    // Ksqrt2V0 = K*sqrt(2*V0) got from DEN computation function, Q24
    temp64 = ((int64)V0>>4) - (int64)Ksqrt2V0 + (int64)KSqrQ24;
    A2Q24 = saturate_s32EQFD_v2(temp64);

#ifdef DEBUG
    printf("A1q24=%d, A2q24=%d \n", A1Q24, A2Q24);
    printf("A1q24=%f, A2q24=%f \n\n", (float)A1Q24/268435456.0*16, (float)A2Q24/268435456.0*16);
#endif

    /* divide by DEN */
    // Asuume all the denominators are < 2, fixed to Q30
    // A0 is always normalized to 1. Hence A0Q30 = 2^30
    A0Q30 = 1 << 30;

    // Compute A1Q30
    dsplib_approx_divideEQFD_v2(A1Q24, DEN ,&A1Q30, &SR);
#ifdef DEBUG
    printf("dsplib_divide: DEN= %d, %d, SR: %d\n", DEN, A1Q30, SR);
#endif
    if (SR>30)
        A1Q30 = A1Q30 >> (SR-30);
    else
        A1Q30 = A1Q30 << (30-SR);

    // Compute A2Q30
    dsplib_approx_divideEQFD_v2(A2Q24, DEN ,&A2Q30, &SR);
#ifdef DEBUG
    printf("dsplib_divide: DEN= %d, %d, SR: %d\n", DEN, A2Q30, SR);
#endif
   if (SR>30)
        A2Q30 = A2Q30 >> (SR-30);
    else
        A2Q30 = A2Q30 << (30-SR);

    denomenator[0]=A0Q30;
    denomenator[1]=A1Q30;
    denomenator[2]=A2Q30;
    return 1;
}/*end of band boost den coefs calc*/

/********* End of function to calculate treblecut coefs.*****/

/******* Calculate coefficients for the Bass cut     ***/

/********************************************************/
/* Function to calculate the coefficients of BassCut    */
/* Inputs: V0 (q0), Q(q0) fc(q0), fs(q0),               */
/* Equations: K=tan(Wc/2). Wc=2*pi*fc/fs                */
/*            V0 = 10^(gain/20)                         */
/*            DEN = 1+sqrt(2V0)*k+V0*k^2                */
/*            B(0)= (1+sqrt(2)K+K^2)/DEN                */
/*            B(1)= 2*(K^2-1)/DEN                       */
/*            B(2)= (1-sqrt(2)*K +K^2)/DEN              */
/*            A(1)= 2*(V0*K^2-1)/DEN                    */
/*            A(2)= (1-sqrt(2*V0)K+V0*k^2)/DEN          */
/********************************************************/
void calculateBassCut_v2(int32 V0, int32 fc, int32 fs, int32 *bassNum, int32 *bassDen)
{
    int32 DEN, K,Ksqrt2V0,denSR,index;
    /* Calcualte K*/
    /* Themax value of K is pi/2 = 1.5708*/
    /* hence the Q factor can be Q2.30 (unsigned)*/

    /* calculate V0, V0 in Q28*/
    index = (int32)V0+EQ_GAIN_INDEX_OFFSET;
    if(index > 15)
        index = 15;

    V0=m_V0LinearTable_v2[index];
    //V0=m_V0LinearTable_v2[(int32)V0+EQ_GAIN_INDEX_OFFSET];

    /* calculate DEN, returns K,Ksqrt2V0 & DEN in Q28.*/
    DEN=calculateBassCutDEN_v2(fc, fs, V0,&K, &Ksqrt2V0, &denSR);
#ifdef DEBUG
    printf("%DEN q24= 0x%x, K q28 =0x%x,Q=%d\n", DEN, K, denSR);
    printf("%DEN fp= %04f, K fp = %04f\n", (float)DEN/268435456.0*16, (float)K/268435456.0);
#endif

    /* calculate the numerator coeffs, Qfactor fixed to Q30 */
    calculateNumBassCut_v2(V0, K, bassNum, DEN,denSR);
    /* calculate the denominator coeffs, Qfactor fixed to Q30 */
    calculateDenBassCut_v2(K, bassDen, DEN,Ksqrt2V0,V0,denSR);
}

/*===============================================================*/
/* Description: calculates K and DEN for given filter            */
/*              uses dsp_approx_divide                           */
/* Inputs:      fc: cutoff or center freq.                       */
/*              fs: sampling freq.                               */
/* Outputs:     Kq28=pi*(fc/fs); Q28.                            */
/*              DEN = 1+k/Q+k^2; Shift-factor is denSR           */
/*              Ksqrt2V0 = K*sqrt(2*V0); Q24                     */
/*===============================================================*/
int32 calculateBassCutDEN_v2(int32 cutoffQ0,int32 SamplingFreqQ0,int32 V0,int32 *KQ28,int32 *Ksqrt2V0,int32 *denSR)
{
    int32 piQ29 ,sqrt2Q30;
    int32 SR, K, oneQ24, normden;
    int32 V0double,V0sqrt,V02sqrtQ28,KSqrQ24;
    int64 v0ksqr, den;

    piQ29=0x3243F6A8*2;
    sqrt2Q30=0x5A827998;
    oneQ24 = 1<<24;

    /* Calculate K = fc/fs with qfactor = SR */
    dsplib_approx_divideEQFD_v2(cutoffQ0, SamplingFreqQ0, &K, &SR);
    // fc/fs < 1 always, hence convert K to Q31
    if (SR > 31) {
        K = K >> (SR-31);
    } else {
        K = K << (31-SR);
    }

    /* Calculate K = K*pi = fc/fs*pi; in Q28 */
    /* K = K*pi: Q31 + Q29 - Q32 + Q0 = Q28 */
    K = mul32x32Shift2_v2(K,piQ29,0);

    // if using tan(20/44.1*pi), max K = 6.8 (in Q28)
    if(K >= K_MAX)
        K = K_MAX;
    // Compute K = tan(K) = tan(fc/Fs*pi), output is still Q28
    K = tangent_v2(K, 20);

#ifdef DEBUG
    printf("\nK = 0x%x = %d; \n", K, K);
#endif

    /* Calculate sqrt(2*V0) in Q28 */
    // V0double is in Q28
    V0double = V0;
    // Take Sqrt of V0; V0sqrt = Q14
    V0sqrt=sqrtfpEQFD_v2(V0double);
    // Left-shift V0sqrt to make it Q28
    V0sqrt = V0sqrt << 14;
    // Calculate V02sqrtQ28 = sqrt(2)*sqrt(V0); in Q28
    // V02sqrtQ28=sqrt(2*V0): Q26 + Q30 - Q32 + Q2 = Q28
    V02sqrtQ28=mul32x32Shift2_v2(V0sqrt,sqrt2Q30,2);

    /* Calculate Ksqrt2V0 = sqrt(2*V0)*K; in Q24 */
    // Ksqrt2V0 = K*sqrt(2*V0): Q28 + Q28 - Q32 + Q0 = Q24
    *Ksqrt2V0 = mul32x32Shift2_v2(V02sqrtQ28,K,0); // sqrt(2*V0)*K


    /* calculate V0*K*K; in Q24  */
    // KSqrQ24 = K*K: Q28 + Q28 -Q32 + Q0 = Q24
    KSqrQ24 = mul32x32Shift2_v2(K, K, 0);  // no overflow till now
    // compute v0ksqr = K*K*V0: Q28 + Q24 - Q32 + Q2 = Q22
    v0ksqr = mul32x32Shift2_v2(V0,KSqrQ24,2);

    /* calculate den = (1 + sqrt(2*V0)*K + V0*K*K); in Q24 */
    den = (int64)oneQ24 + (int64)(*Ksqrt2V0) + (v0ksqr<<2);
    qdsp_norm64EQFD_v2(den,&normden,denSR);
#ifdef DEBUG
    printf("\n denSR = %d", *denSR);
#endif

    *KQ28 =K;
    return normden;
} /* End of function to calculate DEN for basscut*/

/*===============================================================*/
/* Description: calculates Numerator coeff. for BassCut          */
/*            B(0)= (1+sqrt(2)K+K^2)/DEN                         */
/*            B(1)= 2*(K^2-1)/DEN                                */
/*            B(2)= (1-sqrt(2)*K +K^2)/DEN                       */
/* inputs: V0 q28, K q28, DEN Qfactor is denSR                   */
/* Output: numerator[], fixed Qfactor to Q30                     */
/*===============================================================*/
int32 calculateNumBassCut_v2(int32 V0 , int32 K, int32 *numerator, int32 DEN, int32 denSR)
{
    // V0;
    int32 KSqrQ24, KSqr2Q26;
    int32 oneQ24 , sqrt2Q30;
    int32 B0Q24,B1Q24, B2Q24, SR;
    int32 B0Q30, B1Q30, B2Q30;
    int64 temp64;

    (void)(V0); // unreferenced parameter, this statement avoids level-4 warning of unreferenced parameter
    sqrt2Q30=0x5A827998;
    oneQ24 = 1<<24;

    /* calculate KsqrQ24 = K*K; in Q24  */
    KSqrQ24 = mul32x32Shift2_v2(K, K, 0);

    /* calculate KSqr2Q24 = qrt(2)*K; in Q26 */
    // KSqr2Q26=sqrt(2)*K: Q30 + Q28 - Q32 + Q0 = Q26
    KSqr2Q26=mul32x32Shift2_v2(sqrt2Q30,K,0);

    /* Calculate B(0) = (1+sqrt(2)*K+K^2)*/
    // B0 in Q24
    temp64 = oneQ24 + ((int64)KSqr2Q26>>2) + (int64)KSqrQ24;
    B0Q24 = saturate_s32EQFD_v2(temp64);

    /* Calculate B(1) = 2*(K^2-1) */
    // B1 in Q24
    temp64 = ((int64)KSqrQ24 - oneQ24) << 1;
    B1Q24 = saturate_s32EQFD_v2(temp64);

    /* Calculate B(2)= (1-sqrt(2)*K +K^2)*/
    // B2 in Q24
    temp64 = oneQ24 - ((int64)KSqr2Q26>>2) + (int64)KSqrQ24;
    B2Q24 = saturate_s32EQFD_v2(temp64);

#ifdef DEBUG
    printf("B0q24=%d, B1q24=%d, B2q24=%d \n", B0Q24, B1Q24, B2Q24);
    printf("B0q24=%04f, B1q24=%04f, B2q24=%04f \n\n",
     (float)B0Q24/268435456.0*16,(float)B1Q24/268435456.0*16, (float)B2Q24/268435456.0*16);
#endif

    /* divide by DEN */
    // B0Q30: Qfactor of SR
    dsplib_approx_divideEQFD_v2(B0Q24, DEN ,&B0Q30, &SR);
#ifdef DEBUG
    printf("dsplib_divide: %d, SR: %d\n", B0Q30, SR);
#endif
    SR=SR-denSR;
    /* Convert B3Q30 to Q30 */
    // Assume BassCut numerator coeffs < 2 always
    if (SR > 30)
        B0Q30 = B0Q30 >> (SR-30);
    else
        B0Q30 = B0Q30 << (30-SR);

    // B1Q30: Qfactor of SR
    dsplib_approx_divideEQFD_v2(B1Q24, DEN ,&B1Q30, &SR);
#ifdef DEBUG
    printf("dsplib_divide: %d, SR: %d\n", B1Q30, SR);
#endif
    SR=SR-denSR;
    /* Convert B1Q30 to Q30 */
    // Assume BassCut numerator coeffs < 2 always
    if (SR > 30)
        B1Q30 = B1Q30 >> (SR-30);
    else
        B1Q30 = B1Q30 << (30-SR);

    // B2Q30: Qfactor of SR
    dsplib_approx_divideEQFD_v2(B2Q24, DEN ,&B2Q30, &SR);
#ifdef DEBUG
    printf("dsplib_divide: %d, SR: %d\n", B2Q30, SR);
#endif
    SR=SR-denSR;
    /* Convert B2Q30 to Q30 */
    // Assume BassCut numerator coeffs < 2 always
    if (SR > 30)
        B2Q30 = B2Q30 >> (SR-30);
    else
        B2Q30 = B2Q30 << (30-SR);

    numerator[0] = B0Q30;
    numerator[1] = B1Q30;
    numerator[2] = B2Q30;
    return 1;
} /* end of bass cut num calculations.*/

/*===============================================================*/
/* Description: calculates denominator coeff. for BassCut        */
/*            A(1)= 2*(V0*K^2-1)/DEN                             */
/*            A(2)= (1-sqrt(2*V0)K+V0*k^2)/DEN                   */
/* inputs:  K Q28, DEN's Qfactor is denSR                        */
/* Output: denominator[], Q30                                    */
/*===============================================================*/
int32 calculateDenBassCut_v2(int32 K, int32 *denomenator, int32 DEN,int32 Ksqrt2V0,int32 V0,int32 denSR)
{
    int32 temp, KSqrQ24;
    int32 oneQ24;
    int32 A1Q24, A2Q24, SR1,SR2,SR;
    int32 A0Q30, A1Q30, A2Q30;
    int64 temp2,temp3;
    oneQ24 = 1<<24;

    /* Calculate KSqrQ24 = K*K; in Q24 */
    // KsqrQ24 = K*K: Q28 + Q28 -Q32 + Q0 = Q24
    KSqrQ24 = mul32x32Shift2_v2(K, K, 0);

    /* Calculate A(1) = (2*(V0*K^2-1)), q28 */
    // For large V0 and K, 32-bit is not big enough to hold temp if it's in Q24
    // temp = V0*K*K: Q28 + Q24 - Q32 + Q2 = Q22
    temp = mul32x32Shift2_v2(V0,KSqrQ24,2); // temp in Q24
    // temp2 = V0*K^2 in Q24 using 64-bit
    temp2 = (int64)temp<<2;
    // temp2 = 2*(V0*K^2 - 1) in Q24 using 64-bit
    temp3 = (int64)(temp2-(int64)oneQ24)*2;

    // A(1) is stored in A1Q24 has shift-factor SR1
    qdsp_norm64EQFD_v2(temp3 ,&A1Q24,&SR1);


    /* Calculate A(2) = ( 1- sqrt(2*V0)K + V0*K^2 ) */
    // Ksqrt2V0 = sqrt(V0*2)*K; in Q24, computed from DEN computation function
    // A(2) is stored in A2Q24, which has shift-factor of SR2
    temp3 = (int64)oneQ24 - (int64)Ksqrt2V0 + (int64)temp2;
    qdsp_norm64EQFD_v2(temp3 ,&A2Q24,&SR2);
   /*  TEST */

#ifdef DEBUG
    printf("A1q24=%d, A2q24=%d \n", A1Q24, A2Q24);
    printf("A1q24=%f, A2q24=%f \n\n", (float)A1Q24/268435456.0, (float)A2Q24/268435456.0);
#endif
    /* divide by DEN */
    // Assume all denominator coeffs < 2, fixed Q-factor to Q30
    A0Q30 = 1 << 30;

    // Compute A1Q30
    dsplib_approx_divideEQFD_v2(A1Q24, DEN ,&A1Q30, &SR);
#ifdef DEBUG
    printf("dsplib_divide: DEN= %d, %d, SR: %d\n", DEN, A1Q30, SR);
#endif
    SR = SR + SR1 - denSR;
    if (SR>30)
        A1Q30 = A1Q30 >> (SR-30);
    else
        A1Q30 = A1Q30 << (30-SR);

    // Compute A2Q30
    dsplib_approx_divideEQFD_v2(A2Q24, DEN ,&A2Q30, &SR);
#ifdef DEBUG
    printf("dsplib_divide: DEN= %d, %d, SR: %d\n", DEN, A2Q30, SR);
#endif
    SR = SR+SR2-denSR;
   if (SR>30)
        A2Q30 = A2Q30 >> (SR-30);
    else
        A2Q30 = A2Q30 << (30-SR);

    denomenator[0]=A0Q30;
    denomenator[1]=A1Q30;
    denomenator[2]=A2Q30;
    return 1;
}/*end of basscut den coefs calc*/

/********* End of function to calculate basscut coefs.*****/

/******* Calculate coefficients for the band cut     ***/

/********************************************************/
/* Function to calculate the coefficients of bandCut    */
/* Inputs: V0 (q0), Q(q0) fc(q0), fs(q0),               */
/* Equations: K=Wc/2. Wc=28pi*fc/fs                     */
/*            V0 = 10^(gain/20)                         */
/*            DEN = 1+V0*k/Q+k^2                        */
/*            B(0)= (1+K/Q+K^2)/DEN                     */
/*            B(1)= 2*(K^2-1)/DEN                       */
/*            B(2)= (1-K/Q +K^2)/DEN                    */
/*            A(1)= 2*(K^2-1)/DEN                       */
/*            A(2)= (1-V0*K/Q+V0*k^2)/DEN               */
/********************************************************/
void calculatebandCut_v2(int32 V0, int32 fc, int32 fs, int32 que, int32 *bandNum, int32 *bandDen)
{
    int32 DEN, K,KbyQ,denSR, index;
    /* Calcualte K*/
    /* Themax value of K is pi/2 = 1.5708*/
    /* hence the Q factor can be Q2.30 (unsigned)*/

    /* calculate V0, V0 in Q28*/
    index = (int32)V0+EQ_GAIN_INDEX_OFFSET;
    if(index > 15)
        index = 15;
    V0 = m_V0LinearTable_v2[index];
    //V0=m_V0LinearTable_v2[(int32)V0+EQ_GAIN_INDEX_OFFSET];

    /* calculate DEN, returns K,KbyQ & DEN in Q28.*/
    DEN=calculatebandCutDEN_v2(fc, fs, V0,que,&K, &KbyQ,&denSR);
#ifdef DEBUG
    printf("%DEN q24= 0x%x, K q27 =0x%x\n", DEN, K, KbyQ);
    printf("%DEN fp= %04f, K fp = %04f\n", (float)DEN/268435456.0, (float)K/268435456.0);
#endif

    /* calculate the numerator coeffs*/
    calculateNumbandCut_v2(V0, K, bandNum, DEN,KbyQ,denSR);
    /* calculate the denominator coeffs*/
    //int32 calculateDenbandCut(int32 K, int32 *denomenator, int32 DEN,int32 Ksqrt2V0,int32 V0,int32 KbyQ28)
    calculateDenbandCut_v2(K, bandDen, DEN,V0,KbyQ,denSR);
}

/*===============================================================*/
/* Description: calculates K and DEN for given filter            */
/*              uses dsp_approx_divide                           */
/* Inputs:      fc: cutoff or center freq.                       */
/*              fs: sampling freq.                               */
/* Outputs:     Kq28=pi*(fc/fs), q28.                            */
/*              DEN = 1+k/Q+k^2; q28                             */
/*===============================================================*/
int32 calculatebandCutDEN_v2(int32 cutoffQ0, int32 SamplingFreqQ0,int32 V0, int32 queQ0,int32* KQ24,int32 *KbyQ27,int32 *denSR)
{
    int32 piQ29;
    int32 SR, K, oneQ24, x, y;
    int32 invque,queSR;
    int64 res;
    int32 norm_den;

    piQ29=0x3243F6A8*2;
    oneQ24=1<<24;

    /* Calculate K = fc/Fs with qfactor = SR */
    // fc/Fs < 1 always, hence represent it using Q31
    dsplib_approx_divideEQFD_v2(cutoffQ0, SamplingFreqQ0, &K, &SR);
    // Convert K to Q31
    if (SR > 31) {
        K = K >> (SR-31);
    } else {
        K = K << (31-SR);
    }

    /* Multiply K by pi, in Q28*/
    // Calculate K = K*pi: Q31 + Q29 - Q32 + Q0 = Q28
    K = mul32x32Shift2_v2(K,piQ29,0);
    // if using tan(20/44.1*pi), max K = 6.8 (in Q28)
    if(K >= K_MAX)
        K = K_MAX;
    K = tangent_v2(K, 20);
#ifdef DEBUG
    printf("\nK = 0x%x = %d; \n", K, K);
#endif

    /* Calculate invque = 1/Q. with qfactor=SR*/
    //if (queQ0 < 256)
    //  queQ0 = 256;
    dsplib_approx_divideEQFD_v2(1, queQ0, &invque, &queSR);
#ifdef DEBUG
    printf("\n invQ =%x SR=%d\n",invque,queSR);
#endif
    //queQ0 = Q*2^8: Q is in Q8 when passed in
    queSR = queSR -8;

    // Assume Q > 1/16 always, hence represent it using Q27
    if (queSR > 27) {
        invque = invque >> (queSR-27);
    } else {
        invque = s32_shl_s32_sat(invque, (int16)(27-queSR));
    }

    /**** Calculate DEN = 1+V0*k/Q+k^2; ****/

    /* Calculate x = K/Q in Q27 */
    // x= K/Q, Q28 + Q27 - Q32 + Q4 = Q27
    // if K is too large and Q is too small, K/Q will be saturated to 2^31-1.
    // This will affect the actual designed filter's bandwidth
    x = mul32x32Shift2_v2(K,invque,4);

    *KbyQ27 = x;
#ifdef DEBUG
    printf("\n KbyQ=%x",x);
#endif

    /* Calculate x= V0*K/Q in Q24       */
    // V0 is obtained using table-look-up, in Q28
    // x = V0*K/Q: Q28 + Q27 - Q32 + Q1 = Q24
    x = mul32x32Shift2_v2(V0,x,1);

    /* Calculate y = K*K in Q24 */
    // y = K*K: Q28 + Q28 - Q32 + Q0 = Q24
    y = mul32x32Shift2_v2(K,K,0);

    res = (int64)oneQ24 + (int64)x + (int64)y;
    qdsp_norm64EQFD_v2(res,&norm_den,denSR);

    *KQ24 = K;
    return norm_den;
} /* End of function to calculate DEN for bandcut*/

/*===============================================================*/
/* Description: calculates Numerator coeff. for bandCut          */
/*            B(0)= (1+K/Q+K^2)/DEN                              */
/*            B(1)= 2*(K^2-1)/DEN                                */
/*            B(2)= (1-K/Q +K^2)/DEN                             */
/* inputs: V0 q28, K q28, DEN Q(denSR), KbyQ (Q27)                     */
/* Output: numerator[], q30                                      */
/*===============================================================*/
int32 calculateNumbandCut_v2(int32 V0, int32 K, int32 *numerator, int32 DEN, int32 KbyQ27, int32 denSR)
{
    int32 KSqrQ24;
    int32 oneQ28 __attribute__ ((unused)), oneQ24;
    int32 B0Q24,B1Q24, B2Q24, SR;
    int32 B0Q30, B1Q30, B2Q30;
    int64 temp64;

    (void)(V0); // unreferenced parameter, this statement avoids level-4 warning of unreferenced parameter
    oneQ28 = 1<<28;
    oneQ24 = 1<<24;

    /* calculate KsqrQ24 = q24  */
    // KsqrQ24 = K*K: Q28 + Q28 - Q32 + Q0 = Q24
    KSqrQ24 = mul32x32Shift2_v2(K, K, 0);

    /* Calculate B(0) = (1+K/Q+K^2)*/
    // B0 in Q24
    temp64 = (int64)oneQ24 +((int64)KbyQ27>>3) + (int64)KSqrQ24;
    B0Q24 = saturate_s32EQFD_v2(temp64);

    /* Calculate B(1) = 2*(K^2-1) */
    // B1 in Q24
    temp64 = (KSqrQ24-(int64)oneQ24) << 1;
    B1Q24 = saturate_s32EQFD_v2(temp64);

    /* Calculate B(2)= (1-K/Q +K^2)*/
    // KbyQ27 = K/Q is got from DEN computaion function, in Q27, needs to right shift 3 bits
    temp64 = (int64)oneQ24 - ((int64)KbyQ27>>3) + (int64)KSqrQ24;
    B2Q24 = saturate_s32EQFD_v2(temp64);

#ifdef DEBUG
    printf("B0q24=%d, B1q24=%d, B2q24=%d \n", B0Q24, B1Q24, B2Q24);
    printf("B1q24=%04f, B2q24=%04f, B3q24=%04f \n\n",
     (float)B0Q24/268435456.0*16,(float)B1Q24/268435456.0*16, (float)B2Q24/268435456.0*16);
#endif

    /* divide by DEN */
    /* Calculate B0Q30 = B0Q24/DEN  */
    // fixed the B0 to Q30 format
    dsplib_approx_divideEQFD_v2(B0Q24, DEN ,&B0Q30, &SR);
#ifdef DEBUG
    printf("dsplib_divide: %d, SR: %d\n", B0Q30, SR);
#endif
    SR = SR-denSR;
    /* Convert B3Q30 to Q30 */
    if (SR > 30)
        B0Q30 = B0Q30 >> (SR-30);
    else
        B0Q30 = B0Q30 << (30-SR);

    /* Calculate B1Q30 = B1Q24/DEN  */
    // fixed the B1 coeffs to Q30 format
    dsplib_approx_divideEQFD_v2(B1Q24, DEN ,&B1Q30, &SR);
#ifdef DEBUG
    printf("dsplib_divide: %d, SR: %d\n", B1Q30, SR);
#endif
    SR = SR-denSR;
    /* Convert B1Q30 to Q30 */
    if (SR > 30)
        B1Q30 = B1Q30 >> (SR-30);
    else
        B1Q30 = B1Q30 << (30-SR);

    /* Calculate B2Q30 = B2Q24/DEN  */
    // fixed the B2 coeffs to Q30 format
    dsplib_approx_divideEQFD_v2(B2Q24, DEN ,&B2Q30, &SR);
#ifdef DEBUG
    printf("dsplib_divide: %d, SR: %d\n", B2Q30, SR);
#endif
    SR=SR-denSR;
    /* Convert B2Q30 to Q30 */
    if (SR > 30)
        B2Q30 = B2Q30 >> (SR-30);
    else
        B2Q30 = B2Q30 << (30-SR);

    numerator[0] = B0Q30;
    numerator[1] = B1Q30;
    numerator[2] = B2Q30;
    return 1;
} /* end of band cut num calculations.*/

/*===============================================================*/
/* Description: calculates denominator coeff. for bandCut        */
/*            A(1)= 2*(K^2-1)/DEN                                */
/*            A(2)= (1-K/Q + k^2)/DEN                            */
/* inputs:  K q28, DEN Q(denSR) KbyQ q27                         */
/* Output: denominator[], q30                                    */
/*===============================================================*/
int32 calculateDenbandCut_v2(int32 K, int32 *denomenator, int32 DEN,int32 V0,int32 KbyQ27, int32 denSR)
{
    int32 KSqrQ24,temp;
    int32 oneQ24;
    int32 A1Q24, A2Q24, SR;
    int32 A0Q30, A1Q30, A2Q30;
    int64 temp64;

    oneQ24 = 1<<24;

    /* Calculate K*K in Q24*/
    // KSqrQ24 = K*K: Q28 + Q28 - Q32 + Q0 = Q24
    KSqrQ24 = mul32x32Shift2_v2(K, K, 0);

    /* Calculate A(1) = (2*(K^2-1)) in Q24 */
    temp64 = (KSqrQ24-oneQ24)*2;
    A1Q24 = saturate_s32EQFD_v2(temp64);

    /* Calculate V0*K/Q in Q24 */
    // V0 is got by table-look-up in Q28
    // temp = V0*K/Q, Q28 + Q27 - Q32 + Q4 = Q24
    temp = mul32x32Shift2_v2(V0,KbyQ27,1);

    /* Calculate A(2) = ( 1- V0*K/Q + V0*K^2 ) */
    temp64 = (int64)oneQ24 - (int64)temp + (int64)KSqrQ24;
    A2Q24 = saturate_s32EQFD_v2(temp64);

#ifdef DEBUG
    printf("A1q24=%d, A2q24=%d \n", A1Q24, A2Q24);
    printf("A1q24=%f, A2q24=%f \n\n", (float)A1Q24/268435456.0*16, (float)A2Q24/268435456.0*16);
#endif

    /* divide by DEN */
    A0Q30 = 1 << 30;

    /* Calculate A1Q30 = A1Q24/DEN  */
    // fixed the A1 coeff to Q30 format
    dsplib_approx_divideEQFD_v2(A1Q24, DEN ,&A1Q30, &SR);
#ifdef DEBUG
    printf("dsplib_divide: DEN= %d, %d, SR: %d\n", DEN, A1Q30, SR);
#endif
    SR=SR-denSR;
    if (SR>30)
        A1Q30 = A1Q30 >> (SR-30);
    else
        A1Q30 = A1Q30 << (30-SR);

    /* Calculate A2Q30 = A2Q24/DEN  */
    // fixed the A2 coeff to Q30 format
    dsplib_approx_divideEQFD_v2(A2Q24, DEN ,&A2Q30, &SR);
#ifdef DEBUG
    printf("dsplib_divide: DEN= %d, %d, SR: %d\n", DEN, A2Q30, SR);
#endif
    SR=SR-denSR;
   if (SR>30)
        A2Q30 = A2Q30 >> (SR-30);
    else
        A2Q30 = A2Q30 << (30-SR);

    denomenator[0]=A0Q30;
    denomenator[1]=A1Q30;
    denomenator[2]=A2Q30;
    return 1;
}/*end of bandcut den coefs calc*/

/********* End of function to calculate bandcut coefs.*****/


/******* Calculate coefficients for the treble boost     ***/

/********************************************************/
/* Function to calculate the coefficients of Trebleboost*/
/* Inputs: V0 (q0), Q(q0) fc(q0), fs(q0),               */
/* Equations: K=Wc/2. Wc=28pi*fc/fs                     */
/*            V0 = 10^(gain/20)                         */
/*            DEN = (1+sqrt(2)*K +K^2)                  */
/*            B(0)= (V0+sqrt(2*V0)K+K^2)/DEN            */
/*            B(1)= 2*(K^2-V0)/DEN                      */
/*            B(2)= (V0-sqrt(2V0)*k+k^2)/DEN            */
/*            A(1)= 2*(K^2-1)/DEN                       */
/*            A(2)= (1-sqrt(2)K+k^2)/DEN                */
/********************************************************/
void calculateTrebleboost_v2(int32 V0, int32 fc, int32 fs, int32 *bassNum, int32 *bassDen, int32 *shift)
{
    int32 mySR __attribute__ ((unused)), DEN, K, index;
    /* Calcualte K*/
    /* Themax value of K is tan(20/44.1*pi) = 6.8*/
    /* hence the Q factor can be Q3.28 (unsigned)*/

    /* calculate V0, V0 in Q28*/
    index = (int32)V0+EQ_GAIN_INDEX_OFFSET;
    if(index > 15)
        index = 15;

    V0=m_V0LinearTable_v2[index];
    //V0=m_V0LinearTable_v2[(int32)V0+EQ_GAIN_INDEX_OFFSET];

    /* calculate DEN, returns K,KbyQ & DEN in Q28.*/
    //DEN=calculateTrebleboostDEN(fc, fs, V0,&K);
    // trebleboost and baseboost use the same DEN
    DEN=calculateBoostDEN_v2(fc, fs,&K);
#ifdef DEBUG
    //printf("%DEN q27= 0x%x, K q28 =0x%x\n", DEN, K, Ksqrt2V0);
    printf("%DEN fp= %04f, K fp = %04f\n", (float)DEN/134217728.0, (float)K/268435456.0);
#endif

    /* calculate the numerator coeffs*/
    *shift= calculateNumTrebleboost_v2(V0, K, bassNum, DEN);
    /* calculate the denominator coeffs*/
    mySR=calculateDenTrebleboost_v2(K, bassDen, DEN);

 }

/*===============================================================*/
/* Description: calculates Numerator coeff. for Trebleboost      */
/*            DEN = (1+sqrt(2)*K +K^2)                  */
/*            B(0)= (V0+sqrt(2*V0)K+K^2)/DEN            */
/*            B(1)= 2*(K^2-V0)/DEN                      */
/*            B(2)= (V0-sqrt(2V0)*k+k^2)/DEN            */
/* inputs: V0 q28, K q28, DEN q27,KbyQ (q28)                     */
/* Output: numerator[], q30                                      */
/*         shift, q0                                             */
/*===============================================================*/
int32 calculateNumTrebleboost_v2(int32 V0, int32 K, int32 *numerator, int32 DEN)
{
    int32 KSqrQ24,V0double,V0sqrt,x;
    int32 sqrt2Q30;
    int32 B0Q24,B1Q24, B2Q24, SR1,SR2,SR3;
    int32 B0Q30, B1Q30, B2Q30;
    int32 Smax;
    int64 den;

    sqrt2Q30=0x5A827998;

    /* calculate KsqrQ24 = Q24  */
    KSqrQ24 = mul32x32Shift2_v2(K, K, 0);

    /**** Calculate sqrt(V0*2) in Q28 ****/
    // v0DOUBLE is in Q28/
    V0double = V0;
    // Compute Sqrt(V0); for V0sqrt in Q14 */
    V0sqrt=sqrtfpEQFD_v2(V0double);
    // Shift V0sqrt to be Q28
    V0sqrt = V0sqrt << 14;
    // Compute sqrt(2*V0) = sqrt(2)*sqrt(v0) in Q28
    // sqrt(V0): Q28 + Q30 - Q32 + Q2 = Q28
    V0sqrt=mul32x32Shift2_v2(V0sqrt,sqrt2Q30,2);


    /* Calculate B(0) = V0+sqrt(2*V0)k+k^2; */
    // Compute x= K *sqrt(2*V0),
    // x = K*sqrt(2*V0): Q28 + Q28 - Q32 + Q0 = Q24
    x = mul32x32Shift2_v2(K,V0sqrt,0);
    // den=V0+sqrt(2*V0)*K+K^2: Q24
    den = ((int64)V0>>4)+(int64)x+(int64)KSqrQ24;
    //den=(int64)den>>1;
    // B0 in Q24
    B0Q24 = saturate_s32EQFD_v2(den);

    /* Calculate B(1) = 2*(K^2-V0),Q24 */
    B1Q24 = (KSqrQ24 - (V0>>4));
    B1Q24 = B1Q24<<1;

    /* Calculate B(2)= (V0-sqrt(2V0)*k+k^2), Q24*/
    den = ((int64)V0>>4)-(int64)x+(int64)KSqrQ24;
    B2Q24 = saturate_s32EQFD_v2(den);

#ifdef DEBUG
    printf("B0q24=%d, B1q24=%d, B2q24=%d \n", B0Q24, B1Q24, B2Q24);
    printf("B1q24=%04f, B2q24=%04f, B3q24=%04f \n\n",
     (float)B0Q24/268435456.0,(float)B1Q24/268435456.0, (float)B2Q24/268435456.0);
#endif
    /* divide by DEN */
    /* B1Q30 = qSR */
    dsplib_approx_divideEQFD_v2(B0Q24, DEN ,&B0Q30, &SR1);
#ifdef DEBUG
    printf("dsplib_divide: %d, SR: %d\n", B0Q30, SR1);
#endif
    /* Convert B3Q30 to Q30 */
    dsplib_approx_divideEQFD_v2(B1Q24, DEN ,&B1Q30, &SR2);
#ifdef DEBUG
    printf("dsplib_divide: %d, SR: %d\n", B1Q30, SR2);
#endif

    dsplib_approx_divideEQFD_v2(B2Q24, DEN ,&B2Q30, &SR3);
#ifdef DEBUG
    printf("dsplib_divide: %d, SR: %d\n", B2Q30, SR3);
#endif

    // find the maximum SR of the three
    //Smax =(SR1>SR2) ? SR1 : SR2;
    //Smax =(Smax>SR3) ? Smax : SR3;
    //SR1=SR1-1;
    //SR2=SR2-1;
    Smax=SR1;
    if(Smax>SR2) Smax=SR2;
    if(Smax>SR2) Smax=SR3;
    numerator[0] = B0Q30>>(SR1-Smax); // to compensate for Q24 of B0,B1
    numerator[1] = B1Q30>>(SR2-Smax);
    numerator[2] = B2Q30>>(SR3-Smax);
#ifdef DEBUG
    printf("\nSmax=%d\n",Smax);
#endif

    return Smax;
} /* end of band boost num calculations.*/

/*===============================================================*/
/* Description: calculates denominator coeff. for Trebleboost    */
/*            A(1)= 2*(K^2-1)/DEN                                */
/*            A(2)= (1-sqrt(2)K+k^2)/DEN                         */
/* inputs:  K q28, DEN q28, KbyQ q28                             */
/* Output: denominator[], q30                                    */
/*===============================================================*/
int32 calculateDenTrebleboost_v2(int32 K, int32 *denomenator, int32 DEN)
{
    int32 KSqrQ24, Ksqrt2Q26;
    int32 oneQ24, sqrt2Q30;
    int32 A1Q24, A2Q24, SR;
    int32 A0Q30, A1Q30, A2Q30;
    int64 temp64;

    sqrt2Q30=0x5A827998;
    oneQ24 = 1<<24;

    /* Compute KsqrQ24 = K*K in Q24 */
    // K is in Q28, KSqrQ24 = K^2: Q28+Q28-Q32+Q0=Q24
    KSqrQ24 = mul32x32Shift2_v2(K, K, 0);

    /* Calculate A(1) = (2*(K^2-1))*/
    // A(1) is in Q24
    A1Q24 = (KSqrQ24-oneQ24)<<1;

    /* Calculate A(2) = ( 1- sqrt(2)K + K^2 ) */
    // Ksqrt2Q26 = K*sqrt(2): Q28+Q30-Q32+Q0 = Q26
    Ksqrt2Q26 = mul32x32Shift2_v2(K,sqrt2Q30,0);

    temp64 = (int64)oneQ24 - ((int64)Ksqrt2Q26>>2) + (int64)KSqrQ24;
    A2Q24 = saturate_s32EQFD_v2(temp64);

#ifdef DEBUG
    printf("A1q24=%d, A2q24=%d \n", A1Q24, A2Q24);
    printf("A1q24=%f, A2q24=%f \n\n", (float)A1Q24/268435456.0*16, (float)A2Q24/268435456.0*16);
#endif

    /**** divide by DEN ****/
    // For denominator, it always assume that denominators are < 2, hence fixed Q fact to 30
    A0Q30 = 1 << 30;

    // Compute A1 = A(1)/DEN with SR Qfactor
    dsplib_approx_divideEQFD_v2(A1Q24, DEN ,&A1Q30, &SR);
#ifdef DEBUG
    printf("dsplib_divide: DEN= %d, %d, SR: %d\n", DEN, A1Q30, SR);
#endif
    // Shift A1 to Q30 format
    if (SR>30)
        A1Q30 = A1Q30 >> (SR-30);
    else
        A1Q30 = A1Q30 << (30-SR);

    // Compute A2 = A(2)/DEN with SR Qfactor
    dsplib_approx_divideEQFD_v2(A2Q24, DEN ,&A2Q30, &SR);
#ifdef DEBUG
    printf("dsplib_divide: DEN= %d, %d, SR: %d\n", DEN, A2Q30, SR);
#endif
    // Shift A1 to Q30 format
   if (SR>30)
        A2Q30 = A2Q30 >> (SR-30);
    else
        A2Q30 = A2Q30 << (30-SR);

    denomenator[0]=A0Q30;
    denomenator[1]=A1Q30;
    denomenator[2]=A2Q30;
    return 1;
}/*end of band boost den coefs calc*/

/********* End of function to calculate trebleboost coefs.*****/

/* input qx, output qx/2 */
int32 sqrtfpEQFD_v2(int32 a)
{
    short i;
    int32 y=0;
    for(i=15;i>=0;i--) {
        if(a-(y+(2<<(i-1)))*(y+(2<<(i-1))) >= 0) {
            y+=(2 << (i-1));
            //printf("%d : %d\n", i, y);
        }
    }
#ifdef DEBUG
    printf("sqrt Result: 0x%x = %04f\n", y, (float)y/4096.0);
#endif
    return y;
}

PPStatus dsplib_approx_invertEQFD_v2(int32 input,int32 *result,int32 *shift_factor)
{
  int32 norm_divisor;
  int32 n1,interp;
  int64 val;
  int32 index;
  int32 r;
  /* check for negative input (invalid) */
  if (input < 0)
  {
    //fprintf(stderr, "dsp_libs.c -> dsplib_approx_invert(): negative input, invalid.\n");
      return PPFAILURE;
    //exit(-1);
  }

  /* bit-align-normalize input */
  qdsp_norm_v2(input, &norm_divisor, &r);
  /* determine inverse LUT index and interpolation factor */
  n1     = norm_divisor >> (15-EQ_DSP_INV_LUT_BITS);
  interp = n1 % (1<<16);
  index  = (n1 >> 16) - (EQ_DSP_INV_LUT_SIZE-1);

  /* inverse linear interpolation between LUT entries */
  val = ((int64)(m_iIinvTable_v2[index])<<16);
  val += (int64)interp * ((int64)(m_iIinvTable_v2[index+1]) - (int64)m_iIinvTable_v2[index]);
  /* return results */
  *result       = (int32)(val>>16);
  *shift_factor = r-31;
  return PPSUCCESS;
}



/******************************************************************/
/* Fucntion: qdsp_norm64EQFD(int64 input,int32 *norm_inp,int32 *shift); */
/******************************************************************/

void qdsp_norm64EQFD_v2(int64 inp,int32 *norm_inp,int32 *shift)
{
  int32 sf;
  int64 inp1;

  sf = 0;
  inp1 = (int64)(inp>>16);
#ifdef DEBUG
  printf("\ninput=%x",inp1);
#endif
  if (inp)
  {
    /* non-zero input, shift left until "normed" */
    while (((inp << 1) ^ ~inp) & (int64)0x8000000000000000LL)
    {
      inp <<= 1;
      sf++;
    }
    inp1 = (int64)(inp>>32)&0xffffffff;
    *norm_inp= (int32)(inp1);
    *shift = sf-32;
  }
  else
  {
    /* zero input, leave as zero */
    *norm_inp      = 0;
    *shift = 0;
  }


}

int32 saturate_s32EQFD_v2(int64 var1)
{
    if (var1 > (int32) 0x7FFFFFFFL)
    {
        var1 = 0x7FFFFFFFL;
    }
    else if (var1 < (int32)0x80000000L)
    {
        var1 = (int32)0x80000000L;
    }
    return ((int32) var1);
}
