/* Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef EQUALIZER_FILTER_DESIGN_H
#define EQUALIZER_FILTER_DESIGN_H

#include "msiir_api.h"
#include "equalizer_api.h"


#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


/*=============================================================================
Constants
=============================================================================*/

#define EQ_MAX_BANDS 12
#define K_MAX 388276097
    
/* Linear gains. Q28 */
/* To convert logarithmic gains to linear gains*/
#define EQ_GAIN_INDEX_OFFSET 0
#define EQ_MAX_GAIN 1500  // millibels; 15 dB
#define EQ_MIN_GAIN -1500  // millibels; -15 dB

/* number of bits to index the inverse lookup table (actually 2^(val-1) segments) */
#define EQ_DSP_INV_LUT_BITS 8
//#define EQ_DSP_SQRT_LUT_BITS 5
//#define EQ_DSP_INV_SQRT_LUT_BITS 5

/* number of entries in the inverse LUT */
#define EQ_DSP_INV_LUT_SIZE (1+(1<<(EQ_DSP_INV_LUT_BITS-1)))
//#define EQ_DSP_SQRT_LUT_SIZE ((1<<(EQ_DSP_SQRT_LUT_BITS-2))*3 + 1)
//#define EQ_DSP_INV_SQRT_LUT_SIZE ((1<<(EQ_DSP_INV_SQRT_LUT_BITS-2))*3 + 1)

static const int32 m_V0LinearTable_v2[16]={
    268435456, /* 0 */
    301189535,
    337940217,
    379175160, /* 3 */
    425441527,
    477353244,
    535599149, /* 6 */
    600952130,
    674279380,
    756553907, /* 9 */
    848867446,
    952444939,
    1068660799, /* 12 */
    1199057137,
    1345364236,
    1509523501 /* 15 */
};

/* this LUT has been generated using the matlab function         */
/* "find_optimal_lut_mse_cantoni.m" in                           */
/*  \\vivekv\Public\Jaguar\cdma1x\matlab. See the matlab file    */
/* for details of the mathematical analysis.                     */

static const int32 m_iIinvTable_v2[EQ_DSP_INV_LUT_SIZE] = {
//static const int32 m_iIinvTable[129] = {
    2147461924,2130815118,2114424597,2098284255,2082388470,2066731709,2051308626,2036114026,
    2021142870,2006390265,1991851461,1977521842,1963396927,1949472361,1935743910,1922207461,
    1908859013,1895694677,1882710671,1869903314,1857269025,1844804320,1832505807,1820370185,
    1808394239,1796574837,1784908931,1773393550,1762025799,1750802857,1739721975,1728780472,
    1717975735,1707305216,1696766429,1686356950,1676074413,1665916511,1655880992,1645965655,
    1636168357,1626487001,1616919541,1607463979,1598118364,1588880788,1579749390,1570722350,
    1561797887,1552974264,1544249782,1535622778,1527091628,1518654744,1510310572,1502057591,
    1493894315,1485819289,1477831091,1469928326,1462109632,1454373675,1446719148,1439144771,
    1431649293,1424231488,1416890154,1409624114,1402432217,1395313333,1388266356,1381290202,
    1374383809,1367546135,1360776160,1354072883,1347435325,1340862522,1334353531,1327907429,
    1321523308,1315200279,1308937468,1302734019,1296589093,1290501865,1284471527,1278497284,
    1272578357,1266713982,1260903408,1255145899,1249440730,1243787190,1238184583,1232632223,
    1227129437,1221675565,1216269956,1210911973,1205600990,1200336391,1195117570,1189943933,
    1184814896,1179729885,1174688335,1169689691,1164733409,1159818951,1154945791,1150113410,
    1145321299,1140568955,1135855887,1131181610,1126545646,1121947526,1117386789,1112862980,
    1108375654,1103924370,1099508696,1095128207,1090782483,1086471112,1082193688,1077949816,
    1073739086
};

/*=============================================================================
Typedef
=============================================================================*/

typedef struct eq_band_internal_specs_t
{
                                  
    eq_filter_type_t            FiltType;         // Filter type                  
    uint32            FreqHz;           // Filter frequency param     
    int32             iFilterGain;      // Filter gain (dB)                                 
    int32             iQFactor;         // band pass filter quality factor 
    uint32            uiBandIdx;        // band index                   

} eq_band_internal_specs_t; 



typedef struct EQFilterDesign_t
{

    eq_band_internal_specs_t  *pEQFilterDesignData;
    int32      piFilterCoeff[EQ_MAX_BANDS*(MSIIR_NUM_COEFFS + MSIIR_DEN_COEFFS)];
    int16      piNumShiftFactor[EQ_MAX_BANDS];  // numerator shift factor
    uint32     uiSampleRate;
    uint16     uiNumBands;

} EQFilterDesign_t;



/*===========================================================================
*     Function Declarations 
* ==========================================================================*/

    

int16 ProcessEQFD_v2(EQFilterDesign_t * EQFilterDesign_ptr);
EQ_RESULT GetBandFreqRangeEQFD_v2 (eq_band_specs_t  *pEQFilterDesignData, uint32 uiSampleRate, uint16 uiNumBands, uint16 uiBandIdx, uint32 *piMinFreq, uint32 *piMaxFreq);
    




#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif  /* EQUALIZER_FILTER_DESIGN_H */
