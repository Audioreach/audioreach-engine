/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/*===========================================================================*]
[* FILE NAME: mathlib.c                                                      *]
[* DESCRIPTION:                                                              *]
[*    Contains math functions for post processor.                            *]
[*    Part of the code is adapted from QSound math functions.                *]
[*  FUNCTION LIST:                                                           *]
[*    See audpp_mathlib.h                                                          *]
[*===========================================================================*/
#include "audpp_mathlib.h"
#include "audio_divide_qx.h"
#include "ar_defs.h"
/*=============================================================================
      Q15 math
=============================================================================*/
static int16_t Q15_init(int32_t xL32Q15)
{
    if (xL32Q15 > Q15_ONE)
    {
        return Q15_ONE;
    }
    else if (xL32Q15 < Q15_MINUSONE)
    {
        return Q15_MINUSONE;
    }
    else
    {   
        return s16_extract_s32_l(xL32Q15);
    }
}

int16_t Q15_mult(int16_t aL16Q15, int16_t bL16Q15)
{   // one of the input doesn't have to be Q15
    if (aL16Q15==Q15_ONE)           return bL16Q15;
    if (bL16Q15==Q15_ONE)           return aL16Q15;
    if (aL16Q15==0 || bL16Q15==0)   return Q15_ZERO;
    return s16_extract_s64_h_rnd(s40_mult_s16_s16_shift(aL16Q15, bL16Q15, 1));
}

int16_t Q15_mult3(int16_t aL16Q15, int16_t bL16Q15, int16_t cL16Q15)
{
    return Q15_mult(Q15_mult(aL16Q15, bL16Q15), cL16Q15);
}

void Q15_multBy(int16_t *aL16Q15, int16_t bL16Q15 )
{
    *aL16Q15 = Q15_mult(*aL16Q15, bL16Q15);
}

int16_t Q15_scaleByInt16(int16_t scaleL16Q15, int16_t xL16)
{
    if (scaleL16Q15 == Q15_ZERO || xL16==0) return 0;
    if (scaleL16Q15 == Q15_ONE)             return xL16;
    return s16_extract_s40_h(s40_mult_s16_s16_shift(xL16, scaleL16Q15, 1));
}

int16_t Q15_negate(int16_t scaleL16Q15)
{
    return scaleL16Q15 == -32768 ? 0x7fff : (int16_t)-scaleL16Q15;
}

int16_t Q15_initRatio(int32_t numer, int32_t denom)
{
    return Q15_initQ16(divide_qx(numer, denom, 16));
}

int16_t Q15_initQ16(int32_t initL32Q16)
{
    return Q15_init(initL32Q16 >> 1);
}

int16_t Q15_initQ23(int32_t initL32Q23)
{
    return Q15_init((initL32Q23 + 128) >> 8);
}

/* tables of scale values (L16Q15) for mB conversion */
static const int16_t T1[] = 
{
    32767, 24403, 18174,
};
static const int16_t T2[] = 
{
    32767, 32170, 31583, 31006,
    30440, 29885, 29339, 28804,
    28278, 27762, 27255, 26758,
    26269, 25790, 25319, 24857,
};
static const int16_t T3[] = 
{
    32767, 32730, 32693, 32655,
    32617, 32580, 32542, 32505,
    32468, 32430, 32393, 32356,
    32318, 32281, 32244, 32207,
};

int16_t Q15_initMB(int16_t mB)
{
    int32_t product;
    if (mB >= 0    ) return Q15_ONE;
    if (mB <= -8000) return 0;
    mB = -mB;

    product = 32768;
    while (mB >= 2404) 
    {
        product >>= 4;
        mB -= 2404;
    }
    while (mB >= 602) 
    {
        product >>= 1;
        mB -= 602;
    }
    if (mB >= 256) 
    {
        product = (product * T1[mB>>8] + (1<<14)) >> 15;
        mB &= 255;
    }
    if (mB >= 16) 
    {
        product = (product * T2[mB>>4] + (1<<14)) >> 15;
        mB &= 15;
    }
    if (mB != 0) 
    {
        product = (product * T3[mB] + (1<<14)) >> 15;
    }
    return (int16_t)product;
}

/* binary search for next set of bits in mB calculation */
static int16_t find_mb(int16_t *mB, const int16_t *row, int16_t n, 
                     int16_t scaleL16, int16_t target)
{
    const int16_t *base = row;
    const int16_t *end = row+n;
    int16_t update = scaleL16;
    int16_t test;
    while (n > 1) 
    {
        const int16_t* mid = base+n/2;
        test = (int16_t)(((int32_t)scaleL16 * *mid + (1 << 14)) >> 15);
        if (test < target) 
        {
            end = mid;
        }
        else 
        {
            base = mid;
            update = test;
        }
        n = (int16_t)(end - base);
    }
    *mB = (int16_t)(*mB << 4 | (base - row));
    return update;
}

int16_t Q15_getMB(int16_t scaleL16)
{
    int16_t mB = 0, mB2, index, x;

    if (scaleL16 == Q15_ZERO) return -9600;
    if (scaleL16 == Q15_ONE ) return 0;
    while (scaleL16 < 2048) 
    {
        scaleL16 <<= 4;
        mB += 2404;
    }
    while (scaleL16 < 16384) 
    {
        scaleL16 <<= 1;
        mB += 602;
    }
    if (scaleL16 == 16384) 
    {
        return -mB-602;
    }
    
    for (index = 0; index < 2 && T1[index+1] > scaleL16; index++) 
    {
    }
    mB2 = index;
    x = T1[index];
    x = find_mb(&mB2, T2, 16, x, scaleL16);
    x = find_mb(&mB2, T3, 16, x, scaleL16);
    return -(mB + mB2);
}


/*=============================================================================
      Q16 math
=============================================================================*/
// Notes from origional code:
// Use basic C operations to do 40 bit multiplication; is made much
// slower and complicated because of the absence of a carry bit.
// Optimize this in assembler or using intrinsics if you can.
#define lower(x) ((x)&0xffff)
#define upper(x) ((x)>>16)

int32_t Q16_mult(int32_t aL16Q16, int32_t bL16Q16)
{
#if 1 //bit-exact version to QSound function Fix16_Multiply()
    return (int32_t)(((int64)aL16Q16*bL16Q16+(1L<<15))>>16);

#else
    uint32_t a1, a2, b1, b2, a1b1, a1b2, a2b1, a2b2;
    int32_t product;
    boolean negative;

    if (aL16Q16 == 0 || bL16Q16 == 0) return 0;

    negative = FALSE;
    if (aL16Q16 < 0) aL16Q16 = -aL16Q16, negative = TRUE;
    if (bL16Q16 < 0) bL16Q16 = -bL16Q16, negative = !negative;

    if (aL16Q16 == Q16_ONE) 
    {
        product = bL16Q16;
    }
    else if (bL16Q16 == Q16_ONE) 
    {
        product = aL16Q16;
    }
    else if (aL16Q16 < Q16_ONE && bL16Q16 < Q16_ONE) 
    {
        product = ((uint32_t)aL16Q16 * (uint32_t)bL16Q16 + Q16_ONE/2) >> 16;
    }
    else 
    {
        a1 = upper(aL16Q16);
        a2 = lower(aL16Q16);
        b1 = upper(bL16Q16);
        b2 = lower(bL16Q16);
        a1b1 = a1 * b1;
        a1b2 = a1 * b2;
        a2b1 = a2 * b1;
        a2b2 = a2 * b2;
        product = (a1b1<<16) + a1b2 + a2b1 + ((a2b2+Q16_ONE/2) >> 16);
    }
    return negative ? -product : product;
#endif
}

int32_t Q16_square(int32_t xL32Q16)
{
    uint32_t x1, x2, x1x1, x1x2, x2x2;
    
    if (xL32Q16 == 0      ) return 0;
    if (xL32Q16 <  0      ) xL32Q16 = -xL32Q16;
    if (xL32Q16 == Q16_ONE) return xL32Q16;
    if (xL32Q16 <  Q16_ONE) 
        return ((uint32_t)xL32Q16 * (uint32_t)xL32Q16 + Q16_ONE/2) >> 16;

    x1 = upper(xL32Q16);
    x2 = lower(xL32Q16);
    x1x1 = x1 * x1;
    x1x2 = x1 * x2;
    x2x2 = x2 * x2;
    return (x1x1 << 16) + 2 * x1x2 + ((x2x2 + Q16_ONE/2) >> 16);
}

int32_t Q16_sqrt(int32_t aL32Q16)
{
    uint32_t root, remHi, remLo, testDiv;
    int16_t count;
    
    root = 0;               // Clear root
    remHi = 0;              // Clear high part of partial remainder
    remLo = aL32Q16;        // Get argument into low part of partial remainder
    count = 23;

    do 
    {
        remHi = (remHi << 2) | (remLo >> 30); remLo <<= 2; // get 2 bits of arg
        root <<= 1;                   // Get ready for the next bit in the root
        testDiv = (root << 1) + 1;    // Test divisor
        if (remHi >= testDiv) 
        {
            remHi -= testDiv;
            root += 1;
        }
    } while (count-- != 0);

    return root;
}

int32_t Q16_reciprocalU(int32_t denom)
{
    uint32_t x, y, t;
    int i;
    if(!(denom>0))
    {
        return 0;
    }

    x = 1;
    y = 0;
    for (i = 1; i<=31; i++) 
    {
        x <<= 1;                        // shift x||y left by one bit
        y <<= 1;                        // ...
        if (x >= (uint32_t)denom) 
        {
            x -= denom;
            y += 1;
        }
    }
    t = (int32_t)x>>31;                   // all 1's if x[31]==1
    x <<= 1;                            // shift x||y left by one bit
    y <<= 1;                            // ...
    if ((x|t) >= (uint32_t)denom) 
    {
        x -= denom;
        y += 1;
    }
    // x is remainder, y is quotient
    return x >= (uint32_t)denom >> 1 ? y+1 : y;
}

int32_t Q16_reciprocal(int32_t denom)
{
    if (denom < 0) 
    {
        return -Q16_reciprocalU(-denom);
    }
    return Q16_reciprocalU(denom);
}

/*---------------------------------------------------------------------------*/
/* Q16_Divide_Truncated()                                                    */
/* - Version of Q16_Divide without rounding. Used by variable delay          */
/*---------------------------------------------------------------------------*/
int32_t Q16_divide_truncated(int32_t numerL32Q16, int32_t denomL32Q16)
{
    boolean negative;
    uint32_t xUL32, yUL32, tUL32;
    int16_t i;
    
    if (numerL32Q16 == 0) return 0;

    negative = FALSE;
    if (numerL32Q16 < 0) 
    {
        negative = TRUE;
        numerL32Q16 = -numerL32Q16;
    }
    if (denomL32Q16 < 0) 
    {
        negative = !negative;
        denomL32Q16 = -denomL32Q16;
    }

    xUL32 = numerL32Q16 >> 16;
    yUL32 = numerL32Q16 << 16;
    for (i = 1; i <= 32; i++) 
    {
        tUL32 = (int32_t)xUL32 >> 31;             // all 1's if x[31]==1
        xUL32 = (xUL32 << 1) | (yUL32 >> 31);   // shift x||y left by one bit
        yUL32 <<= 1;                            // upshift 1, remove one bit
        if ((xUL32|tUL32) >= (uint32_t)denomL32Q16) 
        {
            xUL32 -= denomL32Q16;
            yUL32 += 1;
        }
    } /* end of for (i = 1; i <= 32; i++) */

    /* x is remainder, y is quotient */
    return negative ? -(int32_t)yUL32 : (int32_t)yUL32;
} /*-------------- end of function Q16_divide_truncated ---------------------*/

/*=============================================================================
      Q23 math
=============================================================================*/
// Notes from origional code:
// Use basic C operations to do 40 bit multiplication; is made much
// slower and complicated because of the absence of a carry bit.
// Optimize this in assembler or using intrinsics if you can.
int32_t Q23_mult(int32_t aL32Q23, int32_t bL32Q23)
{
    boolean negative;
    int32_t product;
    uint32_t a1, a2, b1, b2, a1b2, a2b1, a2b2, w, u, v;

    if (aL32Q23 == 0 || bL32Q23 == 0) return 0;

    negative = FALSE;
    if (aL32Q23 < 0) aL32Q23 = -aL32Q23, negative = TRUE;
    if (bL32Q23 < 0) bL32Q23 = -bL32Q23, negative = !negative;
    
    if (aL32Q23 == Q23_ONE) 
    {
        product = bL32Q23;
    }
    else if (bL32Q23 == Q23_ONE) 
    {
        product = aL32Q23;
    }
    else 
    {
        a1 = upper(aL32Q23);
        a2 = lower(aL32Q23);
        b1 = upper(bL32Q23);
        b2 = lower(bL32Q23);
        a1b2 = a1 * b2;
        a2b1 = a2 * b1;
        a2b2 = a2 * b2;
        w = lower(a1b2)+lower(a2b1)+upper(a2b2)+(1<<(22-16));
                                                // s16_add_s16_s16 in rounding
        u = a1*b1+upper(a1b2)+upper(a2b1)+upper(w);      
                                                // u,v is 40 bit product
        v = (lower(w)<<16)+lower(a2b2);
        product = (u<<9)|(v>>23);
    }
    return negative ? -product : product;
}


int32_t Q23_square(int32_t xL32Q23)
{
    uint32_t x1, x2, x1x2, x2x2, w, u, v;
    
    if (xL32Q23 == 0      ) return 0;
    if (xL32Q23 <  0      ) xL32Q23 = -xL32Q23;
    if (xL32Q23 == Q23_ONE) return xL32Q23;
    x1 = upper(xL32Q23);
    x2 = lower(xL32Q23);
    x1x2 = x1 * x2;
    x2x2 = x2 * x2;
    w = 2*lower(x1x2) + upper(x2x2) + (1<<(22-16)); // s16_add_s16_s16 in rounding
    u = x1*x1 + 2*upper(x1x2) + upper(w);
    v = (lower(w)<<16) + lower(x2x2);
    return (u<<9)|(v>>23);
}

int32_t Q23_sqrt(int32_t aL32Q23)
{
    uint32_t root, remHi, remLo, testDiv;

    int count;

    if (aL32Q23 == 0 || aL32Q23 == Q23_ONE) return aL32Q23;
    root = 0;           // Clear root
    remHi = 0;          // Clear high part of partial remainder
    remLo = aL32Q23;    // Get argument into low part of partial remainder
    count = 26;

    remHi = (remHi << 1) | (remLo >> 31); remLo <<= 1;  // get 2 bits of arg
    root <<= 1;                     // Get ready for the next bit in the root 
    testDiv = (root << 1) + 1;      // Test divisor
    if (remHi >= testDiv) 
    {
        remHi -= testDiv;
        root += 1;
    }
    do 
    {
        remHi = (remHi << 2) | (remLo >> 30); remLo <<= 2; // get 2 bits of arg
        root <<= 1;                 // Get ready for the next bit in the root
        testDiv = (root << 1) + 1;  // Test divisor
        if (remHi >= testDiv) 
        {
            remHi -= testDiv;
            root += 1;
        }
    } while (count-- != 0);

    return root;
}

int32_t Q23_reciprocalU(int32_t denom)
{
    uint32_t x, y, t, i;
    if(!(denom>0))
    {
        return 0;
    }

    x = 1<<14;
    y = 0;
    for (i = 1; i <= 32; i++) 
    {
        t = (int32_t)x>>31;               // all 1's if x[31]==1
        x = (x<<1) | (y>>31);           // shift x||y left by one bit
        y <<= 1;                        // ...
        if ((x|t)>=(uint32_t)denom) 
        {
            x -= denom;
            y += 1;
        }
    }
    // x is remainder, y is quotient
    return x>=(uint32_t)denom >> 1 ? y+1 : y;
}

int32_t Q23_reciprocal(int32_t denom)
{
    if (denom<0) 
    {
        return -Q23_reciprocalU(-denom);
    }
    return Q23_reciprocalU(denom);
}


int32_t Q23_sine0(int32_t xL32Q23)
{
    int32_t x2 = Q23_square(xL32Q23);
    int32_t dz = xL32Q23;
    int32_t sine = xL32Q23;
    int32_t n = 2;
    while (1) 
    {
        dz = Q23_mult(dz, divide_int32(-x2, n*(n+1)));
        if (dz==0) 
        {
            break;
        }
        sine += dz;
        n += 2;
    }
    if(!(sine >= -Q23_ONE && sine <= Q23_ONE))
    {
        return 0;
    }
    return sine;
}

int32_t Q23_cosine0(int32_t xL32Q23)
{
    int32_t x2 = Q23_square(xL32Q23);
    int32_t dz = -x2/2;
    int32_t cosine = Q23_ONE + dz;
    int32_t n = 3;
    while (1) 
    {
        dz = Q23_mult(dz, divide_int32(-x2, (n*(n+1))));
        if (dz==0) break;
        cosine += dz;
        n += 2;
    }
    if(!(cosine >= -Q23_ONE && cosine <= Q23_ONE))
    {
        return 0;
    }
    return cosine;
}

int32_t Q23_sine(int32_t xL32Q23)
{
    if (xL32Q23 <= Q23_PI_2 ) return  Q23_sine0(xL32Q23);
    if (xL32Q23 <= Q23_PI   ) return  Q23_sine0(Q23_PI - xL32Q23);
    if (xL32Q23 <= Q23_PI3_2) return -Q23_sine0(xL32Q23 - Q23_PI);
    if (xL32Q23 <= Q23_TWOPI) return -Q23_sine0(Q23_TWOPI - xL32Q23);
    if (xL32Q23 <  0        ) return -Q23_sine(-xL32Q23);
    while (xL32Q23 >= Q23_TWOPI) xL32Q23 -= Q23_TWOPI;
    return Q23_sine(xL32Q23);
}


int32_t Q23_cosine(int32_t xL32Q23)
{
    if (xL32Q23 <= Q23_PI_2 ) return  Q23_cosine0(xL32Q23);
    if (xL32Q23 <= Q23_PI   ) return -Q23_cosine0(Q23_PI - xL32Q23);
    if (xL32Q23 <= Q23_PI3_2) return -Q23_cosine0(xL32Q23 - Q23_PI);
    if (xL32Q23 <= Q23_TWOPI) return  Q23_cosine0(Q23_TWOPI - xL32Q23);
    if (xL32Q23 <  0        ) return  Q23_cosine(-xL32Q23);
    while (xL32Q23 >= Q23_TWOPI) xL32Q23 -= Q23_TWOPI;
    return Q23_cosine(xL32Q23);
}


int32_t Q23_exp0(int32_t xL32Q23)
{
    int32_t exp = Q23_ONE + xL32Q23;
    int32_t dz = xL32Q23;
    int32_t n = 2;
    while ((dz = Q23_mult(dz, divide_int32(xL32Q23, n)))!=0) 
    {
        exp += dz;
        n += 1;
    }
    return exp;
}

#define Q23_EXP_STEP        (8496933)
#define Q23_EXP_SCALE       (23098968)
#define Q23_INVEXP_STEP     (-11901566)
#define Q23_INVEXP_SCALE    (2030125)
#define Q23_EXP_ONE         (22802601)
#define Q23_EXP_MINUSONE    (3085996)

int32_t Q23_exp(int32_t xL32Q23)
{
    int32_t expL32Q23;
    if (xL32Q23 >= Q23_EXP_STEP) 
    {
        expL32Q23 = Q23_EXP_SCALE;
        while (1) 
        {
            xL32Q23 -= Q23_EXP_STEP;
            if (xL32Q23 < Q23_EXP_STEP) 
            {
                break;
            }
            expL32Q23 = Q23_mult(expL32Q23, Q23_EXP_SCALE);
        }
    }
    else if (xL32Q23 <= Q23_INVEXP_STEP) 
    {
        expL32Q23 = Q23_INVEXP_SCALE;
        while (1) 
        {
            xL32Q23 -= Q23_INVEXP_STEP;
            if (xL32Q23 > Q23_INVEXP_STEP)
            {
                break;
            }
            expL32Q23 = Q23_mult(expL32Q23, Q23_INVEXP_SCALE);
        }
    }
    else 
    {
        return Q23_exp0(xL32Q23);
    }
    return Q23_mult(expL32Q23, Q23_exp0(xL32Q23));
}


#define Q23_ln2 (5814540)

int32_t Q23_ln(int32_t xL32Q23)
{
    int32_t power2, n;
    int32_t lnL32Q23, dzL32Q23, deltaL32Q23;

    if (xL32Q23 == Q23_ONE) 
    {
        return 0;
    }

    // Normalize argument to range 0.5 to 1 because the Taylor series 
    // expansion converges more quickly as x approaches 1.  Shifts are 
    // fast and precise.
    power2 = 0;
    while (xL32Q23 > Q23_ONE) 
    {
        xL32Q23 >>= 1;
        power2++;
    }
    while (xL32Q23 < Q23_ONE>>1) 
    {
        xL32Q23 <<= 1;
        power2--;
    }

    // ln(1-x) = -x - x**2/2 - x**3/3 - x**4/4 - x**5/5 ...
    xL32Q23 = Q23_ONE - xL32Q23;

    lnL32Q23 = -xL32Q23;
    dzL32Q23 = xL32Q23;
    n = 2;
    while (1) 
    {
        dzL32Q23 = Q23_mult(dzL32Q23, xL32Q23);
        deltaL32Q23 = divide_int32(dzL32Q23, n);
        if (deltaL32Q23 == 0) 
        {
            break;
        }
        lnL32Q23 -= deltaL32Q23;
        n += 1;
    }

    lnL32Q23 += power2 * Q23_ln2;

    return lnL32Q23;
}

int32_t Q23_pow(int32_t xL32Q23, int32_t yL32Q23)
{
    if (xL32Q23 == 0) return 0;
    if (yL32Q23 == 0) return Q23_ONE;
    return Q23_exp(Q23_mult(yL32Q23, Q23_ln(xL32Q23)));
}


#define Q23_ln10_2000 (9658)

int32_t Q23_initMB(int16_t mB)
{
    if (mB == 0) return Q23_ONE;
	if (mB>=4816) return Q23_MAX;
	if (mB<=-12000) return 0;

    return Q23_exp(mB * Q23_ln10_2000);
}

int32_t Q23_getMB(int32_t scaleL32Q23)
{
    if (scaleL32Q23 == Q23_ONE) return 0;
    if (scaleL32Q23 == 0      ) return (-2147483647 - 1);  // INT_MIN
    return (int32_t)Q23_mult(869, Q23_ln(scaleL32Q23));
}


