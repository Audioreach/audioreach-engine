/*=======================================================================
  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
  SPDX-License-Identifier: BSD-3-Clause-Clear
=========================================================================*/

#include "audio_divide_qx.h"
#include "audio_basic_op.h"
#include "audio_basic_op_ext.h"

/*==========================================================================*/
/* FUNCTION: divide_int32                                                   */
/*                                                                          */
/* DESCRIPTION: numerator/denominator (both in the same Q) and outputs      */
/*              quotient in outputQL16.                                     */
/*                                                                          */
/* INPUTS: numeratorL32: numerator in Qin.                                  */
/*         denominatorL32: denominator in Qin.                              */
/* OUTPUTS: quotient: quotient of operation in Q31.                         */
/*                                                                          */
/* REQUIREMENTS:  numerator!= denominator. And both numerator and           */
/*                      denominator are positive.                           */
/* IMPLEMENTATION NOTES:                                                    */
/*                                                                          */
/* Since its easier to explain using Q0 output, lets assume that output Q   */
/* factor is 0.                                                             */
/*        if(num >= 2^31*den)                                               */
/*           num = num - den;                                               */
/*           quotient = quotient<<1+1;                                      */
/*        else                                                              */
/*           quotient = quotient<<1;                                        */
/*      num = num<<1;                                                       */
/*                                                                          */
/*==========================================================================*/
int32_t divide_int32(int32_t numeratorL32, int32_t denominatorL32)
{
    int32_t quotientL32=0;
    int16_t nShift, dShift;
    int16_t i, count;
    int16_t negFlag=0;
    int40 shiftNumerL40, tempL40;

    if(numeratorL32 < 0)
    {
        negFlag = 1;
        numeratorL32 = -numeratorL32;
    }
    if(denominatorL32 < 0)
    {
        negFlag = !negFlag;
        denominatorL32 = -denominatorL32;
}

    if( (numeratorL32 < denominatorL32))
    {
        return 0;
    } /* end of sanity checks */
    /* normalize numerator and denominator */
    nShift=s16_norm_s32(numeratorL32);
    shiftNumerL40 = s32_shl_s32_sat(numeratorL32, nShift);
    dShift = s16_norm_s32(denominatorL32);
    denominatorL32 = s32_shl_s32_sat(denominatorL32, dShift);

    count = s16_add_s16_s16(dShift,1);
    count = s16_sub_s16_s16(count, nShift);
    for(i = 0; i<count; i++)
    {
        tempL40 = s40_sub_s40_s40(shiftNumerL40, denominatorL32);
        shiftNumerL40 = s40_shl_s40(shiftNumerL40,1);
        quotientL32 = s32_shl_s32_sat(quotientL32,1 );
        if (tempL40 >=0)
        {
            shiftNumerL40 = s40_shl_s40(tempL40,1);
            quotientL32 = s32_add_s32_s32(quotientL32,1);
        }
    }/* end of repetitions*/
    if(negFlag)
        quotientL32 = -quotientL32;
    return quotientL32;
} /*-------------------- end of function divide_int32 -----------------------*/

/***************************************************************************
*
*   FUNCTION NAME: s16_div_s16_s16_sat
*
*   PURPOSE:
*
*     Divide var1 by var2 using the shift and subtract method.  
*     Produces a result which is the fractional 
*     integer division of var1 by var2; var1 and var2 must be positive and 
*     var2 must be greater or equal to var1; the result is positive 
*     (leading bit equal to 0) and truncated to 16 bits.
*     If var1 = var2 then div(var1,var2) = 32767.
*
*   INPUTS:
*
*     var1
*                     16 bit short signed integer (int16_t) whose value
*                     falls in the range 0xffff 8000 <= var1 <= 0x0000 7fff.
*     var2
*                     16 bit short signed integer (int16_t) whose value
*                     falls in the range 0xffff 8000 <= var2 <= 0x0000 7fff.
*                     var2 != 0.
*
*   OUTPUTS:
*
*     none
*
*   RETURN VALUE:
*
*     var_out
*                     16 bit short signed integer (int16_t) whose value
*                     falls in the range
*                     0xffff 8000 <= swOut <= 0x0000 7fff.
*
*   IMPLEMENTATION:
*
*     In the case where var1==var2 the function returns 0x7fff.  The output
*     is undefined for invalid inputs.  This implementation returns zero.
*
*   KEYWORDS: div
*
*************************************************************************/

int16_t s16_div_s16_s16_sat(int16_t var1, int16_t var2)
{
    int16_t var_out = 0;
    int16_t iteration;
    int32_t L_num, L_denom;

#ifdef WMOPS_FX
    counter_fx.s16_div_s16_s16++;
#endif

    if ((var1 > var2) || (var1 < 0) || (var2 < 0))
    {
      var_out = 0x7fff;
//      printf ("Division Error var1=%d  var2=%d\n", var1, var2);
//      exit (0);
    }
    if (var2 == 0)
    {
        var_out = 0x7fff;
//        printf ("Division by 0, Fatal error \n");
//        exit (0);              
    }
    if (var1 == 0)
    {
        var_out = 0;
    }
    else
    {
        if (var1 == var2)
        {
            var_out = MAX_16;
        }
        else
        {
            L_num = (int32_t )var1;
            L_denom = (int32_t )var2;

            for (iteration = 0; iteration < 15; iteration++)
            {
                var_out <<= 1;
                L_num <<= 1;

                if (L_num >= L_denom)
                {
                    L_num = s32_sub_s32_s32_sat (L_num, L_denom);
                    var_out = s16_add_s16_s16_sat (var_out, 1);
                }
            }
        }
    }

    return (var_out);

}

/***************************************************************************
*
*   FUNCTION NAME: s32_div_normalized s32_s32
*
*   PURPOSE:
*
*     Divide var1 by var2 and provide normalized output.
*     The two inputs are normalized so as to represent them with full
*     32-bit precision in the range [0,1). The normalized inputs are
*     truncated to upper 16-bits and divided and the normalized
*     quotient is returned with 16-bit precision where the 16-bits
*     are in LSB. The normalized quotient is in Q16.15 format.
*     The required shift factor to adjust the quotient value
*     is returned in the pointer qShiftL16Ptr.
*     To get the output in a desired 32-bit Qformat Qm.(31-m),
*     left shift the output by  qShiftL16Ptr+(16-m).
*     To get the output in a desired 16-bit Qformat Qm.(15-m)with 
*     16-bit result in the LSB of the 32-bit output, left shift the
*     output by  qShiftL16Ptr-m.
*
*   INPUTS:
*
*     var1
*               32 bit signed integer (int16_t) whose value
*               falls in the range 0x8000 0000 <= var1 <= 0x7fff ffff.
*     var2
*               32 bit signed integer (int16_t) whose value
*               falls in the range 0x8000 0000 <= var1 <= 0x7fff ffff.
*               var2 != 0.
*
*   OUTPUTS:
*
*     none
*
*   RETURN VALUE:
*
*     outL32
*                     32 bit signed integer (int32_t) whose value
*                     falls in the range
*                     0x8000 0000 <= var1 <= 0x7fff ffff.
*     qShiftL16Ptr:   pointer to modified shift factor for  
*                     denormalizing the quotient   
*
*   IMPLEMENTATION:
*
*     In the case where var1=0, output is returned as 0.
*     In the case where var2=0, output is maxed out.
*
*   KEYWORDS: div
*
*************************************************************************/

int32_t s32_div_s32_s32_normalized(int32_t var1, int32_t var2, int16_t *qShiftL16Ptr)
{
     int32_t outL32=0;       
     int16_t var1_exp, var2_exp, tmp1, tmp2, sign = 1;

#ifdef WMOPS_FX
    counter_fx.s32_div_s32_s32_normalized++;
#endif

     if(var1 == 0)
     {
        qShiftL16Ptr[0] = 0;
        return outL32;
     } 
     if(var1 < 0)
     {
        var1 = s32_neg_s32_sat(var1);
        sign = -1;
     }
     if(var2 < 0)
     {
        var2 = s32_neg_s32_sat(var2);
        sign = -sign;
     }

     if(var2 != 0)
     {
        // Compute the exponents of the numerator and denominator
        var1_exp = s16_norm_s32( var1 );
        var2_exp = s16_norm_s32( var2 );
            
        // Normalize the inputs such that numerator < denominator and are in full 32-bit precision
        tmp1 = s16_extract_s32_h( s32_shl_s32_sat( var1, var1_exp-1 ));
        tmp2 = s16_extract_s32_h( s32_shl_s32_sat( var2, var2_exp ));

        // Perform division with 16-bit precision
        outL32 = (int32_t )s16_div_s16_s16_sat( tmp1, tmp2);    
        
        // Shift factor value for de-normalizing the quotient
        qShiftL16Ptr[0] = var2_exp - var1_exp +1;
     }
     else
     {
        outL32 = 0xFFFF;
        qShiftL16Ptr[0] = 15;
     }

     // Negate the quotient if either numerator or denominator is negative
     if(sign < 0)
     {
        outL32 = -outL32;
     }      

     return outL32;

}    

/*==========================================================================*/
/* FUNCTION: divide_qx                                                      */
/*                                                                          */
/* DESCRIPTION: numerator/denominator (both in the same Q) and outputs      */
/*              quotient in Q31.                                            */
/*                                                                          */
/* INPUTS: numeratorL32: numerator in Qin.                                  */
/*         denominatorL32: denominator in Qin.                              */
/*         outputQL16: Q factor in which output is required.                */
/* OUTPUTS: quotient: quotient of operation in Q(outputQL16).               */
/*                                                                          */
/* IMPLEMENTATION NOTES:                                                    */
/*     This is a wrapper function to divide two 32-bit numbers in the same  */
/*  Q factor to produce output in required Q factor.                        */
/*==========================================================================*/
int32_t divide_qx(int32_t numeratorL32, int32_t denominatorL32, int16_t outputQL16) 
{
    int32_t quotientL32Qx;
    int16_t negativeFlag = 0;
    int32_t oneL32Qx;

    /* --------- re-direct divide function for bit-exact matching ----------*/
    /* commented out for actual implementation: we just use divide_qx       */
    // if (outputQL16 ==23) return Q23_divide(numeratorL32, denominatorL32);
    // else if (outputQL16 ==16) return Q16_divide(numeratorL32, denominatorL32);
    /* ------------------end of bit-exact matching code --------------------*/
 
    if (numeratorL32==0) return 0;

    if (numeratorL32<0)
    {
        numeratorL32 = -numeratorL32;
        negativeFlag = 1;
    }
    if (denominatorL32<0) 
    {
        denominatorL32 = -denominatorL32;
        negativeFlag = !negativeFlag;
    }

    if( outputQL16 == 31)
    { // if outout Q factor is 31
        oneL32Qx = (int32_t)0x7fffffffL;
    }
    else 
    { // else
        oneL32Qx = 1<<outputQL16;
    }

    quotientL32Qx = numeratorL32==denominatorL32 ? oneL32Qx : 
    divide_int32_qx(numeratorL32, denominatorL32, outputQL16);
    return negativeFlag ? -quotientL32Qx : quotientL32Qx; 
}
/*---------------- End of divide_qx function    ----------------------------*/