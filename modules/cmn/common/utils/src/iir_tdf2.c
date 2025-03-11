/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* This function calculates the response of a two-pole IIR filter*/
/* that is implemented in TDF2 form                              */
/* Input and output are in same Q-format (Q23)                   */
/* dencoefs->a(1),numcoefs->b(0), only 5 coefs are stored        */
#include "audio_iir_tdf2.h"
#include "audio_basic_op.h"
#include "audio_basic_op_ext.h"
#if ((defined __hexagon__) || (defined __qdsp6__))
#include "audio_iir_tdf2_asm.h"
#include <hexagon_protos.h>
#include <hexagon_types.h>
#endif

#define SHIFTUP 3
#define SHIFTDN SHIFTUP
#define PRECISION 0
#ifndef __qdsp6__
int iirTDF2(int16_t *inp,
            int16_t *out,
            uint16_t samples,
            int32_t *numcoefs,
            int32_t *dencoefs,
            int32_t *mem,
            int16_t shiftn,
            int16_t shiftd)
{
#if ((defined __hexagon__) || (defined __qdsp6__))
   long long ytemp,w1temp,w2temp,yscaled;
   long long w1t,w2t;

   int b0, b1, b2, a1, a2, w1, w2, input, temp;
   uint16_t i;

   b0 = *numcoefs++;
   b1 = *numcoefs++;
   b2 = *numcoefs++;

   a1 = *dencoefs++;
   a2 = *dencoefs++;

   w1 = mem[0];
   w2 = mem[1];

   /* repeat the loop for every sample*/
   /* equations are y = b0*x+w1    */
   /*                w1= b1*x-a1*y+w2  */
   /*                w2= b2*x-a2*y     */

   for(i=0; i< samples; i++)
   {
      input = *inp++;

      // ytemp = b0*x
      ytemp = Q6_P_mpy_RR(b0, input);
      // scaled version of y: b0*x + w1
      yscaled = Q6_P_aslacc_PR(w1, ytemp,(shiftn-SHIFTDN-16));

      // round the result
      ytemp = (yscaled + 0x1000)>>13;

      // calculate w1
      // b1*x
      w1temp = Q6_P_mpy_RR(b1, input);

      // b1*x + w2
      w1temp = Q6_P_aslacc_PR(w2, w1temp, (shiftn-SHIFTDN-16));

      // a1*yscaled
      w1t = Q6_P_mpy_RR((int)yscaled, a1);

      // b1*x +w2 - a1*y
      w1temp = Q6_P_aslnac_PR(w1temp, w1t,(shiftd-32));  /* q compensation      */

      w1 = Q6_R_sat_P(w1temp);

      // calculate w2
      // b2*x
      w2temp = Q6_P_mpy_RR(b2, input);

      w2temp = Q6_P_asl_PR(w2temp, (shiftn-SHIFTDN-16));

      // a2*yscaled
      w2t = Q6_P_mpy_RR((int)yscaled, a2);

      // b2*x - a2*y
      w2temp = Q6_P_aslnac_PR(w2temp, w2t,(shiftd-32));  /* q compensation      */

      w2 = Q6_R_sat_P(w2temp);

      // store the output
      *out++= Q6_R_sath_R((int)ytemp);
   }

   mem[0] = w1;
   mem[1] = w2;

   return(0);

#else  //#if ((defined __hexagon__) || (defined __qdsp6__))
    int40 ytemp,w1temp,w2temp,yscaled;
    int40 w1t,w2t;

    int32_t *pncs,*pdcs;
    int32_t temp;
    uint32_t flag=0;
    uint16_t i;

    /* repeat the loop for every sample*/
    /* equations are y = b0*x+w1       */
    /*               w1= b1*x-a1*y+w2  */
    /*               w2= b2*x-a2*y     */

    for(i=0;i<samples;i++)
    {

        pncs=numcoefs;      /*initialize to numcoefs*/
        pdcs=dencoefs;      /*initialize to dencoefs*/

        ytemp = s40_mult_s32_s16(*pncs++, *inp);
                            /* ytemp = b0*x in 48 bits*/
        ytemp = s40_shl_s40(ytemp, -16);
                            /* ytemp = b0*x in 32 bits*/

#if PRECISION
        ytemp = s40_shl_s40(ytemp, shiftn);
                            /* ytemp in required q format*/
        yscaled = s40_shl_s40(ytemp, -SHIFTDN);
#else
        yscaled = s40_shl_s40(ytemp,(shiftn-SHIFTDN));
#endif
        temp=*mem++;        /* temp = scaled down w1 */
                            /* mem->w2               */

        yscaled = s40_add_s40_s32(yscaled, temp);
                            /* scaled version of y */

        ytemp = s40_shl_s40(yscaled, SHIFTUP);
                            /* unscaled version of y*/
        ytemp = s40_add_s40_s32(ytemp, 0x8000); /* round the result */

        ytemp = s40_shl_s40(ytemp, -16);

        /*calculate w1*/
        w1temp = s40_mult_s32_s16(*pncs++, *inp); /*b1*x in 48 bits */
        w1temp = s40_shl_s40(w1temp, -16);        /*b1*x in 32 bits */
#if PRECISION
        w1temp = s40_shl_s40(w1temp,shiftn);/*b1*x in q format*/
        w1temp = s40_shl_s40(w1temp,-(SHIFTDN)); /*scaled version of b1*x */
#else
        w1temp = s40_shl_s40(w1temp, (shiftn-SHIFTDN));
#endif
        /* a1*yscaled           */
        w1t = s40_mult_s32_s32_shift((int32_t)yscaled, *pdcs++,0);
        w1t = s40_shl_s40(w1t,shiftd);  /* q compensation       */
        w1t = s40_add_s40_s32(w1t, -(*mem--)); /* w1t=a1*y-w2          */
        w1temp = s40_sub_s40_s40(w1temp, w1t);  /* w1temp=b1*x-a1*y+w2  */
                                            /*w1temp is scaled by 3*/
        w1temp = s32_saturate_s40(w1temp);

        *mem++ = (int32_t)w1temp;

        /*calculate w2*/
        w2temp = s40_mult_s32_s16(*pncs++, *inp++); /*b2*x in 48 bits */
        w2temp = s40_shl_s40(w2temp, -16); /*b2*x in 32 bits */
#if PRECISION
        w2temp = s40_shl_s40(w2temp, shiftn); /*b2*x q compensation*/
        w2temp = s40_shl_s40(w2temp, -SHIFTDN); /*scale b2*x        */
#else
        w2temp = s40_shl_s40(w2temp, (shiftn-SHIFTDN));
#endif
        /* a2*yscaled            */
        w2t = s40_mult_s32_s32_shift((int32_t)yscaled, *pdcs++,0);
        w2t = s40_shl_s40(w2t,shiftd);
        w2temp = s40_sub_s40_s40(w2temp,w2t); /* w2temp scaled version */

        w2temp = s32_saturate_s40(w2temp);

        *mem-- = (int32_t)w2temp;
        /*simulate the bus saturation*/
        /*store the output*/
        *out++=s16_saturate_s32((int32_t)ytemp);
                                          /* Performing this here in order  */
                                          /* to support in-place computation*/
}
    return(flag);
#endif //#if ((defined __hexagon__) || (defined __qdsp6__))
}
#endif


#ifndef QDSP6_ASM_IIRTDF2_32
void iirTDF2_32(int32 *inp,
            int32 *out,
            int32 samples,
            int32 *numcoefs,
            int32 *dencoefs,
            int64 *mem,
            int16 shiftn,
            int16 shiftd)
{
#if ((defined __hexagon__) || (defined __qdsp6__))
    /*
    * y in Q(64 - SHIFT - max(shiftn,shiftd) - 4)
    * yScaled in Q(32 - SHIFT)
    * w1 in Q(64 - SHIFT - max(shiftn,shiftd) - 3)
    * w2 in Q(64 - SHIFT - max(shiftn,shiftd) - 1)
    */
    int64 y, w1, w2;
    int32 yScaled; // yScaled will be the scaled version of y used while calculating w1 and w2

    int64 b0TimesX, b1TimesX, b2TimesX;
    int64 a1TimesY, a2TimesY;
    int64 w1Temp, w2Temp;

    int32 b0, b1, b2, a1, a2;
    int32 i, p5;

    //int16 shiftDiff = shiftn - shiftd;
    int32 shiftDiffX;
    int32 shiftDiffY;
    int32 shiftY;
    if (shiftn < shiftd) {
        shiftDiffX = shiftn - shiftd;
        shiftDiffY = 0;
        shiftY = shiftd-28;
    } else {
        shiftDiffX = 0;
        shiftDiffY = shiftd - shiftn;
        shiftY = shiftn-28;
    }

    if (shiftY <= -1){
		p5 = (int32)(1 << (-shiftY - 1));
	} else {
		p5 = 0;
	}

    b0 = *numcoefs;
    b1 = *(numcoefs + 1);
    b2 = *(numcoefs + 2);
    a1 = *dencoefs;
    a2 = *(dencoefs + 1);

    /* repeat the loop for every sample*/
    /* equations are y = b0*x+w1       */
    /*               w1= b1*x-a1*y+w2  */ // use yScaled
    /*               w2= b2*x-a2*y     */ // use yScaled

    for (i = 0; i < samples; ++i)
    {
        /******************************* calculate y ********************************/
        b0TimesX = Q6_P_mpy_RR( b0 , *inp);
        b0TimesX = Q6_P_asl_PR( b0TimesX, shiftDiffX - 4 );

        w1Temp = Q6_P_asr_PI( mem[0], 1 );
        y = Q6_P_add_PP( b0TimesX , w1Temp );

        // saturate y to 32-bits Q(32-SHIFT)
        // amount of right shift = 64 - SHIFT - max(shiftn,shiftd) - 4 - (32 - SHIFT)
        yScaled = Q6_R_sat_P(Q6_P_asl_PR(Q6_P_add_RP(p5,y),shiftY));

        /******************************* calculate w1 *******************************/
        b1TimesX = Q6_P_mpy_RR( b1 , *inp );
        a1TimesY = Q6_P_mpy_RR( a1 , yScaled );

        b1TimesX = Q6_P_asl_PR(b1TimesX, shiftDiffX - 3);
        a1TimesY = Q6_P_asl_PR(a1TimesY, shiftDiffY - 3);

        w1 = Q6_P_sub_PP(b1TimesX,a1TimesY);
        w2Temp = Q6_P_asr_PI( mem[1], 2 );
        w1 = Q6_P_add_PP(w1,w2Temp);
        mem[0] = w1;

        /******************************* calculate w2 *******************************/
        b2TimesX = Q6_P_mpy_RR( b2 , *inp++ );
        // inp incremented to next input location in the above line
        a2TimesY = Q6_P_mpy_RR( a2 , yScaled );
        b2TimesX = Q6_P_asl_PR(b2TimesX, shiftDiffX - 1);
        a2TimesY = Q6_P_asl_PR(a2TimesY, shiftDiffY - 1);

        w2 = Q6_P_sub_PP( b2TimesX , a2TimesY);
        mem[1] = w2;

        /***************************** store the output ******************************/
        *out++ = yScaled;
        /* Performing this here in order to support in-place computation */
    }

#else /* __qdsp6__ */

    int64 y, w1, w2;
    int32 yScaled; // yScaled will be the scaled version of y used while calculating w1 and w2
   /*
    * y in Q(64 - SHIFT - max(shiftn,shiftd) - 4)
    * yScaled in Q(32 - SHIFT)
    * w1 in Q(64 - SHIFT - max(shiftn,shiftd) - 3)
    * w2 in Q(64 - SHIFT - max(shiftn,shiftd) - 1)
    */

    int64 b0TimesX, b1TimesX, b2TimesX;
    int64 a1TimesY, a2TimesY;
    int64 w1Temp, w2Temp;

    int32 b0, b1, b2, a1, a2;
    int32 i, p5;

    int16 shiftDiff = shiftn - shiftd;
    int16 shiftY = 0;
    if (shiftn < shiftd) {
        shiftY = (int16)(-(28-shiftd));
    } else {
        shiftY = (int16)(-(28-shiftn));
    }

    if (shiftY <= -1){
		p5 = (int32)(1 << (-shiftY - 1));
	} else {
		p5 = 0;
	}

    b0 = *numcoefs;
    b1 = *(numcoefs + 1);
    b2 = *(numcoefs + 2);
    a1 = *dencoefs;
    a2 = *(dencoefs + 1);

    /* repeat the loop for every sample*/
    /* equations are y = b0*x+w1       */
    /*               w1= b1*x-a1*y+w2  */ // use yScaled
    /*               w2= b2*x-a2*y     */ // use yScaled

    if (shiftn < shiftd)
    {
        for (i = 0; i < samples; ++i)
        {
            /******************************* calculate y ********************************/
            b0TimesX = s64_mult_s32_s32( b0 , *inp);                                // Q(59-shiftn)
            b0TimesX = s64_shl_s64(b0TimesX, shiftDiff - 4);                        // Q(55-shiftd)

            w1Temp = *mem++; // after this, mem points to w2                        // Q(55-shiftd)
            y = s64_add_s64_s64( b0TimesX , w1Temp );                               // Q(55-shiftd)

            // saturate y to 32-bits in Q27
            yScaled = s32_saturate_s64(s64_shl_s64(s64_add_s64_s32(y, p5),shiftY)); // Q27,rounding

            /******************************* calculate w1 *******************************/
            b1TimesX = s64_mult_s32_s32( b1 , *inp );                               // Q(59-shiftn)
            a1TimesY = s64_mult_s32_s32( a1 , yScaled );                            // Q(59-shiftd)
            b1TimesX = s64_shl_s64( b1TimesX , shiftDiff - 4 );                     // Q(55-shiftd)
            a1TimesY = s64_shl_s64( a1TimesY, -4);                                  // Q(55-shiftd)

            w1 = s64_sub_s64_s64(b1TimesX,a1TimesY);                                // Q(55-shiftd)
            w2Temp = *mem--; // after this, mem points to w1                        // Q(55-shiftd)
            w1 = s64_add_s64_s64(w1,w2Temp);                                        // Q(55-shiftd)
            *mem++ = w1; // after this, mem points to w2                            // Q(55-shiftd)

            /******************************* calculate w2 *******************************/
            b2TimesX = s64_mult_s32_s32( b2 , *inp++ );                             // Q(59-shiftn)
            // inp incremented to next input location in the above line
            a2TimesY = s64_mult_s32_s32( a2 , yScaled );                            // Q(59-shiftd)
            b2TimesX = s64_shl_s64( b2TimesX , shiftDiff - 4 );                     // Q(55-shiftd)
            a2TimesY = s64_shl_s64( a2TimesY, -4);                                  // Q(55-shiftd)

            w2 = s64_sub_s64_s64( b2TimesX , a2TimesY);                             // Q(55-shiftd)
            *mem-- = w2; // after this, mem points to w1                            // Q(55-shiftd)

            /***************************** store the output ******************************/
            *out++ = yScaled;                                                       // Q27
            /* Performing this here in order to support in-place computation */
        }
    }
    else
    {
        for (i = 0; i < samples; ++i)
        {
            /******************************* calculate y ********************************/
            b0TimesX = s64_mult_s32_s32( b0 , *inp);                                 // Q(59-shiftn)
            b0TimesX = s64_shl_s64(b0TimesX, -4);                                    // Q(55-shiftn)

            w1Temp = *mem++; // after this, mem points to w2                         // Q(55-shiftn)
            y = s64_add_s64_s64( b0TimesX , w1Temp );                                // Q(55-shiftn)

            // saturate y to 32-bits in Q27
            yScaled = s32_saturate_s64(s64_shl_s64(s64_add_s64_s32(y, p5),shiftY)); // Q27, rounding

            /******************************* calculate w1 *******************************/
            b1TimesX = s64_mult_s32_s32( b1 , *inp );                                // Q(59-shiftn)
            a1TimesY = s64_mult_s32_s32( a1 , yScaled );                             // Q(59-shiftd)
            a1TimesY = s64_shl_s64( a1TimesY , -shiftDiff - 4 );                     // Q(55-shiftn)
            b1TimesX = s64_shl_s64( b1TimesX, -4);                                   // Q(55-shiftn)

            w1 = s64_sub_s64_s64(b1TimesX,a1TimesY);                                 // Q(55-shiftn)
            w2Temp = *mem--; // after this, mem points to w1                         // Q(55-shiftn)
            w1 = s64_add_s64_s64(w1,w2Temp);                                         // Q(55-shiftn)
            *mem++ = w1; // after this, mem points to w2                             // Q(55-shiftn)

            /******************************* calculate w2 *******************************/
            b2TimesX = s64_mult_s32_s32( b2 , *inp++ );                              // Q(59-shiftn)
            // inp incremented to next input location in the above line
            a2TimesY = s64_mult_s32_s32( a2 , yScaled );                             // Q(59-shiftd)
            a2TimesY = s64_shl_s64( a2TimesY , -shiftDiff - 4 );                     // Q(55-shiftn)
            b2TimesX = s64_shl_s64( b2TimesX, -4 );                                  // Q(55-shiftn)

            w2 = s64_sub_s64_s64( b2TimesX , a2TimesY);                              // Q(55-shiftn)
            *mem-- = w2; // after this, mem points to w1                             // Q(55-shiftn)

            /***************************** store the output ******************************/
            *out++ = yScaled;                                                        // Q27
            /* Performing this here in order to support in-place computation */
        }
    }
#endif /* __qdsp6__ for iirTDF2_32 */
}
#endif /* ifndef QDSP6_ASM_IIRTDF2_32 */

#ifndef QDSP6_ASM_IIRTDF2_16
void iirTDF2_16(int16 *inp,
            int16 *out,
            int32 samples,
            int32 *numcoefs,
            int32 *dencoefs,
            int64 *mem,
            int16 shiftn,
            int16 shiftd)
{
#if ((defined __hexagon__) || (defined __qdsp6__))

    int64 y, w1, w2;
    int32 yScaled;
   /*
    * y in Q(48 - SHIFT - max(shiftn,shiftd) - 4)
    * yScaled in Q(16 - SHIFT)
    * w1 in Q(48 - SHIFT - max(shiftn,shiftd) - 3)
    * w2 in Q(48 - SHIFT - max(shiftn,shiftd) - 1)
    */

    int64 b0TimesX, b1TimesX, b2TimesX;
    int64 a1TimesY, a2TimesY;
    int64 w1Temp, w2Temp;

    int32 b0, b1, b2, a1, a2;
    int32 i, p5;

    int32 shiftDiffX = 0;
    int32 shiftDiffY = 0;
    int32 shiftY = 0;
    if (shiftn < shiftd) {
        shiftY = (shiftd-12-GUARD_BITS_16);
        shiftDiffX = shiftn - shiftd;
    } else {
        shiftY = (shiftn-12-GUARD_BITS_16);
        shiftDiffY = shiftd - shiftn;
    }

    if (shiftY <= -1){
		p5 = (int32)(1 << (-shiftY - 1));
	} else {
		p5 = 0;
	}

    b0 = *numcoefs;
    b1 = *(numcoefs + 1);
    b2 = *(numcoefs + 2);
    a1 = *dencoefs;
    a2 = *(dencoefs + 1);

    /* repeat the loop for every sample*/
    /* equations are y = b0*x+w1       */
    /*               w1= b1*x-a1*y+w2  */ // use yScaled
    /*               w2= b2*x-a2*y     */ // use yScaled
    for (i = 0; i < samples; ++i)
    {
        /******************************* calculate y ********************************/
        b0TimesX = Q6_P_mpy_RR( b0 , *inp);                     //Q(47-shiftn)
        b0TimesX = Q6_P_asl_PR( b0TimesX , shiftDiffX - 4 );    //Q(47-shiftd- 4) = Q(43-shiftd)

        w1Temp = mem[0];                                        //mem1=Q(43-shiftd)
        y = Q6_P_add_PP( b0TimesX , w1Temp );                   //Q(43-shiftd)

        // saturate y to 32-bits Q(32-SHIFT)
        yScaled = Q6_R_sat_P(Q6_P_asl_PR(Q6_P_add_RP(p5,y),shiftY));            //Q(47-(15-shiftd)-shiftd-4) = Q(32 - 4) = Q(28), rounding

        /******************************* calculate w1 *******************************/
        b1TimesX = Q6_P_mpy_RR( b1 , *inp );                   //Q(47 - shiftn)
        a1TimesY = Q6_P_mpy_RR( a1 , yScaled );                //Q(28+32-shiftd) = Q(60-shiftd)
        b1TimesX = Q6_P_asl_PR( b1TimesX , shiftDiffX - 4 );   //Q(47-shiftn + shiftn-shiftd -4) = Q(43-shiftd)
        a1TimesY = Q6_P_asl_PR( a1TimesY, shiftDiffY-20+GUARD_BITS_16);    //Q(60-shiftd-20+3) = Q(43-shiftd)

        w1 = Q6_P_sub_PP(b1TimesX,a1TimesY);                   //Q(43-shiftd)
        w2Temp = mem[1];                                       //mem2=Q(43-shiftd)
        w1 = Q6_P_add_PP(w1,w2Temp);                           //Q(43-shiftd)
        mem[0] = w1;                                           //mem1=Q(43-shiftd)

        /******************************* calculate w2 *******************************/
        b2TimesX = Q6_P_mpy_RR( b2 , *inp++ );                 //Q(47-shiftn)
        // inp incremented to next input location in the above line
        a2TimesY = Q6_P_mpy_RR( a2 , yScaled );                //Q(28+32-shiftd) = Q(60-shiftd)
        b2TimesX = Q6_P_asl_PR( b2TimesX , shiftDiffX - 4);    //Q(47-shiftn+shiftn-shiftd-4) = Q(43-shiftd)
        a2TimesY = Q6_P_asl_PR( a2TimesY, shiftDiffY-20+GUARD_BITS_16);    //Q(60-shiftd-20+3) = Q(43-shiftd)

        w2 = Q6_P_sub_PP( b2TimesX , a2TimesY);               //Q(43-shiftd)
        mem[1] = w2;                                          //mem2=Q(43-shiftd)

        /***************************** store the output ******************************/
        yScaled = Q6_R_add_RR_sat(yScaled, 0x1000);
        *out++ = Q6_R_sath_R( yScaled >> (16-GUARD_BITS_16) );
        /* Performing this here in order to support in-place computation */
    }

#else /* __qdsp6__ */

    int64 y, w1, w2;
    int32 yScaled;
   /*
    * y in Q(48 - SHIFT - max(shiftn,shiftd) - 4)
    * yScaled in Q(16 - SHIFT)
    * w1 in Q(48 - SHIFT - max(shiftn,shiftd) - 3)
    * w2 in Q(48 - SHIFT - max(shiftn,shiftd) - 1)
    */

    int64 b0TimesX, b1TimesX, b2TimesX;
    int64 a1TimesY, a2TimesY;
    int64 w1Temp, w2Temp;

    int32 b0, b1, b2, a1, a2;
    int32 i, p5;

    int16 shiftDiff = shiftn - shiftd;
    int16 shiftY = 0;
    if (shiftn < shiftd) {
        shiftY = (int16)(-(12-shiftd+GUARD_BITS_16));
    } else {
        shiftY = (int16)(-(12-shiftn+GUARD_BITS_16));
    }

    if (shiftY <= -1){
		p5 = (int32)(1 << (-shiftY - 1));
	} else {
		p5 = 0;
	}

    b0 = *numcoefs;
    b1 = *(numcoefs + 1);
    b2 = *(numcoefs + 2);
    a1 = *dencoefs;
    a2 = *(dencoefs + 1);

    /* repeat the loop for every sample*/
    /* equations are y = b0*x+w1       */
    /*               w1= b1*x-a1*y+w2  */ // use yScaled
    /*               w2= b2*x-a2*y     */ // use yScaled

    if (shiftn < shiftd)
    {
        for (i = 0; i < samples; ++i)
        {
            /******************************* calculate y ********************************/
            b0TimesX = s64_mult_s32_s16( b0 , *inp);                                //Q(47-shiftn)
            b0TimesX = s64_shl_s64( b0TimesX , shiftDiff - 4 );                     //Q(47-shiftd- 4) = Q(43-shiftd)

            w1Temp = *mem++;                    // after this, mem points to w2     //mem1=Q(43-shiftd)
            y = s64_add_s64_s64( b0TimesX , w1Temp );                               //Q(43-shiftd)

            // saturate y to 32-bits Q(32-SHIFT)
            yScaled = s32_saturate_s64(s64_shl_s64(s64_add_s64_s32(y, p5),shiftY)); //Q(47-(15-shiftd)-shiftd-4) = Q(32 - 4) = Q(28), rounding

            /******************************* calculate w1 *******************************/
            b1TimesX = s64_mult_s32_s16( b1 , *inp );                               //Q(47 - shiftn)
            a1TimesY = s64_mult_s32_s32( a1 , yScaled );                            //Q(28+32-shiftd) = Q(60-shiftd)
            b1TimesX = s64_shl_s64( b1TimesX , shiftDiff - 4 );                     //Q(47-shiftn + shiftn-shiftd -4) = Q(43-shiftd)
            a1TimesY = s64_shl_s64( a1TimesY, -20+GUARD_BITS_16);                   //Q(60-shiftd-20+3) = Q(43-shiftd)

            w1 = s64_sub_s64_s64(b1TimesX,a1TimesY);                                //Q(43-shiftd)
            w2Temp = *mem--;                                                        //mem2=Q(43-shiftd)
            w1 = s64_add_s64_s64(w1,w2Temp);                                        //Q(43-shiftd)
            *mem++ = w1; // after this, mem points to w2*/                          //mem1=Q(43-shiftd)

            /******************************* calculate w2 *******************************/
            b2TimesX = s64_mult_s32_s16( b2 , *inp++ );                             //Q(47-shiftn)
            // inp incremented to next input location in the above line
            a2TimesY = s64_mult_s32_s32( a2 , yScaled );                            //Q(28+32-shiftd) = Q(60-shiftd)
            b2TimesX = s64_shl_s64( b2TimesX , shiftDiff - 4 );                     //Q(47-shiftn+shiftn-shiftd-4) = Q(43-shiftd)
            a2TimesY = s64_shl_s64( a2TimesY, -20+GUARD_BITS_16);                   //Q(60-shiftd-20+3) = Q(43-shiftd)

            w2 = s64_sub_s64_s64( b2TimesX , a2TimesY);                             //Q(43-shiftd)
            *mem-- = w2; // after this, mem points to w1*/                          //mem2=Q(43-shiftd)

            /***************************** store the output ******************************/
            yScaled = s32_add_s32_s32_sat(yScaled, 0x1000);                         //L32Q(28)
            *out++ = s16_saturate_s32(yScaled >> (16-GUARD_BITS_16));               //Q(28-13)=Q15
            /* Performing this here in order to support in-place computation */
        }
    }
    else
    {
        for (i = 0; i < samples; ++i)
        {
            /******************************* calculate y ********************************/
            b0TimesX = s64_mult_s32_s16( b0 , *inp); // Q(48 - SHIFT - shiftn)      //Q(47-shiftn)
            b0TimesX = s64_shl_s64(b0TimesX, -4);                                   //Q(43-shiftn)

            w1Temp = *mem++; // after this, mem points to w2                        //mem1=Q(43-shiftn)
            y = s64_add_s64_s64( b0TimesX , w1Temp );                               //Q(43-shiftn)

            // saturate y to 32-bits Q(32-4)
            yScaled = s32_saturate_s64(s64_shl_s64(s64_add_s64_s32(y, p5),shiftY)) ; // Q(43-shiftn-15+shiftn) = Q(28), rounding

            /******************************* calculate w1 *******************************/
            b1TimesX = s64_mult_s32_s16( b1 , *inp );                               //Q(47-shiftn)
            a1TimesY = s64_mult_s32_s32( a1 , yScaled );                            //Q(28+32-shiftd) = Q(60-shiftd)
            a1TimesY = s64_shl_s64( a1TimesY , -shiftDiff - 20 + GUARD_BITS_16);    //Q(60-shiftd-shiftn+shiftd-20+3) = Q(43-shiftn)
            b1TimesX = s64_shl_s64( b1TimesX, - 4);                                 //Q(43-shiftn)

            w1 = s64_sub_s64_s64(b1TimesX,a1TimesY);                                //Q(43-shiftn)
            w2Temp = *mem--;  // after this, mem points to w1                       //mem2=Q(43-shiftn),  Q(43-shiftn)
            w1 = s64_add_s64_s64(w1,w2Temp);                                        //Q(43-shiftn)
            *mem++ = w1; // after this, mem points to w2*/                          //mem1 = Q(43-shiftn)

            /******************************* calculate w2 *******************************/
            b2TimesX = s64_mult_s32_s16( b2 , *inp++ );                             //Q(47-shiftn)
            // inp incremented to next input location in the above line
            a2TimesY = s64_mult_s32_s32( a2 , yScaled );                            //Q(28+32-shiftd) = Q(60-shiftd)
            a2TimesY = s64_shl_s64( a2TimesY , -shiftDiff - 20 + GUARD_BITS_16);    //Q(60-shiftd-shiftn+shiftd-20+3) = Q(43-shiftn)
            b2TimesX = s64_shl_s64( b2TimesX, -4 );                                 //Q(43-shiftn)

            w2 = s64_sub_s64_s64( b2TimesX , a2TimesY);                             //Q(43-shiftn)
            *mem-- = w2; // after this, mem points to w1*/                          //mem2=Q(43-shiftn)

            /***************************** store the output ******************************/
            yScaled = s32_add_s32_s32_sat(yScaled, 0x1000);                         //L32Q(28)
            *out++ = s16_saturate_s32(yScaled >> (16 - GUARD_BITS_16));             //Q(28-13) = Q(15)
            /* Performing this here in order to support in-place computation */
        }
    }

#endif /* __qdsp6__ */
}

#endif /* ifndef QDSP6_ASM_IIRTDF2_16 */

