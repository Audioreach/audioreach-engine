/* Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
/****************************************************************************
 * FILE NAME: tangent.c
 * DESCRIPTION:
 *    Compute the tangent(x) value.
*****************************************************************************/
# include "tangent.h"


int32 tangent_v2( int32 theta, int n )
{
    int32 angle, c, c2,factor, poweroftwo;
	int32 j, s, s2, t=0, K_SR=0;

    int32 angles[ANGLES_LENGTH] = {
    843314857,
    497837829,
    263043837,
    133525159,
    67021687,
    33543516,
    16775851,
    8388437,
    4194283,
    2097149,
    1048576,
    524288,
    262144,
    131072,
    65536,
    32768,
    16384,
    8192,
    4096,
    2048};
    angle = angles[0];
    theta = theta << 2;
    c = UNIT_DEFINED_QFACTOR;
    poweroftwo = UNIT_DEFINED_QFACTOR;
    s = 0;


    for ( j = 1; j <= n; j++ )
    {
        if ( theta < 0.0 )
        {
              factor = -poweroftwo;
              theta = theta + angle;
        }
        else
        {
              factor = poweroftwo;
              theta = theta - angle;
        }
        c2 =  c - mul32x32Shift2_v2(factor, s, 2);
        s2 = mul32x32Shift2_v2(factor, c, 2) + s;

        c = c2;
        s = s2;

        poweroftwo = poweroftwo >> 1;
        
        //  Update the angle from table, or just divided by two.
        if ( j > ANGLES_LENGTH -1 )
        {
            angle = angle >> 1;
        }
        else
        {
            angle = angles[j];
        }
    }

    dsplib_approx_divideEQFD_v2(s,c, &t, &K_SR); 

    // shift the output to make the tangent(x) output to be Q28 fixedpoint format
    if (K_SR > 28) {
        t = t >> (K_SR-28);
    } else {
        t = t << (28-K_SR);
    }

  return t;
}

/* multiply two 32-bit numbers, shiftLeft */
/* if x = q20, y = q20, shift = 0 
   => result = q20+q20-q32+q0 = q8 */
int32 mul32x32Shift2_v2(int32 x, int32 y, int16 shift)
{
    short isNeg;
    uint16 xh, xl, yh, yl;
    uint32 result;
    uint64 res64, resm;

    isNeg = 0;

    if (x < 0) {
        x = -x;
        isNeg = ~isNeg;
    }
    if (y < 0) {
        y = -y;
        isNeg = ~isNeg;
    }
    //printf("\nx=%x, y=%x,shift=%d",x,y,shift);
    xh = (uint16)( x >> 16 );
    xl = (uint16)( x & 0x0000FFFF );
    yh = (uint16)( y >> 16 );
    yl = (uint16)( y & 0x0000FFFF );

    res64 = (uint64)((xl*yl)+0x8000);
    res64 = res64 &(int64)(0x0ffffffff);
    //printf("\nLower=%x",(uint32)res64);
    res64 = (uint64)(res64>>(16-shift));
    //printf("\nLower=%x",(uint32)res64);
    
    resm  = (uint64)((uint64)xh*yl+(uint64)xl*yh)<<shift;
    //printf("\nMiddle=%x",(uint64)resm);
    res64 = (uint64)(res64+resm);
    //printf("\nMiddle=%x",(uint32)res64);
    res64 = res64>>16;
    
    resm  = (uint64)(xh*yh)<<shift;
    //printf("\nHigher=%x",(uint64)resm);
    res64 = res64+resm;
    result = s32_saturate_s64(res64);
    //printf("\nresults=%x\n",result);
    
    return ( isNeg ? - ( (int32)result ) : (int32)result );
}


PPStatus dsplib_approx_divideEQFD_v2(int32 numer,int32 denom,int32 *result,int32 *shift_factor)
{
  int32 norm_num,r,s_d,s_n;
  
    
/*using the other one*/  
  //dsplib_approx_invert(denom,&r,&s_d);
  PPStatus retval;
  //denom is in Q28 for bandboost, but 1/denom = r is in Q31
  retval = dsplib_taylor_invert_v2(denom,&r,&s_d,21);

  if (retval)
  {
      return retval;
  }
  s_d=s_d+1;
  qdsp_norm_v2(numer,&norm_num,&s_n);
  *shift_factor = s_n - s_d;

  //printf("\nInverse=%x,num=%x\n",r,norm_num);

  *result = mul32x32Shift2_v2(norm_num,r,1);
  //printf("\nresult=%x\n",*result);

  return PPSUCCESS;
}

/*Function to do inverse of a number using Taylor's series expansion*/
/* 1/x = 1+y+y^2+...+y^iters, where y=1-x.*/
PPStatus dsplib_taylor_invert_v2(int32 input,int32 *res,int32 *shift_factor,int32 iters)
{
 int32 norm_divisor;
 int32 r;
 int32 y;
 short i;
 uint32 result;
 int32 oneQ30;
 oneQ30 = 1<<30;
 result = 0;

 /* check for negative input (invalid) */
  if (input < 0)
  {
    //fprintf(stderr, "dsp_libs.c -> dsplib_approx_invert(): negative input, invalid.\n");
      return PPFAILURE;
    //exit(-1);
  }
 /* bit-align-normalize input */
  /* output wd always be in 0.5 to 1 */
  qdsp_norm_v2(input, &norm_divisor, &r);

  //printf("\ninput=%d,sr=%d",norm_divisor,r);

   /* calculate y in Q30*/
   y=(int32)0x40000000 - (norm_divisor>>1);
   result = oneQ30+y; /*result = 1+y */
   for(i=0;i<iters;i++)
   {
       /*result = 1+y*result  */
       result=oneQ30+mul32x32Shift2_v2(y,result,2);
       //printf("\nresult=%d",result);
   }

 *res=result;
 *shift_factor=r-31;

return PPSUCCESS;
}/*End of Taylor's series expansion*/

int32 qdsp_norm_v2(int32 input,int32 *output, int32 *shift_factor) 
{
  int32 sf;
  sf = 0;

  if (input)
  { 
    /* non-zero input, shift left until "normed" */
    while (((input << 1) ^ ~input) & 0x80000000)
    {
      input <<= 1;
      sf++;
    }
    *output       = input;
    *shift_factor = sf;
  } 
  else
  {
    /* zero input, leave as zero */
    *output       = 0;
    *shift_factor = 0;
  }
  return 1;
}
