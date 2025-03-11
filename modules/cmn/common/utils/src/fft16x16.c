/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "audio_basic_op_ext.h"
#include "audio_complex_basic_op.h"

#include "fft_util.h"


/*****************************************************************/
/*     A fixed-point implementation of radix-4 scaled FFT        */
/*****************************************************************/
/*     Scale factor: 1/N                                         */
/*****************************************************************/
void sfft16x16(cint2x16 *input, int N, cint2x16 *w, cint2x16 *output) 
{
  int i, j, k1, k2, n, m;
  int LOG2N;
  cint2x16 Wa,Wb,Wc;
  cint2x16 A[4];


     LOG2N =  31 - s32_cl0_s32(N); 
  /*************************************/
  /*    Stage 1                        */
  /*  read input in bit-reversed order */
  /*************************************/
     for(i=0, m=0; i<N; i+=4) 
     {
       A[0]= input[ bitrev_s32(i  , LOG2N) ];
       A[1]= input[ bitrev_s32(i+1, LOG2N) ];
       A[2]= input[ bitrev_s32(i+2, LOG2N) ];
       A[3]= input[ bitrev_s32(i+3, LOG2N) ];

       sButterflyRadix4qv3_c32( A );

       Wb = w[m++];      // Wb = w[j];
       Wa = w[m++];      // Wa = w[2*j+1];
       Wc = w[m++];      // Wc = cmult_r(Wa,Wb);
       
       output[i  ] = A[0];
       output[i+1] = c32_mult_c32_c32( A[1], Wa );
       output[i+2] = c32_mult_c32_c32( A[2], Wb );
       output[i+3] = c32_mult_c32_c32( A[3], Wc );
     }

  /************************************/ 
  /*  Other Radix-4 stages            */
  /************************************/
     k1=  4;                           // # in each group
     k2 = N/16;                        // # of groups

     for(n = LOG2N-2; n >2 ; n -=2) 
     {
       for (i=0, m=0; i< k2; i++) 
       {
          Wb = w[m++];      // Wb = w[i];
          Wa = w[m++];      // Wa = w[2*i+1];
          Wc = w[m++];      // Wc = cmult_r(Wa,Wb);
         
          for(j=0; j< k1; j++) 
          {
               A[0]= output[(4*i + 0)*k1 + j ];
               A[1]= output[(4*i + 1)*k1 + j ];
               A[2]= output[(4*i + 2)*k1 + j ];
               A[3]= output[(4*i + 3)*k1 + j ];  

               sButterflyRadix4qv3_c32( A );

               output[(4*i + 0)*k1 + j] = A[0];
               output[(4*i + 1)*k1 + j] = c32_mult_c32_c32(A[1], Wa);
               output[(4*i + 2)*k1 + j] = c32_mult_c32_c32(A[2], Wb);
               output[(4*i + 3)*k1 + j] = c32_mult_c32_c32(A[3], Wc);
          }
       }
       k1 = k1 << 2;
       k2 = k2 >> 2;
    }

    if(n == 2) 
    {
     /************************************/ 
     /*  last Radix-4 stage              */
     /************************************/
     for(j=0; j< N/4; j++)
     {
          A[0]= output[0*(N/4) + j];
          A[1]= output[1*(N/4) + j];
          A[2]= output[2*(N/4) + j];
          A[3]= output[3*(N/4) + j];  

          sButterflyRadix4_c32( A );

          // No multiplcations needed
          output[0*(N/4) + j] = A[0];
          output[1*(N/4) + j] = A[1];
          output[2*(N/4) + j] = A[2];
          output[3*(N/4) + j] = A[3];
      }
    }
    else
    {
     /************************************/ 
     /*  last Radix-2 stage              */
     /************************************/
      for(i=0; i<N/2; i++) {
          A[0] = output[i    ];
          A[1] = output[i+N/2];

          sButterflyRadix2_c32( A );

          output[i    ] = A[0]; 
          output[i+N/2] = A[1]; 
      }
    }
}



/*****************************************************************/
/*     A fixed-point implementation of radix-4 FFT               */
/*****************************************************************/
/*  NOTE: The ONLY difference with scaled version is butterfly   */
/*        does not have scaling factor of 1/2 or 1/4             */
/*****************************************************************/
void fft16x16(cint2x16 *input, int N, cint2x16 *w, cint2x16 *output) 
{
  int i, j, k1, k2, n, m;
  int LOG2N;
  cint2x16 Wa,Wb,Wc;
  cint2x16 A[4];


     LOG2N =  31 - s32_cl0_s32(N); 
  /*************************************/
  /*    Stage 1                        */
  /*  read input in bit-reversed order */
  /*************************************/
     for(i=0, m=0; i<N; i+=4) 
     {
       A[0]= input[ bitrev_s32(i  , LOG2N) ];
       A[1]= input[ bitrev_s32(i+1, LOG2N) ];
       A[2]= input[ bitrev_s32(i+2, LOG2N) ];
       A[3]= input[ bitrev_s32(i+3, LOG2N) ];

       ButterflyRadix4qv3_c32( A );

       Wb = w[m++];      // Wb = w[j];
       Wa = w[m++];      // Wa = w[2*j+1];
       Wc = w[m++];      // Wc = cmult_r(Wa,Wb);
       
       output[i  ] = A[0];
       output[i+1] = c32_mult_c32_c32( A[1], Wa );
       output[i+2] = c32_mult_c32_c32( A[2], Wb );
       output[i+3] = c32_mult_c32_c32( A[3], Wc );
     }

  /************************************/ 
  /*  Other Radix-4 stages            */
  /************************************/
     k1=  4;                           // # in each group
     k2 = N/16;                        // # of groups

     for(n = LOG2N-2; n >2 ; n -=2) 
     {
       for (i=0, m=0; i< k2; i++) 
       {
          Wb = w[m++];      // Wb = w[i];
          Wa = w[m++];      // Wa = w[2*i+1];
          Wc = w[m++];      // Wc = cmult_r(Wa,Wb);
         
          for(j=0; j< k1; j++) 
          {
               A[0]= output[(4*i + 0)*k1 + j ];
               A[1]= output[(4*i + 1)*k1 + j ];
               A[2]= output[(4*i + 2)*k1 + j ];
               A[3]= output[(4*i + 3)*k1 + j ];  

               ButterflyRadix4qv3_c32( A );

               output[(4*i + 0)*k1 + j] = A[0];
               output[(4*i + 1)*k1 + j] = c32_mult_c32_c32(A[1], Wa);
               output[(4*i + 2)*k1 + j] = c32_mult_c32_c32(A[2], Wb);
               output[(4*i + 3)*k1 + j] = c32_mult_c32_c32(A[3], Wc);
          }
       }
       k1 = k1 << 2;
       k2 = k2 >> 2;
    }

    if(n == 2) 
    {
     /************************************/ 
     /*  last Radix-4 stage              */
     /************************************/
     for(j=0; j< N/4; j++)
     {
          A[0]= output[0*(N/4) + j];
          A[1]= output[1*(N/4) + j];
          A[2]= output[2*(N/4) + j];
          A[3]= output[3*(N/4) + j];  

          ButterflyRadix4_c32( A );

          // No multiplcations needed
          output[0*(N/4) + j] = A[0];
          output[1*(N/4) + j] = A[1];
          output[2*(N/4) + j] = A[2];
          output[3*(N/4) + j] = A[3];
      }
    }
    else
    {
     /************************************/ 
     /*  last Radix-2 stage              */
     /************************************/
      for(i=0; i<N/2; i++) {
          A[0] = output[i    ];
          A[1] = output[i+N/2];

          ButterflyRadix2_c32( A );

          output[i    ] = A[0]; 
          output[i+N/2] = A[1]; 
      }
    }
}
