/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*****************************************************************/
/*     This program is a fixed-point implementation of           */
/*     radix-4 FFT. It is used as reference for ASM              */
/*     implementation.                                           */
/*****************************************************************/

#ifndef __qdsp6__
#include "ffsp_basic_ops.h"
#include "fft_util.h"
#include "fft.h"

void ifft32x16(CWord2x32 *input, int N, CWord2x16 *w, CWord2x32 *output) {

  int i, j, k1, k2, n, m;
  int LOG2N;
  CWord2x16 Wa,Wb,Wc;
  CWord2x32 A[4];

 /*************************************/
 /*    Stage 1                        */
 /*  read input in bit-reversed order */
 /**************************************/
     LOG2N =  31 - cl0(N); 

     j = 0;
     for(i=0; i<N; i+=4) {

        A[0]= input[ bitrev(i  , LOG2N) ];
        A[1]= input[ bitrev(i+1, LOG2N) ];
        A[2]= input[ bitrev(i+2, LOG2N) ];
        A[3]= input[ bitrev(i+3, LOG2N) ];

       ifftButterflyRadix4( A );

       Wb = w[j  ];      // Wb = w[j];
       Wa = w[j+1];      // Wa = w[2*j+1];
       Wc = w[j+2];      // Wc = cmult_r(Wa,Wb);
       j += 3;
       

       output[i  ] = A[0];
       output[i+1] = L_cmult32x16( A[1], conjugate(Wa) );
       output[i+2] = L_cmult32x16( A[2], conjugate(Wb) );
       output[i+3] = L_cmult32x16( A[3], conjugate(Wc) );
     }

  /************************************/ 
  /*  Other Radix-4 stages            */
  /************************************/
     k1=  4;                           // # in each group
     k2 = N/16;                        // # of groups

    for(n = 0; n < (LOG2N/2 -1) ; n++) {

       m = 0;

       for (i=0; i< k2; i++) {

          Wb = w[m  ];      // Wb = w[i];
          Wa = w[m+1];      // Wa = w[2*i+1];
          Wc = w[m+2];      // Wc = cmult_r(Wa,Wb);
          m += 3;

          for(j=0; j< k1; j++) {
               A[0]= output[(4*i + 0)*k1 + j ];
               A[1]= output[(4*i + 1)*k1 + j ];
               A[2]= output[(4*i + 2)*k1 + j ];
               A[3]= output[(4*i + 3)*k1 + j ];  

               ifftButterflyRadix4( A );

               output[(4*i + 0)*k1 + j] = A[0];
               output[(4*i + 1)*k1 + j] = L_cmult32x16(A[1], conjugate(Wa));
               output[(4*i + 2)*k1 + j] = L_cmult32x16(A[2], conjugate(Wb));
               output[(4*i + 3)*k1 + j] = L_cmult32x16(A[3], conjugate(Wc));
          }
       }
       k1 = k1 << 2;
       k2 = k2 >> 2;
    }

    if(LOG2N&1) {

  /************************************/ 
  /*  Radix-2 stage                   */
  /************************************/

      for(i=0; i<N/2; i++) {
          A[0] = output[i    ];
          A[1] = output[i+N/2];

          ifftButterflyRadix2( A );

          output[i    ] = A[0];	
          output[i+N/2] = A[1];	
      }
    }

}



void rifft(CWord2x32 *input, int N, const CWord2x16 *Wt1, const CWord2x16 *Wt2, CWord2x32 *output) 
{
  int i;
  CWord2x32 X, Y;
  CWord2x32 *Z = (CWord2x32 *) output;
  
 /*************************************/
 /*  Calculate last stage butterflies */
 /**************************************/
  // modify input in place
  // calculate IFFT at k=0, k=N/2

  input[0] = L_complex ( L_real( L_Vavg(input[0], input[N/2])),
                         L_real(L_Vnavg(input[0], input[N/2])) );

  for (i=1; i<= N/4; i++) {

     X =  L_Vavg(input[i], L_conjugate(input[N/2-i]));
     Y = L_Vnavg(input[i], L_conjugate(input[N/2-i]));

     Y = L_cmult32x16(Y, conjugate(Wt2[i-1]));
     input[i    ] = L_Vsub(X, Y);
     //input[i    ] = L_Vnavg(X, Y);

     if (i!= N/4) {
       input[N/2-i] = L_conjugate(L_Vadd(X, Y));
       //input[N/2-i] = L_conjugate(L_Vavg(X, Y));
        }
  }

 /*************************************/
 /* Construct complex array with even */
 /* input as real and odd input as    */ 
 /* imaginary parts                   */
 /* Then do N/2-point complex FFT     */
 /*************************************/

  ifft32x16(input, N/2, Wt1, Z); 
}
#endif
