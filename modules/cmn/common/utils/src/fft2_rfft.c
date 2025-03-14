/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*============================================================================
  FILE:          fft2_rfft.c

  OVERVIEW:      Implement real-valued forward and inverse FFT (QDSP6
                 reference).

  DEPENDENCIES:  None

============================================================================*/

/*****************************************************************/
/*     This program is used to as a reference for                */
/*     QDSP6 implementation.                                     */
/* ============================================================= */    
/*  Reference:                                                   */
/*  "Real-Valued Fast Fourier Transform Algorithms",             */
/*  IEEE Trans. on ASSP, vol 35, pp. 849-863, June 1987          */
/*****************************************************************/

#include "fft2_util.h"
#include "fft2_private_comdef.h"

void cmplxfft(CWord2x16 *input, int N, const CWord2x16 *w, CWord2x16 *output) 
{
  int i, j, k1, k2, n, m;
  int LOG2N;
  CWord2x16 Wa,Wb,Wc;
  CWord2x16 A[4];


     LOG2N =  31 - cl0(N); 
 /*************************************/
 /*    Stage 1                        */
 /*  read input in bit-reversed order */
 /**************************************/

     j = 0;
     for(i=0; i<N; i+=4) {

        A[0]= input[ bitrev(i  , LOG2N) ];
        A[1]= input[ bitrev(i+1, LOG2N) ];
        A[2]= input[ bitrev(i+2, LOG2N) ];
        A[3]= input[ bitrev(i+3, LOG2N) ];

       ButterflyRadix4Scaled( A );

       Wb = w[j  ];      // Wb = w[j];
       Wa = w[j+1];      // Wa = w[2*j+1];
       Wc = w[j+2];      // Wc = cmult_r(Wa,Wb);
       j += 3;
       
       output[i  ] = A[0];
       output[i+1] = cmult_r( A[1], Wa );
       output[i+2] = cmult_r( A[2], Wb );
       output[i+3] = cmult_r( A[3], Wc );
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

               ButterflyRadix4Scaled( A );

               output[(4*i + 0)*k1 + j] = A[0];
               output[(4*i + 1)*k1 + j] = cmult_r(A[1], Wa);
               output[(4*i + 2)*k1 + j] = cmult_r(A[2], Wb);
               output[(4*i + 3)*k1 + j] = cmult_r(A[3], Wc);
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

        ButterflyRadix2Scaled( A );

        output[i    ] = A[0];	
        output[i+N/2] = A[1];	
	  }
    }
}

void cmplxifft(CWord2x16 *input, int N, const CWord2x16 *w, CWord2x16 *output)
{
  int i, j, k1, k2, n, m;
  int LOG2N;
  CWord2x16 Wa,Wb,Wc;
  CWord2x16 A[4];


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
       output[i+1] = cmult_cr( A[1], Wa );
       output[i+2] = cmult_cr( A[2], Wb );
       output[i+3] = cmult_cr( A[3], Wc );
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
               output[(4*i + 1)*k1 + j] = cmult_cr(A[1], Wa);
               output[(4*i + 2)*k1 + j] = cmult_cr(A[2], Wb);
               output[(4*i + 3)*k1 + j] = cmult_cr(A[3], Wc);
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


/***************************************************************/
/* Function: N-point scaled complex-valued FFT                 */
/***************************************************************/
/* - scaling factor = 1/N                                      */
/***************************************************************/

void cmplxfft32x16_scaled(CWord2x32 *input, int N, const CWord2x16 *w, CWord2x32 *output) {

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

       ButterflyRadix4Scaled_32x16( A );

       Wb = w[j  ];      // Wb = w[j];
       Wa = w[j+1];      // Wa = w[2*j+1];
       Wc = w[j+2];      // Wc = cmult_r(Wa,Wb);
       j += 3;
       

       output[i  ] = A[0];
       output[i+1] = L_cmult32x16( A[1], Wa );
       output[i+2] = L_cmult32x16( A[2], Wb );
       output[i+3] = L_cmult32x16( A[3], Wc );
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

               ButterflyRadix4Scaled_32x16( A );

               output[(4*i + 0)*k1 + j] = A[0];
               output[(4*i + 1)*k1 + j] = L_cmult32x16(A[1], Wa);
               output[(4*i + 2)*k1 + j] = L_cmult32x16(A[2], Wb);
               output[(4*i + 3)*k1 + j] = L_cmult32x16(A[3], Wc);
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

          ButterflyRadix2Scaled_32x16( A );

          output[i    ] = A[0];	
          output[i+N/2] = A[1];	
      }
    }

}

/***************************************************************/
/* Function: N-point unscaled complex-valued FFT               */
/***************************************************************/

static void cmplxfft32x16(CWord2x32 *input, int N, const CWord2x16 *w, CWord2x32 *output) {

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

       ButterflyRadix4_32x16( A );

       Wb = w[j  ];      // Wb = w[j];
       Wa = w[j+1];      // Wa = w[2*j+1];
       Wc = w[j+2];      // Wc = cmult_r(Wa,Wb);
       j += 3;

       output[i  ] = A[0];
       output[i+1] = L_cmult32x16( A[1], Wa );
       output[i+2] = L_cmult32x16( A[2], Wb );
       output[i+3] = L_cmult32x16( A[3], Wc );
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

               ButterflyRadix4_32x16( A );

               output[(4*i + 0)*k1 + j] = A[0];
               output[(4*i + 1)*k1 + j] = L_cmult32x16(A[1], Wa);
               output[(4*i + 2)*k1 + j] = L_cmult32x16(A[2], Wb);
               output[(4*i + 3)*k1 + j] = L_cmult32x16(A[3], Wc);
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

          ButterflyRadix2_32x16( A );

          output[i    ] = A[0];
          output[i+N/2] = A[1];
      }
    }

}


/***************************************************************/
/* Function: N-point scaled complex-valued inverse FFT         */
/***************************************************************/
/* - scaling factor = 1/N                                      */
/***************************************************************/

void cmplxifft32x16_scaled(CWord2x32 *input, int N, const CWord2x16 *w, CWord2x32 *output) {

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

       ifftButterflyRadix4Scaled_32x16( A );

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

               ifftButterflyRadix4Scaled_32x16( A );

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

          ifftButterflyRadix2Scaled_32x16( A );

          output[i    ] = A[0];	
          output[i+N/2] = A[1];	
      }
    }

}


/***************************************************************/
/* Function: N-point unscaled complex-valued inverse FFT         */
/***************************************************************/

void cmplxifft32x16(CWord2x32 *input, int N, const CWord2x16 *w, CWord2x32 *output) {

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

       ifftButterflyRadix4_32x16( A );

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

               ifftButterflyRadix4_32x16( A );

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

          ifftButterflyRadix2_32x16( A );

          output[i    ] = A[0];	
          output[i+N/2] = A[1];	
      }
    }

}


/***************************************************************/
/* Function: N-point scaled real-valued FFT                    */
/***************************************************************/
/* - scaling factor = 2/N                                      */
/***************************************************************/

void Ffecns_rfft(Word16 *input, int N, const CWord2x16 *Wt1, const CWord2x16 *Wt2, int16 fc, CWord2x16 *output) 
{
  int i;
  CWord2x16 X, Y;
  CWord2x16 *Z = output;

 /*************************************/
 /* Construct complex array with even */
 /* input as real and odd input as    */ 
 /* imaginary parts                   */
 /* Then do N/2-point complex FFT     */
 /*************************************/

  cmplxfft((CWord2x16 *)input, N/2, Wt1, Z); 
  
 /*************************************/
 /*  Calculate last stage butterflies */
 /**************************************/
  // calculate FFT at k=0, k=N/2
  X = complex(real(Z[0]), 0);
  Y = complex(imag(Z[0]), 0);

  output[0  ] = cadd(X,Y);
  output[N/2] = csub(X,Y);

  {
     for (i=1; i<= N/4; i++) {

        X =  cavg(Z[i], conjugate(Z[N/2-i]));
        Y = cnavg(Z[i], conjugate(Z[N/2-i]));

        Y = cmult_r(Wt2[fc*i-1],Y);
        output[i    ] = csub(X, Y);

        if (i!= N/4) {
           output[N/2-i] = conjugate(cadd(X, Y));
        }
     }
  }
}

void Ffecns_rifft(CWord2x16 *input, int N, const CWord2x16 *Wt1, const CWord2x16 *Wt2, int16 fc, Word16 *output) 
{
  int i;
  CWord2x16 X, Y;
  CWord2x16 *Z = (CWord2x16 *) output;

 /*************************************/
 /*  Calculate last stage butterflies */
 /**************************************/
  // modify input in place
  // calculate IFFT at k=0, k=N/2

  input[0] = complex ( real(cavg(input[0], input[N/2])),
                       real(cnavg(input[0], input[N/2])) );

  {
     for (i=1; i<= N/4; i++) {

        X =  cavg(input[i], conjugate(input[N/2-i]));
        Y = cnavg(input[i], conjugate(input[N/2-i]));

        Y = cmult_cr(Y,Wt2[fc*i-1]);
        input[i    ] = csub(X, Y);

        if (i!= N/4) {
           input[N/2-i] = conjugate(cadd(X, Y));
        }
     }
  }

 /*************************************/
 /* Construct complex array with even */
 /* input as real and odd input as    */ 
 /* imaginary parts                   */
 /* Then do N/2-point complex FFT     */
 /*************************************/

  cmplxifft(input, N/2, Wt1, Z); 
}


/***************************************************************/
/* Function: N-point scaled real-valued FFT for 32-bit input   */
/*           and 16-bit twiddle factors                        */
/***************************************************************/
/* - scaling factor = 2/N                                      */
/***************************************************************/

void Ffecns_rfft_32x16_scaled(Word32 *input, int N, const CWord2x16 *Wt1, const CWord2x16 *Wt2, int16 fc, CWord2x32 *output) 
{
  int i;
  CWord2x32 X, Y;
  CWord2x32 *Z = output;

 /*************************************/
 /* Construct complex array with even */
 /* input as real and odd input as    */
 /* imaginary parts                   */
 /* Then do N/2-point complex FFT     */
 /*************************************/

  cmplxfft32x16_scaled((CWord2x32 *)input, N/2, Wt1, Z);

 /*************************************/
 /*  Calculate last stage butterflies */
 /**************************************/
  // calculate FFT at k=0, k=N/2
  X = L_complex(L_real(Z[0]), 0);
  Y = L_complex(L_imag(Z[0]), 0);

  output[0  ] = L_Vadd(X,Y);
  output[N/2] = L_Vsub(X,Y);

  {
     for (i=1; i<= N/4; i++) {

        X =  L_Vavg(Z[i], L_conjugate(Z[N/2-i]));
        Y = L_Vnavg(Z[i], L_conjugate(Z[N/2-i]));

        Y = L_cmult32x16(Y, Wt2[fc*i-1]);
        output[i    ] = L_Vsub(X, Y);

        if (i!= N/4) {
           output[N/2-i] = L_conjugate(L_Vadd(X, Y));
        }
     }
  }
}


/***************************************************************/
/* Function: N-point unscaled real-valued FFT for 32-bit input */
/*           and 16-bit twiddle factors                        */
/***************************************************************/

void Ffecns_rfft_32x16(Word32 *input, int N, const CWord2x16 *Wt1, const CWord2x16 *Wt2, int16 fc, CWord2x32 *output) 
{
  int i;
  CWord2x32 X, Y;
  CWord2x32 *Z = output;

 /*************************************/
 /* Construct complex array with even */
 /* input as real and odd input as    */
 /* imaginary parts                   */
 /* Then do N/2-point complex FFT     */
 /*************************************/

  cmplxfft32x16((CWord2x32 *)input, N/2, Wt1, Z);

 /*************************************/
 /*  Calculate last stage butterflies */
 /**************************************/
  // calculate FFT at k=0, k=N/2
  X = L_complex(L_real(Z[0]), 0);
  Y = L_complex(L_imag(Z[0]), 0);

  output[0  ] = L_Vadd(X,Y);
  output[N/2] = L_Vsub(X,Y);

  {
     for (i=1; i<= N/4; i++) {

        X =  L_Vavg(Z[i], L_conjugate(Z[N/2-i]));
        Y = L_Vnavg(Z[i], L_conjugate(Z[N/2-i]));

        Y = L_cmult32x16(Y, Wt2[fc*i-1]);
        output[i    ] = L_Vsub(X, Y);

        if (i!= N/4) {
           output[N/2-i] = L_conjugate(L_Vadd(X, Y));
        }
     }
  }
}

/***************************************************************/
/* Function: N-point scaled real-valued FFT for 32-bit input   */
/*           and 16-bit twiddle factors                        */
/***************************************************************/
/* - scaling factor = 1/N                                      */
/***************************************************************/
/* - requires 64-bit alignment of output                       */
/***************************************************************/

void Ffecns_rifft_32x16_scaled(CWord2x32 *input, int N, const CWord2x16 *Wt1, const CWord2x16 *Wt2, int16 fc, Word32 *output) 
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

  {
     for (i=1; i<= N/4; i++) {

        X =  L_Vavg(input[i], L_conjugate(input[N/2-i]));
        Y = L_Vnavg(input[i], L_conjugate(input[N/2-i]));

        Y = L_cmult32x16(Y, conjugate(Wt2[fc*i-1]));
        input[i    ] = L_Vsub(X, Y);
        //input[i    ] = L_Vnavg(X, Y);

        if (i!= N/4) {
           input[N/2-i] = L_conjugate(L_Vadd(X, Y));
           //input[N/2-i] = L_conjugate(L_Vavg(X, Y));
        }
     }
  }

 /*************************************/
 /* Construct complex array with even */
 /* input as real and odd input as    */ 
 /* imaginary parts                   */
 /* Then do N/2-point complex FFT     */
 /*************************************/

  cmplxifft32x16_scaled(input, N/2, Wt1, Z); 
}

/*----------------------- FFT upgrade ------------------------------------ */
CWord2x32 L_cmult32x32r( CWord2x32 x, CWord2x32 y)
{
   Word32 xRe, xIm; 
   Word32 yRe, yIm;
   Word32 zRe, zIm;
   Word64 tmpL64;

   xRe = L_real(x);  xIm = L_imag(x);
   yRe = L_real(y);  yIm = L_imag(y);
   
   // the complex MPY is defined in such a way in order
   // to facilitate faster ASM implementation on Q6
   tmpL64 = s64_mult_s32_s32_shift(xRe, yRe, 31);    //Q31+15
   tmpL64 = s64_add_s64_s64( tmpL64, s64_mult_s32_s32_shift(L_negate(xIm), yIm, 31) );
   zRe = s32_saturate_s64(s64_shl_s64(tmpL64, -30));

   tmpL64 = s64_mult_s32_s32_shift(xIm, yRe, 31);
   tmpL64 = s64_add_s64_s64( tmpL64, s64_mult_s32_s32_shift(xRe, yIm, 31));
   zIm = s32_saturate_s64(s64_shl_s64(tmpL64, -30));

   return (L_complex(zRe, zIm));
}

/***************************************************************/
/* Function: N-point unscaled complex-valued FFT               */
/***************************************************************/

void cmplxfft32x32(CWord2x32 *input, int N, const CWord2x32 *w, CWord2x32 *output)
{
  int i, j, k1, k2, n, m;
  int LOG2N;
  CWord2x32 Wa,Wb,Wc;
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

       ButterflyRadix4_32x16( A );
       //ButterflyRadix4Scaled_32x16( A );

       Wb = w[j  ];      // Wb = w[j];
       Wa = w[j+1];      // Wa = w[2*j+1];
       Wc = w[j+2];      // Wc = cmult_r(Wa,Wb);
       j += 3;

       output[i  ] = A[0];
       output[i+1] = L_cmult32x32r( A[1], Wa );
       output[i+2] = L_cmult32x32r( A[2], Wb );
       output[i+3] = L_cmult32x32r( A[3], Wc );
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

               ButterflyRadix4_32x16( A );
               //ButterflyRadix4Scaled_32x16( A );

               output[(4*i + 0)*k1 + j] = A[0];
               output[(4*i + 1)*k1 + j] = L_cmult32x32r(A[1], Wa);
               output[(4*i + 2)*k1 + j] = L_cmult32x32r(A[2], Wb);
               output[(4*i + 3)*k1 + j] = L_cmult32x32r(A[3], Wc);
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

          ButterflyRadix2_32x16( A );
          //ButterflyRadix2Scaled_32x16( A );

          output[i    ] = A[0];
          output[i+N/2] = A[1];
      }
    }

}

/***************************************************************/
/* Function: N-point scaled complex-valued inverse FFT         */
/***************************************************************/
/* - scaling factor = 1/N                                      */
/***************************************************************/

void cmplxifft32x32_scaled(CWord2x32 *input, int N, const CWord2x32 *w, CWord2x32 *output) {

  int i, j, k1, k2, n, m;
  int LOG2N;
  CWord2x32 Wa,Wb,Wc;
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

       ifftButterflyRadix4Scaled_32x16( A );
       //ifftButterflyRadix4_32x16( A );

       Wb = w[j  ];      // Wb = w[j];
       Wa = w[j+1];      // Wa = w[2*j+1];
       Wc = w[j+2];      // Wc = cmult_r(Wa,Wb);
       j += 3;
       
       output[i  ] = A[0];
       output[i+1] = L_cmult32x32r( A[1], L_conjugate(Wa) );
       output[i+2] = L_cmult32x32r( A[2], L_conjugate(Wb) );
       output[i+3] = L_cmult32x32r( A[3], L_conjugate(Wc) );
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

               ifftButterflyRadix4Scaled_32x16( A );
               //ifftButterflyRadix4_32x16 ( A );

               output[(4*i + 0)*k1 + j] = A[0];
               output[(4*i + 1)*k1 + j] = L_cmult32x32r(A[1], L_conjugate(Wa));
               output[(4*i + 2)*k1 + j] = L_cmult32x32r(A[2], L_conjugate(Wb));
               output[(4*i + 3)*k1 + j] = L_cmult32x32r(A[3], L_conjugate(Wc));
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

          ifftButterflyRadix2Scaled_32x16( A );
          //ifftButterflyRadix2_32x16( A );

          output[i    ] = A[0];	
          output[i+N/2] = A[1];	
       }
    }
}

/***************************************************************/
/* Function: N-point unscaled real-valued FFT for 32-bit input */
/*           and 32-bit twiddle factors                        */
/***************************************************************/

void Ffecns_rfft_32x32(Word32 *input, int N, const CWord2x32 *Wt1, const CWord2x32 *Wt2, int16 fc, CWord2x32 *output) 
{
  int32 i;
  CWord2x32 X, Y;
  CWord2x32 *Z = output;

 /*************************************/
 /* Construct complex array with even */
 /* input as real and odd input as    */
 /* imaginary parts                   */
 /* Then do N/2-point complex FFT     */
 /*************************************/

  cmplxfft32x32((CWord2x32 *)input, N/2, Wt1, Z);

 /*************************************/
 /*  Calculate last stage butterflies */
 /**************************************/
  // calculate FFT at k=0, k=N/2
  X = L_complex(L_real(Z[0]), 0);
  Y = L_complex(L_imag(Z[0]), 0);

  output[0  ] = L_Vadd(X,Y);
  output[N/2] = L_Vsub(X,Y);

  {
     for (i=1; i<= N/4; i++) 
     {
        X =  L_Vavg(Z[i], L_conjugate(Z[N/2-i]));
        Y = L_Vnavg(Z[i], L_conjugate(Z[N/2-i]));
        Y = L_cmult32x32r(Y, Wt2[fc*i-1]);

        output[i    ] = L_Vsub(X, Y);
        //output[i    ] = L_Vnavg(X, Y);

        if (i!= N/4) {
           output[N/2-i] = L_conjugate(L_Vadd(X, Y));
        }
     }
  }
}


/***************************************************************/

/* Function: N-point scaled real-valued FFT for 32-bit input   */
/*           and 32-bit twiddle factors                        */
/***************************************************************/
/* - scaling factor = 1/N                                      */
/***************************************************************/
/* - requires 64-bit alignment of output                       */
/***************************************************************/
void Ffecns_rifft_32x32_scaled(CWord2x32 *input, int N, const CWord2x32 *Wt1, const CWord2x32 *Wt2, int16 fc, Word32 *output) 
{
  int32 i;
  CWord2x32 X, Y;
  CWord2x32 *Z = (CWord2x32 *) output;
  
 /*************************************/
 /*  Calculate last stage butterflies */
 /**************************************/
  // modify input in place
  // calculate IFFT at k=0, k=N/2

  input[0] = L_complex ( L_real( L_Vavg(input[0], input[N/2])),
                         L_real(L_Vnavg(input[0], input[N/2])) );

  {
     for (i=1; i<= N/4; i++)
     {
        X = L_Vavg(input[i], L_conjugate(input[N/2-i]));
        Y = L_Vnavg(input[i], L_conjugate(input[N/2-i]));
        Y = L_cmult32x32r(Y, L_conjugate(Wt2[fc*i-1]));

        input[i    ] = L_Vsub(X, Y);
        //input[i    ] = L_Vnavg(X, Y);

        if (i!= N/4)
        {
           input[N/2-i] = L_conjugate(L_Vadd(X, Y));
           //input[N/2-i] = L_conjugate(L_Vavg(X, Y));
        }
     }
  }

  /*************************************/
  /* Construct complex array with even */
  /* input as real and odd input as    */ 
  /* imaginary parts                   */
  /* Then do N/2-point complex FFT     */
  /*************************************/

  cmplxifft32x32_scaled(input, N/2, Wt1, Z); 
}
