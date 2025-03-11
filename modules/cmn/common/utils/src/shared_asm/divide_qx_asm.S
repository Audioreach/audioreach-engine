/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* ------------------------------------------------------------- */
/*  INCLUDES                                                     */
/* ------------------------------------------------------------- */

//#include "asm_macros.h"
#include "adsp_redef.h"
/* ------------------------------------------------------------- */
/*  CONSTANTS                                                    */
/* ------------------------------------------------------------- */

#define DSP_INV_LUT_BITS   4                             // number of bits to index the inverse 
                                                         // lookup table (actually 2^(val-1) segments) 
#define DSP_INV_LUT_SIZE  (1+(1<<(DSP_INV_LUT_BITS-1)))  // number of entries in the inverse LUT 

//#define DSP_SQRT_LUT_BITS  5
//#define DSP_INV_SQRT_LUT_BITS  5


#define DSP_SQRT_LUT_SIZE ( (1<<(DSP_SQRT_LUT_BITS-2))*3 + 1)
#define DSP_INV_SQRT_LUT_SIZE ( (1<<(DSP_INV_SQRT_LUT_BITS-2))*3 + 1)

#define DSP_LOG_LUT_BITS      5
#define DSP_LOG_LUT_SIZE      ((1<<DSP_LOG_LUT_BITS) + 1)

#define INVLOG2Q13               0x6a4d
#define EXP10C0                  0x7fbd
#define EXP10C1                  0x556f
#define EXP10C2                  0x15ef

/* ------------------------------------------------------------- */
/*  LOOK UP TABLES                                               */
/*                                                               */
/* this LUT has been generated using the matlab function         */
/* "find_optimal_lut_mse_cantoni.m" in                           */
/*  \\vivekv\Public\Jaguar\cdma1x\matlab. See the matlab file    */
/* for details of the mathematical analysis.                     */
/* ------------------------------------------------------------- */

	.data
	.p2align 6
	.type	log2table, @object
	.size	log2table, 66
log2table:
	.hword	30720,30766,30810,30852,30894,30935,30974,31012
	.hword	31050,31086,31122,31157,31191,31224,31256,31288
	.hword	31319,31350,31379,31409,31437,31465,31493,31520
	.hword	31547,31573,31599,31624,31649,31673,31697,31721
	.hword	31744



	.p2align 3
	.type	invSqRootLut, @object
	.size	invSqRootLut, 50
invSqRootLut:
	.hword	32738,30870,29290,27930,26743,25696,24762,23924
	.hword	23165,22474,21841,21259,20721,20222,19757,19323
	.hword	18917,18535,18175,17835,17514,17209,16920,16645
	.hword	16383



	.p2align 3
	.type	sqRootLut, @object
	.size	sqRootLut, 50
sqRootLut:
	.hword	16389,17382,18322,19215,20069,20888,21676,22437
	.hword	23172,23885,24578,25251,25907,26546,27171,27782
	.hword	28379,28964,29538,30100,30652,31195,31728,32253
	.hword	-32767



	.p2align DSP_INV_LUT_BITS
	.type	invTable, @object
	.size	invTable, 18
invTable:
	.hword	32690,29066,26171,23798,21820,20145,18709,17463
	.hword	16373



	.p2align 5
	.type	invTable1, @object
	.size	invTable1, 32
invTable1:
	.hword  3624,32690
	.hword  2895,29066
	.hword  2373,26171
	.hword  1978,23798
	.hword  1675,21820
	.hword  1436,20145
	.hword  1246,18709
	.hword  1090,17463



	.text
/*---------------------------------------------------------------------------------------------------   
 * dsplib_log10 ()
 *
 * DSP library function "log10". 
 * 
 * Returns 10*log10(x). 
 * Uses 10*log10(x)=3.0103*log2(x) approximation to calculate.
 * Uses lookup table for log2 with 7 MSB of input.
 *
 * Inputs:
 *     x              integer to find 10log10 of (> 0)
 *
 * Return:
 *     10log10(x) in 32Q23 unsigned.
 * 
 * Cycle-count:
 *--------------------------------------------------------------------------------------------------*/  
	.p2align 4
	.globl dsplib_log10
	.type	dsplib_log10, @function
dsplib_log10:
    {
        R3 = ADD(PC, ##log2table@PCREL)
    }
	{ P0 = CMP.GT(R0,#0)				//[ P0 = (x>0) ?                           ]
	  IF !P0.new R0 = #-1				//[ if(x<=0) return(-1)                    ]
	  R2 = NORMAMT(R0)				//[ expon = norm_l(x)                      ]
      //R3.H = #HI(log2table)               //[ set R3 = log2table                     ]
	}
	{ IF !P0 JUMPR R31				//[ if (x<=0) return -1                    ]
	  R0 = ASL(R0,R2)				//[ y= x <<  expon                         ]
      //R3.L = #LO(log2table)               //[ set R3 = log2table                     ]
	  R6 = #24660					//[ R6 = 0x6054                            ]
	}
	R3 = TABLEIDXH(R0,#5,#25)			//[ &log2table[mantis]                     ]	
	{ R3 = MEMH(R3)					//[ log2table[mantis]                      ]
	  R4 = MEMH(R3+#2)				//[ log2table[mantis+1]                    ]
	  R2 = ASL(R2,#10)				//[ expon << 10                            ]
	  R5.L = #0x8000
	}
	{ R3 = COMBINE(R3.L,R5.L)			//[ R3 = log2table[mantis]<<16 + 0x8000    ]
	  R4 = SUB(R4,R3)				//[ log2table[mantis+1]-log2table[mantis]  ]
	  R5 = ASR(R0,#9)				//[ R5.L = frac = (y>>9)&0xFFFF            ]
	  R0 = #0					//[ R0 = 0                                 ]
	}
	{ R3 += MPYU(R5.L,R4.L)				//[ interpolation                          ]
	  R0 -= MPY(R2.L,R6.L)				//[ R0 = -0x6045*expon                     ]
	}
	{ R0 += MPY(R3.H,R6.L)				//[ 0x6054*(logval>>16) - 0x6045*expon     ]
	  JUMPR R31					//[ return                                 ]
	}
	.size	dsplib_log10, .-dsplib_log10





/*---------------------------------------------------------------------------------------------------   
 * dsplib_approx_invert()
 *
 *     DSP library function "approx_invert".
 *     Approximates inversion:  out ~= 2^(31-floor(lg2(in)))/in
 *
 *     Approximation done with a linearly interpolated lookup table. With
 *     9 point entries (8 line segments) the maximum error is 0.238%. The 
 *     number to be inverted must be positive for valid results. If not 
 *     positive, then the lookup table is invalidly indexed.
 * 
 *     If input is Qi and the output is Qo, then
 *        Qo = 32 + (*shift_factor) - Qi.
 *
 * Inputs:
 *     input          integer to be inverted
 *
 * Return:
 *     int64 (shift_factor : result)
 *     where
 *      result        Q-shifted inverse
 *                    result ~= 2^(31-floor(lg2(input))) / input
 *                             = 2^(32+(*shift_factor)) / input
 *      shift_factor  (output_Q_factor - 32) of integer inverse
 *                    shift_factor = -1-floor(lg2(input))
 *
 *      if input <=0:  return 
 *        result       = 0xFFFFFFFF
 *        shift_factor = 0xFFFFFFFF
 *--------------------------------------------------------------------------------------------------*/  
	.p2align 4
	.globl dsplib_approx_invert
	.type	dsplib_approx_invert, @function
dsplib_approx_invert:
    {
        R2 = ADD(PC, ##invTable@PCREL)
    }
	{ R4 = NORMAMT(R0)				//[ r = norm_l(input)                      ]
	  R3 = R0					//[ save input into R3                     ]
	  P0 = CMP.GT(R0,#0)				//[ P0 = (input>0)?                        ]
      //  R2.H = #HI(invTable)              //[ set R2= invTable                       ]
	}						//[                                        ]
	{ IF !P0 JUMPR R31				//[ if(input<=0) return                    ]
	  R1:0 = COMBINE(#-1,#-1)			//[ result=-1, shift_factor=-1             ] 
	  R3 = ASL(R3,R4)				//[ norm_divisor                           ]
      // R2.L = #LO(invTable)               //[ set R2= invTable                       ]
	}						//[                                        ]
	{ R2 = TABLEIDXH(R3,#(DSP_INV_LUT_BITS-1),#(31-DSP_INV_LUT_BITS))
							//[ R2 = &invTable[index]                  ]
	  R3 = ASR(R3,#(15-DSP_INV_LUT_BITS))		//[ n1=norm_divisior>>(15-DSP_INV_LUT_BITS)]
	  R1 = ADD(R4,#-31)				//[ shift_fact = r-31                      ]
	}
	{ R4 = MEMH(R2)					//[ R4 = invTable[index]                   ]
	  R5 = MEMH(R2+#2)				//[ R5 = invTable[index+1]                 ]
	}						//[                                        ]
	{ R0 = ASLH(R4)					//[ invTable[index]<<16                    ]
	  R4 = SUB(R4,R5)				//[ invTable[index+1]-invTable[index]      ]
	}
	{ R0 -= MPYU(R3.L,R4.L)				//[ result = invTable[index]<<16 + interp* ] 
	  JUMPR R31					//[ (invTable[index+1]-invTable[index])    ] 
	}						//[ return                                 ]
	.size	dsplib_approx_invert, .-dsplib_approx_invert



/*---------------------------------------------------------------------------------------------------   
 * dsplib_approx_divide()
 *
 *     DSP library function "approx_divide".
 *     Approximates inversion:  out ~= numer*2^(31-floor(lg2(in)))/denom
 *
 *     Approximation done with a linearly interpolated lookup table. With
 *     9 point entries (8 line segments) the maximum error is 0.238%. The 
 *     number to be inverted must be positive for valid results. If not 
 *     positive, then the lookup table is invalidly indexed.
 * 
 *
 * Inputs:
 *     numer          integer numerator
 *     denom          integer denominator
 *
 * Return:
 *     int64 (shift_factor : result)
 *     where
 *      result        - quotient
 *                    result ~= numer*2^(31-floor(lg2(input))) / denom
 *                             = numer*2^(32+(*shift_factor)) / denom
 *      shift_factor  - Qfactor of (*result)
 *
 * 
 * Notes : (*result << *shift_factor) will be floating point result in Q0
 *
 *--------------------------------------------------------------------------------------------------*/  
	.p2align 4
	.global dsplib_approx_divide
	.type	dsplib_approx_divide, @function
dsplib_approx_divide:
	{ P0 = CMP.GT(R1,#0)				//[ P0 = (denom>0)?                        ]
	  IF !P0.new JUMP:nt dsplib_approx_divide_error	//[ if(denom<=0) return                    ]
	  //R2.H = #HI(invTable)				//[ inv: R2= invTable                      ]
	  R3 = NORMAMT(R1)				//[ inv: r = norm_l(denom)                 ]
	}						//[                                        ]
	{ R1 = ASL(R1,R3)				//[ inv: R1= norm_divisor                  ]
	  //R2.L = #LO(invTable)				//[ inv: R2= invTable                      ]
      R2 = ADD(PC, ##invTable@PCREL)
	  R3 = ADD(R3,#-30)				//[ inv: s_d = r-31+1                      ]
	}						//[                                        ]
	R2 = TABLEIDXH(R1,#(DSP_INV_LUT_BITS-1),#(31-DSP_INV_LUT_BITS))
							//[ inv: R2 = &invTable[index]             ]
	{ R1 = ASR(R1,#(15-DSP_INV_LUT_BITS))		//[ inv: n1= norm_divisor>>(15-.._LUT_BITS)]
	  R4.L = #0x8000				//[ rounding factor for r	
	  R5 = MEMH(R2)					//[ inv: R5 = invTable[index]              ]
	  R6 = MEMH(R2+#2)				//[ inv: R6 = invTable[index+1]            ]
	}						//[                                        ]
	{ R4 = COMBINE(R5.L,R4.L)			//[ inv: invTable[index]<<16               ]
	  R5 = SUB(R5,R6)				//[ inv: invTable[index+1]-invTable[index] ]
	  R2 = NORMAMT(R0)				//[ s_n = norm_l(numer)                    ]
	}						//[                                        ]
	{ R4 -= MPYU(R1.L,R5.L)				//[ inv: r = invert(denom)+0x8000          ]
	  R0 = ASL(R0,R2)				//[ norm_num=L_shl(numer,s_n)              ]
	}						//[                                        ]
	{ R0 = MPY(R0,R4.H):<<1:rnd:sat			//[ R0 = result =L_mult(norm_num,rnd(r))   ]
	  R1 = SUB(R3,R2)				//[ return R1 = shift_factor = s_d - s_n   ]
	  JUMPR R31					//[ return                                 ] 
	}

dsplib_approx_divide_error:	
	{ R1:0 = COMBINE(#-1,#-1)			//[ result=-1, shift_factor=-1             ] 
	  JUMPR R31
	}
	.size	dsplib_approx_divide, .-dsplib_approx_divide



/*---------------------------------------------------------------------------------------------------   
 * dsplib_sqrt_lut()
 *
 *     DSP library function "sq_root_lut".
 *     Approximates inversion:  out=sqrt(in)
 *
 *
 * Inputs:
 *     input          integer input number in Q0 ( > 0 )
 *     round_factor   integer rounding factor, any bit location
 *
 * Return:
 *     result        sqrt(input), in Q16
 * 
 * Cycle-count:  8
 * 
 *--------------------------------------------------------------------------------------------------*/  
	.p2align 4
	.globl dsplib_sqrt_lut
	.type	dsplib_sqrt_lut, @function
dsplib_sqrt_lut:
    {
        R5 = ADD(PC, ##sqRootLut@PCREL)
    }
	{ P0 = CMP.GT(R0,#0)				//[ P0 = (input>0)?                       ]
	  IF !P0.new JUMP:nt .dsplib_sqrtlut_excpt		 		
                                    //[ if (input<=0) return (-1 or 0)         ]
	  R2 = NORMAMT(R0)				//[ r2 = norm_l(input)                     ]
      R5 = add(R5, #-16)
      //R5.H = #HI(sqRootLut-16)            //[ set R9 = sqRootLut - 8                 ]
	}
	{ P1 = TSTBIT(R2,#0)				//[ P1 = !(r2&1)                           ]
	  IF  P1.new R4 = ADD(R2,#-10)			//[ if(r2&1) R4 = r2-10                    ]
	  IF !P1.new R4 = ADD(R2,#-11)			//[ else R4 = r2-11                        ]
	}
	{ R0 = ASL(R0,R4)				//[ R0 = n1                                ]
	  R2 = NEG(R2)					//[ -r2                                    ]
	  R3 = #2					//[ used for address                       ]
      //R5.L = #LO(sqRootLut-16)            //[ set R9 = sqRootLut-8                   ]
	}
	R5 += MPYU(R0.H,R3.L)				//[ &sqRootLut[index]                      ]
	{ R3 = MEMUH(R5)				//[ sqRootLut[index]                       ]
	  R4 = MEMUH(R5+#2)				//[ sqRootLut[index+1]                     ]
	  R2 = ASR(R2,#1)				//[ r = (-r2)>>1                           ]
	}
	{ R3 = ASLH(R3)					//[ sqRootLut[index]<<16                   ]
	  R4 = SUB(R4,R3)				//[ sqRootLut[index+1]-sqRootLut[index]    ]
	  R2 = ADD(R2,#1)				//[ R2 = r+1                               ]
	}
	{ R3 += MPYU(R0.L,R4.L)				//[ interpolation and R3= val              ]
	  R0 = R1					//[ R0 = round_factor                      ]
	}
	{ R0 += ASL(R3,R2)				//[ result = val << (r+1) + round_factor   ] 
	  JUMPR R31					//[ return                                 ]
	}

.dsplib_sqrtlut_excpt:
	{ P0 = CMP.EQ(R0,#0)		 		//[ if (input<0) return -1                 ]
	  IF !P0.new R0 = #-1				//[ if (input==0) return 0                 ]
	  JUMPR R31					//[ return                                 ]
    }
	.size	dsplib_sqrt_lut, .-dsplib_sqrt_lut




/*---------------------------------------------------------------------------------------------------   
 * dsplib_inv_sqrt_lut()
 *
 *     DSP library function "inv_square_root_lut".
 *     Approximates inversion:  out=1/sqrt(in)
 *
 *
 * Inputs:
 *     input          integer input number in Q0 ( > 0 )
 *     round_factor   integer rounding factor, any bit location
 *
 * Return:
 *     result        1/sqrt(input), in Q30
 * 
 * Cycle-count: 8
 *--------------------------------------------------------------------------------------------------*/  
	.p2align 4
	.globl dsplib_inv_sqrt_lut
	.type	dsplib_inv_sqrt_lut, @function
dsplib_inv_sqrt_lut:
    {
        R5 = ADD(PC, ##invSqRootLut@PCREL)
    }
	{ P0 = CMP.GT(R0,#0)				//[ P0 = (input> 0)?                       ]
	  IF !P0.new R0 = #-1				//[ if (input<=0) return -1                 ]
	  R2 = NORMAMT(R0)				//[ r2 = norm_l(input)                     ]
      R5 = add(R5, #-16)
      //R5.H = #HI(invSqRootLut-16)         //[ set R9 = invSqRootLut - 8              ]
	}
	{ IF !P0 JUMPR R31		 		//[ if (input<0) return (-1)               ]
	  P1 = TSTBIT(R2,#0)				//[ P1 = !(r2&1)                           ]
	  IF  P1.new R4 = ADD(R2,#-10)			//[ if(r2&1) R4 = r2-10                    ]
	  IF !P1.new R4 = ADD(R2,#-11)			//[ else R4 = r2-11                        ]
	}
	{ R0 = ASL(R0,R4)				//[ R0 = n1                                ]
	  R2 = NEG(R2)					//[ -r2                                    ]
	  R3 = #2					//[ used for address                       ]
      //R5.L = #LO(invSqRootLut-16)         //[ set R9 = invSqRootLut-8                ]
	}
	R5 += MPYU(R0.H,R3.L)				//[ &sqRootLut[index]                      ]
	{ R3 = MEMUH(R5)				//[ sqRootLut[index]                       ]
	  R4 = MEMUH(R5+#2)				//[ sqRootLut[index+1]                     ]
	  R2 = ASR(R2,#1)				//[ r = (-r2)>>1                           ]
	}
	{ R3 = ASLH(R3)					//[ sqRootLut[index]<<16                   ]
	  R4 = SUB(R3,R4)				//[ sqRootLut[index]-sqRootLut[index+1]    ]
	  R2 = ADD(R2,#16)				//[ r += 16                                ]
	}
	{ R3 -= MPYU(R0.L,R4.L)				//[ interpolation and R3= val              ]
	  R0 = R1					//[ R0 = round_factor                      ]
	}
	{ R0 += ASR(R3,R2)				//[ result = (val >> r) + round_factor     ] 
	  JUMPR R31					//[ return                                 ]
	}
	.size	dsplib_inv_sqrt_lut, .-dsplib_inv_sqrt_lut




/*---------------------------------------------------------------------------------------------------   
 *  dsplib_exp10()
 *  
 *            Calculate 10^x where x is in the range [-1,1].
 *  Input:
 *       x    int32 input, Q26
 *  Return:
 *            10^x  in Q15
 *
 *  Cycle-count: 8
 *--------------------------------------------------------------------------------------------------*/  
	.p2align 4
	.globl dsplib_exp10
	.type	dsplib_exp10, @function
dsplib_exp10:
	{ R1 = #INVLOG2Q13				//[ INVLOG2Q13                             ]
	  R5 = #EXP10C1					//[ EXP10C1                                ]
	  R6 = #EXP10C2					//[ EXP10C2                                ]
	  R7 = #-15					//[ set R7 = -15                           ]
	}
	{ R3:2 = MPY(R0,R1)				//[ x * INVLOG2Q13                         ]
	  R0.H = #EXP10C0				//[ R1:0 = EXP10C0 << 16                   ]
	  R1 = #0					//[ R1:0 = EXP10C0 << 16                   ]
	}
	{ R3:2 = ASR(R3:2,#23)				//[ acc = (x*INVLOG2Q13 >> (16+7)          ]
	  R0.L = #0					//[ R1:0 = EXP10C0 << 16                   ]
	}
	{ R4 = ADD(R7.L,R2.H)				//[ sh  = (acc>>16) - 15                   ]
	  R2 = NOT(R2)					//[ inv = ~acc                             ]
	}						//[ &0xFFFF is merged with MPYU            ]
	R3 = MPYU(R2.L,R2.L)				//[ (uint16)inv * (uint16)inv              ]
	R1:0 -= MPYU(R5.L,R2.L)				//[ acc = (EXP10C0<<16) - EXP10C1*inv      ]
	R1:0 += MPYU(R6.L,R3.H)				//[ acc += EXP10C2*(inv*inv >> 16)         ]
	{ R1:0 = ASL(R1:0,R4)				//[ D_shl(acc, sh)                         ]
	  JUMPR R31					//[ return                                 ]
	}
	.size	dsplib_exp10, .-dsplib_exp10





/*---------------------------------------------------------------------------------------------------   
 *  dsplib_newton_invert()
 *  
 *            Inverts a 32-bit positive number using Newton's method
 *  Input:
 *       input    int32 input to be inverted, Q0
 *
 *  Return:
 *       int64 (sr: result)
 *       where
 *         result   Inverted rounded result in Q(45-sr)
 *                  0x80000000 for input <= 0
 *         sr       int32, shift normalization factor
 *
 *  Cycle-count: 18
 *--------------------------------------------------------------------------------------------------*/  
	.p2align 4
	.globl dsplib_newtons_invert
	.type	dsplib_newtons_invert, @function
dsplib_newtons_invert:
	{ P0 = CMP.GT(R0,#0)				//[ P0 =(input>0)                          ]
	  IF !P0.new JUMP:nt dsplib_newtonsInv_LE0	//[ if (input<=0) jump                     ]
	  R5 = NORMAMT(R0)				//[ R5 = sf = norm_l(input)                ] 
	  R4.H = #0x4000				//[ R4 = 0x40008000                        ]
	}						//[                                        ]
	{ R2 = ASL(R0,R5)				//[ R2 = num = L_shl(input,sf)             ]
	  R0 = #0x5555					//[ x =0x5555                              ]
	  R4.L = #0x8000				//[ R4 = 0x40008000                        ]
	  LOOP0(dsplib_NewtonsInv_LOOP,#5)		//[ setup loop                             ]
	}

	.falign
dsplib_NewtonsInv_LOOP:					//[ loop start                             ]
	R4 -= MPY(R0.L,R2.H)				//[ R4 = 0x40008000 - x*num                ]
	{ R3 = MPY(R0.L,R4.H)				//[ R3 = ((0x40008000-x*num)>>16)*x        ]
	  R4.H = #0x4000				//[ set R4 = 0x40008000                    ]
	}
	{ R0 = ASR(R3,#12):rnd				//[ R0 = x = (R3<<3 + 0x8000) >> 16        ] 
	  R4.L = #0x8000				//[ set R4 = 0x40008000                    ]
	}:endloop0					//[ loop end                               ]

	{ R1 = R5					//[ return value                           ]
	  JUMPR R31					//[ return                                 ]
	}

dsplib_newtonsInv_LE0:
	{ R1:0 = COMBINE(#-1,#-1)			//[ R1:0 = -1 : -1                         ]
	  JUMPR R31					//[ return                                 ]
	}
	.size	dsplib_newtons_invert, .-dsplib_newtons_invert



/*-------------------------------------------------------------------------------------------------
 *  dsplib_invert()
 *
 *     DSP library function "dsplib_invert".
 *     Calculate inversion:  1/in
 *
 *  Inputs:
 *     Word16 input         integer to be inverted
 *
 *  Return:
 *     Word64 (shift_factor : result)
 *
 *      if input <=0:  return 
 *        result       = 0xFFFFFFFF
 *        shift_factor = 0xFFFFFFFF
 *
 *      - result has 16-bit precision in LSB
 *        
 *      - Decimal-point:
 *        If input is in Qi and the output is Qo, then
 *         o = (*shift_factor) -i
 *
 *        For example, input is 0x0F00 in Q10, i.e., x= 3.75 
 *        the function returns 
 *        result = 17476, shift_factor is 26, and thus the 
 *        actual result is in Q(26-10), i.e., inversion is
 *        17476/(2^16) = 0.26666260
 *      
 *  Cycle-count: 18
 *----------------------------------------------------------------------------------------------*/  

	.p2align 5
	.globl dsplib_invert
	.type	dsplib_invert, @function
dsplib_invert:
	{ R2 = #0x4000
	  R3 = ASLH(R0)					//[ L_denom = input<<16                    ]
	  R4.H = #0x8000				//[ L_num = 0x80000000                     ]
	  R0 = #0					//[ initialize result=0                    ]
	}
	{ P0 = CMP.GT(R3,#0)				//[ P0 = (input >0)                        ]
	  IF !P0.new JUMP:nt dsplib_invert_LE0		//[ if (input<=0) jump invert_LE0          ] 
	  R4.L = #0					//[ L_num = 0x80000000                     ]
	  R5 = NORMAMT(R3)				//[ nshft = norm_l(L_denom)                ]
	}
	{ R3 = ASL(R3,R5)				//[ L_denom <<= nshft                      ]
	  R1 = SUB(#29,R5)				//[ nshft = 29 - nshft                     ]
	  LOOP0(dsplib_invert_LOOP,#14)			//[ setup loop (unroll once)               ]
	}

	.falign
dsplib_invert_LOOP:
	{ P0 = CMP.GTU(R3,R4)				//[ if (L_num>= L_denom)                   ]
	  IF !P0.new R4 = SUB(R4,R3)			//[  then L_num -= L_denom                 ]
	  IF !P0.new R0 = ADD(R0,R2)			//[  then  result++                        ]
	  R3:2 = VLSRW(R3:2,#1)				//[ equivalent to L_denom <<1, result <<1  ]
	}:endloop0

	{ P0 = CMP.GTU(R3,R4)				//[ if (L_num>= L_denom)                   ]
	  IF !P0.new R0 = ADD(R0,R2)			//[  then  result++                        ]
	  JUMPR R31
	}

dsplib_invert_LE0:
	{ R1:0 = COMBINE(#-1,#-1)			//[ R1:0 = -1 : -1                         ]
	  JUMPR R31					//[ return                                 ]
	}
	.size	dsplib_invert, .-dsplib_invert



/*---------------------------------------------------------------------------------------------------   
 * dsplib_approx_invert_simple()
 *
 *     DSP library function "approx_invert".
 *     Approximates inversion:  out ~= 2^(31-floor(lg2(in)))/in
 *
 *     Approximation done with a linearly interpolated lookup table. With
 *     9 point entries (8 line segments) the maximum error is 0.238%. The 
 *     number to be inverted must be positive for valid results. If not 
 *     positive, then the lookup table is invalidly indexed.
 * 
 *     If input is Qi and the output is Qo, then
 *        Qo = 32 + (*shift_factor) - Qi.
 *
 * Inputs:
 *     input          integer to be inverted
 *
 * Return:
 *     int64 (shift_factor : result)
 *     where
 *      result        Q-shifted inverse
 *                    result ~= 2^(31-floor(lg2(input))) / input
 *                             = 2^(32+(*shift_factor)) / input
 *      shift_factor  (output_Q_factor - 32) of integer inverse
 *                    shift_factor = -1-floor(lg2(input))
 *
 *      if input <=0:  return 
 *        result       = 0xFFFFFFFF
 *        shift_factor = 0xFFFFFFFF
 *--------------------------------------------------------------------------------------------------*/  
	.p2align 4
	.globl dsplib_approx_invert_simple
	.type	dsplib_approx_invert_simple, @function
dsplib_approx_invert_simple:
    { R2 = ADD(PC, ##invTable1@PCREL)
    }
	{ R4 = NORMAMT(R0)				//[ r = norm_l(input)                      ]
	  R3 = R0					//[ save input into R3                     ]
	  P0 = CMP.GT(R0,#0)				//[ P0 = (input>0)?                        ]
    //  R2.H = #HI(invTable1)               //[ set R2= invTable1                      ]
	}						//[                                        ]
	{ IF !P0 JUMPR R31				//[ if(input<=0) return                    ]
	  R1:0 = COMBINE(#-1,#-1)			//[ result=-1, shift_factor=-1             ] 
	  R3 = ASL(R3,R4)				//[ norm_divisor                           ]
    //  R2.L = #LO(invTable1)               //[ set R2= invTable                       ]
	}						//[                                        ]
	{ R3 = ASR(R3,#(15-DSP_INV_LUT_BITS))		//[ n1=norm_divisior>>(15-DSP_INV_LUT_BITS)]
	  R1 = ADD(R4,#-31)				//[ shift_fact = r-31                      ]
	  R2 = TABLEIDXW(R3,#(DSP_INV_LUT_BITS-1),#(31-DSP_INV_LUT_BITS))
	}						//[ R2 = &invTable[index]                  ]
	R0 = MEMW(R2)					//[ R0 = invTable1[index]                  ]
	{ R0 -= MPYU(R3.L,R0.L)				//[ result = invTable1[index] - interp*    ] 
	  JUMPR R31					//[ (invTable1[index]*0x0FFFF)             ] 
	}						//[ return                                 ]
	.size	dsplib_approx_invert_simple, .-dsplib_approx_invert_simple


/*======================================================================*/
/*  FUNCTION  audio_divide_sp                                           */
/*======================================================================*/
/*  DESCRIPTIONS                                                        */
/*  ============                                                        */
/*  Single precision divide                                             */
/*  INPUTS                                                              */
/*  ============                                                        */
/*  R0: int32 Lnum                                                      */
/*  R1: int32 Ldenom                                                    */
/*  R2: Word16 Qadj                                                     */
/*  OUTPUTS                                                             */
/*  R0: Output int32 Lquotient in Q(31-Qadj)                            */
/*  Registers Affected:                                                 */
/*  Hardware Loops affected :                                           */
/*  Assumptions:None                                                    */
/* Stack Memory Frame Allocated (in Bytes): 0                           */
/*  Cycle Count:                                                        */
/*            Author         Date          Comments                     */
/*           --------       ------        ----------                    */
/*           danielp       12/13/2010      Creation                     */
/*           katitkar      04/19/2011     Reduction of inner loop to    */
/*                                        3 cycles per changes suggested*/
/*					  by cpoliset                   */
/*           katitkar      04/22/2011     Change inputs to 32 bits      */
/*======================================================================*/
/* **********************************************************************/
/*======================================================================*/
/*                                                                      */
/*======================================================================*/
   .p2align 4
   .globl audio_divide_sp
   .type audio_divide_sp, @function
audio_divide_sp:
   {
      R14 = SUB(#0, R1);                // -Ldenom
      R9   = NORMAMT(R0);                  // norm_l(Lnum)
      P3 = CMP.GT(R1, #-1);               // test if Ldenom negative
      R4 = ADD(R2,#-1);                  // Qadj - 1
   }
   {
      R1 = MUX(P3,R1,R14);       // if( Ldenom < 0) Ldenom = -Ldenom
      R4 = ADD(R9,R4);                   // Qadj = nnum + (Qadj - 1)
      R0 = ASL(R0,R9);               // Lnum <<= n1
   }
   {
      P1   = CMP.EQ(R1,#0);          // Ldenom == 0 test
      R8   = NORMAMT(R1);                  // norm_l(Ldenom)
      P2 = CMP.GT(R0, #-1);                // test if Lnum negative
      R5 = SUB(#0, R0);             //negative num, to use for multiplication in case of -ve denom
   }
   {
      if( P1) JUMP .Ldivide_dp_fix_denom0
      R14 = #0;
      R4   = SUB(R8,R4);                 // Qadj = ndenom - Qadj
      R1 = ASL(R1,R8);               // Ldenom << ndenom
   }
   {
      LOOP0(.Ldivide_dp_loop,#5)
#if (__QDSP6_ARCH__ > 3)
      R15 = ##0x40000000;        // setup R15:14 = 2.0
#else
      R15.L = #0;
#endif
      R0 = MUX(P3, R0, R5);
   }
   {
#if (__QDSP6_ARCH__ <= 3)
      R15.H = #0x4000;
#endif
      R12  = ##(0x55555555);       // initial guess
	}

.falign
.Ldivide_dp_loop:
   {
      R15:14 -= MPY(R12,R1);                  // (Ltemp*(int32) Ldenom) >> 32...multiply and use upper (implicit shift by 32)
   }
   {
      R13:12 = MPY(R12,R15);              // Ltemp*Lacc
      R15.L = #0;
   }
   {
      R15.H = #0x4000;
      R13:12 = ASR(R13:12,#29);           // finish l_mult_s32xs32_shift(Ltemp3, (int32)Lacc, 3);
      R14 = #0;
   }:endloop0

   {
      R13:12 = MPY(R12,R0);               // Ltemp3*(int32)Lnum (num negated if den<0)
      R4 = ADD(R4,#-31);                  // combine ASL of -31 and Qadj
      P0 = CMP.GT(R4, #-33);
      if (!P0.new) JUMP:nt .Ldivide_dp_large_shift;
   }
   {
      R13:12 = ASL(R13:12,R4);
   }
   {
      R0 = SAT(R13:12);
      JUMPR R31;
   }
.falign
.Ldivide_dp_fix_denom0:
   {
      R0 = ##(0x80000000);
      R1 = ##(0x7FFFFFFF);
   }
   {
      R12  = ##(0x55555555);       // initial guess   
      R0 = MUX(P2,R1,R0);                 // max positive value if Lnum >= 0, max negative value otherwise
      JUMPR R31;
   }
.falign
.Ldivide_dp_large_shift:
   {
      R9:8 = COMBINE(#0, #0);
   }
   {
      P0 = CMP.GT(R9:8, R13:12);
   }
   {
      R0 = MUX(P0, #-1, #0);
      JUMPR R31;
   }
.size audio_divide_sp, .-audio_divide_sp
