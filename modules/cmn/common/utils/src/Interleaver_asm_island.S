/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 */

	.TEXT	
	.p2align 4
	.globl interleaver_16
	.type	interleaver_16, @function

interleaver_16:
        //[ R0 = inBuf_L                ]
        //[ R1 = inBuf_R                ]
        //[ R2 = outBuf                 ]
        //[ R3 = numSamplesToInterleave ]
    
	P0 = cmp.eq(R3, #0)           //[ is R3 0? ]
	if (P0) JUMPR R31             //[ if so, return ]

    { R5 = memh(R0++#2)             //[ R5 = *inBuf_L++ ]
	  R6 = memh(R1++#2)             //[ R6 = *inBuf_R++ ]
	  R3 = add(R3, #-1)             //[ Loop for (numSamplesToInterleave - 1) times ]
	}

	P0 = cmp.eq(R3, #0)
	if (P0) jump .last_copy_intl

	LOOP0(.intl_16_start, R3)
		
	.falign
.intl_16_start:
	{ memh(R2++#2) = R5             //[ *outBuf++ = R5  ]
	  R5 = memh(R0++#2)             //[ R5 = *inBuf_L++ ]
	}
	
	{ memh(R2++#2) = R6             //[ *outBuf++ = R6  ]
	  R6 = memh(R1++#2)             //[ R6 = *inBuf_R++ ]
	} :endloop0

.last_copy_intl:
    memh(R2++#2) = R5
	memh(R2) = R6
	
	JUMPR R31                       //[ return          ]
	
	.size	interleaver_16, .-interleaver_16
		
		
		

	.TEXT	
	.p2align 4
	.globl deinterleaver_16
	.type	deinterleaver_16, @function

deinterleaver_16:
        //[ R0 = inBuf                    ]
        //[ R1 = outBuf_L                 ]
        //[ R2 = outBuf_R                 ]
        //[ R3 = numSamplesToDeInterleave ]

	P0 = cmp.eq(R3, #0)           //[ is R3 0? ]
	if (P0) JUMPR R31             //[ if so, return ]
        
    R5 = memh(R0++#2)               //[ R5 = *inBuf++    ]
	{ R6 = memh(R0++#2)             //[ R6 = *inBuf++    ]
	  R3 = add(R3, #-1)             //[ Loop for (numSamplesToDeInterleave - 1) times ]
	}

	P0 = cmp.eq(R3, #0)
	if (P0) jump .last_copy_deintl

	LOOP0(.deintl_16_start, R3)
			
	.falign
.deintl_16_start:
	{ memh(R1++#2) = R5             //[ *outBuf_L++ = R5 ]
	  R5 = memh(R0++#2)             //[ R5 = *inBuf++    ]
	}
	
	{ memh(R2++#2) = R6             //[ *outBuf_R++ = R6 ]
	  R6 = memh(R0++#2)             //[ R6 = *inBuf++    ]
	} :endloop0
	
.last_copy_deintl:
    memh(R1) = R5
	memh(R2) = R6

	JUMPR R31                       //[ return           ]
	
	.size	deinterleaver_16, .-deinterleaver_16


/**************************************************************/
/* Function:    align_interleaver_16                          */
/**************************************************************/
/* Description: Interleaver for 16 bit stereo using vector    */
/*                instructions                                */
/* Reg Inputs:  R0 = inBuf_L, must be 32 bit aligned          */
/*              R1 = inBuf_R, must be 32 bit aligned          */
/*              R2 = outBuf , must be 64 bit aligned          */
/*              R3 = Num of smaples to interleave             */
/* Mem Inputs:  Input samples,                                */
/* Mem Outputs: Output samples,                               */
/**************************************************************/	
	
	
	.TEXT	
	.p2align 4
	.globl align_interleaver_16
	.type  align_interleaver_16, @function
	
	align_interleaver_16:
		//[ R0 = inBuf_L                ]
		//[ R1 = inBuf_R                ]
		//[ R2 = outBuf                 ]
		//[ R3 = numSamplesToInterleave ]
		{
		 
		  R5 = memw(R0++#4)             //[ R5 = *inBuf_L++ ] load two 16 bit samples at a time
		  R6 = memw(R1++#4)             //[ R6 = *inBuf_R++ ] load two 16 bit samples at a time
		  R3 = asr(R3,#1)               //Loop for numSamplesToInterleave/2
		  R4 = and(R3,#0x1)             //check numSamplesToInterleave are odd or even
		}					

		{     
		  P1 = cmp.eq(R4,#0x1)          //check numSamplesToInterleave are odd or even		  
		  R3 = add(R3, #-1)             //[  Loop for (numSamplesToInterleave/2 - 1) times ]		  
		}

		{
		P0 = cmp.eq(R3, #0)	     	   //checking loopcount is zero or not
		LOOP0(.align_intl_16_start, R3)    // set loop
		}
		
		if (P0) jump .align_last_copy_intl			
		

		.falign
		.align_intl_16_start:
		
		{
		  R9:8 = packhl(R6,R5)           //this will store like as R6[0] R5[0] R6[1] R5[1]
		  R5 = memw(R0++#4)             //[ R5 =read next two samples  from inBuf_L]
		  R6 = memw(R1++#4)             //[ R5 =read next two samples  from inBuf_R]
		}
		
		{
		  memd(R2++#8) = R9:8            //store 4 interleaved samples in outBuf
     	        }:endloop0

		.align_last_copy_intl:
                {
		 R9:8 = packhl(R6,R5)		 
		}
		
#ifndef FEATURE_POPQUEUE_CONFLICT
		{
		 memd(R2++#8) = R9:8
		 if(!P1) JUMPR R31      //if even return
		}
#else
		{
		 memd(R2++#8) = R9:8
		
}
{
		 if(!P1) JUMPR R31      //if even return
}
#endif
		 
		 
		{
		  R5 = memh(R0++#2)       //odd samples processing
		  R6 = memh(R1++#2)          
		} 	 
	 	
	 	memh(R2++#2) = R5 
#ifndef FEATURE_POPQUEUE_CONFLICT
	 	{
	 	memh(R2++#2) = R6 	 	 
		JUMPR R31                       //[ return          ]
		}      
#else
	 	{
	 	memh(R2++#2) = R6 	 	 
		
}
{
		JUMPR R31                       //[ return          ]
}
#endif
		.size	align_interleaver_16, .-align_interleaver_16



/**************************************************************/
/* Function:    align_deinterleaver_16                          */
/**************************************************************/
/* Description: DeInterleaver for 16 bit stereo using vector  */
/*                instructions                                */
/* Reg Inputs:  R0 = inBuf, must be 64 bit aligned            */
/*              R1 = outBuf_L, must be 32 bit aligned         */
/*              R2 = outBuf_R , must be 32 bit aligned        */
/*              R3 = Num of smaples to interleave             */
/* Mem Inputs:  Input samples,                                */
/* Mem Outputs: Output samples,                               */
/**************************************************************/		

	.TEXT	
	.p2align 4
	.globl align_deinterleaver_16
	.type	align_deinterleaver_16, @function

align_deinterleaver_16:
        //[ R0 = inBuf                    ]
        //[ R1 = outBuf_L                 ]
        //[ R2 = outBuf_R                 ]
        //[ R3 = numSamplesToDeInterleave ]

	
	{
	 
	  R7:6 = memd(R0++#8)           //[ R5 = *inBuf++ ,load 4 samples at a time   ]
	  R3 = asr(R3,#1)
	  R4  = and(R3,#0x1)        //check numSamplesToInterleave are odd or even
	}
	
		
        {	
          P1 = cmp.eq(R4,#0x1)	    	 //checking if samples are odd or even 
	  R3 = add(R3, #-1)	  
	  R9:8 = packhl(R7,R6)          
	}
	
	
	{
	 P0 = cmp.eq(R3, #0)	     	
	 LOOP0(.align_deintl_16_start, R3)
	}	
	
	if (P0) jump .align_last_copy_deintl
	
			
	.falign
.align_deintl_16_start:
	{ 
	  memw(R1++#4) = R8             //[ *outBuf_L++ = R5 ]
	  R7:6 = memd(R0++#8)            //[ R5 = *inBuf++    ]
	}	
	{ 
	  memw(R2++#4) = R9             //[ *outBuf_R++ = R6 ]
	  R9:8 = packhl(R7,R6)	  
	}:endloop0
	
.align_last_copy_deintl:
        
        memw(R1++#4) = R8
        
#ifndef FEATURE_POPQUEUE_CONFLICT
        {
         memw(R2++#4) = R9
         if(!P1)  JUMPR R31          //if even return
        }
#else
        {
         memw(R2++#4) = R9
        
}
{
         if(!P1)  JUMPR R31          //if even return
}
#endif
        
        
        R8 = memh(R0++#2)          //processing remaining samples for odd case.
        
        {
         R9 = memh(R0++#2)
         memh(R1++#2) = R8 
        }
        
#ifndef FEATURE_POPQUEUE_CONFLICT
        {
         memh(R2++#2) = R9
         JUMPR R31                       //[ return           ]
        }
#else
        {
         memh(R2++#2) = R9
        
}
{
         JUMPR R31                       //[ return           ]
}
#endif
         
        
	.size	align_deinterleaver_16, .-align_deinterleaver_16
	
	

	
	
