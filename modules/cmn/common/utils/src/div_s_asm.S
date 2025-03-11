#if ((defined __hexagon__) || (defined __qdsp6__))

/**************************************************************/
/* div_s      : Fractional Integer division of dividend by    */
/*              divisor, where dividend is positive, and      */
/*          divisor is positive and greater than the dividend.*/
/*             The result is positive and truncated to 16-bits*/
/* Inputs     : R0 : dividend (i) 16 bits                     */
/*              R1 : divisor (i) 16 bits                      */
/* Outputs    : R0 the result of the division truncated to    */
/*              16-bits                                       */
/* Registers Affected: R0-R6                                  */
/* Hardware Loops affected: Loop0                             */
/* Assumptions: dividend > 0,divisor>0 and dividend < divisor */
/* Stack Memory Frame Allocated (in Bytes): 0                 */
/* Bit Exact Tested with                                      */
/* Cycle Count: 53 cycles (worst case)                        */
/**************************************************************/
 
#endif
