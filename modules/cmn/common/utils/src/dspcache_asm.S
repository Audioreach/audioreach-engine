/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 */

    /*================================================================================================*/
    /*  ARCHITECTURE : QDSP6V4                                                                        */
    /*================================================================================================*/
   .file    "dspcache_asm.S"

    /*================================================================================================*/
    /*  FILE LEVEL DEFINITIONS                                                                        */
    /*================================================================================================*/

    /*================================================================================================*/
    /*  FUNCTION      : dspcache_linear_l2fetch                                                       */
    /*================================================================================================*/
    /*  CYCLE-COUNT   : 4 if request < ~64KB, 6 if > ~64KB.                                           */
    /*================================================================================================*/
    /*  PARAMETERS    :                                                                               */
    /*  ============                                                                                  */
    /*   R0:  addr                                                                                    */
    /*   R1:  bytes                                                                                   */
    /*================================================================================================*/
    /* IMPLEMENTATION SUMMARY   :                                                                     */
    /* ======================                                                                         */
    /*   Issues L2FETCH for up to 64KB. Divides bytes by 256, uses 256 for stride and 255 for width   */
    /*   (effectively resulting in width = 256), and bytes/256 + 1 for height. No alignment           */
    /*   is required, but the addr must be a validly mapped virtual address.                          */
    /*                                                                                                */
    /*================================================================================================*/

   .text
   .p2align 4
   .globl dspcache_linear_l2fetch
   .type dspcache_linear_l2fetch, @function
dspcache_linear_l2fetch:
   {
     R1 = ASR(R1,#8)                   // bytes/256
     R2 = ##0x0100FF01                 // inital L2FETCH config [stride = 256 : width = 255 : height = 1]
     R3 = #254
   }{
     P0 = CMP.GT(R1,#254);             // (bytes > ~64KB)
     IF (P0.NEW) R2 = ADD(R2,R3)       // Cap height to 255
     IF (!P0.NEW) R2 = ADD(R2,R1)      // L2FETCH config [stride = 256 : width = 255 : height = bytes/256 + 1]
   }{
     L2FETCH(R0,R2)                    // fetch
   }{
     JUMPR R31                         // return
   }
   .size  dspcache_linear_l2fetch, .-dspcache_linear_l2fetch

    /*================================================================================================*/
    /*  FUNCTION      : dspcache_box_l2fetch                                                          */
    /*================================================================================================*/
    /*  CYCLE-COUNT   : 5                                                                             */
    /*================================================================================================*/
    /*  PARAMETERS    :                                                                               */
    /*  ============                                                                                  */
    /*   R0:  addr                                                                                    */
    /*   R1:  width                                                                                   */
    /*   R2:  height                                                                                  */
    /*   R3:  stride                                                                                  */
    /*================================================================================================*/
    /* IMPLEMENTATION SUMMARY   :                                                                     */
    /* ======================                                                                         */
    /*   Issues box-type L2FETCH for up to 255 bytes width x 255 bytes height, with stride up to      */
    /*   16383. No alignment is necessary, but addr must be a validly mapped virtual address.         */
    /*                                                                                                */
    /*================================================================================================*/

   .text
   .p2align 4
   .globl dspcache_box_l2fetch
   .type dspcache_box_l2fetch, @function
dspcache_box_l2fetch:
   {
     R3 = ASL(R3,#16)                  // L2FETCH [stride]
     R1 = SATUB(R1)                    // limit width to 255
   }{
     R2 = SATUB(R2)                    // limit height to 255
     R3 |= ASL(R1,#8)                  // L2FETCH [stride : width]
   }{
     R3 = OR(R3,R2)                    // L2FETCH [stride : width : height]
   }{
     L2FETCH(R0,R3)                    // fetch
   }{
     JUMPR R31                         // return
   }
   .size  dspcache_box_l2fetch, .-dspcache_box_l2fetch

    /*================================================================================================*/
    /*  FUNCTION      : dspcache_linear_dczeroa                                                        */
    /*================================================================================================*/
    /*  CYCLE-COUNT   : 6 + ~(bytes/32)                                                               */
    /*================================================================================================*/
    /*  PARAMETERS    :                                                                               */
    /*  ============                                                                                  */
    /*   R0:  addr                                                                                    */
    /*   R1:  bytes                                                                                   */
    /*================================================================================================*/
    /* IMPLEMENTATION SUMMARY   :                                                                     */
    /* ======================                                                                         */
    /*   Issues DCZERO on all cache lines fully contained in the requested buffer.                    */
    /*                                                                                                */
    /*================================================================================================*/

   .text
   .p2align 4
   .globl dspcache_linear_dczeroa
   .type dspcache_linear_dczeroa, @function
dspcache_linear_dczeroa:
   {
     IF (R0==#0) JUMP:NT dspcache_linear_dczeroa_end // check for NULL addr
     R0 = ADD(R0,#31)                  // find first 32-byte boundary
     R1 = ADD(R0,R1)                   // find right edge
   }{
     R0 = AND(R0,#~31)                 // find first 32-byte boundary
   }{
     R1 = SUB(R1,R0)                   // adjust bytes for left-alignment
   }{
     R1 = ASR(R1,#5)                   // cache lines = bytes/32
     IF (!CMP.GT(R1.NEW,#0)) JUMP:NT dspcache_linear_dczeroa_end // skip loop if no full cache lines.
   }{
     LOOP0(dspcache_linear_dczeroa_loop, R1) // loop setup
   }
   .falign
dspcache_linear_dczeroa_loop:
   {
     DCZEROA(R0)                       // clear cache line
     R0 = ADD(R0,#32)                  // advance to next cache line
   }:endloop0
   .falign
dspcache_linear_dczeroa_end:
   {
     JUMPR R31                         // return
   }
   .size  dspcache_linear_dczeroa, .-dspcache_linear_dczeroa

    /*================================================================================================*/
    /*  FUNCTION      : dspcache_box_dczero                                                           */
    /*================================================================================================*/
    /*  CYCLE-COUNT   : 3 + height * (5 + width/32)                                                   */
    /*================================================================================================*/
    /*  PARAMETERS    :                                                                               */
    /*  ============                                                                                  */
    /*   R0:  addr                                                                                    */
    /*   R1:  width                                                                                   */
    /*   R2:  height                                                                                  */
    /*   R3:  stride                                                                                  */
    /*================================================================================================*/
    /* IMPLEMENTATION SUMMARY   :                                                                     */
    /* ======================                                                                         */
    /*   Issues DCZERO on all cache lines fully contained in the requested box. Each row is handled   */
    /*   separately, in case stride is not a multiple of the cache line size.                         */
    /*                                                                                                */
    /*================================================================================================*/

   .text
   .p2align 4
   .globl dspcache_box_dczeroa
   .type dspcache_box_dczeroa, @function
dspcache_box_dczeroa:
   {
     P0 = !CMP.EQ(R0,#0)               //check for NULL pointer
     P0 = CMP.GT(R2,#0)               // (height > 0)
     IF (!P0.NEW) JUMPR:NT R31                // return
     R5 = ADD(R0,#31)                  // finding first cache line in first row.
   }{
     LOOP1(dspcache_box_dczeroa_outer_loop,R2)
     R6 = ADD(R0,R1)                   // right edge
     R0 = ADD(R0,R3)                   // start of second row
   }
   .falign
dspcache_box_dczeroa_outer_loop:
   {
     R5 = AND(R5,#~31)                 // point to start of cache line
   }{
     R6 = SUB(R6,R5)                   // adjust width to aligned row start
   }{
     R6 = ASR(R6,#5)                   // cache lines = bytes / 32
     IF(!CMP.GT(R6.NEW,#0)) JUMP:NT dspcache_box_dczeroa_skip_inner_loop // if no full cache lines, skip inner loop
   }{
     LOOP0(dspcache_box_dczeroa_inner_loop, R6) // loop setup
   }
   .falign
dspcache_box_dczeroa_inner_loop:
   {
     DCZEROA(R5)                       // clear cache line
     R5 = ADD(R5,#32)                  // increment to next cache line
   }:endloop0
   .falign
dspcache_box_dczeroa_skip_inner_loop:
   {
     R5 = ADD(R0,#31)                  // find first cache line in next row
     R6 = ADD(R0,R1)                   // find right edge of next row.
     R0 = ADD(R0,R3)                   // increment to row after next.
   }:endloop1
   {
     JUMPR R31                         // return
   }
   .size  dspcache_box_dczeroa, .-dspcache_box_dczeroa
