//
//  asm64.s
//  KdImageTrack
//
//  Created on 23/06/2015.
//  Copyright (c) 2015 Kudan. All rights reserved.
//

#if defined(__arm64__) || defined(__aarch64__)

.align 2
#ifdef __APPLE__
.globl _matchasm
.private_extern _matchasm
#else
.globl matchasm
#endif


query .req x0
train .req x1
count .req x2
idx .req x3
minVal .req x4
minIdx .req x5
tmp1 .req x6
tmp2 .req x7

#ifdef __APPLE__
_matchasm:
#else
matchasm:
#endif
    mov idx, #0
    mov minVal, #256

    ld1 {v0.2d}, [query], 16
    ld1 {v1.2d}, [query], 16

.loop2:
    // load train
    ld1 {v2.2d}, [train], 16
    ld1 {v3.2d}, [train], 16


    eor.16b v2, v0, v2
    eor.16b v3, v1, v3

    cnt.16b v2, v2
    cnt.16b v3, v3

    uaddlp.8h v2, v2
    uaddlp.8h v3, v3

    uaddlp.4s v2, v2
    uaddlp.4s v3, v3

    uaddlp.2d v2, v2
    uaddlp.2d v3, v3

    addp.2d d2, v2
    addp.2d d3, v3

    add d2, d2, d3

    // score
    fmov tmp1, d2

    cmp tmp1, minVal
    bgt next

    mov minVal, tmp1
    mov minIdx, idx


next:
    add idx, idx, #1

    subs count, count, #1
    bgt .loop2

    mov x0, minIdx
    lsl x0, x0, #32

    orr x0, x0, minVal

    ret



// zncc arm64
.align 1
#ifdef __APPLE__
.globl _neonncc64
.private_extern _neonncc64
#else
.globl neonncc64
#endif

ptr1 .req x0                    // (patch - patchMean) array ptr
ptr2 .req x1                    // ROI image array ptr
ptr_step .req x2                // the pointer increasment, to jump to next line
imgMean .req x3                 // image ROI mean

zeroConst .req w4               // stores zero

#ifdef __APPLE__
_neonncc64:
#else
neonncc64:
#endif
//---------Push------------
stp d8,d9,[sp,#-16]!
stp d10,d11,[sp,#-16]!
stp d12,d13,[sp,#-16]!
stp d14,d15,[sp,#-16]!

//----------1 cycle--------
// load one col  8 pixels (short) from patchDiff, one col from imgDiff
ld1 {v0.8h}, [ptr1], #16        // col 1
ld1 {v1.16b}, [ptr2], ptr_step

dup v31.16b, w3                 // imagemean
mov zeroConst, #0

//----------1 cycle--------
ld1 {v2.8h}, [ptr1], #16        // col 2
ld1 {v3.16b}, [ptr2], ptr_step

dup v16.4s, zeroConst           // initialize first half of the accumulated sum
dup v17.4s, zeroConst           // initialize second half of the accumulated sum

//----------1 cycle--------
ld1 {v4.8h}, [ptr1], #16        // col 3
ld1 {v5.16b}, [ptr2], ptr_step

dup v18.4s, zeroConst           // initialize first half of the accumulated sum3
dup v19.4s, zeroconst           // initialize second half of the accumulated sum3

//----------1 cycle--------
ld1 {v6.8h}, [ptr1], #16        // col 4
ld1 {v7.16b}, [ptr2], ptr_step

usubl v1.8h, v1.8b, v31.8b      // col 1 - mean

//----------1 cycle--------
ld1 {v8.8h}, [ptr1], #16        // col 5
ld1 {v9.8b}, [ptr2], ptr_step

usubl v3.8h, v3.8b, v31.8b      // col 2 - mean

smlal v16.4s, v0.4h, v1.4h      // tem col 1 * img col 1
smlal2 v17.4s, v0.8h, v1.8h     // tem col 1 * img col 1
smlal v18.4s, v1.4h, v1.4h      // img col 1 * img col 1
smlal2 v19.4s, v1.8h, v1.8h     // img col 1 * img col 1

//----------1 cycle--------
ld1 {v10.8h}, [ptr1], #16       // col 6
ld1 {v11.8b}, [ptr2], ptr_step

usubl v5.8h, v5.8b, v31.8b      // col 3 - mean

smlal v16.4s, v2.4h, v3.4h      // tem col 2 * img col 2
smlal2 v17.4s, v2.8h, v3.8h     // tem col 1 * img col 1
smlal v18.4s, v3.4h, v3.4h      // img col 2 * img col 2
smlal2 v19.4s, v3.8h, v3.8h

//----------1 cycle--------
ld1 {v12.8h}, [ptr1], #16       // col 7
ld1 {v13.8b}, [ptr2], ptr_step

usubl v7.8h, v7.8b, v31.8b      // col 4 - mean

smlal v16.4s, v4.4h, v5.4h      // tem col 3 * img col 3
smlal2 v17.4s, v4.8h, v5.8h     // tem col 1 * img col 1
smlal v18.4s, v5.4h, v5.4h      // img col 3 * img col 3
smlal2 v19.4s, v5.8h, v5.8h

//----------1 cycle--------
ld1 {v14.8h}, [ptr1]            // col 8
ld1 {v15.8b}, [ptr2]

usubl v9.8h, v9.8b, v31.8b      // col 5 - mean

smlal v16.4s, v6.4h, v7.4h      // tem col 4 * img col 4
smlal2 v17.4s, v6.8h, v7.8h     // tem col 4 * img col 4
smlal v18.4s, v7.4h, v7.4h      // img col 4 * img col 4
smlal2 v19.4s, v7.8h, v7.8h     // img col 4 * img col 4

//----------1 cycle--------
usubl v11.8h, v11.8b, v31.8b    // col 6 - mean

smlal v16.4s, v8.4h, v9.4h      // tem col 5 * img col 5
smlal2 v17.4s, v8.8h, v9.8h     // tem col 5 * img col 5
smlal v18.4s, v9.4h, v9.4h      // img col 5 * img col 5
smlal2 v19.4s, v9.8h, v9.8h     // img col 5 * img col 5

//----------1 cycle--------
usubl v13.8h, v13.8b, v31.8b    // col 7 - mean

smlal v16.4s, v10.4h, v11.4h    // tem col 6 * img col 6
smlal2 v17.4s, v10.8h, v11.8h   // tem col 6 * img col 6
smlal v18.4s, v11.4h, v11.4h    // img col 6 * img col 6
smlal2 v19.4s, v11.8h, v11.8h   // img col 6 * img col 6

//----------1 cycle--------
usubl v15.8h, v15.8b, v31.8b    // col 7 - mean

smlal v16.4s, v12.4h, v13.4h    // tem col 7 * img col 7
smlal2 v17.4s, v12.8h, v13.8h   // tem col 7 * img col 7
smlal v18.4s, v13.4h, v13.4h    // img col 7 * img col 7
smlal2 v19.4s, v13.8h, v13.8h   // img col 7 * img col 7

//----------1 cycle--------
smlal v16.4s, v14.4h, v15.4h    // tem col 8 * img col 8
smlal2 v17.4s, v14.8h, v15.8h   // tem col 8 * img col 8
smlal v18.4s, v15.4h, v15.4h    // img col 8 * img col 8
smlal2 v19.4s, v15.8h, v15.8h   // img col 8 * img col 8

//----------1 cycle--------
add v0.4s, v16.4s, v17.4s       // sum, first half + sencond half
add v1.4s, v18.4s, v19.4s       // sum3, first half + sencond half

addv s0, v0.4s                  // sum
addv s1, v1.4s                  // sum3

//----------1 cycle--------
// Pack sum and sum3 scalar to v0 vector and return
trn1 v0.4s, v1.4s, v0.4s

//----------1 cycle--------
mov x0, v0.d[0]                 //bit allocation: sum (0-31bit), sum3(32-63bit)

//-----------Pop-----------
ldp d14,d15,[sp],#16
ldp d12,d13,[sp],#16
ldp d10,d11,[sp],#16
ldp d8,d9,[sp],#16

ret



#endif
