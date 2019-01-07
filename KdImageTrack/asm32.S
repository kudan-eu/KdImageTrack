#if defined(__arm__)

.align 2
#ifdef __APPLE__
.globl _matchasm
.private_extern _matchasm
#else
.globl matchasm
#endif


query .req r0
train .req r1
count2 .req r2
idx .req r4
minVal .req r5
minIdx .req r6

#ifdef __APPLE__
_matchasm:
#else
matchasm:
#endif

    pld [train]
    push {idx, minVal, minIdx}

    mov idx, #0
    mov minVal, #256

    // load query, split over q0 and q1		
    vld1.64 {q0-q1}, [query]!

.loop2:
    // load train
    pld [train, #64];
    vld1.64 {q2-q3}, [train]!

    // xor.
    veor q2, q0, q2
    veor q3, q1, q3

    // count bits.
    vcnt.i8 q2, q2
    vcnt.i8 q3, q3

    // sum counts.
    vpaddl.u8 q2, q2
    vpaddl.u8 q3, q3

    vpaddl.u16 q2, q2
    vpaddl.u16 q3, q3

    vpadd.u32 d4, d4, d5
    vpadd.u32 d6, d6, d7

    vpadd.u32 d4, d4, d6
    vpadd.u32 d4, d4, d4

    // return sum.
    vmov.u32 r0, d4[0]

    cmp r0, minVal
    movlt minVal, r0
    movlt minIdx, idx

    add idx, #1

    subs count2, #1
    bgt .loop2

    // return min distance and its query index.
    mov r0, minVal
    mov r1, minIdx

    pop {idx, minVal, minIdx}
    bx lr

.align 2
#ifdef __APPLE__
.globl _neonncc
.private_extern _neonncc
#else
.globl neonncc
#endif

patch .req r0
image .req r1
templateMean .req r2
imageMean .req r3
imageWidth .req r4
zeroConstant .req r2

heightIdx .req r5

#ifdef __APPLE__
_neonncc:
#else
neonncc:
#endif

    push {heightIdx, imageWidth}

    ldr imageWidth, [sp, #8]

    mov heightIdx, #0

    // fill q14 with the 16-bit template mean.
    vdup.16 q14, templateMean

    // fill q15 with the 16-bit image mean.
    vdup.16 q15, imageMean

    // q10 and q11 are sum
    mov zeroConstant, #0
    vdup.32 q10, zeroConstant
    vdup.32 q11, zeroConstant

    // q12 and q13 are sum3
    vdup.32 q12, zeroConstant
    vdup.32 q13, zeroConstant

loopHeight:

    // load 8 pixels from patch
    vld1.64 {d0}, [patch]!

    // expand to 16-bit
    vmovl.u8 q0, d0

    // d0 and d1 contain 16-bit template, subtract mean
    vsub.s16 q0, q0, q14

    // expand to signed 32-bit
    vmovl.s16 q1, d0
    vmovl.s16 q2, d1

    // load 8 pixels from image. don't auto increment since image has a stride.
    vld1.64 {d0}, [image]

    // expand to 16-bit
    vmovl.u8 q0, d0

    // d0 and d1 contain 16-bit image, subtract mean
    vsub.s16 q0, q0, q15

    // expand to signed 32-bit
    vmovl.s16 q3, d0
    vmovl.s16 q8, d1

    // sum0 += template * image
    vmla.i32 q10, q1, q3
    vmla.i32 q11, q2, q8

    // sum3 += image * image
    vmla.i32 q12, q3, q3
    vmla.i32 q13, q8, q8

    // image is not and has a stride.
    add image, image, imageWidth

    // 8 iterations.
    add heightIdx, heightIdx, #1
    cmp heightIdx, #8
    bne loopHeight

    // combine sum lanes
    vpadd.i32 d20, d20, d21
    vpadd.i32 d22, d22, d23

    vpadd.i32 d24, d24, d25
    vpadd.i32 d26, d26, d27

    vpadd.i32 d20, d20, d22
    vpadd.i32 d24, d24, d26

    vmov.s32 r1, d20[0]
    vmov.s32 r2, d20[1]

    // sum0
    add r0, r1, r2

    vmov.s32 r1, d24[0]
    vmov.s32 r2, d24[1]

    // sum3
    add r1, r1, r2

    // return
    pop {heightIdx, imageWidth}
    bx lr



#endif
