//
//  IntelSSE_x86.hpp
//  KdImageTrack
//
//  Created by WANG Zirui on 06/12/2016.
//  Copyright © 2016 Kudan. All rights reserved.
//

#if defined(__AVX__)

#ifndef IntelSSE_x86_h
#define IntelSSE_x86_h

#include <opencv2/opencv.hpp>

// header for AVX intrinsics
#include <immintrin.h>

// This struct is only to hold the return value of znccIntrinsics().
struct sums{
    int sum;
    int sum3;
};

// ZNCC() using intrinsics, return the 'sum' and 'sum3' as a struct.
sums znccIntrinsics(short templatePixelArray[], unsigned char *imgPtr, unsigned imageMean, unsigned imgStep);

// descriptorMatching() using intrinsics
void descriptorIntrinsicMatching(cv::Mat &queryDescriptors, cv::Mat &trainDescriptors, int output_best_idx[], int output_best_score[]);

#endif /* IntelSSE_x86_h */

#endif
