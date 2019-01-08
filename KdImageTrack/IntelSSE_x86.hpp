/**
 *    This file is part of KdImageTrack.
 *
 *   KdImageTrack is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   KdImageTrack is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with KdImageTrack. If not, see <https://www.gnu.org/licenses/>.
 *
 *    Copyright (c) 2015 Kudan. All rights reserved.
 **/

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
