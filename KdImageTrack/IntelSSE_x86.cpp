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

#include "IntelSSE_x86.hpp"

sums znccIntrinsics(short templatePixelArray[], unsigned char *imgPtr, unsigned imageMean, unsigned imgStep){
    
    sums sums;
    sums.sum = 0;
    sums.sum3 = 0;
    
    __m256i tempVec12, tempVec34, tempVec56, tempVec78;
    __m128i tempVec1, tempVec2, tempVec3, tempVec4, tempVec5, tempVec6, tempVec7, tempVec8;
    __m128i imgVec1, imgVec2, imgVec3, imgVec4, imgVec5, imgVec6, imgVec7, imgVec8;
    __m128i imgMeanVec;
    __m128i sumVec1, sumVec2, sumVec3, sumVec4, sumVec5, sumVec6, sumVec7, sumVec8;
    __m128i sum3Vec1, sum3Vec2, sum3Vec3, sum3Vec4, sum3Vec5, sum3Vec6, sum3Vec7, sum3Vec8;
    __m128i temSumVec;
    __m128i temSum3Vec;
    
    // load image means
    imgMeanVec = _mm_set1_epi16((short)imageMean);
    
    // load 8 rows of template differeces, unpack one row
    tempVec12 = _mm256_load_si256((__m256i*) &templatePixelArray[0*16]);
    tempVec34 = _mm256_load_si256((__m256i*) &templatePixelArray[1*16]);
    tempVec56 = _mm256_load_si256((__m256i*) &templatePixelArray[2*16]);
    tempVec78 = _mm256_load_si256((__m256i*) &templatePixelArray[3*16]);
    
    tempVec1 = _mm256_extractf128_si256(tempVec12, 0);
    tempVec2 = _mm256_extractf128_si256(tempVec12, 1);
    tempVec3 = _mm256_extractf128_si256(tempVec34, 0);
    tempVec4 = _mm256_extractf128_si256(tempVec34, 1);
    tempVec5 = _mm256_extractf128_si256(tempVec56, 0);
    tempVec6 = _mm256_extractf128_si256(tempVec56, 1);
    tempVec7 = _mm256_extractf128_si256(tempVec78, 0);
    tempVec8 = _mm256_extractf128_si256(tempVec78, 1);
    
    // load 8 rows of image pixels
    imgVec1 = _mm_cvtepu8_epi16(_mm_loadl_epi64((__m128i*) imgPtr));    //row 1
    imgVec2 = _mm_cvtepu8_epi16(_mm_loadl_epi64((__m128i*) (imgPtr+imgStep))); //row 2
    imgVec3 = _mm_cvtepu8_epi16(_mm_loadl_epi64((__m128i*) (imgPtr+2*imgStep)));    //row 1
    imgVec4 = _mm_cvtepu8_epi16(_mm_loadl_epi64((__m128i*) (imgPtr+3*imgStep))); //row 2
    imgVec5 = _mm_cvtepu8_epi16(_mm_loadl_epi64((__m128i*) (imgPtr+4*imgStep)));    //row 1
    imgVec6 = _mm_cvtepu8_epi16(_mm_loadl_epi64((__m128i*) (imgPtr+5*imgStep))); //row 2
    imgVec7 = _mm_cvtepu8_epi16(_mm_loadl_epi64((__m128i*) (imgPtr+6*imgStep)));    //row 1
    imgVec8 = _mm_cvtepu8_epi16(_mm_loadl_epi64((__m128i*) (imgPtr+7*imgStep))); //row 2
    
    // image pixels - image means
    imgVec1 = _mm_sub_epi16(imgVec1,imgMeanVec);
    imgVec2 = _mm_sub_epi16(imgVec2,imgMeanVec);
    imgVec3 = _mm_sub_epi16(imgVec3,imgMeanVec);
    imgVec4 = _mm_sub_epi16(imgVec4,imgMeanVec);
    imgVec5 = _mm_sub_epi16(imgVec5,imgMeanVec);
    imgVec6 = _mm_sub_epi16(imgVec6,imgMeanVec);
    imgVec7 = _mm_sub_epi16(imgVec7,imgMeanVec);
    imgVec8 = _mm_sub_epi16(imgVec8,imgMeanVec);
    
    // template diff * image diff and sum
    sumVec1 = _mm_madd_epi16(imgVec1, tempVec1);
    sumVec2 = _mm_madd_epi16(imgVec2, tempVec2);
    sumVec3 = _mm_madd_epi16(imgVec3, tempVec3);
    sumVec4 = _mm_madd_epi16(imgVec4, tempVec4);
    sumVec5 = _mm_madd_epi16(imgVec5, tempVec5);
    sumVec6 = _mm_madd_epi16(imgVec6, tempVec6);
    sumVec7 = _mm_madd_epi16(imgVec7, tempVec7);
    sumVec8 = _mm_madd_epi16(imgVec8, tempVec8);
    
    // image diff square and sum
    sum3Vec1 = _mm_madd_epi16(imgVec1, imgVec1);
    sum3Vec2 = _mm_madd_epi16(imgVec2, imgVec2);
    sum3Vec3 = _mm_madd_epi16(imgVec3, imgVec3);
    sum3Vec4 = _mm_madd_epi16(imgVec4, imgVec4);
    sum3Vec5 = _mm_madd_epi16(imgVec5, imgVec5);
    sum3Vec6 = _mm_madd_epi16(imgVec6, imgVec6);
    sum3Vec7 = _mm_madd_epi16(imgVec7, imgVec7);
    sum3Vec8 = _mm_madd_epi16(imgVec8, imgVec8);
    
    // add all elements
    sumVec1 = _mm_add_epi32(sumVec1, sumVec2);      // row 1+2
    sumVec3 = _mm_add_epi32(sumVec3, sumVec4);      // row 3+4
    sumVec5 = _mm_add_epi32(sumVec5, sumVec6);      // row 5+6
    sumVec7 = _mm_add_epi32(sumVec7, sumVec8);      // row 7+8
    
    sum3Vec1 = _mm_add_epi32(sum3Vec1, sum3Vec2);
    sum3Vec3 = _mm_add_epi32(sum3Vec3, sum3Vec4);
    sum3Vec5 = _mm_add_epi32(sum3Vec5, sum3Vec6);
    sum3Vec7 = _mm_add_epi32(sum3Vec7, sum3Vec8);
    
    sumVec1 = _mm_add_epi32(sumVec1, sumVec3);      // row 1+2+3+4
    sumVec5 = _mm_add_epi32(sumVec5, sumVec7);      // row 5+6+7+8
    
    sum3Vec1 = _mm_add_epi32(sum3Vec1, sum3Vec3);
    sum3Vec5 = _mm_add_epi32(sum3Vec5, sum3Vec7);
    
    sumVec1 = _mm_add_epi32(sumVec1, sumVec5);      // row 1+2+3+4+5+6+7+8
    sum3Vec1 = _mm_add_epi32(sum3Vec1, sum3Vec5);
    
    temSumVec = _mm_hadd_epi32(sumVec1, sumVec1);
    temSumVec = _mm_hadd_epi32(temSumVec, temSumVec);
    
    temSum3Vec = _mm_hadd_epi32(sum3Vec1, sum3Vec1);
    temSum3Vec = _mm_hadd_epi32(temSum3Vec, temSum3Vec);
    
    sums.sum += _mm_extract_epi32(temSumVec,0);
    sums.sum3 += _mm_extract_epi32(temSum3Vec,0);
    
    return sums;
    
}

void descriptorIntrinsicMatching(cv::Mat &queryDescriptors, cv::Mat &trainDescriptors, int output_best_idx[], int output_best_score[]){
    __m256i trainData, queryData;
    __m128i trainData1, trainData2, queryData1, queryData2;
    __m128i xor1, xor2;
    unsigned long long xor11, xor12, xor21, xor22;
    long long sum11, sum12, sum21, sum22;
    int distance;
    
    for (int rowQuery = 0; rowQuery < queryDescriptors.rows; rowQuery++){
        int lowest_distance = 256;
        int lowest_idx = -1;
        
        queryData = _mm256_load_si256((__m256i*) (queryDescriptors.ptr()+rowQuery*32));
        queryData1 = _mm256_extractf128_si256(queryData, 0);
        queryData2 = _mm256_extractf128_si256(queryData, 1);
        
        for (int rowTrain = 0; rowTrain < trainDescriptors.rows; rowTrain++){
            
            trainData = _mm256_load_si256((__m256i*) (trainDescriptors.ptr()+rowTrain*32));
            trainData1 = _mm256_extractf128_si256(trainData, 0);
            trainData2 = _mm256_extractf128_si256(trainData, 1);
            
            // XOR
            xor1 = _mm_xor_si128(trainData1, queryData1);
            xor2 = _mm_xor_si128(trainData2, queryData2);
            
            // Count bit 1
            xor11 = _mm_extract_epi64(xor1, 0);
            xor12 = _mm_extract_epi64(xor1, 1);
            xor21 = _mm_extract_epi64(xor2, 0);
            xor22 = _mm_extract_epi64(xor2, 1);
            
            sum11 = _mm_popcnt_u64(xor11);
            sum12 = _mm_popcnt_u64(xor12);
            sum21 = _mm_popcnt_u64(xor21);
            sum22 = _mm_popcnt_u64(xor22);
            
            distance = (int)((sum11+sum12)+(sum21+sum22));
            
            // lowest_distance = MIN(distance, lowest_distance);
            
            if (distance < lowest_distance){
                lowest_distance = distance;
                lowest_idx = rowTrain;
            }
            
        } // end train
        
        output_best_idx[rowQuery] = lowest_idx;
        output_best_score[rowQuery] = lowest_distance;
        
    } // end query
}

#endif
