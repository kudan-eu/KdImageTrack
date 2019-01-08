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
#include "PatchMatcher.h"
#include "Profiler.hpp"
#include "Helper.h"
#include "Logging.hpp"
#include <math.h>
#include "Logging.hpp"
#include "IntelSSE_x86.hpp"

Namespace(KdImageTrack)

#ifdef __cplusplus
extern "C" {
#endif
    void nccasm(unsigned char *patch, unsigned char *image, uint32_t *sum1, uint32_t *sum2);
#ifdef __cplusplus
}
#endif

static cv::Point2f pointByAffine(cv::Mat M, cv::Point2d &point)
{
    float x = M.at<float>(0, 0) * point.x + M.at<float>(0, 1) * point.y + M.at<float>(0, 2);
    float y = M.at<float>(1, 0) * point.x + M.at<float>(1, 1) * point.y + M.at<float>(1, 2);
    
    return cv::Point2f(x, y);
}


inline float roundf(float x)
{
   return x >= 0.0f ? floorf(x + 0.5f) : ceilf(x - 0.5f);
}

/* Returns false immediately if it tries to sample off the edge of the image */
bool myWarpAffine(cv::Mat warped, cv::Mat src, cv::Mat affineMatrix){
    const int size = 8;
    
    const int srcCols = src.cols;
    const int srcRows = src.rows;
    
    for (int i = 0; i < size; i++) {
        float i1 = affineMatrix.at<float>(0, 1) * i + affineMatrix.at<float>(0, 2);
        float i2 = affineMatrix.at<float>(1, 1) * i + affineMatrix.at<float>(1, 2);
        
        for (int j = 0; j < size; j++) {
            
            float x = affineMatrix.at<float>(0, 0) * j + i1;
            float y = affineMatrix.at<float>(1, 0) * j + i2;
            
            int fx = (int)x;
            int cx = fx + 1;
            
            int fy = (int)y;
            int cy = fy + 1;
            
            // prevent sampling from outside the source image.
            if (fx < 0 || fy < 0) {
                return false;
            }
            if (cx >= srcCols || cy >= srcRows) {
                return false;
            }
            
            float dx = 1 - (x - fx);
            float dy = 1 - (y - fy);
            
            unsigned fxfy = src.at<unsigned char>(fy, fx);
            unsigned cxfy = src.at<unsigned char>(fy, cx);
            unsigned fxcy = src.at<unsigned char>(cy, fx);
            unsigned cxcy = src.at<unsigned char>(cy, cx);
            
            // average in x
            float bx1 = fxfy * dx + (1 - dx) * cxfy;
            float bx2 = fxcy * dx + (1 - dx) * cxcy;
            
            float pixel = bx1 * dy + (1 - dy) * bx2;
            
            warped.at<unsigned char>(i, j) = (unsigned char)pixel;
        }
    }
    return true;
}

cv::Mat getAffine(std::vector<cv::Point2f> transformedPoints, std::vector<cv::Point2f> points)
{
    cv::Mat m(3, 3, CV_32F);
    
    m.at<float>(0, 0) = transformedPoints[0].x;
    m.at<float>(0, 1) = transformedPoints[1].x;
    m.at<float>(0, 2) = transformedPoints[2].x;
    
    m.at<float>(1, 0) = transformedPoints[0].y;
    m.at<float>(1, 1) = transformedPoints[1].y;
    m.at<float>(1, 2) = transformedPoints[2].y;
    
    m.at<float>(2, 0) = 1;
    m.at<float>(2, 1) = 1;
    m.at<float>(2, 2) = 1;

    cv::Mat m2(2, 3, CV_32F);
    
    m2.at<float>(0, 0) = points[0].x;
    m2.at<float>(0, 1) = points[1].x;
    m2.at<float>(0, 2) = points[2].x;
    
    m2.at<float>(1, 0) = points[0].y;
    m2.at<float>(1, 1) = points[1].y;
    m2.at<float>(1, 2) = points[2].y;

    m = m.inv();
    
    cv::Mat m3 = m2 * m;
    
    return m3;
}

cv::Mat getAffineTransform(cv::Mat inverseHomography, cv::Rect region, float *determinant, cv::Point2f *centre)
{
    // static to reduce allocation overhead.
    std::vector<cv::Point2f> points(3);
    std::vector<cv::Point2f> transformedPoints(3);
    
    points[0] = cv::Point2f(region.x, region.y);
    points[1] = cv::Point2f(region.x, region.y + region.height);
    points[2] = cv::Point2f(region.x + region.width, region.y + region.height);
    
    // transform back to the reference image.
    cv::perspectiveTransform(points, transformedPoints, inverseHomography);
    
    // calculate the affine transformation.
    cv::Mat affine = getAffine(transformedPoints, points);
    
    // position the affine patch at (0, 0).
    affine.at<float>(0, 2) -= region.x;
    affine.at<float>(1, 2) -= region.y;
    
    // make a 3x3 matrix so we can calculate the determinant.
    cv::Mat affineSquare(3, 3, CV_32F);
    float *d = (float *)affineSquare.data;
    float *s = (float *)affine.data;
    
    for (int i = 0; i < 6; i++) {
        d[i] = s[i];
    }
    
    d[6] = 0;
    d[7] = 0;
    d[8] = 1;
    
    // determine which pyramid level to use as a source.
    *determinant = 1.0 / cv::determinant(affineSquare);
    
    cv::invertAffineTransform(affine, affine);
    
    // centre of region in marker space.
    if (centre != NULL) {
        cv::Point2d middle = cv::Point2f(region.width * 0.5, region.height * 0.5);
        *centre = pointByAffine(affine, middle);
    }
    
    return affine;
}

/* Return false if the affine warp goes off the edge of the image */
bool affineWarp(std::shared_ptr<Marker> marker, cv::Mat inverseHomography, cv::Rect region, cv::Point2f *centre, cv::Mat &warpedAffine)
{
    std::vector<cv::Point2f> points(3);
    std::vector<cv::Point2f> transformedPoints(3);

    points[0] = cv::Point2f(region.x, region.y);
    points[1] = cv::Point2f(region.x, region.y + region.height);
    points[2] = cv::Point2f(region.x + region.width, region.y + region.height);

    // transform back to the reference image.
    cv::perspectiveTransform(points, transformedPoints, inverseHomography);
    
    // calculate the affine transformation.
    cv::Mat affine = getAffine(transformedPoints, points);

    
    // position the affine patch at (0, 0).
    affine.at<float>(0, 2) -= region.x;
    affine.at<float>(1, 2) -= region.y;
    
    // make a 3x3 matrix so we can calculate the determinant.
    cv::Mat affineSquare(3, 3, CV_32F);
    float *d = (float *)affineSquare.data;
    float *s = (float *)affine.data;
    
    for (int i = 0; i < 6; i++) {
        d[i] = s[i];
    }
    
    d[6] = 0;
    d[7] = 0;
    d[8] = 1;
    
    affine = affine(cv::Rect(0, 0, 3, 2));
    
    // determine which pyramid level to use as a source.
    float determinant = cv::determinant(affineSquare);
    
    cv::Mat referenceImage = marker->trackingImages[0].image;
    float scale = 1.0;
    
    determinant = 1 / determinant;
    
    int level = 0;
    int numberOfReferenceImages = (int)marker->trackingImages.size();
    
    for (int i = 0; i < numberOfReferenceImages; i++) {
        float f = determinant / (1 << (2 * level));
        if (f < 1) {
            break;
        }
        
        scale = 1.0 / (1 << level);
        
        referenceImage = marker->trackingImages[i].image;
        level++;
    }
    
    cv::invertAffineTransform(affine, affine);
    
    // centre of region in marker space.
    if (centre != NULL) {
        cv::Point2d middle = cv::Point2f(region.width * 0.5, region.height * 0.5);
        *centre = pointByAffine(affine, middle);
    }
    
    // scale to the chosen pyramid level.
    affine *= scale;
    
    // warp the reference image to create a patch matching the input.
    return myWarpAffine(warpedAffine, referenceImage, affine);
    
    
}

/* Return false if the affine warp goes off the edge of the image */
bool affineWarpFast(std::shared_ptr<Marker> marker, cv::Mat affine, float determinant, cv::Mat &warpedAffine)
{
    cv::Mat referenceImage = marker->trackingImages[0].image;
    float scale = 1.0;
    
    int level = 0;
    int numberOfReferenceImages = (int)marker->trackingImages.size();
    
    for (int i = 0; i < numberOfReferenceImages; i++) {
        float f = determinant / (1 << (2 * level));
        if (f < 1) {
            break;
        }
        
        scale = 1.0 / (1 << level);
        
        referenceImage = marker->trackingImages[i].image;
        level++;
    }
    
    // scale to the chosen pyramid level.
    affine *= scale;
    
    // warp the reference image to create a patch matching the input.
    return myWarpAffine(warpedAffine, referenceImage, affine);
    
}

#ifdef __cplusplus
extern "C"
{
#endif
    uint64_t neonncc(unsigned char *patch, unsigned char *image, int patchMean, int imageMean, unsigned width);
    uint64_t neonncc64(short *patchDiffArray, unsigned char *imgArray, int imgPtrStep, int imgMean);
#ifdef __cplusplus
}
#endif

bool ncc(cv::Mat result, cv::Mat image, cv::Mat patch, int *variance, int minVariance)
{
    int resultWidth = image.cols - patch.cols + 1;
    int resultHeight = image.rows - patch.rows + 1;
    
    const int patchWidth = 8;
    const int patchHeight = 8;
    
    int sum2 = 0;
    
    // calculate mean.
    unsigned templateMean = 0;

    for (int u = 0; u < patchWidth; u++) {
        for (int v = 0; v < patchHeight; v++) {
            unsigned templatePixel = patch.at<unsigned char>(v, u);
            templateMean += templatePixel;
        }
    }
    
    templateMean = roundf(templateMean / 64.0);
    
    // calculate templatePixel array
    short templatePixelArray[64]; // the difference could be negative
    int i = 0;
    for (int u = 0; u < patchWidth; u++) {
        for (int v = 0; v < patchHeight; v++) {
            templatePixelArray[i++] = patch.at<unsigned char>(u, v) - templateMean;
        }
    }
    
    // calculate pixel variance.
    int s = 0;
    
    for (int u = 0; u < patchWidth; u++) {
        for (int v = 0; v < patchHeight; v++) {
            int templatePixel = patch.at<unsigned char>(v, u);
            int tmp = (templatePixel - (int)templateMean);
            tmp *= tmp;
            s += tmp;
        }
    }
    
    int v = s / 64;
    
    // standard deviation.
    int stdDev = sqrtf(v);
    *variance = stdDev;
    
    // don't waste time on patches with low variance.
    if (stdDev < minVariance) {
        return false;
    }
    
    
    for (int u = 0; u < patchWidth; u++) {
        for (int v = 0; v < patchHeight; v++) {
            int templatePixel = patch.at<unsigned char>(v, u);
            templatePixel -= (int)templateMean;
            sum2 += templatePixel * templatePixel;
        }
    }
    
    
    for (int x = 0; x < resultWidth; x++) {
        for (int y = 0; y < resultHeight; y++) {
            int sum = 0;
            int sum3 = 0;
            
            // mean of image under inspection.
            unsigned imageMean = 0;
            for (int u = 0; u < patchWidth; u++) {
                for (int v = 0; v < patchHeight; v++) {
                    unsigned imagePixel = image.at<unsigned char>(y + v, x + u);
                    imageMean += imagePixel;
                }
            }
            
            imageMean = roundf(imageMean / 64.0);

            

#if defined(__arm__)
            unsigned char *start = &image.at<unsigned char>(y, x);
            
            uint64_t s = neonncc(patch.data, start, templateMean, imageMean, (unsigned)image.step);
            sum3 = (s >> 32) & 0xffffffff;
            sum = s & 0xffffffff;

#elif defined(__arm64__) || defined(__aarch64__)
            // integrated NEON ASM code.
            unsigned char *start = &image.at<unsigned char>(y,x);
            
            uint64_t s = neonncc64(templatePixelArray, start, (int)image.step, imageMean); //14 bytes between image lines
            
            sum = (s >> 32) & 0xffffffff;
            sum3 = s & 0xffffffff;
            
#elif defined(__AVX__)
            // integrated SSE/AVX code.
            unsigned char *start = &image.at<unsigned char>(y,x);

            sums sums = znccIntrinsics(templatePixelArray, start, imageMean, (unsigned)image.step);
            sum = sums.sum;
            sum3 = sums.sum3;
#else
            
            for (int u = 0; u < patchHeight; u++) {
                for (int v = 0; v < patchWidth; v++) {
                    int templatePixel = patch.at<unsigned char>(v, u) - (int)templateMean;
                    int imagePixel = image.at<unsigned char>(y + v, x + u) - (int)imageMean;
                    sum += templatePixel * imagePixel;
                    sum3 += imagePixel * imagePixel;
                }
            }
            
#endif
            if (sum2 == 0 || sum3 == 0) {
                result.at<float>(y, x) = 0.f;
            }
            else {
                float tmp = (float)sum2 * sum3;
                tmp = sqrt(tmp);
                
                if (tmp == 0) {
                    result.at<float>(y, x) = 0.f;
                }
                else {
                    tmp = sum / tmp;
                    
                    result.at<float>(y, x) = tmp;
                }
            }
        }
    }
    
    return true;
}

bool locatePatch(cv::Mat src, cv::Mat queryPatch, cv::Point2f &position, float *score, int *variance, bool *subpixelLocalised, float minThreshold, float minVariance)
{
    double maxVal;
    double minVal;
    
    // find best NCC response.
    int resultWidth = src.cols - queryPatch.cols + 1;
    int resultHeight = src.rows - queryPatch.rows + 1;
    
    cv::Mat result(resultHeight, resultWidth, CV_32F);

    bool success = ncc(result, src, queryPatch, variance, minVariance);

    if (success == false) {
        return false;
    }
    
    cv::Point matchLoc;
    
    // find best match with 1-pixel accuracy.
    cv::minMaxLoc(result, &minVal, &maxVal, NULL, &matchLoc, cv::Mat());
    
    *score = maxVal;
    
    if (maxVal < minThreshold) {
        // not a good enough match.
        return false;
    }
    
    if (matchLoc.x == 0 || matchLoc.y == 0 || matchLoc.x == result.cols - 1 || matchLoc.y == result.rows - 1) {
        // won't be able to localise subpixel.
        position.x = matchLoc.x + queryPatch.cols / 2.0 - 1;
        position.y = matchLoc.y + queryPatch.rows / 2.0 - 1;
        
        *subpixelLocalised = false;
        
        return true;
    }
    
    // 3x3 matrix surrounding the maximum.
    float localArea[3][3];
    
    localArea[0][0] = result.at<float>(cv::Point(matchLoc.x - 1, matchLoc.y - 1));
    localArea[1][0] = result.at<float>(cv::Point(matchLoc.x, matchLoc.y - 1));
    localArea[2][0] = result.at<float>(cv::Point(matchLoc.x + 1, matchLoc.y - 1));
    
    localArea[0][1] = result.at<float>(cv::Point(matchLoc.x - 1, matchLoc.y));
    localArea[1][1] = result.at<float>(cv::Point(matchLoc.x, matchLoc.y));
    localArea[2][1] = result.at<float>(cv::Point(matchLoc.x + 1, matchLoc.y));
    
    localArea[0][2] = result.at<float>(cv::Point(matchLoc.x - 1, matchLoc.y + 1));
    localArea[1][2] = result.at<float>(cv::Point(matchLoc.x, matchLoc.y + 1));
    localArea[2][2] = result.at<float>(cv::Point(matchLoc.x + 1, matchLoc.y + 1));
    
    int x = 1;
    int y = 1;
    
    // fit quadratic curve to find subpixel coordinates of the maximum.
    float dx = (localArea[x + 1][y] - localArea[x - 1][y]) / 2.f;
    float dy = (localArea[x][y + 1] - localArea[x][y - 1]) / 2.f;
    float dxx = (localArea[x + 1][y] + localArea[x- 1 ][y] - 2 * localArea[x][y]);
    float dyy = (localArea[x][y + 1] + localArea[x][y - 1] - 2 * localArea[x][y]);
    float dxy = (localArea[x + 1][y + 1] - localArea[x + 1][y - 1] - localArea[x - 1][y + 1] + localArea[x - 1][y - 1]) / 4.f;
    
    float det = 1 / (dxx * dyy - dxy * dxy);
    
    // subpixel position relatives to localArea.
    float ix = x - (dyy * dx - dxy * dy) * det;
    float iy = y - (dxx * dy - dxy * dx) * det;
    
    // subpixel position relative to the source image.
    position.x = ix + matchLoc.x + queryPatch.cols / 2.0 - 1;
    position.y = iy + matchLoc.y + queryPatch.rows / 2.0 - 1;
    
    *subpixelLocalised = true;
    
    return true;
}

class ParallelMatch : public cv::ParallelLoopBody
{
private:
    cv::Point2f *keptWarpedPoints;
    bool havePreviousHomography;
    cv::Mat homographyInverse;
    cv::Mat previousHomographyInverse;
    std::shared_ptr<Marker> marker;
    float matchThreshold;
    cv::Mat camImage;
    
    // writes
    uint8_t *pointWasFound;
    cv::Point2f *pointLocationInImage;
    cv::Point2f *pointLocationInMarker;
    
public:
    ParallelMatch(cv::Point2f *kept, bool havePrevHomog, cv::Mat homogInverse, cv::Mat prevHomogInverse, std::shared_ptr<Marker> m, float threshold, cv::Mat img, uint8_t *found, cv::Point2f *locImage, cv::Point2f *locMarker)
    {
        keptWarpedPoints = kept;
        havePreviousHomography = havePrevHomog;
        homographyInverse = homogInverse;
        previousHomographyInverse = prevHomogInverse;
        marker = m;
        matchThreshold = threshold;
        camImage = img;
        
        pointWasFound = found;
        pointLocationInImage = locImage;
        pointLocationInMarker = locMarker;
    }
    
    virtual void operator()(const cv::Range &range) const
    {
        for (int i = range.start; i < range.end; i++) {
            
            // point in marker.
            cv::Point2f markerPoint;
            
            // predicted position of point in camera.
            const cv::Point2f &warpedPoint = keptWarpedPoints[i];
            
            // roi in the camera image.
            cv::Point2f roundedPoint;
            roundedPoint.x = roundf(warpedPoint.x);
            roundedPoint.y = roundf(warpedPoint.y);
            
            const int patchArea = 8;
            const int searchArea = 14;
            const int halfSearchArea = searchArea / 2;
            const int halfPatchArea = patchArea / 2;
            
            cv::Rect inputRect = cv::Rect(roundedPoint.x - halfSearchArea, roundedPoint.y - halfSearchArea, searchArea, searchArea);
            cv::Rect camRect = cv::Rect(roundedPoint.x - halfPatchArea, roundedPoint.y - halfPatchArea, patchArea, patchArea);
            
            cv::Point2f centre;
            cv::Mat warpedAffine(patchArea, patchArea, CV_8UC1);
            
            // figure out what was under that patch over the exposure time.
            const int numberOfSamples = 1;
            int numberOfGoodSamples = 0;
            
            // this is to see whether a warped patch is actually available:
            bool hasWarp = false;
            
            if (havePreviousHomography == false || numberOfSamples == 1) {
                hasWarp = affineWarp(marker, homographyInverse, camRect, &markerPoint, warpedAffine);
                printlog(LOG_PATCH,"Using one sample: haswarp = %i \n", int(hasWarp));
            } else {
                
                // the sum of all the samples.
                cv::Mat sumMat(8, 8, CV_32F);
                
                for (int k = 0; k < 8; k++) {
                    for (int l = 0; l < 8; l++) {
                        sumMat.at<float>(k, l) = 0;
                    }
                }
                
                float destDeterminant;
                cv::Mat destAffine = getAffineTransform(homographyInverse, camRect, &destDeterminant, &markerPoint);
                
                float prevDeterminant;
                cv::Mat prevAffine = getAffineTransform(previousHomographyInverse, camRect, &prevDeterminant, NULL);
                
                cv::Mat warpedAffineLerped(8, 8, CV_8UC1);
                for (int j = 0; j < numberOfSamples; j++) {
                    // interp position 0..1;
                    float t = (float)j / (numberOfSamples - 1);
                    
                    const float stepSize = 1.0 / numberOfSamples;
                    t *= 1 - stepSize;
                    t += stepSize;
                    
                    cv::Mat affine = lerpAffine(prevAffine, destAffine, t);
                    float determinant = lerp(prevDeterminant, destDeterminant, t);
                    
                    // create a warp at the desired time.
                    
                    bool hasThisWarp = affineWarpFast(marker, affine, determinant, warpedAffineLerped);
                    
                    if (hasThisWarp) {
                        // accumulate samples.
                        for (int k = 0; k < 8; k++) {
                            for (int l = 0; l < 8; l++) {
                                sumMat.at<float>(k, l) += warpedAffineLerped.at<unsigned char>(k, l);
                            }
                        }
                        numberOfGoodSamples++;
                    }
                }
                
                printlog(LOG_PATCH,"Using %i samples, of which %i are good \n", numberOfSamples, numberOfGoodSamples);

                
                // take the average using the number of samples actually obtained (if any)
                if (numberOfGoodSamples > 0) {
                    // calculate average of the samples.
                    for (int k = 0; k < 8; k++) {
                        for (int l = 0; l < 8; l++) {
                            float average = sumMat.at<float>(k, l) / numberOfGoodSamples;
                            warpedAffine.at<unsigned char>(k, l) = roundf(average);
                        }
                    }
                    hasWarp = true;
                }
            }
            
            // only try locating the patch if there is an affine warp availabele:
            if (hasWarp) {
                cv::Mat inputROI = camImage(inputRect);
                cv::Point2f patchPosition;
                
                float score;
                int variance;
                bool subpixelLocalised;
                
                bool located = locatePatch(inputROI, warpedAffine, patchPosition, &score, &variance, &subpixelLocalised, matchThreshold, 10);

                if (located == true) {
                    cv::Point2f point(roundedPoint.x - halfSearchArea + patchPosition.x, roundedPoint.y - halfSearchArea + patchPosition.y);
                    
                    pointWasFound[i] = true;
                    pointLocationInImage[i] = point;
                    pointLocationInMarker[i] = markerPoint;
                }
            }
            else pointWasFound[i] = false;
        }
    }
};


bool PatchMatcher::match()
{

    
    printlog(LOG_PATCH,"Patch matching on image %i \n", currentCameraImage->timestamp );

    // width of the patch in pixels.
    const float patchArea = 8;
    const float halfPatchArea = patchArea / 2;
    
    // width of the search area in pixels.
    const float searchArea = searchRadius;
    const float halfSearchArea = searchArea / 2;
        
    // camera image level to search.
    cv::Mat camImage;
    cv::Mat homog;
    cv::Mat previousHomog;
    
    switch (resolution) {
        case CameraImage::CameraImageFull:
            camImage = currentCameraImage->getFull();
            homog = homography->getFullHomography();
            if (previousHomography != nullptr) {
                previousHomog = previousHomography->getFullHomography();
            }
            break;
        case CameraImage::CameraImageHalf:
            camImage = currentCameraImage->getHalf();
            homog = homography->getHalfHomography();
            if (previousHomography != nullptr) {
                previousHomog = previousHomography->getHalfHomography();
            }
            break;
        case CameraImage::CameraImageQuarter:
            camImage = currentCameraImage->getQuarter();
            homog = homography->getQuarterHomography();
            if (previousHomography != nullptr) {
                previousHomog = previousHomography->getQuarterHomography();
            }
            break;
        default:
            assert(0);
    }
    
    // some points may not be within the bounds of the camera image. add more until we have enough.
    int camWidth = camImage.cols;
    int camHeight = camImage.rows;
    
    const int pointsRequired = patchesToSearch;
    const int pointsPerLoop = pointsRequired * 1.5;
    
    std::vector<cv::Point2f> keptPointsInMarker;
    std::vector<cv::Point2f> keptWarpedPoints;
    
    keptPointsInMarker.reserve(pointsRequired);
    keptWarpedPoints.reserve(pointsRequired);

    cv::Mat homographyInverse = homog.inv();
    
    int offset = 0;
    while (true) {
        std::vector<cv::Point2f> points;
        std::vector<cv::KeyPoint> &keypointSource = marker->trackingImages[1].keypoints;
        
        for (int i = 0; i < pointsPerLoop && offset < keypointSource.size(); i++, offset++) {
            cv::KeyPoint &keypoint = keypointSource[offset];
            points.push_back(keypoint.pt);
        }
        
        // warp the points from marker space to camera space.
        std::vector<cv::Point2f> warpedPoints;
        
        if (points.size() == 0) {
            break;
        }
        cv::perspectiveTransform(points, warpedPoints, homog);
        
        std::vector<cv::Point2f> withinCamera;
        std::vector<cv::Point2f> withinCameraMarker;
        
        withinCamera.reserve(pointsPerLoop);
        withinCameraMarker.reserve(pointsPerLoop);

        std::vector<cv::Point2f> withinCameraCorners;
        withinCameraCorners.reserve(pointsPerLoop * 4);
        
        for (int i = 0; i < warpedPoints.size(); i++) {
            cv::Point2f &point = warpedPoints[i];
            
            // ensure the point's search area is within the camera bounds.
            if (point.x - halfSearchArea < 0 || point.y - halfSearchArea < 0) {
                continue;
            }
            
            if (point.x + halfSearchArea >= camWidth || point.y + halfSearchArea >= camHeight) {
                continue;
            }
            
            withinCameraMarker.push_back(points[i]);
            withinCamera.push_back(point);
            
            // the patch's corners.
            cv::Point2f topLeft(point.x - halfPatchArea, point.y - halfPatchArea);
            cv::Point2f topRight(point.x + halfPatchArea, point.y - halfPatchArea);
            cv::Point2f bottomLeft(point.x - halfPatchArea, point.y + halfPatchArea);
            cv::Point2f bottomRight(point.x + halfPatchArea, point.y + halfPatchArea);
            
            withinCameraCorners.push_back(topLeft);
            withinCameraCorners.push_back(topRight);
            withinCameraCorners.push_back(bottomLeft);
            withinCameraCorners.push_back(bottomRight);
        }
        
        // unproject the patch's corners back to marker space.
        if (withinCameraCorners.size() == 0) {
            continue;
        }
        cv::perspectiveTransform(withinCameraCorners, withinCameraCorners, homographyInverse);
        
        // ensure patch is within marker bounds.
        for (int i = 0; i < withinCamera.size(); i++) {
            bool failed = false;
            for (int j = 0; j < 4; j++) {
                cv::Point2f &point = withinCameraCorners[i * 4 + j];
                if (point.x < 0 || point.y < 0) {
                    failed = true;
                    break;
                }
                if (point.x >= marker->image.size().width || point.y >= marker->image.size().height) {
                    failed = true;
                    break;
                }
            }
            if (failed == true) {
                continue;
            }
            
            keptPointsInMarker.push_back(withinCameraMarker[i]);
            keptWarpedPoints.push_back(withinCamera[i]);
        }
        
        
        if (keptWarpedPoints.size() > pointsRequired || offset >= keypointSource.size()) {
            break;
        }
    }
    
    if (keptWarpedPoints.size() > pointsRequired) {
        keptWarpedPoints.resize(pointsRequired);
        keptPointsInMarker.resize(pointsRequired);
    }
    
    int pointsFound = 0;
    
    // matching points in both camera image space and marker space.
    std::vector<cv::Point2f> foundPointsInImage;
    std::vector<cv::Point2f> foundPointsInMarker;
    
    cv::Mat previousHomographyInverse;
    
    bool havePreviousHomography = false;
    if (previousHomog.empty() == false) {
        previousHomographyInverse = previousHomog.inv();
        havePreviousHomography = true;
    }
    
    std::vector<float> matchScores;
    
    std::vector<cv::Point2f> projectedMarkerPoints;
    
    std::vector<uint8_t> pointWasFound;
    std::vector<cv::Point2f> pointLocationInImage;
    std::vector<cv::Point2f> pointLocationInMarker;
    
    pointWasFound.resize(keptWarpedPoints.size());
    pointLocationInImage.resize(keptWarpedPoints.size());
    pointLocationInMarker.resize(keptWarpedPoints.size());
    
    // loop
    Profiler profiler;
    
    profiler.start();
    
    cv::parallel_for_(cv::Range(0, (int)keptWarpedPoints.size()), ParallelMatch(&keptWarpedPoints[0], havePreviousHomography, homographyInverse, previousHomographyInverse, marker, matchThreshold, camImage, &pointWasFound[0], &pointLocationInImage[0], &pointLocationInMarker[0]));
    
    profiler.end();
        
    std::vector<cv::Point2f> tmpPointsInImage;
    std::vector<cv::Point2f> tmpPointsInMarker;
    
    for (int i = 0; i < pointWasFound.size(); i++) {
        if (pointWasFound[i]) {
            tmpPointsInImage.push_back(pointLocationInImage[i]);
            tmpPointsInMarker.push_back(pointLocationInMarker[i]);
            pointsFound++;
        }
    }
    
    if (pointsFound < 10) {
        return false;
    }
    
    std::vector<unsigned char> inliersMask(tmpPointsInImage.size());

    
    float reprojectionThreshold = 0;
    
    // reprojection threshold is based on the camera resolution. higher resolution = pixels are smaller = higher reprojection error isn't as noticable.
    switch (resolution) {
        case CameraImage::CameraImageFull:
            reprojectionThreshold = 4;
            break;
        case CameraImage::CameraImageHalf:
            reprojectionThreshold = 2;
            break;
        case CameraImage::CameraImageQuarter:
            reprojectionThreshold = 1;
            break;
    }
    
    homog = cv::findHomography(tmpPointsInMarker, tmpPointsInImage, CV_RANSAC, reprojectionThreshold, inliersMask, 100);

    if (homog.empty()) {
        return false;
    }
    

    
    // count inliers.
    int numberOfInliers = 0;
    
    std::vector<cv::Point2f> keptInMarker;
    std::vector<cv::Point2f> keptInCamera;
    std::vector<cv::Point2f> keptInMarkerOriginal;
    
    keptInMarker.reserve(tmpPointsInMarker.size());
    keptInCamera.reserve(tmpPointsInImage.size());
    keptInMarkerOriginal.reserve(inliersMask.size());
    
    for (int i = 0; i < inliersMask.size(); i++) {
        if (inliersMask[i]) {
            numberOfInliers++;
            keptInMarker.push_back(tmpPointsInMarker[i]);
            keptInCamera.push_back(tmpPointsInImage[i]);
            keptInMarkerOriginal.push_back(tmpPointsInMarker[i]);
        }
    }
    
    
    int minimum = 10;
    int ratioOfAttempts = keptWarpedPoints.size() * 0.25;
    
    if (ratioOfAttempts > minimum) {
        minimum = ratioOfAttempts;
    }
    
    if (numberOfInliers < minimum) {
        return false;
    }

    foundPointsInMarker = keptInMarkerOriginal;
    
    std::vector<cv::Point2f> projected;
    cv::perspectiveTransform(foundPointsInMarker, projected, homog);
    
    foundPointsInImage = projected;
    
    homography = std::make_shared<Homography>();
    
    // homography was calculated at the chosen resolution.
    switch (resolution) {
        case CameraImage::CameraImageFull:
            homography->setFullHomography(homog);
            patchPointsInCamera.setFullPoints(foundPointsInImage);
            break;
        case CameraImage::CameraImageHalf:
            homography->setHalfHomography(homog);
            patchPointsInCamera.setHalfPoints(foundPointsInImage);
            break;

        case CameraImage::CameraImageQuarter:
            homography->setQuarterHomography(homog);
            patchPointsInCamera.setQuarterPoints(foundPointsInImage);
            break;
        default:
            assert(0);
    }
    
    if (debug) {
        debugState->patchPointsFailed = failedPatchPointsInCamera;
        debugState->patchPointsSucceeded = patchPointsInCamera;
    }
    
    patchPointsInMarker = foundPointsInMarker;
    
    return true;
}


bool PatchMatcher::matchOnly(CameraImage::CameraResolution resolution, std::vector<cv::Point2f> &matchedMarkerPoints, std::vector<cv::Point2f> &matchedImagePoints)
{
    printlog(LOG_PATCH,"Patch matching on image %i \n", currentCameraImage->timestamp );
    
    
    // width of the patch in pixels.
    const float patchArea = 8;
    const float halfPatchArea = patchArea / 2;
    
    // width of the search area in pixels.
    const float searchArea = searchRadius;
    const float halfSearchArea = searchArea / 2;
    
    // camera image level to search.
    cv::Mat camImage;
    cv::Mat homog;
    cv::Mat previousHomog;
    
    switch (resolution) {
        case CameraImage::CameraImageFull:
            camImage = currentCameraImage->getFull();
            homog = homography->getFullHomography();
            if (previousHomography != nullptr) {
                previousHomog = previousHomography->getFullHomography();
            }
            break;
        case CameraImage::CameraImageHalf:
            camImage = currentCameraImage->getHalf();
            homog = homography->getHalfHomography();
            if (previousHomography != nullptr) {
                previousHomog = previousHomography->getHalfHomography();
            }
            break;
        case CameraImage::CameraImageQuarter:
            camImage = currentCameraImage->getQuarter();
            homog = homography->getQuarterHomography();
            if (previousHomography != nullptr) {
                previousHomog = previousHomography->getQuarterHomography();
            }
            break;
        default:
            assert(0);
    }
    
    // some points may not be within the bounds of the camera image. add more until we have enough.
    int camWidth = camImage.cols;
    int camHeight = camImage.rows;
    
    const int pointsRequired = patchesToSearch;
    const int pointsPerLoop = pointsRequired * 1.5;
    
    std::vector<cv::Point2f> keptPointsInMarker;
    std::vector<cv::Point2f> keptWarpedPoints;
    
    keptPointsInMarker.reserve(pointsRequired);
    keptWarpedPoints.reserve(pointsRequired);
    
    cv::Mat homographyInverse = homog.inv();
    
    int offset = 0;
    while (true) {
        std::vector<cv::Point2f> points;
        std::vector<cv::KeyPoint> &keypointSource = marker->trackingImages[1].keypoints;
        
        for (int i = 0; i < pointsPerLoop && offset < keypointSource.size(); i++, offset++) {
            cv::KeyPoint &keypoint = keypointSource[offset];
            points.push_back(keypoint.pt);
        }
        
        // warp the points from marker space to camera space.
        std::vector<cv::Point2f> warpedPoints;
        
        if (points.size() == 0) {
            break;
        }
        cv::perspectiveTransform(points, warpedPoints, homog);
        
        std::vector<cv::Point2f> withinCamera;
        std::vector<cv::Point2f> withinCameraMarker;
        
        withinCamera.reserve(pointsPerLoop);
        withinCameraMarker.reserve(pointsPerLoop);
        
        std::vector<cv::Point2f> withinCameraCorners;
        withinCameraCorners.reserve(pointsPerLoop * 4);
        
        for (int i = 0; i < warpedPoints.size(); i++) {
            cv::Point2f &point = warpedPoints[i];
            
            // ensure the point's search area is within the camera bounds.
            if (point.x - halfSearchArea < 0 || point.y - halfSearchArea < 0) {
                continue;
            }
            
            if (point.x + halfSearchArea >= camWidth || point.y + halfSearchArea >= camHeight) {
                continue;
            }
            
            withinCameraMarker.push_back(points[i]);
            withinCamera.push_back(point);
            
            // the patch's corners.
            cv::Point2f topLeft(point.x - halfPatchArea, point.y - halfPatchArea);
            cv::Point2f topRight(point.x + halfPatchArea, point.y - halfPatchArea);
            cv::Point2f bottomLeft(point.x - halfPatchArea, point.y + halfPatchArea);
            cv::Point2f bottomRight(point.x + halfPatchArea, point.y + halfPatchArea);
            
            withinCameraCorners.push_back(topLeft);
            withinCameraCorners.push_back(topRight);
            withinCameraCorners.push_back(bottomLeft);
            withinCameraCorners.push_back(bottomRight);
        }
        
        // unproject the patch's corners back to marker space.
        if (withinCameraCorners.size() == 0) {
            continue;
        }
        cv::perspectiveTransform(withinCameraCorners, withinCameraCorners, homographyInverse);
        
        // ensure patch is within marker bounds.
        for (int i = 0; i < withinCamera.size(); i++) {
            bool failed = false;
            for (int j = 0; j < 4; j++) {
                cv::Point2f &point = withinCameraCorners[i * 4 + j];
                if (point.x < 0 || point.y < 0) {
                    failed = true;
                    break;
                }
                if (point.x >= marker->image.size().width || point.y >= marker->image.size().height) {
                    failed = true;
                    break;
                }
            }
            if (failed == true) {
                continue;
            }
            
            keptPointsInMarker.push_back(withinCameraMarker[i]);
            keptWarpedPoints.push_back(withinCamera[i]);
        }
        
        
        if (keptWarpedPoints.size() > pointsRequired || offset >= keypointSource.size()) {
            break;
        }
    }
    
    if (keptWarpedPoints.size() > pointsRequired) {
        keptWarpedPoints.resize(pointsRequired);
        keptPointsInMarker.resize(pointsRequired);
    }
    
    int pointsFound = 0;
    
    // matching points in both camera image space and marker space.
    std::vector<cv::Point2f> foundPointsInImage;
    std::vector<cv::Point2f> foundPointsInMarker;
    
    cv::Mat previousHomographyInverse;
    
    bool havePreviousHomography = false;
    if (previousHomog.empty() == false) {
        previousHomographyInverse = previousHomog.inv();
        havePreviousHomography = true;
    }
    
    std::vector<float> matchScores;
    
    std::vector<cv::Point2f> projectedMarkerPoints;
    
    std::vector<uint8_t> pointWasFound;
    std::vector<cv::Point2f> pointLocationInImage;
    std::vector<cv::Point2f> pointLocationInMarker;
    
    pointWasFound.resize(keptWarpedPoints.size());
    pointLocationInImage.resize(keptWarpedPoints.size());
    pointLocationInMarker.resize(keptWarpedPoints.size());
    
    // loop
    Profiler profiler;
    
    profiler.start();
    
    cv::parallel_for_(cv::Range(0, (int)keptWarpedPoints.size()), ParallelMatch(&keptWarpedPoints[0], havePreviousHomography, homographyInverse, previousHomographyInverse, marker, matchThreshold, camImage, &pointWasFound[0], &pointLocationInImage[0], &pointLocationInMarker[0]));
    
    profiler.end();
    
    std::vector<cv::Point2f> tmpPointsInImage;
    std::vector<cv::Point2f> tmpPointsInMarker;
    
    for (int i = 0; i < pointWasFound.size(); i++) {
        if (pointWasFound[i]) {
            tmpPointsInImage.push_back(pointLocationInImage[i]);
            tmpPointsInMarker.push_back(pointLocationInMarker[i]);
            pointsFound++;
        }
    }
    
    if (pointsFound < 10) {
        return false;
    }
    
    std::vector<unsigned char> inliersMask(tmpPointsInImage.size());
    
    
    float reprojectionThreshold = 0;
    
    // reprojection threshold is based on the camera resolution. higher resolution = pixels are smaller = higher reprojection error isn't as noticable.
    switch (resolution) {
        case CameraImage::CameraImageFull:
            reprojectionThreshold = 4;
            break;
        case CameraImage::CameraImageHalf:
            reprojectionThreshold = 2;
            break;
        case CameraImage::CameraImageQuarter:
            reprojectionThreshold = 1;
            break;
    }
    
    homog = cv::findHomography(tmpPointsInMarker, tmpPointsInImage, CV_RANSAC, reprojectionThreshold, inliersMask, 100);

    
    if (homog.empty()) {
        return false;
    }
    
    
    
    // count inliers.
    int numberOfInliers = 0;
    
    std::vector<cv::Point2f> keptInMarker;
    std::vector<cv::Point2f> keptInCamera;
    std::vector<cv::Point2f> keptInMarkerOriginal;
    
    keptInMarker.reserve(tmpPointsInMarker.size());
    keptInCamera.reserve(tmpPointsInImage.size());
    keptInMarkerOriginal.reserve(inliersMask.size());
    
    for (int i = 0; i < inliersMask.size(); i++) {
        if (inliersMask[i]) {
            numberOfInliers++;
            keptInMarker.push_back(tmpPointsInMarker[i]);
            keptInCamera.push_back(tmpPointsInImage[i]);
            keptInMarkerOriginal.push_back(tmpPointsInMarker[i]);
        }
    }
    
    
    int minimum = 10;
    int ratioOfAttempts = keptWarpedPoints.size() * 0.25;
    
    if (ratioOfAttempts > minimum) {
        minimum = ratioOfAttempts;
    }
    
    if (numberOfInliers < minimum) {
        return false;
    }
    

    
    std::vector<cv::Point2f> projected;
    cv::perspectiveTransform(keptInMarkerOriginal, projected, homog);
    
    matchedMarkerPoints = keptInMarkerOriginal;
    matchedImagePoints = projected;

    
    
    
    return true;
}

NamespaceEnd
