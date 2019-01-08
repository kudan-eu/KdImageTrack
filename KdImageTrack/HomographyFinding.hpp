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

#ifndef HomographyFinding_hpp
#define HomographyFinding_hpp

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "Macros.h"

Namespace(KdImageTrack)

#define HYP_SIZE 4



/* Attemtps to find the Homography which projects srcPoints onto dstPoints. Uses RANSAC.
 Returns an empty matrix if none can be found.
 This matrix will have DOUBLE type, i.e. CV_64F. Despite the fact its input are 2D float points. */
cv::Mat findHomographyRANSAC(std::vector<cv::Point2f> &srcPoints, std::vector<cv::Point2f> &dstPoints, float reprojectionThreshold, int maxIterations, bool doFinalSolution, std::vector<unsigned char> &inlierMask, int &numIteratopns);

NamespaceEnd

#endif /* HomographyFinding_hpp */
