//
//  HomographyFinding.hpp
//  VisionDev-demo
//
//  Created on 16/03/2016.
//  Copyright © 2016 Kudan. All rights reserved.
//

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
