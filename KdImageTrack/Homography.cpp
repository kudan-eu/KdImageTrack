//
//  Homography.cpp
//  PlanarTracker
//
//  Created on 08/03/2015.
//  Copyright (c) 2015 Kudan. All rights reserved.
//

#include "Homography.h"

Namespace(KdImageTrack)

static cv::Mat scale3x3Mat(cv::Mat mat, float scale)
{
    cv::Mat scaleMat(3, 3, CV_64F);
    float v[] = {
        scale, 0, 0,
        0, scale, 0,
        0, 0, 1
    };
    
    double *data = (double *)scaleMat.data;
    for (int i = 0; i < 9; i++) {
        data[i] = v[i];
    }
    
    return scaleMat * mat;
}

cv::Mat Homography::getFullHomography()
{
    if (fullHomography.empty()) {
        // check to see which homography we have.
        if (halfHomography.empty() == false) {
            fullHomography = scale3x3Mat(halfHomography, 2);
        } else if (quarterHomography.empty() == false) {
            fullHomography = scale3x3Mat(quarterHomography, 4);
        }
    }
    
    return fullHomography;
}

cv::Mat Homography::getHalfHomography()
{
    if (halfHomography.empty()) {
        // check to see which homography we have.
        if (fullHomography.empty() == false) {
            halfHomography = scale3x3Mat(fullHomography, 0.5);
        } else if (quarterHomography.empty() == false) {
            halfHomography = scale3x3Mat(quarterHomography, 2);
        }
    }
    
    return halfHomography;
}

cv::Mat Homography::getQuarterHomography()
{
    if (quarterHomography.empty()) {
        // check to see which homography we have.
        if (fullHomography.empty() == false) {
            quarterHomography = scale3x3Mat(fullHomography, 0.25);
        } else if (halfHomography.empty() == false) {
            quarterHomography = scale3x3Mat(halfHomography, 0.5);
        }
    }
    
    return quarterHomography;
}

NamespaceEnd
