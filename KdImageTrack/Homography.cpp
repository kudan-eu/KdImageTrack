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
