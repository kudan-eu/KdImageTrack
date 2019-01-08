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

#ifndef __PlanarTracker__Homography__
#define __PlanarTracker__Homography__

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "Macros.h"

Namespace(KdImageTrack)

class Homography
{
private:
    cv::Mat fullHomography;
    cv::Mat halfHomography;
    cv::Mat quarterHomography;
    
public:
    Homography()
    {
        
    }
    
    void setFullHomography(cv::Mat homography)
    {
        fullHomography = homography;
        halfHomography = cv::Mat();
        quarterHomography = cv::Mat();
    }
    
    void setHalfHomography(cv::Mat homography)
    {
        halfHomography = homography;
        fullHomography = cv::Mat();
        quarterHomography = cv::Mat();
    }
    
    void setQuarterHomography(cv::Mat homography)
    {
        quarterHomography = homography;
        fullHomography = cv::Mat();
        halfHomography = cv::Mat();
    }
    
    cv::Mat getFullHomography();
    cv::Mat getHalfHomography();
    cv::Mat getQuarterHomography();
};

NamespaceEnd

#endif /* defined(__PlanarTracker__Homography__) */
