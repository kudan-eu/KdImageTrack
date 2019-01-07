//
//  Homography.h
//  PlanarTracker
//
//  Created on 08/03/2015.
//  Copyright (c) 2015 Kudan. All rights reserved.
//

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
