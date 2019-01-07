//
//  ImagePoints.h
//  PlanarTracker
//
//  Created on 08/03/2015.
//  Copyright (c) 2015 Kudan. All rights reserved.
//

#ifndef __PlanarTracker__ImagePoints__
#define __PlanarTracker__ImagePoints__

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "Macros.h"

Namespace(KdImageTrack)

class ImagePoints
{
    std::vector<cv::Point2f> fullPoints;
    std::vector<cv::Point2f> halfPoints;
    std::vector<cv::Point2f> quarterPoints;
public:
    
    ImagePoints()
    {
        
    }
    
    void setFullPoints(std::vector<cv::Point2f> points)
    {
        fullPoints = points;
        halfPoints.clear();
        quarterPoints.clear();
    }
    
    void setHalfPoints(std::vector<cv::Point2f> points)
    {
        halfPoints = points;
        fullPoints.clear();
        quarterPoints.clear();
    }
    
    void setQuarterPoints(std::vector<cv::Point2f> points)
    {
        quarterPoints = points;
        fullPoints.clear();
        halfPoints.clear();
    }
    
    std::vector<cv::Point2f> &getFullPoints();
    std::vector<cv::Point2f> &getHalfPoints();
    std::vector<cv::Point2f> &getQuarterPoints();
    
};

NamespaceEnd

#endif /* defined(__PlanarTracker__ImagePoints__) */
