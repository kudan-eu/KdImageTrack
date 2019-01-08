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
