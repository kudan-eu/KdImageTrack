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

#ifndef PlanarTracker_CameraCalibration_h
#define PlanarTracker_CameraCalibration_h

#include "Macros.h"

Namespace(KdImageTrack)

class CameraCalibration
{
public:
    float focalX;
    float focalY;
    float principalX;
    float principalY;
    
    CameraCalibration()
    {
        focalX = 0;
        focalY = 0;
        principalX = 0;
        principalY = 0;
    }
    
    CameraCalibration(float fx, float fy, float px, float py)
    {
        focalX = fx;
        focalY = fy;
        principalX = px;
        principalY = py;
    }
    
    cv::Mat getMatrix()
    {
        
        cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
        
        K.at<float>(0, 0) = focalX;
        K.at<float>(1, 1) = focalY;
        K.at<float>(0, 2) = principalX;
        K.at<float>(1, 2) = principalY;
        
        return K;
    }
    
    cv::Mat getMatrix64()
    {
        
        cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
        
        K.at<double>(0, 0) = focalX;
        K.at<double>(1, 1) = focalY;
        K.at<double>(0, 2) = principalX;
        K.at<double>(1, 2) = principalY;
        
        return K;
    }
    
};


NamespaceEnd

#endif
