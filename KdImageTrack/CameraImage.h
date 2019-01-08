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

#ifndef __MarkerTracker__CameraImage__
#define __MarkerTracker__CameraImage__


#include <vector>
#include <opencv2/opencv.hpp>
#include "Macros.h"

Namespace(KdImageTrack)

// provides full, half and quarter versions of the current camera frame (with widths 640,320,160)
class CameraImage
{
private:
    cv::Mat srcImage;
    cv::Mat fullImage;
    cv::Mat halfImage;
    cv::Mat quarterImage;
    
public:
    cv::Mat getFull();
    cv::Mat getHalf();
    cv::Mat getQuarter();
    
    int timestamp;
    
    CameraImage(cv::Mat src, int t = -1);
    CameraImage()
    {
        
    }
    
    typedef enum {
        CameraImageFull,
        CameraImageHalf,
        CameraImageQuarter,
    } CameraResolution;
};

NamespaceEnd

#endif /* defined(__MarkerTracker__CameraImage__) */
