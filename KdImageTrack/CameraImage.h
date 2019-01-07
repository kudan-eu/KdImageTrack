//
//  CameraImage.h
//  MarkerTracker
//
//  Created on 24/02/2015.
//  Copyright (c) 2015 Kudan. All rights reserved.
//

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
