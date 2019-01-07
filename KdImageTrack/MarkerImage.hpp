//
//  MarkerImage.hpp
//  KdImageTrack
//
//  Created on 17/11/2016.
//  Copyright © 2016 Kudan. All rights reserved.
//

#ifndef MarkerImage_hpp
#define MarkerImage_hpp

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "Macros.h"

Namespace(KdImageTrack)

class MarkerImage
{
public:
    int level;
    cv::Mat image;
    std::vector<cv::KeyPoint> keypoints;
    cv::Size scale;
    
    MarkerImage()
    {
        
    }
};

NamespaceEnd

#endif /* MarkerImage_hpp */
