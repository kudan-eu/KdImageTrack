//
//  ImageProcessing.cpp
//  VisionDev-demo
//
//  Created on 19/01/2016.
//  Copyright © 2016 Kudan. All rights reserved.
//

#include "ImageProcessing.hpp"
#include "Logging.hpp"

Namespace(KdImageTrack)

cv::Scalar averageColour(cv::Mat &src, cv::Size targetSize)
{
    cv::Mat downsampledImage;
    
    cv::resize(src, downsampledImage, targetSize);
    
    cv::Scalar averageColour = cv::mean(downsampledImage);
        
    return averageColour;
}

NamespaceEnd
