//
//  ImageProcessing.hpp
//  VisionDev-demo
//
//  Created on 19/01/2016.
//  Copyright © 2016 Kudan. All rights reserved.
/* This is a file for storing various image processing functions, such as custom types of resize, concatenation, etc. */


#ifndef ImageProcessing_hpp
#define ImageProcessing_hpp

#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "Macros.h"

Namespace(KdImageTrack)
   
cv::Scalar averageColour(cv::Mat &src, cv::Size targetSize);

NamespaceEnd

#endif /* ImageProcessing_hpp */
