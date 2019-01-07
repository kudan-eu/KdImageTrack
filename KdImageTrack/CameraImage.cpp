//
//  CameraImage.cpp
//  MarkerTracker
//
//  Created on 24/02/2015.
//  Copyright (c) 2015 Kudan. All rights reserved.
//

#include "CameraImage.h"

Namespace(KdImageTrack)

CameraImage::CameraImage(cv::Mat src, int t)
{
    srcImage = src;
    fullImage = src;
    timestamp = t;
}

cv::Mat CameraImage::getFull()
{
    return fullImage;
}

cv::Mat CameraImage::getHalf()
{
    if (halfImage.cols == 0) {
        cv::resize(fullImage, halfImage, cv::Size(fullImage.cols / 2, fullImage.rows / 2));
    }
    return halfImage;
}

cv::Mat CameraImage::getQuarter()
{
    if (quarterImage.cols == 0) {
        getHalf();
        cv::resize(halfImage, quarterImage, cv::Size(halfImage.cols / 2, halfImage.rows / 2));
    }
    return quarterImage;
}

NamespaceEnd
