//
//  PoseEstimator.h
//  PlanarTracker
//
//  Created on 05/03/2015.
//  Copyright (c) 2015 Kudan. All rights reserved.
//

#ifndef __PlanarTracker__PoseEstimator__
#define __PlanarTracker__PoseEstimator__

#include <stdio.h>
#include <memory>
#include <opencv2/opencv.hpp>
#include "Marker.h"
#include "glm/glm.hpp"
#include "Homography.h"
#include "DoubleExponentialSmoothing.h"
#include "Macros.h"

Namespace(KdImageTrack)

class PoseEstimator
{
private:
    std::shared_ptr<Marker> marker;
    cv::Mat cameraIntrinsics;
    std::shared_ptr<Homography> homography;
    
public:
    bool shouldFilterPose;
    DoubleExponentialSmoothingTracker doubleExponentialSmoothing;
    
    PoseEstimator()
    {
        shouldFilterPose = false;
    }
    
    void setMarker(std::shared_ptr<Marker> m)
    {
        marker = m;
    }
    
    void setHomography(std::shared_ptr<Homography> h)
    {
        homography = h;
    }
    
    void setIntrinsics(float focalX, float focalY, float principalX, float principalY);
    void estimatePose(glm::vec3 &position, glm::quat &orientation);
};

NamespaceEnd

#endif /* defined(__PlanarTracker__PoseEstimator__) */
