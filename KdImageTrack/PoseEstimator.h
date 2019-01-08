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
