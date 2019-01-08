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

#ifndef MarkerDetectorResult_hpp
#define MarkerDetectorResult_hpp

#include <stdio.h>
#include <memory>
#include <opencv2/opencv.hpp>
#include "glm/glm.hpp"
#include "glm/gtc/quaternion.hpp"
#include "Macros.h"

Namespace(KdImageTrack)

class Marker;
class Homography;

class MarkerDetectorResult
{
public:
    std::shared_ptr<Marker> marker;
    
    // homography of the marker in the full camera image.
    std::shared_ptr<Homography> homography;
    
    // filtered matches.
    std::vector<cv::DMatch> matches;
    
    int matchesAfterHammingFilter;
    int matchesAfterOrientationFilter;
    int matchesAfterHomographyFilter;
    
    glm::vec3 position;
    glm::quat orientation;
    
    bool fromFlowRecovery;
    

    MarkerDetectorResult(std::shared_ptr<Marker> m);
    
    // There needs to be some way of deciding between markers, and there might be many. For now, use num inilers (remaining matches)
    
    float getScore();
};

NamespaceEnd

#endif /* MarkerDetectorResult_hpp */
