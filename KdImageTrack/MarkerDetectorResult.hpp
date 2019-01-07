//
//  MarkerDetectorResult.hpp
//  KdImageTrack
//
//  Created on 17/11/2016.
//  Copyright © 2016 Kudan. All rights reserved.
//

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
