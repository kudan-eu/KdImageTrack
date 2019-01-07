//
//  Geometry.hpp
//  VisionDev-demo
//
//  Created on 19/01/2016.
//  Copyright © 2016 Kudan. All rights reserved.
//

#ifndef Geometry_hpp
#define Geometry_hpp

#include <stdio.h>

#include <opencv2/core/core.hpp>
#include "glm/glm.hpp"
#include "glm/gtc/quaternion.hpp"

#include "Macros.h"

Namespace(KdImageTrack)

cv::Point2f projectWithH(cv::Point2f pt , cv::Mat H);

// Given a pose from OpenCV (as given by solvePnP), represetned by an exponential map 3-vector and a 3D translation, get the position and orientation as a GLM vctor and quaternion
void getPoseFromVectors(cv::Mat rvec, cv::Mat tvec, glm::vec3 &position, glm::quat &orientation);

NamespaceEnd
    
#endif /* Geometry_hpp */
