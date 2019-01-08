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
