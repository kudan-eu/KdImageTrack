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

#include "PoseEstimator.h"
#include "Geometry.hpp"
#include "Logging.hpp"
#include "glm/glm.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "glm/gtc/quaternion.hpp"

Namespace(KdImageTrack)

void PoseEstimator::setIntrinsics(float focalX, float focalY, float principalX, float principalY)
{
    cameraIntrinsics = cv::Mat::eye(3, 3, CV_32F);
    
    cameraIntrinsics.at<float>(0, 0) = focalX;
    cameraIntrinsics.at<float>(1, 1) = focalY;
    cameraIntrinsics.at<float>(0, 2) = principalX;
    cameraIntrinsics.at<float>(1, 2) = principalY;
}

void PoseEstimator::estimatePose(glm::vec3 &position, glm::quat &orientation)
{
    // Note: now using the actual image size in 3D and in 2D
    // They still have separate cariables in case on needs to be changed or scaled later
    
    // corners of marker in marker image space. This is the boundary you want to have displayed (so after any cropping and scaling)
    std::vector<cv::Point2f> corners2d = marker->displayBoundary;
    
    // corners of marker in camera image space. Simpply project the above by the current tracking homography
    std::vector<cv::Point2f> cameraPoints;
    cv::perspectiveTransform(corners2d, cameraPoints, homography->getFullHomography());
    
    
    // corners of marker in 3D marker space. These are simply the virtual corners (like the tracked corners but offset according to any cropping), centred about the origin, on the XY plane
    
    std::vector<cv::Point3f> corners3d;
    // Get the virtual boundary mean
    cv::Point2f displayCentre(0,0);
    for (auto &d : marker->virtualBoundary) {
        displayCentre += d;
    }
    displayCentre /= (float)marker->virtualBoundary.size();
    
    // Then subtract the mean, and put at z=0
    for (auto &d : marker->virtualBoundary) {
        corners3d.push_back( cv::Point3f(d.x - displayCentre.x, d.y - displayCentre.y, 0));
    }
    
    
    // Note: in this coordinate frame, Y is down (when Z is away from you). This matches the image convention (top left is 0,0, so positive Y goes down), i.e. a right handed coordinate system
   // The coordinate frame handedness could be changed by re-order in the 3D points
    
    
    // solve.
    cv::Mat_<double> rvec, tvec;
    cv::solvePnP(corners3d, cameraPoints, cameraIntrinsics, std::vector<float>(), rvec, tvec);

    getPoseFromVectors(rvec, tvec, position, orientation);

    return;
}

NamespaceEnd
