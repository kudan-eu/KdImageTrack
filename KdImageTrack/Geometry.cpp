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

#include "Geometry.hpp"
#include <opencv2/calib3d.hpp>

Namespace(KdImageTrack)

cv::Point2f projectWithH(cv::Point2f pt , cv::Mat H)
{
    cv::Point2f ppt;
    
    // this is extension of 2D point to homogeneous point, multiplication by the matrix H, then renormalisation bby z-coordinate, done manually without conversion to matrixes"
    
    double z = pt.x * H.at<double>(2, 0) + pt.y * H.at<double>(2, 1) + H.at<double>(2, 2);
    
    ppt.x = (pt.x * H.at<double>(0, 0) + pt.y * H.at<double>(0, 1) + H.at<double>(0, 2)) / z;
    ppt.y = (pt.x * H.at<double>(1, 0) + pt.y * H.at<double>(1, 1) + H.at<double>(1, 2)) / z;
    
    
    return ppt;
}

void getPoseFromVectors(cv::Mat rvec, cv::Mat tvec, glm::vec3 &position, glm::quat &orientation)
{
        
    // get the camera T and interpret this as the position:
    
    position.x = tvec.at<double>(0,0);
    position.y = tvec.at<double>(1,0);
    position.z = tvec.at<double>(2,0);
    
    // get rotation as a 3x3 rotation matrix
    cv::Mat rot;
    cv::Rodrigues(rvec, rot);
    
    // Convert to GLM
    glm::mat3 R;
    
    for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 3; col++) {
            // transpose, or just the ordering of glm vs OpenCV
            R[col][row] = rot.at<double>(row, col);
        }
    }
    
    // rotation matrix to quaternion
    orientation = glm::quat_cast(R);
}

NamespaceEnd
