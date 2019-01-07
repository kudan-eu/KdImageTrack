//
//  Geometry.cpp
//  VisionDev-demo
//
//  Created on 19/01/2016.
//  Copyright © 2016 Kudan. All rights reserved.
//

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
