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
#include "Helper.h"
#include "glm/gtc/type_ptr.hpp"

Namespace(KdImageTrack)


cv::Mat lerpHomography(cv::Mat start, cv::Mat end, float t)
{
    cv::Mat result = (1 - t) * start + t * end;
    return result;
}

float lerp(float a, float b, float t)
{
    return (1 - t) * a + t * b;
}

cv::Mat lerpAffine(cv::Mat start, cv::Mat end, float t)
{
    cv::Mat dst(2, 3, CV_32F);
    
    dst.at<float>(0, 0) = lerp(start.at<float>(0, 0), end.at<float>(0, 0), t);
    dst.at<float>(0, 1) = lerp(start.at<float>(0, 1), end.at<float>(0, 1), t);
    dst.at<float>(0, 2) = lerp(start.at<float>(0, 2), end.at<float>(0, 2), t);
    dst.at<float>(1, 0) = lerp(start.at<float>(1, 0), end.at<float>(1, 0), t);
    dst.at<float>(1, 1) = lerp(start.at<float>(1, 1), end.at<float>(1, 1), t);
    dst.at<float>(1, 2) = lerp(start.at<float>(1, 2), end.at<float>(1, 2), t);
    
    return dst;
}


glm::mat4 getProjectionMatrix(float cameraWidth, float cameraHeight, float focalX, float focalY, float prinX, float prinY, float nearPlane, float farPlane)
{

    
    float m[16] = { 0 };
    m[0] = 2 * focalX / cameraWidth;
    m[5] = 2 * focalY / cameraHeight;
    m[8] = 2 * (prinX / cameraWidth) - 1;
    m[9] = 2 * (prinY / cameraHeight) - 1;
    m[10] = -(farPlane + nearPlane) / (farPlane - nearPlane);
    m[11] = -1;
    m[14] = -2 * farPlane * nearPlane / (farPlane - nearPlane);

    glm::mat4 projection = glm::make_mat4(m);
    return projection;
}

NamespaceEnd

// certain versions of android have this unavailable
// reimplemented here.
#ifdef __ANDROID__

namespace std {
    int stoi( const std::string& s )
    {
        std::istringstream str(s);
        int i;
        str >> i;
        return i;
    }
}

#endif
