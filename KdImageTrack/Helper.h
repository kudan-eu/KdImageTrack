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

#ifndef __Helper__
#define __Helper__

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "glm/glm.hpp"
#include "CameraCalibration.h"
#include "Macros.h"

Namespace(KdImageTrack)


cv::Mat lerpHomography(cv::Mat start, cv::Mat end, float t);

cv::Mat lerpAffine(cv::Mat start, cv::Mat end, float t);

float lerp(float a, float b, float t);

glm::mat4 getProjectionMatrix(float width, float height, float focalX, float focalY, float prinX, float prinY, float nearPlane, float farPlane);

NamespaceEnd

// Not defined on some platforms
#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

#ifdef __ANDROID__
namespace std
{
    // for Android, where string functions are not available!
    template < typename T >
    std::string to_string( const T& n )
    {
        std::ostringstream ss;
        ss << n;
        return ss.str() ;
    }
    
    int stoi( const std::string& s );
}
#endif

#endif /* defined(__Helper__) */
