//
//  Helper.h
//  PlanarTracker
//
//  Created on 03/04/2015.
//  Copyright (c) 2015 Kudan. All rights reserved.
//

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
