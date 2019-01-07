//
//  DebugState.h
//  PlanarTracker
//
//  Created on 16/03/2015.
//  Copyright (c) 2015 Kudan. All rights reserved.
//

#ifndef __PlanarTracker__DebugState__
#define __PlanarTracker__DebugState__

#include <stdio.h>
#include <memory>
#include <opencv2/opencv.hpp>
#include "CameraImage.h"
#include "ImagePoints.h"
#include "Macros.h"

Namespace(KdImageTrack)

class DebugStatePatch;

// the state of a tracked frame.
class DebugState
{
public:
    std::vector<DebugStatePatch> patches;
    std::shared_ptr<CameraImage> cameraImage;
    
    ImagePoints patchPointsFailed;
    ImagePoints patchPointsSucceeded;
    
    DebugState()
    {
        
    }
};

class DebugStatePatch
{
public:
    cv::Mat searchArea;
    cv::Mat warpedPatch;
    cv::Mat warpedPatchNoBlur;
    
    float matchScore;
    int variance;
    bool subpixelLocalised;
    
    cv::Point2f matchPositionInSearchArea;
    cv::Point2f matchPositionInCamera;
    cv::Point2f affineInMarker;
    
    DebugStatePatch()
    {
        matchScore = 0;
        variance = 0;
        subpixelLocalised = false;
    }
};

NamespaceEnd

#endif /* defined(__PlanarTracker__DebugState__) */
