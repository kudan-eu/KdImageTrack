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
