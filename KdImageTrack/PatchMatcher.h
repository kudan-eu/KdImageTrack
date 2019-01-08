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

#ifndef __PlanarTracker__PatchMatcher__
#define __PlanarTracker__PatchMatcher__

#include <stdio.h>
#include "Marker.h"
#include "CameraImage.h"
#include "Homography.h"
#include "ImagePoints.h"
#include "DebugState.h"
#include "Macros.h"
#include <memory>

Namespace(KdImageTrack)


class PatchMatcher
{
private:
    std::shared_ptr<CameraImage> currentCameraImage;
    std::shared_ptr<Marker> marker;
    std::shared_ptr<Homography> homography;
    
public:
    CameraImage::CameraResolution resolution;
    
    int patchesToSearch;
    
    bool debug;
    std::shared_ptr<DebugState> debugState;

    std::shared_ptr<Homography> previousHomography;

    int searchRadius;
    float matchThreshold;

    ImagePoints patchPointsInCamera;
    ImagePoints failedPatchPointsInCamera;
    
    std::vector<cv::Point2f> patchPointsInMarker;
    
    void setCameraImage(std::shared_ptr<CameraImage> cameraImage)
    {
        currentCameraImage = cameraImage;
    }
    
    void setMarker(std::shared_ptr<Marker> m)
    {
        marker = m;
    }
    
    PatchMatcher()
    {
        homography = std::make_shared<Homography>();
        previousHomography = std::make_shared<Homography>();
        debugState = nullptr;
        marker = nullptr;
        searchRadius = 16;
        matchThreshold = 0.65;
        debug = false;
        patchesToSearch = 100;
        
        resolution = CameraImage::CameraImageQuarter;
    }
    
    void setHomography(const std::shared_ptr<Homography> h)
    {
        previousHomography = homography;
        homography = h;
    }
    
    std::shared_ptr<Homography> getHomography()
    {
        return homography;
    }
    
    bool match();
    
    /* Do the matching, but don't alter any state - return the matched points (at whatever resolution is being used) */
    bool matchOnly(CameraImage::CameraResolution resolution, std::vector<cv::Point2f> &matchedMarkerPoints, std::vector<cv::Point2f> &matchedImagePoints) ;
};

NamespaceEnd

#endif /* defined(__PlanarTracker__PatchMatcher__) */
