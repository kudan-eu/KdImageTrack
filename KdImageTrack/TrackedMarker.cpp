//
//  TrackedMarker.cpp
//  KdImageTrack
//
//  Created on 17/11/2016.
//  Copyright © 2016 Kudan. All rights reserved.
//

#include "TrackedMarker.hpp"
#include "Marker.h"
#include "Homography.h"

Namespace(KdImageTrack)

void TrackedMarker::resetHomography()
{
    prevHomography = nullptr;
    homography = nullptr;
}

void TrackedMarker::setHomography(std::shared_ptr<Homography> h)
{
    prevHomography = homography;
    homography = h;
}

TrackedMarker::TrackedMarker()
{
    
    
    framesInOpticalFlow = 0;
    framesOutOfOpticalFlow = 0;
    
    
    // this should always be created at the same time as the tracked marker. If it's still null anywhere, something has gone wrong
    
    
    opticalFlowTracker = nullptr;
    
    lastPosition = glm::vec3(0, 0, 0);
    lastOrientation = glm::quat(0, 0, 0, 0);
    
    isSmooth = false;
    unreliablePose = false;
    
    timeSinceFlowKeyframe = 0;
    timeSinceFlowRecovery = 0;
    fromFlowRecovery = false;
    lifetime = 0;
}

// This is used for comparing tracked markers (in verification)
float TrackedMarker::getScore()
{
    // return the number of matched inlying points!
    return patchMatcher.patchPointsInMarker.size();
}

NamespaceEnd
