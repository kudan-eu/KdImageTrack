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
