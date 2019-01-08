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
#ifndef TrackedMarker_hpp
#define TrackedMarker_hpp

#include <stdio.h>
#include <memory>
#include "PatchMatcher.h"


#include "glm/glm.hpp"
#include "glm/gtc/quaternion.hpp"
#include "Macros.h"

Namespace(KdImageTrack)

class Marker;
class Homography;
class OpticalFlowTracker;

class TrackedMarker
{
public:
    std::shared_ptr<Marker> marker;
    std::shared_ptr<Homography> homography;
    std::shared_ptr<Homography> weightedHomography;
    std::shared_ptr<OpticalFlowTracker> opticalFlowTracker;
    
    PatchMatcher patchMatcher;
    
    
    // Current tracked marker position:
    glm::vec3 position;
    glm::quat orientation;
    
    // Position in previous frame, for recovery:
    glm::vec3 lastPosition;
    glm::quat lastOrientation;
    
    // Records whether smoothing was used on this marker or not, i.e. if the motion was low enough for smoothing to be valid
    bool isSmooth;
    
    // Used to be in TrackedMarkerResult. Record whether it is good or not (based on optical flow)
    bool unreliablePose;
    
    // These keep track of the state of a tracked marker
    int lifetime;                   // Counter of how many times it has been tracked: useful in flow recovery
    int framesInOpticalFlow;
    int framesOutOfOpticalFlow;
    
    
    std::shared_ptr<Homography> prevHomography;
    
    int timeSinceFlowKeyframe;
    int timeSinceFlowRecovery;
    
    int fromFlowRecovery;
    
    
    
    void resetHomography();
    
    void setHomography(std::shared_ptr<Homography> h);
    
    TrackedMarker();
    
    // This is used for comparing tracked markers (in verification)
    float getScore();
    
    
};

NamespaceEnd

#endif /* TrackedMarker_hpp */
