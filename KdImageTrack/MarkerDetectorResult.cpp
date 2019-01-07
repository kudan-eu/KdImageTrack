//
//  MarkerDetectorResult.cpp
//  KdImageTrack
//
//  Created on 17/11/2016.
//  Copyright © 2016 Kudan. All rights reserved.
//

#include "MarkerDetectorResult.hpp"
#include "Marker.h"

#include "Homography.h"

Namespace(KdImageTrack)


MarkerDetectorResult::MarkerDetectorResult(std::shared_ptr<Marker> m)
{
    homography = std::make_shared<Homography>();
    
    marker = m;
    
    matchesAfterHammingFilter = 0;
    matchesAfterOrientationFilter = 0;
    matchesAfterHomographyFilter = 0;
    
    fromFlowRecovery = false;
}

// There needs to be some way of deciding between markers, and there might be many. For now, use num inilers (remaining matches)

float MarkerDetectorResult::getScore()
{
    return matches.size();
}


NamespaceEnd
