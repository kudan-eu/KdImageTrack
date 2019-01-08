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
