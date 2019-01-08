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

#include "ImageProcessing.hpp"
#include "Logging.hpp"

Namespace(KdImageTrack)

cv::Scalar averageColour(cv::Mat &src, cv::Size targetSize)
{
    cv::Mat downsampledImage;
    
    cv::resize(src, downsampledImage, targetSize);
    
    cv::Scalar averageColour = cv::mean(downsampledImage);
        
    return averageColour;
}

NamespaceEnd
