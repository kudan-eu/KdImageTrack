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

#include "ImagePoints.h"

Namespace(KdImageTrack)

std::vector<cv::Point2f> &ImagePoints::getFullPoints()
{
    if (fullPoints.empty()) {
        // figure out which scale exists.
        if (halfPoints.empty() == false) {
            for (int i = 0; i < halfPoints.size(); i++) {
                cv::Point2f point = halfPoints[i];
                point.x *= 2;
                point.y *= 2;
                
                fullPoints.push_back(point);
            }
        } else if (quarterPoints.empty() == false) {
            for (int i = 0; i < quarterPoints.size(); i++) {
                cv::Point2f point = quarterPoints[i];
                point.x *= 4;
                point.y *= 4;
                
                fullPoints.push_back(point);
            }
        }
    }
    
    return fullPoints;
}

std::vector<cv::Point2f> &ImagePoints::getHalfPoints()
{
    if (halfPoints.empty()) {
        // figure out which scale exists.
        if (fullPoints.empty() == false) {
            for (int i = 0; i < fullPoints.size(); i++) {
                cv::Point2f point = fullPoints[i];
                point.x *= 0.5;
                point.y *= 0.5;
                
                halfPoints.push_back(point);
            }
        } else if (quarterPoints.empty() == false) {
            for (int i = 0; i < quarterPoints.size(); i++) {
                cv::Point2f point = quarterPoints[i];
                point.x *= 2;
                point.y *= 2;
                
                halfPoints.push_back(point);
            }
        }
    }
    
    return halfPoints;
}

std::vector<cv::Point2f> &ImagePoints::getQuarterPoints()
{
    if (quarterPoints.empty()) {
        // figure out which scale exists.
        if (fullPoints.empty() == false) {
            for (int i = 0; i < fullPoints.size(); i++) {
                cv::Point2f point = fullPoints[i];
                point.x *= 0.25;
                point.y *= 0.25;
                
                quarterPoints.push_back(point);
            }
        
        } else if (halfPoints.empty() == false) {
            for (int i = 0; i < halfPoints.size(); i++) {
                cv::Point2f point = halfPoints[i];
                point.x *= 0.5;
                point.y *= 0.5;
                
                quarterPoints.push_back(point);
            }
        }
    }
    
    return quarterPoints;
}

NamespaceEnd
