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

#include "CameraImage.h"

Namespace(KdImageTrack)

CameraImage::CameraImage(cv::Mat src, int t)
{
    srcImage = src;
    fullImage = src;
    timestamp = t;
}

cv::Mat CameraImage::getFull()
{
    return fullImage;
}

cv::Mat CameraImage::getHalf()
{
    if (halfImage.cols == 0) {
        cv::resize(fullImage, halfImage, cv::Size(fullImage.cols / 2, fullImage.rows / 2));
    }
    return halfImage;
}

cv::Mat CameraImage::getQuarter()
{
    if (quarterImage.cols == 0) {
        getHalf();
        cv::resize(halfImage, quarterImage, cv::Size(halfImage.cols / 2, halfImage.rows / 2));
    }
    return quarterImage;
}

NamespaceEnd
