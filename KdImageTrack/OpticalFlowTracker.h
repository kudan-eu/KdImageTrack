//
//  OpticalFlowTracker.h
//  PlanarTracker
//
//  Created on 06/03/2015.
//  Copyright (c) 2015 Kudan. All rights reserved.
//

#ifndef __PlanarTracker__OpticalFlowTracker__
#define __PlanarTracker__OpticalFlowTracker__

#include <stdio.h>
#include <memory>
#include "CameraImage.h"
#include "Marker.h"
#include "ImagePoints.h"
#include "Homography.h"
#include "Macros.h"

Namespace(KdImageTrack)

class OpticalFlowTracker
{
private:
    cv::Mat getEstimatedHomography();
public:
    CameraImage::CameraResolution resolution;
    
    std::shared_ptr<CameraImage> previousCameraImage;
    std::shared_ptr<CameraImage> currentCameraImage;
    
    ImagePoints previousPointsInCamera;
    std::vector<cv::Point2f> previousPointsInMarker;
    
    
    // Store these in the object too, for when sharing flows
    std::vector<cv::Point2f> currentFrameMarkerFeatures;
    std::vector<cv::Point2f> currentFrameFeatures;
    
    // To indicate if this object has been tracked already, by the shared flow function
    bool isTracked;

    
    std::shared_ptr<Homography> homography;
    std::shared_ptr<Homography> previousHomography;
    
    std::vector<cv::Mat> previousPyramid;
    
    OpticalFlowTracker()
    {
        resolution = CameraImage::CameraImageQuarter;
        homography = std::make_shared<Homography>();
        previousHomography = std::make_shared<Homography>();
        isTracked = false;
    }
    
    void setCurrentCameraImage(std::shared_ptr<CameraImage> image)
    {
        previousCameraImage = currentCameraImage;
        currentCameraImage = image;
    }
    
    
    bool track();
    bool calculateHomography();
    
    /** Sometimes multiple markers will be flowing between the same frames. Use this to group the points from multiple markers and flow them together, then store the result in each individual tracker again */
    static bool trackShared(std::vector<std::shared_ptr<OpticalFlowTracker>> trackers, std::shared_ptr<CameraImage> image) ;
    
};

NamespaceEnd

#endif /* defined(__PlanarTracker__OpticalFlowTracker__) */

