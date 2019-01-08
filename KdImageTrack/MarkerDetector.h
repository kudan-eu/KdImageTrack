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

#ifndef __PlanarTracker__MarkerDetector__
#define __PlanarTracker__MarkerDetector__

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "Marker.h"
#include "CameraImage.h"
#include "glm/glm.hpp"
#include "glm/gtc/quaternion.hpp"
#include "Homography.h"
#include "MarkerDetectorResult.hpp"

#include <future>
#include "Macros.h"

Namespace(KdImageTrack)



class MarkerDetector
{
public:
    // Different ways of detecting markers, depending what you want
    enum MarkerChoiceMethod
    {
        CHOOSE_FIRST,       // Like the original version. Return as soon as you've found something
        CHOOSE_ALL,         // Return a list of all markers which are detected
        CHOOSE_ORDERED,     // Return a list of all markers which are detected, sorted by quality
        CHOOSE_BEST        // Pick the best detection out of all markers
    };
    
    
    
private:
    std::vector<cv::KeyPoint> currentFrameKeypoints;
    cv::Mat currentFrameDescriptors;
    std::shared_ptr<CameraImage> currentCameraImage;
    
    std::vector<std::shared_ptr<Marker>> markers;
    
    std::vector<cv::DMatch> filterHamming(std::vector<cv::DMatch> &matches, float hammingThreshold);
    std::vector<cv::DMatch> filterOrientation(std::vector<cv::DMatch> &matches, std::shared_ptr<Marker> marker, float angleDifference);
    std::vector<cv::DMatch> filterHomography(std::vector<cv::DMatch> &matches, std::shared_ptr<Marker> marker, float reprojectionError, cv::Mat &homography);
    
    void extractFeatures(std::vector<std::vector<cv::Point2f>> maskRegions);
    bool matchMarker(MarkerDetectorResult &result);
    
    std::vector<MarkerDetectorResult> results;
    std::shared_ptr<Marker> lastDetectedMarker;
    
    std::vector<std::shared_ptr<Marker> > failedDetections;

   
    // Settings:
    MarkerChoiceMethod markerChoiceMethod;
    
    int numThreads;
    

    // Maximum permitted Hamming distance for ORB matches (used in hamming filter). Value of 0 means no filter.
    int hammingMatchThreshold; // Default 70
    
    // The threshold on keypoint orientation used for the orientation filter. Value of 0 means no filter
    float orientationFilterAngleThreshold; // Default: 8
    
    // The number of keypoint matches between camera and marker (after homography) required for detection
    int minMatchesForDetection; // Default: 30
    

    // Set the default number of threads to use for parallel detection (this allows threading to be toggled without worrying about number of threads)
#ifdef __APPLE__
    // OS devices have 2 (or 3) cores (2 should still be beneficial on a dual-core machine since it will at least speed up detection when tracking is not running):
    const int defaultNumThreads = 2;
#else
    // Android usually quad-core; so three plus the tracking thread uses all of them
    const int defaultNumThreads = 3;
#endif
    
    
public:
    
    void detectMarkers(std::vector<std::shared_ptr<Marker>> markersToDetect, std::vector<std::vector<cv::Point2f>> maskRegions);
    
    /* For use in threading or on its own. It does NOT do keypoint extraction or result selection. Calls machMarker on the markers within basically
    This returs a vector of detected marker results.  */

    std::vector<MarkerDetectorResult> detectSubsetOfMarkers(std::vector<std::shared_ptr<Marker>> markersToDetect);

    /* This is used for multithreading the marker detector only */
    void detectSubsetOfMarkersPromised(std::vector<std::shared_ptr<Marker>> markersToDetect, std::promise<std::vector<MarkerDetectorResult>> &&promise);

    
    bool addMarker(std::shared_ptr<Marker> marker);
    int removeMarker(std::shared_ptr<Marker> marker);
    
    void setCameraImage(std::shared_ptr<CameraImage> cameraImage)

    {
        currentCameraImage = cameraImage;
    }
    
    std::shared_ptr<CameraImage> getCameraImage()
    {
        return currentCameraImage;
    }
    
    int getNumberOfDetectedMarkers()
    {
        return (int)results.size();
    }
   
    int getNumberOfMarkers()
    {
        return (int)markers.size();
    }

    // Get a pointer to a marker, by its list position.
    const std::shared_ptr<Marker> getMarker(int m)
    {
        return markers[m];
    }
    
    // Return the vector of all markers - this is usually the list to detect
    const std::vector<std::shared_ptr<Marker> > getMarkers()
    {
        return markers;
    }
    
    std::vector<MarkerDetectorResult> &getDetectedMarkers()
    {
        return results;
    }
    
    void clearDetectedMarkers()
    {
        failedDetections.clear();
        results.clear();
    }
    
    void addResult(MarkerDetectorResult &result)
    {
        results.push_back(result);
    }
    
    
    std::vector<cv::KeyPoint> getKeypoints()
    {
        return currentFrameKeypoints;
    }
    
    std::vector<std::shared_ptr<Marker> > getFailedDetections()
    {
        return failedDetections;
    }
    
    MarkerDetector()
    {
        // defaults
        markerChoiceMethod = CHOOSE_BEST;
        
        hammingMatchThreshold = defaultHammingMatchThreshold;
        orientationFilterAngleThreshold = defaultOrientationFilterAngleThreshold;
        minMatchesForDetection = defaultMinMatchesForDetection;

        numThreads = defaultNumThreads;
        
    }
    
    /** 
     Query whether detection is run in multiple threads or not */
    bool isThreading()
    {
        return (numThreads > 1);
    }

    
    /** 
     Set whether detection uses multiple threads (for multiple markers).
     The correct number of threads for the device is set automatically
     */
    void toggleThreading(bool useThreading)
    {
        numThreads = useThreading? defaultNumThreads : 1;
    }


    
    // settings
    void setMarkerChoiceMethod(MarkerChoiceMethod m)
    {
        markerChoiceMethod = m;
    }
    
    void setNumThreads(int t)
    {
        numThreads = t;
    }
    
    /**
     Set threshold used in hamming filter, which cuts any potential match with a higher distance than this
     Set to a value of 0 to skip this filter 
     Default value: 70
     */
    void setHammingThreshold(int ham) {
        // basic validation: makes no sense to be negative
        if (ham >= 0)
            hammingMatchThreshold = ham;
    }
    
    /** 
     Set the angle threshold used in the orientation filter, which cuts matches which do differ from the dominant orientation change in the image by more than this value.
     Set to a value of 0 to skip this filter
     Angle is in degrees
     Default value: 8
     */
    void setOrientationFilterThreshold(float angle) {
        // basic validation: don't allow negative angles
        if (angle >= 0)
            orientationFilterAngleThreshold = angle;
    }
    
    /**
     Set the minimum number of (post homography inlier) matches required for a marker to be considered a match
     Default value: 30
     */
    void setNumMatchesRequired(int numMatches) {
        // basic validation: can't be negative
         if (numMatches > 0)
             minMatchesForDetection = numMatches;
    }
    
    // Definition of defaults (constants by definition)
    
    static constexpr int defaultHammingMatchThreshold = 70;
    static constexpr float defaultOrientationFilterAngleThreshold = 8.f;
    static constexpr int defaultMinMatchesForDetection = 30;
    
};

NamespaceEnd

#endif /* defined(__PlanarTracker__MarkerDetector__) */
