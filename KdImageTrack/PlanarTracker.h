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

#ifndef __MarkerTracker__PlanarTracker__
#define __MarkerTracker__PlanarTracker__

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "CameraImage.h"
#include "Marker.h"
#include "MarkerDetector.h"
#include "PatchMatcher.h"
#include "PoseEstimator.h"
#include "CameraCalibration.h"
#include "Homography.h"
#include "OpticalFlowTracker.h"
#include "Logging.hpp"
#include "glm/glm.hpp"
#include "glm/gtc/quaternion.hpp"

#include <thread>
#include <queue>
#include "Macros.h"

Namespace(KdImageTrack)

class TrackedMarker;

class PlanarTracker
{
public:
    
    // Options for verifying (with patch matching) new detections, before adding to the trackedMarker list
    enum VerifierChoiceMethod
    {
        VERIFY_FIRST,   // Verify only the first one in the list (assuming there is only one deteted marker available, or it is an ordered list)
        VERIFY_BEST,    // Attempt to verify all, and only keep the best one (according to number of inliers)
        VERIFY_ALL      // Verify and keep all viable detections. Use with care, will allow overlapping detections!
    };
    
private:
    
    std::shared_ptr<CameraImage> currentCameraImage;
    MarkerDetector markerDetector;
    // Note: PatchMatchers now belong to TrackedMarkers
    
    PoseEstimator poseEstimator;
    CameraCalibration cameraCalibration;
    
    unsigned currentFrameNumber;
    
    // markers that are currently being tracked.
    std::vector< std::shared_ptr<TrackedMarker> > trackedMarkers;
    
    bool shouldDebug;
    

    std::mutex detectionMutex;
    std::thread detectionThread;
    
    
    // New settings:
    
    bool doThreadedDetection;
    
    int maxToTrack;     // Controls how many things you can track at once. If there are enough being tracked, the detector will not be called

    
    // Visualisation only:
    std::vector<cv::KeyPoint> lastKeypoints;
    
    
    VerifierChoiceMethod verifierChoiceMethod;
    

    bool intrinsicsAreKnown;
  
    bool useFlowRecovery;     // The default global setting for using optical flow recovery mode

    
    std::queue<std::shared_ptr<Marker>> newMarkerQueue;
    std::queue<std::shared_ptr<Marker>> removeMarkerQueue;
    
    std::mutex markerQueueMutex;
    bool newMarkerQueueEmpty;
    bool removeMarkerQueueEmpty;
    
public:
    
    // For video tests
    int numDetectionsAttempted, numDetectionsMade, numDetectionsKept, numTracksAttempted;

    
    // Get the resolution required for a particular patch matcher
    CameraImage::CameraResolution getResolution(PatchMatcher &patchMatcher, std::shared_ptr<Homography> homography, std::shared_ptr<Homography> previousHomography, std::shared_ptr<Marker> marker, float *maxDistance);
    bool trackMarker(std::shared_ptr<TrackedMarker> trackedMarker);
    
    std::shared_ptr<DebugState> debugState;
    
    // Main processin function, calls tracker and detector
    void processFrame(cv::Mat image);
    
    
    void updateCamera(cv::Mat image)
    {
        currentCameraImage = std::make_shared<CameraImage>(image, currentFrameNumber);
    }
    
    /* Call trackMarker on each marker in the trackedMarker list
     Returns whether the detector should be run or not */
    bool trackAllMarkers();
    
    /* If multiple markers are present and being tracked simultaneously, try to do their optical flow computations together to save time 
     Returns true if this was done, false otherwise */
    bool processSharedOpticalFlow();
    
    // Non-threaded version. Just calls the marker detector
    void detectMarkersNow();
    
    void detectMarkersInBackground();
    
    // This is what does the detection, possibly in a thread:
    void detectMarkers(std::vector<std::shared_ptr<Marker> > markersToDetect, std::vector<std::vector<cv::Point2f> > maskRegions);
    
    /* After detection has been run, verify them with patch matcher and add to list as appropriate
     This takes a camera image as an argument - you might be verifying in the past for threaded detection */
    void verifyDetections(std::shared_ptr<CameraImage> cameraImage);
    
    /* This is to set up a new tracked marker. Assumes what has been passed in is a new TrackedMarker with only its marker pointer set
     This calls the patch matcher to verify it, then sets various data such as optical flow initialisation
     If matching fails this returns false
     It does NOT add it to any lists that's your responsibility */
    std::shared_ptr<TrackedMarker> setupNewTrackedMarker(std::shared_ptr<Marker> marker, std::shared_ptr<CameraImage> cameraImage, std::shared_ptr<Homography> homography);
    
    /* This is to set up a new tracked marker. Assumes what has been passed in is a new TrackedMarker with only its marker pointer set
     This calls the patch matcher to verify it, then sets various data such as optical flow initialisation
     If matching fails this returns false
     It does NOT add it to any lists that's your responsibility */
    bool setupNewTrackedMarker(TrackedMarker &trackedMarker, cv::Ptr<CameraImage> cameraImage, cv::Ptr<Homography> homography);
    
    // Get the list of markers to pass to the detector
    std::vector<std::shared_ptr<Marker> > getMarkersToDetect();
    
    // Adds a marker to the detector. Returns false if it will not be added (because it does not have enough points for example)
    bool addMarker(std::shared_ptr<Marker> marker);

    /// Removes this marker from the detector. Does not return anything, because it can't know about state, because it's in general queued
    void removeMarker(std::shared_ptr<Marker> marker);
    
    void processMarkerQueues();
    
    
    int getNumberOfMarkersDetected()
    {
        return markerDetector.getNumberOfDetectedMarkers();
    }
    
    const MarkerDetectorResult &getDetectedMarker(int position)
    {
        return markerDetector.getDetectedMarkers()[position];
    }
    

    bool hasCameraCalibration() {
        return intrinsicsAreKnown;
    }
    CameraCalibration getCameraCalibration()
    {
        return cameraCalibration;
    }
    
    int getNumberOfTrackedMarkers()
    {
        return (int)trackedMarkers.size();
    }
    
    int getNumberOfMarkers()
    {
        return (int)markerDetector.getNumberOfMarkers();
    }
    
    std::shared_ptr<TrackedMarker> getTrackedMarker(int position)
    {
        return trackedMarkers[position];
    }
    
    std::vector<std::vector<cv::Point2f> > getTrackedBoundaries() ;
    
    /* Get the current camera image. Returns an empty image if not available */
    cv::Mat getCameraImage();

    // Return the keypoints saved out of the detector last time it was called
    std::vector<cv::KeyPoint> getKeypoints()
    {
        return lastKeypoints;
    }
    
    
    
    /** OPTICAL FLOW RECOVERY **/
    
    bool isUsingFlowRecovery()
    {
        return useFlowRecovery;
    }
    
    /* Set whether flow recovery for all markers happens. This can be overridden on a per-marker basis by using the force- and prohibit- functions */
    void setUseFlowRecovery(bool f)
    {
        useFlowRecovery = f;
    }
    
    /* Gets the number of markers for which flow recovery is enabled.
     If the global useFlowRecovery settings is on, this is tha same as the number of markers */
    int numMarkersFlowRecoverable()
    {
        
        std::vector<std::shared_ptr<Marker> > markers = markerDetector.getMarkers();

        int num = 0;
        for (int m = 0; m < markers.size(); m++) {
            num += markers[m]->isFlowRecoverable(useFlowRecovery);
        }
        return num;
    }
    
    /* This is stronger than switching off flow recovery: it prohibits flow recovery for every marker, irrespective of the global setting */
    void prohibitFlowRecovery()
    {
        std::vector<std::shared_ptr<Marker> > markers = markerDetector.getMarkers();

        for (int m = 0; m < markers.size(); m++) {
            markers[m]->prohibitFlowRecovery();
        }
    }
    
    
private:
    // These functions are to be used by the tracker, not for initiating recovery from outside! The process is always automatic.
    
    /* Run the optical flow recovery for one marker. This is called when tracking has failed, not during detection
     Returns true iff the marker was recovered successfully */

    bool doFlowRecoveryForMarker(std::shared_ptr<Marker> marker);
    
    /* Determine whether now is an apprropriate time to create a recovery keyframe for a marker, and is so, create, add to the set, and remove an old ones nevessary
     Returns true iff a new keyframe was created */
    bool attemptNewFlowKeyframe(std::shared_ptr<TrackedMarker> trackedMarker);

    
public:
    // end of optical flow recovery functions
    
    

    PlanarTracker();

    
    ~PlanarTracker();
    
    // SETTINGS:
    
    /* Toggle whether the tracker and detector are run in separate threads. If this is off, the whole tracker will be waiting for the (much slower) detector.
     This should not be set to false, unless all asynchronous detection/tracking is dealt with elsewhere (which will be impossible when multiple markers are used). */
    void setThreading(bool t)
    {
        doThreadedDetection = t;
    }
    
    /* This sets the number of threads used to parallelise detection (indepndent of the parallelisation between tracking and detection) */
    void setNumDetectionThreads(int t)
    {
        markerDetector.setNumThreads(t);
    }
    
    /* For setting all threading settings at once:
     if set to 0, no threading is done anywhere
     if set to 1, there is one thread - i.e. not rheading (so same as 0)
     if set to 2, detection and tracking are parallelised, (one thread each)
     if set to >=3, detection and tracking are parallelised, and detection itself is also threaded with that many threads-1 (so the total number of threads spawned will be as requested)
    */
    void setNumThreads(int t)
    {
        if (t <= 1) {
            setThreading(false);
            setNumDetectionThreads(0);
        }
        else if (t == 2) {
            setThreading(true);
            setNumDetectionThreads(0);
        }
        else {
            setThreading(true);
            setNumDetectionThreads(t - 1);
        }
    }
    
    /* This sets the number of threads used to parallelise detection (indepndent of the parallelisation between tracking and detection) */
    void toggleParallelDetection(bool isParallel)
    {
        markerDetector.toggleThreading(isParallel);
    }
    
    bool isDetectorParallel()
    {
        return markerDetector.isThreading();
    }
    
    
    /* How to choose markers on detection? These are the options:
     CHOOSE_FIRST       Return as soon as something is detected (matches>threshold), and search for the last detected marker first. (Like the previous pre-VisionDev version).
     CHOOSE_ALL         Return a list of all markers which are detected
     CHOOSE_ORDERED     Return a list of all markers which are detected, sorted by quality
     CHOOSE_BEST        Pick the best detection out of all markers - return at most one. This is currently the default.
     */
    void setMarkerChoiceMethod(MarkerDetector::MarkerChoiceMethod m)
    {
        markerDetector.setMarkerChoiceMethod(m);
    }
    
    /* Set how many markers can be tracked at once. This prevents the detector from running if there are already enough markers tracked.
     Default is 0, i.e. detect and track as many markers as possible */
    void setMaximumSimultaneousTracking(int t)
    {
        maxToTrack = t;
    }
    
    void clearTracking()
    {
        trackedMarkers.clear();
    }
    
    void setIntrinsics(float focalX, float focalY, float prinX, float prinY);
    
    
    
    // This is for checking an image larger than this is never processed. It is the caller's responsibility to ensure this.
    static const int maxImageWidth = 640;
    
    // === MarkerDetector settings ===
    
    
    /**
     Set threshold used in hamming filter, which cuts any potential match with a higher distance than this
     Set to a value of 0 to skip this filter
     Default value: 70
     */
    void setHammingThreshold(int ham) {
        markerDetector.setHammingThreshold(ham);
    }
    
    /**
     Set the angle threshold used in the orientation filter, which cuts matches which do differ from the dominant orientation change in the image by more than this value.
     Set to a value of 0 to skip this filter
     Angle is in degrees
     Default value: 8
     */
    void setOrientationFilterThreshold(float angle) {
        markerDetector.setOrientationFilterThreshold(angle);
    }
    
    /**
     Set the minimum number of (post homography inlier) matches required for a marker to be considered a match
     Default value: 30
     */
    void setNumMatchesRequired(int numMatches) {
        markerDetector.setNumMatchesRequired(numMatches);
    }
    
    
    // === Extended Markers ==
    
    /**
     Of all the markers currently being tracked, are any of them involved in extended marker in any way?
     i.e. could they be extended or switched?
     */
    bool trackedMarkersUseExtensions();
    
    /**
     Of ALL the markers in the detector, are any of them involved in extended marker in any way?
     i.e. could they be extended or switched?
     Probably obselete!
     */
    bool anyMarkersUseExtensions();
    
    
    /**
     The main function for initialising extended markers (should only be called if any of the markers currently tracked support this)
     This will look at tracked markers, and for those which are not extended (but could be) will try to create new extensions
     @param image The current camera image
     @return The number of new extensions created*/
    int extendMarkers(cv::Mat image);
    
        
    /**
     Attempt to create a new exteded marker, from an existing marker, at its current tracked state
     @param image The current camera image (for extracting the new marker image)
     @param marker The marker which is to be extended
     @param trackedMarker Representing the current tracked state of the marker in the image
     @return True iff a new marker was created and added to the tracker
     */
    bool createExtension(cv::Mat image, std::shared_ptr<Marker> marker, std::shared_ptr<TrackedMarker> trackedMarker);
    
   
    /** 
     Once a new extended marker is created, it might not necessarily be good enough (could be on a featureless region of the image for example). This makes sure the new marker fulfills various criteria
     */
    bool checkNewMarker(std::shared_ptr<Marker> marker);
    
    /** 
     This is called during tracking when any of the markershave extensions, in order to see if for any marker, any of its other extensions (i.e. markers with the same root) would be better candidates for tracking right now.
     If so, it will switch to tracking these, and create new TrackedMarker objects as needed */
    void switchTrackedExtensions(cv::Mat image);
};

NamespaceEnd

#endif /* defined(__MarkerTracker__PlanarTracker__) */
