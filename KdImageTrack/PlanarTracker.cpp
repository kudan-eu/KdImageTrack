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

#include "PlanarTracker.h"
#include "MarkerDetectorResult.hpp"
#include "CameraImage.h"
#include "TrackedMarker.hpp"
#include "glm/glm.hpp"
#include "glm/gtc/quaternion.hpp"
#include "Profiler.hpp"
#include "Helper.h"
#include "Logging.hpp"
#include <unordered_set>

Namespace(KdImageTrack)

static cv::Mat getAffine(cv::Mat inverseHomography, cv::Rect region, float *determinant)
{
    // static to reduce allocation overhead.
    static std::vector<cv::Point2f> points(3);
    static std::vector<cv::Point2f> transformedPoints(3);
    
    points[0] = cv::Point2f(region.x, region.y);
    points[1] = cv::Point2f(region.x, region.y + region.height);
    points[2] = cv::Point2f(region.x + region.width, region.y + region.height);
    
    // transform back to the reference image.
    cv::perspectiveTransform(points, transformedPoints, inverseHomography);
    
    // calculate the affine transformation.
    cv::Mat affine = cv::getAffineTransform(transformedPoints, points);
    
    // position the affine patch at (0, 0).
    affine.at<double>(0, 2) -= region.x;
    affine.at<double>(1, 2) -= region.y;
    
    // make a 3x3 matrix so we can calculate the determinant.
    cv::Mat affineSquare(3, 3, CV_32F);
    float *d = (float *)affineSquare.data;
    double *s = (double *)affine.data;
    
    for (int i = 0; i < 6; i++) {
        d[i] = s[i];
    }
    
    d[6] = 0;
    d[7] = 0;
    d[8] = 1;
    
    affine = affine(cv::Rect(0, 0, 3, 2));
    
    // determine which pyramid level to use as a source.
    *determinant = 1.0 / cv::determinant(affineSquare);
    
    cv::invertAffineTransform(affine, affine);
    
    return affine;
}


std::shared_ptr<Homography> getEstimatedHomog(std::shared_ptr<TrackedMarker> trackedMarker)
{
    if (trackedMarker->prevHomography == nullptr) {
        // not enough data to lerp.
        return trackedMarker->homography;
    }
 
    cv::Mat prevHomog = trackedMarker->prevHomography->getFullHomography();
    cv::Mat currentHomog = trackedMarker->homography->getFullHomography();
    
    cv::Mat lerpedHomog = lerpHomography(prevHomog, currentHomog, 2);

    std::shared_ptr<Homography> homography = std::make_shared<Homography>();
    homography->setFullHomography(lerpedHomog);
    
    return homography;
}



PlanarTracker::PlanarTracker()
{
    shouldDebug = true;
    
    intrinsicsAreKnown = false;
    
    
    currentCameraImage = nullptr;
    
    
#ifndef __APPLE__
    // For Android devices, assume there are four cores
    cv::setNumThreads(4);
    // (this means something different on iOS, but defaults to using all cores anyway)
#endif
    
    currentFrameNumber = 0;
    
    
    doThreadedDetection = 1;
    maxToTrack = 0;
    
    verifierChoiceMethod = VERIFY_FIRST; // because default is to only detect best, so should only be one
    
    useFlowRecovery = false;
    
    printdebug("Using OpenCV version %i.%i.%i \n",currentFrameNumber,CV_MAJOR_VERSION,CV_MINOR_VERSION,CV_SUBMINOR_VERSION);
    
    newMarkerQueueEmpty = true;
    removeMarkerQueueEmpty = true;
    
}


PlanarTracker::~PlanarTracker()
{
    // if there is a thread which might be running, wait for it to finish (do nothing with it)
    if (detectionThread.joinable()) {
        detectionThread.join();
    }
}



CameraImage::CameraResolution PlanarTracker::getResolution(PatchMatcher &patchMatcher, std::shared_ptr<Homography> homography, std::shared_ptr<Homography> previousHomography, std::shared_ptr<Marker> marker, float *max)
{
    // calculate how near the camera is to the centre of the marker.
    cv::Rect r1(320, 240, 8, 8);
    cv::Rect r2(160, 120, 4, 4);
    cv::Rect r3(80, 60, 2, 2);
    
    float d1, d2, d3;
    
    // calculate determinants at each camera level.
    getAffine(homography->getFullHomography(), r1, &d1);
    getAffine(homography->getHalfHomography(), r2, &d2);
    getAffine(homography->getQuarterHomography(), r3, &d3);
    
    patchMatcher.resolution = CameraImage::CameraImageFull;
    
    // assume no motion, select the closest camera resolution for the marker scale.
    if (d2 > 1) {
        patchMatcher.resolution = CameraImage::CameraImageHalf;
    }
    if (d3 > 1) {
        patchMatcher.resolution = CameraImage::CameraImageQuarter;
    }
    
    // calculate effect of motion. more motion = lower the camera level, within reason.
    std::vector<cv::Point2f> candidates;
    
    for (int i = 0; i < marker->trackingImages[0].keypoints.size(); i++) {
        candidates.push_back(marker->trackingImages[0].keypoints[i].pt);
    }

    std::vector<cv::Point2f> projected;
    cv::perspectiveTransform(candidates, projected, homography->getFullHomography());
    
    std::vector<cv::Point2f> keptCam;
    std::vector<cv::Point2f> keptMarker;
    
    int imageWidth = currentCameraImage->getFull().size().width;
    int imageHeight = currentCameraImage->getFull().size().height;
    
    for (int i = 0; i < projected.size(); i++) {
        cv::Point2f &cameraPoint = projected[i];
        cv::Point2f &markerPoint = candidates[i];
        
        // only keep points that are within camera bounds.
        if (cameraPoint.x > 0 && cameraPoint.y > 0 && cameraPoint.x < imageWidth && cameraPoint.y < imageHeight) {
            keptCam.push_back(cameraPoint);
            keptMarker.push_back(markerPoint);
        }
    }
    
    projected.clear();
    
    if (previousHomography == nullptr) {
        return patchMatcher.resolution;
    }
    
    // project kept points by previous homography.
    if (previousHomography->getFullHomography().empty()) {
        return patchMatcher.resolution;
    }

    if (keptMarker.size() == 0) {
        return patchMatcher.resolution;
    }
    
    if (previousHomography->getFullHomography().cols != 3 || previousHomography->getFullHomography().rows != 3) {
        return patchMatcher.resolution;
    }
    
    cv::perspectiveTransform(keptMarker, projected, previousHomography->getFullHomography());
    
    float maxDistance = 0;
    
    // find the largest distance a point has moved.
    for (int i = 0; i < keptMarker.size(); i++) {
        cv::Point2f diff = keptCam[i] - projected[i];
        
        diff.x = fabs(diff.x);
        diff.y = fabs(diff.y);
        
        // frame to frame distance between point.
        float distance = sqrtf(diff.x * diff.x + diff.y * diff.y);
        
        if (distance > maxDistance) {
            maxDistance = distance;
        }
    }
    
    if (max != NULL) {
        *max = maxDistance;
    }
    
    int motionBlurLevel = 0;
    
    // set motion blur level based on the maximum distance a tracked point has travelled.
    if (maxDistance > 8) {
        motionBlurLevel = 1;
    }
    if (maxDistance > 16) {
        motionBlurLevel = 2;
    }
    
    if (motionBlurLevel == 2) {
        // check whether the desired level will have enough quality.
        if (d3 > 0.03) {
            patchMatcher.resolution = CameraImage::CameraImageQuarter;
            patchMatcher.matchThreshold -= 0.1;
        } else {
            // quarter camera image is too low quality for this scale, go to the next motion blur level.
            motionBlurLevel = 1;
        }
    }
    
    if (motionBlurLevel == 1) {
        if (patchMatcher.resolution == CameraImage::CameraImageFull) {
            if (d2 > 0.03 ) {
                patchMatcher.resolution = CameraImage::CameraImageHalf;
            }
        } else if (patchMatcher.resolution == CameraImage::CameraImageHalf) {
            if (d3 > 0.03) {
                patchMatcher.resolution = CameraImage::CameraImageQuarter;
            }
        }
        patchMatcher.matchThreshold -= 0.1;
    }
    
    return patchMatcher.resolution;
}

bool PlanarTracker::trackMarker(std::shared_ptr<TrackedMarker> trackedMarker)
{

    // check if it's being deleted, and if so, remove from the tracking list
    if (trackedMarker->marker->toBeRemoved) {
        printlog(LOG_TRACK, "Failing to track a deleted marker %s \n", trackedMarker->marker->name.c_str());
        return false;
    }
    
    // Check this is in principle trackable, i.e. absolute minimum number of points (should not have been added):
    std::vector<cv::KeyPoint> &keypointSource = trackedMarker->marker->trackingImages[1].keypoints;
    if (keypointSource.size() < 4) {
        printlog(LOG_TRACK, "Marker %s cannot be tracked: too few points \n",trackedMarker->marker->name.c_str());
        return false;
    }

    
    std::shared_ptr<OpticalFlowTracker> opticalFlowTracker = trackedMarker->opticalFlowTracker;
    PatchMatcher &patchMatcher = trackedMarker->patchMatcher;

    
    const bool useOpticalFlow = true;
    
    // the homography from the coarse search stage.
    std::shared_ptr<Homography> coarseHomography;
    
    bool opticalFlowFailed = false;
    if (useOpticalFlow) {
       
        // Is it necessary to track this marker or has it already been done?
        if (!opticalFlowTracker->isTracked) {
            
            // coarse search using optical flow.
            opticalFlowTracker->setCurrentCameraImage(currentCameraImage);

            if (opticalFlowTracker->track() == false) {
                opticalFlowFailed = true;
            }
        }
        
        
        // Now that the optical flow tracking has been done (in one way or another), calculate the homography for this marker:
        if (!opticalFlowFailed && opticalFlowTracker->calculateHomography() == false) {
            printlog(LOG_FLOW,"failed optical flow homography \n");

            opticalFlowFailed = true;
        }
        
        // ony use this as the coarse homography if opticl flow didn't fail
        if (!opticalFlowFailed) {
            coarseHomography = opticalFlowTracker->homography;
        }


    }
    
    // If NOT using optical flow, or using optical flow but it FAILED, then to coarse patch match instead:
    if (!useOpticalFlow || opticalFlowFailed) {

        // coarse search from motion prediction.
        patchMatcher.matchThreshold = 0.5;
        patchMatcher.searchRadius = 16;
        patchMatcher.patchesToSearch = 30;
        
        patchMatcher.setHomography(getEstimatedHomog(trackedMarker));
        patchMatcher.resolution = CameraImage::CameraImageQuarter;
        patchMatcher.setMarker(trackedMarker->marker);
        patchMatcher.setCameraImage(currentCameraImage);
        
        
        if (patchMatcher.match() == false) {
            return false;
        }
        // if successful, set coarse homography
        coarseHomography = patchMatcher.getHomography();
    }
    
    // fine search.
    patchMatcher.matchThreshold = 0.6;
    patchMatcher.searchRadius = 14;
    patchMatcher.patchesToSearch = 100;

    float maxDistance;
    getResolution(patchMatcher, coarseHomography, trackedMarker->homography, trackedMarker->marker, &maxDistance);
    
    // match patches using homography from optical flow.
    patchMatcher.setMarker(trackedMarker->marker);
    patchMatcher.setCameraImage(currentCameraImage);
    
    
    if (trackedMarker->framesInOpticalFlow > 0) {
        // increase the search radius due to optical flow inaccuracy.
        patchMatcher.searchRadius = 14;
        patchMatcher.matchThreshold = 0.60;
    }
    
    patchMatcher.setHomography(coarseHomography);
    
    bool useCoarsePose = false;
    bool successFullyMatchedPatches = true;
    
    if (shouldDebug == true) {
        patchMatcher.debug = true;
        patchMatcher.debugState = debugState;
    } else {
        patchMatcher.debug = false;
        patchMatcher.debugState = nullptr;
    }
    
    
    if (patchMatcher.match() == false) {

        useCoarsePose = true;

        successFullyMatchedPatches = false;
        trackedMarker->framesInOpticalFlow++;
        trackedMarker->framesOutOfOpticalFlow = 1;
    } else {
        trackedMarker->framesInOpticalFlow = 0;
    }
    
    // only count if active, and shut off after a while
    if (trackedMarker->framesOutOfOpticalFlow > 0) {
        trackedMarker->framesOutOfOpticalFlow++;
        if (trackedMarker->framesOutOfOpticalFlow > 30) {
            trackedMarker->framesOutOfOpticalFlow = 0;
        }

    }
    
    
    
    const int maxFramesInOpticalFlow = 10;
    if (trackedMarker->framesInOpticalFlow > maxFramesInOpticalFlow) {
        return false;
    }
    
    // calculate the pose.
    poseEstimator.setMarker(trackedMarker->marker);
    poseEstimator.shouldFilterPose = false;
    
    std::shared_ptr<Homography> homography = nullptr;

    if (useCoarsePose == true) {
        // Don't just use the optical flow pose: use whatever the coarse homograph is (could be patch matcher)

        homography = coarseHomography;
        
    } else {
        homography = patchMatcher.getHomography();
    }
    
    // return that tracking failed
    if (homography == nullptr || homography->getFullHomography().empty()) {
        printerr("Tracked homography is null or empty\n");
        return false;
    }
    
    std::shared_ptr<Homography> tmpHomography = std::make_shared<Homography>();
    


    if (maxDistance < 10) {

        trackedMarker->isSmooth = true;

        // weight
        if (trackedMarker->prevHomography != nullptr && trackedMarker->weightedHomography != nullptr) {
            cv::Mat prev = trackedMarker->weightedHomography->getFullHomography();
            cv::Mat cur = homography->getFullHomography().clone();
            
            float weight = 0;
            if (maxDistance < 2) {
                weight = 0.6;
            } else if (maxDistance < 5) {
                weight = 0.4;
            } else {
                weight = 0.2;
            }
            
            cv::accumulateWeighted(prev, cur, weight);
            
            tmpHomography->setFullHomography(cur);
            trackedMarker->weightedHomography = tmpHomography;
        } else {
            trackedMarker->weightedHomography = homography;
            tmpHomography = homography;
        }
    }
    else {
        
        trackedMarker->isSmooth = true;
        
        tmpHomography = homography;
        trackedMarker->weightedHomography = homography;
    }
    
    // If the pose is from optical flow only, then mark it as dubious
    trackedMarker->unreliablePose = (trackedMarker->framesInOpticalFlow > 0);
    
    if (trackedMarker->unreliablePose) {
        
        // If there is no last pose - i.e. if this was the first frame - then just fail
        if ( trackedMarker->lastPosition == glm::vec3(0, 0, 0)) {

            printlog(LOG_TRACK,"Wanted last pose, but does not exist. Fail! \n");
            return false;
        }
        
        printlog(LOG_TRACK,"KEEPING LAST POSE! \n");
        trackedMarker->position = trackedMarker->lastPosition;
        trackedMarker->orientation = trackedMarker->lastOrientation;
    }
    else {
        // compute the pose properly
        poseEstimator.setHomography(tmpHomography);
        poseEstimator.estimatePose(trackedMarker->position, trackedMarker->orientation);
        
        trackedMarker->lastPosition = trackedMarker->position;
        trackedMarker->lastOrientation = trackedMarker->orientation;
        
    }
    
    

    
    trackedMarker->setHomography(homography);
    
    // feed successfully matches patches to the optical flow tracker for the next frame.
    if (successFullyMatchedPatches == true) {
        opticalFlowTracker->previousPointsInCamera = patchMatcher.patchPointsInCamera;
        opticalFlowTracker->previousPointsInMarker = patchMatcher.patchPointsInMarker;
    }
    
    

    // Flow recovery: create keyframe, if these conditions are met:

    if (maxDistance < 10 && successFullyMatchedPatches && trackedMarker->marker->isFlowRecoverable(useFlowRecovery)) {
        attemptNewFlowKeyframe(trackedMarker);
    }
    
    
    
    
    trackedMarker->timeSinceFlowKeyframe++;
    trackedMarker->lifetime++;

    
    return true;
}


bool PlanarTracker::attemptNewFlowKeyframe(std::shared_ptr<TrackedMarker> trackedMarker) {


    std::shared_ptr<Marker> marker = trackedMarker->marker;

    
    
    // if it was set to default

    PatchMatcher &patchMatcher = trackedMarker->patchMatcher;
    std::shared_ptr<Homography> homography = trackedMarker->homography;
    
    bool hasCreated = false;
    
    if ((trackedMarker->framesInOpticalFlow == 0 && trackedMarker->framesOutOfOpticalFlow == 0 && (trackedMarker->lifetime > 30 || !trackedMarker->fromFlowRecovery))) {
        //  and trackedMarker.timeSinceFlowKeyframe > 30
        
        // prevous pyramid is the one which was just put into the optical flow (ready for next frame)
        std::vector<cv::Mat> pyramid = trackedMarker->opticalFlowTracker->previousPyramid;
        
        int leastSimilarKeyframe;
        cv::Mat smallImage;

        if (trackedMarker->marker->checkForNewKeyframe(currentCameraImage->getQuarter(), pyramid, homography->getQuarterHomography(), patchMatcher.patchPointsInCamera.getQuarterPoints(),leastSimilarKeyframe,smallImage)) {

            
            std::vector<cv::Point2f> quarterMatchedPoints;
            
            bool isMatched = true;
            if (patchMatcher.resolution != CameraImage::CameraImageQuarter) {
                
                printlog(LOG_TRACK,"Not matched at quarter! \n");
                patchMatcher.resolution = CameraImage::CameraImageQuarter;
                
                
                std::vector<cv::Point2f> markerPoints;
                isMatched = patchMatcher.matchOnly(CameraImage::CameraImageQuarter, markerPoints, quarterMatchedPoints);
                
            }
            else {
                quarterMatchedPoints = patchMatcher.patchPointsInCamera.getQuarterPoints();
            }
            
            if (isMatched) {
                
                
                marker->removeFlowKeyframeIfNecessary(leastSimilarKeyframe);
                
                int numF = trackedMarker->marker->createFlowKeyframe(currentCameraImage->getQuarter(), pyramid, homography->getQuarterHomography(),quarterMatchedPoints,smallImage);
                printlog(LOG_TRACK,"There are now %i fow keyframes on marker %s \n",numF,marker->name.c_str());
                trackedMarker->timeSinceFlowKeyframe = 0;


                
                hasCreated = true;
            }
            else printlog(LOG_TRACK,"match failed \n");
            
        }
  
    }
    return hasCreated;
}


bool PlanarTracker::doFlowRecoveryForMarker(std::shared_ptr<Marker> marker)
{
    
    if (marker->flowKeyframes.size() == 0) {
        printlog(LOG_TRACK,"Can't recover on marker %s, no keyframes \n", marker->name.c_str());
        return false;
    }
    printlog(LOG_TRACK,"Relcalisating marker %s \n", marker->name.c_str());
    cv::Mat bestRecoveredHomography = marker->localiseByFlowKeyframes(currentCameraImage->getQuarter());
    
    if (!bestRecoveredHomography.empty()) {
        
        std::shared_ptr<Homography> newHomography = std::make_shared<Homography>();
        newHomography->setQuarterHomography(bestRecoveredHomography);
        
        
        std::shared_ptr<TrackedMarker> trackedMarker = setupNewTrackedMarker(marker, currentCameraImage, newHomography);
        
        if (trackedMarker != nullptr) {
            printlog(LOG_TRACK,"Ok, have recovered and initialised marker %s! \n",marker->name.c_str());
            
            printlog(LOG_RECOVER, "RECOVERED Marker %s IMMEDIATELY \n",marker->name.c_str());
            
            // remove from list if it exists
            for (int t = 0; t < trackedMarkers.size(); t++) {
                if (trackedMarkers[t]->marker == marker) {

                    trackedMarkers.erase(trackedMarkers.begin() + t);
                    printlog(LOG_TRACK,"Already in tracked marker list, so removed ");
                    break;
                }
            }
        

        
            trackedMarker->fromFlowRecovery = 2;
            trackedMarker->timeSinceFlowRecovery = 0;
            
            // the tracked marker (result) now knows the homography:
            trackedMarker->homography = newHomography;
            trackedMarker->weightedHomography = newHomography;
            
            poseEstimator.setHomography(newHomography);

            poseEstimator.estimatePose(trackedMarker->position, trackedMarker->orientation);
#ifdef SHOW_DEBUG

            for (int t = 0; t < trackedMarkers.size(); t++) {
                if (trackedMarkers[t]->marker == trackedMarker->marker) {
                    printdebug("DUPLICATE TRACK for marker %s \n",trackedMarker->marker->name.c_str());
                }
            }
#endif
            // Important: add this new tracked marker to the list:
            trackedMarkers.push_back( trackedMarker );

            
            printlog(LOG_TRACK,"OK, there are now %lu tracked markers \n",trackedMarkers.size());

        
            return true;
        }
    }
    return false;
    
}


cv::Mat PlanarTracker::getCameraImage()
{
    if (currentCameraImage) {
        return currentCameraImage->getFull();
    }
    else return cv::Mat();
}


void PlanarTracker::setIntrinsics(float focalX, float focalY, float prinX, float prinY) {
    intrinsicsAreKnown = true;
    poseEstimator.setIntrinsics(focalX, focalY, prinX, prinY);
    cameraCalibration = CameraCalibration(focalX, focalY, prinX, prinY);
}




void PlanarTracker::processFrame(cv::Mat image)
{
    if (image.channels() != 1) printerr("Wrong number of channels: got %i, this image should be grescale \n",image.channels());
    if (image.empty()) {
        printerr("Empty image in PlanarTracker \n");
        return;
    }
    // for output!
    numDetectionsAttempted = 0;
    numDetectionsMade = 0;
    numDetectionsKept = 0;
    numTracksAttempted = 0;

    // Do at the start only!
    

    if (image.size().width > maxImageWidth) {
        printerr("Image too large: %i x %i (max width is %i, please resize before passing) \n", image.size().width, image.size().height, maxImageWidth);
        return;
    }
   
    
    if (!intrinsicsAreKnown) {
        printerr("Cannot track: no intrinsics set \n");
        return;
    }


    // create different resolution versions of the camera image that are required by processing.
    currentCameraImage = std::make_shared<CameraImage>(image, currentFrameNumber);

    
    if (shouldDebug) {
        // store camera images.
        debugState = std::make_shared<DebugState>();
        debugState->cameraImage = currentCameraImage;
    }
    else {
        debugState = nullptr;

    }
    
    
    // If there are newly detected markers, verify them
    // Track everything that now exists
    // Then, if necessary, run the detector
    
    
    if (doThreadedDetection) {        
        if (detectionMutex.try_lock()) {
            verifyDetections(markerDetector.getCameraImage());
            
            processMarkerQueues();
            detectionMutex.unlock();

        }
    }
    else {
        processMarkerQueues();
    }

    // if possible, do optical flow for multiple markers at the same time:
    processSharedOpticalFlow();
    
    bool isLookingForMarkers = trackAllMarkers();
//#define DRAW_TRACK_DEBUG
#ifdef DRAW_TRACK_DEBUG
    cv::namedWindow("Tracker Debug");
    cv::Mat trackerDebugImg;
    cv::cvtColor(image, trackerDebugImg, CV_GRAY2BGR);
    
    for (int t = 0; t < trackedMarkers.size(); t++) {
        std::shared_ptr<TrackedMarker> trackedMarker = trackedMarkers[t];
        std::shared_ptr<Marker> marker = trackedMarker->marker;
        int w = marker->image.size().width;
        int h = marker->image.size().height;
        std::vector<cv::Point2f> markerBoundary = { cv::Point2f(0,0), cv::Point2f(0,h), cv::Point2f(w,h), cv::Point2f(w,0) };
        std::vector<cv::Point2f> warpedBoundary;
        std::shared_ptr<Homography> homog = trackedMarker->homography;
        if (homog) {
            cv::Mat H = homog->getFullHomography();
            if (H.rows == 3) {
                cv::perspectiveTransform(markerBoundary, warpedBoundary, H);
                for (int i = 0; i < warpedBoundary.size(); i++) {
                    int ii = (i+1)%warpedBoundary.size();
                    cv::line(trackerDebugImg, warpedBoundary[i], warpedBoundary[ii], cv::Scalar(60,200,60), 2);
                }
            }
        }
    }
    
    cv::imshow("Tracker Debug", trackerDebugImg);
#endif
    
    if (trackedMarkersUseExtensions()) {
        // Extended markers: for all tracked markers, see if they can be expanded (if not already)
        extendMarkers(image);
        
        // For any tracked markers that have extensions, see if switching can be done
        switchTrackedExtensions(image);
    }
    
    if (isLookingForMarkers) {
        if (doThreadedDetection) {
            detectMarkersInBackground();
        }
        else {
            detectMarkersNow();
        }
    }
    else lastKeypoints.clear(); // this is just to clear the keypoints to draw if you're not detecting
   
    
    currentFrameNumber++;
}


bool PlanarTracker::processSharedOpticalFlow()
{
    
    // If you're not tracking more than one marker, there's no need
    const int minForSharing = 2;
    
    if (trackedMarkers.size() < minForSharing) {
        return false;
    }
    
    
    printlog(LOG_FLOW,"Attempting shared flow, timestamp now is %i \n", currentCameraImage->timestamp);
    
    // A list of smart pointers to optical flow trackers, so they can be accessed/modified inside the shared flow function
    // This was previously done with reference wrappers but not so portable
    std::vector<std::shared_ptr<OpticalFlowTracker>> trackersToShare;
    
    // the one chosen for sharing will always be the first in the list. This means you can't share for example markers [2, 3, 4] if marker [1] has an older frame
    // BUT since the markers in the tracked marker list are always in order, the ones at the start of the list should be the tracked ones in the last frame, and therefore shareable

    std::shared_ptr<CameraImage> sharedCurrentImage = nullptr;
    
    // Gather up which optical flow trackers can be used together
    // This means they must have the same current image (which will soon become the previous image)
    for (int t = 0; t < trackedMarkers.size(); t++) {
        std::shared_ptr<TrackedMarker> trackedMarker = trackedMarkers[t];
        if (trackedMarker->opticalFlowTracker->currentCameraImage != nullptr) {
            
            if (trackedMarker->opticalFlowTracker->previousHomography != nullptr) {
                if (!trackedMarker->opticalFlowTracker->previousHomography->getQuarterHomography().empty()) {
                    
                    if (sharedCurrentImage == nullptr) {
                        printlog(LOG_FLOW,"OK, marker %s defines the camera image at time stamp of %i \n",trackedMarker->marker->name.c_str(),trackedMarker->opticalFlowTracker->currentCameraImage->timestamp);



                        trackersToShare.push_back( trackedMarker->opticalFlowTracker );

                        sharedCurrentImage = trackedMarker->opticalFlowTracker->currentCameraImage;
                    }
                    else  if (trackedMarker->opticalFlowTracker->currentCameraImage == sharedCurrentImage) {
                        
                        trackersToShare.push_back( trackedMarker->opticalFlowTracker );
                        
                        printlog(LOG_FLOW,"OK, marker %s has current stamp of %i \n",trackedMarker->marker->name.c_str(),trackedMarker->opticalFlowTracker->currentCameraImage->timestamp);
                        
                    }
                    else printlog(LOG_FLOW,"Can't share on marker %s since it has a different previous image, time stamp of %i \n",trackedMarker->marker->name.c_str(),trackedMarker->opticalFlowTracker->currentCameraImage->timestamp);
                }
                else printlog(LOG_FLOW,"Can't share on marker %s since its previous homography is empty \n",trackedMarker->marker->name.c_str());
            }
            else printlog(LOG_FLOW,"Can't share on marker %s since it has no previous homography \n",trackedMarker->marker->name.c_str());
        }
        else printlog(LOG_FLOW,"Can't share on marker %s since it has NO CURRENT image?! \n",trackedMarker->marker->name.c_str());



    }
    
    if (sharedCurrentImage == currentCameraImage) {
        printlog(LOG_FLOW,"This is a problem: the current and previous images will be the same!\n");
        return false;
    }
    if (trackersToShare.size() < minForSharing) {
        printlog(LOG_FLOW,"Can't do shared optical flow, only %lu in list \n", trackersToShare.size());
        trackersToShare.clear();
        return false;
    }
    

    printlog(LOG_FLOW," SHARE OPTICAL FLOW on %lu markers ... \n", trackersToShare.size());
   

    bool isTracked = OpticalFlowTracker::trackShared(trackersToShare, currentCameraImage);
    
    printlog(LOG_FLOW," isTracked : %i trackersToShare : %lu \n", int(isTracked), trackersToShare.size());
    
    return isTracked;
}


bool PlanarTracker::trackAllMarkers()
{


    std::vector<std::shared_ptr<Marker>> failedMarkers;

    
    // check that no two markers extended from the same root are to be looked at
    std::unordered_set<std::shared_ptr<Marker>> uniqueRoots;
    for (auto t : trackedMarkers) {
        std::shared_ptr<Marker> root = t->marker;
        if (root->extensionRoot != nullptr) {
            root = root->extensionRoot;
        }
        // you MUST be able to insert each of these to the unique set
        assert( uniqueRoots.insert(root).second );
    }
    
    
    int numTracked = 0;
    numTracksAttempted = (int)trackedMarkers.size();
    // for each marker detected in the previous frame, attempt to track it in the current one.
    for (int i = 0; i < trackedMarkers.size(); i++) {
        std::shared_ptr<TrackedMarker> trackedMarker = trackedMarkers[i];
        
        bool tracked = trackMarker(trackedMarker);
        if (tracked) {
            numTracked++;

        }
        else {

            failedMarkers.push_back(trackedMarker->marker);
            printlog(LOG_TRACK,"Tracking failed for marker %s",trackedMarker->marker->name.c_str());

            
            // tracking failed, so remove from list
            trackedMarkers.erase( trackedMarkers.begin() + i);
            i--;
            printlog(LOG_TRACK,"Decremented , i = %i \n", i);
            
            
        }
    }
    
    if (failedMarkers.size() > 0 ) {
        printlog(LOG_TRACK,"Attempting to recover on %lu failed markers \n", failedMarkers.size());
        for (int m = 0; m < failedMarkers.size(); m++) {
            if (failedMarkers[m]->isFlowRecoverable(useFlowRecovery)) {
                
                bool isRecovered = doFlowRecoveryForMarker(failedMarkers[m]);
                printlog(LOG_TRACK,"Recovered %i for marker %s \n", int(isRecovered), failedMarkers[m]->name.c_str());
                
                // if it was recovered ok, then say it was tracked so the detector doesn't need to be run
                if (isRecovered) {
                    numTracked++;
                }
            }
        }
    }

    // Decide whether to run detection or not:
    // If there is a maximum limit to the number which can be tracked, use it (make sure it's no more than th enumber of markers); otherwise use the total
    int numMarkers = markerDetector.getNumberOfMarkers();
    
    int numTracksWanted = maxToTrack>0 ? std::min(maxToTrack, numMarkers):numMarkers;
    
    bool doDetection = (numTracked < numTracksWanted);

    // if the number of markers tracked is less than the number of markers that you want to track, then yes, run detection....
    return doDetection;
}

std::vector<std::shared_ptr<Marker> > PlanarTracker::getMarkersToDetect()
{
    std::vector<std::shared_ptr<Marker> > markerList;

    bool checkForExtensions = false;
    
    // simplest case: nothing tracked
    if (trackedMarkers.size() == 0) {
        
        markerList = markerDetector.getMarkers();
        
        
        // in the simplest case, we have chosen all markers, so just need to see whether to use extended markers
        for (auto m : markerList) {
            if (m->isExtended()) {
                // As soon as one is known to have an extensions, then yes we do need to check
                checkForExtensions = true;
                // No need to check further for now
                break;
            }
        }
    }
    else {
   
    
        std::unordered_set<std::shared_ptr<Marker>> trackedMarkerSet;
        
        for (std::shared_ptr<TrackedMarker> trackedMarker : trackedMarkers) {
            
            // check for root marker
            std::shared_ptr<Marker> thisMarker = trackedMarker->marker;
            // If this does have a root, use that one for comparison
            if (thisMarker->extensionRoot != nullptr) {
                thisMarker = thisMarker->extensionRoot;
            }
            
            trackedMarkerSet.insert(thisMarker);
        }

        
        // look through all the markers and identify the ones not tracked (in any of their incarnations)
        for (std::shared_ptr<Marker> marker : markerDetector.getMarkers()) {
            
            // If this marker is NOT in the set, it should be detected
            // This is comparing the list of root markers (in the detector) to the root markers of those tracked
            if (trackedMarkerSet.find(marker) == trackedMarkerSet.end()) {
                if (marker->isExtended()) {
                    checkForExtensions = true;
                }
                markerList.push_back(marker);
            }

        }
    }
    
    
    printlog(LOG_EXT, "There are initially %lu markers to detect \n", markerList.size());
    
    // However it was generated, if extensions are being used in this set, then need to add those to the list as well
    if (checkForExtensions) {
        
        
        printlog(LOG_EXT, " All root markers to detect: \n");
        
        for (std::shared_ptr<Marker> marker : markerList) {
            printlog(LOG_EXT, "Marker %s \n", marker->name.c_str());
        }
        
        // If any of these markers have extensions, then try to detect those too!
        
        std::vector<std::shared_ptr<Marker> > extensionList;
        for (std::shared_ptr<Marker> marker : markerList) {
            // don't worry if it has a parent...
            // in fact, if an extneded marker (something with a parent) has got into the list at this point, then this violates my design
            assert(marker->extensionParent == nullptr);
            
            // for all extensions of this marker!
            // it's basically a linked list
            std::shared_ptr<Marker> extension = marker->extendedMarker;
            while (extension != nullptr) {
                extensionList.push_back(extension);
                extension = extension->extendedMarker;
            }
            
        }
        
        // if there are extensions, add them to the list
        if (extensionList.size() > 0) {
            printlog(LOG_EXT, "Adding %lu extended markers to the list to be detected! \n", extensionList.size());
            markerList.reserve(markerList.size() + extensionList.size());
            for (auto e : extensionList) {
                printlog(LOG_EXT, "Extended marker %s \n", e->name.c_str());

                markerList.push_back(e);
            }
        }
        printlog(LOG_EXT, "There are now %lu markers to detect! \n", markerList.size());
    }
    else {
        printlog(LOG_EXT, "There is no need to look for extensions on these markers \n");

    }
    
    
    return markerList;
}

std::vector<std::vector<cv::Point2f> > PlanarTracker::getTrackedBoundaries()
{
    std::vector<std::vector<cv::Point2f> > allTrackedBoundaries;
    size_t T = trackedMarkers.size();
    allTrackedBoundaries.reserve(T);
    for (size_t t = 0; t < T; t++) {

        std::shared_ptr<TrackedMarker> trackedMarker = trackedMarkers[t];
        cv::Mat H = trackedMarker->homography->getFullHomography();
        int w = trackedMarker->marker->image.size().width;
        int h = trackedMarker->marker->image.size().height;
        std::vector< cv::Point2f > markerBoundary = { cv::Point2f(0,0),cv::Point2f(0,h),cv::Point2f(w,h),cv::Point2f(w,0) };

        std::vector< cv::Point2f > cameraBoundary;
        cv::perspectiveTransform(markerBoundary, cameraBoundary, H);
        allTrackedBoundaries.push_back(cameraBoundary);
    }
    return allTrackedBoundaries;
}


void PlanarTracker::detectMarkersNow()
{
    
    printlog(LOG_DETECT," WARNING: Running detection in the same thread as tracking \n");
    // Do NOT clear the tracked markers. They will clear themselves.
    
    std::vector<std::shared_ptr<Marker> > markersToDetect = getMarkersToDetect();
    std::vector<std::vector<cv::Point2f> > trackedBoundaries = getTrackedBoundaries();

        
    detectMarkers(markersToDetect, trackedBoundaries);
    
    // verify on CURRENT image
    verifyDetections(currentCameraImage);
    
}

void PlanarTracker::detectMarkers(std::vector<std::shared_ptr<Marker>> markersToDetect, std::vector<std::vector<cv::Point2f> > maskRegions)
{
    
    Profiler timer;
    timer.start();
    
    assert(currentCameraImage != nullptr);
    
    // The core of detection is just this:
    markerDetector.setCameraImage(currentCameraImage); // pass a pointer to the current image. It's a smart pointer so it should be fine...
    markerDetector.detectMarkers(markersToDetect, maskRegions);

    // store for visualisation only:
    lastKeypoints = markerDetector.getKeypoints();
    
    std::vector<std::shared_ptr<Marker> > failedDetections = markerDetector.getFailedDetections();
    for (int f = 0; f < failedDetections.size(); f++) {
        std::shared_ptr<Marker> marker = failedDetections[f];

        if (marker->isFlowRecoverable(useFlowRecovery)) {
            
            cv::Mat bestRecoveredHomography = marker->localiseByFlowKeyframes(markerDetector.getCameraImage()->getQuarter());
            
            if (!bestRecoveredHomography.empty()) {
                MarkerDetectorResult result(marker);

                result.homography = std::make_shared<Homography>();
                result.homography->setQuarterHomography(bestRecoveredHomography);
                result.fromFlowRecovery = true;
                
                printlog(LOG_TRACK,"Set fromFlowRecovery = %i \n", int(result.fromFlowRecovery));
                markerDetector.addResult(result);
            }

        }
    }
    

    
    timer.end();
    float detTime = timer.getTimeTaken();
    printtime("PlanarTracker::detectMarkers done in %f s \n", detTime);
    
    // If this is being called from a thread, unlock this to say that it's finished
    if (doThreadedDetection) {
        detectionMutex.unlock();
    }
}

void PlanarTracker::detectMarkersInBackground()
{
    
    printlog(LOG_DETECT,"Detection running in its own thread \n");
    // if it CAN be locked (return true) then the detector is not running - go ahead
    bool isLocked = detectionMutex.try_lock();
    if (isLocked) {
        
        // it's not threaded at the moment, so you can gather the IDs of markers to detect safely
        std::vector<std::shared_ptr<Marker> > markersToDetect = getMarkersToDetect();
        std::vector<std::vector<cv::Point2f> > trackedBoundaries = getTrackedBoundaries();
        
        // If you can lock it, but no detection thread exists, it was the first run
        if (detectionThread.joinable()) {
            detectionThread.join();
            
        }
        
        // the detect markers function unlocks the mutex
        detectionThread = std::thread(&PlanarTracker::detectMarkers, this, markersToDetect, trackedBoundaries);

    }
}


std::shared_ptr<TrackedMarker> PlanarTracker::setupNewTrackedMarker(std::shared_ptr<Marker> marker, std::shared_ptr<CameraImage> cameraImage, std::shared_ptr<Homography> homography) {
    
               
    std::shared_ptr<TrackedMarker> trackedMarker = std::make_shared<TrackedMarker>();
    trackedMarker->patchMatcher.setMarker(marker);
    trackedMarker->patchMatcher.setCameraImage(cameraImage);
    
    trackedMarker->marker = marker;

    
    
    trackedMarker->patchMatcher.setHomography(homography);
    trackedMarker->patchMatcher.searchRadius = 16;
    trackedMarker->patchMatcher.previousHomography = nullptr;

    
    bool isMatched = trackedMarker->patchMatcher.match();
    if (isMatched) {
        trackedMarker->patchMatcher.resolution = CameraImage::CameraImageFull;
        
        glm::vec3 translation;
        glm::quat orientation;
        

        trackedMarker->resetHomography();
        trackedMarker->setHomography(trackedMarker->patchMatcher.getHomography());
        
        // this is a new tracked marker, so it needs an explicit new optical flow object
        trackedMarker->opticalFlowTracker = std::make_shared<OpticalFlowTracker>();
        trackedMarker->opticalFlowTracker->previousPyramid.clear();
        trackedMarker->opticalFlowTracker->previousPointsInCamera = trackedMarker->patchMatcher.patchPointsInCamera;
        trackedMarker->opticalFlowTracker->previousPointsInMarker = trackedMarker->patchMatcher.patchPointsInMarker;
        trackedMarker->opticalFlowTracker->setCurrentCameraImage(cameraImage);

        return trackedMarker;
    }
    return nullptr;
}

void PlanarTracker::verifyDetections(std::shared_ptr<CameraImage> cameraImage)
{
    // This should ALWAYS be called after the marker detector has finished detecting
    // So there will be no thread conflicts when accessing the data
    

    if (markerDetector.getNumberOfDetectedMarkers() > 0) {
        
        std::vector<MarkerDetectorResult> &results = markerDetector.getDetectedMarkers();
        printlog(LOG_TRACK,"Verifying %lu detections from the past at frame %i , currently %lu tracked markers \n", results.size(), cameraImage->timestamp, trackedMarkers.size());

        std::vector<std::shared_ptr<TrackedMarker>> verifiedMarkers;
        verifiedMarkers.reserve(results.size());
        numDetectionsMade = (int)results.size();
        

        
        for (size_t r = 0, R = results.size(); r < R; r++) {
            MarkerDetectorResult &result = results[r];
            
            
            // check:
#ifdef SHOW_DEBUG
            for (int t = 0; t < trackedMarkers.size(); t++) {
                if (trackedMarkers[t]->marker == result.marker) {
                    printdebug("Marker %s is already tracked. This should not have happened! \n",result.marker->name.c_str() );

                }
            }
#endif
            
            
            std::shared_ptr<TrackedMarker> trackedMarker = setupNewTrackedMarker(result.marker, cameraImage, result.homography);
            if (trackedMarker) {



                const bool useMarkerHomography = false;
                
                poseEstimator.setMarker(trackedMarker->marker);


                
                if (useMarkerHomography == true) {
                    poseEstimator.setHomography(result.homography);
                } else {
                    poseEstimator.setHomography(trackedMarker->patchMatcher.getHomography());
                }
                poseEstimator.doubleExponentialSmoothing.reset();
                poseEstimator.estimatePose(result.position, result.orientation);
                
                // did this come from optical flow?
                trackedMarker->fromFlowRecovery = result.fromFlowRecovery;
                printlog(LOG_TRACK,"Have set trackedMarker.fromFlowRecovery = %i \n",int(trackedMarker->fromFlowRecovery));


                
                verifiedMarkers.push_back(trackedMarker);
                printlog(LOG_TRACK,"Verified %s, there are now %lu \n",trackedMarker->marker->name.c_str(),verifiedMarkers.size());


                
                if (verifierChoiceMethod == VERIFY_FIRST) {
                    printlog(LOG_TRACK,"Verify first only, and have one in the list - so stop now! \n");
                    break;
                }
            }
            else {
                
                printlog(LOG_TRACK,"Marker %s failed verification  \n", result.marker->name.c_str());
            }
            
        }
        printlog(LOG_TRACK,"Verified %lu markers \n", verifiedMarkers.size());
        
        
        if (verifiedMarkers.size() > 0) {
            
            // if you actually want ALL of them, or there is one, pass the whole list
            if (verifierChoiceMethod == VERIFY_ALL || verifiedMarkers.size() == 1) {

                
#ifdef SHOW_DEBUG
                for (int t = 0; t < trackedMarkers.size(); t++) {
                    for (int v = 0; v < verifiedMarkers.size(); v++) {
                        if (trackedMarkers[t]->marker == verifiedMarkers[v]->marker) {
                            printdebug("DUPLICATE TRACK for verified marker %i = %s \n",v,verifiedMarkers[v]->marker->name.c_str());
                        }
                    }
                }
#endif
                
                trackedMarkers.insert( trackedMarkers.end(), verifiedMarkers.begin(), verifiedMarkers.end() );
                printlog(LOG_TRACK,"Added ALL %lu verified markers to tracked markers, thre are now %lu \n", verifiedMarkers.size(), trackedMarkers.size());
                numDetectionsKept += verifiedMarkers.size();
            }
            else if (verifierChoiceMethod == VERIFY_BEST) {
                
                
                printlog(LOG_TRACK,"There are %lu verified tracked markers. ... \n", verifiedMarkers.size());
                
             
                // Pick the best one:

                int bestVerifiedId = -1;
                float bestVerifiedScore = 0;
                for (int v = 0; v < verifiedMarkers.size(); v++) {

                    float vScore = verifiedMarkers[v]->getScore();
                    
                    if (vScore > bestVerifiedScore || bestVerifiedId < 0) {
                        bestVerifiedId = v;
                        bestVerifiedScore = vScore;
                    }
                }
                
                std::shared_ptr<TrackedMarker> trackedMarker = verifiedMarkers[bestVerifiedId];
#ifdef SHOW_DEBUG

                for (int t = 0; t < trackedMarkers.size(); t++)
                    if (trackedMarkers[t]->marker == trackedMarker->marker)
                        printdebug("DUPLICATE TRACK for marker %s \n",trackedMarker->marker->name.c_str());
#endif
                
                // keep the top one only
                numDetectionsKept++;

                trackedMarkers.push_back( trackedMarker );
                printlog(LOG_TRACK,"Have chosen the best at position %i with score %f, there are now %lu tracked markers \n",bestVerifiedId,bestVerifiedScore,trackedMarkers.size());


            }
            else
                printerr("This should not happen. verifierChoiceMethod %i not valid \n", verifierChoiceMethod);
        }
        
        

        // make sure you don't get them again on the next frame!
        markerDetector.clearDetectedMarkers();
        
    }
}


bool PlanarTracker::addMarker(std::shared_ptr<Marker> marker)
{
    
    if (marker->isMarkerValid()) {
        

        markerQueueMutex.lock();
        // Test if it is a good marker here
        newMarkerQueue.push(marker);
        newMarkerQueueEmpty = false;
        markerQueueMutex.unlock();
        
        
        printlog(LOG_DETECT, "Eneuqueuing a valid marker , now have %lu \n", newMarkerQueue.size());
        
        return true;
        
    }
    else {
        return false;
        
    }
    
}


void PlanarTracker::removeMarker(std::shared_ptr<Marker> marker)
{
    markerQueueMutex.lock();
    removeMarkerQueue.push(marker);
    removeMarkerQueueEmpty = false;
    markerQueueMutex.unlock();

}

void PlanarTracker::processMarkerQueues()
{
    if (!newMarkerQueueEmpty) {
        printlog(LOG_DETECT, "Processing markers queues: new marker queue not empty \n");
        
        // Get exclusive access to the new marker queue, add each to the marker detector, whiel removing from queue:
        markerQueueMutex.lock();

        while (!newMarkerQueue.empty()) {
         
            std::shared_ptr<Marker> newMarker = newMarkerQueue.front();
            
            
             markerDetector.addMarker(newMarker);
            
            
            newMarkerQueue.pop();
        }
        newMarkerQueueEmpty = true;
        markerQueueMutex.unlock();

    }
    
    if (!removeMarkerQueueEmpty) {
        printlog(LOG_DETECT, "Processing markers queues: removal marker queue not empty \n");
        
        // Get exclusive access to the new marker queue, add each to the marker detector, while removing from queue:
        markerQueueMutex.lock();
        
        
        
        
        while (!removeMarkerQueue.empty()) {
            
            std::shared_ptr<Marker> markerToRemove = removeMarkerQueue.front();
            
            
            markerDetector.removeMarker(markerToRemove);
            
            
            removeMarkerQueue.pop();
        }
        
        removeMarkerQueueEmpty = true;

        markerQueueMutex.unlock();

    }
}


// Extended Markers
bool PlanarTracker::trackedMarkersUseExtensions()
{
    // are any of the markers here extesible
    for (auto t : trackedMarkers) {
        if (t->marker->canBeExtended() || t->marker->isExtended() || t->marker->isExtension()) {
            return true;
        }
    }
    return false;
}

bool PlanarTracker::anyMarkersUseExtensions()
{
    // are any of the markers here extesible
    for (auto marker : markerDetector.getMarkers()) {
        if (marker->canBeExtended() || marker->isExtended() || marker->isExtension()) {
            return true;
        }
    }
    return false;
}



/** There are various things that a traked marker must have/be to be able to be extended.
 This checks each of these and returns true only if the marker has passed them all
 */
bool extensionCriteriaSatisfied(std::shared_ptr<TrackedMarker> trackedMarker)
{
    // Old enough for extension?
    if (trackedMarker->lifetime < ExtendedMarkerSettings::minExtensionLifetime) {
        return false;
    }
    
    // Is this using just optical flow? In which case, not good enough for extension
    if (trackedMarker->framesInOpticalFlow > 0) {
        return false;
    }
    
    // If this is not smooth, then extension should not be attempted
    if (!trackedMarker->isSmooth) {
        return false;
    }
    
    // If these criteria are met, then extension can be done
    return true;
}

int PlanarTracker::extendMarkers(cv::Mat image)
{

    int numExtensions = 0;
    
    for (std::shared_ptr<TrackedMarker> trackedMarker : trackedMarkers) {
        
        if (extensionCriteriaSatisfied(trackedMarker)) {
            std::shared_ptr<Marker> marker = trackedMarker->marker;
            
            if (marker->canBeExtended()) {
                
                bool created = createExtension(image, marker, trackedMarker);
                
                numExtensions += created;
            }
            else {
                printlog(LOG_EXT, "Marker %s can't be extended \n", trackedMarker->marker->name.c_str());
            }
        }
        else {
            printlog(LOG_EXT, "Marker %s doesn't meet extension criteria yet \n", trackedMarker->marker->name.c_str());
        }
        
        
    }
    
    return numExtensions;
    
}

/** Checks whether a boundary (list of 2D corners) is entirely within an image (specified by a size, i.e. at the origin) or not
 @param imageSize Size representing the image extent ((0,0) is the top left)
 @param boundary A vector of points representing the (anticlockwise) boundary of a region
 @return True iff all of the boundary points are within the rectangle bounded by (0,0) and imageSize
 */
bool boundaryInsideImage(cv::Size imageSize, std::vector<cv::Point2f> boundary)
{
    for (int b = 0; b < boundary.size(); b++) {
        if (boundary[b].x < 0 || boundary[b].y < 0 || boundary[b].x >= imageSize.width || boundary[b].y >= imageSize.height) {
            return false;
        }
    }
    return true;
}


/** For assessing extended marker candidates. This function checks how much like a square/rectangle the given region is, by looking at its corner angles.
 All the angles must be reasonably close to 90 degrees for it to count as sufficiently square-like. This is a proxy for the region being viewed front-on
 @param boundary An anticlockwise boundary (four points) representing a region in the image.
 @param angleThreshold The maximum permitted difference between each angle and a right angle
 @return Whether this boundary is sufficiently square-like or not
 */
bool checkBoundarySquareness(std::vector<cv::Point2f> boundary, float angleThreshold)
{
    // if there is no threshold, it's fine
    if (angleThreshold <= 0) return true;
 
    // check all corners:
    
    for (int a = 0; a < boundary.size(); a++) {
        
        int ap = (a+1) % (int) boundary.size();
        int an = (a-1 + (int) boundary.size()) % (int) boundary.size();
        cv::Point2f v0 = boundary[ap]-boundary[a];
        cv::Point2f v1 = boundary[an]-boundary[a];
        
        float arad = acos(v0.dot(v1) / (cv::norm(v0)*cv::norm(v1)));
        float adeg = 180*arad/M_PI;
        
        // if the difference between this angle and a right angle is large, this is not much like a square
        if (fabs(90-adeg) >= angleThreshold) {
            return false;
        }
    }
    
    return true;

    
    
}

/**
 Some checks to perform on a potential new marker boundary (expressed in the image space) before being allowed to be an extension
 */
bool extendedBoundaryChecks(cv::Size imageSize, std::vector<cv::Point2f> extendedBoundary)
{
    // All the extended boundary corners must be inside the camera image:

    if (!boundaryInsideImage(imageSize, extendedBoundary)) return false;

    if (!checkBoundarySquareness(extendedBoundary, ExtendedMarkerSettings::extensionCornerAngleThreshold)) return false;
    
        
    return true;
}


bool PlanarTracker::checkNewMarker(std::shared_ptr<Marker> marker)
{
    // There must be enough keypoints, i.e. the new marker needs to be sufficiently salient
    if (marker->keypoints.size() <= ExtendedMarkerSettings::minDescriptors) {
        return false;
    }
    
    // There need to be enough tracking image leves - this should almost always be the case...
    if (marker->trackingImages.size() <= 1) {
        return false;
    }
    
    // But is needed to make sure that there are enough tracking keypoints in this level
    if (marker->trackingImages[1].keypoints.size() <= ExtendedMarkerSettings::minTrackingPoints) {
        return false;
    }

    return true;
}


/**
 Each marker has a name, and this is also the case with extended markers. Since extended markers are created automatically from markers, they have the same name, but with 'Ext #id' appended, where id is the number of times the root marker has been extended. This function checks whether the marker being extended from is a root (in which case it's just marker name + 1) or not (in which case the name is assigned w.r.t the root marker and the count is incremented)
 */
std::string getExtensionName(std::shared_ptr<Marker> marker)
{
    
    // if it came from an extension, then read the number and increment it. Otherwise, just add 'Ext. 1'
    size_t extPos = marker->name.find("Ext.");
    if (extPos == std::string::npos) {
        
        return marker->name + " Ext. 1";
    }
    else {
        // This is a bit messy. Get the name without the ' Ext .#' part
        std::string lastNameOnly = marker->name.substr(0, extPos);
        // Then get the bit after 'Ext. ' which should just be a number
        std::string lastExtNumStr = marker->name.substr(extPos + std::string("Ext. ").size(), std::string::npos);
        
        // If not, there will be an exception here, where this is converted to a number
        int lastExtNum = std::stoi(lastExtNumStr);
        
        // Now, set the name to be the same as the name without the ' Ext...' part, plus 'Ext. ' and the NEXT number
        return lastNameOnly + "Ext. " + std::to_string(lastExtNum+1);
    }
    
}

bool PlanarTracker::createExtension(cv::Mat image, std::shared_ptr<Marker> marker, std::shared_ptr<TrackedMarker> trackedMarker)
{
    assert( trackedMarker->homography != nullptr);
    cv::Mat trackedHomography = trackedMarker->homography->getFullHomography();
    if (trackedHomography.rows != 3 || trackedHomography.cols != 3) {
        return false;
    }
    
    
    // Compse the tracked homography (marker -> image) with the extension homography (image -> offset)
    cv::Mat XH = trackedHomography * marker->extensionHomography;
    
    // Use this to get the boundary of the extension region in the current image:
    std::vector<cv::Point2f> extendedBoundary;
    cv::perspectiveTransform(marker->trackedBoundary, extendedBoundary, XH);

    
    // The boundary needs to fulfill various criteria (e.g. visibility)
    bool boundaryIsOk = extendedBoundaryChecks(image.size(), extendedBoundary);
    
    if (!boundaryIsOk) {
        return false;
    }
    
    // Get the warped image from the extended boundary region (by warping the whole image using the inverse homography)
    cv::Mat XHi = XH.inv();
    cv::Mat newMarkerImage;
    cv::warpPerspective(image, newMarkerImage, XHi, marker->image.size());
    
    
    // Create a marker directly from this image:
    std::shared_ptr<Marker> newMarker = std::make_shared<Marker>(newMarkerImage);
    
#ifdef DRAW_EXTENSIONS
    cv::namedWindow("New extension");
    cv::imshow("New extension", newMarkerImage);
#endif

    // The new marker needs to be checked:
    if (!checkNewMarker(newMarker)) {
        return false;
    }
    
    // the marker is OK, so set it as being the extension of the marker we started from:
    marker->extendedMarker = newMarker;
    
    // And set the link in the opposite direction too:
    newMarker->extensionParent = marker;
    
    // Store a pointer to the origin of all these extensions
    if (marker->extensionRoot == nullptr) {
        // If this marker doesn't have a root, then it is the root of everything
        newMarker->extensionRoot = marker;
    }
    else {
        // Otherwise, point to the same place
        newMarker->extensionRoot = marker->extensionRoot;
    }
    
    
    // BOUNDARIES:
    // A marker inherits its virtual boundary from its parent:
    newMarker->virtualBoundary = marker->virtualBoundary;
    // A marker inherits the display boundary, but transforms it by the INVERSE of the homography which extended it, i.e. the display boundary should look like the boundary of the original marker
    cv::perspectiveTransform(marker->displayBoundary, newMarker->displayBoundary, marker->extensionHomography.inv());

    
    // the new marker is extensible too, so as to keep going:
    newMarker->setExtensible(true);

    newMarker->name = getExtensionName(marker);
    
    printlog(LOG_EXT, "Created a marker called %s \n", newMarker->name.c_str());
    
    return true;
}



/** 
 Get the mean error between corners of boundary and candidate. Get the least possible error, given all associations.
 This is to find the difference between a marker region and a potential marker region, for deiding whether it's worth switching trackables
 */
float getBoundaryDistance(std::vector<cv::Point2f> boundary, std::vector<cv::Point2f> candidate)
{
    assert (candidate.size() == boundary.size());
    
    size_t N = boundary.size();
    float minError = 0;
    // so there are N possible orientations for N corners
    for (int i = 0; i < N; i++) {
        
        float totalError = 0;
        for (int c = 0; c < N; c++) {
            float err = cv::norm(boundary[ (i + c) % N] - candidate[c]);
            totalError += err;
            
            // if it's already exceeeded the min without many connections, skip the rest (unless it's the first time around)
            if (i > 0 && totalError > minError) {
                break;
            }
        }
        if (totalError < minError || i == 0) {
            minError = totalError;
        }
    }
    return minError/float(N);
    
}


#ifdef DRAW_EXTENSIONS
/** Given an image and an anticlockwise lists of corner points, this draws the boundary on the image. For debugging only!
 */
void drawBoundary(cv::Mat image, std::vector<cv::Point2f> &boundary, cv::Scalar colour, float lineWidth)
{
    if (boundary.size() > 1) {
        for (int i = 0; i < boundary.size(); i++) {
            int ii = (i+1)%boundary.size();
            cv::line( image, boundary[i], boundary[ii], colour, lineWidth);
        }
    }
    
}
#endif

void PlanarTracker::switchTrackedExtensions(cv::Mat image)
{
    // for any tracked markers which have extensions, see if they could be switched
    
    for (int t = 0; t < trackedMarkers.size(); t++) {
        std::shared_ptr<TrackedMarker> trackedMarker = trackedMarkers[t];
        std::shared_ptr<Marker> marker = trackedMarker->marker;
        assert(marker != nullptr);
        assert(trackedMarker->homography != nullptr);
        
            
        
        if (marker->isExtended() || marker->isExtension()) {
            printlog(LOG_EXT, "Marker %s could try to switch \n", marker->name.c_str());
        }
        
        
        cv::Mat H = trackedMarker->homography->getFullHomography();
        if (H.rows == 3 && H.cols == 3) {
            
            // no extensions reachable
            if (marker->extensionParent == nullptr && marker->extendedMarker == nullptr) {
                continue;
            }
            
            // Choose the best marker of these to track
            // Imagine if the marker image was in the centre of the image - which projected marker would be the closest?
            
            cv::Point2f offsetPt(image.size().width/2 -  marker->image.size().width/2,
                                 image.size().height/2 - marker->image.size().height/2);
            
            // add offset to the stored marker boundary:
            std::vector<cv::Point2f> markerBoundaryOnImage = marker->trackedBoundary;
            for (int m = 0; m < markerBoundaryOnImage.size(); m++) {
                markerBoundaryOnImage[m] += offsetPt;
            }
            
            
            
            
            // check parent and child individually, and compare to the current marker
            
            
            std::vector<cv::Point2f> currentMarkerBoundary;
            cv::perspectiveTransform(marker->trackedBoundary, currentMarkerBoundary, H);
            
            // Get the error of the current boundary compared to the marker image. The multiplier controls how much the current one is favoured: if switchResistance is high, then the error is multiplied by a smaller number, so is harder to beat
            float currentBoundaryError = getBoundaryDistance(markerBoundaryOnImage, currentMarkerBoundary) * (1.f-ExtendedMarkerSettings::switchResistance);
            
            
            std::shared_ptr<Marker> bestExtendedMarker = nullptr;
            cv::Mat bestExtendedHomography;

#ifdef DRAW_EXTENSIONS
            std::vector<cv::Point2f> parentBoundary, extensionBoundary;
            float parentBoundaryError, extensionBoundaryError;
#endif
            
            if (marker->extensionParent) {
                
                // then the homography from the current tracked marker to its parent is the inverse of its parent's extension hmography
                cv::Mat X = marker->extensionParent->extensionHomography.inv();
                
                // then where would that be in the current image?
                cv::Mat HX = H*X;
                
                std::vector<cv::Point2f> switchCandidateBoundary;
                cv::perspectiveTransform(marker->trackedBoundary, switchCandidateBoundary, HX);
                
                float boundaryError = getBoundaryDistance(markerBoundaryOnImage, switchCandidateBoundary);

                if (boundaryError < currentBoundaryError) {
                    bestExtendedMarker = marker->extensionParent;
                    bestExtendedHomography = HX;
                }
                
#ifdef DRAW_EXTENSIONS
                parentBoundary = switchCandidateBoundary;
                parentBoundaryError = boundaryError;
#endif
            }
            
            
            if (marker->extendedMarker) {
                
                // In future, if there were multiple extended markers, loop through them here.
                
                // then the homography from the current tracked marker to its extensions is simply its extension hmography
                cv::Mat X = marker->extensionHomography;
                
                // then where would that be in the current image?
                cv::Mat HX = H*X;
                
                std::vector<cv::Point2f> switchCandidateBoundary;
                cv::perspectiveTransform(marker->trackedBoundary, switchCandidateBoundary, HX);
                
                float boundaryError = getBoundaryDistance(markerBoundaryOnImage, switchCandidateBoundary);
                
                if (boundaryError < currentBoundaryError) {
                    bestExtendedMarker = marker->extendedMarker;
                    bestExtendedHomography = HX;
                }
                
#ifdef DRAW_EXTENSIONS
                extensionBoundary = switchCandidateBoundary;
                extensionBoundaryError = boundaryError;
#endif
                
            }
            
            
            
#ifdef DRAW_EXTENSIONS

            // draw it for now
            cv::namedWindow("Switching");
            cv::Mat switchImage;
            cv::cvtColor(image, switchImage, CV_GRAY2BGR);
            
            drawBoundary(switchImage, markerBoundaryOnImage, cv::Scalar(0,0,200), 1);
            
            
            drawBoundary(switchImage, currentMarkerBoundary, cv::Scalar(0,255-currentBoundaryError,0), 2);
            
            drawBoundary(switchImage, parentBoundary, cv::Scalar(255-parentBoundaryError,0,75), 2);
            drawBoundary(switchImage, extensionBoundary, cv::Scalar(0,255-extensionBoundaryError,255-extensionBoundaryError), 2);
            
            
            cv::imshow("Switching", switchImage);
#endif
       
            
           
            
            // If the best marker exists and is NOT the current marker:
            if (bestExtendedMarker != nullptr) {
                
                printlog(LOG_EXT, "Switching to marker %s \n", bestExtendedMarker->name.c_str());
                
                std::shared_ptr<Homography> newHomography = std::make_shared<Homography>();
                newHomography->setFullHomography(bestExtendedHomography);
                
                std::shared_ptr<TrackedMarker> newTrackedMarker = setupNewTrackedMarker(bestExtendedMarker, currentCameraImage, newHomography);

                if (newTrackedMarker) {
                    
                    // set the pose too! just set it to whatever was on the previous marker. no reason this should make sense, it's just a pose now...
                    newTrackedMarker->position = trackedMarker->position;
                    newTrackedMarker->orientation = trackedMarker->orientation;
                    
                    // replace in the list!
                    trackedMarkers[t] = newTrackedMarker;
                }
                
            }

        }
    }
}

NamespaceEnd
