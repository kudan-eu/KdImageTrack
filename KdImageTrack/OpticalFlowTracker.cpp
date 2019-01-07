//
//  OpticalFlowTracker.cpp
//  PlanarTracker
//
//  Created on 06/03/2015.
//  Copyright (c) 2015 Kudan. All rights reserved.
//

#include "OpticalFlowTracker.h"
#include "Helper.h"


#include "Logging.hpp"

Namespace(KdImageTrack)

cv::Mat OpticalFlowTracker::getEstimatedHomography()
{
    cv::Mat prevHomog;
    cv::Mat currentHomog;
    
    switch (resolution) {
        case CameraImage::CameraImageFull:
            prevHomog = previousHomography->getFullHomography();
            currentHomog = homography->getFullHomography();
            break;
        case CameraImage::CameraImageHalf:
            prevHomog = previousHomography->getHalfHomography();
            currentHomog = homography->getHalfHomography();
            break;
        case CameraImage::CameraImageQuarter:
            prevHomog = previousHomography->getQuarterHomography();
            currentHomog = homography->getQuarterHomography();
            break;
    }
    
    
    if (prevHomog.empty()) {
        return cv::Mat();
    }
    
    if (currentHomog.empty()) {
        return cv::Mat();
    }
    
    cv::Mat lerpedHomog = lerpHomography(prevHomog, currentHomog, 2);
    
    lerpedHomog = lerpedHomog * currentHomog.inv();
    return lerpedHomog;
}


bool OpticalFlowTracker::trackShared(std::vector<std::shared_ptr<OpticalFlowTracker>> trackers, std::shared_ptr<CameraImage> image)
{
    
    
    // set the current image on all trackers
    size_t totalNumPoints = 0;
    for (int t = 0; t < trackers.size(); t++) {
        
        std::shared_ptr<OpticalFlowTracker> tracker = trackers[t];
        tracker->setCurrentCameraImage(image);
        /// count how many points you're going to need, to reserve
        totalNumPoints += tracker->previousPointsInMarker.size();
    }
    
    printlog(LOG_FLOW,"DOing shared optical flow, have %lu trackers \n", trackers.size());
    if (trackers.size() == 0) {
        return false;
    }
    
    // They must all have the same current frame, so pick the first one (this should be verified before calling the function)
    std::shared_ptr<OpticalFlowTracker> firstTracker = trackers[0];
    
    cv::Mat previousImage;
    cv::Mat currentImage;
    
    // assume each tracker is at the same resolution (currently hardcded to Quarter):
    CameraImage::CameraResolution resolution = firstTracker->resolution;
    
    const int minimumPointsRequired = 10;

    
    switch (resolution) {
        case CameraImage::CameraImageFull:
            previousImage = firstTracker->previousCameraImage->getFull();
            currentImage = firstTracker->currentCameraImage->getFull();
            break;
        case CameraImage::CameraImageHalf:
            previousImage = firstTracker->previousCameraImage->getHalf();
            currentImage = firstTracker->currentCameraImage->getHalf();
            break;
        case CameraImage::CameraImageQuarter:
            previousImage = firstTracker->previousCameraImage->getQuarter();
            currentImage = firstTracker->currentCameraImage->getQuarter();
            break;
    }

    // for each tracker, get all its previous points, estimate homographies, and warp:
    
    std::vector<cv::Point2f> previousPoints;
    std::vector<cv::Point2f> points;
    
    
    std::vector<cv::Point2f> allPreviousPointsInMarker;

    
    std::vector<int> trackerIdsForPoints;
    std::vector<size_t> numPointsPerTracker;
    
    
    std::vector<unsigned char> status;
    std::vector<float> error;
    
    // reserve space in all lists:
    
    previousPoints.reserve(totalNumPoints);
    points.reserve(totalNumPoints);
    
    allPreviousPointsInMarker.reserve(totalNumPoints);
    trackerIdsForPoints.reserve(totalNumPoints);
    numPointsPerTracker.reserve(totalNumPoints);


    int numValidTrackers = 0;
    
    
    for (int t = 0; t < trackers.size(); t++) {
        std::shared_ptr<OpticalFlowTracker> tracker = trackers[t];
        cv::Mat estimatedHomography = tracker->getEstimatedHomography();
        
        // Any tracker which is passed in should have a revious homography - because it is necessary to warp points as predictions for the optical flow
        // If this is NOT the case, then skip this one, and just track on its own later
        if (estimatedHomography.empty()) {
            continue;
        }
        
        std::vector<cv::Point2f> trackerPreviousPoints;
        switch (resolution) {
            case CameraImage::CameraImageFull:
                trackerPreviousPoints = tracker->previousPointsInCamera.getFullPoints();
                break;
            case CameraImage::CameraImageHalf:
                trackerPreviousPoints = tracker->previousPointsInCamera.getHalfPoints();
                break;
            case CameraImage::CameraImageQuarter:
                trackerPreviousPoints = tracker->previousPointsInCamera.getQuarterPoints();
                break;
        }
        
        // If this tracker doesn't have enough points, skip it. This means no flow will be calculated for it
        if (trackerPreviousPoints.size() < minimumPointsRequired) {
            continue;
        }

        printlog(LOG_FLOW, "At tracker %i, trackerPreviousPoints : %lu \n", t, trackerPreviousPoints.size());
        
        // Warp by homography:
        std::vector<cv::Point2f> trackerPoints;
        cv::perspectiveTransform(trackerPreviousPoints, trackerPoints, estimatedHomography);
        
        numPointsPerTracker.push_back(trackerPoints.size());
        
        // record the points, warped points, and marker points in one big list
        // record the list index of the tracker whence each point, for reassembly later
        for (int p = 0; p < trackerPoints.size(); p++) {
            previousPoints.push_back(trackerPreviousPoints[p]);
            points.push_back(trackerPoints[p]);
            allPreviousPointsInMarker.push_back( tracker->previousPointsInMarker[p] );
            trackerIdsForPoints.push_back(t);
        }
        
        numValidTrackers++;
    }
    

    // One could check here that there are indeed enough markers left (after possible skipping some above) to bother wth sharing flows
    // However, quitting now would just mean calling track() on everthing separately and re-doing the above computations
    
    // calculate optical flow at quarter camera resolution.
    cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01);
    
    const int pyramidLevels = 3;
    cv::Size winSize(5, 5);

    
    
    std::vector<cv::Mat> previousImagePyramid;
    std::vector<cv::Mat> currentImagePyramid;
    
    // Again look at the first one in the list (the oldest) for the saved pyramid
    if (firstTracker->previousPyramid.empty()) {
        cv::buildOpticalFlowPyramid(previousImage, firstTracker->previousPyramid, winSize, pyramidLevels);
    } else {
        // use pyramid generated in previous frame.
        previousImagePyramid = firstTracker->previousPyramid;
    }
    
    // Calculate the CURRENT pyramid, from the current image:
    cv::buildOpticalFlowPyramid(currentImage, currentImagePyramid, winSize, pyramidLevels);
    
    // Store the pyramid for the next frame. Store in ALL trackers: you'll only use the first one, but you don't know which will survive tracking
    for (int t = 0; t < trackers.size(); t++) {
        std::shared_ptr<OpticalFlowTracker> tracker = trackers[t];
        tracker->previousPyramid = currentImagePyramid;
    }

    // Assumed above that there is always a homography and predicted point set available
    int opticalFlowFlags = cv::OPTFLOW_USE_INITIAL_FLOW;

#ifdef DRAW_FLOW
    // save before flow, for drawing only:
    std::vector<cv::Point2f> predictedPoints = points;
#endif
    
    // =========== =========== =========== =========== ===========
    // calculate optical flow.
    cv::calcOpticalFlowPyrLK(previousImagePyramid, currentImagePyramid, previousPoints, points, status, error, winSize, pyramidLevels, criteria, opticalFlowFlags);
    // =========== =========== =========== =========== ===========
    
    printlog(LOG_FLOW,"Have done SHARED flow on set of %lu points \n", previousPoints.size());
    
#ifdef DRAW_FLOW
    bool showFlowImage = false;
    if (showFlowImage) {
        cv::Mat flowImage;
        cv::cvtColor(currentImage, flowImage, CV_GRAY2BGR);
        int sc = 4;
        
        cv::resize(flowImage, flowImage, flowImage.size()*sc, 0, 0, CV_INTER_NN);
        
        for (int c = 0; c < previousPoints.size(); c++) {
            cv::circle( flowImage, sc * predictedPoints[c], 4, cv::Scalar(255, 0, 200), 1);
            
            cv::circle( flowImage, sc * previousPoints[c], 3, cv::Scalar(255, 0, 0), 1);
            cv::circle( flowImage, sc * points[c], 5, cv::Scalar(0, 255, 0), 1);
            cv::line( flowImage, sc * previousPoints[c], sc * points[c], cv::Scalar(0, 255, 255), 1);
        }
        
        cv::namedWindow("Shared Flow");
        cv::imshow("Shared Flow", flowImage);
    }
#endif
    
    // Clear all the tracker points and reserve lists of the max possible size
    for (int t = 0; t < trackers.size(); t++) {
        std::shared_ptr<OpticalFlowTracker> tracker = trackers[t];
        // store these in the object now:
        tracker->currentFrameFeatures.clear();
        tracker->currentFrameMarkerFeatures.clear();
    
        tracker->currentFrameFeatures.reserve(numPointsPerTracker[t]);
        tracker->currentFrameMarkerFeatures.reserve(numPointsPerTracker[t]);
        tracker->isTracked = true;
    }
    
    const float errorThreshold = 15;
    

    // Now put the flow'd points back into their trackers
    for (int i = 0; i < points.size(); i++) {
        if (status[i] && error[i] < errorThreshold ) {
            
            // which tracker did this come from
            int trackerId = trackerIdsForPoints[i];
            
            // Recovering the reference for every point is a bit annoying, but shouldn't be much of a performance issue
            std::shared_ptr<OpticalFlowTracker> tracker = trackers[trackerId];
            cv::Point2f &point = points[i];
            
            // store the point in the correct tracker
            tracker->currentFrameFeatures.push_back(point);
            // Important: using index i, so this has to relate to the global list, so can't just use tracker.previousPointsInMarker[i] because that doesn't make sense
            tracker->currentFrameMarkerFeatures.push_back(allPreviousPointsInMarker[i]);

        }
    }
    
    return true;
    
}

bool OpticalFlowTracker::track()
{
    
    
    
    printlog(LOG_FLOW,"Optical flow between frames %i - %i \n", previousCameraImage->timestamp, currentCameraImage->timestamp);
    
    std::vector<cv::Point2f> points;
    std::vector<unsigned char> status;
    std::vector<float> error;
    
    const int minimumPointsRequired = 10;
    if (previousPointsInMarker.size() < minimumPointsRequired) {
        return false;
    }
        
    cv::Mat previousImage;
    cv::Mat currentImage;
    std::vector<cv::Point2f> previousPoints;
    
    switch (resolution) {
        case CameraImage::CameraImageFull:
            previousImage = previousCameraImage->getFull();
            currentImage = currentCameraImage->getFull();
            previousPoints = previousPointsInCamera.getFullPoints();
            break;
        case CameraImage::CameraImageHalf:
            previousImage = previousCameraImage->getHalf();
            currentImage = currentCameraImage->getHalf();
            previousPoints = previousPointsInCamera.getHalfPoints();
            break;
        case CameraImage::CameraImageQuarter:
            previousImage = previousCameraImage->getQuarter();
            currentImage = currentCameraImage->getQuarter();
            previousPoints = previousPointsInCamera.getQuarterPoints();
            break;
    }
    
    cv::Mat estimatedHomography = getEstimatedHomography();
    int opticalFlowFlags = 0;
    
    if (estimatedHomography.empty() == false) {
        cv::perspectiveTransform(previousPoints, points, estimatedHomography);
        opticalFlowFlags |= cv::OPTFLOW_USE_INITIAL_FLOW;
    }
    
    // calculate optical flow at quarter camera resolution.
    cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01);
 
    const int pyramidLevels = 3;
    cv::Size winSize(5, 5);
    
    std::vector<cv::Mat> previousImagePyramid;
    std::vector<cv::Mat> currentImagePyramid;
    
    if (previousPyramid.empty()) {
        cv::buildOpticalFlowPyramid(previousImage, previousImagePyramid, winSize, pyramidLevels);
    } else {
        // use pyramid generated in previous frame.
        previousImagePyramid = previousPyramid;
    }

    cv::buildOpticalFlowPyramid(currentImage, currentImagePyramid, winSize, pyramidLevels);
    
    // for the next frame.
    previousPyramid = currentImagePyramid;
    
    // calculate optical flow.
    cv::calcOpticalFlowPyrLK(previousImagePyramid, currentImagePyramid, previousPoints, points, status, error, winSize, pyramidLevels, criteria, opticalFlowFlags);

    // store these in the object now:
    currentFrameFeatures.clear();
    currentFrameMarkerFeatures.clear();
    
    currentFrameFeatures.reserve(points.size());
    currentFrameMarkerFeatures.reserve(points.size());

    const float errorThreshold = 15;
    
    for (int i = 0; i < points.size(); i++) {
        if (status[i] && error[i] < errorThreshold ) {
            cv::Point2f &point = points[i];
            
            currentFrameFeatures.push_back(point);
            currentFrameMarkerFeatures.push_back(previousPointsInMarker[i]);
        }
    }

    
    
    isTracked = true;
    return true;
}

bool OpticalFlowTracker::calculateHomography()
{

    isTracked = false;

    
    const int minimumTrackThreshold = 10;
    if (currentFrameFeatures.size() < minimumTrackThreshold) {
        printlog(LOG_FLOW," Not enough points \n");
        return false;
    }

    printlog(LOG_FLOW," Trying to calculate homography after flow with %lu features and %lu on marker \n", currentFrameFeatures.size(), currentFrameMarkerFeatures.size());

    

    
    // calculate homography.
    std::vector<unsigned char> inliersMask(currentFrameFeatures.size());

    cv::Mat homog = cv::findHomography(currentFrameMarkerFeatures, currentFrameFeatures, CV_RANSAC, 1, inliersMask, 500);
    
 
    if (homog.empty()) {
        printlog(LOG_FLOW," Empty homography \n");
        return false;
    }

    
    
    
    previousHomography = homography;
    homography = std::make_shared<Homography>();

    
    switch (resolution) {
        case CameraImage::CameraImageFull:
            previousPointsInCamera.setFullPoints(currentFrameFeatures);
            homography->setFullHomography(homog);
            break;
        case CameraImage::CameraImageHalf:
            previousPointsInCamera.setHalfPoints(currentFrameFeatures);
            homography->setHalfHomography(homog);
            break;
        case CameraImage::CameraImageQuarter:
            previousPointsInCamera.setQuarterPoints(currentFrameFeatures);
            homography->setQuarterHomography(homog);
            break;
    }
    
    previousPointsInMarker = currentFrameMarkerFeatures;
    
    
    return true;
}

NamespaceEnd
