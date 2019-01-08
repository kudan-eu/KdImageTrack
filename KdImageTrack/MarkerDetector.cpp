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

#include "MarkerDetector.h"
#include "MarkerDetectorResult.hpp"
#include "Helper.h"
#include "Profiler.hpp"

#include "Logging.hpp"
#include "HomographyFinding.hpp"

#include "IntelSSE_x86.hpp"


#include <thread>

Namespace(KdImageTrack)

//#define MAC_MODE


void MarkerDetector::extractFeatures(std::vector<std::vector<cv::Point2f>> maskRegions)
{
    const int numberOfPoints = 500;
    bool doMasking = maskRegions.size()>0;
    cv::Ptr<cv::ORB> orb = cv::ORB::create( doMasking? 1000000:numberOfPoints);
    
    // discard previous keypoints and descriptors.
    currentFrameKeypoints.clear();
    currentFrameDescriptors = cv::Mat();
    
    cv::Mat image = currentCameraImage->getFull();
    
    /* Illumination Compensation
     This normalises the image (so that it fills the range [0, 255] befire extracting descriptors, to cope with dark conditions which normally break ORB
     This does not affect the stored camera image, and will not affect anything other than descriptor extraction (like tracking)
     */
    cv::Mat normImg;
    cv::normalize(image, normImg, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    image = normImg; // This should make sure it never edits the original image inside currentCameraImage

    
    if (maskRegions.size()  > 0) {
        // If there are regions to mask out - detect ALL keypoints, then remove from the list any within the region
        
        printlog(LOG_DETECT,"MASKING KEYPOINTS with %lu regions \n", maskRegions.size());
        
        currentFrameKeypoints.reserve(numberOfPoints);
        std::vector<cv::KeyPoint> detectedKeypoints;
        // Detect full set of keypoints, then sort and mask
        orb->detect(image, detectedKeypoints);
        
        // sort keypoints
        std::sort(detectedKeypoints.begin(), detectedKeypoints.end(), [](cv::KeyPoint const &a, cv::KeyPoint const &b) {   return a.response > b.response;  });
                  
        
        for (int d = 0; d < detectedKeypoints.size(); d++) {
            bool masked = 0;
            for (int r = 0; r < maskRegions.size(); r++) {
                // It returns positive (inside), negative (outside), or zero (on an edge) value
                if ( cv::pointPolygonTest(maskRegions[r], detectedKeypoints[d].pt, 0) >= 0) {
                    masked = true;
                    break;
                }
            }
            if (!masked) {
                currentFrameKeypoints.push_back(detectedKeypoints[d]);
                if (currentFrameKeypoints.size() >= numberOfPoints) {
                    break;
                }
            }
        }
        printlog(LOG_DETECT,"From a total of %lu detected points have masked down to %lu \n", detectedKeypoints.size(), currentFrameKeypoints.size());
        
        if (currentFrameKeypoints.size() > 0) {
            // now compute descriptors for only the kept keypoints
            orb->compute(image, currentFrameKeypoints, currentFrameDescriptors);
        }
        
    }
    else {
        printlog(LOG_DETECT,"Not masking! \n");
        // detect keypoints (and compute descriptors) in current frame.
        orb->detectAndCompute(image, cv::Mat(), currentFrameKeypoints, currentFrameDescriptors);
    }
    
    if (currentFrameKeypoints.size() == 0) {
        return;
    }
    

}


std::vector<cv::DMatch> MarkerDetector::filterHamming(std::vector<cv::DMatch> &matches, float hammingThreshold)
{
    std::vector<cv::DMatch> tmpMatches;
    
    for (int i = 0; i < matches.size(); i++) {
        cv::DMatch &match = matches[i];
        
        if (match.distance < hammingThreshold) {
            tmpMatches.push_back(match);
        }
    }
    return tmpMatches;
}

std::vector<cv::DMatch> MarkerDetector::filterOrientation(std::vector<cv::DMatch> &matches, std::shared_ptr<Marker> marker, float angleDifference)
{
    std::vector<std::vector<cv::DMatch>> hist(36);
    std::vector<std::vector<float>> angles(36);
    
    for (int i = 0; i < matches.size(); i++) {
        cv::KeyPoint keypoint = currentFrameKeypoints[matches[i].queryIdx];
        cv::KeyPoint keypoint2 = marker->keypoints[matches[i].trainIdx];
        
        float keypointAngle = keypoint2.angle - marker->keypointOrientations[matches[i].trainIdx];
        if (keypointAngle < 0) {
            keypointAngle = 360 + keypointAngle;
        }
        
        assert(keypointAngle >= 0);
        assert(keypointAngle < 360);
        
        int diff = keypointAngle - keypoint.angle;
        if (diff < 0) {
            diff = 360 + diff;
        }
        
        assert(diff / 10 < 36);
        std::vector<cv::DMatch> &bin = hist[diff / 10];
        bin.push_back(matches[i]);
        
        std::vector<float> &angle = angles[diff / 10];
        angle.push_back(diff);
    }
    
    int max = 0;
    int maxIndex = 0;
    
    for (int i = 0; i < hist.size(); i++) {
        if (hist[i].size() > max) {
            maxIndex = i;
            max = (int)hist[i].size();
        }
    }
    
    int max2 = 0;
    int maxIndex2 = 0;
    for (int i = 0; i < hist.size(); i++) {
        if (hist[i].size() > max2 && i != maxIndex) {
            maxIndex2 = i;
            max2 = (int)hist[i].size();
        }
    }
    
    
    
    float mean = 0;
    for (int i = 0; i < angles[maxIndex].size(); i++) {
        mean += angles[maxIndex][i];
    }
    
    mean /= angles[maxIndex].size();
    
    std::vector<cv::DMatch> nearMean;
    for (int i = 0; i < 36; i++) {
        std::vector<float> &a = angles[i];
        std::vector<cv::DMatch> &m = hist[i];
        
        for (int j = 0; j < a.size(); j++) {
            float angle = a[j];
            
            float tolerance = angleDifference;
            
            float above = mean + tolerance;
            float below = mean - tolerance;
            
            if (angle < above && angle > below) {
                nearMean.push_back(m[j]);
            }
        }
        
    }
    
    return nearMean;
}

std::vector<cv::DMatch> MarkerDetector::filterHomography(std::vector<cv::DMatch> &matches, std::shared_ptr<Marker> marker, float reprojectionError, cv::Mat &homography)
{
    
    
    const int minimumPointsRequired = 10;
    if (matches.size() < minimumPointsRequired) {
        return std::vector<cv::DMatch>();
    }
    
    std::vector<cv::Point2f> src(matches.size());
    std::vector<cv::Point2f> dst(matches.size());

    // need access to actual points to find homography.
    for (int i = 0; i < matches.size(); i++) {
        src[i] = marker->keypoints[matches[i].trainIdx].pt;
        dst[i] = currentFrameKeypoints[matches[i].queryIdx].pt;
    }
    
    std::vector<unsigned char> inliersMask(src.size());
    
    
   
    // find the homography.
    
    // The new implementation of homography finding:
    const bool useNewHomography = true;
    if (useNewHomography) {
    
        bool doFinalSolution = false; // Don't do full refinement here, or indeed at all: turns out not to be necessary
        int numIterationsDone = 0;
        int maxIterations = 200;
        homography = findHomographyRANSAC(src, dst, reprojectionError, maxIterations, doFinalSolution, inliersMask, numIterationsDone);

    }
    else {
        
        // Legacy code: Use built-in OpenCV function. This might give slightly better results (more detections) in difficult situations, but is much slower.
        homography = cv::findHomography(src, dst, CV_RANSAC, reprojectionError, inliersMask, 200);

        
    }
  
    // Gather the inlier matches from the mask provided by either type of homography finding

    std::vector<cv::DMatch> inliers;
    for (int i = 0; i < inliersMask.size(); i++) {
        if (inliersMask[i]) {
            inliers.push_back(matches[i]);
        }
    }
    

    
    
    return inliers;
}


#ifdef __cplusplus
extern "C"
{
#endif
    uint64_t matchasm(unsigned char *query, unsigned char *train, unsigned trainSize);
#ifdef __cplusplus
}
#endif
#if !defined(MAC_MODE) && ( defined(__arm__) || defined(__arm64__) || defined(__aarch64__) )



class ParallelDescriptorMatcher : public cv::ParallelLoopBody
{
    
    cv::Mat markerDescriptors;
    cv::Mat currentFrameDescriptors;
    
    cv::DMatch *matches;
    
    int trainSize;
public:
    ParallelDescriptorMatcher(cv::Mat mDesc, cv::Mat cDesc, cv::DMatch *m)
    {
        markerDescriptors = mDesc;
        currentFrameDescriptors = cDesc;
        matches = m;
        
        trainSize = markerDescriptors.rows;
    }
    
    virtual void operator()(const cv::Range &range) const
    {
        for (int i = range.start; i < range.end; i++) {
            
            unsigned char *q = currentFrameDescriptors.data + i * 32;
            
            uint64_t r = matchasm(q, markerDescriptors.data, trainSize);
            uint32_t minIdx = r >> 32;
            uint32_t minVal = r & 0xffffffff;
            
            cv::DMatch &match = matches[i];
            match.queryIdx = i;
            match.trainIdx = minIdx;
            match.distance = minVal;
        }
    }
    
};
#endif


bool MarkerDetector::matchMarker(MarkerDetectorResult &result)
{
    
    // Check this has keypoints (it must have at least the absolute minimum)
    if (result.marker->keypoints.size() < 4) {
        printerr("Marker %s cannot be detected: too few keypoints!\n", result.marker->name.c_str());
        return false;
    }
    // bruteforce match keypoints.
    cv::BFMatcher matcher(cv::NORM_HAMMING, false);
    
    const int querySize = currentFrameDescriptors.rows;
    
    if (querySize == 0) {
        return false;
    }
    
    std::vector<cv::DMatch> matches(querySize);
  
    // only use the Neon matcher if Mac mode is off
#if !defined(MAC_MODE) && ( defined(__arm__) || defined(__arm64__) || defined(__aarch64__) )
    #define USE_NEON_MATCHER
#endif
    
#ifdef USE_NEON_MATCHER
    cv::parallel_for_(cv::Range(0, querySize), ParallelDescriptorMatcher(result.marker->descriptors, currentFrameDescriptors, &matches[0]));
#elif defined(__AVX__)
    // AVX Intrinsic
    int output_best_idx[querySize];
    int output_best_score[querySize];
    
    descriptorIntrinsicMatching(currentFrameDescriptors, result.marker->descriptors, output_best_idx, output_best_score);
    
    cv::DMatch intrinsicDMatch;
    for(int i = 0; i < currentFrameDescriptors.rows; i++){
        intrinsicDMatch.trainIdx = output_best_idx[i];
        intrinsicDMatch.queryIdx = i;
        intrinsicDMatch.distance = output_best_score[i];
        matches.push_back(intrinsicDMatch);
    }
#else
    matcher.match(currentFrameDescriptors, result.marker->descriptors, matches);
#endif
    
    std::vector<cv::DMatch> tmpMatches;
    
    // remove matches over a certain hamming distance.

    printlog(LOG_TRACK, "Checking %lu matches against hamming threshold of %i \n",matches.size(),hammingMatchThreshold);
    if (hammingMatchThreshold > 0) // Don't run the filter if there is no threshold
        matches = filterHamming(matches, hammingMatchThreshold); // Use member variable hammingMatchThreshold
    
    result.matchesAfterHammingFilter = (int)matches.size();
    
    
    printlog(LOG_TRACK, "Checking %lu matches against angle threshold of %f \n",matches.size(),orientationFilterAngleThreshold);
    
    if (orientationFilterAngleThreshold > 0) // Don't run the filter if there is no threshold
        // remove matches which don't fit the dominant marker-corrected orientation.
        matches = filterOrientation(matches, result.marker, orientationFilterAngleThreshold); // Use member variable orientationFilterAngleThreshold
    
    result.matchesAfterOrientationFilter = (int)matches.size();
    
    const int minimumAfterOrientation = std::max(30,minMatchesForDetection); // also use the minMatchesForDetection, because if it's > 30 but less than that, it will still fail
    if (result.matchesAfterOrientationFilter < minimumAfterOrientation) {
        return false;
    }
    
    
    // attempt to find a homography.
    cv::Mat homography;
    
    const float reprojectionError = 5;  // pixels.
    std::vector<cv::DMatch> phMatches = matches;
    
    
    matches = filterHomography(matches, result.marker, reprojectionError, homography);
    


    result.matchesAfterHomographyFilter = (int)matches.size();
    
    if (homography.empty()) {
        // couldn't find homography.
        return false;
    }
    
    // following filtering, how many correct feature matches are required to be considered a marker detection.
    
    printlog(LOG_TRACK, "Checking %lu matches against threshld of %i \n",matches.size(),minMatchesForDetection);
    if (matches.size() >= minMatchesForDetection) { // use member variable
        result.matches = matches;
        result.homography->setFullHomography(homography);
        
        // for now NEVER return true, which means you keep detecting:
        return true;
    }
    
    return false;
}





std::vector<MarkerDetectorResult> MarkerDetector::detectSubsetOfMarkers(std::vector<std::shared_ptr<Marker>> markersToDetect)
{

    Profiler profiler;
    printlog(LOG_DETECT," markersToDetect : %lu \n", markersToDetect.size());
    profiler.start();
    std::vector<MarkerDetectorResult> localResults;
    
    for (int i = 0; i < markersToDetect.size(); i++) {
        
        std::shared_ptr<Marker> marker = markersToDetect[i];

        printlog(LOG_DETECT,"Detecting marker %i %s \n", i, marker->name.c_str());
        MarkerDetectorResult result(marker);
        
        bool found = matchMarker(result);
        if (found == true) {
            localResults.push_back(result);
            
            // If you only want the first one that's detected, quit now
            
            
            if (markerChoiceMethod == CHOOSE_FIRST) {
                printlog(LOG_DETECT,"Picking only the FIRST result (in a thread) \n");
                lastDetectedMarker = marker;
                // don't search the rest of the markers if we redetect the previously detected marker since chances are this is correct.
                break;
            }
        }
    }
    profiler.end();
    float subsetTime = profiler.getTimeTaken();
    printtime("Time for one subset: %f \n", subsetTime);
    return localResults;
}



void MarkerDetector::detectSubsetOfMarkersPromised(std::vector<std::shared_ptr<Marker>> markersToDetect, std::promise<std::vector<MarkerDetectorResult>> &&promise) {

    
    std::vector<MarkerDetectorResult> localResults;
    
    std::string markerNameList;
    for (int i = 0; i < markersToDetect.size(); i++)
        markerNameList += markersToDetect[i]->name + ", ";
    
    printlog(LOG_DETECT, "Calling THREADED detection SUBSET on %lu markers: %s \n",markersToDetect.size(),markerNameList.c_str());
    
    for (int i = 0; i < markersToDetect.size(); i++) {
        std::shared_ptr<Marker> marker = markersToDetect[i];

        MarkerDetectorResult result(marker);
        
        bool found = matchMarker(result);
        if (found == true) {
            localResults.push_back(result);
            
            // If you only want the first one that's detected, quit now
            
            
            if (markerChoiceMethod == CHOOSE_FIRST) {
                printlog(LOG_DETECT, "Picking only the FIRST result (in a thread) \n");
                lastDetectedMarker = marker;
                // don't search the rest of the markers if we redetect the previously detected marker since chances are this is correct.
                break;
            }
        }
    }
    
    promise.set_value( localResults );
}


void MarkerDetector::detectMarkers(std::vector<std::shared_ptr<Marker>> markersToDetect, std::vector<std::vector<cv::Point2f>> maskRegions)

{
    printlog(LOG_DETECT,"Running detector\n");
    // clear any previous detection results.
    results.clear();
    failedDetections.clear();
    
    if (markersToDetect.size() == 0) {
        return;
    }    
    
    extractFeatures(maskRegions);

    Profiler profiler;
    profiler.start();
    
    
    if (markerChoiceMethod == CHOOSE_FIRST && markersToDetect.size() > 1) {
        // As per the original method - try to detect the last detected marker before the others (check it's in the list though!)
       
        bool lastFoundInList = false;
        for (int i = 0; i < markersToDetect.size(); i++) {
            if (markersToDetect[i] == lastDetectedMarker) {
                // erase from wherever it is in the list
                markersToDetect.erase(markersToDetect.begin() + i);
                lastFoundInList = true;
                break;
            }
        }
        // and put it at the START:
        if (lastFoundInList) {
            markersToDetect.insert(markersToDetect.begin(), lastDetectedMarker);
        }
    
        printlog(LOG_DETECT,"Want to pick only the first one, and the list begins with marker %s \n", markersToDetect[0]->name.c_str());
        
    }
    
    
    
    
    
    // no point using more threads than markers!
    int threadsToUse = std::min(numThreads, int(markersToDetect.size()));
    
    Profiler detectionTimer;
    detectionTimer.start();
    // match current frame against markers.
    if (threadsToUse > 1) {
        
        printlog(LOG_DETECT," ****** MARKER THREDING with threadsToUse = %i  \n", threadsToUse);
        
        // just for reserving space...
        int numPerThread = ceil(markersToDetect.size() / float(threadsToUse));
        
   
        
        
        // Threading done without using async and doing all the joining manually - async was causing problems
        
         std::vector<std::thread> treadList;
         std::vector<std::future<std::vector<MarkerDetectorResult>>> futureResults;
         treadList.reserve(threadsToUse);
         futureResults.reserve(threadsToUse);
        
         for (int t = 0; t < threadsToUse; t++) {
             std::vector<std::shared_ptr<Marker>> markerSubset;

             markerSubset.reserve( numPerThread );
             
             // split the list of markers equally - by starting at the index equal to the thread id and jumping by num threads
             for (int d = t; d < markersToDetect.size(); d += threadsToUse)
             markerSubset.push_back(markersToDetect[d]);
             
             std::promise< std::vector<MarkerDetectorResult> > resultPromise;
             std::future< std::vector<MarkerDetectorResult> > promiseFuture = resultPromise.get_future();
             
             std::thread detThread(&MarkerDetector::detectSubsetOfMarkersPromised,this,markerSubset,std::move(resultPromise));
             
             printlog(LOG_DETECT, "Created detection thread %i \n",t);
             treadList.push_back( std::move(detThread) );
             futureResults.push_back( std::move(promiseFuture) );
         }
         
         for (int t = 0; t < treadList.size(); t++) {
             printlog(LOG_DETECT, "joining detection thread %i \n",t);
             treadList[t].join();
             std::vector<MarkerDetectorResult> threadResults = futureResults[t].get();
             printlog(LOG_DETECT, "joined detection thread %i \n",t);
             
             std::string resultNames;
             for (int tr = 0; tr < threadResults.size(); tr++) {
                 resultNames += threadResults[tr].marker->name + ", ";
             }
             printlog(LOG_DETECT, "Got result of %lu in thread %i: %s \n",threadResults.size(),t,resultNames.c_str());
             
             // If you only want the first result, pick the first one in the first thread with any result and ignore everything else
             if (markerChoiceMethod == CHOOSE_FIRST && threadResults.size() > 0) {
                 results.push_back(threadResults[0]);
                 printlog(LOG_DETECT, "Want only the FIRST result so using the first one in thread %i, got list of %lu \n",t,results.size());
                 break;
                 // this means you are not calling join on any of the other threads!
             }
             else {
                 results.insert( results.begin(), threadResults.begin(),threadResults.end());
                 printlog(LOG_DETECT, "Got %lu results from thread future, added to main results, now: %lu \n",threadResults.size(),results.size());
             }
         }
         printlog(LOG_DETECT, "Joined everything! \n");
        
    }
    else {
        // default to single-threaded more
        printlog(LOG_DETECT, "****** Using only one detection thread!\n");
        results = detectSubsetOfMarkers(markersToDetect);

        
    }
    detectionTimer.end();
    float detectionTime = detectionTimer.getTimeTaken();
    printtime("Time for the actual detection: %f (%i threads) \n", detectionTime, threadsToUse);
    
    
    // Only need to sort or select if there is more than one result - otherwise just pass through
    // If CHOOSE_FIRST was chosen there will only be one result anyway
    if (results.size() > 1) {
        printlog(LOG_DETECT,"There are %lu detection candidate results \n", results.size());
        
        if (markerChoiceMethod == CHOOSE_BEST) {
         
            int bestResultIndex = -1;
            float bestResultScore = 0;
            for (int r = 0, R = int(results.size()); r < R; r++) {
             
                float resultScore = results[r].getScore();
                if (resultScore > bestResultScore || bestResultIndex < 0) {
                    bestResultScore = resultScore;
                    bestResultIndex = r;
                }
                
            }
            // bestResultIndex will never remain -1 so long as there's anything in the list!
            MarkerDetectorResult bestResult = results[bestResultIndex];
            results.clear();
            results.push_back(bestResult);
            lastDetectedMarker = bestResult.marker;
            
            printlog(LOG_DETECT,"Picked the best with score %f: %s \n", bestResultScore, bestResult.marker->name.c_str());
        }
        else if (markerChoiceMethod == CHOOSE_ORDERED) {
         
            std::sort(results.begin(), results.end(), [&](MarkerDetectorResult  &resultA,  MarkerDetectorResult  &resultB) {
                return (resultA.getScore() > resultB.getScore());
            });
            
            printlog(LOG_DETECT,"Have sorted results: \n");
            for (int r = 0; r < results.size(); r++) {
                printlog(LOG_DETECT," At %i result %s with score %f \n", r, results[r].marker->name.c_str(), results[r].getScore());
            }
            
            // There is guaranteed to be one in the list
            lastDetectedMarker = results[0].marker;
        }
        // for CHOOSE_ALL there is nothing to do - we already have the whole list
        
    }
    else printlog(LOG_DETECT,"Only %lu result \n", results.size());
    
    
    // if nothing was detected, then record everything as a failure. That should be enough.
    if (results.size() == 0) {
        failedDetections = markersToDetect;
    }
    
    
    
   
    profiler.end();
    
    
    if (results.size() > 0) {
        std::string allNames;
        for (int r = 0; r < results.size(); r++) {
            allNames += results[r].marker->name + ", ";
        }
        
        printlog(LOG_DETECT," ************** Detected %lu results: %s \n", results.size(), allNames.c_str());
    }


}

bool MarkerDetector::addMarker(std::shared_ptr<Marker> marker)
{
    if (marker->isMarkerValid()) {
        markers.push_back(marker);
        return true;
    }
    else {
        printerr("Marker '%s' is not valid: image may be missing or empty\n", marker->name.c_str());
        return false;
    }
}


int MarkerDetector::removeMarker(std::shared_ptr<Marker> marker)
{
    int numRemoved = 0;
    // To remove, find it in the list
    for (int m = int(markers.size())-1; m >= 0; m--) {
        if (markers[m] == marker) {
            
            markers.erase(markers.begin() + m);
            numRemoved++;
            
        }
    }
    return numRemoved;
}


NamespaceEnd
