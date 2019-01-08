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

#ifndef __MarkerTracker__Marker__
#define __MarkerTracker__Marker__

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <memory>

#include "MarkerImage.hpp"
#include "Macros.h"

Namespace(KdImageTrack)

struct FlowKeyframe
{
    cv::Mat image;
    cv::Mat smallImage;
    std::vector<cv::Mat> imagePyramid;
    cv::Mat homography;
    std::vector<cv::Point2f> points;
    std::vector<cv::Point2f> boundary;
    
};

/** This stores settings for extended markers
 */
struct ExtendedMarkerSettings
{
    
    // How old must a marker be (number of tracking frames since creation) to be allowed to have an extension. This stops stacks of extensions being created instantly, or before markers are tracked well
    static const int minExtensionLifetime = 30;
    
    // The maximum permitted deviation between the corners of a potential marker region and 90, to ensure front-on viewing
    static const int extensionCornerAngleThreshold = 30;
    
    // Thresholds on the number of descriptor and tracking points a newly created extended marker must have to be kept
    static const int minDescriptors = 30;
    static const int minTrackingPoints = 40;
    
    // Controls the reluctance to switch between extensions of a marker during tracking (higher number means it needs to be much better in order to steal tracking)
    static constexpr float switchResistance = 0.1;
    
};


enum tristate
{
    tfalse = 0,
    ttrue = 1,
    tdefault = 2
};

class Marker
{
private:

    bool processForDetection();
    bool processForTracking();
    
    // The number of keypoints (ORB) which must be present for this to be successfully created

    const int minNumberOfPoints = 4; // Changed from previous hard-coded min match number of 30 to absolute minimm of 4

    
    // Record whether this marker was created OK. If not it should not be added
    bool markerProcessedSuccessfully;
    
    // Set Optical Flow Recovery parameters here!
    // except for useFlowRecovery (the main on-off switch) which is in PlanarTracker, but there is an override:
    tristate flowRecoveryStatus = tdefault;
    
    const int flowMaxKeyframes = 30;    // After this many keyframes it starts deleting dissimilar ones
    const int flowMaxToSearch = 10;     // Search only the first ones by similarity to do recovery (set to 0 to search all)
    
    // Various settings:
    
    const int flowSmallScale = 5;                   // Small-scale images used for coarse initial search: this is the down-scale factor (from quarter-res images already)
    const float flowGaussianScale = 10;             // Blurring on small scale images
    
    const float flowErrorThreshold = 15;            // Error on optical flow track to consider it good, for attempting homography
    
    const int flowMinimumMatches = 15;              // Threshold on number of straight optical flow matches to continue
    const int flowMinimumInliers = 10;              // Threshold on number of post-homography inliers for declaring a keyframe match (still seeks the best)
    const int flowPointsForSimilarKeyframe = 30;    // This many optical flow tracks are needed to say a keyframe is too similar to an existing one

    const int maxMarkerDimension = 320;
    
public:
    cv::Mat image;
    std::string name;
    
    std::vector<cv::KeyPoint> keypoints;
    std::vector<float> keypointOrientations;
    cv::Mat descriptors;
    
    // store the size, before the marker was down-scaled
    cv::Size originalImageSize;

    std::vector<MarkerImage> trackingImages;
    
    
    std::vector<cv::Point2f> trackedBoundary, virtualBoundary, displayBoundary;
    bool wasAutoCropped;
    
    bool isFlowRecoverable(bool globalSetting)
    {
        if (flowRecoveryStatus == tfalse){
            return false;
        }
        else if (flowRecoveryStatus == ttrue) {
            return true;
        }
        else {
            return globalSetting;
        }
    }
    
    void allowFlowRecovery()
    {
        flowRecoveryStatus = tdefault;
    }
    
    void forceFlowRecovery()
    {
        flowRecoveryStatus = ttrue;
    }
    
    void prohibitFlowRecovery()
    {
        flowRecoveryStatus = tfalse;
    }
    
    Marker();
    Marker(cv::Mat input, bool doAutoCrop = false);
    
    bool process();
    void resize(int maxDimension);
    bool autoCropRequired(cv::Rect &cropRectangle);
    void autoCrop(int maxDimension, cv::Rect cropRectangle);
    
    unsigned getNumberOfFeatures()
    {
        return (unsigned)keypoints.size();
    }
    
    float getVirtualWidth();
    float getVirtualHeight();
    
    void write(FILE *fp);
    bool read(FILE *fp);

    size_t read(const unsigned char *data, size_t len);

    
    // accessor to query whether this marker was created OK. This should not be modifiable externally
    bool isMarkerValid()
    {
        return markerProcessedSuccessfully;
    }
    
    /// === Flow Recovery === 

    // The marker holds its own list of optical flow recovery frames...
    std::vector<FlowKeyframe> flowKeyframes;
    
    float flowPercentCheckSum;
    int flowPercentCheckCount;
    
    int minimumPossibleComparisons, totalFrameComparisons, totalFramesToCompare;
    
    

    bool checkForNewKeyframe(cv::Mat currentImage, std::vector<cv::Mat> &currentPyramid, cv::Mat markerHomography, std::vector<cv::Point2f> currentPoints, int &worstKeyframeId, cv::Mat &smallImage) ;
    // Creates and stores a new optical flow keyframe from the arguments
    // Returns the number of frames now
    int createFlowKeyframe(cv::Mat currentImage, std::vector<cv::Mat> currentPyramid, cv::Mat markerHomography, std::vector<cv::Point2f> currentPoints, cv::Mat smallImage) ;

    void removeFlowKeyframeIfNecessary(int kfId);
    
    
    cv::Mat localiseByFlowKeyframes(cv::Mat currentImage) ;
    
    
    // This is how we know if this marker needs to be extended or not
    bool extendedMarkersOn;
    cv::Mat extensionHomography;
    // This is a pointer to a single extension of this marker (can only have one for now)
    std::shared_ptr<Marker> extendedMarker;
    // This points to the marker from which this was created, if it is an extension
    std::shared_ptr<Marker> extensionParent;
    // This points to the original root marker (i.e. the one registered in the detector) from which this was ultimately created
    std::shared_ptr<Marker> extensionRoot;
    
    bool toBeRemoved;
    
    // Extended Markers
    
    /** Set whether this marker should be able to use extended markers or not
     */
    void setExtensible(bool ext);
    
    /** Query whether this marker is involved in extended markers at all
    */
    bool isExtensible();
    
    /** Query whether this marker has already been extended
     */
    bool isExtended();
    
    /** Query whether this marker is itself an extension of something
     */
    bool isExtension();
    
    /** Query whether this marker can be extended now, i.e. it is extensible but not already done
     */
    bool canBeExtended();
    
    void clearExtendedMarkers();
    
    /** Query whether this marker was actually auto-cropped (if it was requested, and actually done)
     */
    bool isAutoCropped();
    
    
    bool setVirtualBoundary(cv::Point corner, cv::Size size);
    bool resetVirtualBoundary();
};

NamespaceEnd

#endif /* defined(__MarkerTracker__Marker__) */
