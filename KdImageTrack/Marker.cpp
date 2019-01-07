//
//  Marker.cpp
//  MarkerTracker
//
//  Created on 24/02/2015.
//  Copyright (c) 2015 Kudan. All rights reserved.
//

#include "Marker.h"
#include "Logging.hpp"
#include "Profiler.hpp"
#include <stdint.h>
#include "Logging.hpp"
#include "Helper.h"

Namespace(KdImageTrack)

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif


Marker::Marker()
{
    // The default constructor does all initialisation
    markerProcessedSuccessfully = false;
    
    // Init flow recovery things
    flowPercentCheckSum = 0.f;
    flowPercentCheckCount = 0;
    
    minimumPossibleComparisons = 0;
    totalFrameComparisons = 0;
    totalFramesToCompare = 0;

    extendedMarkersOn = false;
    wasAutoCropped = false;
    
    toBeRemoved = false;
}

Marker::Marker(cv::Mat input, bool doAutoCrop) : Marker() // make sure to call default constructor always!
{


    int conversion = -1;
    
    switch (input.type()) {
            // explicitly check for the one channel case
        case CV_8UC1:
            conversion = -1;
            break;
        case CV_8UC4:
            conversion = CV_RGBA2GRAY;
            break;
        case CV_8UC3:
            conversion = CV_RGB2GRAY;
            break;
            
        default:
            // an image with any other type is not valid (must be one, three or four channels)
            return;
    }
    
    if (conversion >= 0) {
        cv::cvtColor(input, image, conversion);
    } else {
        image = input;
    }

    // Save the original input size before changing anything:
    originalImageSize = image.size();

    // If auto cropping is on, get the cropped rectangle
    cv::Rect croppingRectangle;
    if (doAutoCrop && autoCropRequired(croppingRectangle)) {
        
        // Only run auto cropping if it is required
        autoCrop(maxMarkerDimension, croppingRectangle);
        
        // The tracked and virtual boundaries are saved within the above
    }
    else {
        
        // If not auto cropping, resize the image:
        resize(maxMarkerDimension);

        // get the boudary:
        int imageWidth = image.size().width;
        int imageHeight = image.size().height;
        trackedBoundary = { cv::Point2f(0, 0), cv::Point2f(0, imageHeight), cv::Point2f(imageWidth, imageHeight), cv::Point2f(imageWidth, 0)};
    
    
        // just the same - there is no difference between the tracked boundary and the virtual boundary if auto cropping is not being used
        virtualBoundary = trackedBoundary;
    }
    
    // In general, for a base marker, there should be no difference between the virtual (before cropping) boundary and that which is displayed
    // This comes into effect when extended markers are used
    displayBoundary = virtualBoundary;

    
    // Record whether this marker was succesfully processed for detection and tracking:
    markerProcessedSuccessfully = process();

}

float Marker::getVirtualWidth()
{
    return (virtualBoundary[2].x - virtualBoundary[0].x);
}

float Marker::getVirtualHeight()
{
    return (virtualBoundary[2].y - virtualBoundary[0].y);
}


void Marker::resize(int maxDimension)
{
    
    int originalWidth = image.size().width;
    int originalHeight = image.size().height;
    if (originalWidth <= maxDimension && originalHeight <= maxDimension) {
        // no need.
        return;
    }
    
    // we need to scale.
    float aspectRatio = originalWidth / (float)originalHeight;
    
    float scale = 0;
    if (aspectRatio > 1) {
        // landscape.
        scale = maxDimension / (float)originalWidth;
    } else {
        // portrait
        scale = maxDimension / (float)originalHeight;
    }
    
    cv::Size size(originalWidth * scale, originalHeight * scale);
    
    printlog(LOG_EXT, "resizing to %dx%d\n", size.width, size.height);
    
    cv::resize(image, image, size, 0, 0, CV_INTER_LANCZOS4);
}





cv::Mat cov2(std::vector<cv::Point2f> points, cv::Point2f &meanpt) {
    size_t P = points.size();
    if (P==0) {
        meanpt = cv::Point2f(0,0);
        return cv::Mat::zeros(2,2,CV_32F);
    }
    if (P==1) {
        meanpt = points[0];
        return cv::Mat::eye(2,2,CV_32F);
    }
    
    meanpt =  cv::Point2f(0,0);
    for (size_t p = 0; p < P; p++) {
        meanpt += points[p];
    }
    meanpt.x /= float(P);
    meanpt.y /= float(P);
    
    cv::Mat centred( int(P), 2,CV_32F);
    for (int p = 0; p < P; p++) {
        centred.at<float>(p,0) = points[p].x-meanpt.x;
        centred.at<float>(p,1) = points[p].y-meanpt.y;
    }
    
    cv::Mat cTr;
    cv::transpose(centred,cTr);
    cv::Mat prod = cTr*centred;
    return prod/float(P-1);
}

float mahalanobis(cv::Point2f x, cv::Point2f mu, cv::Mat cov) {
    
    cv::Mat invCov = cov.inv();
    cv::Mat d(2,1,CV_32F);
    d.at<float>(0,0) = x.x - mu.x;
    d.at<float>(1,0) = x.y - mu.y;
    cv::Mat dt;
    cv::transpose(d,dt);
    
    cv::Mat mul = dt * invCov * d;
    
    float dist = sqrt( mul.at<float>(0,0));
    return dist;
    
}


//#define DRAW_CROP

#ifdef DRAW_CROP
void drawCovariance(cv::Mat &img, cv::Point2f pt, cv::Mat cov, float nsig, cv::Scalar col, float lineWidth) {
    
    
    cv::Mat eigenvalues, eigenvectors;
    cv::eigen(cov, eigenvalues, eigenvectors);
    
    
    float eigenValue0 = eigenvalues.at<float>(0,0);
    float eigenValue1 = eigenvalues.at<float>(1,0);
    
    
    // assume row vectors
    float ev00 = eigenvectors.at<float>(0,0);
    float ev01 = eigenvectors.at<float>(0,1);
    cv::Point2f ev0(ev00,ev01);
    
    
    // Get angle to x axis:
    float ang = acos( ev0.x / cv::norm(ev0));
    float cr =  - ev0.y; // cross product
    if (cr > 0) ang = 2*M_PI-ang;
    
    
    if (ang < 0) ang += 2*M_PI;
    
    if ((fabs(eigenValue0) > 0.001) && (fabs(eigenValue1) > 0.001)) {
        cv::ellipse(img, pt, cv::Size(nsig*sqrt(eigenValue0),nsig*sqrt(eigenValue1)), 180.f*ang/M_PI, 0, 360, col,lineWidth);
    }
    
    cv::Point2f majorEnd( nsig*sqrt(eigenValue0)*cos(ang), nsig*sqrt(eigenValue0)*sin(ang));
    cv::line(img, pt,pt+majorEnd, col*0.5, lineWidth);
    cv::Point2f minorEnd( nsig*sqrt(eigenValue1)*cos(ang+M_PI/2.f), nsig*sqrt(eigenValue1)*sin(ang+M_PI/2.f));
    cv::line(img, pt,pt+minorEnd, col*0.5, lineWidth);
    
    
    
}
#endif

/**
 Standalone function which gets the cropping rectangle of a given image
 */
cv::Rect getCroppingRectangle(cv::Mat image)
{
    assert(image.channels() == 1);
    
    
    std::vector<cv::KeyPoint> keypoints;
    cv::FAST(image,keypoints,20);

    std::vector<cv::Point2f> points;
    points.reserve(keypoints.size());
    for (auto &k : keypoints) {
        points.push_back(k.pt);
    }
    cv::Point2f kptMean;
    cv::Mat kptCov =  cov2(points,kptMean);
    
    float autoCropSigma = 2.f;
    int minx, maxx, miny, maxy;
    minx = miny = INT_MAX;
    maxx = maxy = INT_MIN;
    
    for (auto &p : points) {
        if (mahalanobis(p, kptMean, kptCov) < autoCropSigma) {
            // Get the bounding rectangle of the valid points
            
            if (p.x < minx) {
                minx = p.x;
            }
            if (p.y < miny) {
                miny = p.y;
            }
            if (p.x > maxx) {
                maxx = p.x;
            }
            if (p.y > maxy) {
                maxy = p.y;
            }
            
        }
    }
    
    cv::Rect cropRegion = cv::Rect(cv::Point(minx, miny), cv::Size(maxx-minx, maxy-miny));
    
    printlog(LOG_EXT, "Auto crop has decided on region at %i, %i of  %i x %i \n", cropRegion.x, cropRegion.y, cropRegion.width, cropRegion.height);
    
    
#ifdef DRAW_CROP
    cv::namedWindow("Cropping");
    cv::Mat cropVis;
    cv::cvtColor(image, cropVis, CV_GRAY2BGR);
    
    for (auto &k : keypoints) {
        cv::circle(cropVis, k.pt, 4, cv::Scalar(200,0,0), 1);
    }
    
    cv::Scalar boundCol(0,0,255);
    cv::Scalar covCol(0,255,255);
    drawCovariance(cropVis, kptMean, kptCov, autoCropSigma, covCol, 2);
    cv::line(cropVis, cv::Point2f(minx,0), cv::Point2f(minx,h), boundCol);
    cv::line(cropVis, cv::Point2f(maxx,0), cv::Point2f(maxx,h), boundCol);
    cv::line(cropVis, cv::Point2f(0,miny), cv::Point2f(w,miny), boundCol);
    cv::line(cropVis, cv::Point2f(0,maxy), cv::Point2f(w,maxy), boundCol);
    
    cv::imshow("Cropping", cropVis);
#endif
    
    
    
    return cropRegion;
}

/** 
 Standalone function which determines whether this cropping rectangle is actually significant, i.e. not too close to the egdes
 */
bool isCropSignificant(cv::Mat image, cv::Rect cropRegion)
{
    int minGap = 30;
    
    float w = image.size().width;
    float h = image.size().height;
    
    int gapLeft = cropRegion.x;
    int gapRight = w - cropRegion.x - cropRegion.width;
    
    int gapTop = cropRegion.y;
    int gapBottom = h - cropRegion.y - cropRegion.height;
    
    
#ifdef DRAW_CROP
    cv::namedWindow("Crop significance");
    cv::Mat cropSignificance;
    cv::cvtColor(image, cropSignificance, CV_GRAY2BGR);
    cv::rectangle(cropSignificance, cropRegion, cv::Scalar(0,255,0),2);
    
    float cMidX = cropRegion.x + cropRegion.width / 2;
    float cMidY = cropRegion.y + cropRegion.height / 2;
    
    cv::Point2f midLeft(0, cMidY);
    cv::Point2f midRight(w-1, cMidY);
    cv::Point2f midTop(cMidX, 0);
    cv::Point2f midBottom(cMidX, h-1);
    
    cv::Point2f cropLeft(cropRegion.x, cMidY);
    cv::Point2f cropRight(cropRegion.x + cropRegion.width - 1, cMidY);
    
    cv::Point2f cropTop(cMidX, cropRegion.y);
    cv::Point2f cropBottom(cMidX, cropRegion.y + cropRegion.height);
    
    cv::Scalar borderColGood(255,255,0);
    cv::Scalar borderColSmall(0,0,200);
    
    cv::line(cropSignificance, midLeft, cropLeft, (gapLeft >= minGap) ? borderColGood : borderColSmall, 3);
    cv::line(cropSignificance, midRight, cropRight, (gapRight >= minGap) ? borderColGood : borderColSmall, 3);
    cv::line(cropSignificance, midTop, cropTop, (gapTop >= minGap) ? borderColGood : borderColSmall, 3);
    cv::line(cropSignificance, midBottom, cropBottom, (gapBottom >= minGap) ? borderColGood : borderColSmall, 3);

    
    
    cv::imshow("Crop significance", cropSignificance);
#endif
    
    // if all of the gaps are too small, then this is not good!
    if (gapLeft < minGap && gapRight < minGap && gapTop < minGap && gapBottom < minGap) {
        printlog(LOG_EXT, "All the gaps (%i, %i, %i, %i) are too small (need %i)! No need to crop \n", gapLeft, gapRight, gapTop, gapRight, minGap);
        return false;
    }
    
    // Otherwise, crop OK
    return true;
}


cv::Rect padCropRegion(cv::Mat image, cv::Rect cropRegion)
{
    int padAmount = 40;
    
    printlog(LOG_EXT, "Trying to pad the crop region by %i \n", padAmount);
    int padX = std::max(cropRegion.x - padAmount, 0);
    int padY = std::max(cropRegion.y - padAmount, 0);
    
    int padXX = std::min(cropRegion.x + cropRegion.width + padAmount, image.size().width-1);
    int padYY = std::min(cropRegion.y + cropRegion.height + padAmount, image.size().height-1);
    
    cv::Rect rect(cv::Point(padX, padY), cv::Size(padXX-padX, padYY-padY));
    return rect;
}
bool Marker::autoCropRequired(cv::Rect &cropRectangle)
{
    
    // find what the cropping rectangle would be, given the content of the image:
    cropRectangle = getCroppingRectangle(image);
    
    cropRectangle = padCropRegion(image, cropRectangle);
    
    // Just check if this rectantle is worth using, and is not too close to the edge
    bool useCrop = isCropSignificant(image, cropRectangle);

    return useCrop;

}

void Marker::autoCrop(int maxDimension, cv::Rect croppingRectangle)
{

    int originalWidth = image.size().width;
    int originalHeight = image.size().height;
    
    // goal of this: find the best max-320 region within the image, and crop
    // then the virtual size is with respect to the aspect of the input image, and the tracked size is still the region actually used
    
    
    printlog(LOG_EXT, "The cropping rectangle, of an %i x %i image, is at %i,%i with size %i x %i \n", originalWidth, originalHeight, croppingRectangle.x, croppingRectangle.y, croppingRectangle.width, croppingRectangle.height);

    
    // a cropping rectangle outside the image bounds should never be given! Since this will always be automatic, can check with assertions
    assert (croppingRectangle.x >= 0);
    assert (croppingRectangle.x + croppingRectangle.width < originalWidth);
    assert (croppingRectangle.y >= 0);
    assert (croppingRectangle.y + croppingRectangle.height < originalHeight);
    
    
    // this is the virtual region with respect to the cropped image - it is of the same size (of course) but just offset in the other direction
    cv::Rect virtualRectangle( cv::Point(-croppingRectangle.x, -croppingRectangle.y), originalImageSize);

    // crop the image by selecting a sub-region
    cv::Mat croppedInput = image(croppingRectangle);
    
    
    // maybe we need to scale now?
    float aspectRatio = croppedInput.size().width / (float)croppedInput.size().height;
    
    float scale = 1.f;
    if (aspectRatio > 1) {
        // landscape.
        scale = maxDimension / (float)croppedInput.size().width;
    } else {
        // portrait
        scale = maxDimension / (float)croppedInput.size().height;
    }
    
    // only do scaling if the image is too large (don't need it if it's already OK, and never up-scale)
    if (scale < 1.f) {
        cv::Size size(croppedInput.size().width * scale, croppedInput.size().height * scale);
    
        cv::Mat resizedInput;
        cv::resize(croppedInput, resizedInput, size, 0, 0, CV_INTER_LANCZOS4);
        
        printlog(LOG_EXT, "Resize after cropping - scale by factor of %f\n", scale);

        
        image = resizedInput;
    }
    else {
        printlog(LOG_EXT, "No need to resize after cropping\n");
        image = croppedInput;
    }
    
    
    // now save the tracked boundary in the obvious way (the bounds of the current image)
    int imageWidth = image.size().width;
    int imageHeight = image.size().height;
    trackedBoundary = { cv::Point2f(0, 0), cv::Point2f(0, imageHeight), cv::Point2f(imageWidth, imageHeight), cv::Point2f(imageWidth, 0)};
    
    
    // the crop rectangle is at the scale of the original image
    // the cropped image has now been scaled down
    // scaling the cropping rectangle obviously gives the new image size
    // so the current image scaled up gives the cropped rectangle
    // so the virtual rectangle, calculated above, after scaling down, gives the virtual boundary with respect to the scaled and cropped marker
    
    virtualBoundary =  {
        cv::Point2f(scale*virtualRectangle.x, scale*virtualRectangle.y),
        cv::Point2f(scale*(virtualRectangle.x), scale*(scale*virtualRectangle.y + virtualRectangle.height)),
        cv::Point2f(scale*(virtualRectangle.x + virtualRectangle.width), scale*(virtualRectangle.y + virtualRectangle.height)),
        cv::Point2f(scale*(virtualRectangle.x + virtualRectangle.width), scale*virtualRectangle.y)
    };
    
    // remember that this was auto-cropped
    wasAutoCropped = true;
    
#ifdef DRAW_CROP
    cv::namedWindow("Final Crop");
    cv::imshow("Final Crop", image);
#endif
    
}

#define DEG_TO_RAD(x) ((x) * M_PI / 180.0)

static cv::Mat MakeRotation(float x, float y, float z)
{
    cv::Mat X = cv::Mat::eye(4, 4, CV_32F);
    cv::Mat Y = cv::Mat::eye(4, 4, CV_32F);
    cv::Mat Z = cv::Mat::eye(4, 4, CV_32F);
    
    float sinx = sinf(x);
    float siny = sinf(y);
    float sinz = sinf(z);
    
    float cosx = cosf(x);
    float cosy = cosf(y);
    float cosz = cosf(z);
    
    X.at<float>(1, 1) = cosx;
    X.at<float>(1, 2) = -sinx;
    X.at<float>(2, 1) = sinx;
    X.at<float>(2, 2) = cosx;
    
    Y.at<float>(0, 0) = cosy;
    Y.at<float>(0, 2) = siny;
    Y.at<float>(2, 0) = -siny;
    Y.at<float>(2, 2) = cosy;
    
    Z.at<float>(0, 0) = cosz;
    Z.at<float>(0, 1) = -sinz;
    Z.at<float>(1, 0) = sinz;
    Z.at<float>(1, 1) = cosz;
    
    cv::Mat R = X * Y * Z;
    
    return R;
}

static void warpImage(const cv::Mat input, cv::Mat output, float yaw, float pitch, float roll, cv::Mat &inverseAffine)
{
    cv::Mat to_centre = cv::Mat::eye(4, 4, CV_32F);
    cv::Mat to_origin = cv::Mat::eye(4, 4, CV_32F);
    
    to_origin.at<float>(0, 3) = -input.cols / 2;
    to_origin.at<float>(1, 3) = -input.rows / 2;
    
    to_centre.at<float>(0, 3) = input.cols / 2;
    to_centre.at<float>(1, 3) = input.rows / 2;
    
    cv::Mat R = MakeRotation(pitch, yaw, roll);
    cv::Mat T = to_centre * R * to_origin;
    
    cv::Mat affine(2, 3, CV_32F);
    
    affine.at<float>(0, 0) = T.at<float>(0, 0);
    affine.at<float>(0, 1) = T.at<float>(0, 1);
    affine.at<float>(0, 2) = T.at<float>(0, 3);
    affine.at<float>(1, 0) = T.at<float>(1, 0);
    affine.at<float>(1, 1) = T.at<float>(1, 1);
    affine.at<float>(1, 2) = T.at<float>(1, 3);
    
    inverseAffine = cv::Mat::eye(3, 3, CV_32F);
    cv::Mat sub = inverseAffine(cv::Rect(0, 0, 3, 2));
    affine.copyTo(sub);
    
    inverseAffine = inverseAffine.inv();
    inverseAffine = inverseAffine(cv::Rect(0, 0, 3, 2));
    
    cv::warpAffine(input, output, affine, cv::Size(), cv::INTER_CUBIC, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
}

static cv::Point2f pointByAffine(cv::Mat M, cv::Point2f &point)
{
    cv::Mat src(3, 1, CV_32F);
    
    src.at<float>(0, 0) = point.x;
    src.at<float>(1, 0) = point.y;
    src.at<float>(2, 0) = 1.0;
    
    cv::Mat dst = M * src;
    return cv::Point2f(dst.at<float>(0, 0), dst.at<float>(1, 0));
}

bool Marker::processForDetection()
{
    const int featuresForOriginalImage = 500;
    
    cv::Ptr<cv::ORB> detector = cv::ORB::create(featuresForOriginalImage);

    // Detect and compute descriptors in one step now (new in OpenCV3)

    detector->detectAndCompute(image, cv::Mat(), keypoints, descriptors);
    
    if (keypoints.size() == 0) {
        // no features detected.
        return false;
    }
    // zero rotation for main image.
    for (int i = 0; i < keypoints.size(); i++) {
        keypointOrientations.push_back(0);
    }
   
    // maximum number of features to create for each synthesised marker image.
    const int featuresPerAffineTransform = 100;
    detector->setMaxFeatures(featuresPerAffineTransform);
    
    // viewing angles to synthesise.
    const int synthesisedViewingAngles[] = {
        50,
    };
    
    const int synthesizedZAngleStep = 45;
    
    for (int i = 0; i < 180; i += synthesizedZAngleStep) {
        for (int j = 0; j < sizeof synthesisedViewingAngles / sizeof synthesisedViewingAngles; j++) {
            float rad = DEG_TO_RAD(i);
            float angle = DEG_TO_RAD(synthesisedViewingAngles[j]);
            
            cv::Mat warpedImage(image.rows, image.cols, CV_8UC1);
            cv::Mat inverseAffine;
            
            // warp the image.
            warpImage(image, warpedImage, 0, angle, rad, inverseAffine);
            
            assert(warpedImage.cols > 0 && warpedImage.rows > 0);
            
            std::vector<cv::KeyPoint> keypointsTmp;
            cv::Mat descriptorsTmp;
            
            // detect features and compute descriptors on warped images.
            detector->detectAndCompute(warpedImage, cv::Mat(), keypointsTmp, descriptorsTmp);
            
            // don't concatenate when no keypoints
            if (keypointsTmp.size() > 0) {
                for (int i = 0; i < keypointsTmp.size(); i++) {
                    cv::Point2f &point = keypointsTmp[i].pt;
                    
                    // convert the warped point back to the original image's coordinate space.
                    cv::Point2f unwarpedPoint = pointByAffine(inverseAffine, point);
                    keypointsTmp[i].pt = unwarpedPoint;
                    
                    // add to existing keypoints.
                    keypoints.push_back(keypointsTmp[i]);
                }
                
                for (int k = 0; k < keypointsTmp.size(); k++) {
                    // for each warped keypoint, note the z-axis rotation.
                    keypointOrientations.push_back(i);
                }
                
                // add to existing descriptors.
                cv::Mat combinedDescriptors;
                cv::vconcat(descriptors, descriptorsTmp, combinedDescriptors);
                descriptors = combinedDescriptors;
            }
        }
    }
    
    if (keypoints.size() < minNumberOfPoints) {
        // After keypoint detection and warp synthesis, if there are not enough keypoints:
        return false;
    }
    
    return true;
}

bool Marker::processForTracking()
{
    // number of pyramid levels.
    const int maxLevel = 4;

    cv::Mat prevLevelImage = image;
    
    for (int level = 0; level <= maxLevel; level++) {
        cv::Mat levelImage;
        cv::Size size;
        
        // generate appropriately sized image.
        if (level == 0) {
            // special case, no resize;
            levelImage = image;
            size = cv::Size(image.cols, image.rows);
        } else {
            size = cv::Size(prevLevelImage.cols * 0.5, prevLevelImage.rows * 0.5);
            cv::resize(prevLevelImage, levelImage, size);
        }
        
        MarkerImage markerImage;
        markerImage.level = pow(2, level);
        markerImage.scale = size;
        markerImage.image = levelImage;
        
        cv::Ptr<cv::GFTTDetector> detector = cv::GFTTDetector::create(1000, 0.05, 4);
        detector->detect(levelImage, markerImage.keypoints);
        
        // sort keypoints by response (best response first).
        std::sort(markerImage.keypoints.begin(), markerImage.keypoints.end(), [](cv::KeyPoint const &a, cv::KeyPoint const &b) {
            return a.response > b.response;
        });
        
        std::vector<cv::Point2f> points;
        for (int i = 0; i < markerImage.keypoints.size(); i++) {
            markerImage.keypoints[i].pt *= markerImage.level;
            points.push_back(markerImage.keypoints[i].pt);
        }
        
        if (points.size() == 0) {
            break;
        }
        
        trackingImages.push_back(markerImage);
        prevLevelImage = levelImage;
    }
    
    // Check there are enough points at level 1.

    const int minNumberOfPointsForTracking = 10; // This number comes from (amongst other things) that fact that at least ten points are needed for optical flow
    if (trackingImages.size() < 1 || trackingImages[1].keypoints.size() < minNumberOfPointsForTracking) {
        if (trackingImages.size() < 1) {
            printerr("Marker '%s' does not have any tracking images! \n", name.c_str());
        }
        if (trackingImages[1].keypoints.size() < minNumberOfPointsForTracking) {
            printerr("Marker '%s' does not have enough points to track (has %lu, need %i) \n", name.c_str(), trackingImages[1].keypoints.size(), minNumberOfPointsForTracking);
        }

        return false;
    }
    
    return true;
}

bool Marker::process()
{
    // Both detection and tracking must be processed successful for a marker to be valid
    return (
            // generate feature descriptors for detection.
            processForDetection()
            &&
            // generate images and features used for tracking.
            processForTracking()
            );
}


void Marker::write(FILE *fp)
{
    char n[128] = { 0 };
    strncpy(n, name.c_str(), sizeof n - 1);
    
    fwrite(n, sizeof n, 1, fp);
    
    float width = originalImageSize.width;
    float height = originalImageSize.height;
    
    fwrite(&width, sizeof width, 1, fp);
    fwrite(&height, sizeof height, 1, fp);
    
    uint32_t numberOfDescriptors = (uint32_t)keypoints.size();
    fwrite(&numberOfDescriptors, sizeof numberOfDescriptors, 1, fp);
    
    for (int i = 0; i < numberOfDescriptors; i++) {
        cv::KeyPoint &kp = keypoints[i];
        
        float x = kp.pt.x;
        float y = kp.pt.y;
        float angle = kp.angle;
        
        fwrite(&x, sizeof x, 1, fp);
        fwrite(&y, sizeof y, 1, fp);
        fwrite(&angle, sizeof angle, 1, fp);
    }
    
    for (int i = 0; i < numberOfDescriptors; i++) {
        float orientation = keypointOrientations[i];
        
        fwrite(&orientation, sizeof orientation, 1, fp);
    }
    
    const int descriptorSize = 32;
    fwrite(descriptors.data, numberOfDescriptors * descriptorSize, 1, fp);
    
    std::vector<uint8_t> imageData;
    cv::imencode(".jpg", image, imageData);
    
    uint32_t imageSize = (uint32_t)imageData.size();
        
    fwrite(&imageSize, sizeof imageSize, 1, fp);
    fwrite(&imageData[0], imageSize, 1, fp);
}

bool Marker::read(FILE *fp)
{
    
    // basic check: must not be null
    if (fp == 0) {
        return false;
    }
    
    char n[128] = { 0 };
    if (fread(n, sizeof n, 1, fp) != 1) {
        return false;
    }
    
    name = n;
    
    float width, height;
    
    fread(&width, sizeof width, 1, fp);
    fread(&height, sizeof height, 1, fp);
    
    originalImageSize = cv::Size(width, height);
    
    uint32_t numberOfDescriptors;
    fread(&numberOfDescriptors, sizeof numberOfDescriptors, 1, fp);
    

    for (int i = 0; i < numberOfDescriptors; i++) {
        float x;
        float y;
        float angle;
        
        fread(&x, sizeof x, 1, fp);
        fread(&y, sizeof y, 1, fp);
        fread(&angle, sizeof angle, 1, fp);
        
        cv::KeyPoint kp;
        kp.pt.x = x;
        kp.pt.y = y;
        kp.angle = angle;
        
        keypoints.push_back(kp);
    }


    
    for (int i = 0; i < numberOfDescriptors; i++) {
        float orientation;
        
        fread(&orientation, sizeof orientation, 1, fp);
        keypointOrientations.push_back(orientation);
    }
    
    const int descriptorSize = 32;
    
    descriptors = cv::Mat(numberOfDescriptors, 32, CV_8UC1);
    fread(descriptors.data, numberOfDescriptors * descriptorSize, 1, fp);
    
    uint32_t imageSize;
    fread(&imageSize, sizeof imageSize, 1, fp);
    
    std::vector<uint8_t> imageData;
    imageData.resize(imageSize);
    
    fread(&imageData[0], imageSize, 1, fp);
    
    image = cv::imdecode(imageData, CV_LOAD_IMAGE_UNCHANGED);
    
    // Need to set the boundaries too:
    int imageWidth = image.size().width;
    int imageHeight = image.size().height;
    trackedBoundary = { cv::Point2f(0, 0), cv::Point2f(0, imageHeight), cv::Point2f(imageWidth, imageHeight), cv::Point2f(imageWidth, 0)};
    virtualBoundary = trackedBoundary;
    displayBoundary = virtualBoundary;
    
    
    if (image.rows == 0 || image.cols == 0) {
        printerr("Marker '%s' has no image (zero pixels) - it is not being loaded \n", name.c_str());
        return false;
    }
    
    // Check that there are enough keypoints, otherwise return:
    if (keypoints.size() < minNumberOfPoints) { // You need at least this many points to detect it
        markerProcessedSuccessfully = false;
        printerr("Marker '%s' has too few descriptors - it is not being loaded \n", name.c_str());

        return false;
    }
    
    markerProcessedSuccessfully = processForTracking();
    
    // Check that tracking is OK:
    if (!markerProcessedSuccessfully) {
        printerr("Marker '%s' will not be able to be tracked - it is not being loaded \n", name.c_str());
        return false;
    }
    
    return true;
}


size_t Marker::read(const unsigned char *data, size_t len)
{
    char n[128] = { 0 };
    const unsigned char *src = data;

    memcpy(n, src, sizeof n);
    src += sizeof n;
    
    name = n;

    float width, height;
    
    memcpy(&width, src, sizeof width);
    src += sizeof width;
    
    memcpy(&height, src, sizeof height);
    src += sizeof height;
    
    originalImageSize = cv::Size(width, height);
    
    uint32_t numberOfDescriptors;
    
    memcpy(&numberOfDescriptors, src, sizeof numberOfDescriptors);
    src += sizeof numberOfDescriptors;
    
    
    for (int i = 0; i < numberOfDescriptors; i++) {
        float x;
        float y;
        float angle;
    
        
        memcpy(&x, src, sizeof x);
        src += sizeof x;
        
        memcpy(&y, src, sizeof y);
        src += sizeof y;
        
        memcpy(&angle, src, sizeof angle);
        src += sizeof angle;
        
        cv::KeyPoint kp;
        kp.pt.x = x;
        kp.pt.y = y;
        kp.angle = angle;
        
        keypoints.push_back(kp);
    }
    
    for (int i = 0; i < numberOfDescriptors; i++) {
        float orientation;

        memcpy(&orientation, src, sizeof orientation);
        src += sizeof orientation;
        
        keypointOrientations.push_back(orientation);
    }
    
    const int descriptorSize = 32;
    
    descriptors = cv::Mat(numberOfDescriptors, 32, CV_8UC1);
    
    memcpy(descriptors.data, src, numberOfDescriptors * descriptorSize);
    src += numberOfDescriptors * descriptorSize;
    
    
    uint32_t imageSize;
    memcpy(&imageSize, src, sizeof imageSize);
    src += sizeof imageSize;
    
    std::vector<uint8_t> imageData;
    imageData.resize(imageSize);
    
    
    memcpy(&imageData[0], src, imageSize);
    src += imageSize;
    
    image = cv::imdecode(imageData, CV_LOAD_IMAGE_UNCHANGED);
    
    
    // Need to set the boundaries too:
    int imageWidth = image.size().width;
    int imageHeight = image.size().height;
    trackedBoundary = { cv::Point2f(0, 0), cv::Point2f(0, imageHeight), cv::Point2f(imageWidth, imageHeight), cv::Point2f(imageWidth, 0)};
    virtualBoundary = trackedBoundary;
    displayBoundary = virtualBoundary;
    
    
    markerProcessedSuccessfully = processForTracking();
    
    return src - data;
}

void Marker::removeFlowKeyframeIfNecessary(int kfId)
{
    if (flowKeyframes.size() > flowMaxKeyframes) {
        if (kfId >=0 && kfId < flowKeyframes.size()) {
            flowKeyframes.erase(flowKeyframes.begin() + kfId);
            printlog(LOG_RECOVER, " Removed keyframw number %i There are now %lu keyframes! \n", kfId, flowKeyframes.size());
        }
    }
}


bool Marker::checkForNewKeyframe(cv::Mat currentImage, std::vector<cv::Mat> &currentPyramid, cv::Mat markerHomography, std::vector<cv::Point2f> currentPoints, int &worstKeyframeId, cv::Mat &smallImage)
{

    worstKeyframeId = -1;
    
    
    // If there are no keyframes then of course you want one:
    if (flowKeyframes.size() == 0) {
        return true;
    }
    
    const int pyramidLevels = 3;
    cv::Size winSize(5, 5);
    
    
    if (currentPyramid.size() == 0) {
        printdebug("WARNING: No pyramid for keyframe \n");
        cv::buildOpticalFlowPyramid(currentImage , currentPyramid, winSize, pyramidLevels);
    }
    
    

    
    
    
    printlog(LOG_TRACK," Doing optical flow based comparion... \n");

    
    cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01);

    int opticalFlowFlags = 0;
    
    
    
    int mw = image.size().width;
    int mh = image.size().height;
    
    std::vector<cv::Point2f> mBoundary = {cv::Point2f(0, 0), cv::Point2f(0, mh), cv::Point2f(mw, mh), cv::Point2f(mw, 0)};
    std::vector<cv::Point2f> currentBoundary, smallBoundary;
    cv::perspectiveTransform(mBoundary, currentBoundary, markerHomography);
    
    Profiler sortProfilter;
    sortProfilter.start();
    std::vector<int> orderedKeyframeIds;
    
    std::vector<float> allDistances;
    std::vector<int> allIds;

    
    // ORDERED BY IMAGE SIMILARITY:
    
    

    
    cv::Mat currentImageSmall;
    cv::resize(currentImage, currentImageSmall, cv::Size(currentImage.cols / flowSmallScale, currentImage.rows / flowSmallScale));
    cv::GaussianBlur(currentImageSmall, currentImageSmall, cv::Size(3, 3), flowGaussianScale);

    smallImage = currentImageSmall;
    
    for (int k = 0; k <flowKeyframes.size(); k++) {
     
        cv::Mat kfImageSmall = flowKeyframes[k].smallImage;
        cv::Mat diffImg;
        cv::absdiff(kfImageSmall, currentImageSmall, diffImg);
      
        cv::Scalar meanVal = cv::mean(diffImg);
        uchar meanDiff = meanVal[0];
        allDistances.push_back(meanDiff);
        allIds.push_back(k);
    }

    
    std::sort( allIds.begin(), allIds.end(), [&](int a, int b) { return allDistances[a] < allDistances[b]; });
    
    orderedKeyframeIds = allIds;
    
    // save the last one in the list as being most dissimilar from the current situation
    worstKeyframeId = orderedKeyframeIds.back();
    
    sortProfilter.end();
    float timeForSetup = sortProfilter.getTimeTaken();
    
    printlog(LOG_TRACK,"Time for pre-sorting list = %f s \n", timeForSetup);
 
    for (int f = 0; f < allIds.size(); f++) {
        printlog(LOG_TRACK," Sorted BY IMAGE COMOPARISON at %i is id %i with distance %f \n", f, allIds[f], allDistances[allIds[f]]);
    }

    
    
    int numKeyframes = (int)flowKeyframes.size();
    int numKeyframesChecked = 0;
    

    
    
    bool similarExists = false;

    
    float totalTimeForCheck = 0;
    for (int i = 0; i < orderedKeyframeIds.size(); i++) {
        
        int k = orderedKeyframeIds[i];
    
        printlog(LOG_TRACK,"Checking similarity to keyframe %i of %lu in %s \n", k, flowKeyframes.size(), name.c_str());
        
        Profiler profiler;
        profiler.start();
        FlowKeyframe &kf = flowKeyframes[k];
        std::vector<cv::Point2f> currentFlowPoints;
        
        std::vector<uchar> status;
        std::vector<float> error;
        
        cv::calcOpticalFlowPyrLK(currentPyramid, kf.imagePyramid, currentPoints, currentFlowPoints, status, error, winSize, pyramidLevels, criteria, opticalFlowFlags);
        
        std::vector<cv::Point2f> currentPointsOk;

        std::vector<cv::Point2f> currentFlowPointsOk;
        std::vector<bool> currentFlowPointsInside;
        int numInside = 0;
        for (int p = 0; p < status.size(); p++) {
            if (status[p] && error[p] < flowErrorThreshold ) {

                currentFlowPointsOk.push_back(currentFlowPoints[p]);
                currentPointsOk.push_back(currentPoints[p]);
                bool inside = cv::pointPolygonTest(kf.boundary, currentFlowPoints[p], 0)>1;
                    
                currentFlowPointsInside.push_back(inside);
                numInside++;
            }
        }
        
        
        numKeyframesChecked++;
        
        bool isSimilar =  (numInside > flowPointsForSimilarKeyframe);
        
        profiler.end();

        float timeForCheck = profiler.getTimeTaken();

        totalTimeForCheck += timeForCheck;
        
#ifdef DRAW_RECOVERY
        if (drawFlowRecovery == 2) {
            cv::Mat bothImages = cv::Mat(currentImage.rows, currentImage.cols * 2, CV_8U);
            currentImage.copyTo(bothImages(cv::Rect(cv::Point(0, 0), currentImage.size())));
            cv::Point2f kOff(currentImage.cols, 0);
            kf.image.copyTo(bothImages(cv::Rect(kOff, kf.image.size())));
            
            
            cv::cvtColor(bothImages, bothImages, CV_GRAY2BGR);
            
            for (int p = 0; p < currentFlowPointsOk.size(); p++) {
                cv::circle( bothImages, kOff + currentFlowPointsOk[p], 6, currentFlowPointsInside[p]?cv::Scalar(0, 255, 1):cv::Scalar(0, 1, 200), 1);
                cv::line( bothImages, currentPointsOk[p], kOff + currentFlowPointsOk[p], currentFlowPointsInside[p]?cv::Scalar(0, 255, 1):cv::Scalar(0, 1, 200), 1);
                
            }
            
            
            for (int b = 0; b < kf.boundary.size(); b++) {
                cv::line(bothImages, kOff + kf.boundary[b], kOff + kf.boundary[(b + 1) % kf.boundary.size()], cv::Scalar(200, 0, 0), 2);
            }
        
            
            cv::putText(bothImages, "Keyframe " + std::to_string(k), cv::Point2f(10, 10), 0, 1.4, cv::Scalar(0, 255, 0), 1);
            cv::rectangle(bothImages, cv::Rect(cv::Point(0, 0), bothImages.size()), isSimilar?cv::Scalar(rand()%120, 0, 255):cv::Scalar(rand()%120, 255, 0), 4);
            
            cv::namedWindow("Comparison");
            cv::imshow("Comparison", bothImages);
            cv::waitKey(10);
        }
#endif
        
        
        if (isSimilar) {
            printlog(LOG_TRACK,"Found somthing similar at keyframe %i \n", k);
            // something similar exists now! So can return that
            similarExists = true;
            break;
        }
    
    }
    
    if (numKeyframes > 0) {

        
        
        float percentChecked = 100.f * float(numKeyframesChecked) / float(numKeyframes);
        float meanTimePerCheck = totalTimeForCheck / float(numKeyframesChecked);
        printlog(LOG_TRACK,"Similarity ? %i Needed to check %i out of %i keyrames = %f%%. Mean time per check = %f s \n", int(similarExists == true), numKeyframesChecked, numKeyframes, percentChecked, meanTimePerCheck);
        
        flowPercentCheckSum = flowPercentCheckSum + percentChecked;
        
        flowPercentCheckCount++;
        
        float flowPercentCheckMean = flowPercentCheckSum / float(flowPercentCheckCount);
        
        printlog(LOG_TRACK,"Sum = %f count = %i, That's an average of %f%% \n", flowPercentCheckSum, flowPercentCheckCount, flowPercentCheckMean);
        
        minimumPossibleComparisons++;
        totalFrameComparisons+=numKeyframesChecked;
        totalFramesToCompare += numKeyframes;
        
        printlog(LOG_TRACK,"In total, made %i comparisons (min %i) of possible %i = %f%% \n", totalFrameComparisons, minimumPossibleComparisons, totalFramesToCompare, 100.f * totalFrameComparisons / float(totalFramesToCompare));
    }
    
    
    return !similarExists;
    
}





// Creates and stores a new optical flow keyframe from the arguments
// Returns the number of frames now
int Marker::createFlowKeyframe(cv::Mat currentImage, std::vector<cv::Mat> currentPyramid, cv::Mat markerHomography, std::vector<cv::Point2f> currentPoints, cv::Mat smallImage)
{
    
    FlowKeyframe kf;
    kf.image = currentImage.clone();
    kf.imagePyramid = currentPyramid;
    
    if (currentPyramid.size() == 0) {
    
        printdebug("WARNING: No pyramid for keyframe \n");
        const int pyramidLevels = 3;
        cv::Size winSize(5, 5);
        cv::buildOpticalFlowPyramid(kf.image , kf.imagePyramid, winSize, pyramidLevels);
    }
   
    if (smallImage.rows == 0) {
        cv::resize(kf.image, kf.smallImage, cv::Size(kf.image.cols / flowSmallScale, kf.image.rows / flowSmallScale));
        cv::GaussianBlur(kf.smallImage, kf.smallImage, cv::Size(3, 3), flowGaussianScale);
        
    }
    else
        kf.smallImage = smallImage;
    
    kf.homography = markerHomography.clone();
    
    kf.points = currentPoints;
    
    int mw = image.size().width;
    int mh = image.size().height;
    std::vector<cv::Point2f> mBoundary = {cv::Point2f(0, 0), cv::Point2f(0, mh), cv::Point2f(mw, mh), cv::Point2f(mw, 0)};
    
    cv::perspectiveTransform(mBoundary, kf.boundary, kf.homography);
    
    
#ifdef DRAW_RECOVERY
    
    int kfid = flowKeyframes.size();
    
    if (drawFlowRecovery > 0) {
        
        
        std::string kfName = "Keyframe " + name;
        
        
        
        if (drawFlowRecovery == 2) {
            kfName += " " + std::to_string(kfid);
        }
        
        cv::namedWindow(kfName);
        

        cv::Mat kfImage;
        cv::cvtColor(kf.image, kfImage, CV_GRAY2BGR);

        
       
        for (int b = 0; b < kf.boundary.size(); b++) {
            cv::line(kfImage, kf.boundary[b], kf.boundary[(b + 1) % kf.boundary.size()], cv::Scalar(0, 255, 255), 2);
        }
        
        for (int p = 0; p < kf.points.size(); p++) {
            cv::circle( kfImage, kf.points[p], 5, cv::Scalar(0, 0, 200), 1);
        }

        cv::putText(kfImage, "#" + std::to_string(kfid) + " Keyframe", cv::Point2f(20, 30), 0, 0.5, cv::Scalar(0, 255, 255), 1);
        

        
        cv::imshow(kfName, kfImage);
    }

#endif
    
    
    
    flowKeyframes.push_back(kf);
    
    
    return int(flowKeyframes.size());
}



cv::Mat Marker::localiseByFlowKeyframes(cv::Mat currentImage)
{
    std::vector<cv::Mat> currentImagePyramid;
    
    int mw = image.size().width;
    int mh = image.size().height;
    std::vector<cv::Point2f> mBoundary = {cv::Point2f(0, 0), cv::Point2f(0, mh), cv::Point2f(mw, mh), cv::Point2f(mw, 0)};

    
    const int pyramidLevels = 3;
    cv::Size winSize(5, 5);
    int opticalFlowFlags = 0;
    
    cv::buildOpticalFlowPyramid(currentImage, currentImagePyramid, winSize, pyramidLevels);
    cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01);
    
    
    
    
    Profiler presortProfiler;

    presortProfiler.start();
    cv::Mat currentImageSmall;
    cv::resize(currentImage, currentImageSmall, cv::Size(currentImage.cols / flowSmallScale, currentImage.rows / flowSmallScale));
    cv::GaussianBlur(currentImageSmall, currentImageSmall, cv::Size(3, 3), flowGaussianScale);
    
    
    std::vector<float> allDistances;
    std::vector<int> allIds;
    
    for (int k = 0; k <flowKeyframes.size(); k++) {
        
        cv::Mat kfImageSmall = flowKeyframes[k].smallImage;
        cv::Mat diffImg;
        cv::absdiff(kfImageSmall, currentImageSmall, diffImg);
        
        cv::Scalar meanVal = cv::mean(diffImg);
        uchar meanDiff = meanVal[0];
        allDistances.push_back(meanDiff);
        allIds.push_back(k);
    }
    std::sort( allIds.begin(), allIds.end(), [&](int a, int b) { return allDistances[a] < allDistances[b]; });

    
    std::vector<int> orderedKeyframeIds = allIds;
    

    // only search up to some number (unless that's zero)
    int numToSearch = flowMaxToSearch>0 ? std::min(flowMaxToSearch, int(orderedKeyframeIds.size())) : int(orderedKeyframeIds.size());
    
    
    presortProfiler.end();
    float presortTime = presortProfiler.getTimeTaken();
    
    int bestKeyframeId = -1;
    int bestPositionId = -1;
    int maxNumInliers = 0;
    cv::Mat bestH;
    
    
    
    float totalRecoveryTime = 0;
    int numRecoveryAttempts = 0;


    // don't search the WHOLE list
    for (int i = 0; i < numToSearch; i++) {
        int k = orderedKeyframeIds[i];
        
        Profiler recoveryProfiler;
        recoveryProfiler.start();
        FlowKeyframe &kf = flowKeyframes[k];
        
        std::vector<cv::Point2f> flowPoints;
        std::vector<uchar> status;
        std::vector<float> error;
        
        cv::calcOpticalFlowPyrLK(kf.imagePyramid, currentImagePyramid, kf.points, flowPoints, status, error, winSize, pyramidLevels, criteria, opticalFlowFlags);
        
        float errorThreshold = 15;
        
        std::vector<cv::Point2f> goodKfPoints;
        std::vector<cv::Point2f> goodFlowPoints;
        for (int p = 0; p < kf.points.size(); p++) {
            if (status[p] && error[p] < errorThreshold ) {
                goodKfPoints.push_back(kf.points[p]);
                goodFlowPoints.push_back(flowPoints[p]);
            }
        }
        std::vector<unsigned char> inliersMask;
        
        std::vector<cv::Point2f> goodKfProjections;
        cv::Mat homog, composedH;
         int numInliers = 0;
        
        if (goodKfPoints.size() > flowMinimumMatches) {
            homog = cv::findHomography(goodKfPoints, goodFlowPoints, CV_RANSAC, 1, inliersMask);
            
            cv::perspectiveTransform(goodKfPoints, goodKfProjections, homog);
            
            for (int i = 0; i < inliersMask.size(); i++) {
                numInliers += inliersMask[i]>0;
            }
            
            
            printlog(LOG_TRACK," At keyframe %i, Got proejctions: %lu and %i inliers \n", k, goodKfProjections.size(), numInliers);
            
            composedH = homog * kf.homography;
            
            if (numInliers > flowMinimumInliers && numInliers > maxNumInliers) {
                maxNumInliers = numInliers;
                bestKeyframeId = k;
                bestPositionId = i;
                bestH = composedH;
                
            }

            
        }
       
        
        recoveryProfiler.end();
        totalRecoveryTime += recoveryProfiler.getTimeTaken();
        numRecoveryAttempts ++;

#ifdef DRAW_RECOVERY
        
        
        if (drawFlowRecovery == 2) {
            std::string kfName = "Keyframe Search";
            cv::Mat kfImage = cv::Mat(currentImage.rows * 2, currentImage.cols, CV_8U);
            cv::Point2f cOff(0, kf.image.rows);
            kf.image.copyTo(kfImage(cv::Rect(cv::Point(0, 0), kf.image.size())));
            currentImage.copyTo(kfImage(cv::Rect(cOff, currentImage.size())));
            
            cv::cvtColor(kfImage, kfImage, CV_GRAY2BGR);
            
            std::vector<cv::Point2f> hBoundary;
            cv::perspectiveTransform(mBoundary, hBoundary, kf.homography);
            
            for (int b = 0; b < hBoundary.size(); b++) {
                cv::line(kfImage, hBoundary[b], hBoundary[(b + 1) % hBoundary.size()], cv::Scalar(0, 255, 255), 2);
            }

            
            
            for (int p = 0; p < kf.points.size(); p++) {
                
                cv::circle( kfImage, kf.points[p], 5, cv::Scalar(0, 200, 200), 1);

                cv::Scalar flowCol = (status[p] && error[p] < errorThreshold )? cv::Scalar(0, 255, 0):cv::Scalar(0, 0, 200);
                cv::line( kfImage, kf.points[p], cOff + flowPoints[p], flowCol, 1);
            }
            
            for (int i = 0; i < inliersMask.size(); i++) {
                if (inliersMask[i]) {
                    cv::line( kfImage, goodKfPoints[i], cOff + goodFlowPoints[i], cv::Scalar(255, 0, 0), 1);
                }
            }
            for (int p = 0; p < goodKfProjections.size(); p++) {
                cv::Scalar projCol =  (inliersMask[p])?cv::Scalar(255, 155, 1):cv::Scalar(200, 0, 200);
                cv::circle( kfImage, cOff + goodKfProjections[p], 6, projCol, 1);
            }
            
            if (!homog.empty()) {

                std::vector<cv::Point2f> fBoundary;
                
                cv::perspectiveTransform(mBoundary, fBoundary, composedH);
                for (int b = 0; b < fBoundary.size(); b++) {
                    cv::line(kfImage, cOff + fBoundary[b], cOff + fBoundary[(b + 1) % fBoundary.size()], cv::Scalar(0, 100, 205), 2);
                }
            }
            
            
            cv::putText(kfImage, "Kf " +std::to_string(k) + ": " + std::to_string(numInliers) + " inliers", cv::Point2f(10, kfImage.rows - 20), 0, 0.4, cv::Scalar(0, 0, 255), 1.5);
            cv::imshow(kfName, kfImage);

        }
#endif
        
    }
    
    float meanRecoveryTime = totalRecoveryTime / float(numRecoveryAttempts);
    

    
    
    if (bestKeyframeId != -1) {
        
        
        printlog(LOG_TRACK,"Returning from flow recovery with best at id %i (position %i of %i 'to search'). Did %i attempts with mean time of %f s. presortTime = %f s \n", bestKeyframeId, bestPositionId, numToSearch, numRecoveryAttempts, meanRecoveryTime, presortTime);
  
#ifdef DRAW_RECOVERY
        
        if (bestPositionId >= numToSearch) {
            printlog(LOG_TRACK," WOULD HAVE MISSED THIS ONE! \n");
            
        }
        if (drawFlowRecovery > 0) {
            FlowKeyframe &bestKf = flowKeyframes[bestKeyframeId];
            std::string kfName = "Matched keyframe for " + name;;
            cv::Mat bestKfImage = cv::Mat(currentImage.rows * 2, currentImage.cols, CV_8U);
            cv::Point2f cOff(0, bestKf.image.rows);
            bestKf.image.copyTo(bestKfImage(cv::Rect(cv::Point(0, 0), bestKf.image.size())));
            currentImage.copyTo(bestKfImage(cv::Rect(cOff, currentImage.size())));

            cv::cvtColor(bestKfImage, bestKfImage, CV_GRAY2BGR);
            
            
            std::vector<cv::Point2f> hBoundary;
            cv::perspectiveTransform(mBoundary, hBoundary, bestKf.homography);
            
            std::vector<cv::Point2f> fBoundary;
            cv::perspectiveTransform(mBoundary, fBoundary, bestH);
            
            for (int b = 0; b < hBoundary.size(); b++) {
                cv::line(bestKfImage, hBoundary[b], hBoundary[(b + 1) % hBoundary.size()], cv::Scalar(0, 255, 255), 2);

                cv::line(bestKfImage, cOff + fBoundary[b], cOff + fBoundary[(b + 1) % fBoundary.size()], cv::Scalar(0, 100, 205), 2);
                
                
                cv::line(bestKfImage, hBoundary[b], cOff + fBoundary[b],  cv::Scalar(0, 200, 0), 1);
                
            }

            cv::putText(bestKfImage,"Matched " + std::to_string(bestKeyframeId), cv::Point2f(5, 20), 0, 0.5, cv::Scalar(0, 255, 0), 1.5);
            cv::namedWindow(kfName);
            
            cv::imshow(kfName, bestKfImage);
        }
#endif
    }
    return bestH;
    
}


cv::Mat makeRelativeHomography(float dx, float dy, float scale, float width, float height) {
    
    
    cv::Mat T1 =cv::Mat::eye(3,3,CV_64F);
    
    // move so that the centres will be in the same place after scaling
    T1.at<double>(0,2) = (width/2.f)*(1.f-scale);
    T1.at<double>(1,2) = (height/2.f)*(1.f-scale);
    
    // do the scaling
    cv::Mat S =cv::Mat::eye(3,3,CV_64F);
    S.at<double>(2,2) = 1.f/scale;
    
    // shift after scaling by the amount you wanted to:
    cv::Mat T2 =cv::Mat::eye(3,3,CV_64F);
    T2.at<double>(0,2) = dx*width/scale;
    T2.at<double>(1,2) = dy*height/scale;
    
    // compose:
    cv::Mat H = T1*S*T2;
    
    return H;
}

bool Marker::resetVirtualBoundary()
{
    // Do not allow the virtual boundary to be changed if this is involved with extended markers
    if (extensionRoot != nullptr || extendedMarker != nullptr || extensionParent != nullptr) {
        return false;
    }
    // Cannot virtually crop an already auto-cropped marker (could be possible, currently disabled for convenience)
    if (wasAutoCropped) {
        return false;
    }
    
    virtualBoundary = trackedBoundary;
    displayBoundary = virtualBoundary;
    
    return true;
}

bool Marker::setVirtualBoundary(cv::Point corner, cv::Size size)
{
    // Do not allow the virtual boundary to be changed if this is involved with extended markers
    if (extensionRoot != nullptr || extendedMarker != nullptr || extensionParent != nullptr) {
        return false;
    }
    // Cannot virtually crop an already auto-cropped marker (could be possible, currently disabled for convenience)
    if (wasAutoCropped) {
        return false;
    }
    
    assert(trackedBoundary.size() == 4);
    assert(virtualBoundary.size() == 4);
    

    // simply set the boundary virtual boundary to the given location and size!
    
    // remember: the boundary is layed out top-left anti-clockwise:
    // 0 - 3
    // |   |
    // 1 - 2
    
    virtualBoundary[0].x = virtualBoundary[1].x = corner.x;
    virtualBoundary[0].y = virtualBoundary[3].y = corner.y;
    
    virtualBoundary[3].x = virtualBoundary[2].x = corner.x + size.width;
    virtualBoundary[1].y = virtualBoundary[2].y = corner.y + size.height;
    
    // Set the display boundary to the same (assume it's not an extended marker!)
    displayBoundary = virtualBoundary;
    
    return true;
}

void Marker::setExtensible(bool ext)
{
    extendedMarkersOn = ext;
    
    if (extendedMarkersOn) {
        // use the size of the image
        extensionHomography = makeRelativeHomography(0, 0, 2, image.size().width, image.size().height);
    }
}

bool Marker::isExtensible()
{
    return extendedMarkersOn;
}

bool Marker::isExtended()
{
    return (extendedMarker != nullptr);
}

bool Marker::isExtension()
{
    return (extensionParent != nullptr);
}

bool Marker::canBeExtended()
{
    return (extendedMarkersOn && extendedMarker == nullptr);
}

void Marker::clearExtendedMarkers()
{
    // recursively delete them!
    if (extendedMarker) {
        extendedMarker->clearExtendedMarkers();
        
        // make sure there is no link to the parent/root or children, to avoid double linkage
        extendedMarker->extensionParent = nullptr;
        extendedMarker->extensionRoot = nullptr;
        extendedMarker->extendedMarker = nullptr;
        extendedMarker->toBeRemoved = true;
        
        // should delete it because nothing will refer to it:
        extendedMarker = nullptr;
    }
    
    
}

bool Marker::isAutoCropped()
{
    return wasAutoCropped;
}

NamespaceEnd

