//
//  main.cpp
//  Demo-macOS
//
//  Created on 20/04/2016 - 20/09/2016.
//  Copyright © 2016 Kudan. All rights reserved.
//


#include <iostream>
#include <fstream>

// include only the Kudan Tracker Interface:
#include "KdImageTrack.h"


// OpenCV is required for this demo:
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// ==== Utility functions ====

/** 
 Project a 3D point to 2D image given the camera pose (R, T) and calibration (K)
 This reflects the x-coordinate so that the screen coordinates align with what is used inside the tracker
 */
KudanVector2 project(KudanVector3 X, KudanMatrix3 &intrinsicMatrix, KudanMatrix3 &rotationMatrix, KudanVector3 &position, float w)
{
    
    // to project a 3D point X (3x1) according to a camera with rotation R (3x3) and translation T (3x1), and the camera intrinsic matrkx K (3x3), xh = K[R|T]X = K*(RX + T), where xh is the homogeneous point

    
    KudanVector3 RX = rotationMatrix * X;
    KudanVector3 RXplusT = RX + position; // this is the point X expressed in the camera's coordinate frame
    
    // Project using the intrinsicmatrix:
    KudanVector3 XH = intrinsicMatrix * RXplusT;
    
    // Divide the homogeneous coordinates through by the z coordinate
    KudanVector2 pt( XH.x / XH.z , XH.y / XH.z);
    
    return pt;
    
}



/** 
 Helper function for drawing a line between two KudanVector2 points, using OpenCV
 */
void drawLine(cv::Mat &img, KudanVector2 &A, KudanVector2 &B, uchar r, uchar g, uchar b, float lineWidth)
{
    cv::line(img, cv::Point2f(A.x, A.y), cv::Point2f(B.x, B.y), cv::Scalar(b, g, r), lineWidth);
}

/** 
 Draw a projected marker on an OpenCV image, given camera calibration(K), camera pose (R, T) in Kudan matrix types, and marker size
 */
KudanVector2 drawProjection(cv::Mat &img, KudanMatrix3 &K, KudanMatrix3 &R, KudanVector3 &T, float markerWidth, float markerHeight, uchar r, uchar g, uchar b, float lineWidth = 2, float boxHeight = 0, float textSize = 0)
{
    
    int w = img.size().width;
    
    KudanVector3 C00(-markerWidth/2.f,-markerHeight/2.f,0);
    KudanVector3 C01(-markerWidth/2.f,markerHeight/2.f,0);
    KudanVector3 C11(markerWidth/2.f,markerHeight/2.f,0);
    KudanVector3 C10(markerWidth/2.f,-markerHeight/2.f,0);
                               
    // Project the four corners surrounding the origin (0,0,0) which make a 2D rectangle of size markerWidth x markerHeight lying in the XY plane. This coincides with the marker corners in the image by definition
    KudanVector2 p00 = project(C00, K, R, T, w);
    KudanVector2 p01 = project(C01, K, R, T, w);
    KudanVector2 p11 = project(C11, K, R, T, w);
    KudanVector2 p10 = project(C10, K, R, T, w);
    
    
    
    
    // Anticlockwise quad:
    drawLine(img,p00,p01, r, g, b, lineWidth);
    drawLine(img,p01,p11, r, g, b, lineWidth);
    drawLine(img,p11,p10, r, g, b, lineWidth);
    drawLine(img,p10,p00, r, g, b, lineWidth);
    
    if (textSize > 0) {
     
        float lengthX = (C00-C10).norm();
        float lengthY = (C00-C01).norm();
        
        cv::Point2f midX((p00.x + p10.x) / 2.0 , (p00.y + p10.y) / 2.0);
        cv::Point2f midY((p00.x + p01.x) / 2.0 , (p00.y + p01.y) / 2.0);
        cv::putText(img, "X : " + std::to_string(lengthX), midX, 0, textSize, cv::Scalar(255-r, 255-g, 255-b), textSize);
        cv::putText(img, "Y : " + std::to_string(lengthY), midY, 0, textSize, cv::Scalar(255-r, 255-g, 255-b), textSize);

    }
    
    if (boxHeight > 0) {
        
        // Get another such rectangle raised (towards the viewer) in the Z direction
        // Note that this is done by SUBTRACTING the z value, because this is a right handed coordinate frame, so when 0,0 is the top left corner of an image, the Z axis points away from you
        KudanVector2 p00z = project(KudanVector3(-markerWidth/2.f,-markerHeight/2.f, -boxHeight), K, R, T, w);
        KudanVector2 p01z = project(KudanVector3(-markerWidth/2.f,markerHeight/2.f, -boxHeight), K, R, T, w);
        KudanVector2 p11z = project(KudanVector3(markerWidth/2.f,markerHeight/2.f, -boxHeight), K, R, T, w);
        KudanVector2 p10z = project(KudanVector3(markerWidth/2.f,-markerHeight/2.f, -boxHeight), K, R, T, w);
        
        // Anticlockwise quad:
        drawLine(img, p00z, p01z, r, g, b, lineWidth);
        drawLine(img, p01z, p11z, r, g, b, lineWidth);
        drawLine(img, p11z, p10z, r, g, b, lineWidth);
        drawLine(img, p10z, p00z, r, g, b, lineWidth);
        
        // Connect the two quads together
        drawLine(img, p00, p00z, r, g, b, lineWidth);
        drawLine(img, p01, p01z, r, g, b, lineWidth);
        drawLine(img, p11, p11z, r, g, b, lineWidth);
        drawLine(img, p10, p10z, r, g, b, lineWidth);
        
        
    }
    
    // As can be seen above, the rectangle surrounds the origin, which is the centre of the marker in its own coordinate frame (by definition). So the porjection of the origin by this camera gives the centre of the marker in the image:
    KudanVector2 objectCentre = project(KudanVector3(0,0,0), K, R, T, w);
    return objectCentre;
    
    
}

cv::Mat toCV(KudanMatrix3 &M)
{
    cv::Mat C(3,3,CV_64F);
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            C.at<double>(r,c) = M(r,c);
        }
    }
    return C;
}

/**
 Draw a projected marker on an OpenCV image, given camera calibration(K), camera pose (R, T) in Kudan matrix types, and marker size
 */
cv::Point2f drawProjectionCV(cv::Mat &img, KudanMatrix3 &KK, KudanMatrix3 &RR, KudanVector3 &TT, float markerWidth, float markerHeight, uchar r, uchar g, uchar b, float lineWidth = 2, float boxHeight = 0, float textSize = 0)
{
    cv::Mat K = toCV(KK);
    cv::Mat R = toCV(RR);
    cv::Mat T = (cv::Mat_<double>(3,1) << TT.x, TT.y, TT.z);
    
    cv::Point3f C00(-markerWidth/2.f,-markerHeight/2.f,0);
    cv::Point3f C01(-markerWidth/2.f,markerHeight/2.f,0);
    cv::Point3f C11(markerWidth/2.f,markerHeight/2.f,0);
    cv::Point3f C10(markerWidth/2.f,-markerHeight/2.f,0);
    
    std::vector<cv::Point3f> worldPoints = {C00, C01, C11, C10};
    
    if (boxHeight > 0) {
        worldPoints.reserve(9);
        worldPoints.push_back(cv::Point3f(-markerWidth/2.f,-markerHeight/2.f, -boxHeight));
        worldPoints.push_back(cv::Point3f(-markerWidth/2.f,markerHeight/2.f, -boxHeight));
        worldPoints.push_back(cv::Point3f(markerWidth/2.f,markerHeight/2.f, -boxHeight));
        worldPoints.push_back(cv::Point3f(markerWidth/2.f,-markerHeight/2.f, -boxHeight));
    }
    
    worldPoints.push_back(cv::Point3f(0, 0, 0));

    
    
    
    std::vector<cv::Point2f> imagePoints;
    cv::projectPoints(worldPoints, R, T, K, cv::Mat(), imagePoints);
    
    
    cv::Point2f p00 = imagePoints[0];
    cv::Point2f p01 = imagePoints[1];
    cv::Point2f p11 = imagePoints[2];
    cv::Point2f p10 = imagePoints[3];
    
    cv::Scalar bgr(b, g, r);
    // Anticlockwise quad:
    cv::line(img,p00,p01, bgr, lineWidth);
    cv::line(img,p01,p11, bgr, lineWidth);
    cv::line(img,p11,p10, bgr, lineWidth);
    cv::line(img,p10,p00, bgr, lineWidth);
    
    
    if (boxHeight > 0) {

    
        // Get another such rectangle raised in the Z direction
        cv::Point2f p00z = imagePoints[4];
        cv::Point2f p01z = imagePoints[5];
        cv::Point2f p11z = imagePoints[6];
        cv::Point2f p10z = imagePoints[7];
        
        // Anticlockwise quad:
        cv::line(img, p00z, p01z, bgr, lineWidth);
        cv::line(img, p01z, p11z, bgr, lineWidth);
        cv::line(img, p11z, p10z, bgr, lineWidth);
        cv::line(img, p10z, p00z, bgr, lineWidth);
        
        // Connect the two quads together
        cv::line(img, p00, p00z, bgr, lineWidth);
        cv::line(img, p01, p01z, bgr, lineWidth);
        cv::line(img, p11, p11z, bgr, lineWidth);
        cv::line(img, p10, p10z, bgr, lineWidth);
        
        
    }
    
    
    
    return imagePoints[imagePoints.size()-1];
    
    
}

/** A function to crop a specified size out of any image. This will first crop the image to the correct aspect ratio then scale to the correct size.
 */
cv::Mat getCroppedRegion(cv::Mat image, int targetWidth, int targetHeight)
{
    int imageWidth = image.size().width;
    int imageHeight = image.size().height;
    
    float targetAspect = targetWidth / float(targetHeight);
    
    float imageWidthFixedHeight = imageHeight*targetAspect;
    float imageHeightFixedWidth = imageWidth/targetAspect;
    
    cv::Mat cropped;
    if (imageWidthFixedHeight == imageWidth && imageHeight == imageHeightFixedWidth) {
        cropped = image;
    }
    else if (imageHeightFixedWidth <= imageHeight) {
        cv::Rect subRect(cv::Point(0, imageHeight/2 - imageHeightFixedWidth/2), cv::Size(imageWidth, imageHeightFixedWidth));
        cropped = image(subRect);
    }
    else if (imageWidthFixedHeight <= imageWidth) {
        cv::Rect subRect(cv::Point(imageWidth/2 - imageWidthFixedHeight/2, 0), cv::Size(imageWidthFixedHeight, imageHeight));
        cropped = image(subRect);
    }
    else {
        printf("Not possible to crop an aspect ratio of %f out of an image of %i x %i \n", targetAspect, imageWidth, imageHeight);
        assert(0);
    }
    
    
    cv::Mat cropResize;
    cv::resize(cropped, cropResize, cv::Size(targetWidth, targetHeight));
    return cropResize;

}

/** This just reads the first line from a file and returns it
 */
std::string loadKey(std::string file)
{
    std::ifstream fileStream;
    fileStream.open(file);
    if (fileStream.is_open()) {
        std::string line;
        getline(fileStream, line);
        return line;
    }
    else {
        printf("COULD NOT OPEN FILE! \n");
        return "";
    }
    
}

/**
 This function describes a simple demo program, to run on a desktop PC with a webcam as input
 This uses the Lego marker file 
 */
void interfaceDemo()
{
    

    // the KudanImgageTracker (and other classes) will throw a KudanException if something goes wrong:
    try {
        
        
        // Set up the demo:
        
        
        // If this video file is defined, it will load it as input; otherwise, load the first available webcam
        std::string videoFile;// =   e.g. "... Video/Input/Lego1.mov";
        
        // Video input done with OpenCV:
        cv::VideoCapture videoCapture;
        
        if (videoFile == "") {
            videoCapture.open(0);
        }
        else {
            videoCapture.open(videoFile);
        }
        
        bool isOpen = videoCapture.isOpened();
        if (!isOpen) {
            printf("Failed to open video capture \n");
            return;
        }
        
        // Window to display the demo:
        cv::namedWindow("Interface Demo");
        

        
        // choose an (arbitrary) image size to crop/resize the input to
        int imageDisplayWidth = 800;
        int imageDisplayHeight = 600;

        
        
    // ==== IMAGE TRACKER ====
        
        
        std::string markerFolder = "Markers/";
        
        // Set up the intrinsics, by setting the size, and using the function to guess the intrinsics (if they are known, use setIntrinsics())
        KudanCameraParameters cameraParameters;
        cameraParameters.setSize(imageDisplayWidth, imageDisplayHeight);
        cameraParameters.guessIntrinsics();
        
        // Retrieve the camera calibration matrix from the parameters, because it is needed for drawing:
        KudanMatrix3 K = cameraParameters.getMatrix();
        
        
        
        
        // Create the tracker:
        KdImageTracker tracker;
        
        // set global tracker properties:
        tracker.setMaximumSimultaneousTracking(2);
        
        // The tracker needs to know the intrinsics:
        tracker.setCameraParameters(cameraParameters);

    
    // == Lego trackable example ==
        
        /* Note: it is NOT possible to create trackables without initialising them. This will not be allowed:
        std::shared_ptr<KdImageTrackable> blankTrackable = std::make_shared<KdImageTrackable>();
        */
        
        // An example of creating a trackable and adding it to the tracker
        std::shared_ptr<KdImageTrackable> legoTrackable = KdImageTrackable::createFromImageFile(markerFolder + "lego.jpg","Lego", true);
        
        // Example of setting a property on a trackable (and demonstrates recovery mode)
        legoTrackable->forceRecoveryMode();
        
        
        // Turn virtual cropping on/off (this makes the trackable object look like a different size from the actual marker
        const bool doVirtualCropping = false;
        
        if (doVirtualCropping) {
            printf("Is lego auto cropped? %i \n", legoTrackable->isAutoCropped());

            // Virtual cropping with relative size only, doesn't matter what the units are of course
            float ww = legoTrackable->getWidth();
            float hh = legoTrackable->getHeight();
            printf("Original size of lego is %f x %f \n", ww, hh);
            bool hasVirtualCrop = legoTrackable->setVirtualCrop(ww/2, hh/2, ww/2, hh/2);
            printf("Done virtual cropping? %i New size is %f x %f \n", hasVirtualCrop, legoTrackable->getWidth(), legoTrackable->getHeight());
        }

        
        // Setting size after auto cropping still works, and this will now be the measured virtual size
        legoTrackable->setWidth(200);
        
        // Must add the trackable to the marker once it has been created
        bool addedOk = tracker.addTrackable(legoTrackable);
        printf("Added lego? %i \n",addedOk);
        
        // == Auto-cropping example ==
        
        std::shared_ptr<KdImageTrackable> cropMarker = KdImageTrackable::createFromImageFile(markerFolder + "On Fire.jpg","OnFire", /* enable auto-crop*/ true);
        

        bool addedCroppingMarker = tracker.addTrackable(cropMarker);
        printf("addedCroppingMarker = %i \n", addedCroppingMarker);
        
        cropMarker->setWidth(1000);
        
        printf("Is 'On Fire' auto cropped? %i \n", cropMarker->isAutoCropped());

        // This should NOT work if it is auto cropped - currently combining auto and virtual cropping is not enabled, because it's tricky to get right.
        bool setFireCrop = cropMarker->setVirtualCrop(400,300, 100,100);
        printf("setFireCrop? %i \n", setFireCrop);
        
        // Cropped marker not extensible for now
        cropMarker->setExtensible(false);
        bool isRunning = true;
        cv::Mat colour, grey;
        bool isProcessing = true;
        bool processOnce = false;
        
        // ==== MAIN LOOP ====
        
        int numFrames = videoCapture.get(cv::CAP_PROP_FRAME_COUNT);
        while (isRunning) {
            
            if (numFrames == 0 || videoCapture.get(cv::CAP_PROP_POS_FRAMES) < numFrames) {
                // * Read CAMERA *
                videoCapture.grab();
                videoCapture.read(colour);

                // rescale and crop to desired size (for display)
                colour = getCroppedRegion(colour, imageDisplayWidth, imageDisplayHeight);
            
            
                // Tracker requires greyscale data:
                cv::cvtColor(colour, grey, CV_BGR2GRAY);
                
            }
            if (colour.rows == 0 || grey.rows == 0) {
                printf("Camera image error \n");
                isRunning = false;
                continue;
            }
            
            // * Get image to DRAW *
            cv::Mat displayImage = colour; // don't bother copying, the colour image is not used again
            
            
            
            uchar *imageData = grey.data;
            int imageWidth = grey.size().width;
            int imageHeight = grey.size().height;
            
            cv::Point2f textPos(20,30);

            
            {
                // * TRACK *
                if (isProcessing || processOnce) {
                    tracker.processFrame(imageData, imageWidth, imageHeight, grey.channels(), 0 /*padding*/, false);
                }

                
                // Get the list of currently tracked trackables (use getTrackables() to get them all)
                std::vector<std::shared_ptr<KdImageTrackable>> detectedTrackables = tracker.getDetectedTrackables();
                
                for (int d = 0; d < detectedTrackables.size(); d++) {
                    
                    std::shared_ptr<KdImageTrackable> trackable = detectedTrackables[d];
                    
                    // Get pose of camera w.r.t. the trackable
                    KudanVector3 trackablePosition = trackable->getPosition();
                    KudanQuaternion trackableRotation = trackable->getOrientation();
                    
                    
                    
                    // Trackable's actual size is needed for drawing
                    int trackableWidth = trackable->getWidth();
                    int trackableHeight = trackable->getHeight();

                    
                    // The colour of the marker indicates whether tracking is good (usually blue, red if failing)
                    // and whether it was re-detected using recovery mode (cyan / green)
                    uchar trackR, trackG, trackB;
                    trackR = trackG = trackB = 0;
                                        
                    if (!trackable->isTrackingReliable()) {
                        // If tracking is not reliable, draw in red
                        trackR = 255;
                    }
                    else if (trackable->isRecovered() == 1) {
                        // recovery: green
                        trackG = 255;
                    }
                    else if (trackable->isRecovered() == 2) {
                        // quick recovery: cyan
                        trackB = 255;
                        trackG = 255;
                    }
                    else {
                        // normal tracking: blue
                        trackB = 255;
                    }
                    
                    // This draws a box around the marker's projected pose in 2D, using its 3D size (trackablePosition, trackableWidth) and the pose information. This works because the position T of the marker in the camera coodinate frame and marker's rotation about its centre (as returned by the tracker) are equivalent to the rotation and translation parameters needed for projection of a point X into a 2D image point x via the camera at R, T with calibration K:  x = K[R|t]X = (KR + T)X. Note the conversion of the quaternion to a rotation matrix.
                    KudanMatrix3 trackableRotationMatrix(trackableRotation);
                    KudanVector2 projectedCentre = drawProjection(displayImage, K, trackableRotationMatrix, trackablePosition, trackableWidth, trackableHeight, trackR, trackG, trackB, 2, trackableWidth/4 /* height of 3D box */, 0.8 /* text size */);
                    
                    // The above function also returns the centre of the marker projected into the image (i.e. projeciton of marker origin (0,0,0). Use that to draw the label:
                    cv::putText(displayImage, trackable->getName(), cv::Point2f(projectedCentre.x, projectedCentre.y) + cv::Point2f(10,10), 0, 0.5, cv::Scalar(trackB, trackG, trackR), 2);

                    
                    // Draw each tracked point
                    std::vector<KudanVector2> trackedPoints = trackable->getTrackedPoints();
                    for (int t = 0; t < trackedPoints.size(); t++) {
                        KudanVector2 &trackedPoint = trackedPoints[t];
                        cv::circle(displayImage, cv::Point2f(trackedPoint.x , trackedPoint.y) , 3, cv::Scalar(trackB, trackG, trackR)*0.75,1);
                    }
                    
                    
                    
                    
                    
                    std::vector<KudanVector2> corners = trackable->getTrackedCorners();
                    size_t C = corners.size();
                    if (C == 4) {
                        for (int c = 0; c < C; c++) {
                            size_t cc = (c+1)%C;
                            drawLine(displayImage, corners[c], corners[cc], 255, 0, 155, 3);
                        }
                    }

                    
                }
                
                
                
                
                
            
            
                int numTracked = tracker.getNumberOfDetectedTrackables();
                int numTrackables = tracker.getNumberOfTrackables();
            
                cv::putText(displayImage, "Tracking " + std::to_string(numTracked) + " / " + std::to_string(numTrackables), textPos, 0, 1, cv::Scalar(0,155,255), 2);
            }
            
            // Paused:
            if (!isProcessing) {
                cv::line(displayImage, cv::Point(5,10), cv::Point(5,30), cv::Scalar(0,255,100), 4);
                cv::line(displayImage, cv::Point(15,10), cv::Point(15,30), cv::Scalar(0,255,100), 4);
            }
            if (processOnce) {
                cv::line(displayImage, cv::Point(5,10), cv::Point(15,20), cv::Scalar(255,255,100), 4);
                cv::line(displayImage, cv::Point(5,30), cv::Point(15,20), cv::Scalar(255,255,100), 4);
            }
            
            processOnce = false;
            
            
            // Show image and handle UI
            cv::imshow("Interface Demo",displayImage);
            
            
            // * UI *
            int keyId = cv::waitKey(1);
            
            
            if (keyId == 27 /*escape*/) {
                isRunning = false;
            }

            // add more UI here ...
            else if (keyId == 'e') {
                bool addedOk = tracker.addTrackable(legoTrackable);
                printf("Added lego? %i \n",addedOk);
            }
            else if (keyId == 'r') {
                tracker.removeTrackable(legoTrackable);
                printf("Should have removed lego\n");
            
            }
            else if (keyId == 'x') {
                bool isExt = legoTrackable->isExtensible();
                printf("Currently: %i \n", isExt);
                legoTrackable->setExtensible(!isExt);
                printf("Lego now extensible? %i \n", legoTrackable->isExtensible());
            }
            else if (keyId == 'X') {
                legoTrackable->clearExtensions();
            }
            else if (keyId == 32) {
                isProcessing = !isProcessing;
            }
            else if (keyId == 46) {
                printf("Once! \n");
                processOnce = true;
                isProcessing = false;
            }
            // add more UI here ...
            else if (keyId != -1) {
                printf("Unkown key %i\n",keyId);
            }

        }
        
        
        // Clean up OpenCV
        cv::destroyWindow("Interface Demo");
    }
    catch (KudanException &e) {
        printf("Caught exception: %s \n", e.what());
    }
}


int main(int argc, const char * argv[])
{
    
    
    std::cout << "This is a desktop webcam demo for the KdImageTrack Interface\n";
    std::cout << " Version is " << getKdImageTrackVersion() << std::endl;
    
    interfaceDemo();
    return 0;
}
