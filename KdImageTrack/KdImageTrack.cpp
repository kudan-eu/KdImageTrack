//
//  KdImageTrack.cpp
//  KdImageTrack
//
//  Created on 14/09/2015.
//  Copyright (c) 2015 Kudan. All rights reserved.
//

#include "KdImageTrack.h"
#include "PlanarTracker.h"
#include "TrackedMarker.hpp"
#include "Marker.h"
#include "ImageProcessing.hpp"
#include "Helper.h"

#include <string>
#include "glm/glm.hpp"
#include "glm/gtc/quaternion.hpp"
#include "glm/gtc/matrix_transform.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#if defined(__APPLE__)
#include "TargetConditionals.h"
#endif

using namespace KdImageTrack;

void error(std::string msg)
{
    printerr("%s\n",msg.c_str());
}


std::string getKdImageTrackVersion()
{
    return std::to_string(int(VERSION_MAJOR)) + "." + std::to_string(int(VERSION_MINOR)) + "." + std::to_string(int(VERSION_BUILD));
}

KudanVector2::KudanVector2()
{
    zero();
}

void KudanVector2::zero()
{
    x = y = 0.f;
}

KudanVector2::KudanVector2(float xx, float yy)
: x(xx),y(yy)
{
}

std::string KudanVector2::toString()
{
    return std::to_string(x) + ", " + std::to_string(y);
}

KudanVector3::KudanVector3()
{
    zero();
}

KudanVector3::KudanVector3(float xx, float yy, float zz)
: x(xx), y(yy), z(zz)
{
}

void KudanVector3::zero()
{
    x = y = z = 0.f;
}

KudanVector3 KudanVector3::add(KudanVector3 vec)
{
    
    KudanVector3 result;
    result.x = x + vec.x;
    result.y = y + vec.y;
    result.z = z + vec.z;
    
    return result;
}

KudanVector3 KudanVector3::operator+(KudanVector3 vec)
{
    return add(vec);
}

KudanVector3 KudanVector3::subtract(KudanVector3 vec)
{
    
    KudanVector3 result;
    result.x = x - vec.x;
    result.y = y - vec.y;
    result.z = z - vec.z;
    
    return result;
}

KudanVector3 KudanVector3::operator-(KudanVector3 vec)
{
    return subtract(vec);
}

KudanVector3 KudanVector3::multiply(float scalar)
{
    
    KudanVector3 result;
    result.x = x * scalar;
    result.y = y * scalar;
    result.z = z * scalar;
    
    return result;
}

KudanVector3 KudanVector3::operator*(float scalar)
{
    return multiply(scalar);
}


float KudanVector3::norm()
{
    return sqrt(x*x + y*y + z*z);
}


std::string KudanVector3::toString()
{
    return std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z);
}

KudanQuaternion::KudanQuaternion()
{
    zero();
}

KudanQuaternion::KudanQuaternion(float xx, float yy, float zz, float ww)
: x(xx), y(yy), z(zz), w(ww)
{
}

KudanMatrix3 KudanQuaternion::quaternionToRotation(KudanQuaternion &q)
{
    return KudanMatrix3(q);
}

KudanMatrix3 KudanQuaternion::toRotation()
{
    KudanQuaternion &q = *this;
    return KudanMatrix3(q);
}

std::string KudanQuaternion::toString()
{
    return "[" + std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z) + "; " + std::to_string(w) + "]";
}


float KudanQuaternion::norm()
{
    return sqrt(x*x + y*y + z*z + w*w);
}

void KudanQuaternion::normalise()
{
    float n = norm();
    x /= n;
    y /= n;
    z /= n;
    w /= n;
}

KudanMatrix3::KudanMatrix3(KudanQuaternion &q)
{
    // Manual conversion from KudanQuaternion to rotation matrix (avoids including other libraries)
    
    float qxx = q.x * q.x;
    float qyy = q.y * q.y;
    float qzz = q.z * q.z;
    float qxz = q.x * q.z;
    float qxy = q.x * q.y;
    float qyz = q.y * q.z;
    float qwx = q.w * q.x;
    float qwy = q.w * q.y;
    float qwz = q.w * q.z;
    
    KudanMatrix3 &R = *this;
    
    R(0,0) = 1 - 2 * (qyy +  qzz);
    R(1,0) = 2 * (qxy + qwz);
    R(2,0) = 2 * (qxz - qwy);
    
    R(0,1) = 2 * (qxy - qwz);
    R(1,1) = 1 - 2 * (qxx +  qzz);
    R(2,1) = 2 * (qyz + qwx);
    
    R(0,2) = 2 * (qxz + qwy);
    R(1,2) = 2 * (qyz - qwx);
    R(2,2) = 1 - 2 * (qxx +  qyy);
}


void KudanQuaternion::zero()
{
    x = y = z = w = 0.f;
}


KudanMatrix3::KudanMatrix3()
{
    for (int i = 0; i < 9; i++) {
        data[i] = 0.f;
    }
}


float& KudanMatrix3::operator()(int r, int c)
{
    // stored as column major
    return data[c*3 + r];
}

KudanVector3 KudanMatrix3::multiply(KudanVector3 vector)
{
    // implements matrix x vector multiplication
    
    KudanVector3 result;
    KudanMatrix3 self = *this;
    result.x = self(0, 0) * vector.x + self(0, 1) * vector.y + self(0, 2) * vector.z;
    result.y = self(1, 0) * vector.x + self(1, 1) * vector.y + self(1, 2) * vector.z;
    result.z = self(2, 0) * vector.x + self(2, 1) * vector.y + self(2, 2) * vector.z;
    
    return result;
}

KudanVector3 KudanMatrix3::operator*(KudanVector3 vector)
{
    return multiply(vector);
}

std::string KudanMatrix3::toString()
{
    std::string ms = "\n";
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            ms += std::to_string((*this)(r,c));
            if (c < 2) {
                ms += ", ";
            }
        }
        ms += "\n";
    }
    ms += "\n";
    
    return ms;
}

KudanMatrix4::KudanMatrix4()
{
    for (int i = 0; i < 16; i++) {
        data[i] = 0.f;
    }
}

float& KudanMatrix4::operator()(int r, int c) {
    // stored as column major
    return data[c*4 + r];
}


std::string KudanMatrix4::toString()
{
    std::string ms = "\n";
    for (int r = 0; r < 4; r++) {
        for (int c = 0; c < 4; c++) {
            ms += std::to_string((*this)(r,c));
            if (c < 3) {
                ms += ", ";
            }
        }
        ms += "\n";
    }
    ms += "\n";
    
    return ms;
}

cv::Mat matFromData(const char unsigned *data, int width, int height, int channels, int padding, bool requireFlip, bool requiresCopy = true /* copy the data by default, because this is passed around inside the tracker and should not be allowed to be freed from outside */ )
{
    if (data == 0) {
        throw KudanException("Cannot convert null pointer to image!");
    }
    
    int matType;
    
    switch (channels) {
        case 1:
            matType = CV_8UC1;
            break;
            
        case 2:
            matType = CV_8UC2;
            break;
            
        case 3:
            matType = CV_8UC3;
            break;
            
        case 4:
            matType = CV_8UC4;
            break;
            
        default:
            throw KudanException("Wrong number of channels for image: " + std::to_string(channels));
            break;
    }
    
    cv::Mat matNoCopy(height, width, matType, (unsigned char*)data);
    
    // But, that doesn't copy the data. This is needed since the data might be released elsewhere
    cv::Mat mat;
    if (requiresCopy) {
        mat = matNoCopy.clone();
    }
    else {
        // But it might sometimes be desirable to NOT copy the data.
        mat = matNoCopy;
    }
    
    if (requireFlip) {
        cv::Mat flipped;
        cv::flip(mat, flipped, -1);
        return flipped;
    }
    else {
        return mat;
    }
}

KudanVector3 KudanUtilities::getAverageImageColour(const unsigned char *imageData, int width, int height, int channels, int padding, int targetWidth, int targetHeight)
{
    cv::Mat image = matFromData(imageData, width, height, channels, padding, false, false /* don't copy, just getting the mean */);
    
    cv::Scalar averageScalar = averageColour(image, cv::Size(targetWidth, targetHeight) );
    
    KudanVector3 averageVec;
    averageVec.x = averageScalar[0];
    averageVec.y = averageScalar[1];
    averageVec.z = averageScalar[2];
    
    return averageVec;
}

class KdImageTrackable::Private
{
public:
    
    std::shared_ptr<Marker> marker;
    
    bool isTracked;
    
    bool unreliablePose;
    int recoveryType;
    
    float realWorldScaleFactor;
    
    
    /// The number of points currently being tracked on the image
    int numTrackedPoints;
    
    /// A list of the 2D points currently being tracked
    std::vector<KudanVector2> trackedPoints;
    
    /// The four corners of the tracked region in anticlockwise order
    std::vector<KudanVector2> trackedCorners;
    
    // Store the pose of the marker at each tracked frame
    KudanVector3 position;
    KudanQuaternion orientation;
    
    
    Private()
    {
        realWorldScaleFactor = 1.0;
        
        isTracked = false;
        
        unreliablePose = 0;
        recoveryType = 0;
        
        numTrackedPoints = -1;
        
    }
    
    ~Private()
    {
    }
    
    void setName(std::string name)
    {
        marker->name = name;
    }
    
    std::string getName()
    {
        return marker->name;
    }
    
    
    bool checkAspect(float w, float h) {
        float inputAspect = w/h;
        float markerAspect = marker->getVirtualWidth() / float(marker->getVirtualHeight());
        float aspectTolerance = 0.01;
        return (fabs(inputAspect-markerAspect) < aspectTolerance);
        
    }
    
    float getWidth()
    {
        return marker->getVirtualWidth() * realWorldScaleFactor;
    }
    
    float getHeight()
    {
        return marker->getVirtualHeight() * realWorldScaleFactor;
    }
    
    
    void setWidth(float w)
    {
        // find the scale such that this is the true width
        float trackableImageWidth = marker->getVirtualWidth();
        float resalScaleFactor = w/trackableImageWidth;
        realWorldScaleFactor = resalScaleFactor;
    }
    
    void setHeight(float h)
    {
        // find the scale such that this is the true height
        float trackableImageHeight = marker->getVirtualHeight();
        float resalScaleFactor = h/trackableImageHeight;
        realWorldScaleFactor = resalScaleFactor;
    }
    
    bool initialise(std::shared_ptr<Marker> m)
    {
        
        if (m == nullptr) return false;
        if (!m->isMarkerValid()) return false;
        
        marker = m;
        
        return true;
    }
    
    static std::shared_ptr<KdImageTrackable> createFromMarker(std::shared_ptr<Marker> marker)
    {
        // Create an initialise the private internal data on the outside
        std::shared_ptr<Private> privateData = std::make_shared<Private>();
        if (privateData->initialise(marker)) {
            // If this is initialised OK, create a Trackable in the only way possible: pass the private data to a constructor
            std::shared_ptr<KdImageTrackable> trackable = std::make_shared<KdImageTrackable>(privateData);
            
            return trackable;
        }
        else {
            // If the private data was not initialised OK, then don't even create the Trackable
            return nullptr;
        }
    }
    
    
    /**
     For a given pointer to a marker, does that associate with this trackable?
     Currently this means either this trackable's marker pointer IS this marker, or it is this marker's ROOT (for extended markers)
     */
    bool isMarker(std::shared_ptr<Marker> markerPtr)
    {
        return (markerPtr == marker || markerPtr->extensionRoot == marker);
    }
};


KdImageTrackable::KdImageTrackable(std::shared_ptr<KdImageTrackable::Private> privatePtr)
{
    privateData = privatePtr;
}


std::shared_ptr<KdImageTrackable> KdImageTrackable::createFromImageFile(std::string path, std::string name, bool autoCrop)
{
    cv::Mat trackableImage = cv::imread(path);
    if (trackableImage.empty()) {
        throw KudanException("Could not read trackable" + (name==""?"":" '"+name+"'") + " (file path " + path + ")");
    }
    
    if (name == "") {
        // If no name is provided, derive on from the file name

        name = path;
        size_t pathPos = name.rfind("/");
        if (pathPos != std::string::npos) {
            name = name.substr(pathPos+1,std::string::npos);
        }
        
        size_t extPos = name.rfind(".");
        if (extPos != std::string::npos) {
            name = name.substr(0,extPos);
        }
    }
    
    // Any colour conversion is done inside the Marker constructor
    std::shared_ptr<Marker> marker = std::make_shared<Marker>(trackableImage, autoCrop);
    marker->name = name;
    
    std::shared_ptr<KdImageTrackable> trackable = KdImageTrackable::Private::createFromMarker(marker);
    
    if (trackable) {
        trackable->setWidth(trackableImage.size().width);
    }
    
    return trackable;
}

std::shared_ptr<KdImageTrackable> KdImageTrackable::createFromFilePointer(FILE* filePointer)
{
    // try to read the marker from a file pointer
    std::shared_ptr<Marker> marker = std::make_shared<Marker>();
    
    bool success = marker->read(filePointer);
    if (success == false) {
        error("Failed to read trackable from file");
        return nullptr;
    }
    
    std::shared_ptr<KdImageTrackable> trackable = KdImageTrackable::Private::createFromMarker(marker);
    
    if (trackable && marker->originalImageSize.width > 0) {
        trackable->setWidth(marker->originalImageSize.width);
    }
    
    return trackable;
}



std::shared_ptr<KdImageTrackable> KdImageTrackable::createFromImageData(const unsigned char *imageData, std::string name, int width, int height, int channels, int padding)
{
    cv::Mat image = matFromData(imageData, width, height, channels, padding, false);
    
    // Any colour conversion is done inside the Marker constructor
    std::shared_ptr<Marker> marker = std::make_shared<Marker>(image);
    marker->name = name;
    
    std::shared_ptr<KdImageTrackable> trackable = KdImageTrackable::Private::createFromMarker(marker);
    
    if (trackable) {
        trackable->setWidth(width);
    }
    
    return trackable;
}



std::string KdImageTrackable::getName()
{
    return privateData->getName();
}


void KdImageTrackable::setName(std::string n)
{
    if (privateData->marker == nullptr) {
        throw KudanException("Not initialised!");
    }
    privateData->setName(n);
}

float KdImageTrackable::getWidth()
{
    return privateData->getWidth();
}

float KdImageTrackable::getHeight()
{
    return privateData->getHeight();
}

void KdImageTrackable::setWidth(float w)
{
    privateData->setWidth(w);
}

void KdImageTrackable::setHeight(float h)
{
    privateData->setHeight(h);
}


bool KdImageTrackable::setSize(float w, float h)
{
    if (privateData->marker == nullptr) {
        throw KudanException("Not initialised!");
    }
    if (w <= 0 || h <= 0) {
        throw KudanException("Invalid size: " + std::to_string(w) + ", " + std::to_string(h));
    }
    
    if (!privateData->checkAspect(w,h)) {
        return false;
    }
    setWidth(w);
    
    return true;
}

void KdImageTrackable::setScale(float scale)
{
    privateData->realWorldScaleFactor = scale;
}

bool KdImageTrackable::resetVirtualCrop()
{
    return privateData->marker->resetVirtualBoundary();
}

bool KdImageTrackable::setVirtualCrop(float x, float y, float w, float h)
{
    return privateData->marker->setVirtualBoundary(cv::Point(x/privateData->realWorldScaleFactor, y/privateData->realWorldScaleFactor), cv::Size(w/privateData->realWorldScaleFactor, h/privateData->realWorldScaleFactor));
}

bool KdImageTrackable::isAutoCropped()
{
    return privateData->marker->isAutoCropped();
}

KudanVector3 KdImageTrackable::getPosition()
{
    // Note: scale the position by the real world scale factor. This is equivalent to scaing the points going in to the pose estimation
    return privateData->position * privateData->realWorldScaleFactor;
}

KudanQuaternion KdImageTrackable::getOrientation()
{
    return privateData->orientation;
}

bool KdImageTrackable::isTracked()
{
    return privateData->isTracked;
}

int KdImageTrackable::getNumTrackedPoints()
{
    return privateData->numTrackedPoints;
}

std::vector<KudanVector2> KdImageTrackable::getTrackedPoints()
{
    return privateData->trackedPoints;
}


std::vector<KudanVector2> KdImageTrackable::getTrackedCorners()
{
    return privateData->trackedCorners;
}

bool KdImageTrackable::isTrackingReliable()
{
    return privateData->unreliablePose==false;
}

void KdImageTrackable::allowRecoveryMode()
{
    privateData->marker->allowFlowRecovery();
}

void KdImageTrackable::forceRecoveryMode()
{
    privateData->marker->forceFlowRecovery();
}

void KdImageTrackable::prohibitRecoveryMode()
{
    privateData->marker->prohibitFlowRecovery();
}


bool KdImageTrackable::queryRecoveryMode(bool globalSetting)
{
    return privateData->marker->isFlowRecoverable(globalSetting);
}

int KdImageTrackable::isRecovered()
{
    return privateData->recoveryType;
}



void KdImageTrackable::setExtensible(bool isExtensible)
{
    privateData->marker->setExtensible(isExtensible);
}

bool KdImageTrackable::isExtensible()
{
    return privateData->marker->isExtensible();
}

bool KdImageTrackable::isExtended()
{
    return privateData->marker->isExtended();
}

void KdImageTrackable::clearExtensions()
{
    privateData->marker->clearExtendedMarkers();
    
}

KudanCameraParameters::KudanCameraParameters()
{
    w = h = 0;
    fx = fy = px = py = 0.f;
}

KudanCameraParameters::KudanCameraParameters(float focalLengthX, float focalLengthY, float principalPointX, float principalPointY, float width, float height)
{
    setSize(width,height);
    setIntrinsics(focalLengthX, focalLengthY, principalPointX, principalPointY);
}


void KudanCameraParameters::setIntrinsics(float focalLengthX, float focalLengthY, float principalPointX, float principalPointY)
{
    if (focalLengthX <= 0 || focalLengthY <= 0) {
        throw KudanException("Focal length cannot be set negative or zero!");
    }
    
    if (principalPointX <= 0 || principalPointY <= 0) {
        throw KudanException("Principal point cannot be set negative or zero!");
    }
    
    
    
    fx = focalLengthX;
    fy = focalLengthY;
    px = principalPointX;
    py = principalPointY;
    
}

// Define symbol for use on Android.
const int KudanCameraParameters::maxImageSize;

void KudanCameraParameters::setSize(int width, int height)
{
    if (width <= 0 || height <= 0) {
        throw KudanException("Image size cannot be set negative or zero!");
    }
    
    if (width > maxImageSize || height > maxImageSize) {
        throw KudanException("Image size too large (%" + std::to_string(width) + " x " + std::to_string(height) + " maximum permitted dimension is " + std::to_string(maxImageSize) + ")!");
    }
    
    w = width;
    h = height;
}


void KudanCameraParameters::guessIntrinsics()
{
    // This check needs to be done: the size is always checked when it's set, but it might not have been set
    if (w <= 0 || h <= 0) {
        throw KudanException("Bad size: " + std::to_string(w) + " x " + std::to_string(h) + ". Cannot guess intrinsics \n");
    }
    
    // then use one of these known sizes:
    if (w == 640 && h == 480) {
        printlog(LOG_DETECT, "Guessing intrinsics: detected 640x480. Using stored values \n");
        setIntrinsics(546.38889333, 546.79196157, 319.38145619, 234.44848367);
    }
    else if (w == 640 && h == 360) {
        printlog(LOG_DETECT, "Guessing intrinsics: detected 640x360. Using stored values \n");
        setIntrinsics(485.96432317873200, 485.96432317873200, 318.5892333984375, 197.3907470703125); // Microsoft settings
    }
    // Or just use the midpoint and default focal length
    else {
        printlog(LOG_DETECT, "Guessing intrinsics: Unkown resolution: %ix%i. Approximating parameters \n", w, h);
        const float defaultFocalLength = std::max(w,h);
        setIntrinsics(defaultFocalLength, defaultFocalLength, w/2.f, h/2.f);
        
    }
}

void KudanCameraParameters::getFocalLength(float &focalX, float &focalY)
{
    focalX = fx;
    focalY = fy;
}

void KudanCameraParameters::getPrincipalPoint(float &principalX, float &principalY)
{
    principalX = px;
    principalY = py;
}

void KudanCameraParameters::getSize(int &width, int &height)
{
    width = w;
    height = h;
}

bool KudanCameraParameters::isValid()
{
    if (fx <= 0 || fy <= 0 || px <= 0 || py <= 0 || w <= 0 || h <= 0) {
        return false;
    }
    else {
        return true;
    }
}

int KudanCameraParameters::width()
{
    return w;
}
int KudanCameraParameters::height()
{
    return h;
}



KudanMatrix3 KudanCameraParameters::getMatrix()
{
    KudanMatrix3 cal; // elements default to zero
    cal(0,0) = fx;                  cal(0,2) = px;
                    cal(1,1) = fy;  cal(1,2) = py;
                                    cal(2,2) = 1.0;
    return cal;
}

KudanMatrix4 KudanCameraParameters::getProjectionMatrix(float near, float far)
{
    
    KudanMatrix4 matrix;
    matrix(0,0) = 2 * fx / w;
    matrix(1,1) = 2 * fy / h;
    
    matrix(0,2) = 2 * (px / w) - 1;
    matrix(1,2) = 2 * (py / h) - 1;
    
    matrix(2,2) = -(far + near) / (far - near);
    matrix(3,2) = -1;
    matrix(2,3) = -2 * far * near / (far - near);
    
    return matrix;
}



class KdImageTracker::Private
{
public:
    
    std::shared_ptr<PlanarTracker> tracker;
    
    std::vector<std::shared_ptr<KdImageTrackable>> trackables;
    std::vector<std::shared_ptr<KdImageTrackable>> detectedTrackables;
    
    bool hasCameraParameters;
    KudanCameraParameters cameraParameters;
    
    float imageScaleFactor;
    int imageScaledWidth, imageScaledHeight;
    
    Private()
    {
        tracker  = std::make_shared<PlanarTracker>();
        hasCameraParameters = false;
        imageScaleFactor = 0.f;
        imageScaledWidth = imageScaledHeight = 0;
    }
    
    ~Private()
    {
        tracker = nullptr;
        
        trackables = std::vector<std::shared_ptr<KdImageTrackable>>();
        detectedTrackables = std::vector<std::shared_ptr<KdImageTrackable>>();
    }
};

#pragma mark - KdImageTracker
KdImageTracker::KdImageTracker()
{
    // everything important is initialised in the private data constructor
    privateData = std::make_shared<KdImageTracker::Private>();
}

KdImageTracker::~KdImageTracker()
{
    privateData = nullptr;
}


bool KdImageTracker::processFrame(const unsigned char *img, int width, int height, int channels, int padding, bool requireFlip)
{
    cv::Mat image = matFromData(img, width, height, channels, padding, requireFlip);
    
    if (image.empty()) {
        throw KudanException("Empty image");
    }
    
    if (!hasCameraCalibration()) {
        throw KudanException("No camera parameters!");
    }
    
    
    if (image.size().width != privateData->cameraParameters.width() || image.size().height != privateData->cameraParameters.height()) {
        
        throw KudanException("Input image is the wrong size: received " + std::to_string(image.size().width) + " x " + std::to_string(image.size().height) + ", size " + std::to_string(privateData->cameraParameters.width()) + " x " + std::to_string(privateData->cameraParameters.height()) + " was specified");
    }
    
    
    
    cv::Mat scaledImage;
    if (privateData->imageScaleFactor != 1) {
        cv::Size newSize(privateData->imageScaledWidth, privateData->imageScaledHeight);
        cv::resize(image, scaledImage, newSize);
    }
    else {
        scaledImage = image;
    }
    
    
    if (scaledImage.channels() == 3) {
        cv::cvtColor(scaledImage, scaledImage, CV_BGR2GRAY);
    }
    privateData->tracker->processFrame(scaledImage);
    
    // clear detection status of all trackables.
    for (auto trackable : privateData->trackables) {
        trackable->privateData->isTracked = false;
        trackable->privateData->position.zero();
        trackable->privateData->orientation.zero();
        
        trackable->privateData->numTrackedPoints = 0;
        trackable->privateData->trackedPoints.clear();
        
        trackable->privateData->trackedCorners.clear();
    }
    
    // clear detected trackable
    privateData->detectedTrackables.clear();
    
    
    
    float upScale = 1.0/privateData->imageScaleFactor;
    
    for (int i = 0; i < privateData->tracker->getNumberOfTrackedMarkers(); i++) {
        std::shared_ptr<TrackedMarker> result = privateData->tracker->getTrackedMarker(i);
        
        std::shared_ptr<KdImageTrackable> trackable;
        // Search through all stored trackables to find the one which matches this tracked marker
        for (auto t : privateData->trackables) {
            if (t->privateData->isMarker(result->marker)) {
                trackable = t;
            }
        }
        
        
        if (trackable != nullptr) {
            
            // Copy data from the TrackedMarker to the KdImageTrackable for output (converting formats as necessary)
            // This means the KdImageTrackable has a snapshot of the state at the time of tracking (don't allow uers to point directly into the TrackedMarker)
            trackable->privateData->isTracked = true;
            trackable->privateData->position.x = result->position.x;
			trackable->privateData->position.y = result->position.y;
			trackable->privateData->position.z = result->position.z;
            trackable->privateData->orientation.x = result->orientation.x;
			trackable->privateData->orientation.y = result->orientation.y;
			trackable->privateData->orientation.z = result->orientation.z;
			trackable->privateData->orientation.w = result->orientation.w;
            
            
            trackable->privateData->unreliablePose = result->unreliablePose;
            trackable->privateData->recoveryType = result->fromFlowRecovery;
            
            
            std::vector<cv::Point2f> camPt = result->patchMatcher.patchPointsInCamera.getFullPoints();
            trackable->privateData->trackedPoints.reserve(camPt.size());
            for (int p = 0; p < camPt.size(); p++) {
                // scale the points according to the scale difference between the tracked and input images
                trackable->privateData->trackedPoints.push_back( KudanVector2(camPt[p].x * upScale, camPt[p].y * upScale) );
            }
            // guarantee numTrackedPoints to be the same as the length of the list of tracked points
            trackable->privateData->numTrackedPoints = int(trackable->privateData->trackedPoints.size());
            
            if (result->homography) {
                cv::Mat H = result->homography->getFullHomography();
                // Transform the tracked boundary
                std::vector<cv::Point2f> warpedBoundary;
                cv::perspectiveTransform(result->marker->trackedBoundary, warpedBoundary, H);
                // The trackedCorners list is cleared (for all trackables) above
                for (auto &pt : warpedBoundary) {
                    trackable->privateData->trackedCorners.push_back(KudanVector2(pt.x*upScale, pt.y*upScale));
                }
            }
            
            
            // add to list of detected trackables (as well as marking as detected: the two are always in synch due to this function)
            privateData->detectedTrackables.push_back(trackable);
        }
    }
    
    return true;
}



int KdImageTracker::getNumberOfTrackables()
{
    return (int)privateData->trackables.size();
}

int KdImageTracker::getNumberOfDetectedTrackables()
{
    return (int)privateData->detectedTrackables.size();
}

std::vector<std::shared_ptr<KdImageTrackable>> KdImageTracker::getTrackables()
{
    return privateData->trackables;
}

std::vector<std::shared_ptr<KdImageTrackable>> KdImageTracker::getDetectedTrackables()
{
    return privateData->detectedTrackables;
}



bool KdImageTracker::addTrackable(std::shared_ptr<KdImageTrackable> kudanTrackable)
{
    if (kudanTrackable == nullptr) {
        return false;
    }
    std::shared_ptr<Marker> marker = kudanTrackable->privateData->marker;
    
    if (marker == nullptr) {
        return false;
    }
    
    bool addedOk = privateData->tracker->addMarker(marker);
    
    if (!addedOk) {
        return false;
    }
    
    privateData->trackables.push_back(kudanTrackable);
    
    return true;
}


bool KdImageTracker::removeTrackable(std::shared_ptr<KdImageTrackable> kudanTrackable)
{
    // Loop through all trackables until this one is found, then erase it, then return true
    // If not found, just return false.
    for (auto iterator = privateData->trackables.begin(); iterator != privateData->trackables.end(); ++iterator) {
        std::shared_ptr<KdImageTrackable> trackablePtr = *iterator;
        if (trackablePtr == kudanTrackable) {
            
            privateData->tracker->removeMarker(trackablePtr->privateData->marker);
            
            privateData->trackables.erase(iterator);
            return true;
        }
    }
    
    return false;
}

bool KdImageTracker::addTrackable(const std::string path, std::string name)
{
    
    std::shared_ptr<KdImageTrackable> trackable = KdImageTrackable::createFromImageFile(path, name);
    if (trackable == nullptr) {
        return false;
    }
    
    return addTrackable(trackable);
}

int KdImageTracker::addTrackableSet(const std::string path)
{
    FILE *fp = fopen(path.c_str(), "rb");
    
    if (fp == 0) {
        throw KudanException("Could not read trackable set");
    }
    
    int numAdded = 0;
    while (!feof(fp)) {
        std::shared_ptr<Marker> marker = std::make_shared<Marker>();
        
        bool success = marker->read(fp);
        if (success == false) {
            error("Failed to read trackable from set file");
            continue;
        }
        
        std::shared_ptr<KdImageTrackable> trackable =  KdImageTrackable::Private::createFromMarker(marker);
        
        if (trackable) {
            trackable->setWidth(trackable->privateData->marker->originalImageSize.width);
            numAdded += addTrackable(trackable);
        }
    }
    
    fclose(fp);
    return numAdded;
}

int KdImageTracker::addTrackableSet(const unsigned char *data, size_t size)
{
	size_t n = 0;

    int numAdded = 0;
	while (n < size) {
        std::shared_ptr<Marker> marker = std::make_shared<Marker>();

		size_t x = marker->read(data + n, size);
		n += x;

		if (x == 0) {
			break;
        }
        std::shared_ptr<KdImageTrackable> trackable =  KdImageTrackable::Private::createFromMarker(marker);
        
        if (trackable) {
            numAdded += addTrackable(trackable);
        }
    }
    return numAdded;
}


std::shared_ptr<KdImageTrackable> KdImageTracker::getTrackable(std::string name)
{
    for (auto t : privateData->trackables) {
        if (t->getName() == name) return t;
    }
    return nullptr;
}

void KdImageTracker::setCameraParameters(KudanCameraParameters &cp)
{
    // Check this is a valid calibration
    if (!cp.isValid()) {
        throw KudanException("Invalid camera parameters - check values are positive \n");
    }
    
    privateData->cameraParameters = cp;
    float focalX, focalY, prinX, prinY;
    int cameraWidth, cameraHeight;
    privateData->cameraParameters.getFocalLength(focalX, focalY);
    privateData->cameraParameters.getPrincipalPoint(prinX, prinY);
    privateData->cameraParameters.getSize(cameraWidth, cameraHeight);
    
    int maxWidth = PlanarTracker::maxImageWidth;
    
    if (cameraWidth > maxWidth) {
        privateData->imageScaleFactor = maxWidth/float(cameraWidth);
        int scaledHeight = cameraHeight * privateData->imageScaleFactor;
        privateData->imageScaledWidth = maxWidth;
        privateData->imageScaledHeight = scaledHeight;
    }
    else {
        privateData->imageScaleFactor = 1.f;
        privateData->imageScaledWidth = 0;
        privateData->imageScaledHeight = 0;
    }
    
    
    // Scale the intrinsic parameters by the image scale, if it has been set!
    privateData->tracker->setIntrinsics(focalX*privateData->imageScaleFactor, focalY*privateData->imageScaleFactor, prinX*privateData->imageScaleFactor, prinY*privateData->imageScaleFactor);
    privateData->hasCameraParameters = true;
}

KudanCameraParameters &KdImageTracker::getCameraParameters()
{
    return privateData->cameraParameters;
}


bool KdImageTracker::hasCameraCalibration()
{
    return privateData->hasCameraParameters;
}



/* Quadratic fitting for parameter scaling!
 Given a default parameter value and a permitted range (min,max), this fits a quadratic passing through those three values for input values of 0.5, 0 and 1 respectively
 Uses matrix inversion to solve a linear system of equations (it's the input (x) values which get squared, not the parameters, so linear is fine)
 Returns the parameters a,b,c of the quadratic ax**2 + bx + c which fits those values
 This works for min > max (i.e. reversed slope) but might go strange very a default outside the min-max range
 */
void estimateQuadratic(float minVal, float defaultVal, float maxVal, float &a, float &b, float &c)
{
    cv::Mat coeff = cv::Mat(3,3,CV_32F);
    coeff.at<float>(0,0) = 0;
    coeff.at<float>(0,1) = 0;
    coeff.at<float>(0,2) = 1.0;
    
    coeff.at<float>(1,0) = 0.5*0.5;
    coeff.at<float>(1,1) = 0.5;
    coeff.at<float>(1,2) = 1;
    
    coeff.at<float>(2,0) = 1;
    coeff.at<float>(2,1) = 1;
    coeff.at<float>(2,2) = 1;
    
    cv::Mat coeffI = coeff.inv();
    cv::Mat targetVals = (cv::Mat_<float>(3,1) << minVal, defaultVal, maxVal);
    cv::Mat param = coeffI * targetVals;
    
    
    a = param.at<float>(0,0);
    b = param.at<float>(1,0);
    c = param.at<float>(2,0);
}

/*
 For a scaling value s returns the result of the quadratic defined by as**2 + bs * c
 */
float quadratic(float scaling, float a, float b, float c)
{
    return (a*scaling*scaling + b*scaling + c);
}

/* Uses the quadratic fitting above to find the value of a parameter specified by a scaling value [0,1] and a permitted range (min-max) and default, where 0.5->default
 */
float parameterFromQuadratic(float scaling, float minVal, float defaultVal, float maxVal)
{
    float a,b,c;
    estimateQuadratic(minVal, defaultVal, maxVal, a, b, c);
    
    return quadratic(scaling,a,b,c);
}


void KdImageTracker::setDetectionSensitivity(float sensitivity)
{
    // range clamping:
    if (sensitivity < 0) sensitivity = 0;
    if (sensitivity > 1) sensitivity = 1;
    
    // Linear scales for everything at the moment, using value = min permitted value  +  sensitivty * range of permitted values
    
    
    // The maximum range possible:
    float minHamming = 0;
    float defaultHamming = MarkerDetector::defaultHammingMatchThreshold;
    float maxHamming = 255;
    // more sensitive => higher hamming threshold (i.e. more permissive, fewer matches cut)
    privateData->tracker->setHammingThreshold( parameterFromQuadratic(sensitivity,minHamming,defaultHamming,maxHamming) );
    
    
    // Arbitrary range of angles!
    float minOrientation = 2;
    float defaultOrientation = MarkerDetector::defaultOrientationFilterAngleThreshold;
    float maxOrientation = 45;
    
    // More sensitive => larger angle (more permissive in what it allows through)
    privateData->tracker->setOrientationFilterThreshold(parameterFromQuadratic(sensitivity,minOrientation,defaultOrientation,maxOrientation));
    
    int minMatchesRequired = 4; // the absolute geometric minimum
    int defaultMatchesRequired = MarkerDetector::defaultMinMatchesForDetection;
    int maxMatchesRequired = 100; // equal to the number (curently) required for tracking, roughly the highest sensible number
    
    // More sensitive => LOWER number of matches required (sensitive i.e. more permissive, so doesn't need as many matches to be sure)
    privateData->tracker->setNumMatchesRequired(parameterFromQuadratic(sensitivity,maxMatchesRequired,defaultMatchesRequired,minMatchesRequired) );
}


void KdImageTracker::resetDetectionSensitiviy()
{
    // Known defaults, should maybe be defined inside the PlanarTracker / MarkerDetector?
    privateData->tracker->setHammingThreshold( MarkerDetector::defaultHammingMatchThreshold );
    privateData->tracker->setOrientationFilterThreshold( MarkerDetector::defaultOrientationFilterAngleThreshold );
    privateData->tracker->setNumMatchesRequired( MarkerDetector::defaultMinMatchesForDetection );
}




void KdImageTracker::setMaximumSimultaneousTracking(int maxTrack)
{
    privateData->tracker->setMaximumSimultaneousTracking(maxTrack);
}



/**
 Set whether the detector is allowed to look for multiple trackables in parallel or not
 */
void KdImageTracker::toggleParallelDetection(bool doParallel)
{
    privateData->tracker->toggleParallelDetection(doParallel);
}

/**
 Query whether the detector is allowed to look for multiple trackables in parallel or not
 */
bool KdImageTracker::isDetectorParallel()
{
    return privateData->tracker->isDetectorParallel();
}


void KdImageTracker::setRecoveryMode(bool recovery)
{
    privateData->tracker->setUseFlowRecovery(recovery);
}


void KdImageTracker::prohibitRecoveryMode()
{
    privateData->tracker->prohibitFlowRecovery();
}

bool KdImageTracker::queryRecoveryMode()
{
    return privateData->tracker->isUsingFlowRecovery();
}

bool KdImageTracker::queryRecoveryMode(std::shared_ptr<KdImageTrackable> trackable)
{
    if (trackable == nullptr) {
        return false;
    }
    bool globalSetting = privateData->tracker->isUsingFlowRecovery();
    
    return trackable->queryRecoveryMode(globalSetting);
}


KudanMatrix3 KdImageTracker::getCameraMatrix()
{
    if (!hasCameraCalibration()) {
        throw KudanException("Cannot return camera calibration: no intrinsics known");
    }
    
    
    // The camera parameters object stores the intrinsic parameters, and is able to convert them to a calibration matrix
    
    KudanMatrix3 cm = privateData->cameraParameters.getMatrix();
    
    return cm;
}



KudanMatrix4 KdImageTracker::getProjectionMatrix(float nearPlane, float farPlane)
{
    if (!hasCameraCalibration()) {
        
        throw KudanException("Cannot return camera matrix: no intrinsics or image size known");
    }
    
    // The camera parameters object stores the intrinsic parameters and image size, and is able to form them into a projection matrix
    return privateData->cameraParameters.getProjectionMatrix(nearPlane, farPlane);
}



#include <string.h>
#include <stdio.h>
// This just passes the error message through to runtime_error, which will print it with what()
KudanException::KudanException(std::string msg) : std::runtime_error(msg)
{
}
