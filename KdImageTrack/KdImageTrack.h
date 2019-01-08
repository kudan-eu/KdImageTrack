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

#ifndef __KdImageTrack__Interface__
#define __KdImageTrack__Interface__

#include <stdio.h>
#include <memory>
#include <vector>

#include <string>

// Version:
#define VERSION_MAJOR 2
#define VERSION_MINOR 2
#define VERSION_BUILD 0

/**
 Basic error output function. It takes a sring and passes it through to printerr() in Logging
 Does not allow format strings.
 */
void error(std::string msg);
std::string getKdImageTrackVersion();

/**
 Simple representation of a 2D point.
 */
class KudanVector2
{
public:
    /** Public x, y coefficients of the vector */
    float x;
    float y;
    
    /**
     Default constructor initialises the data to the zero vector (0,0,0)'
     */
    KudanVector2();
    
    /**
     Constructor for initialising the data to the given vector
     @param x,y Coefficients of the vector
     */
    KudanVector2(float x, float y);
    
    /**
     Set the all coefficients to zero
     */
    void zero();
    
    /**
     Returns a representation of this as a string (x,y)
     @return A string representation of this vector
     */
    std::string toString();
};


/** 
 Simple representation of a 3D point or vector.
 */
class KudanVector3
{
public:
    /** Public x, y, z coefficients of the vector */
	float x;
	float y;
	float z;
    
    /**
     Default constructor initialises the data to the zero vector (0,0,0)'
     */
    KudanVector3();
    
    /**
     Constructor for initialising the data to the given vector
     @param x,y,z Coefficients of the vector
     */
    KudanVector3(float x, float y, float z);
    
    /** 
     Set the all coefficients to zero
     */
    void zero();
    
    /**
     Add this to another 3x1 vector
     @param vec The other vector
     @return A new 3x1 vector this+vec
     */
    KudanVector3 add(KudanVector3 vec);
    
    /** Overloaded addition operator - calls the add function
     @param vec The vector to add to this
     @return The result of this+vec
     */
    KudanVector3 operator+(KudanVector3 vec);
    
    /** Subtract another vector from this
     @param vec The vector to subtract from this
     @return The result of this-vec
     */
    KudanVector3 subtract(KudanVector3 vec);
    
    /** Overloaded subtaction operator - calls the subtract function
     @param vec The vector to subtract from this
     @return The result of this-vec
     */
    KudanVector3 operator-(KudanVector3 vec);
    
    /** Mutliply this vector X by a scalar s = Xs
     @param scalar The value with which to multiply the elements of this vector
     @return A new vector representing the vector multiplied by the scalar
     */
    KudanVector3 multiply(float scalar);
    
    /** Overloaded multiplication operator - calls the multiply function
     @param scalar The value with which to multiply the elements of this vector
     @return A new vector representing the vector multiplied by the scalar
     */
    KudanVector3 operator*(float scalar);

    
    /** Return the L2 (Euclidean) norm of this vector
     @return A float representing the vector's length
     */
    float norm();
    
    /**
     Returns a representation of this as a string (x,y,z)
     @return A string representation of this vector
     */
    std::string toString();
};


class KudanMatrix3;
/** 
 Simple representation of a quaternion representing a rotation.
 */
class KudanQuaternion
{
public:
    /** Public x, y, z, w coefficients of the quaternion */
	float x;
	float y;
	float z;
	float w;
    
    /**
     Default constructor initialises the data to the zero quaternion (0,0,0,0)'
     */
    KudanQuaternion();
    /**
     Constructor for initialising the data to the given quaternion. Careful of the order: x, y, z, w
     @param x,y,z,w Coefficients of the quaternion
     */
    KudanQuaternion(float x, float y, float z, float w);
    /**
     Set the all coefficients to zero
     */
    void zero();
    
    /**
     A function to convert from a quaternion to a rotation matrix
     @param q The input quaternion
     @return a KudanMatrix3 rotation matrix 
     */
    static KudanMatrix3 quaternionToRotation(KudanQuaternion &q);
    
    /**
     Turns this quaternion into a rotation matrix
     @return a KudanMatrix3 rotation matrix representing the same rotation as this
     */
    KudanMatrix3 toRotation();
    
    /**
     Returns a representation of this as a string (x,y,z;w)
     @return A string representation of this quaternion
     */
    std::string toString();
    
    /** Get the norm of this Quaternion (should usually be 1).
     This is the Euclidean/L2 norm of the 4-vector representing the quaternion
     */
    float norm();
    
    /** Normalises this quaternion (so it is a 4-vector with Euclidean/L2 norm 1)
     */
    void normalise();
};

/**
 Represents an intrinsic camera calibration matrix (i.e. 3x3). Data are stored as an array of floating point type in column major order
 0 3 6
 1 4 7
 2 4 8
 */
class KudanMatrix3
{
public:
    /// Raw data, stored vectorised in column major order 
    float data[9];
    /**
     Default constructor, sets all elements to 0
     */
    KudanMatrix3();
    
    /** Constructor to make a rotation matrix from a quaternion
     @param q The input quaternion
     */
    KudanMatrix3(KudanQuaternion &q);
    
    /**
     Access operator for the elements. This abstracts away details of the storage. It can be used to get and set values because it returns a reference
     @param row,col The coordinates of the data element to retrieve
     @return A reference to the respective element
     */
    float& operator()(int row, int col);
    
    /** 
     Multiply a vector by this matrix M (vector on the right)
     @param vector A 3x1 vector V
     @return A new 3x1 vector, the result of M*V
     */
    KudanVector3 multiply(KudanVector3 vector);
    
    
    /** Overloaded multiplication operator - calls the matrix-vector multiply function
     @param vector A 3x1 vector V
     @return A new 3x1 vector, the result of M*V
     */
    KudanVector3 operator*(KudanVector3 vector);
    
    /**
     Returns a representation of this as a string (three lines with line breaks)
     @return A string representation of this matrix
     */
    std::string toString();
    
};


/**
 Represents a 4x4 matrix. Data are stored as an array of floating point type in column major order.
 0 4 8  12
 1 5 9  13
 2 6 10 14
 3 7 11 15
 */
class KudanMatrix4
{
public:
    /// Raw data, stored vectorised in column major order
	float data[16];
    /**
     Default constructor, sets all elements to 0 
     */
    KudanMatrix4();
    
    /**
     Access operator for the elements. This abstracts away details of the storage. It can be used to get and set values because it returns a reference
     @param row,col The coordinates of the data element to retrieve
     @return A reference to the respective element
     */
    float& operator()(int row, int col);
    
    /**
     Returns a representation of this as a string (four lines with line breaks)
     @return A string representation of this matrix
     */
    std::string toString();
};



/** 
 Contains various utility functions which the Kudan interface also provides. These are various computer vision and image processing routines not specifically linked to any of the core features (tracking, mapping etc) but which could be useful for related tasks
 */
class KudanUtilities
{
public:
    
    /**
     Get the average colour from a BGR image. Given an image represented by a pointer to unsigned char data, with specified dimensions and channels, this computes the average BGR colour (after down-sampling to a given size for efficienty)
     @param imageData Pointer to the data expressed as a contiguous array of unsigned chars
     @param width, height Size of the image data
     @param channels The number of channels in the data
     @param padding Padding of the data at the end of each row (?)
     @param targetWidth, targetHeight Size to which the image is resized in order to accelerate computation of the mean
     @return A vector representing the mean colour as BGR in the range [0,255]^3
     @throws KudanException is thrown if the data pointer is null or if the number of channels is not valid
     */
    static KudanVector3 getAverageImageColour(const unsigned char *imageData,  int width, int height, int channels, int padding, int targetWidth, int targetHeight);
    
};

/** 
 An image which can be tracked.
 An Image Trackable is a representation of an image which can be detected and tracked by the KdImageTracker. It stores a representation of the 2D image suitable for tracking, with various other settings (such as its size in pixels). The class will also represent the pose of the tracked image with respect to the camera when tracking is successful, plus other state.
 */
class KdImageTrackable
{
private:
    
    friend class KdImageTracker;

    class Private;
    std::shared_ptr<Private> privateData;

public:
    
    /** 
     This is the only constructor, which cannot be used (because it needs Private data access). 
     Create a KdImageTrackable using the createFromImageFile (etc) static methods
     */
    KdImageTrackable(std::shared_ptr<Private>);
    
    /** 
     Create a new Trackable from an image file. The image is loaded from the specified path, if it exists, and processed to create a Trackable object.
     @param path Location of th eimage file (jpeg or png)
     @param name Name of this Trackable (optional). If not given, the name is derived from the path
     @return On success a pointer to the new KdImageTrackable; a nullptr on fail 
     @throw KudanException is thrown if the image cannot be read or is empty.
     */
    static std::shared_ptr<KdImageTrackable> createFromImageFile(std::string path, std::string name = "", bool autoCrop = false);
    
    /** 
     Create a new Trackable directly from a file. This is for loading data processed by the Kudan Toolkit in the .KARMarker format, not for reading image files.
     @param filePointer A file pointer refering to a .KARMarker file, created with the Kudan Toolkit.
     @return On success a pointer to the new KdImageTrackable; a nullptr on fail
     */
    static std::shared_ptr<KdImageTrackable> createFromFilePointer(FILE* filePointer);
    
    /** 
     Create a new Trackable from image data. These data should be in the form of a pointer to an unsigned character array. The format of the data must match the specified size and number of channels, otherwise this can cause errors which won't be detected. If Kudan is being used with OpenCV this is easily used with a cv::Mat: createFromImageData(mat.data, "name", mat.size().width, mat.size().height, mat.channels(), 0);
     In general the image data should be ordered such that the channel c pixel at (x,y) is at position [(width * y + x)*channels + c] (in the same manner as OpenCV)
     @param imageData Pointer to the actual image data
     @param name Name of this Trackable
     @param width, height, channels, padding Parameters of the image data
     @return On success a pointer to the new KdImageTrackable; a nullptr on fail
     @throw KudanException is thrown if the data pointer is null or if a wrong number of channels is given
     */
    static std::shared_ptr<KdImageTrackable> createFromImageData(const unsigned char *imageData, std::string name, int width, int height, int channels, int padding);

    /** Query whether tracking of this Trackable is currently deemed reliable.
     @return True is tracking is happening in the normal way, false if tracking is not able to use the full matching method
     */
    bool isTrackingReliable();
    
    /** Query whether the current tracking state was achieved through recovery-based detection (related to the recovery mode).
     This returns 0 if detection happened in the usual way, and nonzero if recovery was used (1 indicates recovery re-detection, 2 indicates instant recovery while tracking)
     @return An integer representing whether recovery was used (1, 2) or not (0)
     */
    int isRecovered();
    
    /** Is this Trackable currently detectd by the image tracker?
     This is equivalent to asking whether this Trackable will be in the detected Trackables list
     @return A boolean indicating whether this Trackable was successfully tracked in the most recent process frame. 
     */
    bool isTracked();
    
    /** Get the 6dof pose of the image Trackable, with respect to the camera.
     This represents the position of the centre of the image in 3D in the camera coordinate frame.
     @return A KudanVector3 which represents a 3D point (x, y, z)
     */
    KudanVector3 getPosition();
    
    /** Get the 6dof pose of the image Trackable, with respect to the camera.
     This represents the orientation of the image about its own centre, as a unit quaternion
     @return a KudanQuaternion which represents a 4D quaternion (x, y, z, w)
     */
    KudanQuaternion getOrientation();
    
    /** 
     Return the number of points used to track this Trackable and estimate its pose. This can be used as a crude way of evaluating how well this Trackable is tracked.
     @return int A number, representing the number of tracked points.
     */
    int getNumTrackedPoints();
    
    /** 
     Get a vector of 2D points representing the locations of points on the *camera image* used for tracking this Trackable.
     This will have length equivalent to getNumTrackedPoints()
     @return A vector of KudanPoints, describing points on the current camera image
     */
    std::vector<KudanVector2> getTrackedPoints();
    
    /** 
     Get a vector of exaclty 4 (or if tracking is not running, zero) points, which are the corners of the tracked region in the camera image
     @return A vector of 0 or 4 KudanPoints, describing the corners of a quadrilateral in the image 
     */
    std::vector<KudanVector2> getTrackedCorners();
    
    /** 
     Get the name of this Trackable.
     @return A string representing the name of this trackable
     */
    std::string getName();
    
    /** 
     Set the name on this Trackable. This is stored in the unerlying tracker state and is used to find the correct data (be careful with duplicates)
     @param name The name which will be assigned to this trackable
     */
    void setName(std::string name);
    
    /**
     Get the size of the Trackable image with respect to 3D units along its local X axis. If the size has been set, this affects the width returned.
     @return The width of the trackable
     */
    float getWidth();
    
    /**
     Get the size of the Trackable image with respect to 3D units along its local Y axis. If the size has been set, this affects the height returned.
     @return The height of the trackable
     */
    float getHeight();
    
    /** Set the value which represents the width of this trackable (in the units / reference frame of your choice)
     This sets what size the trackable will have in the 3D coordinate frame in which its pose is represented. It does not affect tracking.
     @param width The width of the trackable in some units
     */
    void setWidth(float width);
    
    /** Set the value which represents the height of this trackable (in the units / reference frame of your choice)
     This sets what size the trackable will have in the 3D coordinate frame in which its pose is represented. It does not affect tracking.
     @param height The height of the trackable in some units
     */
    void setHeight(float height);
    
    /**
      Set the Trackable's size. This might change some graphics but will not change the underlying image or tracking behaviour. It should be used to specify dimensions which make sense for the given Trackable (e.g. real world scale).
     This will not allow you to change the aspect ratio! Set only to consistent multiplies of the current size (altering the width or height independently does not make sense - this is really a scale factor)
     @param width,height The dimensions of the Trackable image. These can be in any units, but should always have the same aspect ratio as the actual image.
     @throws KudanException is thrown if the Trackable is not initialised or if the width or height are invalid
     @return A boolean indicating succcess (true) or failure (false) of changing the size
     */
    bool setSize(float width, float height);
    
    /** 
     Set the scale at which this trackale is represented in the 3D coordinate system. This does not change internal state, and is equivalent to calling setSize.
     Using setScale(1) will mean the trackable's size is represented in pixel units of the actual trackable image
     @param scale Scale factor relating the trackable image size to the apparent size in 3D
     */
    void setScale(float scale);
    
    /**
     This allows the size of the displayed region to be manually changed, to arbitrary location and size.
     This does not change the tracking at all, but just the way the result of the tracking is viewed.
     For example, setting the x and y parameters to 0 and the width and height to 10 will make it appear as though the trackable is a very small square, occupying the top left corner only of the trackable image.
     Note that the units are the same as those in which the trackable size is expressed in. This means that setting the virtual cropping should be done after specifying the real world size (for example, setting the width to 100 then setting the cropped width to 50 will mean that half of the horizontal size of the trackable will be covered).
     Some checks are carried out (resulting area must be positive, for example), and there are some limitations (a trackable which is already extended or auto-cropped cannot be re-cropped). The return value will report whether cropping was applied.
     @param x,y The coordinates of the top left corner of the virtual tracked, region in trackable units
     @param w,h The size of the virtual tracked region, in trackable units
     @return A boolean indicating whether the virtual cropping was applied successfully.
     */
    bool setVirtualCrop(float x, float y, float w, float h);
    
    /**
     If this trackable has been virtually cropped, this rests it to its initial state.
     This might not always be possible, e.g. if the trackable has since been extended
     @return Whether this trackable could be reset to its original size */
    bool resetVirtualCrop();
    
    /**
     This returns whether the trackable was automatically cropped or not. Note that even if auto-cropping was requested, it might not have been applied, if it was deemed not to be necessary: this returns whether cropping actually happened.
     Note also that virtual cropping cannot be applied to an auto-cropped marker */
    bool isAutoCropped();
    
    /**
     Turn on recovery mode for this Trackable, if enabled. This Trackable can use recover mode if the global setting is switched on (default to global setting). This makes re-detection of previously seen Trackables quicker and possible from a greater range of angles.
     */
    void allowRecoveryMode();

    /**
     Turn on recovery mode for this Trackable. This Trackable will have recovery mode emabled irrespective of the global setting.
     */
    void forceRecoveryMode();
    
    /**
     Switch off recovery mode for this Trackable. This prevents this Trackable from using recovery mode, irrespective of the global setting.
     */
    void prohibitRecoveryMode();
    
    /** 
     Query recovery mode for this Trackable. This could depend on the global setting, depending how it was set.
     @param globalSetting Specifies whether recovery mode is allowed in the tracker
     @return true if recovery mode is enabled for this Trackable, given the Trackable's setting and the global state; otherwise false.
     */
    bool queryRecoveryMode(bool globalSetting);
    
    /**
     Turn on/off extended detection and tracking for this Trackable. This will allow new Trackables to be automatically created at increasing scales with respect to the original trackable image, to increase the range from which it can be detected and tracked.
     @param isExtensible Set whether extended tracking and detection is on (true) of off (false). If switched on, new trackbles will be automatically created; if switched off, this stops, but extisting extensions will remain (see clearExtensions) */
    void setExtensible(bool isExtensible);
    
    /**
     Query whether this Trackable can be extended or not (see setExtensible)
     */
    bool isExtensible();
    
    /**
     Query whether this Trackable has existing extensions or not (i.e. extended tracking and detection was enabled and new Trackables have been created
     */
    bool isExtended();
    
    /**
     If this Trackable has any extensions, it will delete them (so they can be re-created). If any of the extensions are being tracked, tracking will immediately fail. */
    void clearExtensions();

    
};



/**
 Represents the intrinsic calibration parameters of a pin-hole camera.
 This encodes the focal length (x and y separate) and principal point, as well as the image size. If the intrinsics are not known, this class can be used to guess them for a camera of a given size. This is used to specify the parameters of a camera for the tracker.
 */
class KudanCameraParameters
{
private:
    
    int w,h;
    float fx,fy,px,py;

public:
    /// This is to stop obviously wrong camera parameters being passed in
    static const int maxImageSize = 1 << 30;

    /**
     Default constructor sets everything to zero.
     Should not be available to user!
     */
    KudanCameraParameters();
    
    /**
     Create an instance using known camera parameters.
     @param focalLengthX,focalLengthY The focal length of the camera in pixels
     @param principalPointX,principalPointY The principal point of the camera in pixels
     @param width,height The size of the images created by this camera
     @throw KudanException is thrown if the parameters are invalid, including: the image size is zero, negative, or too large in any dimension; if the principal point is outside the image; and if the focal length is zero or negative.
     */
    KudanCameraParameters(float focalLengthX, float focalLengthY, float principalPointX, float principalPointY, float width, float height);
    
    /**
     (Re)set the intrinsic parameters (does not change the image size)
     @param focalLengthX,focalLengthY The focal length of the camera in pixels
     @param principalPointX,principalPointY The principal point of the camera in pixels
     @throw KudanException is thrown if the parameters are not valid
     */
    void setIntrinsics(float focalLengthX, float focalLengthY, float principalPointX, float principalPointY);

    /**
     (Re)set the intrinsic size (does not change the intrinsic parameters)
     @param width,height The size of the images created by this camera
     @throw KudanException is thrown if the parameters are not valid
     */
    void setSize(int width, int height);

    /** 
     Assuming the width and height have been set, this guesses appropriate values for the intrinsics.
     If the image size corresponds to a certain set of common devices, the pre-calibrated parameters are used. Otherwise, appropriate approximations are used given the image size.
     This will throw a KudanException if the parameters are not valid
     @throw KudanException is thrown if the parameters are not valid
     */
    void guessIntrinsics();
    
    /**
     Get the 3x3 camera calibration matrix. The data are stored in column major order.
     @return a KudanMatrix3 representing the 3x3 instinsic camera matrix
     */
    KudanMatrix3 getMatrix();

    /**
     Get the 4x4 OpenGL-style projection matrix
     @param near Rendering frustum near clip plane
     @param far Rendering frustum far clip plane
     @return a KudanMatrix representing the 4x4 OpenGL proejction matrix
     */
    KudanMatrix4 getProjectionMatrix(float near, float far);

    /**
     Access the intrinsic parameters of the camera.
     This returns the focal length in the X and Y directions
     @param [out] focalX,focalY Focal length in pixels
     */
    void getFocalLength(float &focalX, float &focalY);
    
    /**
     Access the intrinsic parameters of the camera.
     This returns the principal point in pixel units.
     @param [out] principalX,principalY Principal point in pixels
     */
    void getPrincipalPoint(float &principalX, float &principalY);
    
    /**
     Access the intrinsic parameters of the camera.
     This returns the size in pixels of the image that is expected from the camera
     @param [out] width,height Image size for this camera
     */
    void getSize(int &width, int &height);
    
    /**
     Utility function to check that valid values are set. This is only needed because there is a defaut constructor! 
     @return A boolean indicating whether this camera is valid
     */
    bool isValid();
    
    /**
     Access the intrinsic parameters of the camera.
     This returns the width in pixels of the image that is expected from the camera
     @return The width of the expected camera image
     */
    int width();
    
    /**
     Access the intrinsic parameters of the camera.
     This returns the height in pixels of the image that is expected from the camera
     @return The height of the expected camera image
     */
    int height();
};

/**
 @brief The image tracker, for detecting and tracking KdImageTrackables in an image stram 
 */
class KdImageTracker
{
private:
    
    class Private;
    std::shared_ptr<Private> privateData;
    
public:
    
    /**
     Default constructor. This automatically initialises the tracker.
     */
    KdImageTracker();
    
    /**
     This deinitialises the tracker automatically
     */
    ~KdImageTracker();
    
    /**
     Process a new frame for detection and tracking. Any non-tracked Trackables will be searched for (depending on settings), and any Trackables visible in the previous frame will be tracked into this frame.
     This will fail (return false) if a valid license is not set.
     @param image 8bpp greyscale image data.
     @param width,height size of the image
     @param channels number of channels in the image (mono, colour, colour+alpha)
     @param padding the number of bytes to ignore at the end of each line
     @param requireFlip some devices provide the image upside down (rotation of 180 degrees). This flips the image (in both axes) before processing
     @return A boolean indicating whether tracking happened (true) or not (false)
     @throw KudanException is thrown if there is no calibration data or an empty image.
     */
    bool processFrame(const unsigned char *image, int width, int height, int channels, int padding, bool requireFlip);
    
    /**
     Get the number of Trackables loaded successfully into the tracker
     */
    int getNumberOfTrackables();
    
    /**
     Get a list of pointers to all existing Trackables
     @return A list of (pointers to) KdImageTrackable objects, representing all of the Trackables currently loaded.
     */
    std::vector<std::shared_ptr<KdImageTrackable>> getTrackables();
    
    /**
     Get the number of Trackables successfully tracked in this frame
     @return The number of currently tracked Trackables.
     */
    int getNumberOfDetectedTrackables();
    
    /**
     Get a list of pointers to all Trackables successfuly tracked in the last frame
     @return A list of (pointers to) KdImageTrackable objects, representing all of the Trackables currently tracked.
     */
    std::vector<std::shared_ptr<KdImageTrackable>> getDetectedTrackables();
    
    /**
     Create and add a single Trackable from an image.
     @param path The location of the image
     @param name A name for the Trackable. If empty (default) the name is derived from the image path.
     @return boolean indicating if the Trackable was created and added successfully */
    bool addTrackable(const std::string path, std::string name = "");
    
    /**
     Load a Trackable set from an file (created with the Toolkit)
     @param path The location of the .KARMarker file
     @return The number of Trackables successfully loaded from this set*/
    int addTrackableSet(const std::string path);
    
    /**
     Load a Trackable set from data (created with the Toolkit)
     @param data Raw Trackable file data
     @param length The length of the data
     @return The number of Trackables successfully loaded */
    int addTrackableSet(const unsigned char *data, size_t length);
    
    /**
     Add a Trackable to the tracker. The Trackable should be valid and created already.
     @return Boolean indicating whether it was successfully added to the underlying tracker */
    bool addTrackable(std::shared_ptr<KdImageTrackable> kudanTrackable);
    
    /**
     Caution: not implemented yet
     */
    bool removeTrackable(std::shared_ptr<KdImageTrackable> kudanTrackable);
    
    /**
     Get a pointer to a KdImageTrackable by its name.
     This is so that settigns can be set on it */
    std::shared_ptr<KdImageTrackable> getTrackable(std::string name);
    
    // === Camera calibration ===
    
    /**
     Queries if a calibration has been set (with setCameraParameters).
     @return A boolean which is true if the calibration has been set
     */
    bool hasCameraCalibration();
    
    /**
     Sets the camera parameters using given camera parameters.
     @param cameraParameters Object of KudanCameraParameters which defines the camera intrinsics and image size.
     @throws KudanException is thrown if the parameters are invalid
     */
    void setCameraParameters(KudanCameraParameters &cameraParameters);
    
    /**
     Get a (reference to) the camera parameters set on the tracker.
     @return A reference to a KudanCameraParameters object. Altering this will change the one stored in the tracker
     */
    KudanCameraParameters &getCameraParameters();
    
    /** 
     Get the calibration matrix for this camera, derived from the intrinsic parameters.
     This is a 3x3 matrix represented as the float array within a KudanMatrix3
     @return A KudanMatrix3 representing the intrinsic parameters
     @throws KudanException is thrown if the parameters have not been set
     */
    KudanMatrix3 getCameraMatrix();
    
    /** 
     Get the projection matrix for this camera, derived from the intrinsic parameters and the given clipping planes
     This is a 4x4 matrix represented as the float array within a KudanMatrix.
     @param nearPlane, farPlane The clipping planes to be used for rendering (not part of the camera parameters).
     @return A KudanMatrix representing the intrinsic parameters plus clipping planes.
     @throws KudanException is thrown if the parameters have not been set
     */
    KudanMatrix4 getProjectionMatrix(float nearPlane, float farPlane);

    
    // === Tuning ===
    
    /**
     Set the detector sensitivity. High sensitivity values mean the detector will more readily make detections (at the expense of accuracy). Lower values indicate more caution / robustness at the expense of missing detections
     @param sensitivity Sensitivity value in range [0,1]
     */
    void setDetectionSensitivity(float sensitivity);
    
    /**
     Resets the detector sensitivity to default values
     */
    void resetDetectionSensitiviy();
    
    // === Computer vision ===
    
    /** 
     Set the maximum number of Trackables that can be tracked at any one time. This does not limit the number which are searched for, until all possible Trackables are being tracked (at which point detection stops)
     Setting to 0 means there is no limit
     @param maxTrack The maximum number of Trackables which can be tracked at a time.
     */
    void setMaximumSimultaneousTracking(int maxTrack);
    
    /** 
     Set whether the detector is allowed to look for multiple Trackables in parallel or not.
     The number of threads used (if on) is set automatcally and is platform-specific.
     @param doParallel Set if parallel detection is on or off
     */
    void toggleParallelDetection(bool doParallel);
    
    /**
     Query whether the detector is allowed to look for multiple Trackables in parallel or not
     @return Boolean indicating whether parallel detection is enabled.
     */
    bool isDetectorParallel();
    
    /**
     Toggle recover mode. This turns recovery mode on/off for all Trackables, unless set otherwise on individual Trackables
     @param recovery Set whether recovery mode is on or off
     */
    void setRecoveryMode(bool recovery);
    
    /** 
     Completely switch off recovery mode. Recovery mode cannot be used on ANY Trackable, despite individual settings.
     */
    void prohibitRecoveryMode();
    
    /**
     Query whether recovery mode is being used (for Trackables which are not set individually)
     @return A boolean indicating if recovery mode is globally on or off
     */
    bool queryRecoveryMode();
    
    /** 
     Query whether recovery mode is being used for the specific Trackable. This needs to look at the global state to find out.
     @param trackable Pointer to a Trackable object. Returns false if Trackable is nullptr.
     @return A boolean indicating if recovery mode is on or off for the specified Trackable.
     */
    bool queryRecoveryMode(std::shared_ptr<KdImageTrackable> trackable);

};



/**
 @brief Exception class thrown by all the Kudan classes 
 */
class KudanException : public std::runtime_error
{
  
public:
    /** 
     Creates an exception and sets the message stored within
     */
    KudanException(std::string msg);
};


#endif /* defined(__KdImageTrack__Interface__) */
