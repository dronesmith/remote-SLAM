#define VERSION "0.1.2xxx"

//  KudanSLAM
//  Copyright © 2016 Kudan. All rights reserved.
//

#ifndef Interface_hpp
#define Interface_hpp

#include <memory>
#include <vector>
#include <string>

// Forward declarations for types within namespaces:
namespace cv
{
    class Mat;
}

namespace KudanSlamInternal
{
    class SLAMSystem;
}

/** A simple representation of a 2D vector
 */
class KudanVector2
{
public:
    /// X coordinate
    float x;
    /// Y coordinate
    float y;
};

/** A simple representation of a 3D vector
*/
class KudanVector3
{
public:
    /// X coordinate
    float x;
    /// Y coordinate
    float y;
    /// Z coordinate
    float z;
};

/** A simple representation of a 3D orientation, as a quaternion.
 This provides a convenient class for representing the data, but no methods for manipulation.
 */
class KudanQuaternion
{
public:
    /// X coordinate
    float x;
    /// Y coordinate
    float y;
    /// Z coordinate
    float z;
    /// W coordinate
    float w;
};

/** A simple representation of a 3x3 matrix of floating point numbers.
 */
class KudanMatrix3
{
public:
    /// Representation of matrix data in row-major format (i.e. first row, then second row, etc)
    float data[9];
};

/** A simple representation of a 4x4 matrix of floating point numbers.
 */
class KudanMatrix4
{
public:
    /// Representation of matrix data in row-major format (i.e. first row, then second row, etc)
    float data[16];
};

/** A basic representation of an image. The image data are represented as 8-bit (unsigned char) arrays. This assumes rectangular single-channel (greyscale) images.
 */
class KudanImage
{
public:
    /// Default initialisation (ensures that data are null and size is zero unless specified)
    KudanImage();
    /// Width of the image. Defaults to 0 if not initialised.
    size_t width;
    /// Height of the image. Defaults to 0 if not initialised.
    size_t height;
    /// The image data, for a single channel 8-bit image. The size allocated to this must correspond to width * height. Defaults to NULL if not initialised.
    unsigned char *data;
};

/** A representation of a camera frame, for input to the SLAM system. This will contain image data (for one or more cameras), plus other inputs (e.g. IMU)
 */
class KudanFrame
{
public:
    /// Left greyscale image (for stereo, or the single monocular image)
    KudanImage leftImage;
    /// Right greyscale image for a stereo pair
    KudanImage rightImage;
};


/** This class represents a whole SLAM system. Create an object of this to run SLAM!
 */
class KudanSLAM
{

public:

    /** This describes the states that the SLAM system can be in, from initialisation through to tracking, including failure and recovery.
        */
    enum class State
    {
        /// Not doing anything, especially before initialisation
        Idle,
        /// Attempting to initialise tracking
        Initialising,
        /// Tracking is happening
        Tracking,
        /// Tracking has been lost (recovery might be in progress)
        Lost,
        /// Tracking was recently recovered (intermediate careful state)
        Found,
        /// Tracking and mapping have finished and will not resume
        Finished,
    };
    
    /** This describes different return conditions for the processFrame function, describing what (if anything) went wrong.
     */
    enum class ProcessFrameError
    {
        /// No error: processing was fine (tracking might not be!)
        EverythingIsFine,
        /// A problem with the input image. Check the input images are provided and the data are the right size / not null
        InputImageError,
        /// A problem with the camera calibration. Check it has been given to SLAM and is valid
        CalibrationError,
        /// The frame was processed for initialisation but this failed for some reason. NOTE: This is not an error as such, as initialisation can fail (depending on input) but might be indicative of a deeper problem
        InitialisationError, // Not sure about this one!
        /// Unknown error occurred / error type is undefined
        Unknown
    };
    
    /** This describes things that can go wrong when trying to set camera parameters
     */
    enum class CameraParameterError
    {
        /// No error, everything was fine
        EverythingIsFine,
        /// The size is invalid, might be zero or negative
        BadSize,
        /// The focal length is invalid, might be zero or negative
        BadFocalLength,
        /// The focal length is invalid, might be zero or negative, or more than the image size (principal point must lie within the image)
        BadPrincipalPoint,
        /// The calibration used for this operation was not valid
        InvalidCalibration,
        /// Unknown error occurred / error type is undefined
        Unknown
    };
    
private:


    bool requestCameraPoses;
    bool requestCameraMatrices;
    bool request2DPoints;
    bool request3DPoints;
    
    std::shared_ptr<KudanSlamInternal::SLAMSystem> slamSystem;
    
    
    std::vector<KudanVector3> visiblePoints, allMapPoints, trackedPoints3D;
    std::vector<KudanVector2> projections, trackedPoints;
    
    
    KudanVector3 cameraT;
    KudanQuaternion cameraQ;
    KudanMatrix3 cameraR;
    KudanMatrix4 cameraViewMatrix;
    
    KudanVector3 cameraC;
    KudanQuaternion cameraQt;
    KudanMatrix3 cameraRt;
    KudanMatrix4 cameraTransformMatrix;
    
    // Camera
    class CameraDistortionInformation;
    std::shared_ptr<CameraDistortionInformation> cameraDistortionInformation;
    bool didUndistortFrame;
    
    
    /// A more direct version for setting rectification parameters - this is for internal use
    CameraParameterError setCameraRectification(cv::Mat K_l, cv::Mat R_l,
            std::vector<float> distortionLeft,
            cv::Mat K_r, cv::Mat R_r,
            std::vector<float> distortionRight);
            
            
public:


    /** Default constructor for the SLAM system interface. This will initialise everything to default settings.
     */
    KudanSLAM();
    
    
    /** This sets the initialisation variable, so SLAM will initialise on the next frame.
     */
    void startTracking();
    
    
    
    /** This is the function with which new data are passed into SLAM to process. Send in one frame of image/auxiliary data and it will be processed, yielding a new estimate of the camera pose on completion, and (ultimately) an updated map, after optimisation is finished.
     @param frame A frame of data for processing. This could comprise a single image (mono SLAM), a stereo pair (stereo SLAM) or other data (not implemented yet!). Note that this is a *reference* and can be modified by any undistortion/rectification
     @return Return a boolean indicating successful processing (tracking might still have failed, this reports whether processing happened correctly)
     */
    ProcessFrameError processFrame(KudanFrame &frame);
    
    
    /** Set up the camera intrinsic calibration. This is required for every invocation of SLAM
     @param imageWidth The size of the camera's image
     @param imageHeight The size of the camera's image
     @param focalLengthX The focal length of the camera in the X direction
     @param focalLengthY The focal length of the camera in the Y direction
     @param principalPointX The principal point of the camera (X coordinate)
     @param principalPointY The principal point of the camera (Y coordinate)
     @return A CameraParameterError indicating whether the parameter combination is valid and was successfully set on the SLAM system, or otherwise a code identifying the error
     */
    CameraParameterError setCameraCalibration(float imageWidth, float imageHeight, float focalLengthX, float focalLengthY, float principalPointX, float principalPointY);
    
    /** Set the radial and tangential distortion parameters for the camera.
     If this is set, it will activate distortion mode, i.e. the image will be undistorted before passing to the SLAM tracker.
     NOTE: This must be called after setting the camera intrinsics.
     @param distortionCoefficients Radial (k) and tangential (p) coefficients, in the standard OpenCV order [k0, k1, p0, p1, k2, k3, k4, k5]
     @return A CameraParameterError indicating whether the distortion was set up correctly, and otherwise what went wrong
     */
    CameraParameterError setCameraDistortion(std::vector<float> distortionCoefficients);
    
    /** Set the radial and tangential distortion parameters for the left and right cameras separately for a stereo pair.
     If this is set, it will activate distortion mode, i.e. the image will be undistorted before passing to the SLAM tracker.
     NOTE: This must be called after setting the camera intrinsics.
     @param distortionCoefficientsLeft Radial (k) and tangential (p) coefficients for the left camera, in the standard OpenCV order [k0, k1, p0, p1, k2, k3, k4, k5]
     @param distortionCoefficientsRight Radial (k) and tangential (p) coefficients for the right camera, in the standard OpenCV order [k0, k1, p0, p1, k2, k3, k4, k5]
     @return A CameraParameterError indicating whether the distortion was set up correctly, and otherwise what went wrong
     */
    CameraParameterError setCameraDistortion(std::vector<float> distortionCoefficientsLeft, std::vector<float> distortionCoefficientsRight);
    
    /** Set the rectification extrinsics for a stereo camera pair, plus undistortion for each individual camera.
     NOTE: This must be called after setting the camera intrinsics. Those intrinsics are treated as the intrinsics after rectification; the intrinsics before rectification (for each camera) must also be given.
     @param intrinsicsLeft The intrinsics of the left camera prior to rectification (and undistortion)
     @param rotationLeft The rotation matrix of the left camera with respect to the stereo origin
     @param distortionCoefficientsLeft Distortion coefficients (radial and tangential) for the left camera
     @param intrinsicsRight The intrinsics of the right camera prior to rectification (and undistortion)
     @param rotationRight The rotation matrix of the right camera with respect to the stereo origin
     @param distortionCoefficientsRight Distortion coefficients (radial and tangential) for the right camera
     @return A CameraParameterError indicating whether the distortion/rectification was set up correctly, and otherwise what went wrong
     */
    CameraParameterError setCameraRectification(KudanMatrix3 intrinsicsLeft, KudanMatrix3 rotationLeft,
            std::vector<float> distortionCoefficientsLeft,
            KudanMatrix3 intrinsicsRight, KudanMatrix3 rotationRight,
            std::vector<float> distortionCoefficientsRight);
            
            
    /** Set the rectification extrinsics for a stereo camera pair, plus undistortion for each individual camera.
     NOTE: This must be called after setting the camera intrinsics. Those intrinsics are treated as the intrinsics after rectification; the intrinsics before rectification (for each camera) must also be given.
     @param fx_l The focal length (x) of the left camera prior to rectification (and undistortion)
     @param fy_l The focal length (y) of the left camera prior to rectification (and undistortion)
     @param cx_l The principal point (x) of the left camera prior to rectification (and undistortion)
     @param cy_l The principal point (y) of the left camera prior to rectification (and undistortion)
     @param rotationLeft The rotation matrix of the left camera with respect to the stereo origin
     @param distortionCoefficientsLeft Distortion coefficients (radial and tangential) for the left camera
     @param fx_r The focal length (x) of the right camera prior to rectification (and undistortion)
     @param fy_r The focal length (y) of the right camera prior to rectification (and undistortion)
     @param cx_r The principal point (x) of the right camera prior to rectification (and undistortion)
     @param cy_r The principal point (y) of the right camera prior to rectification (and undistortion)
     @param rotationRight The rotation matrix of the right camera with respect to the stereo origin
     @param distortionCoefficientsRight Distortion coefficients (radial and tangential) for the right camera
    @return A CameraParameterError indicating whether the distortion/rectification was set up correctly, and otherwise what went wrong
     */
    CameraParameterError setCameraRectification(float fx_l, float fy_l, float cx_l, float cy_l, KudanMatrix3 rotationLeft,
            std::vector<float> distortionCoefficientsLeft,
            float fx_r, float fy_r, float cx_r, float cy_r, KudanMatrix3 rotationRight,
            std::vector<float> distortionCoefficientsRight);
            
    /** If distortion parameters have been set, this clears them. Future frames will not be undistorted.
     */
    void resetCameraDistortion();
    
    /** Query whether the last frame processed by SLAM was automatically undistorted or not
     */
    bool frameWasUndistorted();
    
    /** Accessor for the camera's focal length. This should have been set already with setCameraCalibration
     @return Focal length in the X,Y directions represented as a 2D point
     */
    KudanVector2 getFocalLength();
    
    /** Accessor for the camera's principal point. This should have been set already with setCameraCalibration
     @return Principal point represented as a 2D point
     */
    KudanVector2 getPrincipalPoint();
    
    /** Accessor for the camera calibration intrinsics matrix. This is a 3x3 matrix encoding the focal length and principal point (skew is always zero), suitable for use in projection from 3D to 2D.
     @return A KudanMatrix3 class representing the 3x3 intrinsics matrix
     */
    KudanMatrix3 getCalibrationMatrix();
    
    /** Reset the SLAM system. This will return the underlying SLAM system to its initial state, resetting all calibration and settings. Be careful to re-load the necessary settings if used.
     */
    void reset();
    
    /** Get the current SLAM state (e.g. initialising, tracking) as an enum of type SLAMState
     @return The state represented as a State enum. Use describeSlamState to get a readable string
     */
    KudanSLAM::State getState();
    
    /** The SLAM state is represented by an enum: this function converts to a readable string if possible.
     Note this is a static function, it takes a state as input, it does not return the current SLAM state
     @param state The state to be described
     @return string representation of the input state
     */
    static std::string describeSlamState(KudanSLAM::State state);
    
    /// One of many overloaded error-code stringifying functions
    static std::string describeErrorCode(ProcessFrameError code);
    /// One of many overloaded error-code stringifying functions
    static std::string describeErrorCode(CameraParameterError code);
    
    /** Save the current SLAM status of all requested variables (camera, 2D/3D points etc).
     This is called automatically by the processFrame function to snapshot the state after the last update. It could be called to get the most recent state at other times (?)
     @return Whether any state was saved
     */
    bool saveSnapshot();
    
    
    /** Request that the camera poses (represented as rotation matrices, quaternions and translations in camera and world space) should be stored on the next run
     @param request A request to store these data or not
     */
    void setRequestCameraPoses(bool request);
    
    /** Request that the camera poses (represented as 4x4 matrices in camera and world space) should be stored on the next run
     @param request A request to store these data or not
     */
    void setRequestCameraMatrices(bool request);
    
    /** Set whether 3D points are to be saved during the snapshot.
     These are the map points and visible/tracked points, which will be available through the getMapPoints3D functions etc
     @param request A request to store these data or not
     */
    void setRequestPoints3D(bool request);
    
    /** Set whether 2D points are to be saved during the snapshot.
     These are the map points and visible/tracked points, which will be available through the getMapPoints3D functions etc
     @param request A request to store these data or not
     */
    void setRequestPoints2D(bool request);
    
    
    /** Set that the SLAM system will be running in monocular mode. Expect single-image input.
     @return A boolean indicating whether this setting was set correctly
     */
    bool setMonocularMode();
    
    /** Set that the SLAM system will be running in stereo mode. Expect stereo input.
     @return A boolean indicating whether this setting was set correctly
     */
    bool setStereoMode();
    
    
    /** Set the SLAM system to use patches for tracking and expansion
     @return A boolean indicating whether this setting was set correctly
     */
    bool setPatchMode();
    
    
    /** Set the SLAM system to use descriptors for tracking and expansion
     @return A boolean indicating whether this setting was set correctly
     */
    bool setDescriptorMode();
    
    /** Request expansion now. This should *not* be needed in usual SLAM, but remains to provide a way to specify manual expansion.
     @param shouldExpand To set whether expansion should happen now. If set to true, SLAM will expand at the next opportunity; if false, it will cancel any pending expansion requests.
     */
    void setShouldExpand(bool shouldExpand);
    
    
    
    /** Get the map points as projected into 2D by the current camera pose (only those which are in view)
     @return Vector of KudanVector2 representing 2D points
     */
    std::vector<KudanVector2> getMapPoints2D();
    
    /** Get the current tracked points in the 2D image
     @return Vector of KudanVector2 representing 2D points
     */
    std::vector<KudanVector2> getTrackedPoints2D();
    
    /** Get the full set of map points in the 3D world coordinate frame
     @return Vector of KudanVector2 representing 3D points
     */
    std::vector<KudanVector3> getMapPoints3D();
    
    /** Get the subset of 3D points which are currently visible from the current camera (corresponds to MapPoints2D)
     @return Vector of KudanVector2 representing 3D points
     */
    std::vector<KudanVector3> getVisiblePoints3D();
    
    /** Get the subset of 3D points (in the world coordinate frame) which were tracked in the last frame
     @return Vector of KudanVector2 representing 3D points
     */
    std::vector<KudanVector3> getTrackedPoints3D();
    
    /** Get the camera translation. This is the 'T' in the projection matrix P = K[R|T], i.e. the position of the world origin in the camera coordinate frame (*not* the camera centre in the world coordinate frame
     @return A KudanVector3 representing the camera translation (camera coordinate frame)
     */
    KudanVector3 getCameraTranslation();
    
    /** Get the camera rotation. This is the 'R' in the projection matrix P = K[R|T], i.e. the orientation of the camera about the world coordinate frame, not the orientation of the camera about its centre
     @return A KudanMatrix3 representing the camera orientation (camera coordinate frame)
     */
    KudanMatrix3 getCameraRotation();
    
    /** Get the camera rotation as a quaternion. This is the quaternion for of 'R' in the projection matrix P = K[R|T],  i.e. the orientation of the camera about the world coordinate frame, not the orientation of the camera about its centre
     @return A KudanQuaternion representing the camera orientation (camera coordinate frame)
     */
    KudanQuaternion getCameraQuaternion();
    
    /** Get the position of the camera centre in the world coordinate frame (camera transform form)
     @return A KudanVector3 representing the camera position (world coordinate frame)
     */
    KudanVector3 getCameraPosition();
    
    /** Get the camera orientation in the world coordinate frame, about its own centre (camera transform form)
     @return A KudanMatrix3 representing the camera orientation (world coordinate frame)
     */
    KudanMatrix3 getCameraOrientationInWorld();
    
    /** Get the camera orientation in the world coordinate frame, as a quaternion, about its own centre (camera transform form)
     @return A KudanMatrix3 representing the camera orientation (world coordinate frame)
     */
    KudanQuaternion getCameraQuaternionInWorld();
    
    /** Get a 4x4 matrix representing the camera transformation. This is the transformation from world space to camera space, i.e. represents the camera position in the world coordinate frame.
     It has the form of a 4x4 matrix where the upper-left 3x3 is the rotation matrix of the camera about its centre, and the upper-right 3x1 is the camera's 3D position in the world coordinate frame.
     @return A 4x4 matrix representing the camera transformation
     */
    KudanMatrix4 getCameraTransformMatrix();
    
    /** Get a 4x4 matrix representing the camera view matrix. This is the transformation from camera space to world space, i.e. it represents the transformation of 3D points in the camera coordinate frame to points in the world. It is used as part of the 3D-to-2D projection operation, x = K.P.X (this is P, for a 4x4 camera K and a 4D homogeneous point X).
     It has the form of a 4x4 matrix where the upper-left 3x3 is the rotation matrix of the world about the camera centre, and the upper-right 3x1 is the world origin's 3D position in the camera coordinate frame (i.e. R and T)
     @return A 4x4 matrix representing the camera transformation
     */
    KudanMatrix4 getCameraViewMatrix();
    
    /** Get the total number of keyframes accessible to the tracker
     @return Number of keyframes
     */
    int getNumKeyframes();
    
    /** Get the number of keyframes accessible to the tracker which are not culled
     @return Number of non-culled keyframes
     */
    int getNumGoodKeyframes();
    
    /** Get the time taken for the last frame to be processed
     The processing time for the last frame in seconds
     @return Processing time for the last frame (s)
     */
    float getTrackingTime();
    
    /** Get the effective frame rate as of the past processed frame
     @return The effective frame rate for the last frame (Hz)
     */
    float getTrackingRate();
    
    
    /** Is bundle adjustment happening right now?
     */
    bool isBundleAdjusting();
    
    /** Print SLAM version number if available
     */
    static std::string version()
    {
#ifdef VERSION
        std::string version = VERSION;
        return "Version " + version;
#else
        return "Unknown version!";
#endif
    }
    
};


#endif /* Interface_hpp */
