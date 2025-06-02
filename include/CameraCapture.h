#ifndef CAMERA_CAPTURE_H
#define CAMERA_CAPTURE_H

#include <opencv2/opencv.hpp>
#include <string>
#include <mutex>
#include <thread>

// ZED SDK headers (conditional compilation)
#ifdef USE_ZED_SDK
#include <sl/Camera.hpp>
#endif

/**
 * @brief Camera capture modes
 */
enum class CameraMode {
    STANDARD_CAMERA,  // Standard USB camera
    ZED_CAMERA        // ZED stereo camera
};

/**
 * @brief Class for camera operations and frame capture with standard or ZED cameras
 */
class CameraCapture {
public:
    /**
     * @brief Construct a new Camera Capture object
     */
    CameraCapture();
    
    /**
     * @brief Destroy the Camera Capture object
     */
    ~CameraCapture();

    /**
     * @brief Initialize standard camera
     * 
     * @param cameraId Camera device ID
     * @param width Desired frame width
     * @param height Desired frame height
     * @param fps Desired frame rate
     * @return true if initialization successful
     * @return false if initialization failed
     */
    bool initStandardCamera(int cameraId = 0, int width = 640, int height = 480, int fps = 30);
    
#ifdef USE_ZED_SDK
    /**
     * @brief Initialize ZED stereo camera
     * 
     * @param width Desired frame width
     * @param height Desired frame height
     * @param fps Desired frame rate
     * @param depth_mode Depth mode (PERFORMANCE, QUALITY, ULTRA)
     * @return true if initialization successful
     * @return false if initialization failed
     */
    bool initZEDCamera(int width = 1280, int height = 720, int fps = 30, 
                       const std::string& depth_mode = "PERFORMANCE");
                       
    /**
     * @brief Initialize the ZED camera with a SVO recording file for testing
     * 
     * @param svoFile Path to SVO file
     * @return true if initialization successful
     * @return false if initialization failed
     */
    bool initWithSVO(const std::string& svoFile);
#endif
    
    /**
     * @brief Initialize the camera with a video file for testing
     * 
     * @param videoFile Path to video file
     * @return true if initialization successful
     * @return false if initialization failed
     */
    bool initWithVideoFile(const std::string& videoFile);
    
    /**
     * @brief Get the latest frame from camera
     * 
     * @param frame Output color frame
     * @param depthMap Optional output depth map (only valid with ZED camera)
     * @return true if frame captured successfully
     * @return false if capture failed
     */
    bool getFrame(cv::Mat& frame, cv::Mat* depthMap = nullptr);
    
    /**
     * @brief Start capturing frames in a separate thread
     * 
     * @return true if successfully started
     * @return false if failed to start
     */
    bool startCapture();
    
    /**
     * @brief Stop capturing frames
     */
    void stopCapture();
    
    /**
     * @brief Check if camera is properly connected and initialized
     * 
     * @return true if camera is ready
     * @return false if camera is not ready
     */
    bool isReady() const;

    /**
     * @brief Get frame width
     * 
     * @return int Frame width
     */
    int getWidth() const;
    
    /**
     * @brief Get frame height
     * 
     * @return int Frame height
     */
    int getHeight() const;
    
    /**
     * @brief Get the camera mode (standard or ZED)
     * 
     * @return CameraMode Current camera mode
     */
    CameraMode getCameraMode() const;
    
#ifdef USE_ZED_SDK
    /**
     * @brief Set the depth confidence threshold for ZED camera
     * 
     * @param confidenceThreshold Confidence threshold (1-100)
     */
    void setDepthConfidenceThreshold(int confidenceThreshold);
#endif

private:
    /**
     * @brief Thread function for continuous frame capture
     */
    void captureThread();

#ifdef USE_ZED_SDK
    /**
     * @brief Convert ZED image format (BGRA) to OpenCV format (BGR)
     * 
     * @param zedImage ZED image
     * @return cv::Mat OpenCV image
     */
    cv::Mat zedImageToCV(const sl::Mat& zedImage);

    sl::Camera m_zed;                // ZED camera object
    sl::RuntimeParameters m_runtime; // ZED runtime parameters
    cv::Mat m_latestDepthFrame;      // Latest captured depth frame
#endif

    cv::VideoCapture m_camera;       // OpenCV camera object
    cv::Mat m_latestFrame;           // Latest captured color frame
    std::mutex m_frameMutex;         // Mutex for thread safety
    
    bool m_isRunning;                // Thread control flag
    bool m_isReady;                  // Camera initialization status
    CameraMode m_cameraMode;         // Current camera mode
    
    int m_width;                     // Frame width
    int m_height;                    // Frame height
    int m_fps;                       // Frame rate
    
    std::thread m_captureThread;     // Frame capture thread
};

#endif // CAMERA_CAPTURE_H 