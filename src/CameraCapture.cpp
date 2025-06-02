#include "CameraCapture.h"
#include <iostream>
#include <thread>
#include <chrono>

CameraCapture::CameraCapture()
    : m_isRunning(false),
      m_isReady(false),
      m_cameraMode(CameraMode::STANDARD_CAMERA),
      m_width(640),
      m_height(480),
      m_fps(30) {
}

CameraCapture::~CameraCapture() {
    stopCapture();
    
    // Release camera resources
    if (m_isReady) {
        if (m_cameraMode == CameraMode::STANDARD_CAMERA) {
            m_camera.release();
        }
#ifdef USE_ZED_SDK
        else if (m_cameraMode == CameraMode::ZED_CAMERA) {
            m_zed.close();
        }
#endif
        m_isReady = false;
    }
}

bool CameraCapture::initStandardCamera(int cameraId, int width, int height, int fps) {
    // Stop any existing capture
    stopCapture();
    
    // Store settings
    m_width = width;
    m_height = height;
    m_fps = fps;
    m_cameraMode = CameraMode::STANDARD_CAMERA;
    
    // Open camera with specified ID
    m_camera.open(cameraId);
    if (!m_camera.isOpened()) {
        std::cerr << "Error: Could not open camera with ID " << cameraId << std::endl;
        m_isReady = false;
        return false;
    }
    
    // Set camera properties
    m_camera.set(cv::CAP_PROP_FRAME_WIDTH, width);
    m_camera.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    m_camera.set(cv::CAP_PROP_FPS, fps);
    
    // Check actual settings (may differ from requested due to camera limitations)
    int actualWidth = m_camera.get(cv::CAP_PROP_FRAME_WIDTH);
    int actualHeight = m_camera.get(cv::CAP_PROP_FRAME_HEIGHT);
    int actualFps = m_camera.get(cv::CAP_PROP_FPS);
    
    std::cout << "Camera opened with resolution: " << actualWidth << "x" << actualHeight
              << " @" << actualFps << "fps" << std::endl;
    
    // Update member variables with actual values
    m_width = actualWidth;
    m_height = actualHeight;
    m_fps = actualFps;
    
    m_isReady = true;
    std::cout << "Standard camera initialized successfully" << std::endl;
    return true;
}

#ifdef USE_ZED_SDK
bool CameraCapture::initZEDCamera(int width, int height, int fps, const std::string& depth_mode) {
    // Stop any existing capture
    stopCapture();
    
    // Store settings
    m_width = width;
    m_height = height;
    m_fps = fps;
    m_cameraMode = CameraMode::ZED_CAMERA;
    
    // Initialize ZED camera
    sl::InitParameters init_params;
    
    // Set camera resolution based on width and height
    if (width <= 672 && height <= 376) {
        init_params.camera_resolution = sl::RESOLUTION::VGA;
    } else if (width <= 1280 && height <= 720) {
        init_params.camera_resolution = sl::RESOLUTION::HD720;
    } else if (width <= 1920 && height <= 1080) {
        init_params.camera_resolution = sl::RESOLUTION::HD1080;
    } else if (width <= 2208 && height <= 1242) {
        init_params.camera_resolution = sl::RESOLUTION::HD2K;
    } else {
        std::cerr << "Invalid resolution: " << width << "x" << height 
                  << ". Using HD720 (1280x720) instead." << std::endl;
        init_params.camera_resolution = sl::RESOLUTION::HD720;
        m_width = 1280;
        m_height = 720;
    }
    
    // Set camera FPS
    init_params.camera_fps = fps;
    init_params.coordinate_units = sl::UNIT::METER; // Use meters for depth
    
    // Set depth mode
    if (depth_mode == "QUALITY") {
        init_params.depth_mode = sl::DEPTH_MODE::QUALITY;
    } else if (depth_mode == "ULTRA") {
        init_params.depth_mode = sl::DEPTH_MODE::ULTRA;
    } else if (depth_mode == "NEURAL") {
        init_params.depth_mode = sl::DEPTH_MODE::NEURAL;
    } else {
        init_params.depth_mode = sl::DEPTH_MODE::PERFORMANCE;
    }
    
    // Open the camera
    sl::ERROR_CODE err = m_zed.open(init_params);
    if (err != sl::ERROR_CODE::SUCCESS) {
        std::cerr << "Error opening ZED camera: " << sl::toString(err) << std::endl;
        m_isReady = false;
        return false;
    }
    
    // Get camera information
    sl::CameraInformation cam_info = m_zed.getCameraInformation();
    std::cout << "ZED camera opened with resolution: " << cam_info.camera_configuration.resolution.width 
              << "x" << cam_info.camera_configuration.resolution.height 
              << " @" << cam_info.camera_configuration.fps << "fps" << std::endl;
    
    // Update member variables with actual values
    m_width = cam_info.camera_configuration.resolution.width;
    m_height = cam_info.camera_configuration.resolution.height;
    m_fps = cam_info.camera_configuration.fps;
    
    // Initialize runtime parameters
    m_runtime.measure3D_reference_frame = sl::REFERENCE_FRAME::CAMERA;
    m_runtime.enable_fill_mode = true;
    m_runtime.confidence_threshold = 50; // Default confidence threshold (0-100)
    
    m_isReady = true;
    std::cout << "ZED camera initialized successfully" << std::endl;
    return true;
}

bool CameraCapture::initWithSVO(const std::string& svoFile) {
    // Stop any existing capture
    stopCapture();
    
    m_cameraMode = CameraMode::ZED_CAMERA;
    
    // Initialize ZED camera with SVO file
    sl::InitParameters init_params;
    init_params.input.setFromSVOFile(svoFile.c_str());
    init_params.coordinate_units = sl::UNIT::METER;
    init_params.depth_mode = sl::DEPTH_MODE::PERFORMANCE;
    
    // Open the camera
    sl::ERROR_CODE err = m_zed.open(init_params);
    if (err != sl::ERROR_CODE::SUCCESS) {
        std::cerr << "Error opening SVO file: " << sl::toString(err) << std::endl;
        m_isReady = false;
        return false;
    }
    
    // Get SVO information
    sl::CameraInformation cam_info = m_zed.getCameraInformation();
    m_width = cam_info.camera_configuration.resolution.width;
    m_height = cam_info.camera_configuration.resolution.height;
    m_fps = cam_info.camera_configuration.fps;
    
    std::cout << "ZED SVO file opened with resolution: " << m_width << "x" << m_height 
              << " @" << m_fps << "fps" << std::endl;
    
    // Initialize runtime parameters
    m_runtime.measure3D_reference_frame = sl::REFERENCE_FRAME::CAMERA;
    m_runtime.enable_fill_mode = true;
    m_runtime.confidence_threshold = 50; // Default confidence threshold
    
    m_isReady = true;
    std::cout << "ZED SVO initialized successfully" << std::endl;
    return true;
}

void CameraCapture::setDepthConfidenceThreshold(int confidenceThreshold) {
    if (m_cameraMode != CameraMode::ZED_CAMERA) {
        std::cerr << "Error: Depth confidence setting only available with ZED camera" << std::endl;
        return;
    }
    
    if (confidenceThreshold >= 0 && confidenceThreshold <= 100) {
        m_runtime.confidence_threshold = confidenceThreshold;
        std::cout << "Depth confidence threshold set to " << confidenceThreshold << std::endl;
    } else {
        std::cerr << "Invalid confidence threshold. Must be between 0 and 100." << std::endl;
    }
}
#endif

bool CameraCapture::initWithVideoFile(const std::string& videoFile) {
    // Stop any existing capture
    stopCapture();
    
    // Set camera mode
    m_cameraMode = CameraMode::STANDARD_CAMERA;
    
    // Open video file
    m_camera.open(videoFile);
    if (!m_camera.isOpened()) {
        std::cerr << "Error: Could not open video file: " << videoFile << std::endl;
        m_isReady = false;
        return false;
    }
    
    // Get video properties
    m_width = static_cast<int>(m_camera.get(cv::CAP_PROP_FRAME_WIDTH));
    m_height = static_cast<int>(m_camera.get(cv::CAP_PROP_FRAME_HEIGHT));
    m_fps = static_cast<int>(m_camera.get(cv::CAP_PROP_FPS));
    
    std::cout << "Video file opened with resolution: " << m_width << "x" << m_height
              << " @" << m_fps << "fps" << std::endl;
    
    m_isReady = true;
    std::cout << "Video initialized successfully" << std::endl;
    return true;
}

bool CameraCapture::getFrame(cv::Mat& frame, cv::Mat* depthMap) {
    if (!m_isReady) {
        return false;
    }
    
    // Lock to safely access the latest frames
    std::lock_guard<std::mutex> lock(m_frameMutex);
    
    if (m_latestFrame.empty()) {
        return false;
    }
    
    // Copy the latest color frame
    m_latestFrame.copyTo(frame);
    
#ifdef USE_ZED_SDK
    // Copy depth map if using ZED and depth map is requested
    if (m_cameraMode == CameraMode::ZED_CAMERA && 
        depthMap != nullptr && 
        !m_latestDepthFrame.empty()) {
        m_latestDepthFrame.copyTo(*depthMap);
    }
#endif
    
    return true;
}

bool CameraCapture::startCapture() {
    if (!m_isReady) {
        std::cerr << "Camera not initialized" << std::endl;
        return false;
    }
    
    if (m_isRunning) {
        std::cerr << "Capture already running" << std::endl;
        return true; // Already running is not an error
    }
    
    // Start the capture thread
    m_isRunning = true;
    m_captureThread = std::thread(&CameraCapture::captureThread, this);
    
    std::cout << "Camera capture started" << std::endl;
    return true;
}

void CameraCapture::stopCapture() {
    m_isRunning = false;
    
    // Wait for capture thread to finish
    if (m_captureThread.joinable()) {
        m_captureThread.join();
    }
    
    std::cout << "Camera capture stopped" << std::endl;
}

bool CameraCapture::isReady() const {
    return m_isReady;
}

int CameraCapture::getWidth() const {
    return m_width;
}

int CameraCapture::getHeight() const {
    return m_height;
}

CameraMode CameraCapture::getCameraMode() const {
    return m_cameraMode;
}

void CameraCapture::captureThread() {
    if (m_cameraMode == CameraMode::STANDARD_CAMERA) {
        cv::Mat frame;
        
        while (m_isRunning) {
            // Grab a new frame from camera
            bool success = m_camera.read(frame);
            
            if (success && !frame.empty()) {
                // Update the latest frame
                std::lock_guard<std::mutex> lock(m_frameMutex);
                frame.copyTo(m_latestFrame);
            } else {
                // Handle end of video file or camera disconnection
                if (m_camera.get(cv::CAP_PROP_POS_FRAMES) == m_camera.get(cv::CAP_PROP_FRAME_COUNT)) {
                    // End of video file, rewind to beginning for looping
                    m_camera.set(cv::CAP_PROP_POS_FRAMES, 0);
                } else {
                    std::cerr << "Error: Failed to capture frame" << std::endl;
                    // Add a short delay before retrying
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
            }
        }
    }
#ifdef USE_ZED_SDK
    else if (m_cameraMode == CameraMode::ZED_CAMERA) {
        // ZED SDK containers for images
        sl::Mat leftImage, depthImage;
        
        while (m_isRunning) {
            // Grab a new frame from ZED camera
            sl::ERROR_CODE err = m_zed.grab(m_runtime);
            
            if (err == sl::ERROR_CODE::SUCCESS) {
                // Retrieve left image
                m_zed.retrieveImage(leftImage, sl::VIEW::LEFT);
                
                // Retrieve depth map
                m_zed.retrieveImage(depthImage, sl::VIEW::DEPTH);
                
                // Convert to OpenCV format
                cv::Mat leftCV = zedImageToCV(leftImage);
                cv::Mat depthCV = zedImageToCV(depthImage);
                
                // Update the latest frames
                {
                    std::lock_guard<std::mutex> lock(m_frameMutex);
                    leftCV.copyTo(m_latestFrame);
                    depthCV.copyTo(m_latestDepthFrame);
                }
            } else if (err == sl::ERROR_CODE::END_OF_SVOFILE_REACHED) {
                // If using an SVO file and reached the end, reset to start
                m_zed.setSVOPosition(0);
            } else {
                std::cerr << "ZED grab failed: " << sl::toString(err) << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
    }
#endif
}

#ifdef USE_ZED_SDK
cv::Mat CameraCapture::zedImageToCV(const sl::Mat& zedImage) {
    // Get ZED image resolution and data
    int width = zedImage.getWidth();
    int height = zedImage.getHeight();
    
    // Create OpenCV Mat from ZED Mat data
    cv::Mat cvImage(height, width, CV_8UC4);
    memcpy(cvImage.data, zedImage.getPtr<sl::uchar1>(), width * height * 4);
    
    // Convert BGRA to BGR (ZED SDK uses BGRA, OpenCV typically uses BGR)
    cv::Mat bgrImage;
    cv::cvtColor(cvImage, bgrImage, cv::COLOR_BGRA2BGR);
    
    return bgrImage;
}
#endif 