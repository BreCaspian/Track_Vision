#include "TrackVision.h"
#include <iostream>
#include <thread>
#include <chrono>

TrackVision::TrackVision(const std::string& configFile)
    : m_isRunning(false),
      m_showDebugView(true),
      m_frameCounter(0),
      m_lastDeviation(0.0f),
      m_configFile(configFile),
      m_currentMode(FOLLOW_LINE),
      m_avoidStep(0),
      m_lineDetectedCounter(0),
      m_preferredAvoidanceDirection("right"),
      m_turnTimeMs(500),
      m_forwardTimeMs(800),
      m_checkIntervalMs(100),
      m_minLineDetectionFrames(3),
      m_obstacleDistanceThreshold(1.0f),
      m_clearPathThreshold(1.5f)
{
}

bool TrackVision::init()
{
    try {
        // Load configuration
        if (!m_config.loadFromFile(m_configFile)) {
            std::cerr << "Failed to load configuration from " << m_configFile << std::endl;
            return false;
        }

        // Get camera settings
        int cameraId = m_config.getInt("camera.id", 0);
        int width = m_config.getInt("camera.width", 640);
        int height = m_config.getInt("camera.height", 480);
        int fps = m_config.getInt("camera.fps", 30);
        std::string cameraMode = m_config.getString("camera.mode", "standard"); // "standard" or "zed"

        // Initialize camera based on mode
        if (cameraMode == "zed") {
#ifdef USE_ZED_SDK
            std::string depthMode = m_config.getString("camera.depth_mode", "PERFORMANCE");
            if (!m_camera.initZEDCamera(width, height, fps, depthMode)) {
                std::cerr << "Failed to initialize ZED camera" << std::endl;
                return false;
            }
#else
            std::cerr << "ZED camera mode selected but USE_ZED_SDK not defined" << std::endl;
            return false;
#endif
        } else {
            if (!m_camera.initStandardCamera(cameraId, width, height, fps)) {
                std::cerr << "Failed to initialize standard camera id " << cameraId << std::endl;
                return false;
            }
        }

        // Get line detector settings
        int threshold = m_config.getInt("detector.threshold", 127);
        float roiYStart = m_config.getFloat("detector.roi_y_start", 0.6f);
        float roiHeight = m_config.getFloat("detector.roi_height", 0.3f);
        float roiWidth = m_config.getFloat("detector.roi_width", 0.5f);
        int cannyLow = m_config.getInt("detector.canny_low", 50);
        int cannyHigh = m_config.getInt("detector.canny_high", 150);
        int houghThreshold = m_config.getInt("detector.hough_threshold", 50);
        int minLineLength = m_config.getInt("detector.min_line_length", 30);
        int maxLineGap = m_config.getInt("detector.max_line_gap", 10);
        int obstacleThreshold = m_config.getInt("detector.obstacle_contour_threshold", 5);

        // Initialize line detector
        if (!m_lineDetector.init(threshold, roiYStart, roiHeight, roiWidth,
                               cannyLow, cannyHigh, houghThreshold,
                               minLineLength, maxLineGap, obstacleThreshold)) {
            std::cerr << "Failed to initialize line detector" << std::endl;
            return false;
        }

        // Get serial communication settings
        std::string serialPort = m_config.getString("serial.port", "COM3");
        int baudRate = m_config.getInt("serial.baudrate", 9600);

        // Initialize serial communication
        if (!m_serialComm.init(serialPort, baudRate)) {
            std::cerr << "Failed to initialize serial communication on " << serialPort << std::endl;
            return false;
        }

        // Get avoidance settings
        m_preferredAvoidanceDirection = m_config.getString("avoidance.preferred_direction", "right");
        m_turnTimeMs = m_config.getInt("avoidance.turn_time_ms", 500);
        m_forwardTimeMs = m_config.getInt("avoidance.forward_time_ms", 800);
        m_checkIntervalMs = m_config.getInt("avoidance.check_interval_ms", 100);
        m_minLineDetectionFrames = m_config.getInt("avoidance.min_line_detection_frames", 3);
        m_obstacleDistanceThreshold = m_config.getFloat("avoidance.obstacle_distance_threshold", 1.0f);
        m_clearPathThreshold = m_config.getFloat("avoidance.clear_path_threshold", 1.5f);

        // Get debug settings
        m_showDebugView = m_config.getBool("debug.show_view", true);

        std::cout << "TrackVision initialized successfully" << std::endl;
        return true;
    }
    catch (const std::exception& e) {
        std::cerr << "Exception during initialization: " << e.what() << std::endl;
        return false;
    }
}

void TrackVision::run()
{
    if (!m_camera.isReady()) {
        std::cerr << "Camera not initialized" << std::endl;
        return;
    }

    if (!m_serialComm.isReady()) {
        std::cerr << "Serial communication not initialized" << std::endl;
        return;
    }

    // Start camera capture thread
    if (!m_camera.startCapture()) {
        std::cerr << "Failed to start camera capture" << std::endl;
        return;
    }

    // Start serial communication thread
    m_serialComm.start();

    m_isRunning = true;
    m_currentMode = FOLLOW_LINE;
    m_avoidStep = 0;
    m_lineDetectedCounter = 0;

    cv::Mat frame, depthMap, processedFrame;

    while (m_isRunning) {
        // Capture frame and depth map if available
        if (m_camera.getCameraMode() == CameraMode::ZED_CAMERA) {
            if (!m_camera.getFrame(frame, &depthMap)) {
                std::cerr << "Failed to capture frame and depth map" << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
        } else {
            if (!m_camera.getFrame(frame)) {
                std::cerr << "Failed to capture frame" << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            // No depth map available with standard camera
            depthMap = cv::Mat();
        }

        if (frame.empty()) {
            std::cerr << "Empty frame received" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        // Process frame for line detection
        processedFrame = processFrame(frame);

        // Update state machine with frame and depth information
        updateStateMachine(frame, depthMap);

        // Display processed frame if debug view is enabled
        if (m_showDebugView) {
            displayFrame(frame, processedFrame, m_lastDeviation, m_obstacleResult, m_currentMode);

            // Check for key press
            int key = cv::waitKey(1);
            if (key == 27) { // ESC key
                stop();
            }
        }

        m_frameCounter++;
    }

    // Stop camera capture
    m_camera.stopCapture();

    // Stop serial communication
    m_serialComm.stop();
}

void TrackVision::stop()
{
    m_isRunning = false;

    // Send stop command
    m_serialComm.sendCommand(CommandType::STOP);

    if (m_showDebugView) {
        cv::destroyAllWindows();
    }
}

bool TrackVision::isRunning() const
{
    return m_isRunning;
}

cv::Mat TrackVision::processFrame(const cv::Mat& frame)
{
    // Process frame with line detector
    cv::Mat processedFrame = m_lineDetector.processImage(frame, m_lineResult);
    
    // Store last deviation
    m_lastDeviation = m_lineResult.deviation;
    
    return processedFrame;
}

bool TrackVision::detectObstacleByContour(const cv::Mat& frame)
{
    return m_lineDetector.detectObstacle(frame);
}

bool TrackVision::detectObstacleByDepth(const cv::Mat& frame, const cv::Mat& depthMap, ObstacleDetectionResult& result)
{
    if (depthMap.empty()) {
        return false;
    }
    
    int width = frame.cols;
    int height = frame.rows;
    
    // Define ROIs for center, left, and right regions
    int centerWidth = width / 4;
    int sideWidth = width / 5;
    int roiHeight = height / 4;
    int roiY = height * 2 / 3; // Lower third of the image
    
    // Center ROI
    cv::Rect centerROI(width/2 - centerWidth/2, roiY, centerWidth, roiHeight);
    
    // Left and right ROIs
    cv::Rect leftROI(width/6 - sideWidth/2, roiY, sideWidth, roiHeight);
    cv::Rect rightROI(width*5/6 - sideWidth/2, roiY, sideWidth, roiHeight);
    
    // Calculate average depths
    float centerDepth = calculateAverageDepth(depthMap, centerROI);
    float leftDepth = calculateAverageDepth(depthMap, leftROI);
    float rightDepth = calculateAverageDepth(depthMap, rightROI);
    
    // Fill result structure
    result.obstacleDetected = (centerDepth > 0.0f && centerDepth < m_obstacleDistanceThreshold);
    result.centerDistance = centerDepth;
    result.leftDistance = leftDepth;
    result.rightDistance = rightDepth;
    result.leftClear = (leftDepth > m_clearPathThreshold);
    result.rightClear = (rightDepth > m_clearPathThreshold);
    result.centerROI = centerROI;
    result.leftROI = leftROI;
    result.rightROI = rightROI;
    
    return result.obstacleDetected;
}

float TrackVision::calculateAverageDepth(const cv::Mat& depthMap, const cv::Rect& roi)
{
    if (depthMap.empty() || roi.x < 0 || roi.y < 0 || 
        roi.x + roi.width > depthMap.cols || roi.y + roi.height > depthMap.rows) {
        return -1.0f;
    }
    
    cv::Mat roiDepth = depthMap(roi);
    
    // Count valid depth values (non-zero)
    int validCount = 0;
    float sumDepth = 0.0f;
    
    for (int y = 0; y < roiDepth.rows; y++) {
        for (int x = 0; x < roiDepth.cols; x++) {
            float depth = roiDepth.at<float>(y, x);
            if (depth > 0.0f) { // Valid depth value
                sumDepth += depth;
                validCount++;
            }
        }
    }
    
    if (validCount > 0) {
        return sumDepth / validCount;
    } else {
        return -1.0f; // No valid depth values
    }
}

void TrackVision::updateStateMachine(const cv::Mat& frame, const cv::Mat& depthMap)
{
    auto currentTime = std::chrono::steady_clock::now();
    auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
        currentTime - m_lastStateChangeTime).count();

    // 准备视觉数据结构
    VisionData visionData;
    visionData.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        currentTime.time_since_epoch()).count() / 1000.0f;
    visionData.line_offset = m_lastDeviation;
    visionData.line_found = m_lineResult.lineDetected ? 1 : 0;
    
    // Initialize obstacle detection result
    m_obstacleResult.obstacleDetected = false;
    
    // Detect obstacles based on available data
    bool obstacleDetected = false;
    if (!depthMap.empty()) {
        // Use depth-based obstacle detection if depth map is available
        obstacleDetected = detectObstacleByDepth(frame, depthMap, m_obstacleResult);
        
        // 更新视觉数据结构中的障碍物信息
        visionData.obstacle_detected = obstacleDetected ? 1 : 0;
    } else {
        // Fall back to contour-based detection
        obstacleDetected = detectObstacleByContour(frame);
        
        // 更新视觉数据结构中的障碍物信息
        visionData.obstacle_detected = obstacleDetected ? 1 : 0;
        
        // Set basic obstacle result for display
        m_obstacleResult.obstacleDetected = obstacleDetected;
        m_obstacleResult.centerDistance = obstacleDetected ? 0.5f : 2.0f; // Dummy values
        m_obstacleResult.leftDistance = 1.0f;
        m_obstacleResult.rightDistance = 1.0f;
        m_obstacleResult.leftClear = !obstacleDetected;
        m_obstacleResult.rightClear = !obstacleDetected;
        
        // Create ROIs for visualization
        int width = frame.cols;
        int height = frame.rows;
        m_obstacleResult.centerROI = cv::Rect(width/2 - width/8, height*2/3, width/4, height/4);
        m_obstacleResult.leftROI = cv::Rect(width/6 - width/10, height*2/3, width/5, height/4);
        m_obstacleResult.rightROI = cv::Rect(width*5/6 - width/10, height*2/3, width/5, height/4);
    }

    // State machine
    switch (m_currentMode) {
        case FOLLOW_LINE:
            if (obstacleDetected) {
                // 更新视觉数据，表示发现障碍物，进入避障模式
                visionData.obstacle_detected = 1;
                
                m_currentMode = OBSTACLE_DETECTED;
                m_lastStateChangeTime = currentTime;
                std::cout << "State: OBSTACLE_DETECTED" << std::endl;
            } else {
                // 正常循迹
                visionData.obstacle_detected = 0;
                visionData.avoid_direction = AVOID_NONE;
            }
            break;

        case OBSTACLE_DETECTED:
            // Determine avoidance direction based on depth information
            if (!depthMap.empty() && m_obstacleResult.leftClear && m_obstacleResult.rightClear) {
                // Both sides clear, use preferred direction or choose based on more space
                if (m_obstacleResult.leftDistance > m_obstacleResult.rightDistance + 0.5f) {
                    m_currentMode = AVOIDING_LEFT;
                    visionData.avoid_direction = AVOID_LEFT;
                    std::cout << "State: AVOIDING_LEFT (more space on left)" << std::endl;
                } else {
                    m_currentMode = AVOIDING_RIGHT;
                    visionData.avoid_direction = AVOID_RIGHT;
                    std::cout << "State: AVOIDING_RIGHT (more space on right or equal)" << std::endl;
                }
            } else if (!depthMap.empty() && m_obstacleResult.leftClear) {
                // Only left side clear
                m_currentMode = AVOIDING_LEFT;
                visionData.avoid_direction = AVOID_LEFT;
                std::cout << "State: AVOIDING_LEFT (only left clear)" << std::endl;
            } else if (!depthMap.empty() && m_obstacleResult.rightClear) {
                // Only right side clear
                m_currentMode = AVOIDING_RIGHT;
                visionData.avoid_direction = AVOID_RIGHT;
                std::cout << "State: AVOIDING_RIGHT (only right clear)" << std::endl;
            } else {
                // Use preferred direction if depth info not available or neither side clear
                if (m_preferredAvoidanceDirection == "left") {
                    m_currentMode = AVOIDING_LEFT;
                    visionData.avoid_direction = AVOID_LEFT;
                    std::cout << "State: AVOIDING_LEFT (preferred)" << std::endl;
                } else {
                    m_currentMode = AVOIDING_RIGHT;
                    visionData.avoid_direction = AVOID_RIGHT;
                    std::cout << "State: AVOIDING_RIGHT (preferred)" << std::endl;
                }
            }
            m_avoidStep = 0;
            m_lastStateChangeTime = currentTime;
            break;

        case AVOIDING_LEFT:
            // 保持避障方向信息
            visionData.obstacle_detected = 1;
            visionData.avoid_direction = AVOID_LEFT;
            
            // 下面的状态机逻辑保持不变，但不再需要单独发送命令
            if (m_avoidStep == 0 && elapsedMs >= m_turnTimeMs) {
                m_avoidStep = 1;
                m_lastStateChangeTime = currentTime;
                std::cout << "Avoidance Step 1: Moving Forward" << std::endl;
            }
            else if (m_avoidStep == 1 && elapsedMs >= m_forwardTimeMs) {
                m_avoidStep = 2;
                m_lastStateChangeTime = currentTime;
                std::cout << "Avoidance Step 2: Turning back" << std::endl;
            }
            else if (m_avoidStep == 2 && elapsedMs >= m_turnTimeMs) {
                m_avoidStep = 3;
                m_lastStateChangeTime = currentTime;
                std::cout << "Avoidance Step 3: Moving Forward Again" << std::endl;
            }
            else if (m_avoidStep == 3) {
                // Check periodically if we've reacquired the line
                if (elapsedMs >= m_checkIntervalMs) {
                    if (m_lineResult.lineDetected && !obstacleDetected) {
                        m_lineDetectedCounter++;
                        if (m_lineDetectedCounter >= m_minLineDetectionFrames) {
                            m_currentMode = LINE_REACQUIRED;
                            m_lastStateChangeTime = currentTime;
                            std::cout << "State: LINE_REACQUIRED" << std::endl;
                            m_lineDetectedCounter = 0;
                        } else {
                            // Reset timer to check again in a bit
                            m_lastStateChangeTime = currentTime;
                        }
                    } else {
                        // Reset counter if line not detected
                        m_lineDetectedCounter = 0;
                        // Reset timer to check again in a bit
                        m_lastStateChangeTime = currentTime;
                    }
                }
            }
            break;

        case AVOIDING_RIGHT:
            // 保持避障方向信息
            visionData.obstacle_detected = 1;
            visionData.avoid_direction = AVOID_RIGHT;
            
            // 下面的状态机逻辑保持不变，但不再需要单独发送命令
            if (m_avoidStep == 0 && elapsedMs >= m_turnTimeMs) {
                m_avoidStep = 1;
                m_lastStateChangeTime = currentTime;
                std::cout << "Avoidance Step 1: Moving Forward" << std::endl;
            }
            else if (m_avoidStep == 1 && elapsedMs >= m_forwardTimeMs) {
                m_avoidStep = 2;
                m_lastStateChangeTime = currentTime;
                std::cout << "Avoidance Step 2: Turning back" << std::endl;
            }
            else if (m_avoidStep == 2 && elapsedMs >= m_turnTimeMs) {
                m_avoidStep = 3;
                m_lastStateChangeTime = currentTime;
                std::cout << "Avoidance Step 3: Moving Forward Again" << std::endl;
            }
            else if (m_avoidStep == 3) {
                // Check periodically if we've reacquired the line
                if (elapsedMs >= m_checkIntervalMs) {
                    if (m_lineResult.lineDetected && !obstacleDetected) {
                        m_lineDetectedCounter++;
                        if (m_lineDetectedCounter >= m_minLineDetectionFrames) {
                            m_currentMode = LINE_REACQUIRED;
                            m_lastStateChangeTime = currentTime;
                            std::cout << "State: LINE_REACQUIRED" << std::endl;
                            m_lineDetectedCounter = 0;
                        } else {
                            // Reset timer to check again in a bit
                            m_lastStateChangeTime = currentTime;
                        }
                    } else {
                        // Reset counter if line not detected
                        m_lineDetectedCounter = 0;
                        // Reset timer to check again in a bit
                        m_lastStateChangeTime = currentTime;
                    }
                }
            }
            break;

        case LINE_REACQUIRED:
            // 返回到寻线状态
            visionData.obstacle_detected = 0;
            visionData.avoid_direction = AVOID_NONE;
            
            // Wait a moment and then return to normal line following
            if (elapsedMs >= 500) {
                m_currentMode = FOLLOW_LINE;
                std::cout << "State: Back to FOLLOW_LINE" << std::endl;
            }
            break;
    }
    
    // 发送视觉数据结构
    m_serialComm.sendVisionData(visionData);
}

void TrackVision::executeTimedCommand(const std::string& command, int delayMs)
{
    // 创建视觉数据包
    VisionData data;
    data.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count() / 1000.0f;
    
    // 根据命令设置数据包
    if (command == "CMD:FORWARD") {
        data.obstacle_detected = 0;
        data.avoid_direction = AVOID_NONE;
    } else if (command == "CMD:STOP") {
        data.obstacle_detected = 1;
        data.avoid_direction = AVOID_NONE;
    } else if (command == "CMD:TURN_LEFT") {
        data.obstacle_detected = 1;
        data.avoid_direction = AVOID_LEFT;
    } else if (command == "CMD:TURN_RIGHT") {
        data.obstacle_detected = 1;
        data.avoid_direction = AVOID_RIGHT;
    }
    
    // 发送数据包
    m_serialComm.sendVisionData(data);
    
    // 等待指定的延时
    std::this_thread::sleep_for(std::chrono::milliseconds(delayMs));
}

void TrackVision::displayFrame(const cv::Mat& originalFrame, 
                             const cv::Mat& processedFrame, 
                             float deviation, 
                             const ObstacleDetectionResult& obstacleResult,
                             TrackingMode mode)
{
    cv::Mat display = processedFrame.clone();
    
    // Add state information
    std::string stateText;
    switch (mode) {
        case FOLLOW_LINE:
            stateText = "State: FOLLOW_LINE";
            break;
        case OBSTACLE_DETECTED:
            stateText = "State: OBSTACLE_DETECTED";
            break;
        case AVOIDING_LEFT:
            stateText = "State: AVOIDING_LEFT";
            break;
        case AVOIDING_RIGHT:
            stateText = "State: AVOIDING_RIGHT";
            break;
        case LINE_REACQUIRED:
            stateText = "State: LINE_REACQUIRED";
            break;
    }
    
    cv::putText(display, stateText, cv::Point(10, 30), 
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
    
    // Add deviation information
    std::string deviationText = "Deviation: " + std::to_string(deviation);
    cv::putText(display, deviationText, cv::Point(10, 60), 
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 0), 2);
    
    // Add obstacle information
    if (obstacleResult.obstacleDetected) {
        std::string obstacleText = "OBSTACLE DETECTED";
        cv::putText(display, obstacleText, cv::Point(10, 90), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
    }
    
    // Draw ROIs on the original frame
    cv::Mat originalWithOverlay = originalFrame.clone();
    
    // Draw center ROI
    cv::Scalar centerColor = obstacleResult.obstacleDetected ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0);
    cv::rectangle(originalWithOverlay, obstacleResult.centerROI, centerColor, 2);
    
    // Draw left ROI
    cv::Scalar leftColor = obstacleResult.leftClear ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
    cv::rectangle(originalWithOverlay, obstacleResult.leftROI, leftColor, 2);
    
    // Draw right ROI
    cv::Scalar rightColor = obstacleResult.rightClear ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
    cv::rectangle(originalWithOverlay, obstacleResult.rightROI, rightColor, 2);
    
    // Add depth information
    if (obstacleResult.centerDistance > 0) {
        std::string centerDepthText = "C: " + std::to_string(obstacleResult.centerDistance).substr(0, 4) + "m";
        cv::putText(originalWithOverlay, centerDepthText, 
                    cv::Point(obstacleResult.centerROI.x, obstacleResult.centerROI.y - 5), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, centerColor, 2);
    }
    
    if (obstacleResult.leftDistance > 0) {
        std::string leftDepthText = "L: " + std::to_string(obstacleResult.leftDistance).substr(0, 4) + "m";
        cv::putText(originalWithOverlay, leftDepthText, 
                    cv::Point(obstacleResult.leftROI.x, obstacleResult.leftROI.y - 5), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, leftColor, 2);
    }
    
    if (obstacleResult.rightDistance > 0) {
        std::string rightDepthText = "R: " + std::to_string(obstacleResult.rightDistance).substr(0, 4) + "m";
        cv::putText(originalWithOverlay, rightDepthText, 
                    cv::Point(obstacleResult.rightROI.x, obstacleResult.rightROI.y - 5), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, rightColor, 2);
    }
    
    // Display the frames
    cv::imshow("Track Vision", display);
    cv::imshow("Original with ROIs", originalWithOverlay);
} 