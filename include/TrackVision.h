#ifndef TRACK_VISION_H
#define TRACK_VISION_H

#include <opencv2/opencv.hpp>
#include <string>
#include <chrono>
#include "LineDetector.h"
#include "SerialCommunicator.h"
#include "ConfigManager.h"
#include "CameraCapture.h"
#include "CommProtocol.h"

// 系统状态枚举
enum TrackingMode {
    FOLLOW_LINE,        // 正常循迹模式
    OBSTACLE_DETECTED,  // 发现障碍物
    AVOIDING_LEFT,      // 向左避障
    AVOIDING_RIGHT,     // 向右避障
    LINE_REACQUIRED     // 重新获取到循迹线
};

/**
 * @brief Obstacle detection result structure
 */
struct ObstacleDetectionResult {
    bool obstacleDetected;     // 是否检测到障碍物
    float centerDistance;      // 中心区域平均深度 (meters)
    float leftDistance;        // 左侧区域平均深度
    float rightDistance;       // 右侧区域平均深度
    bool leftClear;            // 左侧是否可通行
    bool rightClear;           // 右侧是否可通行
    cv::Rect centerROI;        // 中心检测区域
    cv::Rect leftROI;          // 左侧检测区域
    cv::Rect rightROI;         // 右侧检测区域
};

/**
 * @brief Main class for the blind track vision system
 * 
 * Handles the overall process of capturing images,
 * detecting lines and obstacles, and communicating with the control system.
 * Uses struct-based binary protocol for one-way communication (vision to control only).
 */
class TrackVision {
public:
    /**
     * @brief Construct a new Track Vision object
     * 
     * @param configFile Path to the configuration file
     */
    TrackVision(const std::string& configFile = "config/vision_config.json");
    
    /**
     * @brief Initialize the vision system
     * 
     * @return true if initialization successful
     * @return false if initialization failed
     */
    bool init();
    
    /**
     * @brief Run the main processing loop
     */
    void run();
    
    /**
     * @brief Stop the processing and clean up resources
     */
    void stop();

    /**
     * @brief Check if the system is currently running
     * 
     * @return true if running
     * @return false if not running
     */
    bool isRunning() const;

private:
    /**
     * @brief Process a single frame
     * 
     * @param frame Input frame to process
     * @return cv::Mat Processed frame for visualization
     */
    cv::Mat processFrame(const cv::Mat& frame);

    /**
     * @brief Display the processed frame with overlay information
     * 
     * @param originalFrame Original input frame
     * @param processedFrame Processed frame with detections
     * @param deviation Calculated deviation from center
     * @param obstacleResult Obstacle detection result
     * @param mode Current tracking mode
     */
    void displayFrame(const cv::Mat& originalFrame, 
                     const cv::Mat& processedFrame, 
                     float deviation, 
                     const ObstacleDetectionResult& obstacleResult,
                     TrackingMode mode);
    
    /**
     * @brief Detect if there's an obstacle in the path using contour analysis
     * 
     * @param frame Input frame to check
     * @return true if obstacle detected
     * @return false if no obstacle
     */
    bool detectObstacleByContour(const cv::Mat& frame);
    
    /**
     * @brief Detect if there's an obstacle in the path using depth information
     * 
     * @param frame Input color frame
     * @param depthMap Input depth map from ZED camera
     * @param result Output detection result
     * @return true if obstacle detected
     * @return false if no obstacle
     */
    bool detectObstacleByDepth(const cv::Mat& frame, const cv::Mat& depthMap, ObstacleDetectionResult& result);
    
    /**
     * @brief Calculate average depth in a region of interest
     * 
     * @param depthMap Input depth map
     * @param roi Region of interest
     * @return float Average depth in meters, -1.0 if invalid
     */
    float calculateAverageDepth(const cv::Mat& depthMap, const cv::Rect& roi);
    
    /**
     * @brief Handle state machine transitions
     * 
     * @param frame Current frame for analysis
     * @param depthMap Current depth map (if available)
     */
    void updateStateMachine(const cv::Mat& frame, const cv::Mat& depthMap);
    
    /**
     * @brief Execute a timed command with delay
     * 
     * @param command Command to send
     * @param delayMs Milliseconds to wait after sending
     */
    void executeTimedCommand(const std::string& command, int delayMs);

    // Components
    CameraCapture m_camera;
    LineDetector m_lineDetector;
    SerialCommunicator m_serialComm;
    ConfigManager m_config;
    
    // State variables
    bool m_isRunning;
    bool m_showDebugView;
    int m_frameCounter;
    float m_lastDeviation;
    std::string m_configFile;
    TrackingMode m_currentMode;
    int m_avoidStep;
    int m_lineDetectedCounter;  // Counter for consecutive frames with line detected
    std::chrono::time_point<std::chrono::steady_clock> m_lastStateChangeTime;
    
    // Configuration parameters
    std::string m_preferredAvoidanceDirection; // Default direction if both sides equal
    int m_turnTimeMs;
    int m_forwardTimeMs;
    int m_checkIntervalMs;
    int m_minLineDetectionFrames;  // Minimum frames to confirm line reacquired
    float m_obstacleDistanceThreshold;  // Distance in meters to detect obstacle
    float m_clearPathThreshold;         // Distance in meters to consider path clear
    
    // Detection results
    LineDetectionResult m_lineResult;
    ObstacleDetectionResult m_obstacleResult;
};

#endif // TRACK_VISION_H 