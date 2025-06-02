#ifndef LINE_DETECTOR_H
#define LINE_DETECTOR_H

#include <opencv2/opencv.hpp>
#include <vector>

/**
 * @brief Line detection result structure
 */
struct LineDetectionResult {
    float deviation;           // Deviation from center (-1.0 to 1.0)
    cv::Point centerPoint;     // Center point of the detected line
    bool lineDetected;         // Whether any line was detected
    std::vector<cv::Point> linePoints; // Points along the detected line
    std::vector<cv::Vec4i> detectedLines; // Raw detected Hough lines
};

/**
 * @brief Class for detecting blind track lines and obstacles in images
 */
class LineDetector {
public:
    /**
     * @brief Construct a new Line Detector object
     */
    LineDetector();

    /**
     * @brief Initialize the line detector with parameters
     * 
     * @param thresholdValue Threshold value for binary conversion
     * @param roiYStart Y-coordinate start for region of interest
     * @param roiHeight Height of ROI as proportion of image height
     * @param roiWidth Width of ROI as proportion of image width
     * @param cannyLow Lower threshold for Canny edge detection
     * @param cannyHigh Higher threshold for Canny edge detection
     * @param houghThreshold Threshold for Hough line detection
     * @param minLineLength Minimum length of line to detect
     * @param maxLineGap Maximum gap in line to consider as single line
     * @param obstacleThreshold Minimum number of contours for obstacle detection
     * @return true if initialization successful
     * @return false if initialization failed
     */
    bool init(int thresholdValue = 127,
              float roiYStart = 0.6,
              float roiHeight = 0.3,
              float roiWidth = 0.5,
              int cannyLow = 50,
              int cannyHigh = 150,
              int houghThreshold = 50,
              int minLineLength = 30,
              int maxLineGap = 10,
              int obstacleThreshold = 5);

    /**
     * @brief Process an image and detect lines
     * 
     * @param image Input image to process
     * @param result Output detection result
     * @return cv::Mat Processed image for visualization
     */
    cv::Mat processImage(const cv::Mat& image, LineDetectionResult& result);

    /**
     * @brief Detect if there's an obstacle in the ROI region
     * 
     * @param image Input image to check
     * @return true if obstacle detected
     * @return false if no obstacle
     */
    bool detectObstacle(const cv::Mat& image);

    /**
     * @brief Set threshold value for binary conversion
     * 
     * @param threshold New threshold value
     */
    void setThreshold(int threshold);

    /**
     * @brief Set ROI (Region of Interest) parameters
     * 
     * @param roiYStart Y-coordinate start for ROI as proportion (0.0-1.0)
     * @param roiHeight Height of ROI as proportion of image height
     * @param roiWidth Width of ROI as proportion of image width
     */
    void setROI(float roiYStart, float roiHeight, float roiWidth);

    /**
     * @brief Get the current ROI mask
     * 
     * @return cv::Mat Current ROI mask
     */
    cv::Mat getROIMask() const;

    /**
     * @brief Set Canny edge detection parameters
     * 
     * @param lowThreshold Lower threshold for Canny
     * @param highThreshold Higher threshold for Canny
     */
    void setCannyParameters(int lowThreshold, int highThreshold);

    /**
     * @brief Set Hough transform parameters
     * 
     * @param threshold Threshold for Hough line detection
     * @param minLineLength Minimum length of line to detect
     * @param maxLineGap Maximum gap in line to consider as single line
     */
    void setHoughParameters(int threshold, int minLineLength, int maxLineGap);

private:
    /**
     * @brief Apply preprocessing to the image (grayscale, blur, threshold)
     * 
     * @param image Input image
     * @return cv::Mat Preprocessed image
     */
    cv::Mat preprocess(const cv::Mat& image);

    /**
     * @brief Extract the region of interest from the image
     * 
     * @param image Input image
     * @return cv::Mat Image with only ROI visible
     */
    cv::Mat extractROI(const cv::Mat& image);

    /**
     * @brief Find lines using HoughLinesP
     * 
     * @param processedImage Edge-detected image
     * @return std::vector<cv::Vec4i> Detected line segments
     */
    std::vector<cv::Vec4i> findLines(const cv::Mat& processedImage);

    /**
     * @brief Calculate deviation from the image center
     * 
     * @param lines Detected lines from Hough transform
     * @param imageWidth Width of the image
     * @return float Normalized deviation (-1.0 to 1.0)
     */
    float calculateDeviation(const std::vector<cv::Vec4i>& lines, int imageWidth);

    int m_thresholdValue;      // Threshold for binary conversion
    float m_roiYStart;         // Y-coordinate start for ROI as proportion
    float m_roiHeight;         // Height of ROI as proportion of image height
    float m_roiWidth;          // Width of ROI as proportion of image width
    int m_cannyLowThreshold;   // Canny edge detection low threshold
    int m_cannyHighThreshold;  // Canny edge detection high threshold
    int m_houghThreshold;      // Hough transform threshold
    int m_minLineLength;       // Minimum length of line to detect
    int m_maxLineGap;          // Maximum gap in line to consider as single line
    int m_obstacleThreshold;   // Minimum number of contours for obstacle detection
    cv::Mat m_roiMask;         // ROI mask for visualization
};

#endif // LINE_DETECTOR_H 