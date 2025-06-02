#include "LineDetector.h"
#include <iostream>

LineDetector::LineDetector()
    : m_thresholdValue(127),
      m_roiYStart(0.6),
      m_roiHeight(0.3),
      m_roiWidth(0.5),
      m_cannyLowThreshold(50),
      m_cannyHighThreshold(150),
      m_houghThreshold(50),
      m_minLineLength(30),
      m_maxLineGap(10),
      m_obstacleThreshold(5) {
}

bool LineDetector::init(int thresholdValue, float roiYStart, float roiHeight, float roiWidth, 
                        int cannyLow, int cannyHigh, int houghThreshold, 
                        int minLineLength, int maxLineGap, int obstacleThreshold) {
    m_thresholdValue = thresholdValue;
    m_roiYStart = roiYStart;
    m_roiHeight = roiHeight;
    m_roiWidth = roiWidth;
    m_cannyLowThreshold = cannyLow;
    m_cannyHighThreshold = cannyHigh;
    m_houghThreshold = houghThreshold;
    m_minLineLength = minLineLength;
    m_maxLineGap = maxLineGap;
    m_obstacleThreshold = obstacleThreshold;
    
    return true;
}

cv::Mat LineDetector::processImage(const cv::Mat& image, LineDetectionResult& result) {
    if (image.empty()) {
        result.lineDetected = false;
        result.deviation = 0.0f;
        return image.clone();
    }

    // Initialize result
    result.lineDetected = false;
    result.deviation = 0.0f;
    result.centerPoint = cv::Point(image.cols / 2, image.rows / 2);
    result.linePoints.clear();
    result.detectedLines.clear();

    // Process the image
    cv::Mat processed = preprocess(image);
    cv::Mat roi = extractROI(processed);
    
    // Create a color version for visualization
    cv::Mat visualOutput;
    cv::cvtColor(processed, visualOutput, cv::COLOR_GRAY2BGR);
    
    // Draw ROI rectangle
    int roiY = static_cast<int>(m_roiYStart * image.rows);
    int roiHeight = static_cast<int>(m_roiHeight * image.rows);
    int roiWidth = static_cast<int>(m_roiWidth * image.cols);
    int roiX = (image.cols - roiWidth) / 2;
    cv::rectangle(visualOutput, cv::Rect(roiX, roiY, roiWidth, roiHeight), cv::Scalar(0, 255, 0), 2);

    // Apply Canny edge detection
    cv::Mat edges;
    cv::Canny(roi, edges, m_cannyLowThreshold, m_cannyHighThreshold);
    
    // Find lines using HoughLinesP
    std::vector<cv::Vec4i> lines = findLines(edges);
    result.detectedLines = lines;
    
    if (!lines.empty()) {
        result.lineDetected = true;
        
        // Calculate deviation using detected lines
        result.deviation = calculateDeviation(lines, image.cols);
        
        // Draw the detected lines
        for (const auto& line : lines) {
            // Adjust line coordinates to account for ROI
            cv::Point pt1(line[0] + roiX, line[1] + roiY);
            cv::Point pt2(line[2] + roiX, line[3] + roiY);
            cv::line(visualOutput, pt1, pt2, cv::Scalar(0, 0, 255), 2);
            
            // Add line points for further processing
            result.linePoints.push_back(pt1);
            result.linePoints.push_back(pt2);
        }
        
        // Estimate center point
        if (!result.linePoints.empty()) {
            int sumX = 0, sumY = 0;
            for (const auto& point : result.linePoints) {
                sumX += point.x;
                sumY += point.y;
            }
            result.centerPoint.x = sumX / result.linePoints.size();
            result.centerPoint.y = sumY / result.linePoints.size();
            
            // Draw center point
            cv::circle(visualOutput, result.centerPoint, 5, cv::Scalar(255, 0, 0), -1);
        }
    }

    return visualOutput;
}

bool LineDetector::detectObstacle(const cv::Mat& image) {
    if (image.empty()) {
        return false;
    }

    // Extract ROI for obstacle detection
    int roiY = static_cast<int>(m_roiYStart * image.rows);
    int roiHeight = static_cast<int>(m_roiHeight * image.rows);
    int roiWidth = static_cast<int>(m_roiWidth * image.cols);
    int roiX = (image.cols - roiWidth) / 2;
    
    cv::Mat roi = image(cv::Rect(roiX, roiY, roiWidth, roiHeight)).clone();
    
    // Convert to grayscale
    cv::Mat gray;
    if (roi.channels() == 3) {
        cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = roi.clone();
    }
    
    // Apply blur and threshold
    cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0);
    cv::Mat binary;
    cv::threshold(gray, binary, m_thresholdValue, 255, cv::THRESH_BINARY);
    
    // Find contours in the binary image
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(binary, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    // Check if we have a significant line pattern
    // If not enough contours/edges are found, we consider it an obstacle
    return (contours.size() < m_obstacleThreshold);
}

cv::Mat LineDetector::preprocess(const cv::Mat& image) {
    cv::Mat gray, blurred;
    
    // Convert to grayscale
    if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = image.clone();
    }
    
    // Apply Gaussian blur
    cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 0);
    
    // Detect vertical edges using Sobel (good for detecting stripes in blind tracks)
    cv::Mat sobelX;
    cv::Sobel(blurred, sobelX, CV_8U, 1, 0, 3);
    
    // Apply binary threshold
    cv::Mat binary;
    cv::threshold(sobelX, binary, m_thresholdValue, 255, cv::THRESH_BINARY);
    
    return binary;
}

cv::Mat LineDetector::extractROI(const cv::Mat& image) {
    int roiY = static_cast<int>(m_roiYStart * image.rows);
    int roiHeight = static_cast<int>(m_roiHeight * image.rows);
    int roiWidth = static_cast<int>(m_roiWidth * image.cols);
    int roiX = (image.cols - roiWidth) / 2;
    
    cv::Rect roi(roiX, roiY, roiWidth, roiHeight);
    
    // Create and store ROI mask for visualization
    m_roiMask = cv::Mat::zeros(image.size(), CV_8UC1);
    m_roiMask(roi) = 255;
    
    // Return ROI
    return image(roi).clone();
}

std::vector<cv::Vec4i> LineDetector::findLines(const cv::Mat& processedImage) {
    std::vector<cv::Vec4i> lines;
    
    // Use Hough Line Transform to find lines
    cv::HoughLinesP(processedImage, lines, 1, CV_PI/180, 
                   m_houghThreshold, m_minLineLength, m_maxLineGap);
    
    return lines;
}

float LineDetector::calculateDeviation(const std::vector<cv::Vec4i>& lines, int imageWidth) {
    if (lines.empty()) {
        return 0.0f;
    }
    
    // Calculate average line center
    float sumX = 0.0f;
    for (const auto& line : lines) {
        float midX = (line[0] + line[2]) / 2.0f;
        sumX += midX;
    }
    
    float avgX = sumX / lines.size();
    
    // Normalize to [-1, 1] range
    // Deviation is negative if line is to the left of center, positive if to the right
    float imageCenter = imageWidth / 2.0f;
    float maxDeviation = imageCenter;  // Maximum possible deviation
    
    float deviation = (avgX - imageCenter) / maxDeviation;
    
    // Clamp to [-1, 1]
    if (deviation < -1.0f) deviation = -1.0f;
    if (deviation > 1.0f) deviation = 1.0f;
    
    return deviation;
}

void LineDetector::setThreshold(int threshold) {
    m_thresholdValue = threshold;
}

void LineDetector::setROI(float roiYStart, float roiHeight, float roiWidth) {
    m_roiYStart = roiYStart;
    m_roiHeight = roiHeight;
    m_roiWidth = roiWidth;
}

cv::Mat LineDetector::getROIMask() const {
    return m_roiMask;
}

void LineDetector::setCannyParameters(int lowThreshold, int highThreshold) {
    m_cannyLowThreshold = lowThreshold;
    m_cannyHighThreshold = highThreshold;
}

void LineDetector::setHoughParameters(int threshold, int minLineLength, int maxLineGap) {
    m_houghThreshold = threshold;
    m_minLineLength = minLineLength;
    m_maxLineGap = maxLineGap;
} 