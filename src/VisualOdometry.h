#pragma once

#include <filesystem>
#include <opencv2/core.hpp>
#include <vector>

class VisualOdometry {
public:
    VisualOdometry();

    std::vector<cv::Point2f> getKeyPointsByImageIndex(
        const std::filesystem::path& imagePath,
        int imageIndex
    ) const;

    bool trackPointsLucasKanade(
        const std::vector<cv::Point2f>& previousPoints,
        const std::filesystem::path& previousImagePath,
        const std::filesystem::path& currentImagePath,
        std::vector<cv::Point2f>& trackedPreviousPoints,
        std::vector<cv::Point2f>& trackedCurrentPoints
    ) const;

    bool findEssentialMatrix(
        const std::vector<cv::Point2f>& previousPoints,
        const std::vector<cv::Point2f>& currentPoints,
        const cv::Mat& cameraMatrix,
        cv::Mat& essentialMatrix,
        std::vector<cv::Point2f>& filteredPreviousPoints,
        std::vector<cv::Point2f>& filteredCurrentPoints
    ) const;

    bool recoverCameraPose(
        const cv::Mat& essentialMatrix,
        const std::vector<cv::Point2f>& previousPoints,
        const std::vector<cv::Point2f>& currentPoints,
        const cv::Mat& cameraMatrix,
        cv::Mat& rotation,
        cv::Mat& translation,
        int& inlierCount
    ) const;

    double calculateScaleFromPoses(
        const cv::Mat& previousPose,
        const cv::Mat& currentPose
    ) const;
};