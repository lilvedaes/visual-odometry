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

    std::vector<cv::Point2f> getKeyPointsLucasKanade(
        const std::vector<cv::Point2f>& previousPoints,
        const std::filesystem::path& previousImagePath,
        const std::filesystem::path& currentImagePath
    ) const;

    double calculateScaleFromPoses(
        const cv::Mat& previousPose,
        const cv::Mat& currentPose
    ) const;
};