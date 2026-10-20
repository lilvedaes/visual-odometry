#pragma once

#include <opencv2/core.hpp>
#include <string>

class TrajectoryDrawer {
private:
    std::string windowName;
    cv::Size windowSize;
    cv::Point trajectoryOrigin;
    double trajectoryScale;
    cv::Scalar estimatedTrajectoryColor;
    cv::Scalar groundTruthTrajectoryColor;
    int trajectoryLineThickness;
    int displayDelayMilliseconds;

public:
    TrajectoryDrawer(
        const std::string& windowName,
        const cv::Size& windowSize,
        const cv::Point& trajectoryOrigin,
        double trajectoryScale,
        const cv::Scalar& estimatedTrajectoryColor,
        const cv::Scalar& groundTruthTrajectoryColor,
        int trajectoryLineThickness,
        int displayDelayMilliseconds
    );

    void drawEstimatedTrajectory(
        const cv::Mat& trajectory,
        const cv::Mat& previousPosition,
        const cv::Mat& currentPosition
    ) const;

    void drawGroundTruthTrajectory(
        const cv::Mat& trajectory,
        const cv::Mat& previousPose,
        const cv::Mat& currentPose
    ) const;

    void show(const cv::Mat& trajectory) const;
};
