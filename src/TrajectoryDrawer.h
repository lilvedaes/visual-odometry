#pragma once

#include <opencv2/core.hpp>
#include <string>

class TrajectoryDrawer {
private:
    std::string windowName;
    cv::Size windowSize;
    cv::Size trajectorySize;
    cv::Mat trajectory;
    cv::Point trajectoryOrigin;
    double trajectoryScale;
    cv::Scalar estimatedTrajectoryColor;
    cv::Scalar groundTruthTrajectoryColor;
    int trajectoryLineThickness;
    int displayDelayMilliseconds;

    static cv::Point toTrajectoryPoint(
        const cv::Point& origin,
        double scale,
        const cv::Mat& position
    );

public:
    TrajectoryDrawer(
        const std::string& windowName,
        const cv::Size& windowSize,
        const cv::Size& trajectorySize,
        const cv::Point& trajectoryOrigin,
        double trajectoryScale,
        const cv::Scalar& estimatedTrajectoryColor,
        const cv::Scalar& groundTruthTrajectoryColor,
        int trajectoryLineThickness,
        int displayDelayMilliseconds
    );

    void drawEstimatedTrajectory(
        const cv::Mat& previousPosition,
        const cv::Mat& currentPosition
    );

    void drawGroundTruthTrajectory(
        const cv::Mat& previousPose,
        const cv::Mat& currentPose
    );

    void show() const;
};
