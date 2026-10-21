#pragma once

#include <opencv2/core.hpp>
#include <string>
#include <vector>

class TrajectoryDrawer {
private:
    struct TrajectorySegment {
        cv::Point3d previousPosition;
        cv::Point3d currentPosition;
        cv::Scalar color;
    };

    std::string windowName;
    cv::Size windowSize;
    cv::Size trajectorySize;
    cv::Mat trajectory;
    double trajectoryScale;
    cv::Scalar estimatedTrajectoryColor;
    cv::Scalar groundTruthTrajectoryColor;
    std::vector<TrajectorySegment> segments;
    double minX;
    double maxX;
    double minZ;
    double maxZ;
    int trajectoryLineThickness;
    int displayDelayMilliseconds;

    static cv::Point3d toPoint3D(const cv::Mat& position);
    cv::Point toTrajectoryPoint(const cv::Point3d& position) const;
    void addSegment(
        const cv::Mat& previousPosition,
        const cv::Mat& currentPosition,
        const cv::Scalar& color
    );
    void updateBounds(const cv::Point3d& position);
    void redraw();

public:
    TrajectoryDrawer(
        const std::string& windowName,
        const cv::Size& windowSize,
        const cv::Size& trajectorySize,
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
